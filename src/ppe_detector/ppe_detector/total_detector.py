import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import time

class TotalDetector(Node):
    def __init__(self):
        super().__init__('total_detector_node')
        
        # ==================== 파라미터 ====================
        self.declare_parameter('pose_model', 'yolo11n-pose.pt')
        self.declare_parameter('ppe_model', 'models/yolo26n_model/origin2/weights/best.pt')
        self.declare_parameter('target_width', 640)
        self.declare_parameter('skip_frame', 1)       # 프레임 스킵 (2~4 권장)
        self.declare_parameter('enable_debug', True)
        self.declare_parameter('ppe_conf', 0.5)      # 신뢰도 약간 낮춤 (검출률 확보)
        self.declare_parameter('use_tensorrt', False)
        
        # ==================== 디바이스 설정 ====================
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.use_half = (self.device == 'cuda')
        self.get_logger().info(f"🚀 Device: {self.device} (Half: {self.use_half})")

        # ==================== 모델 로드 ====================
        pose_path = self.get_parameter('pose_model').value
        ppe_path = self.get_parameter('ppe_model').value
        use_trt = self.get_parameter('use_tensorrt').value
        
        if use_trt and not pose_path.endswith('.engine'):
            pose_path = pose_path.replace('.pt', '.engine')
            ppe_path = ppe_path.replace('.pt', '.engine')

        try:
            self.get_logger().info("⏳ Loading Models...")
            self.pose_model = YOLO(pose_path)
            self.ppe_model = YOLO(ppe_path)
            
            # Fuse 연산 (PyTorch 모델의 경우 Conv+BN 레이어 병합으로 속도 향상)
            if not use_trt:
                self.get_logger().info("⚡ Fusing layers for speed...")
                # self.pose_model.fuse() # YOLOv11/v8은 로드 시 자동 fuse 될 수 있으나 명시 가능
                
            # 웜업
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.pose_model(dummy, device=self.device, half=self.use_half, verbose=False)
            self.ppe_model(dummy, device=self.device, half=self.use_half, verbose=False)
            self.get_logger().info("✅ Models Loaded & Warmed Up")

        except Exception as e:
            self.get_logger().error(f"❌ Model Load Error: {e}")
            raise e

        # ==================== ROS 통신 ====================
        # Best Effort 필수 (영상 끊김 방지)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(Image, '/image_raw', self.img_callback, qos_profile)
        
        # Publisher (Queue size 줄임)
        self.pub_debug = self.create_publisher(Image, '/ppe_debug', 2)
        self.pub_status = self.create_publisher(String, '/what_detected', 5)
        self.pub_safe = self.create_publisher(Bool, '/all_detected', 5)

        self.bridge = CvBridge()
        
        # 캐싱된 설정값
        self.TARGET_WIDTH = self.get_parameter('target_width').value
        self.SKIP_FRAME = self.get_parameter('skip_frame').value
        self.DEBUG_ENABLED = self.get_parameter('enable_debug').value
        self.PPE_CONF = self.get_parameter('ppe_conf').value
        
        # 비율 상수
        self.RATIO_HEAD = 0.35
        self.RATIO_BODY = 0.40
        self.RATIO_HAND = 0.20
        self.RATIO_EAR = 0.10

        self.frame_cnt = 0
        self.prev_time = time.time()

    @torch.no_grad() # Gradient 계산 비활성화 (메모리/속도 최적화)
    def img_callback(self, msg):
        self.frame_cnt += 1
        
        # [최적화 1] Bridge 변환 전 스킵 확인 (CPU 부하 대폭 감소)
        if self.frame_cnt % self.SKIP_FRAME != 0:
            return

        curr_time = time.time()
        
        # [최적화 2] 이미지 변환 및 리사이징
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_img.shape[:2]
            
            # 리사이즈 (필요한 경우만)
            if w != self.TARGET_WIDTH:
                scale = self.TARGET_WIDTH / w
                new_h = int(h * scale)
                cv_img = cv2.resize(cv_img, (self.TARGET_WIDTH, new_h), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().warn(f"Img conversion failed: {e}")
            return

        # ==================== 1. Pose 추론 ====================
        # classes=[0]으로 사람만 필터링 (불필요한 객체 탐지 방지)
        pose_res = self.pose_model(cv_img, device=self.device, half=self.use_half, 
                                 verbose=False, classes=[0], max_det=1) 
                                 # max_det=1: 가장 큰 사람 1명만 빠르게 잡기

        if not pose_res or not pose_res[0].boxes:
            if self.DEBUG_ENABLED:
                self.publish_debug(cv_img, "No Person", False, 0)
            return

        # 데이터 추출 (CPU 이동 최소화)
        res = pose_res[0]
        # boxes가 텐서 상태일 수 있음. 한 번만 cpu로 이동
        boxes_cpu = res.boxes.xyxy.cpu().numpy()
        kpts_cpu = res.keypoints.xy.cpu().numpy()[0]
        confs_cpu = res.keypoints.conf.cpu().numpy()[0]
        
        p_bbox = boxes_cpu[0]
        p_width = max(10.0, p_bbox[2] - p_bbox[0])

        # ==================== 2. ROI 기반 PPE 추론 ====================
        x1, y1, x2, y2 = map(int, p_bbox)
        
        # ROI 클램핑 (이미지 범위 벗어나지 않게)
        margin = int((x2 - x1) * 0.15)
        ih, iw = cv_img.shape[:2]
        rx1 = max(0, x1 - margin)
        ry1 = max(0, y1 - margin)
        rx2 = min(iw, x2 + margin)
        ry2 = min(ih, y2 + margin)

        # ROI가 너무 작으면 패스
        if (rx2 - rx1) < 10 or (ry2 - ry1) < 10:
            return

        person_roi = cv_img[ry1:ry2, rx1:rx2] # Numpy slicing은 빠름 (View)

        # PPE 추론
        ppe_res = self.ppe_model(person_roi, device=self.device, half=self.use_half,
                               verbose=False, conf=self.PPE_CONF, iou=0.5)

        # PPE 좌표 매핑용 딕셔너리
        detected_ppes = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        
        if ppe_res and ppe_res[0].boxes:
            # 텐서 연산을 최대한 활용하거나, 루프를 최소화
            ppe_boxes = ppe_res[0].boxes
            clss = ppe_boxes.cls.cpu().numpy()
            xys = ppe_boxes.xyxy.cpu().numpy()
            
            names = self.ppe_model.names
            
            for i, cls_idx in enumerate(clss):
                cls_name = names[int(cls_idx)].lower()
                bx1, by1, bx2, by2 = xys[i]
                
                # Global 좌표 변환
                cx = (bx1 + bx2) / 2 + rx1
                cy = (by1 + by2) / 2 + ry1
                
                # 문자열 포함 검사 최적화
                if "helmet" in cls_name or "hard" in cls_name:
                    detected_ppes["helmet"].append((cx, cy))
                elif "vest" in cls_name:
                    detected_ppes["vest"].append((cx, cy))
                elif "glove" in cls_name:
                    detected_ppes["gloves"].append((cx, cy))
                elif "ear" in cls_name or "plug" in cls_name:
                    detected_ppes["earplug"].append((cx, cy))

        # ==================== 3. 좌표 계산 및 매칭 (Pure Math) ====================
        # (이 부분은 Numpy 연산이라 매우 빠르므로 기존 로직 유지하되 함수 호출 overhead만 제거)
        
        # Keypoints Helper
        def get_pt(idx): return kpts_cpu[idx] if confs_cpu[idx] > 0.5 else None

        pt_nose = get_pt(0)
        pt_eye_l, pt_eye_r = get_pt(1), get_pt(2)
        pt_ear_l, pt_ear_r = get_pt(3), get_pt(4)
        pt_sh_l, pt_sh_r = get_pt(5), get_pt(6)
        pt_hip_l, pt_hip_r = get_pt(11), get_pt(12)
        pt_wr_l, pt_wr_r = get_pt(9), get_pt(10)
        pt_el_l, pt_el_r = get_pt(7), get_pt(8)

        # --- Coordinates Logic (Condensed) ---
        eyes_c = (pt_eye_l + pt_eye_r)/2 if (pt_eye_l is not None and pt_eye_r is not None) else None
        
        # Head
        head_c = None
        if pt_nose is not None and eyes_c is not None:
            head_c = eyes_c + (eyes_c - pt_nose) * 2.5
        elif pt_sh_l is not None and pt_sh_r is not None:
            head_c = (pt_sh_l + pt_sh_r)/2 + [0, -p_width*0.35]
            
        # Body
        body_c = None
        sh_c = (pt_sh_l + pt_sh_r)/2 if (pt_sh_l is not None and pt_sh_r is not None) else None
        if sh_c is not None:
            if pt_hip_l is not None and pt_hip_r is not None:
                body_c = sh_c * 0.7 + ((pt_hip_l + pt_hip_r)/2) * 0.3
            else:
                body_c = sh_c + [0, p_width*0.25]
        
        # Hands
        def get_hand(el, wr, is_l):
            if wr is None: return None
            if el is not None: return wr + (wr - el) * 0.4
            offset = p_width * 0.15 * 1.5
            return wr + [0, -offset] if (sh_c is not None and wr[1] < sh_c[1]) else wr + [0, offset]

        hl_c = get_hand(pt_el_l, pt_wr_l, True)
        hr_c = get_hand(pt_el_r, pt_wr_r, False)

        # --- Matching ---
        status = {}
        
        def check(center, ratio, p_type, label, draw_img):
            if center is None: return False
            half = (p_width * ratio) / 2
            x1, y1 = int(center[0]-half), int(center[1]-half)
            x2, y2 = int(center[0]+half), int(center[1]+half)
            
            matched = False
            for (px, py) in detected_ppes[p_type]:
                if x1 <= px <= x2 and y1 <= py <= y2:
                    matched = True
                    break
            
            # [최적화 3] Debug Enabled일 때만 그리기 연산 수행
            if self.DEBUG_ENABLED and draw_img is not None:
                color = (0, 255, 0) if matched else (0, 0, 255)
                cv2.rectangle(draw_img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(draw_img, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            return matched

        draw_target = cv_img if self.DEBUG_ENABLED else None
        
        status["helmet"] = check(head_c, self.RATIO_HEAD, "helmet", "H", draw_target)
        status["vest"] = check(body_c, self.RATIO_BODY, "vest", "V", draw_target)
        
        l_ok = check(hl_c, self.RATIO_HAND, "gloves", "L_G", draw_target)
        r_ok = check(hr_c, self.RATIO_HAND, "gloves", "R_G", draw_target)
        status["gloves"] = (l_ok or hl_c is None) and (r_ok or hr_c is None)
        if hl_c is None and hr_c is None: status["gloves"] = False

        el_ok = check(pt_ear_l, self.RATIO_EAR, "earplug", "L_E", draw_target)
        er_ok = check(pt_ear_r, self.RATIO_EAR, "earplug", "R_E", draw_target)
        status["earplug"] = (el_ok or pt_ear_l is None) and (er_ok or pt_ear_r is None)
        if pt_ear_l is None and pt_ear_r is None: status["earplug"] = False

        # ==================== 4. 결과 전송 ====================
        required = ["helmet", "vest", "gloves"]
        all_safe = all(status.get(k, False) for k in required)
        
        self.pub_status.publish(String(data=", ".join([k for k,v in status.items() if v])))
        self.pub_safe.publish(Bool(data=all_safe))

        # FPS 계산
        dt = curr_time - self.prev_time
        fps = 1.0 / dt if dt > 0 else 0
        self.prev_time = curr_time

        if self.DEBUG_ENABLED:
            self.publish_debug(cv_img, f"{'SAFE' if all_safe else 'UNSAFE'} {fps:.1f}fps", all_safe, fps)

    def publish_debug(self, img, text, is_safe, fps):
        # [최적화 4] 디버그 이미지 발행 빈도 제한 (FPS가 30이어도 디버그는 15FPS만 등)
        # 하지만 여기선 원본 싱크를 위해 매번 보내되, 인코딩 에러만 방어
        try:
            color = (0, 255, 0) if is_safe else (0, 0, 255)
            cv2.rectangle(img, (0, 0), (250, 40), (0, 0, 0), -1)
            cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # ROI 그리기 (옵션)
            # cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (255,255,0), 1)

            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TotalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
