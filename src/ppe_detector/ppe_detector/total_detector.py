import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor   # [추가] 멀티스레드 실행기
from rclpy.callback_groups import ReentrantCallbackGroup # [추가] 재진입 가능한 콜백 그룹
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
        self.declare_parameter('skip_frame', 1)       # 프레임 스킵
        self.declare_parameter('enable_debug', True)
        self.declare_parameter('ppe_conf', 0.5)      
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
            
            # 모델을 명시적으로 GPU로 이동 (확실하게 하기 위함)
            self.pose_model.to(self.device)
            self.ppe_model.to(self.device)
            
            # Fuse 연산
            if not use_trt:
                self.get_logger().info("⚡ Fusing layers for speed...")
                # self.pose_model.fuse() 
                
            # 웜업 (Warm-up)
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            # 첫 실행으로 메모리 할당 및 초기화
            self.pose_model(dummy, device=self.device, half=self.use_half, verbose=False)
            self.ppe_model(dummy, device=self.device, half=self.use_half, verbose=False)
            self.get_logger().info("✅ Models Loaded & Warmed Up")

        except Exception as e:
            self.get_logger().error(f"❌ Model Load Error: {e}")
            raise e

        # ==================== ROS 통신 ====================
        # [핵심] 병렬 처리를 위한 콜백 그룹 생성
        # 이 그룹에 속한 콜백들은 여러 스레드에서 동시에 실행될 수 있음
        self.cb_group = ReentrantCallbackGroup()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # [핵심] Subscription에 callback_group 지정
        self.sub = self.create_subscription(
            Image, 
            '/image_raw', 
            self.img_callback, 
            qos_profile,
            callback_group=self.cb_group  # <--- 여기 추가됨
        )
        
        self.pub_debug = self.create_publisher(Image, '/ppe_debug', 2, callback_group=self.cb_group)
        self.pub_status = self.create_publisher(String, '/what_detected', 5, callback_group=self.cb_group)
        self.pub_safe = self.create_publisher(Bool, '/all_detected', 5, callback_group=self.cb_group)

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

    @torch.no_grad()
    def img_callback(self, msg):
        self.frame_cnt += 1
        
        if self.frame_cnt % self.SKIP_FRAME != 0:
            return

        curr_time = time.time()
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_img.shape[:2]
            
            if w != self.TARGET_WIDTH:
                scale = self.TARGET_WIDTH / w
                new_h = int(h * scale)
                cv_img = cv2.resize(cv_img, (self.TARGET_WIDTH, new_h), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().warn(f"Img conversion failed: {e}")
            return

        # ==================== 1. Pose 추론 ====================
        pose_res = self.pose_model(cv_img, device=self.device, half=self.use_half, 
                                 verbose=False, classes=[0], max_det=1) 

        if not pose_res or not pose_res[0].boxes:
            if self.DEBUG_ENABLED:
                self.publish_debug(cv_img, "No Person", False, 0)
            return

        res = pose_res[0]
        boxes_cpu = res.boxes.xyxy.cpu().numpy()
        kpts_cpu = res.keypoints.xy.cpu().numpy()[0]
        confs_cpu = res.keypoints.conf.cpu().numpy()[0]
        
        p_bbox = boxes_cpu[0]
        p_width = max(10.0, p_bbox[2] - p_bbox[0])

        # ==================== 2. ROI 기반 PPE 추론 ====================
        x1, y1, x2, y2 = map(int, p_bbox)
        
        margin = int((x2 - x1) * 0.15)
        ih, iw = cv_img.shape[:2]
        rx1 = max(0, x1 - margin)
        ry1 = max(0, y1 - margin)
        rx2 = min(iw, x2 + margin)
        ry2 = min(ih, y2 + margin)

        if (rx2 - rx1) < 10 or (ry2 - ry1) < 10:
            return

        person_roi = cv_img[ry1:ry2, rx1:rx2]

        ppe_res = self.ppe_model(person_roi, device=self.device, half=self.use_half,
                               verbose=False, conf=self.PPE_CONF, iou=0.5)

        detected_ppes = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        
        if ppe_res and ppe_res[0].boxes:
            ppe_boxes = ppe_res[0].boxes
            clss = ppe_boxes.cls.cpu().numpy()
            xys = ppe_boxes.xyxy.cpu().numpy()
            names = self.ppe_model.names
            
            for i, cls_idx in enumerate(clss):
                cls_name = names[int(cls_idx)].lower()
                bx1, by1, bx2, by2 = xys[i]
                
                cx = (bx1 + bx2) / 2 + rx1
                cy = (by1 + by2) / 2 + ry1
                
                if "helmet" in cls_name or "hard" in cls_name:
                    detected_ppes["helmet"].append((cx, cy))
                elif "vest" in cls_name:
                    detected_ppes["vest"].append((cx, cy))
                elif "glove" in cls_name:
                    detected_ppes["gloves"].append((cx, cy))
                elif "ear" in cls_name or "plug" in cls_name:
                    detected_ppes["earplug"].append((cx, cy))

        # ==================== 3. 좌표 계산 및 매칭 ====================
        def get_pt(idx): return kpts_cpu[idx] if confs_cpu[idx] > 0.5 else None

        pt_nose = get_pt(0)
        pt_eye_l, pt_eye_r = get_pt(1), get_pt(2)
        pt_ear_l, pt_ear_r = get_pt(3), get_pt(4)
        pt_sh_l, pt_sh_r = get_pt(5), get_pt(6)
        pt_hip_l, pt_hip_r = get_pt(11), get_pt(12)
        pt_wr_l, pt_wr_r = get_pt(9), get_pt(10)
        pt_el_l, pt_el_r = get_pt(7), get_pt(8)

        eyes_c = (pt_eye_l + pt_eye_r)/2 if (pt_eye_l is not None and pt_eye_r is not None) else None
        
        head_c = None
        if pt_nose is not None and eyes_c is not None:
            head_c = eyes_c + (eyes_c - pt_nose) * 2.5
        elif pt_sh_l is not None and pt_sh_r is not None:
            head_c = (pt_sh_l + pt_sh_r)/2 + [0, -p_width*0.35]
            
        body_c = None
        sh_c = (pt_sh_l + pt_sh_r)/2 if (pt_sh_l is not None and pt_sh_r is not None) else None
        if sh_c is not None:
            if pt_hip_l is not None and pt_hip_r is not None:
                body_c = sh_c * 0.7 + ((pt_hip_l + pt_hip_r)/2) * 0.3
            else:
                body_c = sh_c + [0, p_width*0.25]
        
        def get_hand(el, wr, is_l):
            if wr is None: return None
            if el is not None: return wr + (wr - el) * 0.4
            offset = p_width * 0.15 * 1.5
            return wr + [0, -offset] if (sh_c is not None and wr[1] < sh_c[1]) else wr + [0, offset]

        hl_c = get_hand(pt_el_l, pt_wr_l, True)
        hr_c = get_hand(pt_el_r, pt_wr_r, False)

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

        dt = curr_time - self.prev_time
        fps = 1.0 / dt if dt > 0 else 0
        self.prev_time = curr_time

        if self.DEBUG_ENABLED:
            self.publish_debug(cv_img, f"{'SAFE' if all_safe else 'UNSAFE'} {fps:.1f}fps", all_safe, fps)

    def publish_debug(self, img, text, is_safe, fps):
        try:
            color = (0, 255, 0) if is_safe else (0, 0, 255)
            cv2.rectangle(img, (0, 0), (250, 40), (0, 0, 0), -1)
            cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = TotalDetector()
    
    # [핵심] MultiThreadedExecutor 사용
    # num_threads=4: 동시에 4개의 스레드까지 사용하여 콜백 처리 (이미지 수신 / 추론 등)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin() # rclpy.spin(node) 대신 executor 사용
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
