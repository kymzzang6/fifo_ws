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
        
        # ==================== 파라미터 선언 ====================
        self.declare_parameter('pose_model', 'yolo11n-pose.pt')
        self.declare_parameter('ppe_model', '/home/ym/sw_ws/src/sw_hybrid/PPE_yolo/jjin_final_model/yolo11n_model/origin1/weights/best.pt')
        self.declare_parameter('target_width', 640)  # ✅ 입력 이미지 크기
        self.declare_parameter('skip_frame', 4)      # ✅ 프레임 스킵 (높을수록 빠름)
        self.declare_parameter('enable_debug', True) # ✅ 디버그 이미지 토글
        self.declare_parameter('ppe_conf', 0.6)      # ✅ PPE 검출 threshold
        self.declare_parameter('use_tensorrt', False) # ✅ TensorRT 사용 여부
        
        # ==================== 디바이스 설정 ====================
        if torch.cuda.is_available():
            self.device = 'cuda'
            self.use_half = True
            self.get_logger().info(f"🚀 CUDA Detected! GPU: {torch.cuda.get_device_name(0)}")
        else:
            self.device = 'cpu'
            self.use_half = False
            self.get_logger().warn("⚠️ CUDA not available. Using CPU.")

        # ==================== 모델 로드 ====================
        pose_path = self.get_parameter('pose_model').value
        ppe_path = self.get_parameter('ppe_model').value
        use_trt = self.get_parameter('use_tensorrt').value
        
        # TensorRT 사용 시 .engine 파일로 변경
        if use_trt:
            pose_path = pose_path.replace('.pt', '.engine')
            ppe_path = ppe_path.replace('.pt', '.engine')
            self.get_logger().info("🔥 TensorRT Mode Enabled")

        self.get_logger().info("⏳ Loading Models...")
        try:
            self.pose_model = YOLO(pose_path)
            self.ppe_model = YOLO(ppe_path)
            
            if self.device == 'cuda':
                self.pose_model.to('cuda')
                self.ppe_model.to('cuda')
                
                # ✅ 워밍업 추론 (첫 프레임 느린 현상 방지)
                dummy = np.zeros((640, 640, 3), dtype=np.uint8)
                self.pose_model(dummy, device=self.device, verbose=False)
                self.ppe_model(dummy, device=self.device, verbose=False)
                self.get_logger().info("✅ Models warmed up")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load models: {e}")
            return

        # ==================== 통신 설정 ====================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, qos_profile)
        
        self.debug_pub = self.create_publisher(Image, '/ppe_debug', 10)
        self.status_pub = self.create_publisher(String, '/what_detected', 10)
        self.safe_pub = self.create_publisher(Bool, '/all_detected', 10)

        self.bridge = CvBridge()

        # ==================== 설정값 ====================
        self.RATIO_HEAD = 0.35
        self.RATIO_BODY = 0.40
        self.RATIO_HAND = 0.20
        self.RATIO_EAR = 0.10
        
        self.TARGET_WIDTH = self.get_parameter('target_width').value
        self.SKIP_FRAME = self.get_parameter('skip_frame').value
        self.DEBUG_ENABLED = self.get_parameter('enable_debug').value
        self.PPE_CONF = self.get_parameter('ppe_conf').value
        
        self.frame_count = 0
        self.last_time = time.time()  # ✅ FPS 측정용
        self.fps = 0.0

        self.get_logger().info(f"✅ Node Ready | Width: {self.TARGET_WIDTH} | Skip: {self.SKIP_FRAME}")


    @torch.no_grad()
    def listener_callback(self, msg):
        self.frame_count += 1
        
        # ==================== 프레임 스킵 ====================
        if self.frame_count % self.SKIP_FRAME != 0:
            return

        # ==================== 이미지 변환 및 리사이징 ====================
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ✅ 최적화 1: 이미지 크기 축소 (가장 큰 성능 향상)
            h, w = cv_image.shape[:2]
            if w > self.TARGET_WIDTH:
                scale = self.TARGET_WIDTH / w
                cv_image = cv2.resize(cv_image, None, fx=scale, fy=scale, 
                                     interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # ==================== Pose 추론 ====================
        pose_results = self.pose_model(cv_image, device=self.device, 
                                       half=self.use_half, verbose=False)
        
        keypoints = None
        p_width = 0
        p_bbox = None

        # 사람이 검출되지 않으면 종료
        if len(pose_results) == 0 or len(pose_results[0].boxes) == 0:
            self.publish_debug(cv_image, "No Person Detected", False)
            return

        # Pose 데이터 추출
        res = pose_results[0]
        p_bbox = res.boxes.xyxy.cpu().numpy()[0]
        p_width = p_bbox[2] - p_bbox[0]
        if p_width < 10:
            p_width = 10.0
        
        keypoints = res.keypoints.xy.cpu().numpy()[0]
        confs = res.keypoints.conf.cpu().numpy()[0]
        
        # Keypoint 유효성 체크 함수
        def get_pt(idx):
            return keypoints[idx] if confs[idx] > 0.5 else None

        # ==================== ROI 기반 PPE 추론 (최적화 2) ====================
        x1, y1, x2, y2 = map(int, p_bbox)
        
        # BBox 확장 (여유 공간)
        margin = int((x2 - x1) * 0.15)
        roi_x1 = max(0, x1 - margin)
        roi_y1 = max(0, y1 - margin)
        roi_x2 = min(cv_image.shape[1], x2 + margin)
        roi_y2 = min(cv_image.shape[0], y2 + margin)
        
        # ✅ Person ROI만 잘라서 PPE 검출 (전체 이미지 대비 3~5배 빠름)
        person_roi = cv_image[roi_y1:roi_y2, roi_x1:roi_x2]
        ppe_results = self.ppe_model(person_roi, device=self.device, 
                                     verbose=False, conf=self.PPE_CONF)
        
        # PPE 분류 및 좌표 보정
        detected_ppes = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        
        if len(ppe_results) > 0 and len(ppe_results[0].boxes) > 0:
            for box in ppe_results[0].boxes:
                cls_name = self.ppe_model.names[int(box.cls[0])].lower()
                bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                
                # ✅ ROI 좌표 → 원본 이미지 좌표 변환
                cx = (bx1 + bx2) / 2 + roi_x1
                cy = (by1 + by2) / 2 + roi_y1
                
                # PPE 분류
                if "helmet" in cls_name or "hardhat" in cls_name:
                    detected_ppes["helmet"].append((cx, cy))
                elif "vest" in cls_name:
                    detected_ppes["vest"].append((cx, cy))
                elif "glove" in cls_name:
                    detected_ppes["gloves"].append((cx, cy))
                elif "ear" in cls_name or "plug" in cls_name:
                    detected_ppes["earplug"].append((cx, cy))

        # ==================== 좌표 계산 (기존 로직) ====================
        pt_nose = get_pt(0)
        pt_eye_l, pt_eye_r = get_pt(1), get_pt(2)
        pt_ear_l, pt_ear_r = get_pt(3), get_pt(4)
        pt_sh_l, pt_sh_r = get_pt(5), get_pt(6)
        pt_hip_l, pt_hip_r = get_pt(11), get_pt(12)
        pt_wr_l, pt_wr_r = get_pt(9), get_pt(10)
        pt_el_l, pt_el_r = get_pt(7), get_pt(8)

        # --- Head 좌표 ---
        head_center = None
        eyes_center = None
        if pt_eye_l is not None and pt_eye_r is not None:
            eyes_center = (pt_eye_l + pt_eye_r) / 2
        
        if pt_nose is not None and eyes_center is not None:
            vec = eyes_center - pt_nose
            head_center = eyes_center + (vec * 2.5) if np.linalg.norm(vec) > 1 else eyes_center + [0, -p_width*0.2]
        elif pt_ear_l is not None or pt_ear_r is not None:
            ears = [p for p in [pt_ear_l, pt_ear_r] if p is not None]
            ear_center = np.mean(ears, axis=0)
            head_center = ear_center + [0, -p_width*0.25]
        elif pt_sh_l is not None and pt_sh_r is not None:
            sh_center = (pt_sh_l + pt_sh_r) / 2
            head_center = sh_center + [0, -p_width*0.35]

        # --- Body 좌표 ---
        body_center = None
        sh_center_body = None
        if pt_sh_l is not None and pt_sh_r is not None:
            sh_center_body = (pt_sh_l + pt_sh_r) / 2
            if pt_hip_l is not None and pt_hip_r is not None:
                hip_center = (pt_hip_l + pt_hip_r) / 2
                body_center = sh_center_body * 0.7 + hip_center * 0.3
            else:
                body_center = sh_center_body + [0, p_width*0.25]

        # --- Hands 좌표 ---
        unit_len = np.linalg.norm(pt_nose - eyes_center) if (pt_nose is not None and eyes_center is not None) else p_width * 0.15
        
        def calc_hand(el, wr):
            if wr is None:
                return None
            if el is not None:
                return wr + (wr - el) * 0.4
            
            offset = unit_len * 1.5
            is_up = (sh_center_body is not None) and (wr[1] < sh_center_body[1])
            return wr + [0, -offset] if is_up else wr + [0, offset]

        hand_l_center = calc_hand(pt_el_l, pt_wr_l)
        hand_r_center = calc_hand(pt_el_r, pt_wr_r)

        # ==================== 매칭 로직 ====================
        def create_box(center, ratio):
            if center is None:
                return [-1, -1, -1, -1]
            half = (p_width * ratio) / 2
            return [int(center[0]-half), int(center[1]-half), 
                   int(center[0]+half), int(center[1]+half)]
        
        status = {"helmet": False, "vest": False, "gloves": False, "earplug": False}
        
        def check_match(bbox, ppe_type, label, img):
            if bbox[0] == -1:
                return False
            
            x1, y1, x2, y2 = bbox
            is_matched = False
            
            # 기본 BBox 그리기 (파랑)
            if self.DEBUG_ENABLED:
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # PPE 포함 여부 확인
            for (px, py) in detected_ppes[ppe_type]:
                if x1 <= px <= x2 and y1 <= py <= y2:
                    is_matched = True
                    if self.DEBUG_ENABLED:
                        cv2.circle(img, (int(px), int(py)), 5, (0, 255, 0), -1)
                    break
            
            # 결과 표시
            if self.DEBUG_ENABLED:
                if is_matched:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img, f"{label}: OK", (x1, y1-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    cv2.putText(img, f"{label}: MISS", (x1, y1-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            return is_matched

        # 매칭 수행
        status["helmet"] = check_match(create_box(head_center, self.RATIO_HEAD), 
                                       "helmet", "HEAD", cv_image)
        status["vest"] = check_match(create_box(body_center, self.RATIO_BODY), 
                                     "vest", "BODY", cv_image)
        
        # 장갑 (양손 체크)
        l_ok = check_match(create_box(hand_l_center, self.RATIO_HAND), 
                          "gloves", "L_HAND", cv_image)
        r_ok = check_match(create_box(hand_r_center, self.RATIO_HAND), 
                          "gloves", "R_HAND", cv_image)
        status["gloves"] = (l_ok or hand_l_center is None) and (r_ok or hand_r_center is None)
        if hand_l_center is None and hand_r_center is None:
            status["gloves"] = False

        # 귀마개
        el_ok = check_match(create_box(pt_ear_l, self.RATIO_EAR), 
                           "earplug", "L_EAR", cv_image)
        er_ok = check_match(create_box(pt_ear_r, self.RATIO_EAR), 
                           "earplug", "R_EAR", cv_image)
        status["earplug"] = (el_ok or pt_ear_l is None) and (er_ok or pt_ear_r is None)
        if pt_ear_l is None and pt_ear_r is None:
            status["earplug"] = False

        # ==================== 결과 퍼블리시 ====================
        detected_list = [k for k, v in status.items() if v]
        checklist = ["helmet", "vest", "gloves"]  # 필수 장비 목록
        all_safe = all(status[k] for k in checklist)

        self.status_pub.publish(String(data=", ".join(detected_list)))
        self.safe_pub.publish(Bool(data=all_safe))
        
        # ✅ FPS 계산
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.last_time) if (current_time - self.last_time) > 0 else 0
        self.last_time = current_time
        
        # 디버그 이미지 발행
        status_text = f"{'SAFE' if all_safe else 'UNSAFE'} | FPS: {self.fps:.1f}"
        self.publish_debug(cv_image, status_text, all_safe)


    def publish_debug(self, img, text, is_safe):
        if not self.DEBUG_ENABLED:
            return
        
        color = (0, 255, 0) if is_safe else (0, 0, 255)
        # 상단 상태 바
        cv2.rectangle(img, (0, 0), (img.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.8, color, 2)
        
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Debug publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TotalDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
