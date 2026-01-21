import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO

# 커스텀 메시지
from yolo_msgs.msg import PoseDetect

class PoseDetector(Node):
    def __init__(self):
        super().__init__('pose_detector_node')
        
        # 1. 디바이스 설정
        if torch.cuda.is_available():
            self.device = 'cuda'
            self.use_half = True 
            self.get_logger().info(f"🚀 CUDA Detected! Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            self.device = 'cpu'
            self.use_half = False
            self.get_logger().warn("⚠️ CUDA not available. Running on CPU (slower).")

        # 2. 모델 설정
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        model_path = self.get_parameter('model_path').value
        
        self.get_logger().info(f"Loading Pose model: {model_path}...")
        try:
            self.model = YOLO(model_path)
            if self.device == 'cuda':
                self.model.to('cuda')
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

        # 3. 통신 설정 (QoS 수정됨: usb_cam 호환성 확보)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.listener_callback, 
            qos_profile)
        
        self.publisher_ = self.create_publisher(PoseDetect, '/body_points', 10)
        
        self.bridge = CvBridge()

        # [설정] 부위별 BBox 비율 설정
        self.RATIO_HEAD = 0.35  # 헬멧 고려하여 넉넉하게
        self.RATIO_BODY = 0.40
        self.RATIO_HAND = 0.20
        self.RATIO_EAR = 0.10

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            # self.get_logger().error(f"CvBridge Error: {e}")
            return

        # --- 추론 실행 ---
        results = self.model(cv_image, device=self.device, half=self.use_half, verbose=False)
        
        body_msg = PoseDetect()
        body_msg.header = msg.header 
        body_msg.header.frame_id = "camera_link"

        # 초기값 세팅 (-1.0)
        empty_box = [-1.0, -1.0, -1.0, -1.0]
        body_msg.head = empty_box
        body_msg.body = empty_box
        body_msg.hand_left = empty_box
        body_msg.hand_right = empty_box
        body_msg.ear_left = empty_box
        body_msg.ear_right = empty_box

        if len(results) > 0 and results[0].boxes.shape[0] > 0 and results[0].keypoints.xy.shape[1] > 0:
            
            # 1. 사람 전체 정보
            person_box = results[0].boxes.xyxy.cpu().numpy()[0]
            p_width = person_box[2] - person_box[0]
            if p_width < 10: p_width = 10.0

            # 2. 키포인트 추출
            keypoints = results[0].keypoints.xy.cpu().numpy()[0]
            confs = results[0].keypoints.conf.cpu().numpy()[0]
            CONF_THRES = 0.5

            def get_pt(idx):
                if confs[idx] > CONF_THRES:
                    return keypoints[idx]
                return None
            
            def create_dynamic_bbox(center_pt, ratio):
                if center_pt is None: return empty_box
                cx, cy = center_pt
                box_size = p_width * ratio
                half_size = box_size / 2
                return [float(cx - half_size), float(cy - half_size), 
                        float(cx + half_size), float(cy + half_size)]

            # Keypoints: 0:Nose, 1:L-Eye, 2:R-Eye, 3:L-Ear, 4:R-Ear, 5:L-Sh, 6:R-Sh, 7:L-El, 8:R-El, 9:L-Wr, 10:R-Wr, 11:L-Hip, 12:R-Hip

            pt_nose = get_pt(0)
            pt_eye_l, pt_eye_r = get_pt(1), get_pt(2)
            pt_ear_l, pt_ear_r = get_pt(3), get_pt(4)
            pt_sh_l, pt_sh_r = get_pt(5), get_pt(6)
            pt_hip_l, pt_hip_r = get_pt(11), get_pt(12)

            # -------------------------------------------------------------------------
            # [1] 머리 (Cranium) 좌표 계산 - 3단계 계층 구조
            # -------------------------------------------------------------------------
            head_target = None
            
            # 눈 중점 계산
            eyes_center = None
            if pt_eye_l is not None and pt_eye_r is not None:
                eyes_center = (pt_eye_l + pt_eye_r) / 2
            elif pt_eye_l is not None: eyes_center = pt_eye_l
            elif pt_eye_r is not None: eyes_center = pt_eye_r

            # [전략 1] 얼굴 정면/측면: 코->눈 벡터 연장
            if pt_nose is not None and eyes_center is not None:
                vec_nose_to_eye = eyes_center - pt_nose
                # 벡터가 유효하면 사용, 아니면 수직 위로
                if np.linalg.norm(vec_nose_to_eye) > 1.0:
                    head_target = eyes_center + (vec_nose_to_eye * 2.5) # 코-눈 거리의 2.5배 위
                else:
                    head_target = eyes_center + np.array([0, -p_width * 0.2])

            # [전략 2] 뒤통수/측면: 귀 중심에서 상승
            elif pt_ear_l is not None or pt_ear_r is not None:
                ears = [p for p in [pt_ear_l, pt_ear_r] if p is not None]
                ear_center = np.mean(ears, axis=0)
                if pt_sh_l is not None and pt_sh_r is not None:
                    sh_center = (pt_sh_l + pt_sh_r) / 2
                    vec_neck = ear_center - sh_center
                    head_target = ear_center + (vec_neck * 0.8) # 목 길이만큼 더 상승
                else:
                    head_target = ear_center + np.array([0, -p_width * 0.25])

            # [전략 3] 완전 뒷모습: 척추 벡터 연장 (생체 역학적 추론)
            elif pt_sh_l is not None and pt_sh_r is not None:
                sh_center = (pt_sh_l + pt_sh_r) / 2
                if pt_hip_l is not None and pt_hip_r is not None:
                    hip_center = (pt_hip_l + pt_hip_r) / 2
                    vec_spine = sh_center - hip_center
                    head_target = sh_center + (vec_spine * 0.4) # 척추 길이의 40% 위
                else:
                    head_target = sh_center + np.array([0, -p_width * 0.35])

            if head_target is not None:
                body_msg.head = create_dynamic_bbox(head_target, self.RATIO_HEAD)

            # -------------------------------------------------------------------------
            # [2] 기타 부위 (귀, 몸통)
            # -------------------------------------------------------------------------
            if pt_ear_l is not None: body_msg.ear_left = create_dynamic_bbox(pt_ear_l, self.RATIO_EAR)
            if pt_ear_r is not None: body_msg.ear_right = create_dynamic_bbox(pt_ear_r, self.RATIO_EAR)

            sh_center_for_body = None
            if pt_sh_l is not None and pt_sh_r is not None:
                sh_center_for_body = (pt_sh_l + pt_sh_r) / 2
                if pt_hip_l is not None and pt_hip_r is not None:
                    hip_center = (pt_hip_l + pt_hip_r) / 2
                    body_center = sh_center_for_body * 0.7 + hip_center * 0.3
                else:
                    body_center = sh_center_for_body + np.array([0, p_width * 0.25])
                body_msg.body = create_dynamic_bbox(body_center, self.RATIO_BODY)

            # -------------------------------------------------------------------------
            # [3] 손 (Hand) - 팔꿈치 미검출 시 안면 비례 보정 적용
            # -------------------------------------------------------------------------
            
            # 3-1. 기준 길이(Scale) 산출: "코-눈 거리"
            unit_length = 0.0
            if pt_nose is not None and eyes_center is not None:
                unit_length = np.linalg.norm(pt_nose - eyes_center)
            
            # 얼굴이 안 보이면 사람 너비로 대체 (fallback)
            if unit_length < 5.0: 
                unit_length = p_width * 0.15

            def calc_hand(pt_el, pt_wr):
                if pt_wr is None: return empty_box
                
                target = pt_wr # 기본값

                if pt_el is not None:
                    # Case A: 팔꿈치가 보임 -> 벡터 연장
                    vec = pt_wr - pt_el
                    target = pt_wr + (vec * 0.4)
                else:
                    # Case B: 팔꿈치 안 보임 -> 안면 비례 보정 (요청사항 반영)
                    # 손가락 길이 추정 (눈-코 거리의 약 1.5배)
                    offset_dist = unit_length * 1.5
                    
                    # 어깨보다 손이 위에 있으면 위로, 아래에 있으면 아래로 보정
                    # (sh_center_for_body 변수 활용)
                    is_hand_up = False
                    if sh_center_for_body is not None:
                        # y값이 작을수록 위쪽임
                        if pt_wr[1] < sh_center_for_body[1]:
                            is_hand_up = True
                    
                    if is_hand_up:
                        target = pt_wr + np.array([0, -offset_dist]) # 위로
                    else:
                        target = pt_wr + np.array([0, offset_dist])  # 아래로

                return create_dynamic_bbox(target, self.RATIO_HAND)

            body_msg.hand_left = calc_hand(get_pt(7), get_pt(9))
            body_msg.hand_right = calc_hand(get_pt(8), get_pt(10))

        # 메시지 발행
        self.publisher_.publish(body_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
