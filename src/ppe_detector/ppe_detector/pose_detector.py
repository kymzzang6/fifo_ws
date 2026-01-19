import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO

# 커스텀 메시지 (패키지명 확인)
from yolo_msgs.msg import PoseDetect

class PoseDetector(Node):
    def __init__(self):
        super().__init__('pose_detector_node')
        
        # 1. 디바이스 설정 (자동 감지)
        if torch.cuda.is_available():
            self.device = 'cuda'
            self.use_half = True # FP16 연산 (속도 향상)
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

        # 3. 통신 설정
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
        
        self.publisher_ = self.create_publisher(PoseDetect, '/body_points', 10)
        # 디버그 퍼블리셔 제거됨

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # YOLO는 BGR 이미지를 사용하므로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # --- 추론 실행 ---
        results = self.model(cv_image, device=self.device, half=self.use_half, verbose=False)
        
        # 메시지 초기화
        body_msg = PoseDetect()
        
        # 헤더 정보 입력
        body_msg.header.stamp = self.get_clock().now().to_msg()
        body_msg.header.frame_id = "camera_link"

        # 좌표 초기화 (-1.0)
        body_msg.head = [-1.0, -1.0]
        body_msg.body = [-1.0, -1.0]
        body_msg.hand_left = [-1.0, -1.0]
        body_msg.hand_right = [-1.0, -1.0]
        body_msg.ear_left = [-1.0, -1.0]
        body_msg.ear_right = [-1.0, -1.0]

        # 디버그 그리기용 이미지 복사 부분 삭제됨

        if len(results) > 0 and results[0].keypoints is not None and results[0].keypoints.xy.shape[0] > 0:

            # GPU 텐서를 CPU numpy로 변환
            keypoints = results[0].keypoints.xy.cpu().numpy()[0] # (17, 2)
            confs = results[0].keypoints.conf.cpu().numpy()[0]   # (17,)
            CONF_THRES = 0.5

            def get_pt(idx):
                if confs[idx] > CONF_THRES:
                    return keypoints[idx]
                return None
            
            # --- 좌표 계산 로직 ---

            # 1. 귀 (Ears)
            pt_ear_l = get_pt(3)
            pt_ear_r = get_pt(4)
            if pt_ear_l is not None: body_msg.ear_left = pt_ear_l.tolist()
            if pt_ear_r is not None: body_msg.ear_right = pt_ear_r.tolist()

            # 2. 어깨 & 골반
            pt_sh_l = get_pt(5)
            pt_sh_r = get_pt(6)
            pt_hip_l = get_pt(11)
            pt_hip_r = get_pt(12)

            # 3. 머리 (Head) - 정수리 추정
            if pt_ear_l is not None and pt_ear_r is not None:
                ear_center = (pt_ear_l + pt_ear_r) / 2
                
                if pt_sh_l is not None and pt_sh_r is not None:
                    sh_center = (pt_sh_l + pt_sh_r) / 2
                    neck_to_ear = ear_center - sh_center
                    head_top = ear_center + (neck_to_ear * 0.8)
                else:
                    head_top = ear_center + np.array([0, -30])

                body_msg.head = head_top.tolist()

            # 4. 몸통 (Body) - 심장/명치 부근
            if pt_sh_l is not None and pt_sh_r is not None:
                sh_center = (pt_sh_l + pt_sh_r) / 2
                
                if pt_hip_l is not None and pt_hip_r is not None:
                    hip_center = (pt_hip_l + pt_hip_r) / 2
                    body_heart = (1 - 0.3) * sh_center + 0.3 * hip_center
                else: 
                    body_heart = sh_center + np.array([0, 50])
                body_msg.body = body_heart.tolist()
                    
            # 5. 손 (Hands)
            # 왼손
            pt_el_l = get_pt(7)
            pt_wr_l = get_pt(9)
            if pt_wr_l is not None:
                if pt_el_l is not None:
                    vec = pt_wr_l - pt_el_l
                    hand_tip = pt_wr_l + (vec * 0.3)
                    body_msg.hand_left = hand_tip.tolist()
                else:
                    body_msg.hand_left = pt_wr_l.tolist()

            # 오른손
            pt_el_r = get_pt(8)
            pt_wr_r = get_pt(10)
            if pt_wr_r is not None:
                if pt_el_r is not None:
                    vec = pt_wr_r - pt_el_r
                    hand_tip = pt_wr_r + (vec * 0.3)
                    body_msg.hand_right = hand_tip.tolist()
                else:
                    body_msg.hand_right = pt_wr_r.tolist()

            # 디버그 그리기(cv2.circle 등) 부분 삭제됨

        self.publisher_.publish(body_msg)
        # 디버그 퍼블리시 부분 삭제됨

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
