import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO
from yolo_msgs.msg import PoseDetect
import time

class PoseDetector(Node):
    def __init__(self):
        super().__init__('pose_node')
        
        # 디바이스 체크
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.use_half = (self.device == 'cuda')
        self.get_logger().info(f"🚀 Pose Node using {self.device}")

        # 파라미터
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        model_path = self.get_parameter('model_path').value
        
        try:
            self.model = YOLO(model_path)
            if self.device == 'cuda':
                self.model.to('cuda')
        except Exception as e:
            self.get_logger().error(f"Model Error: {e}")

        # 통신 설정 (최적화)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # [최적화]
        )

        self.sub = self.create_subscription(Image, '/image_raw', self.listener_callback, qos_profile)
        self.pub = self.create_publisher(PoseDetect, '/body_points', 10)
        self.bridge = CvBridge()
        
        self.last_proc_time = 0

        # 박스 비율 상수
        self.RATIO_HEAD = 0.35
        self.RATIO_BODY = 0.40
        self.RATIO_HAND = 0.20
        self.RATIO_EAR = 0.10

    def listener_callback(self, msg):
        # [최적화] 15 FPS 제한 (과부하 방지)
        now = self.get_clock().now().nanoseconds
        if (now - self.last_proc_time) < 66666666: # 약 66ms
            return
        self.last_proc_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except:
            return

        # 추론: 사람(class=0)만 탐지, 최대 1명
        results = self.model(cv_img, device=self.device, half=self.use_half, verbose=False, classes=[0], max_det=1)
        
        body_msg = PoseDetect()
        body_msg.header = msg.header
        
        # 빈 박스 초기화
        empty = [-1.0]*4
        for attr in ['head', 'body', 'hand_left', 'hand_right', 'ear_left', 'ear_right']:
            setattr(body_msg, attr, empty)

        if results and len(results[0].boxes) > 0 and len(results[0].keypoints) > 0:
            
            # --- 좌표 계산 로직 (기존과 동일하지만 간소화) ---
            kpts = results[0].keypoints.xy.cpu().numpy()[0]
            confs = results[0].keypoints.conf.cpu().numpy()[0]
            box = results[0].boxes.xyxy.cpu().numpy()[0]
            p_width = max(10.0, box[2] - box[0])

            def get_kpt(idx): return kpts[idx] if confs[idx] > 0.5 else None
            
            def make_box(center, ratio):
                if center is None: return empty
                s = (p_width * ratio) / 2
                return [center[0]-s, center[1]-s, center[0]+s, center[1]+s]

            # Keypoints mapping
            nose = get_kpt(0)
            eyes = [get_kpt(1), get_kpt(2)]
            ears = [get_kpt(3), get_kpt(4)]
            shs = [get_kpt(5), get_kpt(6)]
            els = [get_kpt(7), get_kpt(8)]
            wrs = [get_kpt(9), get_kpt(10)]
            hips = [get_kpt(11), get_kpt(12)]

            # 1. Head
            eyes_valid = [e for e in eyes if e is not None]
            if nose is not None and len(eyes_valid) > 0:
                eye_c = np.mean(eyes_valid, axis=0)
                head_c = eye_c + (eye_c - nose) * 2.5
                body_msg.head = make_box(head_c, self.RATIO_HEAD)
            elif len(ears) > 0 and all(e is not None for e in ears):
                 body_msg.head = make_box(np.mean(ears, axis=0) + [0, -p_width*0.2], self.RATIO_HEAD)

            # 2. Body
            sh_valid = [s for s in shs if s is not None]
            if len(sh_valid) == 2:
                sh_c = np.mean(sh_valid, axis=0)
                hip_valid = [h for h in hips if h is not None]
                if len(hip_valid) == 2:
                    hip_c = np.mean(hip_valid, axis=0)
                    body_msg.body = make_box(sh_c*0.6 + hip_c*0.4, self.RATIO_BODY)
                else:
                    body_msg.body = make_box(sh_c + [0, p_width*0.3], self.RATIO_BODY)

            # 3. Hands & Ears
            body_msg.hand_left = make_box(wrs[0], self.RATIO_HAND)
            body_msg.hand_right = make_box(wrs[1], self.RATIO_HAND)
            body_msg.ear_left = make_box(ears[0], self.RATIO_EAR)
            body_msg.ear_right = make_box(ears[1], self.RATIO_EAR)

        self.pub.publish(body_msg)

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
