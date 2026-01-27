import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import torch
from ultralytics import YOLO
import cv2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock

class YoloDiagnostic(Node):
    def __init__(self):
        super().__init__('yolo_diagnostic_node')
        self.callback_group = ReentrantCallbackGroup()
        
        # 모델 로드 (가볍게 테스트)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using Device: {self.device}")
        
        # 실제 모델 경로로 수정 필요
        try:
            self.model = YOLO('models/yolo26n_model/origin2_addhand/weights/best.pt')
            if self.device == 'cuda':
                self.model.to('cuda')
        except:
            self.model = None # 모델 없이 통신만 테스트할 경우

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        self.sub = self.create_subscription(Image, '/image_raw', self.listener_callback, qos_profile, callback_group=self.callback_group)
        self.bridge = CvBridge()
        
        self.last_recv_time = time.time()
        self.last_infer_time = time.time()
        
        self.latest_msg = None
        self.lock = Lock()
        
        # 추론 루프 타이머
        self.timer = self.create_timer(0.01, self.inference_loop, callback_group=self.callback_group)

    def listener_callback(self, msg):
        now = time.time()
        dt = now - self.last_recv_time
        self.last_recv_time = now
        
        # 수신 간격이 0.1초(100ms) 넘어가면 경고 출력
        if dt > 0.1:
            self.get_logger().warn(f"🔴 [RECV LAG] Image received after {dt:.4f}s")
        
        with self.lock:
            self.latest_msg = msg

    def inference_loop(self):
        msg = None
        with self.lock:
            if self.latest_msg:
                msg = self.latest_msg
                self.latest_msg = None
        
        if msg is None:
            return

        t_start = time.time()
        
        # 1. 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        t_conv = time.time()
        
        # 2. 추론 (있으면)
        if self.model:
            results = self.model(cv_img, verbose=False, half=True)
            
        t_infer = time.time()
        
        # 3. 주기 측정
        dt_loop = t_start - self.last_infer_time
        self.last_infer_time = t_start
        
        # 추론 루프 자체가 늦게 돌면 경고
        if dt_loop > 0.15:
             self.get_logger().warn(f"🟠 [LOOP LAG] Inference loop started after {dt_loop:.4f}s")
             
        # 추론이 너무 오래 걸리면 경고
        infer_duration = t_infer - t_start
        if infer_duration > 0.1:
            self.get_logger().warn(f"🟡 [SLOW GPU] Inference took {infer_duration:.4f}s")

def main():
    rclpy.init()
    node = YoloDiagnostic()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
