import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import torch # GPU 확인용

# 커스텀 메시지
from yolo_msgs.msg import PPEDetect
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 1. 파라미터 설정
        self.declare_parameter('model_path', '/home/ym/sw_ws/src/sw_hybrid/PPE_yolo/jjin_final_model/yolo11n_model/merged_8/weights/best.pt')
        self.declare_parameter('conf_thres', 0.5)
        model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('conf_thres').value

        # 디바이스 자동 설정 (GPU 권장)
        if torch.cuda.is_available():
            self.device = 'cuda'
            self.get_logger().info(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            self.device = 'cpu'
            self.get_logger().warn("⚠️ Running on CPU.")

        # 2. 모델 로드
        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        try:
            self.model = YOLO(model_path)
            if self.device == 'cuda':
                self.model.to('cuda')
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

        # 3. 통신 설정
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
        
        self.publisher_ = self.create_publisher(PPEDetect, '/class_info', 10)
        # 디버그 퍼블리셔 제거됨

        self.bridge = CvBridge()
        
        # 4. FPS 계산용 변수 초기화
        self.prev_time = 0
        
        self.get_logger().info("YOLO Detector Node (No Debug Image) Started.")

    def listener_callback(self, msg):
        # --- FPS 계산 (로그 출력용으로 남겨둠, 필요 없으면 삭제 가능) ---
        current_time = time.time()
        # fps = 0.0
        if self.prev_time != 0:
            time_diff = current_time - self.prev_time
            # if time_diff > 0:
            #     fps = 1.0 / time_diff
        self.prev_time = current_time

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 추론 (FP16 반정밀도 사용 시 속도 향상 가능: half=True)
        results = self.model(cv_image, device=self.device, verbose=False, conf=self.conf_thres)
        
        class_names = []
        center_xs = []
        center_ys = []
        confidences = []

        if len(results) > 0:
            result = results[0]
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                
                # 타겟 클래스 필터링
                target_classes = ['helmet', 'vest', 'gloves', 'earplug']
                
                if class_name in target_classes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    conf = float(box.conf[0])

                    class_names.append(class_name)
                    center_xs.append(cx)
                    center_ys.append(cy)
                    confidences.append(conf)

                    # 디버그 그리기 부분 삭제됨

        # 메시지 생성 및 퍼블리시
        info_msg = PPEDetect()
        
        # 헤더 정보 입력
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = "camera_link"

        info_msg.classes = class_names
        info_msg.center_xs = center_xs
        info_msg.center_ys = center_ys
        info_msg.confidences = confidences
        
        self.publisher_.publish(info_msg)

        # 디버그 이미지 발행 부분 삭제됨

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
