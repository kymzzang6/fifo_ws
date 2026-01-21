import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import torch
from ultralytics import YOLO
import cv2
import numpy as np

# 커스텀 메시지
from yolo_msgs.msg import PPEDetect

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 1. 파라미터 설정
        self.declare_parameter('model_path', '/home/ym/sw_ws/src/sw_hybrid/PPE_yolo/jjin_final_model/yolo11n_model/origin1/weights/best.pt')
        self.declare_parameter('conf_thres', 0.5)
        model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('conf_thres').value

        # 디바이스 자동 설정
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
        
        self.publisher_ = self.create_publisher(PPEDetect, '/class_info', 10)
        
        # 🆕 디버깅 이미지 퍼블리셔 추가
        self.debug_image_publisher = self.create_publisher(Image, '/yolo_detected_image', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Detector Node Started (with debug image publishing).")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 추론
        results = self.model(cv_image, device=self.device, verbose=False, conf=self.conf_thres)
        
        class_names = []
        center_xs = []
        center_ys = []
        confidences = []

        # 🆕 디버깅 이미지 생성
        debug_image = cv_image.copy()

        if len(results) > 0:
            result = results[0]
            
            # YOLO 결과로 bounding box 그리기
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                
                target_classes = ['helmet', 'vest', 'gloves', 'earplug', 'hand'] 
                
                if class_name.lower() in target_classes or any(t in class_name.lower() for t in target_classes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # 중심점 계산
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    
                    class_names.append(class_name)
                    center_xs.append(cx)
                    center_ys.append(cy)
                    confidences.append(conf)
                    
                    # 🆕 Bounding box 그리기 (녹색)
                    cv2.rectangle(debug_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # 🆕 클래스 이름과 신뢰도 텍스트 그리기
                    label = f"{class_name}: {conf:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    cv2.rectangle(debug_image, (int(x1), int(y1)-label_size[1]-10), 
                                (int(x1)+label_size[0], int(y1)), (0, 255, 0), -1)
                    cv2.putText(debug_image, label, (int(x1), int(y1)-5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    
                    # 🆕 중심점 그리기 (빨간색 원)
                    cv2.circle(debug_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # 🆕 디버깅 이미지 발행
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_img_msg.header = msg.header  # 동일한 타임스탬프 유지
            self.debug_image_publisher.publish(debug_img_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publishing error: {e}")

        # 기존 PPEDetect 메시지 발행
        info_msg = PPEDetect()
        info_msg.header = msg.header 
        info_msg.classes = class_names
        info_msg.center_xs = center_xs
        info_msg.center_ys = center_ys
        info_msg.confidences = confidences
        
        self.publisher_.publish(info_msg)

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
