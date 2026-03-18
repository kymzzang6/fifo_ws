import rclpy
from rclpy.node import Node
import cv2
import math
from ultralytics import YOLO
from std_msgs.msg import Header

# 커스텀 메시지 import (패키지명은 실제 패키지명으로 수정 필요)
from yolo_msgs.msg import PPEDetect


class PPEDetectorNode(Node):
    def __init__(self):
        super().__init__('ppe_detector_node')
        
        # 파라미터 선언
        self.declare_parameter('model_path', '/home/caps/fifo_ws/src/ppe_detector/models/yolo26n_model/origin3_addhand4/weights/best.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('max_det', 10)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('timer_period', 0.033)  # 약 30Hz
        
        # 파라미터 가져오기
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.max_det = self.get_parameter('max_det').value
        camera_id = self.get_parameter('camera_id').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        timer_period = self.get_parameter('timer_period').value
        
        # YOLO 모델 로드
        self.get_logger().info(f'모델 로딩 중: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('모델 로딩 완료!')
        
        # 웹캠 설정
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다!')
            return
        
        # Publisher 생성
        self.pub = self.create_publisher(PPEDetect, '/class_info', 10)
        
        # Timer 생성 (주기적으로 추론 실행)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('PPE Detector 노드 시작! /class_info 토픽으로 퍼블리시 중...')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')
            return
        
        # YOLO 추론
        results = self.model(
            frame,
            stream=True,
            verbose=False,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det
        )
        
        # PPEDetect 메시지 생성
        msg = PPEDetect()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        
        # 탐지 결과 처리
        for r in results:
            boxes = r.boxes
            
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # 중심점 계산
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                
                # 신뢰도 및 클래스
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = self.model.names[cls]
                
                # 메시지에 추가
                msg.classes.append(class_name)
                msg.center_xs.append(cx)
                msg.center_ys.append(cy)
                msg.confidences.append(conf)
        
        # 메시지 퍼블리시
        self.pub.publish(msg)
        
        # 로그 출력 (선택사항)
        if len(msg.classes) > 0:
            self.get_logger().info(
                f'탐지된 객체: {len(msg.classes)}개 - {msg.classes}',
                throttle_duration_sec=1.0  # 1초마다 한 번만 출력
            )
    
    def destroy_node(self):
        # 리소스 정리
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PPEDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
