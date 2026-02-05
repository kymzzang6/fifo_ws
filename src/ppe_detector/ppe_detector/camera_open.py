#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraOpenNode(Node):
    def __init__(self):
        super().__init__('camera_open_node')
        
        # 1. 파라미터 설정 (보여주신 코드의 설정값 반영)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        
        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        
        # 2. OpenCV로 카메라 열기
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'카메라(ID: {camera_id})를 열 수 없습니다!')
            exit(1)
            
        # 3. ROS 설정
        self.bridge = CvBridge()
        
        # QoS 설정: 영상 전송은 끊기더라도 최신 프레임이 중요하므로 BEST_EFFORT 권장
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile)
        
        # 타이머 설정 (FPS에 맞춰 콜백 실행)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"카메라 노드 시작됨 ({width}x{height} @ {fps}fps)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # OpenCV 이미지(numpy) -> ROS 이미지 메시지 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 헤더 정보 채우기
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            
            # 토픽 발행
            self.pub.publish(msg)
            
            # (선택사항) 디버깅용: 로컬 화면에 띄우기 -> 실제 로봇 구동 시에는 주석 처리 추천
            # cv2.imshow("Camera Publisher", frame)
            # cv2.waitKey(1)
        else:
            self.get_logger().warning("프레임을 읽을 수 없습니다.")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraOpenNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()