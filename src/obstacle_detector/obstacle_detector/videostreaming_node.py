import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # 1. /image_raw 토픽으로 Image 메시지를 발행할 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 2. 타이머 설정 (0.033초마다 실행 = 약 30 FPS)
        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 3. OpenCV 비디오 캡처 객체 생성 (0번 카메라)
        self.cap = cv2.VideoCapture(0)
        
        # 4. ROS 이미지와 OpenCV 이미지 간 변환을 위한 CvBridge 객체
        self.br = CvBridge()
        
        self.get_logger().info('Webcam Publisher Node has started!')

    def timer_callback(self):
        # 카메라로부터 프레임 읽기
        ret, frame = self.cap.read()
        
        if ret:
            # OpenCV 이미지를 ROS Image 메시지로 변환하여 발행
            # 인코딩은 보통 'bgr8'을 사용합니다.
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture image from webcam')

    def destroy_node(self):
        # 종료 시 카메라 자원 해제
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()