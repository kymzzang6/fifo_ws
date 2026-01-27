import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import time

class SimpleCamNode(Node):
    def __init__(self):
        super().__init__('simple_cam_node')
        
        # 파라미터 설정 (필요시 변경)
        self.declare_parameter('device_id', 0)      # 카메라 번호 (/dev/video0)
        self.declare_parameter('width', 640)        # 해상도 너비
        self.declare_parameter('height', 480)       # 해상도 높이
        self.declare_parameter('fps', 30)           # 목표 FPS
        
        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # OpenCV로 카메라 열기
        self.cap = cv2.VideoCapture(device_id)
        
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 해상도 및 FPS 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # 실제 설정된 값 확인
        real_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        real_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"📷 Camera Open: {int(real_w)}x{int(real_h)} @ {real_fps}fps (MJPG)")
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open camera!")
            exit()

        # 퍼블리셔 및 타이머 설정
        self.publisher_ = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()
        
        # 타이머 주기는 FPS에 맞춰 설정 (약간 더 빠르게 설정해서 지연 방지)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # 타임스탬프 찍기
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("⚠️ Frame dropped or camera disconnected")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
