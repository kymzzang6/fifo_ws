#!/home/caps/fifo_ws/src/obstacle_detector/venv/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class EfficientADDebugger(Node):
    def __init__(self):
        super().__init__('efficientad_debugger')
        
        self.bridge = CvBridge()
        
        # 최신 데이터를 저장할 변수
        self.latest_image = None
        self.latest_poses = []
        
        self.sub_image = self.create_subscription(Image, '/image_raw', self.img_callback, 10)
        self.sub_poses = self.create_subscription(PoseArray, '/obstacle_pos', self.pose_callback, 10)
        self.sub_heatmap = self.create_subscription(Image, '/image_debug', self.heatmap_callback, 10)
        
        self.pub_debug = self.create_publisher(Image, '/debug/image_overlay', 10)
        self.get_logger().info("디버깅 노드가 시작되었습니다. 영상 오버레이를 준비합니다.")

    def img_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def pose_callback(self, msg):
        self.latest_poses = [(int(p.position.x), int(p.position.y)) for p in msg.poses]

    def heatmap_callback(self, msg):
        if self.latest_image is None:
            return
            
        # 히트맵 처리 및 리사이즈
        heatmap_mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        orig_h, orig_w = self.latest_image.shape[:2]
        heatmap_resized = cv2.resize(heatmap_mono, (orig_w, orig_h))
        
        # 컬러맵 적용 (Jet Color)
        heatmap_color = cv2.applyColorMap(heatmap_resized, cv2.COLORMAP_JET)
        
        # 원본 이미지와 블렌딩 (Alpha=0.4)
        overlay = cv2.addWeighted(self.latest_image, 0.6, heatmap_color, 0.4, 0)
        
        # 장애물 좌표 (빨간색 십자 마커 표시)
        for (x, y) in self.latest_poses:
            cv2.drawMarker(overlay, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(overlay, f"({x},{y})", (x+10, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 화면에 즉시 띄워 시각적 디버깅 수행
        cv2.imshow("Anomaly Debugger (EfficientAD)", overlay)
        cv2.waitKey(1)
        
        # 다른 토픽이나 Rviz2에서도 볼 수 있게 발행
        debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EfficientADDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()