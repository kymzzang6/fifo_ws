#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image  # 💡 이미지 토픽용
from cv_bridge import CvBridge    # 💡 OpenCV <-> ROS 변환용
import cv2
import numpy as np
import os

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        
        # 1. 퍼블리셔 설정
        self.publisher_pos = self.create_publisher(Point, '/obstacle_pos', 10)
        self.publisher_img = self.create_publisher(Image, '/image_debug', 10) # 💡 디버깅 이미지 토픽
        
        self.bridge = CvBridge() # 💡 변환 도구 초기화

        # 2. 카메라 초기화 (기존 젯슨 고유 경로)
        camera_index = '/dev/v4l/by-id/usb-SN0002_1080P_USB_Camera_44434000_P030C01_SN0002-video-index0'
        camera_index = os.path.realpath(camera_index)
        
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라 연결 실패!")
            return

        # 카메라 및 IPM 파라미터
        self.camera_matrix = np.array([
            [468.85160531,   0.        , 296.98624005],
            [  0.        , 470.4410217 , 262.10497125],
            [  0.        ,   0.        ,   1.        ]
        ])
        self.dist_coeffs = np.array([ 0.07647929, -0.04246   ,  0.0021503 ,  0.00667308, -0.01955115])

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        src_points = np.float32([[0, self.height*0.5], [self.width, self.height*0.5], [self.width, self.height], [0, self.height]])
        dst_points = np.float32([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]])
        self.ipm_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.reference_img = None
        self.reference_edges = None
        self.ref_path = "reference_floor.jpg"

        self.get_logger().info("✅ 장애물 탐지 & 디버깅 이미지 노드 가동 중 (y: 0~470 연산)")
        self.timer = self.create_timer(0.033, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        # [단계 1] 왜곡 보정 및 IPM
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)
        ipm_img = cv2.warpPerspective(undistorted, self.ipm_matrix, (self.width, self.height))
        gray_ipm = cv2.cvtColor(ipm_img, cv2.COLOR_BGR2GRAY)

        # 💡 하단 10픽셀 연산 제외
        gray_ipm[self.height - 10 : self.height, :] = 0

        # [단계 2] 전처리 및 엣지
        normalized_ipm = self.clahe.apply(gray_ipm)
        edge_blur = cv2.GaussianBlur(normalized_ipm, (11, 11), 0)
        current_edges = cv2.Canny(edge_blur, 70, 150)

        # [단계 3] 골든 레퍼런스 로드/생성
        if self.reference_img is None:
            if os.path.exists(self.ref_path):
                self.reference_img = cv2.imread(self.ref_path, cv2.IMREAD_GRAYSCALE)
                ref_norm = self.clahe.apply(self.reference_img)
                self.reference_edges = cv2.Canny(cv2.GaussianBlur(ref_norm, (11, 11), 0), 70, 150)
            else:
                cv2.imwrite(self.ref_path, gray_ipm)
                self.reference_img = gray_ipm.copy()
                self.reference_edges = current_edges.copy()
            return

        # [단계 4] 엣지 차영상 및 형태학적 연산
        dilate_kernel = np.ones((5, 5), np.uint8)
        diff_edges = cv2.subtract(current_edges, cv2.dilate(self.reference_edges, dilate_kernel, iterations=1))
        _, thresh = cv2.threshold(diff_edges, 127, 255, cv2.THRESH_BINARY)

        close_kernel = np.ones((15, 15), np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel, iterations=3)
        morph = cv2.dilate(morph, np.ones((5, 5), np.uint8), iterations=2)
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)

        # [단계 5] 동적 마스킹 (서버 랙 등 수직 구조물 제외)
        lines = cv2.HoughLinesP(current_edges, 1, np.pi/180, 100, minLineLength=150, maxLineGap=20)
        left_rack_x, right_rack_x = None, None
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(x2 - x1) < 30: # 수직선 판단
                    if x1 < self.width // 2: left_rack_x = max(x1, x2) if left_rack_x is None else max(left_rack_x, max(x1, x2))
                    else: right_rack_x = min(x1, x2) if right_rack_x is None else min(right_rack_x, min(x1, x2))
        
        # 마스크 적용
        if left_rack_x is not None: cv2.rectangle(morph, (0, 0), (min(left_rack_x, int(self.width*0.3)), self.height), 0, -1)
        if right_rack_x is not None: cv2.rectangle(morph, (max(right_rack_x, int(self.width*0.7)), 0), (self.width, self.height), 0, -1)

        # [단계 6] 장애물 판단 및 위치 추출
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_ipm = ipm_img.copy()
        
        closest_y = -1
        target_pt = None

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if cv2.contourArea(cnt) < 1000: continue
            
            # 장애물 표시 (초록색)
            cv2.rectangle(debug_ipm, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cx, cy = x + w // 2, y + h // 2
            
            if cy > closest_y: # 더 아래(가까운)에 있는 객체 선택
                closest_y = cy
                target_pt = (cx, cy)

        # 💡 토픽 발행 1: 위치 좌표
        if target_pt:
            msg = Point()
            msg.x, msg.y, msg.z = float(target_pt[0]), float(target_pt[1]), 0.0
            self.publisher_pos.publish(msg)
            cv2.circle(debug_ipm, target_pt, 7, (255, 0, 255), -1)

        # [단계 7] 화면 합성 및 토픽 발행 2: 디버깅 이미지
        resized_orig = cv2.resize(undistorted, (self.width // 2, self.height // 2))
        resized_debug = cv2.resize(debug_ipm, (self.width // 2, self.height // 2))
        combined_view = np.hstack((resized_orig, resized_debug))

        try:
            # 💡 OpenCV 이미지를 ROS Image 메시지로 변환하여 퍼블리시
            img_msg = self.bridge.cv2_to_imgmsg(combined_view, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"
            self.publisher_img.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 전송 실패: {e}")

        # 로컬 확인용 (필요 없으면 주석 처리)
        #cv2.imshow("Debug View", combined_view)
        #cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()