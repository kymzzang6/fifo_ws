#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class InteractiveCalibrator(Node):
    def __init__(self):
        super().__init__('interactive_calibrator')
        self.bridge = CvBridge()
        
        # 이미지 버퍼
        self.rgb_img = None
        self.thermal_img = None
        
        # 🌟 정합을 위한 초기 파라미터 🌟
        self.dx = 0.0       # X축 이동
        self.dy = 0.0       # Y축 이동
        self.scale = 2.0    # 크기 확대/축소 (열화상이 작으므로 2배부터 시작)
        self.angle = 0.0    # 회전 각도
        self.alpha = 0.6    # 열화상 영상의 투명도
        
        # 구독자 설정
        self.sub_rgb = self.create_subscription(Image, "/camera/color/image_raw", self.rgb_cb, 10)
        self.sub_thermal = self.create_subscription(Image, "/thermal/video", self.thermal_cb, 10)
        
        # OpenCV GUI 타이머 (약 30 FPS)
        self.timer = self.create_timer(0.033, self.cv_loop)
        cv2.namedWindow('Interactive Calibration (Astra + Thermal)', cv2.WINDOW_AUTOSIZE)
        
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("=== 🎮 대화형 열화상-RGB 수동 정합 노드 ===")
        self.get_logger().info("[W/A/S/D] : 상/좌/하/우 이동 (Shift 누른채로 누르면 미세 이동)")
        self.get_logger().info("[ + / - ] : 크기 확대 / 축소")
        self.get_logger().info("[ Q / E ] : 좌회전 / 우회전")
        self.get_logger().info("[ [ / ] ] : 열화상 투명도 감소 / 증가")
        self.get_logger().info("[ P ] : 현재 변환 행렬 및 파라미터 터미널에 출력")
        self.get_logger().info("[ ESC ] : 프로그램 종료")

    def rgb_cb(self, msg):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB 변환 에러: {e}")

    def thermal_cb(self, msg):
        try:
            # passthrough로 받아 16bit raw 데이터도 처리할 수 있게 함
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 만약 흑백 1채널 영상이라면
            if len(cv_img.shape) == 2:
                # 데이터를 0~255로 정규화 (까맣게 나오는 현상 방지)
                cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                # 시인성을 위해 열화상 전용 컬러맵(JET) 적용
                cv_img = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)
            
            self.thermal_img = cv_img
        except Exception as e:
            self.get_logger().error(f"Thermal 변환 에러: {e}")

    def cv_loop(self):
        # 영상이 둘 다 들어오기 전까지 대기 화면 출력
        if self.rgb_img is None or self.thermal_img is None:
            wait_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(wait_img, "Waiting for /camera/color/image_raw and /thermal/video ...", 
                        (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('Interactive Calibration (Astra + Thermal)', wait_img)
            cv2.waitKey(1)
            return

        # 1. 원본 이미지 크기 추출
        h_t, w_t = self.thermal_img.shape[:2]
        h_r, w_r = self.rgb_img.shape[:2]

        # 2. 어파인(Affine) 변환 행렬 생성
        # 회전 중심은 열화상 영상의 중앙
        center = (w_t / 2.0, h_t / 2.0)
        M = cv2.getRotationMatrix2D(center, self.angle, self.scale)
        # 사용자가 입력한 dx, dy만큼 이동 반영
        M[0, 2] += self.dx
        M[1, 2] += self.dy

        # 3. 열화상 영상 및 마스크 변환 (아스트라 카메라 크기에 맞춤)
        warped_t = cv2.warpAffine(self.thermal_img, M, (w_r, h_r))
        
        mask = np.ones((h_t, w_t), dtype=np.uint8) * 255
        warped_mask = cv2.warpAffine(mask, M, (w_r, h_r))
        mask_3ch = cv2.cvtColor(warped_mask, cv2.COLOR_GRAY2BGR)

        # 4. 합성 (Alpha Blending) - 열화상 영상이 있는 부분만 자연스럽게 합성
        bg_roi = cv2.bitwise_and(self.rgb_img, cv2.bitwise_not(mask_3ch))
        fg_roi = cv2.addWeighted(cv2.bitwise_and(self.rgb_img, mask_3ch), 1.0 - self.alpha, warped_t, self.alpha, 0)
        result = cv2.add(bg_roi, fg_roi)

        # 5. UI 텍스트 출력
        ui_text = f"X: {self.dx:.1f} | Y: {self.dy:.1f} | Scale: {self.scale:.2f} | Ang: {self.angle:.1f} | Opacity: {self.alpha:.2f}"
        cv2.putText(result, ui_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3) # 그림자
        cv2.putText(result, ui_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Interactive Calibration (Astra + Thermal)', result)

        # 6. 키보드 입력 처리
        key = cv2.waitKey(1)
        if key == -1:
            return

        char_key = chr(key & 0xFF) if key < 256 else ''

        step = 10 if not (char_key.isupper() and char_key in 'WASD') else 1 # Shift 누르면 미세 조절
        
        if key & 0xFF == 27:  # ESC
            self.get_logger().info("프로그램을 종료합니다.")
            rclpy.shutdown()
            sys.exit(0)
        elif char_key.lower() == 'w': self.dy -= step
        elif char_key.lower() == 's': self.dy += step
        elif char_key.lower() == 'a': self.dx -= step
        elif char_key.lower() == 'd': self.dx += step
        elif char_key == '=' or char_key == '+': self.scale += 0.05
        elif char_key == '-' or char_key == '_': self.scale -= 0.05
        elif char_key.lower() == 'q': self.angle += 1.0
        elif char_key.lower() == 'e': self.angle -= 1.0
        elif char_key == '[': self.alpha = max(0.0, self.alpha - 0.1)
        elif char_key == ']': self.alpha = min(1.0, self.alpha + 0.1)
        elif char_key.lower() == 'p':
            print("\n" + "="*50)
            print("🚀 [최종 산출된 변환 파라미터]")
            print(f"X Offset  : {self.dx}")
            print(f"Y Offset  : {self.dy}")
            print(f"Scale     : {self.scale}")
            print(f"Angle     : {self.angle}")
            print("\n🎯 [실제 적용을 위한 2x3 Affine Matrix (M)]")
            print(repr(M))
            print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    calibrator = InteractiveCalibrator()
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
