#!/usr/bin/python3
import cv2
import threading
import numpy as np
from ultralytics import YOLO
from yolo_msgs.msg import PPEDetect
from std_msgs.msg import Header

import rclpy
from rclpy.node import Node

class PPEDetectorNode(Node):
    def __init__(self):
        super().__init__('ppe_detector_node')
        self.publisher_ = self.create_publisher(PPEDetect, '/class_info', 10)
        
        # 모델 로드 (FP32 강제 설정 포함)
        model_path = "/home/ym/fifo_ws/src/ppe_detector/models/yolo26n_model/origin3_addhand/weights/best.pt"
        self.model = YOLO(model_path)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            exit()
        
        self.get_logger().info(">>> [검증 모드] BGR vs RGB 비교 시작")

    def run_detection_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret: break

            # 1. 원본 (BGR)
            frame_bgr = frame.copy()
            
            # 2. 변환 (RGB) - 여기가 핵심입니다.
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # === 비교 추론 수행 ===
            # 두 가지 버전을 다 넣어봅니다.
            results_bgr = self.model(frame_bgr, verbose=False, conf=0.01, iou=0.5, half=False)
            results_rgb = self.model(frame_rgb, verbose=False, conf=0.01, iou=0.5, half=False)
            
            # === 점수 비교 출력 ===
            max_conf_bgr = 0.0
            max_conf_rgb = 0.0
            
            if len(results_bgr[0].boxes) > 0:
                max_conf_bgr = results_bgr[0].boxes.conf.max().item()
            
            if len(results_rgb[0].boxes) > 0:
                max_conf_rgb = results_rgb[0].boxes.conf.max().item()

            print(f">>> [비교] BGR점수: {max_conf_bgr:.4f} vs RGB점수: {max_conf_rgb:.4f}")

            if max_conf_rgb > 0.5:
                print("!!! RGB 변환이 정답입니다 !!!")
            
            # === (중요) 화면 표시 및 메시지는 '더 높은 점수'가 나온 쪽을 사용 ===
            final_results = results_rgb if max_conf_rgb > max_conf_bgr else results_bgr
            
            # 메시지 발행 준비
            det_classes = []
            det_center_xs = []
            det_center_ys = []
            det_confidences = []
            num_objects = 0

            for r in final_results:
                boxes = r.boxes
                num_objects = len(boxes)
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    
                    det_classes.append(class_name)
                    det_center_xs.append(float(cx))
                    det_center_ys.append(float(cy))
                    det_confidences.append(conf)

                    # 그림 그리기 (화면 표시는 무조건 원본 frame에 해야 색이 맞음)
                    if conf > 0.4: # 시각화는 0.4 이상만
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 메시지 발행
            msg = PPEDetect()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.classes = det_classes
            msg.center_xs = det_center_xs
            msg.center_ys = det_center_ys
            msg.confidences = det_confidences
            self.publisher_.publish(msg)

            cv2.imshow('Comparison Test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PPEDetectorNode()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    try: node.run_detection_loop()
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__':
    main()