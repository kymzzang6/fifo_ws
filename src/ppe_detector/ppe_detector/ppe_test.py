import cv2
import math
import time
import numpy as np
from ultralytics import YOLO

# 모델 로드 (경로는 사용자 환경에 맞게 유지)
model = YOLO("/home/ym/fifo_ws/src/ppe_detector/models/yolo26n_model/origin3_addhand3/weights/best.pt")

# 웹캠 설정
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev_time = 0

# ✅ 피부색 범위 설정 (HSV)
lower_skin = np.array([0, 5, 150], dtype=np.uint8)
upper_skin = np.array([20, 255, 255], dtype=np.uint8)

def is_skin_dominant(roi_img, threshold=0.4):
    if roi_img.size == 0: return False, 0.0
    
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_skin, upper_skin)
    
    skin_pixels = cv2.countNonZero(mask)
    total_pixels = roi_img.shape[0] * roi_img.shape[1]
    
    ratio = skin_pixels / total_pixels
    return ratio > threshold, ratio

print(">>> 다중 객체 실시간 추론 중... 종료하려면 'q'를 누르세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(
        frame,
        stream=True,
        verbose=False,
        conf=0.6,
        iou=0.5,
        max_det=10
    )

    for r in results:
        boxes = r.boxes
        
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 좌표 클립핑
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

            conf = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            class_name = model.names[cls]

            # ---------------------------------------------------------
            # [추가 기능] BBox 중심 좌표 및 HSV 추출 디버깅
            # ---------------------------------------------------------
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            # 중심점 좌표가 프레임 내부에 있는지 안전장치
            if 0 <= cx < frame.shape[1] and 0 <= cy < frame.shape[0]:
                # 1. 중심 픽셀 1개의 BGR 값 가져오기
                pixel_bgr = frame[cy, cx]
                
                # 2. BGR -> HSV 변환 (cv2.cvtColor는 3차원 배열을 입력으로 받음)
                pixel_array = np.uint8([[pixel_bgr]])
                pixel_hsv = cv2.cvtColor(pixel_array, cv2.COLOR_BGR2HSV)[0][0]
                
                h_val, s_val, v_val = pixel_hsv
                
                # 디버깅: 콘솔 출력
                print(f"[{class_name}] Center HSV: H={h_val}, S={s_val}, V={v_val}")
                
                # 디버깅: 화면에 중심점과 HSV 값 그리기
                # (중심점: 노란색 원)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                # (HSV 텍스트: 박스 하단에 표시)
                hsv_text = f"HSV: {h_val},{s_val},{v_val}"
                cv2.putText(frame, hsv_text, (cx - 20, cy + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            # ---------------------------------------------------------

            # ROI 추출 및 피부색 검사
            roi = frame[y1:y2, x1:x2]
            is_skin, skin_ratio = is_skin_dominant(roi)
            
            # 로직 보정
            final_label = class_name
            color = (0, 255, 0)

            if class_name == 'gloves' and is_skin:
                final_label = f"Hand (Corr.)"
                color = (0, 0, 255)
                
            # elif class_name == 'hand' and not is_skin:
            #     final_label = f"Gloves (Corr.)"
            #     color = (255, 0, 0)

            # 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label_text = f"{final_label} {conf} (skin:{skin_ratio:.2f})"
            cv2.putText(frame, label_text, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    
    cv2.putText(frame, f"FPS: {int(fps)}", (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
    cv2.imshow('Multi-Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()