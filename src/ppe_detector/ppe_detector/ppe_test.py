import cv2
import math
import time
from ultralytics import YOLO

# 모델 로드
model = YOLO("/home/ym/fifo_ws/src/ppe_detector/models/yolo26n_model/origin3_addhand/weights/best.pt")

# 웹캠 설정
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev_time = 0

print(">>> 다중 객체 실시간 추론 중... 종료하려면 'q'를 누르세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ✅ 수정된 추론 설정 (다중 객체 최적화)
    results = model(
        frame,
        stream=True,
        verbose=False,
        conf=0.75,      # 신뢰도 낮춤 (더 많은 객체 탐지
        iou=0.5,       # NMS IoU 임계값 (0.5: 겹침 허용 증가)
        max_det=10    # 최대 탐지 개수 (기본값 유지)
    )

    for r in results:
        boxes = r.boxes
        
        # ✅ 탐지된 객체 개수 화면에 표시
        num_objects = len(boxes)
        
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            conf = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            class_name = model.names[cls]

            # 박스 그리기 (conf 0.3 이상이면 모두 표시)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 중심점
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            
            # 라벨
            label = f"{class_name} {conf}"
            cv2.putText(frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            boxes = r.boxes

        if len(boxes) > 0:
            print("=== DEBUG YOLO BOXES ===")
            print("boxes.data shape:", boxes.data.shape)
            print("boxes.data example:\n", boxes.data[:5])  # 앞 5개만
            print("boxes.conf:", boxes.conf[:5])
            print("boxes.cls:", boxes.cls[:5])

    # FPS 및 탐지된 객체 수 표시
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    
    cv2.putText(frame, f"FPS: {int(fps)}", (20, 30), 
               cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
    cv2.putText(frame, f"Objects: {num_objects}", (20, 60), 
               cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    cv2.imshow('Multi-Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
