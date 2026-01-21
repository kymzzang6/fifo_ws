import cv2
from ultralytics import YOLOWorld
import time

# ==========================================
# 1. 모델 설정
# ==========================================
# 가벼운 's' 버전 모델 로드 (자동으로 다운로드 됨)
# 성능이 더 필요하면 'yolov8m-worldv2.pt' 또는 'yolov8l-worldv2.pt' 사용 가능
print("모델을 로드 중입니다... 잠시만 기다려주세요.")
model = YOLOWorld('yolov8s-worldv2.pt')

# 탐지하고 싶은 부위를 텍스트로 지정 (Custom Classes)
# 한글 대신 영어로 입력해야 정확도가 높습니다.
target_classes = ["human head", "human hand", "human torso"]
model.set_classes(target_classes)

# ==========================================
# 2. 웹캠 설정 및 실시간 루프
# ==========================================
cap = cv2.VideoCapture(0)  # 0번은 보통 기본 웹캠, 안되면 1번으로 변경

# 화면에 표시할 색상 (B, G, R) - 순서대로 머리, 손, 몸통
colors = {
    "human head": (0, 255, 255),  # 노란색
    "human hand": (0, 0, 255),    # 빨간색
    "human torso": (255, 0, 0)    # 파란색
}

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("탐지를 시작합니다. 종료하려면 'q'를 누르세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3. 모델 추론 (confidence threshold 0.2 이상만 탐지)
    # stream=True 옵션은 메모리 효율성을 위해 제너레이터 반환
    results = model.predict(frame, conf=0.2, verbose=False)

    # 4. 결과 시각화 (OpenCV로 직접 그리기)
    for result in results:
        boxes = result.boxes  # 감지된 박스 정보들
        
        for box in boxes:
            # 좌표 가져오기 (x1, y1, x2, y2)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # 클래스 이름 및 신뢰도 가져오기
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            
            # 현재 탐지된 클래스 이름 (예: 'human hand')
            # model.names는 전체 클래스 리스트가 아니라, 현재 set_classes로 지정된 것들 기준일 수 있음
            # 안전하게 target_classes 리스트나 result.names 참조
            cls_name = model.names[cls_id]

            # 우리가 찾는 타겟인지 확인 (혹시 모르니)
            if cls_name in target_classes:
                color = colors.get(cls_name, (0, 255, 0)) # 지정된 색 없으면 초록색
                
                # BBox 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # 라벨 텍스트 (예: Hand 0.85)
                label = f"{cls_name.split()[-1]} {conf:.2f}" # 'human hand' -> 'hand'만 표시
                
                # 텍스트 배경 박스 그리기 (가독성 위해)
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(frame, (x1, y1 - 20), (x1 + w, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    # 화면 출력
    cv2.imshow('YOLO-World Body Part Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
