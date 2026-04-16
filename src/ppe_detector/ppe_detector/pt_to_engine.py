from ultralytics import YOLO

from ultralytics import YOLO

# 1. 모델 경로 설정 (본인의 .pt 파일 경로로 수정)
model_path = '/home/caps/fifo_ws/src/ppe_detector/models/yolo26s_model/origin3_addhand5/weights/best.pt'

# 2. 모델 로드
model = YOLO(model_path)

# 3. TensorRT 엔진으로 변환 (export)
# imgsz=[480, 640] -> (높이, 너비) 순서입니다.
# half=True -> FP16 모드 (속도 향상, 메모리 절약)
# device=0 -> GPU 사용
success = model.export(
    format='engine',
    imgsz=[480, 640],  # (Height, Width)
    half=True,
    device=0,
    simplify=True      # ONNX 단순화 (권장)
)

print(f"변환 완료! 저장된 경로: {success}")