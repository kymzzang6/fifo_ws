import cv2
import mediapipe as mp

# 미디어파이프 포즈 모델 로드
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# 웹캠을 열어 실시간으로 영상을 가져옵니다.
cap = cv2.VideoCapture(0)

# 포즈 모델 사용
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("카메라로부터 영상을 가져올 수 없습니다.")
            continue
        
        # BGR 이미지를 RGB로 변환
        frame = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        
        # 프레임을 포즈 모델에 전달하여 포즈를 처리
        results = pose.process(frame)
        
        # RGB 이미지를 다시 BGR로 변환하여 OpenCV에서 사용
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 포즈 랜드마크가 감지되면 랜드마크와 연결선 그리기
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=results.pose_landmarks,
                connections=mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
            )            

        # 결과 화면 출력
        cv2.imshow('Pose Detection', frame)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

# 웹캠을 닫고 모든 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()