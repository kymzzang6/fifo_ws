import cv2

# 1. 웹캠 캡처 객체 생성 (기본 카메라: 0)
cap = cv2.VideoCapture(0)

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("웹캠을 열 수 없습니다. 카메라 연결을 확인해 주세요.")
    exit()

# 2. 저장할 영상의 속성 설정
# 현재 웹캠의 기본 해상도를 가져옵니다.
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 30.0  # 초당 프레임 수 (필요에 따라 조절 가능)

# 영상 코덱 설정 (mp4 파일용 코덱)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

# 저장을 위한 VideoWriter 객체 생성
# (파일명, 코덱, FPS, 화면 크기)
out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

print("녹화를 시작합니다. 종료하려면 키보드에서 'q'를 누르세요.")

# 3. 영상 녹화 루프
while True:
    # 프레임 단위로 영상 읽기
    ret, frame = cap.read()
    
    # 영상을 제대로 읽어오지 못한 경우 종료
    if not ret:
        print("프레임을 받아올 수 없습니다. 녹화를 종료합니다.")
        break

    # 읽어온 프레임을 파일에 저장
    out.write(frame)

    # 화면에도 현재 프레임을 표시
    cv2.imshow('Webcam Recording', frame)

    # 키보드 입력 대기 (1ms 마다 확인)
    # 'q' 키를 누르면 루프 탈출
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 4. 사용한 자원 해제 및 창 닫기
cap.release()          # 웹캠 연결 해제
out.release()          # 저장 객체 해제 (파일이 정상적으로 닫힘)
cv2.destroyAllWindows() # 모든 OpenCV 창 닫기

print("녹화가 종료되었습니다. 'output.mp4' 파일을 확인해 주세요.")