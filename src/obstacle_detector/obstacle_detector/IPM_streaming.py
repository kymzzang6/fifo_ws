import cv2
import numpy as np
import os

def test_golden_reference_detection(video_source):
    cap = cv2.VideoCapture(video_source)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if not cap.isOpened():
        print("❌ 카메라 영상을 불러올 수 없습니다. 카메라 연결을 확인해주세요.")
        return

    # 1. 카메라 캘리브레이션 파라미터 (기존 동일)
    camera_matrix = np.array([
        [468.85160531,   0.        , 296.98624005],
        [  0.        , 470.4410217 , 262.10497125],
        [  0.        ,   0.        ,   1.        ]
    ])
    dist_coeffs = np.array([ 0.07647929, -0.04246   ,  0.0021503 ,  0.00667308, -0.01955115])

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 2. IPM 변환 행렬 (기존 동일)
    src_points = np.float32([
        [0, height * 0.5],       
        [width, height * 0.5],   
        [width, height],          
        [0, height]               
    ])
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    ipm_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    
    reference_img = None
    reference_edges = None
    ref_path = "reference_floor.jpg"

    print("✅ 엣지 기반 실시간 테스트 분석을 시작합니다. (종료: 'q', 일시정지: '스페이스바')")
    print("⚠️ 주의: 프로그램 시작 시 카메라는 '장애물이 없는 빈 바닥'을 보고 있어야 합니다!")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # [단계 1] 왜곡 보정 및 IPM 변환
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix)
        ipm_img = cv2.warpPerspective(undistorted, ipm_matrix, (width, height))
        gray_ipm = cv2.cvtColor(ipm_img, cv2.COLOR_BGR2GRAY)

        # 💡 [단계 2] 엣지 추출용 전처리 (타협점 적용)
        normalized_ipm = clahe.apply(gray_ipm)
        edge_blur = cv2.GaussianBlur(normalized_ipm, (11, 11), 0) # 13->11로 살짝 낮춤
        current_edges = cv2.Canny(edge_blur, 70, 150)             # 너무 둔감하지 않게 임계값 조절

        # 💡 [단계 3] 골든 레퍼런스(기준 바닥) 로드
        if reference_img is None:
            if os.path.exists(ref_path):
                ref_loaded = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
                if ref_loaded is not None and ref_loaded.shape == gray_ipm.shape:
                    reference_img = ref_loaded
                    ref_blur = cv2.GaussianBlur(clahe.apply(reference_img), (11, 11), 0)
                    reference_edges = cv2.Canny(ref_blur, 70, 150)
                else:
                    cv2.imwrite(ref_path, gray_ipm)
                    reference_img = gray_ipm.copy()
                    reference_edges = current_edges.copy()
            else:
                cv2.imwrite(ref_path, gray_ipm)
                reference_img = gray_ipm.copy()
                reference_edges = current_edges.copy()
            continue
        
        # 💡 [단계 4] 엣지 차영상
        dilate_kernel = np.ones((5, 5), np.uint8)
        dilated_ref_edges = cv2.dilate(reference_edges, dilate_kernel, iterations=1)
        diff_edges = cv2.subtract(current_edges, dilated_ref_edges)
        _, thresh = cv2.threshold(diff_edges, 127, 255, cv2.THRESH_BINARY)

        # 💡 [단계 5] 형태학적 연산 (여기가 핵심 변경점!)
        # 1. 선과 선 사이를 강력하게 이어 붙이기 위해 커널을 15x15로 거대하게 만듭니다.
        close_kernel = np.ones((15, 15), np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel, iterations=3)
        
        # 2. 내부를 확실하게 칠하기 위해 한 번 강하게 팽창시킵니다.
        fill_kernel = np.ones((5, 5), np.uint8)
        morph = cv2.dilate(morph, fill_kernel, iterations=2)
        
        # 3. 마지막으로 남아있는 미세한 바닥 질감 노이즈를 지웁니다.
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, fill_kernel, iterations=1)

        # [단계 5.5] 동적 마스킹 (기존 동일)
        lines = cv2.HoughLinesP(current_edges, 1, np.pi/180, threshold=100, minLineLength=150, maxLineGap=20)
        left_rack_x, right_rack_x = None, None
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(x2 - x1) < 30:
                    if min(y1, y2) < height * 0.1 or max(y1, y2) > height * 0.9:
                        if x1 < width // 2: 
                            left_rack_x = max(x1, x2) if left_rack_x is None else max(left_rack_x, max(x1, x2))
                        else:               
                            right_rack_x = min(x1, x2) if right_rack_x is None else min(right_rack_x, min(x1, x2))

        max_mask_limit = int(width * 0.3)
        if left_rack_x is not None and left_rack_x > max_mask_limit: left_rack_x = max_mask_limit
        if right_rack_x is not None and right_rack_x < width - max_mask_limit: right_rack_x = width - max_mask_limit

        if left_rack_x is not None: cv2.rectangle(morph, (0, 0), (left_rack_x, height), 0, -1)
        if right_rack_x is not None: cv2.rectangle(morph, (right_rack_x, 0), (width, height), 0, -1)

        # [단계 6] 덩어리 판별 및 필터링
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_ipm = ipm_img.copy()

        if left_rack_x is not None: cv2.line(debug_ipm, (left_rack_x, 0), (left_rack_x, height), (255, 0, 0), 2)
        if right_rack_x is not None: cv2.line(debug_ipm, (right_rack_x, 0), (right_rack_x, height), (255, 0, 0), 2)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            aspect_ratio = float(w) / h if h != 0 else 0

            # 윤곽선이 꽉 찬 박스가 되었으므로 실제 장애물은 면적이 수천~수만이 됩니다.
            # 1000 이하는 무조건 노이즈로 봅니다.
            if area < 1000:
                continue

            is_touching_left_rack = (left_rack_x is not None) and (x <= left_rack_x + 5)
            is_touching_right_rack = (right_rack_x is not None) and (x + w >= right_rack_x - 5)
            is_abnormal_ratio = aspect_ratio > 3.0 or aspect_ratio < 0.3

            if (is_touching_left_rack or is_touching_right_rack) and is_abnormal_ratio:
                cv2.rectangle(debug_ipm, (x, y), (x + w, y + h), (0, 0, 255), 2)
            else:
                cv2.rectangle(debug_ipm, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.putText(debug_ipm, "OBSTACLE", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 화면 출력
        resized_original = cv2.resize(undistorted, (width // 2, height // 2))
        resized_debug = cv2.resize(debug_ipm, (width // 2, height // 2))
        combined_view = np.hstack((resized_original, resized_debug))

        cv2.imshow("Left: Original | Right: Target Detection", combined_view)
        cv2.imshow("Edge Difference Mask", cv2.resize(morph, (width // 2, height // 2)))

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'): break
        elif key == ord(' '): cv2.waitKey(0)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_index = '/dev/v4l/by-id/usb-SN0002_1080P_USB_Camera_44434000_P030C01_SN0002-video-index0'
    camera_index = os.path.realpath(camera_index)
    test_golden_reference_detection(camera_index)