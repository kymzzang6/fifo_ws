import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import mediapipe as mp
from ultralytics import YOLO
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters


class IntegratedPPENode(Node):
    def __init__(self):
        super().__init__('integrated_ppe_node')

        # ========== 파라미터 선언 ==========
        self.declare_parameter('ppe_model_path', '/home/caps/fifo_ws/src/ppe_detector/models/yolo26n_model/origin3_addhand4/weights/best.engine')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('max_det', 10)
        self.declare_parameter('check_list', ["helmet", "vest", "gloves"])
        self.declare_parameter('mp_min_detection_confidence', 0.5)
        self.declare_parameter('mp_min_tracking_confidence', 0.5)
        self.declare_parameter('mp_model_complexity', 1)

        # 파라미터 가져오기
        ppe_model_path      = self.get_parameter('ppe_model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold  = self.get_parameter('iou_threshold').value
        self.max_det        = self.get_parameter('max_det').value
        self.check_targets  = self.get_parameter('check_list').value
        mp_det_conf         = self.get_parameter('mp_min_detection_confidence').value
        mp_trk_conf         = self.get_parameter('mp_min_tracking_confidence').value
        mp_complexity       = self.get_parameter('mp_model_complexity').value

        # ========== 디바이스 설정 ==========
        self.device   = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.use_half = (self.device == 'cuda')
        self.get_logger().info(f'🚀 통합 노드 실행 중 - 디바이스: {self.device}')

        # ========== YOLO PPE 모델 로드 ==========
        self.get_logger().info(f'PPE 모델 로딩 중: {ppe_model_path}')
        self.ppe_model = YOLO(ppe_model_path, task='detect')
        self.get_logger().info('PPE 모델 로딩 완료!')

        # ========== MediaPipe Pose 초기화 ==========
        self.mp_pose    = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=mp_det_conf,
            min_tracking_confidence=mp_trk_conf,
            model_complexity=mp_complexity
        )
        self.get_logger().info(f'MediaPipe Pose 로딩 완료! (complexity={mp_complexity})')

        # ========== Publisher 생성 ==========
        self.pub_safe    = self.create_publisher(Bool,   '/all_detected',   10)
        self.pub_status  = self.create_publisher(String, '/what_detected',  10)
        self.pub_wearing = self.create_publisher(String, '/what_wearing',   10)
        self.pub_human   = self.create_publisher(Bool,   '/human_detected', 10)
        self.pub_depth   = self.create_publisher(Float32,'/person_depth',   10)
        self.pub_debug   = self.create_publisher(Image,  '/ppe_debug',      10)

        # ========== CvBridge 및 Subscriber (RGB + Depth 동기화) ==========
        self.bridge = CvBridge()
        
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')

        # 두 센서 데이터의 시간이 최대한 맞는 짝을 묶어서 콜백으로 보냄
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info('📡 Astra 카메라 RGB + Depth 동기화 구독 시작')

        # ========== 박스 비율 상수 (어깨 너비 기준) ==========
        self.RATIO_HEAD      = 0.35
        self.RATIO_BODY      = 0.40
        self.RATIO_HAND      = 0.25   # 좌우 확장 (기존 0.20 → 0.25)
        self.RATIO_HAND_UP   = 0.25   # 손 박스 위쪽 추가 확장 비율
        self.RATIO_EAR       = 0.10

        self.get_logger().info('✅ 통합 PPE 탐지 노드 시작! (ROS 2 통신 기반)')


    # ===========================================================
    #  동기화 콜백 (컬러 + 뎁스가 세트로 들어올 때마다 자동 실행됨)
    # ===========================================================
    def sync_callback(self, rgb_msg, depth_msg):
        try:
            # ROS 2 Image 메시지를 OpenCV 포맷으로 변환
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # 뎁스 이미지는 Astra의 기본 형식인 16UC1 (16비트 정수 밀리미터) 원본 포맷 유지
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f'⚠️ 이미지 변환 실패: {e}')
            return

        cv_img = frame.copy()
        h_img, w_img = frame.shape[:2]
        d_h, d_w = depth_frame.shape[:2]

        # ========== 1. PPE 탐지 (YOLO Engine) ==========
        ppe_results = self.ppe_model(
            frame,
            stream=True,
            verbose=False,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det,
            device=self.device,
            half=self.use_half
        )

        detected_ppes      = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        raw_detected_names = []

        for r in ppe_results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                cls = int(box.cls[0])

                real_class_name = (
                    self.ppe_model.names[cls]
                    if hasattr(self.ppe_model, 'names')
                    else str(cls)
                )
                lower_name = real_class_name.lower()
                raw_detected_names.append(real_class_name)

                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 255), 1)
                cv2.putText(cv_img, real_class_name, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                entry = (cx, cy, float(x1), float(y1), float(x2), float(y2))

                if "helmet" in lower_name or "hard" in lower_name:
                    detected_ppes["helmet"].append(entry)
                elif "vest" in lower_name:
                    detected_ppes["vest"].append(entry)
                elif "glove" in lower_name:
                    detected_ppes["gloves"].append(entry)
                elif "ear" in lower_name or "plug" in lower_name:
                    detected_ppes["earplug"].append(entry)

        # ========== 2. MediaPipe Pose 추정 ==========
        rgb_frame   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_result = self.pose.process(rgb_frame)

        is_human_detected = (pose_result.pose_landmarks is not None)
        self.pub_human.publish(Bool(data=is_human_detected))

        empty = [-1.0] * 4
        body_boxes = {
            'head':       empty.copy(),
            'body':       empty.copy(),
            'hand_left':  empty.copy(),
            'hand_right': empty.copy(),
            'ear_left':   empty.copy(),
            'ear_right':  empty.copy()
        }

        # 사람의 가슴(Body) 중심 좌표 기록용 변수
        person_center_x, person_center_y = -1, -1

        if is_human_detected:
            lm = pose_result.pose_landmarks.landmark

            def get_pt(idx, vis_thresh=0.5):
                p = lm[idx]
                if p.visibility < vis_thresh:
                    return None
                return np.array([p.x * w_img, p.y * h_img], dtype=float)

            def make_box(center, ratio):
                if center is None:
                    return empty.copy()
                s = (body_width * ratio) / 2.0
                return [
                    center[0] - s, center[1] - s,
                    center[0] + s, center[1] + s
                ]

            def make_hand_box(center, ratio, up_ratio=self.RATIO_HAND_UP):
                if center is None:
                    return empty.copy()
                s      = (body_width * ratio) / 2.0
                up_ext = body_width * up_ratio
                return [
                    center[0] - s,
                    center[1] - s - up_ext,
                    center[0] + s,
                    center[1] + s
                ]

            pt_sh_l = get_pt(11)
            pt_sh_r = get_pt(12)

            if pt_sh_l is not None and pt_sh_r is not None:
                body_width = float(np.linalg.norm(pt_sh_r - pt_sh_l)) * 2.0
            else:
                body_width = w_img * 0.3

            # ── 1. HEAD ──────────────────────────────────────────
            pt_nose  = get_pt(0)
            pt_eye_l = get_pt(2, vis_thresh=0.3)
            pt_eye_r = get_pt(5, vis_thresh=0.3)

            head_c = None
            if pt_nose is not None:
                if pt_eye_l is not None and pt_eye_r is not None:
                    eyes_mid = (pt_eye_l + pt_eye_r) / 2.0
                    head_c = eyes_mid + (eyes_mid - pt_nose) * 2.0
                else:
                    head_c = pt_nose + np.array([0.0, -body_width * 0.15])
            elif pt_sh_l is not None and pt_sh_r is not None:
                sh_c   = (pt_sh_l + pt_sh_r) / 2.0
                head_c = sh_c + np.array([0.0, -body_width * 0.35])

            body_boxes['head'] = make_box(head_c, self.RATIO_HEAD)

            # ── 2. BODY ──────────────────────────────────────────
            pt_hip_l = get_pt(23)
            pt_hip_r = get_pt(24)

            body_c = None
            if pt_sh_l is not None and pt_sh_r is not None:
                sh_c = (pt_sh_l + pt_sh_r) / 2.0
                if pt_hip_l is not None and pt_hip_r is not None:
                    hip_c  = (pt_hip_l + pt_hip_r) / 2.0
                    body_c = sh_c * 0.5 + hip_c * 0.5
                else:
                    body_c = sh_c + np.array([0.0, body_width * 0.2])

            body_boxes['body'] = make_box(body_c, self.RATIO_BODY)
            
            # ★ 사람의 대표 뎁스를 뽑기 위해 Body Center의 좌표를 저장
            if body_c is not None:
                person_center_x, person_center_y = int(body_c[0]), int(body_c[1])

            # ── 3. HAND ──────────────────────────────────────────
            def get_hand_center_direct(indices, vis_thresh=0.3):
                pts = [get_pt(idx, vis_thresh=vis_thresh) for idx in indices]
                valid_pts = [p for p in pts if p is not None]
                if not valid_pts:
                    return None
                return np.mean(valid_pts, axis=0)

            hand_center_l = get_hand_center_direct([15, 17, 19, 21])
            hand_center_r = get_hand_center_direct([16, 18, 20, 22])

            body_boxes['hand_left']  = make_hand_box(hand_center_l, self.RATIO_HAND)
            body_boxes['hand_right'] = make_hand_box(hand_center_r, self.RATIO_HAND)

            # ── 4. EAR ───────────────────────────────────────────
            pt_ear_l = get_pt(7, vis_thresh=0.3)
            pt_ear_r = get_pt(8, vis_thresh=0.3)

            body_boxes['ear_left']  = make_box(pt_ear_l, self.RATIO_EAR)
            body_boxes['ear_right'] = make_box(pt_ear_r, self.RATIO_EAR)

            self.mp_drawing.draw_landmarks(
                image=cv_img,
                landmark_list=pose_result.pose_landmarks,
                connections=self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(
                    color=(0, 0, 255), thickness=2, circle_radius=2
                ),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(
                    color=(0, 255, 0), thickness=2
                )
            )

        # ========== ★ 2-5. 사람의 거리(Depth) 측정 및 토픽 발행 ==========
        if is_human_detected and person_center_x >= 0 and person_center_y >= 0:
            # 해상도가 다를 수 있으므로 컬러 영상 비율에 맞춰 뎁스 이미지 좌표를 매핑
            depth_x = int(person_center_x * (d_w / w_img))
            depth_y = int(person_center_y * (d_h / h_img))
            
            # 뎁스 이미지 인덱스 초과 방지
            depth_x = max(0, min(depth_x, d_w - 1))
            depth_y = max(0, min(depth_y, d_h - 1))
            
            # 배열에서 깊이값(mm) 추출 후 발행
            distance_mm = float(depth_frame[depth_y, depth_x])
            
            depth_msg_out = Float32()
            depth_msg_out.data = distance_mm
            self.pub_depth.publish(depth_msg_out)
            
            # 디버그 텍스트 추가 (보라색 글씨)
            cv2.putText(cv_img, f"Depth: {distance_mm:.0f} mm",
                        (person_center_x - 50, person_center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)


        # ========== 3. 착용 여부 판단 ==========
        wearing_list = set()

        def boxes_intersect(body_box, det_box):
            bx1, by1, bx2, by2 = body_box
            dx1, dy1, dx2, dy2 = det_box
            return not (dx2 < bx1 or dx1 > bx2 or dy2 < by1 or dy1 > by2)

        def check_ppe(bbox, type_key, label, add_to_wearing=True, report_name=None):
            if bbox[0] == -1.0:
                return False

            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

            found = False
            for (px, py, dx1, dy1, dx2, dy2) in detected_ppes[type_key]:
                if boxes_intersect(
                    [float(x1), float(y1), float(x2), float(y2)],
                    [dx1, dy1, dx2, dy2]
                ):
                    found = True
                    cv2.circle(cv_img, (int(px), int(py)), 5, (0, 255, 0), -1)
                    break

            color = (0, 255, 0) if found else (0, 0, 255)
            cv2.putText(cv_img, f"{label}: {'OK' if found else 'NO'}",
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            if found:
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)
                if add_to_wearing and report_name:
                    wearing_list.add(report_name)

            return found

        h_ok = check_ppe(body_boxes['head'],       "helmet",  "Head",   report_name="helmet")
        v_ok = check_ppe(body_boxes['body'],        "vest",    "Body",   report_name="vest")

        l_g  = check_ppe(body_boxes['hand_left'],  "gloves",  "L_Hand", add_to_wearing=False)
        r_g  = check_ppe(body_boxes['hand_right'], "gloves",  "R_Hand", add_to_wearing=False)
        g_ok = (l_g and r_g)

        if g_ok:
            wearing_list.add("gloves")

        l_e  = check_ppe(body_boxes['ear_left'],  "earplug", "L_Ear",  report_name="earplug")
        r_e  = check_ppe(body_boxes['ear_right'], "earplug", "R_Ear",  report_name="earplug")
        e_ok = (l_e and r_e)

        # ========== 4. 결과 전송 ==========
        unique_raw_names = list(set(raw_detected_names))
        self.pub_status.publish(String(data=", ".join(unique_raw_names)))

        wearing_str = ", ".join(sorted(wearing_list))
        self.pub_wearing.publish(String(data=wearing_str))

        is_all_safe = True
        for target in self.check_targets:
            if target == "helmet"  and not h_ok: is_all_safe = False
            if target == "vest"    and not v_ok: is_all_safe = False
            if target == "gloves"  and not g_ok: is_all_safe = False
            if target == "earplug" and not e_ok: is_all_safe = False

        self.pub_safe.publish(Bool(data=is_all_safe))

        # ========== 5. 디버그 UI ==========
        res_color = (0, 255, 0) if is_all_safe else (0, 0, 255)
        cv2.rectangle(cv_img, (0, 0), (320, 60), (0, 0, 0), -1)
        cv2.putText(cv_img,
                    f"Status: {'SAFE' if is_all_safe else 'UNSAFE'}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, res_color, 2)

        human_color = (0, 255, 0) if is_human_detected else (0, 255, 255)
        cv2.putText(cv_img,
                    f"Human: {'YES' if is_human_detected else 'NO'}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, human_color, 2)

        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
        except Exception as e:
            self.get_logger().warn(f'디버그 이미지 전송 실패: {e}')

        if wearing_list or is_human_detected:
            self.get_logger().info(
                f'🧑 사람: {is_human_detected} | 👔 착용: {wearing_str} | 안전: {is_all_safe}',
                throttle_duration_sec=2.0
            )


    # ===========================================================
    def destroy_node(self):
        self.pose.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IntegratedPPENode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
