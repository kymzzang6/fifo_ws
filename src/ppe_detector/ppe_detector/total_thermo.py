#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import mediapipe as mp
from ultralytics import YOLO
from std_msgs.msg import Bool, String, Float32, Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
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
        
        ppe_model_path = self.get_parameter('ppe_model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold  = self.get_parameter('iou_threshold').value
        self.max_det        = self.get_parameter('max_det').value
        self.check_targets  = self.get_parameter('check_list').value


        # ========== 센서 퓨전: 좌표 역변환 (RGB -> 열화상) ==========
        self.M = np.array([
            [ 3.18782303,  0.27889838, 48.24025468],
            [-0.27889838,  3.18782303, 51.04248811]
        ], dtype=np.float32)
        self.M_inv = cv2.invertAffineTransform(self.M)


        # ========== 디바이스 및 모델 설정 ==========
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.use_half = (self.device == 'cuda')
        self.get_logger().info(f'🚀 통합 퓨전 노드 실행 중 - 디바이스: {self.device}')


        self.ppe_model = YOLO(ppe_model_path, task='detect')


        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5, model_complexity=1)


        # ========== Publisher 생성 ==========
        self.pub_safe    = self.create_publisher(Bool,   '/all_detected',   10)
        self.pub_status  = self.create_publisher(String, '/what_detected',  10)
        self.pub_wearing = self.create_publisher(String, '/what_wearing',   10)
        self.pub_human   = self.create_publisher(Bool,   '/human_detected', 10)
        self.pub_depth   = self.create_publisher(Float32,'/human_depth',    10)
        self.pub_debug   = self.create_publisher(Image,  '/ppe_debug',      10)
        self.pub_human_pos = self.create_publisher(Point, '/human_pos',     10)


        # C++ 온도 계산 노드로 보낼 좌표 퍼블리셔
        self.pub_face_bbox  = self.create_publisher(Float32MultiArray, '/bbox/face',   10)
    
        # C++ 노드로부터 계산된 온도 수신
        self.create_subscription(Float32, '/thermal/face_temp', self.face_temp_cb, 10)
        
        self.face_temp = 0.0
        self.lhand_temp = 0.0
        self.rhand_temp = 0.0


        # ========== 영상 데이터 Subscriber ==========
        self.bridge = CvBridge()
        self.rgb_sub   = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')


        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)


        self.RATIO_HEAD    = 0.35
        self.RATIO_BODY    = 0.40
        self.RATIO_HAND    = 0.25
        self.RATIO_HAND_UP = 0.25
        self.RATIO_EAR     = 0.10


    # === C++ 노드 온도 수신 콜백 ===
    def face_temp_cb(self, msg): self.face_temp = msg.data
    def lhand_temp_cb(self, msg): self.lhand_temp = msg.data
    def rhand_temp_cb(self, msg): self.rhand_temp = msg.data


    # === BBox 역변환 및 송신 ===
    def publish_thermal_bbox(self, publisher, rgb_box):
        msg = Float32MultiArray()
        if rgb_box[0] == -1.0:
            msg.data = [-1.0, -1.0, -1.0, -1.0]
        else:
            pt1 = np.array([[[rgb_box[0], rgb_box[1]]]], dtype=np.float32)
            pt2 = np.array([[[rgb_box[2], rgb_box[3]]]], dtype=np.float32)
            
            t_pt1 = cv2.transform(pt1, self.M_inv)[0][0]
            t_pt2 = cv2.transform(pt2, self.M_inv)[0][0]
            msg.data = [float(t_pt1[0]), float(t_pt1[1]), float(t_pt2[0]), float(t_pt2[1])]
        publisher.publish(msg)


    # === 교차 영역 확인 함수 ===
    def boxes_intersect(self, body_box, det_box):
        bx1, by1, bx2, by2 = body_box
        dx1, dy1, dx2, dy2 = det_box
        return not (dx2 < bx1 or dx1 > bx2 or dy2 < by1 or dy1 > by2)


    # === 메인 콜백 ===
    def sync_callback(self, rgb_msg, depth_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f'⚠️ 이미지 변환 실패: {e}')
            return


        cv_img = frame.copy()
        h_img, w_img = frame.shape[:2]
        d_h, d_w = depth_frame.shape[:2]


        # ========== 1. YOLO PPE 탐지 ==========
        ppe_results = self.ppe_model(
            frame, stream=True, verbose=False, conf=self.conf_threshold, 
            iou=self.iou_threshold, max_det=self.max_det, device=self.device, half=self.use_half
        )
        
        # 🌟 YOLO의 glove와 hand 클래스를 둘 다 추적하도록 복구
        detected_ppes = {"helmet": [], "vest": [], "gloves": [], "earplug": [], "hand": []}  
        raw_detected_names = []


        for r in ppe_results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)
                cls = int(box.cls[0])
                real_class_name = self.ppe_model.names[cls] if hasattr(self.ppe_model, 'names') else str(cls)
                lower_name = real_class_name.lower()
                raw_detected_names.append(real_class_name)


                # 화면에 YOLO 바운딩 박스 그리기
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 255), 1)
                cv2.putText(cv_img, real_class_name, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


                entry = (cx, cy, float(x1), float(y1), float(x2), float(y2))
                if "helmet" in lower_name or "hard" in lower_name: detected_ppes["helmet"].append(entry)
                elif "vest" in lower_name: detected_ppes["vest"].append(entry)
                elif "ear" in lower_name or "plug" in lower_name: detected_ppes["earplug"].append(entry)
                elif "glove" in lower_name: detected_ppes["gloves"].append(entry)
                elif "hand" in lower_name: detected_ppes["hand"].append(entry)


        # ========== 2. MediaPipe Pose 탐지 ==========
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_result = self.pose.process(rgb_frame)


        is_human_detected = (pose_result.pose_landmarks is not None)
        self.pub_human.publish(Bool(data=is_human_detected))


        empty = [-1.0] * 4
        body_boxes = {'head': empty.copy(), 'body': empty.copy(), 'hand_left': empty.copy(), 'hand_right': empty.copy(), 'ear_left': empty.copy(), 'ear_right': empty.copy()}
        person_center_x, person_center_y = -1, -1


        if is_human_detected:
            lm = pose_result.pose_landmarks.landmark
            def get_pt(idx, vis=0.5): return np.array([lm[idx].x * w_img, lm[idx].y * h_img], dtype=float) if lm[idx].visibility >= vis else None
            def make_box(c, ratio, up_r=0): return [c[0] - (body_width*ratio/2), c[1] - (body_width*ratio/2) - (body_width*up_r), c[0] + (body_width*ratio/2), c[1] + (body_width*ratio/2)] if c is not None else empty.copy()


            pt_sh_l, pt_sh_r = get_pt(11), get_pt(12)
            body_width = float(np.linalg.norm(pt_sh_r - pt_sh_l)) * 2.0 if pt_sh_l is not None and pt_sh_r is not None else w_img * 0.3


            pt_nose = get_pt(0)
            head_c = pt_nose + np.array([0.0, -body_width * 0.15]) if pt_nose is not None else None
            if head_c is None and pt_sh_l is not None and pt_sh_r is not None: head_c = ((pt_sh_l + pt_sh_r) / 2.0) + np.array([0.0, -body_width * 0.35])
            body_boxes['head'] = make_box(head_c, self.RATIO_HEAD)


            pt_hip_l, pt_hip_r = get_pt(23), get_pt(24)
            body_c = (pt_sh_l + pt_sh_r) / 2.0 if pt_sh_l is not None and pt_sh_r is not None else None
            if body_c is not None and pt_hip_l is not None and pt_hip_r is not None: body_c = body_c * 0.5 + ((pt_hip_l + pt_hip_r) / 2.0) * 0.5
            body_boxes['body'] = make_box(body_c, self.RATIO_BODY)
            if body_c is not None: person_center_x, person_center_y = int(body_c[0]), int(body_c[1])


            def get_hand_center(indices):
                valid = [get_pt(i, 0.3) for i in indices if get_pt(i, 0.3) is not None]
                return np.mean(valid, axis=0) if valid else None
            body_boxes['hand_left']  = make_box(get_hand_center([15, 17, 19, 21]), self.RATIO_HAND, self.RATIO_HAND_UP)
            body_boxes['hand_right'] = make_box(get_hand_center([16, 18, 20, 22]), self.RATIO_HAND, self.RATIO_HAND_UP)
            body_boxes['ear_left']  = make_box(get_pt(7, 0.3), self.RATIO_EAR)
            body_boxes['ear_right'] = make_box(get_pt(8, 0.3), self.RATIO_EAR)


        wearing_list = set()
        
        # C++ 노드로 좌표 퍼블리시
        self.publish_thermal_bbox(self.pub_face_bbox, body_boxes['head'])
 
        if is_human_detected and person_center_x >= 0 and person_center_y >= 0:
            dx = max(0, min(int(person_center_x * (d_w / w_img)), d_w - 1))
            dy = max(0, min(int(person_center_y * (d_h / h_img)), d_h - 1))
            dist_mm = float(depth_frame[dy, dx])
            self.pub_depth.publish(Float32(data=dist_mm))

            # ✅ MediaPipe 기반의 중심점을 사용하여 /human_pos 발행
            pos_msg = Point()
            pos_msg.x = float(person_center_x - (w_img / 2.0))
            pos_msg.y = 0.0
            pos_msg.z = float(dist_mm)
            self.pub_human_pos.publish(pos_msg)
            
            if self.face_temp > 0.0:
                cv2.putText(cv_img, f"Face: {self.face_temp:.1f}C", (int(body_boxes['head'][0]), int(body_boxes['head'][1]) - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)


        def check_yolo(bbox, type_key, label, report_name):
            if bbox[0] == -1.0: return False
            x1, y1, x2, y2 = map(int, bbox)
            found = any(not (dx2 < x1 or dx1 > x2 or dy2 < y1 or dy1 > y2) for (_, _, dx1, dy1, dx2, dy2) in detected_ppes[type_key])
            color = (0, 255, 0) if found else (0, 0, 255)
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(cv_img, f"{label}: {'OK' if found else 'NO'}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            if found: wearing_list.add(report_name)
            return found


        h_ok = check_yolo(body_boxes['head'], "helmet", "Head", "helmet")
        v_ok = check_yolo(body_boxes['body'], "vest", "Body", "vest")
        e_ok = (check_yolo(body_boxes['ear_left'], "earplug", "L_Ear", "earplug") and check_yolo(body_boxes['ear_right'], "earplug", "R_Ear", "earplug"))


        # ========== 🌟 3. 장갑 착용 여부 판단 (YOLO 전용으로 수정) ==========
        def check_fusion_glove(hand_box, temp, label):
            if hand_box[0] == -1.0: return False
            
            # YOLO 결과 확인 (MediaPipe 손 박스와 겹치는지)
            yolo_glove_found = any(self.boxes_intersect(hand_box, [dx1, dy1, dx2, dy2]) for (_, _, dx1, dy1, dx2, dy2) in detected_ppes["gloves"])
            yolo_hand_found = any(self.boxes_intersect(hand_box, [dx1, dy1, dx2, dy2]) for (_, _, dx1, dy1, dx2, dy2) in detected_ppes["hand"])
            
            is_glove = False
            state_text = "Unknown"
            
            # 조건 1: YOLO가 장갑이라 인식한 경우
            if yolo_glove_found:
                is_glove = True
                state_text = "Gloves"
            
            # 조건 2: YOLO가 맨손이라 인식한 경우
            if yolo_hand_found:
                is_glove = False
                state_text = "Hand"
                
            if not is_glove and state_text == "Unknown":
                state_text = "Hand"


            x1, y1, x2, y2 = map(int, hand_box)
            color = (0, 255, 0) if is_glove else (0, 0, 255)
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(cv_img, f"{label}: {state_text}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            return is_glove


        # 기존 코드 호환을 위해 temp 매개변수 구조는 유지합니다.
        l_glove_ok = check_fusion_glove(body_boxes['hand_left'], self.lhand_temp, "L_Hand")
        r_glove_ok = check_fusion_glove(body_boxes['hand_right'], self.rhand_temp, "R_Hand")
        g_ok = (l_glove_ok and r_glove_ok)


        if g_ok: wearing_list.add("gloves")


        # ========== 4. 상태 퍼블리시 및 디버그 UI ==========
        self.pub_status.publish(String(data=", ".join(list(set(raw_detected_names)))))
        self.pub_wearing.publish(String(data=", ".join(sorted(wearing_list))))


        # 1. 각 대상별 현재 착용 상태를 딕셔너리로 매핑
        status_map = {
            "helmet": h_ok,
            "vest": v_ok,
            "gloves": g_ok,
            "earplug": e_ok
        }


        # 2. check_targets 리스트에 있는 항목들만 검사하여 모두 True인지 확인
        is_all_safe = all(status_map.get(t, True) for t in self.check_targets)



        self.pub_safe.publish(Bool(data=is_all_safe))


        res_color = (0, 255, 0) if is_all_safe else (0, 0, 255)
        cv2.rectangle(cv_img, (0, 0), (320, 60), (0, 0, 0), -1)
        cv2.putText(cv_img, f"Status: {'SAFE' if is_all_safe else 'UNSAFE'}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, res_color, 2)
        cv2.putText(cv_img, f"Human: {'YES' if is_human_detected else 'NO'}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if is_human_detected else (0, 255, 255), 2)


        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
        except: pass


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
