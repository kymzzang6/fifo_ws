import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import math

# 커스텀 메시지
from yolo_msgs.msg import PoseDetect, PPEDetect

class PPEMatcherNode(Node):
    def __init__(self):
        super().__init__('ppe_matcher_node')

        # --- 1. 파라미터 ---
        self.declare_parameter('match_threshold', 20.0) # ppe ~ pose 좌표 거리
        self.match_threshold = self.get_parameter('match_threshold').value

        self.declare_parameter('check_list', ["helmet", "vest", "gloves"]) 
        self.check_targets = self.get_parameter('check_list').value
        
        self.get_logger().info(f"Check List: {self.check_targets}")

        # --- 2. 구독 (3개 토픽 동기화) ---
        self.pose_sub = message_filters.Subscriber(self, PoseDetect, '/body_points')
        self.class_sub = message_filters.Subscriber(self, PPEDetect, '/class_info')
        self.img_sub = message_filters.Subscriber(self, Image, '/image_raw') # 디버그 그리기용 원본

        # 3개 토픽 동기화 (allow_headerless=True 추가됨)
        # 이 옵션은 헤더가 없거나 타임스탬프가 비어있는 메시지도 
        # 도착한 시간(ROS Time) 기준으로 동기화 해줍니다.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pose_sub, self.class_sub, self.img_sub], 
            queue_size=10, 
            slop=1.0,
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        # --- 3. 퍼블리셔 ---
        self.all_detected_pub = self.create_publisher(Bool, '/all_detected', 10)
        self.what_detected_pub = self.create_publisher(String, '/what_detected', 10)
        self.debug_image_pub = self.create_publisher(Image, '/ppe_debug', 10) # 디버그 이미지

        self.bridge = CvBridge()

    def sync_callback(self, pose_msg, class_msg, img_msg):
        # 1. 이미지 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 2. PPE 파싱
        detected_objects = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        for cls, cx, cy in zip(class_msg.classes, class_msg.center_xs, class_msg.center_ys):
            cls_lower = cls.lower()
            if "helmet" in cls_lower or "hardhat" in cls_lower:
                detected_objects["helmet"].append((cx, cy))
            elif "vest" in cls_lower:
                detected_objects["vest"].append((cx, cy))
            elif "glove" in cls_lower:
                detected_objects["gloves"].append((cx, cy))
            elif "ear" in cls_lower or "plug" in cls_lower:
                detected_objects["earplug"].append((cx, cy))

        # 3. 매칭 및 상태 체크
        status = {"helmet": False, "vest": False, "gloves": False, "earplug": False}
        
        # 디버그 그리기 헬퍼 함수
        def draw_line(start_pt, end_pt, color=(0, 255, 0)):
            cv2.line(cv_image, (int(start_pt[0]), int(start_pt[1])), 
                     (int(end_pt[0]), int(end_pt[1])), color, 2)

        def check_and_draw(body_pt, target_key, label):
            """거리 체크하고 선 그리기"""
            if body_pt[0] == -1.0: return False
            
            bx, by = body_pt
            min_dist = float('inf')
            closest_obj = None

            # 가장 가까운 장비 찾기
            for (ox, oy) in detected_objects[target_key]:
                dist = math.hypot(bx - ox, by - oy)
                if dist < min_dist:
                    min_dist = dist
                    closest_obj = (ox, oy)
            
            # 매칭 성공 시 초록선, 실패 시 빨간선(혹은 안 그림)
            is_matched = min_dist < self.match_threshold
            
            if is_matched and closest_obj:
                draw_line((bx, by), closest_obj, (0, 255, 0)) # Green Line
                cv2.putText(cv_image, f"{label}: OK", (int(bx), int(by)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                # 매칭 실패해도 신체 부위는 표시
                cv2.circle(cv_image, (int(bx), int(by)), 5, (0, 0, 255), -1)
                cv2.putText(cv_image, f"{label}: X", (int(bx), int(by)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            return is_matched

        # (1) Head ↔ Helmet
        status["helmet"] = check_and_draw(pose_msg.head, "helmet", "Helmet")

        # (2) Body ↔ Vest
        status["vest"] = check_and_draw(pose_msg.body, "vest", "Vest")

        # (3) Hands ↔ Gloves
        l_ok = check_and_draw(pose_msg.hand_left, "gloves", "L_Glove")
        r_ok = check_and_draw(pose_msg.hand_right, "gloves", "R_Glove")
        
        # 손이 안 보이면 False, 하나라도 보이면 그 손에 대한 결과 반영
        if pose_msg.hand_left[0] == -1 and pose_msg.hand_right[0] == -1:
            status["gloves"] = False
        else:
            # 보이는 손은 다 착용해야 OK (한쪽만 보이면 그 쪽만 체크)
            # 로직: (왼손안보임 OR 왼손착용) AND (오른손안보임 OR 오른손착용)
            l_check = (pose_msg.hand_left[0] == -1) or l_ok
            r_check = (pose_msg.hand_right[0] == -1) or r_ok
            status["gloves"] = l_check and r_check

        # (4) Ears ↔ Earplugs
        el_ok = check_and_draw(pose_msg.ear_left, "earplug", "L_Ear")
        er_ok = check_and_draw(pose_msg.ear_right, "earplug", "R_Ear")
        
        if pose_msg.ear_left[0] == -1 and pose_msg.ear_right[0] == -1:
            status["earplug"] = False
        else:
            el_check = (pose_msg.ear_left[0] == -1) or el_ok
            er_check = (pose_msg.ear_right[0] == -1) or er_ok
            status["earplug"] = el_check and er_check

        # 4. 결과 퍼블리시
        detected_list = [k for k, v in status.items() if v]
        msg_str = String()
        msg_str.data = ", ".join(detected_list)
        self.what_detected_pub.publish(msg_str)

        all_safe = True
        for target in self.check_targets:
            if target in status and not status[target]:
                all_safe = False
                break
        
        msg_bool = Bool()
        msg_bool.data = all_safe
        self.all_detected_pub.publish(msg_bool)

        # 5. 디버그 이미지 발행 (화면에 전체 결과 표시)
        color = (0, 255, 0) if all_safe else (0, 0, 255)
        text = "ALL SAFE" if all_safe else "UNSAFE"
        cv2.putText(cv_image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
        
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = PPEMatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
