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
        # 이제 거리 threshold 대신, bbox 포함 여부를 보므로 거리 파라미터는 덜 중요해졌지만
        # 박스를 살짝 벗어난 경우도 허용하고 싶다면 margin을 줄 수 있습니다.
        self.declare_parameter('check_list', ["helmet", "vest", "gloves"]) 
        self.check_targets = self.get_parameter('check_list').value
        
        self.get_logger().info(f"Check List: {self.check_targets}")

        # --- 2. 구독 (3개 토픽 동기화) ---
        self.pose_sub = message_filters.Subscriber(self, PoseDetect, '/body_points')
        self.class_sub = message_filters.Subscriber(self, PPEDetect, '/class_info')
        self.img_sub = message_filters.Subscriber(self, Image, '/image_raw') 

        # 3개 토픽 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pose_sub, self.class_sub, self.img_sub], 
            queue_size=10, 
            slop=0.5, # 동기화 허용 오차 (초)
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        # --- 3. 퍼블리셔 ---
        self.all_detected_pub = self.create_publisher(Bool, '/all_detected', 10)
        self.what_detected_pub = self.create_publisher(String, '/what_detected', 10)
        self.debug_image_pub = self.create_publisher(Image, '/ppe_debug', 10)

        self.bridge = CvBridge()

    def sync_callback(self, pose_msg, class_msg, img_msg):
        # 1. 이미지 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 2. PPE 파싱 (중심점 추출)
        # class_msg는 [class_name, cx, cy] 정보를 담고 있다고 가정
        detected_objects = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        
        if len(class_msg.classes) > 0:
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
        
        # --- 핵심 로직: Point in Box 확인 ---
        def is_point_in_box(point, bbox):
            px, py = point
            x1, y1, x2, y2 = bbox
            # BBox가 유효하지 않으면(-1) False
            if x1 == -1: return False
            
            # 박스 안에 점이 있는지 확인 (경계선 포함)
            return (x1 <= px <= x2) and (y1 <= py <= y2)

        def check_and_draw(body_bbox, target_key, label):
            """
            body_bbox: [x1, y1, x2, y2]
            target_key: 'helmet', 'vest' ...
            """
            # 신체 부위가 감지되지 않았으면(-1) 스킵
            if body_bbox[0] == -1.0: 
                return False
            
            x1, y1, x2, y2 = map(int, body_bbox)
            
            # 신체 부위 박스 그리기 (파란색)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            is_matched = False
            matched_obj = None

            # 해당 부위 박스 안에 들어오는 PPE가 있는지 검사
            for (ox, oy) in detected_objects[target_key]:
                if is_point_in_box((ox, oy), body_bbox):
                    is_matched = True
                    matched_obj = (int(ox), int(oy))
                    break # 하나라도 찾으면 OK
            
            # 결과 표시
            if is_matched:
                # 매칭된 PPE 표시 (초록색 점)
                if matched_obj:
                    cv2.circle(cv_image, matched_obj, 8, (0, 255, 0), -1)
                
                # 텍스트 표시
                cv2.putText(cv_image, f"{label}: OK", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # 박스 색상을 초록으로 덧칠
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            else:
                # 매칭 실패 (빨간색 텍스트)
                cv2.putText(cv_image, f"{label}: Missing", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            return is_matched

        # (1) Head ↔ Helmet
        status["helmet"] = check_and_draw(pose_msg.head, "helmet", "Helmet")

        # (2) Body ↔ Vest
        status["vest"] = check_and_draw(pose_msg.body, "vest", "Vest")

        # (3) Hands ↔ Gloves
        l_ok = check_and_draw(pose_msg.hand_left, "gloves", "L_Glove")
        r_ok = check_and_draw(pose_msg.hand_right, "gloves", "R_Glove")
        
        # 로직: 손이 화면에 안 보이면(-1) '검사 제외(Pass)'로 칠지, '불합격'으로 칠지 결정해야 함.
        # 여기서는 "화면에 보이는 손은 반드시 장갑을 껴야 한다"는 로직 적용
        
        # 왼손 검사: 안보이면 True(통과), 보이면 l_ok 결과 따름
        l_check = True if pose_msg.hand_left[0] == -1 else l_ok
        # 오른손 검사
        r_check = True if pose_msg.hand_right[0] == -1 else r_ok
        
        # 양손 다 안 보이면 -> False (작업자가 없는 것으로 간주하거나, 상황에 따라 True로 변경 가능)
        if pose_msg.hand_left[0] == -1 and pose_msg.hand_right[0] == -1:
            status["gloves"] = False 
        else:
            status["gloves"] = l_check and r_check

        # (4) Ears ↔ Earplugs (같은 로직)
        el_ok = check_and_draw(pose_msg.ear_left, "earplug", "L_Ear")
        er_ok = check_and_draw(pose_msg.ear_right, "earplug", "R_Ear")
        
        if pose_msg.ear_left[0] == -1 and pose_msg.ear_right[0] == -1:
            status["earplug"] = False # 귀가 아예 안 보이면 검사 불가
        else:
            el_check = True if pose_msg.ear_left[0] == -1 else el_ok
            er_check = True if pose_msg.ear_right[0] == -1 else er_ok
            status["earplug"] = el_check and er_check


        # 4. 결과 퍼블리시
        detected_list = [k for k, v in status.items() if v]
        msg_str = String()
        msg_str.data = ", ".join(detected_list)
        self.what_detected_pub.publish(msg_str)

        all_safe = True
        for target in self.check_targets:
            # 타겟 리스트에 있는 항목 중 하나라도 False면 불합격
            if target in status and not status[target]:
                all_safe = False
                break
        
        msg_bool = Bool()
        msg_bool.data = all_safe
        self.all_detected_pub.publish(msg_bool)

        # 5. 디버그 이미지 발행
        color = (0, 255, 0) if all_safe else (0, 0, 255)
        text = "SAFE" if all_safe else "UNSAFE"
        
        # 화면 상단에 상태 표시 바 그리기
        cv2.rectangle(cv_image, (0, 0), (cv_image.shape[1], 40), (0,0,0), -1)
        cv2.putText(cv_image, f"Status: {text}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
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
