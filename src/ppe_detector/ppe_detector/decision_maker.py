import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
from yolo_msgs.msg import PoseDetect, PPEDetect

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.declare_parameter('check_list', ["helmet", "vest", "gloves"]) 
        self.check_targets = self.get_parameter('check_list').value

        # 동기화 구독
        self.pose_sub = message_filters.Subscriber(self, PoseDetect, '/body_points')
        self.class_sub = message_filters.Subscriber(self, PPEDetect, '/class_info')
        self.img_sub = message_filters.Subscriber(self, Image, '/image_raw') 

        # [최적화] slop=0.1~0.2 (동기화 여유 시간)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pose_sub, self.class_sub, self.img_sub], 
            queue_size=10, slop=0.15, allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        self.pub_safe = self.create_publisher(Bool, '/all_detected', 10)
        self.pub_status = self.create_publisher(String, '/what_detected', 10)
        self.pub_debug = self.create_publisher(Image, '/ppe_debug', 10)
        self.bridge = CvBridge()

    def sync_callback(self, pose_msg, class_msg, img_msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except: return

        # 1. PPE 위치 파싱
        detected_ppes = {"helmet": [], "vest": [], "gloves": [], "earplug": []}
        for cls, cx, cy in zip(class_msg.classes, class_msg.center_xs, class_msg.center_ys):
            name = cls.lower()
            if "helmet" in name or "hard" in name: detected_ppes["helmet"].append((cx, cy))
            elif "vest" in name: detected_ppes["vest"].append((cx, cy))
            elif "glove" in name: detected_ppes["gloves"].append((cx, cy))
            elif "ear" in name or "plug" in name: detected_ppes["earplug"].append((cx, cy))

        # 2. 매칭 로직
        status = {}
        
        def check(bbox, type_key, label):
            if bbox[0] == -1.0: return None # 미검출
            x1, y1, x2, y2 = map(int, bbox)
            
            # 박스 그리기 (기본: 파랑)
            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            found = False
            for (px, py) in detected_ppes[type_key]:
                if x1 <= px <= x2 and y1 <= py <= y2:
                    found = True
                    cv2.circle(cv_img, (int(px), int(py)), 5, (0,255,0), -1)
                    break
            
            color = (0, 255, 0) if found else (0, 0, 255)
            txt = "OK" if found else "NO"
            cv2.putText(cv_img, f"{label}: {txt}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            if found: cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)
            
            return found

        # 각 부위 검사
        status["helmet"] = check(pose_msg.head, "helmet", "Head")
        status["vest"] = check(pose_msg.body, "vest", "Body")
        
        l_g = check(pose_msg.hand_left, "gloves", "L_Hand")
        r_g = check(pose_msg.hand_right, "gloves", "R_Hand")
        status["gloves"] = (l_g is not False) and (r_g is not False) # 안 보이면(None) 패스

        l_e = check(pose_msg.ear_left, "earplug", "L_Ear")
        r_e = check(pose_msg.ear_right, "earplug", "R_Ear")
        status["earplug"] = (l_e is not False) and (r_e is not False)

        # 3. 최종 판단
        safe_list = []
        is_all_safe = True
        
        for k in self.check_targets:
            res = status.get(k, None)
            if res is True: safe_list.append(k)
            if res is False: is_all_safe = False # 하나라도 누락되면 Unsafe

        # 4. 결과 전송
        self.pub_status.publish(String(data=", ".join(safe_list)))
        self.pub_safe.publish(Bool(data=is_all_safe))

        # 디버그 UI
        res_color = (0, 255, 0) if is_all_safe else (0, 0, 255)
        cv2.rectangle(cv_img, (0,0), (320, 30), (0,0,0), -1)
        cv2.putText(cv_img, f"Status: {'SAFE' if is_all_safe else 'UNSAFE'}", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, res_color, 2)
        
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
