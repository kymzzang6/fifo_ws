import os
import time
import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class CheckHuman(Node):
    """
    Subscribes: /human_detected (std_msgs/Bool)
    Controls HA switch: switch.mr60bha2_mr60_gate_camera
      - True  -> turn_on immediately
      - False -> turn_off after off_hold_sec (anti-flicker)
    """

    def __init__(self):
        super().__init__("check_human")

        self.declare_parameter("person_topic", "/human_detected")
        self.declare_parameter("off_hold_sec", 2.0)

        self.person_topic = self.get_parameter("person_topic").value
        self.off_hold_sec = float(self.get_parameter("off_hold_sec").value)

        self.ha_url = os.environ.get("HA_URL", "http://localhost:8123").rstrip("/")
        self.ha_token = os.environ.get("HA_TOKEN", "")
        self.switch_entity = os.environ.get("MR60_SWITCH_ENTITY", "")

        if not self.ha_token:
            raise RuntimeError("환경변수 HA_TOKEN이 필요합니다(HA Long-Lived Token).")
        if not self.switch_entity:
            raise RuntimeError("환경변수 MR60_SWITCH_ENTITY가 필요합니다(예: switch.xxx).")

        self.session = requests.Session()
        self.headers = {
            "Authorization": f"Bearer {self.ha_token}",
            "Content-Type": "application/json",
        }

        self.last_true_time = 0.0
        self.state = None  # None/True/False

        self.create_subscription(Bool, self.person_topic, self.cb, 10)
        self.create_timer(0.5, self.tick)

        self.get_logger().info(
            f"Subscribe: {self.person_topic} -> HA switch {self.switch_entity}, off_hold={self.off_hold_sec}s"
        )

    def ha_set(self, on: bool):
        svc = "turn_on" if on else "turn_off"
        url = f"{self.ha_url}/api/services/switch/{svc}"
        payload = {"entity_id": self.switch_entity}
        r = self.session.post(url, headers=self.headers, json=payload, timeout=3)

        if r.status_code not in (200, 201):
            self.get_logger().warn(f"HA service failed {r.status_code}: {r.text[:200]}")

    def cb(self, msg: Bool):
        now = time.time()
        if msg.data:
            self.last_true_time = now
            if self.state is not True:
                self.ha_set(True)
                self.state = True
                self.get_logger().info("MR60 Gate -> ON (human detected)")

    def tick(self):
        if self.state is True:
            now = time.time()
            if (now - self.last_true_time) >= self.off_hold_sec:
                self.ha_set(False)
                self.state = False
                self.get_logger().info("MR60 Gate -> OFF (human not detected)")


def main():
    rclpy.init()
    node = CheckHuman()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
