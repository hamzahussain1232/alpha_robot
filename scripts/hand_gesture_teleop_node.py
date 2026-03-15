#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class HandGestureTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_gesture_teleop_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_topic", "/key_vel_in")
        self.declare_parameter("debug_topic", "/gesture/debug")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("gesture_hold_sec", 0.60)
        self.declare_parameter("min_detection_confidence", 0.55)
        self.declare_parameter("min_tracking_confidence", 0.55)
        self.declare_parameter("max_num_hands", 1)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.gesture_hold_sec = max(0.1, float(self.get_parameter("gesture_hold_sec").value))
        self.min_detection_confidence = float(self.get_parameter("min_detection_confidence").value)
        self.min_tracking_confidence = float(self.get_parameter("min_tracking_confidence").value)
        self.max_num_hands = max(1, int(self.get_parameter("max_num_hands").value))

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.latest_gesture: str = "stop"
        self.last_seen_ts = time.monotonic()
        self.last_debug: Optional[str] = None
        self.mp = None
        self.hands = None
        self._init_mediapipe()
        if self.hands is None:
            self.get_logger().error(
                "Hand gesture backend is unavailable. Node will only publish stop commands."
            )
        else:
            self.get_logger().info(
                "Hand gesture teleop ready: 1 finger=forward, 2 fingers=left, otherwise=stop"
            )

    def _init_mediapipe(self) -> None:
        try:
            import mediapipe as mp  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self.get_logger().error(
                f"mediapipe import failed: {exc}. Install it in your environment first."
            )
            return

        self.mp = mp
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_num_hands,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

    @staticmethod
    def _count_fingers(landmarks, handedness_label: str) -> int:
        tip_ids = [4, 8, 12, 16, 20]
        pip_ids = [3, 6, 10, 14, 18]

        count = 0

        thumb_tip = landmarks[tip_ids[0]]
        thumb_ip = landmarks[pip_ids[0]]
        if handedness_label.lower() == "right":
            thumb_up = thumb_tip.x < thumb_ip.x
        else:
            thumb_up = thumb_tip.x > thumb_ip.x
        if thumb_up:
            count += 1

        for i in range(1, 5):
            tip = landmarks[tip_ids[i]]
            pip = landmarks[pip_ids[i]]
            if tip.y < pip.y:
                count += 1

        return count

    def _set_gesture(self, gesture: str) -> None:
        self.latest_gesture = gesture
        self.last_seen_ts = time.monotonic()
        if self.publish_debug and self.last_debug != gesture:
            self.last_debug = gesture
            msg = String()
            msg.data = f"gesture={gesture}"
            self.debug_pub.publish(msg)
            self.get_logger().info(msg.data)

    def _on_image(self, msg: Image) -> None:
        if self.hands is None or self.mp is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        rgb = frame[:, :, ::-1]
        result = self.hands.process(rgb)
        if not result.multi_hand_landmarks:
            return

        hand_landmarks = result.multi_hand_landmarks[0]
        handedness_label = "Right"
        if result.multi_handedness and len(result.multi_handedness) > 0:
            handedness_label = result.multi_handedness[0].classification[0].label

        finger_count = self._count_fingers(hand_landmarks.landmark, handedness_label)
        if finger_count == 1:
            self._set_gesture("forward")
        elif finger_count == 2:
            self._set_gesture("left")
        else:
            self._set_gesture("stop")

    def _twist_for_gesture(self, gesture: str) -> Twist:
        tw = Twist()
        if gesture == "forward":
            tw.linear.x = self.linear_speed
        elif gesture == "left":
            tw.angular.z = self.angular_speed
        return tw

    def _on_timer(self) -> None:
        age = time.monotonic() - self.last_seen_ts
        gesture = self.latest_gesture if age <= self.gesture_hold_sec else "stop"
        self.cmd_pub.publish(self._twist_for_gesture(gesture))


def main() -> None:
    rclpy.init()
    node = HandGestureTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
