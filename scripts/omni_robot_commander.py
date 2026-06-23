#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LINEAR_SPEED = 0.12
ANGULAR_SPEED = 0.35


class OmniRobotCommander(Node):
    def __init__(self):
        super().__init__("omni_robot_commander")

        self.create_subscription(String, "/omni/voice/text", self.voice_cb, 10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/voice_vel", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )

        self.is_busy = False

        self.get_logger().info(
            "Omni commander ready: /omni/voice/text -> /voice_vel and arm commands"
        )

    def voice_cb(self, msg: String):
        if self.is_busy:
            self.get_logger().warn("Arm sequence is running. Command ignored.")
            return

        command = msg.data.lower().strip()
        self.get_logger().info(f"Voice/dashboard command: {command}")

        if any(word in command for word in ["stop", "halt", "freeze"]):
            self._send_twist(0.0, 0.0)
            self.get_logger().info("Action: stop")
            return

        if any(word in command for word in ["forward", "ahead", "straight"]):
            self._send_twist(LINEAR_SPEED, 0.0)
            self.get_logger().info("Action: forward")

        elif "back" in command:
            self._send_twist(-LINEAR_SPEED, 0.0)
            self.get_logger().info("Action: backward")

        elif "left" in command:
            self._send_twist(0.0, ANGULAR_SPEED)
            self.get_logger().info("Action: left")

        elif "right" in command:
            self._send_twist(0.0, -ANGULAR_SPEED)
            self.get_logger().info("Action: right")

        if any(phrase in command for phrase in [
            "give me cup",
            "give me my cup",
            "grab cup",
            "pick cup",
        ]):
            threading.Thread(target=self._sequence_cup, daemon=True).start()

        elif "wave" in command or "hello" in command:
            threading.Thread(target=self._sequence_wave, daemon=True).start()

        elif "raise" in command or "arm up" in command:
            self._send_arm([0.0, 1.0, 0.5, 0.0, 0.0, 0.0])

        elif "lower" in command or "arm down" in command:
            self._send_arm([0.0, -0.5, -0.5, 0.0, 0.0, 0.0])

        elif "reset" in command or "relax" in command or "home arm" in command:
            self._send_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _send_twist(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _send_arm(self, positions):
        msg = JointTrajectory()
        msg.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = 1
        msg.points.append(point)

        self.arm_pub.publish(msg)

    def _sequence_cup(self):
        self.is_busy = True
        try:
            self._send_arm([0.0, 0.5, 0.5, 0.0, 0.0, -0.5])
            time.sleep(2.0)

            self._send_arm([0.0, 0.5, 0.5, 0.0, 0.0, 1.0])
            time.sleep(2.0)

            self._send_arm([0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
            time.sleep(2.0)

            for _ in range(5):
                self._send_twist(0.0, ANGULAR_SPEED)
                time.sleep(0.10)

            self._send_twist(0.0, 0.0)
            self._send_arm([1.0, 1.0, -0.5, 0.0, 0.0, 1.0])
        finally:
            self.is_busy = False

    def _sequence_wave(self):
        self.is_busy = True
        try:
            self._send_arm([0.0, -0.3, 0.3, 0.0, 0.0, 0.0])
            time.sleep(1.0)

            self._send_arm([0.0, 0.3, -0.3, 0.0, 0.0, 0.0])
            time.sleep(1.0)

            self._send_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        finally:
            self.is_busy = False


def main():
    rclpy.init()
    node = OmniRobotCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
