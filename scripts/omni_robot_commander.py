#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import re
import time
import threading

LINEAR_SPEED = 0.12
ANGULAR_SPEED = 0.35

class OmniRobotCommander(Node):
    def __init__(self):
        super().__init__('omni_robot_commander')
        self.create_subscription(String, '/omni/voice/text', self.voice_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.get_logger().info('Omni Voice Commander is listening to /omni/voice/text and publishing to /cmd_vel!')
        self.is_busy = False

    def voice_cb(self, msg):
        if self.is_busy:
            return

        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Analyzing Command: {cmd}")

        if "stop" in cmd or "halt" in cmd or "freeze" in cmd:
            self._send_twist(0.0, 0.0)
            self._send_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.get_logger().info("Action: Stop everything.")
            return

        if any(word in cmd for word in ["forward", "ahead", "straight"]):
            self._send_twist(LINEAR_SPEED, 0.0)
            self.get_logger().info("Action: Drive Forward")
        elif "back" in cmd:
            self._send_twist(-LINEAR_SPEED, 0.0)
            self.get_logger().info("Action: Drive Backward")
        elif "left" in cmd:
            self._send_twist(0.0, ANGULAR_SPEED)
            self.get_logger().info("Action: Turn Left")
        elif "right" in cmd:
            self._send_twist(0.0, -ANGULAR_SPEED)
            self.get_logger().info("Action: Turn Right")
            
        if "give me my cup" in cmd or "give me cup" in cmd or "grab cup" in cmd or "pick cup" in cmd:
            self.get_logger().info("Action: Picking up Cup!")
            threading.Thread(target=self._sequence_cup).start()
            
        elif "wave" in cmd or "hello" in cmd:
            self.get_logger().info("Action: Arm Waving")
            threading.Thread(target=self._sequence_wave).start()
            
        elif "raise" in cmd or "up" in cmd or "high" in cmd:
            self.get_logger().info("Action: Raise Arm")
            self._send_arm([0.0, 1.0, 0.5, 0.0, 0.0, 0.0])
        elif "down" in cmd or "low" in cmd:
            self.get_logger().info("Action: Lower Arm")
            self._send_arm([0.0, -0.5, -0.5, 0.0, 0.0, 0.0])
        elif "reset" in cmd or "relax" in cmd:
            self.get_logger().info("Action: Reset Arm")
            self._send_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _sequence_cup(self):
        self.is_busy = True
        self._send_arm([0.0, 0.5, 0.5, 0.0, 0.0, -0.5])
        time.sleep(2)
        self._send_arm([0.0, 0.5, 0.5, 0.0, 0.0, 1.0]) 
        time.sleep(2)
        self._send_arm([0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        time.sleep(2)
        self._send_twist(0.0, ANGULAR_SPEED)
        time.sleep(0.5)
        self._send_twist(0.0, 0.0)
        self._send_arm([1.0, 1.0, -0.5, 0.0, 0.0, 1.0])
        self.is_busy = False

    def _sequence_wave(self):
        self.is_busy = True
        self._send_arm([0.0, -0.3, 0.3, 0.0, 0.0, 0.0])
        time.sleep(1)
        self._send_arm([0.0, 0.3, -0.3, 0.0, 0.0, 0.0])
        time.sleep(1)
        self._send_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.is_busy = False

    def _send_twist(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_vel_pub.publish(t)

    def _send_arm(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.arm_pub.publish(msg)

def main():
    rclpy.init()
    node = OmniRobotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
