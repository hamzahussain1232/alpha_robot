#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import select

# Key Mappings
key_mapping = {
    '1': (0, 0.1), 'q': (0, -0.1),
    '2': (1, 0.1), 'w': (1, -0.1),
    '3': (2, 0.1), 'e': (2, -0.1),
    '4': (3, 0.1), 'r': (3, -0.1),
    '5': (4, 0.1), 't': (4, -0.1),
    '6': (5, 0.1), 'y': (5, -0.1),
}

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.positions = [0.0] * 6
        self.get_logger().info("Arm Teleop Started! Use 1-6 to increase, Q-Y to decrease. CTRL+C to quit.")

    def publish_positions(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000 
        msg.points = [point]
        self.pub.publish(msg)

def get_key():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = ArmTeleop()
    node.publish_positions()
    try:
        while rclpy.ok():
            key = get_key()
            if key in key_mapping:
                idx, change = key_mapping[key]
                node.positions[idx] += change
                node.positions[idx] = max(-3.14, min(3.14, node.positions[idx]))
                print(f"Joints: {[round(p,2) for p in node.positions]}")
                node.publish_positions()
            elif key == '\x03': 
                break
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()