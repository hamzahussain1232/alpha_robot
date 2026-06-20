#!/usr/bin/env python3
import argparse
import math
import select
import sys
import termios
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


INCREASE_KEYS = {
    '1': 0,
    '2': 1,
    '3': 2,
    '4': 3,
    '5': 4,
    '6': 5,
}

DECREASE_KEYS = {
    'q': 0,
    'w': 1,
    'e': 2,
    'r': 3,
    't': 4,
    'y': 5,
}


def get_key(timeout: float):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class NanoArmKeyboardRos(Node):
    def __init__(
        self,
        topic: str,
        joint_state_topic: str,
        joint_names,
        min_rad,
        max_rad,
        step_deg: float,
        move_duration: float,
    ):
        super().__init__('nano_arm_keyboard_ros')
        self.topic = topic
        self.joint_state_topic = joint_state_topic
        self.joint_names = list(joint_names)
        self.min_rad = list(min_rad)
        self.max_rad = list(max_rad)
        self.step_rad = max(math.radians(1.0), math.radians(step_deg))
        self.move_duration = max(0.05, move_duration)
        self.positions = [0.0] * len(self.joint_names)
        self.has_feedback = False
        self.last_action = 'Ready'

        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)
        self.sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, 10
        )

    def _joint_state_cb(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        updated = False
        for i, name in enumerate(self.joint_names):
            idx = name_to_idx.get(name)
            if idx is None or idx >= len(msg.position):
                continue
            self.positions[i] = self._clamp(i, float(msg.position[idx]))
            updated = True
        if updated:
            self.has_feedback = True

    def _clamp(self, i: int, value: float):
        return max(self.min_rad[i], min(self.max_rad[i], value))

    def _publish_positions(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names[:]
        point = JointTrajectoryPoint()
        point.positions = self.positions[:]
        point.time_from_start = Duration(
            sec=int(self.move_duration),
            nanosec=int((self.move_duration % 1.0) * 1e9),
        )
        msg.points = [point]
        self.pub.publish(msg)

    def adjust_joint(self, idx: int, delta_rad: float):
        self.positions[idx] = self._clamp(idx, self.positions[idx] + delta_rad)
        self._publish_positions()
        self.last_action = (
            f'joint_{idx + 1} -> {math.degrees(self.positions[idx]):.1f} deg'
        )

    def home(self):
        self.positions = [0.0] * len(self.joint_names)
        self._publish_positions()
        self.last_action = 'Sent HOME (all joints 0 deg)'

    def change_step(self, scale: float):
        deg = math.degrees(self.step_rad)
        deg = max(1.0, min(30.0, round(deg * scale)))
        self.step_rad = math.radians(deg)
        self.last_action = f'Step size set to {deg:.0f} deg'

    def render(self):
        lines = []
        lines.append('\033[2J\033[H')
        lines.append('Nano Arm ROS Keyboard Teleop')
        lines.append('')
        lines.append(f'Trajectory topic: {self.topic}')
        lines.append(f'JointState topic: {self.joint_state_topic}')
        lines.append(
            f'Feedback: {"YES" if self.has_feedback else "NO (will still command arm)"}'
        )
        lines.append(f'Step: {math.degrees(self.step_rad):.0f} deg')
        lines.append(f'Move duration: {self.move_duration:.2f} s')
        lines.append(f'Last action: {self.last_action}')
        lines.append('')
        lines.append('Controls:')
        lines.append('  1/2/3/4/5/6  increase joint 1..6')
        lines.append('  q/w/e/r/t/y  decrease joint 1..6')
        lines.append('  -            smaller step')
        lines.append('  = or +       bigger step')
        lines.append('  h            home all joints')
        lines.append('  Ctrl+C       quit')
        lines.append('')
        lines.append('Joint   Position(deg)')
        for i, pos in enumerate(self.positions, start=1):
            lines.append(f'  {i}      {math.degrees(pos):>7.1f}')
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(
        description='Keyboard teleop for Nano arm via ROS trajectory topic'
    )
    parser.add_argument(
        '--topic',
        default='/arm_controller/joint_trajectory',
        help='JointTrajectory topic consumed by nano_arm_driver',
    )
    parser.add_argument(
        '--joint-state-topic',
        default='/joint_states',
        help='JointState topic for RViz feedback',
    )
    parser.add_argument(
        '--move-duration',
        type=float,
        default=0.35,
        help='Trajectory point time_from_start in seconds',
    )
    parser.add_argument(
        '--step-deg',
        type=float,
        default=8.0,
        help='Per-key press increment in degrees',
    )
    args = parser.parse_args()

    rclpy.init()
    node = NanoArmKeyboardRos(
        topic=args.topic,
        joint_state_topic=args.joint_state_topic,
        joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        min_rad=[-1.57] * 6,
        max_rad=[1.57] * 6,
        step_deg=args.step_deg,
        move_duration=args.move_duration,
    )

    node.render()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(0.10)
            if key in INCREASE_KEYS:
                node.adjust_joint(INCREASE_KEYS[key], node.step_rad)
                node.render()
            elif key in DECREASE_KEYS:
                node.adjust_joint(DECREASE_KEYS[key], -node.step_rad)
                node.render()
            elif key in ('-', '_'):
                node.change_step(0.5)
                node.render()
            elif key in ('=', '+'):
                node.change_step(2.0)
                node.render()
            elif key in ('h', 'H'):
                node.home()
                node.render()
            elif key == '\x03':
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
