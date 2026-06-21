#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


HELP = """
WASD Teleop
  w: forward
  s: backward
  a: turn left
  d: turn right
  x or space: stop
  q: quit
"""


def get_key(timeout=0.05):
    """Read single key from stdin with timeout."""
    try:
        fd = sys.stdin.fileno()
    except Exception:
        # If stdin is not a TTY, return empty
        return ''
    
    try:
        old = termios.tcgetattr(fd)
    except Exception:
        # If we can't get termios settings, return empty
        return ''
    
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    except Exception:
        return ''
    finally:
        try:
            
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('use_stamped', False)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.55)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('stop_timeout', 0.3)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.use_stamped = bool(self.get_parameter('use_stamped').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.stop_timeout = float(self.get_parameter('stop_timeout').value)

        msg_type = TwistStamped if self.use_stamped else Twist
        self.pub = self.create_publisher(msg_type, self.cmd_topic, 10)
        self.last_key_time = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self._has_ever_sent_motion = False
        self._sent_idle_stop = False

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish)
        self.get_logger().info(HELP.strip())
        self.get_logger().info(
            f'Publishing {"TwistStamped" if self.use_stamped else "Twist"} to {self.cmd_topic}'
        )

    def handle_key(self, key: str):
        if key == 'w':
            self.current_linear = self.linear_speed
            self.current_angular = 0.0
        elif key == 's':
            self.current_linear = -self.linear_speed
            self.current_angular = 0.0
        elif key == 'a':
            self.current_linear = 0.0
            self.current_angular = self.angular_speed
        elif key == 'd':
            self.current_linear = 0.0
            self.current_angular = -self.angular_speed
        elif key in ['x', ' ']:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self._has_ever_sent_motion = True
            self._sent_idle_stop = False
        self.last_key_time = time.time()
        if key in ['w', 'a', 's', 'd']:
            self._has_ever_sent_motion = True
            self._sent_idle_stop = False

    def _publish(self):
        # Before first motion key, publish nothing. This prevents a detached/non-interactive
        # teleop process from flooding zero commands and blocking other controllers.
        if not self._has_ever_sent_motion:
            return

        if (time.time() - self.last_key_time) > self.stop_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.current_linear
            angular = self.current_angular

        # Once timed out to zero, publish a single stop command then stay quiet.
        if linear == 0.0 and angular == 0.0 and self._sent_idle_stop:
            return

        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear)
            msg.twist.angular.z = float(angular)
        else:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
        self.pub.publish(msg)
        self._sent_idle_stop = (linear == 0.0 and angular == 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()
    node.get_logger().info("=" * 60)
    node.get_logger().info("WASD Teleop Node Started")
    node.get_logger().info(f"Publishing to: {node.cmd_topic}")
    node.get_logger().info(f"Press keys in the terminal window (w/a/s/d or q to quit)")
    node.get_logger().info("=" * 60)
    
    try:
        while rclpy.ok():
            key = get_key()
            if key:
                if key == 'q' or key == '\x03':
                    node.get_logger().info("Quit command received")
                    break
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
