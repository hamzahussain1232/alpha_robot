#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        self.declare_parameter('input_topic', '/cmd_vel_nav_unstamped')
        self.declare_parameter('output_topic', '/cmd_vel_nav')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('debug', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.debug = bool(self.get_parameter('debug').value)

        self.sub = self.create_subscription(Twist, self.input_topic, self._cb, 10)
        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)

        self.get_logger().info(
            f'Stamping {self.input_topic} (Twist) -> {self.output_topic} (TwistStamped)'
        )

    def _cb(self, msg: Twist):
        vals = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn('Ignoring Twist with NaN/Inf values')
            return

        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)

        if self.debug and (msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.get_logger().info(
                f'cmd v={msg.linear.x:.3f} w={msg.angular.z:.3f} -> {self.output_topic}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
