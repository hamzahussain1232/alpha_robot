#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class KeyboardBridge(Node):
    def __init__(self):
        super().__init__('keyboard_bridge')
        # Accept both the project-specific keyboard topic and default teleop topic.
        self.sub_key = self.create_subscription(Twist, '/key_vel_in', self.listener_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        # Publish "Stamped" commands to the Mux
        self.pub = self.create_publisher(TwistStamped, '/key_vel', 10)
        self.get_logger().info(
            'Keyboard Bridge Started: converting /key_vel_in or /cmd_vel (Twist) -> /key_vel (TwistStamped)'
        )

    def listener_callback(self, msg):
        vals = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn('Ignoring keyboard command with NaN/Inf values')
            return

        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' # No specific frame needed
        stamped_msg.twist = msg
        self.pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBridge()
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
