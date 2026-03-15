#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceTextPublisher(Node):
    def __init__(self):
        super().__init__("voice_text_publisher")
        self.pub = self.create_publisher(String, "/voice/text", 10)

    def publish_once(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'Published /voice/text: "{text}"')


def main():
    parser = argparse.ArgumentParser(description="Publish one voice text command")
    parser.add_argument("--text", required=True, help='Command text, e.g. --text "move forward"')
    args = parser.parse_args()

    rclpy.init()
    node = VoiceTextPublisher()
    node.publish_once(args.text)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
