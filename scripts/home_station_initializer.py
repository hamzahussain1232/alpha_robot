#!/usr/bin/env python3
"""
AlphaRobot Home Station initializer.

Reads the saved 'home' pose for one selected map and publishes it to AMCL
through /initialpose after Navigation Mode has started.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import yaml

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class HomeStationInitializer(Node):
    def __init__(self) -> None:
        super().__init__("home_station_initializer")

        self.declare_parameter(
            "locations_file",
            str(
                Path.home()
                / "ros2_ws/maps/locations/home_map_final.yaml"
            ),
        )
        self.declare_parameter("delay_seconds", 18.0)
        self.declare_parameter("wait_for_amcl_seconds", 25.0)
        self.declare_parameter("publish_count", 6)

        self.locations_file = Path(
            str(self.get_parameter("locations_file").value)
        ).expanduser()

        self.delay_seconds = float(
            self.get_parameter("delay_seconds").value
        )

        self.wait_for_amcl_seconds = float(
            self.get_parameter("wait_for_amcl_seconds").value
        )

        self.publish_count = max(
            1,
            int(self.get_parameter("publish_count").value),
        )

        self.home_pose = self.load_home_pose()

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

    def load_home_pose(self) -> dict:
        if not self.locations_file.is_file():
            raise RuntimeError(
                f"Saved locations file was not found: {self.locations_file}"
            )

        raw = yaml.safe_load(
            self.locations_file.read_text()
        ) or {}

        locations = raw.get("locations", {})

        if not isinstance(locations, dict):
            raise RuntimeError(
                "Location file does not contain a valid locations section."
            )

        home = locations.get("home")

        if not isinstance(home, dict):
            raise RuntimeError(
                "No Home Station is saved for this selected map."
            )

        try:
            return {
                "x": float(home["x"]),
                "y": float(home["y"]),
                "yaw_deg": float(home.get("yaw_deg", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "Saved Home Station coordinates are invalid."
            ) from exc

    def make_message(self) -> PoseWithCovarianceStamped:
        yaw = math.radians(self.home_pose["yaw_deg"])

        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"

        message.pose.pose.position.x = self.home_pose["x"]
        message.pose.pose.position.y = self.home_pose["y"]
        message.pose.pose.position.z = 0.0

        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Position uncertainty: 0.20 m²
        # Heading uncertainty: roughly 15 degrees squared.
        message.pose.covariance[0] = 0.20
        message.pose.covariance[7] = 0.20
        message.pose.covariance[35] = 0.0685

        return message

    def wait_seconds(self, seconds: float) -> None:
        end_time = time.monotonic() + max(0.0, seconds)

        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_home_pose(self) -> None:
        self.get_logger().info(
            "Waiting for Nav2 and AMCL before Home initialization..."
        )

        self.wait_seconds(self.delay_seconds)

        deadline = time.monotonic() + self.wait_for_amcl_seconds

        while (
            rclpy.ok()
            and self.publisher.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.2)

        subscribers = self.publisher.get_subscription_count()

        if subscribers < 1:
            raise RuntimeError(
                "No /initialpose subscriber appeared. "
                "Navigation/AMCL may not be running yet."
            )

        message = self.make_message()

        self.get_logger().info(
            "Sending saved Home Station to AMCL: "
            f"x={self.home_pose['x']:.3f}, "
            f"y={self.home_pose['y']:.3f}, "
            f"yaw={self.home_pose['yaw_deg']:.2f} degrees"
        )

        for number in range(self.publish_count):
            message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)

            self.get_logger().info(
                f"Home pose sent to AMCL "
                f"({number + 1}/{self.publish_count})."
            )

            self.wait_seconds(0.35)


def main() -> int:
    rclpy.init()
    node = None

    try:
        node = HomeStationInitializer()
        node.send_home_pose()
        return 0

    except Exception as exc:
        print(
            f"HOME_INITIALIZATION_ERROR: {exc}",
            file=sys.stderr,
        )
        return 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
