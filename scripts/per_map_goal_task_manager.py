#!/usr/bin/env python3
"""
AlphaRobot per-map voice destination manager.

Reads named positions from a selected map-specific YAML file:
~/ros2_ws/maps/locations/<map_name>.yaml

Listens:
  /omni/voice/text
Publishes status:
  /home_task/status
Uses Nav2:
  /navigate_to_pose
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Dict, Optional

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PerMapGoalTaskManager(Node):
    def __init__(self) -> None:
        super().__init__("per_map_goal_task_manager")

        self.declare_parameter(
            "locations_file",
            str(Path.home() / "ros2_ws/maps/locations/home_map_final.yaml"),
        )
        self.declare_parameter("voice_topic", "/omni/voice/text")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")
        self.declare_parameter("status_topic", "/home_task/status")
        self.declare_parameter("action_name", "/navigate_to_pose")

        self.locations_file = Path(
            str(self.get_parameter("locations_file").value)
        ).expanduser()

        self.locations: Dict[str, Dict[str, float]] = {}
        self._reload_locations()

        self.estop_active = False
        self.active_goal = None
        self.active_destination: Optional[str] = None
        self.last_distance: Optional[float] = None

        voice_topic = str(self.get_parameter("voice_topic").value)
        estop_topic = str(self.get_parameter("emergency_stop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(
            String,
            voice_topic,
            self.voice_callback,
            10,
        )

        self.create_subscription(
            Bool,
            estop_topic,
            self.estop_callback,
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            action_name,
        )

        self.publish_status(
            "READY",
            message=(
                "Per-map navigation ready. "
                f"Locations file: {self.locations_file.name}"
            ),
        )

        self.get_logger().info(
            f"Per-map voice manager ready. Locations: "
            f"{', '.join(sorted(self.locations.keys())) or 'none yet'}"
        )

    @staticmethod
    def clean_text(value: str) -> str:
        value = str(value).strip().lower()
        value = re.sub(r"[^a-z0-9\s]", " ", value)
        return " ".join(value.split())

    @staticmethod
    def clean_location_name(value: str) -> str:
        value = str(value).strip().lower()
        value = re.sub(r"[^a-z0-9\s_-]", " ", value)
        value = re.sub(r"[\s-]+", "_", value)
        value = re.sub(r"_+", "_", value).strip("_")
        return value

    def load_locations(self) -> Dict[str, Dict[str, float]]:
        if not self.locations_file.is_file():
            return {}

        try:
            raw = yaml.safe_load(self.locations_file.read_text()) or {}
        except Exception as exc:
            self.get_logger().warning(
                f"Could not read locations file: {exc}"
            )
            return {}

        raw_locations = raw.get("locations", {})

        if not isinstance(raw_locations, dict):
            return {}

        locations: Dict[str, Dict[str, float]] = {}

        for raw_name, raw_pose in raw_locations.items():
            if not isinstance(raw_pose, dict):
                continue

            name = self.clean_location_name(raw_name)

            if not name:
                continue

            try:
                locations[name] = {
                    "x": float(raw_pose["x"]),
                    "y": float(raw_pose["y"]),
                    "yaw_deg": float(raw_pose.get("yaw_deg", 0.0)),
                }
            except (KeyError, TypeError, ValueError):
                continue

        return locations

    def _reload_locations(self) -> None:
        self.locations = self.load_locations()

    def publish_status(
        self,
        state: str,
        destination: str = "",
        message: str = "",
    ) -> None:
        payload = {
            "state": state,
            "destination": destination,
            "message": message,
        }

        self.status_pub.publish(
            String(data=json.dumps(payload))
        )

        self.get_logger().info(
            f"TASK [{state}] destination={destination or '-'} "
            f"message={message}"
        )

    def estop_callback(self, msg: Bool) -> None:
        previous = self.estop_active
        self.estop_active = bool(msg.data)

        if self.estop_active and not previous:
            self.publish_status(
                "EMERGENCY_STOP",
                destination=self.active_destination or "",
                message="Emergency stop received. Cancelling navigation.",
            )
            self.cancel_navigation()

        elif not self.estop_active and previous:
            self.publish_status(
                "READY",
                message="Emergency stop released. Ready for command.",
            )

    def resolve_destination(self, command: str) -> Optional[str]:
        self._reload_locations()

        if not self.locations:
            return None

        command = self.clean_text(command)

        home_phrases = (
            "home",
            "go home",
            "return home",
            "move to home",
            "go to home",
            "navigate home",
        )

        if "home" in self.locations and command in home_phrases:
            return "home"

        for destination in sorted(
            self.locations.keys(),
            key=len,
            reverse=True,
        ):
            readable = destination.replace("_", " ")

            phrases = (
                readable,
                f"go to {readable}",
                f"move to {readable}",
                f"navigate to {readable}",
                f"take me to {readable}",
            )

            if command in phrases:
                return destination

        return None

    def voice_callback(self, msg: String) -> None:
        command = self.clean_text(msg.data)

        if not command:
            return

        self.get_logger().info(
            f"Voice/text command received: {command}"
        )

        cancel_phrases = (
            "cancel",
            "cancel navigation",
            "cancel goal",
            "stop navigation",
            "abort navigation",
            "stop moving",
        )

        if command in cancel_phrases:
            self.cancel_navigation()
            return

        if self.estop_active:
            self.publish_status(
                "BLOCKED",
                message="Emergency stop is active. Release it first.",
            )
            return

        destination = self.resolve_destination(command)

        if destination is None:
            self.publish_status(
                "UNKNOWN_COMMAND",
                message=(
                    "Destination was not found for this map. "
                    "Save Home, Kitchen, Drawing Room, Bedroom, or another "
                    "location first."
                ),
            )
            return

        if self.active_goal is not None:
            self.publish_status(
                "BUSY",
                destination=self.active_destination or "",
                message=(
                    "Navigation is already active. "
                    "Say cancel navigation first."
                ),
            )
            return

        self.send_goal(destination)

    def make_pose(self, destination: str) -> PoseStamped:
        location = self.locations[destination]
        yaw = math.radians(location["yaw_deg"])

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = location["x"]
        pose.pose.position.y = location["y"]
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_goal(self, destination: str) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.publish_status(
                "NAV2_NOT_READY",
                destination=destination,
                message="NavigateToPose action server is not ready.",
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(destination)

        self.active_destination = destination
        self.last_distance = None

        self.publish_status(
            "SENDING_GOAL",
            destination=destination,
            message="Sending destination to Nav2.",
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.active_goal = None
            self.publish_status(
                "GOAL_ERROR",
                destination=self.active_destination or "",
                message=f"Could not send goal: {exc}",
            )
            self.active_destination = None
            return

        if not goal_handle.accepted:
            self.active_goal = None
            self.publish_status(
                "GOAL_REJECTED",
                destination=self.active_destination or "",
                message="Nav2 rejected destination.",
            )
            self.active_destination = None
            return

        self.active_goal = goal_handle

        self.publish_status(
            "NAVIGATING",
            destination=self.active_destination or "",
            message="Goal accepted. Robot is navigating.",
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.goal_result_callback
        )

    def feedback_callback(self, feedback_msg) -> None:
        distance = float(
            feedback_msg.feedback.distance_remaining
        )

        if (
            self.last_distance is None
            or abs(distance - self.last_distance) >= 0.20
        ):
            self.last_distance = distance

            self.publish_status(
                "NAVIGATING",
                destination=self.active_destination or "",
                message=f"Distance remaining: {distance:.2f} m",
            )

    def goal_result_callback(self, future) -> None:
        destination = self.active_destination or ""

        try:
            result = future.result()
            status = result.status
            error_message = str(
                result.result.error_msg or ""
            )
        except Exception as exc:
            status = -1
            error_message = str(exc)

        self.active_goal = None
        self.active_destination = None
        self.last_distance = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(
                "SUCCEEDED",
                destination=destination,
                message="Destination reached.",
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status(
                "CANCELED",
                destination=destination,
                message="Navigation cancelled.",
            )

        else:
            self.publish_status(
                "FAILED",
                destination=destination,
                message=(
                    error_message
                    or f"Nav2 finished with status {status}."
                ),
            )

    def cancel_navigation(self) -> None:
        if self.active_goal is None:
            self.publish_status(
                "READY",
                message="No active navigation goal.",
            )
            return

        destination = self.active_destination or ""

        self.publish_status(
            "CANCELING",
            destination=destination,
            message="Cancel request sent to Nav2.",
        )

        try:
            self.active_goal.cancel_goal_async()
        except Exception as exc:
            self.publish_status(
                "CANCEL_ERROR",
                destination=destination,
                message=f"Could not cancel goal: {exc}",
            )


def main() -> None:
    rclpy.init()

    node = None

    try:
        node = PerMapGoalTaskManager()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
