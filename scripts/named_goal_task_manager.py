#!/usr/bin/env python3
"""
AlphaRobot named-location voice navigation.

Listens on: /omni/voice/text
Sends Nav2 goals through: /navigate_to_pose
Reads named map poses from: config/named_locations.yaml
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


class NamedGoalTaskManager(Node):
    def __init__(self) -> None:
        super().__init__("named_goal_task_manager")

        self.declare_parameter(
            "locations_file",
            str(
                Path.home()
                / "ros2_ws/src/articubot_one/config/named_locations.yaml"
            ),
        )
        self.declare_parameter("voice_topic", "/omni/voice/text")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")
        self.declare_parameter("status_topic", "/home_task/status")
        self.declare_parameter("action_name", "/navigate_to_pose")

        self.locations_file = Path(
            str(self.get_parameter("locations_file").value)
        ).expanduser()

        self.locations = self._load_locations()

        self._estop_active = False
        self._active_goal = None
        self._active_destination: Optional[str] = None
        self._last_feedback_distance: Optional[float] = None

        voice_topic = str(self.get_parameter("voice_topic").value)
        estop_topic = str(self.get_parameter("emergency_stop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(
            String,
            voice_topic,
            self._voice_callback,
            10,
        )

        self.create_subscription(
            Bool,
            estop_topic,
            self._estop_callback,
            10,
        )

        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        self._publish_status(
            "READY",
            message=(
                "Named navigation ready. "
                f"Available locations: {', '.join(sorted(self.locations.keys()))}"
            ),
        )

        self.get_logger().info(
            "Named Goal Task Manager ready. "
            f"Listening on {voice_topic}; using action {action_name}."
        )
        self.get_logger().info(
            f"Loaded locations: {', '.join(sorted(self.locations.keys()))}"
        )

    def _load_locations(self) -> Dict[str, Dict[str, float]]:
        if not self.locations_file.is_file():
            raise FileNotFoundError(
                f"Location file not found: {self.locations_file}"
            )

        raw = yaml.safe_load(self.locations_file.read_text()) or {}
        raw_locations = raw.get("locations", {})

        if not isinstance(raw_locations, dict):
            raise ValueError("named_locations.yaml needs a top-level 'locations:' section.")

        locations: Dict[str, Dict[str, float]] = {}

        for name, pose in raw_locations.items():
            if not isinstance(pose, dict):
                continue

            try:
                locations[str(name).strip().lower()] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw_deg": float(pose.get("yaw_deg", 0.0)),
                }
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f"Skipping invalid location '{name}': {exc}"
                )

        if not locations:
            raise ValueError("No valid named locations were loaded.")

        return locations

    @staticmethod
    def _clean_command(text: str) -> str:
        text = str(text).strip().lower()
        text = re.sub(r"[^a-z0-9\s]", " ", text)
        return " ".join(text.split())

    def _publish_status(
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
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"TASK [{state}] destination={destination or '-'} message={message}"
        )

    def _estop_callback(self, msg: Bool) -> None:
        was_active = self._estop_active
        self._estop_active = bool(msg.data)

        if self._estop_active and not was_active:
            self._publish_status(
                "EMERGENCY_STOP",
                destination=self._active_destination or "",
                message="Emergency stop received. Cancelling navigation.",
            )
            self._cancel_navigation()

        elif not self._estop_active and was_active:
            self._publish_status(
                "READY",
                message="Emergency stop released. Ready for a new command.",
            )

    def _voice_callback(self, msg: String) -> None:
        command = self._clean_command(msg.data)

        if not command:
            return

        self.get_logger().info(f"Voice/text command received: '{command}'")

        cancel_phrases = (
            "cancel navigation",
            "cancel goal",
            "stop navigation",
            "abort navigation",
            "stop moving",
        )

        if any(phrase in command for phrase in cancel_phrases):
            self._cancel_navigation()
            return

        if self._estop_active:
            self._publish_status(
                "BLOCKED",
                message="Emergency stop is active. Release it before navigation.",
            )
            return

        destination = self._extract_destination(command)

        if destination is None:
            self._publish_status(
                "UNKNOWN_COMMAND",
                message=(
                    "Command not recognized. Try: move to kitchen, "
                    "move to drawing room, or go home."
                ),
            )
            return

        if self._active_goal is not None:
            self._publish_status(
                "BUSY",
                destination=self._active_destination or "",
                message="Navigation is already active. Say 'cancel navigation' first.",
            )
            return

        self._send_named_goal(destination)

    def _extract_destination(self, command: str) -> Optional[str]:
        aliases = {
            "home": (
                "go home",
                "return home",
                "move to home",
                "go to home",
                "home",
            ),
            "kitchen": (
                "move to kitchen",
                "go to kitchen",
                "kitchen",
            ),
            "drawing_room": (
                "move to drawing room",
                "go to drawing room",
                "drawing room",
                "move to living room",
                "go to living room",
                "living room",
                "lounge",
            ),
        }

        for destination, phrases in aliases.items():
            if destination not in self.locations:
                continue

            if any(phrase in command for phrase in phrases):
                return destination

        return None

    def _make_pose(self, destination: str) -> PoseStamped:
        location = self.locations[destination]
        yaw_rad = math.radians(location["yaw_deg"])

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = location["x"]
        pose.pose.position.y = location["y"]
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        return pose

    def _send_named_goal(self, destination: str) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._publish_status(
                "NAV2_NOT_READY",
                destination=destination,
                message=(
                    "NavigateToPose action server is unavailable. "
                    "Confirm Nav2 is active before sending a command."
                ),
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(destination)

        self._active_destination = destination
        self._last_feedback_distance = None

        self._publish_status(
            "SENDING_GOAL",
            destination=destination,
            message="Sending destination to Nav2.",
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active_goal = None
            self._publish_status(
                "GOAL_ERROR",
                destination=self._active_destination or "",
                message=f"Could not send goal: {exc}",
            )
            self._active_destination = None
            return

        if not goal_handle.accepted:
            self._active_goal = None
            self._publish_status(
                "GOAL_REJECTED",
                destination=self._active_destination or "",
                message="Nav2 rejected the destination.",
            )
            self._active_destination = None
            return

        self._active_goal = goal_handle

        self._publish_status(
            "NAVIGATING",
            destination=self._active_destination or "",
            message="Goal accepted. Robot is navigating.",
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = float(feedback.distance_remaining)

        if (
            self._last_feedback_distance is None
            or abs(distance - self._last_feedback_distance) >= 0.20
        ):
            self._last_feedback_distance = distance
            self._publish_status(
                "NAVIGATING",
                destination=self._active_destination or "",
                message=f"Distance remaining: {distance:.2f} m",
            )

    def _goal_result_callback(self, future) -> None:
        destination = self._active_destination or ""

        try:
            result = future.result()
            status = result.status
            error_msg = str(result.result.error_msg or "")
        except Exception as exc:
            status = -1
            error_msg = str(exc)

        self._active_goal = None
        self._active_destination = None
        self._last_feedback_distance = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(
                "SUCCEEDED",
                destination=destination,
                message="Destination reached.",
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self._publish_status(
                "CANCELED",
                destination=destination,
                message="Navigation was cancelled.",
            )
        else:
            self._publish_status(
                "FAILED",
                destination=destination,
                message=error_msg or f"Nav2 finished with status {status}.",
            )

    def _cancel_navigation(self) -> None:
        if self._active_goal is None:
            self._publish_status(
                "READY",
                message="There is no active navigation goal.",
            )
            return

        destination = self._active_destination or ""
        self._publish_status(
            "CANCELING",
            destination=destination,
            message="Cancel request sent to Nav2.",
        )

        try:
            self._active_goal.cancel_goal_async()
        except Exception as exc:
            self._publish_status(
                "CANCEL_ERROR",
                destination=destination,
                message=f"Could not cancel goal: {exc}",
            )


def main() -> None:
    rclpy.init()

    manager = None
    try:
        manager = NamedGoalTaskManager()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[named_goal_task_manager] Fatal startup error: {exc}")
        raise
    finally:
        if manager is not None:
            manager.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
