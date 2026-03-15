#!/usr/bin/env python3
import json
import math
import re
import time
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("voice_topic", "/voice/text")
        self.declare_parameter("feedback_topic", "/voice/feedback")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("object_query_topic", "/task/object_query")
        self.declare_parameter("object_result_topic", "/task/object_result")
        self.declare_parameter("markers_topic", "/perception/markers")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("home_pose", [0.0, 0.0, 0.0])  # x, y, yaw
        self.declare_parameter("command_prefixes", ["bring", "fetch", "get", "find"])
        self.declare_parameter(
            "marker_command_prefixes",
            ["move to", "go to", "navigate to", "drive to", "move marker", "go marker", "marker"],
        )
        self.declare_parameter("ignore_words", ["me", "my", "the", "a", "an", "please", "robot"])
        self.declare_parameter("cancel_keywords", ["cancel task", "stop task", "abort task"])
        self.declare_parameter("marker_ttl_sec", 5.0)
        self.declare_parameter("marker_min_score", 0.25)
        self.declare_parameter("arm_joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("arm_pick", [0.0, 0.6, -1.1, 0.6, 0.0, 0.0])
        self.declare_parameter("arm_carry", [0.0, 0.1, -0.7, 0.4, 0.0, 0.0])
        self.declare_parameter("arm_place", [0.0, 0.3, -0.6, 0.2, 0.0, 0.0])
        self.declare_parameter("arm_move_duration_sec", 2.0)
        self.declare_parameter("pick_wait_sec", 2.0)
        self.declare_parameter("place_wait_sec", 1.2)
        self.declare_parameter("allow_new_tasks_while_busy", False)
        self.declare_parameter("query_timeout_sec", 6.0)

        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.object_query_topic = str(self.get_parameter("object_query_topic").value)
        self.object_result_topic = str(self.get_parameter("object_result_topic").value)
        self.markers_topic = str(self.get_parameter("markers_topic").value)
        self.navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.home_pose = [float(v) for v in self.get_parameter("home_pose").value]
        self.command_prefixes = [self._normalize(str(v)) for v in self.get_parameter("command_prefixes").value]
        self.marker_command_prefixes = [
            self._normalize(str(v)) for v in self.get_parameter("marker_command_prefixes").value
        ]
        self.ignore_words = {self._normalize(str(v)) for v in self.get_parameter("ignore_words").value}
        self.cancel_keywords = [self._normalize(str(v)) for v in self.get_parameter("cancel_keywords").value]
        self.marker_ttl_sec = float(self.get_parameter("marker_ttl_sec").value)
        self.marker_min_score = float(self.get_parameter("marker_min_score").value)
        self.arm_joint_names = [str(v) for v in self.get_parameter("arm_joint_names").value]
        self.arm_pick = [float(v) for v in self.get_parameter("arm_pick").value]
        self.arm_carry = [float(v) for v in self.get_parameter("arm_carry").value]
        self.arm_place = [float(v) for v in self.get_parameter("arm_place").value]
        self.arm_move_duration_sec = float(self.get_parameter("arm_move_duration_sec").value)
        self.pick_wait_sec = float(self.get_parameter("pick_wait_sec").value)
        self.place_wait_sec = float(self.get_parameter("place_wait_sec").value)
        self.allow_new_tasks_while_busy = bool(self.get_parameter("allow_new_tasks_while_busy").value)
        self.query_timeout_sec = float(self.get_parameter("query_timeout_sec").value)

        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.object_query_pub = self.create_publisher(String, self.object_query_topic, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        self.voice_sub = self.create_subscription(String, self.voice_topic, self._on_voice, 10)
        self.object_result_sub = self.create_subscription(
            String, self.object_result_topic, self._on_object_result, 10
        )
        self.markers_sub = self.create_subscription(String, self.markers_topic, self._on_markers, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action_name)

        self.state = "IDLE"
        self.current_object = ""
        self.query_sent_time = None
        self.object_target = None
        self.current_marker_id = None
        self.latest_markers: Dict[int, Dict] = {}
        self.goal_stage = ""
        self.goal_handle = None
        self.query_timer = self.create_timer(0.2, self._query_watchdog)

        self.number_words = {
            "zero": 0,
            "one": 1,
            "won": 1,
            "two": 2,
            "too": 2,
            "three": 3,
            "tree": 3,
            "four": 4,
            "for": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
        }

        self.get_logger().info(
            "Task manager ready. Say: bring cup / fetch medicine / move to 1"
        )

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join((text or "").lower().strip().split())

    def _publish_feedback(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _set_state(self, state: str) -> None:
        self.state = state
        self.get_logger().info(f"Task state -> {state}")

    def _reset_task(self) -> None:
        self._set_state("IDLE")
        self.current_object = ""
        self.query_sent_time = None
        self.object_target = None
        self.current_marker_id = None
        self.goal_stage = ""
        self.goal_handle = None

    def _cancel_task(self, reason: str) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._publish_feedback(f"task canceled: {reason}")
        self._reset_task()

    def _is_busy(self) -> bool:
        return self.state != "IDLE"

    def _handle_cancel(self, text: str) -> bool:
        for kw in self.cancel_keywords:
            if kw and kw in text:
                self._cancel_task("voice cancel")
                return True
        return False

    def _parse_number(self, token: str) -> Optional[int]:
        token = self._normalize(token)
        token = re.sub(r"[^a-z0-9]", "", token)
        if not token:
            return None
        if token.isdigit():
            try:
                return int(token)
            except Exception:
                return None
        return self.number_words.get(token)

    def _parse_marker_request(self, text: str) -> Optional[int]:
        for prefix in self.marker_command_prefixes:
            if not prefix:
                continue
            idx = text.find(prefix)
            if idx < 0:
                continue
            tail = text[idx + len(prefix) :].strip()
            if not tail:
                continue
            tail = re.sub(r"^marker\s+", "", tail).strip()
            tokens = [t for t in tail.split() if t and t not in self.ignore_words]
            for token in tokens:
                marker_id = self._parse_number(token)
                if marker_id is not None:
                    return marker_id
        return None

    def _parse_object_request(self, text: str) -> Optional[str]:
        tokens = text.split()
        if not tokens:
            return None

        for idx, token in enumerate(tokens):
            if token not in self.command_prefixes:
                continue
            tail = [t for t in tokens[idx + 1 :] if t and t not in self.ignore_words]
            if not tail:
                continue
            return " ".join(tail)
        return None

    def _on_voice(self, msg: String) -> None:
        text = self._normalize(msg.data)
        if not text:
            return

        if self._handle_cancel(text):
            return

        marker_id = self._parse_marker_request(text)
        if marker_id is not None:
            if self._is_busy() and not self.allow_new_tasks_while_busy:
                self._publish_feedback("task already running")
                return
            self._start_marker_navigation(marker_id)
            return

        query = self._parse_object_request(text)
        if query is None:
            return

        if self._is_busy() and not self.allow_new_tasks_while_busy:
            self._publish_feedback("task already running")
            return

        self.current_object = query
        self.query_sent_time = self.get_clock().now()
        self._set_state("WAIT_OBJECT")
        self._publish_feedback(f"looking for {query}")
        query_msg = String()
        query_msg.data = query
        self.object_query_pub.publish(query_msg)

    def _prune_markers(self) -> None:
        if not self.latest_markers:
            return
        now = time.monotonic()
        self.latest_markers = {
            mid: entry
            for mid, entry in self.latest_markers.items()
            if now - float(entry.get("seen_at_mono", 0.0)) <= self.marker_ttl_sec
        }

    def _on_markers(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Invalid markers JSON")
            return

        if isinstance(payload, dict):
            markers = payload.get("markers", [])
        elif isinstance(payload, list):
            markers = payload
        else:
            markers = []

        if not isinstance(markers, list):
            return

        now = time.monotonic()
        for marker in markers:
            if not isinstance(marker, dict):
                continue
            try:
                marker_id = int(marker.get("id"))
                score = float(marker.get("score", 1.0))
            except Exception:
                continue
            if score < self.marker_min_score:
                continue

            pose = marker.get("pose", {})
            if not isinstance(pose, dict):
                pose = {}

            self.latest_markers[marker_id] = {
                "id": marker_id,
                "score": score,
                "pose": {
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "yaw": float(pose.get("yaw", 0.0)),
                },
                "seen_at_mono": now,
            }
        self._prune_markers()

    def _start_marker_navigation(self, marker_id: int) -> None:
        self._prune_markers()
        entry = self.latest_markers.get(marker_id)
        if entry is None:
            self._publish_feedback(f"marker {marker_id} not visible")
            return

        pose = entry.get("pose", {})
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        self.current_marker_id = marker_id
        self._publish_feedback(f"going to marker {marker_id}")
        self._send_nav_goal(x, y, yaw, stage="TO_MARKER")

    def _query_watchdog(self) -> None:
        if self.state != "WAIT_OBJECT" or self.query_sent_time is None:
            return
        dt = (self.get_clock().now() - self.query_sent_time).nanoseconds / 1e9
        if dt > self.query_timeout_sec:
            self._publish_feedback(f"could not find {self.current_object}")
            self._reset_task()

    def _on_object_result(self, msg: String) -> None:
        if self.state != "WAIT_OBJECT":
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Invalid object_result JSON")
            return

        if not payload.get("ok", False):
            reason = payload.get("reason", "not_found")
            self._publish_feedback(f"{self.current_object} not found ({reason})")
            self._reset_task()
            return

        pose = payload.get("pose", {})
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        self.object_target = {"x": x, "y": y, "yaw": yaw}

        self._publish_feedback(f"going to {self.current_object}")
        self._send_nav_goal(x, y, yaw, stage="TO_OBJECT")

    def _send_nav_goal(self, x: float, y: float, yaw: float, stage: str) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._publish_feedback("navigation server not ready")
            self._reset_task()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.goal_stage = stage
        self._set_state(f"NAV_{stage}")
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal send failed: {exc}")
            self._publish_feedback("navigation goal failed")
            self._reset_task()
            return

        if not goal_handle.accepted:
            self._publish_feedback("navigation goal rejected")
            self._reset_task()
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _publish_arm_pose(self, positions: List[float]) -> None:
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().warn("Arm pose length mismatch")
            return
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(self.arm_move_duration_sec)
        point.time_from_start.nanosec = int(
            (self.arm_move_duration_sec - int(self.arm_move_duration_sec)) * 1e9
        )
        traj.points = [point]
        self.arm_pub.publish(traj)

    def _one_shot(self, delay_sec: float, cb) -> None:
        holder = {"timer": None}

        def _wrapped():
            t = holder["timer"]
            if t is not None:
                t.cancel()
            cb()

        holder["timer"] = self.create_timer(max(0.01, delay_sec), _wrapped)

    def _on_nav_result(self, future) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(f"Navigation result failed: {exc}")
            self._publish_feedback("navigation failed")
            self._reset_task()
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_feedback("navigation could not reach target")
            self._reset_task()
            return

        if self.goal_stage == "TO_MARKER":
            self._publish_feedback(f"arrived at marker {self.current_marker_id}")
            self._reset_task()
            return

        if self.goal_stage == "TO_OBJECT":
            self._publish_feedback(f"arrived at {self.current_object}, picking")
            self._publish_arm_pose(self.arm_pick)

            def _after_pick():
                self._publish_arm_pose(self.arm_carry)
                hx, hy, hyaw = self.home_pose
                self._publish_feedback("returning home")
                self._send_nav_goal(hx, hy, hyaw, stage="TO_HOME")

            self._one_shot(self.pick_wait_sec, _after_pick)
            return

        if self.goal_stage == "TO_HOME":
            self._publish_feedback("placing item")
            self._publish_arm_pose(self.arm_place)

            def _finish():
                self._publish_feedback(f"task complete: delivered {self.current_object}")
                self._reset_task()

            self._one_shot(self.place_wait_sec, _finish)
            return

        self._publish_feedback("task finished")
        self._reset_task()


def main() -> None:
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
