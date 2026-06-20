#!/usr/bin/env python3
import json
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class FetchBottleNode(Node):
    def __init__(self) -> None:
        super().__init__("fetch_bottle_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("target_topic", "/task/target_object")

        self.declare_parameter("target_label", "bottle")
        self.declare_parameter("min_valid_score", 0.35)
        self.declare_parameter("stable_detection_frames", 4)
        self.declare_parameter("lost_target_timeout_sec", 0.8)

        self.declare_parameter("loop_hz", 10.0)
        self.declare_parameter("search_spin_speed", 0.35)
        self.declare_parameter("align_kp", 0.9)
        self.declare_parameter("approach_align_kp", 0.65)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("align_deadband_norm", 0.08)

        self.declare_parameter("approach_linear_speed", 0.08)
        self.declare_parameter("approach_stop_bbox_height_ratio", 0.35)
        self.declare_parameter("approach_stop_bbox_width_ratio", 0.30)
        self.declare_parameter("search_timeout_sec", 0.0)
        self.declare_parameter("approach_timeout_sec", 30.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)

        self.target_label = self._norm(str(self.get_parameter("target_label").value))
        self.min_valid_score = float(self.get_parameter("min_valid_score").value)
        self.stable_detection_frames = int(self.get_parameter("stable_detection_frames").value)
        self.lost_target_timeout_sec = float(self.get_parameter("lost_target_timeout_sec").value)

        self.loop_hz = float(self.get_parameter("loop_hz").value)
        self.search_spin_speed = float(self.get_parameter("search_spin_speed").value)
        self.align_kp = float(self.get_parameter("align_kp").value)
        self.approach_align_kp = float(self.get_parameter("approach_align_kp").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.align_deadband_norm = float(self.get_parameter("align_deadband_norm").value)

        self.approach_linear_speed = float(self.get_parameter("approach_linear_speed").value)
        self.approach_stop_bbox_height_ratio = float(
            self.get_parameter("approach_stop_bbox_height_ratio").value
        )
        self.approach_stop_bbox_width_ratio = float(
            self.get_parameter("approach_stop_bbox_width_ratio").value
        )
        self.search_timeout_sec = float(self.get_parameter("search_timeout_sec").value)
        self.approach_timeout_sec = float(self.get_parameter("approach_timeout_sec").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.det_sub = self.create_subscription(String, self.detections_topic, self._on_detections, 10)
        self.target_sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        self.state = "SEARCH"
        self.state_since = time.monotonic()
        self.last_target_time = 0.0
        self.last_seen: Optional[Dict[str, Any]] = None
        self.stable_hits = 0

        self.timer = self.create_timer(1.0 / max(1.0, self.loop_hz), self._tick)

        self._publish_status(f"fetch_phase1_ready target={self.target_label} state={self.state}")
        self.get_logger().info(
            f"Fetch Bottle Phase-1 ready. target={self.target_label} detections_topic={self.detections_topic} cmd_topic={self.cmd_topic}"
        )

    @staticmethod
    def _norm(text: str) -> str:
        return " ".join((text or "").lower().strip().split())

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _set_state(self, next_state: str, note: str = "") -> None:
        if next_state == self.state:
            return
        self.state = next_state
        self.state_since = time.monotonic()
        suffix = f" note={note}" if note else ""
        self._publish_status(f"fetch_phase1_state={self.state}{suffix}")
        self.get_logger().info(f"State -> {self.state}{suffix}")

    def _stop_robot(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _on_target(self, msg: String) -> None:
        new_target = self._norm(msg.data)
        if not new_target:
            return
        if new_target == self.target_label:
            return

        self.target_label = new_target
        self.last_seen = None
        self.stable_hits = 0
        self.last_target_time = 0.0
        self._set_state("SEARCH", note=f"new_target={self.target_label}")

    def _label_match(self, obj: Dict[str, Any]) -> bool:
        label = self._norm(str(obj.get("label", "")))
        if label == self.target_label:
            return True
        aliases = obj.get("aliases", [])
        if isinstance(aliases, list):
            for alias in aliases:
                if self._norm(str(alias)) == self.target_label:
                    return True
        return False

    def _extract_best_target(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        objs = payload.get("objects", [])
        if not isinstance(objs, list):
            return None

        best = None
        best_score = -1.0
        for obj in objs:
            if not isinstance(obj, dict):
                continue
            if not self._label_match(obj):
                continue

            score = float(obj.get("score", 0.0))
            if score < self.min_valid_score:
                continue

            bbox = obj.get("bbox", {})
            if not isinstance(bbox, dict):
                continue
            try:
                x1 = float(bbox.get("x1", 0.0))
                y1 = float(bbox.get("y1", 0.0))
                x2 = float(bbox.get("x2", 0.0))
                y2 = float(bbox.get("y2", 0.0))
                image_w = max(1.0, float(bbox.get("image_w", 1.0)))
                image_h = max(1.0, float(bbox.get("image_h", 1.0)))
            except Exception:
                continue

            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)
            cx = 0.5 * (x1 + x2)

            error_x_norm = (cx - (0.5 * image_w)) / (0.5 * image_w)
            width_ratio = bw / image_w
            height_ratio = bh / image_h

            candidate = {
                "score": score,
                "error_x_norm": float(error_x_norm),
                "width_ratio": float(width_ratio),
                "height_ratio": float(height_ratio),
            }

            if score > best_score:
                best_score = score
                best = candidate

        return best

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        best = self._extract_best_target(payload)
        if best is None:
            return

        self.last_seen = best
        self.last_target_time = time.monotonic()
        self.stable_hits += 1

    def _target_recent(self) -> bool:
        if self.last_target_time <= 0.0:
            return False
        return (time.monotonic() - self.last_target_time) <= self.lost_target_timeout_sec

    def _tick(self) -> None:
        now = time.monotonic()

        if self.state == "DONE":
            self._stop_robot()
            return

        if self.state in ("ALIGN", "APPROACH") and not self._target_recent():
            self.stable_hits = 0
            self._set_state("SEARCH", note="target_lost")

        if self.state == "SEARCH":
            if self.search_timeout_sec > 0.0 and (now - self.state_since) > self.search_timeout_sec:
                self._stop_robot()
                self._set_state("DONE", note="search_timeout")
                return

            if self._target_recent() and self.stable_hits >= self.stable_detection_frames and self.last_seen is not None:
                self._set_state("ALIGN", note="target_locked")
                self._stop_robot()
                return

            self._publish_cmd(0.0, self.search_spin_speed)
            return

        if self.last_seen is None:
            self._set_state("SEARCH", note="missing_last_seen")
            return

        err = float(self.last_seen["error_x_norm"])

        if self.state == "ALIGN":
            if abs(err) <= self.align_deadband_norm:
                self._stop_robot()
                self._set_state("APPROACH", note="centered")
                return

            ang = self._clamp(-self.align_kp * err, -self.max_angular_speed, self.max_angular_speed)
            self._publish_cmd(0.0, ang)
            return

        if self.state == "APPROACH":
            if self.approach_timeout_sec > 0.0 and (now - self.state_since) > self.approach_timeout_sec:
                self._stop_robot()
                self._set_state("DONE", note="approach_timeout")
                return

            h_ratio = float(self.last_seen["height_ratio"])
            w_ratio = float(self.last_seen["width_ratio"])

            close_enough = (
                h_ratio >= self.approach_stop_bbox_height_ratio
                or w_ratio >= self.approach_stop_bbox_width_ratio
            )
            if close_enough:
                self._stop_robot()
                self._set_state("DONE", note="grasp_distance_reached")
                self._publish_status("fetch_phase1_done: ready_for_pick")
                return

            ang = self._clamp(-self.approach_align_kp * err, -self.max_angular_speed, self.max_angular_speed)
            self._publish_cmd(self.approach_linear_speed, ang)
            return


def main() -> None:
    rclpy.init()
    node = FetchBottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_robot()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
