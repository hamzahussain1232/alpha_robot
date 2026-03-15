#!/usr/bin/env python3
import json
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CameraSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_safety_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("lock_topic", "pause_navigation")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("min_score", 0.45)
        self.declare_parameter("allowed_labels", [])
        self.declare_parameter("min_bbox_area_ratio", 0.028)
        self.declare_parameter("min_bbox_height_ratio", 0.20)
        self.declare_parameter("trigger_bottom_ratio", 0.60)
        self.declare_parameter("center_band_half_width", 0.44)
        self.declare_parameter("lock_hold_sec", 0.9)
        self.declare_parameter("detections_timeout_sec", 1.5)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.lock_topic = str(self.get_parameter("lock_topic").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.min_score = float(self.get_parameter("min_score").value)
        self.allowed_labels = {
            self._norm(label) for label in self.get_parameter("allowed_labels").value if self._norm(label)
        }
        self.min_bbox_area_ratio = float(self.get_parameter("min_bbox_area_ratio").value)
        self.min_bbox_height_ratio = float(self.get_parameter("min_bbox_height_ratio").value)
        self.trigger_bottom_ratio = float(self.get_parameter("trigger_bottom_ratio").value)
        self.center_band_half_width = float(self.get_parameter("center_band_half_width").value)
        self.lock_hold_sec = float(self.get_parameter("lock_hold_sec").value)
        self.detections_timeout_sec = float(self.get_parameter("detections_timeout_sec").value)

        self.lock_pub = self.create_publisher(Bool, self.lock_topic, 10)
        self.det_sub = self.create_subscription(String, self.detections_topic, self._on_detections, 10)

        self.last_detections_ts = 0.0
        self.last_trigger_ts = 0.0
        self.lock_active = False
        self.last_reason = ""
        self.last_log_ts = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        labels_info = "all" if not self.allowed_labels else ",".join(sorted(self.allowed_labels))
        self.get_logger().info(
            f"Camera safety ready. detections={self.detections_topic} lock={self.lock_topic} labels={labels_info}"
        )

    @staticmethod
    def _norm(text: str) -> str:
        return " ".join(str(text).strip().lower().split())

    def _is_dangerous_object(self, obj: Dict[str, Any]) -> bool:
        label = self._norm(obj.get("label", ""))
        if self.allowed_labels and label not in self.allowed_labels:
            return False

        try:
            score = float(obj.get("score", 0.0))
        except Exception:
            return False
        if score < self.min_score:
            return False

        bbox = obj.get("bbox", {})
        if not isinstance(bbox, dict):
            return False

        try:
            x1 = float(bbox.get("x1", 0.0))
            y1 = float(bbox.get("y1", 0.0))
            x2 = float(bbox.get("x2", 0.0))
            y2 = float(bbox.get("y2", 0.0))
            img_w = float(bbox.get("image_w", 0.0))
            img_h = float(bbox.get("image_h", 0.0))
        except Exception:
            return False

        if img_w <= 1.0 or img_h <= 1.0:
            return False

        bw = max(0.0, x2 - x1)
        bh = max(0.0, y2 - y1)
        if bw <= 1.0 or bh <= 1.0:
            return False

        area_ratio = (bw * bh) / (img_w * img_h)
        height_ratio = bh / img_h
        cx_norm = ((x1 + x2) * 0.5) / img_w
        bottom_norm = max(y1, y2) / img_h

        in_forward_band = abs(cx_norm - 0.5) <= self.center_band_half_width
        near_floor_region = bottom_norm >= self.trigger_bottom_ratio
        large_enough = area_ratio >= self.min_bbox_area_ratio or height_ratio >= self.min_bbox_height_ratio

        if in_forward_band and near_floor_region and large_enough:
            self.last_reason = (
                f"label={label} score={score:.2f} area={area_ratio:.3f} "
                f"h={height_ratio:.3f} x={cx_norm:.3f} yb={bottom_norm:.3f}"
            )
            return True
        return False

    def _on_detections(self, msg: String) -> None:
        now = time.monotonic()
        self.last_detections_ts = now

        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        objects: List[Dict[str, Any]]
        if isinstance(payload, dict):
            raw = payload.get("objects", [])
            objects = raw if isinstance(raw, list) else []
        elif isinstance(payload, list):
            objects = payload
        else:
            objects = []

        for obj in objects:
            if isinstance(obj, dict) and self._is_dangerous_object(obj):
                self.last_trigger_ts = now
                break

    def _on_timer(self) -> None:
        now = time.monotonic()
        detections_fresh = (now - self.last_detections_ts) <= self.detections_timeout_sec
        lock_requested = detections_fresh and (now - self.last_trigger_ts) <= self.lock_hold_sec

        if lock_requested != self.lock_active:
            self.lock_active = lock_requested
            if lock_requested:
                self.get_logger().warn(f"Camera safety lock ON: {self.last_reason}")
            else:
                self.get_logger().info("Camera safety lock OFF")
        elif self.lock_active and (now - self.last_log_ts) > 2.0:
            self.last_log_ts = now
            self.get_logger().info("Camera safety lock active")

        msg = Bool()
        msg.data = bool(lock_requested)
        self.lock_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraSafetyNode()
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
