#!/usr/bin/env python3
import json
import math
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detector_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("backend", "auto")  # auto|ultralytics|off
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_threshold", 0.45)
        # NOTE: Avoid [] as default because ROS2 may infer BYTE_ARRAY type.
        self.declare_parameter("target_labels", ["cup", "medicine box", "bottle"])
        self.declare_parameter("max_objects", 8)
        self.declare_parameter("process_every_n_frames", 3)
        self.declare_parameter("publish_empty_detections", False)
        self.declare_parameter("enable_pose_estimation", True)
        self.declare_parameter("require_robot_pose_for_detections", True)
        self.declare_parameter("camera_hfov_deg", 69.0)
        self.declare_parameter("default_distance_m", 1.0)
        self.declare_parameter("label_distance_json", '{"cup":1.0,"medicine box":1.1,"bottle":1.1}')
        self.declare_parameter(
            "class_aliases_json",
            '{"cup":["mug","coffee cup"],"medicine box":["medicine","pill box"],"bottle":["water bottle"]}',
        )

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.model_path = str(self.get_parameter("model_path").value).strip()
        self.device = str(self.get_parameter("device").value).strip()
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.target_labels = {self._norm(v) for v in self.get_parameter("target_labels").value}
        self.max_objects = int(self.get_parameter("max_objects").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.publish_empty_detections = bool(self.get_parameter("publish_empty_detections").value)
        self.enable_pose_estimation = bool(self.get_parameter("enable_pose_estimation").value)
        self.require_robot_pose_for_detections = bool(
            self.get_parameter("require_robot_pose_for_detections").value
        )
        self.camera_hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.default_distance_m = float(self.get_parameter("default_distance_m").value)
        self.label_distance_map = self._parse_json_dict(
            str(self.get_parameter("label_distance_json").value)
        )
        self.class_aliases = self._parse_aliases(
            str(self.get_parameter("class_aliases_json").value)
        )

        self.det_pub = self.create_publisher(String, self.detections_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.img_sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_robot_pose, 10
        )

        self.cv_bridge = None
        self.model = None
        self.model_names = {}
        self.frame_counter = 0
        self.last_warn_ts = 0.0
        self.robot_pose_map = None  # (x,y,yaw)

        self._init_backend()
        self.get_logger().info(
            f"Object detector ready. backend={self.backend} image_topic={self.image_topic} detections_topic={self.detections_topic}"
        )

    @staticmethod
    def _norm(text: str) -> str:
        return " ".join(str(text).lower().strip().split())

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
        # For planar robot, x=y=0 quaternion assumption.
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _parse_json_dict(self, s: str) -> Dict[str, float]:
        try:
            raw = json.loads(s)
            if not isinstance(raw, dict):
                return {}
            out = {}
            for k, v in raw.items():
                out[self._norm(k)] = float(v)
            return out
        except Exception:
            return {}

    def _parse_aliases(self, s: str) -> Dict[str, List[str]]:
        try:
            raw = json.loads(s)
            if not isinstance(raw, dict):
                return {}
            out = {}
            for k, vals in raw.items():
                key = self._norm(k)
                aliases = []
                if isinstance(vals, list):
                    aliases = [self._norm(v) for v in vals if self._norm(v)]
                out[key] = aliases
            return out
        except Exception:
            return {}

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _warn_throttled(self, text: str, period_sec: float = 5.0) -> None:
        now = time.monotonic()
        if now - self.last_warn_ts > period_sec:
            self.last_warn_ts = now
            self.get_logger().warn(text)

    def _init_backend(self) -> None:
        if self.backend == "off":
            return

        try:
            from cv_bridge import CvBridge  # pylint: disable=import-outside-toplevel

            self.cv_bridge = CvBridge()
        except Exception as exc:
            self._warn_throttled(f"cv_bridge unavailable: {exc}")
            self.backend = "off"
            return

        # Try ultralytics backend.
        if self.backend in ("auto", "ultralytics"):
            try:
                from ultralytics import YOLO  # pylint: disable=import-outside-toplevel
            except Exception as exc:
                self._warn_throttled(f"ultralytics not available: {exc}")
                if self.backend == "ultralytics":
                    self.backend = "off"
                return

            if not self.model_path:
                self._warn_throttled("model_path is empty; detector disabled until model_path is set")
                self.backend = "off"
                return

            try:
                self.model = YOLO(self.model_path)
                # result.names is map class_id -> name
                self.model_names = dict(getattr(self.model, "names", {}))
                self.backend = "ultralytics"
                self.get_logger().info(f"Loaded detector model: {self.model_path}")
            except Exception as exc:
                self._warn_throttled(f"Failed loading model '{self.model_path}': {exc}")
                self.backend = "off"

    def _on_robot_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(float(q.z), float(q.w))
        self.robot_pose_map = (float(p.x), float(p.y), yaw)

    def _estimate_object_pose(self, cx_px: float, img_w: int, label: str) -> Optional[Dict[str, float]]:
        if not self.enable_pose_estimation:
            return None
        if self.robot_pose_map is None:
            if self.require_robot_pose_for_detections:
                return None
            return {"x": 0.0, "y": 0.0, "yaw": 0.0}

        rx, ry, ryaw = self.robot_pose_map
        bearing = (0.5 - (cx_px / float(max(1, img_w)))) * self.camera_hfov_rad
        distance = self.label_distance_map.get(self._norm(label), self.default_distance_m)
        oyaw = ryaw + bearing
        ox = rx + distance * math.cos(oyaw)
        oy = ry + distance * math.sin(oyaw)
        return {"x": float(ox), "y": float(oy), "yaw": float(oyaw)}

    def _publish_detections(self, objects: List[Dict]) -> None:
        payload = {"objects": objects, "source": "camera_detector"}
        msg = String()
        msg.data = json.dumps(payload)
        self.det_pub.publish(msg)

    def _on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        if self.backend != "ultralytics" or self.model is None or self.cv_bridge is None:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._warn_throttled(f"Failed converting image: {exc}")
            return

        try:
            results = self.model.predict(
                source=frame,
                conf=self.confidence_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self._warn_throttled(f"Detector inference failed: {exc}")
            return

        if not results:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        h, w = frame.shape[:2]
        objects: List[Dict] = []
        for box in boxes:
            if len(objects) >= self.max_objects:
                break
            try:
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            except Exception:
                continue

            label_raw = str(self.model_names.get(cls_id, f"class_{cls_id}"))
            label = self._norm(label_raw)
            if self.target_labels and label not in self.target_labels:
                continue

            cx = 0.5 * (x1 + x2)
            pose = self._estimate_object_pose(cx, w, label)
            if pose is None:
                # Need robot pose for meaningful map target.
                continue

            aliases = self.class_aliases.get(label, [])
            objects.append(
                {
                    "label": label,
                    "aliases": aliases,
                    "score": score,
                    "frame_id": "map",
                    "pose": pose,
                    "bbox": {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "image_w": w, "image_h": h},
                }
            )

        if objects or self.publish_empty_detections:
            self._publish_detections(objects)


def main() -> None:
    rclpy.init()
    node = ObjectDetectorNode()
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
