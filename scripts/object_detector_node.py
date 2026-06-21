#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


COCO80_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]


class ObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detector_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("annotated_image_topic", "/perception/annotated_image")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("publish_annotated_image_compressed", True)
        self.declare_parameter("annotated_image_jpeg_quality", 70)
        self.declare_parameter("backend", "auto")  # auto|ultralytics|opencv_onnx|onnxruntime|off
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("onnx_input_width", 320)
        self.declare_parameter("onnx_input_height", 320)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("class_names_json", "")
        self.declare_parameter("confidence_threshold", 0.45)
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
        self.annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.publish_annotated_image_compressed = bool(
            self.get_parameter("publish_annotated_image_compressed").value
        )
        self.annotated_image_jpeg_quality = int(
            self.get_parameter("annotated_image_jpeg_quality").value
        )
        self.annotated_image_jpeg_quality = max(10, min(95, self.annotated_image_jpeg_quality))
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.model_path = str(self.get_parameter("model_path").value).strip()
        self.model_path = self._resolve_model_path(self.model_path)
        self.device = str(self.get_parameter("device").value).strip()
        self.onnx_input_width = int(self.get_parameter("onnx_input_width").value)
        self.onnx_input_height = int(self.get_parameter("onnx_input_height").value)
        self.nms_threshold = float(self.get_parameter("nms_threshold").value)
        self.class_names_json = str(self.get_parameter("class_names_json").value).strip()
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

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.det_pub = self.create_publisher(String, self.detections_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, sensor_qos)
        self.annotated_compressed_topic = f"{self.annotated_image_topic}/compressed"
        self.annotated_compressed_pub = self.create_publisher(
            CompressedImage, self.annotated_compressed_topic, sensor_qos
        )
        self.img_sub = self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_robot_pose, 10
        )

        self.cv_bridge = None
        self.cv2 = None
        self.model = None
        self.ort_session = None
        self.model_names: Dict[int, str] = {}
        self.frame_counter = 0
        self.last_warn_ts = 0.0
        self.robot_pose_map = None  # (x,y,yaw)

        self._init_backend()
        self.get_logger().info(
            "Object detector ready. "
            f"backend={self.backend} image_topic={self.image_topic} "
            f"detections_topic={self.detections_topic} "
            f"annotated_topic={self.annotated_image_topic} "
            f"annotated_compressed_topic={self.annotated_compressed_topic}"
        )

    @staticmethod
    def _norm(text: str) -> str:
        return " ".join(str(text).lower().strip().split())

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
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

    def _resolve_model_path(self, path_str: str) -> str:
        path = Path(path_str).expanduser()
        if not path_str:
            return ""
        if path.is_absolute():
            return str(path)

        candidates = [Path.cwd() / path]
        try:
            pkg_prefix = Path(get_package_prefix("articubot_one"))
            workspace_root = pkg_prefix.parent.parent
            candidates.extend(
                [
                    workspace_root / path,
                    workspace_root / "models" / path.name,
                    pkg_prefix / path,
                ]
            )
        except Exception:
            pass

        script_dir = Path(__file__).resolve().parent
        candidates.extend(
            [
                script_dir / path,
                script_dir.parent / path,
                script_dir.parent.parent / path,
            ]
        )

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(path)

    def _parse_class_names(self, s: str) -> Dict[int, str]:
        if not s:
            return {idx: name for idx, name in enumerate(COCO80_NAMES)}
        try:
            raw = json.loads(s)
            if isinstance(raw, list):
                return {idx: self._norm(name) for idx, name in enumerate(raw)}
            if isinstance(raw, dict):
                return {int(k): self._norm(v) for k, v in raw.items()}
        except Exception:
            pass
        return {idx: name for idx, name in enumerate(COCO80_NAMES)}

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
            import cv2  # pylint: disable=import-outside-toplevel

            self.cv_bridge = CvBridge()
            self.cv2 = cv2
        except Exception as exc:
            self._warn_throttled(f"OpenCV / cv_bridge unavailable: {exc}")
            self.backend = "off"
            return

        if self.backend in ("auto", "onnxruntime"):
            if self.model_path and self.model_path.lower().endswith(".onnx"):
                try:
                    import onnxruntime as ort  # pylint: disable=import-outside-toplevel

                    providers = ["CPUExecutionProvider"]
                    self.ort_session = ort.InferenceSession(self.model_path, providers=providers)
                    self.model_names = self._parse_class_names(self.class_names_json)
                    self.backend = "onnxruntime"
                    self.get_logger().info(f"Loaded ONNX Runtime detector model: {self.model_path}")
                    self._publish_status(f"Detector ready (ONNX Runtime): {self.model_path}")
                    return
                except Exception as exc:
                    self._warn_throttled(f"Failed loading ONNX Runtime model '{self.model_path}': {exc}")
                    if self.backend == "onnxruntime":
                        self.backend = "off"
                        return

        if self.backend in ("auto", "opencv_onnx"):
            if self.model_path and self.model_path.lower().endswith(".onnx"):
                try:
                    self.model = self.cv2.dnn.readNet(self.model_path)
                    self.model_names = self._parse_class_names(self.class_names_json)
                    self.backend = "opencv_onnx"
                    self.get_logger().info(f"Loaded ONNX detector model: {self.model_path}")
                    self._publish_status(f"Detector ready (OpenCV ONNX): {self.model_path}")
                    return
                except Exception as exc:
                    self._warn_throttled(f"Failed loading ONNX model '{self.model_path}': {exc}")
                    if self.backend == "opencv_onnx":
                        self.backend = "off"
                        return

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
                self.model_names = {
                    int(idx): self._norm(name) for idx, name in dict(getattr(self.model, "names", {})).items()
                }
                self.backend = "ultralytics"
                self.get_logger().info(f"Loaded detector model: {self.model_path}")
                self._publish_status(f"Detector ready (Ultralytics): {self.model_path}")
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
        payload = {"objects": objects, "source": "camera_detector", "backend": self.backend}
        msg = String()
        msg.data = json.dumps(payload)
        self.det_pub.publish(msg)

    def _resolve_target_label(self, label: str) -> Optional[Tuple[str, List[str]]]:
        label = self._norm(label)
        if not self.target_labels:
            return label, self.class_aliases.get(label, [])
        if label in self.target_labels:
            return label, self.class_aliases.get(label, [])
        for canonical, aliases in self.class_aliases.items():
            if label in aliases and canonical in self.target_labels:
                return canonical, aliases
        return None

    def _extract_onnx_rows(self, output) -> List[List[float]]:
        if output is None:
            return []
        arr = output
        if isinstance(arr, (list, tuple)):
            if not arr:
                return []
            arr = arr[0]
        if hasattr(arr, "shape") and len(arr.shape) == 3:
            arr = arr[0]
        if not hasattr(arr, "shape"):
            return []
        if len(arr.shape) != 2:
            return []
        if arr.shape[0] < arr.shape[1] and arr.shape[0] <= 256:
            arr = arr.transpose()
        return arr.tolist()

    def _infer_opencv_onnx(self, frame) -> List[Dict]:
        blob = self.cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(self.onnx_input_width, self.onnx_input_height),
            swapRB=True,
            crop=False,
        )
        self.model.setInput(blob)
        outputs = self.model.forward()
        rows = self._extract_onnx_rows(outputs)
        if not rows:
            return []

        frame_h, frame_w = frame.shape[:2]
        scale_x = frame_w / float(self.onnx_input_width)
        scale_y = frame_h / float(self.onnx_input_height)
        boxes = []
        scores = []
        class_ids = []

        num_classes = len(self.model_names) if self.model_names else 80
        for row in rows:
            if len(row) < 6:
                continue
            remaining = len(row) - 4
            if remaining == num_classes:
                class_scores = row[4:]
            elif remaining > num_classes:
                objectness = row[4]
                class_scores = [objectness * score for score in row[5: 5 + num_classes]]
            else:
                class_scores = row[4:]
            if not class_scores:
                continue

            best_class = max(range(len(class_scores)), key=lambda idx: class_scores[idx])
            score = float(class_scores[best_class])
            if score < self.confidence_threshold:
                continue

            cx, cy, width, height = [float(v) for v in row[:4]]
            x = int((cx - width / 2.0) * scale_x)
            y = int((cy - height / 2.0) * scale_y)
            w = int(width * scale_x)
            h = int(height * scale_y)
            boxes.append([x, y, w, h])
            scores.append(score)
            class_ids.append(best_class)

        if not boxes:
            return []

        indices = self.cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        detections = []
        if len(indices) == 0:
            return detections
        for idx in indices.flatten().tolist():
            x, y, w, h = boxes[idx]
            detections.append(
                {
                    "cls_id": int(class_ids[idx]),
                    "label_raw": self.model_names.get(int(class_ids[idx]), f"class_{class_ids[idx]}"),
                    "score": float(scores[idx]),
                    "x1": float(max(0, x)),
                    "y1": float(max(0, y)),
                    "x2": float(max(0, x + w)),
                    "y2": float(max(0, y + h)),
                }
            )
        return detections

    def _infer_onnxruntime(self, frame) -> List[Dict]:
        blob = self.cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(self.onnx_input_width, self.onnx_input_height),
            swapRB=True,
            crop=False,
        )
        input_name = self.ort_session.get_inputs()[0].name
        outputs = self.ort_session.run(None, {input_name: blob})
        rows = self._extract_onnx_rows(outputs)
        if not rows:
            return []

        frame_h, frame_w = frame.shape[:2]
        scale_x = frame_w / float(self.onnx_input_width)
        scale_y = frame_h / float(self.onnx_input_height)
        boxes = []
        scores = []
        class_ids = []

        num_classes = len(self.model_names) if self.model_names else 80
        for row in rows:
            if len(row) < 6:
                continue
            remaining = len(row) - 4
            if remaining == num_classes:
                class_scores = row[4:]
            elif remaining > num_classes:
                objectness = row[4]
                class_scores = [objectness * score for score in row[5: 5 + num_classes]]
            else:
                class_scores = row[4:]
            if not class_scores:
                continue

            best_class = max(range(len(class_scores)), key=lambda idx: class_scores[idx])
            score = float(class_scores[best_class])
            if score < self.confidence_threshold:
                continue

            cx, cy, width, height = [float(v) for v in row[:4]]
            x = int((cx - width / 2.0) * scale_x)
            y = int((cy - height / 2.0) * scale_y)
            w = int(width * scale_x)
            h = int(height * scale_y)
            boxes.append([x, y, w, h])
            scores.append(score)
            class_ids.append(best_class)

        if not boxes:
            return []

        indices = self.cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        detections = []
        if len(indices) == 0:
            return detections
        for idx in indices.flatten().tolist():
            x, y, w, h = boxes[idx]
            detections.append(
                {
                    "cls_id": int(class_ids[idx]),
                    "label_raw": self.model_names.get(int(class_ids[idx]), f"class_{class_ids[idx]}"),
                    "score": float(scores[idx]),
                    "x1": float(max(0, x)),
                    "y1": float(max(0, y)),
                    "x2": float(max(0, x + w)),
                    "y2": float(max(0, y + h)),
                }
            )
        return detections

    def _infer_ultralytics(self, frame) -> List[Dict]:
        try:
            results = self.model.predict(
                source=frame,
                conf=self.confidence_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self._warn_throttled(f"Detector inference failed: {exc}")
            return []

        if not results:
            return []

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        detections = []
        for box in boxes:
            try:
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            except Exception:
                continue
            detections.append(
                {
                    "cls_id": cls_id,
                    "label_raw": self.model_names.get(cls_id, f"class_{cls_id}"),
                    "score": score,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            )
        return detections

    def _build_objects(self, detections: List[Dict], frame_w: int, frame_h: int) -> List[Dict]:
        objects: List[Dict] = []
        for det in detections:
            if len(objects) >= self.max_objects:
                break

            resolved = self._resolve_target_label(str(det["label_raw"]))
            if resolved is None:
                continue
            label, aliases = resolved
            cx = 0.5 * (det["x1"] + det["x2"])
            pose = self._estimate_object_pose(cx, frame_w, label)
            if pose is None:
                continue
            objects.append(
                {
                    "label": label,
                    "aliases": aliases,
                    "score": float(det["score"]),
                    "frame_id": "map",
                    "pose": pose,
                    "bbox": {
                        "x1": float(det["x1"]),
                        "y1": float(det["y1"]),
                        "x2": float(det["x2"]),
                        "y2": float(det["y2"]),
                        "image_w": frame_w,
                        "image_h": frame_h,
                    },
                }
            )
        return objects

    def _publish_annotated_image(self, frame, detections: List[Dict], src_msg: Image) -> None:
        if (
            (not self.publish_annotated_image and not self.publish_annotated_image_compressed)
            or self.cv_bridge is None
            or self.cv2 is None
        ):
            return
        annotated = frame.copy()
        for det in detections[: self.max_objects]:
            x1 = int(det["x1"])
            y1 = int(det["y1"])
            x2 = int(det["x2"])
            y2 = int(det["y2"])
            label = str(det["label_raw"])
            score = float(det["score"])
            self.cv2.rectangle(annotated, (x1, y1), (x2, y2), (40, 220, 120), 2)
            text = f"{label} {score:.2f}"
            self.cv2.putText(
                annotated,
                text,
                (x1, max(18, y1 - 8)),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (40, 220, 120),
                2,
                self.cv2.LINE_AA,
            )

        if self.publish_annotated_image:
            msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            msg.header = src_msg.header
            self.annotated_pub.publish(msg)

        if self.publish_annotated_image_compressed:
            try:
                ok, encoded = self.cv2.imencode(
                    ".jpg",
                    annotated,
                    [int(self.cv2.IMWRITE_JPEG_QUALITY), int(self.annotated_image_jpeg_quality)],
                )
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header = src_msg.header
                    cmsg.format = "jpeg"
                    cmsg.data = encoded.tobytes()
                    self.annotated_compressed_pub.publish(cmsg)
            except Exception as exc:
                self._warn_throttled(f"Failed publishing annotated compressed image: {exc}")

    def _on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        if self.backend == "off" or self.cv_bridge is None:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        if self.backend in ("ultralytics", "opencv_onnx") and self.model is None:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        if self.backend == "onnxruntime" and self.ort_session is None:
            if self.publish_empty_detections:
                self._publish_detections([])
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._warn_throttled(f"Failed converting image: {exc}")
            return

        if self.backend == "ultralytics":
            detections = self._infer_ultralytics(frame)
        elif self.backend == "opencv_onnx":
            try:
                detections = self._infer_opencv_onnx(frame)
            except Exception as exc:
                self._warn_throttled(f"OpenCV ONNX inference failed: {exc}")
                detections = []
        elif self.backend == "onnxruntime":
            try:
                detections = self._infer_onnxruntime(frame)
            except Exception as exc:
                self._warn_throttled(f"ONNX Runtime inference failed: {exc}")
                detections = []
        else:
            detections = []

        frame_h, frame_w = frame.shape[:2]
        objects = self._build_objects(detections, frame_w, frame_h)
        self._publish_annotated_image(frame, detections, msg)

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
