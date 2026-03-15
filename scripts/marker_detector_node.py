#!/usr/bin/env python3
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class MarkerDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_detector_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("markers_topic", "/perception/markers")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_length_m", 0.12)
        self.declare_parameter("process_every_n_frames", 2)
        self.declare_parameter("max_markers", 12)
        # NOTE: Avoid [] as default because ROS2 may infer BYTE_ARRAY type.
        self.declare_parameter("target_marker_ids", [1, 2, 3])
        self.declare_parameter("require_robot_pose", True)
        self.declare_parameter("camera_hfov_deg", 69.0)
        self.declare_parameter("fallback_distance_m", 1.2)
        self.declare_parameter("publish_empty_markers", False)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_forward_offset_m", 0.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.markers_topic = str(self.get_parameter("markers_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.dictionary_name = str(self.get_parameter("dictionary").value).strip()
        self.marker_length_m = float(self.get_parameter("marker_length_m").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.max_markers = int(self.get_parameter("max_markers").value)
        self.target_marker_ids = {int(v) for v in self.get_parameter("target_marker_ids").value}
        self.require_robot_pose = bool(self.get_parameter("require_robot_pose").value)
        self.camera_hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.fallback_distance_m = float(self.get_parameter("fallback_distance_m").value)
        self.publish_empty_markers = bool(self.get_parameter("publish_empty_markers").value)
        self.camera_yaw_offset_rad = math.radians(
            float(self.get_parameter("camera_yaw_offset_deg").value)
        )
        self.camera_forward_offset_m = float(self.get_parameter("camera_forward_offset_m").value)

        self.markers_pub = self.create_publisher(String, self.markers_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_robot_pose, 10
        )

        self.cv2 = None
        self.aruco = None
        self.cv_bridge = None
        self.aruco_dict = None
        self.detector_params = None
        self.frame_counter = 0
        self.last_warn_ts = 0.0
        self.robot_pose_map = None  # (x,y,yaw)
        self.camera_matrix = None
        self.dist_coeffs = None

        self._init_backend()
        self.get_logger().info(
            f"Marker detector ready. image_topic={self.image_topic} markers_topic={self.markers_topic} dict={self.dictionary_name}"
        )

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _warn_throttled(self, text: str, period_sec: float = 5.0) -> None:
        now = time.monotonic()
        if now - self.last_warn_ts > period_sec:
            self.last_warn_ts = now
            self.get_logger().warn(text)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_markers(self, markers: List[Dict]) -> None:
        msg = String()
        msg.data = json.dumps({"markers": markers, "source": "aruco"})
        self.markers_pub.publish(msg)

    def _init_backend(self) -> None:
        try:
            import cv2  # pylint: disable=import-outside-toplevel
            from cv_bridge import CvBridge  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self._warn_throttled(f"OpenCV/cv_bridge not available: {exc}")
            return

        if not hasattr(cv2, "aruco"):
            self._warn_throttled("OpenCV aruco module not found (install opencv-contrib-python)")
            return

        self.cv2 = cv2
        self.aruco = cv2.aruco
        self.cv_bridge = CvBridge()

        dict_id = getattr(self.aruco, self.dictionary_name, None)
        if dict_id is None:
            self._warn_throttled(f'Unknown aruco dictionary "{self.dictionary_name}"')
            return

        self.aruco_dict = self.aruco.getPredefinedDictionary(dict_id)
        try:
            self.detector_params = self.aruco.DetectorParameters()
        except Exception:
            self.detector_params = self.aruco.DetectorParameters_create()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9:
            self.camera_matrix = [
                [float(msg.k[0]), float(msg.k[1]), float(msg.k[2])],
                [float(msg.k[3]), float(msg.k[4]), float(msg.k[5])],
                [float(msg.k[6]), float(msg.k[7]), float(msg.k[8])],
            ]
        if msg.d:
            self.dist_coeffs = [float(v) for v in msg.d]
        else:
            self.dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

    def _on_robot_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(float(q.z), float(q.w))
        self.robot_pose_map = (float(p.x), float(p.y), yaw)

    @staticmethod
    def _marker_center(corners) -> Tuple[float, float]:
        xs = [float(pt[0]) for pt in corners]
        ys = [float(pt[1]) for pt in corners]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    @staticmethod
    def _avg_edge_length(corners) -> float:
        if len(corners) != 4:
            return 0.0
        d = 0.0
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            d += math.hypot(float(x2 - x1), float(y2 - y1))
        return d / 4.0

    def _pose_from_bearing_distance(self, cx: float, img_w: int, distance: float) -> Optional[Dict]:
        if self.robot_pose_map is None:
            if self.require_robot_pose:
                return None
            return {"x": 0.0, "y": 0.0, "yaw": 0.0}

        rx, ry, ryaw = self.robot_pose_map
        cyaw = ryaw + self.camera_yaw_offset_rad
        bearing = (0.5 - (cx / float(max(1, img_w)))) * self.camera_hfov_rad
        yaw = cyaw + bearing
        cam_x = rx + self.camera_forward_offset_m * math.cos(cyaw)
        cam_y = ry + self.camera_forward_offset_m * math.sin(cyaw)
        mx = cam_x + distance * math.cos(yaw)
        my = cam_y + distance * math.sin(yaw)
        return {"x": float(mx), "y": float(my), "yaw": float(yaw)}

    def _estimate_distance(self, marker_corners, img_w: int) -> float:
        # If camera intrinsics are available, use simple pinhole relation from marker pixel size.
        edge_px = max(1.0, self._avg_edge_length(marker_corners))
        if self.camera_matrix is not None:
            fx = float(self.camera_matrix[0][0])
            if fx > 1e-6:
                return float((self.marker_length_m * fx) / edge_px)

        # Fallback approximate focal from HFOV.
        focal_px = (img_w / 2.0) / math.tan(self.camera_hfov_rad / 2.0)
        return float((self.marker_length_m * focal_px) / edge_px) if edge_px > 0 else self.fallback_distance_m

    def _on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        if self.cv2 is None or self.aruco is None or self.cv_bridge is None or self.aruco_dict is None:
            if self.publish_empty_markers:
                self._publish_markers([])
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._warn_throttled(f"Image conversion failed: {exc}")
            return

        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params,
        )

        if ids is None or len(ids) == 0:
            if self.publish_empty_markers:
                self._publish_markers([])
            return

        h, w = frame.shape[:2]
        markers = []
        for i, marker_id_arr in enumerate(ids):
            if len(markers) >= self.max_markers:
                break
            marker_id = int(marker_id_arr[0])
            if self.target_marker_ids and marker_id not in self.target_marker_ids:
                continue

            pts = corners[i][0]  # 4x2
            cx, _ = self._marker_center(pts)
            distance = self._estimate_distance(pts, w)
            pose = self._pose_from_bearing_distance(cx, w, distance)
            if pose is None:
                continue

            markers.append(
                {
                    "id": marker_id,
                    "label": f"marker_{marker_id}",
                    "score": 1.0,
                    "frame_id": "map",
                    "pose": pose,
                    "distance_m": float(distance),
                    "image_w": int(w),
                    "image_h": int(h),
                }
            )

        if markers or self.publish_empty_markers:
            self._publish_markers(markers)


def main() -> None:
    rclpy.init()
    node = MarkerDetectorNode()
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
