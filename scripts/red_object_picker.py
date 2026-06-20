#!/usr/bin/env python3
import json
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RedObjectPicker(Node):
    def __init__(self) -> None:
        super().__init__("red_object_picker")

        self.bridge = CvBridge()
        self.joint_names = [f"joint_{idx}" for idx in range(1, 7)]

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("debug_image_topic", "/perception/red_picker/debug_image")
        self.declare_parameter(
            "debug_compressed_topic", "/perception/red_picker/debug_image/compressed"
        )
        self.declare_parameter("target_topic", "/perception/red_picker/target")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("publish_raw_debug_image", False)
        self.declare_parameter("publish_compressed_debug_image", True)
        self.declare_parameter("compressed_jpeg_quality", 70)
        self.declare_parameter("process_every_n_frames", 2)
        self.declare_parameter("move_to_scan_pose_on_startup", True)
        self.declare_parameter("move_to_scan_pose_on_lost", False)
        self.declare_parameter("lost_target_timeout_sec", 1.5)
        self.declare_parameter("command_period_sec", 0.7)
        self.declare_parameter("move_duration_sec", 0.8)
        self.declare_parameter("pause_after_move_sec", 0.9)
        self.declare_parameter("pause_after_grip_sec", 1.2)
        self.declare_parameter("lower_red1", [0, 120, 70])
        self.declare_parameter("upper_red1", [10, 255, 255])
        self.declare_parameter("lower_red2", [170, 120, 70])
        self.declare_parameter("upper_red2", [180, 255, 255])
        self.declare_parameter("blur_kernel_size", 5)
        self.declare_parameter("morph_open_iterations", 1)
        self.declare_parameter("morph_close_iterations", 2)
        self.declare_parameter("min_contour_area_px", 500.0)
        self.declare_parameter("center_tolerance_x", 0.10)
        self.declare_parameter("center_tolerance_y", 0.12)
        self.declare_parameter("approach_fill_ratio", 0.06)
        self.declare_parameter("pick_fill_ratio_threshold", 0.11)
        self.declare_parameter("auto_pick", True)
        self.declare_parameter("scan_pose", [0.0, 0.45, -1.0, 0.45, 0.0, 0.0])
        self.declare_parameter("carry_pose", [0.0, 0.15, -0.7, 0.35, 0.0, 1.0])
        self.declare_parameter("gripper_open_rad", 0.0)
        self.declare_parameter("gripper_close_rad", 1.0)
        self.declare_parameter("x_joint_index", 0)
        self.declare_parameter("y_joint_index", 1)
        self.declare_parameter("reach_joint_index", 2)
        self.declare_parameter("lift_joint_index", 1)
        self.declare_parameter("gripper_joint_index", 5)
        self.declare_parameter("x_joint_gain", -0.35)
        self.declare_parameter("y_joint_gain", 0.25)
        self.declare_parameter("reach_step_rad", -0.08)
        self.declare_parameter("pregrasp_reach_delta_rad", -0.12)
        self.declare_parameter("lift_delta_rad", -0.25)
        self.declare_parameter("command_joint_min_rad", [-1.57] * 6)
        self.declare_parameter("command_joint_max_rad", [1.57] * 6)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_compressed_topic = str(self.get_parameter("debug_compressed_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.publish_raw_debug_image = bool(
            self.get_parameter("publish_raw_debug_image").value
        )
        self.publish_compressed_debug_image = bool(
            self.get_parameter("publish_compressed_debug_image").value
        )
        self.compressed_jpeg_quality = int(
            self.get_parameter("compressed_jpeg_quality").value
        )
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.move_to_scan_pose_on_startup = bool(
            self.get_parameter("move_to_scan_pose_on_startup").value
        )
        self.move_to_scan_pose_on_lost = bool(
            self.get_parameter("move_to_scan_pose_on_lost").value
        )
        self.lost_target_timeout_sec = float(self.get_parameter("lost_target_timeout_sec").value)
        self.command_period_sec = float(self.get_parameter("command_period_sec").value)
        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        self.pause_after_move_sec = float(self.get_parameter("pause_after_move_sec").value)
        self.pause_after_grip_sec = float(self.get_parameter("pause_after_grip_sec").value)
        self.lower_red1 = np.array(self.get_parameter("lower_red1").value, dtype=np.uint8)
        self.upper_red1 = np.array(self.get_parameter("upper_red1").value, dtype=np.uint8)
        self.lower_red2 = np.array(self.get_parameter("lower_red2").value, dtype=np.uint8)
        self.upper_red2 = np.array(self.get_parameter("upper_red2").value, dtype=np.uint8)
        self.blur_kernel_size = int(self.get_parameter("blur_kernel_size").value)
        self.morph_open_iterations = int(self.get_parameter("morph_open_iterations").value)
        self.morph_close_iterations = int(self.get_parameter("morph_close_iterations").value)
        self.min_contour_area_px = float(self.get_parameter("min_contour_area_px").value)
        self.center_tolerance_x = float(self.get_parameter("center_tolerance_x").value)
        self.center_tolerance_y = float(self.get_parameter("center_tolerance_y").value)
        self.approach_fill_ratio = float(self.get_parameter("approach_fill_ratio").value)
        self.pick_fill_ratio_threshold = float(
            self.get_parameter("pick_fill_ratio_threshold").value
        )
        self.auto_pick = bool(self.get_parameter("auto_pick").value)
        self.scan_pose = [float(v) for v in self.get_parameter("scan_pose").value]
        self.carry_pose = [float(v) for v in self.get_parameter("carry_pose").value]
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.x_joint_index = int(self.get_parameter("x_joint_index").value)
        self.y_joint_index = int(self.get_parameter("y_joint_index").value)
        self.reach_joint_index = int(self.get_parameter("reach_joint_index").value)
        self.lift_joint_index = int(self.get_parameter("lift_joint_index").value)
        self.gripper_joint_index = int(self.get_parameter("gripper_joint_index").value)
        self.x_joint_gain = float(self.get_parameter("x_joint_gain").value)
        self.y_joint_gain = float(self.get_parameter("y_joint_gain").value)
        self.reach_step_rad = float(self.get_parameter("reach_step_rad").value)
        self.pregrasp_reach_delta_rad = float(
            self.get_parameter("pregrasp_reach_delta_rad").value
        )
        self.lift_delta_rad = float(self.get_parameter("lift_delta_rad").value)
        self.command_joint_min_rad = [
            float(v) for v in self.get_parameter("command_joint_min_rad").value
        ]
        self.command_joint_max_rad = [
            float(v) for v in self.get_parameter("command_joint_max_rad").value
        ]

        self._validate_lengths()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile
        )
        self.joint_sub = self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10
        )
        self.arm_pub = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.debug_compressed_pub = self.create_publisher(
            CompressedImage, self.debug_compressed_topic, 10
        )
        self.target_pub = self.create_publisher(String, self.target_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.frame_counter = 0
        self.current_pose = self.scan_pose[:]
        self.has_joint_state = False
        self.started_scan_pose = False
        self.sequence_active = False
        self.last_target_seen_time = 0.0
        self.last_command_time = 0.0
        self.state = "starting"

        self.startup_timer = self.create_timer(1.0, self._startup_scan_pose_once)

        self.get_logger().info(
            "Red object picker ready. This uses image coordinates from the Pi camera and "
            "moves the arm toward the red blob before running a grasp sequence."
        )

    def _validate_lengths(self) -> None:
        expected = len(self.joint_names)
        for name, values in (
            ("scan_pose", self.scan_pose),
            ("carry_pose", self.carry_pose),
            ("command_joint_min_rad", self.command_joint_min_rad),
            ("command_joint_max_rad", self.command_joint_max_rad),
        ):
            if len(values) != expected:
                raise RuntimeError(f"{name} must contain {expected} values")

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        updated = self.current_pose[:]
        seen_any = False
        for local_idx, joint_name in enumerate(self.joint_names):
            incoming_idx = name_to_index.get(joint_name)
            if incoming_idx is None or incoming_idx >= len(msg.position):
                continue
            updated[local_idx] = float(msg.position[incoming_idx])
            seen_any = True

        if seen_any:
            self.current_pose = updated
            self.has_joint_state = True

    def _startup_scan_pose_once(self) -> None:
        if self.started_scan_pose or not self.move_to_scan_pose_on_startup:
            return
        self.started_scan_pose = True
        self._publish_status("moving arm to scan pose")
        self._send_arm_pose(self.scan_pose, self.move_duration_sec)
        self.state = "scan_pose"

    def image_callback(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self._warn_once(f"Failed to convert image: {exc}")
            return

        detection = self._detect_largest_red_blob(frame)
        now = time.monotonic()

        if detection is not None:
            self.last_target_seen_time = now

        if detection is None and self.move_to_scan_pose_on_lost:
            if (
                not self.sequence_active
                and now - self.last_target_seen_time > self.lost_target_timeout_sec
                and now - self.last_command_time > (self.command_period_sec * 2.0)
            ):
                self.state = "searching"
                self._publish_status("red target lost; returning to scan pose")
                self._send_arm_pose(self.scan_pose, self.move_duration_sec)

        if detection is None:
            self._publish_target({
                "detected": False,
                "state": self.state,
                "busy": self.sequence_active,
            })
            debug_frame = self._draw_debug(frame, None)
            self._publish_debug_image(debug_frame, msg.header.frame_id)
            return

        x_error = float(detection["x_error"])
        y_error = float(detection["y_error"])
        fill_ratio = float(detection["fill_ratio"])
        aligned = (
            abs(x_error) <= self.center_tolerance_x
            and abs(y_error) <= self.center_tolerance_y
        )

        if not self.sequence_active and (now - self.last_command_time) >= self.command_period_sec:
            next_pose = self.current_pose[:]
            action = "hold"

            if abs(x_error) > self.center_tolerance_x:
                next_pose[self.x_joint_index] += self.x_joint_gain * x_error
                action = "align_x"

            if abs(y_error) > self.center_tolerance_y:
                next_pose[self.y_joint_index] += self.y_joint_gain * y_error
                action = "align_y" if action == "hold" else f"{action}+align_y"

            if fill_ratio < self.approach_fill_ratio:
                next_pose[self.reach_joint_index] += self.reach_step_rad
                action = "approach" if action == "hold" else f"{action}+approach"

            next_pose = self._clamp_pose(next_pose)

            if action != "hold":
                self.state = action
                self._publish_status(f"red picker: {action}")
                self._send_arm_pose(next_pose, self.move_duration_sec)
            elif self.auto_pick and aligned and fill_ratio >= self.pick_fill_ratio_threshold:
                self.state = "pick_sequence"
                threading.Thread(
                    target=self._run_pick_sequence,
                    args=(self.current_pose[:],),
                    daemon=True,
                ).start()

        target_payload = {
            "detected": True,
            "state": self.state,
            "busy": self.sequence_active,
            "cx_px": int(detection["cx"]),
            "cy_px": int(detection["cy"]),
            "width_px": int(detection["w"]),
            "height_px": int(detection["h"]),
            "area_px": float(detection["area"]),
            "fill_ratio": fill_ratio,
            "x_error": x_error,
            "y_error": y_error,
            "aligned": aligned,
        }
        self._publish_target(target_payload)
        debug_frame = self._draw_debug(frame, detection)
        self._publish_debug_image(debug_frame, msg.header.frame_id)

    def _detect_largest_red_blob(self, frame: np.ndarray) -> Optional[Dict[str, float]]:
        blurred = frame
        if self.blur_kernel_size >= 3 and self.blur_kernel_size % 2 == 1:
            blurred = cv2.GaussianBlur(frame, (self.blur_kernel_size, self.blur_kernel_size), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        if self.morph_open_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
                iterations=self.morph_open_iterations,
            )
        if self.morph_close_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=self.morph_close_iterations,
            )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_contour_area_px:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None

        img_h, img_w = frame.shape[:2]
        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
        x_error = (cx - (img_w / 2.0)) / max(1.0, img_w / 2.0)
        y_error = (cy - (img_h / 2.0)) / max(1.0, img_h / 2.0)

        return {
            "x": float(x),
            "y": float(y),
            "w": float(w),
            "h": float(h),
            "cx": cx,
            "cy": cy,
            "area": area,
            "fill_ratio": area / float(img_w * img_h),
            "x_error": float(x_error),
            "y_error": float(y_error),
        }

    def _draw_debug(self, frame: np.ndarray, detection: Optional[Dict[str, float]]) -> np.ndarray:
        debug = frame.copy()
        img_h, img_w = debug.shape[:2]
        cv2.line(debug, (img_w // 2, 0), (img_w // 2, img_h), (255, 255, 0), 1)
        cv2.line(debug, (0, img_h // 2), (img_w, img_h // 2), (255, 255, 0), 1)

        if detection is not None:
            x = int(detection["x"])
            y = int(detection["y"])
            w = int(detection["w"])
            h = int(detection["h"])
            cx = int(detection["cx"])
            cy = int(detection["cy"])
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"red target x={detection['x_error']:+.2f} y={detection['y_error']:+.2f}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                debug,
                f"fill={detection['fill_ratio']:.3f} state={self.state}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
        else:
            cv2.putText(
                debug,
                f"no red target state={self.state}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        return debug

    def _run_pick_sequence(self, base_pose: List[float]) -> None:
        self.sequence_active = True
        try:
            open_pose = base_pose[:]
            open_pose[self.gripper_joint_index] = self.gripper_open_rad
            self._publish_status("red picker: open gripper")
            self._send_arm_pose(open_pose, self.move_duration_sec)
            time.sleep(self.pause_after_move_sec)

            pregrasp_pose = open_pose[:]
            pregrasp_pose[self.reach_joint_index] += self.pregrasp_reach_delta_rad
            pregrasp_pose = self._clamp_pose(pregrasp_pose)
            self._publish_status("red picker: pregrasp")
            self._send_arm_pose(pregrasp_pose, self.move_duration_sec)
            time.sleep(self.pause_after_move_sec)

            close_pose = pregrasp_pose[:]
            close_pose[self.gripper_joint_index] = self.gripper_close_rad
            self._publish_status("red picker: close gripper")
            self._send_arm_pose(close_pose, self.move_duration_sec)
            time.sleep(self.pause_after_grip_sec)

            lift_pose = close_pose[:]
            lift_pose[self.lift_joint_index] += self.lift_delta_rad
            lift_pose = self._clamp_pose(lift_pose)
            self._publish_status("red picker: lift")
            self._send_arm_pose(lift_pose, self.move_duration_sec)
            time.sleep(self.pause_after_move_sec)

            carry_pose = self.carry_pose[:]
            carry_pose[self.gripper_joint_index] = self.gripper_close_rad
            carry_pose = self._clamp_pose(carry_pose)
            self._publish_status("red picker: carry pose")
            self._send_arm_pose(carry_pose, self.move_duration_sec)
            time.sleep(self.pause_after_move_sec)

            self.state = "carry_pose"
        finally:
            self.sequence_active = False

    def _clamp_pose(self, pose: List[float]) -> List[float]:
        return [
            max(low, min(high, float(value)))
            for value, low, high in zip(
                pose, self.command_joint_min_rad, self.command_joint_max_rad
            )
        ]

    def _send_arm_pose(self, positions: List[float], duration_sec: float) -> None:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        msg = JointTrajectory()
        msg.joint_names = self.joint_names[:]
        msg.points = [point]
        self.arm_pub.publish(msg)
        self.last_command_time = time.monotonic()

    def _publish_debug_image(self, frame: np.ndarray, frame_id: str) -> None:
        stamp = self.get_clock().now().to_msg()
        resolved_frame_id = frame_id or "camera_link"

        if self.publish_raw_debug_image:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = resolved_frame_id
            self.debug_pub.publish(msg)

        if self.publish_compressed_debug_image:
            encode_params = [
                int(cv2.IMWRITE_JPEG_QUALITY),
                max(20, min(95, self.compressed_jpeg_quality)),
            ]
            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = stamp
                comp_msg.header.frame_id = resolved_frame_id
                comp_msg.format = "jpeg"
                comp_msg.data = encoded.tobytes()
                self.debug_compressed_pub.publish(comp_msg)

    def _publish_target(self, payload: Dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.target_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _warn_once(self, text: str) -> None:
        self.get_logger().warn(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedObjectPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
