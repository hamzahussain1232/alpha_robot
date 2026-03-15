#!/usr/bin/env python3
import os
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CaptureDatasetNode(Node):
    def __init__(self) -> None:
        super().__init__("capture_dataset_node")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter(
            "output_root",
            os.path.expanduser("~/ros2_ws/src/articubot_one/assets/ml/dataset/images_all"),
        )
        self.declare_parameter("class_name", "medicine_bottle")
        self.declare_parameter("save_every_n_frames", 5)
        self.declare_parameter("max_images", 300)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("show_preview", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_root = Path(str(self.get_parameter("output_root").value)).expanduser()
        self.class_name = str(self.get_parameter("class_name").value).strip().lower().replace(" ", "_")
        self.save_every_n_frames = max(1, int(self.get_parameter("save_every_n_frames").value))
        self.max_images = max(1, int(self.get_parameter("max_images").value))
        self.image_index = max(0, int(self.get_parameter("start_index").value))
        self.jpeg_quality = min(100, max(50, int(self.get_parameter("jpeg_quality").value)))
        self.show_preview = bool(self.get_parameter("show_preview").value)

        self.output_root.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.start_ts = time.time()

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.get_logger().info(
            f"Dataset capture started: topic={self.image_topic} class={self.class_name} "
            f"max_images={self.max_images} save_every_n_frames={self.save_every_n_frames} "
            f"out={self.output_root}"
        )

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if self.saved_count >= self.max_images:
            self.get_logger().info("Capture complete, stopping node.")
            rclpy.shutdown()
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image convert failed: {exc}")
            return

        if self.show_preview:
            cv2.imshow("dataset_capture_preview", frame)
            cv2.waitKey(1)

        if self.frame_count % self.save_every_n_frames != 0:
            return

        stamp = int(time.time() * 1000)
        filename = f"{self.class_name}_{stamp}_{self.image_index:06d}.jpg"
        filepath = self.output_root / filename
        ok = cv2.imwrite(
            str(filepath),
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warn(f"Failed to save image: {filepath}")
            return

        self.image_index += 1
        self.saved_count += 1
        if self.saved_count % 20 == 0 or self.saved_count == 1:
            elapsed = max(1e-6, time.time() - self.start_ts)
            rate = self.saved_count / elapsed
            self.get_logger().info(
                f"Saved {self.saved_count}/{self.max_images} images ({rate:.2f} img/s): {filepath.name}"
            )


def main() -> None:
    rclpy.init()
    node = CaptureDatasetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
