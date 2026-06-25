#!/usr/bin/env python3

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PhoneMjpegCamera(Node):
    def __init__(self):
        super().__init__("phone_mjpeg_camera")

        self.declare_parameter("stream_url", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "phone_camera_optical_frame")
        self.declare_parameter("width", 480)
        self.declare_parameter("height", 360)
        self.declare_parameter("publish_fps", 15.0)
        self.declare_parameter("retry_sec", 2.0)

        self.stream_url = str(
            self.get_parameter("stream_url").value
        ).strip()

        self.image_topic = str(
            self.get_parameter("image_topic").value
        )

        self.frame_id = str(
            self.get_parameter("frame_id").value
        )

        self.width = int(
            self.get_parameter("width").value
        )

        self.height = int(
            self.get_parameter("height").value
        )

        self.publish_fps = max(
            1.0,
            float(self.get_parameter("publish_fps").value)
        )

        self.retry_sec = max(
            0.5,
            float(self.get_parameter("retry_sec").value)
        )

        if not self.stream_url:
            raise RuntimeError(
                "stream_url is required. Example: "
                "http://PHONE_IP:8080/video"
            )

        cv2.setNumThreads(1)

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            self.image_topic,
            qos_profile_sensor_data,
        )

        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.total_received = 0
        self.total_published = 0

        self.worker = threading.Thread(
            target=self.capture_loop,
            daemon=True,
        )
        self.worker.start()

        self.timer = self.create_timer(
            1.0 / self.publish_fps,
            self.publish_latest_frame,
        )

        self.get_logger().info(
            f"Low-latency phone camera bridge: {self.stream_url}"
        )

        self.get_logger().info(
            f"Publishing latest frames to: {self.image_topic}"
        )

    def open_capture(self):
        self.get_logger().info(
            "Connecting to Pixel camera stream..."
        )

        cap = cv2.VideoCapture(
            self.stream_url,
            cv2.CAP_FFMPEG,
        )

        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(self.stream_url)

        if not cap.isOpened():
            cap.release()
            return None

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info(
            "Pixel camera stream connected."
        )

        return cap

    def capture_loop(self):
        while self.running and rclpy.ok():
            cap = self.open_capture()

            if cap is None:
                self.get_logger().warn(
                    "Cannot connect to phone stream. Retrying..."
                )
                time.sleep(self.retry_sec)
                continue

            while self.running and rclpy.ok():
                ok, frame = cap.read()

                if not ok or frame is None:
                    self.get_logger().warn(
                        "Phone stream interrupted. Reconnecting..."
                    )
                    break

                if (
                    frame.shape[1] != self.width
                    or frame.shape[0] != self.height
                ):
                    frame = cv2.resize(
                        frame,
                        (self.width, self.height),
                        interpolation=cv2.INTER_AREA,
                    )

                with self.frame_lock:
                    self.latest_frame = frame

                self.total_received += 1

            cap.release()
            time.sleep(self.retry_sec)

    def publish_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return

            frame = self.latest_frame.copy()

        message = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        self.publisher.publish(message)

        self.total_published += 1

        if self.total_published % 150 == 0:
            self.get_logger().info(
                f"Received={self.total_received}, "
                f"published={self.total_published}"
            )

    def destroy_node(self):
        self.running = False

        if self.worker.is_alive():
            self.worker.join(timeout=2.0)

        super().destroy_node()


def main():
    rclpy.init()
    node = PhoneMjpegCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
