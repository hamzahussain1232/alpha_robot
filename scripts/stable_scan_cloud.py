#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def rotate_point(q, x, y, z):
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Quaternion rotation: p' = q * p * q^-1, expanded to avoid extra deps.
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z

    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


class StableScanCloud(Node):
    def __init__(self):
        super().__init__('stable_scan_cloud')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/stable_scan_cloud')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('max_age_sec', 0.0)
        self.declare_parameter('stride', 1)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('ignore_range_margin', 0.05)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.ignore_range_margin = max(0.0, float(self.get_parameter('ignore_range_margin').value))

        self.points = deque()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Publishing stable map-frame scan cloud: {self.scan_topic} -> {self.cloud_topic}'
        )

    def on_scan(self, scan):
        if not scan.header.frame_id:
            return

        stamp = Time.from_msg(scan.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().debug(f'No timestamped transform for scan yet: {exc}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        now = self.get_clock().now().nanoseconds / 1e9
        cutoff = now - self.max_age_sec

        while self.points and self.points[0][0] <= cutoff:
            self.points.popleft()

        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if i % self.stride:
                angle += scan.angle_increment
                continue
            no_hit_threshold = min(self.max_range, scan.range_max) - self.ignore_range_margin
            if math.isfinite(r) and self.min_range <= r <= no_hit_threshold:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                lz = 0.0
                rx, ry, rz = rotate_point(q, lx, ly, lz)
                self.points.append((now, (rx + t.x, ry + t.y, rz + t.z)))
            angle += scan.angle_increment

        cloud = point_cloud2.create_cloud_xyz32(
            scan.header,
            [p for _, p in self.points],
        )
        cloud.header.frame_id = self.target_frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = StableScanCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
