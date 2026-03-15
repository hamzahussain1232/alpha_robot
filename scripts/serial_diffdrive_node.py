#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Int64MultiArray

import serial


class SerialDiffDrive(Node):
    def __init__(self):
        super().__init__('serial_diffdrive_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/diff_cont/cmd_vel')
        self.declare_parameter('use_stamped', True)
        self.declare_parameter('wheel_separation', 0.31)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('timeout_sec', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.use_stamped = bool(self.get_parameter('use_stamped').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.last_cmd_time = time.time()

        self.encoder_pub = self.create_publisher(Int64MultiArray, '/encoder_counts', 10)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
        except Exception as exc:
            self.get_logger().error(f'Failed to open serial {self.port}: {exc}')
            raise

        if self.use_stamped:
            self.sub = self.create_subscription(
                TwistStamped, self.cmd_vel_topic, self._cmd_cb_stamped, 10
            )
        else:
            self.sub = self.create_subscription(
                Twist, self.cmd_vel_topic, self._cmd_cb, 10
            )

        self.timer = self.create_timer(0.05, self._tick)
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _cmd_cb_stamped(self, msg: TwistStamped):
        self._handle_cmd(msg.twist)

    def _cmd_cb(self, msg: Twist):
        self._handle_cmd(msg)

    def _handle_cmd(self, msg: Twist):
        if not all(math.isfinite(v) for v in (
            msg.linear.x, msg.angular.z
        )):
            self.get_logger().warn('Ignoring cmd_vel with NaN/Inf')
            return

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Differential drive kinematics
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        left_pwm = self._vel_to_pwm(v_left, v, w)
        right_pwm = self._vel_to_pwm(v_right, v, w)

        self._send_pwm(left_pwm, right_pwm)
        self.last_cmd_time = time.time()

    def _vel_to_pwm(self, v_wheel, v, w):
        # Scale by max linear or max angular (whichever dominates)
        scale = self.max_linear
        if abs(w) > 1e-6:
            scale = max(scale, abs(w) * self.wheel_separation / 2.0)
        if scale <= 1e-6:
            return 0
        pwm = int((v_wheel / scale) * self.max_pwm)
        return max(-self.max_pwm, min(self.max_pwm, pwm))

    def _send_pwm(self, left_pwm, right_pwm):
        try:
            line = f'M {left_pwm} {right_pwm}\n'
            self.ser.write(line.encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'Failed to write serial: {exc}')

    def _tick(self):
        # Stop if no commands for a while
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self._send_pwm(0, 0)

    def _read_loop(self):
        buffer = b''
        while rclpy.ok():
            try:
                chunk = self.ser.read(128)
                if chunk:
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        self._handle_line(line.decode('ascii', errors='ignore').strip())
                else:
                    time.sleep(0.01)
            except Exception as exc:
                self.get_logger().warn(f'Serial read error: {exc}')
                time.sleep(0.1)

    def _handle_line(self, line: str):
        if not line:
            return
        if line.startswith('E '):
            parts = line.split()
            if len(parts) == 5:
                try:
                    counts = [int(p) for p in parts[1:]]
                    msg = Int64MultiArray()
                    msg.data = counts
                    self.encoder_pub.publish(msg)
                except ValueError:
                    self.get_logger().warn(f'Bad encoder line: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialDiffDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
