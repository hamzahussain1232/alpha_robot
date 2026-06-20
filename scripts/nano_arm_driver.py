#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

import serial


def open_serial_port(port: str, baud: int, timeout: float):
    try:
        return serial.Serial(port, baud, timeout=timeout, exclusive=True)
    except TypeError:
        return serial.Serial(port, baud, timeout=timeout)


class NanoArmDriver(Node):
    def __init__(self):
        super().__init__('nano_arm_driver')

        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
        )
        self.declare_parameter('baud', 9600)
        self.declare_parameter('serial_boot_wait_sec', 2.0)
        self.declare_parameter('command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_names', joint_names)
        self.declare_parameter('joint_min_rad', [-1.57] * 6)
        self.declare_parameter('joint_max_rad', [1.57] * 6)
        self.declare_parameter('servo_home_deg', [90.0] * 6)
        self.declare_parameter('servo_direction', [1, 1, 1, 1, 1, 1])
        self.declare_parameter('servo_min_deg', [0.0] * 6)
        self.declare_parameter('servo_max_deg', [180.0] * 6)
        self.declare_parameter('gripper_joint_index', 5)
        self.declare_parameter('gripper_open_deg', 90.0)
        self.declare_parameter('gripper_close_deg', 180.0)
        self.declare_parameter('startup_positions_rad', [0.0] * 6)
        self.declare_parameter('write_on_startup', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('default_move_duration_sec', 0.5)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('debug', False)

        self.port = str(self.get_parameter('port').value).strip()
        self.baud = int(self.get_parameter('baud').value)
        self.serial_boot_wait_sec = float(self.get_parameter('serial_boot_wait_sec').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.joint_names = [str(v) for v in self.get_parameter('joint_names').value]
        self.joint_min_rad = [float(v) for v in self.get_parameter('joint_min_rad').value]
        self.joint_max_rad = [float(v) for v in self.get_parameter('joint_max_rad').value]
        self.servo_home_deg = [float(v) for v in self.get_parameter('servo_home_deg').value]
        self.servo_direction = [int(v) for v in self.get_parameter('servo_direction').value]
        self.servo_min_deg = [float(v) for v in self.get_parameter('servo_min_deg').value]
        self.servo_max_deg = [float(v) for v in self.get_parameter('servo_max_deg').value]
        self.gripper_joint_index = int(self.get_parameter('gripper_joint_index').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)
        self.gripper_close_deg = float(self.get_parameter('gripper_close_deg').value)
        self.startup_positions_rad = [
            float(v) for v in self.get_parameter('startup_positions_rad').value
        ]
        self.write_on_startup = bool(self.get_parameter('write_on_startup').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.default_move_duration_sec = float(
            self.get_parameter('default_move_duration_sec').value
        )
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)
        self.debug = bool(self.get_parameter('debug').value)

        self._validate_lengths()

        self._state_lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._current_positions = self.startup_positions_rad[:]
        self._start_positions = self.startup_positions_rad[:]
        self._target_positions = self.startup_positions_rad[:]
        self._motion_start_time = time.time()
        self._motion_duration = 0.0
        self._last_sent_angles = None
        self._last_serial_error = 0.0
        self._last_debug_log = 0.0
        self._serial_ready_at = 0.0
        self.ser = None
        self._ensure_serial_ready()
        if self.write_on_startup:
            self._send_positions(self._current_positions, force=True)

        self.traj_sub = self.create_subscription(
            JointTrajectory, self.command_topic, self._trajectory_cb, 10
        )
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self._tick)

        self.get_logger().info(
            f'Nano arm driver listening on {self.command_topic} via {self.port} @ {self.baud}'
        )

    def _validate_lengths(self):
        expected = len(self.joint_names)
        arrays = {
            'joint_min_rad': self.joint_min_rad,
            'joint_max_rad': self.joint_max_rad,
            'servo_home_deg': self.servo_home_deg,
            'servo_direction': self.servo_direction,
            'servo_min_deg': self.servo_min_deg,
            'servo_max_deg': self.servo_max_deg,
            'startup_positions_rad': self.startup_positions_rad,
        }
        for name, values in arrays.items():
            if len(values) != expected:
                raise RuntimeError(
                    f'{name} length {len(values)} does not match joint_names length {expected}'
                )

    def _ensure_serial_ready(self):
        if self.ser is not None and getattr(self.ser, 'is_open', False):
            return
        try:
            ser = open_serial_port(self.port, self.baud, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.ser = ser
            self._serial_ready_at = time.time() + self.serial_boot_wait_sec
        except Exception as exc:
            now = time.time()
            if (now - self._last_serial_error) > 2.0:
                self._last_serial_error = now
                self.get_logger().error(f'Failed to open Nano serial port {self.port}: {exc}')

    def _trajectory_cb(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Ignoring empty arm trajectory')
            return

        point = msg.points[-1]
        if not point.positions:
            self.get_logger().warn('Ignoring arm trajectory with no positions')
            return

        names = list(msg.joint_names) if msg.joint_names else self.joint_names
        if len(names) != len(point.positions):
            self.get_logger().warn('Ignoring arm trajectory with mismatched names and positions')
            return

        with self._state_lock:
            now = time.time()
            self._current_positions = self._interpolated_positions_locked(now)
            self._start_positions = self._current_positions[:]
            requested = self._target_positions[:]
            name_to_index = {name: idx for idx, name in enumerate(self.joint_names)}
            for name, position in zip(names, point.positions):
                idx = name_to_index.get(name)
                if idx is None:
                    continue
                requested[idx] = self._clamp_joint_rad(idx, float(position))

            duration = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            if duration <= 0.0:
                duration = self.default_move_duration_sec

            self._target_positions = requested
            self._motion_start_time = now
            self._motion_duration = max(0.05, duration)
            if self.debug:
                self.get_logger().info(
                    'Received arm target rad=' + str([round(v, 3) for v in self._target_positions])
                )

    def _tick(self):
        now = time.time()
        with self._state_lock:
            self._current_positions = self._interpolated_positions_locked(now)
            positions = self._current_positions[:]

        self._send_positions(positions)
        if self.publish_joint_states:
            self._publish_joint_states(positions, now)

    def _interpolated_positions_locked(self, now: float):
        if self._motion_duration <= 1e-6:
            return self._target_positions[:]
        progress = (now - self._motion_start_time) / self._motion_duration
        progress = max(0.0, min(1.0, progress))
        if progress >= 1.0:
            return self._target_positions[:]
        return [
            start + ((target - start) * progress)
            for start, target in zip(self._start_positions, self._target_positions)
        ]

    def _send_positions(self, positions, force: bool = False):
        angles = [self._joint_to_servo_deg(idx, pos) for idx, pos in enumerate(positions)]
        rounded = [int(round(v)) for v in angles]

        if (not force) and self._last_sent_angles == rounded:
            return

        self._ensure_serial_ready()
        if self.ser is None or not getattr(self.ser, 'is_open', False):
            return
        if time.time() < self._serial_ready_at:
            return

        line = 'A ' + ' '.join(str(v) for v in rounded) + '\n'
        try:
            with self._serial_lock:
                self.ser.write(line.encode('ascii'))
                self.ser.flush()
            self._last_sent_angles = rounded
            if self.debug:
                now = time.time()
                if (now - self._last_debug_log) > 1.0:
                    self._last_debug_log = now
                    self.get_logger().info(f'Sent Nano arm angles deg={rounded}')
        except Exception as exc:
            try:
                self.ser.close()
            except BaseException:
                pass
            self.ser = None
            now = time.time()
            if (now - self._last_serial_error) > 1.0:
                self._last_serial_error = now
                self.get_logger().error(f'Failed to write Nano serial command: {exc}')

    def _publish_joint_states(self, positions, now: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names[:]
        msg.position = positions[:]
        if self._motion_duration > 1e-6 and (now - self._motion_start_time) < self._motion_duration:
            msg.velocity = [
                (target - start) / self._motion_duration
                for start, target in zip(self._start_positions, self._target_positions)
            ]
        else:
            msg.velocity = [0.0] * len(self.joint_names)
        self.joint_pub.publish(msg)

    def _clamp_joint_rad(self, idx: int, position_rad: float) -> float:
        return max(self.joint_min_rad[idx], min(self.joint_max_rad[idx], position_rad))

    def _joint_to_servo_deg(self, idx: int, position_rad: float) -> float:
        clamped = self._clamp_joint_rad(idx, position_rad)
        servo_deg = self.servo_home_deg[idx] + (
            self.servo_direction[idx] * math.degrees(clamped)
        )
        servo_min = self.servo_min_deg[idx]
        servo_max = self.servo_max_deg[idx]
        if idx == self.gripper_joint_index and self.gripper_close_deg >= 0.0:
            servo_max = min(servo_max, self.gripper_close_deg)
        return max(servo_min, min(servo_max, servo_deg))

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except BaseException:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NanoArmDriver()
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
