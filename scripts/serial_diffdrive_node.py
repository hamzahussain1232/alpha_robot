#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray

import serial
import tf2_ros


class SerialDiffDrive(Node):
    def __init__(self):
        super().__init__('serial_diffdrive_node')

        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/diff_cont/cmd_vel')
        self.declare_parameter('use_stamped', True)
        self.declare_parameter('enable_fallback_cmd_vel', True)
        self.declare_parameter('fallback_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('fallback_timeout_sec', 0.2)
        self.declare_parameter('wheel_separation', 0.31)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('encoder_cpr', 330)
        self.declare_parameter('encoder_sign_fl', 1)
        self.declare_parameter('encoder_sign_fr', 1)
        self.declare_parameter('encoder_sign_rl', 1)
        self.declare_parameter('encoder_sign_rr', 1)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_linear_scale', 1.0)
        self.declare_parameter('odom_angular_scale', 0.735)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 1.7)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('min_nonzero_pwm', 130)
        self.declare_parameter('min_turn_pwm', 150)
        self.declare_parameter('turn_pwm_scale', 1.0)
        self.declare_parameter('reverse_pwm_scale', 1.0)
        self.declare_parameter('left_forward_pwm_scale', 1.0)
        self.declare_parameter('left_reverse_pwm_scale', 1.0)
        self.declare_parameter('right_forward_pwm_scale', 1.0)
        self.declare_parameter('right_reverse_pwm_scale', 1.0)
        self.declare_parameter('turn_in_place_threshold', 0.05)
        self.declare_parameter('turn_assist_cmd_w_threshold', 0.0)
        self.declare_parameter('turn_assist_min_pwm_delta', 0)
        self.declare_parameter('straight_trim_pwm', 8)
        self.declare_parameter('straight_trim_min_cmd_vel', 0.05)
        self.declare_parameter('straight_trim_max_cmd_w', 0.15)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('serial_boot_wait_sec', 2.0)
        self.declare_parameter('reconnect_on_empty_read_error', False)
        self.declare_parameter('debug', True)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.use_stamped = bool(self.get_parameter('use_stamped').value)
        self.enable_fallback_cmd_vel = bool(self.get_parameter('enable_fallback_cmd_vel').value)
        self.fallback_cmd_vel_topic = self.get_parameter('fallback_cmd_vel_topic').value
        self.fallback_timeout_sec = float(self.get_parameter('fallback_timeout_sec').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.encoder_cpr = int(self.get_parameter('encoder_cpr').value)
        self.encoder_signs = [
            self._normalize_sign(self.get_parameter('encoder_sign_fl').value),
            self._normalize_sign(self.get_parameter('encoder_sign_fr').value),
            self._normalize_sign(self.get_parameter('encoder_sign_rl').value),
            self._normalize_sign(self.get_parameter('encoder_sign_rr').value),
        ]
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_linear_scale = float(self.get_parameter('odom_linear_scale').value)
        self.odom_angular_scale = float(self.get_parameter('odom_angular_scale').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.min_nonzero_pwm = int(self.get_parameter('min_nonzero_pwm').value)
        self.min_turn_pwm = int(self.get_parameter('min_turn_pwm').value)
        self.turn_pwm_scale = float(self.get_parameter('turn_pwm_scale').value)
        self.reverse_pwm_scale = float(self.get_parameter('reverse_pwm_scale').value)
        self.left_forward_pwm_scale = float(self.get_parameter('left_forward_pwm_scale').value)
        self.left_reverse_pwm_scale = float(self.get_parameter('left_reverse_pwm_scale').value)
        self.right_forward_pwm_scale = float(self.get_parameter('right_forward_pwm_scale').value)
        self.right_reverse_pwm_scale = float(self.get_parameter('right_reverse_pwm_scale').value)
        self.turn_in_place_threshold = float(self.get_parameter('turn_in_place_threshold').value)
        self.turn_assist_cmd_w_threshold = float(
            self.get_parameter('turn_assist_cmd_w_threshold').value
        )
        self.turn_assist_min_pwm_delta = int(
            self.get_parameter('turn_assist_min_pwm_delta').value
        )
        self.straight_trim_pwm = int(self.get_parameter('straight_trim_pwm').value)
        self.straight_trim_min_cmd_vel = float(self.get_parameter('straight_trim_min_cmd_vel').value)
        self.straight_trim_max_cmd_w = float(self.get_parameter('straight_trim_max_cmd_w').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.serial_boot_wait_sec = float(self.get_parameter('serial_boot_wait_sec').value)
        self.reconnect_on_empty_read_error = bool(self.get_parameter('reconnect_on_empty_read_error').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.last_cmd_time = time.time()
        self.last_stamped_cmd_time = 0.0
        self.last_stamped_nonzero_cmd_time = 0.0
        self._last_cmd = (0.0, 0.0)
        self._last_pwm = (0, 0)
        self._last_debug_log = 0.0
        self._last_fallback_drop_log = 0.0
        self._last_tx_log = 0.0
        self._last_tx = (None, None)
        self._serial_ready_at = 0.0
        self._last_boot_wait_log = 0.0
        self._last_encoder_log = 0.0
        self._last_transient_read_err_log = 0.0
        self._empty_read_err_count = 0
        self._empty_read_err_window_start = 0.0

        self.encoder_pub = self.create_publisher(Int64MultiArray, '/encoder_counts', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._last_counts = None
        self._last_enc_time = None
        self._wheel_pos = [0.0, 0.0, 0.0, 0.0]  # fl, fr, rl, rr
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self.ser = None
        self._serial_lock = threading.Lock()
        self._connected = False
        self._last_open_log = 0.0

        if self.use_stamped:
            self.sub = self.create_subscription(
                TwistStamped, self.cmd_vel_topic, self._cmd_cb_stamped, 10
            )
        else:
            self.sub = self.create_subscription(
                Twist, self.cmd_vel_topic, self._cmd_cb, 10
            )

        self._fallback_enabled = False
        if self.enable_fallback_cmd_vel:
            # In direct mode these can point to the same topic; avoid duplicate writes.
            if (not self.use_stamped) and (self.fallback_cmd_vel_topic == self.cmd_vel_topic):
                self.get_logger().warn(
                    'Fallback cmd_vel disabled: fallback topic equals primary topic in Twist mode'
                )
            else:
                # Fallback path for plain Twist (useful if mux/bridge is not active).
                self.sub_fallback = self.create_subscription(
                    Twist, self.fallback_cmd_vel_topic, self._cmd_cb_fallback, 10
                )
                self._fallback_enabled = True

        self.get_logger().info(
            f'Listening on {self.cmd_vel_topic} '
            f'({"TwistStamped" if self.use_stamped else "Twist"})'
        )
        if self._fallback_enabled:
            self.get_logger().info(f'Fallback on {self.fallback_cmd_vel_topic} (Twist)')
        self.get_logger().info(
            f'Encoder signs FL/FR/RL/RR = '
            f'{self.encoder_signs[0]}/{self.encoder_signs[1]}/'
            f'{self.encoder_signs[2]}/{self.encoder_signs[3]}'
        )

        self.timer = self.create_timer(0.05, self._tick)
        self.open_timer = self.create_timer(1.0, self._try_open_serial)
        self.debug_timer = self.create_timer(1.0, self._debug_tick)
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _try_open_serial(self):
        if self.ser is not None and getattr(self.ser, "is_open", False):
            if not self._connected:
                self._connected = True
                self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
            return

        now = time.time()
        # Throttle open failure logs to avoid spam
        if (now - self._last_open_log) > 2.0:
            self._last_open_log = now
            self.get_logger().warn(f'Trying to open serial {self.port} @ {self.baud}...')

        try:
            with self._serial_lock:
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.05,
                    write_timeout=0.05,
                )
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                # A real serial reopen can reset the Arduino, so re-seed encoder
                # integration on the next valid frame instead of diffing across the reset.
                self._last_counts = None
                self._last_enc_time = None
                self._serial_ready_at = time.time() + max(0.0, self.serial_boot_wait_sec)
                self._connected = True
                self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
                if self.serial_boot_wait_sec > 0.0:
                    self.get_logger().info(
                        f'Waiting {self.serial_boot_wait_sec:.1f}s for Arduino boot before TX'
                    )
        except Exception as exc:
            self._connected = False
            if (now - self._last_open_log) > 2.0:
                self._last_open_log = now
                self.get_logger().warn(f'Failed to open serial {self.port}: {exc}')

    def _cmd_cb_stamped(self, msg: TwistStamped):
        self._handle_cmd(msg.twist, source='stamped')

    @staticmethod
    def _normalize_sign(value):
        return -1 if int(value) < 0 else 1

    def _cmd_cb(self, msg: Twist):
        self._handle_cmd(msg, source='plain')

    def _cmd_cb_fallback(self, msg: Twist):
        self._handle_cmd(msg, source='fallback')

    def _handle_cmd(self, msg: Twist, source: str):
        now = time.time()
        if source in ('plain', 'fallback') and self.use_stamped:
            # Ignore fallback only if we recently received NON-ZERO stamped commands.
            # This prevents a zero-only stamped stream from blocking real manual commands.
            if (now - self.last_stamped_nonzero_cmd_time) < self.fallback_timeout_sec:
                if self.debug and (now - self._last_fallback_drop_log) > 2.0:
                    self._last_fallback_drop_log = now
                    self.get_logger().warn(
                        'Ignoring /cmd_vel fallback because non-zero stamped commands are active'
                    )
                return

        if not all(math.isfinite(v) for v in (
            msg.linear.x, msg.angular.z
        )):
            self.get_logger().warn('Ignoring cmd_vel with NaN/Inf')
            return

        v = max(-self.max_linear, min(self.max_linear, float(msg.linear.x)))
        w = max(-self.max_angular, min(self.max_angular, float(msg.angular.z)))

        # Differential drive kinematics
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        in_place_turn = abs(v) < self.turn_in_place_threshold and abs(w) > 1e-3
        left_pwm = self._vel_to_pwm(v_left, in_place_turn, side='left')
        right_pwm = self._vel_to_pwm(v_right, in_place_turn, side='right')
        left_pwm, right_pwm = self._apply_straight_trim(left_pwm, right_pwm, v, w, in_place_turn)
        left_pwm, right_pwm = self._apply_turn_assist(left_pwm, right_pwm, w, in_place_turn)

        self._send_pwm(left_pwm, right_pwm)
        self.last_cmd_time = now
        self._last_cmd = (v, w)
        self._last_pwm = (left_pwm, right_pwm)
        if source == 'stamped':
            self.last_stamped_cmd_time = now
            if abs(v) > 1e-4 or abs(w) > 1e-4:
                self.last_stamped_nonzero_cmd_time = now

    def _vel_to_pwm(self, v_wheel, in_place_turn, side):
        scale = self.max_linear
        if in_place_turn:
            scale = self.max_angular * self.wheel_separation / 2.0
        if scale <= 1e-6:
            return 0
        pwm = int((v_wheel / scale) * self.max_pwm)
        if v_wheel < 0.0:
            pwm = int(pwm * self.reverse_pwm_scale)
        pwm = int(pwm * self._directional_pwm_scale(side, v_wheel))
        if in_place_turn:
            pwm = int(pwm * self.turn_pwm_scale)
        if pwm != 0:
            sign = 1 if pwm > 0 else -1
            pwm_abs = abs(pwm)
            min_pwm = self.min_turn_pwm if in_place_turn else self.min_nonzero_pwm
            if pwm_abs < min_pwm:
                pwm = sign * min_pwm
        return max(-self.max_pwm, min(self.max_pwm, pwm))

    def _directional_pwm_scale(self, side, v_wheel):
        if side == 'left':
            return self.left_forward_pwm_scale if v_wheel >= 0.0 else self.left_reverse_pwm_scale
        return self.right_forward_pwm_scale if v_wheel >= 0.0 else self.right_reverse_pwm_scale

    def _apply_straight_trim(self, left_pwm, right_pwm, v, w, in_place_turn):
        if in_place_turn:
            return left_pwm, right_pwm
        if abs(v) < self.straight_trim_min_cmd_vel:
            return left_pwm, right_pwm
        if abs(w) > self.straight_trim_max_cmd_w:
            return left_pwm, right_pwm
        trim = int(self.straight_trim_pwm)
        if trim <= 0:
            return left_pwm, right_pwm

        direction = 1 if v >= 0.0 else -1
        left_pwm = max(-self.max_pwm, min(self.max_pwm, int(left_pwm - (direction * trim))))
        right_pwm = max(-self.max_pwm, min(self.max_pwm, int(right_pwm + (direction * trim))))
        return left_pwm, right_pwm

    def _apply_turn_assist(self, left_pwm, right_pwm, w, in_place_turn):
        if in_place_turn:
            return left_pwm, right_pwm
        if self.turn_assist_min_pwm_delta <= 0:
            return left_pwm, right_pwm
        if abs(w) < self.turn_assist_cmd_w_threshold:
            return left_pwm, right_pwm

        required_delta = int(self.turn_assist_min_pwm_delta)
        if w > 0.0:
            current_delta = right_pwm - left_pwm
            if current_delta >= required_delta:
                return left_pwm, right_pwm
            right_pwm = min(self.max_pwm, right_pwm + (required_delta - current_delta))
            current_delta = right_pwm - left_pwm
            if current_delta < required_delta:
                left_pwm = max(-self.max_pwm, right_pwm - required_delta)
        else:
            current_delta = left_pwm - right_pwm
            if current_delta >= required_delta:
                return left_pwm, right_pwm
            left_pwm = min(self.max_pwm, left_pwm + (required_delta - current_delta))
            current_delta = left_pwm - right_pwm
            if current_delta < required_delta:
                right_pwm = max(-self.max_pwm, left_pwm - required_delta)

        return left_pwm, right_pwm

    def _send_pwm(self, left_pwm, right_pwm):
        if self.ser is None or not getattr(self.ser, "is_open", False):
            if self.debug:
                now = time.time()
                if (now - self._last_debug_log) > 2.0:
                    self._last_debug_log = now
                    self.get_logger().warn(
                        f'Dropping PWM L={left_pwm} R={right_pwm} (serial not connected)'
                    )
            return
        now = time.time()
        if now < self._serial_ready_at:
            if self.debug and (now - self._last_boot_wait_log) > 1.0:
                self._last_boot_wait_log = now
                self.get_logger().info('Serial open, waiting for Arduino boot window')
            return
        try:
            line = f'M {left_pwm} {right_pwm}\n'
            with self._serial_lock:
                self.ser.write(line.encode('ascii'))
            if self.debug:
                if (now - self._last_tx_log) > 1.0 or (left_pwm, right_pwm) != self._last_tx:
                    self._last_tx_log = now
                    self._last_tx = (left_pwm, right_pwm)
                    self.get_logger().info(f'TX M {left_pwm} {right_pwm}')
        except Exception as exc:
            self.get_logger().error(f'Failed to write serial: {exc}')

    def _tick(self):
        # Stop if no commands for a while
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self._send_pwm(0, 0)

    def _debug_tick(self):
        if not self.debug:
            return
        now = time.time()
        if (now - self._last_debug_log) < 1.0:
            return
        self._last_debug_log = now
        v, w = self._last_cmd
        lp, rp = self._last_pwm
        serial_state = 'open' if (self.ser is not None and getattr(self.ser, "is_open", False)) else 'closed'
        self.get_logger().info(
            f'cmd v={v:.3f} w={w:.3f} -> pwm L={lp} R={rp} (serial {serial_state})'
        )

    def _read_loop(self):
        buffer = b''
        while rclpy.ok():
            try:
                if self.ser is None or not getattr(self.ser, "is_open", False):
                    time.sleep(0.05)
                    continue
                with self._serial_lock:
                    chunk = self.ser.read(128)
                if chunk:
                    self._empty_read_err_count = 0
                    self._empty_read_err_window_start = 0.0
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        self._handle_line(line.decode('ascii', errors='ignore').strip())
                else:
                    time.sleep(0.01)
            except Exception as exc:
                err = str(exc)
                now = time.time()
                # Common transient right after Arduino resets on USB open.
                # Avoid immediate close/reopen loops that delay motion.
                if (
                    'device reports readiness to read but returned no data' in err
                    and now < (self._serial_ready_at + 2.0)
                ):
                    if (now - self._last_transient_read_err_log) > 1.0:
                        self._last_transient_read_err_log = now
                        self.get_logger().warn(f'Transient serial read error during boot window: {err}')
                    time.sleep(0.05)
                    continue

                if 'device reports readiness to read but returned no data' in err:
                    # This CH340/USB backend warning can happen briefly even while the
                    # port is still usable. By default, keep the serial link open to
                    # avoid reset/reconnect loops that jitter odometry in RViz.
                    if not self.reconnect_on_empty_read_error:
                        if (now - self._last_transient_read_err_log) > 1.0:
                            self._last_transient_read_err_log = now
                            self.get_logger().warn(f'Transient serial read error: {err}')
                        time.sleep(0.05)
                        continue

                    # Optional fallback for users who want aggressive reconnecting.
                    if self._empty_read_err_window_start == 0.0 or (now - self._empty_read_err_window_start) > 3.0:
                        self._empty_read_err_window_start = now
                        self._empty_read_err_count = 0
                    self._empty_read_err_count += 1
                    if self._empty_read_err_count < 10:
                        if (now - self._last_transient_read_err_log) > 1.0:
                            self._last_transient_read_err_log = now
                            self.get_logger().warn(f'Transient serial read error: {err}')
                        time.sleep(0.05)
                        continue
                    self.get_logger().warn(
                        'Persistent serial empty-read errors detected; reconnecting serial port'
                    )

                self.get_logger().warn(f'Serial read error: {err}')
                with self._serial_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                self._connected = False
                self._empty_read_err_count = 0
                self._empty_read_err_window_start = 0.0
                time.sleep(0.2)

    def _handle_line(self, line: str):
        if not line:
            return
        if line.startswith('E '):
            parts = line.split()
            if len(parts) == 5:
                try:
                    raw_counts = [int(p) for p in parts[1:]]
                    counts = [
                        sign * count for sign, count in zip(self.encoder_signs, raw_counts)
                    ]
                    msg = Int64MultiArray()
                    msg.data = counts
                    self.encoder_pub.publish(msg)
                    self._update_odometry(counts)
                    if self.debug:
                        now = time.time()
                        if (now - self._last_encoder_log) > 1.0:
                            self._last_encoder_log = now
                            self.get_logger().info(
                                f'Arduino raw={raw_counts} corrected={counts}'
                            )
                except ValueError:
                    self.get_logger().warn(f'Bad encoder line: {line}')
        elif line.startswith('RX '):
            if self.debug:
                self.get_logger().info(f'Arduino {line}')
        else:
            if self.debug:
                self.get_logger().info(f'Arduino {line}')

    def _update_odometry(self, counts):
        now = time.time()
        if self._last_counts is None:
            self._last_counts = counts
            self._last_enc_time = now
            return

        dt = now - self._last_enc_time if self._last_enc_time else 0.0
        if dt <= 0.0:
            return

        deltas = [c - p for c, p in zip(counts, self._last_counts)]
        self._last_counts = counts
        self._last_enc_time = now

        # Convert counts to wheel rotation (rad) and distance (m)
        counts_per_rev = max(1, self.encoder_cpr)
        rad_per_count = (2.0 * math.pi) / counts_per_rev
        wheel_delta_rad = [d * rad_per_count for d in deltas]

        # Update wheel positions
        for i in range(4):
            self._wheel_pos[i] += wheel_delta_rad[i]

        # Average front/rear for each side
        left_rad = (wheel_delta_rad[0] + wheel_delta_rad[2]) / 2.0
        right_rad = (wheel_delta_rad[1] + wheel_delta_rad[3]) / 2.0
        dl = left_rad * self.wheel_radius
        dr = right_rad * self.wheel_radius

        # Differential drive integration
        ds = ((dl + dr) / 2.0) * self.odom_linear_scale
        dtheta = ((dr - dl) / max(1e-6, self.wheel_separation)) * self.odom_angular_scale

        yaw_mid = self._yaw + (dtheta / 2.0)
        self._x += ds * math.cos(yaw_mid)
        self._y += ds * math.sin(yaw_mid)
        self._yaw += dtheta

        # Publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        js.position = [
            self._wheel_pos[0],
            self._wheel_pos[1],
            self._wheel_pos[2],
            self._wheel_pos[3],
        ]
        if dt > 0:
            js.velocity = [
                wheel_delta_rad[0] / dt,
                wheel_delta_rad[1] / dt,
                wheel_delta_rad[2] / dt,
                wheel_delta_rad[3] / dt,
            ]
        self.joint_pub.publish(js)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = js.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self._x)
        odom.pose.pose.position.y = float(self._y)
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(ds / dt)
        odom.twist.twist.angular.z = float(dtheta / dt)
        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = float(self._x)
            t.transform.translation.y = float(self._y)
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


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
