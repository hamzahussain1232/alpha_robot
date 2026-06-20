#!/usr/bin/env python3
import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path

import serial
import yaml


JOINT_LABELS = [
    '1 base',
    '2 shoulder',
    '3 elbow',
    '4 wrist_pitch',
    '5 wrist_roll',
    '6 gripper',
]

VALID_PREFIXES = ('NANO_ARM_READY', 'READY', 'OK', 'POS', 'HOME')


def open_serial_port(port: str, baud: int, timeout: float, write_timeout: float):
    try:
        return serial.Serial(
            port,
            baud,
            timeout=timeout,
            write_timeout=write_timeout,
            exclusive=True,
        )
    except TypeError:
        return serial.Serial(
            port,
            baud,
            timeout=timeout,
            write_timeout=write_timeout,
        )


def find_default_profile_path() -> Path:
    candidates = [
        Path.cwd() / 'src' / 'articubot_one' / 'config' / 'arm_nano_calibration.yaml',
        Path.home() / 'ros2_ws' / 'src' / 'articubot_one' / 'config' / 'arm_nano_calibration.yaml',
        Path(__file__).resolve().parents[2] / 'share' / 'articubot_one' / 'config' / 'arm_nano_calibration.yaml',
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[1]


def default_profile():
    return {
        'joint_min_rad': [-1.57] * 6,
        'joint_max_rad': [1.57] * 6,
        'servo_home_deg': [90.0] * 6,
        'servo_direction': [1] * 6,
        'servo_min_deg': [0.0] * 6,
        'servo_max_deg': [180.0] * 6,
        'startup_positions_rad': [0.0] * 6,
        'gripper_joint_index': 5,
        'gripper_open_deg': 90.0,
        'gripper_close_deg': 180.0,
    }


def load_profile(path: Path):
    profile = default_profile()
    if not path.exists():
        return profile
    with path.open('r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    params = raw.get('nano_arm_driver', {}).get('ros__parameters', {})
    for key, default_value in profile.items():
        if key not in params:
            continue
        if isinstance(default_value, list):
            profile[key] = [float(v) if isinstance(default_value[0], float) else int(v) for v in params[key]]
        elif isinstance(default_value, float):
            profile[key] = float(params[key])
        else:
            profile[key] = int(params[key])
    return profile


def save_profile(path: Path, profile):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        'nano_arm_driver': {
            'ros__parameters': {
                'joint_min_rad': [round(v, 4) for v in profile['joint_min_rad']],
                'joint_max_rad': [round(v, 4) for v in profile['joint_max_rad']],
                'servo_home_deg': [round(v, 1) for v in profile['servo_home_deg']],
                'servo_direction': [int(v) for v in profile['servo_direction']],
                'servo_min_deg': [round(v, 1) for v in profile['servo_min_deg']],
                'servo_max_deg': [round(v, 1) for v in profile['servo_max_deg']],
                'startup_positions_rad': [round(v, 4) for v in profile['startup_positions_rad']],
                'gripper_joint_index': int(profile['gripper_joint_index']),
                'gripper_open_deg': round(profile['gripper_open_deg'], 1),
                'gripper_close_deg': round(profile['gripper_close_deg'], 1),
            }
        }
    }
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, sort_keys=False)


def parse_reply(line: str):
    parts = line.split()
    if len(parts) != 7 or parts[0] not in VALID_PREFIXES:
        return None
    try:
        return parts[0], [max(0, min(180, int(v))) for v in parts[1:]]
    except ValueError:
        return None


def get_key(timeout: float):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class ArmCalibrationUI:
    def __init__(self, port: str, baud: int, boot_wait: float, profile_path: Path, step_deg: int):
        self.port = port
        self.baud = baud
        self.profile_path = profile_path
        self.profile = load_profile(profile_path)
        self.step_deg = max(1, step_deg)
        self.selected = 0
        self.angles = [int(round(v)) for v in self.profile['servo_home_deg']]
        self.last_action = f'Loaded profile: {profile_path}'
        self.ser = open_serial_port(self.port, self.baud, timeout=0.2, write_timeout=0.2)
        time.sleep(max(0.2, boot_wait))
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.poll()

    def send_raw(self, command: str):
        self.ser.write((command.strip() + '\n').encode('ascii'))
        self.ser.flush()
        deadline = time.time() + 1.0
        while time.time() < deadline:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            parsed = parse_reply(line)
            if parsed is None:
                if line == 'ERR':
                    return line
                continue
            prefix, values = parsed
            self.angles = values
            return f'{prefix} {" ".join(str(v) for v in values)}'
        return ''

    def clamp_for_selected(self, value: int) -> int:
        servo_min = int(round(self.profile['servo_min_deg'][self.selected]))
        servo_max = int(round(self.profile['servo_max_deg'][self.selected]))
        return max(servo_min, min(servo_max, value))

    def move_selected(self, delta: int):
        target = self.clamp_for_selected(self.angles[self.selected] + delta)
        self.angles[self.selected] = target
        reply = self.send_raw(f'S {self.selected + 1} {target}')
        self.last_action = reply or f'Sent servo {self.selected + 1} -> {target}'

    def send_selected_home(self):
        target = int(round(self.profile['servo_home_deg'][self.selected]))
        self.angles[self.selected] = self.clamp_for_selected(target)
        reply = self.send_raw(f'S {self.selected + 1} {self.angles[self.selected]}')
        self.last_action = reply or f'Moved servo {self.selected + 1} to home'

    def poll(self):
        reply = self.send_raw('P')
        self.last_action = reply or 'Polled Nano arm state'

    def home_all(self):
        targets = [self.clamp_saved(i, self.profile['servo_home_deg'][i]) for i in range(6)]
        reply = self.send_raw('A ' + ' '.join(str(v) for v in targets))
        self.last_action = reply or 'Moved all servos to home'

    def clamp_saved(self, idx: int, value: float) -> int:
        return max(
            int(round(self.profile['servo_min_deg'][idx])),
            min(int(round(self.profile['servo_max_deg'][idx])), int(round(value))),
        )

    def set_min(self):
        current = float(self.angles[self.selected])
        self.profile['servo_min_deg'][self.selected] = current
        if self.profile['servo_home_deg'][self.selected] < current:
            self.profile['servo_home_deg'][self.selected] = current
        if self.profile['servo_max_deg'][self.selected] < current:
            self.profile['servo_max_deg'][self.selected] = current
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Set servo {self.selected + 1} min to {int(current)} deg'

    def set_home(self):
        current = float(self.angles[self.selected])
        self.profile['servo_home_deg'][self.selected] = current
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Set servo {self.selected + 1} home to {int(current)} deg'

    def set_max(self):
        current = float(self.angles[self.selected])
        self.profile['servo_max_deg'][self.selected] = current
        if self.profile['servo_home_deg'][self.selected] > current:
            self.profile['servo_home_deg'][self.selected] = current
        if self.profile['servo_min_deg'][self.selected] > current:
            self.profile['servo_min_deg'][self.selected] = current
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Set servo {self.selected + 1} max to {int(current)} deg'

    def set_direction(self, direction: int):
        self.profile['servo_direction'][self.selected] = direction
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Set servo {self.selected + 1} direction to {direction:+d}'

    def set_gripper_open(self):
        if self.selected != self.profile['gripper_joint_index']:
            self.last_action = 'Select the gripper servo first to save gripper open angle'
            return
        self.profile['gripper_open_deg'] = float(self.angles[self.selected])
        self.last_action = f'Set gripper open angle to {self.angles[self.selected]} deg'

    def set_gripper_close(self):
        if self.selected != self.profile['gripper_joint_index']:
            self.last_action = 'Select the gripper servo first to save gripper close angle'
            return
        current = float(self.angles[self.selected])
        self.profile['gripper_close_deg'] = current
        self.profile['servo_max_deg'][self.selected] = min(
            self.profile['servo_max_deg'][self.selected],
            current,
        )
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Set gripper close safety limit to {int(current)} deg'

    def _refresh_joint_limits(self, idx: int):
        home = float(self.profile['servo_home_deg'][idx])
        direction = int(self.profile['servo_direction'][idx])
        servo_min = float(self.profile['servo_min_deg'][idx])
        servo_max = float(self.profile['servo_max_deg'][idx])
        low = (servo_min - home) / direction
        high = (servo_max - home) / direction
        joint_low_deg = min(low, high)
        joint_high_deg = max(low, high)
        self.profile['joint_min_rad'][idx] = math.radians(joint_low_deg)
        self.profile['joint_max_rad'][idx] = math.radians(joint_high_deg)
        self.profile['startup_positions_rad'][idx] = 0.0

    def save(self):
        for idx in range(6):
            self._refresh_joint_limits(idx)
        save_profile(self.profile_path, self.profile)
        self.last_action = f'Saved calibration to {self.profile_path}'

    def reset_selected_limits(self):
        self.profile['servo_min_deg'][self.selected] = 0.0
        self.profile['servo_max_deg'][self.selected] = 180.0
        self.profile['servo_home_deg'][self.selected] = 90.0
        self.profile['servo_direction'][self.selected] = 1
        self._refresh_joint_limits(self.selected)
        self.last_action = f'Reset servo {self.selected + 1} to default limits'

    def render(self):
        lines = [
            '\033[2J\033[H',
            'Arm Calibration Console',
            '',
            f'Port: {self.port}    Baud: {self.baud}    Step: {self.step_deg} deg',
            f'Profile: {self.profile_path}',
            f'Last action: {self.last_action}',
            '',
            'Workflow:',
            '  1. Select a servo with 1..6',
            '  2. Move with a/d inside the safe range',
            '  3. Save min with m, home with h, max with x',
            '  4. For gripper, save open with o and safe close with c',
            '  5. Press s to write the calibration profile',
            '',
            'Controls:',
            '  1..6 select servo     a/d move selected servo',
            '  - / + change step     p poll angles from Nano',
            '  m set min             h set home             x set max',
            '  n direction = +1      v direction = -1',
            '  o gripper open        c gripper close safety',
            '  u move selected home  H move all homes',
            '  R reset selected      s save profile         Ctrl+C quit',
            '',
            'Servo                 cur   min  home  max  dir',
        ]

        for idx in range(6):
            marker = '>' if idx == self.selected else ' '
            lines.append(
                f'{marker} {JOINT_LABELS[idx]:<18} '
                f'{self.angles[idx]:>3}   '
                f'{int(round(self.profile["servo_min_deg"][idx])):>3}   '
                f'{int(round(self.profile["servo_home_deg"][idx])):>3}   '
                f'{int(round(self.profile["servo_max_deg"][idx])):>3}   '
                f'{int(self.profile["servo_direction"][idx]):>+2}'
            )

        lines.extend([
            '',
            f'Gripper servo: {self.profile["gripper_joint_index"] + 1}    '
            f'open: {int(round(self.profile["gripper_open_deg"]))} deg    '
            f'close safety: {int(round(self.profile["gripper_close_deg"]))} deg',
            '',
            'Recommendation:',
            '  Leave 5-10 deg margin away from any hard stop.',
            '  For the gripper, store a close angle that holds the object without buzzing.',
        ])
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()

    def close(self):
        try:
            self.ser.close()
        except BaseException:
            pass


def main():
    parser = argparse.ArgumentParser(description='Interactive calibration tool for the Nano arm')
    parser.add_argument(
        '--port',
        default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
    )
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--boot-wait', type=float, default=2.0)
    parser.add_argument('--step-deg', type=int, default=5)
    parser.add_argument('--profile', type=Path, default=find_default_profile_path())
    args = parser.parse_args()

    ui = ArmCalibrationUI(
        port=args.port,
        baud=args.baud,
        boot_wait=args.boot_wait,
        profile_path=args.profile,
        step_deg=args.step_deg,
    )
    ui.render()

    try:
        while True:
            key = get_key(0.15)
            if key in ('1', '2', '3', '4', '5', '6'):
                ui.selected = int(key) - 1
            elif key in ('a', 'A'):
                ui.move_selected(-ui.step_deg)
            elif key in ('d', 'D'):
                ui.move_selected(ui.step_deg)
            elif key in ('-', '_'):
                ui.step_deg = max(1, ui.step_deg - 1)
                ui.last_action = f'Step size set to {ui.step_deg} deg'
            elif key in ('=', '+'):
                ui.step_deg = min(30, ui.step_deg + 1)
                ui.last_action = f'Step size set to {ui.step_deg} deg'
            elif key in ('p', 'P'):
                ui.poll()
            elif key == 'm':
                ui.set_min()
            elif key in ('h',):
                ui.set_home()
            elif key in ('x', 'X'):
                ui.set_max()
            elif key in ('n', 'N'):
                ui.set_direction(1)
            elif key in ('v', 'V'):
                ui.set_direction(-1)
            elif key in ('o', 'O'):
                ui.set_gripper_open()
            elif key in ('c', 'C'):
                ui.set_gripper_close()
            elif key in ('u', 'U'):
                ui.send_selected_home()
            elif key == 'H':
                ui.home_all()
            elif key in ('r', 'R'):
                ui.reset_selected_limits()
            elif key in ('s', 'S'):
                ui.save()
            elif key == '\x03':
                break
            ui.render()
    finally:
        ui.close()


if __name__ == '__main__':
    main()
