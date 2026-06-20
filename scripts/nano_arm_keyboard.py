#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import serial


INCREASE_KEYS = {
    '1': 0,
    '2': 1,
    '3': 2,
    '4': 3,
    '5': 4,
    '6': 5,
}

DECREASE_KEYS = {
    'q': 0,
    'w': 1,
    'e': 2,
    'r': 3,
    't': 4,
    'y': 5,
}


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


class NanoArmKeyboard:
    def __init__(self, port: str, baud: int, boot_wait: float, step_deg: int):
        self.port = port
        self.baud = baud
        self.boot_wait = boot_wait
        self.step_deg = max(1, step_deg)
        self.angles = [90, 90, 90, 90, 90, 90]
        self.last_action = 'Connecting'
        self.ser = open_serial_port(self.port, self.baud, timeout=0.2, write_timeout=0.2)
        time.sleep(max(0.2, self.boot_wait))
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        reply = self.send_raw('P')
        if reply:
            self.last_action = f'Connected: {reply}'
        else:
            self.last_action = 'Connected, but no reply yet'

    def send_raw(self, command: str):
        self.ser.write((command.strip() + '\n').encode('ascii'))
        self.ser.flush()
        deadline = time.time() + 1.0
        while time.time() < deadline:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            parsed = self.parse_reply(line)
            if parsed is not None:
                prefix, values = parsed
                self.angles = values
                return f'{prefix} {" ".join(str(v) for v in values)}'
            if self.is_diagnostic_line(line):
                return line
        return ''

    @staticmethod
    def parse_reply(line: str):
        parts = line.split()
        if len(parts) != 7:
            return None
        if parts[0] not in ('NANO_ARM_READY', 'READY', 'OK', 'POS', 'HOME'):
            return None
        try:
            values = [max(0, min(180, int(v))) for v in parts[1:]]
        except ValueError:
            return None
        return parts[0], values

    @staticmethod
    def is_diagnostic_line(line: str) -> bool:
        return line in ('ERR',)

    def send_angles(self):
        cmd = 'A ' + ' '.join(str(v) for v in self.angles)
        reply = self.send_raw(cmd)
        self.last_action = reply or f'Sent: {cmd}'

    def send_single_servo(self, idx: int):
        cmd = f'S {idx + 1} {self.angles[idx]}'
        reply = self.send_raw(cmd)
        self.last_action = reply or f'Sent: {cmd}'

    def home(self):
        reply = self.send_raw('H')
        self.last_action = reply or 'Sent home command'

    def poll(self):
        reply = self.send_raw('P')
        self.last_action = reply or 'Polled current angles'

    def adjust_joint(self, idx: int, delta: int):
        self.angles[idx] = max(0, min(180, self.angles[idx] + delta))
        self.send_single_servo(idx)

    def change_step(self, scale: float):
        self.step_deg = max(1, min(30, int(round(self.step_deg * scale))))
        self.last_action = f'Step size set to {self.step_deg} deg'

    def render(self):
        lines = []
        lines.append('\033[2J\033[H')
        lines.append('Nano Arm Direct Keyboard Test')
        lines.append('')
        lines.append(f'Port: {self.port}    Baud: {self.baud}')
        lines.append(f'Step: {self.step_deg} deg')
        lines.append(f'Last action: {self.last_action}')
        lines.append('')
        lines.append('Controls:')
        lines.append('  1/2/3/4/5/6  increase servo 1..6')
        lines.append('  q/w/e/r/t/y  decrease servo 1..6')
        lines.append('  -            smaller step')
        lines.append('  = or +       bigger step')
        lines.append('  h            home all servos')
        lines.append('  p            read current angles from Nano')
        lines.append('  Ctrl+C       quit')
        lines.append('')
        lines.append('Servo  Angle(deg)')
        for idx, angle in enumerate(self.angles, start=1):
            lines.append(f'  {idx}      {angle:>3}')
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()

    def close(self):
        try:
            self.ser.close()
        except BaseException:
            pass


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


def main():
    parser = argparse.ArgumentParser(description='Direct keyboard test tool for Nano arm servos')
    parser.add_argument(
        '--port',
        default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
    )
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--boot-wait', type=float, default=2.0)
    parser.add_argument('--step-deg', type=int, default=15)
    args = parser.parse_args()

    tester = NanoArmKeyboard(
        port=args.port,
        baud=args.baud,
        boot_wait=args.boot_wait,
        step_deg=args.step_deg,
    )
    tester.render()

    try:
        while True:
            key = get_key(0.15)
            if key in INCREASE_KEYS:
                tester.adjust_joint(INCREASE_KEYS[key], tester.step_deg)
                tester.render()
            elif key in DECREASE_KEYS:
                tester.adjust_joint(DECREASE_KEYS[key], -tester.step_deg)
                tester.render()
            elif key in ('-', '_'):
                tester.change_step(0.5)
                tester.render()
            elif key in ('=', '+'):
                tester.change_step(2.0)
                tester.render()
            elif key in ('h', 'H'):
                tester.home()
                tester.render()
            elif key in ('p', 'P'):
                tester.poll()
                tester.render()
            elif key == '\x03':
                break
            else:
                tester.render()
    finally:
        tester.close()


if __name__ == '__main__':
    main()
