#!/usr/bin/env python3
import argparse
import time

import serial


VALID_PREFIXES = ('NANO_ARM_READY', 'READY', 'OK', 'POS', 'HOME')


def read_reply(ser, timeout_sec: float = 1.0) -> str:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        candidate = ser.readline().decode('ascii', errors='ignore').strip()
        if not candidate:
            continue
        if candidate == 'ERR' or candidate.startswith(VALID_PREFIXES):
            return candidate
    return ''


def send_command(ser, command: str) -> str:
    ser.write((command.strip() + '\n').encode('ascii'))
    ser.flush()
    return read_reply(ser)


def main():
    parser = argparse.ArgumentParser(
        description='Directly sweep one Nano arm servo left and right for a fixed time'
    )
    parser.add_argument(
        '--port',
        default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
    )
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--boot-wait', type=float, default=2.0)
    parser.add_argument('--servo', type=int, default=1, choices=range(1, 7))
    parser.add_argument('--left-deg', type=int, default=30)
    parser.add_argument('--right-deg', type=int, default=150)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument(
        '--hold-sec',
        type=float,
        default=1.0,
        help='How long to hold each side before switching',
    )
    parser.add_argument(
        '--center-at-end',
        action='store_true',
        help='Return the servo to 90 degrees after the sweep',
    )
    args = parser.parse_args()

    left_deg = max(0, min(180, args.left_deg))
    right_deg = max(0, min(180, args.right_deg))

    with serial.Serial(args.port, args.baud, timeout=0.5, write_timeout=0.5) as ser:
        time.sleep(max(0.2, args.boot_wait))
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        startup_reply = send_command(ser, 'P')
        if startup_reply:
            print(f'Start: {startup_reply}')

        end_time = time.time() + max(0.1, args.duration)
        target = left_deg
        while time.time() < end_time:
            command = f'S {args.servo} {target}'
            reply = send_command(ser, command)
            print(f'Sent: {command}')
            print(f'Reply: {reply or "(no reply)"}')
            time.sleep(max(0.05, args.hold_sec))
            target = right_deg if target == left_deg else left_deg

        if args.center_at_end:
            command = f'S {args.servo} 90'
            reply = send_command(ser, command)
            print(f'Sent: {command}')
            print(f'Reply: {reply or "(no reply)"}')


if __name__ == '__main__':
    main()
