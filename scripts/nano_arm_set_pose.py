#!/usr/bin/env python3
import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description='Send one explicit pose to the Nano arm')
    parser.add_argument(
        '--port',
        default='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0',
    )
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--boot-wait', type=float, default=2.0)
    parser.add_argument('--servo', type=int, choices=range(1, 7))
    parser.add_argument('--deg', type=int)
    parser.add_argument(
        '--angles',
        nargs=6,
        type=int,
        metavar=('A1', 'A2', 'A3', 'A4', 'A5', 'A6'),
        help='Six servo angles in degrees (0..180)',
    )
    args = parser.parse_args()

    if args.angles is None and (args.servo is None or args.deg is None):
        parser.error('use either --angles A1 A2 A3 A4 A5 A6 or --servo N --deg X')
    if args.angles is not None and (args.servo is not None or args.deg is not None):
        parser.error('choose either --angles or --servo/--deg, not both')

    if args.angles is not None:
        angles = [max(0, min(180, value)) for value in args.angles]
        line = 'A ' + ' '.join(str(v) for v in angles) + '\n'
    else:
        line = f'S {args.servo} {max(0, min(180, int(args.deg)))}\n'

    ser = serial.Serial(args.port, args.baud, timeout=0.5, write_timeout=0.5)
    try:
        time.sleep(max(0.2, args.boot_wait))
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(line.encode('ascii'))
        ser.flush()
        reply = ''
        deadline = time.time() + 1.0
        while time.time() < deadline:
            candidate = ser.readline().decode('ascii', errors='ignore').strip()
            if not candidate:
                continue
            if candidate == 'ERR' or candidate.startswith(('NANO_ARM_READY', 'READY', 'OK', 'POS', 'HOME')):
                reply = candidate
                break
        print(f'Sent: {line.strip()}')
        print(f'Reply: {reply or "(no reply)"}')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
