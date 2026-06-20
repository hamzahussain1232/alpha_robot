#!/usr/bin/env python3
import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description='Simple serial self-test for Arduino drive.')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--left', type=int, default=150)
    parser.add_argument('--right', type=int, default=150)
    parser.add_argument('--duration', type=float, default=1.0)
    args = parser.parse_args()

    print(f'Opening {args.port} @ {args.baud}')
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    try:
        # Arduino auto-reset on serial open; give it time to boot.
        time.sleep(2.0)
        cmd = f'M {args.left} {args.right}\n'.encode('ascii')
        print(f'Sending: {cmd!r}')
        ser.write(cmd)

        end_time = time.time() + max(0.1, args.duration)
        saw_line = False
        while time.time() < end_time:
            line = ser.readline()
            if line:
                saw_line = True
                print(f'RX: {line.decode("ascii", errors="ignore").strip()}')
            else:
                time.sleep(0.01)

        ser.write(b'M 0 0\n')
        if not saw_line:
            print('No serial output seen. Firmware may not be running or baud/port is wrong.')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
