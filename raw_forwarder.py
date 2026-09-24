"""Lightweight forwarder: reads MTF-01P over UART on the RPi and forwards
valid Micolink frames unchanged over UDP (raw bytes). Lets
calibration_tool.py run on another machine while tuning the filter on
live raw data (not just the processed velocity).

For production deployment (just the resulting velocity) use sensor_node.py.
"""
import argparse
import socket

import serial

from mtf01_link import MicolinkParser


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyAMA0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--udp-target', default='255.255.255.255:12346')
    args = ap.parse_args()

    ip, port = args.udp_target.rsplit(':', 1)
    port = int(port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if ip.endswith('255'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = MicolinkParser()
    print(f"Forwarding raw Micolink frames -> {ip}:{port}")

    n = 0
    try:
        while True:
            chunk = ser.read(64)
            for byte in chunk:
                if parser.feed(byte) is not None:
                    sock.sendto(parser.last_raw_packet, (ip, port))
                    n += 1
                    if n % 50 == 0:
                        print(f"\r{n} frames forwarded", end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()


if __name__ == '__main__':
    main()
