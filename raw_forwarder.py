"""Lightweight forwarder: reads MTF-01P over UART on the RPi and forwards
valid Micolink frames unchanged over UDP (raw bytes). Lets
calibration_tool.py run on another machine while tuning the filter on
live raw data (not just the processed velocity).

For production deployment (just the resulting velocity) use sensor_node.py.

Also the source for logging through the network TAP: the systemd service
of-forward (scripts/setup_logging_service.sh) runs it at boot as
    raw_forwarder.py --udp-target 192.168.20.255:12346 --quiet
(subnet broadcast: without a default gateway 255.255.255.255 is "network unreachable")
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
    ap.add_argument('--quiet', action='store_true', help='no frame counter (systemd service)')
    args = ap.parse_args()

    ip, port = args.udp_target.rsplit(':', 1)
    port = int(port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if ip.endswith('255'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = MicolinkParser()
    print(f"Forwarding raw Micolink frames -> {ip}:{port}")

    n = errors = 0
    try:
        while True:
            # whatever is already buffered, else block for 1 byte: a frame goes out as soon as its
            # last byte arrives (read(64) waited for ~2.5 frames → bursts and up to tens of ms delay)
            chunk = ser.read(ser.in_waiting or 1)
            for byte in chunk:
                if parser.feed(byte) is not None:
                    try:
                        sock.sendto(parser.last_raw_packet, (ip, port))
                    except OSError as e:          # link down / no route yet (boot) - keep reading
                        errors += 1
                        if errors in (1, 100) or errors % 10000 == 0:
                            print(f"UDP send error #{errors}: {e}", flush=True)
                        continue
                    n += 1
                    if n % 50 == 0 and not args.quiet:
                        print(f"\r{n} frames forwarded", end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()


if __name__ == '__main__':
    main()
