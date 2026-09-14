"""Lehký forwarder: čte MTF-01P po UART na RPi a validní Micolink rámce
přeposílá beze změny přes UDP (raw bajty). Slouží k tomu, aby šlo
calibration_tool.py spouštět na jiném stroji a přitom ladit filtr na živých
syrových datech (ne až na zpracované rychlosti).

Pro provozní nasazení (jen výsledná rychlost) použij sensor_node.py.
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
    print(f"Přeposílám raw Micolink rámce -> {ip}:{port}")

    n = 0
    try:
        while True:
            chunk = ser.read(64)
            for byte in chunk:
                if parser.feed(byte) is not None:
                    sock.sendto(parser.last_raw_packet, (ip, port))
                    n += 1
                    if n % 50 == 0:
                        print(f"\r{n} rámců přeposláno", end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()


if __name__ == '__main__':
    main()
