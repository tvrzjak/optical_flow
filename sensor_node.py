"""Headless sensor node - reads MTF-01P over UART, filters it and sends
the result as JSON lines over UDP. Meant to run directly on the RPi next
to the sensor.

Usage:
    python3 sensor_node.py --udp-target 192.168.1.50:12345
    python3 sensor_node.py --udp-target 255.255.255.255:12345   # broadcast

UDP message format (one JSON object per line, newline-terminated):
    {"t":1699999999.123,"vx":0.0,"vy":0.0,"h":15.1,
     "flow_ok":true,"dist_ok":true,"static":true,"q":180,"calib":false}
"""
import argparse
import json
import os
import signal
import socket
import time

import serial

from mtf01_link import MicolinkParser
from estimator import Config, FlowEstimator
from calib_store import load_calibration, save_calibration

DEFAULT_CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.json')

running = True


def _sigint(sig, frame):
    global running
    running = False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyAMA0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--udp-target', default='127.0.0.1:12345', help='IP:PORT to send to, broadcast allowed')
    ap.add_argument('--height', type=float, default=None,
                     help='reference sensor height during calibration [cm]; '
                          'if omitted, the initial height is estimated purely from the sensor')
    ap.add_argument('--quality-min', type=int, default=30)
    ap.add_argument('--calib-file', default=DEFAULT_CALIB_FILE)
    ap.add_argument('--recalibrate', action='store_true', help='ignore the saved calibration and measure again')
    args = ap.parse_args()

    ip, port = args.udp_target.rsplit(':', 1)
    port = int(port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if ip.endswith('255'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    calibration = None if args.recalibrate else load_calibration(args.calib_file)
    cfg = Config(target_height_cm=args.height, quality_min=args.quality_min)
    est = FlowEstimator(cfg, calibration)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = MicolinkParser()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    if est.calibrating:
        if args.height is None:
            print("Calibrating at rest, initial height will be estimated from the sensor, hold the sensor still...")
        else:
            print(f"Calibrating at rest at height ~{args.height} cm, hold the sensor still...")
    last_print = 0.0

    while running:
        chunk = ser.read(64)
        if not chunk:
            continue
        for byte in chunk:
            frame = parser.feed(byte)
            if frame is None:
                continue
            est_out = est.update(frame)
            if est_out.calibrating:
                if time.time() - last_print > 0.5:
                    print(f"\rCalibrating: {est_out.calib_progress*100:5.1f}%", end='', flush=True)
                    last_print = time.time()
                if not est.calibrating:
                    print(f"\nDONE | dist_offset={est.dist_offset:.2f} cm  "
                          f"bias=({est.bias_x:.1f}, {est.bias_y:.1f})")
                continue

            msg = {
                't': est_out.t, 'vx': round(est_out.vx, 2), 'vy': round(est_out.vy, 2),
                'h': round(est_out.height_cm, 2), 'flow_ok': est_out.flow_ok,
                'dist_ok': est_out.dist_ok, 'static': est_out.stationary,
                'q': est_out.flow_quality, 'calib': False,
            }
            try:
                sock.sendto((json.dumps(msg) + '\n').encode(), (ip, port))
            except OSError as e:
                print('UDP send error:', e)

            if time.time() - last_print > 0.5:
                print(f"\rVx={est_out.vx:6.1f} Vy={est_out.vy:6.1f} cm/s | H={est_out.height_cm:5.1f} cm "
                      f"| q={est_out.flow_quality:3d} | static={est_out.stationary}   ", end='', flush=True)
                last_print = time.time()

    print("\nShutting down, saving calibration...")
    save_calibration(args.calib_file, est.export_calibration())
    ser.close()
    sock.close()


if __name__ == '__main__':
    main()
