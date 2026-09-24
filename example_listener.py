"""Example of how to receive velocity from sensor_node.py anywhere on the network - just a socket + json.loads."""
import argparse
import json
import socket

ap = argparse.ArgumentParser()
ap.add_argument('--port', type=int, default=12345)
args = ap.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', args.port))
print(f"Listening on port {args.port}...")

while True:
    data, addr = sock.recvfrom(1024)
    msg = json.loads(data.decode().strip())
    print(f"Vx={msg['vx']:6.1f} Vy={msg['vy']:6.1f} cm/s | h={msg['h']:5.1f} cm | "
          f"static={msg['static']} | q={msg['q']}")
