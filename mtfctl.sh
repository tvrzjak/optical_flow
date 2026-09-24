#!/usr/bin/env bash
# Jednotné spouštění nástrojů MTF-01P - určeno pro rychlé použití po
# přihlášení přes telnet (viz docs/MTF01P_manual.md, kap. 12).
#
# Použití:
#   ./mtfctl.sh sensor  [--udp-target IP:PORT] [--height CM] [další parametry sensor_node.py]
#   ./mtfctl.sh calib   [--height CM] [další parametry calibration_tool.py]
#   ./mtfctl.sh calib-remote --udp-raw PORT
#   ./mtfctl.sh forward [--udp-target IP:PORT]
#   ./mtfctl.sh listen  [--port PORT]
#
# Bez --height se počáteční výška odhadne čistě ze senzoru (viz kap. 6).
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=python3

cmd="${1:-}"
[ $# -gt 0 ] && shift

case "$cmd" in
    sensor)
        exec "$PY" "$DIR/sensor_node.py" "$@" ;;
    calib)
        exec "$PY" "$DIR/calibration_tool.py" --serial /dev/ttyAMA0 "$@" ;;
    calib-remote)
        exec "$PY" "$DIR/calibration_tool.py" "$@" ;;
    forward)
        exec "$PY" "$DIR/raw_forwarder.py" "$@" ;;
    listen)
        exec "$PY" "$DIR/example_listener.py" "$@" ;;
    *)
        cat <<'EOF'
MTF-01P - nástroje (podrobnosti viz docs/MTF01P_manual.md, kap. 8)

  mtfctl.sh sensor       [--udp-target IP:PORT] [--height CM] ...
      provozní odesílání rychlosti přes UDP (sensor_node.py)

  mtfctl.sh calib        [--height CM] ...
      lokální kalibrace/diagnostika, senzor přímo na RPi (calibration_tool.py --serial)

  mtfctl.sh calib-remote --udp-raw PORT
      kalibrace/diagnostika na jiném stroji ze surových dat raw_forwarder.py

  mtfctl.sh forward      [--udp-target IP:PORT]
      přeposílání syrových Micolink rámců přes UDP (raw_forwarder.py)

  mtfctl.sh listen       [--port PORT]
      testovací příjem výsledné rychlosti přes UDP (example_listener.py)

Bez parametru --height se počáteční výška odhadne čistě ze senzoru.
EOF
        exit 1 ;;
esac
