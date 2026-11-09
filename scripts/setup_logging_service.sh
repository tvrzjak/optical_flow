#!/usr/bin/env bash
# Logování přes síťový TAP: systemd služba "of-forward" po startu RPi sama
# spustí raw_forwarder.py a posílá syrové Micolink rámce senzoru jako
# broadcast (výchozí 192.168.20.255:12346 - broadcast podsítě; bez výchozí
# brány je 255.255.255.255 "network unreachable"). Rámce zachytí TAP s HW časem
# a rozparsuje je velocity_stack/scripts/tap_capture.py (typ "optical_flow").
#
# Broadcast nepotřebuje žádného příjemce ani ARP, takže RPi posílá i zapojené
# jen do portu TAPu, za kterým nic není (TAP jen odposlouchává).
#
# UART smí mít otevřený jen jeden proces - pro kalibraci / sensor_node je
# potřeba službu zastavit. Proto se účtu telnet menu (mtfop) povolí přes
# sudoers jen start/stop/restart/status této jedné služby (příkaz
# "logging start|stop|status" v mtf_shell.py).
#
# Spustit JEDNOU jako root:
#   sudo bash scripts/setup_logging_service.sh [IP:PORT]
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Spusť jako root: sudo bash $0 [IP:PORT]" >&2
    exit 1
fi

TARGET="${1:-192.168.20.255:12346}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_USER="$(stat -c %U "$PROJECT_DIR")"
SERVICE=of-forward
UNIT="/etc/systemd/system/$SERVICE.service"
SUDOERS="/etc/sudoers.d/mtfop-$SERVICE"
SYSTEMCTL="$(command -v systemctl)"

echo "== Služba $SERVICE: $PROJECT_DIR/raw_forwarder.py -> $TARGET (uživatel $RUN_USER) =="
cat > "$UNIT" <<EOF
[Unit]
Description=MTF-01P raw Micolink frames -> UDP $TARGET (logging through the network TAP)
After=network-online.target
Wants=network-online.target

[Service]
User=$RUN_USER
Group=dialout
WorkingDirectory=$PROJECT_DIR
ExecStart=/usr/bin/python3 -u $PROJECT_DIR/raw_forwarder.py --udp-target $TARGET --quiet
Restart=always
RestartSec=2
Nice=-5

[Install]
WantedBy=multi-user.target
EOF

echo "== sudoers: mtfop smí jen start/stop/restart/status $SERVICE =="
cat > "$SUDOERS.tmp" <<EOF
mtfop ALL=(root) NOPASSWD: $SYSTEMCTL start $SERVICE.service, $SYSTEMCTL stop $SERVICE.service, $SYSTEMCTL restart $SERVICE.service, $SYSTEMCTL status $SERVICE.service
EOF
visudo -cf "$SUDOERS.tmp"
chmod 0440 "$SUDOERS.tmp"
mv "$SUDOERS.tmp" "$SUDOERS"

echo "== NetworkManager: eth0 drží IP a routu i bez linky (RPi zapojené jen do TAPu) =="
cat > /etc/NetworkManager/conf.d/90-eth0-ignore-carrier.conf <<EOF
# Logování přes TAP (RPi -> port TAPu, za ním nic): eth0 si drží statickou IP
# a routu i bez carrieru, of-forward pak posílá broadcast, kdykoli je linka.
[device-eth0-ignore-carrier]
match-device=interface-name:eth0
ignore-carrier=yes
EOF
nmcli general reload conf || true

systemctl daemon-reload
systemctl enable "$SERVICE.service"
systemctl restart "$SERVICE.service"
sleep 2
systemctl --no-pager --lines=5 status "$SERVICE.service" || true
