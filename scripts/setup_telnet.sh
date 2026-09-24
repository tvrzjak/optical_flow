#!/usr/bin/env bash
# Zapne telnet přístup na RPi hned po startu (systemd socket activation):
# po připojení RPi k PC stačí "telnet <adresa-rpi>" a hned je k dispozici
# menu nástrojů z docs/MTF01P_manual.md (kap. 12) - BEZ login/heslo výzvy.
#
# Jak to funguje: telnetd (-E/--exec-login) místo /bin/login spustí náš
# pomocný skript (root-drop wrapper). Ten běží (nutně) jako root - jen proto,
# aby vůbec mohl systémově "shodit" práva - a OKAMŽITĚ, dřív než
# cokoliv přečte od klienta, přepne na neprivilegovaný účet "mtfop"
# (setuid/setgid) a teprve pak spustí mtf_shell.py. Root se tedy nikdy
# nedostane k žádnému vstupu od klienta ani ke spuštění senzorových
# nástrojů.
#
# Účet "mtfop" nemá (a nesmí mít) žádné heslo - přístup k němu jde
# záměrně jen tudy, ne přes SSH/konzoli/su na dálku.
#
# BEZPEČNOSTNÍ DŮSLEDEK: kdokoliv, kdo se po síti dostane na port 23
# tohoto RPi, může bez ověření spouštět nástroje z mtf_shell.py (forward/
# sensor/listen/calib) s libovolnými síťovými cíli. Použij jen na
# důvěryhodném přímém/izolovaném spojení RPi-PC, nikdy ve sdílené síti
# ani přes internet. Pro plný (bash) vzdálený přístup použij SSH
# (heslem chráněné, `ssh of@<hostname-rpi>.local`), telnet už bash
# nenabízí.
#
# Spustit JEDNOU jako root:
#   sudo bash scripts/setup_telnet.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Spusť jako root: sudo bash $0" >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MTF_SHELL="$PROJECT_DIR/mtf_shell.py"
AUTOLOGIN_HELPER="/usr/local/sbin/mtf-telnet-autologin"
TELNET_USER="mtfop"

echo "== Instaluji telnet server (inetutils-telnetd) =="
apt-get update -qq
apt-get install -y inetutils-telnetd

echo "== Připravuji neprivilegovaný účet '$TELNET_USER' =="
chmod +x "$MTF_SHELL"
grep -qxF "$MTF_SHELL" /etc/shells || echo "$MTF_SHELL" >> /etc/shells

PROJECT_OWNER="$(stat -c '%U' "$PROJECT_DIR")"
if id "$TELNET_USER" &>/dev/null; then
    usermod -s "$MTF_SHELL" -G "dialout,$PROJECT_OWNER" "$TELNET_USER"
else
    useradd -m -s "$MTF_SHELL" -G "dialout,$PROJECT_OWNER" "$TELNET_USER"
fi
# Žádné heslo - přístup jen přes telnet_autologin.py (root helper níže),
# ne přímým přihlášením (ssh/su/konzole) tímto účtem.
passwd -l "$TELNET_USER" >/dev/null

echo "== Zapisuji root-drop wrapper $AUTOLOGIN_HELPER =="
cat > "$AUTOLOGIN_HELPER" <<EOF
#!/usr/bin/env python3
"""Spouští telnetd místo /bin/login (viz scripts/setup_telnet.sh) - BEZ
ověření hesla. Jediný účel: přepnout z root (pod kterým telnetd tento
skript spustí) na neprivilegovaný účet '$TELNET_USER' a rovnou předat
řízení mtf_shell.py, DŘÍV než se cokoliv od klienta zpracuje.
"""
import os
import pwd
import sys

TARGET_USER = '$TELNET_USER'
SHELL_PATH = '$MTF_SHELL'


def main():
    pw = pwd.getpwnam(TARGET_USER)
    if os.getuid() == 0:
        os.initgroups(TARGET_USER, pw.pw_gid)
        os.setgid(pw.pw_gid)
        os.setuid(pw.pw_uid)
    os.environ['HOME'] = pw.pw_dir
    os.environ['USER'] = TARGET_USER
    os.environ['LOGNAME'] = TARGET_USER
    os.chdir(pw.pw_dir)
    os.execv(sys.executable, [sys.executable, SHELL_PATH])


if __name__ == '__main__':
    main()
EOF
chmod 755 "$AUTOLOGIN_HELPER"
chown root:root "$AUTOLOGIN_HELPER"

echo "== Zapisuji systemd jednotky telnet.socket / telnet@.service =="
cat > /etc/systemd/system/telnet.socket <<'EOF'
[Unit]
Description=Telnet Server Socket (MTF-01P vzdalene spousteni nastroju)

[Socket]
ListenStream=23
Accept=yes

[Install]
WantedBy=sockets.target
EOF

cat > /etc/systemd/system/telnet@.service <<EOF
[Unit]
Description=Telnet Server Connection Handler (auto-login do mtf_shell.py)

[Service]
ExecStart=/usr/sbin/telnetd -h -E $AUTOLOGIN_HELPER
StandardInput=socket
StandardOutput=socket
StandardError=socket
EOF

echo "== Aktivuji socket =="
systemctl daemon-reload
systemctl enable --now telnet.socket

echo
echo "Hotovo. Telnet naslouchá od dalšího startu automaticky (i po rebootu)."
echo "Z PC: telnet $(hostname).local   (nebo telnet <IP adresa RPi>)"
echo "-> žádná výzva login/heslo, rovnou 'mtf>' menu."
echo
echo "Plný shell vzdáleně: ssh $PROJECT_OWNER@$(hostname).local (heslem chráněné)."
systemctl status telnet.socket --no-pager || true
