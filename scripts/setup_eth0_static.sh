#!/usr/bin/env bash
# Nastaví na eth0 pevnou statickou IP adresu, která PŘEŽIJE reboot -
# bez závislosti na DHCP (na přímém spojení RPi-PC žádný DHCP server
# není, takže "ipv4.method: auto" po startu čeká ~45 s na DHCP, selže
# a rozhraní zůstane "disconnected", dokud ho někdo ručně neaktivuje).
#
# Použití (jako root):
#   sudo bash scripts/setup_eth0_static.sh [IP/PREFIX]
#   sudo bash scripts/setup_eth0_static.sh 192.168.60.101/24    # výchozí
#
# Pozn.: nmcli úpravu existujícího profilu "netplan-eth0" (vytvořeného
# GUI/netplan integrací) nešlo spolehlivě přepnout z auto na manual -
# změna se po chvíli sama vracela zpět na "auto" (pravděpodobně kvůli
# souhře netplan<->NetworkManager). Řešení: založit nový, čistý profil
# "eth0-static" a ten starý jen vypnout (autoconnect no), aby si
# nekonkurovaly o stejné rozhraní.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Spusť jako root: sudo bash $0 [IP/PREFIX]" >&2
    exit 1
fi

ADDR="${1:-192.168.60.101/24}"
IFACE="eth0"
NEW_CON="eth0-static"
OLD_CON="netplan-eth0"

echo "== Ruším staré konkurující profily pro $IFACE (autoconnect no) =="
for c in $(nmcli -t -f NAME,DEVICE connection show | awk -F: -v d="$IFACE" '$2==d{print $1}'); do
    if [ "$c" != "$NEW_CON" ]; then
        echo "  - $c"
        nmcli connection modify "$c" autoconnect no || true
        nmcli connection down "$c" 2>/dev/null || true
    fi
done

echo "== Vytvářím/aktualizuji statický profil '$NEW_CON' ($ADDR) =="
if nmcli -t -f NAME connection show | grep -qxF "$NEW_CON"; then
    nmcli connection modify "$NEW_CON" ipv4.method manual ipv4.addresses "$ADDR" \
        connection.interface-name "$IFACE" connection.autoconnect yes
else
    nmcli connection add type ethernet con-name "$NEW_CON" ifname "$IFACE" \
        ipv4.method manual ipv4.addresses "$ADDR" ipv6.method link-local autoconnect yes
fi

echo "== Aktivuji =="
nmcli connection up "$NEW_CON"

echo
echo "Hotovo. $IFACE by měl mít $ADDR trvale, i po rebootu (žádné čekání na DHCP)."
ip -brief addr show "$IFACE"
