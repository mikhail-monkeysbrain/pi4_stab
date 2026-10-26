#!/usr/bin/env bash
set -euo pipefail

IFACE="wlan0"
HOME_NET="Dom"
AP_NET="JTZero-Flight"
AP_IP="192.168.2.1/24"

log() {
    echo "[JTZERO NETWORK] $*"
}

# Ждём готовности NetworkManager
for i in $(seq 1 20); do
    if nmcli general status >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

nmcli radio wifi on

CURRENT=$(nmcli -g GENERAL.CONNECTION \
    device show "$IFACE" 2>/dev/null | head -n1)

# Не трогаем работающий Dom
if [[ "$CURRENT" == "$HOME_NET" ]]; then
    log "Dom уже подключён."
    exit 0
fi

# Пробуем подключиться к Dom
log "Проверяем доступность Dom..."

if nmcli --wait 20 connection up "$HOME_NET" \
    ifname "$IFACE"; then
    log "Успешное подключение к Dom."
    exit 0
fi

# Dom недоступен — создаём точку доступа
log "Dom недоступен. Запускаем JTZero-Flight."

if ! nmcli -g GENERAL.CONNECTION \
    device show "$IFACE" 2>/dev/null |
    grep -Fxq "$AP_NET"; then

    nmcli --wait 20 connection up "$AP_NET" \
        ifname "$IFACE"
fi

log "Точка доступа JTZero-Flight запущена."
log "IP: ${AP_IP}"
