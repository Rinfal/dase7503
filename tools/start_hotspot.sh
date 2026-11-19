#!/usr/bin/env bash
# Use the spare built-in radio; preserve the USB Wi-Fi internet connection.
set -euo pipefail
uplink=wlx7c10c9142324
hotspot=DASE7503-Car
secret_dir="$HOME/.config/dase7503"
if [[ $(nmcli -g GENERAL.STATE device show "$uplink") != 100* ]]; then
    echo 'USB Wi-Fi不在线，拒绝改变无线网络。'
    exit 1
fi
if [[ $(nmcli -g WIFI-PROPERTIES.AP device show wlan0) != yes ]]; then
    echo '内置Wi-Fi不支持热点。'
    exit 1
fi
mkdir -p "$secret_dir"
chmod 700 "$secret_dir"
if [[ ! -f "$secret_dir/hotspot-password" ]]; then
    (umask 077; python3 -c 'import secrets; print(secrets.token_hex(6))' > "$secret_dir/hotspot-password")
fi
if ! nmcli -g NAME connection show | rg -Fxq "$hotspot"; then
    nmcli connection add type wifi ifname wlan0 con-name "$hotspot" ssid "$hotspot" \
      802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
      802-11-wireless.powersave 2 wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn \
      wifi-sec.psk "$(cat "$secret_dir/hotspot-password")" \
      ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv4.never-default yes \
      ipv6.method disabled connection.autoconnect no
fi
nmcli --wait 20 connection up "$hotspot" ifname wlan0
[[ $(nmcli -g GENERAL.STATE device show "$uplink") == 100* ]]
ip -4 route show default
echo '热点：DASE7503-Car；网页：http://10.42.0.1:8080'
echo "热点口令文件：$secret_dir/hotspot-password"
