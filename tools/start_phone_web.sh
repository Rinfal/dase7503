#!/usr/bin/env bash
# Serve only on the existing hotspot. Never reconfigure networking here.
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
while ! ip -4 -o address show dev wlan0 | /usr/bin/grep -q 'inet 10\.42\.50\.1/24 '; do
    sleep 2
done
exec bash "$project_root/tools/start_remote.sh" --ros-args \
    -p host:=10.42.50.1 -p port:=8080 \
    -p compressed_preview_topic:=/qr/preview/compressed \
    -p hardware_profile:="$HOME/.config/dase7503/phone-profile.yaml"
