#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} != --locked ]]; then
    exec /usr/bin/flock --nonblock "$XDG_RUNTIME_DIR/dase7503-agent.lock" \
        bash "${BASH_SOURCE[0]}" --locked
fi
controller_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
while [[ ! -e "$controller_port" ]]; do
    sleep 2
done
# Normalize CP2102 control lines before opening the high-speed agent, matching
# the recovered connection procedure. Sends no serial bytes or actuator commands.
python3 "$project_root/tools/serial_probe.py" --port "$controller_port" --seconds 0.5 > /dev/null
exec bash "$project_root/tools/start_agent.sh"
