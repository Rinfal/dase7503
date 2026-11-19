#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
controller_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
if [[ ! -e "$controller_port" ]]; then
    echo '未找到已确认的 ESP32 CP2102 串口。'
    exit 1
fi
if [[ ! -r "$controller_port" || ! -w "$controller_port" ]]; then
    echo '当前会话没有串口权限。请在 dialout 组生效的会话中运行。'
    exit 1
fi
if [[ ${ROS_DOMAIN_ID} != 20 ]]; then
    echo '当前控制板固件使用 ROS_DOMAIN_ID=20，请先核对环境。'
    exit 1
fi
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$controller_port" -b 921600 -v4
