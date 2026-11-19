project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '缺少 /opt/ros/jazzy/setup.bash，请先完成依赖安装。'
    return 1
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$project_root/camera_ws/install/setup.bash" ]]; then
    source "$project_root/camera_ws/install/setup.bash"
fi
if [[ -f "$project_root/robot_ws/install/setup.bash" ]]; then
    source "$project_root/robot_ws/install/setup.bash"
fi
if [[ -f "$project_root/agent_ws/install/setup.bash" ]]; then
    source "$project_root/agent_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
