#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo '请在本机终端运行：sudo bash tools/install_system.sh'
    exit 1
fi
source /etc/os-release
if [[ ${ID} != ubuntu || ${VERSION_CODENAME} != noble ]]; then
    echo '此脚本仅支持 Ubuntu 24.04 Noble。'
    exit 1
fi
project_user=${SUDO_USER:-}
if [[ -z ${project_user} || ${project_user} == root ]]; then
    echo '请从普通用户终端通过 sudo 执行，以便设置串口和摄像头组。'
    exit 1
fi
apt-get update
apt-get install -y ca-certificates curl git software-properties-common
add-apt-repository -y universe
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    download_dir=$(mktemp -d)
    trap 'rm -rf "$download_dir"' EXIT
    curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest > "$download_dir/release.json"
    release_url=$(python3 -c 'import json,sys; release=json.load(open(sys.argv[1])); print(next(asset["browser_download_url"] for asset in release["assets"] if asset["name"].startswith("ros2-apt-source_") and asset["name"].endswith(".noble_all.deb")))' "$download_dir/release.json")
    [[ ${release_url} == https://github.com/ros-infrastructure/ros-apt-source/releases/download/* ]]
    curl -fL --retry 3 "$release_url" -o "$download_dir/ros2-apt-source.deb"
    dpkg -i "$download_dir/ros2-apt-source.deb"
fi
apt-get update
apt-get install -y \
    ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-robot-localization ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager ros-jazzy-image-view \
    ros-jazzy-camera-calibration-parsers ros-jazzy-camera-calibration \
    ros-jazzy-diagnostic-msgs ros-jazzy-xacro \
    python3-colcon-common-extensions python3-colcon-meson python3-rosdep \
    python3-opencv python3-numpy python3-pyzbar python3-yaml python3-serial \
    python3-pytest python3-pip python3-venv python3-jinja2 python3-ply \
    python3-yaml python3-pybind11 python3-pytest-cov \
    build-essential cmake ninja-build meson pkg-config \
    libyaml-dev libssl-dev libgnutls28-dev libevent-dev libudev-dev \
    libdrm-dev libjpeg-dev libtiff-dev libzbar0 libboost-dev nlohmann-json3-dev \
    libcamera-tools v4l-utils
usermod -aG dialout,video,render "$project_user"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi
echo '基础依赖安装完成。串口/摄像头组权限在注销并重新登录后生效。'
echo '无需立即重启。请回到聊天通知我，以便继续构建与检查。'
