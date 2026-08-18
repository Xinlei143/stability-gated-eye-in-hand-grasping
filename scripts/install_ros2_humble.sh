#!/usr/bin/env bash
set -eo pipefail

# Configure the official ROS 2 apt source and install Humble on Ubuntu 22.04.
# This follows the ROS 2 Humble deb-package installation flow. It is
# idempotent and does not source a user shell profile or start ROS nodes.

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别操作系统。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "只支持Ubuntu 22.04；当前为 ${PRETTY_NAME:-unknown}。" >&2
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble已安装。"
  exit 0
fi

sudo apt update
sudo apt install -y \
  ca-certificates curl python3 software-properties-common
sudo add-apt-repository -y universe

release_json="$(curl -fsSL --retry 5 \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest)"
ros_apt_version="$(
  python3 -c \
    'import json,sys; print(json.load(sys.stdin)["tag_name"])' \
    <<< "${release_json}"
)"
if [[ ! "${ros_apt_version}" =~ ^[0-9A-Za-z][0-9A-Za-z.+:~_-]*$ ]]; then
  echo "ROS apt source版本格式异常：${ros_apt_version}" >&2
  exit 1
fi

codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}"
ros_apt_deb="/tmp/ros2-apt-source.deb"
curl -fL --retry 5 \
  -o "${ros_apt_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${codename}_all.deb"
sudo dpkg -i "${ros_apt_deb}"

sudo apt update
# ROS documentation warns that systemd/udev packages on a fresh Jammy system
# should be updated before ROS dependencies are installed.
sudo apt install -y --only-upgrade systemd udev libudev1 || true
sudo apt install -y ros-humble-desktop ros-dev-tools

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
echo "ROS 2 Humble安装完成。"
