#!/usr/bin/env bash
set -eo pipefail

# Configure/install ROS first when this is a fresh Ubuntu 22.04 machine.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  "${script_dir}/install_ros2_humble.sh"
fi

sudo apt update
sudo apt install -y \
  build-essential cmake git git-lfs patch rsync curl wget ca-certificates ripgrep \
  can-utils ethtool iproute2 usbutils \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-venv python3-pip python3-dev python3-opencv python3-yaml \
  libgflags-dev nlohmann-json3-dev libdw-dev libssl-dev \
  libgoogle-glog-dev libusb-1.0-0-dev mesa-utils libgl1 \
  ros-humble-desktop ros-humble-moveit \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-camera-info-manager \
  ros-humble-diagnostic-updater ros-humble-backward-ros

# Python runtime packages, including python-can and piper_sdk, belong in the
# project venv and are installed by install_python_runtime.sh. Do not install
# them into ~/.local, because production runs deliberately disable user site.
