#!/usr/bin/env bash
set -eo pipefail

# Install the udev rules shipped by the pinned Orbbec driver. This does not
# launch the camera, but the camera must be unplugged/replugged afterwards.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

udev_script="${PROJECT_ROOT}/workspaces/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"
if [[ ! -f "${udev_script}" ]]; then
  echo "缺少Orbbec udev安装脚本：${udev_script}" >&2
  exit 1
fi

sudo bash "${udev_script}"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Orbbec udev规则已安装；请拔插DaBai DC1后再做设备检查。"
