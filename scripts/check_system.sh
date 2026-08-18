#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source_complete_stack

echo "===== Artifacts ====="
ls -lh "${CHECKPOINT_PATH}" "${CALIBRATION_PATH}"
echo "===== CAN ====="
ip -brief link show "${CAN_PORT}"
echo "===== Devices ====="
lsusb | grep -i -E '2bc5|orbbec' || true
echo "===== Nodes ====="
ros2 node list | sort -u
echo "===== Command path ====="
ros2 topic info /joint_states
echo "===== MoveIt execution ====="
ros2 param get /move_group allow_trajectory_execution
echo "===== Camera/depth/segmentation rates ====="
timeout 5 ros2 topic hz /camera/color/image_raw || true
timeout 5 ros2 topic hz /camera/depth/image_raw || true
timeout 5 ros2 topic hz /foam_segmentation/mask || true
