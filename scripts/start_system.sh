#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source_complete_stack
require_file "${CHECKPOINT_PATH}"
require_file "${CALIBRATION_PATH}"

can_state="$(ip -brief link show "${CAN_PORT}" 2>/dev/null || true)"
if [[ "${can_state}" != *" UP "* ]]; then
  echo "${CAN_PORT}尚未UP；请先运行 scripts/setup_can.sh" >&2
  exit 1
fi

echo "注意：auto_enable=${AUTO_ENABLE}；启动驱动时空夹爪可能先收到0 mm命令。"
exec ros2 launch foam_grasp system.launch.py \
  checkpoint:="${CHECKPOINT_PATH}" \
  calibration_file:="${CALIBRATION_PATH}" \
  can_port:="${CAN_PORT}" \
  auto_enable:="${AUTO_ENABLE}" \
  use_rviz:="${USE_RVIZ}"
