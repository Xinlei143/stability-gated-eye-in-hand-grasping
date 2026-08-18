#!/usr/bin/env bash
set -eo pipefail

# Copies only required runtime artifacts. Nothing in the legacy layout is deleted.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
LEGACY_HOME="${LEGACY_HOME:-${HOME}}"

old_model="${LEGACY_HOME}/foam_grasp_dataset/training_runs/deeplabv3_resnet50/best_model.pth"
old_calibration="${LEGACY_HOME}/foam_grasp_dataset/calibration/handeye_eye_in_hand.json"

require_file "${old_model}"
require_file "${old_calibration}"
install -m 0644 "${old_model}" "${CHECKPOINT_PATH}"
install -m 0644 "${old_calibration}" "${CALIBRATION_PATH}"

copy_source_tree() {
  local old_src="$1"
  local new_src="$2"
  if [[ -d "${old_src}" ]]; then
    mkdir -p "${new_src}"
    rsync -a "${old_src}/" "${new_src}/"
    echo "已复制源码：${old_src}"
  else
    echo "未找到源码目录，跳过：${old_src}" >&2
  fi
}

copy_source_tree "${LEGACY_HOME}/orbbec_legacy_ws/src" \
  "${PROJECT_ROOT}/workspaces/orbbec_ws/src"
copy_source_tree "${LEGACY_HOME}/ROS_button/piper_ros/src" \
  "${PROJECT_ROOT}/workspaces/piper_ws/src"
copy_source_tree "${LEGACY_HOME}/ROS_button/moveit2_ws/src" \
  "${PROJECT_ROOT}/workspaces/moveit_ws/src"

LEGACY_HOME="${LEGACY_HOME}" "${PROJECT_ROOT}/scripts/capture_environment.sh"

echo "运行文件和第三方源码迁移完成。原目录没有被移动或删除。"
echo "大数据集请按需执行："
echo "rsync -a --info=progress2 ${LEGACY_HOME}/foam_grasp_dataset/ ${PROJECT_ROOT}/data/"
