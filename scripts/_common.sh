#!/usr/bin/env bash

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FOAM_GRASP_ROOT="${PROJECT_ROOT}"

if [[ -f "${PROJECT_ROOT}/config/project.env" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/config/project.env"
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
CAN_PORT="${CAN_PORT:-can0}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
AUTO_ENABLE="${AUTO_ENABLE:-true}"
USE_RVIZ="${USE_RVIZ:-true}"
OBSERVE_BEFORE_GRASP="${OBSERVE_BEFORE_GRASP:-true}"
OBSERVE_JOINTS="${OBSERVE_JOINTS:-0.142604700 0.269998232 -0.653208024 0.008059128 1.078754404 0.054111288}"
OBSERVE_SETTLE_SECONDS="${OBSERVE_SETTLE_SECONDS:-2.0}"
OBSERVE_SLOWDOWN="${OBSERVE_SLOWDOWN:-1.50}"
PREGRASP_SLOWDOWN="${PREGRASP_SLOWDOWN:-1.25}"
ARM_SPEED_PERCENT="${ARM_SPEED_PERCENT:-15}"
AUTO_STAGE_PAUSE="${AUTO_STAGE_PAUSE:-0.50}"
CARTESIAN_JOINT_RATE="${CARTESIAN_JOINT_RATE:-0.10}"

CHECKPOINT_PATH="${PROJECT_ROOT}/runtime/models/best_model.pth"
CALIBRATION_PATH="${PROJECT_ROOT}/runtime/calibration/handeye_eye_in_hand.json"
RUNTIME_VENV="${PROJECT_ROOT}/venvs/runtime"

source_if_present() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck disable=SC1090
    source "${setup_file}"
    return 0
  fi
  return 1
}

source_vendor_stack() {
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_if_present "${PROJECT_ROOT}/workspaces/orbbec_ws/install/setup.bash" || true
  source_if_present "${PROJECT_ROOT}/workspaces/piper_ws/install/setup.bash" || true
  source_if_present "${PROJECT_ROOT}/workspaces/moveit_ws/install/setup.bash" || true
}

source_complete_stack() {
  source_vendor_stack
  if [[ -f "${RUNTIME_VENV}/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${RUNTIME_VENV}/bin/activate"
    export PYTHONNOUSERSITE=1
    # Vendor ROS Python entry points (notably Piper) may still have a
    # /usr/bin/python3 shebang. Make the project-owned pure-Python runtime
    # dependencies visible to those processes without falling back to
    # untracked packages under ~/.local.
    runtime_site="$(${RUNTIME_VENV}/bin/python -c \
      'import site; print(site.getsitepackages()[0])')"
    export PYTHONPATH="${runtime_site}${PYTHONPATH:+:${PYTHONPATH}}"
    # PyTorch cu121 loads this pip-provided shared library at import time.
    # Exporting the project-relative directory makes the venv relocatable and
    # avoids relying solely on wheel RPATH behavior after a directory move.
    nvjitlink_lib="${runtime_site}/nvidia/nvjitlink/lib"
    if [[ -d "${nvjitlink_lib}" ]]; then
      export LD_LIBRARY_PATH="${nvjitlink_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
  fi
  if ! source_if_present "${PROJECT_ROOT}/workspaces/app_ws/install/setup.bash"; then
    echo "应用工作空间尚未构建：请运行 scripts/build_all.sh" >&2
    return 1
  fi
}

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "缺少文件：$1" >&2
    return 1
  fi
}
