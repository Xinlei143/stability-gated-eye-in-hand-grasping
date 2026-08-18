#!/usr/bin/env bash
set -eo pipefail

# Read-only cold-install acceptance test. This script does not start ROS nodes,
# configure CAN, enable Piper, or publish any motion command.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source_complete_stack

"${PROJECT_ROOT}/scripts/verify_vendor_locks.sh"
"${PROJECT_ROOT}/scripts/verify_runtime_assets.sh"

check_prefix() {
  local package="$1"
  local expected_root="$2"
  local actual
  actual="$(ros2 pkg prefix "${package}")"
  case "${actual}" in
    "${expected_root}"/install/*)
      echo "[OK] ${package}: ${actual}"
      ;;
    *)
      echo "${package}解析到了错误overlay：${actual}" >&2
      echo "期望位于：${expected_root}/install" >&2
      exit 1
      ;;
  esac
}

check_prefix orbbec_camera "${PROJECT_ROOT}/workspaces/orbbec_ws"
check_prefix piper "${PROJECT_ROOT}/workspaces/piper_ws"
check_prefix moveit_ros_move_group "${PROJECT_ROOT}/workspaces/moveit_ws"
check_prefix foam_grasp "${PROJECT_ROOT}/workspaces/app_ws"

required_executables=(
  segmentation_node
  depth_fusion_node
  camera_to_base_node
  target_latch_node
  grasp_pose_preview_node
  move_to_observe
  object_grasp_sequence
)
available_executables="$(ros2 pkg executables foam_grasp)"
for executable in "${required_executables[@]}"; do
  if ! grep -Fqx "foam_grasp ${executable}" \
      <<<"${available_executables}"; then
    echo "foam_grasp缺少可执行程序：${executable}" >&2
    exit 1
  fi
done
echo "[OK] foam_grasp ROS 2 executables"

PYTHONNOUSERSITE=1 "${RUNTIME_VENV}/bin/python" <<'PY'
import cv2
import torch
import torchvision
from cv_bridge import CvBridge

if not torch.cuda.is_available():
    raise SystemExit("PyTorch无法使用CUDA")
x = torch.randn(128, 128, device="cuda")
_ = x @ x
print(
    "[OK] runtime imports and CUDA:",
    torch.__version__,
    torchvision.__version__,
    torch.cuda.get_device_name(0),
)
PY

PYTHONNOUSERSITE=1 /usr/bin/python3 <<'PY'
import can
import piper_sdk

print("[OK] Piper system interpreter imports:", can.__version__, piper_sdk.__file__)
PY

ros2 launch foam_grasp system.launch.py --show-args >/dev/null
ros2 launch foam_grasp safe_plan_only.launch.py --show-args >/dev/null
echo "[OK] launch descriptions parse successfully"
echo "安装后只读烟雾测试通过；未启动ROS或机械臂。"
