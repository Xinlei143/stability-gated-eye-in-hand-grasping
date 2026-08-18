#!/usr/bin/env bash
set -eo pipefail

# Read-only layout/status report. It does not source ROS or access hardware.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

status=0
check_path() {
  local label="$1"
  local path="$2"
  if [[ -e "${path}" ]]; then
    printf '[OK]      %-24s %s\n' "${label}" "${path}"
  else
    printf '[MISSING] %-24s %s\n' "${label}" "${path}"
    status=1
  fi
}

check_packages() {
  local label="$1"
  local source_dir="$2"
  local count
  count="$(find "${source_dir}" -name package.xml -type f 2>/dev/null | wc -l)"
  if (( count > 0 )); then
    printf '[OK]      %-24s %s packages\n' "${label}" "${count}"
  else
    printf '[MISSING] %-24s no package.xml under %s\n' "${label}" "${source_dir}"
    status=1
  fi
}

echo "===== Foam Grasp project status ====="
echo "root: ${PROJECT_ROOT}"
check_path "ROS Humble" "/opt/ros/${ROS_DISTRO}/setup.bash"
check_path "Orbbec lock" "${PROJECT_ROOT}/dependencies/orbbec.repos"
check_path "Piper lock" "${PROJECT_ROOT}/dependencies/piper.repos"
check_path "Runtime release" "${PROJECT_ROOT}/config/runtime-release.env"
check_packages "Orbbec source" "${PROJECT_ROOT}/workspaces/orbbec_ws/src"
check_packages "Piper source" "${PROJECT_ROOT}/workspaces/piper_ws/src"
check_packages "Application source" "${PROJECT_ROOT}/workspaces/app_ws/src"
check_path "Runtime venv" "${RUNTIME_VENV}/bin/python"
check_path "Segmentation model" "${CHECKPOINT_PATH}"
check_path "Hand-eye calibration" "${CALIBRATION_PATH}"
check_path "Application overlay" "${PROJECT_ROOT}/workspaces/app_ws/install/setup.bash"

if [[ -x "${RUNTIME_VENV}/bin/python" ]]; then
  if PYTHONNOUSERSITE=1 "${RUNTIME_VENV}/bin/python" - <<'PY' >/dev/null 2>&1
import can
import piper_sdk
import torch
import cv2
from cv_bridge import CvBridge
if not torch.cuda.is_available():
    raise SystemExit("CUDA unavailable")
PY
  then
    printf '[OK]      %-24s %s\n' "Runtime imports" "torch, cv_bridge, python-can, piper_sdk"
  else
    printf '[BROKEN]  %-24s %s\n' "Runtime imports" "run scripts/install_python_runtime.sh"
    status=1
  fi
fi

if [[ -x "${RUNTIME_VENV}/bin/python" \
    && -f "${CHECKPOINT_PATH}" \
    && -f "${CALIBRATION_PATH}" ]]; then
  if "${PROJECT_ROOT}/scripts/verify_runtime_assets.sh" \
      >/dev/null 2>&1; then
    printf '[OK]      %-24s %s\n' \
      "Runtime assets" "checkpoint metadata + calibration schema"
  else
    printf '[BROKEN]  %-24s %s\n' \
      "Runtime assets" "run scripts/verify_runtime_assets.sh"
    status=1
  fi
fi

if [[ -f "${CHECKPOINT_PATH}" ]]; then
  model_size="$(wc -c < "${CHECKPOINT_PATH}")"
  if (( model_size < 100 * 1024 * 1024 )); then
    printf '[BROKEN]  %-24s %s\n' \
      "Segmentation model" "file is too small/LFS pointer"
    status=1
  else
    du -h "${CHECKPOINT_PATH}"
  fi
fi
if [[ -f "${CALIBRATION_PATH}" ]]; then
  if python3 -m json.tool "${CALIBRATION_PATH}" >/dev/null 2>&1; then
    printf '[OK]      %-24s %s\n' \
      "Calibration JSON" "valid JSON"
  else
    printf '[BROKEN]  %-24s %s\n' \
      "Calibration JSON" "invalid JSON"
    status=1
  fi
fi
echo "disk usage: $(du -sh "${PROJECT_ROOT}" | awk '{print $1}')"

if (( status == 0 )); then
  echo "状态：迁移所需的核心文件齐全。"
else
  echo "状态：项目尚不完整，请根据[MISSING]补齐。" >&2
fi
exit "${status}"
