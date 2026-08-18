#!/usr/bin/env bash
set -eo pipefail

# Creates a relocatable runtime deployment bundle. It includes vendor source,
# checkpoint and calibration, but excludes generated builds, venvs and data.
# System/Python dependencies are reinstalled on the destination computer.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

require_file "${CHECKPOINT_PATH}"
require_file "${CALIBRATION_PATH}"

require_ros_source() {
  local workspace="$1"
  local label="$2"
  if ! find "${workspace}/src" -name package.xml -print -quit 2>/dev/null \
      | grep -q .; then
    echo "缺少${label}源码：${workspace}/src" >&2
    echo "请先运行 scripts/migrate_legacy.sh 或 scripts/import_vendor.sh" >&2
    exit 1
  fi
}

require_ros_source "${PROJECT_ROOT}/workspaces/orbbec_ws" "Orbbec"
require_ros_source "${PROJECT_ROOT}/workspaces/piper_ws" "Piper"
require_ros_source "${PROJECT_ROOT}/workspaces/app_ws" "foam_grasp"

output="${1:-${PROJECT_ROOT}/../foam_grasp_project_deploy_$(date +%Y%m%d_%H%M%S).tar.gz}"
mkdir -p "$(dirname "${output}")"
parent="$(dirname "${PROJECT_ROOT}")"
name="$(basename "${PROJECT_ROOT}")"

tar \
  --exclude="${name}/workspaces/*_ws/build" \
  --exclude="${name}/workspaces/*_ws/install" \
  --exclude="${name}/workspaces/*_ws/log" \
  --exclude="${name}/venvs" \
  --exclude="${name}/data/*" \
  --exclude="${name}/runtime/logs/*" \
  --exclude="__pycache__" \
  --exclude="*.pyc" \
  -czf "${output}" \
  -C "${parent}" "${name}"

echo "部署包：${output}"
du -h "${output}"

checksum_file="${output}.sha256"
(
  cd "$(dirname "${output}")"
  sha256sum "$(basename "${output}")" > "$(basename "${checksum_file}")"
)

manifest_file="${output}.manifest.txt"
{
  echo "Foam Grasp transfer bundle"
  echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "archive=$(basename "${output}")"
  echo "project_dir=${name}"
  echo "ros_distro=${ROS_DISTRO}"
  echo "includes=app_source,vendor_source,model,calibration,configuration,docs"
  echo "excludes=build,install,log,venv,data"
  echo "model_sha256=$(sha256sum "${CHECKPOINT_PATH}" | awk '{print $1}')"
  echo "calibration_sha256=$(sha256sum "${CALIBRATION_PATH}" | awk '{print $1}')"
} > "${manifest_file}"

echo "校验文件：${checksum_file}"
echo "内容清单：${manifest_file}"
