#!/usr/bin/env bash
set -eo pipefail

# Export the public/source engineering package. Models, calibration, datasets,
# vendor source, generated workspaces and virtual environments are excluded.
# Use package_for_transfer.sh instead when a private deployable bundle is needed.

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

package_version="$(
  awk -F'[<>]' '/<version>/{print $3; exit}' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp/package.xml"
)"
if [[ -z "${package_version}" ]]; then
  echo "无法读取foam_grasp版本" >&2
  exit 1
fi

PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-${TMPDIR:-/tmp}/foam_grasp_pycache}" \
  "${PROJECT_ROOT}/scripts/validate_project.sh"

output="${1:-${PROJECT_ROOT}/../foam_grasp_project_source_v${package_version}.tar.gz}"
mkdir -p "$(dirname "${output}")"
parent="$(dirname "${PROJECT_ROOT}")"
name="$(basename "${PROJECT_ROOT}")"

COPYFILE_DISABLE=1 tar \
  --exclude="${name}/.git" \
  --exclude="${name}/.DS_Store" \
  --exclude="${name}/._*" \
  --exclude="*/.DS_Store" \
  --exclude="*/._*" \
  --exclude="${name}/workspaces/*_ws/build" \
  --exclude="${name}/workspaces/*_ws/install" \
  --exclude="${name}/workspaces/*_ws/log" \
  --exclude="${name}/workspaces/orbbec_ws/src/*" \
  --exclude="${name}/workspaces/piper_ws/src/*" \
  --exclude="${name}/workspaces/moveit_ws/src/*" \
  --exclude="${name}/workspaces/piper_ws/_upstream" \
  --exclude="${name}/venvs" \
  --exclude="${name}/runtime/models/*.pth" \
  --exclude="${name}/runtime/models/*.pt" \
  --exclude="${name}/runtime/models/*.onnx" \
  --exclude="${name}/runtime/models/*.engine" \
  --exclude="${name}/runtime/calibration/*.json" \
  --exclude="${name}/runtime/logs/*" \
  --exclude="${name}/data/*" \
  --exclude="${name}/dependencies/python-system.freeze.txt" \
  --exclude="${name}/dependencies/ubuntu-packages.txt" \
  --exclude="${name}/dependencies/nvidia-smi.txt" \
  --exclude="__pycache__" \
  --exclude="*.pyc" \
  --exclude="*.tar.gz" \
  -czf "${output}" \
  -C "${parent}" "${name}"

checksum_file="${output}.sha256"
if command -v sha256sum >/dev/null 2>&1; then
  (
    cd "$(dirname "${output}")"
    sha256sum "$(basename "${output}")" > "$(basename "${checksum_file}")"
  )
else
  (
    cd "$(dirname "${output}")"
    shasum -a 256 "$(basename "${output}")" > "$(basename "${checksum_file}")"
  )
fi

manifest_file="${output}.manifest.txt"
{
  echo "Foam Grasp source release"
  echo "version=${package_version}"
  echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "archive=$(basename "${output}")"
  echo "includes=first_party_source,scripts,configuration,documentation,training_tools"
  echo "excludes=model,calibration,data,vendor_source,build,install,log,venv"
  echo "required_private_model=runtime/models/best_model.pth"
  echo "required_private_calibration=runtime/calibration/handeye_eye_in_hand.json"
} > "${manifest_file}"

echo "源码包：${output}"
echo "校验文件：${checksum_file}"
echo "内容清单：${manifest_file}"
du -h "${output}"
