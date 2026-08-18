#!/usr/bin/env bash
set -eo pipefail

# Download the versioned model + rig-specific hand-eye calibration from a
# GitHub Release, verify the release checksum, and install them atomically.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

release_config="${PROJECT_ROOT}/config/runtime-release.env"
require_file "${release_config}"
# shellcheck disable=SC1090
source "${release_config}"

force=false
if [[ "${1:-}" == "--force" ]]; then
  force=true
elif [[ -n "${1:-}" ]]; then
  echo "用法：$0 [--force]" >&2
  exit 2
fi

validate_installed() {
  local installed_manifest="${PROJECT_ROOT}/runtime/manifest.txt"
  [[ -f "${CHECKPOINT_PATH}" \
      && -f "${CALIBRATION_PATH}" \
      && -f "${installed_manifest}" ]] || return 1
  if ! grep -Fqx "rig_id=${RUNTIME_RIG_ID}" "${installed_manifest}" \
      || ! grep -Fqx \
        "release_tag=${RUNTIME_RELEASE_TAG}" "${installed_manifest}"; then
    return 1
  fi

  local model_size manifest_model_sha manifest_calibration_sha
  local actual_model_sha actual_calibration_sha
  model_size="$(wc -c < "${CHECKPOINT_PATH}")"
  if (( model_size < 100 * 1024 * 1024 )); then
    echo "模型文件异常小，可能是Git LFS指针：${CHECKPOINT_PATH}" >&2
    return 1
  fi
  manifest_model_sha="$(
    sed -n 's/^model_sha256=//p' "${installed_manifest}"
  )"
  manifest_calibration_sha="$(
    sed -n 's/^calibration_sha256=//p' "${installed_manifest}"
  )"
  actual_model_sha="$(sha256sum "${CHECKPOINT_PATH}" | awk '{print $1}')"
  actual_calibration_sha="$(
    sha256sum "${CALIBRATION_PATH}" | awk '{print $1}'
  )"
  [[ "${manifest_model_sha}" =~ ^[0-9a-f]{64}$ \
      && "${manifest_model_sha}" == "${actual_model_sha}" ]] || return 1
  [[ "${manifest_calibration_sha}" =~ ^[0-9a-f]{64}$ \
      && "${manifest_calibration_sha}" == "${actual_calibration_sha}" ]] \
    || return 1
  [[ "${actual_model_sha}" == "${RUNTIME_MODEL_SHA256}" \
      && "${model_size}" == "${RUNTIME_MODEL_BYTES}" ]] || return 1
  local calibration_size
  calibration_size="$(wc -c < "${CALIBRATION_PATH}")"
  [[ "${actual_calibration_sha}" == "${RUNTIME_CALIBRATION_SHA256}" \
      && "${calibration_size}" == "${RUNTIME_CALIBRATION_BYTES}" ]] \
    || return 1
  python3 - "${CALIBRATION_PATH}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
with path.open("r", encoding="utf-8") as stream:
    value = json.load(stream)
if not isinstance(value, dict):
    raise SystemExit("手眼标定JSON根节点必须是对象")
PY
}

if [[ "${force}" != "true" ]] && validate_installed; then
  echo "模型和标定已经存在；跳过下载。使用--force可重新安装。"
  exit 0
fi

required_release_vars=(
  RUNTIME_RELEASE_REPOSITORY
  RUNTIME_RELEASE_TAG
  RUNTIME_RELEASE_ASSET
  RUNTIME_RELEASE_CHECKSUM_ASSET
  RUNTIME_RIG_ID
  RUNTIME_MODEL_SHA256
  RUNTIME_MODEL_BYTES
  RUNTIME_CALIBRATION_SHA256
  RUNTIME_CALIBRATION_BYTES
)
for name in "${required_release_vars[@]}"; do
  if [[ -z "${!name:-}" ]]; then
    echo "runtime-release.env缺少：${name}" >&2
    exit 1
  fi
done

base_url="${RUNTIME_ASSET_BASE_URL:-https://github.com/${RUNTIME_RELEASE_REPOSITORY}/releases/download/${RUNTIME_RELEASE_TAG}}"
temporary_dir="$(mktemp -d)"
trap 'rm -rf "${temporary_dir}"' EXIT
archive="${temporary_dir}/${RUNTIME_RELEASE_ASSET}"
checksum="${temporary_dir}/${RUNTIME_RELEASE_CHECKSUM_ASSET}"

echo "下载运行资源：${RUNTIME_RELEASE_TAG}"
curl -fL --retry 5 --retry-delay 2 \
  -o "${archive}" "${base_url}/${RUNTIME_RELEASE_ASSET}"
curl -fL --retry 5 --retry-delay 2 \
  -o "${checksum}" "${base_url}/${RUNTIME_RELEASE_CHECKSUM_ASSET}"

checksum_lines="$(wc -l < "${checksum}")"
checksum_name="$(awk 'NR==1 {print $2}' "${checksum}")"
checksum_digest="$(awk 'NR==1 {print $1}' "${checksum}")"
if (( checksum_lines != 1 )) \
    || [[ "${checksum_name#\\*}" != "${RUNTIME_RELEASE_ASSET}" ]] \
    || [[ ! "${checksum_digest}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "Release校验文件格式或目标文件名不正确。" >&2
  exit 1
fi

(
  cd "${temporary_dir}"
  sha256sum -c "${RUNTIME_RELEASE_CHECKSUM_ASSET}"
)

while IFS= read -r member; do
  if [[ "${member}" = /* || "${member}" == *"../"* ]]; then
    echo "运行资源包包含不安全路径：${member}" >&2
    exit 1
  fi
done < <(tar -tzf "${archive}")

extract_dir="${temporary_dir}/extract"
mkdir -p "${extract_dir}"
tar -xzf "${archive}" -C "${extract_dir}"
payload="${extract_dir}/foam-grasp-runtime"
model_source="${payload}/runtime/models/best_model.pth"
calibration_source="${payload}/runtime/calibration/handeye_eye_in_hand.json"
manifest_source="${payload}/runtime/manifest.txt"
require_file "${model_source}"
require_file "${calibration_source}"
require_file "${manifest_source}"

if ! grep -Fqx "rig_id=${RUNTIME_RIG_ID}" "${manifest_source}"; then
  echo "运行资源的rig_id与项目配置不一致。" >&2
  echo "期望：${RUNTIME_RIG_ID}" >&2
  exit 1
fi
if ! grep -Fqx "release_tag=${RUNTIME_RELEASE_TAG}" "${manifest_source}"; then
  echo "运行资源的release_tag与项目配置不一致。" >&2
  exit 1
fi
manifest_model_sha="$(
  sed -n 's/^model_sha256=//p' "${manifest_source}"
)"
manifest_calibration_sha="$(
  sed -n 's/^calibration_sha256=//p' "${manifest_source}"
)"
actual_model_sha="$(sha256sum "${model_source}" | awk '{print $1}')"
actual_calibration_sha="$(
  sha256sum "${calibration_source}" | awk '{print $1}'
)"
if [[ ! "${manifest_model_sha}" =~ ^[0-9a-f]{64}$ \
    || "${manifest_model_sha}" != "${actual_model_sha}" ]]; then
  echo "模型内部SHA-256校验失败。" >&2
  exit 1
fi
if [[ ! "${manifest_calibration_sha}" =~ ^[0-9a-f]{64}$ \
    || "${manifest_calibration_sha}" != "${actual_calibration_sha}" ]]; then
  echo "标定内部SHA-256校验失败。" >&2
  exit 1
fi
if [[ "${actual_model_sha}" != "${RUNTIME_MODEL_SHA256}" \
    || "$(wc -c < "${model_source}")" != "${RUNTIME_MODEL_BYTES}" ]]; then
  echo "模型和源码固定的v1.0.0指纹不一致。" >&2
  exit 1
fi
if [[ "${actual_calibration_sha}" != "${RUNTIME_CALIBRATION_SHA256}" \
    || "$(wc -c < "${calibration_source}")" \
      != "${RUNTIME_CALIBRATION_BYTES}" ]]; then
  echo "标定和源码固定的rig指纹不一致。" >&2
  exit 1
fi

mkdir -p \
  "$(dirname "${CHECKPOINT_PATH}")" \
  "$(dirname "${CALIBRATION_PATH}")"
install -m 0644 "${model_source}" "${CHECKPOINT_PATH}.new"
install -m 0644 "${calibration_source}" "${CALIBRATION_PATH}.new"
mv -f "${CHECKPOINT_PATH}.new" "${CHECKPOINT_PATH}"
mv -f "${CALIBRATION_PATH}.new" "${CALIBRATION_PATH}"
install -m 0644 "${manifest_source}" "${PROJECT_ROOT}/runtime/manifest.txt"

validate_installed
echo "运行资源安装完成："
ls -lh "${CHECKPOINT_PATH}" "${CALIBRATION_PATH}"
echo "rig_id=${RUNTIME_RIG_ID}"
