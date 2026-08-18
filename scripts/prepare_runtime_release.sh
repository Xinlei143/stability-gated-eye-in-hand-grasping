#!/usr/bin/env bash
set -eo pipefail

# Run this on the already-validated Ubuntu computer. It packages the trained
# checkpoint and the calibration for one physical rig as GitHub Release assets.
# With --upload, GitHub CLI creates/updates the configured release.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

release_config="${PROJECT_ROOT}/config/runtime-release.env"
require_file "${release_config}"
# shellcheck disable=SC1090
source "${release_config}"
require_file "${CHECKPOINT_PATH}"
require_file "${CALIBRATION_PATH}"

upload=false
if [[ "${1:-}" == "--upload" ]]; then
  upload=true
elif [[ -n "${1:-}" ]]; then
  echo "用法：$0 [--upload]" >&2
  exit 2
fi

model_size="$(wc -c < "${CHECKPOINT_PATH}")"
if (( model_size < 100 * 1024 * 1024 )); then
  echo "模型文件异常小，拒绝发布：${CHECKPOINT_PATH}" >&2
  exit 1
fi
"${PROJECT_ROOT}/scripts/verify_runtime_assets.sh"

dist_dir="${PROJECT_ROOT}/dist"
temporary_dir="$(mktemp -d)"
trap 'rm -rf "${temporary_dir}"' EXIT
payload="${temporary_dir}/foam-grasp-runtime"
mkdir -p \
  "${payload}/runtime/models" \
  "${payload}/runtime/calibration"
install -m 0644 "${CHECKPOINT_PATH}" \
  "${payload}/runtime/models/best_model.pth"
install -m 0644 "${CALIBRATION_PATH}" \
  "${payload}/runtime/calibration/handeye_eye_in_hand.json"

cat > "${payload}/runtime/manifest.txt" <<EOF
format_version=1
release_tag=${RUNTIME_RELEASE_TAG}
rig_id=${RUNTIME_RIG_ID}
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
model_sha256=$(sha256sum "${CHECKPOINT_PATH}" | awk '{print $1}')
calibration_sha256=$(sha256sum "${CALIBRATION_PATH}" | awk '{print $1}')
model_bytes=${model_size}
EOF

mkdir -p "${dist_dir}"
archive="${dist_dir}/${RUNTIME_RELEASE_ASSET}"
tar \
  --sort=name \
  --mtime='UTC 1970-01-01' \
  --owner=0 --group=0 --numeric-owner \
  -czf "${archive}" \
  -C "${temporary_dir}" foam-grasp-runtime
checksum="${dist_dir}/${RUNTIME_RELEASE_CHECKSUM_ASSET}"
(
  cd "${dist_dir}"
  sha256sum "${RUNTIME_RELEASE_ASSET}" \
    > "${RUNTIME_RELEASE_CHECKSUM_ASSET}"
)

echo "运行资源已生成："
ls -lh "${archive}" "${checksum}"
cat "${checksum}"

if [[ "${upload}" == "true" ]]; then
  command -v gh >/dev/null 2>&1 || {
    echo "缺少GitHub CLI：gh" >&2
    exit 1
  }
  gh auth status
  if gh release view "${RUNTIME_RELEASE_TAG}" \
      --repo "${RUNTIME_RELEASE_REPOSITORY}" >/dev/null 2>&1; then
    existing_assets="$(
      gh release view "${RUNTIME_RELEASE_TAG}" \
        --repo "${RUNTIME_RELEASE_REPOSITORY}" \
        --json assets \
        --jq '.assets[].name'
    )"
    if grep -Fqx "${RUNTIME_RELEASE_ASSET}" <<<"${existing_assets}" \
        || grep -Fqx \
          "${RUNTIME_RELEASE_CHECKSUM_ASSET}" <<<"${existing_assets}"; then
      echo "Release中已存在同名运行资源，拒绝覆盖不可变版本。" >&2
      echo "模型或标定发生变化时，请提升Release版本。" >&2
      exit 1
    fi
    gh release upload "${RUNTIME_RELEASE_TAG}" \
      "${archive}" "${checksum}" \
      --repo "${RUNTIME_RELEASE_REPOSITORY}"
  else
    gh release create "${RUNTIME_RELEASE_TAG}" \
      "${archive}" "${checksum}" \
      --repo "${RUNTIME_RELEASE_REPOSITORY}" \
      --title "Foam Grasp runtime ${RUNTIME_RELEASE_TAG}" \
      --notes "DeepLabV3-ResNet50 checkpoint and rig-specific hand-eye calibration for ${RUNTIME_RIG_ID}."
  fi
  echo "GitHub Release运行资源上传完成。"
fi
