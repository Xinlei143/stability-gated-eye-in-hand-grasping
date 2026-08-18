#!/usr/bin/env bash
set -eo pipefail

# Read-only verification that a cold machine can discover the immutable source
# tag and both runtime assets. It does not download the large model archive.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

release_config="${PROJECT_ROOT}/config/runtime-release.env"
require_file "${release_config}"
# shellcheck disable=SC1090
source "${release_config}"

required_release_vars=(
  RUNTIME_RELEASE_REPOSITORY
  RUNTIME_RELEASE_TAG
  RUNTIME_RELEASE_ASSET
  RUNTIME_RELEASE_CHECKSUM_ASSET
)
for name in "${required_release_vars[@]}"; do
  if [[ -z "${!name:-}" ]]; then
    echo "runtime-release.env缺少：${name}" >&2
    exit 1
  fi
done

repository_url="https://github.com/${RUNTIME_RELEASE_REPOSITORY}.git"
if ! git ls-remote --exit-code --tags \
    "${repository_url}" "refs/tags/${RUNTIME_RELEASE_TAG}" >/dev/null; then
  echo "远端源码tag不存在：${RUNTIME_RELEASE_TAG}" >&2
  exit 1
fi
echo "[OK] source tag: ${RUNTIME_RELEASE_TAG}"

base_url="https://github.com/${RUNTIME_RELEASE_REPOSITORY}/releases/download/${RUNTIME_RELEASE_TAG}"
temporary_dir="$(mktemp -d)"
trap 'rm -rf "${temporary_dir}"' EXIT
checksum="${temporary_dir}/${RUNTIME_RELEASE_CHECKSUM_ASSET}"

curl -fL --retry 5 --retry-delay 2 \
  -o "${checksum}" \
  "${base_url}/${RUNTIME_RELEASE_CHECKSUM_ASSET}"

checksum_lines="$(wc -l < "${checksum}")"
checksum_name="$(awk 'NR==1 {print $2}' "${checksum}")"
checksum_digest="$(awk 'NR==1 {print $1}' "${checksum}")"
if (( checksum_lines != 1 )) \
    || [[ "${checksum_name#\*}" != "${RUNTIME_RELEASE_ASSET}" ]] \
    || [[ ! "${checksum_digest}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "远端SHA-256文件格式错误。">&2
  exit 1
fi
echo "[OK] checksum asset: ${RUNTIME_RELEASE_CHECKSUM_ASSET}"

# Avoid downloading the roughly 500 MB archive during a publication audit.
# Following the public URL with a HEAD request verifies unauthenticated
# discoverability without transferring the model body.
if ! curl -fIL --retry 5 --retry-delay 2 \
    "${base_url}/${RUNTIME_RELEASE_ASSET}" >/dev/null; then
  echo "运行资源归档不可公开读取：${RUNTIME_RELEASE_ASSET}" >&2
  exit 1
fi
echo "[OK] runtime asset is publicly reachable: ${RUNTIME_RELEASE_ASSET}"
echo "远端一键部署资源检查通过。"
