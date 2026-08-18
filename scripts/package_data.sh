#!/usr/bin/env bash
set -eo pipefail

# Datasets and training outputs are intentionally transferred separately from
# the runtime project, because they are large and are not needed for grasping.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

if ! find "${PROJECT_ROOT}/data" -mindepth 1 -type f \
    ! -name README.md -print -quit | grep -q .; then
  echo "data/中没有可打包的数据集文件。" >&2
  exit 1
fi

output="${1:-${PROJECT_ROOT}/../foam_grasp_data_$(date +%Y%m%d_%H%M%S).tar.gz}"
mkdir -p "$(dirname "${output}")"

tar \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  -czf "${output}" \
  -C "${PROJECT_ROOT}" data

(
  cd "$(dirname "${output}")"
  sha256sum "$(basename "${output}")" > "$(basename "${output}.sha256")"
)

echo "数据包：${output}"
echo "校验文件：${output}.sha256"
du -h "${output}"
