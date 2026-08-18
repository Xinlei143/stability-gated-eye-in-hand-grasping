#!/usr/bin/env bash
set -eo pipefail

# Read-only checks for the supported deployment platform. No ROS or hardware
# process is started.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别操作系统。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "只支持Ubuntu 22.04；当前为 ${PRETTY_NAME:-unknown}。" >&2
  exit 1
fi

architecture="$(dpkg --print-architecture)"
if [[ "${architecture}" != "amd64" ]]; then
  echo "当前发布只验证amd64；检测到${architecture}。" >&2
  exit 1
fi

available_kb="$(df -Pk "${PROJECT_ROOT}" | awk 'NR==2 {print $4}')"
minimum_kb=$((30 * 1024 * 1024))
if (( available_kb < minimum_kb )); then
  echo "可用磁盘空间不足30 GiB。" >&2
  exit 1
fi

if [[ "${FOAM_GRASP_SKIP_GPU_CHECK:-0}" != "1" ]]; then
  if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "未找到nvidia-smi。" >&2
    echo "请先安装NVIDIA驱动并重启，再运行install.sh。" >&2
    exit 1
  fi
  if ! nvidia-smi >/dev/null 2>&1; then
    echo "NVIDIA驱动未正常工作；请修复并重启。" >&2
    exit 1
  fi
fi

echo "部署预检通过：Ubuntu 22.04 amd64，磁盘空间和NVIDIA驱动正常。"
