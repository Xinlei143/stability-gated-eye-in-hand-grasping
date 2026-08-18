#!/usr/bin/env bash
set -eo pipefail

# Run this once on the already-working Ubuntu computer before migration.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
mkdir -p "${PROJECT_ROOT}/dependencies"

capture_repos() {
  local source_dir="$1"
  local output_file="$2"
  if [[ -d "${source_dir}" ]] && command -v vcs >/dev/null 2>&1; then
    vcs export --exact "${source_dir}" > "${output_file}"
    echo "已记录：${output_file}"
  else
    echo "跳过不存在的源码目录：${source_dir}" >&2
  fi
}

capture_single_repo() {
  local repository_dir="$1"
  local repository_name="$2"
  local output_file="$3"
  if git -C "${repository_dir}" rev-parse --is-inside-work-tree \
      >/dev/null 2>&1; then
    local url
    local revision
    url="$(git -C "${repository_dir}" remote get-url origin)"
    revision="$(git -C "${repository_dir}" rev-parse HEAD)"
    {
      echo "repositories:"
      echo "  ${repository_name}:"
      echo "    type: git"
      echo "    url: ${url}"
      echo "    version: ${revision}"
    } > "${output_file}"
    echo "已记录：${output_file}"
    return 0
  fi
  return 1
}

LEGACY_HOME="${LEGACY_HOME:-${HOME}}"

# Prefer the project-owned workspaces. The legacy paths are only fallbacks for
# the first migration from the old /home/<user> layout.
orbbec_src="${PROJECT_ROOT}/workspaces/orbbec_ws/src"
if ! find "${orbbec_src}" -type d -name .git -print -quit 2>/dev/null \
    | grep -q .; then
  orbbec_src="${LEGACY_HOME}/orbbec_legacy_ws/src"
fi
capture_repos "${orbbec_src}" \
  "${PROJECT_ROOT}/dependencies/orbbec.repos"

piper_repo="${PROJECT_ROOT}/workspaces/piper_ws/_upstream/piper_ros"
piper_src="${PROJECT_ROOT}/workspaces/piper_ws/src"
if ! capture_single_repo "${piper_repo}" piper_ros \
    "${PROJECT_ROOT}/dependencies/piper.repos"; then
  if ! find "${piper_src}" -type d -name .git -print -quit 2>/dev/null \
      | grep -q .; then
    piper_repo="${LEGACY_HOME}/ROS_button/piper_ros"
    piper_src="${LEGACY_HOME}/ROS_button/piper_ros/src"
  fi
  capture_single_repo "${piper_repo}" piper_ros \
    "${PROJECT_ROOT}/dependencies/piper.repos" || \
    capture_repos "${piper_src}" \
      "${PROJECT_ROOT}/dependencies/piper.repos"
fi

moveit_src="${PROJECT_ROOT}/workspaces/moveit_ws/src"
if ! find "${moveit_src}" -type d -name .git -print -quit 2>/dev/null \
    | grep -q .; then
  moveit_src="${LEGACY_HOME}/ROS_button/moveit2_ws/src"
fi
if find "${moveit_src}" -type d -name .git -print -quit 2>/dev/null \
    | grep -q .; then
  capture_repos "${moveit_src}" \
    "${PROJECT_ROOT}/dependencies/moveit.repos"
else
  rm -f "${PROJECT_ROOT}/dependencies/moveit.repos"
  echo "当前使用apt MoveIt或没有源码overlay；不生成moveit.repos"
fi

python3 -m pip freeze > "${PROJECT_ROOT}/dependencies/python-system.freeze.txt"
dpkg-query -W > "${PROJECT_ROOT}/dependencies/ubuntu-packages.txt"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi > "${PROJECT_ROOT}/dependencies/nvidia-smi.txt"
fi
