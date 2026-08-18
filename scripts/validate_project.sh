#!/usr/bin/env bash
set -eo pipefail

# Static validation; it does not start ROS nodes or move hardware.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

for script in "${PROJECT_ROOT}"/scripts/*.sh; do
  bash -n "${script}"
done
bash -n "${PROJECT_ROOT}/install.sh"

PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-${TMPDIR:-/tmp}/foam-grasp-pycache}" \
python3 -m py_compile \
  "${PROJECT_ROOT}"/workspaces/app_ws/src/foam_grasp/foam_grasp/*.py \
  "${PROJECT_ROOT}"/workspaces/app_ws/src/foam_grasp/launch/*.launch.py \
  "${PROJECT_ROOT}"/workspaces/app_ws/src/foam_grasp/setup.py \
  "${PROJECT_ROOT}"/training/*.py

if grep -RnsE \
    --exclude-dir='__pycache__' \
    --include='*.py' --include='*.yaml' --include='*.xml' \
    '/home/rl|/Users/' \
    "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp" \
    "${PROJECT_ROOT}/training"; then
  echo "发现不可移植的绝对用户路径" >&2
  exit 1
fi

(cd "${PROJECT_ROOT}/workspaces/app_ws/src/foam_grasp" && \
  python3 setup.py --name --version)

echo "项目静态检查通过；没有启动ROS或发送机械臂命令。"
