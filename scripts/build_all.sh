#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

build_workspace() {
  local workspace="$1"
  local mode="${2:-release}"
  if ! find "${workspace}/src" -name package.xml -print -quit | grep -q .; then
    echo "跳过无ROS包的工作空间：${workspace}"
    return 0
  fi
  if [[ "${mode}" != "symlink" \
      && "${REBUILD_VENDOR:-false}" != "true" \
      && -f "${workspace}/install/setup.bash" ]]; then
    echo "使用已构建的工作空间：${workspace}"
    # shellcheck disable=SC1090
    source "${workspace}/install/setup.bash"
    return 0
  fi
  if [[ "${mode}" == "symlink" ]]; then
    "${RUNTIME_VENV}/bin/python" -c \
      'from colcon_core.command import main; raise SystemExit(main())' \
      --log-base "${workspace}/log" build --base-paths "${workspace}/src" \
      --build-base "${workspace}/build" \
      --install-base "${workspace}/install" \
      --symlink-install
  else
    colcon --log-base "${workspace}/log" build --base-paths "${workspace}/src" \
      --build-base "${workspace}/build" \
      --install-base "${workspace}/install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
  # shellcheck disable=SC1090
  source "${workspace}/install/setup.bash"
}

build_workspace "${PROJECT_ROOT}/workspaces/orbbec_ws"
build_workspace "${PROJECT_ROOT}/workspaces/piper_ws"
build_workspace "${PROJECT_ROOT}/workspaces/moveit_ws"

if [[ ! -f "${RUNTIME_VENV}/bin/activate" ]]; then
  echo "缺少运行环境，请先运行 scripts/install_python_runtime.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${RUNTIME_VENV}/bin/activate"
export PYTHONNOUSERSITE=1
# Calling the system colcon_core through the venv interpreter makes generated
# ROS console-script shebangs point at the NumPy<2/PyTorch interpreter without
# downloading a second copy of colcon from PyPI.
"${RUNTIME_VENV}/bin/python" -c \
  'from colcon_core.command import main; print("venv colcon_core import: OK")'
build_workspace "${PROJECT_ROOT}/workspaces/app_ws" symlink

echo "全部工作空间构建完成。"
