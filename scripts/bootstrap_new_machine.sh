#!/usr/bin/env bash
set -eo pipefail

# Prepare an extracted project on a new Ubuntu computer. This script never
# starts ROS nodes, enables the arm, or sends motion commands.
# System packages are optional because installing them requires sudo/network.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

install_system=false
fetch_runtime=false
install_udev=false
for arg in "$@"; do
  case "${arg}" in
    --install-system) install_system=true ;;
    --fetch-runtime) fetch_runtime=true ;;
    --install-udev) install_udev=true ;;
    -h|--help)
      echo "用法：$0 [--install-system] [--fetch-runtime] [--install-udev]"
      echo "  --install-system  配置ROS源并通过apt安装Ubuntu/ROS依赖"
      echo "  --fetch-runtime   从固定GitHub Release下载模型和标定"
      echo "  --install-udev    安装Orbbec相机udev规则"
      exit 0
      ;;
    *)
      echo "未知参数：${arg}" >&2
      exit 2
      ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别Ubuntu系统。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "当前系统为 ${PRETTY_NAME:-unknown}；已验证平台是Ubuntu 22.04。" >&2
  exit 1
fi

"${PROJECT_ROOT}/scripts/preflight_new_machine.sh"

if [[ "${install_system}" == "true" ]]; then
  "${PROJECT_ROOT}/scripts/install_system_dependencies.sh"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "缺少 /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  echo "请先运行：./scripts/bootstrap_new_machine.sh --install-system" >&2
  exit 1
fi

for workspace in orbbec_ws piper_ws; do
  source_dir="${PROJECT_ROOT}/workspaces/${workspace}/src"
  if ! find "${source_dir}" -name package.xml -print -quit 2>/dev/null \
      | grep -q .; then
    echo "${workspace}源码不存在，将根据dependencies/*.repos导入。"
    "${PROJECT_ROOT}/scripts/import_vendor.sh"
    break
  fi
done
"${PROJECT_ROOT}/scripts/verify_vendor_locks.sh"

if [[ "${fetch_runtime}" == "true" ]]; then
  "${PROJECT_ROOT}/scripts/fetch_runtime_assets.sh"
fi

require_file "${CHECKPOINT_PATH}"
require_file "${CALIBRATION_PATH}"

"${PROJECT_ROOT}/scripts/install_python_runtime.sh"
"${PROJECT_ROOT}/scripts/verify_runtime_assets.sh"
"${PROJECT_ROOT}/scripts/install_ros_dependencies.sh"
REBUILD_VENDOR=true "${PROJECT_ROOT}/scripts/build_all.sh"
"${PROJECT_ROOT}/scripts/validate_project.sh"
"${PROJECT_ROOT}/scripts/project_status.sh"
"${PROJECT_ROOT}/scripts/post_install_smoke.sh"

if [[ "${install_udev}" == "true" ]]; then
  "${PROJECT_ROOT}/scripts/install_orbbec_udev.sh"
fi

echo
echo "新电脑项目环境已准备完成，未启动ROS或机械臂。"
if [[ "${install_udev}" == "true" ]]; then
  echo "下一步：拔插DaBai DC1，再运行 scripts/setup_can.sh 和只读验收。"
else
  echo "下一步：安装Orbbec udev规则，拔插相机，再运行 scripts/setup_can.sh。"
fi
if [[ -f /var/run/reboot-required ]]; then
  echo "系统提示需要重启；请先重启，再连接和启动硬件。"
fi
