#!/usr/bin/env bash
set -eo pipefail

# Resolve apt dependencies for all imported source workspaces. This never
# starts ROS nodes or communicates with the camera/robot.
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

source_paths=()
for workspace in orbbec_ws piper_ws moveit_ws app_ws; do
  candidate="${PROJECT_ROOT}/workspaces/${workspace}/src"
  if find "${candidate}" -name package.xml -print -quit 2>/dev/null \
      | grep -q .; then
    source_paths+=("${candidate}")
  fi
done

if (( ${#source_paths[@]} == 0 )); then
  echo "没有发现可供rosdep检查的ROS源码。" >&2
  exit 1
fi

# ament_python is supplied by the ROS installation itself. The Piper MoveIt
# configs declare warehouse_ros_mongo only as an optional RViz warehouse
# backend; the grasp runtime does not use MongoDB, and Humble does not publish
# that binary package on every supported mirror. Keep both out of the
# mandatory fresh-machine dependency set.
rosdep install \
  --from-paths "${source_paths[@]}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  --skip-keys "ament_python warehouse_ros_mongo" \
  -r -y

echo "ROS源码依赖安装完成。"
