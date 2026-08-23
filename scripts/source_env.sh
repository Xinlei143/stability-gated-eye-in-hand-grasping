#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用：source scripts/source_env.sh" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source_complete_stack
echo "已加载 foam_grasp：${FOAM_GRASP_ROOT}"

gazebo_grasp_fix_lib="${FOAM_GRASP_ROOT}/.external/gazebo-grasp-install/lib"
if [[ -f "${gazebo_grasp_fix_lib}/libgazebo_grasp_fix.so" ]]; then
  export GAZEBO_PLUGIN_PATH="${gazebo_grasp_fix_lib}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
  export LD_LIBRARY_PATH="${gazebo_grasp_fix_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  echo "已加载 gazebo_grasp_fix：${gazebo_grasp_fix_lib}/libgazebo_grasp_fix.so"
fi
