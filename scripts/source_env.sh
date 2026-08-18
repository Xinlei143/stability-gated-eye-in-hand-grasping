#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用：source scripts/source_env.sh" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
source_complete_stack
echo "已加载 foam_grasp：${FOAM_GRASP_ROOT}"

