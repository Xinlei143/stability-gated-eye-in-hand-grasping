#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

import_manifest() {
  local manifest="$1"
  local destination="$2"
  if [[ -s "${manifest}" ]]; then
    mkdir -p "${destination}"
    vcs import --skip-existing "${destination}" < "${manifest}"
  else
    echo "缺少版本清单：${manifest}" >&2
    return 1
  fi
}

import_manifest "${PROJECT_ROOT}/dependencies/orbbec.repos" \
  "${PROJECT_ROOT}/workspaces/orbbec_ws/src"
piper_staging="${PROJECT_ROOT}/workspaces/piper_ws/_upstream"
import_manifest "${PROJECT_ROOT}/dependencies/piper.repos" \
  "${piper_staging}"
if [[ -d "${piper_staging}/piper_ros/src" ]]; then
  # piper_ws/src is a generated mirror of the exact upstream checkout.
  rsync -a --delete "${piper_staging}/piper_ros/src/" \
    "${PROJECT_ROOT}/workspaces/piper_ws/src/"
  piper_patch="$(
    realpath \
      "${PROJECT_ROOT}/patches/piper_ros/0001-use-kdl-kinematics.patch"
  )"
  require_file "${piper_patch}"
  patch \
    --batch \
    --forward \
    --strip=1 \
    --directory="${PROJECT_ROOT}/workspaces/piper_ws" \
    < "${piper_patch}"
fi

# The current machine may use the apt MoveIt packages instead of a source overlay.
if [[ -s "${PROJECT_ROOT}/dependencies/moveit.repos" ]]; then
  import_manifest "${PROJECT_ROOT}/dependencies/moveit.repos" \
    "${PROJECT_ROOT}/workspaces/moveit_ws/src"
fi
