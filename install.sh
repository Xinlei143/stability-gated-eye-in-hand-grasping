#!/usr/bin/env bash
set -eo pipefail

# Full software deployment for a fresh Ubuntu 22.04 machine. It installs and
# builds software but never starts ROS, enables the Piper arm, or sends motion.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${PROJECT_ROOT}/scripts/bootstrap_new_machine.sh" \
  --install-system \
  --fetch-runtime \
  --install-udev \
  "$@"
