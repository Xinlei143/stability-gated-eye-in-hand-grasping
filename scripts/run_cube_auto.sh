#!/usr/bin/env bash
set -eo pipefail

# Backward-compatible entry point for the previously validated cube workflow.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_object_auto.sh" cube
