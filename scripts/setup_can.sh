#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

sudo ip link set "${CAN_PORT}" down || true
sudo ip link set "${CAN_PORT}" type can bitrate "${CAN_BITRATE}"
sudo ip link set "${CAN_PORT}" up
ip -brief link show "${CAN_PORT}"
