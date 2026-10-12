#!/usr/bin/env bash
set -eo pipefail
PTP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
exec python3 "$PTP_DIR/monitor_ros.py" "$@"
