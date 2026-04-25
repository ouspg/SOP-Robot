#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

exec bash tools/with_ros_env.sh ros2 launch robot robot.launch.py "$@"
