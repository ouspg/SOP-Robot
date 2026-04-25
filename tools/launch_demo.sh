#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$ROOT_DIR"

exec bash tools/with_ros_env.sh ros2 launch robot robot.fake.launch.py \
  use_rviz:=false \
  enable_chatbot_ui:=true \
  enable_hand_gestures:=true
