#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

exec bash tools/with_ros_env.sh ros2 launch robot robot.launch.py \
  enable_chatbot_ui:=false \
  enable_calibration_ui:=true \
  enable_voice_stack:=false \
  enable_face_tracker:=false \
  enable_face_tracker_movement:=true \
  enable_full_demo:=false \
  enable_ros2graph_ui:=false \
  use_rviz:=false \
  "$@"
