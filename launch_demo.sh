#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_Q="$(printf "%q" "$ROOT")"

run_tab() {
  local command_q
  command_q="$(printf "%q" "source install/local_setup.sh && $*")"
  gnome-terminal -- bash -lc "cd $ROOT_Q && pixi run bash -lc $command_q; exec bash"
}

run_tab ros2 launch robot robot.launch.py
run_tab ros2 run tts_package service
run_tab ros2 run speech_recognizer speech_recognizer_node
run_tab ros2 run qabot client
run_tab ros2 run hand_gestures hand_gestures_node
run_tab python client/unified_arms_client.py
run_tab ros2 launch face_tracker face_tracker.test.launch.py
run_tab ros2 run full_demo full_demo_node
run_tab ros2 run face_tracker_movement face_tracker_movement_node
run_tab ros2 run image_view image_view --ros-args -r image:=/face_tracker/image_face
