#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROJECT_ROOT"

HOST="$ROS2GRAPH_WEB_HOST"
PORT="$ROS2GRAPH_WEB_PORT"
UPDATE_INTERVAL="$ROS2GRAPH_UPDATE_INTERVAL"

echo "ros2graph_explorer web UI: http://${HOST}:${PORT}/"
exec bash tools/with_ros_env.sh \
    ros2 run ros2graph_explorer ros2graph_explorer \
    --ros-args \
    --log-level warn \
    -p output_format:=none \
    -p update_interval:="${UPDATE_INTERVAL}" \
    -p web_enable:=true \
    -p web_host:="${HOST}" \
    -p web_port:="${PORT}" \
    "$@"
