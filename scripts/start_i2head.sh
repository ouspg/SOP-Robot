#!/bin/bash
# start_i2head.sh - Start i2Head bridge and FACS expression node
# Usage: ./start_i2head.sh [config_dir]

CONFIG_DIR="${1:-/vagrant/config}"

cleanup() {
    echo "Shutting down..."
    kill $BRIDGE_PID $FACS_PID 2>/dev/null
    wait
}

trap cleanup EXIT INT TERM

echo "Starting i2Head bridge (config: $CONFIG_DIR/i2head.yaml)..."
python3 /vagrant/client/i2head_bridge_node.py \
    --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p baud_rate:=115200 \
    -p joint_config:="$CONFIG_DIR/i2head.yaml" \
    -p topic_name:=/i2head/joint_commands &
BRIDGE_PID=$!

sleep 1

echo "Starting i2Head FACS node (config: $CONFIG_DIR/i2head_facs.yaml)..."
python3 /vagrant/client/i2head_facs_node.py \
    --ros-args \
    -p facs_config:="$CONFIG_DIR/i2head_facs.yaml" \
    -p au_topic:=/facs/au \
    -p expression_topic:=/facs/expression \
    -p command_topic:=/i2head/joint_commands &
FACS_PID=$!

echo "i2Head system started."
echo ""
echo "Trigger expression:   ros2 topic pub /facs/expression std_msgs/String \"data: smile\""
echo "Trigger AU directly:  ros2 topic pub /facs/au interface/msg/FACS_au \"{au_id: 12, intensity: 0.8, priority: 2, duration_nanosecs: 300000000}\""
echo "Press Ctrl+C to stop."

wait
