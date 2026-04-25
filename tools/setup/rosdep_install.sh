#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROJECT_ROOT"

if ! command -v rosdep >/dev/null 2>&1; then
    cat >&2 <<'EOF'
ERROR: rosdep is not installed or not on PATH.
Install it on Ubuntu 22.04 with:
  sudo apt update
  sudo apt install -y python3-rosdep
  sudo rosdep init
  rosdep update
EOF
    exit 1
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    cat >&2 <<'EOF'
ERROR: rosdep is not initialized.
Run:
  sudo rosdep init
  rosdep update
EOF
    exit 1
fi

# Pixi owns Python, ML, and the bulk of the ROS user-space packages in this
# workspace. rosdep is kept only for host-level packages that are intentionally
# left to Ubuntu, such as espeak.
PIXIOWNED_KEYS=(
    action_msgs
    ament_index_python
    builtin_interfaces
    control_msgs
    controller_manager
    cv_bridge
    hardware_interface
    launch
    launch_ros
    pluginlib
    python3-yaml
    rclcpp
    rclpy
    robot_state_publisher
    rosidl_default_generators
    rviz2
    sensor_msgs
    std_msgs
    std_srvs
    trajectory_msgs
    xacro
    xdotool
)

rosdep update
ROS_DISTRO=humble rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro humble \
    -r \
    -y \
    --skip-keys "$(printf '%s ' "${PIXIOWNED_KEYS[@]}")"
