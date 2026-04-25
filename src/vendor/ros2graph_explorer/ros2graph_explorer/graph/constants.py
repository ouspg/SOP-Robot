INTERNAL_NODE_NAMES = {
    "/ros2graph_explorer_metrics_probe",
    "ros2graph_explorer_metrics_probe",
    "/ros2graph_explorer",
    "ros2graph_explorer",
    "/ros2cli_daemon",
    "ros2cli_daemon",
}

INTERNAL_NODE_PREFIXES = (
    "/tf_listener",
    "/tf2_buffer",
    "/tf_static_listener",
    "/transform_listener",
    "/_ros2cli_daemon",
    "_ros2cli_daemon",
    "ros2cli_daemon_",
    "/ros2graph_explorer_metrics_probe_",
    "ros2graph_explorer_metrics_probe_",
)

DEPRECATED_CONTROLLER_STATE_SUFFIX = "_controller/state"
DEPRECATED_CONTROLLER_STATE_TYPE = "control_msgs/msg/JointTrajectoryControllerState"


def is_deprecated_controller_state_topic(topic_name: str, type_names: tuple[str, ...]) -> bool:
    """Return true for ros2_control's deprecated JointTrajectoryController ~/state topics."""
    return (
        topic_name.endswith(DEPRECATED_CONTROLLER_STATE_SUFFIX)
        and DEPRECATED_CONTROLLER_STATE_TYPE in type_names
    )


__all__ = [
    "INTERNAL_NODE_NAMES",
    "INTERNAL_NODE_PREFIXES",
    "is_deprecated_controller_state_topic",
]
