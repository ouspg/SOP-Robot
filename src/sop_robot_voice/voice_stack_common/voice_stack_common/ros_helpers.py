"""Minimal ROS helpers shared by the split voice stack nodes."""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import String

from .config import VoiceChatConfig, default_config_path
from .contracts import LOG_TOPIC, PARAM_CONFIG_PATH, PARAM_LOAD_CONFIG_FILE, STATUS_TOPIC


class VoiceNodeBase(Node):
    """Shared ROS base class for nodes that publish stack status and logs."""

    def _init_base(self) -> None:
        self.declare_parameter(PARAM_CONFIG_PATH, str(self._default_config_path()))
        self.declare_parameter(PARAM_LOAD_CONFIG_FILE, True)
        self._config = self._load_config()
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._log_pub = self.create_publisher(String, LOG_TOPIC, 50)

    @staticmethod
    def _default_config_path() -> str:
        return str(default_config_path())

    def _load_config(self) -> VoiceChatConfig:
        config_path = str(self.get_parameter(PARAM_CONFIG_PATH).value)
        load_file = bool(self.get_parameter(PARAM_LOAD_CONFIG_FILE).value)
        if load_file:
            cfg = VoiceChatConfig.load(config_path)
            self.get_logger().info(f"Loaded config from '{config_path}'.")
            return cfg
        self.get_logger().info("Using in-code VoiceChatConfig defaults.")
        return VoiceChatConfig()

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f"status={status}")

    def _publish_log(self, message: str) -> None:
        self._log_pub.publish(String(data=message))
        self.get_logger().info(message)
