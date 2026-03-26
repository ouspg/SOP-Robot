"""
Base class for all voice chatbot ROS 2 nodes.

Consolidates the boilerplate shared by ``stt_node``, ``llm_node``, and
``tts_node``: config loading from a ROS parameter, status/log
publishing, and the ``main()`` entry-point pattern.
"""

from rclpy.node import Node
from std_msgs.msg import String

from voice_chatbot.config import Config
from voice_chatbot.platform_setup import setup_cuda

setup_cuda()


class VoiceNodeBase(Node):
    """Abstract base for the split voice chatbot nodes.

    Subclasses get:
    - ``_config`` loaded from the ``config_path`` ROS parameter.
    - ``_status_pub`` and ``_log_pub`` publishers on ``status`` / ``log``.
    - :meth:`_publish_status` and :meth:`_publish_log` helpers.

    Subclasses must call ``super().__init__(node_name)`` and then
    ``self._init_base()`` in their ``__init__``.
    """

    def _init_base(self) -> None:
        """Declare parameters, load config, and create status/log publishers."""
        self.declare_parameter("config_path", "config.json")
        self.declare_parameter("load_config_file", True)
        self._config = self._load_config()
        self._status_pub = self.create_publisher(String, "status", 10)
        self._log_pub = self.create_publisher(String, "log", 50)

    def _load_config(self) -> Config:
        config_path = str(self.get_parameter("config_path").value)
        load_file = bool(self.get_parameter("load_config_file").value)
        if load_file:
            cfg = Config.load(config_path)
            self.get_logger().info(f"Loaded config from '{config_path}'.")
            return cfg
        self.get_logger().info("Using in-code Config defaults.")
        return Config()

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f"status={status}")

    def _publish_log(self, message: str) -> None:
        self._log_pub.publish(String(data=message))
        self.get_logger().info(message)
