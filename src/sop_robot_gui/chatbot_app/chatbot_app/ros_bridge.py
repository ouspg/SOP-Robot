"""ROS2 bridge between the split voice stack and the PySide6 GUI."""

from __future__ import annotations

import rclpy
from PySide6.QtCore import QObject, QThread, Signal
from rclpy.node import Node as RosNode
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_stack_common.contracts import (
    ASSISTANT_TEXT_TOPIC,
    CLEAR_HISTORY_SERVICE,
    LOG_TOPIC,
    STATUS_TOPIC,
    TRANSCRIPT_TOPIC,
    USER_TEXT_TOPIC,
)

from sop_robot_common.contracts import ARM_ACTION_TOPIC
from sop_robot_common.contracts import (
    HEAD_GESTURE_TOPIC,
    LEFT_HAND_GESTURE_TOPIC,
    RIGHT_HAND_GESTURE_TOPIC,
)


class _RosSpinThread(QThread):
    """Spin the ROS node in a background thread."""

    def __init__(self, node: RosNode, parent: QObject | None = None):
        super().__init__(parent)
        self._node = node
        self._running = True

    def run(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def stop(self) -> None:
        self._running = False


class RosBridge(QObject):
    """Bridge ROS topics and services to Qt signals."""

    log_received = Signal(str)
    status_received = Signal(str)
    chat_message = Signal(str, str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._node: RosNode | None = None
        self._spin_thread: _RosSpinThread | None = None
        self._user_pub = None
        self._arm_action_pub = None
        self._head_command_pub = None
        self._left_hand_command_pub = None
        self._right_hand_command_pub = None
        self._clear_client = None

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    @property
    def node(self) -> RosNode | None:
        return self._node

    def connect_ros(self) -> None:
        if self._node is not None:
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = RosNode("chatbot_app_bridge")

        self._node.create_subscription(String, LOG_TOPIC, self._on_log, 50)
        self._node.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self._node.create_subscription(String, TRANSCRIPT_TOPIC, self._on_transcript, 10)
        self._node.create_subscription(String, ASSISTANT_TEXT_TOPIC, self._on_assistant, 10)

        self._user_pub = self._node.create_publisher(String, USER_TEXT_TOPIC, 10)
        self._arm_action_pub = self._node.create_publisher(String, ARM_ACTION_TOPIC, 10)
        self._head_command_pub = self._node.create_publisher(String, HEAD_GESTURE_TOPIC, 10)
        self._left_hand_command_pub = self._node.create_publisher(
            String,
            LEFT_HAND_GESTURE_TOPIC,
            10,
        )
        self._right_hand_command_pub = self._node.create_publisher(
            String,
            RIGHT_HAND_GESTURE_TOPIC,
            10,
        )
        self._clear_client = self._node.create_client(Trigger, CLEAR_HISTORY_SERVICE)

        self._spin_thread = _RosSpinThread(self._node)
        self._spin_thread.start()

    def disconnect_ros(self) -> None:
        if self._node is None:
            return
        if self._spin_thread is not None:
            self._spin_thread.stop()
            self._spin_thread.wait(3000)
            self._spin_thread = None
        self._user_pub = None
        self._arm_action_pub = None
        self._head_command_pub = None
        self._left_hand_command_pub = None
        self._right_hand_command_pub = None
        self._clear_client = None
        self._node.destroy_node()
        self._node = None
        rclpy.try_shutdown()

    def send_text(self, text: str) -> None:
        if self._user_pub is not None:
            self._user_pub.publish(String(data=text))
            self.chat_message.emit("user", text)

    def send_arm_action(self, action: str) -> None:
        command = action.strip().lower()
        if not command:
            return
        if self._arm_action_pub is not None:
            self._arm_action_pub.publish(String(data=command))
            self.log_received.emit(f"Arm action sent: {command}")

    def send_head_command(self, command: str) -> None:
        self._publish_command(self._head_command_pub, command, "Head command")

    def send_left_hand_command(self, command: str) -> None:
        self._publish_command(self._left_hand_command_pub, command, "Left hand command")

    def send_right_hand_command(self, command: str) -> None:
        self._publish_command(self._right_hand_command_pub, command, "Right hand command")

    def _publish_command(self, publisher, command: str, label: str) -> None:
        normalized = command.strip().lower()
        if not normalized or publisher is None:
            return
        publisher.publish(String(data=normalized))
        self.log_received.emit(f"{label} sent: {normalized}")

    def clear_history(self) -> None:
        if self._clear_client is None:
            return
        if self._clear_client.service_is_ready():
            self._clear_client.call_async(Trigger.Request())
            self.log_received.emit("Keskusteluhistoria tyhjennetty.")
            return
        self.log_received.emit("Clear-history palvelu ei ole viela valmis.")

    def _on_log(self, msg: String) -> None:
        self.log_received.emit(msg.data)

    def _on_status(self, msg: String) -> None:
        self.status_received.emit(msg.data)

    def _on_transcript(self, msg: String) -> None:
        self.chat_message.emit("user", msg.data)

    def _on_assistant(self, msg: String) -> None:
        self.chat_message.emit("assistant", msg.data)
