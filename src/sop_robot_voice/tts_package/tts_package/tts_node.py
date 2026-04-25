"""ROS2 TTS node for the SOP Robot voice stack."""

from __future__ import annotations

import queue
import threading
import traceback
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
import rclpy
from std_msgs.msg import Bool, String

from voice_stack_common.contracts import (
    STATUS_ERROR,
    STATUS_INITIALIZING,
    STATUS_READY,
    STATUS_SPEAKING,
)
from voice_stack_common.ros_helpers import VoiceNodeBase

from .topics import (
    CAN_LISTEN_OUTPUT_TOPIC,
    CANONICAL_INPUT_TOPIC,
    JAW_OUTPUT_TOPIC,
    LEGACY_CAN_LISTEN_OUTPUT_TOPIC,
    LEGACY_INPUT_TOPIC,
    TTS_DONE_OUTPUT_TOPIC,
)
from .tts_backend import TtsBackend


class TTSNode(VoiceNodeBase):
    """Synthesizes assistant replies and plays them back."""

    def __init__(self) -> None:
        super().__init__("tts_package")
        self._init_base()

        package_share = Path(get_package_share_directory("tts_package"))
        default_model_path = package_share / "resource" / "model.pth"
        default_config_path = package_share / "resource" / "config.json"
        default_output_path = Path(tempfile.gettempdir()) / "sop_robot_tts_output.wav"

        self.declare_parameter("model_path", str(default_model_path))
        self.declare_parameter("tts_config_path", str(default_config_path))
        self.declare_parameter("output_path", str(default_output_path))

        self._assistant_sub = self.create_subscription(
            String, CANONICAL_INPUT_TOPIC, self._on_text, 10
        )
        self._legacy_sub = self.create_subscription(
            String, LEGACY_INPUT_TOPIC, self._on_text, 10
        )
        self._tts_done_pub = self.create_publisher(String, TTS_DONE_OUTPUT_TOPIC, 10)
        self._can_listen_pub = self.create_publisher(
            Bool, CAN_LISTEN_OUTPUT_TOPIC, 10
        )
        self._legacy_can_listen_pub = self.create_publisher(
            Bool, LEGACY_CAN_LISTEN_OUTPUT_TOPIC, 10
        )
        self._jaw_pub = self.create_publisher(String, JAW_OUTPUT_TOPIC, 10)

        self._listen_enabled = Bool(data=True)
        self._listen_disabled = Bool(data=False)
        self._synth_queue: queue.Queue[str] = queue.Queue()
        self._running = threading.Event()
        self._running.set()
        self._worker_thread: threading.Thread | None = None
        self._backend: TtsBackend | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status(STATUS_INITIALIZING)
            self._publish_log("TTS node: initializing backend...")
            self._backend = TtsBackend(
                self._config,
                model_path=str(self.get_parameter("model_path").value),
                config_path=str(self.get_parameter("tts_config_path").value),
                output_path=str(self.get_parameter("output_path").value),
            )
            self._worker_thread = threading.Thread(
                target=self._synth_loop,
                name="tts-synth-loop",
                daemon=True,
            )
            self._worker_thread.start()

            self._publish_log("TTS node ready.")
            self._publish_can_listen(self._listen_enabled)
            self._publish_status(STATUS_READY)
        except Exception as exc:
            self._publish_status(STATUS_ERROR)
            self._publish_log(f"TTS node initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self._synth_queue.put(text)

    def _synth_loop(self) -> None:
        while self._running.is_set():
            try:
                text = self._synth_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                assert self._backend is not None
                self._publish_status(STATUS_SPEAKING)
                self._publish_can_listen(self._listen_disabled)
                self._jaw_pub.publish(String(data=text))
                self._backend.speak(text)
                self._tts_done_pub.publish(String(data="done"))
                self._publish_can_listen(self._listen_enabled)
                self._publish_status(STATUS_READY)
            except Exception:
                self._publish_can_listen(self._listen_enabled)
                self._publish_status(STATUS_ERROR)
                self._publish_log("TTS synthesis/playback failed.")
                self.get_logger().error(traceback.format_exc())

    def _publish_can_listen(self, msg: Bool) -> None:
        self._can_listen_pub.publish(msg)
        self._legacy_can_listen_pub.publish(msg)

    def destroy_node(self) -> None:
        self._running.clear()
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
