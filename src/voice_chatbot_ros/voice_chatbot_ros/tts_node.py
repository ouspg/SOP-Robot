"""
ROS 2 TTS node – text-to-speech synthesis and audio playback.

Subscribes to assistant text, synthesises speech, plays it, and
publishes ``tts_done`` so the STT node can reset its VAD.
"""

import queue
import threading
import traceback

import rclpy
from std_msgs.msg import Bool, String

from voice_chatbot.audio_io import AudioIO
from voice_chatbot.tts_engine import TextToSpeech
from voice_chatbot_ros._base import VoiceNodeBase


class VoiceTtsNode(VoiceNodeBase):
    def __init__(self) -> None:
        super().__init__("voice_tts")
        self._init_base()

        self._assistant_sub = self.create_subscription(
            String, "assistant_text", self._on_assistant_text, 10
        )
        self._tts_done_pub = self.create_publisher(String, "tts_done", 10)
        # Publish jaw movement text for jaw_movement_node (absolute topic)
        self._jaw_pub = self.create_publisher(String, "/jaw_topic", 10)
        # Broadcast speaking state so other nodes know when TTS is active
        self._can_listen_pub = self.create_publisher(Bool, "/can_listen", 10)

        self._synth_queue: queue.Queue[str] = queue.Queue()
        self._running = threading.Event()
        self._running.set()
        self._synth_thread: threading.Thread | None = None

        self._audio: AudioIO | None = None
        self._tts: TextToSpeech | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status("initializing")
            self._publish_log("TTS node: initializing audio playback...")
            self._audio = AudioIO(self._config)

            self._publish_log("TTS node: initializing TTS engine...")
            self._tts = TextToSpeech(self._config)

            self._synth_thread = threading.Thread(
                target=self._synth_loop, name="tts-synth-loop", daemon=True
            )
            self._synth_thread.start()

            self._publish_log("TTS node ready.")
            self._publish_status("ready")
        except Exception as exc:
            self._publish_status("error")
            self._publish_log(f"TTS node initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _on_assistant_text(self, msg: String) -> None:
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
                assert self._tts is not None and self._audio is not None
                self._publish_status("speaking")
                # Signal that the robot is speaking
                self._can_listen_pub.publish(Bool(data=False))
                # Send text to jaw movement node for lip sync
                self._jaw_pub.publish(String(data=text))
                audio_out, sample_rate = self._tts.synthesize(text)
                self._audio.play_audio(audio_out, sample_rate)
                self._tts_done_pub.publish(String(data="done"))
                # Signal that speaking is done
                self._can_listen_pub.publish(Bool(data=True))
                self._publish_status("ready")
            except Exception:
                self._publish_status("error")
                self._publish_log("TTS synthesis/playback failed.")
                self.get_logger().error(traceback.format_exc())

    def destroy_node(self) -> None:
        self._running.clear()
        if self._audio is not None:
            self._audio.close()
        if self._synth_thread is not None and self._synth_thread.is_alive():
            self._synth_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceTtsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
