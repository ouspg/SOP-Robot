"""ROS2 ASR node for the SOP Robot voice stack."""

from __future__ import annotations

import threading
import traceback

import rclpy
from rclpy.timer import Timer
from std_msgs.msg import Bool, String
from voice_stack_common.contracts import (
    CAN_LISTEN_TOPIC,
    STATUS_ERROR,
    STATUS_INITIALIZING,
    STATUS_LISTENING,
    STATUS_READY,
    STATUS_SPEECH_DETECTED,
    STATUS_TRANSCRIBING,
    TRANSCRIPT_TOPIC,
    TTS_DONE_TOPIC,
    USER_TEXT_TOPIC,
)

from .asr_base import AsrNodeBase
from .audio_io import AudioIO
from .stt import SpeechToText
from .vad import VoiceActivityDetector


class AsrNode(AsrNodeBase):
    """Microphone capture, VAD, and speech-to-text."""

    def __init__(self) -> None:
        super().__init__("asr_package")
        self._init_base()

        self._user_text_pub = self.create_publisher(String, USER_TEXT_TOPIC, 10)
        self._transcript_pub = self.create_publisher(String, TRANSCRIPT_TOPIC, 10)
        self._can_listen_sub = self.create_subscription(
            Bool, CAN_LISTEN_TOPIC, self._on_can_listen, 10
        )
        self._tts_done_sub = self.create_subscription(String, TTS_DONE_TOPIC, self._on_tts_done, 10)
        self.declare_parameter("test_mode", False)
        self._test_mode = bool(self.get_parameter("test_mode").value)

        self._running = threading.Event()
        self._running.set()
        self._listening_enabled = True
        self._voice_thread: threading.Thread | None = None
        self._test_status_timer: Timer | None = None

        self._audio: AudioIO | None = None
        self._vad: VoiceActivityDetector | None = None
        self._stt: SpeechToText | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status(STATUS_INITIALIZING)
            if self._test_mode:
                self._publish_log("ASR node: test mode enabled; audio capture disabled.")
                self._test_status_timer = self.create_timer(1.0, self._publish_test_status)
                self._publish_test_status()
                return

            self._publish_log("ASR node: initializing audio capture...")
            self._audio = AudioIO(self._config)

            self._publish_log("ASR node: initializing VAD...")
            self._vad = VoiceActivityDetector(self._config)

            self._publish_log(
                f"ASR node: initializing STT backend '{self._config.whisper_backend}'..."
            )
            self._stt = SpeechToText(self._config)

            self._audio.start_capture()
            self._voice_thread = threading.Thread(
                target=self._voice_loop,
                name="voice-capture-loop",
                daemon=True,
            )
            self._voice_thread.start()

            self._publish_log("ASR node ready.")
            self._publish_status(STATUS_LISTENING)
        except Exception as exc:
            self._publish_status(STATUS_ERROR)
            self._publish_log(f"ASR node initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _publish_test_status(self) -> None:
        self._publish_status(STATUS_LISTENING if self._listening_enabled else STATUS_READY)

    def _reset_capture_state(self) -> None:
        if self._audio is not None:
            self._audio.clear_queue()
        if self._vad is not None:
            self._vad.reset()

    def _on_can_listen(self, msg: Bool) -> None:
        self._listening_enabled = bool(msg.data)
        self._reset_capture_state()
        if self._listening_enabled:
            self._publish_log("ASR listen gate enabled.")
            self._publish_status(STATUS_LISTENING)
        else:
            self._publish_log("ASR listen gate disabled.")
            self._publish_status(STATUS_READY)

    def _on_tts_done(self, msg: String) -> None:
        del msg
        self._reset_capture_state()
        self._publish_status(STATUS_LISTENING if self._listening_enabled else STATUS_READY)

    def _voice_loop(self) -> None:
        assert self._audio is not None
        assert self._stt is not None
        assert self._vad is not None

        while self._running.is_set():
            try:
                chunk = self._audio.get_audio_chunk(timeout=0.1)
                if chunk is None:
                    continue

                if not self._listening_enabled:
                    self._audio.clear_queue()
                    continue

                event, audio_data = self._vad.process_chunk(chunk)
                if event == "speech_start":
                    self._publish_status(STATUS_SPEECH_DETECTED)
                elif event == "speech_end" and audio_data is not None:
                    self._publish_status(STATUS_TRANSCRIBING)
                    text = self._stt.transcribe(audio_data)
                    if not text or text.isspace():
                        self._publish_status(STATUS_LISTENING)
                        continue
                    self._transcript_pub.publish(String(data=text))
                    self._user_text_pub.publish(String(data=text))
                    self._publish_log(f"Transcript: {text}")
                    self._publish_status(STATUS_LISTENING)
            except Exception:
                self._publish_status(STATUS_ERROR)
                self._publish_log("ASR voice loop failed.")
                self.get_logger().error(traceback.format_exc())

    def destroy_node(self) -> None:
        self._running.clear()
        if self._test_status_timer is not None:
            self._test_status_timer.cancel()
        if self._audio is not None:
            self._audio.close()
        if self._voice_thread is not None and self._voice_thread.is_alive():
            self._voice_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AsrNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
