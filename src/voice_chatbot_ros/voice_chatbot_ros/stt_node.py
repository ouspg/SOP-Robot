"""
ROS 2 STT node – microphone capture, VAD, and speech-to-text.

Publishes transcribed user text to the LLM node.  Subscribes to
``tts_done`` to clear the mic buffer and reset VAD after playback.
"""

import threading
import traceback

import rclpy
from std_msgs.msg import Bool, String

from voice_chatbot.audio_io import AudioIO
from voice_chatbot.stt import SpeechToText
from voice_chatbot.vad import VoiceActivityDetector
from voice_chatbot_ros._base import VoiceNodeBase


class VoiceSttNode(VoiceNodeBase):
    def __init__(self) -> None:
        super().__init__("voice_stt")
        self._init_base()

        self._user_text_pub = self.create_publisher(String, "user_text", 10)
        self._transcript_pub = self.create_publisher(String, "transcript", 10)
        self._tts_done_sub = self.create_subscription(
            String, "tts_done", self._on_tts_done, 10
        )
        # Allow external nodes (e.g. full_demo) to enable/disable listening
        self._can_listen_sub = self.create_subscription(
            Bool, "can_listen", self._on_can_listen, 10
        )

        self._running = threading.Event()
        self._running.set()
        self._listening_enabled = threading.Event()
        self._listening_enabled.set()  # listening enabled by default
        self._voice_thread: threading.Thread | None = None

        self._audio: AudioIO | None = None
        self._vad: VoiceActivityDetector | None = None
        self._stt: SpeechToText | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status("initializing")
            self._publish_log("STT node: initializing audio capture...")
            self._audio = AudioIO(self._config)

            self._publish_log("STT node: initializing VAD...")
            self._vad = VoiceActivityDetector(self._config)

            self._publish_log("STT node: initializing Whisper STT...")
            self._stt = SpeechToText(self._config)

            self._audio.start_capture()
            self._voice_thread = threading.Thread(
                target=self._voice_loop, name="voice-capture-loop", daemon=True
            )
            self._voice_thread.start()

            self._publish_log("STT node ready.")
            self._publish_status("listening")
        except Exception as exc:
            self._publish_status("error")
            self._publish_log(f"STT node initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _on_can_listen(self, msg: Bool) -> None:
        if msg.data:
            self._listening_enabled.set()
            if self._audio is not None:
                self._audio.clear_queue()
            if self._vad is not None:
                self._vad.reset()
            self._publish_log("Listening enabled by external node.")
            self._publish_status("listening")
        else:
            self._listening_enabled.clear()
            self._publish_log("Listening disabled by external node.")
            self._publish_status("paused")

    def _on_tts_done(self, msg: String) -> None:
        del msg
        if self._audio is not None:
            self._audio.clear_queue()
        if self._vad is not None:
            self._vad.reset()
        if self._listening_enabled.is_set():
            self._publish_status("listening")

    def _voice_loop(self) -> None:
        assert self._audio and self._stt and self._vad

        while self._running.is_set():
            try:
                # Wait until listening is enabled
                if not self._listening_enabled.is_set():
                    # Drain audio while paused to prevent stale buffer
                    self._audio.get_audio_chunk(timeout=0.1)
                    continue

                chunk = self._audio.get_audio_chunk(timeout=0.1)
                if chunk is None:
                    continue

                event, audio_data = self._vad.process_chunk(chunk)
                if event == "speech_start":
                    self._publish_status("speech_detected")
                elif event == "speech_end" and audio_data is not None:
                    self._publish_status("transcribing")
                    text = self._stt.transcribe(audio_data)
                    if not text or text.isspace():
                        self._publish_status("listening")
                        continue
                    self._transcript_pub.publish(String(data=text))
                    self._user_text_pub.publish(String(data=text))
                    self._publish_log(f"Transcript: {text}")
            except Exception:
                self._publish_status("error")
                self._publish_log("STT voice loop failed.")
                self.get_logger().error(traceback.format_exc())

    def destroy_node(self) -> None:
        self._running.clear()
        if self._audio is not None:
            self._audio.close()
        if self._voice_thread is not None and self._voice_thread.is_alive():
            self._voice_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceSttNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
