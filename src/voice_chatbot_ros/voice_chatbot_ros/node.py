"""
ROS 2 monolithic node (legacy) – runs the full pipeline in one process.

.. deprecated:: Prefer the split-node architecture (stt_node, llm_node,
   tts_node) which loads models in parallel and starts faster.
"""

import queue
import threading
import traceback

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_chatbot.audio_io import AudioIO
from voice_chatbot.llm import ChatLLM
from voice_chatbot.stt import SpeechToText
from voice_chatbot.tts_engine import TextToSpeech
from voice_chatbot.vad import VoiceActivityDetector
from voice_chatbot_ros._base import VoiceNodeBase


class VoiceChatbotNode(VoiceNodeBase):
    """Legacy monolithic chatbot node — use split nodes instead."""

    def __init__(self) -> None:
        super().__init__("voice_chatbot")
        self._init_base()

        self.declare_parameter("enable_voice_loop", False)
        self.declare_parameter("enable_tts_playback", True)
        self._enable_voice_loop = bool(self.get_parameter("enable_voice_loop").value)
        self._enable_tts_playback = bool(
            self.get_parameter("enable_tts_playback").value
        )

        self._assistant_pub = self.create_publisher(String, "~/assistant_text", 10)
        self._transcript_pub = self.create_publisher(String, "~/transcript", 10)
        self._user_sub = self.create_subscription(
            String, "~/user_text", self._on_user_text, 10
        )
        self._clear_srv = self.create_service(
            Trigger, "~/clear_history", self._on_clear_history
        )

        self._request_queue: queue.Queue[str] = queue.Queue()
        self._running = threading.Event()
        self._running.set()
        self._request_thread: threading.Thread | None = None
        self._voice_thread: threading.Thread | None = None

        self._audio: AudioIO | None = None
        self._vad: VoiceActivityDetector | None = None
        self._stt: SpeechToText | None = None
        self._tts: TextToSpeech | None = None
        self._llm: ChatLLM | None = None

        self._initialize_runtime()

    def _initialize_runtime(self) -> None:
        try:
            self._publish_status("initializing")

            self._publish_log("Initializing LLM...")
            self._llm = ChatLLM(self._config)

            if self._enable_tts_playback:
                self._publish_log("Initializing audio playback...")
                self._audio = AudioIO(self._config)
                self._publish_log("Initializing TTS...")
                self._tts = TextToSpeech(self._config)

            if self._enable_voice_loop:
                if self._audio is None:
                    self._audio = AudioIO(self._config)
                self._publish_log("Initializing VAD...")
                self._vad = VoiceActivityDetector(self._config)
                self._publish_log("Initializing Whisper STT...")
                self._stt = SpeechToText(self._config)
                self._audio.start_capture()
                self._voice_thread = threading.Thread(
                    target=self._voice_loop, name="voice-capture-loop", daemon=True
                )
                self._voice_thread.start()

            self._request_thread = threading.Thread(
                target=self._request_loop, name="chat-request-loop", daemon=True
            )
            self._request_thread.start()

            ready_state = "listening" if self._enable_voice_loop else "ready"
            self._publish_log("ROS 2 chatbot node ready.")
            self._publish_status(ready_state)
        except Exception as exc:
            self._publish_status("error")
            self._publish_log(f"Initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _on_user_text(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self._publish_log(f"Queued ROS user text: {text}")
            self._request_queue.put(text)

    def _on_clear_history(self, request, response):
        del request
        if self._llm is None:
            response.success = False
            response.message = "LLM is not initialized."
            return response
        self._llm.clear_history()
        self._publish_log("Conversation history cleared.")
        response.success = True
        response.message = "Conversation history cleared."
        return response

    def _voice_loop(self) -> None:
        assert self._audio and self._stt and self._vad
        while self._running.is_set():
            try:
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
                    self._publish_log(f"Transcript: {text}")
                    self._request_queue.put(text)
            except Exception:
                self._publish_status("error")
                self._publish_log("Voice loop failed.")
                self.get_logger().error(traceback.format_exc())

    def _request_loop(self) -> None:
        while self._running.is_set():
            try:
                user_text = self._request_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self._llm is None:
                    continue
                self._publish_status("llm_responding")
                response = self._llm.chat(user_text)
                self._assistant_pub.publish(String(data=response))
                self._publish_log(f"Assistant: {response}")

                if self._enable_tts_playback and self._tts is not None:
                    self._publish_status("speaking")
                    audio_out, sample_rate = self._tts.synthesize(response)
                    assert self._audio is not None
                    self._audio.play_audio(audio_out, sample_rate)
                    if self._enable_voice_loop and self._vad is not None:
                        self._audio.clear_queue()
                        self._vad.reset()

                next_state = "listening" if self._enable_voice_loop else "ready"
                self._publish_status(next_state)
            except Exception:
                self._publish_status("error")
                self._publish_log("Request processing failed.")
                self.get_logger().error(traceback.format_exc())

    def destroy_node(self) -> None:
        self._running.clear()
        if self._audio is not None:
            self._audio.close()
        for thread in [self._voice_thread, self._request_thread]:
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceChatbotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
