#!/usr/bin/python3

from pathlib import Path
from threading import Lock

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import Bool
from std_msgs.msg import String


class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer_node')
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)
        self.subscription = self.create_subscription(Bool, "can_listen", self.callback, 10)

        default_model_path = Path(get_package_share_directory("speech_recognizer")) / "resource" / "whisper-model.bin"
        self.model = (
            self.declare_parameter("model_path", str(default_model_path))
            .get_parameter_value()
            .string_value
        )
        self.language = (
            self.declare_parameter("language", "fi")
            .get_parameter_value()
            .string_value
        )
        self.phrase_time_limit = (
            self.declare_parameter("phrase_time_limit", 10.0)
            .get_parameter_value()
            .double_value
        )
        ambient_noise_duration = (
            self.declare_parameter("ambient_noise_duration", 5.0)
            .get_parameter_value()
            .double_value
        )

        self.r = sr.Recognizer()
        self.m = sr.Microphone()
        self._listen_lock = Lock()
        self._listening_enabled = False
        self._stop_listening = None

        with self.m as source:
            self.r.adjust_for_ambient_noise(source, duration=ambient_noise_duration)

        if not Path(self.model).exists():
            self.get_logger().warning(
                f"Whisper model file was not found at '{self.model}'. "
                "Set the 'model_path' parameter to a valid model before enabling listening."
            )

        self.set_listening_enabled(True)

    def callback(self, msg):
        self.set_listening_enabled(msg.data)

    def set_listening_enabled(self, enabled):
        with self._listen_lock:
            if enabled and not self._listening_enabled:
                self._stop_listening = self.r.listen_in_background(
                    self.m,
                    self.listen,
                    phrase_time_limit=self.phrase_time_limit,
                )
                self._listening_enabled = True
                self.get_logger().info("Speech recognizer listening enabled")
            elif not enabled and self._listening_enabled:
                if self._stop_listening is not None:
                    self._stop_listening(wait_for_stop=False)
                    self._stop_listening = None
                self._listening_enabled = False
                self.get_logger().info("Speech recognizer listening disabled")

    def listen(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        try:
            recognize_whisper = getattr(recognizer, "recognize_whisper", None)
            if not callable(recognize_whisper):
                raise RuntimeError(
                    "speech_recognition.Recognizer.recognize_whisper is unavailable. "
                    "Install a SpeechRecognition build with Whisper support."
                )
            text = recognize_whisper(audio, model=self.model, language=self.language)
            self.get_logger().info(f"Recognized speech: {text}")
            req = String()
            req.data = text
            self.publisher_.publish(req)
        except sr.UnknownValueError:
            self.get_logger().warning("Could not understand audio")
        except sr.RequestError as exc:
            self.get_logger().error(f"Speech recognition request failed: {exc}")
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

    def destroy_node(self):
        self.set_listening_enabled(False)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SpeechRecognizerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
