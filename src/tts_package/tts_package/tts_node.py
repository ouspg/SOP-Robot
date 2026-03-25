from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import simpleaudio as sa
from std_msgs.msg import Bool
from std_msgs.msg import String
from TTS.api import TTS


class TTSService(Node):

    def __init__(self):
        super().__init__('TTS_service')
        self.subscription = self.create_subscription(String, "chatbot_response", self.callback, 10)
        self.publisher = self.create_publisher(Bool, "can_listen", 10)
        self.jaw = self.create_publisher(String, "jaw_topic", 10)
        self.can_listen = Bool(data=True)
        self.cant_listen = Bool(data=False)

        package_share = Path(get_package_share_directory("tts_package"))
        default_model_path = package_share / "resource" / "model.pth"
        default_config_path = package_share / "resource" / "config.json"
        default_output_path = Path(tempfile.gettempdir()) / "sop_robot_tts_output.wav"

        model_path = (
            self.declare_parameter("model_path", str(default_model_path))
            .get_parameter_value()
            .string_value
        )
        config_path = (
            self.declare_parameter("config_path", str(default_config_path))
            .get_parameter_value()
            .string_value
        )
        output_path = (
            self.declare_parameter("output_path", str(default_output_path))
            .get_parameter_value()
            .string_value
        )

        self.output = Path(output_path)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.synthetizer = TTS(model_path=model_path, config_path=config_path).synthesizer

    def callback(self, msg):
        text = msg.data.strip()
        if not text:
            self.get_logger().warning("Received empty TTS request")
            return

        self.get_logger().info(f"Incoming request to synthesize string: {text}")
        self.executor.submit(self._synthesize_and_play, text)

    def _synthesize_and_play(self, text):
        self.publisher.publish(self.cant_listen)
        try:
            wav = self.synthetizer.tts(text)
            self.synthetizer.save_wav(wav, str(self.output))
            wave_obj = sa.WaveObject.from_wave_file(str(self.output))
            jaw_msg = String(data=text)
            self.jaw.publish(jaw_msg)
            play_obj = wave_obj.play()
            play_obj.wait_done()
        except FileNotFoundError as exc:
            self.get_logger().error(f"TTS resource file is missing: {exc}")
        except RuntimeError as exc:
            self.get_logger().error(f"TTS playback failed: {exc}")
        finally:
            self.publisher.publish(self.can_listen)

    def destroy_node(self):
        self.executor.shutdown(wait=True, cancel_futures=False)
        return super().destroy_node()


def main():
    rclpy.init()

    tts_service = TTSService()

    rclpy.spin(tts_service)

    tts_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
