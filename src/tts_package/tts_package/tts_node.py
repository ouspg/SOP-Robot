import os
import subprocess
from pathlib import Path

import rclpy
from TTS.api import TTS
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TTSService(Node):
    def __init__(self):
        super().__init__('tts_service')
        self.subscription = self.create_subscription(
            String,
            'chatbot_response',
            self.callback,
            10,
        )
        self.publisher = self.create_publisher(Bool, 'can_listen', 10)
        self.jaw = self.create_publisher(String, 'jaw_topic', 10)

        project_root = Path(
            os.environ.get('SOP_ROBOT_ROOT', Path.cwd())
        ).resolve()
        resource_dir = project_root / 'src' / 'tts_package' / 'resource'
        self.output = resource_dir / 'output.wav'
        self.synthesizer = TTS(
            model_path=str(resource_dir / 'model.pth'),
            config_path=str(resource_dir / 'config.json'),
        ).synthesizer
        self.get_logger().info('Finnish TTS ready.')

    def callback(self, message):
        self.publisher.publish(Bool(data=False))
        try:
            self.get_logger().info(
                f'Synthesizing: {message.data}'
            )
            wav = self.synthesizer.tts(message.data)
            self.synthesizer.save_wav(wav, str(self.output))
            self.jaw.publish(message)
            subprocess.run(
                ['pw-play', str(self.output)],
                check=True,
                timeout=120,
            )
        except Exception as error:
            self.get_logger().error(f'TTS playback failed: {error}')
        finally:
            self.publisher.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = TTSService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
