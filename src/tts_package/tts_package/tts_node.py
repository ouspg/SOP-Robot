# Copyright 2026 Aapo2001
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
