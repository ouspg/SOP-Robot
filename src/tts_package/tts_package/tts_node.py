from TTS.utils.synthetizer import Synthetizer

from rclpy.node import Node

from tts_package.srv import StringToWav

import rclpy
from rclpy.node import Node


class TTSService(Node):

    def __init__(self):
        super().__init__('TTS_service')
        self.srv = self.create_service(StringToWav, 'StringToWav', self.stringToWav_callback)
        self.synthetizer = Synthetizer(
            "./resource/model.pth",
            "./resource/config.json")
        self.output = "./resource/output"



    def stringToWav_callback(self, request, response):
        try:
            wav = self.synthetizer.tts(sentence)
            self.synthetizer.save_wav(wav, output)
        except Exception:
            response.success = False
        else:
            response.success = True
        self.get_logger().info("Incoming request to synthentize string: %s" % (request.data))
        return response


def main():
    rclpy.init()

    TTSService = TTSService()

    rclpy.spin(TTSService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()