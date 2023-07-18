from TTS.utils.synthesizer import Synthesizer

from rclpy.node import Node

from tts_msgs.srv import StringToWav

import rclpy


class TTSService(Node):

    def __init__(self):
        super().__init__('TTS_service')
        self.srv = self.create_service(
            StringToWav, 'StringToWav', self.stringToWav_callback)
        self.synthetizer = Synthesizer(
            "src/tts_package/resource/model151k.pth",
            "src/tts_package/resource/config.json")
        self.output = "src/tts_package/resource/output.wav"
        self.get_logger().info("Service running...")

    def stringToWav_callback(self, request, response):
        self.get_logger().info("Incoming request to synthentize string: %s" % (request.data))
        try:
            wav = self.synthetizer.tts(request.data)
            self.synthetizer.save_wav(wav, self.output)
        except Exception:
            print(Exception)
            response.success = False
        else:
            response.success = True
        self.get_logger().info("Callback over. Service running...")
        return response


def main():
    rclpy.init()

    tts_service = TTSService()

    rclpy.spin(tts_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
