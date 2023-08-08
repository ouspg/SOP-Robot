import simpleaudio as sa

from TTS.utils.synthesizer import Synthesizer

from rclpy.node import Node

from tts_msgs.srv import StringToWav

import rclpy


class TTSService(Node):

    # Initialize node
    def __init__(self):
        super().__init__('TTS_service')
        self.srv = self.create_service(
            StringToWav, 'StringToWav', self.stringToWav_callback)
        self.synthetizer = Synthesizer(
            "src/tts_package/resource/model.pth",
            "src/tts_package/resource/config.json")
        self.output = "src/tts_package/resource/output.wav"
        self.get_logger().info("Service running...")

    # Callback function. Waits for call and then synthetizes given request and plays synthetized speech.
    def stringToWav_callback(self, request, response):
        self.get_logger().info("Incoming request to synthentize string: %s" % (request.data))
        try:
            wav = self.synthetizer.tts(request.data)
            self.synthetizer.save_wav(wav, self.output)
            self.play_audio()
        except Exception as e:
            self.get_logger().info(f"Error happened: {str(e)}")
            response.success = False
        else:
            response.success = True
        self.get_logger().info("Callback over. Service running...")
        return response

    # Function that plays created .wav file.
    def play_audio(self):
        wave_obj = sa.WaveObject.from_wave_file(self.output)
        play_obj = wave_obj.play()
        play_obj.wait_done()

def main():
    rclpy.init()

    tts_service = TTSService()

    rclpy.spin(tts_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
