from TTS.api import TTS
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import simpleaudio as sa
import rclpy

class TTSService(Node):

    def __init__(self):
        super().__init__('TTS_service')
        self.subscription = self.create_subscription(String, "chatbot_response", self.callback, 10)
        self.publisher_ = self.create_publisher(Bool, "can_listen", 10)
        self.jaw = self.create_publisher(String, "jaw_topic", 10)
        self.can_listen = Bool(data = True)
        self.cant_listen = Bool(data = False)
        self.synthetizer = TTS(
            model_path="./src/tts_package/resource/model.pth",
            config_path="./src/tts_package/resource/config.json").synthesizer
        self.output = "./src/tts_package/resource/output.wav"


    def callback(self, msg):
        if(msg.data != '.'){
            try:
                self.get_logger().info("Incoming request to synthentize string: %s" % (msg.data))
                wav = self.synthetizer.tts(msg.data)
                self.synthetizer.save_wav(wav, self.output)
                self.play_audio(msg)
            except Exception:
                self.get_logger().info("Error happened")
            else:
                pass
        } else {
            self.get_logger().info("Ei vastausta.")
        }


    def play_audio(self, msg):
        self.publisher_.publish(self.cant_listen)
        wave_obj = sa.WaveObject.from_wave_file(self.output)
        self.jaw.publish(msg)
        play_obj = wave_obj.play()
        play_obj.wait_done()     
        self.publisher_.publish(self.can_listen) 
    
def main():
    rclpy.init()

    tts_service = TTSService()

    rclpy.spin(tts_service)

    tts_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()