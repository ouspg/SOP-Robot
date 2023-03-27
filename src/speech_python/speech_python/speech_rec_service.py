import rclpy
from rclpy.node import Node

import speech_recognition as sr

from msg_interface.srv import SpeechRec

class SpeechRecServer(Node):

    def __init__(self):
        super().__init__('speech_rec_server')
        self.server = self.create_service(SpeechRec, "speech_rec_server", self.callback_speech_rec)
        self.get_logger().info("SpeechRecServer started...")

    def callback_speech_rec(self, request, response):

        r = sr.Recognizer()
        
        with sr.Microphone(device_index=0, sample_rate=16000) as source:

            self.get_logger().info("Say something!")
            audio = r.listen(source)
            
            try:
                # for testing purposes, we're just using the default API key
                # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                # instead of `r.recognize_google(audio)`
                self.recognized_data = r.recognize_google(audio, language="fi")
                self.all_data = r.recognize_google(audio, language="fi", show_all=True)

                self.get_logger().info("Google Speech Recognition thinks you said " + self.recognized_data)
            except sr.UnknownValueError:
                self.get_logger().info("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                self.get_logger().info("Could not request results from Google Speech Recognition service; {0}".format(e))     
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()