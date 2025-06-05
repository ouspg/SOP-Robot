#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import speech_recognition as sr



class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer_node')
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)
        self.subscription = self.create_subscription(Bool, "can_listen", self.callback, 10)
        self.model="./src/speech_recognizer/resource/whisper-model.bin"
        self.r = sr.Recognizer()
        self.m  = sr.Microphone()
        with self.m as source:
            self.r.adjust_for_ambient_noise(source, duration=5)
        self.stop_listening = self.r.listen_in_background(self.m, self.listen, phrase_time_limit=10)

    def callback(self, msg):
        if msg.data:
            self.stop_listening = self.r.listen_in_background(self.m, self.listen, phrase_time_limit=10)
        else:
            self.stop_listening()
        


    def listen(self, recognizer: sr.Recognizer, audio: sr.AudioData):
            try:
                text = recognizer.recognize_whisper(audio,model=self.model, language="fi" )
                print(f"You said: {text}")
                req = String()
                req.data = text
                self.publisher_.publish(req)
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"Could not request results from service; {e}")



def main(args=None):
    rclpy.init(args=args)

    node = SpeechRecognizerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

