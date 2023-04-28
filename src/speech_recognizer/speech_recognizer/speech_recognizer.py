#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os
import pocketsphinx as spx

class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer_node')
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)
        self.r = sr.Recognizer()
        self.microphone = sr.Microphone()

    def start_listening(self):
        with self.microphone as source:
            print("Say something!")
            audio = self.r.listen(source)
            try:
                #text = self.r.recognize_google_cloud(audio, credentials_json=None, language="fi" )
                #text = self.r.recognize_google(audio, language="fi" )
                text = self.r.recognize_sphinx(audio)
                #credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS_JSON)
                print(f"You said: {text}")
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
            except sr.UnknownValueError:
                #print("Google Cloud Speech-to-Text could not understand audio")
                print("Could not understand audio")
            except sr.RequestError as e:
                #print(f"Could not request results from Google Cloud Speech-to-Text service; {e}")
                print(f"Could not request results from service; {e}")

def main(args=None):
    rclpy.init(args=args)

    node = SpeechRecognizerNode()

    while rclpy.ok():
        node.start_listening()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

