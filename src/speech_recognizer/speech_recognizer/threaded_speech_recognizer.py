#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os

from threading import Thread
from queue import Queueo

class MTSpeechRecognizerNode(Node):

    def __init__(self):
        super().__init__('mt_speech_recognizer_node')
        self.publisher_ = self.create_publisher(String, 'mt_recognized_speech', 10)
        self.r = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.audio_queue = Queue()

    def start_recognition(self):
        def recognize_worker():
            with self.sr as source:




            while True:
                audio = self.audio_queue.get()
                if audio == None:
                    break

                try:
                    print("Sanoit: " + self.r.recognize_google(audio, language="fi"))
                except sr.UnknownValueError:
                    print("En ymmärtänyt mitä yritit sanoa")
                except sr.RequestError as e:
                    print("En saanut tuloksia Google speech Recognition palvelusta; {0}".format(e))
                audio_queue.task_done()

        recognize_thread = Thread(target=recognize_worker)
        recognize_thread.daemon = True
        recognize_thread.start()
        with sr.Microphone() as source:
            try:
                while True:
                    audio_queue.put(r.listen(source))
            except KeyboardInterrupt:
                pass
        audio_queue.join()
        audio_queue.put(None)
        recognize_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = MTSpeechRecognizerNode()
    node.start_recognition()












