#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os

from threading import Thread
from queue import Queue

r = sr.Recognizer()
audio_queue = Queue()

def recognize_worker():
    while True:
        audio = audio_queue.get()
        if audio == None:
            break

        try:
            print("Sanoit: " + r.recognize_google(audio, language="fi"))
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










