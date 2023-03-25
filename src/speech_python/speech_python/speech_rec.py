import io
import logging
import time
import types
import os
import usb.core
import usb.util
import pocketsphinx

import speech_recognition as sr
from threading import Thread, Event

from msg_interface.msg import SpeechRecognitionCandidates

import pyaudio

class Rec(Thread):
    def __init__(self, name):
        Thread.__init__(self)
        self.event = Event()
        self.name = name

    def run(self):
        print("Starting task")
        while not self.event.is_set():
            
            r = sr.Recognizer()
        
            with sr.Microphone(device_index=0, sample_rate=16000) as source:

                print("Say something!")
                audio = r.listen(source)
            
                try:
                    # for testing purposes, we're just using the default API key
                    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                    # instead of `r.recognize_google(audio)`
                    recognized_data = r.recognize_google(audio, language="fi")
                    all_data = r.recognize_google(audio, language="fi", show_all=True)
                    print("Google Speech Recognition thinks you said " + recognized_data)
                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    print("Could not request results from Google Speech Recognition service; {0}".format(e))        

def speech_rec():
    logging.basicConfig(level=logging.DEBUG)
    quit_event = Event()
    thread = Rec("Harjoitus")
    thread.start()
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print(thread.name + " loppui")
            thread.event.set()
            break
    thread.join()

if __name__ == '__main__':
    speech_rec()                