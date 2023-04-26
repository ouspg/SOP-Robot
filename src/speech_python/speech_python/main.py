import logging
import time
import rclpy

from threading import Thread, Event
from queue import Queue
from enum import Enum

from speech_python.speech_rec_service import Publish
from speech_python.speech_rec_client_async import Rec


from msg_interface.msg import SpeechRecognitionCandidates

def main(args=None):

    logging.basicConfig(level=logging.DEBUG)
    quit_event = Event()
    q_ready = Queue()
    q_msg_data = Queue()
    rec = Rec("Speech Recognition", quit_event)
    pub = Publish("Minimal Publisher", quit_event)
    thread1 = Thread(target = rec.run, args=(q_ready, q_msg_data))
    thread2 = Thread(target = pub.run, args=(q_ready, q_msg_data))
    thread1.start()
    thread2.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print(thread1.name + " exited")
            thread1.event.set()
            print(thread2.name + " exited")
            thread2.event.set()
            break
    q_ready.join()
    q_msg_data.join()

if __name__ == '__main__':
    main()
