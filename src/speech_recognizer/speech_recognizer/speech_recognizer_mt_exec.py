#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import speech_recognition as sr
import threading


class SpeechRecognizerNode(Node):

    def __init__(self):
        super().__init__('mt_speech_recognizer')
        self.publisher_ = self.create_publisher(String, 'mt_recognized_speech', 10)
        self.recognizer_ = sr.Recognizer()

    def start_recognizing(self):
        def recognize():
            with sr.Microphone() as source:
                self.recognizer_.adjust_for_ambient_noise(source)
                self.get_logger().info('Listening...')
                while rclpy.ok():
                    try:
                        #audio = self.recognizer_.listen(source, timeout=5)
                        audio = self.recognizer_.listen(source)
                        text = self.recognizer_.recognize_google(audio)
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Recognized speech: {text}')
                    except sr.UnknownValueError:
                        self.get_logger().warning('Speech not recognized')
                    except sr.RequestError as e:
                        self.get_logger().error(f'Recognition request failed: {e}')

        thread = threading.Thread(target=recognize)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start_recognizing()
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

