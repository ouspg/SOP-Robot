#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import Future

from msg_interface.srv import SpeechRec
from msg_interface.msg import SpeechRecognitionCandidates

class SpeechRecClient(Node):
    
    def __init__(self):
        super().__init__('speech_rec_client')
        self.call_speech_rec_server()
        self.transcript = None
        self.confidence = None
        self.future = Future()
        self.publisher = self.create_publisher(SpeechRecognitionCandidates, "speech_recognition_candidates", 10)
        self.timer = self.create_timer(0.2, self.publish_speech)
        self.get_logger().info("Speech recognition client has been started.")

    def publish_speech(self):
        msg = SpeechRecognitionCandidates()
        if (self.transcript != None | self.confidence != None):
            msg.transcript = self.transcript
            msg.confidence = self.confidence
            self.publisher.publish(msg)
            self.transcript = None
            self.confidence = None

    def call_speech_rec_server(self):
        client = self.create_client(SpeechRec, "speech_rec_server")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")

        request = SpeechRec.Request()

        self.future = client.call_async(request)
        self.future.add_done_callback(self.callback_call_speech_rec_server)

    def callback_call_speech_rec_server(self, future):
        try:
            response = future.result()
            self.transcript = response.trancript
            self.confidence = response.confidence
            self.get_logger().info("Transcript: ", str(response.transcript), "\nConfidence: ", str(response.confidence))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()