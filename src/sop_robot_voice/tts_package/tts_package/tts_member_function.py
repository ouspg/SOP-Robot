"""Compatibility CLI publisher for the TTS node."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .topics import CANONICAL_INPUT_TOPIC


class TTSClientAsync(Node):
    """Publish direct assistant text messages to the TTS node."""

    def __init__(self):
        super().__init__("tts_client_async")
        self.publisher = self.create_publisher(String, CANONICAL_INPUT_TOPIC, 10)

    def send_request(self, data: str):
        req = String()
        req.data = data
        return self.publisher.publish(req)

    def main(self):
        arg = input("Input command: ").strip()
        while arg.lower() != "quit":
            if arg:
                self.send_request(arg)
            time.sleep(0.2)
            arg = input("Input command: ").strip()


def main():
    rclpy.init()
    tts_client = TTSClientAsync()
    tts_client.main()
    rclpy.spin(tts_client)


if __name__ == "__main__":
    main()
