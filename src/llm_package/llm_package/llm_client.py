# readline makes input() edit by character instead of by byte, so backspacing over
# multi-byte letters like ä and ö no longer leaves invalid UTF-8 behind.
import readline  # noqa: F401
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LlmClientAsync(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.publisher = self.create_publisher(String, 'recognized_speech', 10)
    def send_request(self, data):
        req = String()
        req.data = data
        return self.publisher.publish(req)

    def main(self):
        arg = input('Input command: ').lower()
        while arg != 'quit':
            self.send_request(arg)
            time.sleep(2)
            arg = input('Input command: ').lower()


def main():
    rclpy.init()

    llm_client = LlmClientAsync()
    llm_client.main()
    rclpy.spin(llm_client)


if __name__ == '__main__':
    main()
