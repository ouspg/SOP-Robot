import sys

from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time

class ttsClientAsync(Node):

    def __init__(self):
        super().__init__('tts_client_async')
        self.publisher = self.create_publisher(String, "response", 10)

    def send_request(self, data):
        req = String()
        req.data = data
        return self.publisher.publish(req)
    
    def main(self):
        arg = input("Input command: ").lower()
        while arg != "quit":
            self.send_request(arg)
            time.sleep(2)
            arg = input("Input command: ").lower()
    
def main():
    rclpy.init()

    tts_client = ttsClientAsync()
    tts_client.main()
    rclpy.spin(tts_client)


if __name__ == '__main__':
    main()