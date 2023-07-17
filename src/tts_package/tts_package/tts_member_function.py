import sys

from tts_package.srv import StringToWav

import rclpy
from rclpy.node import Node


class ttsClientAsync(Node):

    def __init__(self):
        super().__init__('tts_client_async')
        self.cli = self.create_client(StringToWav, 'StringToWav')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StringToWav.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    tts_client = ttsClientAsync()
    response = tts_client.send_request(int(sys.argv[1]))
    if(response.success):
        minimal_client.get_logger().info(
            'Succesfully synthentized!')
    else:
        minimal_client.get_logger().info(
            'Failed to synthentize!'
        )
    tts_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()