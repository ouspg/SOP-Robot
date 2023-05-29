import rclpy

from std_msgs.msg import String
from rclpy.node import Node

class HeadGestureClient(Node):
    def __init__(self):
        super().__init__("head_gesture_client")

        self.head_gesture_publisher = self.create_publisher(String, "/head_gestures/head_gesture_topic", 10)
        self.available_commands = ["nod", "shake"]
        self.exit_commands = ["quit", "exit"]

        self.logger = self.get_logger()
    
    def send_gesture(self, gesture):
        msg = String()
        msg.data = gesture
        self.head_gesture_publisher.publish(msg)

    def list_available_commands(self):
        print("Available commands:", end=" ")
        for command in self.available_commands[:-1]:
            print(command, end=", ")
        print(self.available_commands[-1])
        print("You can also input 'quit' or 'exit' to quit.")

    def main(self):
        command = ""
        while command not in self.exit_commands:
            command = input("Input command: ").lower()
            if command in self.available_commands:
                self.send_gesture(command)
            elif command not in self.exit_commands:
                self.list_available_commands()



def main(args=None):
    rclpy.init(args=args)

    client = HeadGestureClient();
    client.main()

    rclpy.shutdown()


if __name__ == "__main__":
    main()