import rclpy
import time
import random
from std_msgs.msg import String
from rclpy.node import Node
class HandActionClient(Node):
    def __init__(self):
        super().__init__("right_hand_action_client")
        self.right_hand_gesture_publisher = self.create_publisher(String, "/r_hand/r_hand_topic", 10)
        self.available_commands  = ["open", "fist", "scissors", "point", "thumbs_up", "grasp", "pen_grasp", "hard_rock", "rps", "funny"]
        self.exit_commands = ["quit", "exit"]

        self.logger = self.get_logger()

    def send_gesture(self, gesture):
        msg = String()
        msg.data = gesture
        self.right_hand_gesture_publisher.publish(msg)

    def list_available_commands(self):
        print("Available commands:", end=" ")
        for command in self.available_commands[:-1]:
            print(command, end=", ")
        print(self.available_commands[-1])
        print("You can also input 'quit' or 'exit' to quit.")

    def rps(self):
        self.logger.info(f"Starting rps")
        self.send_gesture("three")
        time.sleep(2)
        self.send_gesture("scissors")
        time.sleep(2)
        self.send_gesture("point")
        time.sleep(2)
        self.send_gesture(random.choice(["open", "fist", "scissors"]))


    def main(self):
        command = ""
        while command not in self.exit_commands:
            command = input("Input command: ").lower()
            if command in self.available_commands:
                if command == "rps":
                    self.rps()
                else:   
                    self.send_gesture(command)
            elif command not in self.exit_commands:
                self.list_available_commands()

def main(args=None):
    rclpy.init(args=args)

    action_client = HandActionClient()
    action_client.main()

    rclpy.shutdown()
        

if __name__ == "__main__":
    main()