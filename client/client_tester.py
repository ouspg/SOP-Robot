import rclpy
from std_msgs.msg import String
from rclpy.node import Node


class UnifiedClientTester(Node):
    # This is used to emulate the "main" program which would send very limited
    # commands like "wave" to the unified arms controller
    def __init__(self):
        super().__init__("unified_arms_client_tester")
        self.arm_action_publisher = self.create_publisher(String, "/arms/arm_action", 10)
        self.available_commands  = [
            "wave",
            "rps",
            "rock",
            "hug",
            "intimidate"
            "searchanddestroy",
        ]
        self.exit_commands = ["quit", "exit"]
        self.logger = self.get_logger()


    def send_action(self, action):
        msg = String()
        msg.data = action
        self.arm_action_publisher.publish(msg)


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
                self.send_action(command)
            elif command not in self.exit_commands:
                self.list_available_commands()


if __name__ == "__main__":
    rclpy.init()

    tester = UnifiedClientTester()
    tester.main()

    rclpy.shutdown()
