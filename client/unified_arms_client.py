import rclpy
import time
import random
from std_msgs.msg import String
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


class RightHandActionClient(Node):
    # TODO unify Left and Right HandAction client
    # by defining left/right when instancing the class
    def __init__(self):
        super().__init__("right_hand_action_client")
        self.right_hand_gesture_publisher = self.create_publisher(String, "/r_hand/r_hand_topic", 10)
        self.available_commands  = [
            "open",
            "fist",
            "scissors",
            "point",
            "thumbs_up",
            "grasp",
            "pen_grasp",
            "hard_rock",
            "rps",
            "funny"
        ]
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


class LeftHandActionClient(Node):
    def __init__(self):
        super().__init__("left_hand_action_client")
        self.left_hand_gesture_publisher = self.create_publisher(String, "/l_hand/l_hand_topic", 10)
        self.available_commands  = [
            "open",
            "fist",
            "scissors",
            "point",
            "thumbs_up",
            "grasp",
            "pen_grasp",
            "hard_rock",
            "rps",
            "funny"
        ]
        self.exit_commands = ["quit", "exit"]
        self.logger = self.get_logger()


    def send_gesture(self, gesture):
        msg = String()
        msg.data = gesture
        self.left_hand_gesture_publisher.publish(msg)


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

# Arms (Shoulders):

class ShoulderController(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'shoulder_controller/joint_trajectory', 10)
        self.args = ['trial', 'zero', 'rps_1']
        self.positions_dict = {
            "zero": [30.0, 90.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0],
            "rps_1": [75.0, 90.0, 80.0, 0.0, 34.0, 80.0, 10.0, 0.0]
        }
        self.logger = self.get_logger()


    def main(self):
        arg = input("Input command: ").lower()
        while arg != "quit":
            if arg not in self.args:
                self.available_commands()
            elif arg == "trial":
                self.trial()
            else:
                self.send_action(arg)
            time.sleep(2)
            arg = input("Input command: ").lower()


    def available_commands(self):
        print("Command not recognized, available commands are")
        print(f"{self.args[:-1]}")
        print(f"and quit.")


    def send_action(self, action):
        self.logger.info(f"Action: {action}")
        self.send_goal(self.positions_dict[action])


    def trial(self):
        command = []
        i = 0
        joints = [
            "R_shoulder lift",
            "R_upper arm roll",
            "R_bicep",
            "R_shoulder out","L_shoulder lift",
            "L_upper arm roll",
            "L_bicep",
            "L_shoulder out"
        ]
        while len(command) < 8:
            angle = float(input(f"Angle for {joints[i]} joint: "))
            if isinstance(angle, float) and angle >= 0 and angle <= 180:
                command.append(angle)
                i += 1
            else:
                print("Angle must be int. Safe values are 0 - 180")
        self.logger.info("Sending positions")
        self.send_goal(command)


    def send_goal(self, action):
        goal_msg = JointTrajectory()
        goal_msg.joint_names = [
            "r_shoulder_lift_joint",
            "r_shoulder_out_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "l_shoulder_lift_joint",
            "l_shoulder_out_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint"
        ]
        point = JointTrajectoryPoint()
        point.positions = action
        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.points = [point]

        self.logger.info(f"Sending request")
        print(type(goal_msg))
        self.publisher_.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_clients = {}
    action_clients["right_hand"] = RightHandActionClient()
    action_clients["left_hand"] = LeftHandActionClient()
    action_clients["shoulders"] = ShoulderController()

    exit_commands = ["exit", "quit"]
    client_selection = ""

    while client_selection not in exit_commands:
        client_selection = input("Select body part to actuate: ").lower()
        if client_selection in action_clients:
            active_client = action_clients[client_selection]
            command = ""
            while command not in exit_commands:
                command = input("Input command: ").lower()
                if command in active_client.available_commands:
                    if command == "rps":
                        active_client.rps()
                    else:
                        active_client.send_gesture(command)
                elif command not in exit_commands:
                    active_client.list_available_commands()
        elif client_selection not in exit_commands:
            for part in action_clients:
                print(part)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
