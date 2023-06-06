import time
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class UpperArmController(Node):
    def __init__(self):
        super().__init__("upper_arm_controller")
        self.control_publisher_ = self.create_publisher(
            FollowJointTrajectory, # action name
            "shoulder_controller/joint_trajectory", # action server name
            10
        )
        self.args = ["trial"]
        self.positions_dict = {

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

    def trial(self):
        command = []
        i = 0
        joints = ["shoulder lift", "upper arm roll", "bicep", "shoulder out"]
        while len(command) < 4:
            angle = int(input(f"Angle for {joints[i]} joint: "))
            if isinstance(angle, int) and angle >= 0 and angle <= 180:
                command.append(angle)
                i += 1
            else:
                print("Angle must be int. Safe values are 0 - 180")
        self.logger.info(f"Sending positions {command}")
        self.send_goal(command)

    def available_commands(self):
        print("Command not recognized, available commands are")
        print(f"{self.args[:-1]}")
        print(f"and quit.")

    def send_action(self, action):
        self.logger.info(f"Action: {action}")
        self.send_goal(self.positions_dict[action])

    def send_goal(self, action):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["r_shoulder_lift_joint", "r_shoulder_out_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
        "l_shoulder_lift_joint", "l_shoulder_out_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint"]
        goal_msg.trajectory.points = []
        point = JointTrajectoryPoint()
        point.positions = action
        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.trajectory.points.append(point)

        self.logger.info(f"Sending request")
        self.control_publisher_.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = UpperArmActionClient()
    action_client.main()
    rclpy.spin(action_client)
        

if __name__ == "__main__":
    main()