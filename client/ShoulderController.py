import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


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
        joints = ["R_shoulder lift", "R_upper arm roll", "R_bicep", "R_shoulder out","L_shoulder lift", "L_upper arm roll", "L_bicep", "L_shoulder out"]
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
        goal_msg.joint_names = ["r_shoulder_lift_joint", "r_shoulder_out_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
        "l_shoulder_lift_joint", "l_shoulder_out_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint"]
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

    controller = ShoulderController()
    controller.main()

    rclpy.spin(controller)


if __name__ == '__main__':
    main()