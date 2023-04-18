import time
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class UpperArmActionClient(Node):
    def __init__(self):
        super().__init__("upper_arm_action_client")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, # action name
            "r_shoulder_controller/follow_joint_trajectory" # action server name
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
            if isinstance(angle, float):
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
        goal_msg.trajectory.joint_names = ["r_shoulder_lift_joint", "r_shoulder_out_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint"]
        goal_msg.trajectory.points = []
        point = JointTrajectoryPoint()
        point.positions = action
        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.trajectory.points.append(point)

        self.logger.info(f"Sending request")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.logger.info(goal_handle)
        if not goal_handle.accepted:
            self.logger.info("Action rejected.")
            return
        self.logger.info("Action accepted.")
        self._get_result_future = goal_handle.get_result()
        self.logger.info(self._get_result_future)
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.logger.info(result)
        self.logger.info("Action complete.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = UpperArmActionClient()
    action_client.main()
    rclpy.spin(action_client)
        

if __name__ == "__main__":
    main()