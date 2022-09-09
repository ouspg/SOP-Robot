import random
import time
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class HandActionClient(Node):
    def __init__(self):
        super().__init__("hand_action_client")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, # action name
            "r_hand_controller/follow_joint_trajectory" # action server name
        )
        self.args = ["open", "fist", "scissors", "point", "thumbs_up", "grasp", "pen_grasp", "hard_rock", "rps", "trial", "funny"]
        self.positions_dict = {
            "open": [-0.3, -0.2, -0.2, 0.0, -0.3],
            "fist": [1.5, 1.7, 1.8, 1.5, 1.3],
            "scissors": [1.5, -0.2, -0.2, 1.5, 1.3],
            "point": [1.5, -0.2, 1.8, 1.5, 1.3],
            "thumbs_up": [-0.3, 1.7, 1.8, 1.5, 1.3],
            "grasp": [1.0, 1.2, 1.3, 1.0, 1.0],
            "pen_grasp": [1.5, 1.2, -0.2, 0.0, -0.3],
            "hard_rock": [-0.3, -0.2, 1.8, 1.5, -0.3],
            "funny": [-0.3, 1.7, -0.2, 1.5, 1.3],
            "three": [1.5, -0.2, -0.2, 0.0, 1.3],
        }
        self.logger = self.get_logger()

    def main(self):
        arg = input("Input command: ").lower()
        while arg != "quit":
            if arg not in self.args:
                self.available_commands()
            elif arg == "rps":
                self.rps()
            elif arg == "trial":
                self.trial()
            else:
                self.send_action(arg)
            time.sleep(2)
            arg = input("Input command: ").lower()

    def trial(self):
        command = []
        i = 0
        fingers = ["thumb", "index", "middle", "ring","pinky"]
        while len(command) < 5:
            angle = float(input(f"Angle for {fingers[i]} finger: "))
            if isinstance(angle, float):
                command.append(angle)
                i += 1
            else:
                print("Angle must be float. Safe values are -0.5-2.0.")
        self.logger.info(f"Sending positions {command}")
        self.send_goal(command)

    def available_commands(self):
        print("Command not recognized, available commands are")
        print(f"{self.args[:-1]}")
        print(f"and quit.")

    def rps(self):
        self.logger.info("Starting rps.")
        self.send_action("three")
        time.sleep(2)
        self.send_action("scissors")
        time.sleep(2)
        self.send_action("point")
        time.sleep(2)
        self.send_action(random.choice(["open", "fist", "scissors"]))

    def send_action(self, action):
        self.get_logger().info(f"Action: {action}")
        self.send_goal(self.positions_dict[action])

    def send_goal(self, action):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["r_thumb_joint", "r_index1_joint", "r_middle1_joint", "r_ring_joint", "r_pinky_joint"]
        goal_msg.trajectory.points = []
        point = JointTrajectoryPoint()
        point.positions = action
        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending request")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Action rejected.")
            return
        self.get_logger().info("Action accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Action complete.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = HandActionClient()
    action_client.main()
    rclpy.spin(action_client)
        

if __name__ == "__main__":
    main()