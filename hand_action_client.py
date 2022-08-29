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
        self.args = ["open", "fist", "scissors", "point", "thumbs_up", "grasp", "pen_grasp", "hard_rock", "rps", "funny"]
        self.positions_dict = {
            "open": [0.0, 0.0, 0.0, 0.0, 0.0],
            "fist": [1.5, 1.5, 1.5, 1.5, 1.5],
            "scissors": [1.5, 0.0, 0.0, 1.5, 1.5],
            "point": [1.5, 0.0, 1.5, 1.5, 1.5],
            "thumbs_up": [0.0, 1.5, 1.5, 1.5, 1.5],
            "grasp": [1.0, 1.0, 1.0, 1.0, 1.0],
            "pen_grasp": [1.0, 1.0, 0.0, 0.0, 0.0],
            "hard_rock": [0.0, 0.0, 1.5, 1.5, 0.0],
            "funny": [0.0, 1.5, 0.0, 1.5, 1.5],
            "three": [1.5, 0.0, 0.0, 0.0, 1.5],
        }
        self.logger = self.get_logger()

    def main(self):
        arg = input("Input command: ").lower()
        while arg != "quit":
            if arg not in self.args:
                self.available_commands()
            elif arg == "rps":
                self.rps()
            else:
                self.send_goal(arg)
            time.sleep(2)
            arg = input("Input command: ").lower()  

    def available_commands(self):
        print("Command not recognized, available commands are")
        print(f"{self.args[:-1]}")
        print(f"and quit.")

    def rps(self):
        self.logger.info("Starting rps.")
        self.send_goal("three")
        time.sleep(2)
        self.send_goal("scissors")
        time.sleep(2)
        self.send_goal("point")
        time.sleep(2)
        self.send_goal(random.choice(["open", "fist", "scissors"]))

    def send_goal(self, action):
        self.get_logger().info(f"Action: {action}")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["r_thumb_joint", "r_index1_joint", "r_middle1_joint", "r_ring_joint", "r_pinky_joint"]
        goal_msg.trajectory.points = []
        point = JointTrajectoryPoint()
        point.positions = self.positions_dict[action]
        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending {action} request")
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