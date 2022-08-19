import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


positions_dict = {
    "fist": [0.0, 0.0, 0.0, 0.0, 0.0],
    "open": [1.5, 1.5, 1.5, 1.5, 1.5],
    "scissors": [1.5, 0.0, 0.0, 1.5, 1.5],
    "point": [1.5, 0.0, 1.5, 1.5, 1.5],
    "thumbs_up": [0.0, 1.5, 1.5, 1.5, 1.5],
    "grasp": [1.0, 1.0, 1.0, 1.0, 1.0],
    "pen_grasp": [1.0, 1.0, 0.0, 0.0, 0.0],
    "hard_rock": [0.0, 0.0, 1.5, 1.5, 0.0],
    "funny": [0.0, 1.5, 0.0, 1.5, 1.5]
}


class HandActionClient(Node):
    def __init__(self):
        super().__init__("hand_action_client")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, # action name
            "r_hand_controller/follow_joint_trajectory" # action server name
        )
    
    def send_goal(self, action):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ["r_thumb_joint", "r_index1_joint", "r_middle1_joint", "r_ring_joint", "r_pinky_joint"]
        goal_msg.trajectory.points = []
        point = JointTrajectoryPoint()
        self.get_logger().info(f"Action: {action}")
        point.positions = action
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
        self.get_logger().info(f"Action complete.")
        rclpy.shutdown()

def main(args=None):
    for action in positions_dict:
        rclpy.init(args=args)
        action_client = HandActionClient()
        action_client.send_goal(positions_dict[action])
        rclpy.spin(action_client)
        time.sleep(2)
        

if __name__ == "__main__":
    main()