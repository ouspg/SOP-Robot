import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String

class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_client')
        self.right_hand_action_client = ActionClient(self, FollowJointTrajectory, "r_hand_controller/follow_joint_trajectory")
        self.left_hand_action_client = ActionClient(self, FollowJointTrajectory, "l_hand_controller/follow_joint_trajectory")
        self.right_hand_gesture_subscription = self.create_subscription(String, "/r_hand/r_hand_topic", self.r_hand_callback, 10)
        self.left_hand_gesture_subscription = self.create_subscription(String, "/l_hand/l_hand_topic", self.l_hand_callback, 10)
        self.args = ["open", "fist", "scissors", "point", "thumbs_up", "grasp", "pen_grasp", "hard_rock", "funny", "three"]
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


    def send_action(self, action, hand):
        self.logger.info(f"Action: {action} for {hand}_hand")
        if hand == "r":
            self.send_right_hand_goal(self.positions_dict[action])
        elif hand == "l":
            self.send_left_hand_goal(self.positions_dict[action])

    def send_right_hand_goal(self, action, duration=Duration(sec=1)):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=action, time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=["r_thumb_joint", "r_index1_joint", "r_middle1_joint", "r_ring_joint", "r_pinky_joint"],
                                              points=[trajectory_points])

        self.right_hand_action_client.wait_for_server()

        self.right_hand_action_client.send_goal_async(goal_msg)
        

    def send_left_hand_goal(self, action, duration=Duration(sec=1)):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=action, time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=["l_thumb_joint", "l_index1_joint", "l_middle1_joint", "l_ring_joint", "l_pinky_joint"],
                                              points=[trajectory_points])

        self.left_hand_action_client.wait_for_server()

        self.left_hand_action_client.send_goal_async(goal_msg)



    def r_hand_callback(self, msg):
        arg = msg.data
        hand = "r"
        self.logger.info(f"{hand}_hand, arg: {arg}")
        if arg not in self.args:
            self.logger.info("Action not implemented")
        else:
            self.send_action(arg, hand)

    
    def l_hand_callback(self, msg):
        arg = msg.data
        hand = "l"
        self.logger.info(f"{hand}_hand, arg: {arg}")
        if arg not in self.args:
            self.logger.info("Action not implemented")
        else:
            self.send_action(arg, hand)


def main():
    print('Hi from hand_gestures.')
    rclpy.init()
    action_client = HandGestureNode()
    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
