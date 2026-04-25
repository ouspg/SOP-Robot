import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sop_robot_common.contracts import (
    LEFT_HAND_GESTURE_TOPIC,
    RIGHT_HAND_GESTURE_TOPIC,
)
from sop_robot_common.joint_calibration import (
    LEFT_HAND_SPEC,
    RIGHT_HAND_SPEC,
    load_joint_calibration,
    load_joint_specs,
    parse_calibration_command,
    parse_set_command,
)
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEFAULT_GESTURE_DURATION = Duration(sec=1)


class HandGestureNode(Node):
    def __init__(self):
        super().__init__("hand_gesture_client")
        self.right_hand_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "r_hand_controller/follow_joint_trajectory",
        )
        self.left_hand_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "l_hand_controller/follow_joint_trajectory",
        )
        self.right_hand_gesture_subscription = self.create_subscription(
            String,
            RIGHT_HAND_GESTURE_TOPIC,
            self.r_hand_callback,
            10,
        )
        self.left_hand_gesture_subscription = self.create_subscription(
            String,
            LEFT_HAND_GESTURE_TOPIC,
            self.l_hand_callback,
            10,
        )
        calibration_path = (
            self.declare_parameter("joint_calibration_path", "")
            .get_parameter_value()
            .string_value
        )
        use_max_limits = (
            self.declare_parameter("joint_calibration_use_max_limits", False)
            .get_parameter_value()
            .bool_value
        )
        joint_specs = load_joint_specs(calibration_path, use_max_limits=use_max_limits)
        self.right_hand_spec = joint_specs[RIGHT_HAND_SPEC.key]
        self.left_hand_spec = joint_specs[LEFT_HAND_SPEC.key]
        calibration = load_joint_calibration(calibration_path, joint_specs)
        self.right_hand_center = calibration[self.right_hand_spec.key]
        self.left_hand_center = calibration[self.left_hand_spec.key]
        self.right_positions_dict = {
            "center": self.right_hand_center,
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
        self.left_positions_dict = {
            "center": self.left_hand_center,
            "open": [-2.35, 2, -0.07, -2.35, 0.19],
            "fist": [-0.78, 0.45, 2, -0.47, -1.72],
            "scissors": [-0.78, 2, -0.07, -0.47, -1.72],
            "point": [-0.78, 2, 2, -0.47, -1.72],
            "thumbs_up": [-2.35, 0.45, 2, -0.47, -1.72],
            "grasp": [-1.3, 0.95, 1.5, -0.97, -1.42],
            "pen_grasp": [-0.78, 0.95, -0.07, -2.35, 0.19],
            "hard_rock": [-2.35, 2, 2, -0.47, 0.19],
            "funny": [-2.35, 0.45, -0.07, -0.47, -1.72],
            "three": [-0.78, 2, -0.07, -2.35, -1.72],
        }
        self.logger = self.get_logger()

    def send_action(self, action, hand):
        self.logger.info(f"Action: {action} for {hand}_hand")
        if hand == "r":
            self.send_right_hand_goal(self.right_positions_dict[action])
        elif hand == "l":
            self.send_left_hand_goal(self.left_positions_dict[action])

    def send_right_hand_goal(self, action, duration=DEFAULT_GESTURE_DURATION):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(
            positions=action,
            time_from_start=duration,
        )
        goal_msg.trajectory = JointTrajectory(
            joint_names=[
                "r_thumb_joint",
                "r_index1_joint",
                "r_middle1_joint",
                "r_ring_joint",
                "r_pinky_joint",
            ],
            points=[trajectory_points],
        )

        self.right_hand_action_client.wait_for_server()
        self.right_hand_action_client.send_goal_async(goal_msg)

    def send_left_hand_goal(self, action, duration=DEFAULT_GESTURE_DURATION):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(
            positions=action,
            time_from_start=duration,
        )
        goal_msg.trajectory = JointTrajectory(
            joint_names=[
                "l_thumb_joint",
                "l_index1_joint",
                "l_middle1_joint",
                "l_ring_joint",
                "l_pinky_joint",
            ],
            points=[trajectory_points],
        )

        self.left_hand_action_client.wait_for_server()
        self.left_hand_action_client.send_goal_async(goal_msg)

    def r_hand_callback(self, msg):
        arg = msg.data
        hand = "r"
        self.logger.info(f"{hand}_hand, arg: {arg}")
        direct_positions = parse_set_command(self.right_hand_spec, arg)
        if direct_positions is not None:
            self.send_right_hand_goal(direct_positions)
            return

        center_positions = parse_calibration_command(self.right_hand_spec, arg)
        if center_positions is not None:
            self.right_hand_center = center_positions
            self.right_positions_dict["center"] = center_positions
            self.logger.info(f"Updated right hand center: {center_positions}")
            return

        if arg not in self.right_positions_dict:
            self.logger.info("Action not implemented")
        else:
            self.send_action(arg, hand)

    def l_hand_callback(self, msg):
        arg = msg.data
        hand = "l"
        self.logger.info(f"{hand}_hand, arg: {arg}")
        direct_positions = parse_set_command(self.left_hand_spec, arg)
        if direct_positions is not None:
            self.send_left_hand_goal(direct_positions)
            return

        center_positions = parse_calibration_command(self.left_hand_spec, arg)
        if center_positions is not None:
            self.left_hand_center = center_positions
            self.left_positions_dict["center"] = center_positions
            self.logger.info(f"Updated left hand center: {center_positions}")
            return

        if arg not in self.left_positions_dict:
            self.logger.info("Action not implemented")
        else:
            self.send_action(arg, hand)


def main():
    print("Hi from hand_gestures.")
    rclpy.init()
    action_client = HandGestureNode()
    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
