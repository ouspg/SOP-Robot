import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class EyeMoverClient(Node):

    def __init__(self):
        super().__init__('eye_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/eyes_controller/follow_joint_trajectory')

    def send_goal(self, vertical, horizontal):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[vertical, horizontal], time_from_start=Duration(sec=1, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_vertical_joint', 'eyes_shift_horizontal_joint'],
                                              points=[trajectory_points])

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main():
    print('Hi from eye_movement.')

    rclpy.init()

    action_client = EyeMoverClient()

    future = action_client.send_goal(-0.5, -0.3)

    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()
