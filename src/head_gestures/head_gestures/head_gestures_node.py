import time
import math
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2

""" Head shake message
ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory
"   { trajectory:
        { joint_names:
            [head_pan_joint],
            points: [
                { positions: [1.25], time_from_start: { sec: 0, nanosec: 200000000 }},
                { positions: [0.25], time_from_start: { sec: 0, nanosec: 500000000 }},
                { positions: [0.75], time_from_start: { sec: 0, nanosec: 900000000 }}
            ]
        }
    }"
"""

"""
    Head joint starting values that don't break the whole thing. Not exactly looking straight ahead.
    head_pan_joint:             0.5
    head_tilt_right_joint:      0.5
    head_tilt_left_joint:      -0.5
    head_tilt_vertical_joint:  -0.5
"""

class HeadGesturesNode(Node):

    def __init__(self):
        super().__init__('head_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        
        self.get_logger().info('Head mover client initialized.')

    def send_horizontal_tilt_goal(self, horizontalTilt):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[-horizontalTilt, horizontalTilt], time_from_start=Duration(sec=1, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_tilt_left_joint', 'head_tilt_right_joint'],
                                              points=[trajectory_points])
        
        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)
        
    def send_pan_and_vertical_tilt_goal(self, pan, verticalTilt):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[pan, verticalTilt], time_from_start=Duration(sec=0, nanosec=400000000))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_pan_joint', 'head_tilt_vertical_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)

    def send_goal(self, pan, verticalTilt, horizontalTilt):
        #Horizontal tilt is done separately and slower because the joints easily get stuck when moving quickly.
        self.send_horizontal_tilt_goal(horizontalTilt)
        time.sleep(1)
        self.send_pan_and_vertical_tilt_goal(pan, verticalTilt)

def main():
    print('Hello from head_gestures.')

    rclpy.init()

    action_client = HeadGesturesNode()

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
