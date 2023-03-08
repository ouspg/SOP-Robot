import time
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
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

class HeadMoverNode(Node):

    def __init__(self):
        super().__init__('head_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        
        timer_period = 1
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0
        
        self.get_logger().info('Head mover client initialized.')
        
        
    def timer_callback(self):
        goal_msg = FollowJointTrajectory.Goal()

        headPan = self.i % 2

        trajectory_points = JointTrajectoryPoint(positions=[headPan], time_from_start=Duration(sec=0, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_pan_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self.get_logger().info("Head pan: " + str(headPan))

        self._action_client.send_goal_async(goal_msg)
        
        self.i += 1


def main():
    print('Hello from head_gestures.')

    rclpy.init()

    action_client = HeadMoverNode()

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
