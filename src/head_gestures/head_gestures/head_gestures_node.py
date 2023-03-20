import time
import math
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

"""
    Good middle values for real head joints. Might not work for simulated robot:
    head_pan_joint:             0.57
    head_tilt_vertical_joint:  -0.55
    head_tilt_left_joint:      -0.5
    head_tilt_right_joint:      0.5
"""

class HeadMoverNode(Node):

    def __init__(self):
        super().__init__('head_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 1)
        # Middle point of image view
        self.middle_x = 640
        self.middle_y = 400
        self.current_pan = 0.57
        self.current_vertical_tilt = -0.55
        self.moving = False
        self.pan_diff = 0
        #self.timer = self.create_timer(0.25, self.turn_to_face)
        self.send_goal(self.current_pan, self.current_vertical_tilt, 0.5)
        time.sleep(1)
        
        self.get_logger().info('Head mover client initialized.')

    def listener_callback(self, msg):
        if not self.moving:
            self.pan_diff, _ = self.transform_face_location_to_head_values(msg.x, msg.y)

            if self.pan_diff != 0: 
                self.moving = True
                time.sleep(0.3)
                self.current_pan = max(min(1.75, self.current_pan + self.pan_diff), -0.25) # limit pan to reasonable values
                self.get_logger().info("Turning head by " + str(self.pan_diff))
                self.send_pan_and_vertical_tilt_goal(self.current_pan, -0.55)
                time.sleep(0.5)
                self.pan_diff = 0
                self.moving = False

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

    def transform_face_location_to_head_values(self, face_location_x, face_location_y):
        """
        Calculates new pan and vertical tilt values corresponding to the face location coordinates given
        as arguments.
        """
        # Calculate face movement
        x_diff = self.middle_x - face_location_x
        y_diff = self.middle_y - face_location_y

        # If the face is close enough to the center, leave the small movements for the eyes.
        if abs(x_diff) < 200:
            x_diff = 0
        if abs(y_diff) < 200:
            y_diff = 0

        # Transform face movement to head joint values
        # Head pan
        h_coeff = -0.0008
        pan = x_diff * h_coeff
        # TODO: Vertical tilt
        v_coeff = 0
        vertical_tilt = y_diff * v_coeff

        return pan, vertical_tilt


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
