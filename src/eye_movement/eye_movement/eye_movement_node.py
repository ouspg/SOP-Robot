import time
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2


class EyeMoverClient(Node):

    def __init__(self):
        super().__init__('eye_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/eyes_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 1)
        # Middle point of image view
        self.middle_x = 640
        self.middle_y = 400
        self.is_glancing = False
        self.get_logger().info('Eye mover client initialized.')

    def listener_callback(self, msg):
        #self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        glance_percentage = 0.0025
        randomvalue = random.uniform(0, 1)

        if not self.is_glancing:
            # Check if doing the glance or not
            if randomvalue <= glance_percentage:
                eye_location_x, eye_location_y = get_random_location()
                self.is_glancing = True
                self.get_logger().info('glance')
            else:
                eye_location_x, eye_location_y = self.transform_face_location_to_eye_location(msg.x, msg.y)

            # Move eyes
            self.send_goal(eye_location_y, eye_location_x)

        if not self.is_glancing and abs(self.middle_x - msg.x) > 200:
            # Center the eyes in pace with the turning head to keep the camera in the direction of the face
            # No need to constantly center if the eyes are only slightly off center
            time.sleep(0.5)
            self.center_eyes(Duration(sec=0, nanosec=500000000))
            time.sleep(0.5)
        elif self.is_glancing:
            # Center the eyes back to the face after glancing
            time.sleep(0.2)
            self.center_eyes(Duration(sec=0, nanosec=0))
            time.sleep(0.2)
            self.is_glancing = False

    def send_goal(self, vertical, horizontal):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[vertical, horizontal], time_from_start=Duration(sec=0, nanosec=250000000))
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_vertical_joint', 'eyes_shift_horizontal_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('eye location x: %f, eye location y: %f' % (horizontal, vertical))

    def transform_face_location_to_eye_location(self, face_location_x, face_location_y):
        """
        Calculates new x and y location for the eyes corresponding to the face location coordinates given
        as arguments.
        """
        # Calculate face movement
        x_diff = self.middle_x - face_location_x
        y_diff = self.middle_y - face_location_y

        # Transform face movement to eye movement
        # Horizontal eye movement
        h_coeff = -0.001666
        eye_location_x = x_diff * h_coeff - 0.5
        # Vertical eye movement
        v_coeff = 0.003125
        eye_location_y = y_diff * v_coeff - 0.75

        return eye_location_x, eye_location_y
    
    def center_eyes(self, duration=Duration(sec=0, nanosec=0)):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[-0.7, -0.75], time_from_start=duration)
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_vertical_joint', 'eyes_shift_horizontal_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('centering eyes')




def get_random_location():
    """
    Returns a random location coordinates.
    """
    random_x = random.uniform(-1.5, 0.5)
    random_y = random.uniform(-1.5, -0)
    return random_x, random_y


def main():
    print('Hi from eye_movement.')

    rclpy.init()

    action_client = EyeMoverClient()

    action_client.center_eyes()

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
