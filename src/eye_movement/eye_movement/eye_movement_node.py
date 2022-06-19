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
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 10)
        # Middle point of image view
        self.middle_x = 640
        self.middle_y = 400
        self.get_logger().info('Eye mover client initialized.')

    def listener_callback(self, msg):
        self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        # Calculate face movement
        y_diff = self.middle_y - msg.y
        x_diff = self.middle_x - msg.x
        # Transform face movement to eye movement
        # Vertical eye movement
        v_coeff = -0.001125
        eye_location_y = y_diff * v_coeff - 0.25
        # Horizontal eye movement
        h_coeff = -0.001563
        eye_location_x = x_diff * h_coeff - 1.0
        # Move eyes
        self.send_goal(eye_location_y, eye_location_x)
        self.get_logger().info('eye location x: %f, eye location y: %f' % (eye_location_x, eye_location_y))

    def send_goal(self, vertical, horizontal):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[vertical, horizontal], time_from_start=Duration(sec=0, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_vertical_joint', 'eyes_shift_horizontal_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)


def main():
    print('Hi from eye_movement.')

    rclpy.init()

    action_client = EyeMoverClient()

    action_client.send_goal(-0.25, -1.0)

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
