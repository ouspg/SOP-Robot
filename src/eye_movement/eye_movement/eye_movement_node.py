import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2


class EyeMoverClient(Node):

    def __init__(self, eye_location_x = 0.0, eye_location_y = 0.0):
        super().__init__('eye_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/eyes_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 10)
        self.eye_location_x = eye_location_x
        self.eye_location_y = eye_location_y

    def listener_callback(self, msg):
        self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        #moving the eyes based on the face location
        if (msg.x < 200) and (msg.y < 130):
            self.eye_location_x += 0.05
            self.eye_location_y -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 200 and msg.x < 400) and (msg.y < 130):
            self.eye_location_y -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y < 130):
            self.eye_location_x -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x < 200) and (msg.y > 130 and msg.y < 260):
            self.eye_location_x += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y > 130 and msg.y < 260):
            self.eye_location_x -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x < 200) and (msg.y > 260):
            self.eye_location_x += 0.05
            self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 200 and msg.x < 400) and (msg.y > 260):
            self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y > 260):
            self.eye_location_x -= 0.05
            self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
    def send_goal(self, vertical, horizontal):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[vertical, horizontal], time_from_start=Duration(sec=1, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['eyes_shift_vertical_joint', 'eyes_shift_horizontal_joint'],
                                              points=[trajectory_points])

        self._send_goal_future = self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg)


def main():
    print('Hi from eye_movement.')

    rclpy.init()

    action_client = EyeMoverClient(0.0, 0.0)
    
    action_client.send_goal(action_client.eye_location_x, action_client.eye_location_y)

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
