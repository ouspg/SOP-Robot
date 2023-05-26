import random
import time
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from face_tracker_msgs.msg import Point2

class HeadActionClient(Node):
    def __init__(self):
        super().__init__("head_action_client")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, # action name
            "head_controller/follow_joint_trajectory" # action server name
        )
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 10)
        self.middle_x = 640
        self.logger = self.get_logger()

    def listener_callback(self, msg):
        self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        eye_location_x = self.transform_face_location_to_eye_location(msg.x)
        self.get_logger().info('eye_x: %f' % (eye_location_x))
        self.send_goal(eye_location_x)
        

    def transform_face_location_to_eye_location(self, face_location_x):
        #x_diff =  face_location_x - self.middle_x
        #h_coeff = -0.00078125
        #eye_location_x = x_diff * h_coeff
        x = -1 + (face_location_x*2)/(self.middle_x)
        return -1 * x


    def send_goal(self, horizontal):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_points = JointTrajectoryPoint(positions=[horizontal], time_from_start=Duration(sec=0, nanosec=0))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_pan_joint'],
                                                points=[trajectory_points])
        self._send_goal_future = self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)




def main(args=None):
    rclpy.init(args=args)
    action_client = HeadActionClient()
    action_client.send_goal(0.0)
    rclpy.spin(action_client)
        
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()