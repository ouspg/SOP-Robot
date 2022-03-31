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
        # self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.listener_callback, 10)
        self.eye_location_x = eye_location_x
        self.eye_location_y = eye_location_y
        self.eye_location = 5
        self.subscription = self.create_subscription(Point2, '/face_tracker/face_location_topic', self.demo_listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        #moving the eyes based on the face location
        #thresh holds for vertical +- 0.15, horizontal +- 0.5
        if (msg.x < 200) and (msg.y < 130):
            if (self.eye_location_x <= 0.45):
                self.eye_location_x += 0.05
            if (self.eye_location_y >= -0.10):
                self.eye_location_y -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 200 and msg.x < 400) and (msg.y < 130):
            if (self.eye_location_y >= -0.10):
                self.eye_location_y -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y < 130):
            if (self.eye_location_x >= -0.45):
                self.eye_location_x -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x < 200) and (msg.y > 130 and msg.y < 260):
            if (self.eye_location_x <= 0.45):
                self.eye_location_x += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y > 130 and msg.y < 260):
            if (self.eye_location_x >= -0.45):
                self.eye_location_x -= 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x < 200) and (msg.y > 260):
            if (self.eye_location_x <= 0.45):
                self.eye_location_x += 0.05
            if (self.eye_location_y <= 0.10):
                self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 200 and msg.x < 400) and (msg.y > 260):
            if (self.eye_location_y <= 0.10):
                self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)
        elif (msg.x > 400) and (msg.y > 260):
            if (self.eye_location_x >= -0.45):
                self.eye_location_x -= 0.05
            if (self.eye_location_y <= 0.10):
                self.eye_location_y += 0.05
            EyeMoverClient.send_goal(self, self.eye_location_x, self.eye_location_y)

    def demo_listener_callback(self, msg):
        self.get_logger().info('x: %d, y: %d' % (msg.x, msg.y))
        #eye location is split in 9 positions:
        #1,2,3
        #4,5,6
        #7,8,9
        # Face in rectangle 1
        if (msg.x < 200) and (msg.y < 130):
            if self.eye_location in [2, 4, 5]:
                self.send_goal(-0.25, 0.5)
                self.eye_location = 1
            elif self.eye_location in [3, 6]:
                self.send_goal(-0.25, 0)
                self.eye_location = 2
            elif self.eye_location in [7, 8]:
                self.send_goal(0, 0.5)
                self.eye_location = 4
            elif self.eye_location == 9:
                self.send_goal(0, 0)
                self.eye_location = 5
        # Face in rectangle 2
        elif (msg.x > 200 and msg.x < 400) and (msg.y < 130):
            if self.eye_location == 4:
                self.send_goal(-0.25, 0.5)
                self.eye_location = 1
            elif self.eye_location == 5:
                self.send_goal(-0.25, 0)
                self.eye_location = 2
            elif self.eye_location == 6:
                self.send_goal(-0.25, -0.5)
                self.eye_location = 3
            elif self.eye_location == 7:
                self.send_goal(0, 0.5)
                self.eye_location = 4
            elif self.eye_location == 8:
                self.send_goal(0, 0)
                self.eye_location = 5
            elif self.eye_location == 9:
                self.send_goal(0, -0.5)
                self.eye_location = 6
        # Face in rectangle 3
        elif (msg.x > 400) and (msg.y < 130):
            if self.eye_location in [2, 5, 6]:
                self.send_goal(-0.25, -0.5)
                self.eye_location = 3
            elif self.eye_location in [1, 4]:
                self.send_goal(-0.25, 0)
                self.eye_location = 2
            elif self.eye_location in [8, 9]:
                self.send_goal(0, -0.5)
                self.eye_location = 6
            elif self.eye_location == 7:
                self.send_goal(0, 0)
                self.eye_location = 5
        # Face in rectangle 4
        elif (msg.x < 200) and (msg.y > 130 and msg.y < 260):
            if self.eye_location == 2:
                self.send_goal(-0.25, 0.5)
                self.eye_location = 1
            elif self.eye_location == 3:
                self.send_goal(-0.25, 0)
                self.eye_location = 2
            elif self.eye_location == 5:
                self.send_goal(0, 0.5)
                self.eye_location = 4
            elif self.eye_location == 6:
                self.send_goal(0, 0)
                self.eye_location = 5
            elif self.eye_location == 8:
                self.send_goal(0.25, 0.5)
                self.eye_location = 7
            elif self.eye_location == 9:
                self.send_goal(0.25, 0)
                self.eye_location = 8
        # Face in rectangle 6
        elif (msg.x > 400) and (msg.y > 130 and msg.y < 260):
            if self.eye_location == 1:
                self.send_goal(-0.25, 0)
                self.eye_location = 2
            elif self.eye_location == 2:
                self.send_goal(-0.25, -0.5)
                self.eye_location = 3
            elif self.eye_location == 4:
                self.send_goal(0, 0)
                self.eye_location = 5
            elif self.eye_location == 5:
                self.send_goal(0, -0.5)
                self.eye_location = 6
            elif self.eye_location == 7:
                self.send_goal(0.25, 0)
                self.eye_location = 8
            elif self.eye_location == 8:
                self.send_goal(0.25, -0.5)
                self.eye_location = 9
        # Face in rectangle 7
        elif (msg.x < 200) and (msg.y > 260):
            if self.eye_location in [4, 5, 8]:
                self.send_goal(0.25, 0.5)
                self.eye_location = 7
            elif self.eye_location in [1, 2]:
                self.send_goal(0, 0.5)
                self.eye_location = 4
            elif self.eye_location in [6, 9]:
                self.send_goal(0, 0.25)
                self.eye_location = 8
            elif self.eye_location == 3:
                self.send_goal(0, 0)
                self.eye_location = 5
        # Face in rectangle 8
        elif (msg.x > 200 and msg.x < 400) and (msg.y > 260):
            if self.eye_location == 1:
                self.send_goal(0, 0.5)
                self.eye_location = 4
            elif self.eye_location == 2:
                self.send_goal(0, 0)
                self.eye_location = 5
            elif self.eye_location == 3:
                self.send_goal(0, -0.5)
                self.eye_location = 6
            elif self.eye_location == 4:
                self.send_goal(0.25, 0.5)
                self.eye_location = 7
            elif self.eye_location == 5:
                self.send_goal(0, 0.25)
                self.eye_location = 8
            elif self.eye_location == 6:
                self.send_goal(0.25, -0.5)
                self.eye_location = 9
        # Face in rectangle 9
        elif (msg.x > 400) and (msg.y > 260):
            if self.eye_location in [5, 6, 8]:
                self.send_goal(0.25, -0.5)
                self.eye_location = 9
            elif self.eye_location in [2, 3]:
                self.send_goal(0, -0.5)
                self.eye_location = 6
            elif self.eye_location in [4, 7]:
                self.send_goal(0.25, 0)
                self.eye_location = 8
            elif self.eye_location == 1:
                self.send_goal(0, 0)
                self.eye_location = 5

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

    #action_client.send_goal(EyeMoverClient.eye_location_y, EyeMoverClient.eye_location_x)
    action_client.send_goal(0, 0)
    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
