import time
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from face_tracker_msgs.msg import Point2

class JawMoverNode(Node):

    def __init__(self):
        super().__init__('jaw_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/jaw_controller/follow_joint_trajectory')
        
        timer_period = 0.2
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0
        self.input = 'Tällä vain testataan suun liikettä'
        
        self.get_logger().info('Jaw mover client initialized.')
        
        
    def timer_callback(self):

        goal_msg = FollowJointTrajectory.Goal()
        #jawPos = 0.1 + 0.4 * (self.i % 2)
        self.i = self.i % len(self.input) 

        char = self.input[self.i].lower()
        jawPos = self.synch_jaw_to_speech(char)
        trajectory_points = JointTrajectoryPoint(positions=[jawPos], time_from_start=Duration(sec=0, nanosec=150000000))
        goal_msg.trajectory = JointTrajectory(joint_names=['head_jaw_joint'], points=[trajectory_points])
        self._send_goal_future = self._action_client.wait_for_server()
        self.get_logger().info("Jaw: " + str(jawPos) + "Letter: " + char)
        self._action_client.send_goal_async(goal_msg)
        self.i += 1

    def synch_jaw_to_speech(self, char):
        vowels = ['a','e','i','o','u','y','ä','ö']
        close = ['i','y','u']
        closeMid = ['e','o','ö']
        open = ['a','ä']
        consonants = ['b','d','f','g','h','j','k','l','m','n','p','r','s','t','v']
        bilabial = ['m','p','b']
        labiodental = ['v','f']
        alveolar = ['t','d','n','s','r','l']
        palatalGlottal = ['j','h']
        velar = ['k','g']
        if char in vowels:
            if char in close:
                return 0.2
            elif char in closeMid:
                return 0.25
            else:
                return 0.35
        else:
            if char in bilabial:
                return 0.05 #Should be 0.0, not sure if hardware can handle
            elif char in labiodental:
                return 0.05
            elif char in alveolar:
                return 0.1
            elif char in velar:
                return 0.2
            else:
                return 0.15

def main():
    print('Hello from jaw_movement.')

    rclpy.init()

    action_client = JawMoverNode()

    rclpy.spin(action_client)

    # Shutdown
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
