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

    """
    TODO: 
    Get input from speech synthesis
    """
    def __init__(self):
        super().__init__('jaw_mover_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/jaw_controller/follow_joint_trajectory')
        
        timer_period = 0.15
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0
        self.jawPos = 0.0
        self.charDuration = Duration(sec=0, nanosec=0)

        self.input = 'Tällä vain testataan suun liikettä ppppp pppppp ooooooooo '
        
        self.get_logger().info('Jaw mover client initialized.')
        
        
    """
    TODO: 
    1. Handle each input string only once instead of looping -> recognize end of string
    2. Get real duration of characters (into variable charDuration)
    3. Handle spaces in string (position, duration)
    """
    def timer_callback(self):

        goal_msg = FollowJointTrajectory.Goal()
        self.i = self.i % len(self.input) 
        char = self.input[self.i].lower()
        if char != ' ':
            self.synch_jaw_to_speech(char)
        trajectory_points = JointTrajectoryPoint(positions=[self.jawPos], time_from_start=self.charDuration)
        goal_msg.trajectory = JointTrajectory(joint_names=['head_jaw_joint'], points=[trajectory_points])
        self._send_goal_future = self._action_client.wait_for_server()
        self.get_logger().info('Jaw: ' + str(self.jawPos) + ' Letter: ' + char)
        self._action_client.send_goal_async(goal_msg)
        self.i += 1
    
    """
    TODO: 
    Define and return real duration for each character in speech
    """
    def synch_jaw_to_speech(self, char):

        vowels = ['a','e','i','o','u','y','ä','ö']
        rounded = ['o','u','y','ö']
        bilabial = ['m','p','b']
        labiodental = ['v','f']
        coronal = ['t','d','n','s','r','l']
        if char in vowels:
            if char in rounded:
                self.jawPos = 0.35
                self.charDuration = Duration(sec=0, nanosec=0)
            else:
                self.jawPos = 0.5
                self.charDuration = Duration(sec=0, nanosec=0)
        else:
            if char in bilabial:
                self.jawPos = 0.0
                self.charDuration = Duration(sec=0, nanosec=0)
            elif char in labiodental:
                self.jawPos = 0.1
                self.charDuration = Duration(sec=0, nanosec=0)
            elif char in coronal:
                self.jawPos = 0.2
                self.charDuration = Duration(sec=0, nanosec=0)
            else:
                self.jawPos = 0.3
                self.charDuration = Duration(sec=0, nanosec=0)

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
