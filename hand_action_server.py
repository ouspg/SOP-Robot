import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node

from robot.action import Test

class HandActionServer(Node):
    def __init__(self):
        super().__init__('hand_action_server')
        self._action_server = ActionServer(
            self,
            Test, # type of action
            'test', # action name
            self.execute_callback   # callback function
        )

    def execute_callback(self, goal_handle):
        """This function MUST return a result message for the action type'
        goal_handle: the action the hand is supposed to make"""
        self.get_logger().info('Executing order')
        command = goal_handle.request.order
        
        feedback_msg = Test.Feedback()
        feedback_msg.moving = 0

        if command == 2:
            self.get_logger().info("rock")
            feedback_msg.moving = 120
            self.get_logger().info(f"Moving towards: {feedback_msg.moving}")
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

        result = Test.Result()
        result.finish = feedback_msg.moving
        return result

def main(args=None):
    rclpy.init(args=args)
    test_action_server = HandActionServer()
    rclpy.spin(test_action_server)

if __name__ == "__main__":
    main()
