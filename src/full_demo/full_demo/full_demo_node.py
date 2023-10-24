import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from face_tracker_msgs.msg import Faces
from face_tracker_msgs.msg import Point2

from enum import Enum

class State(Enum):
    THINKING = 0
    IDLE = 1
    LISTENING = 2

class FullDemoNode(Node):
    def __init__(self):
        super().__init__("full_demo")
        # ROS topics
        self.tts_message_publisher = self.create_publisher(String, "response", 10)
        self.tts_can_listen_subscription = self.create_subscription(Bool, "can_listen", self.update_tts_ready,10)

        self.speech_recognizer_can_listen = self.create_publisher(Bool, "speech_recognizer/can_listen", 10)
        self.speech_recognizer_result = self.create_subscription(String, "speech_recognizer/recognized_speech", self.on_speech_recognized,10)

        self.chatbot_input = self.create_publisher(String, "chatbot/recognized_speech", 10)
        self.chatbot_output = self.create_subscription(String, "chatbot/chatbot_response", self.on_chatbot_response,10)

        self.face_list_subscription = self.create_subscription(Faces, "/face_tracker/face_topic", self.update_face_count, 2)
        self.new_face_subscription = self.create_subscription(Point2, "/face_tracker/face_location_topic", self.say_hello, 2)
        self.tts_ready = True
        # Turn off listening for now
        self.speech_recognizer_can_listen.publish(Bool(data=False))
        self.robot_state = State.IDLE

    def say_hello(self, msg):
        if self.tts_ready and self.robot_state == State.IDLE:
            req = String()
            req.data = "Hei, kysy minulta mitä vaan"
            self.speech_recognizer_can_listen.publish(Bool(data=True))
            self.robot_state = State.LISTENING
            self.get_logger().info("switched state to LISTENING")
            self.tts_message_publisher.publish(req)
            self.t = self.create_timer(20, self.close_timer)
    
    def update_face_count(self, message):
        self.face_count = len(message.faces)
    
    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def on_chatbot_response(self, msg):
        if self.tts_ready:
            # Speak out the message and continue listening
            self.tts_message_publisher.publish(msg)
            self.speech_recognizer_can_listen.publish(Bool(data=True))
            self.robot_state = State.LISTENING
            self.get_logger().info("switched state to LISTENING")
            # Set timeout to return to IDLE if nothing else heard
            self.t = self.create_timer(30, self.close_timer)

    def close_timer(self):
        self.t.cancel()
        self.robot_state = State.IDLE
        self.get_logger().info("switched state to IDLE")


    def on_speech_recognized(self, msg):
        if self.robot_state == State.LISTENING:
            self.get_logger().info("I heard something...")
            self.t.cancel()
            # When speech recognized, stop listening
            self.robot_state = State.THINKING
            self.get_logger().info("switched state to THINKING")
            self.speech_recognizer_can_listen.publish(Bool(data=False))
            self.chatbot_input.publish(msg)

def main():
    rclpy.init()
    demo_node = FullDemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
