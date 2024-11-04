import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from face_tracker_msgs.msg import Faces, Face
from face_tracker_msgs.msg import Point2

from time import time

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

        self.arm_action_publisher = self.create_publisher(String, "/arms/arm_action", 10)

        # Get focused face from face_tracker_movement_node
        self.focused_face_subscription = self.create_subscription(Face, "/face_tracker_movement/focused_face", self.focused_face_callback, 10)

        self.tts_ready = True
        # Turn off listening for now
        self.speech_recognizer_can_listen.publish(Bool(data=False))
        self.robot_state = State.IDLE
        self.get_logger().info("switched state to IDLE")

        # Stores faces that have been greeted to not keep saying hello over and over again
        self.face_greet_time = {}  # Store the last greet time for each face_id
        self.last_greet_time = 0 # Store the time of the last hello


    def say_hello(self, msg):
        if self.tts_ready and self.robot_state == State.IDLE:
            self.robot_state = State.LISTENING
            self.get_logger().info("switched state to LISTENING")
            self.tts_message_publisher.publish(String(data="Hei, kysy minulta mitä vaan"))
            self.arm_action_publisher.publish(String(data="wave"))
            self.t = self.create_timer(30, self.close_timer)
            self.speech_recognizer_can_listen.publish(Bool(data=True))
    
    def update_face_count(self, message):
        self.face_count = len(message.faces)
    
    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def on_chatbot_response(self, msg):
        if self.tts_ready:
            # Speak out the message and continue listening
            self.tts_message_publisher.publish(msg)
            self.speech_timer = self.create_timer(5, self.resume_listening)

    def resume_listening(self):
        self.destroy_timer(self.speech_timer)
        if not self.tts_ready:
            self.speech_timer = self.create_timer(1, self.resume_listening)
        else:
            self.robot_state = State.LISTENING
            self.get_logger().info("switched state to LISTENING")
            # Set timeout to return to IDLE if nothing else heard
            self.t = self.create_timer(30, self.close_timer)
            self.speech_recognizer_can_listen.publish(Bool(data=True))

    def close_timer(self):
        self.destroy_timer(self.t)
        self.robot_state = State.IDLE
        self.get_logger().info("switched state to IDLE")


    def on_speech_recognized(self, msg):
        if self.robot_state == State.LISTENING:
            self.get_logger().info("Heard: "+msg.data)
            self.destroy_timer(self.t)
            # When speech recognized, stop listening
            self.robot_state = State.THINKING
            self.get_logger().info("switched state to THINKING")
            self.speech_recognizer_can_listen.publish(Bool(data=False))
            self.chatbot_input.publish(msg)
    
    def focused_face_callback(self, face):
        '''
        Callback function to decide what to do when a new face is focused.

        If the face has been seen before, the person will be greeted again.
        This can only occur once every 2 minutes per person.
        '''
        num_occurrences = len(face.occurances)
    
        self.get_logger().info(f"Occurrances: {num_occurrences}")
        
        current_time = time()

        if num_occurrences > 1 and current_time - self.last_greet_time > 30 and face.occurances[-1].duration < 20:
            
            # Check that at least one occurance is long enough
            for occurance in face.occurances:
                if occurance.duration > 15:
                    break
            else:
                # All occurances are too short
                return

            if face.face_id in self.face_greet_time:
                # Calculate time since the last greeting
                elapsed_time = current_time - self.face_greet_time[face.face_id]
                if elapsed_time < 120:
                    self.get_logger().info(f"Already greeted {face.face_id} within the previous 2 minutes.")
                    return

            # Update the last greet time for this face_id
            self.face_greet_time[face.face_id] = current_time
            self.last_greet_time = current_time

            # Say hello
            self.get_logger().info(f"Saying hello to previously seen person: {face.face_id}")

            # This is stolen from say_hello. Should be merged into a singular command or sent further along!
            if self.tts_ready and self.robot_state == State.IDLE:
                self.robot_state = State.LISTENING
                self.get_logger().info("switched state to LISTENING")
                self.tts_message_publisher.publish(String(data="Tervetuloa takaisin"))
                # self.arm_action_publisher.publish(String(data="wave"))
                self.t = self.create_timer(30, self.close_timer)
                self.speech_recognizer_can_listen.publish(Bool(data=True))


def main():
    rclpy.init()
    demo_node = FullDemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
