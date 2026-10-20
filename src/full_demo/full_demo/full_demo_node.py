from enum import Enum
from time import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from face_tracker_msgs.msg import Face, Faces


class State(Enum):
    THINKING = 0
    IDLE = 1
    LISTENING = 2


class FullDemoNode(Node):
    def __init__(self):
        super().__init__('full_demo')
        # ROS topics
        self.tts_package_message_publisher = self.create_publisher(
            String, '/tts_package/chatbot_response', 10
        )
        self.tts_package_can_listen_subscription = self.create_subscription(
            Bool, '/tts_package/can_listen', self.update_tts_ready, 10
        )

        self.sst_package_can_listen_publisher = self.create_publisher(
            Bool, '/sst_package/can_listen', 10
        )
        self.sst_package_result_subscription = self.create_subscription(
            String, '/sst_package/recognized_speech', self.on_speech_recognized, 10
        )

        self.llm_package_input_publisher = self.create_publisher(
            String, '/llm_package/recognized_speech', 10
        )
        self.llm_package_output_subscription = self.create_subscription(
            String, '/llm_package/chatbot_response', self.on_llm_response, 10
        )

        self.face_list_subscription = self.create_subscription(
            Faces, '/face_tracker/faces', self.update_face_count, 2
        )

        self.arm_action_publisher = self.create_publisher(String, '/arms/arm_action', 10)

        # Get focused face from face_tracker_movement_node
        self.focused_face_subscription = self.create_subscription(
            Face, '/face_tracker_movement/focused_face', self.focused_face_callback, 10
        )

        self.tts_ready = True
        # Turn off listening for now
        self.sst_package_can_listen_publisher.publish(Bool(data=False))
        self.robot_state = State.IDLE
        self.get_logger().info('switched state to IDLE')

        # Stores faces that have been greeted to not keep saying hello over and over again
        self.face_greet_time = {}  # Store the last greet time for each face_id

    def say_hello(self, greeting):
        if self.tts_ready and self.robot_state == State.IDLE:
            self.get_logger().info(f'Greeting person: {greeting}')
            self.robot_state = State.LISTENING
            self.get_logger().info('switched state to LISTENING')
            self.tts_package_message_publisher.publish(String(data=greeting))
            self.arm_action_publisher.publish(String(data='hold'))
            self.t = self.create_timer(30, self.close_timer)
            self.sst_package_can_listen_publisher.publish(Bool(data=True))

    def update_face_count(self, message):
        self.face_count = len(message.faces)

    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def on_llm_response(self, msg):
        if self.tts_ready:
            # Speak out the message and continue listening
            self.tts_package_message_publisher.publish(msg)
            self.speech_timer = self.create_timer(5, self.resume_listening)

    def resume_listening(self):
        self.destroy_timer(self.speech_timer)
        if not self.tts_ready:
            self.speech_timer = self.create_timer(1, self.resume_listening)
        else:
            self.robot_state = State.LISTENING
            self.get_logger().info('switched state to LISTENING')
            # Set timeout to return to IDLE if nothing else heard
            self.t = self.create_timer(30, self.close_timer)
            self.sst_package_can_listen_publisher.publish(Bool(data=True))

    def close_timer(self):
        self.destroy_timer(self.t)
        self.robot_state = State.IDLE
        self.arm_action_publisher.publish(String(data='zer'))
        self.get_logger().info('switched state to IDLE')

    def on_speech_recognized(self, msg):
        if self.robot_state == State.LISTENING:
            self.get_logger().info('Heard: ' + msg.data)
            self.destroy_timer(self.t)
            # When speech recognized, stop listening
            self.robot_state = State.THINKING
            self.get_logger().info('switched state to THINKING')
            self.sst_package_can_listen_publisher.publish(Bool(data=False))
            self.llm_package_input_publisher.publish(msg)

    def focused_face_callback(self, face):
        """
        Callback function to decide what to do when a new face is focused.

        If the face has been seen before, the person will be greeted again.
        This can only occur once every 2 minutes per person.
        """
        num_occurrences = len(face.occurances)

        self.get_logger().info(f'Occurrances: {num_occurrences}')

        current_time = time()

        if num_occurrences > 1:
            if face.occurances[-1].duration < 20:
                # Check that at least one occurance is long enough
                for occurance in face.occurances:
                    if occurance.duration > 6:
                        break
                else:
                    # All occurances are too short
                    return

                if face.face_id in self.face_greet_time:
                    # Calculate time since the last greeting
                    elapsed_time = current_time - self.face_greet_time[face.face_id]
                    if elapsed_time < 120:
                        self.get_logger().info(
                            f'Already greeted {face.face_id} within the previous 2 minutes.'
                        )
                        return

                # Update the last greet time for this face_id
                self.face_greet_time[face.face_id] = current_time

                self.say_hello(greeting='Tervetuloa takaisin')
        else:
            self.say_hello(greeting='Hei, kysy minulta mitä vaan')


def main():
    rclpy.init()
    demo_node = FullDemoNode()
    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
