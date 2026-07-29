# Copyright 2026 Aapo2001
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        self.tts_message_publisher = self.create_publisher(
            String,
            'chatbot_response',
            10,
        )
        self.tts_can_listen_subscription = self.create_subscription(
            Bool,
            'can_listen',
            self.update_tts_ready,
            10,
        )

        self.sst_can_listen = self.create_publisher(
            Bool,
            'can_listen',
            10,
        )
        self.sst_result = self.create_subscription(
            String,
            'recognized_speech',
            self.on_speech_recognized,
            10,
        )

        self.llm_output = self.create_subscription(
            String,
            'chatbot_response',
            self.on_llm_response,
            10,
        )

        self.face_list_subscription = self.create_subscription(
            Faces,
            '/face_tracker/face_topic',
            self.update_face_count,
            2,
        )
        self.arm_action_publisher = self.create_publisher(
            String,
            '/arms/arm_action',
            10,
        )
        self.focused_face_subscription = self.create_subscription(
            Face,
            '/face_tracker_movement/focused_face',
            self.focused_face_callback,
            10,
        )

        self.tts_ready = True
        self.sst_can_listen.publish(Bool(data=False))
        self.robot_state = State.IDLE
        self.get_logger().info('switched state to IDLE')

        self.face_greet_time = {}

    def say_hello(self, greeting):
        if self.tts_ready and self.robot_state == State.IDLE:
            self.get_logger().info(f'Greeting person: {greeting}')
            self.robot_state = State.LISTENING
            self.get_logger().info('switched state to LISTENING')
            self.tts_message_publisher.publish(String(data=greeting))
            self.arm_action_publisher.publish(String(data='hold'))
            self.t = self.create_timer(30, self.close_timer)
            self.sst_can_listen.publish(Bool(data=True))

    def update_face_count(self, message):
        self.face_count = len(message.faces)

    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def on_llm_response(self, _message):
        if self.tts_ready:
            self.speech_timer = self.create_timer(5, self.resume_listening)

    def resume_listening(self):
        self.destroy_timer(self.speech_timer)
        if not self.tts_ready:
            self.speech_timer = self.create_timer(1, self.resume_listening)
        else:
            self.robot_state = State.LISTENING
            self.get_logger().info('switched state to LISTENING')
            self.t = self.create_timer(30, self.close_timer)
            self.sst_can_listen.publish(Bool(data=True))

    def close_timer(self):
        self.destroy_timer(self.t)
        self.robot_state = State.IDLE
        self.arm_action_publisher.publish(String(data='zer'))
        self.get_logger().info('switched state to IDLE')

    def on_speech_recognized(self, msg):
        if self.robot_state == State.LISTENING:
            self.get_logger().info(f'Heard: {msg.data}')
            self.destroy_timer(self.t)
            self.robot_state = State.THINKING
            self.get_logger().info('switched state to THINKING')
            self.sst_can_listen.publish(Bool(data=False))

    def focused_face_callback(self, face):
        """
        Decide what to do when a newly focused face is observed.

        If the face has been seen before, greet the person again. This can
        happen at most once every two minutes for each person.
        """
        num_occurrences = len(face.occurances)

        self.get_logger().info(f'Occurrances: {num_occurrences}')
        current_time = time()

        if num_occurrences > 1:
            if face.occurances[-1].duration < 20:
                for occurance in face.occurances:
                    if occurance.duration > 6:
                        break
                else:
                    return

                if face.face_id in self.face_greet_time:
                    elapsed_time = current_time - self.face_greet_time[face.face_id]
                    if elapsed_time < 120:
                        self.get_logger().info(
                            f'Already greeted {face.face_id} within the previous 2 minutes.'
                        )
                        return

                self.face_greet_time[face.face_id] = current_time
                self.say_hello(greeting='Tervetuloa takaisin')
        else:
            self.say_hello(greeting='Hei, kysy minulta mitä vaan')


def main(args=None):
    rclpy.init(args=args)
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
