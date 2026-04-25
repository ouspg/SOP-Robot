import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from face_tracker_msgs.msg import Faces, Face
from sop_robot_common.contracts import ARM_ACTION_TOPIC, FACES_TOPIC, FOCUSED_FACE_TOPIC

from time import time

from enum import Enum

class State(Enum):
    THINKING = 0
    IDLE = 1
    LISTENING = 2

class FullDemoNode(Node):
    def __init__(self):
        super().__init__("full_demo")

        # --- Voice chatbot integration (LLM-based) ---
        # Publish greetings directly to voice_chatbot TTS (bypasses LLM)
        self.tts_message_publisher = self.create_publisher(
            String, "/voice_chatbot/assistant_text", 10
        )
        # Control STT listening via voice_chatbot's can_listen topic
        self.stt_can_listen_publisher = self.create_publisher(
            Bool, "/voice_chatbot/can_listen", 10
        )
        # Receive transcribed user speech from voice_chatbot STT
        self.transcript_subscription = self.create_subscription(
            String, "/voice_chatbot/transcript", self.on_speech_recognized, 10
        )
        # Monitor voice_chatbot status (speaking, listening, etc.)
        self.voice_status_subscription = self.create_subscription(
            String, "/voice_chatbot/status", self.on_voice_status, 10
        )
        # Listen for TTS done to know when the robot finished speaking
        self.tts_done_subscription = self.create_subscription(
            String, "/voice_chatbot/tts_done", self.on_tts_done, 10
        )
        # Global can_listen from TTS node (backward compat with jaw_movement etc.)
        self.tts_can_listen_subscription = self.create_subscription(
            Bool, "/can_listen", self.update_tts_ready, 10
        )

        # --- Face tracking (unchanged) ---
        self.face_list_subscription = self.create_subscription(
            Faces, FACES_TOPIC, self.update_face_count, 2
        )
        self.arm_action_publisher = self.create_publisher(String, ARM_ACTION_TOPIC, 10)
        self.focused_face_subscription = self.create_subscription(
            Face, FOCUSED_FACE_TOPIC, self.focused_face_callback, 10
        )

        self.tts_ready = True
        self.robot_state = State.IDLE
        # Start with listening disabled
        self.stt_can_listen_publisher.publish(Bool(data=False))
        self.get_logger().info("switched state to IDLE")

        # Stores faces that have been greeted to not keep saying hello over and over again
        self.face_greet_time = {}  # Store the last greet time for each face_id


    def say_hello(self, greeting):
        if self.tts_ready and self.robot_state == State.IDLE:
            self.get_logger().info(f"Greeting person: {greeting}")
            self.robot_state = State.LISTENING
            self.get_logger().info("switched state to LISTENING")
            # Send greeting to voice_chatbot TTS
            self.tts_message_publisher.publish(String(data=greeting))
            self.arm_action_publisher.publish(String(data="hold"))
            self.t = self.create_timer(30, self.close_timer)
            # Enable STT listening after greeting
            self.stt_can_listen_publisher.publish(Bool(data=True))

    def update_face_count(self, message):
        self.face_count = len(message.faces)

    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def on_voice_status(self, msg):
        """Monitor voice chatbot status for state coordination."""
        status = msg.data
        if status == "speaking":
            self.tts_ready = False
        elif status in ("ready", "listening"):
            self.tts_ready = True

    def on_tts_done(self, msg):
        """Called when voice_chatbot TTS finishes speaking."""
        self.tts_ready = True
        if self.robot_state == State.THINKING:
            # After LLM response is spoken, resume listening
            self.resume_listening()

    def resume_listening(self):
        self.robot_state = State.LISTENING
        self.get_logger().info("switched state to LISTENING")
        # Reset timeout to return to IDLE if nothing else heard
        if hasattr(self, 't'):
            self.destroy_timer(self.t)
        self.t = self.create_timer(30, self.close_timer)
        self.stt_can_listen_publisher.publish(Bool(data=True))

    def close_timer(self):
        self.destroy_timer(self.t)
        self.robot_state = State.IDLE
        # Disable STT when going idle
        self.stt_can_listen_publisher.publish(Bool(data=False))
        self.arm_action_publisher.publish(String(data="zer"))
        self.get_logger().info("switched state to IDLE")


    def on_speech_recognized(self, msg):
        """Called when voice_chatbot STT transcribes user speech.

        The LLM node automatically picks up user_text and generates a
        response which flows to TTS. We just need to update state and
        reset the timeout.
        """
        if self.robot_state == State.LISTENING:
            self.get_logger().info("Heard: " + msg.data)
            if hasattr(self, 't'):
                self.destroy_timer(self.t)
            # The voice chatbot pipeline handles LLM -> TTS automatically
            self.robot_state = State.THINKING
            self.get_logger().info("switched state to THINKING")

    def focused_face_callback(self, face):
        '''
        Callback function to decide what to do when a new face is focused.

        If the face has been seen before, the person will be greeted again.
        This can only occur once every 2 minutes per person.
        '''
        num_occurrences = len(face.occurances)

        self.get_logger().info(f"Occurrances: {num_occurrences}")

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
                        self.get_logger().info(f"Already greeted {face.face_id} within the previous 2 minutes.")
                        return

                # Update the last greet time for this face_id
                self.face_greet_time[face.face_id] = current_time

                self.say_hello(greeting="Tervetuloa takaisin")
        else:
            self.say_hello(greeting="Hei, kysy minulta mitä vaan")

def main():
    rclpy.init()
    demo_node = FullDemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
