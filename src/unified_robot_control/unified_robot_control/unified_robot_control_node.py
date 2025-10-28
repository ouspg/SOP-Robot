#!/usr/bin/env python3
"""
Unified Robot Control Node

This node provides centralized control of the entire robot system using topics only.
It integrates face tracking, speech recognition, dialogue management, and coordinates
with existing robot control nodes (face_tracker_movement, hand_gestures, unified_arms, etc.)

This node acts as a high-level coordinator that:
- Monitors robot state and sensor inputs
- Makes decisions about robot behavior
- Publishes commands to existing control nodes
- Never directly controls hardware

Author: SOP Robot Project
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool
from face_tracker_msgs.msg import Faces, Face

import time
from enum import Enum


class RobotState(Enum):
    """Robot behavior states"""
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    SPEAKING = "speaking"
    TRACKING = "tracking"


class UnifiedRobotControlNode(Node):
    """
    Unified robot control node that coordinates all robot subsystems via topics only.
    
    This node never directly controls hardware. Instead, it publishes commands to 
    existing nodes that handle the actual robot control.
    """

    def __init__(self):
        super().__init__('unified_robot_control')
        
        # Get parameters
        simulation = self.declare_parameter('simulation', True).get_parameter_value().bool_value
        enable_face_tracking = self.declare_parameter('enable_face_tracking', True).get_parameter_value().bool_value
        enable_speech = self.declare_parameter('enable_speech', True).get_parameter_value().bool_value
        enable_arms = self.declare_parameter('enable_arms', True).get_parameter_value().bool_value
        
        self.simulation = simulation
        self.get_logger().info(f"Initializing Unified Robot Control (Simulation: {simulation})")
        
        # State management
        self.robot_state = RobotState.IDLE
        self.tts_ready = True
        self.face_greet_time = {}  # Track when faces were last greeted
        
        # Initialize subscribers
        self._init_subscribers(enable_face_tracking, enable_speech)
        
        # Initialize publishers (all commands go to existing nodes)
        self._init_publishers()
        
        # Create timers
        self.idle_timer = self.create_timer(5.0, self._idle_timer_callback)
        self.state_timer = None
        
        # Tracking state
        self.current_tracked_face = None
        self.visible_faces = []
        
        self.get_logger().info("Unified Robot Control Node initialized and ready")

    def _init_subscribers(self, enable_face_tracking: bool, enable_speech: bool):
        """Initialize all subscribers"""
        
        if enable_face_tracking:
            # Face tracking - get detected faces
            self.face_list_sub = self.create_subscription(
                Faces,
                '/face_tracker/face_topic',  # Topic from face_tracker node
                self._face_list_callback,
                2
            )
            
            # Get focused face from face_tracker_movement node
            self.focused_face_sub = self.create_subscription(
                Face,
                '/face_tracker_movement/focused_face',
                self._focused_face_callback,
                10
            )
        
        if enable_speech:
            # Speech recognition - get recognized speech
            self.speech_sub = self.create_subscription(
                String,
                'speech_recognizer/recognized_speech',
                self._speech_callback,
                10
            )
            
            # QA bot - get responses
            self.chatbot_sub = self.create_subscription(
                String,
                'chatbot/chatbot_response',
                self._chatbot_callback,
                10
            )
            
            # TTS ready status
            self.tts_ready_sub = self.create_subscription(
                Bool,
                'can_listen',
                self._tts_ready_callback,
                10
            )

    def _init_publishers(self):
        """
        Initialize all publishers - these publish commands to existing control nodes
        """
        
        # Speech control - tell speech_recognizer when to listen
        self.speech_control_pub = self.create_publisher(Bool, 'speech_recognizer/can_listen', 10)
        
        # Chatbot - send recognized speech to chatbot
        self.chatbot_input_pub = self.create_publisher(String, 'chatbot/recognized_speech', 10)
        
        # TTS - send text to TTS service
        self.tts_pub = self.create_publisher(String, 'chatbot_response', 10)
        
        # Arms - send commands to unified_arms_client
        self.arm_action_pub = self.create_publisher(String, '/arms/arm_action', 10)
        
        # Hands - send gesture commands to hand_gestures node
        # (These are typically sent by unified_arms_client, but we can send them directly too)
        self.left_hand_pub = self.create_publisher(String, '/l_hand/l_hand_topic', 10)
        self.right_hand_pub = self.create_publisher(String, '/r_hand/r_hand_topic', 10)
        
        # Head gestures - send head gesture commands to face_tracker_movement node
        self.head_gesture_pub = self.create_publisher(String, '/face_tracker_movement/head_gesture_topic', 10)

    # Callback methods
    def _face_list_callback(self, msg: Faces):
        """Process incoming face detection data"""
        self.visible_faces = msg.faces
        self.get_logger().info(f"Received {len(msg.faces)} faces")

    def _focused_face_callback(self, face: Face):
        """Handle focused face from face_tracker_movement node"""
        # This is called when face_tracker_movement finds a face to track
        current_time = time.time()
        
        # Check if we should greet this face
        if face.face_id in self.face_greet_time:
            elapsed = current_time - self.face_greet_time[face.face_id]
            if elapsed < 120:  # 2 minutes cooldown
                return
        
        # Check if this is a new face or returning face
        num_occurrences = len(face.occurances)
        
        if num_occurrences == 1:
            # First time seeing this face - greet them
            self.face_greet_time[face.face_id] = current_time
            self._handle_new_face_detected()
        elif num_occurrences > 1:
            # Returning face - check if we should greet again
            # Only greet if they've been away long enough
            if face.face_id in self.face_greet_time:
                elapsed = current_time - self.face_greet_time[face.face_id]
                if elapsed >= 120:  # 2 minutes passed
                    self.face_greet_time[face.face_id] = current_time
                    self._handle_returning_face()

    def _handle_new_face_detected(self):
        """Handle when a new face is detected"""
        if self.robot_state == RobotState.IDLE:
            self.get_logger().info("New face detected - greeting person")
            
            # Transition to listening state
            self._transition_state(RobotState.LISTENING)
            
            # Send greeting via TTS
            greeting = "Hei, kysy minulta mitä vaan"
            self._say_text(greeting)
            
            # Tell unified_arms to perform hold gesture
            self._perform_arm_gesture("hold")

    def _handle_returning_face(self):
        """Handle when a previously seen face returns"""
        if self.robot_state == RobotState.IDLE:
            self.get_logger().info("Returning face detected - welcoming back")
            
            self._transition_state(RobotState.LISTENING)
            greeting = "Tervetuloa takaisin"
            self._say_text(greeting)
            self._perform_arm_gesture("hold")

    def _speech_callback(self, msg: String):
        """Handle recognized speech"""
        if self.robot_state == RobotState.LISTENING:
            self.get_logger().info(f"Heard: {msg.data}")
            
            # Transition to thinking state
            self._transition_state(RobotState.THINKING)
            
            # Stop listening while processing
            self._stop_listening()
            
            # Forward to chatbot
            chatbot_msg = String()
            chatbot_msg.data = msg.data
            self.chatbot_input_pub.publish(chatbot_msg)

    def _chatbot_callback(self, msg: String):
        """Handle chatbot response"""
        if self.robot_state == RobotState.THINKING:
            self.get_logger().info(f"Chatbot responded: {msg.data}")
            # The chatbot already publishes to chatbot_response topic
            # which TTS subscribes to, so we don't need to republish
            pass

    def _tts_ready_callback(self, msg: Bool):
        """Handle TTS ready status"""
        self.tts_ready = msg.data
        if self.tts_ready and self.robot_state == RobotState.SPEAKING:
            self._resume_listening()

    # State management methods
    def _transition_state(self, new_state: RobotState):
        """Transition to a new state"""
        if self.robot_state != new_state:
            self.get_logger().info(f"State transition: {self.robot_state.value} -> {new_state.value}")
            self.robot_state = new_state

    def _say_text(self, text: str):
        """Send text to TTS service"""
        self._transition_state(RobotState.SPEAKING)
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        
        # Set timer to resume listening after speech
        self.state_timer = self.create_timer(1.0, self._check_tts_and_resume)

    def _check_tts_and_resume(self):
        """Check if TTS is done and resume listening"""
        if self.tts_ready:
            self.destroy_timer(self.state_timer)
            self._resume_listening()

    def _resume_listening(self):
        """Resume listening for speech"""
        self._transition_state(RobotState.LISTENING)
        self._start_listening()
        
        # Set timeout to return to idle
        if self.state_timer is not None:
            self.destroy_timer(self.state_timer)
        self.state_timer = self.create_timer(30.0, self._return_to_idle)

    def _return_to_idle(self):
        """Return to idle state"""
        if self.state_timer is not None:
            self.destroy_timer(self.state_timer)
            self.state_timer = None
        
        self._transition_state(RobotState.IDLE)
        
        # Tell unified_arms to return to zero position
        self._perform_arm_gesture("zero")

    def _start_listening(self):
        """Start speech recognition by publishing to speech_recognizer"""
        msg = Bool()
        msg.data = True
        self.speech_control_pub.publish(msg)

    def _stop_listening(self):
        """Stop speech recognition"""
        msg = Bool()
        msg.data = False
        self.speech_control_pub.publish(msg)

    def _perform_arm_gesture(self, gesture: str):
        """
        Perform an arm gesture by publishing to unified_arms_client
        
        Available gestures:
        - "zero": Return to zero position
        - "hold": Hold position
        - "wave": Wave gesture
        - "rock": Rock gesture
        - "test": Test sequence
        """
        if gesture in ["zero", "hold", "wave", "rock", "test"]:
            msg = String()
            msg.data = gesture
            self.arm_action_pub.publish(msg)
            self.get_logger().info(f"Sent arm gesture command: {gesture}")
        else:
            self.get_logger().warn(f"Unknown arm gesture: {gesture}")

    def _send_hand_gesture(self, hand: str, gesture: str):
        """
        Send a hand gesture command to hand_gestures node
        
        Args:
            hand: "left" or "right"
            gesture: "open", "fist", "scissors", "point", "thumbs_up", etc.
        """
        msg = String()
        msg.data = gesture
        
        if hand == "left":
            self.left_hand_pub.publish(msg)
        elif hand == "right":
            self.right_hand_pub.publish(msg)
        else:
            self.get_logger().warn(f"Unknown hand: {hand}")
            return
        
        self.get_logger().info(f"Sent {hand} hand gesture: {gesture}")

    def _send_head_gesture(self, gesture: str):
        """
        Send a head gesture command to face_tracker_movement node
        
        Args:
            gesture: "nod", "shake", etc.
        """
        msg = String()
        msg.data = gesture
        self.head_gesture_pub.publish(msg)
        self.get_logger().info(f"Sent head gesture: {gesture}")

    def _idle_timer_callback(self):
        """Periodic callback for idle behavior"""
        if self.robot_state == RobotState.IDLE:
            # Can add random idle behaviors here
            pass


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = UnifiedRobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
