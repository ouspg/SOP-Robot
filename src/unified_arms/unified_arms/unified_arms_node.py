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

import random
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

# Define the Arduino serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Replace with the appropriate port
BAUD_RATE = 115200

# Define the ROS2 topic names
TOPIC_NAME = 'shoulder_controller/joint_trajectory'
FEEDBACK_TOPIC_NAME = 'feedback'


class UnifiedArms(Node):
    """Coordinate unified arm and hand gestures."""

    def __init__(self):
        # Shoulder controller
        super().__init__('unified_arms')

        # Initialize the serial connection
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE)
            print('Serial port opened')
        except serial.SerialException:
            self.serial = None
            print('Could not open serial port. Assuming fake robot.')
        # Create a ROS2 publisher for the feedback
        self.publisher = self.create_publisher(
            String,
            FEEDBACK_TOPIC_NAME,
            10,
        )
        self.left_hand_gesture_publisher = self.create_publisher(
            String,
            '/l_hand/l_hand_topic',
            10,
        )
        self.right_hand_gesture_publisher = self.create_publisher(
            String,
            '/r_hand/r_hand_topic',
            10,
        )
        self.right_arm_servo_indices = range(0, 4)
        self.left_arm_servo_indices = range(4, 8)
        self.legacy_positions = {
            'pos': [15, 30],
            'zer': [0, 0],
            'hold': [40, -20],
        }
        # Create main program subscriber
        self.gesture_subscription = self.create_subscription(
            String,
            '/arms/arm_action',
            self.action_callback,
            10,
        )

        self.SHOULDER_POSITIONS = {
            'zero': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'rps_1': [25.0, 20.0, 0.0, 0.0, 45.0, 0.0, 35.0, 0.0],
            'rps_2': [5.0, -20.0, 0.0, 0.0, 45.0, 0.0, 65.0, 0.0],
        }

        self.HAND_ACTIONS = [
            'open',
            'fist',
            'scissors',
            'point',
            'thumbs_up',
            'grasp',
            'pen_grasp',
            'hard_rock',
            'rps',
            'funny',
            'three',
        ]

        self.ACTION_PATTERNS = {
            # Each entry is (hand/arm, sleep_after, action, left/right/both).
            # left or right can be left out to default to both
            'zero': [
                ('hand', 1, 'fist', 'left'),
                ('hand', 1, 'fist', 'right'),
                ('arm', 0, 'zero', 'both'),
            ],
            'test': [
                ('hand', 1, 'fist', 'left'),
                ('hand', 1, 'fist', 'right'),
                ('arm', 1, 'zero'),
                ('hand', 1, 'open', 'left'),
                ('hand', 1, 'open', 'right'),
                ('hand', 1, 'fist', 'left'),
                ('hand', 0, 'fist', 'right'),
            ],
            'wave': [
                ('hand', 0.5, 'fist', 'left'),
                ('arm', 0.5, 'rps_1'),
                ('arm', 0.5, 'rps_2'),
                ('arm', 0.5, 'rps_1'),
                ('arm', 0.5, 'rps_2'),
                ('arm', 0.5, 'rps_1'),
                ('arm', 0.5, 'zero'),
                ('hand', 0, 'open', 'left'),
            ],
            'rock': [
                ('hand', 1, 'hard_rock', 'left'),
                ('arm', 1, 'rps_1'),
                ('arm', 1, 'rps_2'),
                ('arm', 1, 'rps_1'),
                ('arm', 1, 'rps_2'),
                ('arm', 1, 'rps_1'),
                ('arm', 1, 'rps_2'),
                ('arm', 1, 'zero'),
                ('hand', 0, 'open', 'left'),
            ],
        }

        self.exit_commands = ['quit', 'exit']
        self.logger = self.get_logger()

    def action_callback(self, msg):
        arg = msg.data
        self.logger.info(f'arg: {arg}')

        # Action messages for hands can be eg. "l_hand_fist"
        if arg[0:7] in ('r_hand_', 'l_hand_'):
            # Action is for hands (fingers)
            hand = 'right' if arg[0:2] == 'r_' else 'left'
            hand_action = arg[7:]
            if hand_action in self.HAND_ACTIONS:
                self.hand_gesture(hand_action, hand)
            return

        if arg in self.ACTION_PATTERNS:
            # Action is a pattern
            self.perform_action_from_pattern(arg)
        else:
            self.arm_gesture(arg)

    def perform_action_from_pattern(self, pattern):
        if pattern not in self.ACTION_PATTERNS:
            self.logger.info('Action pattern not implemented')
            return
        for action_data in self.ACTION_PATTERNS[pattern]:
            # action_data is (hand/arm, sleep_after, action, side).
            hand_or_arm = action_data[0]
            sleep_after = action_data[1]
            action = action_data[2]
            side = action_data[3] if len(action_data) == 4 else 'both'
            if hand_or_arm == 'hand':
                self.hand_gesture(action, side)
            elif hand_or_arm == 'arm':
                self.arm_gesture(action, side)
            time.sleep(sleep_after)

    def trial(self):
        command = []
        i = 0
        joints = [
            'R_shoulder lift',
            'R_upper arm roll',
            'R_bicep',
            'R_shoulder out',
            'L_shoulder lift',
            'L_upper arm roll',
            'L_bicep',
            'L_shoulder out',
        ]
        while len(command) < 8:
            angle = float(input(f'Angle for {joints[i]} joint: '))
            if isinstance(angle, float) and -90 <= angle <= 90:
                command.append(angle)
                i += 1
            else:
                print('Angle must be a number between -90 and 90')
        self.logger.info('Sending positions')
        self.arm_gesture(command)

    def arm_gesture(self, action, hand='both'):
        self.logger.info(f'Action: {action}')

        if isinstance(action, list):
            positions = action
            servo_indices = range(min(len(positions), 8))
        elif action in self.SHOULDER_POSITIONS:
            positions = self.SHOULDER_POSITIONS[action]
            if hand == 'right':
                servo_indices = self.right_arm_servo_indices
            elif hand == 'left':
                servo_indices = self.left_arm_servo_indices
            elif hand == 'both':
                servo_indices = range(len(positions))
            else:
                self.logger.info(f"Expected 'left', 'right', or 'both' instead of '{hand}'")
                return
        elif action in self.legacy_positions:
            positions = self.legacy_positions[action]
            servo_indices = range(len(positions))
        else:
            self.logger.info(f"Arm action '{action}' is not implemented")
            return

        angles = [
            f'{servo_index}:{round(positions[servo_index])}' for servo_index in servo_indices
        ]
        # Prepare the command to be sent to the Arduino
        command = ','.join(angles)
        self.logger.info(command)
        # Note: Could return at the start of function
        # but this leaves room for implementation for fake robot
        if self.serial:
            # Send the command to the Arduino
            self.serial.write(f'{command}\n'.encode())

            # Read the feedback from the Arduino
            feedback = self.serial.readline().decode().strip()
            self.logger.info(f'Received feedback: {feedback}')

            # Publish the feedback to the ROS2 topic
            feedback_msg = String()
            feedback_msg.data = feedback
            self.publisher.publish(feedback_msg)

    def hand_gesture(self, gesture, hand='both'):
        msg = String()
        msg.data = gesture
        if hand not in ('right', 'left', 'both'):
            self.logger.info(f"Should be 'left' or 'right' instead of: '{hand}'")
            return
        if hand in ('right', 'both'):
            self.right_hand_gesture_publisher.publish(msg)
        if hand in ('left', 'both'):
            self.left_hand_gesture_publisher.publish(msg)

    def list_available_commands(self):
        print('Available commands:', end=' ')
        for command in self.HAND_ACTIONS[:-1]:
            print(command, end=', ')
        print(self.HAND_ACTIONS[-1])
        print("You can also input 'quit' or 'exit' to quit.")

    def rps(self):
        self.logger.info('Starting rps')
        self.hand_gesture('three')
        time.sleep(2)
        self.hand_gesture('scissors')
        time.sleep(2)
        self.hand_gesture('point')
        time.sleep(2)
        self.hand_gesture(random.choice(['open', 'fist', 'scissors']))


def main(args=None):
    rclpy.init(args=args)
    arm_controller = UnifiedArms()
    print('Arm controller ready for messages.')
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
