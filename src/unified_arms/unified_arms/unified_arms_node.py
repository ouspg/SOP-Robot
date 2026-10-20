import random
import time
from math import isfinite
from pathlib import Path

import rclpy
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from unified_arms.shoulder_simulation import joint_positions, pose_positions, servo_commands

# Define the Arduino serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Replace with the appropriate port
BAUD_RATE = 115200

# Define the ROS2 topic names
R_SHOULDER_TOPIC = 'r_shoulder_controller/joint_trajectory'
L_SHOULDER_TOPIC = 'l_shoulder_controller/joint_trajectory'
FEEDBACK_TOPIC_NAME = 'feedback'


class UnifiedArms(Node):
    """Coordinate unified arm and hand gestures."""

    def __init__(self):
        # Shoulder controller
        super().__init__('unified_arms')

        self.declare_parameter('simulation', False)
        self.declare_parameter('movement_duration', 0.5)
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        self.serial = None
        # Initialize the serial connection
        if not self.simulation:
            try:
                self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE)
                self.get_logger().info('Serial port opened')
            except serial.SerialException:
                self.simulation = True
                self.get_logger().info('Could not open serial port. Assuming fake robot.')
        calibration_file = (
            Path(get_package_share_directory('inmoov_description'))
            / 'config'
            / 'shoulder_simulation.yaml'
        )
        self.shoulder_calibration = yaml.safe_load(calibration_file.read_text())
        if self.simulation:
            self.shoulder_publishers = {
                'r': self.create_publisher(JointTrajectory, R_SHOULDER_TOPIC, 10),
                'l': self.create_publisher(JointTrajectory, L_SHOULDER_TOPIC, 10),
            }
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
        self.ids = sorted(
            settings['servo_id']
            for settings in self.shoulder_calibration['joints'].values()
            if 'servo_id' in settings
        )
        self.neutral_pose = [
            settings['neutral_degrees']
            for settings in sorted(
                self.shoulder_calibration['joints'].values(),
                key=lambda settings: settings['pose_index'],
            )
        ]
        self.pos = [45, 45, -1, -1, -1, -1]
        self.zero = self.neutral_pose[:6]
        self.hold = [70, -1, 70, -1, -1, -1]
        # Create main program subscriber
        self.gesture_subscription = self.create_subscription(
            String,
            '/arms/arm_action',
            self.action_callback,
            10,
        )

        self.SHOULDER_POSITIONS = {
            'zero': self.neutral_pose[:8],
            'rps_1': [55.0, 110.0, 10.0, 79.0, 80.0, 45.0, 0.0, 0.0],
            'rps_2': [35.0, 70.0, 10.0, 79.0, 80.0, 75.0, 0.0, 0.0],
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

        if arg.startswith(('r_arm_', 'l_arm_')):
            self.arm_gesture(arg[6:], 'right' if arg.startswith('r_') else 'left')
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
            'L_shoulder lift',
            'L_upper arm roll',
            'L_bicep',
        ]
        if self.simulation:
            joints.extend(['R_shoulder out', 'L_shoulder out', 'R_wrist roll', 'L_wrist roll'])
        while len(command) < len(joints):
            angle = float(input(f'Angle for {joints[i]} joint: '))
            if isinstance(angle, float) and 0 <= angle <= 180:
                command.append(angle)
                i += 1
            else:
                print('Angle must be int. Safe values are 0 - 180')
        self.logger.info('Sending positions')
        self.arm_gesture(command)

    def arm_gesture(self, action, hand='both'):
        self.logger.info(f'Action: {action}')

        if self.simulation:
            self.simulate_gesture(action, hand)
            return

        try:
            commands = servo_commands(self.gesture_pose(action), self.shoulder_calibration, hand)
        except (TypeError, ValueError) as error:
            self.logger.warning(f'Invalid arm command: {error}')
            return
        if not commands:
            return
        # Prepare the command to be sent to the Arduino
        command = ','.join(f'{servo_id}:{angle}' for servo_id, angle in commands.items()) + '\n'
        self.logger.info(command.strip())
        if self.serial:
            # Send the command to the Arduino
            self.serial.write(command.encode())

            # Read the feedback from the Arduino
            feedback = self.serial.readline().decode().strip()
            self.logger.info(f'Received feedback: {feedback}')

            # Publish the feedback to the ROS2 topic
            feedback_msg = String()
            feedback_msg.data = feedback
            self.publisher.publish(feedback_msg)

    def simulate_arm(self, servo_commands):
        """Move the same joints and angular distances as the serial command."""
        positions = joint_positions(servo_commands, self.shoulder_calibration)
        self.publish_simulated_positions(positions)

    def simulate_gesture(self, action, hand='both'):
        """Use every joint in a preset instead of truncating it to two servos."""
        try:
            positions = pose_positions(self.gesture_pose(action), self.shoulder_calibration, hand)
        except (TypeError, ValueError) as error:
            self.logger.warning(f'Invalid simulated arm pose: {error}')
            return
        self.publish_simulated_positions(positions)

    def gesture_pose(self, action):
        """Resolve gesture values in firmware ID order for both hardware modes."""
        if isinstance(action, (list, tuple)):
            return action
        if action.startswith('pose:'):
            return [float(angle) for angle in action[5:].split(',')]
        if action in ('zero', 'zer'):
            return self.neutral_pose
        if action in self.SHOULDER_POSITIONS:
            return self.SHOULDER_POSITIONS[action]
        if action in ('pos', 'hold'):
            return getattr(self, action)
        raise ValueError(f'Arm action not implemented: {action}')

    def publish_simulated_positions(self, positions):
        """Publish partial trajectories to each arm without resetting other joints."""
        duration = self.get_parameter('movement_duration').get_parameter_value().double_value
        if not isfinite(duration) or duration <= 0:
            self.logger.error('movement_duration must be positive')
            return
        nanoseconds = int(duration * 1_000_000_000)
        for side, publisher in self.shoulder_publishers.items():
            targets = {
                joint: angle for joint, angle in positions.items() if joint.startswith(side)
            }
            if not targets:
                continue
            point = JointTrajectoryPoint(
                positions=list(targets.values()),
                time_from_start=Duration(
                    sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000,
                ),
            )
            publisher.publish(JointTrajectory(joint_names=list(targets), points=[point]))

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
