import random
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# Define the Arduino serial port and baud rate
SERIAL_PORT = "/dev/ttyACM0"  # Replace with the appropriate port
BAUD_RATE = 115200

# Define the ROS2 topic names
TOPIC_NAME = "shoulder_controller/joint_trajectory"
FEEDBACK_TOPIC_NAME = "feedback"


class UnifiedArms(Node):
    def __init__(self):
        # ShoulderController
        super().__init__("minimal_publisher")
        self.logger = self.get_logger()

        serial_port = (
            self.declare_parameter("serial_port", SERIAL_PORT).get_parameter_value().string_value
        )
        baud_rate = (
            self.declare_parameter("baud_rate", BAUD_RATE).get_parameter_value().integer_value
        )
        self.ids = list(
            self.declare_parameter("servo_ids", [2, 3]).get_parameter_value().integer_array_value
        )
        self._pattern_executor = ThreadPoolExecutor(max_workers=1)

        # Initialize the serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate)
            self.logger.info(f"Serial port opened on {serial_port}")
        except serial.SerialException as exc:
            self.serial = None
            self.logger.warning(
                f"Could not open serial port '{serial_port}': {exc}. Assuming fake robot."
            )
        # Create a ROS2 publisher for the feedback
        self.publisher = self.create_publisher(String, FEEDBACK_TOPIC_NAME, 10)
        self.left_hand_gesture_publisher = self.create_publisher(String, "/l_hand/l_hand_topic", 10)
        self.right_hand_gesture_publisher = self.create_publisher(
            String, "/r_hand/r_hand_topic", 10
        )
        self.pos = [45, 120]
        self.zero = [30, 90]
        self.hold = [70, 70]
        # Create main program subscriber
        self.gesture_subscription = self.create_subscription(
            String, "/arms/arm_action", self.action_callback, 10
        )

        # self.publisher_ = self.create_publisher(JointTrajectory, 'shoulder_controller/joint_trajectory', 10)

        self.SHOULDER_POSITIONS = {
            "zero": [30.0, 90.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0],
            "rps_1": [55.0, 110.0, 10.0, 0.0, 79.0, 80.0, 45.0, 0.0],
            "rps_2": [35.0, 70.0, 10.0, 0.0, 79.0, 80.0, 75.0, 0.0],
        }

        self.HAND_ACTIONS = [
            "open",
            "fist",
            "scissors",
            "point",
            "thumbs_up",
            "grasp",
            "pen_grasp",
            "hard_rock",
            "rps",
            "funny",
            "three",
        ]

        self.ACTION_PATTERNS = {
            # These should be (hand/arm, sleef_after, action, left/right/both(default))
            # left or right can be left out to default to both
            "zero": [
                ("hand", 1, "fist", "left"),
                ("hand", 1, "fist", "right"),
                ("arm", 0, "zero", "both"),
            ],
            "test": [
                ("hand", 1, "fist", "left"),
                ("hand", 1, "fist", "right"),
                ("arm", 1, "zero"),
                ("hand", 1, "open", "left"),
                ("hand", 1, "open", "right"),
                ("hand", 1, "fist", "left"),
                ("hand", 0, "fist", "right"),
            ],
            "wave": [
                ("hand", 0.5, "fist", "left"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "rps_2"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "rps_2"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "zero"),
                ("hand", 0, "open", "left"),
            ],
            "rock": [
                ("hand", 1, "hard_rock", "left"),
                ("arm", 1, "rps_1"),
                ("arm", 1, "rps_2"),
                ("arm", 1, "rps_1"),
                ("arm", 1, "rps_2"),
                ("arm", 1, "rps_1"),
                ("arm", 1, "rps_2"),
                ("arm", 1, "zero"),
                ("hand", 0, "open", "left"),
            ],
        }

        self.exit_commands = ["quit", "exit"]

    def action_callback(self, msg):
        arg = msg.data
        if not arg:
            self.logger.warning("Received empty arm action")
            return
        self.logger.info(f"arg: {arg}")

        # Action messages for hands can be eg. "l_hand_fist"
        if arg[0:7] in ("r_hand_", "l_hand_"):
            # Action is for hands (fingers)
            hand = "right" if arg[0:2] == "r_" else "left"
            hand_action = arg[7:]
            if hand_action in self.HAND_ACTIONS:
                self.hand_gesture(hand_action, hand)
            return

        if arg in self.ACTION_PATTERNS:
            # Action is a pattern
            self._pattern_executor.submit(self.perform_action_from_pattern, arg)
        else:
            self.arm_gesture(arg)

    def perform_action_from_pattern(self, pattern):
        if pattern not in self.ACTION_PATTERNS:
            self.logger.info("Action pattern not implemented")
            return
        for action_data in self.ACTION_PATTERNS[pattern]:
            # action_data should be (hand/arm, sleef_after, action, left/right/both(default))
            hand_or_arm = action_data[0]
            sleep_after = action_data[1]
            action = action_data[2]
            side = action_data[3] if len(action_data) == 4 else "both"
            if hand_or_arm == "hand":
                self.hand_gesture(action, side)
            elif hand_or_arm == "arm":
                self.arm_gesture(action, side)
            time.sleep(sleep_after)

    def trial(self):
        command = []
        i = 0
        joints = [
            "R_shoulder lift",
            "R_upper arm roll",
            "R_bicep",
            "R_shoulder out",
            "L_shoulder lift",
            "L_upper arm roll",
            "L_bicep",
            "L_shoulder out",
        ]
        while len(command) < 8:
            angle = float(input(f"Angle for {joints[i]} joint: "))
            if isinstance(angle, float) and angle >= 0 and angle <= 180:
                command.append(angle)
                i += 1
            else:
                print("Angle must be int. Safe values are 0 - 180")
        self.logger.info("Sending positions")
        self.arm_gesture(command)

    def arm_gesture(self, action, hand="both"):
        self.logger.info(f"Action: {action}")

        positions = []
        if action in self.SHOULDER_POSITIONS:
            if hand == "right":
                # Replace left hand values with -1.0 which means do nothing for that servo
                positions = self.SHOULDER_POSITIONS[action][4:] + [-1.0, -1.0, -1.0, -1.0]
            elif hand == "left":
                # Replace right hand values with -1.0 which means do nothing for that servo
                positions = [-1.0, -1.0, -1.0, -1.0] + self.SHOULDER_POSITIONS[action][:4]
            elif hand == "both":
                positions = self.SHOULDER_POSITIONS[action]
            else:
                self.logger.info(
                    f"Should be 'left', 'right' or 'both'(default) instead of: '{hand}'"
                )
        elif action == "pos":
            positions = self.pos
        elif action == "zer":
            positions = self.zero
        elif action == "hold":
            positions = self.hold
        else:
            self.logger.warning(f"Unknown arm action '{action}'")
            return

        if len(positions) < len(self.ids):
            self.logger.error(
                f"Configured positions for action '{action}' are shorter than servo_ids. "
                f"positions={positions}, servo_ids={self.ids}"
            )
            return
        angles = []
        for i in range(len(self.ids)):
            angles.append(f"{self.ids[i]}:{positions[i]}")
        # Prepare the command to be sent to the Arduino
        command = ",".join(str(angle) for angle in angles)
        self.logger.info(command)
        # Note: Could return at the start of function
        # but this leaves room for implementation for fake robot
        if self.serial:
            # Send the command to the Arduino
            self.serial.write(command.encode())

            # Read the feedback from the Arduino
            feedback = self.serial.readline().decode().strip()
            self.logger.info("Received feedback: %s" % feedback)

            # Publish the feedback to the ROS2 topic
            feedback_msg = String()
            feedback_msg.data = feedback
            self.publisher.publish(feedback_msg)

    def hand_gesture(self, gesture, hand="both"):
        msg = String()
        msg.data = gesture
        if hand not in ("right", "left", "both"):
            self.logger.info(f"Should be 'left' or 'right' instead of: '{hand}'")
            return
        if hand in ("right", "both"):
            self.right_hand_gesture_publisher.publish(msg)
        if hand in ("left", "both"):
            self.left_hand_gesture_publisher.publish(msg)

    def list_available_commands(self):
        print("Available commands:", end=" ")
        for command in self.HAND_ACTIONS[:-1]:
            print(command, end=", ")
        print(self.HAND_ACTIONS[-1])
        print("You can also input 'quit' or 'exit' to quit.")

    def rps(self):
        self.logger.info("Starting rps")
        self.hand_gesture("three")
        time.sleep(2)
        self.hand_gesture("scissors")
        time.sleep(2)
        self.hand_gesture("point")
        time.sleep(2)
        self.hand_gesture(random.choice(["open", "fist", "scissors"]))

    def destroy_node(self):
        self._pattern_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    armController = UnifiedArms()
    armController.get_logger().info("Arm controller ready for messages.")
    try:
        rclpy.spin(armController)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        armController.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
