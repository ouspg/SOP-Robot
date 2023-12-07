import rclpy
import time
import random
import serial
from std_msgs.msg import String
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

# Define the Arduino serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Replace with the appropriate port
BAUD_RATE = 115200

# Define the ROS2 topic names
TOPIC_NAME = 'shoulder_controller/joint_trajectory'
FEEDBACK_TOPIC_NAME = 'feedback'

class UnifiedArms(Node):
    def __init__(self):
        #ShoulderController
        super().__init__('minimal_publisher')

        # Initialize the serial connection
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE)
            print("Serial port opened")
        except serial.SerialException:
            self.serial = None
            print("Could not open serial port. Assuming fake robot.")
        # Create a ROS2 publisher for the feedback
        self.publisher                    = self.create_publisher(String, FEEDBACK_TOPIC_NAME, 10)
        self.left_hand_gesture_publisher  = self.create_publisher(String, "/l_hand/l_hand_topic", 10)
        self.right_hand_gesture_publisher = self.create_publisher(String, "/r_hand/r_hand_topic", 10)

        #Create main program subscriber
        self.gesture_subscription = self.create_subscription(String, "/arms/arm_action", self.action_callback, 10)

        #self.publisher_ = self.create_publisher(JointTrajectory, 'shoulder_controller/joint_trajectory', 10)

        self.SHOULDER_POSITIONS = {
            "zero": [30.0, 90.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0],
            "rps_1": [30.0, 90.0, 10.0, 0.0, 79.0, 80.0, 45.0, 0.0],
            "rps_2": [30.0, 90.0, 10.0, 0.0, 79.0, 80.0, 75.0, 0.0]
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
            "three"
        ]

        self.ACTION_PATTERNS = {
            # These should be (hand/arm, sleef_after, action, left/right/both(default))
            # left or right can be left out to default to both
            "zero": [
                ("hand", 1, "fist", "left"),
                ("hand", 1, "fist", "right"),
                ("arm", 0, "zero", "both")
            ],
            "test": [
                ("hand", 1, "fist", "left"),
                ("hand", 1, "fist", "right"),
                ("arm", 1, "zero"),
                ("hand", 1, "open", "left"),
                ("hand", 1, "open", "right"),
                ("hand", 1, "fist", "left"),
                ("hand", 0, "fist", "right")
            ],
            "wave": [
                ("hand", 0.5, "fist", "left"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "rps_2"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "rps_2"),
                ("arm", 0.5, "rps_1"),
                ("arm", 0.5, "zero"),
                ("hand", 0, "open", "left")
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
                ("hand", 0, "open", "left")
            ]
        }

        self.exit_commands = ["quit", "exit"]
        self.logger = self.get_logger()


    def action_callback(self, msg):
        arg = msg.data
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
            self.perform_action_from_pattern(arg)
        else:
            self.logger.info("Action not implemented")


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
                self.arm_gesture
            time.sleep(sleep_after)


    def trial(self):
        command = []
        i = 0
        joints = ["R_shoulder lift", "R_upper arm roll", "R_bicep", "R_shoulder out","L_shoulder lift", "L_upper arm roll", "L_bicep", "L_shoulder out"]
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

        goal_msg = JointTrajectory()
        goal_msg.joint_names = ["r_shoulder_lift_joint", "r_shoulder_out_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint",
        "l_shoulder_lift_joint", "l_shoulder_out_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint"]
        point = JointTrajectoryPoint()

        if hand == "right":
            # Replace left hand values with -1.0 which means do nothing for that servo
            point.positions = self.SHOULDER_POSITIONS[action][4:] + [-1.0, -1.0, -1.0, -1.0]
        elif hand == "left":
            # Replace right hand values with -1.0 which means do nothing for that servo
            point.positions = [-1.0, -1.0, -1.0, -1.0] + self.SHOULDER_POSITIONS[action][:4]
        elif hand == "both":
            point.positions = self.SHOULDER_POSITIONS[action]
        else:
            self.logger.info(f"Should be 'left', 'right' or 'both'(default) instead of: '{hand}'")

        dur = Duration()
        dur.sec = 1
        point.time_from_start = dur
        goal_msg.points = [point]

        self.logger.info(f"Sending request")
        print(type(goal_msg))
        #self.publisher_.publish(goal_msg)

        # Extract the desired angles from the received message
        angles = []
        for point in goal_msg.points:
            angles.extend(point.positions)

        # Prepare the command to be sent to the Arduino
        command = ','.join(str(angle) for angle in angles)

        # Note: Could return at the start of function
        # but this leaves room for implementation for fake robot
        if self.serial:
            # Send the command to the Arduino
            self.serial.write(command.encode())

            # Read the feedback from the Arduino
            feedback = self.serial.readline().decode().strip()
            self.logger.info('Received feedback: %s' % feedback)

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
        self.logger.info(f"Starting rps")
        self.send_gesture("three")
        time.sleep(2)
        self.send_gesture("scissors")
        time.sleep(2)
        self.send_gesture("point")
        time.sleep(2)
        self.send_gesture(random.choice(["open", "fist", "scissors"]))



def main(args=None):
    rclpy.init(args=args)
    armController = UnifiedArms()
    print("Arm controller ready for messages.")
    rclpy.spin(armController)


if __name__ == "__main__":
    main()
