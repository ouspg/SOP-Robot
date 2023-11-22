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
        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE)
        # Create a ROS2 publisher for the feedback
        self.publisher                    = self.create_publisher(String, FEEDBACK_TOPIC_NAME, 10)
        self.left_hand_gesture_publisher  = self.create_publisher(String, "/l_hand/l_hand_topic", 10)
        self.right_hand_gesture_publisher = self.create_publisher(String, "/r_hand/r_hand_topic", 10)

        #Create main program subscriber
        self.gesture_subscription = self.create_subscription(String, "/arms/arm_action", self.action_callback, 10)

        #self.publisher_ = self.create_publisher(JointTrajectory, 'shoulder_controller/joint_trajectory', 10)
        self.args = ['wave', 'rock', 'test', 'zero',
            'l_hand_open', 'l_hand_fist', 'l_hand_scissors', 'l_hand_point', 'l_hand_thumbs_up', 'l_hand_grasp', 'l_hand_pen_grasp', 'l_hand_hard_rock', 'l_hand_rps', 'l_hand_funny',
            'r_hand_open', 'r_hand_fist', 'r_hand_scissors', 'r_hand_point', 'r_hand_thumbs_up', 'r_hand_grasp', 'r_hand_pen_grasp', 'r_hand_hard_rock', 'r_hand_rps', 'r_hand_funny',
        ]
        self.positions_dict = {
            "zero": [30.0, 90.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0],
            "rps_1": [30.0, 90.0, 10.0, 0.0, 79.0, 80.0, 45.0, 0.0],
            "rps_2": [30.0, 90.0, 10.0, 0.0, 79.0, 80.0, 75.0, 0.0]
        }

        self.available_commands = [
            "open",
            "fist",
            "scissors",
            "point",
            "thumbs_up",
            "grasp",
            "pen_grasp",
            "hard_rock",
            "rps",
            "funny"
        ]

        self.exit_commands = ["quit", "exit"]
        self.logger = self.get_logger()


    def action_callback(self, msg):
        arg = msg.data
        hand = "r" #TODO tätä pitäis muuttaa
        self.logger.info(f"arg: {arg}")
        match arg:
            case 'wave':
                self.action_wave()
            case 'rock':
                self.action_rock()
            case 'test':
                self.action_test()
            case 'zero':
                self.action_zero()
            case 'l_hand_open':
                self.hand_gesture("left", "open")
            case 'l_hand_fist':
                self.hand_gesture("left", "fist")
            case 'l_hand_scissors':
                self.hand_gesture("left", "scissors")
            case 'l_hand_point':
                self.hand_gesture("left", "point")
            case 'l_hand_thumbs_up':
                self.hand_gesture("left", "thumbs_up")
            case 'l_hand_grasp':
                self.hand_gesture("left", "grasp")
            case 'l_hand_pen_grasp':
                self.hand_gesture("left", "pen_grasp")
            case 'l_hand_hard_rock':
                self.hand_gesture("left", "hard_rock")
            case 'l_hand_rps':
                self.hand_gesture("left", "rps")
            case 'l_hand_funny':
                self.hand_gesture("left", "funny")
            case 'r_hand_open':
                self.hand_gesture("right", "open")
            case 'r_hand_fist':
                self.hand_gesture("right", "fist")
            case 'r_hand_scissors':
                self.hand_gesture("right", "scissors")
            case 'r_hand_point':
                self.hand_gesture("right", "point")
            case 'r_hand_thumbs_up':
                self.hand_gesture("right", "thumbs_up")
            case 'r_hand_grasp':
                self.hand_gesture("right", "grasp")
            case 'r_hand_pen_grasp':
                self.hand_gesture("right", "pen_grasp")
            case 'r_hand_hard_rock':
                self.hand_gesture("right", "hard_rock")
            case 'r_hand_rps':
                self.hand_gesture("right", "rps")
            case 'r_hand_funny':
                self.hand_gesture("right", "funny")
            case _:
                self.logger.info("Action not implemented")
#Action functions

    def action_zero(self):
        self.hand_gesture("left", "fist")
        time.sleep(1)
        self.hand_gesture("right", "fist")
        time.sleep(1)
        self.arm_gesture("zero")

    def action_test(self):
        self.hand_gesture("left", "fist")
        time.sleep(1)
        self.hand_gesture("right", "fist")
        time.sleep(1)
        self.arm_gesture("zero")
        time.sleep(1)
        self.hand_gesture("left", "open")
        time.sleep(1)
        self.hand_gesture("right", "open")
        time.sleep(1)
        self.hand_gesture("left", "fist")
        time.sleep(1)
        self.hand_gesture("right", "fist")
        time.sleep(1)

    def action_wave(self):
        self.hand_gesture("left", "fist")
        time.sleep(0.5)
        self.arm_gesture("rps_1")
        time.sleep(0.5)
        self.arm_gesture("rps_2")
        time.sleep(0.5)
        self.arm_gesture("rps_1")
        time.sleep(0.5)
        self.arm_gesture("rps_2")
        time.sleep(0.5)
        self.arm_gesture("rps_1")
        time.sleep(0.5)
        self.arm_gesture("zero")
        time.sleep(0.5)
        self.hand_gesture("left", "open")

    def action_rock(self):
        self.hand_gesture("left", "hard_rock")
        time.sleep(1)
        self.arm_gesture("rps_1")
        time.sleep(1)
        self.arm_gesture("rps_2")
        time.sleep(1)
        self.arm_gesture("rps_1")
        time.sleep(1)
        self.arm_gesture("rps_2")
        time.sleep(1)
        self.arm_gesture("rps_1")
        time.sleep(1)
        self.arm_gesture("rps_2")
        time.sleep(1)
        self.arm_gesture("zero")
        time.sleep(1)
        self.hand_gesture("left", "open")




#ShoulderController
    def available_commands(self):
        print("Command not recognized, available commands are")
        print(f"{self.args[:-1]}")
        print(f"and quit.")

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

        match hand:
            case "right":
                # Replace left hand values with -1.0 which means do nothing for that servo
                point.positions = self.positions_dict[action][4:] + [-1.0, -1.0, -1.0, -1.0]
            case "left":
                # Replace right hand values with -1.0 which means do nothing for that servo
                point.positions = [-1.0, -1.0, -1.0, -1.0] + self.positions_dict[action][:4]
            case "both":
                point.positions = self.positions_dict[action]
            case _:
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

        # Send the command to the Arduino
        self.serial.write(command.encode())

        # Read the feedback from the Arduino
        feedback = self.serial.readline().decode().strip()
        self.get_logger().info('Received feedback: %s' % feedback)

        # Publish the feedback to the ROS2 topic
        feedback_msg = String()
        feedback_msg.data = feedback
        self.publisher.publish(feedback_msg)


#Hands
    def hand_gesture(self, hand, gesture):
        msg = String()
        msg.data = gesture
        if hand == "right":
            self.right_hand_gesture_publisher.publish(msg)
        elif hand == "left":
            self.left_hand_gesture_publisher.publish(msg)
        else:
            self.logger.info(f"Should be 'left' or 'right' instead of: '{hand}'")
        self.left_hand_gesture_publisher.publish(msg)


    def list_available_commands(self):
        print("Available commands:", end=" ")
        for command in self.available_commands[:-1]:
            print(command, end=", ")
        print(self.available_commands[-1])
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
    rclpy.spin(armController)



if __name__ == "__main__":
    main()
