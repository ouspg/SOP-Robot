import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
import serial

# Define the Arduino serial port and baud rate
SERIAL_PORT = '/dev/ttyACM0'  # Replace with the appropriate port
BAUD_RATE = 115200

# Define the ROS2 topic names
TOPIC_NAME = 'shoulder_controller/joint_trajectory'
FEEDBACK_TOPIC_NAME = 'feedback'

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # Initialize the serial connection
        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE)

        # Subscribe to the ROS2 topic
        self.subscription = self.create_subscription(
            JointTrajectory,
            TOPIC_NAME,
            self.topic_callback,
            10
        )

        # Create a ROS2 publisher for the feedback
        self.publisher = self.create_publisher(String, FEEDBACK_TOPIC_NAME, 10)

    def topic_callback(self, msg):
        # Extract the desired angles from the received message
        angles = []
        for point in msg.points:
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

def main(args=None):
    rclpy.init(args=args)

    node = ArduinoSerialNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()