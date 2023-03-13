import rclpy
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from std_msgs.msg import String
from face_tracker_msgs.msg import Point2


class FaceTracker(Node):
    def __init__(self):
        super().__init__("mock_face_tracker")

        self.face_location_publisher = self.create_publisher(Point2, 'face_tracker/face_location_topic', 10)

        self.face_location = None  # Variable for face location

        self.get_logger().info("Mock face tracker initialized")


    def publish_face_location(self):
        # Check that there is a location to publish
        if self.face_location:
            # Publish face location
            self.face_location_publisher.publish(self.face_location)
            # Set location back to None to prevent publishing same location multiple times
            self.face_location = None

    def parse_coordinates(self, command):
        if ',' in command:
            x, y = command.split(',')
        elif ' ' in command:
            x, y = command.split(' ')
        else:
            return None
        return [int(x), int(y)]      

    def send_coordinates(self, command):
        coordinates = self.parse_coordinates(command)
        if coordinates:
            self.face_location = Point2(x=coordinates[0], y=coordinates[1])
            self.publish_face_location()
            return True
        else:
            return False


def main(args=None):
    # Initialize
    rclpy.init(args=args)
    tracker = FaceTracker()
    
    command = None
    print("Input coordinates")
    while command != "exit":
        try:
            command = input("> ").lower()
            success = tracker.send_coordinates(command)
            if not success and command != "exit":
                raise ValueError
        except (ValueError, TypeError):
            print(
                'Usage: Input the x- and y-coordinates on a single line separated by a comma or a space. The coordinates must be integers. Input the command "exit" to exit the program'
                )

    # Shutdown
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
