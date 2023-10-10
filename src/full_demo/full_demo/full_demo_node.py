import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from face_tracker_msgs.msg import Faces
from face_tracker_msgs.msg import Point2

class FullDemoNode(Node):
    def __init__(self):
        super().__init__("full_demo")
        # ROS topics
        self.tts_message_publisher = self.create_publisher(String, "response", 10)
        self.tts_can_listen_subscription = self.create_subscription(Bool, "can_listen", self.update_tts_ready,10)

        self.face_list_subscription = self.create_subscription(Faces, "/face_tracker/face_topic", self.update_face_count, 2)
        self.new_face_subscription = self.create_subscription(Point2, "/face_tracker/face_location_topic", self.new_face_detected, 2)
        self.tts_ready = True
        self.can_say_hello = True

    def say_hello(self):
        if self.tts_ready and self.can_say_hello:
            req = String()
            req.data = "Hei vaan."
            self.can_say_hello = False
            self.tts_message_publisher.publish(req)
            self.t = self.create_timer(5, self.close_timer)
    
    def close_timer(self):
        self.t.cancel()
        self.can_say_hello = True

    def update_face_count(self, message):
        self.face_count = len(message.faces)
    
    def update_tts_ready(self, message):
        self.tts_ready = message.data

    def new_face_detected(self, message):
        self.say_hello()

def main():
    rclpy.init()
    demo_node = FullDemoNode()
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
