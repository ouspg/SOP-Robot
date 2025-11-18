from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CvBridgePublisher:
    def __init__(self, node, topic_name: str, queue_size: int = 10):
        """
        node: an existing rclpy.node.Node instance.
        """
        self.node = node
        self.publisher = node.create_publisher(
            Image,
            topic_name,
            queue_size
        )
        self.bridge = CvBridge()

    def to_cv2(self, img_msg: Image):
        """
        Converts a ROS Image message to an OpenCV BGR image.
        """
        return self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

    def publish(self, frame):
        """
        Publishes an OpenCV BGR image using rclpy.
        """
        ros_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_msg)
