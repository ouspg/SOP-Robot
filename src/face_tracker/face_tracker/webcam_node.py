import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class WebcamError(Exception):
    """signal that webcam has stopped working"""
    pass

class WebCamNode(Node):
    def __init__(self):
        super().__init__("webcam")

        self.logger = self.get_logger()

        # Node parameters
        raw_image_topic = (
            self.declare_parameter(
                "raw_image", "/raw_image"
            )
            .get_parameter_value()
            .string_value
        )
        
        self.index = (
            self.declare_parameter("index", 0)
            .get_parameter_value().integer_value
        )
        self.width = (
            self.declare_parameter("width", None)
            .get_parameter_value().integer_value
        )
        self.height = (
            self.declare_parameter("height", None)
            .get_parameter_value().integer_value
        )
        self.fps = (
            self.declare_parameter("fps", None)
            .get_parameter_value().integer_value
        )
        self.mjpg = (
            self.declare_parameter("mjpg", False)
            .get_parameter_value()._bool_value
        )

        self.logger.info(f"Webcam node parameters:\n" +
                         f"index={self.index}\n" +
                         f"width={self.width}\n" +
                         f"height={self.height}\n" +
                         f"fps={self.fps}\n" +
                         f"mjpg={self.mjpg}")

        self.face_img_publisher = self.create_publisher(Image, raw_image_topic, 5)
        timer_period = 1.0 / self.fps if self.fps and self.fps > 0 else 1.0 / 30.0
        self.cap = None
        self.open_webcam()
        self.capture_timer = self.create_timer(timer_period, self.capture_frame)

    def capture_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self.logger.error("Webcam is not available, attempting reopen")
            self.open_webcam()
            return

        try:
            ret, frame = self.cap.read()
            if not ret:
                raise WebcamError
        except WebcamError:
            self.logger.error("Webcam frame capture failed, restarting device")
            self.close_webcam()
            self.open_webcam()
            return

        try:
            self.face_img_publisher.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as exc:
            self.logger.warning(f"Could not convert OpenCV image to ROS image: {exc}")

    def open_webcam(self):
        '''
        Open webcam handle
        '''
        self.cap = cv2.VideoCapture(self.index)

        if not self.cap.isOpened():
            self.logger.error("Cannot open webcam")
            self.cap.release()
            self.cap = None
            return

        # Set video capture parameters
        if self.width:
            W = self.width
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        if self.height:
            H = self.height
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        if self.mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if self.fps:
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.logger.info(f"Webcam fps={fps}, shape={w},{h}")


    def close_webcam(self):
        '''
        Destroy webcam handle and close all windows
        '''
        if self.cap is None:
            return
        self.logger.info("closing webcam handle...")
        self.cap.release()
        self.cap = None
        cv2.destroyAllWindows()
        self.logger.info("Webcam closed!")

    def destroy_node(self):
        self.close_webcam()
        return super().destroy_node()

def main(args=None):
    # Initialize
    rclpy.init(args=args)
    webcam = WebCamNode()

    rclpy.spin(webcam)
    webcam.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
