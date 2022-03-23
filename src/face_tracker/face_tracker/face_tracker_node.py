import rclpy
import cv2
import dlib
import os
import numpy as np
import dlib

from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from face_tracker_msgs.msg import Faces, Face, Point2

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class FaceTracker(Node):
    def __init__(self):
        super().__init__("face_tracker")

        image_topic = (
            self.declare_parameter("image_topic", "/image_raw")
            .get_parameter_value()
            .string_value
        )
        face_image_topic = (
            self.declare_parameter(
                "face_image_topic", "image_face"
            )  # non-absolute paths are inside the current node namespace
            .get_parameter_value()
            .string_value
        )
        face_topic = (
            self.declare_parameter(
                "face_topic", "faces"
            )  # non-absolute paths are inside the current node namespace
            .get_parameter_value()
            .string_value
        )
        predictor = (
            self.declare_parameter("predictor", "shape_predictor_68_face_landmarks.dat")
            .get_parameter_value()
            .string_value
        )

        self.face_detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(
            os.path.join(
                get_package_share_directory("face_tracker"),
                "predictors",
                predictor,
            )
        )

        # Create subscription, that receives camera frames
        self.subscriber = self.create_subscription(
            Image,
            image_topic,
            self.on_frame_received,
            10,
        )
        self.face_img_publisher = self.create_publisher(Image, face_image_topic, 10)
        self.face_publisher = self.create_publisher(Faces, face_topic, 10)
        self.face_location_publisher = self.create_publisher(Point2, 'face_location_topic', 10)

        timer_period = 0.5    # Face publish interval in seconds
        self.face_location = None  # Variable for face location
        # Call publish_face_location every timer_period seconds
        self.timer = self.create_timer(timer_period, self.publish_face_location)

        self.first_face_detected = False

        self.trackers = []

    def on_frame_received(self, img: Image):
        try:
            # Convert ros img to opencv compatible format
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")


            if not self.first_face_detected:
                # Uses HOG + SVM, CNN would probably have better detection at higher computing cost
                # Todo: use dlib correlation_tracker to track the same face across frames?
                faces = self.face_detector(cv2_gray_img)

                msg_faces = []

                for face in faces:
                    (x1, y1, x2, y2) = (
                        face.left(),
                        face.top(),
                        face.right(),
                        face.bottom(),
                        )

                    green = (0, 255, 0)


                    # Draw rectangle around the face
                    cv2.rectangle(cv2_bgr_img, (x1, y1), (x2, y2), green, 1)

                    #initialize tracker
                    tracker = dlib.correlation_tracker()
                    #start track on face detected on first frame
                    rect = dlib.rectangle(x1, y1, x2, y2)
                    tracker.start_track(cv2_gray_img, rect)
                    self.trackers.append(tracker)
                    msg_face = Face(top_left=Point2(x=x1, y=y1), bottom_right=Point2(x=x2, y=y2))



                    msg_faces.append(msg_face)

                if len(msg_faces) > 0:
                        # Calculate midpoint of one face
                        self.face_location = Point2(x=round((msg_faces[0].top_left.x + msg_faces[0].bottom_right.x) / 2),
                        y=round((msg_faces[0].top_left.y + msg_faces[0].bottom_right.y) / 2))
                        self.first_face_detected = True

                # Publish image that has rectangles around the detected faces
                self.face_img_publisher.publish(bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
                self.face_publisher.publish(Faces(faces=msg_faces))

            #if first face is detected
            else:
                green = (0, 255, 0)
                for tracker in self.trackers:
                    tracker.update(cv2_bgr_img)
                    pos = tracker.get_position()

                    #unpack the positions

                    x1 = int(pos.left())
                    y1 = int(pos.top())
                    x2 = int(pos.right())
                    y2 = int(pos.bottom())

                    msg_faces = []

                    

                    msg_face = Face(top_left=Point2(x=x1, y=y1), bottom_right=Point2(x=x2, y=y2))
                    msg_faces.append(msg_face)
                    #draw bounding box
                    cv2.rectangle(cv2_bgr_img, (x1, y1), (x2, y2), green, 1)

                if len(msg_faces) > 0:
                        # Calculate midpoint of one face
                        self.face_location = Point2(x=round((msg_faces[0].top_left.x + msg_faces[0].bottom_right.x) / 2),
                        y=round((msg_faces[0].top_left.y + msg_faces[0].bottom_right.y) / 2))
                
                # Publish image that has rectangles around the detected faces
                self.face_img_publisher.publish(bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
                self.face_publisher.publish(Faces(faces=msg_faces))
        except CvBridgeError as e:
            self.get_logger().warn("Could not convert ros img to opencv image: ", e)
        except Exception as e:
            self.get_logger().error(e)


    def publish_face_location(self):
        # Check that there is a location to publish
        if self.face_location:
            # Publish face location
            self.face_location_publisher.publish(self.face_location)
            # Set location back to None to prevent publishing same location multiple times
            self.face_location = None

def main(args=None):
    # Initialize
    rclpy.init(args=args)
    tracker = FaceTracker()

    # Do work
    rclpy.spin(tracker)

    # Shutdown
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
