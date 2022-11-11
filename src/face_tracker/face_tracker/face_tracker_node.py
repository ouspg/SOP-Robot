import rclpy
import cv2
import dlib
import os
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from face_tracker_msgs.msg import Faces, Face, Point2

from cv_bridge import CvBridge, CvBridgeError

from .lip_movement_net import LipMovementDetector

bridge = CvBridge()

class FaceTracker(Node):
    def __init__(self, lip_movement_detection=True):
        super().__init__("face_tracker")
        self.lip_movement_detection = lip_movement_detection

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
        if self.lip_movement_detection:
            lip_movement_detector = (
                self.declare_parameter("lip_movement_detector", "1_32_False_True_0.25_lip_motion_net_model.h5")
                .get_parameter_value()
                .string_value
            )
           # Initialize lip movement detector
            self.get_logger().info('Initializing lip movement detector...')
            self.lip_movement_detector = LipMovementDetector(
                os.path.join(
                    get_package_share_directory("face_tracker"),
                    "models",
                    lip_movement_detector,
                ),
                self.predictor
            )
            self.get_logger().info('Lip movement detector initialized.')
        else:
            self.get_logger().info('Lip movement detection disabled.')

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

        #timer_period = 0.5    # Face publish interval in seconds
        self.face_location = None  # Variable for face location
        # Call publish_face_location every timer_period seconds
        #self.timer = self.create_timer(timer_period, self.publish_face_location)

        self.frame = 0

        self.trackers = []
        self.speaking_states = []

        self.face_size_frame = 0
        self.face_distance1 = []
        self.face_distance2 = []

    def on_frame_received(self, img: Image):
        try:
            # Convert ros img to opencv compatible format
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")
            font = cv2.FONT_HERSHEY_SIMPLEX
            msg_faces = []

            # Do face detection on frame 0
            if self.frame == 0:
                self.trackers = []
                # Uses HOG + SVM, CNN would probably have better detection at higher computing cost
                # Todo: use dlib correlation_tracker to track the same face across frames?
                faces = self.face_detector(cv2_gray_img)

                # Initialize new input sequences for lip movement detector if the number of detected faces change
                if self.lip_movement_detection:
                    if len(faces) != len(self.speaking_states):
                        self.speaking_states = []
                        self.lip_movement_detector.initialize_input_sequence(len(faces))

                for i, face in enumerate(faces):
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

                    if self.lip_movement_detection:
                        # Determine if the face is speaking or silent
                        state = self.lip_movement_detector.test_video_frame(cv2_gray_img, rect, i)
                        try:
                            self.speaking_states[i] = state
                        except IndexError:
                            self.speaking_states.append(state)
                        # Write the speaking/silent state below the face's bounding box
                        cv2.putText(cv2_bgr_img, state, (x1 + 2, y2 + 10 - 3), font, 0.3, (255, 255, 255), 1, cv2.LINE_AA)

                    msg_face = Face(top_left=Point2(x=x1, y=y1), bottom_right=Point2(x=x2, y=y2))
                    msg_faces.append(msg_face)

            else:
                green = (0, 255, 0)
                for i, tracker in enumerate(self.trackers):
                    tracker.update(cv2_gray_img)
                    pos = tracker.get_position()

                    #unpack the positions
                    x1 = int(pos.left())
                    y1 = int(pos.top())
                    x2 = int(pos.right())
                    y2 = int(pos.bottom())

                    rect = dlib.rectangle(x1, y1, x2, y2)

                    if self.lip_movement_detection:
                        # Determine if the face is speaking or silent
                        state = self.lip_movement_detector.test_video_frame(cv2_gray_img, rect, i)
                        self.speaking_states[i] = state
                        # Write the speaking/silent state below the face's bounding box
                        cv2.putText(cv2_bgr_img, state, (x1 + 2, y2 + 10 - 3), font, 0.3, (255, 255, 255), 1, cv2.LINE_AA)

                    msg_face = Face(top_left=Point2(x=x1, y=y1), bottom_right=Point2(x=x2, y=y2))
                    msg_faces.append(msg_face)
                    #draw bounding box
                    cv2.rectangle(cv2_bgr_img, (x1, y1), (x2, y2), green, 1)

            if len(msg_faces) > 0:
                face_sizes = []

                if self.lip_movement_detection:
                    # Get indices of speaking faces
                    speaking_idx = [i for i, state in enumerate(self.speaking_states) if state == 'speaking']

                    # Find the largest speaking face
                    if len(speaking_idx) > 0:
                        for face in [msg_faces[i] for i in speaking_idx]:
                            # Calculate the length of the face bounding box diagonal
                            face_sizes.append(np.sqrt((face.top_left.x - face.bottom_right.x)**2 +
                                                      (face.top_left.y - face.bottom_right.y)**2))
                    # If no speaking faces, just find the largest face
                    else:
                        for face in msg_faces:
                            face_sizes.append(np.sqrt((face.top_left.x - face.bottom_right.x)**2 +
                                                      (face.top_left.y - face.bottom_right.y)**2))
                else:
                    for face in msg_faces:
                        face_sizes.append(np.sqrt((face.top_left.x - face.bottom_right.x)**2 +
                                                  (face.top_left.y - face.bottom_right.y)**2))

                #face distance function
                if self.face_size_frame == 0:
                    face_distance2_original = []
                    for face in msg_faces:
                        self.face_distance2.append(np.sqrt((face.top_left.x - face.bottom_right.x)**2 +
                                                    (face.top_left.y - face.bottom_right.y)**2))

                    face_distance2_original = self.face_distance2
                    self.face_distance2.sort(reverse=True)
                    try:
                        for i in range(len(face_distance2_original)):
                            self.get_logger().info('s: ')
                            self.get_logger().info(str(self.face_distance2[i]))
                            self.get_logger().info(', o: ')
                            self.get_logger().info(str(face_distance2_original[i]))
                    except ValueError:
                        pass
                    except TypeError:
                        pass
                    if not self.face_distance1:
                        self.face_distance1 = self.face_distance2
                    else:
                        max_difference = 0
                        difference_threshhold = 10
                        max_difference_face_location = 0
                        try:
                            for i in range(len(self.face_distance1)):
                                if(max_difference < self.face_distance2[i] - self.face_distance1[i]):
                                    max_difference = self.face_distance2[i] - self.face_distance1[i]
                                    max_difference_face_location = i
                            if max_difference > difference_threshhold:
                                idx = face_distance2_original.index(self.face_distance2[max_difference_face_location])
                                #calculating midpoint of the face
                                self.face_location = Point2(x=round((msg_faces[idx].top_left.x + msg_faces[idx].bottom_right.x) / 2),
                                            y=round((msg_faces[idx].top_left.y + msg_faces[idx].bottom_right.y) / 2))
                                self.face_distance1 = self.face_distance2
                                self.face_distance2 = []
                                self.get_logger().info(str(max_difference))
                                # Publish image that has rectangles around the detected faces
                                self.face_img_publisher.publish(bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
                                self.face_publisher.publish(Faces(faces=msg_faces))
                            else:
                                self.face_distance1 = self.face_distance2
                                self.face_distance2 = []

                        except IndexError:
                            pass
                # Get the index of the largest face
                idx = face_sizes.index(max(face_sizes))
                # Calculate midpoint of largest face
                self.face_location = Point2(x=round((msg_faces[idx].top_left.x + msg_faces[idx].bottom_right.x) / 2),
                                            y=round((msg_faces[idx].top_left.y + msg_faces[idx].bottom_right.y) / 2))
                self.frame += 1
                self.face_size_frame += 1
                # Set frame to zero for new detection every nth frame
                n = 25
                self.frame = self.frame % n

                #set frame to zero for new face size detection every n_face_size frame
                n_face_size = 5
                self.face_size_frame = self.face_size_frame % n_face_size

                self.publish_face_location()

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
    tracker = FaceTracker(True)     # True enables lip movement detection, False disables it

    # Do work
    rclpy.spin(tracker)

    # Shutdown
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
