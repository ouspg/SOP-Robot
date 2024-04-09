import cProfile
import io
import pstats
import rclpy
import cv2
import dlib
import os
import numpy as np
import time
import sys
import traceback
from typing import List

from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from face_tracker_msgs.msg import Faces, Face as FaceMsg, Point2

from cv_bridge import CvBridge, CvBridgeError

from .lip_movement_net import LipMovementDetector
from .face_recognition import FaceRecognizer
from .face import Face
from .face_analyzer import FaceAnalyzer

bridge = CvBridge()

class WebcamError(Exception):
    """signal that webcam has stopped working"""
    pass

# pr = cProfile.Profile()

class FaceTrackerNode(Node):
    def __init__(self, lip_movement_detection=True, face_recognition=True, correlation_tracking=True):
        super().__init__("face_tracker_node")
        self.logger = self.get_logger()

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

        lip_movement_detector = None
        if lip_movement_detection:
            self.predictor = dlib.shape_predictor(
                os.path.join(
                    get_package_share_directory("face_tracker"),
                    "predictors",
                    predictor,
                )
            )
            lip_movement_detector_model = (
                self.declare_parameter("lip_movement_detector", "1_32_False_True_0.25_lip_motion_net_model.h5")
                .get_parameter_value()
                .string_value
            )
           # Initialize lip movement detector
            self.logger.info('Initializing lip movement detector...')
            lip_movement_detector = LipMovementDetector(
                os.path.join(
                    get_package_share_directory("face_tracker"),
                    "models",
                    lip_movement_detector_model,
                ),
                self.predictor
            )
            self.logger.info('Lip movement detector initialized.')
        else:
            self.logger.info('Lip movement detection disabled.')

        self.face_tracker = FaceAnalyzer(self.logger.get_child("Face_Analyzer"),
                                        lip_movement_detector,
                                        face_recognition,
                                        correlation_tracking)
        # Create subscription, that receives camera frames
        self.subscriber = self.create_subscription(
            Image,
            image_topic,
            self.on_frame_received,
            1,
        )
        self.face_img_publisher = self.create_publisher(Image, face_image_topic, 5)
        self.face_publisher = self.create_publisher(Faces, "face_topic", 1)
        self.face_location_publisher = self.create_publisher(Point2, 'face_location_topic', 1)

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.fps = FramesPerSecond()
        self.fps.start()

    # def profile_cycle(self):
    #     global pr
    #     pr.disable()
    #     s = io.StringIO()
    #     ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
    #     ps.print_stats()
    #     self.logger.info("Profiler: -----------------------------")
    #     self.logger.info(s.getvalue())
    #     self.logger.info("Profiler: -----------------------------")

    #     pr = cProfile.Profile()
    #     pr.enable()

    def on_frame_received(self, img: Image):
        # convert ros img to opencv image
        cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")

        msg_faces = []
        faces = self.face_tracker.on_frame_received(cv2_bgr_img)
        # loop through all faces
        for i, face in enumerate(faces):
            msg_face = FaceMsg(top_left=Point2(x=face.left, y=face.top), bottom_right=Point2(x=face.right, y=face.bottom))
            msg_faces.append(msg_face)

        # Draw fps to the frame
        cv2.putText(cv2_bgr_img,
                    '%.2f' % self.fps.fps,
                    (10, 20),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        # publish faces
        try:
            # Publish modified frame image
            self.face_img_publisher.publish(bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
        except CvBridgeError as e:
            self.logger.warn("Could not convert ros img to opencv image: ", e)
        # self.publish_face_location() largest face calculating not implemented yet
        self.face_publisher.publish(Faces(faces=msg_faces))

        self.fps.update_fps()

class FramesPerSecond:
    """
    Class for calculating real time fps of video stream. Code is based from stack owerflow thread:
    https://stackoverflow.com/questions/55154753/trouble-calculating-fps-on-output-video-stream
    """
    def __init__(self):
        self.startTime = None
        self.total_number_of_frames = 0
        self.counter = 0
        self.frameRate = 1  # The number of seconds to wait for each measurement.
        self.fps = 0

    def start(self):
        self.startTime = time.time()  # Returns a UNIX timestamp.

    def update_fps(self):
        self.total_number_of_frames += 1
        self.counter += 1  # Count will increase until the if condition executes.
        if self._elapsed_time() > self.frameRate:  # We measure the self only after 1 second has passed.
            self.fps = self.counter / self._elapsed_time()
            self.counter = 0  # reset the counter for next iteration.
            self.start()  # reset the start time.

    def _elapsed_time(self):
        return time.time() - self.startTime

def main(args=None):
    # Initialize
    rclpy.init(args=args)
    tracker = FaceTrackerNode(lip_movement_detection=True, face_recognition=True, correlation_tracking=False)

    # Do work
    rclpy.spin(tracker)

    # Shutdown
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
