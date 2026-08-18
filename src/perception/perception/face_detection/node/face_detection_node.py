import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

from face_tracker_msgs.msg import Faces, Face as FaceMsg, Point2, Occurance
from perception.face_detection.core.face_analyzer import FaceAnalyzer
from perception.face_detection.core.lip_movement_net import LipMovementDetector
from perception.face_detection.util.fps_tracker import FPSTracker
from perception.face_detection.util.ros_utils import CvBridgePublisher
from core.util import run_node
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__("face_detection_node")

        self.declare_parameter("image_topic", "/i2e_webcam")
        self.declare_parameter("face_image_topic", "face_detection")
        self.declare_parameter("face_topic", "faces")
        self.declare_parameter("lip_motion_model", "1_32_False_True_0.25_lip_motion_net_model.h5")
        self.declare_parameter("shape_predictor", "shape_predictor_68_face_landmarks.dat")
        self.declare_parameter("face_recognizer_enabled", True)
        self.declare_parameter("correlation_tracker", True)
        self.declare_parameter("cluster_similarity_threshold", 0.3)
        self.declare_parameter("subcluster_similarity_threshold", 0.2)
        self.declare_parameter("pair_similarity_maximum", 1.0)
        self.declare_parameter("face_recognition_model", "SFace")
        self.declare_parameter("face_detection_model", "yunet")

        image_topic = self.get_parameter("image_topic").value
        face_image_topic = self.get_parameter("face_image_topic").value
        face_topic = self.get_parameter("face_topic").value
        model_name = self.get_parameter("lip_motion_model").value
        shape_predictor_name = self.get_parameter("shape_predictor").value
        face_recognizer_enabled = self.get_parameter("face_recognizer_enabled").value
        correlation_tracker = self.get_parameter("correlation_tracker").value
        cluster_similarity_threshold = self.get_parameter("cluster_similarity_threshold").value
        subcluster_similarity_threshold = self.get_parameter("subcluster_similarity_threshold").value
        pair_similarity_maximum = self.get_parameter("pair_similarity_maximum").value
        face_recognition_model = self.get_parameter("face_recognition_model").value
        face_detection_model = self.get_parameter("face_detection_model").value

        pkg_share = get_package_share_directory("perception")

        model_path = os.path.join(
            pkg_share,
            "face_detection",
            "models",
            model_name
        )

        shape_predictor_path = os.path.join(
            pkg_share,
            "face_detection",
            "predictors",
            shape_predictor_name
        )

        # Initialize components
        self.lip_detector = LipMovementDetector(
            model_path=model_path,
            shape_predictor_path=shape_predictor_path
        )
        self.face_analyzer = FaceAnalyzer(
            logger=self.get_logger(),
            lip_movement_detector=self.lip_detector,
            face_recognizer_enabled=face_recognizer_enabled,
            correlation_tracker=correlation_tracker,
            cluster_similarity_threshold=cluster_similarity_threshold,
            subcluster_similarity_threshold=subcluster_similarity_threshold,
            pair_similarity_maximum=pair_similarity_maximum,
            face_recognition_model=face_recognition_model,
            face_detection_model=face_detection_model,
        )

        #FPS is drawn onto the
        self.fps_tracker = FPSTracker()

        self.face_img_pub = CvBridgePublisher(self, face_image_topic)
        self.get_logger().info(f"publisher ['{self.face_img_pub.publisher.topic_name}'] added.")

        self.face_pub = self.create_publisher(Faces, face_topic, 1)
        self.get_logger().info(f"publisher ['{self.face_pub.topic_name}'] added.")

        self.subscriber = self.create_subscription(Image, image_topic, self.on_frame_received, 1)

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.get_logger().info("FaceTrackerNode initialized.")

    def on_frame_received(self, img_msg: Image):
        """Handle incoming camera frames and process faces."""

        frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # Run analysis
        faces = self.face_analyzer.on_frame_received(frame)

        # Convert to ROS messages
        msg_faces = []
        for face in faces:
            occurances = [
                Occurance(
                    start_time=float(o["start_time"]),
                    end_time=float(o["end_time"]),
                    duration=float(o["duration"])
                )
                for o in face["previous_occurances"]
            ]
            msg_faces.append(
                FaceMsg(
                    top_left=Point2(x=face["left"], y=face["top"]),
                    bottom_right=Point2(x=face["right"], y=face["bottom"]),
                    diagonal=face["diagonal"],
                    face_id=face["face_id"],
                    speaking=face["speaking"],
                    occurances=occurances
                )
            )

        # Draw FPS on frame
        cv2.putText(frame, f"{self.fps_tracker.fps:.2f}", (10, 20), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Publish results
        try:
            self.face_img_pub.publish(frame)
        except CvBridgeError as e:
            self.logger.warn("Could not convertros img to opencv image: :", e)
    
        if len(msg_faces) > 0:
            self.face_pub.publish(Faces(faces=msg_faces))

        self.fps_tracker.update()


def main(args=None):
    run_node(FaceDetectionNode)


if __name__ == "__main__":
    main()
