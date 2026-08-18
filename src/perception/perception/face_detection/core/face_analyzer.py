import cv2
import numpy as np
from typing import List, Optional

from perception.face_detection.core.lip_movement_net import LipMovementDetector
from perception.face_detection.core.face_recognition import FaceRecognizer
from perception.face_detection.core.face import Face
from perception.face_detection.core.clustering import LinksCluster

class FaceAnalyzer:
    """Analyze frames to detect faces, perform recognition, track faces, and detect lip movement."""

    def __init__(
        self,
        logger,
        lip_movement_detector: Optional[LipMovementDetector] = None,
        face_recognizer_enabled: bool = True,
        correlation_tracker: bool = True,
        cluster_similarity_threshold: float = 0.3,
        subcluster_similarity_threshold: float = 0.2,
        pair_similarity_maximum: float = 1.0,
        face_recognition_model: str = "SFace",
        face_detection_model: str = "yunet",
    ):
        self.logger = logger
        self.correlation_tracker_enabled = correlation_tracker
        self.lip_movement_detector = lip_movement_detector

        # Initialize face recognition
        if face_recognizer_enabled:
            self.face_recognizer = FaceRecognizer(
                logger=self.logger,
                model_name=face_recognition_model,
                detector_backend=face_detection_model,
            )
        else:
            self.face_recognizer = None

        # Clustering
        self.cluster = LinksCluster(
            cluster_similarity_threshold,
            subcluster_similarity_threshold,
            pair_similarity_maximum,
            store_vectors=True,
            logger=self.logger,
        )

        self.faces: List[Face] = []
        self.frame_counter = 0
        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def on_frame_received(self, frame: np.ndarray) -> List[dict]:
        """Process a single video frame: detection, recognition, lip movement, and drawing."""
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Perform detection or update tracking
        if self.frame_counter == 0:
            old_faces_count = len(self.faces)
            self.faces = self.analyze_frame(frame)

            if self.lip_movement_detector and len(self.faces) != old_faces_count:
                self.lip_movement_detector.initialize_input_sequence(len(self.faces))
        else:
            for face in self.faces:
                face.update_location(frame)

        # Update faces
        for i, face in enumerate(self.faces):
            if self.lip_movement_detector:
                face.speaking = self.lip_movement_detector.test_video_frame(gray_frame, face.rect, i)
            self.draw_face_info(frame, face)

        # Display info on frame
        cv2.putText(frame, f"Faces in frame: {len(self.faces)}", (10, 20), self.font, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Clusters: {len(self.cluster.clusters)}", (10, 40), self.font, 0.5, (255, 255, 255), 1)

        # Increment frame counter for correlation tracking
        if self.correlation_tracker_enabled:
            self.frame_counter = (self.frame_counter + 1) % 5

        return [face.as_dict() for face in self.faces]

    def analyze_frame(self, frame: np.ndarray) -> List[Face]:
        """Detect faces, compute embeddings, perform recognition and clustering."""
        faces: List[Face] = []

        if not self.face_recognizer:
            return faces

        face_objs = self.face_recognizer.extract_faces(frame)

        for face_obj in face_objs:
            face_img = face_obj["face"]
            region = face_obj["facial_area"]
            x, y, w, h = region["x"], region["y"], region["w"], region["h"]

            representation = self.face_recognizer.represent(face_img)
            cluster_prediction = self.cluster.predict(np.array(representation))

            face = Face(left=x, right=x + w, top=y, bottom=y + h,
                        image=face_img,
                        representation=representation,
                        cluster_dict=cluster_prediction)

            if self.correlation_tracker_enabled:
                face.start_track(frame)

            faces.append(face)

        return faces

    def draw_face_info(self, frame: np.ndarray, face: Face):
        """Draw bounding boxes and overlay information on the frame."""
        green = (0, 255, 0)
        cv2.rectangle(frame, (face.left, face.top), (face.right, face.bottom), green, 1)

        if self.lip_movement_detector:
            cv2.putText(frame, f"Speaking: {face.speaking}", (face.left + 2, face.top + 20),
                        self.font, 0.3, (255, 255, 255), 1)

        if face.cluster_dict:
            cv2.putText(frame, f"Cluster ID: {face.cluster_dict['id']}", (face.left + 2, face.top + 10),
                        self.font, 0.3, (255, 255, 255), 1)
