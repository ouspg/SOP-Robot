import dlib
import cv2
import os
import numpy as np
import time
import sys
import traceback
from typing import List
from pathlib import Path

from .lip_movement_net import LipMovementDetector
from .face_recognition import FaceRecognizer
from .face import Face
from .links_cluster import LinksCluster, Subcluster

DEFAULT_FACE_DB_PATH = os.path.expanduser('~')+"/database"

class FaceAnalyzer:

    def __init__(self, logger, lip_movement_detector: LipMovementDetector=None, face_recognizer=True, correlation_tracker=True):
        self.logger = logger
        self.correlation_tracker_enabled = correlation_tracker
        self.lip_movement_detector: LipMovementDetector = lip_movement_detector

        # Face recognition
        if face_recognizer:
            self.face_recognizer = FaceRecognizer(db_path=DEFAULT_FACE_DB_PATH,
                                                  logger=self.logger,
                                                  model_name="SFace",
                                                  detector_backend="yunet",
                                                  distance_metric="cosine") # uses our own implemenation for distance
        else:
            self.face_recognizer = None

        # TODO: add faces to sql database and read faces from it
        # self.face_ids, self.face_representations = self.face_recognizer.get_database_representations()
        self.face_ids = []
        self.face_representations = []

        self.cluster_similarity_threshold = 0.3
        self.subcluster_similarity_threshold = 0.2
        self.pair_similarity_maximum = 1.0
        self.cluster = LinksCluster(self.cluster_similarity_threshold,
                                    self.subcluster_similarity_threshold,
                                    self.pair_similarity_maximum,
                                    store_vectors=True,
                                    logger=self.logger)

        self.frame = 0
        self.faces: List[Face] = []

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        # self.timer = self.create_timer(2, self.profile_cycle)
        # pr.enable()
            
    def on_frame_received(self, frame: cv2.typing.MatLike):
        cv2_gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Get the face locations
        if self.frame == 0:
            faces_len_old = len(self.faces)
            # Use face detection to get face locations
            self.faces = self.analyze_frame(frame)

            # Initialize new input sequences for lip movement detector if the number of detected faces change
            if self.lip_movement_detector is not None:
                if faces_len_old != len(self.faces):
                    #TODO: original implementation had speaking state clearing here
                    self.lip_movement_detector.initialize_input_sequence(len(self.faces))

            # self.logger.info(f"Face detection: faces={len(self.faces)}")
            
        else:
            # Use dlib correlation tracker to update face locations
            for face in self.faces:
                face.update_location(frame)

            # self.logger.info(f"correlation tracking: faces={len(self.faces)}")
        
        # loop through all faces
        for i, face in enumerate(self.faces):
            if self.lip_movement_detector is not None:
                # Determine if the face is speaking or silent
                face.speaking = self.lip_movement_detector.test_video_frame(cv2_gray_img, face.rect, i)

            # Draw information to frame
            self.draw_face_info(frame, face)

        cv2.putText(frame,
                    f"Faces in current frame{len(self.faces)}",
                    (100,10),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        cv2.putText(frame,
                    f"Clusters in database {len(self.cluster.clusters)}",
                    (100,30),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        if self.correlation_tracker_enabled:
            self.frame += 1
            # Set frame to zero for new detection every nth frame.
            # Large values lead to drifting of the detected faces
            n = 5
            self.frame = self.frame % n
        
        return [face.as_dict() for face in self.faces]
    
    def analyze_frame(self, frame):
        """
        Get face objects from frame. Do face detection and recognition. Intialize dlib correlation trackers.
        """
        faces: List[Face] = []

        # Uses deepface to extract face locations from frame
        face_objs = self.face_recognizer.extract_faces(frame)
        
        for face_obj in face_objs:
            
            face_img = face_obj["face"]
            face_region = face_obj["facial_area"]
            x = face_region["x"]
            y = face_region["y"]
            w = face_region["w"]
            h = face_region["h"]

            face: Face = None
            representation: List[float] = self.face_recognizer.represent(face_img)

            # Compare face to the database

            cluster_predictation = self.cluster.predict(np.array(representation))

            if face is None:
                # Matching face not found, create new one
                face = Face(x, x + w, y, y + h, face_img, representation, cluster_predictation)

                # self.logger.info("new face found")

            if self.correlation_tracker_enabled:
                face.start_track(frame)
            faces.append(face)
        return faces

    def draw_face_info(self, frame, face:Face):
        """
        Draws rectangle around face and other information to display 
        """
        green = (0, 255, 0)

        # Draw rectangle around the face
        cv2.rectangle(frame, (face.left, face.top), (face.right, face.bottom), green, 1)

        if face.speaking is not None:
            cv2.putText(frame,
                        face.speaking,
                        (face.left + 2, face.bottom + 10 - 3),
                        self.font,
                        0.3,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)

        if face.cluster_dict:
            cv2.putText(frame,
                        f"Matching cluster: {face.cluster_dict['id']}",
                        (face.left + 2, face.top + 10),
                        self.font,
                        0.3,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)
