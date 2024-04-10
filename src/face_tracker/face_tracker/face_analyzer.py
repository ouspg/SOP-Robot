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

        # For debugging
        # cv2.putText(frame,
        #             f"Faces in the database {len(self.face_representations)}",
        #             (100,10),
        #             self.font,
        #             0.3,
        #             (255, 255, 255),
        #             1,
        #             cv2.LINE_AA)

        cv2.putText(frame,
                    f"Faces in the faces list {len(self.faces)}",
                    (100,10),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        cv2.putText(frame,
                    f"Subclusters {len(self.cluster.clusters)}",
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
        
        return self.faces
    
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

            identity = None
            distance = None

            # # Compare face to the database

            predictation = self.cluster.predict(np.array(representation))
            identity = predictation
            self.logger.info(f"cluster identity = {identity}")

            # if self.face_recognizer:
            # if len(self.face_representations) != 0:
            #     matching_index, distance = self.face_recognizer.match_face(representation, self.face_representations)
            #     if matching_index is not None:
            #         identity = self.face_ids[matching_index]

            # # Compare face to previously found faces using distance between them
            # if len(self.faces) != 0:

            #     matched_face, matching_type = self.find_matching_face((x, y, w, h), representation, self.faces)

            #     if matched_face is not None:
            #         matched_face.update(x, x + w, y, y + h, face_img, representation, identity, distance, matching_type)
            #         face = matched_face
            #         if face.identity_is_valid and face.identity == None:
            #             self.add_face_to_database(face)

            #         # self.logger.info("Same face found")

            if face is None:
                # Matching face not found, create new one
                face = Face(x, x + w, y, y + h, face_img, representation, identity, distance)

                self.logger.info("new face found")

            if self.correlation_tracker_enabled:
                face.start_track(frame)
            faces.append(face)
        return faces

    def add_face_to_database(self, face: Face):
        """
        Method to add new face to the face database
        """
        # TODO Add face to sqlite database
        identity = f"new_{len(self.face_representations)}"
        self.face_representations.append(face.representation)
        self.face_ids.append(identity)
        face.identity = identity
        self.logger.info("Face added to the face database")

    # TODO: adjust distance_treshold
    def find_matching_face(self, face_coords, representation, faces, distance_treshold_multiplier=1):
        """
        Method for finding maching face in faces list.

        face_coords Tuple(x, y, w, h)

        representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model

        faces List[Face]: List of Face object, where matching face are looked from.

        return (Face, String): where Face is the maching Face object or None
                               and String="representation", if match is done with face representaitons
                               String="distance", if if match is done with face position.
        """
        # compare representations
        representations = [face.representation for face in faces]
        matching_index, distance = self.face_recognizer.match_face(representation, representations)
        if matching_index is not None:
            return faces[matching_index], "representation"
        
        (x, y, w, h) = face_coords

        # Find closes face
        closest_face, distance = self.closest_face(x, y, w, h, faces)
        if distance < (closest_face.right - closest_face.left) * distance_treshold_multiplier:
            return closest_face, "distance"
        
        return None, None

    @staticmethod
    def closest_face(x, y, w, h, faces):
        """
        Method to find closes face from list of faces.
        Returns: tuple (closes face, distance) or None
        """
        if not faces:
            return None

        closest_face: Face = None
        distance = None

        #middle point
        middle_point = np.array([x + w / 2, y + h / 2])

        closest_face: Face = None
        min_distance = None

        for face in faces:
            # TODO: use deepface to verify that faces are same?
            # Calculate distance to face
            face_middle_point = np.array([face.left + (face.right - face.left) / 2,
                                          face.top + (face.bottom - face.top) / 2])
            distance = np.linalg.norm(middle_point-face_middle_point)
            if not min_distance or distance < min_distance:
                min_distance = distance
                closest_face = face

        return closest_face, min_distance

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

        # if face.identity:
        cv2.putText(frame,
                    f"Identity: {face.identity}",
                    (face.left + 2, face.top + 10),
                    self.font,
                    0.3,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)  
               
            # cv2.putText(frame,
            #             f"Last result: {face.last_identity}, {face.last_identity_distance}",
            #             (face.left + 2, face.top + 20),
            #             self.font,
            #             0.3,
            #             (255, 255, 255),
            #             1,
            #             cv2.LINE_AA)

    def publish_face_location(self):
        # Check that there is a location to publish
        if self.face_location:
            # Publish face location
            self.face_location_publisher.publish(self.face_location)
            # Set location back to None to prevent publishing same location multiple times
            self.face_location = None
