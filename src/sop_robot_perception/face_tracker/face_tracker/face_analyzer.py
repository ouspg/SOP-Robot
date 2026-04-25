# pyright: reportOptionalMemberAccess=false, reportCallIssue=false, reportArgumentType=false

import cv2
import numpy as np
import time
from typing import List

from .face import Face
from .face_recognition import FaceRecognizer
from .identity_store import SessionIdentityStore

class FaceAnalyzer:

    def __init__(self,
                logger,
                lip_movement_detector=None,
                face_recognizer=True,
                correlation_tracker=True,
                cluster_similarity_threshold=0.3,
                subcluster_similarity_threshold=0.2,
                pair_similarity_maximum=1.0,
                face_recognition_model="SFace",
                face_detection_model="yunet",
                prefer_gpu=False,
                gpu_face_recognition_model="SFace",
                gpu_face_detection_model="yolov8n",
                face_detection_confidence=0.6,
                face_detection_imgsz=640,
                no_face_detection_interval_frames=1,
                no_face_detection_warmup_frames=3,
                face_detection_interval_frames=5,
                face_identity_refresh_seconds=3.0,
                track_match_iou_threshold=0.25,
                recent_face_memory_seconds=2.0,
                identity_store_max_identities=8,
                identity_store_ttl_seconds=300.0,
                draw_debug=True):
        self.logger = logger
        self.correlation_tracker_enabled = correlation_tracker
        self.lip_movement_detector = lip_movement_detector
        self.face_detection_interval_frames = max(1, int(face_detection_interval_frames))
        self.face_identity_refresh_seconds = max(0.0, float(face_identity_refresh_seconds))
        self.track_match_iou_threshold = max(0.0, float(track_match_iou_threshold))
        self.recent_face_memory_seconds = max(0.0, float(recent_face_memory_seconds))
        self.identity_store_max_identities = max(0, int(identity_store_max_identities))
        self.identity_store_ttl_seconds = max(0.0, float(identity_store_ttl_seconds))
        self.no_face_detection_interval_frames = max(
            1,
            int(no_face_detection_interval_frames),
        )
        self.no_face_detection_warmup_frames = max(
            0,
            int(no_face_detection_warmup_frames),
        )
        self.draw_debug = bool(draw_debug)

        # Face recognition
        if face_recognizer:
            self.face_recognizer = FaceRecognizer(logger=self.logger,
                                                  model_name=face_recognition_model,
                                                  detector_backend=face_detection_model,
                                                  prefer_gpu=prefer_gpu,
                                                  gpu_face_recognition_model=gpu_face_recognition_model,
                                                  gpu_face_detection_model=gpu_face_detection_model,
                                                  face_detection_confidence=face_detection_confidence,
                                                  face_detection_imgsz=face_detection_imgsz)
        else:
            self.face_recognizer = None

        self.face_ids = []
        self.face_representations = []

        self.cluster_similarity_threshold = cluster_similarity_threshold
        self.subcluster_similarity_threshold = subcluster_similarity_threshold
        self.pair_similarity_maximum = pair_similarity_maximum
        self.cluster = SessionIdentityStore(
            match_similarity_threshold=self.subcluster_similarity_threshold,
            max_identities=self.identity_store_max_identities,
            ttl_seconds=self.identity_store_ttl_seconds,
            logger=self.logger,
        )

        self.frame = 0
        self.processed_frames = 0
        self.empty_scene_frames = 0
        self.faces: List[Face] = []
        self.recent_faces: List[Face] = []
        self.recent_faces_seen_at = 0.0

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        # self.timer = self.create_timer(2, self.profile_cycle)
        # pr.enable()
            
    def on_frame_received(self, frame: cv2.typing.MatLike):
        cv2_gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        run_detector = self._should_run_detector()

        # Get the face locations
        if run_detector:
            faces_len_old = len(self.faces)
            # Use face detection to get face locations
            self.faces = self.analyze_frame(frame)
            if self.faces:
                self.empty_scene_frames = 0
            else:
                self.empty_scene_frames += 1

            # Initialize new input sequences for lip movement detector if the number of detected faces change
            if self.lip_movement_detector is not None:
                if faces_len_old != len(self.faces):
                    #TODO: original implementation had speaking state clearing here
                    self.lip_movement_detector.initialize_input_sequence(len(self.faces))

            # self.logger.info(f"Face detection: faces={len(self.faces)}")
            
        elif self.correlation_tracker_enabled:
            # Use dlib correlation tracker to update face locations
            for face in self.faces:
                face.update_location(frame)

            # self.logger.info(f"correlation tracking: faces={len(self.faces)}")
        else:
            self.faces = []
            self.empty_scene_frames += 1
        
        # loop through all faces
        for i, face in enumerate(self.faces):
            if self.lip_movement_detector is not None:
                # Determine if the face is speaking or silent
                face.speaking = self.lip_movement_detector.test_video_frame(cv2_gray_img, face.rect, i)

            if self.draw_debug:
                # Draw information to frame
                self.draw_face_info(frame, face)

        if self.draw_debug:
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
            self.frame = self.frame % self.face_detection_interval_frames

        self.processed_frames += 1
        
        return [face.as_dict() for face in self.faces]

    def _should_run_detector(self) -> bool:
        if self.correlation_tracker_enabled and self.faces:
            return self.frame == 0

        if self.faces:
            return True

        if self.no_face_detection_interval_frames <= 1:
            return True

        if self.empty_scene_frames < self.no_face_detection_warmup_frames:
            return True

        return self.processed_frames % self.no_face_detection_interval_frames == 0
    
    def analyze_frame(self, frame):
        """
        Get face objects from frame. Do face detection and recognition. Intialize dlib correlation trackers.
        """
        faces: List[Face] = []
        now = time.time()
        previous_faces = self._previous_faces_for_matching(now)
        matched_previous_faces: set[int] = set()

        # Uses deepface to extract face locations from frame
        face_objs = self.face_recognizer.extract_faces(frame)
        
        for face_obj in face_objs:
            
            face_img = face_obj["face"]
            face_region = face_obj["facial_area"]
            x = face_region["x"]
            y = face_region["y"]
            w = face_region["w"]
            h = face_region["h"]

            previous_face = self._match_previous_face(
                x,
                y,
                x + w,
                y + h,
                previous_faces,
                matched_previous_faces,
            )

            if previous_face is None or previous_face.cluster_dict is None:
                representation: List[float] = self.face_recognizer.represent(face_img)
                cluster_predictation = self.cluster.predict(np.array(representation))
                last_recognition_time = now
                speaking = None
            else:
                representation = previous_face.representation
                cluster_id = previous_face.cluster_dict["id"]
                cluster_predictation = self.cluster.touch_cluster(cluster_id)
                if cluster_predictation is None:
                    cluster_predictation = previous_face.cluster_dict

                last_recognition_time = previous_face.last_recognition_time
                should_refresh_identity = (
                    now - previous_face.last_recognition_time
                    >= self.face_identity_refresh_seconds
                )
                if should_refresh_identity:
                    refreshed_representation: List[float] = self.face_recognizer.represent(face_img)
                    cluster_predictation, accepted = self.cluster.update_known_cluster(
                        cluster_id,
                        np.array(refreshed_representation),
                    )
                    last_recognition_time = now
                    if accepted:
                        representation = refreshed_representation
                    else:
                        self.logger.debug(
                            "Ignoring transient face embedding that does not match "
                            f"tracked cluster {cluster_id}"
                        )

                speaking = previous_face.speaking

            # Matching face not found, create new one
            face = Face(
                x,
                x + w,
                y,
                y + h,
                face_img,
                representation,
                cluster_predictation,
                last_recognition_time=last_recognition_time,
                speaking=speaking,
            )

            if self.correlation_tracker_enabled:
                face.start_track(frame)
            faces.append(face)
        if faces:
            self.recent_faces = faces
            self.recent_faces_seen_at = now
        return faces

    def _previous_faces_for_matching(self, now: float) -> List[Face]:
        if self.faces:
            return self.faces
        if now - self.recent_faces_seen_at <= self.recent_face_memory_seconds:
            return self.recent_faces
        return []

    def _match_previous_face(
        self,
        left: int,
        top: int,
        right: int,
        bottom: int,
        previous_faces: List[Face],
        matched_previous_faces: set[int],
    ) -> Face | None:
        best_index = None
        best_iou = 0.0

        for index, previous_face in enumerate(previous_faces):
            if index in matched_previous_faces:
                continue

            iou = self._bbox_iou(
                left,
                top,
                right,
                bottom,
                previous_face.left,
                previous_face.top,
                previous_face.right,
                previous_face.bottom,
            )
            if iou > best_iou:
                best_iou = iou
                best_index = index

        if best_index is None or best_iou < self.track_match_iou_threshold:
            return None

        matched_previous_faces.add(best_index)
        return previous_faces[best_index]

    @staticmethod
    def _bbox_iou(
        left_a: int,
        top_a: int,
        right_a: int,
        bottom_a: int,
        left_b: int,
        top_b: int,
        right_b: int,
        bottom_b: int,
    ) -> float:
        intersection_left = max(left_a, left_b)
        intersection_top = max(top_a, top_b)
        intersection_right = min(right_a, right_b)
        intersection_bottom = min(bottom_a, bottom_b)

        intersection_width = max(0, intersection_right - intersection_left)
        intersection_height = max(0, intersection_bottom - intersection_top)
        intersection_area = intersection_width * intersection_height
        if intersection_area == 0:
            return 0.0

        area_a = max(0, right_a - left_a) * max(0, bottom_a - top_a)
        area_b = max(0, right_b - left_b) * max(0, bottom_b - top_b)
        union_area = area_a + area_b - intersection_area
        if union_area <= 0:
            return 0.0

        return intersection_area / union_area

    def draw_face_info(self, frame, face:Face):
        """
        Draws rectangle around face and other information to display 
        """
        green = (0, 255, 0)

        # Draw rectangle around the face
        cv2.rectangle(frame, (face.left, face.top), (face.right, face.bottom), green, 1)
        
        if self.lip_movement_detector is not None:
            cv2.putText(frame,
                        f"Face is speaking: {face.speaking}",
                        (face.left + 2, face.top + 20),
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
