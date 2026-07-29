import math
from typing import Any, cast

import dlib

dlib_api = cast(Any, dlib)


class Face:
    def __init__(self, left, right, top, bottom, image, representation, cluster_dict):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

        self.diagonal = math.sqrt((right - left) ** 2 + (bottom - top) ** 2)

        self.image = image
        self.representation: list[float] = representation

        self.rect = dlib_api.rectangle(left, top, right, bottom)
        self.correlation_tracker: Any | None = None

        self.speaking: str | None = None

        self.concurrent_validations = 0

        self.identity_is_valid = False
        self.cluster_dict = cluster_dict

    def start_track(self, frame):
        """
        Init and start dlib correlation tracker.
        """
        self.rect = dlib_api.rectangle(self.left, self.top, self.right, self.bottom)
        tracker = dlib_api.correlation_tracker()
        tracker.start_track(frame, self.rect)
        self.correlation_tracker = tracker

    def update_location(self, frame):
        """
        Update face location with dlib correlation tracker.
        """
        tracker = self.correlation_tracker
        if tracker is None:
            raise RuntimeError('Cannot update a face before its correlation tracker is started.')
        tracker.update(frame)
        pos = tracker.get_position()

        # unpack the face position
        self.left = int(pos.left())
        self.right = int(pos.right())
        self.top = int(pos.top())
        self.bottom = int(pos.bottom())

    def as_dict(self):
        """
        Return the class parameters as python dictionary:

        Reurns: Dictionary of face information:
            'left': face left coordinate in the frame,
            'right': face right coordinate in the frame,
            'top': face top coordinate in the frame,
            'bottom': face bottom coordinate in the frame,
            'diagonal': Diagonal length of the frame,
            'face_id': string of uuid4 or None, identifier of the face,
            'previous_occurances': List[dict] or None. List of previous occurances,
                                   when face has been visible. Dict includes keys
                                   "start_time", "stop_time" and "duration".
        """
        if self.cluster_dict is None:
            face_id = ''
            previous_occurances = []
        else:
            face_id = self.cluster_dict['id']
            previous_occurances = self.cluster_dict['conversations']
        speaking = '' if self.speaking is None else self.speaking
        return {
            'left': self.left,
            'right': self.right,
            'top': self.top,
            'bottom': self.bottom,
            'diagonal': self.diagonal,
            'face_id': face_id,
            'previous_occurances': previous_occurances,
            'speaking': speaking,
        }
