import dlib
from typing import List

class Face():
    def __init__(self, left, right, top, bottom, image, representation, cluster_dict):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

        self.image = image
        self.representation: List[float] = representation

        self.rect = dlib.rectangle(left, top, right, bottom)
        self.correlation_tracker = None # dlib correlation tracker

        self.speaking = None

        self.concurrent_validations = 0

        self.identity_is_valid = False
        self.cluster_dict = cluster_dict

    def start_track(self, frame):
        """
        Init and start dlib correlation tracker.
        """
        self.rect = dlib.rectangle(self.left, self.top, self.right, self.bottom)
        self.correlation_tracker = dlib.correlation_tracker()
        self.correlation_tracker.start_track(frame, self.rect)
    
    def update_location(self, frame):
        """
        Update face location with dlib correlation tracker.
        """
        self.correlation_tracker.update(frame)
        pos = self.correlation_tracker.get_position()
        
        #unpack the face position
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
            'bottom': face bottom coordinate in the frame:,
            'face_id': string of uuid4 or None, identifier of the face,
            'previous_occurances': List[dict] or None. List of previous occurances, when face has been visible. 
                                   Dict  includes keys "start_time", "stop_time" and "duration".
        """
        if self.cluster_dict is None:
            face_id = ""
            previous_occurances = []
        else:
            face_id = self.cluster_dict["id"]
            previous_occurances = self.cluster_dict["conversations"]
        if self.speaking is None:
            speaking = ""
        else:
            speaking = self.speaking
        return {
            'left': self.left,
            'right': self.right,
            'top': self.top,
            'bottom': self.bottom,
            'face_id': face_id,
            'previous_occurances': previous_occurances,
            'speaking': speaking,
        }
