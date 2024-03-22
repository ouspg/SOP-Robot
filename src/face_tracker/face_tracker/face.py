import dlib

from collections import deque
from collections import Counter

# TODO: change to better name, plain face has conflict with Face class in face_tracker_msg
class Face():
    def __init__(self, left, right, top, bottom):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

        self.rect = dlib.rectangle(left, top, right, bottom)
        self.correlation_tracker = None # dlib correlation tracker

        self.speaking = None
        self.identity = None
        self.last_identity = None
        self.identity_deque: deque = deque(maxlen=10)

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
    
    def update_identity(self, identity):
        """
        Update face identity deque with new identity value from face recognition result. 
        Doen't add None to the deque.
        Calculate the face identity by calculating the most common identity in the deque.
        Update the resent identity value.
        """
        if identity is not None:
            self.identity_deque.append(identity)
            
            # Do not update unnecessarily
            if identity != self.identity:
                identity_counts = Counter(self.identity_deque)
                self.identity = identity_counts.most_common(1)[0][0]

        self.last_identity = identity
        


