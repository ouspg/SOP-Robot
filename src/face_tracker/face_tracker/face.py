import dlib

# TODO: change to better name, plain face has conflict with Face class in face_tracker_msg
class FaceStruct():
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

        self.rect = dlib.rectangle(x1, y1, x2, y2)
        self.correlation_tracker = None # dlib correlation tracker

        self.speaking = None
        self.identity = None

    def start_track(self, frame):
        """
        Init and start dlib correlation tracker.
        """
        self.correlation_tracker = dlib.correlation_tracker()
        self.correlation_tracker.start_track(frame, self.rect)
    
    def update_location(self, frame):
        """
        Update face location with dlib correlation tracker.
        """
        self.correlation_tracker.update(frame)
        pos = self.correlation_tracker.get_position()
        
        #unpack the face position
        self.x1 = int(pos.left())
        self.x2 = int(pos.right())
        self.y1 = int(pos.top())
        self.y2 = int(pos.bottom())