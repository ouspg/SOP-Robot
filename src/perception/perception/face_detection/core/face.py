import dlib
import math
from typing import List, Optional, Dict

class Face:
    """Represents a detected face with tracking, lip movement, and clustering info."""

    def __init__(
        self,
        left: int,
        right: int,
        top: int,
        bottom: int,
        image,
        representation: List[float],
        cluster_dict: Optional[Dict] = None,
    ):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom
        self.diagonal = math.sqrt((right - left) ** 2 + (bottom - top) ** 2)

        self.image = image
        self.representation: List[float] = representation
        self.cluster_dict: Optional[Dict] = cluster_dict

        self.rect = dlib.rectangle(left, top, right, bottom)
        self.correlation_tracker: Optional[dlib.correlation_tracker] = None

        self.speaking: Optional[bool] = None
        self.concurrent_validations = 0
        self.identity_is_valid = False

    def start_track(self, frame):
        """Initialize and start a dlib correlation tracker."""
        self.rect = dlib.rectangle(self.left, self.top, self.right, self.bottom)
        self.correlation_tracker = dlib.correlation_tracker()
        self.correlation_tracker.start_track(frame, self.rect)

    def update_location(self, frame):
        """Update face location using the dlib correlation tracker."""
        if not self.correlation_tracker:
            return
        self.correlation_tracker.update(frame)
        pos = self.correlation_tracker.get_position()
        self.left = int(pos.left())
        self.right = int(pos.right())
        self.top = int(pos.top())
        self.bottom = int(pos.bottom())

    def as_dict(self) -> Dict:
        """
        Return the face attributes as a dictionary.

        Returns:
            dict: {
                'left', 'right', 'top', 'bottom', 'diagonal',
                'face_id', 'previous_occurances', 'speaking'
            }
        """
        face_id = self.cluster_dict["id"] if self.cluster_dict else ""
        previous_occurrences = self.cluster_dict.get("conversations", []) if self.cluster_dict else []
        speaking = self.speaking if self.speaking is not None else ""

        return {
            "left": self.left,
            "right": self.right,
            "top": self.top,
            "bottom": self.bottom,
            "diagonal": self.diagonal,
            "face_id": face_id,
            "previous_occurances": previous_occurrences,
            "speaking": speaking,
        }
