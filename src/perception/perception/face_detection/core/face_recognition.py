from typing import List, Dict, Any
from deepface import DeepFace
from deepface.models.FacialRecognition import FacialRecognition
import numpy as np

class FaceRecognizer:
    """Face recognition utility using DeepFace for detection and embedding extraction."""

    def __init__(self, logger, model_name: str, detector_backend: str):
        """
        Initialize the face recognizer and load the model into memory.

        Args:
            logger: Logger instance.
            model_name: Name of the DeepFace recognition model (e.g., 'SFace', 'Facenet').
            detector_backend: Backend for face detection (e.g., 'dlib', 'opencv', 'mtcnn', 'retinaface', 'mediapipe').
        """
        self.logger = logger
        self.model_name = model_name
        self.detector_backend = detector_backend

        # Load the model once
        self.model: FacialRecognition = DeepFace.build_model(model_name=model_name)
        self.logger.info(f"Facial recognition model '{model_name}' loaded into memory.")
        self.logger.info("FaceRecognizer initialized!")

    def extract_faces(self, img: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect faces in an image and filter out very large faces.

        Args:
            img: Image as a NumPy array (BGR format).

        Returns:
            List of dictionaries for each detected face containing:
                - "face": Cropped face image.
                - "facial_area": Dictionary with keys 'x', 'y', 'w', 'h' and optional landmarks.
                - "confidence": Detection confidence score.
        """
        face_objs = DeepFace.extract_faces(
            img_path=img,
            detector_backend=self.detector_backend,
            enforce_detection=False,
        )
        # Filter out faces that are unreasonably large
        return [
            face_obj
            for face_obj in face_objs
            if face_obj["facial_area"]["w"] < img.shape[0] * 0.8
        ]

    def represent(self, img: np.ndarray) -> List[float]:
        """
        Compute the embedding vector for a given face image.

        Args:
            img: Cropped face image as a NumPy array.

        Returns:
            List of floats representing the face embedding vector.
        """
        embeddings = DeepFace.represent(
            img_path=img,
            model_name=self.model_name,
            detector_backend="skip",
        )
        return embeddings[0]["embedding"]
