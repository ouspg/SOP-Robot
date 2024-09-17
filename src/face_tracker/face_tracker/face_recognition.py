import numpy as np
import scipy

from deepface import DeepFace
from deepface.models.FacialRecognition import FacialRecognition
from deepface.modules.verification import find_threshold

class FaceRecognizer(object):

    def __init__(self, db_path, logger, model_name, detector_backend):
        """
        Initialize face recognizer, and create embeddings in the intialization
        """
        self.logger = logger
        self.db_path = db_path
        self.model_name = model_name
        self.detector_backend = detector_backend

        # build models once to store them in the memory
        self.model: FacialRecognition = DeepFace.build_model(model_name=model_name)

        # find custom values for this input set
        self.target_size = self.model.input_shape

        logger.info(f"facial recognition model {model_name} is just built")

        self.logger.info("FaceRecognizer initialized!")
    
    def extract_faces(self, img):
        """
        Extract faces from image. Discards small faces.
        Returns:
        results (List[Dict[str, Any]]): A list of dictionaries, where each dictionary contains:

        - "face" (np.ndarray): The detected face as a NumPy array.

        - "facial_area" (Dict[str, Any]): The detected face's regions as a dictionary containing:
            - keys 'x', 'y', 'w', 'h' with int values
            - keys 'left_eye', 'right_eye' with a tuple of 2 ints as values

        - "confidence" (float): The confidence score associated with the detected face.
        """
        face_objs = DeepFace.extract_faces(
            img_path=img,
            target_size=self.target_size,
            detector_backend=self.detector_backend,
            enforce_detection=False,
        )
        return [face_obj for face_obj in face_objs if face_obj["facial_area"]["w"] < img.shape[0] * 0.8]
    
    def represent(self, img):
        """
        This function calculates vector representation for one face

        Returns representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).
        """
        target_embedding_obj = DeepFace.represent(
            img_path=img,
            model_name=self.model_name,
            detector_backend="skip",
            )
        return target_embedding_obj[0]["embedding"]
