from typing import Any

import torch
from deepface import DeepFace
from deepface.detectors import DetectorWrapper
from deepface.models.FacialRecognition import FacialRecognition


class FaceRecognizer:
    def __init__(
        self,
        logger,
        model_name,
        detector_backend,
        *,
        recognition_enabled=True,
        inference_device='cuda:0',
    ):
        """
        Initialize face recognizer, and create embeddings in the intialization
        """
        self.logger = logger
        self.model_name = model_name
        self.detector_backend = detector_backend
        self.recognition_enabled = recognition_enabled
        self.inference_device = self._select_device(inference_device)

        self._configure_detector()

        self.model: FacialRecognition | None = None
        if recognition_enabled:
            self.model = DeepFace.build_model(model_name=model_name)
            logger.info(f'Facial recognition model {model_name} is ready.')

        self.logger.info(f'Face detector {detector_backend} is ready on {self.inference_device}.')

    def _select_device(self, requested_device: str) -> str:
        if requested_device.startswith('cuda') and not torch.cuda.is_available():
            self.logger.warning(
                f'CUDA device {requested_device} was requested but is unavailable; using CPU.'
            )
            return 'cpu'
        return requested_device

    def _configure_detector(self):
        detector: Any = DetectorWrapper.build_model(self.detector_backend)
        model: Any = getattr(detector, 'model', None)
        if model is None or not hasattr(model, 'to'):
            if self.inference_device.startswith('cuda'):
                self.logger.warning(
                    f'Detector {self.detector_backend} cannot be moved to CUDA; it will use CPU.'
                )
            return
        model.to(self.inference_device)

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
            detector_backend=self.detector_backend,
            enforce_detection=False,
            align=self.recognition_enabled,
        )
        return [
            face_obj
            for face_obj in face_objs
            if face_obj['confidence'] > 0
            and face_obj['facial_area']['w'] < img.shape[1] * 0.8
            and face_obj['facial_area']['h'] < img.shape[0] * 0.8
        ]

    def represent(self, img):
        """
        This function calculates vector representation for one face

        Returns representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).
        """
        if not self.recognition_enabled or self.model is None:
            return []

        target_embedding_obj = DeepFace.represent(
            img_path=img,
            model_name=self.model_name,
            detector_backend='skip',
        )
        return target_embedding_obj[0]['embedding']
