import numpy as np
import cv2
import scipy
import os
import glob
from tqdm import tqdm

from deepface import DeepFace
from deepface.models.FacialRecognition import FacialRecognition
from deepface.modules.verification import find_threshold

class FaceRecognizer(object):

    def __init__(self, db_path, logger, model_name, detector_backend, distance_metric):
        """
        Initialize face recognizer, and create embeddings in the intialization
        """
        self.logger = logger
        self.db_path = db_path
        self.model_name = model_name
        self.detector_backend = detector_backend
        self.distance_metric = distance_metric

        # build models once to store them in the memory
        self.model: FacialRecognition = DeepFace.build_model(model_name=model_name)

        # find custom values for this input set
        self.target_size = self.model.input_shape
        self.distance_treshlod = find_threshold(model_name, distance_metric)

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
        return DeepFace.extract_faces(
            img_path=img,
            target_size=self.target_size,
            detector_backend=self.detector_backend,
            enforce_detection=False,
        )
    
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

    def match_face(self, target_representation, source_representations):
        """
        Match face representation to list of representations.

        - target_representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).

        - source_representations (List[List[float]]): List of Multidimensional vectors representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).

        Returns: Tuple(index, distance) Index and distance of the matched representation
        """
        if self.distance_metric == "cosine":
            distances = self.find_cosine_distances(source_representations, np.array(target_representation))
        elif self.distance_metric == "euclidean":
            distances = self.find_euclidean_distances(source_representations, np.array(target_representation))
        else:
            raise ValueError(f"Invalid distance metric: {self.distance_metric}")
        
        # min_distance_index = np.where(distances == distances.min())
        min_distance_index = np.argmin(distances)
        min_distance = distances[min_distance_index]
        
        if min_distance > self.distance_treshlod:
            return (None, None)
        return (min_distance_index, min_distance)
    
    def get_database_representations(self):
        #TODO: save representations to file
        # Get registered photos and return as npy files
        # File name = id name, embeddings of a photo is the representative for the id
        representations = []
        user_ids = []
        # the tuple of file types, please ADD MORE if you want
        types = ('*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG', '*.JPEG')
        files = []
        for a_type in types:
            files.extend(glob.glob(os.path.join(self.db_path, "**/", a_type)))

        files = list(set(files))

        self.logger.info("Calculate database representations - start")

        for file in tqdm(files):
            embedding_objs = DeepFace.represent(
                img_path=file,
                model_name= self.model_name,
                enforce_detection= False,
                detector_backend= self.detector_backend,
                align= True,
                expand_percentage= 0,
                normalization= "base",
            )
            if embedding_objs is None:
                continue
            user_id = os.path.splitext(os.path.basename(file))[0]
            user_ids.append(user_id)
            representations.append(embedding_objs[0]["embedding"])

        self.logger.info("Calculate database representations - Done")

        self.logger.info(f'there are {len(representations)} ids')
        
        return user_ids, representations

    @staticmethod
    def find_cosine_distances(matrix, vector):
        """
        Compute the cosine distances between each row of matrix and vector.
        """
        v = vector.reshape(1, -1)
        return scipy.spatial.distance.cdist(matrix, v, 'cosine').reshape(-1)

    @staticmethod
    def find_euclidean_distances(matrix, vector):
        """
        Compute the euclidean distances between each row of matrix and vector.
        """
        v = vector.reshape(1, -1)
        return scipy.spatial.distance.cdist(matrix, v, 'euclidean').reshape(-1)
