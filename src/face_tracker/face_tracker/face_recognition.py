import numpy as np

from deepface import DeepFace

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

        # call a dummy find function for db_path once to create embeddings in the initialization
        DeepFace.find(
            img_path=np.zeros([224, 224, 3]),
            db_path=db_path,
            model_name=model_name,
            detector_backend=detector_backend,
            distance_metric=distance_metric,
            enforce_detection=False,
        )

        self.logger.info("FaceRecognizer initialized!")

    def find_match(self, face_img):
        """
        This functin finds matching face from the face database.
        If match is not found, image is saved to the database as a new person.
        """

        dfs = DeepFace.find(
            img_path=face_img,
            db_path=self.db_path,
            model_name=self.model_name,
            detector_backend="skip",
            distance_metric=self.distance_metric,
            enforce_detection=False,
            silent=True,
        )

    def test_deepface(self, logger):
        """
        Function for checking that deepface works.
        TODO: Remove, when something proper is implemented.
        """
        logger.info("test_deepface")

        db_path = "/home/user/testing_deepface/db"

        # Load the image
        img_matti = db_path + "/matti/kuva.jpg"
        img_matti_2 = db_path + "/matti/kuva2.jpg"
        img_rasmus = db_path + "/rasmus/kuva.jpg"

        # Perform facial recognition
        # result = DeepFace.verify(img_matti, img_matti_2)
        result = DeepFace.find(img_path = img_matti, db_path=db_path)

        # Print the result
        print(result)
        logger.info(str(result))

        result = DeepFace.find(img_path = img_rasmus, db_path=db_path)

        # Print the result
        print(result)
        logger.info(str(result))