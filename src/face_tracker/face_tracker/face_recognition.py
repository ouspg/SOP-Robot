import numpy as np
import cv2

from deepface import DeepFace
from deepface.models.FacialRecognition import FacialRecognition

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

        logger.info(f"facial recognition model {model_name} is just built")

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
    
    def extract_faces(self, img):
        """
        Extract faces from image. Discards small faces.
        Returns:
        List[tuple(x, y, w, h)]
        """
        try:
            face_objs = DeepFace.extract_faces(
                img_path=img,
                target_size=self.target_size,
                detector_backend=self.detector_backend,
                enforce_detection=False,
            )
            self.logger.info(f"face_recognition: face_objs: {face_objs}")
            faces = []
            for face_obj in face_objs:
                facial_area = face_obj["facial_area"]
                # if facial_area["w"] <= 130:  # discard small detected faces
                #     continue
                faces.append(
                    (
                        facial_area["x"],
                        facial_area["y"],
                        facial_area["w"],
                        facial_area["h"],
                    )
                )
        except:  # to avoid exception if no face detected
            faces = []
        return faces
    
    def find(self, face_img):
        """
        This function finds match for face from database.
        imput:
            face_img: aligned and cropped face image.
        returns:
            string: matching images name
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

        if len(dfs) > 0:
            # access 1st item
            df = dfs[0]
            #self.logger.info(str(df))

            if df.shape[0] > 0:
                candidate = df.iloc[0]
                label = candidate["identity"]
                name = label.split("/")[-2]
                return name
        return None

    def find_match(self, img, face_coords):
        """
        This function finds matching face from the face database.
        If match is not found, image is saved to the database as a new person.

        face_coords: [x, y, w, h]

        returns: Name of the matched images directory in database
        """

        cropped_img = self._crop_face_image(img, face_coords, padding=None, resize=False)

        dfs = DeepFace.find(
            img_path=img,
            db_path=self.db_path,
            model_name=self.model_name,
            detector_backend=self.detector_backend,
            distance_metric=self.distance_metric,
            enforce_detection=False,
            silent=True,
        )

        if len(dfs) > 0:
            # access 1st item
            df = dfs[0]
            #self.logger.info(str(df))

            if df.shape[0] > 0:
                candidate = df.iloc[0]
                label = candidate["identity"]
                name = label.split("/")[-2]
                return name
        return None

    @staticmethod
    def _crop_face_image(original_img, face_coords, padding=None, resize=True):
        '''
        Crop face image from original_img using face_coords.
        if padding is not None, add padding to face coordinates to include more of the detected face
        for better emotion recognition.
        If no room for padding, return the original sized face image.
        :return: 48x48 image or None
        '''
        # unpack face coordinates from a tuple
        x, y, w, h = face_coords

        if padding is not None and padding > 0:
            # get array/image shape
            y_max, x_max, *_ = original_img.shape

            # check if padded coordinates are within bounds
            if (0 <= y-padding and y+h+padding < y_max and
            0 <= x-padding and x+w+padding < x_max): 
                x -= padding
                y -= padding
                h += padding
                w += padding

        # crop face from the webcam image
        face = original_img[y:y+h, x:x+w]

        if resize:
            face = cv2.resize(face, (124, 124), interpolation=cv2.INTER_NEAREST)
        # return face
        return face

def main(args=None):
    # Initialize

    import logging
    import sys
    import cv2
    import os
    import dlib
    # Set up the logger
    logger = logging.getLogger("")
    logger.setLevel(logging.DEBUG)

    # Create a stream handler (logs to stdout)
    console_handler = logging.StreamHandler(sys.stdout)
    logger.addHandler(console_handler)

    face_db_path = os.path.expanduser('~')+"/database"

    face_recognizer = FaceRecognizer(db_path=face_db_path,
                                     logger=logger,
                                     model_name="VGG-Face",
                                     detector_backend="opencv",
                                     distance_metric="cosine")
    
    face_recognizer.test_deepface(logger)

    # Initialize video capture (0 for default webcam)
    cap = cv2.VideoCapture(0)
    
    face_detector = dlib.get_frontal_face_detector()

    while True:
        # Read a frame from the video stream
        ret, frame = cap.read()

        # Process the frame (e.g., apply filters, resize, etc.)

        # Display the frame
        cv2.imshow("Video Stream", frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
