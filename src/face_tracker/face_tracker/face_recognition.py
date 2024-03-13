import numpy as np
import cv2

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

    def find_match(self, frame, face_coords):
        """
        This function finds matching face from the face database.
        If match is not found, image is saved to the database as a new person.

        face_coords: [x, y, w, h]
        """

        cropped_img = self._crop_face_image(frame, face_coords, padding=20)

        dfs = DeepFace.find(
            img_path=frame,
            db_path=self.db_path,
            model_name=self.model_name,
            detector_backend=self.detector_backend,
            #detector_backend="skip",
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
                return label
        return None

    #@staticmethod
    def _crop_face_image(self, original_img, face_coords, padding=None):
        '''
        Crop face image from original_img using face_coords.
        if padding is not None, add padding to face coordinates to include more of the detected face
        for better emotion recognition.
        If no room for padding, return the original sized face image.
        :return: 48x48 grayscale image and [x, y, w, h] or None
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

        # crop ROI from the webcam image
        face = original_img[y:y+h, x:x+w]

        # TODO: Should the image be resized?
        #try:
            # convert image to grayscale
            # face_gray = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
            # cv2_gray_img = cv2.equalizeHist(face_gray)
            # resize image to 48x48 (try cv2.INTER_AREA for better results but slower)
            # face_gray = cv2.resize(face, (48, 48), interpolation=cv2.INTER_NEAREST)
            # face_np = np.asarray(face_gray[:,:])
        #except Exception as e:
        #    self.logger.error(str(e))
        #    return None
        
        return face
