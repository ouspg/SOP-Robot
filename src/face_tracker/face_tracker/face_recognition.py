from deepface import DeepFace

class FaceRecognizer(object):

    def test_deepface(logger):
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