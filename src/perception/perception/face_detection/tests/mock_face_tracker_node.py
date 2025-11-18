import os
import cv2
import numpy as np
from perception.face_detection.core.face_analyzer import FaceAnalyzer
from perception.face_detection.core.lip_movement_net import LipMovementDetector

# Dummy logger to satisfy logger calls in FaceAnalyzer
class DummyLogger:
    def info(self, msg):
        print("[INFO] ", msg)
    def warning(self, msg):
        print("[WARN] ", msg)
    def error(self, msg):
        print("[ERROR] ", msg)
    def debug(self, msg):
        print("[DEBUG] ", msg)

def test_face_detection_with_mock():
    # Path to test image (place a real image in the tests folder)
    img_path = os.path.join(os.path.dirname(__file__), "data/test_image.jpg")

    # Load the image
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Test image not found at {img_path}")

    # Initialize dummy lip movement detector
    lip_detector = LipMovementDetector(
        model_path=os.path.join(os.path.dirname(__file__), "../models/1_32_False_True_0.25_lip_motion_net_model.h5"),
        shape_predictor_path=os.path.join(os.path.dirname(__file__), "../predictors/shape_predictor_68_face_landmarks.dat")
    )

    # Initialize analyzer
    analyzer = FaceAnalyzer(logger=DummyLogger(), lip_movement_detector=lip_detector)

    # Run face analysis
    faces_info = analyzer.on_frame_received(img)
    print("Faces detected:", faces_info)

    # Display image with overlays
    cv2.imshow("Face Detection Test", img)
    print("Press any key on the image window to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_face_detection_with_mock()
