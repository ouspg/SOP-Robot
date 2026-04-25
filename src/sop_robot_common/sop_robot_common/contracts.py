"""Canonical ROS graph contracts shared outside the voice stack."""

FACE_TRACKER_NAMESPACE = "/face_tracker"
FACE_TRACKER_MOVEMENT_NAMESPACE = "/face_tracker_movement"

RAW_IMAGE_TOPIC = "/image_raw"
FACE_IMAGE_TOPIC = f"{FACE_TRACKER_NAMESPACE}/image_face"
FACES_TOPIC = f"{FACE_TRACKER_NAMESPACE}/faces"

FOCUSED_FACE_TOPIC = f"{FACE_TRACKER_MOVEMENT_NAMESPACE}/focused_face"
HEAD_GESTURE_TOPIC = f"{FACE_TRACKER_MOVEMENT_NAMESPACE}/head_gesture_topic"

ARM_ACTION_TOPIC = "/arms/arm_action"
ARM_FEEDBACK_TOPIC = "/arms/feedback"
LEGACY_ARM_FEEDBACK_TOPIC = "feedback"
RIGHT_HAND_GESTURE_TOPIC = "/r_hand/r_hand_topic"
LEFT_HAND_GESTURE_TOPIC = "/l_hand/l_hand_topic"
