import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from sop_robot_common.contracts import (
    ARM_ACTION_TOPIC,
    ARM_FEEDBACK_TOPIC,
    FACE_IMAGE_TOPIC,
    FACES_TOPIC,
    FACE_TRACKER_MOVEMENT_NAMESPACE,
    FACE_TRACKER_NAMESPACE,
    FOCUSED_FACE_TOPIC,
    HEAD_GESTURE_TOPIC,
    LEFT_HAND_GESTURE_TOPIC,
    RIGHT_HAND_GESTURE_TOPIC,
    RAW_IMAGE_TOPIC,
)


def test_face_tracker_topics_share_the_same_namespace_contract():
    assert RAW_IMAGE_TOPIC == "/image_raw"
    assert FACE_TRACKER_NAMESPACE == "/face_tracker"
    assert FACE_IMAGE_TOPIC == "/face_tracker/image_face"
    assert FACES_TOPIC == "/face_tracker/faces"


def test_motion_topics_share_the_same_namespace_contract():
    assert FACE_TRACKER_MOVEMENT_NAMESPACE == "/face_tracker_movement"
    assert FOCUSED_FACE_TOPIC == "/face_tracker_movement/focused_face"
    assert HEAD_GESTURE_TOPIC == "/face_tracker_movement/head_gesture_topic"
    assert ARM_ACTION_TOPIC == "/arms/arm_action"
    assert ARM_FEEDBACK_TOPIC == "/arms/feedback"
    assert LEFT_HAND_GESTURE_TOPIC == "/l_hand/l_hand_topic"
    assert RIGHT_HAND_GESTURE_TOPIC == "/r_hand/r_hand_topic"
