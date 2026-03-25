from pathlib import Path
import importlib.util
from types import SimpleNamespace


MODULE_PATH = Path(__file__).resolve().parents[1] / "face_tracker_movement" / "face_tracker_movement_node.py"
SPEC = importlib.util.spec_from_file_location("face_tracker_movement_node", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def build_node():
    node = MODULE.FaceTrackerMovementNode.__new__(MODULE.FaceTrackerMovementNode)
    node.camera_angle_eye_horizontal_coeff = -2.0
    node.camera_angle_eye_vertical_coeff = 4.0
    node.camera_angle_head_pan_coeff = -1.5
    node.camera_angle_head_vertical_coeff = -1.25
    node.eyes_servo_state = [0.0, 0.0]
    node.head_state = [0.1, 0.0, 0.0, 1.0]
    node.eye_horizontal_lower_limit = -1.0
    node.eye_horizontal_upper_limit = 1.0
    node.eye_vertical_lower_limit = -0.5
    node.eye_vertical_upper_limit = 0.5
    node.head_pan_lower_limit = -1.0
    node.head_pan_upper_limit = 1.0
    node.head_vertical_lower_limit = 0.5
    node.head_vertical_upper_limit = 1.5
    node.eyes_center_position = [0.0, 0.0]
    node.eyes_joint_ids = [9, 11]
    node.logger = SimpleNamespace(info=lambda *_: None)
    return node


def test_transform_camera_angle_to_eye_location_clamps_values():
    node = build_node()

    horizontal, vertical = node.transform_camera_angle_to_eye_location(-2.0, 2.0)

    assert horizontal == 1.0
    assert vertical == 0.5


def test_transform_camera_angle_to_head_location_uses_head_vertical_coefficient():
    node = build_node()

    pan, vertical = node.transform_camera_angle_to_head_location(0.2, 0.4)

    assert pan == -0.2
    assert vertical == 0.5


def test_eyes_state_callback_updates_camera_space_state():
    node = build_node()
    msg = SimpleNamespace(actual=SimpleNamespace(positions=[0.4, -0.2]))

    node.eyes_state_callback(msg)

    assert node.eyes_servo_state == [0.4, -0.2]
    assert node.eyes_state == [-0.2, -0.05]
