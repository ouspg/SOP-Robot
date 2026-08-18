import pytest
import math
from unittest.mock import MagicMock

from face_tracker_msgs.msg import Face, Point2


class MockFace:
    """Minimal mock for Face msg."""

    def __init__(self, x1=0, y1=0, x2=100, y2=100, face_id=0):
        self.top_left = Point2(x=x1, y=y1)
        self.bottom_right = Point2(x=x2, y=y2)
        self.face_id = face_id
        self.diagonal = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class MockFacesMsg:
    """Minimal mock for Faces msg."""

    def __init__(self, faces=None):
        self.faces = faces if faces else []


class TestGetFaceArea:

    def setup_method(self):
        from behaviour.face_track.node.face_track_node import FaceTrackNode
        self.instance = FaceTrackNode()

    def test_small_face(self):
        face = MockFace(x1=0, y1=0, x2=50, y2=50)
        area = self.instance._FaceTrackNode__get_face_area(face)
        assert area == 2500.0

    def test_large_face(self):
        face = MockFace(x1=0, y1=0, x2=500, y2=500)
        area = self.instance._FaceTrackNode__get_face_area(face)
        assert area == 250000.0

    def test_zero_area_face(self):
        face = MockFace(x1=10, y1=10, x2=10, y2=10)
        area = self.instance._FaceTrackNode__get_face_area(face)
        assert area == 0.0

    def test_negative_coordinates(self):
        face = MockFace(x1=-50, y1=-50, x2=50, y2=50)
        area = self.instance._FaceTrackNode__get_face_area(face)
        assert area == 10000.0


class TestSelectFaceToTrack:

    def setup_method(self):
        from behaviour.face_track.node.face_track_node import FaceTrackNode
        self.instance = FaceTrackNode()

    def test_empty_list(self):
        result = self.instance._FaceTrackNode__select_face_to_track([])
        assert result is None

    def test_single_face(self):
        face = MockFace(x1=0, y1=0, x2=100, y2=100, face_id=1)
        result = self.instance._FaceTrackNode__select_face_to_track([face])
        assert result == face

    def test_largest_face_selected(self):
        small = MockFace(x1=0, y1=0, x2=50, y2=50, face_id=1)
        large = MockFace(x1=0, y1=0, x2=200, y2=200, face_id=2)
        result = self.instance._FaceTrackNode__select_face_to_track([small, large])
        assert result.face_id == 2


class TestPixelToAngle:

    def setup_method(self):
        from behaviour.face_track.node.face_track_node import FaceTrackNode
        self.instance = FaceTrackNode()

    def test_center_face_zero_angle(self):
        face = MockFace(x1=620, y1=460, x2=660, y2=500)
        errors = self._calculate_errors(face)
        assert abs(errors[0]) < 0.01
        assert abs(errors[1]) < 0.01

    def test_face_left_of_center(self):
        face = MockFace(x1=100, y1=460, x2=200, y2=500)
        errors = self._calculate_errors(face)
        assert errors[0] < 0

    def test_face_right_of_center(self):
        face = MockFace(x1=1000, y1=460, x2=1100, y2=500)
        errors = self._calculate_errors(face)
        assert errors[0] > 0

    def _calculate_errors(self, face):
        cx = face.top_left.x + (face.bottom_right.x - face.top_left.x) / 2
        cy = face.top_left.y + (face.bottom_right.y - face.top_left.y) / 2
        dx = cx - self.instance.middle_x
        dy = cy - self.instance.middle_y
        return dx * self.instance.angle_per_pixel, dy * self.instance.angle_per_pixel


class TestRoutingDecision:
    """Tests that face_list_callback routes errors correctly:
    - large horizontal error → head pan
    - small horizontal error → eyes
    - vertical error always → eyes
    - head pitch always 0 (no tilt axis)
    """

    def setup_method(self):
        from behaviour.face_track.node.face_track_node import FaceTrackNode
        self.instance = FaceTrackNode()
        self.instance.goal_publisher = MagicMock()
        self.instance.last_published_tracking_goal = 0.0

    def test_large_horizontal_triggers_head(self):
        self.instance.face_list_callback(self._make_faces(100, 480))
        goal = self.instance.goal_publisher.publish.call_args[0][0]
        assert goal.head_yaw_pan_target != 0.0
        assert goal.eye_shift_horizontal_target == 0.0

    def test_small_horizontal_goes_to_eyes(self):
        self.instance.face_list_callback(self._make_faces(630, 480))
        goal = self.instance.goal_publisher.publish.call_args[0][0]
        assert goal.head_yaw_pan_target == 0.0
        assert goal.eye_shift_horizontal_target != 0.0

    def test_vertical_always_goes_to_eyes(self):
        self.instance.face_list_callback(self._make_faces(640, 200))
        goal = self.instance.goal_publisher.publish.call_args[0][0]
        assert goal.eye_shift_vertical_target != 0.0

    def test_head_pitch_always_zero(self):
        self.instance.face_list_callback(self._make_faces(100, 100))
        goal = self.instance.goal_publisher.publish.call_args[0][0]
        assert goal.head_pitch_tilt_vertical_target == 0.0

    def test_no_faces_does_not_publish(self):
        self.instance.face_list_callback(MockFacesMsg([]))
        assert self.instance.goal_publisher.publish.call_count == 0

    def _make_faces(self, cx, cy, half_w=20, half_h=20):
        face = MockFace(
            x1=cx - half_w,
            y1=cy - half_h,
            x2=cx + half_w,
            y2=cy + half_h,
        )
        return MockFacesMsg([face])
