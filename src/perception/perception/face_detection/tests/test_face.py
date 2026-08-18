import math
import pytest

from perception.face_detection.core.face import Face


class DummyImage:
    shape = (480, 640, 3)


class TestFaceInit:
    """Tests for Face.__init__."""

    def test_diagonal_calculation(self):
        face = Face(left=0, right=100, top=0, bottom=100, image=DummyImage(), representation=[])
        assert face.diagonal == pytest.approx(math.sqrt(100**2 + 100**2))

    def test_diagonal_zero_size(self):
        face = Face(left=50, right=50, top=50, bottom=50, image=DummyImage(), representation=[])
        assert face.diagonal == 0.0

    def test_rect_creation(self):
        face = Face(left=10, right=200, top=30, bottom=240, image=DummyImage(), representation=[])
        assert face.rect.left() == 10
        assert face.rect.right() == 200
        assert face.rect.top() == 30
        assert face.rect.bottom() == 240

    def test_default_speaking_none(self):
        face = Face(left=0, right=50, top=0, bottom=50, image=DummyImage(), representation=[])
        assert face.speaking is None

    def test_default_tracker_none(self):
        face = Face(left=0, right=50, top=0, bottom=80, image=DummyImage(), representation=[])
        assert face.correlation_tracker is None


class TestFaceAsDict:
    """Tests for Face.as_dict."""

    def test_basic_conversion(self):
        face = Face(left=10, right=100, top=20, bottom=120, image=DummyImage(), representation=[])
        face.speaking = True
        result = face.as_dict()
        assert result["left"] == 10
        assert result["right"] == 100
        assert result["top"] == 20
        assert result["bottom"] == 120
        assert result["diagonal"] == pytest.approx(math.sqrt(90**2 + 100**2))
        assert result["speaking"] is True
        assert result["face_id"] == ""

    def test_with_cluster_dict(self):
        face = Face(
            left=0, right=50, top=0, bottom=80,
            image=DummyImage(), representation=[],
            cluster_dict={"id": 42, "conversations": [{"start": 1}]}
        )
        result = face.as_dict()
        assert result["face_id"] == 42
        assert result["previous_occurances"] == [{"start": 1}]

    def test_speaking_empty_when_none(self):
        face = Face(left=0, right=50, top=0, bottom=80, image=DummyImage(), representation=[])
        result = face.as_dict()
        assert result["speaking"] == ""


class TestFaceUpdateLocation:
    """Tests for Face.update_location."""

    def test_update_without_tracker_does_nothing(self):
        face = Face(left=0, right=50, top=0, bottom=80, image=DummyImage(), representation=[])
        face.update_location(None)
        assert face.left == 0 and face.right == 50
