import pytest
import math
import random

from behaviour.gesture.node.head_gesture_node import HeadGestureNode


class TestClamp:

    def setup_method(self):
        self.node = HeadGestureNode()

    def test_within_limits(self):
        result = self.node._clamp(90.0, 0.0, 180.0)
        assert result == 90.0

    def test_below_min(self):
        result = self.node._clamp(-10.0, 0.0, 180.0)
        assert result == 0.0

    def test_above_max(self):
        result = self.node._clamp(200.0, 0.0, 180.0)
        assert result == 180.0

    def test_at_min(self):
        result = self.node._clamp(0.0, 0.0, 180.0)
        assert result == 0.0

    def test_at_max(self):
        result = self.node._clamp(180.0, 0.0, 180.0)
        assert result == 180.0


class TestRadToDeg:

    def setup_method(self):
        self.node = HeadGestureNode()

    def test_zero(self):
        assert self.node._rad_to_deg(0.0) == 0.0

    def test_pi(self):
        assert self.node._rad_to_deg(math.pi) == 180.0

    def test_half_pi(self):
        assert self.node._rad_to_deg(math.pi / 2) == 90.0

    def test_negative(self):
        assert self.node._rad_to_deg(-math.pi) == -180.0


class TestCameraAngleToDeg:

    def setup_method(self):
        self.node = HeadGestureNode()

    def test_zero_input(self):
        result = self.node._camera_angle_to_deg(0.0, -1.04387)
        assert result == 0.0

    def test_positive_input(self):
        result = self.node._camera_angle_to_deg(1.0, -1.04387)
        expected = -1.04387 * 180.0 / math.pi
        assert result == pytest.approx(expected)


class TestInitJointState:

    def setup_method(self):
        self.node = HeadGestureNode()

    def test_sets_defaults(self):
        self.node.head_pan_deg = None
        self.node.eye_h_deg = None
        self.node.eye_v_deg = None
        self.node.jaw_deg = None
        self.node._init_joint_state()
        assert self.node.head_pan_deg == self.node.HEAD_PAN_HOME_DEG
        assert self.node.eye_h_deg == self.node.EYE_H_HOME_DEG
        assert self.node.eye_v_deg == self.node.EYE_V_HOME_DEG
        assert self.node.jaw_deg == self.node.JAW_HOME_DEG

    def test_does_not_override_existing(self):
        self.node.head_pan_deg = 45.0
        self.node._init_joint_state()
        assert self.node.head_pan_deg == 45.0


class TestPriorityArbitration:

    def setup_method(self):
        self.node = HeadGestureNode()
        self.node.head_pan_deg = 90.0
        self.node.eye_h_deg = 90.0
        self.node.eye_v_deg = 90.0
        self.node.jaw_deg = 90.0

    def test_lower_priority_ignored(self):
        self.node.active_priority = self.node.PRIORITY_FACE_TRACK
        goal = MockGoal(priority=self.node.PRIORITY_IDLE, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_FACE_TRACK

    def test_higher_priority_accepted(self):
        self.node.active_priority = self.node.PRIORITY_IDLE
        goal = MockGoal(priority=self.node.PRIORITY_FACE_TRACK, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_IDLE

    def test_gesture_priority_blocks_lower(self):
        self.node.active_priority = self.node.PRIORITY_GESTURE
        goal = MockGoal(priority=self.node.PRIORITY_FACE_TRACK, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_GESTURE

    def test_no_active_priority_accepts_any(self):
        self.node.active_priority = self.node.NO_ACTIVE_PRIORITY
        goal = MockGoal(priority=self.node.PRIORITY_IDLE, head_yaw_pan_target=0.1)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority != self.node.NO_ACTIVE_PRIORITY


class TestMoveGoal:

    def setup_method(self):
        self.node = HeadGestureNode()
        self.node.head_pan_deg = 90.0
        self.node.eye_h_deg = 90.0
        self.node.eye_v_deg = 90.0
        self.node.jaw_deg = 90.0

    def test_head_pan_moves_by_delta(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            head_yaw_pan_target=1.0,
        )
        self.node.move_goal_callback(goal)
        expected_delta = 1.0 * (-1.04387) * 180.0 / math.pi
        assert self.node.head_pan_deg == pytest.approx(90.0 + expected_delta, abs=0.01)

    def test_eye_h_moves_by_delta(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            eye_shift_horizontal_target=0.5,
        )
        self.node.move_goal_callback(goal)
        expected_delta = 0.5 * (-2.67659) * 180.0 / math.pi
        assert self.node.eye_h_deg == pytest.approx(90.0 + expected_delta, abs=0.01)

    def test_eye_v_moves_by_delta(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            eye_shift_vertical_target=0.3,
        )
        self.node.move_goal_callback(goal)
        expected_delta = 0.3 * 4.01489 * 180.0 / math.pi
        assert self.node.eye_v_deg == pytest.approx(90.0 + expected_delta, abs=0.01)

    def test_head_pan_clamped_to_limits(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            head_yaw_pan_target=100.0,
        )
        self.node.move_goal_callback(goal)
        assert self.node.HEAD_PAN_MIN_DEG <= self.node.head_pan_deg <= self.node.HEAD_PAN_MAX_DEG

    def test_no_movement_sets_nothing(self):
        goal = MockGoal(priority=self.node.PRIORITY_FACE_TRACK)
        original_pan = self.node.head_pan_deg
        self.node.move_goal_callback(goal)
        assert self.node.head_pan_deg == original_pan

    def test_jaw_not_affected_by_face_goal(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            head_yaw_pan_target=0.5,
        )
        original_jaw = self.node.jaw_deg
        self.node.move_goal_callback(goal)
        assert self.node.jaw_deg == original_jaw


class TestJawCallback:

    def setup_method(self):
        self.node = HeadGestureNode()
        self.node.head_pan_deg = 90.0
        self.node.eye_h_deg = 90.0
        self.node.eye_v_deg = 90.0
        self.node.jaw_deg = 90.0
        self.node._jaw_text = ""
        self.node._jaw_index = 0

    def test_vowel_opens_jaw(self):
        self.node.jaw_callback(MockStringMsg("a"))
        self.node._jaw_timer_callback()
        assert self.node.jaw_deg > self.node.JAW_HOME_DEG

    def test_consonant_partial_open(self):
        self.node.jaw_callback(MockStringMsg("b"))
        self.node._jaw_timer_callback()
        assert self.node.jaw_deg > self.node.JAW_HOME_DEG and self.node.jaw_deg < self.node.JAW_MAX_DEG

    def test_processes_multiple_chars(self):
        self.node.jaw_callback(MockStringMsg("ab"))
        self.node._jaw_timer_callback()
        first_jaw = self.node.jaw_deg
        self.node._jaw_timer_callback()
        assert self.node.jaw_deg != first_jaw

    def test_empty_text_does_nothing(self):
        self.node.jaw_deg = 120.0
        self.node.jaw_callback(MockStringMsg(""))
        self.node._jaw_timer_callback()
        assert self.node.jaw_deg == 120.0

    def test_head_not_affected_by_jaw(self):
        original_pan = self.node.head_pan_deg
        self.node.jaw_callback(MockStringMsg("a"))
        self.node._jaw_timer_callback()
        assert self.node.head_pan_deg == original_pan


class MockGoal:

    def __init__(
        self,
        priority=1,
        head_yaw_pan_target=0.0,
        head_pitch_tilt_vertical_target=0.0,
        eye_shift_horizontal_target=0.0,
        eye_shift_vertical_target=0.0,
        duration_nanosecs=0,
    ):
        self.priority = priority
        self.head_yaw_pan_target = head_yaw_pan_target
        self.head_pitch_tilt_vertical_target = head_pitch_tilt_vertical_target
        self.eye_shift_horizontal_target = eye_shift_horizontal_target
        self.eye_shift_vertical_target = eye_shift_vertical_target
        self.duration_nanosecs = duration_nanosecs


class MockStringMsg:

    def __init__(self, data=""):
        self.data = data
