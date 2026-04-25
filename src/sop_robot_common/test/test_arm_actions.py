from math import isclose, radians

from sop_robot_common.arm_actions import (
    DEFAULT_SHOULDER_SERVO_IDS,
    FULL_SHOULDER_SERVO_LIMITS,
    SHOULDER_DISPLAY_LIMITS,
    SHOULDER_JOINT_NAMES,
    SHOULDER_SERVO_LIMITS,
    SHOULDER_POSITIONS,
    calibrations_with_centers,
    calibrations_with_centers_and_limits,
    parse_shoulder_calibration_command,
    parse_shoulder_joint_command,
    parse_shoulder_set_command,
    servo_degrees_to_joint_radians,
    servo_degrees_to_display_degrees,
    shoulder_calibration_command,
    shoulder_joint_command,
    shoulder_positions_for_action,
    shoulder_positions_to_joint_goal,
    shoulder_serial_command,
    shoulder_set_command,
    split_hand_command,
)
from sop_robot_common.shoulder_calibration import (
    load_shoulder_center_degrees,
    load_shoulder_limits_degrees,
    reset_shoulder_limits_degrees,
    save_shoulder_center_degrees,
    save_shoulder_limits_degrees,
)


def test_split_hand_command_accepts_known_hand_actions():
    assert split_hand_command("r_hand_open") == ("right", "open")
    assert split_hand_command("l_hand_fist") == ("left", "fist")
    assert split_hand_command("r_hand_missing") is None


def test_legacy_arm_action_expands_to_full_shoulder_pose():
    positions = shoulder_positions_for_action("hold")

    assert positions == [10.0, 70.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0]


def test_named_arm_poses_use_display_direction_offsets():
    assert shoulder_positions_for_action("rps_1") == [
        10.0,
        110.0,
        10.0,
        0.0,
        0.0,
        80.0,
        0.0,
        0.0,
    ]
    assert shoulder_positions_for_action("rps_2") == [
        25.0,
        70.0,
        10.0,
        0.0,
        0.0,
        80.0,
        0.0,
        0.0,
    ]


def test_side_specific_positions_use_noop_sentinel():
    assert shoulder_positions_for_action("zero", "right") == [
        30.0,
        90.0,
        10.0,
        0.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
    ]
    assert shoulder_positions_for_action("zero", "left") == [
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        34.0,
        80.0,
        10.0,
        0.0,
    ]


def test_servo_degrees_to_joint_radians_uses_calibration():
    assert isclose(servo_degrees_to_joint_radians(0, 30.0), radians(0.0))
    assert isclose(servo_degrees_to_joint_radians(0, 31.0), radians(-1.0))
    assert isclose(servo_degrees_to_joint_radians(1, 89.0), radians(1.0))
    assert isclose(servo_degrees_to_joint_radians(2, 11.0), radians(-1.0))
    assert isclose(servo_degrees_to_joint_radians(3, 1.0), radians(-1.0))
    assert isclose(servo_degrees_to_joint_radians(4, 35.0), radians(-1.0))
    assert isclose(servo_degrees_to_joint_radians(5, 81.0), radians(1.0))
    assert isclose(servo_degrees_to_joint_radians(6, 11.0), radians(-1.0))
    assert isclose(servo_degrees_to_joint_radians(7, 1.0), radians(1.0))


def test_zero_servo_positions_display_as_center_degrees():
    display_degrees = [
        servo_degrees_to_display_degrees(index, value)
        for index, value in enumerate(SHOULDER_POSITIONS["zero"])
    ]

    assert display_degrees == [90.0] * len(SHOULDER_JOINT_NAMES)
    assert SHOULDER_DISPLAY_LIMITS[0] == (40.0, 110.0)


def test_shoulder_goal_omits_noop_joints():
    joint_names, positions = shoulder_positions_to_joint_goal(
        [30.0, -1.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0]
    )

    assert joint_names == [SHOULDER_JOINT_NAMES[0], SHOULDER_JOINT_NAMES[3]]
    assert len(positions) == 2


def test_shoulder_serial_command_uses_requested_servo_ids():
    assert shoulder_serial_command([30.0, -1.0, 10.0], DEFAULT_SHOULDER_SERVO_IDS) == (
        "2:30,4:10"
    )


def test_shoulder_set_command_round_trips_and_clamps_values():
    command = shoulder_set_command([90.0] * len(SHOULDER_JOINT_NAMES))

    assert command == "shoulder_set:90,90,90,90,90,90,90,90"
    assert parse_shoulder_set_command(command) == SHOULDER_POSITIONS["zero"]


def test_shoulder_joint_command_accepts_index_label_and_ros_name():
    assert shoulder_joint_command(0, 999.0) == "shoulder_joint:0:110"
    assert parse_shoulder_joint_command("shoulder_joint:0:91") == (0, 29.0)
    assert parse_shoulder_joint_command("shoulder_joint:R_shoulder lift:91") == (
        0,
        29.0,
    )
    assert parse_shoulder_joint_command(
        "shoulder_joint:r_shoulder_lift_joint:91"
    ) == (0, 29.0)


def test_center_overrides_drive_zero_display_and_goal_conversion():
    centers = [40.0, 100.0, 20.0, 5.0, 44.0, 90.0, 20.0, 10.0]
    calibrations = calibrations_with_centers(centers)

    assert shoulder_positions_for_action("zero", center_degrees=centers) == centers
    assert servo_degrees_to_display_degrees(0, 40.0, calibrations) == 90.0
    assert parse_shoulder_joint_command("shoulder_joint:0:91", calibrations) == (
        0,
        39.0,
    )
    _names, positions = shoulder_positions_to_joint_goal(centers, calibrations)
    assert all(isclose(position, radians(0.0)) for position in positions)


def test_shoulder_calibration_command_round_trips_centers():
    command = shoulder_calibration_command([40.0, 100.0, 20.0, 5.0, 44.0, 90.0, 20.0, 10.0])

    assert command == "shoulder_calibration:40,100,20,5,44,90,20,10"
    assert parse_shoulder_calibration_command(command) == [
        40.0,
        100.0,
        20.0,
        5.0,
        44.0,
        90.0,
        20.0,
        10.0,
    ]


def test_shoulder_calibration_yaml_round_trips(tmp_path):
    path = tmp_path / "shoulder_calibration.yaml"
    centers = [40.0, 100.0, 20.0, 5.0, 44.0, 90.0, 20.0, 10.0]

    save_shoulder_center_degrees(path, centers)

    assert load_shoulder_center_degrees(path) == centers


def test_shoulder_limit_yaml_round_trips_and_resets(tmp_path):
    path = tmp_path / "shoulder_calibration.yaml"
    limits = [(-20.0, 100.0)] * len(SHOULDER_JOINT_NAMES)

    save_shoulder_limits_degrees(path, limits)

    assert load_shoulder_limits_degrees(path) == tuple(limits)
    reset_shoulder_limits_degrees(path)
    assert load_shoulder_limits_degrees(path) == SHOULDER_SERVO_LIMITS


def test_full_shoulder_limits_allow_negative_calibration_centers(tmp_path):
    path = tmp_path / "shoulder_calibration.yaml"
    calibrations = calibrations_with_centers_and_limits(None, FULL_SHOULDER_SERVO_LIMITS)
    centers = [-20.0] * len(SHOULDER_JOINT_NAMES)

    save_shoulder_center_degrees(path, centers, calibrations)

    assert load_shoulder_center_degrees(path, calibrations) == centers
