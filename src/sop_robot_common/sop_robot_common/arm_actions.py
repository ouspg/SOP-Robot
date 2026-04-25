"""Shared arm and shoulder command contracts."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import radians

SHOULDER_JOINT_LABELS = (
    "R_shoulder lift",
    "R_upper arm roll",
    "R_bicep",
    "R_shoulder out",
    "L_shoulder lift",
    "L_upper arm roll",
    "L_bicep",
    "L_shoulder out",
)

SHOULDER_JOINT_NAMES = (
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_shoulder_out_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_shoulder_out_joint",
)

DEFAULT_SHOULDER_SERVO_IDS = (2, 3, 4, 5, 6, 7, 8, 9)
SHOULDER_DISPLAY_CENTER_DEGREES = 90.0

HAND_ACTIONS = (
    "open",
    "fist",
    "scissors",
    "point",
    "thumbs_up",
    "grasp",
    "pen_grasp",
    "hard_rock",
    "rps",
    "funny",
    "three",
)

SHOULDER_POSITIONS = {
    "zero": [30.0, 90.0, 10.0, 0.0, 34.0, 80.0, 10.0, 0.0],
    "rps_1": [55.0, 110.0, 10.0, 0.0, 79.0, 80.0, 45.0, 0.0],
    "rps_2": [35.0, 70.0, 10.0, 0.0, 79.0, 80.0, 75.0, 0.0],
}

LEGACY_ARM_POSITIONS = {
    "pos": [45.0, 120.0],
    "zer": [30.0, 90.0],
    "hold": [70.0, 70.0],
}

SHOULDER_ACTION_DISPLAY_OFFSETS = {
    name: [
        position - neutral
        for position, neutral in zip(positions, SHOULDER_POSITIONS["zero"], strict=True)
    ]
    for name, positions in SHOULDER_POSITIONS.items()
}

LEGACY_ARM_DISPLAY_OFFSETS = {
    name: [
        position - neutral
        for position, neutral in zip(positions, SHOULDER_POSITIONS["zero"], strict=False)
    ]
    for name, positions in LEGACY_ARM_POSITIONS.items()
}

ActionStep = tuple[str, float, str, str]

ACTION_PATTERNS: dict[str, tuple[ActionStep, ...]] = {
    "zero": (
        ("hand", 1.0, "fist", "left"),
        ("hand", 1.0, "fist", "right"),
        ("arm", 0.0, "zero", "both"),
    ),
    "test": (
        ("hand", 1.0, "fist", "left"),
        ("hand", 1.0, "fist", "right"),
        ("arm", 1.0, "zero", "both"),
        ("hand", 1.0, "open", "left"),
        ("hand", 1.0, "open", "right"),
        ("hand", 1.0, "fist", "left"),
        ("hand", 0.0, "fist", "right"),
    ),
    "wave": (
        ("hand", 0.5, "fist", "left"),
        ("arm", 0.5, "rps_1", "both"),
        ("arm", 0.5, "rps_2", "both"),
        ("arm", 0.5, "rps_1", "both"),
        ("arm", 0.5, "rps_2", "both"),
        ("arm", 0.5, "rps_1", "both"),
        ("arm", 0.5, "zero", "both"),
        ("hand", 0.0, "open", "left"),
    ),
    "rock": (
        ("hand", 1.0, "hard_rock", "left"),
        ("arm", 1.0, "rps_1", "both"),
        ("arm", 1.0, "rps_2", "both"),
        ("arm", 1.0, "rps_1", "both"),
        ("arm", 1.0, "rps_2", "both"),
        ("arm", 1.0, "rps_1", "both"),
        ("arm", 1.0, "rps_2", "both"),
        ("arm", 1.0, "zero", "both"),
        ("hand", 0.0, "open", "left"),
    ),
}

PATTERN_COMMANDS = tuple(ACTION_PATTERNS)
HAND_COMMANDS = tuple(f"{side}_hand_{action}" for side in ("r", "l") for action in HAND_ACTIONS)
ARM_POSE_COMMANDS = tuple(SHOULDER_POSITIONS) + tuple(LEGACY_ARM_POSITIONS)
AVAILABLE_ARM_COMMANDS = PATTERN_COMMANDS + HAND_COMMANDS + ARM_POSE_COMMANDS


@dataclass(frozen=True)
class ShoulderCalibration:
    servo_min: float
    servo_max: float
    neutral: float
    display_degrees_per_servo_degree: float = 1.0
    rviz_degrees_per_display_degree: float = 1.0


SHOULDER_CALIBRATIONS = (
    ShoulderCalibration(10.0, 80.0, 30.0, -1.0, 1.0),
    ShoulderCalibration(10.0, 180.0, 90.0, 1.0, -1.0),
    ShoulderCalibration(0.0, 100.0, 10.0, -1.0, 1.0),
    ShoulderCalibration(0.0, 60.0, 0.0, 1.0, -1.0),
    ShoulderCalibration(0.0, 180.0, 34.0, -1.0, 1.0),
    ShoulderCalibration(0.0, 180.0, 80.0, 1.0, 1.0),
    ShoulderCalibration(0.0, 100.0, 10.0, -1.0, 1.0),
    ShoulderCalibration(0.0, 115.0, 0.0),
)

SHOULDER_SERVO_LIMITS = tuple(
    (calibration.servo_min, calibration.servo_max)
    for calibration in SHOULDER_CALIBRATIONS
)
FULL_SHOULDER_SERVO_LIMITS = tuple(
    (-180.0, 180.0) for _calibration in SHOULDER_CALIBRATIONS
)
SHOULDER_SET_COMMAND_PREFIX = "shoulder_set:"
SHOULDER_JOINT_COMMAND_PREFIX = "shoulder_joint:"
SHOULDER_CALIBRATION_COMMAND_PREFIX = "shoulder_calibration:"


def split_hand_command(command: str) -> tuple[str, str] | None:
    """Return `(hand, gesture)` for commands like `r_hand_open`."""
    normalized = command.strip().lower()
    if normalized.startswith("r_hand_"):
        gesture = normalized[7:]
        return ("right", gesture) if gesture in HAND_ACTIONS else None
    if normalized.startswith("l_hand_"):
        gesture = normalized[7:]
        return ("left", gesture) if gesture in HAND_ACTIONS else None
    return None


def shoulder_positions_for_action(
    action: str,
    side: str = "both",
    center_degrees: Sequence[float] | None = None,
    limits: Sequence[Sequence[float]] | None = None,
) -> list[float] | None:
    """Resolve an arm action into eight shoulder servo degrees.

    `-1.0` is used as the serial no-op sentinel for joints outside a requested
    side. Fake ros2_control goals simply omit those joints.
    """
    normalized = action.strip().lower()
    calibrations = calibrations_with_centers_and_limits(center_degrees, limits)
    centers = [calibration.neutral for calibration in calibrations]
    if normalized == "zero":
        positions = list(centers)
    elif normalized in SHOULDER_ACTION_DISPLAY_OFFSETS:
        positions = _display_offsets_to_servo_degrees(
            SHOULDER_ACTION_DISPLAY_OFFSETS[normalized],
            calibrations,
        )
    elif normalized == "zer":
        positions = list(centers)
    elif normalized in LEGACY_ARM_DISPLAY_OFFSETS:
        offsets = [0.0] * len(SHOULDER_JOINT_NAMES)
        for index, value in enumerate(LEGACY_ARM_DISPLAY_OFFSETS[normalized]):
            offsets[index] = value
        positions = _display_offsets_to_servo_degrees(offsets, calibrations)
    else:
        return None

    if side == "both":
        return positions
    if side == "right":
        return positions[:4] + [-1.0, -1.0, -1.0, -1.0]
    if side == "left":
        return [-1.0, -1.0, -1.0, -1.0] + positions[4:]
    return None


def _display_offsets_to_servo_degrees(
    display_offsets: Sequence[float],
    calibrations: Sequence[ShoulderCalibration],
) -> list[float]:
    """Convert named pose offsets through the same direction map as the UI sliders."""
    return [
        display_degrees_to_servo_degrees(
            index,
            SHOULDER_DISPLAY_CENTER_DEGREES + float(offset),
            calibrations,
        )
        for index, offset in enumerate(display_offsets[: len(SHOULDER_JOINT_NAMES)])
    ]


def normalize_shoulder_limits(
    limits: Sequence[Sequence[float]] | None,
    *,
    fallback: Sequence[tuple[float, float]] = SHOULDER_SERVO_LIMITS,
) -> tuple[tuple[float, float], ...]:
    normalized = [(float(limit[0]), float(limit[1])) for limit in fallback]
    if limits is None:
        return tuple(normalized)

    for index, limit in enumerate(limits[: len(normalized)]):
        if len(limit) != 2:
            continue
        minimum = min(max(float(limit[0]), -180.0), 180.0)
        maximum = min(max(float(limit[1]), -180.0), 180.0)
        if minimum > maximum:
            minimum, maximum = maximum, minimum
        normalized[index] = (minimum, maximum)
    return tuple(normalized)


def normalize_shoulder_centers(
    center_degrees: Sequence[float] | None,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> list[float]:
    """Return one valid raw-servo center value per shoulder joint."""
    resolved_calibrations = calibrations or SHOULDER_CALIBRATIONS
    centers = [calibration.neutral for calibration in resolved_calibrations]
    if center_degrees is None:
        return centers

    for index, value in enumerate(center_degrees[: len(centers)]):
        centers[index] = clamp_shoulder_servo_degrees(
            index,
            float(value),
            resolved_calibrations,
        )
    return centers


def calibrations_with_centers(
    center_degrees: Sequence[float] | None = None,
) -> tuple[ShoulderCalibration, ...]:
    """Return shoulder calibrations with launch/user center overrides applied."""
    return calibrations_with_centers_and_limits(center_degrees, SHOULDER_SERVO_LIMITS)


def calibrations_with_centers_and_limits(
    center_degrees: Sequence[float] | None = None,
    limits: Sequence[Sequence[float]] | None = None,
) -> tuple[ShoulderCalibration, ...]:
    """Return shoulder calibrations with launch/user center and limit overrides."""
    normalized_limits = normalize_shoulder_limits(limits)
    limit_calibrations = tuple(
        ShoulderCalibration(
            normalized_limits[index][0],
            normalized_limits[index][1],
            calibration.neutral,
            calibration.display_degrees_per_servo_degree,
            calibration.rviz_degrees_per_display_degree,
        )
        for index, calibration in enumerate(SHOULDER_CALIBRATIONS)
    )
    centers = normalize_shoulder_centers(center_degrees, limit_calibrations)
    return tuple(
        ShoulderCalibration(
            limit_calibrations[index].servo_min,
            limit_calibrations[index].servo_max,
            centers[index],
            calibration.display_degrees_per_servo_degree,
            calibration.rviz_degrees_per_display_degree,
        )
        for index, calibration in enumerate(SHOULDER_CALIBRATIONS)
    )


def _calibration_at(
    index: int,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> ShoulderCalibration:
    return (calibrations or SHOULDER_CALIBRATIONS)[index]


def clamp_shoulder_servo_degrees(
    index: int,
    servo_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> float:
    """Clamp a shoulder servo degree value to its configured real-servo range."""
    calibration = _calibration_at(index, calibrations)
    return min(max(servo_degrees, calibration.servo_min), calibration.servo_max)


def servo_degrees_to_display_degrees(
    index: int,
    servo_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> float:
    """Convert raw servo degrees to UI/RViz degrees where 90 is calibrated center."""
    calibration = _calibration_at(index, calibrations)
    constrained = clamp_shoulder_servo_degrees(index, servo_degrees, calibrations)
    return SHOULDER_DISPLAY_CENTER_DEGREES + (
        constrained - calibration.neutral
    ) * calibration.display_degrees_per_servo_degree


def shoulder_display_limits(
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> tuple[tuple[float, float], ...]:
    """Return per-joint UI/RViz display ranges for a calibration set."""
    resolved_calibrations = calibrations or SHOULDER_CALIBRATIONS
    limits: list[tuple[float, float]] = []
    for index, calibration in enumerate(resolved_calibrations):
        first = servo_degrees_to_display_degrees(
            index,
            calibration.servo_min,
            resolved_calibrations,
        )
        second = servo_degrees_to_display_degrees(
            index,
            calibration.servo_max,
            resolved_calibrations,
        )
        limits.append((min(first, second), max(first, second)))
    return tuple(limits)


SHOULDER_DISPLAY_LIMITS = shoulder_display_limits()


def clamp_shoulder_display_degrees(
    index: int,
    display_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> float:
    """Clamp a UI/RViz shoulder degree value to the converted real-servo range."""
    minimum, maximum = shoulder_display_limits(calibrations)[index]
    return min(max(display_degrees, minimum), maximum)


def display_degrees_to_servo_degrees(
    index: int,
    display_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> float:
    """Convert UI/RViz degrees where 90 is center back to raw servo degrees."""
    calibration = _calibration_at(index, calibrations)
    constrained = clamp_shoulder_display_degrees(index, display_degrees, calibrations)
    servo_degrees = calibration.neutral + (
        constrained - SHOULDER_DISPLAY_CENTER_DEGREES
    ) / calibration.display_degrees_per_servo_degree
    return clamp_shoulder_servo_degrees(index, servo_degrees, calibrations)


def servo_degrees_to_joint_radians(
    index: int,
    servo_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> float:
    """Map an Arduino servo degree value to the corresponding URDF joint angle."""
    calibration = _calibration_at(index, calibrations)
    display_offset_degrees = (
        servo_degrees_to_display_degrees(index, servo_degrees, calibrations)
        - SHOULDER_DISPLAY_CENTER_DEGREES
    )
    urdf_degrees = display_offset_degrees * calibration.rviz_degrees_per_display_degree
    return radians(urdf_degrees)


def shoulder_set_command(
    display_degrees: Sequence[float],
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> str:
    """Build a full eight-joint display-degree command for `/arms/arm_action`."""
    positions = [
        clamp_shoulder_display_degrees(index, float(value), calibrations)
        for index, value in enumerate(display_degrees[: len(SHOULDER_JOINT_NAMES)])
    ]
    if len(positions) != len(SHOULDER_JOINT_NAMES):
        raise ValueError("shoulder_set requires one degree value per shoulder joint")
    return SHOULDER_SET_COMMAND_PREFIX + ",".join(f"{value:g}" for value in positions)


def parse_shoulder_set_command(
    command: str,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> list[float] | None:
    """Parse a full display-degree shoulder command into raw servo degrees."""
    normalized = command.strip().lower()
    if not normalized.startswith(SHOULDER_SET_COMMAND_PREFIX):
        return None

    payload = normalized[len(SHOULDER_SET_COMMAND_PREFIX) :].strip()
    try:
        positions = [float(part.strip()) for part in payload.split(",")]
    except ValueError:
        return None

    if len(positions) != len(SHOULDER_JOINT_NAMES):
        return None
    return [
        display_degrees_to_servo_degrees(index, value, calibrations)
        for index, value in enumerate(positions)
    ]


def shoulder_joint_command(
    index: int,
    display_degrees: float,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> str:
    """Build a single-joint display-degree command for `/arms/arm_action`."""
    return (
        f"{SHOULDER_JOINT_COMMAND_PREFIX}{index}:"
        f"{clamp_shoulder_display_degrees(index, display_degrees, calibrations):g}"
    )


def parse_shoulder_joint_command(
    command: str,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> tuple[int, float] | None:
    """Parse a single-joint display-degree shoulder command into raw servo degrees."""
    normalized = command.strip().lower()
    if not normalized.startswith(SHOULDER_JOINT_COMMAND_PREFIX):
        return None

    payload = normalized[len(SHOULDER_JOINT_COMMAND_PREFIX) :].strip()
    parts = payload.split(":", maxsplit=1)
    if len(parts) != 2:
        return None

    index = shoulder_joint_index(parts[0])
    if index is None:
        return None
    try:
        value = float(parts[1].strip())
    except ValueError:
        return None
    return index, display_degrees_to_servo_degrees(index, value, calibrations)


def shoulder_calibration_command(
    center_degrees: Sequence[float],
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> str:
    """Build a raw-servo center calibration update command."""
    centers = normalize_shoulder_centers(center_degrees, calibrations)
    return SHOULDER_CALIBRATION_COMMAND_PREFIX + ",".join(
        f"{value:g}" for value in centers
    )


def parse_shoulder_calibration_command(
    command: str,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> list[float] | None:
    """Parse a raw-servo center calibration update command."""
    normalized = command.strip().lower()
    if not normalized.startswith(SHOULDER_CALIBRATION_COMMAND_PREFIX):
        return None

    payload = normalized[len(SHOULDER_CALIBRATION_COMMAND_PREFIX) :].strip()
    try:
        centers = [float(part.strip()) for part in payload.split(",")]
    except ValueError:
        return None
    if len(centers) != len(SHOULDER_JOINT_NAMES):
        return None
    return normalize_shoulder_centers(centers, calibrations)


def shoulder_joint_index(token: str) -> int | None:
    """Resolve a shoulder joint index from an index, UI label, or ROS joint name."""
    normalized = token.strip().lower()
    try:
        index = int(normalized)
    except ValueError:
        index = -1
    if 0 <= index < len(SHOULDER_JOINT_NAMES):
        return index

    normalized_key = normalized.replace(" ", "_")
    for index, (label, name) in enumerate(
        zip(SHOULDER_JOINT_LABELS, SHOULDER_JOINT_NAMES, strict=True)
    ):
        if normalized == label.lower() or normalized == name.lower():
            return index
        if normalized_key == label.lower().replace(" ", "_"):
            return index
    return None


def shoulder_positions_to_joint_goal(
    servo_degrees: list[float],
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> tuple[list[str], list[float]]:
    """Convert shoulder servo degrees to a partial FollowJointTrajectory goal."""
    joint_names: list[str] = []
    joint_positions: list[float] = []
    for index, servo_degrees_value in enumerate(servo_degrees[: len(SHOULDER_JOINT_NAMES)]):
        if (
            servo_degrees_value < 0.0
            and _calibration_at(index, calibrations).servo_min >= 0.0
        ):
            continue
        joint_names.append(SHOULDER_JOINT_NAMES[index])
        joint_positions.append(
            servo_degrees_to_joint_radians(index, servo_degrees_value, calibrations)
        )
    return joint_names, joint_positions


def shoulder_serial_command(
    servo_degrees: list[float],
    servo_ids: list[int] | tuple[int, ...] = DEFAULT_SHOULDER_SERVO_IDS,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> str:
    """Build the Arduino shoulder command string from servo IDs and degrees."""
    pairs: list[str] = []
    for index, (servo_id, value) in enumerate(zip(servo_ids, servo_degrees, strict=False)):
        if value < 0.0 and _calibration_at(index, calibrations).servo_min >= 0.0:
            continue
        pairs.append(f"{int(servo_id)}:{value:g}")
    return ",".join(pairs)
