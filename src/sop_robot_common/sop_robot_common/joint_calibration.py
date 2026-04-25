"""Shared head, eye, and hand calibration contracts."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from math import degrees, radians
from pathlib import Path
from typing import Any

import yaml

DISPLAY_CENTER_DEGREES = 90.0
FULL_CALIBRATION_LIMIT_DEGREES = (-180.0, 180.0)
FULL_CALIBRATION_LIMIT_RADIANS = tuple(radians(value) for value in FULL_CALIBRATION_LIMIT_DEGREES)


@dataclass(frozen=True)
class JointGroupSpec:
    key: str
    labels: tuple[str, ...]
    joint_names: tuple[str, ...]
    default_centers: tuple[float, ...]
    limits: tuple[tuple[float, float], ...]
    controller: str
    calibration_prefix: str
    set_prefix: str


HEAD_SPEC = JointGroupSpec(
    key="head",
    labels=(
        "Head pan",
        "Head tilt right",
        "Head tilt left",
        "Head tilt vertical",
    ),
    joint_names=(
        "head_pan_joint",
        "head_tilt_right_joint",
        "head_tilt_left_joint",
        "head_tilt_vertical_joint",
    ),
    default_centers=(0.6, 0.5, -0.5, 1.2),
    limits=((-0.75, 1.75), (-1.0, 1.0), (-1.0, 1.0), (0.8, 1.5)),
    controller="head_controller",
    calibration_prefix="head_calibration:",
    set_prefix="head_set:",
)

EYES_SPEC = JointGroupSpec(
    key="eyes",
    labels=("Eyes horizontal", "Eyes vertical"),
    joint_names=("eyes_shift_horizontal_joint", "eyes_shift_vertical_joint"),
    default_centers=(-0.7, -0.75),
    limits=((-2.0, 0.0), (-0.75, -0.2)),
    controller="eyes_controller",
    calibration_prefix="eyes_calibration:",
    set_prefix="eyes_set:",
)

RIGHT_HAND_CENTERS = (-0.3, -0.2, -0.2, 0.0, -0.3)
LEFT_HAND_CENTERS = (-2.35, 2.0, -0.07, -2.35, 0.19)

RIGHT_HAND_SPEC = JointGroupSpec(
    key="r_hand",
    labels=(
        "R thumb",
        "R index",
        "R middle",
        "R ring",
        "R pinky",
    ),
    joint_names=(
        "r_thumb_joint",
        "r_index1_joint",
        "r_middle1_joint",
        "r_ring_joint",
        "r_pinky_joint",
    ),
    default_centers=RIGHT_HAND_CENTERS,
    limits=((-0.3, 1.5), (-0.2, 1.7), (-0.2, 1.8), (0.0, 1.5), (-0.3, 1.3)),
    controller="r_hand_controller",
    calibration_prefix="hand_calibration:",
    set_prefix="hand_set:",
)

LEFT_HAND_SPEC = JointGroupSpec(
    key="l_hand",
    labels=(
        "L thumb",
        "L index",
        "L middle",
        "L ring",
        "L pinky",
    ),
    joint_names=(
        "l_thumb_joint",
        "l_index1_joint",
        "l_middle1_joint",
        "l_ring_joint",
        "l_pinky_joint",
    ),
    default_centers=LEFT_HAND_CENTERS,
    limits=((-2.35, -0.78), (0.45, 2.0), (-0.07, 2.0), (-2.35, -0.47), (-1.72, 0.19)),
    controller="l_hand_controller",
    calibration_prefix="hand_calibration:",
    set_prefix="hand_set:",
)

JOINT_GROUP_SPECS = {
    spec.key: spec
    for spec in (HEAD_SPEC, EYES_SPEC, RIGHT_HAND_SPEC, LEFT_HAND_SPEC)
}


def spec_with_limits(
    spec: JointGroupSpec,
    limits: Sequence[Sequence[float]],
) -> JointGroupSpec:
    """Return a copy of a joint-group spec with validated runtime limits."""
    return replace(spec, limits=normalize_joint_limits(spec, limits))


def max_limited_spec(spec: JointGroupSpec) -> JointGroupSpec:
    """Return a copy of a spec with each joint clamped only to -180..180 degrees."""
    return spec_with_limits(
        spec,
        [FULL_CALIBRATION_LIMIT_RADIANS for _joint_name in spec.joint_names],
    )


def load_joint_specs(
    path: str | Path | None,
    *,
    use_max_limits: bool = False,
) -> dict[str, JointGroupSpec]:
    """Load joint-group specs using saved limits, defaults, or full calibration limits."""
    if use_max_limits:
        return {key: max_limited_spec(spec) for key, spec in JOINT_GROUP_SPECS.items()}
    limits_by_key = load_joint_limits(path)
    return {
        key: spec_with_limits(spec, limits_by_key[key])
        for key, spec in JOINT_GROUP_SPECS.items()
    }


def load_joint_limits(path: str | Path | None) -> dict[str, tuple[tuple[float, float], ...]]:
    """Load per-group min/max limits from YAML, defaulting to the built-in limits."""
    data = _load_yaml(path)
    limits: dict[str, tuple[tuple[float, float], ...]] = {}
    for key, spec in JOINT_GROUP_SPECS.items():
        raw_limits = _limits_or_none(data.get(f"{key}_limits"))
        limits[key] = normalize_joint_limits(spec, raw_limits)
    return limits


def normalize_joint_positions(
    spec: JointGroupSpec,
    values: Sequence[float] | None,
) -> list[float]:
    """Return valid positions for a calibrated joint group."""
    positions = list(spec.default_centers)
    if values is None:
        return positions
    for index, value in enumerate(values[: len(positions)]):
        positions[index] = clamp_joint_position(spec, index, float(value))
    return positions


def clamp_joint_position(spec: JointGroupSpec, index: int, value: float) -> float:
    minimum, maximum = spec.limits[index]
    return min(max(value, minimum), maximum)


def normalize_joint_limits(
    spec: JointGroupSpec,
    values: Sequence[Sequence[float]] | None,
) -> tuple[tuple[float, float], ...]:
    limits = list(spec.limits)
    if values is None:
        return tuple(limits)

    absolute_minimum, absolute_maximum = FULL_CALIBRATION_LIMIT_RADIANS
    for index, value in enumerate(values[: len(limits)]):
        if len(value) != 2:
            continue
        minimum = min(max(float(value[0]), absolute_minimum), absolute_maximum)
        maximum = min(max(float(value[1]), absolute_minimum), absolute_maximum)
        if minimum > maximum:
            minimum, maximum = maximum, minimum
        limits[index] = (minimum, maximum)
    return tuple(limits)


def raw_to_display_degrees(spec: JointGroupSpec, index: int, value: float, center: float) -> float:
    return DISPLAY_CENTER_DEGREES + degrees(value - center)


def display_degrees_to_raw(
    spec: JointGroupSpec,
    index: int,
    display_degrees: float,
    center: float,
) -> float:
    return clamp_joint_position(
        spec,
        index,
        center + radians(display_degrees - DISPLAY_CENTER_DEGREES),
    )


def display_limits(
    spec: JointGroupSpec,
    centers: Sequence[float],
) -> tuple[tuple[float, float], ...]:
    limits: list[tuple[float, float]] = []
    for index, (minimum, maximum) in enumerate(spec.limits):
        first = raw_to_display_degrees(spec, index, minimum, centers[index])
        second = raw_to_display_degrees(spec, index, maximum, centers[index])
        limits.append((min(first, second), max(first, second)))
    return tuple(limits)


def build_set_command(spec: JointGroupSpec, positions: Sequence[float]) -> str:
    normalized = normalize_joint_positions(spec, positions)
    return spec.set_prefix + ",".join(f"{value:g}" for value in normalized)


def build_calibration_command(spec: JointGroupSpec, centers: Sequence[float]) -> str:
    normalized = normalize_joint_positions(spec, centers)
    return spec.calibration_prefix + ",".join(f"{value:g}" for value in normalized)


def parse_set_command(spec: JointGroupSpec, command: str) -> list[float] | None:
    return _parse_command(spec, command, spec.set_prefix)


def parse_calibration_command(spec: JointGroupSpec, command: str) -> list[float] | None:
    return _parse_command(spec, command, spec.calibration_prefix)


def _parse_command(spec: JointGroupSpec, command: str, prefix: str) -> list[float] | None:
    normalized = command.strip().lower()
    if not normalized.startswith(prefix):
        return None
    payload = normalized[len(prefix) :].strip()
    try:
        positions = [float(part.strip()) for part in payload.split(",")]
    except ValueError:
        return None
    if len(positions) != len(spec.joint_names):
        return None
    return normalize_joint_positions(spec, positions)


def load_joint_calibration(
    path: str | Path | None,
    specs: Mapping[str, JointGroupSpec] | None = None,
) -> dict[str, list[float]]:
    """Load all non-shoulder joint calibration centers from YAML."""
    data = _load_yaml(path)
    resolved_specs = specs or JOINT_GROUP_SPECS
    return {
        key: normalize_joint_positions(
            spec,
            _list_or_none(data.get(f"{key}_center_positions")),
        )
        for key, spec in resolved_specs.items()
    }


def save_joint_group_centers(
    path: str | Path,
    key: str,
    centers: Sequence[float],
    spec_override: JointGroupSpec | None = None,
) -> Path:
    """Update one non-shoulder joint group in the shared calibration YAML."""
    if key not in JOINT_GROUP_SPECS:
        raise ValueError(f"Unknown joint calibration group: {key}")
    spec = spec_override or JOINT_GROUP_SPECS[key]
    data = _load_yaml(path)
    normalized = normalize_joint_positions(spec, centers)
    data[f"{key}_center_positions"] = normalized
    data[f"{key}_joints"] = {
        label: value for label, value in zip(spec.labels, normalized, strict=True)
    }
    return _write_yaml(path, data)


def save_joint_group_limits(
    path: str | Path,
    key: str,
    limits: Sequence[Sequence[float]],
) -> Path:
    """Update one non-shoulder joint group's min/max limits in the calibration YAML."""
    if key not in JOINT_GROUP_SPECS:
        raise ValueError(f"Unknown joint calibration group: {key}")
    spec = JOINT_GROUP_SPECS[key]
    data = _load_yaml(path)
    normalized = normalize_joint_limits(spec, limits)
    data[f"{key}_limits"] = [list(limit) for limit in normalized]
    data[f"{key}_limit_degrees"] = {
        label: [degrees(minimum), degrees(maximum)]
        for label, (minimum, maximum) in zip(spec.labels, normalized, strict=True)
    }
    return _write_yaml(path, data)


def reset_joint_group_limits(path: str | Path, key: str) -> Path:
    """Restore one non-shoulder joint group's limits to the built-in defaults."""
    if key not in JOINT_GROUP_SPECS:
        raise ValueError(f"Unknown joint calibration group: {key}")
    return save_joint_group_limits(path, key, JOINT_GROUP_SPECS[key].limits)


def _load_yaml(path: str | Path | None) -> dict[str, Any]:
    if path is None or str(path).strip() == "":
        return {}
    calibration_path = Path(path).expanduser()
    if not calibration_path.is_file():
        return {}
    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


def _write_yaml(path: str | Path, data: dict[str, Any]) -> Path:
    calibration_path = Path(path).expanduser()
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    calibration_path.write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    return calibration_path


def _list_or_none(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)):
        return None
    return [float(item) for item in value]


def _limits_or_none(value: Any) -> list[list[float]] | None:
    if not isinstance(value, (list, tuple)):
        return None
    limits: list[list[float]] = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return None
        limits.append([float(item[0]), float(item[1])])
    return limits
