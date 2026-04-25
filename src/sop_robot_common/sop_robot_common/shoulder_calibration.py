"""Persistent shoulder center calibration helpers."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import yaml

from sop_robot_common.arm_actions import (
    FULL_SHOULDER_SERVO_LIMITS,
    ShoulderCalibration,
    SHOULDER_JOINT_LABELS,
    SHOULDER_SERVO_LIMITS,
    normalize_shoulder_centers,
    normalize_shoulder_limits,
)

DEFAULT_SHOULDER_CALIBRATION = {
    "shoulder_center_degrees": normalize_shoulder_centers(None),
    "shoulder_limits_degrees": [list(limit) for limit in SHOULDER_SERVO_LIMITS],
}


def load_shoulder_center_degrees(
    path: str | Path | None,
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> list[float]:
    """Load raw-servo shoulder center degrees from YAML, falling back to defaults."""
    if path is None or str(path).strip() == "":
        return normalize_shoulder_centers(None, calibrations)

    calibration_path = Path(path).expanduser()
    if not calibration_path.is_file():
        return normalize_shoulder_centers(None, calibrations)

    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return normalize_shoulder_centers(None, calibrations)

    raw_centers = data.get("shoulder_center_degrees")
    if not isinstance(raw_centers, (list, tuple)):
        return normalize_shoulder_centers(None, calibrations)
    return normalize_shoulder_centers([float(value) for value in raw_centers], calibrations)


def load_shoulder_limits_degrees(
    path: str | Path | None,
    *,
    use_max_limits: bool = False,
) -> tuple[tuple[float, float], ...]:
    """Load raw-servo shoulder degree limits, falling back to default or max limits."""
    if use_max_limits:
        return FULL_SHOULDER_SERVO_LIMITS

    data = _load_yaml(path)
    raw_limits = data.get("shoulder_limits_degrees")
    if not isinstance(raw_limits, (list, tuple)):
        return SHOULDER_SERVO_LIMITS
    return normalize_shoulder_limits(raw_limits)


def save_shoulder_center_degrees(
    path: str | Path,
    center_degrees: list[float],
    calibrations: Sequence[ShoulderCalibration] | None = None,
) -> Path:
    """Save raw-servo shoulder center degrees to YAML for future launches."""
    data = _load_yaml(path)
    centers = normalize_shoulder_centers(center_degrees, calibrations)
    data["shoulder_center_degrees"] = centers
    data["shoulder_joints"] = {
        label: center for label, center in zip(SHOULDER_JOINT_LABELS, centers, strict=True)
    }
    return _write_yaml(path, data)


def save_shoulder_limits_degrees(
    path: str | Path,
    limits: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> Path:
    """Save shoulder min/max raw-servo degree limits to YAML for future launches."""
    data = _load_yaml(path)
    normalized_limits = normalize_shoulder_limits(limits)
    data["shoulder_limits_degrees"] = [list(limit) for limit in normalized_limits]
    data["shoulder_limit_joints"] = {
        label: list(limit)
        for label, limit in zip(SHOULDER_JOINT_LABELS, normalized_limits, strict=True)
    }
    return _write_yaml(path, data)


def reset_shoulder_limits_degrees(path: str | Path) -> Path:
    """Restore shoulder limits to the checked-in defaults."""
    return save_shoulder_limits_degrees(path, list(SHOULDER_SERVO_LIMITS))


def _load_yaml(path: str | Path | None) -> dict:
    if path is None or str(path).strip() == "":
        return {}
    calibration_path = Path(path).expanduser()
    if not calibration_path.is_file():
        return {}
    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


def _write_yaml(path: str | Path, data: dict) -> Path:
    calibration_path = Path(path).expanduser()
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    calibration_path.write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    return calibration_path
