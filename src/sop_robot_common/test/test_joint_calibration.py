from math import isclose, radians

from sop_robot_common.joint_calibration import (
    HEAD_SPEC,
    load_joint_calibration,
    load_joint_specs,
    reset_joint_group_limits,
    save_joint_group_centers,
    save_joint_group_limits,
)


def test_joint_limits_round_trip_and_reset(tmp_path):
    path = tmp_path / "joint_calibration.yaml"
    limits = [(radians(-120.0), radians(120.0))] * len(HEAD_SPEC.joint_names)

    save_joint_group_limits(path, HEAD_SPEC.key, limits)

    spec = load_joint_specs(path)[HEAD_SPEC.key]
    assert all(isclose(limit[0], radians(-120.0)) for limit in spec.limits)
    assert all(isclose(limit[1], radians(120.0)) for limit in spec.limits)

    reset_joint_group_limits(path, HEAD_SPEC.key)
    spec = load_joint_specs(path)[HEAD_SPEC.key]
    assert spec.limits == HEAD_SPEC.limits


def test_max_limited_specs_allow_full_calibration_centers(tmp_path):
    path = tmp_path / "joint_calibration.yaml"
    spec = load_joint_specs(path, use_max_limits=True)[HEAD_SPEC.key]
    center = radians(-160.0)

    save_joint_group_centers(path, HEAD_SPEC.key, [center] * len(spec.joint_names), spec)

    spec = load_joint_specs(path, use_max_limits=True)[HEAD_SPEC.key]
    calibration = load_joint_calibration(path, {HEAD_SPEC.key: spec})
    assert spec.limits == tuple((radians(-180.0), radians(180.0)) for _ in spec.joint_names)
    assert calibration[HEAD_SPEC.key] == [center] * len(spec.joint_names)
