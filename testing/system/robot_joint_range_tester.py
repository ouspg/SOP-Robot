#!/usr/bin/env python3
"""Center-check calibrated robot joints, sweep their ranges, and return to center."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Callable

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sop_robot_common.arm_actions import (
    SHOULDER_JOINT_NAMES,
    calibrations_with_centers_and_limits,
    shoulder_positions_to_joint_goal,
)
from sop_robot_common.joint_calibration import (
    EYES_SPEC,
    HEAD_SPEC,
    LEFT_HAND_SPEC,
    RIGHT_HAND_SPEC,
    JointGroupSpec,
    load_joint_calibration,
    load_joint_specs,
)
from sop_robot_common.shoulder_calibration import (
    load_shoulder_center_degrees,
    load_shoulder_limits_degrees,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION_PATH = PROJECT_ROOT / "config" / "shoulder_calibration.yaml"


@dataclass(frozen=True)
class JointGroupPlan:
    key: str
    label: str
    controller: str
    joint_names: tuple[str, ...]
    center_positions: tuple[float, ...]
    min_positions: tuple[float, ...]
    max_positions: tuple[float, ...]


class JointRangeTester(Node):
    def __init__(
        self,
        plans: list[JointGroupPlan],
        duration_seconds: float,
        settle_seconds: float,
        tolerance: float,
        state_timeout_seconds: float,
        dry_run: bool,
    ) -> None:
        super().__init__("robot_joint_range_tester")
        self._plans = plans
        self._duration_seconds = duration_seconds
        self._settle_seconds = settle_seconds
        self._tolerance = tolerance
        self._state_timeout_seconds = state_timeout_seconds
        self._dry_run = dry_run
        self._states: dict[str, dict[str, float]] = {}
        self._state_sources: dict[str, str] = {}
        self._action_clients = {
            plan.key: ActionClient(
                self,
                FollowJointTrajectory,
                f"/{plan.controller}/follow_joint_trajectory",
            )
            for plan in plans
        }
        for plan in plans:
            for topic_suffix in ("controller_state", "state"):
                self.create_subscription(
                    JointTrajectoryControllerState,
                    f"/{plan.controller}/{topic_suffix}",
                    self._state_callback(plan),
                    10,
                )

    def run(self) -> bool:
        success = True
        for plan in self._plans:
            print(f"\n== {plan.label}: center check")
            if not self._send_goal(plan, plan.center_positions):
                success = False
                continue
            if not self._wait_until_close(plan, plan.center_positions):
                success = False
                continue

            print(f"== {plan.label}: range sweep")
            for index, joint_name in enumerate(plan.joint_names):
                for label, positions in (
                    ("min", plan.min_positions),
                    ("max", plan.max_positions),
                ):
                    target = list(plan.center_positions)
                    target[index] = positions[index]
                    print(f"  {joint_name}: {label}")
                    if not self._send_goal(plan, tuple(target)):
                        success = False
                        break
                    time.sleep(self._settle_seconds)
                if not self._send_goal(plan, plan.center_positions):
                    success = False
                    break
                time.sleep(self._settle_seconds)

            print(f"== {plan.label}: final center")
            if not self._send_goal(plan, plan.center_positions):
                success = False
            elif not self._wait_until_close(plan, plan.center_positions):
                success = False

        return success

    def _send_goal(self, plan: JointGroupPlan, positions: tuple[float, ...]) -> bool:
        if self._dry_run:
            print(f"  dry-run {plan.controller}: {list(positions)}")
            return True

        client = self._action_clients[plan.key]
        if not client.wait_for_server(timeout_sec=5.0):
            print(f"  FAIL: action server /{plan.controller}/follow_joint_trajectory not ready")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=list(plan.joint_names),
            points=[
                JointTrajectoryPoint(
                    positions=list(positions),
                    time_from_start=_duration_from_seconds(self._duration_seconds),
                )
            ],
        )
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f"  FAIL: goal rejected by {plan.controller}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=max(5.0, self._duration_seconds + 3.0),
        )
        if result_future.result() is None:
            print(f"  FAIL: timed out waiting for {plan.controller} result")
            return False
        return True

    def _wait_until_close(
        self,
        plan: JointGroupPlan,
        positions: tuple[float, ...],
    ) -> bool:
        if self._dry_run:
            return True
        deadline = time.monotonic() + self._state_timeout_seconds
        last_errors: list[tuple[str, float, float, float]] = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self._states.get(plan.key)
            if state is None:
                continue
            last_errors = []
            for joint_name, target in zip(plan.joint_names, positions, strict=True):
                actual = state.get(joint_name, float("nan"))
                error = abs(actual - target) if math.isfinite(actual) else float("inf")
                last_errors.append((joint_name, actual, target, error))
            max_error = max(error for _name, _actual, _target, error in last_errors)
            if last_errors and max_error <= self._tolerance:
                source = self._state_sources.get(plan.key, "state")
                print(f"  PASS: center within {max_error:.4f} from {source}")
                return True
        self._print_center_failure(plan, last_errors)
        return False

    def _on_controller_state(
        self,
        plan: JointGroupPlan,
        msg: JointTrajectoryControllerState,
    ) -> None:
        joint_names = list(msg.joint_names) or list(plan.joint_names)
        positions, source = _controller_positions(msg)
        if not positions:
            return
        self._states[plan.key] = {
            name: position
            for name, position in zip(joint_names, positions, strict=False)
        }
        self._state_sources[plan.key] = source

    def _state_callback(
        self,
        plan: JointGroupPlan,
    ) -> Callable[[JointTrajectoryControllerState], None]:
        def callback(msg: JointTrajectoryControllerState) -> None:
            self._on_controller_state(plan, msg)

        return callback

    def _print_center_failure(
        self,
        plan: JointGroupPlan,
        errors: list[tuple[str, float, float, float]],
    ) -> None:
        if not errors:
            print(f"  FAIL: {plan.controller} did not publish readable controller state")
            return
        source = self._state_sources.get(plan.key, "state")
        max_error = max(error for _name, _actual, _target, error in errors)
        print(
            f"  FAIL: {plan.controller} did not report the expected center "
            f"(source={source}, max_error={max_error:.4f}, tolerance={self._tolerance:.4f})"
        )
        for joint_name, actual, target, error in errors:
            print(
                f"    {joint_name}: actual={actual:.4f}, "
                f"target={target:.4f}, error={error:.4f}"
            )


def build_plans(calibration_path: Path, requested_groups: set[str]) -> list[JointGroupPlan]:
    plans_by_key: dict[str, JointGroupPlan] = {}

    shoulder_limits = load_shoulder_limits_degrees(calibration_path)
    default_shoulder_calibrations = calibrations_with_centers_and_limits(
        None,
        shoulder_limits,
    )
    shoulder_centers = load_shoulder_center_degrees(
        calibration_path,
        default_shoulder_calibrations,
    )
    shoulder_calibrations = calibrations_with_centers_and_limits(
        shoulder_centers,
        shoulder_limits,
    )
    _, shoulder_center_positions = shoulder_positions_to_joint_goal(
        shoulder_centers,
        shoulder_calibrations,
    )
    _, shoulder_min_positions = shoulder_positions_to_joint_goal(
        [calibration.servo_min for calibration in shoulder_calibrations],
        shoulder_calibrations,
    )
    _, shoulder_max_positions = shoulder_positions_to_joint_goal(
        [calibration.servo_max for calibration in shoulder_calibrations],
        shoulder_calibrations,
    )
    plans_by_key["shoulders"] = JointGroupPlan(
        key="shoulders",
        label="shoulders",
        controller="shoulder_controller",
        joint_names=tuple(SHOULDER_JOINT_NAMES),
        center_positions=tuple(shoulder_center_positions),
        min_positions=tuple(shoulder_min_positions),
        max_positions=tuple(shoulder_max_positions),
    )

    joint_specs = load_joint_specs(calibration_path)
    joint_calibration = load_joint_calibration(calibration_path, joint_specs)
    for spec in (
        joint_specs[HEAD_SPEC.key],
        joint_specs[EYES_SPEC.key],
        joint_specs[RIGHT_HAND_SPEC.key],
        joint_specs[LEFT_HAND_SPEC.key],
    ):
        plans_by_key[spec.key] = _plan_from_spec(spec, joint_calibration[spec.key])

    if "all" in requested_groups:
        return [
            plans_by_key["shoulders"],
            plans_by_key[HEAD_SPEC.key],
            plans_by_key[EYES_SPEC.key],
            plans_by_key[RIGHT_HAND_SPEC.key],
            plans_by_key[LEFT_HAND_SPEC.key],
        ]
    return [plans_by_key[group] for group in requested_groups]


def _plan_from_spec(spec: JointGroupSpec, centers: list[float]) -> JointGroupPlan:
    return JointGroupPlan(
        key=spec.key,
        label=spec.key,
        controller=spec.controller,
        joint_names=spec.joint_names,
        center_positions=tuple(centers),
        min_positions=tuple(limit[0] for limit in spec.limits),
        max_positions=tuple(limit[1] for limit in spec.limits),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibration-path",
        type=Path,
        default=DEFAULT_CALIBRATION_PATH,
        help="Shared joint calibration YAML path.",
    )
    parser.add_argument(
        "--groups",
        default="all",
        help="Comma-separated groups: all,shoulders,head,eyes,r_hand,l_hand.",
    )
    parser.add_argument("--duration", type=float, default=0.8, help="Seconds per goal.")
    parser.add_argument("--settle", type=float, default=0.2, help="Pause after range goals.")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.03,
        help="Controller-state tolerance for center checks.",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=3.0,
        help="Seconds to wait for controller-state center confirmation.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned commands without sending action goals.",
    )
    return parser.parse_args()


def _duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1_000_000_000)
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def _controller_positions(msg: JointTrajectoryControllerState) -> tuple[list[float], str]:
    for source, point in (
        ("actual", msg.actual),
        ("feedback", msg.feedback),
        ("output", msg.output),
        ("reference", msg.reference),
    ):
        positions = list(point.positions)
        if positions:
            return positions, source
    return [], "empty"


def _requested_groups(raw_groups: str) -> set[str]:
    valid_groups = {"all", "shoulders", "head", "eyes", "r_hand", "l_hand"}
    groups = {group.strip() for group in raw_groups.split(",") if group.strip()}
    invalid_groups = groups - valid_groups
    if invalid_groups:
        raise ValueError(f"Unknown group(s): {', '.join(sorted(invalid_groups))}")
    return groups or {"all"}


def main() -> int:
    args = parse_args()
    try:
        groups = _requested_groups(args.groups)
    except ValueError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2

    plans = build_plans(args.calibration_path, groups)
    rclpy.init()
    node = JointRangeTester(
        plans=plans,
        duration_seconds=args.duration,
        settle_seconds=args.settle,
        tolerance=args.tolerance,
        state_timeout_seconds=args.state_timeout,
        dry_run=args.dry_run,
    )
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
