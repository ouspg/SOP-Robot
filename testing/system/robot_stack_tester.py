#!/usr/bin/env python3
"""Automatic integration tester for the SOP-Robot fake stack."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Iterable, TextIO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RESULT_ROOT = PROJECT_ROOT / "test-results"
STALE_GRAPH_CLEAR_TIMEOUT = 20.0

BASE_LAUNCH_ARGS = (
    "use_rviz:=false",
    "enable_chatbot_ui:=false",
    "enable_ros2graph_ui:=false",
    "asr_test_mode:=true",
    "runtime_log_level:=warn",
)

FEATURE_OFF_ARGS = (
    "enable_face_tracker:=false",
    "enable_face_tracker_movement:=false",
    "enable_voice_stack:=false",
    "enable_full_demo:=false",
    "enable_hand_gestures:=false",
    "enable_jaw_movement:=false",
)

COMMON_EXPECTED_NODES = (
    "/controller_manager",
    "/robot_state_publisher",
)

COMMON_EXPECTED_TOPICS = (
    "/joint_states",
    "/robot_description",
)

COMMON_EXPECTED_SERVICES = (
    "/controller_manager/list_controllers",
)

TOPIC_ECHO_FIELDS = {
    "/image_raw": "header.stamp",
    "/face_tracker/image_face": "header.stamp",
    "/joint_states": "header.stamp",
    "/voice_chatbot/status": "data",
}


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    launch_args: tuple[str, ...]
    expected_nodes: tuple[str, ...] = ()
    expected_topics: tuple[str, ...] = ()
    expected_services: tuple[str, ...] = ()
    expected_actions: tuple[str, ...] = ()
    live_topics: tuple[str, ...] = ()
    startup_timeout: float = 35.0


@dataclass
class ManagedLaunch:
    name: str
    process: subprocess.Popen[str]
    log_path: Path
    log_handle: TextIO

    def stop(self) -> None:
        if self.process.poll() is None:
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5)
        self.log_handle.close()


def _feature_args(**enabled: bool) -> tuple[str, ...]:
    values = {
        "enable_face_tracker": False,
        "enable_face_tracker_movement": False,
        "enable_voice_stack": False,
        "enable_full_demo": False,
        "enable_hand_gestures": False,
        "enable_shoulder_controller": False,
        "enable_jaw_movement": False,
    }
    values.update(enabled)
    return tuple(f"{key}:={'true' if value else 'false'}" for key, value in values.items())


def _scenarios() -> tuple[Scenario, ...]:
    return (
        Scenario(
            name="control",
            description="fake ros2_control, robot_state_publisher, controllers",
            launch_args=_feature_args(),
            expected_nodes=COMMON_EXPECTED_NODES,
            expected_topics=COMMON_EXPECTED_TOPICS,
            expected_services=COMMON_EXPECTED_SERVICES,
            expected_actions=(
                "/head_controller/follow_joint_trajectory",
                "/eyes_controller/follow_joint_trajectory",
                "/jaw_controller/follow_joint_trajectory",
            ),
            live_topics=("/joint_states",),
        ),
        Scenario(
            name="face_tracker",
            description="webcam node and face tracker image topics",
            launch_args=_feature_args(enable_face_tracker=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/webcam",
                "/face_tracker/face_tracker_node",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/image_raw",
                "/face_tracker/image_face",
                "/face_tracker/faces",
            ),
            expected_services=COMMON_EXPECTED_SERVICES,
            live_topics=("/image_raw", "/face_tracker/image_face"),
            startup_timeout=45.0,
        ),
        Scenario(
            name="face_tracker_movement",
            description="head/eye face tracking movement bridge",
            launch_args=_feature_args(enable_face_tracker_movement=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/face_tracker_movement",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/face_tracker/faces",
                "/face_tracker_movement/focused_face",
                "/face_tracker_movement/head_gesture_topic",
            ),
            expected_services=COMMON_EXPECTED_SERVICES,
            expected_actions=(
                "/head_controller/follow_joint_trajectory",
                "/eyes_controller/follow_joint_trajectory",
            ),
        ),
        Scenario(
            name="hand_gestures",
            description="hand gesture node and hand action servers",
            launch_args=_feature_args(enable_hand_gestures=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/hand_gestures",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/r_hand/r_hand_topic",
                "/l_hand/l_hand_topic",
            ),
            expected_services=COMMON_EXPECTED_SERVICES,
            expected_actions=(
                "/r_hand_controller/follow_joint_trajectory",
                "/l_hand_controller/follow_joint_trajectory",
            ),
        ),
        Scenario(
            name="shoulder_controller",
            description="shoulder action bridge and fake trajectory controller",
            launch_args=_feature_args(enable_shoulder_controller=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/shoulder_controller_bridge",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/arms/arm_action",
                "/arms/feedback",
                "/r_hand/r_hand_topic",
                "/l_hand/l_hand_topic",
            ),
            expected_services=COMMON_EXPECTED_SERVICES,
            expected_actions=("/shoulder_controller/follow_joint_trajectory",),
        ),
        Scenario(
            name="jaw_movement",
            description="jaw movement node and jaw action server",
            launch_args=_feature_args(enable_jaw_movement=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/jaw_movement",
            ),
            expected_topics=(*COMMON_EXPECTED_TOPICS, "/jaw_topic"),
            expected_services=COMMON_EXPECTED_SERVICES,
            expected_actions=("/jaw_controller/follow_joint_trajectory",),
        ),
        Scenario(
            name="full_demo",
            description="demo coordinator contracts",
            launch_args=_feature_args(enable_full_demo=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/full_demo",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/voice_chatbot/assistant_text",
                "/voice_chatbot/can_listen",
                "/voice_chatbot/tts_done",
                "/voice_chatbot/status",
                "/can_listen",
                "/arms/arm_action",
                "/face_tracker/faces",
                "/face_tracker_movement/focused_face",
            ),
            expected_services=COMMON_EXPECTED_SERVICES,
        ),
        Scenario(
            name="voice_stack",
            description="ASR, LLM, and TTS voice contracts",
            launch_args=_feature_args(enable_voice_stack=True),
            expected_nodes=(
                *COMMON_EXPECTED_NODES,
                "/asr_package",
                "/llm_package",
                "/tts_package",
            ),
            expected_topics=(
                *COMMON_EXPECTED_TOPICS,
                "/voice_chatbot/user_text",
                "/voice_chatbot/transcript",
                "/voice_chatbot/assistant_text",
                "/voice_chatbot/status",
                "/voice_chatbot/log",
                "/voice_chatbot/tts_done",
                "/voice_chatbot/can_listen",
                "/jaw_topic",
            ),
            expected_services=(
                *COMMON_EXPECTED_SERVICES,
                "/voice_chatbot/clear_history",
            ),
            live_topics=("/voice_chatbot/status",),
            startup_timeout=120.0,
        ),
    )


def _all_at_once_scenario(include_gui: bool, include_rviz: bool) -> Scenario:
    expected_nodes = (
        *COMMON_EXPECTED_NODES,
        "/webcam",
        "/face_tracker/face_tracker_node",
        "/face_tracker_movement",
        "/asr_package",
        "/llm_package",
        "/tts_package",
        "/full_demo",
        "/hand_gestures",
        "/shoulder_controller_bridge",
        "/jaw_movement",
    )
    launch_args = (
        "enable_face_tracker:=true",
        "enable_face_tracker_movement:=true",
        "enable_voice_stack:=true",
        f"enable_chatbot_ui:={'true' if include_gui else 'false'}",
        "enable_full_demo:=true",
        "enable_hand_gestures:=true",
        "enable_shoulder_controller:=true",
        "enable_jaw_movement:=true",
        f"use_rviz:={'true' if include_rviz else 'false'}",
    )
    return Scenario(
        name="all_at_once",
        description="full fake stack graph",
        launch_args=launch_args,
        expected_nodes=expected_nodes,
        expected_topics=(
            *COMMON_EXPECTED_TOPICS,
            "/image_raw",
            "/face_tracker/image_face",
            "/face_tracker/faces",
            "/face_tracker_movement/focused_face",
            "/face_tracker_movement/head_gesture_topic",
            "/voice_chatbot/user_text",
            "/voice_chatbot/transcript",
            "/voice_chatbot/assistant_text",
            "/voice_chatbot/status",
            "/voice_chatbot/log",
            "/voice_chatbot/tts_done",
            "/voice_chatbot/can_listen",
            "/can_listen",
            "/jaw_topic",
            "/arms/arm_action",
            "/arms/feedback",
            "/r_hand/r_hand_topic",
            "/l_hand/l_hand_topic",
        ),
        expected_services=(
            *COMMON_EXPECTED_SERVICES,
            "/voice_chatbot/clear_history",
        ),
        expected_actions=(
            "/head_controller/follow_joint_trajectory",
            "/eyes_controller/follow_joint_trajectory",
            "/jaw_controller/follow_joint_trajectory",
            "/r_hand_controller/follow_joint_trajectory",
            "/shoulder_controller/follow_joint_trajectory",
            "/l_hand_controller/follow_joint_trajectory",
        ),
        live_topics=(
            "/joint_states",
            "/image_raw",
            "/face_tracker/image_face",
            "/voice_chatbot/status",
        ),
        startup_timeout=140.0,
    )


def run_command(args: list[str], timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def ros_list(kind: str) -> set[str]:
    result = run_command(["ros2", kind, "list"], timeout=10.0)
    if result.returncode != 0:
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def existing_known_nodes() -> set[str]:
    known: set[str] = set(COMMON_EXPECTED_NODES)
    for scenario in _scenarios():
        known.update(scenario.expected_nodes)
    known.update(_all_at_once_scenario(include_gui=False, include_rviz=False).expected_nodes)
    return ros_list("node").intersection(known)


def launch_scenario(scenario: Scenario, log_dir: Path) -> ManagedLaunch:
    log_path = log_dir / f"{scenario.name}.log"
    log_handle = log_path.open("w", encoding="utf-8")
    command = [
        "ros2",
        "launch",
        "robot",
        "robot.fake.launch.py",
        *BASE_LAUNCH_ARGS,
        *scenario.launch_args,
    ]
    log_handle.write("$ " + " ".join(command) + "\n\n")
    log_handle.flush()
    process = subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return ManagedLaunch(
        name=scenario.name,
        process=process,
        log_path=log_path,
        log_handle=log_handle,
    )


def missing_items(expected: Iterable[str], actual: set[str]) -> list[str]:
    return sorted(item for item in expected if item not in actual)


def wait_for_graph(scenario: Scenario, launch: ManagedLaunch) -> list[str]:
    deadline = time.monotonic() + scenario.startup_timeout
    failures: list[str] = []
    while time.monotonic() < deadline:
        if launch.process.poll() is not None:
            return [f"launch exited early with code {launch.process.returncode}"]

        nodes = ros_list("node")
        topics = ros_list("topic")
        services = ros_list("service")
        actions = ros_list("action")

        failures = []
        for label, expected, actual in (
            ("nodes", scenario.expected_nodes, nodes),
            ("topics", scenario.expected_topics, topics),
            ("services", scenario.expected_services, services),
            ("actions", scenario.expected_actions, actions),
        ):
            missing = missing_items(expected, actual)
            if missing:
                failures.append(f"missing {label}: {', '.join(missing)}")

        if not failures:
            return []
        time.sleep(1.0)

    return failures


def wait_for_live_topic(topic: str, timeout: float) -> str | None:
    command = ["ros2", "topic", "echo", topic, "--once"]
    field = TOPIC_ECHO_FIELDS.get(topic)
    if field:
        command.extend(["--field", field])
    try:
        result = run_command(command, timeout=timeout)
    except subprocess.TimeoutExpired:
        return f"{topic}: no message within {timeout:.0f}s"
    if result.returncode != 0:
        error = (result.stderr or result.stdout).strip().splitlines()
        detail = error[-1] if error else f"exit code {result.returncode}"
        return f"{topic}: echo failed ({detail})"
    return None


def run_scenario(
    scenario: Scenario,
    log_dir: Path,
    live_topics: bool,
    live_timeout: float,
) -> bool:
    print(f"\n== {scenario.name}: {scenario.description}")
    launch = launch_scenario(scenario, log_dir)
    try:
        failures = wait_for_graph(scenario, launch)
        if live_topics and not failures:
            for topic in scenario.live_topics:
                failure = wait_for_live_topic(topic, live_timeout)
                if failure:
                    failures.append(failure)

        if failures:
            print("FAIL")
            for failure in failures:
                print(f"  - {failure}")
            print(f"  log: {launch.log_path}")
            print_log_tail(launch.log_path)
            return False

        print("PASS")
        return True
    finally:
        launch.stop()
        wait_for_known_nodes_to_clear(timeout=STALE_GRAPH_CLEAR_TIMEOUT)


def wait_for_known_nodes_to_clear(timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not existing_known_nodes():
            return
        time.sleep(0.5)


def print_log_tail(path: Path, lines: int = 30) -> None:
    try:
        text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return
    tail = text[-lines:]
    if tail:
        print("  launch log tail:")
        for line in tail:
            print(f"    {line}")


def selected_scenarios(names: list[str]) -> tuple[Scenario, ...]:
    scenarios = _scenarios()
    if not names:
        return scenarios
    by_name = {scenario.name: scenario for scenario in scenarios}
    unknown = sorted(set(names) - set(by_name))
    if unknown:
        raise SystemExit(f"Unknown scenario(s): {', '.join(unknown)}")
    return tuple(by_name[name] for name in names)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        choices=("one-by-one", "all-at-once"),
        help="Run each subsystem in isolation or the full fake stack in one launch.",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Only run this one-by-one scenario. May be repeated.",
    )
    parser.add_argument(
        "--no-live-topics",
        action="store_true",
        help="Only validate graph contracts; do not wait for live topic messages.",
    )
    parser.add_argument(
        "--live-timeout",
        type=float,
        default=12.0,
        help="Seconds to wait for each live topic message.",
    )
    parser.add_argument(
        "--include-gui",
        action="store_true",
        help="Start the unified GUI during the all-at-once test.",
    )
    parser.add_argument(
        "--include-rviz",
        action="store_true",
        help="Start standalone RViz during the all-at-once test.",
    )
    parser.add_argument(
        "--allow-existing-ros-graph",
        action="store_true",
        help="Do not fail if known robot nodes are already running.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.mode == "one-by-one" and (args.include_gui or args.include_rviz):
        raise SystemExit("--include-gui and --include-rviz only apply to all-at-once")

    existing = existing_known_nodes()
    if existing and not args.allow_existing_ros_graph:
        wait_for_known_nodes_to_clear(timeout=STALE_GRAPH_CLEAR_TIMEOUT)
        existing = existing_known_nodes()
    if existing and not args.allow_existing_ros_graph:
        print("Known SOP-Robot nodes are already running:")
        for node in sorted(existing):
            print(f"  - {node}")
        print("Stop the robot first, or pass --allow-existing-ros-graph.")
        return 2

    started_at = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = RESULT_ROOT / f"robot_stack_{started_at}"
    log_dir.mkdir(parents=True, exist_ok=True)

    live_topics = not args.no_live_topics
    if args.mode == "one-by-one":
        scenarios = selected_scenarios(args.scenario)
    else:
        scenarios = (
            _all_at_once_scenario(
                include_gui=args.include_gui,
                include_rviz=args.include_rviz,
            ),
        )

    print(f"Writing launch logs to {log_dir}")
    passed = 0
    failed = 0
    try:
        for scenario in scenarios:
            ok = run_scenario(
                scenario,
                log_dir=log_dir,
                live_topics=live_topics,
                live_timeout=args.live_timeout,
            )
            if ok:
                passed += 1
            else:
                failed += 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130

    print(f"\nSummary: {passed} passed, {failed} failed")
    print(f"Logs: {log_dir}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
