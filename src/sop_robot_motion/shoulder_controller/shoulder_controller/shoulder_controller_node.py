from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sop_robot_common.arm_actions import (
    ACTION_PATTERNS,
    DEFAULT_SHOULDER_SERVO_IDS,
    calibrations_with_centers_and_limits,
    parse_shoulder_calibration_command,
    parse_shoulder_joint_command,
    parse_shoulder_set_command,
    shoulder_positions_for_action,
    shoulder_positions_to_joint_goal,
    shoulder_serial_command,
    split_hand_command,
)
from sop_robot_common.shoulder_calibration import (
    load_shoulder_center_degrees,
    load_shoulder_limits_degrees,
)
from sop_robot_common.contracts import (
    ARM_ACTION_TOPIC,
    ARM_FEEDBACK_TOPIC,
    LEFT_HAND_GESTURE_TOPIC,
    LEGACY_ARM_FEEDBACK_TOPIC,
    RIGHT_HAND_GESTURE_TOPIC,
)
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import serial
except ImportError:  # pragma: no cover - pyserial is an environment dependency.
    serial = None


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


class ShoulderController(Node):
    """Route arm actions to RViz fake joints or real Arduino shoulder servos."""

    def __init__(self) -> None:
        super().__init__("shoulder_controller_bridge")
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._closed = False

        self._use_fake = bool(self._parameter_value("use_fake", True))
        self._serial_enabled = bool(self._parameter_value("serial_enabled", True))
        self._serial_port = str(self._parameter_value("serial_port", SERIAL_PORT))
        self._baud_rate = int(self._parameter_value("baud_rate", BAUD_RATE))
        self._serial_timeout = float(self._parameter_value("serial_timeout", 1.0))
        self._goal_duration_seconds = float(
            self._parameter_value("goal_duration_seconds", 1.0)
        )
        self._controller_name = str(
            self._parameter_value("trajectory_controller", "shoulder_controller")
        ).strip("/")
        self._controller_wait_timeout = float(
            self._parameter_value("controller_wait_timeout", 2.0)
        )
        self._servo_ids = [
            int(servo_id)
            for servo_id in self._parameter_value(
                "servo_ids",
                list(DEFAULT_SHOULDER_SERVO_IDS),
            )
        ]
        self._calibration_path = str(
            self._parameter_value("shoulder_calibration_path", "")
        )
        self._use_max_calibration_limits = bool(
            self._parameter_value("shoulder_calibration_use_max_limits", False)
        )
        self._shoulder_limits = load_shoulder_limits_degrees(
            self._calibration_path,
            use_max_limits=self._use_max_calibration_limits,
        )
        default_calibrations = calibrations_with_centers_and_limits(
            None,
            self._shoulder_limits,
        )
        self._center_degrees = load_shoulder_center_degrees(
            self._calibration_path,
            default_calibrations,
        )
        self._calibrations = calibrations_with_centers_and_limits(
            self._center_degrees,
            self._shoulder_limits,
        )
        self._current_servo_degrees = list(self._center_degrees)

        self._serial = None if self._use_fake else self._open_serial()
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._controller_name}/follow_joint_trajectory",
        )
        self._feedback_publishers = [
            self.create_publisher(String, ARM_FEEDBACK_TOPIC, 10),
            self.create_publisher(String, LEGACY_ARM_FEEDBACK_TOPIC, 10),
        ]
        self._right_hand_publisher = self.create_publisher(
            String,
            RIGHT_HAND_GESTURE_TOPIC,
            10,
        )
        self._left_hand_publisher = self.create_publisher(
            String,
            LEFT_HAND_GESTURE_TOPIC,
            10,
        )
        self._subscription = self.create_subscription(
            String,
            ARM_ACTION_TOPIC,
            self._on_arm_action,
            10,
        )

        mode = "fake ros2_control" if self._use_fake else "real serial"
        self.get_logger().info(f"Shoulder controller started in {mode} mode.")

    def _parameter_value(self, name: str, default: Any) -> Any:
        return self.declare_parameter(name, default).value

    def _open_serial(self):
        if not self._serial_enabled:
            self.get_logger().info("Shoulder serial output disabled.")
            return None
        if serial is None:
            self.get_logger().warning("pyserial is not available; shoulder serial disabled.")
            return None
        try:
            connection = serial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=self._serial_timeout,
                write_timeout=self._serial_timeout,
            )
            self.get_logger().info(f"Shoulder serial port opened on {self._serial_port}")
            return connection
        except Exception as exc:
            self.get_logger().warning(
                f"Could not open shoulder serial port '{self._serial_port}': {exc}"
            )
            return None

    def _on_arm_action(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if not command or self._closed:
            return
        self._executor.submit(self._handle_action, command)

    def _handle_action(self, command: str) -> None:
        try:
            self._dispatch_action(command)
        except Exception as exc:
            self.get_logger().error(f"Shoulder action '{command}' failed: {exc}")

    def _dispatch_action(self, command: str) -> None:
        calibration_update = parse_shoulder_calibration_command(command, self._calibrations)
        if calibration_update is not None:
            self._set_center_degrees(calibration_update)
            return

        hand_command = split_hand_command(command)
        if hand_command is not None:
            hand, gesture = hand_command
            self._publish_hand_gesture(hand, gesture)
            return

        if command in ACTION_PATTERNS:
            self._perform_pattern(command)
            return

        full_positions = parse_shoulder_set_command(command, self._calibrations)
        if full_positions is not None:
            self.send_shoulder_positions(full_positions, "manual shoulder set")
            return

        joint_update = parse_shoulder_joint_command(command, self._calibrations)
        if joint_update is not None:
            index, value = joint_update
            positions = list(self._current_servo_degrees)
            positions[index] = value
            self.send_shoulder_positions(positions, "manual shoulder joint")
            return

        self.send_shoulder_action(command)

    def _perform_pattern(self, pattern: str) -> None:
        for target, sleep_after, action, side in ACTION_PATTERNS[pattern]:
            if self._closed:
                return
            if target == "hand":
                self._publish_hand_gesture(side, action)
            elif target == "arm":
                self.send_shoulder_action(action, side)
            time.sleep(sleep_after)

    def _publish_hand_gesture(self, hand: str, gesture: str) -> None:
        msg = String(data=gesture)
        if hand in ("right", "both"):
            self._right_hand_publisher.publish(msg)
        if hand in ("left", "both"):
            self._left_hand_publisher.publish(msg)

    def _set_center_degrees(self, center_degrees: list[float]) -> None:
        self._center_degrees = list(center_degrees)
        self._calibrations = calibrations_with_centers_and_limits(
            self._center_degrees,
            self._shoulder_limits,
        )
        self.get_logger().info(f"Updated shoulder centers: {self._center_degrees}")

    def send_shoulder_action(self, action: str, side: str = "both") -> None:
        positions = shoulder_positions_for_action(
            action,
            side,
            self._center_degrees,
            self._shoulder_limits,
        )
        if positions is None:
            self.get_logger().warning(f"Unknown shoulder action '{action}'")
            return

        self.send_shoulder_positions(positions, action)

    def send_shoulder_positions(self, positions: list[float], action: str) -> None:
        for index, value in enumerate(positions[: len(self._current_servo_degrees)]):
            if value >= 0.0:
                self._current_servo_degrees[index] = value

        if self._use_fake:
            self._send_fake_goal(positions, action)
            return
        self._send_serial_command(positions, action)

    def _send_fake_goal(self, servo_degrees: list[float], action: str) -> None:
        joint_names, joint_positions = shoulder_positions_to_joint_goal(
            servo_degrees,
            self._calibrations,
        )
        if not joint_names:
            self.get_logger().warning(f"Shoulder action '{action}' did not target any joints")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory(
            joint_names=joint_names,
            points=[
                JointTrajectoryPoint(
                    positions=joint_positions,
                    time_from_start=self._duration_from_seconds(
                        self._goal_duration_seconds
                    ),
                )
            ],
        )

        if not self._action_client.wait_for_server(
            timeout_sec=self._controller_wait_timeout
        ):
            self.get_logger().warning(
                f"Shoulder trajectory controller '/{self._controller_name}' is not ready"
            )
            return
        self._action_client.send_goal_async(goal_msg)

    def _send_serial_command(self, servo_degrees: list[float], action: str) -> None:
        command = shoulder_serial_command(servo_degrees, self._servo_ids, self._calibrations)
        if not command:
            self.get_logger().warning(f"Shoulder action '{action}' produced no serial command")
            return

        self.get_logger().info(command)
        if self._serial is None:
            self.get_logger().warning("Shoulder serial output is unavailable.")
            return

        try:
            self._serial.write(f"{command}\n".encode())
            feedback = self._serial.readline().decode(errors="replace").strip()
        except Exception as exc:
            self.get_logger().warning(f"Shoulder serial write failed: {exc}")
            return

        if feedback:
            self.get_logger().info(f"Received shoulder feedback: {feedback}")
            self._publish_feedback(feedback)

    def _publish_feedback(self, feedback: str) -> None:
        msg = String(data=feedback)
        for publisher in self._feedback_publishers:
            publisher.publish(msg)

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = int((seconds - whole_seconds) * 1_000_000_000)
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    def destroy_node(self) -> None:
        self._closed = True
        self._executor.shutdown(wait=False, cancel_futures=True)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception as exc:
                self.get_logger().warning(f"Could not close shoulder serial port: {exc}")
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ShoulderController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
