"""Arm and hand controls for the unified chatbot app."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from math import degrees, radians
from pathlib import Path
import time
from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QCompleter,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node as RosNode
from std_msgs.msg import String

from sop_robot_common.arm_actions import (
    ACTION_PATTERNS,
    ARM_POSE_COMMANDS,
    AVAILABLE_ARM_COMMANDS,
    DEFAULT_SHOULDER_SERVO_IDS,
    FULL_SHOULDER_SERVO_LIMITS,
    HAND_ACTIONS,
    SHOULDER_JOINT_LABELS,
    SHOULDER_SERVO_LIMITS,
    calibrations_with_centers_and_limits,
    display_degrees_to_servo_degrees,
    servo_degrees_to_display_degrees,
    shoulder_calibration_command,
    shoulder_display_limits,
    shoulder_joint_command,
    shoulder_positions_for_action,
    shoulder_set_command,
)
from sop_robot_common.shoulder_calibration import (
    load_shoulder_center_degrees,
    load_shoulder_limits_degrees,
    reset_shoulder_limits_degrees,
    save_shoulder_center_degrees,
    save_shoulder_limits_degrees,
)
from sop_robot_common.joint_calibration import (
    EYES_SPEC,
    HEAD_SPEC,
    LEFT_HAND_SPEC,
    RIGHT_HAND_SPEC,
    JointGroupSpec,
    build_calibration_command,
    build_set_command,
    clamp_joint_position,
    display_degrees_to_raw,
    display_limits,
    load_joint_calibration,
    load_joint_specs,
    raw_to_display_degrees,
    reset_joint_group_limits,
    save_joint_group_centers,
    save_joint_group_limits,
)
from sop_robot_common.contracts import (
    ARM_ACTION_TOPIC,
    ARM_FEEDBACK_TOPIC,
    LEFT_HAND_GESTURE_TOPIC,
    LEGACY_ARM_FEEDBACK_TOPIC,
    RIGHT_HAND_GESTURE_TOPIC,
)

try:
    import serial
except ImportError:  # pragma: no cover - depends on the active Pixi environment.
    serial = None


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
DEFAULT_SERVO_IDS = list(DEFAULT_SHOULDER_SERVO_IDS)


class UnifiedArmsController:
    """Translate `/arms/arm_action` commands into hand topics and shoulder serial output."""

    def __init__(self, node: RosNode) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._closed = False

        self._serial_enabled = bool(self._parameter_value("arm_serial_enabled", True))
        self._serial_port = str(self._parameter_value("arm_serial_port", SERIAL_PORT))
        self._baud_rate = int(self._parameter_value("arm_baud_rate", BAUD_RATE))
        self._ids = [
            int(servo_id) for servo_id in self._parameter_value("arm_servo_ids", DEFAULT_SERVO_IDS)
        ]

        self._feedback_publishers = [
            node.create_publisher(String, ARM_FEEDBACK_TOPIC, 10),
            node.create_publisher(String, LEGACY_ARM_FEEDBACK_TOPIC, 10),
        ]
        self._left_hand_publisher = node.create_publisher(
            String,
            LEFT_HAND_GESTURE_TOPIC,
            10,
        )
        self._right_hand_publisher = node.create_publisher(
            String,
            RIGHT_HAND_GESTURE_TOPIC,
            10,
        )
        self._action_subscription = node.create_subscription(
            String,
            ARM_ACTION_TOPIC,
            self._on_arm_action,
            10,
        )
        self._serial = self._open_serial()

    def _parameter_value(self, name: str, default: Any) -> Any:
        return self._node.declare_parameter(name, default).value

    def _open_serial(self):
        if not self._serial_enabled:
            self._logger.info("Arm serial output disabled by arm_serial_enabled=false")
            return None
        if serial is None:
            self._logger.warning("pyserial is not available; shoulder serial output disabled.")
            return None
        try:
            connection = serial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=1.0,
                write_timeout=1.0,
            )
            self._logger.info(f"Arm serial port opened on {self._serial_port}")
            return connection
        except Exception as exc:
            self._logger.warning(
                f"Could not open arm serial port '{self._serial_port}': {exc}. Assuming fake robot."
            )
            return None

    def send_action(self, action: str) -> None:
        self._submit_action(action)

    def shutdown(self) -> None:
        self._closed = True
        self._executor.shutdown(wait=False, cancel_futures=True)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception as exc:
                self._logger.warning(f"Could not close arm serial port: {exc}")

    def _on_arm_action(self, msg: String) -> None:
        self._submit_action(msg.data)

    def _submit_action(self, action: str) -> None:
        if self._closed:
            return
        self._executor.submit(self._handle_action, action)

    def _handle_action(self, action: str) -> None:
        command = action.strip().lower()
        if not command:
            self._logger.warning("Received empty arm action")
            return
        try:
            self._dispatch_action(command)
        except Exception as exc:
            self._logger.error(f"Arm action '{command}' failed: {exc}")

    def _dispatch_action(self, command: str) -> None:
        self._logger.info(f"Arm action: {command}")

        if command.startswith(("r_hand_", "l_hand_")):
            hand = "right" if command.startswith("r_hand_") else "left"
            gesture = command[7:]
            if gesture not in HAND_ACTIONS:
                self._logger.warning(f"Unknown hand action '{gesture}'")
                return
            self.hand_gesture(gesture, hand)
            return

        if command in ACTION_PATTERNS:
            self.perform_action_from_pattern(command)
            return

        self.arm_gesture(command)

    def perform_action_from_pattern(self, pattern: str) -> None:
        for hand_or_arm, sleep_after, action, side in ACTION_PATTERNS[pattern]:
            if hand_or_arm == "hand":
                self.hand_gesture(action, side)
            elif hand_or_arm == "arm":
                self.arm_gesture(action, side)
            time.sleep(sleep_after)

    def arm_gesture(self, action: str, side: str = "both") -> None:
        positions = self._positions_for_action(action, side)
        if positions is None:
            self._logger.warning(f"Unknown arm action '{action}'")
            return

        if len(positions) < len(self._ids):
            self._logger.error(
                f"Configured positions for action '{action}' are shorter than servo_ids. "
                f"positions={positions}, servo_ids={self._ids}"
            )
            return

        command = ",".join(
            f"{self._ids[index]}:{positions[index]}" for index in range(len(self._ids))
        )
        self._logger.info(command)

        if self._serial is None:
            return

        try:
            self._serial.write(command.encode())
            feedback = self._serial.readline().decode(errors="replace").strip()
        except Exception as exc:
            self._logger.warning(f"Arm serial write failed: {exc}")
            return

        if feedback:
            self._logger.info(f"Received arm feedback: {feedback}")
            self._publish_feedback(feedback)

    def _positions_for_action(self, action: str, side: str) -> list[float] | None:
        positions = shoulder_positions_for_action(action, side)
        if positions is None and side not in {"left", "right", "both"}:
            self._logger.info(f"Expected side 'left', 'right', or 'both'; got '{side}'")
        return positions

    def hand_gesture(self, gesture: str, hand: str = "both") -> None:
        if hand not in ("right", "left", "both"):
            self._logger.info(f"Expected hand 'left', 'right', or 'both'; got '{hand}'")
            return
        msg = String(data=gesture)
        if hand in ("right", "both"):
            self._right_hand_publisher.publish(msg)
        if hand in ("left", "both"):
            self._left_hand_publisher.publish(msg)

    def _publish_feedback(self, feedback: str) -> None:
        for publisher in self._feedback_publishers:
            publisher.publish(String(data=feedback))


class ArmControlPanel(QGroupBox):
    """Compact Qt controls for publishing arm action commands."""

    action_requested = Signal(str)
    head_command_requested = Signal(str)
    left_hand_command_requested = Signal(str)
    right_hand_command_requested = Signal(str)
    calibration_saved = Signal(str)

    def __init__(
        self,
        parent: QWidget | None = None,
        calibration_path: str | Path | None = None,
        center_degrees: list[float] | None = None,
        calibration_mode: bool = False,
    ) -> None:
        super().__init__("Nivelten kalibrointi" if calibration_mode else "Kasivarret ja kadet", parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self._calibration_mode = calibration_mode
        self._calibration_path = (
            Path(calibration_path).expanduser() if calibration_path else None
        )
        self._shoulder_limits = load_shoulder_limits_degrees(self._calibration_path)
        default_calibrations = calibrations_with_centers_and_limits(
            None,
            self._active_shoulder_limits(),
        )
        self._center_degrees = (
            load_shoulder_center_degrees(self._calibration_path, default_calibrations)
            if center_degrees is None
            else list(center_degrees)
        )
        self._calibrations = calibrations_with_centers_and_limits(
            self._center_degrees,
            self._active_shoulder_limits(),
        )
        self._joint_sliders: list[QSlider] = []
        self._joint_spin_boxes: list[QSpinBox] = []
        self._saved_group_specs = load_joint_specs(self._calibration_path)
        self._group_specs = (
            load_joint_specs(self._calibration_path, use_max_limits=True)
            if self._calibration_mode
            else self._saved_group_specs
        )
        joint_calibration = load_joint_calibration(self._calibration_path, self._group_specs)
        self._group_centers = {
            HEAD_SPEC.key: joint_calibration[HEAD_SPEC.key],
            EYES_SPEC.key: joint_calibration[EYES_SPEC.key],
            RIGHT_HAND_SPEC.key: joint_calibration[RIGHT_HAND_SPEC.key],
            LEFT_HAND_SPEC.key: joint_calibration[LEFT_HAND_SPEC.key],
        }
        self._group_sliders: dict[str, list[QSlider]] = {}
        self._group_spin_boxes: dict[str, list[QSpinBox]] = {}
        self._syncing_joint_controls = False
        self._build_ui()

    def _active_shoulder_limits(self) -> tuple[tuple[float, float], ...]:
        return FULL_SHOULDER_SERVO_LIMITS if self._calibration_mode else self._shoulder_limits

    def _group_spec(self, spec: JointGroupSpec) -> JointGroupSpec:
        return self._group_specs[spec.key]

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        if not self._calibration_mode:
            pattern_row = QHBoxLayout()
            pattern_row.addWidget(QLabel("Kuviot:"))
            for command, label in (
                ("zero", "Zero"),
                ("test", "Test"),
                ("wave", "Wave"),
                ("rock", "Rock"),
                ("hold", "Hold"),
            ):
                button = QPushButton(label)
                button.setMinimumHeight(28)
                button.clicked.connect(
                    lambda _checked=False, selected=command: self._send_named_action(selected)
                )
                pattern_row.addWidget(button)
            layout.addLayout(pattern_row)

            grid = QGridLayout()
            grid.setHorizontalSpacing(6)
            grid.setVerticalSpacing(6)

            self.combo_arm_pose = QComboBox()
            for command in ARM_POSE_COMMANDS:
                self.combo_arm_pose.addItem(command, command)
            self.btn_send_arm_pose = QPushButton("Laheta")
            self.btn_send_arm_pose.clicked.connect(self._send_arm_pose)
            grid.addWidget(QLabel("Olkapaat:"), 0, 0)
            grid.addWidget(self.combo_arm_pose, 0, 1)
            grid.addWidget(self.btn_send_arm_pose, 0, 2)

            self.combo_hand_side = QComboBox()
            self.combo_hand_side.addItem("Molemmat", "both")
            self.combo_hand_side.addItem("Vasen", "left")
            self.combo_hand_side.addItem("Oikea", "right")
            self.combo_hand_gesture = QComboBox()
            for gesture in HAND_ACTIONS:
                self.combo_hand_gesture.addItem(gesture, gesture)
            self.btn_send_hand = QPushButton("Laheta")
            self.btn_send_hand.clicked.connect(self._send_hand_gesture)
            grid.addWidget(QLabel("Kasi:"), 1, 0)
            grid.addWidget(self.combo_hand_side, 1, 1)
            grid.addWidget(self.combo_hand_gesture, 1, 2)
            grid.addWidget(self.btn_send_hand, 1, 3)

            self.edit_custom_command = QLineEdit()
            self.edit_custom_command.setPlaceholderText("Komento")
            self.edit_custom_command.returnPressed.connect(self._send_custom_command)
            completer = QCompleter(list(AVAILABLE_ARM_COMMANDS), self)
            completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
            self.edit_custom_command.setCompleter(completer)
            self.btn_send_custom = QPushButton("Laheta")
            self.btn_send_custom.clicked.connect(self._send_custom_command)
            grid.addWidget(QLabel("Komento:"), 2, 0)
            grid.addWidget(self.edit_custom_command, 2, 1, 1, 2)
            grid.addWidget(self.btn_send_custom, 2, 3)

            layout.addLayout(grid)

        tabs = QTabWidget()
        tabs.addTab(self._build_joint_controls(), "Olkapaat")
        tabs.addTab(self._build_head_controls(), "Paa")
        tabs.addTab(self._build_hand_controls(), "Kadet")
        layout.addWidget(tabs)

    def _build_joint_controls(self) -> QGroupBox:
        joint_box = QGroupBox("Olkanivelet")
        joint_layout = QGridLayout(joint_box)
        joint_layout.setHorizontalSpacing(6)
        joint_layout.setVerticalSpacing(4)

        joint_layout.addWidget(QLabel("Nivel"), 0, 0)
        joint_layout.addWidget(QLabel("Asteet"), 0, 2)

        neutral_positions = self._center_degrees
        for index, label in enumerate(SHOULDER_JOINT_LABELS):
            minimum, maximum = self._shoulder_control_limits()[index]
            neutral_display_degrees = self._shoulder_control_value(index, neutral_positions[index])
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(round(minimum)), int(round(maximum)))
            slider.setSingleStep(1)
            slider.setPageStep(5)
            slider.setTracking(False)
            slider.setValue(int(round(neutral_display_degrees)))
            slider.setMinimumWidth(180)

            spin_box = QSpinBox()
            spin_box.setRange(int(round(minimum)), int(round(maximum)))
            spin_box.setSingleStep(1)
            spin_box.setValue(int(round(neutral_display_degrees)))
            spin_box.setSuffix(" deg")

            slider.valueChanged.connect(
                lambda value, selected=index: self._on_joint_slider_changed(
                    selected,
                    value,
                )
            )
            spin_box.valueChanged.connect(
                lambda value, selected=index: self._on_joint_spin_changed(
                    selected,
                    value,
                )
            )

            row = index + 1
            joint_layout.addWidget(QLabel(label), row, 0)
            joint_layout.addWidget(slider, row, 1)
            joint_layout.addWidget(spin_box, row, 2)
            if self._calibration_mode:
                btn_save_center = QPushButton("Tallenna")
                btn_save_center.clicked.connect(
                    lambda _checked=False, selected=index: self._save_current_joint_center(
                        selected
                    )
                )
                joint_layout.addWidget(btn_save_center, row, 3)
                btn_save_min = QPushButton("Min")
                btn_save_min.clicked.connect(
                    lambda _checked=False, selected=index: self._save_current_joint_limit(
                        selected,
                        "min",
                    )
                )
                btn_save_max = QPushButton("Max")
                btn_save_max.clicked.connect(
                    lambda _checked=False, selected=index: self._save_current_joint_limit(
                        selected,
                        "max",
                    )
                )
                joint_layout.addWidget(btn_save_min, row, 4)
                joint_layout.addWidget(btn_save_max, row, 5)
            self._joint_sliders.append(slider)
            self._joint_spin_boxes.append(spin_box)

        self.btn_zero_joints = QPushButton("Keskita" if self._calibration_mode else "Nollaa nivelet")
        self.btn_zero_joints.clicked.connect(self._zero_joint_controls)
        self.btn_send_all_joints = QPushButton("Laheta kaikki")
        self.btn_send_all_joints.clicked.connect(self._send_all_joints)
        button_row = len(SHOULDER_JOINT_LABELS) + 1
        joint_layout.addWidget(self.btn_zero_joints, button_row, 1)
        joint_layout.addWidget(self.btn_send_all_joints, button_row, 2)
        if self._calibration_mode:
            self.btn_save_all_centers = QPushButton("Tallenna keskikohdat")
            self.btn_save_all_centers.clicked.connect(self._save_all_current_centers)
            joint_layout.addWidget(self.btn_save_all_centers, button_row, 3)
            self.btn_reset_shoulder_limits = QPushButton("Palauta oletusrajat")
            self.btn_reset_shoulder_limits.clicked.connect(self._reset_shoulder_limits)
            joint_layout.addWidget(self.btn_reset_shoulder_limits, button_row, 4, 1, 2)
        return joint_box

    def _build_head_controls(self) -> QGroupBox:
        joint_box = QGroupBox("Paa ja silmat")
        joint_layout = QGridLayout(joint_box)
        joint_layout.setHorizontalSpacing(6)
        joint_layout.setVerticalSpacing(4)
        row = self._add_group_controls(joint_layout, HEAD_SPEC, 0)
        self._add_group_controls(joint_layout, EYES_SPEC, row + 1)
        return joint_box

    def _build_hand_controls(self) -> QGroupBox:
        joint_box = QGroupBox("Sormet")
        joint_layout = QGridLayout(joint_box)
        joint_layout.setHorizontalSpacing(6)
        joint_layout.setVerticalSpacing(4)
        row = self._add_group_controls(joint_layout, RIGHT_HAND_SPEC, 0)
        self._add_group_controls(joint_layout, LEFT_HAND_SPEC, row + 1)
        return joint_box

    def _shoulder_control_limits(self) -> tuple[tuple[float, float], ...]:
        if self._calibration_mode:
            return FULL_SHOULDER_SERVO_LIMITS
        return shoulder_display_limits(self._calibrations)

    def _shoulder_control_value(self, index: int, servo_degrees: float) -> float:
        if self._calibration_mode:
            return servo_degrees
        return servo_degrees_to_display_degrees(index, servo_degrees, self._calibrations)

    def _shoulder_servo_value(self, index: int, control_degrees: float) -> float:
        if self._calibration_mode:
            minimum, maximum = FULL_SHOULDER_SERVO_LIMITS[index]
            return min(max(control_degrees, minimum), maximum)
        return display_degrees_to_servo_degrees(index, control_degrees, self._calibrations)

    def _shoulder_display_command_value(self, index: int, servo_degrees: float) -> float:
        return servo_degrees_to_display_degrees(index, servo_degrees, self._calibrations)

    def _add_group_controls(
        self,
        layout: QGridLayout,
        spec: JointGroupSpec,
        start_row: int,
    ) -> int:
        spec = self._group_spec(spec)
        centers = self._group_centers[spec.key]
        limits = self._group_control_limits(spec)
        self._group_sliders[spec.key] = []
        self._group_spin_boxes[spec.key] = []

        layout.addWidget(QLabel(spec.controller), start_row, 0)
        layout.addWidget(QLabel("Asteet"), start_row, 2)

        for index, label in enumerate(spec.labels):
            minimum, maximum = limits[index]
            display_value = self._group_control_value(spec, index, centers[index])
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(round(minimum)), int(round(maximum)))
            slider.setSingleStep(1)
            slider.setPageStep(5)
            slider.setTracking(False)
            slider.setValue(int(round(display_value)))
            slider.setMinimumWidth(180)

            spin_box = QSpinBox()
            spin_box.setRange(int(round(minimum)), int(round(maximum)))
            spin_box.setSingleStep(1)
            spin_box.setValue(int(round(display_value)))
            spin_box.setSuffix(" deg")

            slider.valueChanged.connect(
                lambda value, group=spec, selected=index: self._on_group_slider_changed(
                    group,
                    selected,
                    value,
                )
            )
            spin_box.valueChanged.connect(
                lambda value, group=spec, selected=index: self._on_group_spin_changed(
                    group,
                    selected,
                    value,
                )
            )

            row = start_row + index + 1
            layout.addWidget(QLabel(label), row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(spin_box, row, 2)
            if self._calibration_mode:
                btn_save_center = QPushButton("Tallenna")
                btn_save_center.clicked.connect(
                    lambda _checked=False, group=spec, selected=index: (
                        self._save_current_group_joint_center(group, selected)
                    )
                )
                layout.addWidget(btn_save_center, row, 3)
                btn_save_min = QPushButton("Min")
                btn_save_min.clicked.connect(
                    lambda _checked=False, group=spec, selected=index: (
                        self._save_current_group_joint_limit(group, selected, "min")
                    )
                )
                btn_save_max = QPushButton("Max")
                btn_save_max.clicked.connect(
                    lambda _checked=False, group=spec, selected=index: (
                        self._save_current_group_joint_limit(group, selected, "max")
                    )
                )
                layout.addWidget(btn_save_min, row, 4)
                layout.addWidget(btn_save_max, row, 5)
            self._group_sliders[spec.key].append(slider)
            self._group_spin_boxes[spec.key].append(spin_box)

        button_row = start_row + len(spec.labels) + 1
        btn_center = QPushButton("Keskita")
        btn_center.clicked.connect(lambda _checked=False, group=spec: self._center_group(group))
        btn_send = QPushButton("Laheta kaikki")
        btn_send.clicked.connect(lambda _checked=False, group=spec: self._send_group(group))
        layout.addWidget(btn_center, button_row, 1)
        layout.addWidget(btn_send, button_row, 2)
        if self._calibration_mode:
            btn_save_all = QPushButton("Tallenna keskikohdat")
            btn_save_all.clicked.connect(
                lambda _checked=False, group=spec: self._save_all_group_centers(group)
            )
            layout.addWidget(btn_save_all, button_row, 3)
            btn_reset_limits = QPushButton("Palauta oletusrajat")
            btn_reset_limits.clicked.connect(
                lambda _checked=False, group=spec: self._reset_group_limits(group)
            )
            layout.addWidget(btn_reset_limits, button_row, 4, 1, 2)
        return button_row

    def _group_control_limits(
        self,
        spec: JointGroupSpec,
    ) -> tuple[tuple[float, float], ...]:
        if self._calibration_mode:
            return tuple((degrees(minimum), degrees(maximum)) for minimum, maximum in spec.limits)
        return display_limits(spec, self._group_centers[spec.key])

    def _group_control_value(
        self,
        spec: JointGroupSpec,
        index: int,
        raw_position: float,
    ) -> float:
        if self._calibration_mode:
            return degrees(raw_position)
        return raw_to_display_degrees(
            spec,
            index,
            raw_position,
            self._group_centers[spec.key][index],
        )

    def _group_raw_value(
        self,
        spec: JointGroupSpec,
        index: int,
        control_degrees: float,
    ) -> float:
        if self._calibration_mode:
            return clamp_joint_position(spec, index, radians(control_degrees))
        return display_degrees_to_raw(
            spec,
            index,
            control_degrees,
            self._group_centers[spec.key][index],
        )

    def _send_arm_pose(self) -> None:
        self._send_named_action(str(self.combo_arm_pose.currentData()))

    def _send_hand_gesture(self) -> None:
        side = str(self.combo_hand_side.currentData())
        gesture = str(self.combo_hand_gesture.currentData())
        if side == "both":
            self.action_requested.emit(f"l_hand_{gesture}")
            self.action_requested.emit(f"r_hand_{gesture}")
            return
        prefix = "l" if side == "left" else "r"
        self.action_requested.emit(f"{prefix}_hand_{gesture}")

    def _send_custom_command(self) -> None:
        command = self.edit_custom_command.text().strip().lower()
        if not command:
            return
        self.edit_custom_command.clear()
        self.action_requested.emit(command)

    def _send_named_action(self, command: str) -> None:
        positions = shoulder_positions_for_action(
            command,
            center_degrees=self._center_degrees,
            limits=self._shoulder_limits,
        )
        if positions is not None:
            self._set_joint_controls(positions)
        self.action_requested.emit(command)

    def _on_joint_slider_changed(self, index: int, value: int) -> None:
        if self._syncing_joint_controls:
            return
        self._syncing_joint_controls = True
        self._joint_spin_boxes[index].setValue(value)
        self._syncing_joint_controls = False
        servo_degrees = self._shoulder_servo_value(index, float(value))
        display_value = self._shoulder_display_command_value(index, servo_degrees)
        self.action_requested.emit(
            shoulder_joint_command(index, display_value, self._calibrations)
        )

    def _on_joint_spin_changed(self, index: int, value: int) -> None:
        if self._syncing_joint_controls:
            return
        self._syncing_joint_controls = True
        self._joint_sliders[index].setValue(value)
        self._syncing_joint_controls = False
        servo_degrees = self._shoulder_servo_value(index, float(value))
        display_value = self._shoulder_display_command_value(index, servo_degrees)
        self.action_requested.emit(
            shoulder_joint_command(index, display_value, self._calibrations)
        )

    def _zero_joint_controls(self) -> None:
        self._set_joint_controls(self._center_degrees)
        self.action_requested.emit(
            shoulder_set_command(self._shoulder_command_values(), self._calibrations)
        )

    def _send_all_joints(self) -> None:
        self.action_requested.emit(
            shoulder_set_command(self._shoulder_command_values(), self._calibrations)
        )

    def _set_joint_controls(self, positions: list[float]) -> None:
        self._syncing_joint_controls = True
        for index, value in enumerate(positions[: len(self._joint_sliders)]):
            if value < 0.0 and self._calibrations[index].servo_min >= 0.0:
                continue
            integer_value = int(round(self._shoulder_control_value(index, value)))
            self._joint_sliders[index].setValue(integer_value)
            self._joint_spin_boxes[index].setValue(integer_value)
        self._syncing_joint_controls = False

    def _joint_values(self) -> list[float]:
        return [float(spin_box.value()) for spin_box in self._joint_spin_boxes]

    def _shoulder_command_values(self) -> list[float]:
        return [
            self._shoulder_display_command_value(index, servo_degrees)
            for index, servo_degrees in enumerate(self._current_servo_degrees())
        ]

    def _current_servo_degrees(self) -> list[float]:
        return [
            self._shoulder_servo_value(
                index,
                float(spin_box.value()),
            )
            for index, spin_box in enumerate(self._joint_spin_boxes)
        ]

    def _save_current_joint_center(self, index: int) -> None:
        current_positions = self._current_servo_degrees()
        centers = list(self._center_degrees)
        centers[index] = current_positions[index]
        self._apply_saved_centers(centers, current_positions, SHOULDER_JOINT_LABELS[index])

    def _save_all_current_centers(self) -> None:
        current_positions = self._current_servo_degrees()
        self._apply_saved_centers(current_positions, current_positions, "kaikki nivelet")

    def _save_current_joint_limit(self, index: int, boundary: str) -> None:
        current_value = self._current_servo_degrees()[index]
        limits = [list(limit) for limit in self._shoulder_limits]
        if boundary == "min":
            limits[index][0] = current_value
            if limits[index][0] > limits[index][1]:
                limits[index][1] = limits[index][0]
        else:
            limits[index][1] = current_value
            if limits[index][1] < limits[index][0]:
                limits[index][0] = limits[index][1]

        self._shoulder_limits = tuple((limit[0], limit[1]) for limit in limits)
        if self._calibration_path is not None:
            saved_path = save_shoulder_limits_degrees(
                self._calibration_path,
                list(self._shoulder_limits),
            )
            self.calibration_saved.emit(f"Olkanivelten rajat tallennettu: {saved_path}")
        else:
            self.calibration_saved.emit("Olkanivelten rajat paivitetty.")
        self.calibration_saved.emit(
            f"{SHOULDER_JOINT_LABELS[index]} {boundary} = {current_value:.0f} deg"
        )

    def _reset_shoulder_limits(self) -> None:
        self._shoulder_limits = SHOULDER_SERVO_LIMITS
        self._calibrations = calibrations_with_centers_and_limits(
            self._center_degrees,
            self._active_shoulder_limits(),
        )
        if self._calibration_path is not None:
            saved_path = reset_shoulder_limits_degrees(self._calibration_path)
            self.calibration_saved.emit(f"Olkanivelten oletusrajat palautettu: {saved_path}")
        else:
            self.calibration_saved.emit("Olkanivelten oletusrajat palautettu.")
        self._refresh_joint_ranges(self._current_servo_degrees())

    def _apply_saved_centers(
        self,
        center_degrees: list[float],
        current_positions: list[float],
        label: str,
    ) -> None:
        self._center_degrees = list(center_degrees)
        self._calibrations = calibrations_with_centers_and_limits(
            self._center_degrees,
            self._active_shoulder_limits(),
        )
        self._refresh_joint_ranges(current_positions)
        if self._calibration_path is not None:
            saved_path = save_shoulder_center_degrees(
                self._calibration_path,
                self._center_degrees,
                self._calibrations,
            )
            self.calibration_saved.emit(f"Olkanivelten keskikohdat tallennettu: {saved_path}")
        else:
            self.calibration_saved.emit("Olkanivelten keskikohdat paivitetty.")
        self.action_requested.emit(
            shoulder_calibration_command(self._center_degrees, self._calibrations)
        )
        self.calibration_saved.emit(f"Keskikohta asetettu: {label}")

    def _refresh_joint_ranges(self, servo_positions: list[float]) -> None:
        self._syncing_joint_controls = True
        limits = self._shoulder_control_limits()
        for index, (slider, spin_box) in enumerate(
            zip(self._joint_sliders, self._joint_spin_boxes, strict=True)
        ):
            minimum, maximum = limits[index]
            slider.setRange(int(round(minimum)), int(round(maximum)))
            spin_box.setRange(int(round(minimum)), int(round(maximum)))
            display_value = int(round(self._shoulder_control_value(index, servo_positions[index])))
            slider.setValue(display_value)
            spin_box.setValue(display_value)
        self._syncing_joint_controls = False

    def _on_group_slider_changed(
        self,
        spec: JointGroupSpec,
        index: int,
        value: int,
    ) -> None:
        if self._syncing_joint_controls:
            return
        self._syncing_joint_controls = True
        self._group_spin_boxes[spec.key][index].setValue(value)
        self._syncing_joint_controls = False
        self._send_group(spec)

    def _on_group_spin_changed(
        self,
        spec: JointGroupSpec,
        index: int,
        value: int,
    ) -> None:
        if self._syncing_joint_controls:
            return
        self._syncing_joint_controls = True
        self._group_sliders[spec.key][index].setValue(value)
        self._syncing_joint_controls = False
        self._send_group(spec)

    def _center_group(self, spec: JointGroupSpec) -> None:
        self._set_group_controls(spec, self._group_centers[spec.key])
        self._send_group(spec)

    def _send_group(self, spec: JointGroupSpec) -> None:
        self._emit_group_command(spec, build_set_command(spec, self._group_raw_values(spec)))

    def _group_raw_values(self, spec: JointGroupSpec) -> list[float]:
        return [
            self._group_raw_value(spec, index, float(spin_box.value()))
            for index, spin_box in enumerate(self._group_spin_boxes[spec.key])
        ]

    def _set_group_controls(self, spec: JointGroupSpec, raw_positions: list[float]) -> None:
        self._syncing_joint_controls = True
        for index, value in enumerate(raw_positions[: len(self._group_sliders[spec.key])]):
            display_value = int(round(self._group_control_value(spec, index, value)))
            self._group_sliders[spec.key][index].setValue(display_value)
            self._group_spin_boxes[spec.key][index].setValue(display_value)
        self._syncing_joint_controls = False

    def _save_current_group_joint_center(
        self,
        spec: JointGroupSpec,
        index: int,
    ) -> None:
        current_positions = self._group_raw_values(spec)
        centers = list(self._group_centers[spec.key])
        centers[index] = current_positions[index]
        self._apply_group_centers(spec, centers, current_positions, spec.labels[index])

    def _save_all_group_centers(self, spec: JointGroupSpec) -> None:
        current_positions = self._group_raw_values(spec)
        self._apply_group_centers(spec, current_positions, current_positions, "kaikki nivelet")

    def _save_current_group_joint_limit(
        self,
        spec: JointGroupSpec,
        index: int,
        boundary: str,
    ) -> None:
        current_value = self._group_raw_values(spec)[index]
        saved_spec = self._saved_group_specs[spec.key]
        limits = [list(limit) for limit in saved_spec.limits]
        if boundary == "min":
            limits[index][0] = current_value
            if limits[index][0] > limits[index][1]:
                limits[index][1] = limits[index][0]
        else:
            limits[index][1] = current_value
            if limits[index][1] < limits[index][0]:
                limits[index][0] = limits[index][1]

        if self._calibration_path is not None:
            saved_path = save_joint_group_limits(self._calibration_path, spec.key, limits)
            self.calibration_saved.emit(f"{spec.controller} rajat tallennettu: {saved_path}")
            self._saved_group_specs = load_joint_specs(self._calibration_path)
        else:
            self.calibration_saved.emit(f"{spec.controller} rajat paivitetty.")
        self.calibration_saved.emit(
            f"{spec.labels[index]} {boundary} = {degrees(current_value):.0f} deg"
        )

    def _reset_group_limits(self, spec: JointGroupSpec) -> None:
        if self._calibration_path is not None:
            saved_path = reset_joint_group_limits(self._calibration_path, spec.key)
            self.calibration_saved.emit(f"{spec.controller} oletusrajat palautettu: {saved_path}")
            self._saved_group_specs = load_joint_specs(self._calibration_path)
        else:
            self.calibration_saved.emit(f"{spec.controller} oletusrajat palautettu.")
        if not self._calibration_mode:
            self._group_specs = self._saved_group_specs
            self._refresh_group_ranges(self._group_spec(spec), self._group_raw_values(spec))

    def _apply_group_centers(
        self,
        spec: JointGroupSpec,
        centers: list[float],
        current_positions: list[float],
        label: str,
    ) -> None:
        self._group_centers[spec.key] = centers
        self._refresh_group_ranges(spec, current_positions)
        if self._calibration_path is not None:
            saved_path = save_joint_group_centers(
                self._calibration_path,
                spec.key,
                centers,
                spec_override=spec,
            )
            self.calibration_saved.emit(f"{spec.controller} keskikohdat tallennettu: {saved_path}")
        else:
            self.calibration_saved.emit(f"{spec.controller} keskikohdat paivitetty.")
        self._emit_group_command(spec, build_calibration_command(spec, centers))
        self.calibration_saved.emit(f"Keskikohta asetettu: {label}")

    def _refresh_group_ranges(
        self,
        spec: JointGroupSpec,
        raw_positions: list[float],
    ) -> None:
        limits = self._group_control_limits(spec)
        self._syncing_joint_controls = True
        for index, (slider, spin_box) in enumerate(
            zip(
                self._group_sliders[spec.key],
                self._group_spin_boxes[spec.key],
                strict=True,
            )
        ):
            minimum, maximum = limits[index]
            slider.setRange(int(round(minimum)), int(round(maximum)))
            spin_box.setRange(int(round(minimum)), int(round(maximum)))
            display_value = int(round(self._group_control_value(spec, index, raw_positions[index])))
            slider.setValue(display_value)
            spin_box.setValue(display_value)
        self._syncing_joint_controls = False

    def _emit_group_command(self, spec: JointGroupSpec, command: str) -> None:
        if spec.key in {HEAD_SPEC.key, EYES_SPEC.key}:
            self.head_command_requested.emit(command)
        elif spec.key == RIGHT_HAND_SPEC.key:
            self.right_hand_command_requested.emit(command)
        elif spec.key == LEFT_HAND_SPEC.key:
            self.left_hand_command_requested.emit(command)
