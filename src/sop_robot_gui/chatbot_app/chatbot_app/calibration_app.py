"""Dedicated SOP-Robot joint calibration GUI."""

from __future__ import annotations

import os
import sys

from voice_stack_common.platform_setup import setup_pyside6

setup_pyside6()

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .arms import ArmControlPanel
from .ros_bridge import RosBridge
from .rviz_embed import RVizEmbedPanel
from .ui_common import APP_STYLESHEET, STATUS_MAP, append_log, update_status_label

_BTN_GREEN = (
    "QPushButton { background-color: #2e7d32; color: white; "
    "font-weight: bold; padding: 4px 16px; border-radius: 4px; }"
    "QPushButton:hover { background-color: #388e3c; }"
    "QPushButton:disabled { background-color: #666; }"
)
_BTN_RED = (
    "QPushButton { background-color: #c62828; color: white; "
    "font-weight: bold; padding: 4px 16px; border-radius: 4px; }"
    "QPushButton:hover { background-color: #d32f2f; }"
    "QPushButton:disabled { background-color: #666; }"
)


class CalibrationWindow(QMainWindow):
    """Window for moving joints to physical centers and saving launch calibration."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SOP-Robot - Joint Calibration")
        self.resize(1300, 820)
        self._bridge = RosBridge(self)
        self._build_ui()
        self._connect_signals()
        self._set_connected_state(False)
        QTimer.singleShot(250, self._on_connect)

    def _build_ui(self) -> None:
        toolbar = QToolBar("Kalibrointi")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.btn_connect = QPushButton("Yhdista")
        self.btn_connect.setMinimumHeight(32)
        self.btn_connect.setStyleSheet(_BTN_GREEN)
        toolbar.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Katkaise")
        self.btn_disconnect.setMinimumHeight(32)
        self.btn_disconnect.setStyleSheet(_BTN_RED)
        toolbar.addWidget(self.btn_disconnect)
        toolbar.addSeparator()

        self.btn_start_rviz = QPushButton("Kaynnista RViz")
        self.btn_start_rviz.setMinimumHeight(32)
        toolbar.addWidget(self.btn_start_rviz)

        self.btn_stop_rviz = QPushButton("Pysayta RViz")
        self.btn_stop_rviz.setMinimumHeight(32)
        self.btn_stop_rviz.setEnabled(False)
        toolbar.addWidget(self.btn_stop_rviz)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        self.label_status = QLabel("Tila: Ei yhdistetty")
        self.label_status.setTextFormat(Qt.TextFormat.RichText)
        self.label_status.setStyleSheet("font-weight: bold; padding-right: 12px;")
        toolbar.addWidget(self.label_status)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        self.setCentralWidget(splitter)

        self.rviz_panel = RVizEmbedPanel()
        splitter.addWidget(self.rviz_panel)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(6)

        calibration_path = os.environ.get(
            "SOP_ROBOT_JOINT_CALIBRATION_PATH",
        ) or os.environ.get(
            "SOP_ROBOT_SHOULDER_CALIBRATION_PATH",
            "config/shoulder_calibration.yaml",
        )
        self.arm_control = ArmControlPanel(
            calibration_path=calibration_path,
            calibration_mode=True,
        )
        right_layout.addWidget(self.arm_control, stretch=4)

        log_label = QLabel("Jarjestelmaloki")
        log_label.setStyleSheet("font-weight: bold; font-size: 13px; padding: 2px;")
        right_layout.addWidget(log_label)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont("Cascadia Mono, Consolas, Courier New", 9))
        self.log_display.setMaximumBlockCount(2000)
        self.log_display.setStyleSheet(
            "QPlainTextEdit { background-color: #11111b; color: #a6adc8; "
            "border: 1px solid #45475a; border-radius: 4px; padding: 4px; }"
        )
        right_layout.addWidget(self.log_display, stretch=1)

        splitter.addWidget(right_widget)
        splitter.setSizes([700, 600])
        self.statusBar().showMessage("Ei yhdistetty")

    def _connect_signals(self) -> None:
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_start_rviz.clicked.connect(self._on_start_rviz)
        self.btn_stop_rviz.clicked.connect(self._on_stop_rviz)
        self.arm_control.action_requested.connect(self._bridge.send_arm_action)
        self.arm_control.head_command_requested.connect(self._bridge.send_head_command)
        self.arm_control.left_hand_command_requested.connect(self._bridge.send_left_hand_command)
        self.arm_control.right_hand_command_requested.connect(
            self._bridge.send_right_hand_command,
        )
        self.arm_control.calibration_saved.connect(
            lambda text: append_log(self.log_display, text),
        )

        queued = Qt.ConnectionType.QueuedConnection
        self._bridge.log_received.connect(
            lambda text: append_log(self.log_display, text),
            queued,
        )
        self._bridge.status_received.connect(
            lambda text: update_status_label(
                self.label_status,
                self.statusBar(),
                STATUS_MAP.get(text) or text,
            ),
            queued,
        )

    def _set_connected_state(self, connected: bool) -> None:
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.arm_control.setEnabled(connected)

    def _on_connect(self) -> None:
        try:
            self._bridge.connect_ros()
            self._set_connected_state(True)
            update_status_label(self.label_status, self.statusBar(), "Yhdistetty")
            append_log(self.log_display, "ROS 2 -yhteys muodostettu.")
        except Exception as exc:
            append_log(self.log_display, f"VIRHE: ROS 2 -yhteys epaonnistui: {exc}")

    def _on_disconnect(self) -> None:
        self._bridge.disconnect_ros()
        self._set_connected_state(False)
        update_status_label(self.label_status, self.statusBar(), "Ei yhdistetty")
        append_log(self.log_display, "ROS 2 -yhteys katkaistu.")

    def _on_start_rviz(self) -> None:
        self.rviz_panel.start_rviz()
        self.btn_start_rviz.setEnabled(False)
        self.btn_stop_rviz.setEnabled(True)
        append_log(self.log_display, "RViz kaynnistetty.")

    def _on_stop_rviz(self) -> None:
        self.rviz_panel.stop_rviz()
        self.btn_start_rviz.setEnabled(True)
        self.btn_stop_rviz.setEnabled(False)
        append_log(self.log_display, "RViz pysaytetty.")

    def closeEvent(self, event) -> None:
        self.rviz_panel.stop_rviz()
        self._bridge.disconnect_ros()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
