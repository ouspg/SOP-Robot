"""
Voice Chatbot – ROS 2 PySide6 desktop GUI.

Connects to the three split ROS 2 nodes (STT, LLM, TTS) via topics
and services.  Does not load any ML models itself.

Usage::

    pixi run ros-app
    # or: python ros_app.py
"""

import sys

from .platform_setup import setup_pyside6

setup_pyside6()

import rclpy
from PySide6.QtCore import QObject, Qt, QThread, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node as RosNode
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .config import Config
from .ui_common import (
    APP_STYLESHEET,
    STATUS_MAP,
    SettingsPanel,
    append_chat,
    append_log,
    update_status_label,
)

# ── ROS 2 spin thread ────────────────────────────────────────────


class _RosSpinThread(QThread):
    """Spins the ROS 2 node in a background thread (50 ms poll)."""

    def __init__(self, node: RosNode, parent: QObject | None = None):
        super().__init__(parent)
        self._node = node
        self._running = True

    def run(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def stop(self) -> None:
        self._running = False


# ── ROS 2 ↔ Qt bridge ────────────────────────────────────────────


class RosBridge(QObject):
    """Bridges ROS 2 topics/services to Qt signals."""

    log_received = Signal(str)
    status_received = Signal(str)
    chat_message = Signal(str, str)  # (role, text)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._node: RosNode | None = None
        self._spin_thread: _RosSpinThread | None = None
        self._user_pub = None
        self._clear_client = None

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def connect_ros(self) -> None:
        if self._node is not None:
            return
        rclpy.init()
        self._node = RosNode("voice_chatbot_gui")

        self._node.create_subscription(String, "/voice_chatbot/log", self._on_log, 50)
        self._node.create_subscription(
            String, "/voice_chatbot/status", self._on_status, 10
        )
        self._node.create_subscription(
            String, "/voice_chatbot/transcript", self._on_transcript, 10
        )
        self._node.create_subscription(
            String, "/voice_chatbot/assistant_text", self._on_assistant, 10
        )

        self._user_pub = self._node.create_publisher(
            String, "/voice_chatbot/user_text", 10
        )
        self._clear_client = self._node.create_client(
            Trigger, "/voice_chatbot/clear_history"
        )

        self._spin_thread = _RosSpinThread(self._node)
        self._spin_thread.start()

    def disconnect_ros(self) -> None:
        if self._node is None:
            return
        if self._spin_thread is not None:
            self._spin_thread.stop()
            self._spin_thread.wait(3000)
            self._spin_thread = None
        self._user_pub = None
        self._clear_client = None
        self._node.destroy_node()
        self._node = None
        rclpy.try_shutdown()

    def send_text(self, text: str) -> None:
        if self._user_pub is not None:
            self._user_pub.publish(String(data=text))
            self.chat_message.emit("user", text)

    def clear_history(self) -> None:
        if self._clear_client is not None and self._clear_client.service_is_ready():
            self._clear_client.call_async(Trigger.Request())
            self.log_received.emit("Keskusteluhistoria tyhjennetty.")

    # ── ROS callbacks ──

    def _on_log(self, msg: String) -> None:
        self.log_received.emit(msg.data)

    def _on_status(self, msg: String) -> None:
        self.status_received.emit(STATUS_MAP.get(msg.data, msg.data))

    def _on_transcript(self, msg: String) -> None:
        self.chat_message.emit("user", msg.data)

    def _on_assistant(self, msg: String) -> None:
        self.chat_message.emit("assistant", msg.data)


# ── Main window ───────────────────────────────────────────────────

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
_BTN_BLUE = (
    "QPushButton { background-color: #1565c0; color: white; "
    "font-weight: bold; padding: 4px 16px; border-radius: 4px; }"
    "QPushButton:hover { background-color: #1976d2; }"
)


class MainWindow(QMainWindow):
    """ROS 2 GUI window with text input and topic-based chat display."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Äänichatbot (ROS 2)")
        self.resize(1100, 720)

        self._config = Config.load()
        self._bridge = RosBridge(self)

        self._build_ui()
        self._connect_signals()
        self._set_connected_state(False)

    def _build_ui(self) -> None:
        toolbar = self.addToolBar("Hallinta")
        toolbar.setMovable(False)

        self.btn_connect = QPushButton("  Yhdistä")
        self.btn_connect.setMinimumHeight(32)
        self.btn_connect.setStyleSheet(_BTN_GREEN)
        toolbar.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("  Katkaise")
        self.btn_disconnect.setMinimumHeight(32)
        self.btn_disconnect.setStyleSheet(_BTN_RED)
        toolbar.addWidget(self.btn_disconnect)

        self.btn_save = QPushButton("  Tallenna asetukset")
        self.btn_save.setMinimumHeight(32)
        self.btn_save.setToolTip(
            "Tallenna asetukset config.json-tiedostoon (voimaan nodien uudelleenkäynnistyksellä)"
        )
        self.btn_save.setStyleSheet(_BTN_BLUE)
        toolbar.addWidget(self.btn_save)

        self.btn_clear = QPushButton("  Tyhjennä keskustelu")
        self.btn_clear.setMinimumHeight(32)
        self.btn_clear.setStyleSheet(
            "QPushButton { padding: 4px 12px; border-radius: 4px; }"
        )
        toolbar.addWidget(self.btn_clear)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        self.label_status = QLabel("Tila: Ei yhdistetty")
        self.label_status.setTextFormat(Qt.TextFormat.RichText)
        self.label_status.setStyleSheet("font-weight: bold; padding-right: 12px;")
        toolbar.addWidget(self.label_status)

        # ── Central area ──
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        self.settings_panel = SettingsPanel(self._config)
        main_layout.addWidget(self.settings_panel)

        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.setChildrenCollapsible(False)

        # Chat panel + text input
        chat_container = QWidget()
        chat_layout = QVBoxLayout(chat_container)
        chat_layout.setContentsMargins(0, 0, 0, 0)
        chat_label = QLabel("Keskustelu")
        chat_label.setStyleSheet("font-weight: bold; font-size: 13px; padding: 2px;")
        chat_layout.addWidget(chat_label)

        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setFont(QFont("Segoe UI", 11))
        self.chat_display.setStyleSheet(
            "QTextEdit { background-color: #1e1e2e; color: #cdd6f4; "
            "border: 1px solid #45475a; border-radius: 4px; padding: 8px; }"
        )
        chat_layout.addWidget(self.chat_display)

        input_row = QHBoxLayout()
        self.text_input = QLineEdit()
        self.text_input.setPlaceholderText("Kirjoita viesti...")
        self.text_input.setMinimumHeight(32)
        self.text_input.setFont(QFont("Segoe UI", 11))
        self.btn_send = QPushButton("Lähetä")
        self.btn_send.setMinimumHeight(32)
        self.btn_send.setStyleSheet(_BTN_GREEN)
        input_row.addWidget(self.text_input)
        input_row.addWidget(self.btn_send)
        chat_layout.addLayout(input_row)
        right_splitter.addWidget(chat_container)

        # Log panel
        log_container = QWidget()
        log_layout = QVBoxLayout(log_container)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_label = QLabel("Järjestelmäloki")
        log_label.setStyleSheet("font-weight: bold; font-size: 13px; padding: 2px;")
        log_layout.addWidget(log_label)

        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont("Cascadia Mono, Consolas, Courier New", 9))
        self.log_display.setMaximumBlockCount(2000)
        self.log_display.setStyleSheet(
            "QPlainTextEdit { background-color: #11111b; color: #a6adc8; "
            "border: 1px solid #45475a; border-radius: 4px; padding: 4px; }"
        )
        log_layout.addWidget(self.log_display)
        right_splitter.addWidget(log_container)

        right_splitter.setSizes([480, 200])
        main_layout.addWidget(right_splitter, stretch=1)

        self.statusBar().showMessage("Ei yhdistetty")

    def _connect_signals(self) -> None:
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_save.clicked.connect(self._on_save_settings)
        self.btn_clear.clicked.connect(self._on_clear)
        self.btn_send.clicked.connect(self._on_send)
        self.text_input.returnPressed.connect(self._on_send)

        Q = Qt.ConnectionType.QueuedConnection
        self._bridge.log_received.connect(lambda t: append_log(self.log_display, t), Q)
        self._bridge.status_received.connect(
            lambda t: update_status_label(self.label_status, self.statusBar(), t), Q
        )
        self._bridge.chat_message.connect(
            lambda r, t: append_chat(self.chat_display, r, t), Q
        )

    def _set_connected_state(self, connected: bool) -> None:
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_send.setEnabled(connected)
        self.text_input.setEnabled(connected)

    # ── Slots ──

    def _on_connect(self) -> None:
        try:
            self._bridge.connect_ros()
            self._set_connected_state(True)
            update_status_label(self.label_status, self.statusBar(), "Yhdistetty")
            append_log(self.log_display, "ROS 2 -yhteys muodostettu.")
        except Exception as exc:
            append_log(self.log_display, f"VIRHE: ROS 2 -yhteys epäonnistui: {exc}")

    def _on_disconnect(self) -> None:
        self._bridge.disconnect_ros()
        self._set_connected_state(False)
        update_status_label(self.label_status, self.statusBar(), "Ei yhdistetty")
        append_log(self.log_display, "ROS 2 -yhteys katkaistu.")

    def _on_save_settings(self) -> None:
        cfg = Config()
        self.settings_panel.write_to_config(cfg)
        cfg.save()
        append_log(self.log_display, "Asetukset tallennettu config.json-tiedostoon.")

    def _on_clear(self) -> None:
        self.chat_display.clear()
        self._bridge.clear_history()

    def _on_send(self) -> None:
        text = self.text_input.text().strip()
        if not text:
            return
        self.text_input.clear()
        self._bridge.send_text(text)

    def closeEvent(self, event) -> None:
        self._bridge.disconnect_ros()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────


def _suppress_qpainter_warnings():
    """Install a Qt message handler that silences harmless QPainter warnings.

    These occur when the Fusion style tries to paint on a widget whose
    backing store isn't ready yet — cosmetic only, does not affect
    functionality.
    """
    from PySide6.QtCore import QtMsgType, qInstallMessageHandler

    def handler(msg_type, context, message):
        if msg_type == QtMsgType.QtWarningMsg and "QPainter" in message:
            return  # swallow
        # Let everything else through to stderr
        sys.stderr.write(f"{message}\n")

    qInstallMessageHandler(handler)


def main() -> None:
    app = QApplication(sys.argv)
    _suppress_qpainter_warnings()
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
