"""Plain ROS2 chatbot GUI."""

from __future__ import annotations

import sys

from voice_stack_common.platform_setup import setup_pyside6

setup_pyside6()

from PySide6.QtCore import Qt
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

from .config import Config
from .ros_bridge import RosBridge
from .ui_common import (
    APP_STYLESHEET,
    STATUS_MAP,
    SettingsPanel,
    append_chat,
    append_log,
    update_status_label,
)

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
    """ROS2 GUI window with text input and topic-based chat display."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Aanichatbot (ROS 2)")
        self.resize(1100, 720)

        self._config = Config.load()
        self._bridge = RosBridge(self)

        self._build_ui()
        self._connect_signals()
        self._set_connected_state(False)

    def _build_ui(self) -> None:
        toolbar = self.addToolBar("Hallinta")
        toolbar.setMovable(False)

        self.btn_connect = QPushButton("  Yhdista")
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
            "Tallenna asetukset voice config tiedostoon. Muutokset tulevat voimaan "
            "nodejen uudelleenkaynnistyksen jalkeen."
        )
        self.btn_save.setStyleSheet(_BTN_BLUE)
        toolbar.addWidget(self.btn_save)

        self.btn_clear = QPushButton("  Tyhjenna keskustelu")
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

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        self.settings_panel = SettingsPanel(self._config)
        main_layout.addWidget(self.settings_panel)

        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.setChildrenCollapsible(False)

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
        self.btn_send = QPushButton("Laheta")
        self.btn_send.setMinimumHeight(32)
        self.btn_send.setStyleSheet(_BTN_GREEN)
        input_row.addWidget(self.text_input)
        input_row.addWidget(self.btn_send)
        chat_layout.addLayout(input_row)
        right_splitter.addWidget(chat_container)

        log_container = QWidget()
        log_layout = QVBoxLayout(log_container)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_label = QLabel("Jarjestelmaloki")
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
        self._bridge.chat_message.connect(
            lambda role, text: append_chat(self.chat_display, role, text),
            queued,
        )

    def _set_connected_state(self, connected: bool) -> None:
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_send.setEnabled(connected)
        self.text_input.setEnabled(connected)

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

    def _on_save_settings(self) -> None:
        cfg = Config.load()
        self.settings_panel.write_to_config(cfg)
        saved_to = cfg.save()
        self._config = cfg
        append_log(self.log_display, f"Asetukset tallennettu: {saved_to}")

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


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
