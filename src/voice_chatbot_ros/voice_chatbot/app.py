"""
Voice Chatbot – standalone PySide6 desktop GUI.

Loads all pipeline models in-process via a background QThread and runs
the audio loop there so the UI stays responsive.

Usage::

    pixi run app
    # or: python -m voice_chatbot.app
"""

import sys
import traceback
from pathlib import Path

from .platform_setup import setup_cuda, setup_pyside6

setup_pyside6()
setup_cuda()

from PySide6.QtCore import QObject, Qt, QThread, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
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
from .ui_common import (
    APP_STYLESHEET,
    LogStream,
    SettingsPanel,
    append_chat,
    append_log,
    update_status_label,
)

# ── Background worker ─────────────────────────────────────────────


class ChatbotWorker(QThread):
    """Loads all pipeline models and runs the audio loop in a background thread."""

    log = Signal(str)
    chat_message = Signal(str, str)  # (role, text)
    status_changed = Signal(str)
    error_occurred = Signal(str)
    models_ready = Signal()

    def __init__(self, config: Config, parent: QObject | None = None):
        super().__init__(parent)
        self._config = config
        self._running = False

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        self._running = True

        # Redirect stdout/stderr so library prints appear in the log panel
        log_stream = LogStream()
        log_stream.message.connect(self.log.emit, Qt.ConnectionType.QueuedConnection)
        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout = log_stream
        sys.stderr = log_stream

        audio = None
        try:
            self._emit_system_info()
            self.status_changed.emit("Ladataan malleja...")

            self.log.emit("[Init] Audio I/O...")
            from .audio_io import AudioIO

            audio = AudioIO(self._config)

            self.log.emit("[Init] Silero-VAD...")
            from .vad import VoiceActivityDetector

            vad = VoiceActivityDetector(self._config)

            self.log.emit("[Init] Whisper STT...")
            from .stt import SpeechToText

            stt = SpeechToText(self._config)

            self.log.emit("[Init] LLM...")
            from .llm import ChatLLM

            llm = ChatLLM(self._config)

            self.log.emit("[Init] Coqui TTS...")
            from .tts_engine import TextToSpeech

            tts = TextToSpeech(self._config)

            self.log.emit("Kaikki mallit ladattu onnistuneesti!")
            self.models_ready.emit()
            self.status_changed.emit("Kuunnellaan...")

            # ── Audio loop ──
            audio.start_capture()
            while self._running:
                chunk = audio.get_audio_chunk(timeout=0.1)
                if chunk is None:
                    continue

                event, audio_data = vad.process_chunk(chunk)

                if event == "speech_start":
                    self.status_changed.emit("Puhe havaittu...")
                elif event == "speech_end" and audio_data is not None:
                    self.status_changed.emit("Käsitellään...")
                    text = stt.transcribe(audio_data)
                    if not text or text.isspace():
                        self.status_changed.emit("Kuunnellaan...")
                        continue

                    self.chat_message.emit("user", text)

                    self.status_changed.emit("LLM vastaa...")
                    response = llm.chat(text)
                    self.chat_message.emit("assistant", response)

                    self.status_changed.emit("Puhutaan...")
                    audio_out, sr = tts.synthesize(response)
                    audio.play_audio(audio_out, sr)

                    audio.clear_queue()
                    vad.reset()
                    self.status_changed.emit("Kuunnellaan...")

        except Exception:
            self.error_occurred.emit(traceback.format_exc())
        finally:
            if audio is not None:
                audio.close()
            sys.stdout = old_stdout
            sys.stderr = old_stderr
            self.status_changed.emit("Pysäytetty")

    def _emit_system_info(self) -> None:
        try:
            import torch

            if torch.cuda.is_available():
                name = torch.cuda.get_device_name(0)
                vram = torch.cuda.get_device_properties(0).total_memory / (1024**3)
                self.log.emit(f"GPU: {name} ({vram:.1f} GB)")
                self.log.emit(f"CUDA: {torch.version.cuda}")
            else:
                self.log.emit("VAROITUS: CUDA ei saatavilla – mallit ajetaan CPU:lla.")
        except ImportError:
            self.log.emit("VAROITUS: PyTorch ei asennettu.")


# ── Main window ───────────────────────────────────────────────────

# Toolbar button styles
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
_BTN_ORANGE = (
    "QPushButton { background-color: #e65100; color: white; "
    "font-weight: bold; padding: 4px 16px; border-radius: 4px; }"
    "QPushButton:hover { background-color: #ef6c00; }"
    "QPushButton:disabled { background-color: #666; }"
)


class MainWindow(QMainWindow):
    """Top-level standalone GUI window."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Äänichatbot")
        self.resize(1100, 720)

        self._config = Config.load()
        self._worker: ChatbotWorker | None = None
        self._pending_restart = False

        self._build_ui()
        self._connect_signals()
        self._set_running_state(False)

    def _build_ui(self) -> None:
        # ── Toolbar ──
        toolbar = self.addToolBar("Hallinta")
        toolbar.setMovable(False)

        self.btn_start = QPushButton("  Käynnistä")
        self.btn_start.setMinimumHeight(32)
        self.btn_start.setStyleSheet(_BTN_GREEN)
        toolbar.addWidget(self.btn_start)

        self.btn_stop = QPushButton("  Pysäytä")
        self.btn_stop.setMinimumHeight(32)
        self.btn_stop.setStyleSheet(_BTN_RED)
        toolbar.addWidget(self.btn_stop)

        self.btn_restart = QPushButton("  Käynnistä uudelleen")
        self.btn_restart.setMinimumHeight(32)
        self.btn_restart.setToolTip(
            "Pysäytä ja käynnistä uudelleen uusilla asetuksilla"
        )
        self.btn_restart.setStyleSheet(_BTN_ORANGE)
        toolbar.addWidget(self.btn_restart)

        self.btn_clear = QPushButton("  Tyhjennä keskustelu")
        self.btn_clear.setMinimumHeight(32)
        self.btn_clear.setStyleSheet(
            "QPushButton { padding: 4px 12px; border-radius: 4px; }"
        )
        toolbar.addWidget(self.btn_clear)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        self.label_status = QLabel("Tila: Valmis")
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

        # Chat panel
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

        self.statusBar().showMessage("Valmis")

    def _connect_signals(self) -> None:
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_restart.clicked.connect(self._on_restart)
        self.btn_clear.clicked.connect(self._on_clear)

    def _set_running_state(self, running: bool) -> None:
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        self.btn_restart.setEnabled(running)
        self.settings_panel.setEnabled(True)

    def _build_config_from_ui(self) -> Config:
        cfg = Config()
        self.settings_panel.write_to_config(cfg)
        cfg.save()
        return cfg

    # ── Slots ──

    def _on_start(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(5000)
            self._worker = None

        self._config = self._build_config_from_ui()

        if not Path(self._config.llm_model_path).exists():
            append_log(
                self.log_display,
                f"VIRHE: LLM-mallia ei löydy: {self._config.llm_model_path}",
            )
            append_log(
                self.log_display,
                "Suorita ensin: pixi run setup-models",
            )
            return

        self._set_running_state(True)
        self.log_display.clear()
        append_log(self.log_display, "Käynnistetään...")

        self._worker = ChatbotWorker(self._config)
        Q = Qt.ConnectionType.QueuedConnection
        self._worker.log.connect(lambda t: append_log(self.log_display, t), Q)
        self._worker.chat_message.connect(
            lambda r, t: append_chat(self.chat_display, r, t), Q
        )
        self._worker.status_changed.connect(
            lambda t: update_status_label(self.label_status, self.statusBar(), t), Q
        )
        self._worker.error_occurred.connect(self._on_error, Q)
        self._worker.models_ready.connect(
            lambda: append_log(self.log_display, "Kaikki mallit valmiina."), Q
        )
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.start()

    def _on_stop(self) -> None:
        if self._worker is not None:
            append_log(self.log_display, "Pysäytetään...")
            self._worker.stop()
            self.btn_stop.setEnabled(False)
            self.btn_restart.setEnabled(False)

    def _on_restart(self) -> None:
        append_log(self.log_display, "Käynnistetään uudelleen uusilla asetuksilla...")
        self._pending_restart = True
        if self._worker is not None:
            self._worker.stop()
        else:
            self._on_start()

    def _on_clear(self) -> None:
        self.chat_display.clear()

    def _on_worker_finished(self) -> None:
        self._worker = None
        if self._pending_restart:
            self._pending_restart = False
            self._on_start()
            return
        self._set_running_state(False)
        update_status_label(self.label_status, self.statusBar(), "Pysäytetty")
        append_log(self.log_display, "Chatbot pysäytetty.")

    def _on_error(self, error_text: str) -> None:
        append_log(self.log_display, f"VIRHE:\n{error_text}")
        update_status_label(self.label_status, self.statusBar(), "Virhe")

    def closeEvent(self, event) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(5000)
        event.accept()


# ── Entry point ───────────────────────────────────────────────────


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
