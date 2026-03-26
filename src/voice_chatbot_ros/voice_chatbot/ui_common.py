"""
Shared PySide6 UI components used by both ``app.py`` and ``ros_app.py``.

Consolidates constants, utility functions, the settings panel, and the
log-stream helper so that both GUI variants stay in sync.
"""

from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QFont, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .config import Config

# ── Model / language constants ────────────────────────────────────

WHISPER_MODELS = [
    "tiny",
    "tiny.en",
    "base",
    "base.en",
    "small",
    "small.en",
    "medium",
    "medium.en",
    "large-v1",
    "large-v2",
    "large-v3",
]

LANGUAGES = {
    "Suomi (fi)": "fi",
    "English (en)": "en",
    "Auto": "auto",
    "Svenska (sv)": "sv",
    "Deutsch (de)": "de",
    "Francais (fr)": "fr",
    "Espanol (es)": "es",
    "Italiano (it)": "it",
    "Portugues (pt)": "pt",
    "Nederlands (nl)": "nl",
    "Polski (pl)": "pl",
    "Japanese (ja)": "ja",
    "Chinese (zh)": "zh",
    "Korean (ko)": "ko",
    "Russian (ru)": "ru",
}

TTS_MODELS = [
    "tts_models/fi/css10/vits",
    "tts_models/en/ljspeech/tacotron2-DDC",
    "tts_models/en/ljspeech/vits",
    "tts_models/en/vctk/vits",
    "tts_models/de/thorsten/tacotron2-DDC",
    "tts_models/de/thorsten/vits",
    "tts_models/fr/mai/tacotron2-DDC",
    "tts_models/es/mai/tacotron2-DDC",
    "tts_models/nl/mai/tacotron2-DDC",
    "tts_models/ja/kokoro/tacotron2-DDC",
    "tts_models/zh-CN/baker/tacotron2-DDC-GST",
    "tts_models/multilingual/multi-dataset/xtts_v2",
]

# Map ROS status strings to Finnish UI labels.
STATUS_MAP = {
    "initializing": "Alustetaan...",
    "ready": "Valmis",
    "listening": "Kuunnellaan...",
    "speech_detected": "Puhe havaittu...",
    "transcribing": "Käsitellään...",
    "llm_responding": "LLM vastaa...",
    "speaking": "Puhutaan...",
    "error": "Virhe",
}

# Global Qt stylesheet applied by both GUI variants.
APP_STYLESHEET = """
    QMainWindow, QWidget { font-family: 'Segoe UI', sans-serif; }
    QGroupBox {
        font-weight: bold;
        border: 1px solid #555;
        border-radius: 4px;
        margin-top: 8px;
        padding-top: 14px;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 8px;
        padding: 0 4px;
    }
    QToolBar { spacing: 6px; padding: 4px; }
"""

# ── Utility functions ─────────────────────────────────────────────


def escape_html(text: str) -> str:
    """Escape HTML special characters and convert newlines to ``<br>``."""
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\n", "<br>")
    )


def set_combo_by_data(combo: QComboBox, data) -> None:
    """Select the combo item whose ``itemData`` matches *data*."""
    for i in range(combo.count()):
        if combo.itemData(i) == data:
            combo.setCurrentIndex(i)
            return


def set_combo_by_text(combo: QComboBox, text: str) -> None:
    """Select the combo item matching *text*, or set edit text if editable."""
    idx = combo.findText(text)
    if idx >= 0:
        combo.setCurrentIndex(idx)
    elif combo.isEditable():
        combo.setEditText(text)


def update_status_label(label: QLabel, statusbar, text: str) -> None:
    """Set the toolbar status label with a coloured indicator dot."""
    if "Kuunnellaan" in text or "Yhdistetty" in text:
        color = "#a6e3a1"  # green
    elif any(k in text for k in ("Käsitellään", "vastaa", "Puhutaan")):
        color = "#f9e2af"  # yellow
    elif "Puhe havaittu" in text:
        color = "#89b4fa"  # blue
    elif "Virhe" in text:
        color = "#f38ba8"  # red
    else:
        color = "#6c7086"  # grey
    label.setText(f'  Tila: {text} <span style="color: {color};">&#9679;</span>')
    statusbar.showMessage(text)


def append_log(log_display: QPlainTextEdit, text: str) -> None:
    """Append text to a log panel and auto-scroll to the bottom."""
    log_display.appendPlainText(text)
    cursor = log_display.textCursor()
    cursor.movePosition(QTextCursor.MoveOperation.End)
    log_display.setTextCursor(cursor)


def append_chat(chat_display: QTextEdit, role: str, text: str) -> None:
    """Append a chat message with role-coloured prefix and auto-scroll."""
    if role == "user":
        color, prefix = "#89b4fa", "Sinä"
    else:
        color, prefix = "#a6e3a1", "Botti"
    html = (
        f'<p style="margin: 6px 0;">'
        f'<span style="color: {color}; font-weight: bold;">{prefix}:</span> '
        f'<span style="color: #cdd6f4;">{escape_html(text)}</span>'
        f"</p>"
    )
    chat_display.append(html)
    cursor = chat_display.textCursor()
    cursor.movePosition(QTextCursor.MoveOperation.End)
    chat_display.setTextCursor(cursor)


# ── LogStream ─────────────────────────────────────────────────────


class LogStream(QObject):
    """Redirect for ``sys.stdout`` / ``sys.stderr`` that emits Qt signals."""

    message = Signal(str)

    def write(self, text: str) -> None:
        if text and text.strip():
            self.message.emit(text.rstrip("\n"))

    def flush(self) -> None:
        pass


# ── Settings panel ────────────────────────────────────────────────


class SettingsPanel(QScrollArea):
    """Left sidebar with all editable model and pipeline settings.

    Shared by both the standalone and ROS 2 GUI.  Each widget maps 1:1
    to a :class:`Config` field.
    """

    def __init__(self, config: Config, parent: QWidget | None = None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setMinimumWidth(310)
        self.setMaximumWidth(420)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)

        # ── Language & models ──
        grp_models = QGroupBox("Kieli ja mallit")
        form_models = QFormLayout(grp_models)

        self.combo_language = QComboBox()
        for label, code in LANGUAGES.items():
            self.combo_language.addItem(label, code)
        form_models.addRow("Kieli:", self.combo_language)

        self.combo_whisper = QComboBox()
        self.combo_whisper.addItems(WHISPER_MODELS)
        form_models.addRow("Whisper-malli:", self.combo_whisper)

        llm_row = QHBoxLayout()
        self.edit_llm_path = QLineEdit()
        self.edit_llm_path.setPlaceholderText("polku .gguf-tiedostoon")
        self.btn_browse_llm = QPushButton("...")
        self.btn_browse_llm.setFixedWidth(32)
        self.btn_browse_llm.setToolTip("Selaa GGUF-tiedostoja")
        self.btn_browse_llm.clicked.connect(self._browse_llm)
        llm_row.addWidget(self.edit_llm_path)
        llm_row.addWidget(self.btn_browse_llm)
        form_models.addRow("LLM-malli:", llm_row)

        self.combo_tts = QComboBox()
        self.combo_tts.setEditable(True)
        self.combo_tts.addItems(TTS_MODELS)
        form_models.addRow("TTS-malli:", self.combo_tts)

        layout.addWidget(grp_models)

        # ── LLM settings ──
        grp_llm = QGroupBox("LLM-asetukset")
        form_llm = QFormLayout(grp_llm)

        self.spin_temperature = QDoubleSpinBox()
        self.spin_temperature.setRange(0.0, 2.0)
        self.spin_temperature.setSingleStep(0.1)
        self.spin_temperature.setDecimals(2)
        form_llm.addRow("Lämpötila:", self.spin_temperature)

        self.spin_max_tokens = QSpinBox()
        self.spin_max_tokens.setRange(32, 4096)
        self.spin_max_tokens.setSingleStep(32)
        form_llm.addRow("Max tokenit:", self.spin_max_tokens)

        self.spin_ctx = QSpinBox()
        self.spin_ctx.setRange(512, 131072)
        self.spin_ctx.setSingleStep(512)
        form_llm.addRow("Konteksti (n_ctx):", self.spin_ctx)

        self.spin_gpu_layers = QSpinBox()
        self.spin_gpu_layers.setRange(-1, 200)
        self.spin_gpu_layers.setSpecialValueText("Kaikki (-1)")
        form_llm.addRow("GPU-tasot:", self.spin_gpu_layers)

        self.spin_turns = QSpinBox()
        self.spin_turns.setRange(1, 100)
        form_llm.addRow("Max keskusteluvuorot:", self.spin_turns)

        self.edit_system_prompt = QPlainTextEdit()
        self.edit_system_prompt.setFixedHeight(100)
        self.edit_system_prompt.setPlaceholderText("Järjestelmäkehote...")
        form_llm.addRow("Järjestelmäkehote:", self.edit_system_prompt)

        layout.addWidget(grp_llm)

        # ── VAD settings ──
        grp_vad = QGroupBox("VAD-asetukset")
        form_vad = QFormLayout(grp_vad)

        self.spin_vad_threshold = QDoubleSpinBox()
        self.spin_vad_threshold.setRange(0.05, 1.0)
        self.spin_vad_threshold.setSingleStep(0.05)
        self.spin_vad_threshold.setDecimals(2)
        form_vad.addRow("Kynnysarvo:", self.spin_vad_threshold)

        self.spin_silence_ms = QSpinBox()
        self.spin_silence_ms.setRange(100, 5000)
        self.spin_silence_ms.setSingleStep(50)
        self.spin_silence_ms.setSuffix(" ms")
        form_vad.addRow("Hiljaisuus (min):", self.spin_silence_ms)

        self.spin_speech_pad = QSpinBox()
        self.spin_speech_pad.setRange(0, 500)
        self.spin_speech_pad.setSingleStep(10)
        self.spin_speech_pad.setSuffix(" ms")
        form_vad.addRow("Puhetäyte:", self.spin_speech_pad)

        self.spin_min_speech = QSpinBox()
        self.spin_min_speech.setRange(50, 5000)
        self.spin_min_speech.setSingleStep(50)
        self.spin_min_speech.setSuffix(" ms")
        form_vad.addRow("Puhe (min):", self.spin_min_speech)

        self.spin_pre_buffer = QSpinBox()
        self.spin_pre_buffer.setRange(100, 2000)
        self.spin_pre_buffer.setSingleStep(50)
        self.spin_pre_buffer.setSuffix(" ms")
        self.spin_pre_buffer.setToolTip(
            "Ääntä tallennetaan ennen puheen tunnistusta, "
            "jotta ensimmäiset tavut eivät leikkaudu pois."
        )
        form_vad.addRow("Esipuskuri:", self.spin_pre_buffer)

        layout.addWidget(grp_vad)

        # ── TTS extra ──
        grp_tts = QGroupBox("TTS-asetukset")
        form_tts = QFormLayout(grp_tts)
        self.chk_tts_gpu = QCheckBox("Käytä GPU:ta")
        form_tts.addRow(self.chk_tts_gpu)
        layout.addWidget(grp_tts)

        layout.addStretch()
        self.setWidget(container)

        self.load_from_config(config)

    # ── Config ↔ widgets ──

    def load_from_config(self, cfg: Config) -> None:
        """Populate all widgets from a :class:`Config` instance."""
        set_combo_by_data(self.combo_language, cfg.language)
        set_combo_by_text(self.combo_whisper, cfg.whisper_model)
        self.edit_llm_path.setText(cfg.llm_model_path)
        set_combo_by_text(self.combo_tts, cfg.tts_model)

        self.spin_temperature.setValue(cfg.llm_temperature)
        self.spin_max_tokens.setValue(cfg.llm_max_tokens)
        self.spin_ctx.setValue(cfg.llm_n_ctx)
        self.spin_gpu_layers.setValue(cfg.llm_n_gpu_layers)
        self.spin_turns.setValue(cfg.max_conversation_turns)
        self.edit_system_prompt.setPlainText(cfg.llm_system_prompt)

        self.spin_vad_threshold.setValue(cfg.vad_threshold)
        self.spin_silence_ms.setValue(cfg.min_silence_duration_ms)
        self.spin_speech_pad.setValue(cfg.speech_pad_ms)
        self.spin_min_speech.setValue(cfg.min_speech_duration_ms)
        self.spin_pre_buffer.setValue(cfg.vad_pre_buffer_ms)

        self.chk_tts_gpu.setChecked(cfg.tts_gpu)

    def write_to_config(self, cfg: Config) -> Config:
        """Read all widget values back into *cfg* and return it."""
        cfg.language = self.combo_language.currentData()
        cfg.whisper_model = self.combo_whisper.currentText()
        cfg.llm_model_path = self.edit_llm_path.text()
        cfg.tts_model = self.combo_tts.currentText()

        cfg.llm_temperature = self.spin_temperature.value()
        cfg.llm_max_tokens = self.spin_max_tokens.value()
        cfg.llm_n_ctx = self.spin_ctx.value()
        cfg.llm_n_gpu_layers = self.spin_gpu_layers.value()
        cfg.max_conversation_turns = self.spin_turns.value()
        cfg.llm_system_prompt = self.edit_system_prompt.toPlainText()

        cfg.vad_threshold = self.spin_vad_threshold.value()
        cfg.min_silence_duration_ms = self.spin_silence_ms.value()
        cfg.speech_pad_ms = self.spin_speech_pad.value()
        cfg.min_speech_duration_ms = self.spin_min_speech.value()
        cfg.vad_pre_buffer_ms = self.spin_pre_buffer.value()

        cfg.tts_gpu = self.chk_tts_gpu.isChecked()
        return cfg

    # ── Slots ──

    def _browse_llm(self) -> None:
        start_dir = str(Path(self.edit_llm_path.text()).parent)
        if not Path(start_dir).is_dir():
            start_dir = "models"
        path, _ = QFileDialog.getOpenFileName(
            self, "Valitse GGUF-malli", start_dir, "GGUF-mallit (*.gguf);;Kaikki (*)"
        )
        if path:
            self.edit_llm_path.setText(path)
