"""Shared PySide6 UI components for the ROS chatbot GUIs."""

from pathlib import Path

from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QTextCursor
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

STATUS_MAP = {
    "initializing": "Alustetaan...",
    "ready": "Valmis",
    "listening": "Kuunnellaan...",
    "speech_detected": "Puhe havaittu...",
    "transcribing": "Kasitellaan...",
    "llm_responding": "LLM vastaa...",
    "speaking": "Puhutaan...",
    "error": "Virhe",
}

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


def escape_html(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\n", "<br>")
    )


def set_combo_by_data(combo: QComboBox, data) -> None:
    for i in range(combo.count()):
        if combo.itemData(i) == data:
            combo.setCurrentIndex(i)
            return


def set_combo_by_text(combo: QComboBox, text: str) -> None:
    idx = combo.findText(text)
    if idx >= 0:
        combo.setCurrentIndex(idx)
    elif combo.isEditable():
        combo.setEditText(text)


def update_status_label(label: QLabel, statusbar, text: str) -> None:
    if "Kuunnellaan" in text or "Yhdistetty" in text:
        color = "#a6e3a1"
    elif any(key in text for key in ("Kasitellaan", "vastaa", "Puhutaan")):
        color = "#f9e2af"
    elif "Puhe havaittu" in text:
        color = "#89b4fa"
    elif "Virhe" in text:
        color = "#f38ba8"
    else:
        color = "#6c7086"
    label.setText(f'  Tila: {text} <span style="color: {color};">&#9679;</span>')
    statusbar.showMessage(text)


def append_log(log_display: QPlainTextEdit, text: str) -> None:
    log_display.appendPlainText(text)
    cursor = log_display.textCursor()
    cursor.movePosition(QTextCursor.MoveOperation.End)
    log_display.setTextCursor(cursor)


def append_chat(chat_display: QTextEdit, role: str, text: str) -> None:
    if role == "user":
        color, prefix = "#89b4fa", "Sina"
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


class LogStream(QObject):
    """Redirect for stdout/stderr that emits Qt signals."""

    message = Signal(str)

    def write(self, text: str) -> None:
        if text and text.strip():
            self.message.emit(text.rstrip("\n"))

    def flush(self) -> None:
        pass


class SettingsPanel(QScrollArea):
    """Left sidebar with editable model and pipeline settings."""

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

        grp_llm = QGroupBox("LLM-asetukset")
        form_llm = QFormLayout(grp_llm)

        self.spin_temperature = QDoubleSpinBox()
        self.spin_temperature.setRange(0.0, 2.0)
        self.spin_temperature.setSingleStep(0.1)
        self.spin_temperature.setDecimals(2)
        form_llm.addRow("Lampotila:", self.spin_temperature)

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
        self.edit_system_prompt.setPlaceholderText("Jarjestelmakehote...")
        form_llm.addRow("Jarjestelmakehote:", self.edit_system_prompt)
        layout.addWidget(grp_llm)

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
        form_vad.addRow("Puhetayte:", self.spin_speech_pad)

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
            "Aanta tallennetaan ennen puheen tunnistusta, jotta ensimmaiset "
            "tavut eivat leikkaudu pois."
        )
        form_vad.addRow("Esipuskuri:", self.spin_pre_buffer)
        layout.addWidget(grp_vad)

        grp_tts = QGroupBox("TTS-asetukset")
        form_tts = QFormLayout(grp_tts)
        self.chk_tts_enabled = QCheckBox("Puhesynteesi kaytossa")
        form_tts.addRow(self.chk_tts_enabled)
        self.chk_tts_gpu = QCheckBox("Kayta GPU:ta")
        form_tts.addRow(self.chk_tts_gpu)
        layout.addWidget(grp_tts)

        layout.addStretch()
        self.setWidget(container)

        self.load_from_config(config)

    def load_from_config(self, cfg: Config) -> None:
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

        self.chk_tts_enabled.setChecked(cfg.tts_enabled)
        self.chk_tts_gpu.setChecked(cfg.tts_gpu)

    def write_to_config(self, cfg: Config) -> Config:
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

        cfg.tts_enabled = self.chk_tts_enabled.isChecked()
        cfg.tts_gpu = self.chk_tts_gpu.isChecked()
        return cfg

    def _browse_llm(self) -> None:
        start_dir = str(Path(self.edit_llm_path.text()).parent)
        if not Path(start_dir).is_dir():
            start_dir = "models"
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Valitse GGUF-malli",
            start_dir,
            "GGUF-mallit (*.gguf);;Kaikki (*)",
        )
        if path:
            self.edit_llm_path.setText(path)
