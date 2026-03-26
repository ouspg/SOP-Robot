"""
Runtime configuration for the voice chatbot pipeline.

All settings are stored in a single ``Config`` dataclass that can be
serialised to / loaded from a JSON file (default: ``config.json``).
The GUI reads and writes this file; the CLI and ROS 2 nodes load it
at startup.

Sections
--------
- **Audio** – sample rate, channels, and chunk size for ``sounddevice``.
- **VAD** – Silero-VAD thresholds, silence/speech durations, pre-buffer.
- **Language** – ISO 639-1 code shared by STT, LLM, and TTS.
- **Whisper** – model size, threading, and GPU toggle.
- **LLM** – GGUF model path, GPU offloading, context window, generation
  parameters, system prompt, and conversation-turn limit.
- **TTS** – Coqui TTS model identifier or local model path, GPU toggle.
- **HuggingFace** – repo coordinates used by ``setup_models.py`` to
  download the GGUF model on first run.
"""

import json
from dataclasses import asdict, dataclass, fields

# Default path for the JSON configuration file (project root).
CONFIG_FILE = "config.json"


@dataclass
class Config:
    """Central configuration dataclass for every pipeline component.

    Default values match a Finnish-language setup with GPU acceleration.
    The GUI's ``SettingsPanel`` maps every widget to one of these fields
    and calls :meth:`save` when the user presses *Start* or *Save*.

    The :meth:`load` classmethod silently ignores unknown keys in the
    JSON file so that old config files remain forward-compatible when
    new fields are added.
    """

    # ── Audio settings ────────────────────────────────────────────
    sample_rate: int = 16000  # Hz – matches Whisper's expected input
    channels: int = 1  # mono microphone capture
    chunk_samples: int = 512  # ≈ 32 ms at 16 kHz (Silero VAD window)

    # ── VAD (Voice Activity Detection) settings ───────────────────
    vad_threshold: float = 0.45  # Silero confidence threshold (0–1)
    min_silence_duration_ms: int = 550  # silence required to end an utterance
    speech_pad_ms: int = 200  # extra audio kept around speech edges
    min_speech_duration_ms: int = 400  # ignore utterances shorter than this
    vad_pre_buffer_ms: int = 500  # audio kept before VAD triggers
    # The pre-buffer prevents the first syllable from being clipped
    # when Silero detects speech slightly after it has already begun.

    # ── Language setting (used across STT, LLM, and TTS) ─────────
    language: str = "fi"  # ISO 639-1 code; "fi" = Finnish

    # ── Whisper STT settings ──────────────────────────────────────
    whisper_model: str = "medium"  # tiny / base / small / medium / large-v*
    whisper_n_threads: int = 4  # CPU threads for CTranslate2
    whisper_gpu: bool = True  # use CUDA if available

    # ── LLM (Large Language Model) settings ───────────────────────
    models_dir: str = "models"  # directory for downloaded models
    llm_model_path: str = "models/Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf"
    llm_n_gpu_layers: int = -1  # -1 = offload ALL layers to GPU
    llm_n_ctx: int = 2048  # context window size (tokens)
    llm_max_tokens: int = 256  # max tokens per assistant reply
    llm_temperature: float = 0.7  # sampling temperature (0 = greedy)
    llm_system_prompt: str = (
        # Bilingual system prompt: English instructions followed by a
        # Finnish summary, to reinforce Finnish-only responses.
        "You are a Finnish-speaking voice assistant. You MUST ALWAYS respond "
        "in Finnish (suomi). NEVER respond in English or any other language. "
        "Keep your responses concise and conversational (2-3 sentences), "
        "suitable for spoken dialogue. If the user's input is unclear, ask "
        "them to repeat in Finnish. "
        "Olet suomenkielinen ääniassistentti. Vastaa aina suomeksi, lyhyesti "
        "ja selkeästi."
    )
    max_conversation_turns: int = 20  # oldest turn pairs are trimmed beyond this

    # ── TTS (Text-to-Speech) settings ─────────────────────────────
    tts_model: str = "tts_models/fi/css10/vits"  # Coqui model identifier
    tts_model_path: str = "models/model.pth"  # local VITS checkpoint
    tts_config_path: str = "models/config.json"  # local VITS config
    tts_gpu: bool = True  # use CUDA for TTS synthesis

    # ── HuggingFace model repo (used by setup_models.py) ─────────
    llm_repo_id: str = "bartowski/Meta-Llama-3.1-8B-Instruct-GGUF"
    llm_filename: str = "Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf"

    # ── Persistence ───────────────────────────────────────────────

    def save(self, path: str = CONFIG_FILE) -> None:
        """Serialise the current config to a JSON file.

        Args:
            path: Output file path.  Defaults to ``config.json``.
        """
        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self), f, indent=2, ensure_ascii=False)

    @classmethod
    def load(cls, path: str = CONFIG_FILE) -> "Config":
        """Load a config from a JSON file, falling back to defaults.

        Keys in the JSON that do not correspond to a dataclass field
        are silently discarded so that older config files still work
        after new fields are added.

        Args:
            path: Input file path.  Defaults to ``config.json``.

        Returns:
            A populated :class:`Config` instance.
        """
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            # Filter out any keys that are not declared fields
            valid_fields = {field.name for field in fields(cls)}
            filtered = {k: v for k, v in data.items() if k in valid_fields}
            return cls(**filtered)
        except (FileNotFoundError, json.JSONDecodeError):
            return cls()
