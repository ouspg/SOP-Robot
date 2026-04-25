"""Shared JSON-backed configuration for the SOP Robot voice stack."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, fields
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - optional outside ROS env
    get_package_share_directory = None

CONFIG_ENV_VAR = "SOP_ROBOT_VOICE_CONFIG"
DEFAULT_CONFIG_BASENAME = "voice_chatbot.json"
DEFAULTS_BASENAME = "voice_chatbot.defaults.json"


def _discover_repo_root() -> Path | None:
    env_root = os.environ.get("SOP_ROBOT_ROOT")
    if env_root:
        candidate = Path(env_root).expanduser()
        if (candidate / "pixi.toml").is_file():
            return candidate

    for base in [Path.cwd(), *Path.cwd().parents]:
        if (base / "pixi.toml").is_file() and (base / "src").is_dir():
            return base

    return None


def default_config_path() -> Path:
    override = os.environ.get(CONFIG_ENV_VAR)
    if override:
        return Path(override).expanduser()

    repo_root = _discover_repo_root()
    if repo_root is not None:
        return repo_root / "config" / DEFAULT_CONFIG_BASENAME

    return Path.home() / ".config" / "sop-robot" / DEFAULT_CONFIG_BASENAME


def default_defaults_path() -> Path | None:
    repo_root = _discover_repo_root()
    if repo_root is not None:
        candidate = repo_root / "config" / DEFAULTS_BASENAME
        if candidate.is_file():
            return candidate

    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory("voice_stack_common"))
            candidate = share_dir / "config" / DEFAULTS_BASENAME
            if candidate.is_file():
                return candidate
        except Exception:
            pass

    return None


def resolve_config_path(path: str | Path | None = None) -> Path:
    return default_config_path() if path is None else Path(path).expanduser()


@dataclass
class VoiceChatConfig:
    sample_rate: int = 16000
    channels: int = 1
    chunk_samples: int = 512
    vad_threshold: float = 0.45
    min_silence_duration_ms: int = 550
    speech_pad_ms: int = 200
    min_speech_duration_ms: int = 400
    vad_pre_buffer_ms: int = 500

    language: str = "fi"

    whisper_backend: str = "faster-whisper"
    whisper_model: str = "medium"
    whisper_n_threads: int = 4
    whisper_gpu: bool = True
    whisper_cpp_repo_id: str = "Finnish-NLP/Finnish-finetuned-whisper-models-ggml-format"
    whisper_cpp_model_filename: str = "ggml-model-fi-medium.bin"
    whisper_cpp_model_path: str = "models/ggml-model-fi-medium.bin"
    crispasr_root: str = "/home/aapot/CrispASR"
    crispasr_python_path: str = "/home/aapot/CrispASR/python"
    crispasr_lib_path: str = "/home/aapot/CrispASR/build/src/libcrispasr.so"
    crispasr_model_path: str = "/home/aapot/CrispASR/parakeet.gguf"
    crispasr_backend: str = "parakeet"

    models_dir: str = "models"
    llm_backend: str = "llama-cpp"
    llm_model_path: str = "models/Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf"
    llm_n_gpu_layers: int = -1
    llm_n_ctx: int = 2048
    llm_max_tokens: int = 256
    llm_temperature: float = 0.7
    llm_system_prompt: str = (
        "You are a Finnish-speaking voice assistant. You MUST ALWAYS respond "
        "in Finnish (suomi). NEVER respond in English or any other language. "
        "Keep your responses concise and conversational (2-3 sentences), "
        "suitable for spoken dialogue. If the user's input is unclear, ask "
        "them to repeat in Finnish. Olet suomenkielinen aaniassistentti. "
        "Vastaa aina suomeksi, lyhyesti ja selkeasti."
    )
    max_conversation_turns: int = 20
    llm_knowledge_base_enabled: bool = True
    llm_knowledge_base_data_dir: str = "legacy/chatbot/chatbot/data"
    llm_knowledge_base_path: str = "models/sop_robot_llm_knowledge_base.sqlite3"
    llm_knowledge_base_recreate: bool = False
    llm_knowledge_base_search_limit: int = 3
    llm_knowledge_base_direct_answer: bool = True

    tts_model: str = "tts_models/fi/css10/vits"
    tts_model_path: str = "models/model.pth"
    tts_config_path: str = "models/config.json"
    tts_gpu: bool = True
    tts_enabled: bool = True

    def save(self, path: str | Path | None = None) -> Path:
        target = resolve_config_path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = target.with_suffix(target.suffix + ".tmp")
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(asdict(self), handle, indent=2, ensure_ascii=False)
        tmp_path.replace(target)
        return target

    @classmethod
    def load(
        cls,
        path: str | Path | None = None,
        defaults_path: str | Path | None = None,
    ) -> "VoiceChatConfig":
        valid_fields = {field.name for field in fields(cls)}

        resolved_path = resolve_config_path(path)
        for candidate in [
            resolved_path,
            Path(defaults_path).expanduser() if defaults_path else None,
            default_defaults_path(),
        ]:
            if candidate is None:
                continue
            try:
                with open(candidate, "r", encoding="utf-8") as handle:
                    data = json.load(handle)
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                continue

            filtered = {key: value for key, value in data.items() if key in valid_fields}
            return cls(**filtered)

        return cls()
