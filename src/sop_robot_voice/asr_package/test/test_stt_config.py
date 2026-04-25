import sys
from pathlib import Path

import numpy as np
import pytest


COMMON_ROOT = Path(__file__).resolve().parents[2] / "voice_stack_common"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
for path in (COMMON_ROOT, PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from asr_package.stt import (
    ASR_BACKENDS,
    SpeechToText,
    normalize_whisper_backend,
    resolve_crispasr_lib_path,
    resolve_crispasr_model_path,
)
from voice_stack_common.config import VoiceChatConfig


def test_whisper_backend_aliases_normalize():
    assert normalize_whisper_backend("faster_whisper") == "faster-whisper"
    assert normalize_whisper_backend("whisper.cpp") == "pywhispercpp"
    assert normalize_whisper_backend("whisper-cpp") == "pywhispercpp"
    assert normalize_whisper_backend("pywhispercpp") == "pywhispercpp"
    assert normalize_whisper_backend("crisp-asr") == "crispasr"
    assert normalize_whisper_backend("parakeet") == "crispasr"


def test_speech_to_text_selects_normalized_backend(monkeypatch):
    class StubTranscriber:
        def __init__(self, config):
            self.config = config

        def transcribe(self, audio_int16):
            assert audio_int16.dtype == np.int16
            return f"stub:{self.config.whisper_backend}"

    monkeypatch.setitem(ASR_BACKENDS, "faster-whisper", StubTranscriber)
    config = VoiceChatConfig(whisper_backend="faster_whisper")

    stt = SpeechToText(config)

    assert stt.transcribe(np.array([0], dtype=np.int16)) == "stub:faster_whisper"


def test_speech_to_text_rejects_unknown_backend():
    config = VoiceChatConfig(whisper_backend="unknown")

    with pytest.raises(ValueError, match="Unsupported whisper_backend"):
        SpeechToText(config)


def test_voice_config_exposes_pywhispercpp_fields():
    config = VoiceChatConfig(
        whisper_backend="pywhispercpp",
        whisper_cpp_model_filename="ggml-model-fi-tiny.bin",
        whisper_cpp_model_path="models/ggml-model-fi-tiny.bin",
    )

    assert config.whisper_backend == "pywhispercpp"
    assert config.whisper_cpp_repo_id == (
        "Finnish-NLP/Finnish-finetuned-whisper-models-ggml-format"
    )
    assert config.whisper_cpp_model_filename == "ggml-model-fi-tiny.bin"


def test_voice_config_exposes_crispasr_fields():
    config = VoiceChatConfig(
        whisper_backend="crispasr",
        crispasr_python_path="/opt/CrispASR/python",
        crispasr_lib_path="/opt/CrispASR/build/src/libcrispasr.so",
        crispasr_model_path="/opt/CrispASR/parakeet.gguf",
        crispasr_backend="parakeet",
    )

    assert config.whisper_backend == "crispasr"
    assert config.crispasr_python_path == "/opt/CrispASR/python"
    assert config.crispasr_lib_path == "/opt/CrispASR/build/src/libcrispasr.so"
    assert config.crispasr_model_path == "/opt/CrispASR/parakeet.gguf"
    assert config.crispasr_backend == "parakeet"


def test_resolve_crispasr_paths(tmp_path):
    root = tmp_path / "CrispASR"
    lib_dir = root / "build" / "src"
    lib_dir.mkdir(parents=True)
    lib_path = lib_dir / "libcrispasr.so"
    model_path = root / "parakeet.gguf"
    lib_path.write_text("library")
    model_path.write_text("model")
    config = VoiceChatConfig(
        crispasr_root=str(root),
        crispasr_lib_path=str(root / "missing.so"),
        crispasr_model_path=str(model_path),
    )

    assert resolve_crispasr_lib_path(config) == lib_path
    assert resolve_crispasr_model_path(config) == model_path
