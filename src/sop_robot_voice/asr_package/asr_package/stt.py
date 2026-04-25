"""Configuration-selected speech-to-text backend facade."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from voice_stack_common.config import VoiceChatConfig

from .backends import (
    CrispAsrTranscriber,
    FasterWhisperTranscriber,
    PyWhisperCppTranscriber,
    Transcriber,
    resolve_crispasr_lib_path,
    resolve_crispasr_model_path,
    resolve_whisper_cpp_model_path,
)


class SpeechToText:
    """Configuration-selected speech-to-text engine."""

    def __init__(self, config: VoiceChatConfig):
        backend = normalize_whisper_backend(config.whisper_backend)
        try:
            backend_factory = ASR_BACKENDS[backend]
        except KeyError:
            raise ValueError(
                "Unsupported whisper_backend "
                f"{config.whisper_backend!r}; use 'faster-whisper', "
                "'pywhispercpp', or 'crispasr'."
            ) from None
        self._impl: Transcriber = backend_factory(config)

    def transcribe(self, audio_int16: np.ndarray) -> str:
        return self._impl.transcribe(audio_int16)

    def close(self) -> None:
        close = getattr(self._impl, "close", None)
        if close is not None:
            close()


def normalize_whisper_backend(raw_backend: str) -> str:
    normalized = raw_backend.strip().lower().replace("_", "-")
    aliases = {
        "faster": "faster-whisper",
        "fasterwhisper": "faster-whisper",
        "crisp": "crispasr",
        "crisp-asr": "crispasr",
        "parakeet": "crispasr",
        "whisper.cpp": "pywhispercpp",
        "whispercpp": "pywhispercpp",
        "whisper-cpp": "pywhispercpp",
        "py-whisper-cpp": "pywhispercpp",
        "pywhisper-cpp": "pywhispercpp",
    }
    return aliases.get(normalized, normalized)


ASR_BACKENDS: dict[str, Callable[[VoiceChatConfig], Transcriber]] = {
    "crispasr": CrispAsrTranscriber,
    "faster-whisper": FasterWhisperTranscriber,
    "pywhispercpp": PyWhisperCppTranscriber,
}


__all__ = [
    "ASR_BACKENDS",
    "CrispAsrTranscriber",
    "FasterWhisperTranscriber",
    "PyWhisperCppTranscriber",
    "SpeechToText",
    "Transcriber",
    "normalize_whisper_backend",
    "resolve_crispasr_lib_path",
    "resolve_crispasr_model_path",
    "resolve_whisper_cpp_model_path",
]
