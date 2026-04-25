"""CrispASR backend."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import numpy as np

from voice_stack_common.config import VoiceChatConfig


class CrispAsrTranscriber:
    """Speech recognition using CrispASR's Python bindings."""

    def __init__(self, config: VoiceChatConfig):
        python_path = Path(config.crispasr_python_path).expanduser()
        if str(python_path) not in sys.path:
            sys.path.insert(0, str(python_path))

        crispasr_module = importlib.import_module("crispasr")
        Session = crispasr_module.Session

        self._language = config.language.strip() or None
        self._backend = config.crispasr_backend.strip() or None
        self._model_path = resolve_crispasr_model_path(config)
        self._lib_path = resolve_crispasr_lib_path(config)
        available_backends = Session.available_backends(lib_path=str(self._lib_path))
        if self._backend and self._backend not in available_backends:
            raise RuntimeError(
                f"CrispASR backend {self._backend!r} is not available in "
                f"{self._lib_path}. Available backends: {available_backends}"
            )

        print(
            f"[STT] Loading CrispASR model '{self._model_path}' "
            f"(backend: {self._backend or 'auto'}, threads: {config.whisper_n_threads}, "
            f"language: {self._language or 'auto'}, lib: {self._lib_path})..."
        )
        self._session = Session(
            str(self._model_path),
            lib_path=str(self._lib_path),
            n_threads=config.whisper_n_threads,
            backend=self._backend,
        )
        print(f"[STT] CrispASR model loaded with backend '{self._session.backend}'.")

    def transcribe(self, audio_int16: np.ndarray) -> str:
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        segments = self._session.transcribe(
            audio_float32,
            sample_rate=16000,
            language=self._language,
        )
        return " ".join(segment.text for segment in segments).strip()

    def close(self) -> None:
        self._session.close()


def resolve_crispasr_model_path(config: VoiceChatConfig) -> Path:
    model_path = Path(config.crispasr_model_path).expanduser()
    if not model_path.is_file():
        raise FileNotFoundError(f"CrispASR model file was not found: {model_path}")
    return model_path


def resolve_crispasr_lib_path(config: VoiceChatConfig) -> Path:
    configured_path = Path(config.crispasr_lib_path).expanduser()
    if configured_path.is_file():
        return configured_path

    root = Path(config.crispasr_root).expanduser()
    candidates = [
        root / "build" / "src" / "libcrispasr.so",
        root / "build" / "src" / "libwhisper.so",
        root / "build" / "src" / "libwhisper.so.1",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        f"CrispASR shared library was not found: {configured_path}. "
        f"Checked fallback paths under {root}."
    )
