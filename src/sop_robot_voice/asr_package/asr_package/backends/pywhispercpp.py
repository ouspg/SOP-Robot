"""pywhispercpp ASR backend."""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np

from voice_stack_common.config import VoiceChatConfig


class PyWhisperCppTranscriber:
    """Whisper transcription using pywhispercpp and GGML model files."""

    def __init__(self, config: VoiceChatConfig):
        from pywhispercpp.model import Model

        self._language = config.language
        self._threads = config.whisper_n_threads
        model_path = resolve_whisper_cpp_model_path(config)
        print(
            f"[STT] Loading pywhispercpp model '{model_path}' "
            f"(threads: {self._threads}, language: {self._language})..."
        )
        self._model = Model(
            str(model_path),
            n_threads=self._threads,
            redirect_whispercpp_logs_to=False,
        )
        print("[STT] pywhispercpp model loaded.")

    def transcribe(self, audio_int16: np.ndarray) -> str:
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        segments = self._model.transcribe(
            audio_float32,
            language=self._language,
            print_progress=False,
            print_realtime=False,
            print_timestamps=False,
        )
        return " ".join(segment.text for segment in segments).strip()


def resolve_whisper_cpp_model_path(config: VoiceChatConfig) -> Path:
    raw_configured_path = config.whisper_cpp_model_path.strip()
    configured_path = Path(raw_configured_path).expanduser()
    model_path = (
        configured_path
        if raw_configured_path
        else Path(config.models_dir).expanduser() / config.whisper_cpp_model_filename
    )
    if model_path.is_file():
        return model_path

    from huggingface_hub import hf_hub_download

    model_path.parent.mkdir(parents=True, exist_ok=True)
    print(
        "[STT] pywhispercpp model file missing; downloading "
        f"{config.whisper_cpp_repo_id}/{config.whisper_cpp_model_filename}..."
    )
    downloaded = Path(
        hf_hub_download(
            repo_id=config.whisper_cpp_repo_id,
            filename=config.whisper_cpp_model_filename,
            local_dir=str(model_path.parent),
        )
    )
    if downloaded.resolve() != model_path.resolve() and downloaded.is_file():
        shutil.copy2(downloaded, model_path)
    return model_path
