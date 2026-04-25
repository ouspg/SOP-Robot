"""faster-whisper ASR backend."""

from __future__ import annotations

import inspect

import numpy as np

from voice_stack_common.config import VoiceChatConfig
from voice_stack_common.platform_setup import setup_cuda


class FasterWhisperTranscriber:
    """Whisper transcription using faster-whisper."""

    def __init__(self, config: VoiceChatConfig):
        setup_cuda()

        from faster_whisper import WhisperModel

        self._language = config.language
        use_gpu = False
        if config.whisper_gpu:
            try:
                import torch

                use_gpu = torch.cuda.is_available()
            except Exception:
                use_gpu = False
        device = "cuda" if use_gpu else "cpu"
        compute_type = "float16" if use_gpu else "int8"
        if config.whisper_gpu and not use_gpu:
            print("[STT] CUDA not available, falling back to CPU Whisper inference.")
        print(
            f"[STT] Loading faster-whisper model '{config.whisper_model}' on {device} "
            f"(compute_type: {compute_type}, language: {self._language})..."
        )
        model_kwargs = {
            "device": device,
            "cpu_threads": config.whisper_n_threads,
        }
        if "compute_type" in inspect.signature(WhisperModel).parameters:
            model_kwargs["compute_type"] = compute_type
        self._model = WhisperModel(config.whisper_model, **model_kwargs)
        print("[STT] faster-whisper model loaded.")

    def transcribe(self, audio_int16: np.ndarray) -> str:
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        segments, _ = self._model.transcribe(audio_float32, language=self._language)
        return " ".join(seg.text for seg in segments).strip()
