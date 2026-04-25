"""Voice activity detection using Silero VAD."""

# pyright: reportPrivateImportUsage=false

from __future__ import annotations

from collections import deque

from voice_stack_common.platform_setup import setup_cuda

setup_cuda()

import numpy as np
import torch
from silero_vad import VADIterator, load_silero_vad

from voice_stack_common.config import VoiceChatConfig

_ENERGY_FLOOR_RMS = 30


class VoiceActivityDetector:
    """Silero-VAD wrapper with energy gating and pre-buffering."""

    def __init__(self, config: VoiceChatConfig):
        model = load_silero_vad()
        self._vad_iterator = VADIterator(
            model,
            threshold=config.vad_threshold,
            sampling_rate=config.sample_rate,
            min_silence_duration_ms=config.min_silence_duration_ms,
            speech_pad_ms=config.speech_pad_ms,
        )
        self._sample_rate = config.sample_rate
        self._min_speech_samples = int(
            config.min_speech_duration_ms * config.sample_rate / 1000
        )

        pre_buf_chunks = max(
            1,
            int(
                config.vad_pre_buffer_ms
                * config.sample_rate
                / 1000
                / config.chunk_samples
            ),
        )
        self._pre_buffer: deque[np.ndarray] = deque(maxlen=pre_buf_chunks)
        self._audio_buffer: list[np.ndarray] = []
        self._is_speech = False

    def process_chunk(
        self, audio_chunk_int16: np.ndarray
    ) -> tuple[str | None, np.ndarray | None]:
        if not self._is_speech:
            rms = np.sqrt(np.mean(audio_chunk_int16.astype(np.float64) ** 2))
            if rms < _ENERGY_FLOOR_RMS:
                self._pre_buffer.append(audio_chunk_int16.copy())
                return (None, None)

        audio_float32 = audio_chunk_int16.astype(np.float32) / 32768.0
        chunk_tensor = torch.tensor(audio_float32, dtype=torch.float32)
        speech_dict = self._vad_iterator(chunk_tensor, return_seconds=False)

        if speech_dict is not None and "start" in speech_dict:
            self._is_speech = True
            self._audio_buffer = list(self._pre_buffer)
            self._audio_buffer.append(audio_chunk_int16.copy())
            self._pre_buffer.clear()
            return ("speech_start", None)

        if self._is_speech:
            self._audio_buffer.append(audio_chunk_int16.copy())

        if speech_dict is not None and "end" in speech_dict:
            self._is_speech = False
            concatenated = np.concatenate(self._audio_buffer)
            self._audio_buffer = []
            self._pre_buffer.clear()

            if len(concatenated) < self._min_speech_samples:
                return (None, None)

            return ("speech_end", concatenated)

        if not self._is_speech:
            self._pre_buffer.append(audio_chunk_int16.copy())

        return (None, None)

    def reset(self) -> None:
        self._vad_iterator.reset_states()
        self._audio_buffer = []
        self._pre_buffer.clear()
        self._is_speech = False
