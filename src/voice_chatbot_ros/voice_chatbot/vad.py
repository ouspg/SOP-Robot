"""
Voice Activity Detection (VAD) using Silero-VAD.

Wraps `silero-vad <https://github.com/snakers4/silero-vad>`_ with
two practical additions that improve accuracy in a real-time voice
assistant pipeline:

1. **Energy gate** – a simple RMS check rejects obviously-silent chunks
   *before* they reach Silero, reducing false positives from background
   hum.  The gate is only active while no speech is in progress; once
   speech has started, every chunk is forwarded so Silero can count
   silence frames and fire ``speech_end``.

2. **Pre-buffer** – a rolling buffer of recent chunks is maintained so
   that audio captured *before* Silero reports ``speech_start`` is
   prepended to the utterance.  Without this the first syllable is
   always clipped, producing garbled Whisper output.

Usage::

    vad = VoiceActivityDetector(config)
    event, audio_data = vad.process_chunk(chunk_int16)
    # event is "speech_start", "speech_end", or None
    # audio_data is the complete utterance (int16) on "speech_end"
"""

from collections import deque

import numpy as np
import torch
from silero_vad import VADIterator, load_silero_vad

from .config import Config

# RMS threshold below which audio is considered silence (int16 scale).
# Only applied BEFORE speech starts — once speech is active, all chunks
# are forwarded to Silero so it can detect the end-of-speech silence.
_ENERGY_FLOOR_RMS = 30


class VoiceActivityDetector:
    """Silero-VAD wrapper with energy gating and pre-buffering.

    Args:
        config: Shared :class:`Config` – reads VAD thresholds, sample
                rate, chunk size, and pre-buffer duration.
    """

    def __init__(self, config: Config):
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

        # Pre-buffer: keeps recent chunks so we capture audio *before*
        # Silero reports "speech_start".  Without this the first syllable
        # is always clipped, producing garbled Whisper output.
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
        """Feed one audio chunk and return a VAD event.

        Args:
            audio_chunk_int16: A 1-D int16 numpy array (one chunk from
                :meth:`AudioIO.get_audio_chunk`).

        Returns:
            A ``(event, audio_data)`` tuple where *event* is one of:

            - ``"speech_start"`` – speech onset detected (audio_data is
              ``None``).
            - ``"speech_end"`` – end of utterance (audio_data is the
              concatenated int16 array of the full utterance, including
              the pre-buffer).
            - ``None`` – no event (audio_data is ``None``).
        """
        # Energy gate — only applied when NOT in speech.
        # During speech we MUST forward all chunks (including silence)
        # to Silero so it can count consecutive silent frames and
        # trigger "speech_end".  Without this, end detection never fires.
        if not self._is_speech:
            rms = np.sqrt(np.mean(audio_chunk_int16.astype(np.float64) ** 2))
            if rms < _ENERGY_FLOOR_RMS:
                self._pre_buffer.append(audio_chunk_int16.copy())
                return (None, None)

        audio_float32 = audio_chunk_int16.astype(np.float32) / 32768.0
        chunk_tensor = torch.from_numpy(audio_float32)

        speech_dict = self._vad_iterator(chunk_tensor, return_seconds=False)

        if speech_dict is not None and "start" in speech_dict:
            self._is_speech = True
            # Prepend the pre-buffer so the onset of speech is preserved
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

        # Feed pre-buffer when not in speech
        if not self._is_speech:
            self._pre_buffer.append(audio_chunk_int16.copy())

        return (None, None)

    def reset(self):
        """Reset all internal state.

        Called after TTS playback to prevent the assistant's own speech
        (picked up by the microphone) from being treated as user input.
        """
        self._vad_iterator.reset_states()
        self._audio_buffer = []
        self._pre_buffer.clear()
        self._is_speech = False
