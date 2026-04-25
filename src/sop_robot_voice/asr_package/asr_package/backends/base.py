"""Common ASR backend interfaces."""

from __future__ import annotations

from typing import Protocol

import numpy as np


class Transcriber(Protocol):
    def transcribe(self, audio_int16: np.ndarray) -> str:
        """Return recognized text from mono 16 kHz int16 audio."""
        ...
