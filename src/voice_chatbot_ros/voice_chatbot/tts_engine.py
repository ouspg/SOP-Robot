"""
Text-to-speech synthesis using Coqui TTS.

Supports local model loading (model.pth + config.json) or the Coqui
model zoo.  Requires espeak-ng on Windows for VITS phonemisation.
"""

import os

from .platform_setup import setup_espeak

# espeak-ng PATH must be set before importing TTS (it probes for the
# backend at import time on some platforms).
setup_espeak()

import numpy as np
import torch
from TTS.api import TTS

from .config import Config


class TextToSpeech:
    """Coqui TTS wrapper with local-model and model-zoo support."""

    def __init__(self, config: Config):
        model_path = config.tts_model_path
        config_path = config.tts_config_path
        device = torch.device(
            "cuda" if torch.cuda.is_available() and config.tts_gpu else "cpu"
        )

        if os.path.isfile(model_path) and os.path.isfile(config_path):
            print(
                f"[TTS] Loading local model '{model_path}' with config '{config_path}'..."
            )
            self._tts = TTS(model_path=model_path, config_path=config_path).to(device)
        else:
            print(f"[TTS] Loading model '{config.tts_model}'...")
            self._tts = TTS(model_name=config.tts_model).to(device)

        if self._tts.synthesizer is None:
            raise RuntimeError("TTS synthesizer is not initialized.")
        self._sample_rate = self._tts.synthesizer.output_sample_rate
        print(f"[TTS] Model loaded (sample rate: {self._sample_rate} Hz).")

    def synthesize(self, text: str) -> tuple[np.ndarray, int]:
        """Convert text to a (audio_float32, sample_rate) tuple."""
        wav = self._tts.tts(text=text)
        audio = np.array(wav, dtype=np.float32)
        return audio, self._sample_rate
