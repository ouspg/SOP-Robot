"""Backend implementation for Coqui TTS synthesis and playback."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
from typing import Protocol, cast

from ament_index_python.packages import get_package_share_directory

from voice_stack_common.audio_platform import find_audio_tool, wslg_pulse_cli_available
from voice_stack_common.config import VoiceChatConfig
from voice_stack_common.platform_setup import setup_cuda

setup_cuda()

import simpleaudio as sa
from TTS.api import TTS


class _SynthesizerEngine(Protocol):
    def tts(self, text: str) -> object: ...
    def save_wav(self, wav: object, path: str) -> None: ...


class _ModelEngine(Protocol):
    def tts_to_file(self, *, text: str, file_path: str) -> None: ...


class TtsBackend:
    """Load a TTS model and play synthesized speech."""

    def __init__(
        self,
        config: VoiceChatConfig,
        model_path: str | None = None,
        config_path: str | None = None,
        output_path: str | None = None,
    ) -> None:
        package_share = Path(get_package_share_directory("tts_package"))
        default_model_path = package_share / "resource" / "model.pth"
        default_config_path = package_share / "resource" / "config.json"
        default_output_path = Path(tempfile.gettempdir()) / "sop_robot_tts_output.wav"

        self._output_path = Path(output_path or default_output_path)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._mode = "model-name"
        self._synthesizer: _SynthesizerEngine | None = None
        self._model_engine: _ModelEngine | None = None

        chosen_model_path = Path(model_path or config.tts_model_path).expanduser()
        chosen_config_path = Path(config_path or config.tts_config_path).expanduser()

        if chosen_model_path.is_file() and chosen_config_path.is_file():
            self._mode = "local-config"
            self._synthesizer = cast(
                _SynthesizerEngine,
                TTS(
                    model_path=str(chosen_model_path),
                    config_path=str(chosen_config_path),
                ).synthesizer,
            )
            return

        if default_model_path.is_file() and default_config_path.is_file():
            self._mode = "legacy-resource"
            self._synthesizer = cast(
                _SynthesizerEngine,
                TTS(
                    model_path=str(default_model_path),
                    config_path=str(default_config_path),
                ).synthesizer,
            )
            return

        use_gpu = bool(config.tts_gpu)
        try:
            import torch

            use_gpu = use_gpu and torch.cuda.is_available()
        except Exception:
            use_gpu = False
        device = "cuda" if use_gpu else "cpu"
        self._model_engine = cast(_ModelEngine, TTS(model_name=config.tts_model).to(device))

    def synthesize(self, text: str) -> Path:
        if self._mode in {"local-config", "legacy-resource"}:
            if self._synthesizer is None:
                raise RuntimeError("Local TTS synthesizer was not initialized.")
            wav = self._synthesizer.tts(text)
            self._synthesizer.save_wav(wav, str(self._output_path))
            return self._output_path

        if self._model_engine is None:
            raise RuntimeError("TTS model engine was not initialized.")
        self._model_engine.tts_to_file(text=text, file_path=str(self._output_path))
        return self._output_path

    def play(self, audio_path: Path) -> None:
        if wslg_pulse_cli_available("paplay"):
            paplay = find_audio_tool("paplay")
            if paplay:
                subprocess.run([paplay, str(audio_path)], check=True)
                return

        wave_obj = sa.WaveObject.from_wave_file(str(audio_path))
        play_obj = wave_obj.play()
        play_obj.wait_done()

    def speak(self, text: str) -> Path:
        audio_path = self.synthesize(text)
        self.play(audio_path)
        return audio_path
