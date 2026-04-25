"""Audio input and output helpers backed by sounddevice."""

from __future__ import annotations

import importlib
import queue
import subprocess
import sys
import threading
from typing import Protocol, cast

import numpy as np

from voice_stack_common.audio_platform import find_audio_tool, wslg_pulse_cli_available
from voice_stack_common.config import VoiceChatConfig
from voice_stack_common.errors import AudioDependencyError


def _load_sounddevice():
    try:
        return importlib.import_module("sounddevice")
    except ModuleNotFoundError as exc:
        raise AudioDependencyError(
            "Python package 'sounddevice' is missing.\n"
            "Install the root pixi environment before launching the voice stack."
        ) from exc
    except OSError as exc:
        if "PortAudio library not found" not in str(exc):
            raise

        help_lines = ["PortAudio library was not found, so audio I/O cannot start."]
        if sys.platform.startswith("linux"):
            help_lines.append("Fix in the pixi environment with: pixi install")
            help_lines.append(
                "If you are not using pixi, install the system PortAudio runtime "
                "(for example 'libportaudio2')."
            )
        raise AudioDependencyError("\n".join(help_lines)) from exc


sd = _load_sounddevice()


class _AudioStream(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class _PulseAudioCaptureStream:
    """Capture microphone PCM from WSLg PulseAudio when PortAudio has no devices."""

    def __init__(
        self,
        audio_queue: queue.Queue[np.ndarray],
        sample_rate: int,
        channels: int,
        chunk_samples: int,
    ) -> None:
        self._audio_queue = audio_queue
        self._sample_rate = sample_rate
        self._channels = channels
        self._chunk_samples = chunk_samples
        self._process: subprocess.Popen[bytes] | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        parec = find_audio_tool("parec")
        if not parec:
            raise AudioDependencyError("WSLg PulseAudio capture tool 'parec' was not found.")

        command = [
            parec,
            "--raw",
            "--format=s16le",
            f"--rate={self._sample_rate}",
            f"--channels={self._channels}",
            "--latency-msec=50",
        ]
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            name="wslg-pulse-capture",
            daemon=True,
        )
        self._reader_thread.start()

    def _read_loop(self) -> None:
        assert self._process is not None
        assert self._process.stdout is not None
        bytes_per_chunk = self._chunk_samples * self._channels * np.dtype(np.int16).itemsize
        while not self._stop_event.is_set():
            data = self._process.stdout.read(bytes_per_chunk)
            if not data:
                break
            samples = np.frombuffer(data, dtype=np.int16)
            if self._channels > 1:
                samples = samples.reshape(-1, self._channels)[:, 0]
            self._audio_queue.put(samples.copy())

    def stop(self) -> None:
        self._stop_event.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1.0)
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

    def close(self) -> None:
        self.stop()
        self._process = None
        self._reader_thread = None


def _sounddevice_has_default(kind: str) -> bool:
    try:
        index = sd.default.device[0 if kind == "input" else 1]
        if index is not None and int(index) >= 0:
            return True
        devices = sd.query_devices()
    except Exception:
        return False

    max_channels_key = "max_input_channels" if kind == "input" else "max_output_channels"
    return any(int(device.get(max_channels_key, 0)) > 0 for device in devices)


class AudioIO:
    """Microphone capture and optional audio playback."""

    def __init__(self, config: VoiceChatConfig):
        self.sample_rate = config.sample_rate
        self.channels = config.channels
        self.chunk_samples = config.chunk_samples
        self._audio_queue: queue.Queue[np.ndarray] = queue.Queue()
        self._stream: _AudioStream | None = None

    def _audio_callback(self, indata, frames, time_info, status):
        del frames, time_info
        if status:
            print(f"[Audio] {status}")
        self._audio_queue.put(indata[:, 0].copy())

    def start_capture(self) -> None:
        if not _sounddevice_has_default("input") and wslg_pulse_cli_available("parec"):
            stream = _PulseAudioCaptureStream(
                self._audio_queue,
                self.sample_rate,
                self.channels,
                self.chunk_samples,
            )
            stream.start()
            self._stream = stream
            print("[Audio] Using WSLg PulseAudio microphone capture via parec.")
            return

        stream = cast(
            _AudioStream,
            sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype="int16",
                blocksize=self.chunk_samples,
                callback=self._audio_callback,
            ),
        )
        stream.start()
        self._stream = stream

    def stop_capture(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def get_audio_chunk(self, timeout: float = 0.1) -> np.ndarray | None:
        try:
            return self._audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def clear_queue(self) -> None:
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

    def play_audio(self, audio: np.ndarray, sample_rate: int) -> None:
        if not _sounddevice_has_default("output") and wslg_pulse_cli_available("paplay"):
            paplay = find_audio_tool("paplay")
            if paplay:
                pcm = np.asarray(audio)
                if pcm.dtype != np.int16:
                    pcm = np.clip(pcm, -1.0, 1.0)
                    pcm = (pcm * np.iinfo(np.int16).max).astype(np.int16)
                subprocess.run(
                    [
                        paplay,
                        "--raw",
                        "--format=s16le",
                        f"--rate={sample_rate}",
                        "--channels=1",
                    ],
                    input=pcm.reshape(-1).tobytes(),
                    check=True,
                )
                return

        sd.play(audio, samplerate=sample_rate)
        sd.wait()

    def close(self) -> None:
        self.stop_capture()
