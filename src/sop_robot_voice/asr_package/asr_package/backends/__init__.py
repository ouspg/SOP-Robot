"""Speech-to-text backend implementations."""

from .base import Transcriber
from .crispasr import (
    CrispAsrTranscriber,
    resolve_crispasr_lib_path,
    resolve_crispasr_model_path,
)
from .faster_whisper import FasterWhisperTranscriber
from .pywhispercpp import PyWhisperCppTranscriber, resolve_whisper_cpp_model_path

__all__ = [
    "CrispAsrTranscriber",
    "FasterWhisperTranscriber",
    "PyWhisperCppTranscriber",
    "Transcriber",
    "resolve_crispasr_lib_path",
    "resolve_crispasr_model_path",
    "resolve_whisper_cpp_model_path",
]
