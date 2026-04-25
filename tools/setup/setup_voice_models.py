"""Download or warm the models used by the in-tree voice stack."""

from __future__ import annotations

import importlib
import inspect
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
COMMON_SRC = PROJECT_ROOT / "src" / "voice_stack_common"
if str(COMMON_SRC) not in sys.path:
    sys.path.insert(0, str(COMMON_SRC))

from voice_stack_common.platform_setup import setup_cuda

from voice_stack_common.config import VoiceChatConfig

setup_cuda()

DEFAULT_LLM_REPO_ID = "mradermacher/Ahma-2-4B-Instruct-GGUF"
DEFAULT_LLM_FILENAME = "Ahma-2-4B-Instruct.Q4_K_S.gguf"
LEGACY_TTS_URL = "https://github.com/ouspg/SOP-Robot/releases/download/model/model.zip"


def check_cuda() -> None:
    print("Checking CUDA availability...")
    try:
        import torch

        if torch.cuda.is_available():
            print(f"  CUDA is available: {torch.cuda.get_device_name(0)}")
            print(f"  CUDA version: {torch.version.cuda}")
        else:
            print("  WARNING: CUDA is not available. Models will run on CPU.")
    except ImportError:
        print("  WARNING: PyTorch is not installed.")


def setup_vad() -> None:
    print("\nSetting up Silero-VAD model...")
    from silero_vad import load_silero_vad

    model = load_silero_vad()
    del model
    print("  Silero-VAD model is ready.")


def setup_whisper(config: VoiceChatConfig) -> None:
    backend = config.whisper_backend.strip().lower().replace("_", "-")
    if backend in {
        "pywhispercpp",
        "pywhisper-cpp",
        "py-whisper-cpp",
        "whisper.cpp",
        "whispercpp",
        "whisper-cpp",
    }:
        setup_whisper_cpp(config)
        return
    if backend in {"crisp", "crispasr", "crisp-asr", "parakeet"}:
        setup_crispasr(config)
        return
    setup_faster_whisper(config)


def setup_faster_whisper(config: VoiceChatConfig) -> None:
    print(f"\nSetting up faster-whisper model '{config.whisper_model}'...")
    import torch
    from faster_whisper import WhisperModel

    use_gpu = config.whisper_gpu and torch.cuda.is_available()
    device = "cuda" if use_gpu else "cpu"
    compute_type = "float16" if use_gpu else "int8"
    model_kwargs = {
        "device": device,
        "cpu_threads": config.whisper_n_threads,
    }
    if "compute_type" in inspect.signature(WhisperModel).parameters:
        model_kwargs["compute_type"] = compute_type
    model = WhisperModel(config.whisper_model, **model_kwargs)
    del model
    print(
        f"  faster-whisper model '{config.whisper_model}' is ready on {device} "
        f"(compute_type: {compute_type})."
    )


def setup_whisper_cpp(config: VoiceChatConfig) -> None:
    print(
        f"\nSetting up pywhispercpp model "
        f"'{config.whisper_cpp_model_filename}'..."
    )
    model_path = Path(config.whisper_cpp_model_path).expanduser()
    if not str(config.whisper_cpp_model_path).strip():
        model_path = Path(config.models_dir).expanduser() / config.whisper_cpp_model_filename

    if model_path.is_file():
        size_gb = model_path.stat().st_size / (1024**3)
        print(f"  pywhispercpp model already exists: {model_path} ({size_gb:.1f} GB)")
    else:
        from huggingface_hub import hf_hub_download

        model_path.parent.mkdir(parents=True, exist_ok=True)
        downloaded_path = hf_hub_download(
            repo_id=config.whisper_cpp_repo_id,
            filename=config.whisper_cpp_model_filename,
            local_dir=str(model_path.parent),
        )
        downloaded = Path(downloaded_path)
        if downloaded.resolve() != model_path.resolve() and downloaded.exists():
            shutil.copy2(downloaded, model_path)
        print(f"  Downloaded pywhispercpp model to: {model_path}")

    from pywhispercpp.model import Model

    model = Model(
        str(model_path),
        n_threads=config.whisper_n_threads,
        redirect_whispercpp_logs_to=False,
    )
    del model
    print(f"  pywhispercpp model is ready: {model_path}")


def setup_crispasr(config: VoiceChatConfig) -> None:
    print("\nSetting up CrispASR backend...")
    python_path = Path(config.crispasr_python_path).expanduser()
    lib_path = Path(config.crispasr_lib_path).expanduser()
    model_path = Path(config.crispasr_model_path).expanduser()

    if not python_path.is_dir():
        raise FileNotFoundError(f"CrispASR Python binding path not found: {python_path}")
    if not lib_path.is_file():
        raise FileNotFoundError(f"CrispASR shared library not found: {lib_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"CrispASR model not found: {model_path}")

    if str(python_path) not in sys.path:
        sys.path.insert(0, str(python_path))

    crispasr_module = importlib.import_module("crispasr")
    Session = crispasr_module.Session

    available = Session.available_backends(lib_path=str(lib_path))
    backend = config.crispasr_backend.strip()
    if backend and backend not in available:
        raise RuntimeError(
            f"CrispASR backend {backend!r} is not available in {lib_path}. "
            f"Available backends: {available}"
        )

    session = Session(
        str(model_path),
        lib_path=str(lib_path),
        n_threads=config.whisper_n_threads,
        backend=backend or None,
    )
    try:
        print(
            f"  CrispASR model is ready: {model_path} "
            f"(backend: {session.backend}, available: {', '.join(available)})"
        )
    finally:
        session.close()


def setup_llm(config: VoiceChatConfig) -> None:
    print("\nSetting up LLM model...")
    model_path = Path(config.llm_model_path)
    if model_path.exists():
        size_gb = model_path.stat().st_size / (1024**3)
        print(f"  LLM model already exists: {model_path} ({size_gb:.1f} GB)")
        return

    os.makedirs(config.models_dir, exist_ok=True)
    from huggingface_hub import hf_hub_download

    downloaded_path = hf_hub_download(
        repo_id=DEFAULT_LLM_REPO_ID,
        filename=DEFAULT_LLM_FILENAME,
        local_dir=config.models_dir,
    )
    downloaded = Path(downloaded_path)
    if downloaded != model_path and downloaded.exists():
        downloaded.rename(model_path)
    print(f"  Downloaded to: {model_path}")


def setup_tts(config: VoiceChatConfig) -> None:
    print("\nSetting up TTS assets...")
    resource_dir = PROJECT_ROOT / "src" / "tts_package" / "resource"
    resource_model = resource_dir / "model.pth"
    resource_config = resource_dir / "config.json"
    if resource_model.is_file() and resource_config.is_file():
        print("  Legacy tts_package model files are already present.")
        return

    resource_dir.mkdir(parents=True, exist_ok=True)
    archive = Path(tempfile.gettempdir()) / "sop_robot_tts_model.zip"
    urllib.request.urlretrieve(LEGACY_TTS_URL, archive)
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(resource_dir)
    archive.unlink(missing_ok=True)
    print(f"  Downloaded legacy tts_package assets to: {resource_dir}")

    if Path(config.tts_model_path).is_file() and Path(config.tts_config_path).is_file():
        print("  Config-specific TTS model files are also present.")


def main() -> None:
    config = VoiceChatConfig.load()
    print("=" * 50)
    print("  SOP Robot Voice Stack - Model Setup")
    print("=" * 50)

    check_cuda()
    setup_vad()
    setup_whisper(config)
    setup_llm(config)
    setup_tts(config)

    print("\n" + "=" * 50)
    print("  Setup complete!")
    print("=" * 50)
    print("\nRun 'pixi run launch-robot-fake' or 'pixi run launch-robot' to start the stack.")


if __name__ == "__main__":
    main()
