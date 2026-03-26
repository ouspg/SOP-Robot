"""
Model bootstrap script — download and validate all required models.

Run once before the first launch.

Usage::

    pixi run setup-models
    # or: python -m voice_chatbot.setup_models
"""

import os
import sys
from pathlib import Path

from .platform_setup import setup_cuda

setup_cuda()

from .config import Config


def check_cuda() -> None:
    """Verify that PyTorch is installed and can see a CUDA GPU."""
    print("Checking CUDA availability...")
    try:
        import torch

        if torch.cuda.is_available():
            print(f"  CUDA is available: {torch.cuda.get_device_name(0)}")
            print(f"  CUDA version: {torch.version.cuda}")
        else:
            print("  WARNING: CUDA is NOT available. Models will run on CPU.")
    except ImportError:
        print("  ERROR: PyTorch is not installed. Run install.bat first.")
        sys.exit(1)


def setup_whisper(config: Config) -> None:
    """Load the configured Whisper model to warm up the cache."""
    print(f"\nSetting up Whisper model '{config.whisper_model}'...")
    try:
        import torch
        from faster_whisper import WhisperModel

        device = "cuda" if config.whisper_gpu and torch.cuda.is_available() else "cpu"
        model = WhisperModel(
            config.whisper_model,
            device=device,
            cpu_threads=config.whisper_n_threads,
        )
        print(f"  Whisper model '{config.whisper_model}' is ready.")
        del model
    except Exception as e:
        print(f"  ERROR setting up Whisper: {e}")
        sys.exit(1)


def setup_llm(config: Config) -> None:
    """Download the GGUF model from HuggingFace if not already present."""
    print("\nSetting up LLM model...")
    model_path = Path(config.llm_model_path)

    if model_path.exists():
        size_gb = model_path.stat().st_size / (1024**3)
        print(f"  LLM model already exists: {model_path} ({size_gb:.1f} GB)")
        return

    print(f"  Downloading '{config.llm_filename}' from '{config.llm_repo_id}'...")
    os.makedirs(config.models_dir, exist_ok=True)

    try:
        from huggingface_hub import hf_hub_download

        downloaded_path = hf_hub_download(
            repo_id=config.llm_repo_id,
            filename=config.llm_filename,
            local_dir=config.models_dir,
        )
        print(f"  Downloaded to: {downloaded_path}")
        downloaded = Path(downloaded_path)
        if downloaded != model_path and downloaded.exists():
            downloaded.rename(model_path)
            print(f"  Renamed to: {model_path}")
    except Exception as e:
        print(f"  ERROR downloading LLM: {e}")
        print(
            f"  You can manually download from: https://huggingface.co/{config.llm_repo_id}"
        )
        print(f"  Place the GGUF file at: {model_path}")
        sys.exit(1)


def setup_tts(config: Config) -> None:
    """Load the configured Coqui TTS model to warm up the cache."""
    print(f"\nSetting up TTS model...")
    try:
        from TTS.api import TTS

        model_path = config.tts_model_path
        config_path = config.tts_config_path

        if os.path.isfile(model_path) and os.path.isfile(config_path):
            print(f"  Loading local model '{model_path}'...")
            tts = TTS(model_path=model_path, config_path=config_path)
        else:
            print(f"  Loading model '{config.tts_model}' from Coqui zoo...")
            tts = TTS(model_name=config.tts_model, gpu=False)

        print(f"  TTS model is ready.")
        del tts
    except Exception as e:
        print(f"  ERROR setting up TTS: {e}")
        sys.exit(1)


def setup_vad() -> None:
    """Load Silero-VAD to ensure it is cached by torch.hub."""
    print("\nSetting up Silero-VAD model...")
    try:
        from silero_vad import load_silero_vad

        model = load_silero_vad()
        print("  Silero-VAD model is ready.")
        del model
    except Exception as e:
        print(f"  ERROR setting up VAD: {e}")
        sys.exit(1)


def main() -> None:
    config = Config.load()
    print("=" * 50)
    print("  Voice Chatbot - Model Setup")
    print("=" * 50)

    check_cuda()
    setup_vad()
    setup_whisper(config)
    setup_llm(config)
    setup_tts(config)

    print("\n" + "=" * 50)
    print("  Setup complete!")
    print("=" * 50)
    print("\nAll models are downloaded and ready.")
    print("Run 'pixi run chatbot' to start the voice chatbot.")


if __name__ == "__main__":
    main()
