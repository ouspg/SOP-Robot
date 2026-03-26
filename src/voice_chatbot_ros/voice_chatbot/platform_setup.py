"""
Platform-specific setup that must run before CUDA or Qt imports.

This module consolidates three environment fixups that were previously
duplicated across every entry point:

1. **CUDA DLL directories** – ``os.add_dll_directory`` for the CUDA
   toolkit so that ``torch``, ``llama_cpp``, and ``faster_whisper`` can
   find ``cublas64_*.dll`` etc.
2. **PySide6 / Robostack DLL conflict** – force Windows to prefer the
   pip-installed PySide6 Qt runtime over the one shipped by Robostack.
3. **espeak-ng PATH** – append the default Windows install directory
   so Coqui TTS can find the phonemiser backend.

Usage — call the functions you need at the **top** of each entry point,
**before** importing any CUDA-backed or Qt library::

    from voice_chatbot.platform_setup import setup_cuda, setup_pyside6

    setup_cuda()      # before torch / llama_cpp / faster_whisper
    setup_pyside6()   # before PySide6 widget imports
"""

import os
import sys
from pathlib import Path


def setup_cuda() -> None:
    """Register CUDA DLL directories on Windows.

    Reads ``CUDA_PATH`` (default ``D:\\cuda``) and adds ``bin/x64`` and
    ``bin`` subdirectories to the DLL search path.  Safe to call on
    Linux (no-ops if ``os.add_dll_directory`` is unavailable).
    """
    if not hasattr(os, "add_dll_directory"):
        return
    cuda_path = os.environ.get("CUDA_PATH", r"D:\cuda")
    for subdir in ("bin/x64", "bin"):
        p = os.path.join(cuda_path, subdir)
        if os.path.isdir(p):
            os.add_dll_directory(p)
            os.environ["PATH"] = p + os.pathsep + os.environ.get("PATH", "")


def setup_pyside6() -> None:
    """Fix PySide6 / Robostack Qt DLL conflicts on Windows.

    When both pip-installed PySide6 and conda-installed Qt (from
    Robostack) are present, Windows may load the wrong platform plugin
    and crash.  This function forces the PySide6 DLL directory onto
    ``PATH`` and overrides ``QT_PLUGIN_PATH``.
    """
    if not hasattr(os, "add_dll_directory"):
        return
    site_packages = [Path(p) for p in sys.path if "site-packages" in p]
    pyside_dir = None
    for pkg_name in ("PySide6", "shiboken6"):
        for base in site_packages:
            dll_dir = base / pkg_name
            if dll_dir.is_dir():
                os.add_dll_directory(str(dll_dir))
                os.environ["PATH"] = (
                    str(dll_dir) + os.pathsep + os.environ.get("PATH", "")
                )
                if pkg_name == "PySide6":
                    pyside_dir = dll_dir

    if pyside_dir is not None:
        plugins_dir = pyside_dir / "plugins"
        platforms_dir = plugins_dir / "platforms"
        os.environ["QT_PLUGIN_PATH"] = str(plugins_dir)
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(platforms_dir)


def setup_espeak() -> None:
    """Add the default eSpeak NG install directory to PATH on Windows.

    Required by Coqui TTS VITS models for phoneme conversion.
    """
    espeak_dir = os.path.join(
        os.environ.get("ProgramFiles", r"C:\Program Files"), "eSpeak NG"
    )
    if os.path.isdir(espeak_dir) and espeak_dir not in os.environ.get("PATH", ""):
        os.environ["PATH"] = espeak_dir + os.pathsep + os.environ["PATH"]
