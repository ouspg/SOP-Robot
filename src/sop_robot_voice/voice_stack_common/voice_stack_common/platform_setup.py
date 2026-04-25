"""Platform-specific runtime setup for native Ubuntu voice stack usage."""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path

_LINUX_CXX_RUNTIME_READY = False
_CUDA_RUNTIME_READY = False
_PYSIDE6_ENV_FLAG = "SOP_ROBOT_PYSIDE6_ENV_READY"


def setup_linux_cxx_runtime() -> None:
    """Preload the environment C++ runtime before CUDA-backed libraries import."""
    global _LINUX_CXX_RUNTIME_READY

    if _LINUX_CXX_RUNTIME_READY or sys.platform != "linux":
        return

    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        return

    lib_dir = Path(prefix) / "lib"
    if not lib_dir.is_dir():
        return

    loaded_any = False
    for name in ("libstdc++.so.6", "libgcc_s.so.1"):
        lib_path = lib_dir / name
        if not lib_path.is_file():
            continue
        try:
            ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
            loaded_any = True
        except OSError:
            continue

    if loaded_any:
        _LINUX_CXX_RUNTIME_READY = True


def setup_cuda() -> None:
    """Perform native runtime setup required before CUDA-backed imports."""
    setup_linux_cxx_runtime()
    preload_cuda_runtime()


def preload_cuda_runtime() -> None:
    """Preload Pixi-provided CUDA libraries before llama-cpp imports."""
    global _CUDA_RUNTIME_READY

    if _CUDA_RUNTIME_READY or sys.platform != "linux":
        return

    cuda_dirs: list[Path] = []
    for entry in map(Path, sys.path):
        if "site-packages" not in str(entry):
            continue
        nvidia_root = entry / "nvidia"
        cuda_dirs.extend(
            [
                nvidia_root / "cu13" / "lib",
                nvidia_root / "cuda_nvrtc" / "lib",
                nvidia_root / "nvvm" / "lib",
                nvidia_root / "cuda_nvcc" / "lib",
                nvidia_root / "cuda_runtime" / "lib",
                nvidia_root / "cublas" / "lib",
                nvidia_root / "cudnn" / "lib",
            ]
        )

    loaded_any = False
    for name in (
        "libnvrtc.so.13",
        "libnvrtc-builtins.so.13.0",
        "libcudart.so.13",
        "libcublas.so.13",
        "libcublasLt.so.13",
    ):
        for lib_dir in cuda_dirs:
            lib_path = lib_dir / name
            if not lib_path.is_file():
                continue
            try:
                ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
                loaded_any = True
                break
            except OSError:
                continue

    if loaded_any:
        _CUDA_RUNTIME_READY = True


def setup_pyside6() -> None:
    """Ensure PySide6 finds its own Qt runtime on Linux."""
    if not sys.platform.startswith("linux"):
        return
    if os.environ.get(_PYSIDE6_ENV_FLAG) == "1":
        return

    for entry in map(Path, sys.path):
        if "site-packages" not in str(entry):
            continue
        qt_root = entry / "PySide6" / "Qt"
        if not qt_root.is_dir():
            continue

        lib_dir = qt_root / "lib"
        plugins_dir = qt_root / "plugins"
        platforms_dir = plugins_dir / "platforms"

        env = os.environ.copy()
        current_ld = env.get("LD_LIBRARY_PATH", "")
        lib_entries = []
        if lib_dir.is_dir():
            lib_entries.append(str(lib_dir))
        conda_prefix = env.get("CONDA_PREFIX")
        if conda_prefix:
            conda_lib = Path(conda_prefix) / "lib"
            if conda_lib.is_dir():
                lib_entries.append(str(conda_lib))
        if lib_entries:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(
                lib_entries + ([current_ld] if current_ld else [])
            )
        if plugins_dir.is_dir():
            env["QT_PLUGIN_PATH"] = str(plugins_dir)
        if platforms_dir.is_dir():
            env["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(platforms_dir)
        env.setdefault("QT_QPA_PLATFORM", "xcb")
        env[_PYSIDE6_ENV_FLAG] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], env)
