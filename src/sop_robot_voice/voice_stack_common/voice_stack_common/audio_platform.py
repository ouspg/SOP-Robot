"""Audio runtime helpers for WSLg without changing native Ubuntu behavior."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def is_wsl() -> bool:
    if not sys.platform.startswith("linux"):
        return False
    if os.environ.get("WSL_DISTRO_NAME"):
        return True
    try:
        return "microsoft" in Path("/proc/version").read_text(encoding="utf-8").lower()
    except OSError:
        return False


def _socket_from_pulse_server(value: str) -> Path | None:
    if not value.startswith("unix:"):
        return None
    return Path(value[5:])


def pick_wslg_pulse_server() -> str | None:
    for candidate in (
        f"unix:/run/user/{os.getuid()}/pulse/native",
        "unix:/mnt/wslg/runtime-dir/pulse/native",
        "unix:/mnt/wslg/PulseServer",
    ):
        socket_path = _socket_from_pulse_server(candidate)
        if socket_path is not None and socket_path.is_socket():
            return candidate
    return None


def setup_wsl_audio_environment() -> str | None:
    """Set WSLg PulseAudio env vars only when running inside WSL."""
    if not is_wsl():
        return None

    runtime_dir = Path(f"/run/user/{os.getuid()}")
    if not os.environ.get("XDG_RUNTIME_DIR") and runtime_dir.is_dir():
        os.environ["XDG_RUNTIME_DIR"] = str(runtime_dir)

    pulse_server = pick_wslg_pulse_server()
    if pulse_server:
        os.environ["PULSE_SERVER"] = pulse_server
    return pulse_server


def find_audio_tool(name: str) -> str | None:
    resolved = shutil.which(name)
    if resolved:
        return resolved

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        candidate = Path(conda_prefix) / "bin" / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def wslg_pulse_cli_available(*tool_names: str) -> bool:
    pulse_server = setup_wsl_audio_environment()
    if not pulse_server:
        return False

    socket_path = _socket_from_pulse_server(pulse_server)
    if socket_path is not None and not socket_path.is_socket():
        return False

    return all(find_audio_tool(name) for name in tool_names)


def pulse_server_responds(timeout: float = 2.0) -> bool:
    if not wslg_pulse_cli_available("pactl"):
        return False
    pactl = find_audio_tool("pactl")
    if not pactl:
        return False
    try:
        subprocess.run(
            [pactl, "info"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return True
