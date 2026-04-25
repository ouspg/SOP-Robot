"""
Enforce a setuptools version compatible with ROS 2 Humble's colcon.
"""

from __future__ import annotations

import subprocess
import sys
from importlib.metadata import PackageNotFoundError, version

MIN_VERSION = (69, 5)
MAX_VERSION = (80, 0)
PIP_SPEC = "setuptools>=69.5,<80"


def _parse_version(raw: str) -> tuple[int, int]:
    parts: list[int] = []
    for token in raw.split("."):
        digits = ""
        for ch in token:
            if ch.isdigit():
                digits += ch
            else:
                break
        if not digits:
            break
        parts.append(int(digits))
        if len(parts) == 2:
            break
    while len(parts) < 2:
        parts.append(0)
    return parts[0], parts[1]


def _is_compatible(raw: str) -> bool:
    parsed = _parse_version(raw)
    return MIN_VERSION <= parsed < MAX_VERSION


def main() -> int:
    try:
        current = version("setuptools")
    except PackageNotFoundError:
        current = None

    if current and _is_compatible(current):
        print(f"[pixi] setuptools {current} is compatible with colcon.")
        return 0

    if current:
        print(
            f"[pixi] setuptools {current} is incompatible with colcon; "
            f"installing {PIP_SPEC}..."
        )
    else:
        print(f"[pixi] setuptools is missing; installing {PIP_SPEC}...")

    subprocess.check_call(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--upgrade",
            "--force-reinstall",
            PIP_SPEC,
        ]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
