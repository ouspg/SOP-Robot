"""Canonical SOP Robot bring-up launch entry point."""

import sys
from pathlib import Path

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from robot_launch.orchestrator import generate_launch_description  # noqa: E402,F401
