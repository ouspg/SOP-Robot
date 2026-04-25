"""Entry point that selects the plain or unified chatbot GUI."""

from __future__ import annotations

import os
import sys


def main() -> None:
    use_unified = "--unified" in sys.argv or os.environ.get(
        "SOP_ROBOT_CHATBOT_UNIFIED"
    ) == "1"
    if use_unified:
        from .unified_app import main as run_unified

        run_unified()
        return

    from .ros_app import main as run_plain

    run_plain()
