"""RViz embed panel for the unified chatbot app."""

from __future__ import annotations

import logging
import re
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtGui import QWindow
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

log = logging.getLogger(__name__)
_DEFAULT_RVIZ_CONFIG = (
    Path(get_package_share_directory("inmoov_description")) / "config" / "inmoov.rviz"
)


def _find_rviz_window_x11() -> int | None:
    candidates: list[int] = []

    try:
        out = subprocess.run(
            ["xdotool", "search", "--class", "rviz"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if out.returncode == 0 and out.stdout.strip():
            candidates.extend(int(line) for line in out.stdout.strip().splitlines())
    except (FileNotFoundError, ValueError, subprocess.TimeoutExpired):
        pass

    try:
        out = subprocess.run(
            ["xdotool", "search", "--name", "RViz"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if out.returncode == 0 and out.stdout.strip():
            candidates.extend(int(line) for line in out.stdout.strip().splitlines())
    except (FileNotFoundError, ValueError, subprocess.TimeoutExpired):
        pass

    candidates.extend(_find_rviz_windows_with_xprop())
    candidates.extend(_find_rviz_windows_with_xwininfo())
    seen: set[int] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if _is_rviz_window(candidate):
            return candidate
    return None


def _find_rviz_windows_with_xprop() -> list[int]:
    try:
        out = subprocess.run(
            ["xprop", "-root", "_NET_CLIENT_LIST", "_NET_CLIENT_LIST_STACKING"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if out.returncode != 0:
        return []
    return [int(match, 16) for match in re.findall(r"0x[0-9a-fA-F]+", out.stdout)]


def _find_rviz_windows_with_xwininfo() -> list[int]:
    try:
        out = subprocess.run(
            ["xwininfo", "-root", "-tree"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    if out.returncode != 0:
        return []

    ids: list[int] = []
    for line in out.stdout.splitlines():
        if "rviz" not in line.lower():
            continue
        match = re.search(r"0x[0-9a-fA-F]+", line)
        if match:
            ids.append(int(match.group(0), 16))
    return ids


def _is_rviz_window(wid: int) -> bool:
    try:
        out = subprocess.run(
            ["xprop", "-id", str(wid), "WM_NAME", "_NET_WM_NAME", "WM_CLASS"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return True
    if out.returncode != 0:
        return False
    text = out.stdout.lower()
    return "rviz" in text


class RVizEmbedPanel(QWidget):
    """Launch rviz2 and try to embed its window."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._process: QProcess | None = None
        self._container: QWidget | None = None
        self._embedded_window: QWindow | None = None
        self._embed_attempts = 0
        self._build_ui()

    def _build_ui(self) -> None:
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(4)

        title = QLabel("RViz")
        title.setStyleSheet("font-weight: bold; font-size: 13px; padding: 2px;")
        self._layout.addWidget(title)

        self._placeholder = QLabel("RViz ei kaynnissa")
        self._placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self._placeholder.setStyleSheet(
            "QLabel { background-color: #11111b; color: #6c7086; "
            "border: 1px solid #45475a; border-radius: 4px; }"
        )
        self._layout.addWidget(self._placeholder)

    @property
    def is_running(self) -> bool:
        return self._container is not None or (
            self._process is not None
            and self._process.state() != QProcess.ProcessState.NotRunning
        )

    def start_rviz(self, config_path: str | None = None) -> None:
        if self.is_running:
            return

        if self.attach_existing_rviz():
            return

        rviz_cfg = config_path or str(_DEFAULT_RVIZ_CONFIG)
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._discard_process_output)
        self._process.finished.connect(self._on_process_finished)
        self._process.start("rviz2", ["-d", rviz_cfg])
        self._placeholder.setText("RViz kaynnistyy...")
        self._embed_attempts = 0
        QTimer.singleShot(3000, self._attempt_embed)

    def attach_existing_rviz(self) -> bool:
        wid = _find_rviz_window_x11() if sys.platform != "win32" else None
        if wid is None:
            return False
        return self._embed_window(wid)

    def stop_rviz(self) -> None:
        if self._container is not None:
            self._layout.removeWidget(self._container)
            self._container.setParent(None)
            self._container.deleteLater()
            self._container = None
            self._embedded_window = None

        if self._process is not None:
            self._process.terminate()
            self._process.waitForFinished(3000)
            if self._process.state() != QProcess.ProcessState.NotRunning:
                self._process.kill()
            self._process = None

        self._placeholder.setText("RViz ei kaynnissa")
        self._placeholder.show()

    def _attempt_embed(self) -> None:
        if not self.is_running:
            return

        self._embed_attempts += 1
        wid = _find_rviz_window_x11() if sys.platform != "win32" else None

        if wid is not None:
            if self._embed_window(wid):
                return

        if self._embed_attempts < 5:
            QTimer.singleShot(2000, self._attempt_embed)
        else:
            self._placeholder.setText("RViz kaynnissa erillisessa ikkunassa")
            log.info(
                "RViz embedding failed after %d attempts, running standalone",
                self._embed_attempts,
            )

    def _embed_window(self, wid: int) -> bool:
        try:
            window = QWindow.fromWinId(wid)
            container = QWidget.createWindowContainer(window, self)
            container.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.Expanding,
            )
            self._placeholder.hide()
            self._layout.addWidget(container)
            self._embedded_window = window
            self._container = container
            log.info("RViz window embedded (wid=%s)", wid)
            return True
        except Exception as exc:
            log.warning("Failed to embed RViz window: %s", exc)
            return False

    def _discard_process_output(self) -> None:
        if self._process is not None:
            self._process.readAllStandardOutput()

    def _on_process_finished(self, exit_code: int, _status) -> None:
        del exit_code
        if self._container is not None:
            self._layout.removeWidget(self._container)
            self._container.setParent(None)
            self._container.deleteLater()
            self._container = None
            self._embedded_window = None
        self._process = None
        self._placeholder.setText("RViz ei kaynnissa")
        self._placeholder.show()
