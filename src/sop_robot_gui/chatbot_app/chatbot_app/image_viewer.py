"""ROS2 image viewer panel for the unified chatbot app."""

from __future__ import annotations

import os
import threading

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node as RosNode
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image as RosImage

DEFAULT_DISPLAY_FPS = 60
MAX_DISPLAY_FPS = 120


class ImageViewerPanel(QWidget):
    """Display a live ROS image topic with a selector."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._node: RosNode | None = None
        self._sub = None
        self._latest_msg: RosImage | None = None
        self._latest_sequence = 0
        self._converted_input_sequence = 0
        self._converted_qimage: QImage | None = None
        self._converted_qimage_sequence = 0
        self._converted_qimage_stamp_ns = 0
        self._displayed_qimage_sequence = 0
        self._topic_generation = 0
        self._target_size = (320, 240)
        self._max_frame_age_seconds = 0.25
        self._stop_converter = False
        self._frame_condition = threading.Condition()
        self._discovery_timer = QTimer(self)
        self._discovery_timer.timeout.connect(self._discover_topics)
        self._build_ui()
        self._converter_thread = threading.Thread(
            target=self._conversion_loop,
            name="chatbot_image_viewer_converter",
            daemon=True,
        )
        self._converter_thread.start()
        self._display_timer = QTimer(self)
        self._display_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._display_timer.timeout.connect(self._display_latest_image)
        self._display_timer.start(self._display_interval_ms())

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        header = QHBoxLayout()
        title = QLabel("Kamerakuva")
        title.setStyleSheet("font-weight: bold; font-size: 13px; padding: 2px;")
        header.addWidget(title)

        self.combo_topic = QComboBox()
        self.combo_topic.setMinimumWidth(200)
        self.combo_topic.addItem("(ei aihetta)", "")
        self.combo_topic.currentIndexChanged.connect(self._on_topic_changed)
        header.addWidget(self.combo_topic)
        header.addStretch()
        layout.addLayout(header)

        self.image_label = QLabel("Ei kuvaa")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet(
            "QLabel { background-color: #11111b; color: #6c7086; "
            "border: 1px solid #45475a; border-radius: 4px; }"
        )
        layout.addWidget(self.image_label)

    def set_node(self, node: RosNode) -> None:
        self._node = node
        self._discover_topics()
        self._discovery_timer.start(5000)

    def detach_node(self) -> None:
        self._discovery_timer.stop()
        self._destroy_subscription()
        self._node = None
        self._clear_latest_image()
        self.image_label.setText("Ei kuvaa")
        self.combo_topic.clear()
        self.combo_topic.addItem("(ei aihetta)", "")

    def _discover_topics(self) -> None:
        if self._node is None:
            return
        topics = self._node.get_topic_names_and_types()
        image_topics = sorted(
            topic for topic, types in topics if "sensor_msgs/msg/Image" in types
        )
        current = self.combo_topic.currentData()
        self.combo_topic.blockSignals(True)
        self.combo_topic.clear()
        self.combo_topic.addItem("(ei aihetta)", "")
        for topic in image_topics:
            self.combo_topic.addItem(topic, topic)
        for i in range(self.combo_topic.count()):
            if self.combo_topic.itemData(i) == current:
                self.combo_topic.setCurrentIndex(i)
                break
        self.combo_topic.blockSignals(False)

    def _on_topic_changed(self, _index: int) -> None:
        self._destroy_subscription()
        topic = self.combo_topic.currentData()
        if not topic or self._node is None:
            self.image_label.setText("Ei kuvaa")
            return
        self._clear_latest_image()
        self._sub = self._node.create_subscription(
            RosImage,
            topic,
            self._on_image,
            self._image_qos(),
        )

    def _destroy_subscription(self) -> None:
        if self._sub is not None and self._node is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    def _on_image(self, msg: RosImage) -> None:
        if self._is_stale(msg):
            return
        with self._frame_condition:
            self._latest_msg = msg
            self._latest_sequence += 1
            self._frame_condition.notify()

    def _display_latest_image(self) -> None:
        target_size = (self.image_label.width(), self.image_label.height())
        with self._frame_condition:
            self._target_size = target_size
            if (
                self._converted_qimage is None
                or self._converted_qimage_sequence == self._displayed_qimage_sequence
            ):
                return
            qimg = self._converted_qimage
            sequence = self._converted_qimage_sequence
            stamp_ns = self._converted_qimage_stamp_ns

        if self._is_stamp_stale(stamp_ns):
            with self._frame_condition:
                self._displayed_qimage_sequence = max(
                    self._displayed_qimage_sequence,
                    sequence,
                )
            return

        self._update_image(qimg)
        with self._frame_condition:
            self._displayed_qimage_sequence = max(
                self._displayed_qimage_sequence,
                sequence,
            )

    def _update_image(self, qimg: QImage) -> None:
        if self.image_label.width() <= 1 or self.image_label.height() <= 1:
            return

        self.image_label.setPixmap(QPixmap.fromImage(qimg))

    def _clear_latest_image(self) -> None:
        with self._frame_condition:
            self._topic_generation += 1
            self._latest_msg = None
            self._latest_sequence += 1
            self._converted_input_sequence = self._latest_sequence
            self._converted_qimage = None
            self._converted_qimage_sequence = self._latest_sequence
            self._converted_qimage_stamp_ns = 0
            self._displayed_qimage_sequence = self._latest_sequence
            self._frame_condition.notify_all()

    def shutdown(self) -> None:
        self.detach_node()
        self._display_timer.stop()
        with self._frame_condition:
            self._stop_converter = True
            self._frame_condition.notify_all()
        self._converter_thread.join(timeout=2.0)

    def _conversion_loop(self) -> None:
        while True:
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: self._stop_converter
                    or (
                        self._latest_msg is not None
                        and self._latest_sequence != self._converted_input_sequence
                    )
                )
                if self._stop_converter:
                    return
                msg = self._latest_msg
                sequence = self._latest_sequence
                generation = self._topic_generation
                target_size = self._target_size

            if msg is None:
                continue

            stamp_ns = self._stamp_ns(msg)
            qimg = None
            if not self._is_stamp_stale(stamp_ns):
                qimg = self._ros_image_to_qimage(msg)
                if qimg is not None:
                    qimg = self._scale_qimage_for_display(qimg, target_size)

            with self._frame_condition:
                if generation != self._topic_generation:
                    continue
                self._converted_input_sequence = max(
                    self._converted_input_sequence,
                    sequence,
                )
                if qimg is not None:
                    self._converted_qimage = qimg
                    self._converted_qimage_sequence = sequence
                    self._converted_qimage_stamp_ns = stamp_ns

    @staticmethod
    def _image_qos() -> QoSProfile:
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

    @staticmethod
    def _display_interval_ms() -> int:
        raw_fps = os.environ.get("SOP_ROBOT_IMAGE_VIEWER_FPS", "")
        try:
            fps = int(raw_fps) if raw_fps else DEFAULT_DISPLAY_FPS
        except ValueError:
            fps = DEFAULT_DISPLAY_FPS

        fps = max(1, min(fps, MAX_DISPLAY_FPS))
        return max(1, int(1000 / fps))

    def _is_stale(self, msg: RosImage) -> bool:
        return self._is_stamp_stale(self._stamp_ns(msg))

    def _is_stamp_stale(self, stamp_ns: int) -> bool:
        if self._node is None or self._max_frame_age_seconds <= 0:
            return False

        if stamp_ns <= 0:
            return False

        now_ns = self._node.get_clock().now().nanoseconds
        return (now_ns - stamp_ns) / 1_000_000_000 > self._max_frame_age_seconds

    @staticmethod
    def _stamp_ns(msg: RosImage) -> int:
        stamp = msg.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _ros_image_to_qimage(msg: RosImage) -> QImage | None:
        encoding = msg.encoding
        height, width = msg.height, msg.width

        if encoding in ("bgr8", "rgb8"):
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            if encoding == "bgr8":
                data = data[:, :, ::-1].copy()
            return QImage(
                data.data,
                width,
                height,
                width * 3,
                QImage.Format.Format_RGB888,
            ).copy()

        if encoding == "mono8":
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            return QImage(
                data.data,
                width,
                height,
                width,
                QImage.Format.Format_Grayscale8,
            ).copy()

        if encoding in ("bgra8", "rgba8"):
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            if encoding == "bgra8":
                data = data[:, :, [2, 1, 0, 3]].copy()
            return QImage(
                data.data,
                width,
                height,
                width * 4,
                QImage.Format.Format_RGBA8888,
            ).copy()

        return None

    @staticmethod
    def _scale_qimage_for_display(qimg: QImage, target_size: tuple[int, int]) -> QImage:
        target_width, target_height = target_size
        if target_width <= 1 or target_height <= 1:
            return qimg
        return qimg.scaled(
            target_width,
            target_height,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )
