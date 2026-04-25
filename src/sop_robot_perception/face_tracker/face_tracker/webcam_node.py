# pyright: reportAttributeAccessIssue=false

import cv2
import rclpy
import os
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
IMAGE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

class WebcamError(Exception):
    """signal that webcam has stopped working"""
    pass

class WebCamNode(Node):
    def __init__(self):
        super().__init__("webcam")

        self.logger = self.get_logger()

        # Node parameters
        raw_image_topic = (
            self.declare_parameter(
                "raw_image", "/raw_image"
            )
            .get_parameter_value()
            .string_value
        )
        
        self.index = (
            self.declare_parameter("index", 0)
            .get_parameter_value().integer_value
        )
        source_param = self.declare_parameter(
            "source",
            "",
            ParameterDescriptor(dynamic_typing=True),
        ).value
        self.source = str(source_param).strip()
        self.width = (
            self.declare_parameter("width", 0)
            .get_parameter_value().integer_value
        )
        self.height = (
            self.declare_parameter("height", 0)
            .get_parameter_value().integer_value
        )
        self.fps = (
            self.declare_parameter("fps", 0)
            .get_parameter_value().integer_value
        )
        self.capture_buffer_size = (
            self.declare_parameter("capture_buffer_size", 1)
            .get_parameter_value().integer_value
        )
        self.read_warning_seconds = (
            self.declare_parameter("read_warning_seconds", 1.0)
            .get_parameter_value().double_value
        )
        self.reconnect_cooldown_seconds = (
            self.declare_parameter("reconnect_cooldown_seconds", 2.0)
            .get_parameter_value().double_value
        )
        self.status_log_interval_seconds = (
            self.declare_parameter("status_log_interval_seconds", 5.0)
            .get_parameter_value().double_value
        )
        self.low_latency_stream = (
            self.declare_parameter("low_latency_stream", True)
            .get_parameter_value()._bool_value
        )
        self.async_capture = (
            self.declare_parameter("async_capture", True)
            .get_parameter_value()._bool_value
        )
        self.mjpg = (
            self.declare_parameter("mjpg", False)
            .get_parameter_value()._bool_value
        )
        self.requested_publish_fps = float(self.fps if self.fps and self.fps > 0 else 30)
        self.source_reported_fps = 0.0
        self._published_frame_count = 0
        self._captured_frame_count = 0
        self._overwritten_frame_count = 0
        self._capture_restart_count = 0
        self._last_reconnect_attempt = 0.0
        self._status_window_started = time.monotonic()
        self._last_published_shape = None
        self._latest_frame = None
        self._latest_frame_stamp = None
        self._latest_frame_monotonic = 0.0
        self._latest_frame_sequence = 0
        self._published_frame_sequence = 0
        self._latest_frame_lock = threading.Lock()
        self._stop_capture_thread = threading.Event()
        self._capture_thread = None

        self.logger.info("Webcam node parameters:\n" +
                         f"source={self.source or '<device index>'}\n" +
                         f"index={self.index}\n" +
                         f"width={self.width}\n" +
                         f"height={self.height}\n" +
                         f"fps={self.fps}\n" +
                         f"capture_buffer_size={self.capture_buffer_size}\n" +
                         f"read_warning_seconds={self.read_warning_seconds}\n" +
                         f"reconnect_cooldown_seconds={self.reconnect_cooldown_seconds}\n" +
                         f"status_log_interval_seconds={self.status_log_interval_seconds}\n" +
                         f"low_latency_stream={self.low_latency_stream}\n" +
                         f"async_capture={self.async_capture}\n" +
                         f"mjpg={self.mjpg}")

        self.capture_source = self._resolve_capture_source()

        self.face_img_publisher = self.create_publisher(Image, raw_image_topic, IMAGE_QOS)
        timer_period = 1.0 / self.requested_publish_fps
        self.cap = None
        self.open_webcam()
        if self.async_capture:
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                name="webcam_latest_frame_capture",
                daemon=True,
            )
            self._capture_thread.start()
            self.capture_timer = self.create_timer(timer_period, self.publish_latest_frame)
        else:
            self.capture_timer = self.create_timer(timer_period, self.capture_frame)

    def _resolve_capture_source(self):
        """Return either an integer camera index or a stream URL/file path."""
        if not self.source:
            return self.index
        if self.source.lstrip("+-").isdigit():
            return int(self.source)
        return self.source

    def _is_network_stream(self):
        return isinstance(self.capture_source, str) and "://" in self.capture_source

    def capture_frame(self):
        frame, stamp, _captured_at = self._read_frame_from_capture()
        if frame is None:
            return
        self._publish_frame(frame, stamp)

    def _capture_loop(self):
        while not self._stop_capture_thread.is_set():
            frame, stamp, captured_at = self._read_frame_from_capture()
            if frame is None:
                time.sleep(0.1)
                continue

            with self._latest_frame_lock:
                if self._latest_frame_sequence != self._published_frame_sequence:
                    self._overwritten_frame_count += 1
                self._latest_frame = frame
                self._latest_frame_stamp = stamp
                self._latest_frame_monotonic = captured_at
                self._latest_frame_sequence += 1
                self._captured_frame_count += 1

    def publish_latest_frame(self):
        should_log_status = False
        with self._latest_frame_lock:
            if (
                self._latest_frame is None
                or self._latest_frame_sequence == self._published_frame_sequence
            ):
                should_log_status = True
                frame = None
                stamp = None
                sequence = 0
            else:
                frame = self._latest_frame
                stamp = self._latest_frame_stamp
                sequence = self._latest_frame_sequence

        if should_log_status:
            self._log_status_if_due()
            return

        if self._publish_frame(frame, stamp):
            with self._latest_frame_lock:
                self._published_frame_sequence = max(
                    self._published_frame_sequence,
                    sequence,
                )

    def _read_frame_from_capture(self):
        if not self._ensure_webcam_open():
            return None, None, 0.0
        cap = self.cap
        if cap is None:
            return None, None, 0.0

        read_started = time.monotonic()
        try:
            ret, frame = cap.read()
            if not ret:
                raise WebcamError
        except WebcamError:
            self.logger.error("Webcam frame capture failed, restarting device")
            self._restart_webcam()
            return None, None, 0.0
        except Exception as exc:
            self.logger.error(f"Webcam frame capture raised {type(exc).__name__}: {exc}")
            self._restart_webcam()
            return None, None, 0.0

        read_elapsed = time.monotonic() - read_started
        if self.read_warning_seconds > 0 and read_elapsed > self.read_warning_seconds:
            self.logger.warning(
                f"Webcam frame read took {read_elapsed:.2f}s. "
                "The camera stream may be stalled or buffering."
            )

        frame = self._resize_frame_for_publish(frame)
        return frame, self.get_clock().now().to_msg(), time.monotonic()

    def _ensure_webcam_open(self):
        if self.cap is not None and self.cap.isOpened():
            return True

        now = time.monotonic()
        if now - self._last_reconnect_attempt < self.reconnect_cooldown_seconds:
            return False

        self.logger.error("Webcam is not available, attempting reopen")
        self._last_reconnect_attempt = now
        self.open_webcam()
        return self.cap is not None and self.cap.isOpened()

    def _restart_webcam(self):
        self.close_webcam()
        self._capture_restart_count += 1
        self._last_reconnect_attempt = 0.0
        self._ensure_webcam_open()

    def _publish_frame(self, frame, stamp):
        try:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = stamp or self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera"
            self.face_img_publisher.publish(image_msg)
            self._published_frame_count += 1
            self._last_published_shape = frame.shape[:2]
            self._log_status_if_due()
            return True
        except CvBridgeError as exc:
            self.logger.warning(f"Could not convert OpenCV image to ROS image: {exc}")
            return False

    def _log_status_if_due(self):
        if self.status_log_interval_seconds <= 0:
            return

        now = time.monotonic()
        elapsed = now - self._status_window_started
        if elapsed < self.status_log_interval_seconds:
            return

        measured_fps = self._published_frame_count / elapsed if elapsed > 0 else 0.0
        if self._last_published_shape is None:
            height, width = 0, 0
        else:
            height, width = self._last_published_shape

        with self._latest_frame_lock:
            captured_frame_count = self._captured_frame_count
            overwritten_frame_count = self._overwritten_frame_count
            latest_frame_age = (
                now - self._latest_frame_monotonic
                if self._latest_frame_monotonic > 0
                else 0.0
            )
            self._captured_frame_count = 0
            self._overwritten_frame_count = 0

        self.logger.info(
            "Webcam FPS status: "
            f"source_reported={self.source_reported_fps:.2f}, "
            f"requested_publish={self.requested_publish_fps:.2f}, "
            f"measured_publish={measured_fps:.2f}, "
            f"captured_fps={captured_frame_count / elapsed:.2f}, "
            f"overwritten_frames={overwritten_frame_count}, "
            f"capture_restarts={self._capture_restart_count}, "
            f"latest_age_ms={latest_frame_age * 1000.0:.1f}, "
            f"published_shape={width}x{height}"
        )
        self._published_frame_count = 0
        self._status_window_started = now

    def _resize_frame_for_publish(self, frame):
        if self.width <= 0 and self.height <= 0:
            return frame

        current_height, current_width = frame.shape[:2]
        target_width = self.width
        target_height = self.height

        if target_width <= 0:
            target_width = max(1, int(round(current_width * target_height / current_height)))
        if target_height <= 0:
            target_height = max(1, int(round(current_height * target_width / current_width)))

        if current_width == target_width and current_height == target_height:
            return frame

        return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)

    def open_webcam(self):
        '''
        Open webcam handle
        '''
        if self.low_latency_stream and self._is_network_stream():
            os.environ.setdefault(
                "OPENCV_FFMPEG_CAPTURE_OPTIONS",
                "fflags;nobuffer|flags;low_delay|max_delay;0",
            )

        self.cap = cv2.VideoCapture(self.capture_source)

        if not self.cap.isOpened():
            self.logger.error(f"Cannot open webcam source '{self.capture_source}'")
            self.cap.release()
            self.cap = None
            return

        # Device property tuning is only reliable for local capture devices.
        if isinstance(self.capture_source, int):
            if self.width:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.mjpg:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            if self.fps:
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        elif self.fps:
            self.logger.info(
                "Network/file camera source detected. The fps parameter controls the "
                "ROS publish timer; OpenCV cannot force the source stream FPS."
            )

        if self.capture_buffer_size > 0:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.capture_buffer_size)

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.source_reported_fps = float(fps or 0.0)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.logger.info(
            f"Opened webcam source '{self.capture_source}' with "
            f"source_reported_fps={self.source_reported_fps:.2f}, "
            f"requested_publish_fps={self.requested_publish_fps:.2f}, "
            f"shape={w}x{h}"
        )

        if (
            self._is_network_stream()
            and self.source_reported_fps > 0
            and abs(self.source_reported_fps - self.requested_publish_fps) > 0.5
        ):
            self.logger.warning(
                "Camera stream reports a different FPS than the ROS publish target: "
                f"source={self.source_reported_fps:.2f}, "
                f"requested_publish={self.requested_publish_fps:.2f}. "
                "For DroidCam, change the phone/server stream FPS or match webcam_fps "
                "to the reported source FPS to avoid blocked reads."
            )


    def close_webcam(self):
        '''
        Destroy webcam handle and close all windows
        '''
        if self.cap is None:
            return
        self.logger.info("closing webcam handle...")
        self.cap.release()
        self.cap = None
        cv2.destroyAllWindows()
        self.logger.info("Webcam closed!")

    def destroy_node(self):
        self._stop_capture_thread.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)
            if self._capture_thread.is_alive():
                self.logger.warning("Webcam capture thread did not stop cleanly.")
        self.close_webcam()
        return super().destroy_node()

def main(args=None):
    # Initialize
    rclpy.init(args=args)
    webcam = WebCamNode()

    rclpy.spin(webcam)
    webcam.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
