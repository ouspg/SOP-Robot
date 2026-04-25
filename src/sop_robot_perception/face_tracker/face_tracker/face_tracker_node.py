# pyright: reportAttributeAccessIssue=false

import rclpy
import cv2
import dlib
import math
import os
import threading
import time
import traceback

from ament_index_python.packages import get_package_share_directory
from sop_robot_common.contracts import FACE_IMAGE_TOPIC, FACES_TOPIC, RAW_IMAGE_TOPIC

from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image
from face_tracker_msgs.msg import Faces, Face as FaceMsg, Point2, Occurance

from cv_bridge import CvBridge, CvBridgeError

from .face_analyzer import FaceAnalyzer

bridge = CvBridge()
IMAGE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

class WebcamError(Exception):
    """signal that webcam has stopped working"""
    pass

# pr = cProfile.Profile()

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__("face_tracker_node")
        self.logger = self.get_logger()

        lip_movement_detection = (
            self.declare_parameter("lip_movement_detection", True)
            .get_parameter_value()
            ._bool_value
        )

        face_recognition = (
            self.declare_parameter("face_recognition", True)
            .get_parameter_value()
            ._bool_value
        )

        correlation_tracking = (
            self.declare_parameter("correlation_tracking", True)
            .get_parameter_value()
            ._bool_value
        )

        cluster_similarity_threshold = (
            self.declare_parameter("cluster_similarity_threshold", 0.3)
            .get_parameter_value()
            .double_value
        )

        subcluster_similarity_threshold = (
            self.declare_parameter("subcluster_similarity_threshold", 0.2)
            .get_parameter_value()
            .double_value
        )

        pair_similarity_maximum = (
            self.declare_parameter("pair_similarity_maximum", 1.0)
            .get_parameter_value()
            .double_value
        )

        face_recognition_model = (
            self.declare_parameter("face_recognition_model", "SFace")
            .get_parameter_value()
            .string_value
        )

        face_detection_model = (
            self.declare_parameter("face_detection_model", "yunet")
            .get_parameter_value()
            .string_value
        )

        prefer_gpu = (
            self.declare_parameter("prefer_gpu", False)
            .get_parameter_value()
            ._bool_value
        )

        gpu_face_recognition_model = (
            self.declare_parameter("gpu_face_recognition_model", "SFace")
            .get_parameter_value()
            .string_value
        )

        gpu_face_detection_model = (
            self.declare_parameter("gpu_face_detection_model", "yolov8n")
            .get_parameter_value()
            .string_value
        )

        face_detection_confidence = (
            self.declare_parameter("face_detection_confidence", 0.6)
            .get_parameter_value()
            .double_value
        )

        face_detection_imgsz = (
            self.declare_parameter("face_detection_imgsz", 640)
            .get_parameter_value()
            .integer_value
        )

        no_face_detection_interval_frames = (
            self.declare_parameter("no_face_detection_interval_frames", 1)
            .get_parameter_value()
            .integer_value
        )

        no_face_detection_warmup_frames = (
            self.declare_parameter("no_face_detection_warmup_frames", 3)
            .get_parameter_value()
            .integer_value
        )

        face_detection_interval_frames = (
            self.declare_parameter("face_detection_interval_frames", 5)
            .get_parameter_value()
            .integer_value
        )

        face_identity_refresh_seconds = (
            self.declare_parameter("face_identity_refresh_seconds", 3.0)
            .get_parameter_value()
            .double_value
        )

        track_match_iou_threshold = (
            self.declare_parameter("track_match_iou_threshold", 0.25)
            .get_parameter_value()
            .double_value
        )

        recent_face_memory_seconds = (
            self.declare_parameter("recent_face_memory_seconds", 2.0)
            .get_parameter_value()
            .double_value
        )

        identity_store_max_identities = (
            self.declare_parameter("identity_store_max_identities", 8)
            .get_parameter_value()
            .integer_value
        )

        identity_store_ttl_seconds = (
            self.declare_parameter("identity_store_ttl_seconds", 300.0)
            .get_parameter_value()
            .double_value
        )

        self.max_published_occurances = (
            self.declare_parameter("max_published_occurances", 5)
            .get_parameter_value()
            .integer_value
        )

        self.max_processing_fps = (
            self.declare_parameter("max_processing_fps", 0.0)
            .get_parameter_value()
            .double_value
        )

        self.processing_width = (
            self.declare_parameter("processing_width", 640)
            .get_parameter_value()
            .integer_value
        )

        self.publish_face_image = (
            self.declare_parameter("publish_face_image", True)
            .get_parameter_value()
            ._bool_value
        )

        self.face_image_publish_every_n_frames = max(
            1,
            self.declare_parameter("face_image_publish_every_n_frames", 2)
            .get_parameter_value()
            .integer_value,
        )

        self.face_image_publish_fps = max(
            0.0,
            self.declare_parameter("face_image_publish_fps", 0.0)
            .get_parameter_value()
            .double_value,
        )

        self.face_image_max_width = (
            self.declare_parameter("face_image_max_width", 640)
            .get_parameter_value()
            .integer_value
        )

        self.async_processing = (
            self.declare_parameter("async_processing", True)
            .get_parameter_value()
            ._bool_value
        )

        self.slow_frame_warning_seconds = (
            self.declare_parameter("slow_frame_warning_seconds", 1.0)
            .get_parameter_value()
            .double_value
        )

        self.processing_status_log_interval_seconds = (
            self.declare_parameter("processing_status_log_interval_seconds", 5.0)
            .get_parameter_value()
            .double_value
        )

        image_topic = (
            self.declare_parameter("image_topic", RAW_IMAGE_TOPIC)
            .get_parameter_value()
            .string_value
        )

        face_image_topic = (
            self.declare_parameter(
                "face_image_topic", FACE_IMAGE_TOPIC
            )  # non-absolute paths are inside the current node namespace
            .get_parameter_value()
            .string_value
        )

        face_topic = (
            self.declare_parameter(
                "face_topic", FACES_TOPIC
            )  # non-absolute paths are inside the current node namespace
            .get_parameter_value()
            .string_value
        )

        predictor = (
            self.declare_parameter("predictor", "shape_predictor_68_face_landmarks.dat")
            .get_parameter_value()
            .string_value
        )

        lip_movement_detector = None
        if lip_movement_detection:
            self.predictor = dlib.shape_predictor(
                os.path.join(
                    get_package_share_directory("face_tracker"),
                    "predictors",
                    predictor,
                )
            )
            lip_movement_detector_model = (
                self.declare_parameter("lip_movement_detector", "1_32_False_True_0.25_lip_motion_net_model.h5")
                .get_parameter_value()
                .string_value
            )
           # Initialize lip movement detector
            from .lip_movement_net import LipMovementDetector
            self.logger.info('Initializing lip movement detector...')
            lip_movement_detector = LipMovementDetector(
                os.path.join(
                    get_package_share_directory("face_tracker"),
                    "models",
                    lip_movement_detector_model,
                ),
                self.predictor
            )
            self.logger.info('Lip movement detector initialized.')
        else:
            self.logger.info('Lip movement detection disabled.')

        self.face_tracker = FaceAnalyzer(self.logger.get_child("Face_Analyzer"),
                                        lip_movement_detector,
                                        face_recognition,
                                        correlation_tracking,
                                        cluster_similarity_threshold,
                                        subcluster_similarity_threshold,
                                        pair_similarity_maximum,
                                        face_recognition_model,
                                        face_detection_model,
                                        prefer_gpu,
                                        gpu_face_recognition_model,
                                        gpu_face_detection_model,
                                        face_detection_confidence,
                                        face_detection_imgsz,
                                        no_face_detection_interval_frames,
                                        no_face_detection_warmup_frames,
                                        face_detection_interval_frames,
                                        face_identity_refresh_seconds,
                                        track_match_iou_threshold,
                                        recent_face_memory_seconds,
                                        identity_store_max_identities,
                                        identity_store_ttl_seconds,
                                        draw_debug=False)

        # Create subscription, that receives camera frames
        self.subscriber = self.create_subscription(
            Image,
            image_topic,
            self.on_frame_received,
            IMAGE_QOS,
        )
        self.face_img_publisher = self.create_publisher(Image, face_image_topic, IMAGE_QOS)
        self.face_publisher = self.create_publisher(Faces, face_topic, 1)
        self._latest_face_image_msg = None
        self._latest_face_image_sequence = 0
        self._published_face_image_sequence = 0
        self._face_image_lock = threading.Lock()
        self._face_image_timer = None
        if self.publish_face_image and self.face_image_publish_fps > 0:
            self._face_image_timer = self.create_timer(
                1.0 / self.face_image_publish_fps,
                self._publish_latest_face_image,
            )

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.fps = FramesPerSecond()
        self.fps.start()
        self.processed_frames = 0
        self._profile_received_frames = 0
        self._profile_rate_skipped_frames = 0
        self._profile_overwritten_pending_frames = 0
        self._last_slow_frame_warning = 0.0
        self._processing_rate_tokens = 1.0
        self._processing_rate_updated_at = time.monotonic()
        self._processing_profile_started_at = time.monotonic()
        self._processing_profile_samples = []
        self._last_debug_image_shape = "not published"

        self._latest_image_msg = None
        self._latest_image_sequence = 0
        self._processed_image_sequence = 0
        self._stop_processing = False
        self._frame_condition = threading.Condition()
        self._processing_thread = None
        if self.async_processing:
            self._processing_thread = threading.Thread(
                target=self._processing_loop,
                name="face_tracker_processing",
                daemon=True,
            )
            self._processing_thread.start()

    #     self.timer = self.create_timer(2, self.profile_cycle)
    #     pr.enable()

    # def profile_cycle(self):
    #     global pr
    #     pr.disable()
    #     s = io.StringIO()
    #     ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
    #     ps.print_stats()
    #     self.logger.info("Profiler: -----------------------------")
    #     self.logger.info(s.getvalue())
    #     self.logger.info("Profiler: -----------------------------")
    #     # Save result to file
    #     pr.create_stats()
    #     filename = f"profiler.prof"
    #     pr.dump_stats(filename)

    #     pr = cProfile.Profile()
    #     pr.enable()

    def on_frame_received(self, img: Image):
        self._profile_received_frames += 1
        if self.async_processing:
            with self._frame_condition:
                if self._latest_image_sequence != self._processed_image_sequence:
                    self._profile_overwritten_pending_frames += max(
                        1,
                        self._latest_image_sequence - self._processed_image_sequence,
                    )
                self._latest_image_msg = img
                self._latest_image_sequence += 1
                self._frame_condition.notify()
            return

        self._process_image_message(img)

    def _processing_loop(self):
        while True:
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: self._stop_processing
                    or self._latest_image_sequence != self._processed_image_sequence
                )
                if self._stop_processing:
                    return

                image_msg = self._latest_image_msg
                sequence = self._latest_image_sequence

            if image_msg is None:
                continue

            try:
                started_at = time.monotonic()
                self._process_image_message(image_msg)
                elapsed = time.monotonic() - started_at
                self._warn_if_slow_frame(elapsed)
            except Exception:
                self.logger.error(
                    "Face tracker processing worker failed:\n"
                    + traceback.format_exc()
                )
                time.sleep(0.25)
            finally:
                with self._frame_condition:
                    self._processed_image_sequence = max(
                        self._processed_image_sequence,
                        sequence,
                    )

    def _process_image_message(self, img: Image):
        if not self._consume_processing_rate_token():
            return

        profile_started_at = time.perf_counter()

        # convert ros img to opencv image
        try:
            step_started_at = time.perf_counter()
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            convert_ms = self._elapsed_ms(step_started_at)
        except CvBridgeError as e:
            self.logger.warning(f"Could not convert ros img to opencv image: {e}")
            return

        step_started_at = time.perf_counter()
        analysis_img, scale_x, scale_y = self._prepare_analysis_frame(cv2_bgr_img)
        prepare_analysis_ms = self._elapsed_ms(step_started_at)

        msg_faces = []
        step_started_at = time.perf_counter()
        faces = self.face_tracker.on_frame_received(analysis_img)
        analyze_ms = self._elapsed_ms(step_started_at)

        step_started_at = time.perf_counter()
        if scale_x != 1.0 or scale_y != 1.0:
            faces = [self._scale_face(face, scale_x, scale_y, cv2_bgr_img) for face in faces]

        # loop through all faces
        for face in faces:
            occurances = []
            previous_occurances = face["previous_occurances"]
            if self.max_published_occurances > 0:
                previous_occurances = previous_occurances[-self.max_published_occurances:]
            for i in previous_occurances:
                occurance = Occurance(start_time=float(i["start_time"]), end_time=float(i["end_time"]), duration=float(i["duration"]))
                occurances.append(occurance)
            msg_face = FaceMsg(top_left=Point2(x=face["left"], y=face["top"]),
                               bottom_right=Point2(x=face["right"], y=face["bottom"]),
                               diagonal=face["diagonal"],
                               face_id=face["face_id"],
                               speaking=face["speaking"],
                               occurances=occurances)
            msg_faces.append(msg_face)
        build_face_msg_ms = self._elapsed_ms(step_started_at)

        should_publish_face_image = (
            self.publish_face_image
            and self.processed_frames % self.face_image_publish_every_n_frames == 0
        )

        debug_prepare_ms = 0.0
        debug_draw_ms = 0.0
        debug_publish_ms = 0.0
        if should_publish_face_image:
            step_started_at = time.perf_counter()
            debug_img = self._prepare_debug_publish_frame(cv2_bgr_img)
            debug_faces = self._scale_faces_for_frame(faces, cv2_bgr_img, debug_img)
            self._last_debug_image_shape = f"{debug_img.shape[1]}x{debug_img.shape[0]}"
            debug_prepare_ms = self._elapsed_ms(step_started_at)

            step_started_at = time.perf_counter()
            self._draw_debug_overlay(debug_img, debug_faces)
            debug_draw_ms = self._elapsed_ms(step_started_at)

            try:
                step_started_at = time.perf_counter()
                debug_msg = bridge.cv2_to_imgmsg(debug_img, "bgr8")
                debug_msg.header = img.header
                self._publish_or_queue_face_image(debug_msg)
                debug_publish_ms = self._elapsed_ms(step_started_at)
            except CvBridgeError as e:
                self.logger.warning(f"Could not convert ros img to opencv image: {e}")

        step_started_at = time.perf_counter()
        # Publish faces info if faces found
        if len(msg_faces) > 0:
            self.face_publisher.publish(Faces(faces=msg_faces))
        publish_faces_ms = self._elapsed_ms(step_started_at)

        self.fps.update_fps()
        self.processed_frames += 1
        self._record_processing_profile(
            {
                "convert": convert_ms,
                "prepare": prepare_analysis_ms,
                "analyze": analyze_ms,
                "build_msg": build_face_msg_ms,
                "debug_prepare": debug_prepare_ms,
                "debug_draw": debug_draw_ms,
                "debug_publish": debug_publish_ms,
                "publish_faces": publish_faces_ms,
                "total": self._elapsed_ms(profile_started_at),
                "faces": float(len(faces)),
                "debug_published": 1.0 if should_publish_face_image else 0.0,
            }
        )

    def _warn_if_slow_frame(self, elapsed_seconds: float):
        if self.slow_frame_warning_seconds <= 0:
            return
        if elapsed_seconds < self.slow_frame_warning_seconds:
            return

        now = time.monotonic()
        if now - self._last_slow_frame_warning < 5.0:
            return

        self._last_slow_frame_warning = now
        self.logger.warning(
            f"Face tracker frame processing took {elapsed_seconds:.2f}s. "
            "Newer camera frames are being kept while stale frames are dropped."
        )

    def _consume_processing_rate_token(self):
        if self.max_processing_fps <= 0:
            return True

        now = time.monotonic()
        elapsed = max(0.0, now - self._processing_rate_updated_at)
        self._processing_rate_updated_at = now

        # Keep the average cap, but allow a small burst so jittery 28-30 FPS
        # cameras do not lose every frame that arrives a few milliseconds early.
        self._processing_rate_tokens = min(
            2.0,
            self._processing_rate_tokens + elapsed * self.max_processing_fps,
        )
        if self._processing_rate_tokens < 1.0:
            self._profile_rate_skipped_frames += 1
            return False

        self._processing_rate_tokens -= 1.0
        return True

    def _prepare_analysis_frame(self, frame):
        height, width = frame.shape[:2]
        if self.processing_width <= 0 or width <= self.processing_width:
            return frame, 1.0, 1.0

        analysis_width = int(self.processing_width)
        analysis_height = max(1, int(round(height * analysis_width / width)))
        analysis_img = cv2.resize(
            frame,
            (analysis_width, analysis_height),
            interpolation=cv2.INTER_AREA,
        )
        return analysis_img, width / analysis_width, height / analysis_height

    def _scale_face(self, face, scale_x, scale_y, frame):
        height, width = frame.shape[:2]
        scaled_face = dict(face)
        scaled_face["left"] = self._scale_coordinate(face["left"], scale_x, width - 1)
        scaled_face["right"] = self._scale_coordinate(face["right"], scale_x, width - 1)
        scaled_face["top"] = self._scale_coordinate(face["top"], scale_y, height - 1)
        scaled_face["bottom"] = self._scale_coordinate(face["bottom"], scale_y, height - 1)
        scaled_face["diagonal"] = face["diagonal"] * ((scale_x + scale_y) / 2.0)
        return scaled_face

    @staticmethod
    def _scale_coordinate(value, scale, upper_bound):
        return max(0, min(int(round(value * scale)), upper_bound))

    def _prepare_debug_publish_frame(self, frame):
        height, width = frame.shape[:2]
        if self.face_image_max_width <= 0 or width <= self.face_image_max_width:
            return frame

        target_width = int(self.face_image_max_width)
        target_height = max(1, int(round(height * target_width / width)))
        return cv2.resize(
            frame,
            (target_width, target_height),
            interpolation=cv2.INTER_AREA,
        )

    def _publish_or_queue_face_image(self, msg):
        if self.face_image_publish_fps <= 0:
            self.face_img_publisher.publish(msg)
            return

        with self._face_image_lock:
            self._latest_face_image_msg = msg
            self._latest_face_image_sequence += 1

    def _publish_latest_face_image(self):
        with self._face_image_lock:
            if (
                self._latest_face_image_msg is None
                or self._latest_face_image_sequence
                == self._published_face_image_sequence
            ):
                return
            msg = self._latest_face_image_msg
            sequence = self._latest_face_image_sequence

        self.face_img_publisher.publish(msg)

        with self._face_image_lock:
            self._published_face_image_sequence = max(
                self._published_face_image_sequence,
                sequence,
            )

    def _scale_faces_for_frame(self, faces, source_frame, target_frame):
        source_height, source_width = source_frame.shape[:2]
        target_height, target_width = target_frame.shape[:2]
        if source_width == target_width and source_height == target_height:
            return faces

        scale_x = target_width / source_width
        scale_y = target_height / source_height
        return [self._scale_face(face, scale_x, scale_y, target_frame) for face in faces]

    def _draw_debug_overlay(self, frame, faces):
        for face in faces:
            self._draw_face_info(frame, face)

        cv2.putText(frame,
                    '%.2f' % self.fps.fps,
                    (10, 20),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        cv2.putText(frame,
                    f"Faces in current frame {len(faces)}",
                    (100,10),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

        cv2.putText(frame,
                    f"Clusters in database {len(self.face_tracker.cluster.clusters)}",
                    (100,30),
                    self.font,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA)

    def _draw_face_info(self, frame, face):
        green = (0, 255, 0)
        cv2.rectangle(
            frame,
            (face["left"], face["top"]),
            (face["right"], face["bottom"]),
            green,
            1,
        )

        if face["speaking"] != "":
            cv2.putText(frame,
                        f"Face is speaking: {face['speaking']}",
                        (face["left"] + 2, face["top"] + 20),
                        self.font,
                        0.3,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)

        if face["face_id"]:
            cv2.putText(frame,
                        f"Matching cluster: {face['face_id']}",
                        (face["left"] + 2, face["top"] + 10),
                        self.font,
                        0.3,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)

    @staticmethod
    def _elapsed_ms(started_at):
        return (time.perf_counter() - started_at) * 1000.0

    def _record_processing_profile(self, sample):
        if self.processing_status_log_interval_seconds <= 0:
            return

        self._processing_profile_samples.append(sample)
        now = time.monotonic()
        elapsed = now - self._processing_profile_started_at
        if elapsed < self.processing_status_log_interval_seconds:
            return

        samples = self._processing_profile_samples
        self._processing_profile_samples = []
        self._processing_profile_started_at = now
        if not samples or elapsed <= 0:
            return
        received_frames = self._profile_received_frames
        rate_skipped_frames = self._profile_rate_skipped_frames
        overwritten_pending_frames = self._profile_overwritten_pending_frames
        self._profile_received_frames = 0
        self._profile_rate_skipped_frames = 0
        self._profile_overwritten_pending_frames = 0

        def average(key):
            return sum(item[key] for item in samples) / len(samples)

        def percentile_95(key):
            values = sorted(item[key] for item in samples)
            index = max(0, math.ceil(len(values) * 0.95) - 1)
            return values[index]

        debug_avg = (
            average("debug_prepare")
            + average("debug_draw")
            + average("debug_publish")
        )
        debug_publish_fps = sum(item["debug_published"] for item in samples) / elapsed
        self.logger.info(
            "Face tracker profile: "
            f"input_fps={received_frames / elapsed:.1f}, "
            f"processed_fps={len(samples) / elapsed:.1f}, "
            f"debug_fps={debug_publish_fps:.1f}, "
            f"total_avg={average('total'):.1f}ms, "
            f"total_p95={percentile_95('total'):.1f}ms, "
            f"convert_avg={average('convert'):.1f}ms, "
            f"prepare_avg={average('prepare'):.1f}ms, "
            f"analyze_avg={average('analyze'):.1f}ms, "
            f"msg_avg={average('build_msg'):.1f}ms, "
            f"debug_avg={debug_avg:.1f}ms, "
            f"rate_skips={rate_skipped_frames}, "
            f"pending_overwrites={overwritten_pending_frames}, "
            f"faces_avg={average('faces'):.2f}, "
            f"debug_image={self._last_debug_image_shape}"
        )

    def destroy_node(self):
        if self._processing_thread is not None:
            with self._frame_condition:
                self._stop_processing = True
                self._frame_condition.notify_all()
            self._processing_thread.join(timeout=2.0)
            if self._processing_thread.is_alive():
                self.logger.warning("Face tracker processing thread did not stop cleanly.")
        return super().destroy_node()

class FramesPerSecond:
    """
    Class for calculating real time fps of video stream. Code is based from stack owerflow thread:
    https://stackoverflow.com/questions/55154753/trouble-calculating-fps-on-output-video-stream
    """
    def __init__(self):
        self.startTime = None
        self.total_number_of_frames = 0
        self.counter = 0
        self.frameRate = 1  # The number of seconds to wait for each measurement.
        self.fps = 0

    def start(self):
        self.startTime = time.time()  # Returns a UNIX timestamp.

    def update_fps(self):
        self.total_number_of_frames += 1
        self.counter += 1  # Count will increase until the if condition executes.
        if self._elapsed_time() > self.frameRate:  # We measure the self only after 1 second has passed.
            self.fps = self.counter / self._elapsed_time()
            self.counter = 0  # reset the counter for next iteration.
            self.start()  # reset the start time.

    def _elapsed_time(self):
        if self.startTime is None:
            return 0.0
        return time.time() - self.startTime

def main(args=None):
    # Initialize
    rclpy.init(args=args)
    tracker = FaceTrackerNode()

    # Do work
    rclpy.spin(tracker)

    # Shutdown
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
