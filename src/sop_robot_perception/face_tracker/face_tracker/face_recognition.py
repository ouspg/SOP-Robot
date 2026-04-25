# pyright: reportPrivateImportUsage=false, reportCallIssue=false, reportArgumentType=false, reportOptionalMemberAccess=false, reportAttributeAccessIssue=false

from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

class FaceRecognizer(object):

    def __init__(
        self,
        logger,
        model_name,
        detector_backend,
        prefer_gpu=False,
        gpu_face_recognition_model="ArcFace",
        gpu_face_detection_model="yolov8n",
        face_detection_confidence=0.6,
        face_detection_imgsz=640,
    ):
        """
        Initialize face recognizer, and create embeddings in the intialization
        """
        self.logger = logger
        self.prefer_gpu = prefer_gpu
        self.cpu_detector_backend = detector_backend
        self.face_detection_confidence = min(
            1.0,
            max(0.0, float(face_detection_confidence)),
        )
        self.face_detection_imgsz = max(0, int(face_detection_imgsz))
        self._yunet_detector = None
        self._yunet_input_size = (0, 0)
        self._yolo_detector = None
        self._yolo_device = None
        self._deepface = None
        self._sface_recognizer = None
        self.model_name, self.detector_backend = self._resolve_runtime_models(
            model_name=model_name,
            detector_backend=detector_backend,
            gpu_face_recognition_model=gpu_face_recognition_model,
            gpu_face_detection_model=gpu_face_detection_model,
        )

        self.model = None
        self._build_direct_sface_recognizer()
        self._build_direct_yunet_detector()
        self._build_direct_yolo_detector()
        if not self._can_run_without_deepface():
            self._ensure_deepface()

        logger.info(f"facial recognition model {self.model_name} is ready")
        logger.info(f"face detection backend is '{self.detector_backend}'")

        self.logger.info("FaceRecognizer initialized!")

    def _can_run_without_deepface(self) -> bool:
        return self._sface_recognizer is not None and (
            self._yunet_detector is not None or self._yolo_detector is not None
        )

    def _ensure_deepface(self):
        if self._deepface is not None:
            return self._deepface

        try:
            from deepface import DeepFace
        except Exception as exc:
            raise RuntimeError(
                "DeepFace is required for the configured face recognition fallback "
                f"({self.model_name}/{self.detector_backend}) but could not be imported. "
                "Use the default SFace+YuNet OpenCV path, or fix the DeepFace/pandas/numpy "
                f"environment. Import error: {exc}"
            ) from exc

        self._deepface = DeepFace
        self.model = self._deepface.build_model(model_name=self.model_name)
        return self._deepface

    def _build_direct_sface_recognizer(self) -> None:
        if self.model_name != "SFace":
            return
        if not hasattr(cv2, "FaceRecognizerSF_create"):
            self.logger.warning(
                "OpenCV FaceRecognizerSF is not available. Falling back to DeepFace SFace."
            )
            return

        weights_path = self._resolve_sface_weights_path()
        if weights_path is None:
            self.logger.warning(
                "Direct OpenCV SFace requested, but no local SFace weights were found. "
                "Falling back to DeepFace SFace."
            )
            return

        try:
            self._sface_recognizer = cv2.FaceRecognizerSF_create(weights_path, "")
        except Exception as exc:
            self._sface_recognizer = None
            self.logger.warning(
                "Could not initialize direct OpenCV SFace recognizer. "
                f"Falling back to DeepFace SFace: {exc}"
            )
            return

        self.logger.info(f"Using direct OpenCV SFace recognizer {weights_path}")

    def _resolve_sface_weights_path(self) -> str | None:
        weights_dir = Path.home() / ".deepface" / "weights"
        candidates = [
            weights_dir / "face_recognition_sface_2021dec.onnx",
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return None

    def _resolve_runtime_models(
        self,
        model_name: str,
        detector_backend: str,
        gpu_face_recognition_model: str,
        gpu_face_detection_model: str,
    ) -> Tuple[str, str]:
        if not self.prefer_gpu:
            self.logger.info(
                "GPU acceleration disabled. Using configured face models as-is."
            )
            return model_name, detector_backend

        torch_gpu_available = self._configure_torch_gpu()
        effective_model_name = model_name
        effective_detector_backend = detector_backend
        switched = []
        tensorflow_gpu_available = False

        # YuNet is OpenCV-based and CPU-only here. Prefer a PyTorch YOLO detector when
        # CUDA is available. TensorFlow is only probed later if we still need it for a
        # TensorFlow-backed detector or recognition model.
        if (
            detector_backend == "yunet"
            and gpu_face_detection_model.startswith("yolo")
            and torch_gpu_available
            and self._has_ultralytics()
        ):
            effective_detector_backend = gpu_face_detection_model
            switched.append(
                f"detection backend {detector_backend} -> {effective_detector_backend}"
            )

        needs_tensorflow_probe = (
            (model_name == "SFace" and gpu_face_recognition_model != model_name)
            or (
                detector_backend == "yunet"
                and not gpu_face_detection_model.startswith("yolo")
            )
        )

        if needs_tensorflow_probe:
            tensorflow_gpu_available = self._configure_tensorflow_gpu()

            # SFace is OpenCV-based in DeepFace. This environment's OpenCV build has no
            # CUDA support, so keep it on CPU unless TensorFlow GPU is actually usable.
            if model_name == "SFace" and tensorflow_gpu_available:
                effective_model_name = gpu_face_recognition_model
                switched.append(
                    f"recognition model {model_name} -> {effective_model_name}"
                )

            if (
                detector_backend == "yunet"
                and not gpu_face_detection_model.startswith("yolo")
                and tensorflow_gpu_available
            ):
                effective_detector_backend = gpu_face_detection_model
                switched.append(
                    f"detection backend {detector_backend} -> {effective_detector_backend}"
                )

        if switched:
            self.logger.info(
                "Switching to GPU-capable DeepFace components: "
                + ", ".join(switched)
            )
        elif tensorflow_gpu_available or torch_gpu_available:
            self.logger.info(
                "GPU is available, but the configured face models are already the best match for this runtime."
            )
        else:
            self.logger.info(
                "No compatible GPU face-tracking backend is available. Using configured CPU path."
            )

        return effective_model_name, effective_detector_backend

    def _configure_tensorflow_gpu(self) -> bool:
        try:
            import tensorflow as tf
        except Exception as exc:
            self.logger.warning(
                f"Could not import TensorFlow for GPU detection: {exc}"
            )
            return False

        gpus = tf.config.list_physical_devices("GPU")
        if not gpus:
            return False

        for gpu in gpus:
            try:
                tf.config.experimental.set_memory_growth(gpu, True)
            except Exception:
                pass

        gpu_names = ", ".join(gpu.name for gpu in gpus)
        self.logger.info(f"TensorFlow GPU devices available: {gpu_names}")

        try:
            with tf.device("/GPU:0"):
                probe = tf.constant([1.0], dtype=tf.float32)
                _ = (probe + probe).numpy()
        except Exception as exc:
            self.logger.warning(
                "TensorFlow GPU probe failed. Keeping TensorFlow-backed face recognition "
                f"on CPU: {exc}"
            )
            return False

        return True

    def _build_direct_yunet_detector(self):
        if self.detector_backend != "yunet":
            return
        if not hasattr(cv2, "FaceDetectorYN_create"):
            self.logger.warning(
                "OpenCV FaceDetectorYN is not available. Falling back to DeepFace YuNet."
            )
            return

        weights_path = self._resolve_yunet_weights_path()
        if weights_path is None:
            self.logger.warning(
                "Direct OpenCV YuNet requested, but no local YuNet weights were found. "
                "Falling back to DeepFace detector dispatch."
            )
            return

        try:
            backend_id, target_id = self._resolve_opencv_dnn_backend()
            self._yunet_detector = cv2.FaceDetectorYN_create(
                weights_path,
                "",
                (320, 320),
                self.face_detection_confidence,
                0.3,
                500,
                backend_id,
                target_id,
            )
            self._yunet_input_size = (320, 320)
        except Exception as exc:
            self._yunet_detector = None
            self.logger.warning(
                "Could not initialize direct OpenCV YuNet detector. "
                f"Falling back to DeepFace detector dispatch: {exc}"
            )
            return

        self.logger.info(
            "Using direct OpenCV YuNet detector "
            f"{weights_path} (conf={self.face_detection_confidence:.2f})"
        )

    def _resolve_yunet_weights_path(self) -> str | None:
        configured_path = Path(self.cpu_detector_backend).expanduser()
        if configured_path.exists():
            return str(configured_path)

        weights_dir = Path.home() / ".deepface" / "weights"
        candidates = [
            weights_dir / "face_detection_yunet_2023mar.onnx",
            weights_dir / "face_detection_yunet_2022mar.onnx",
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return None

    def _resolve_opencv_dnn_backend(self) -> Tuple[int, int]:
        backend_id = getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 0)
        target_id = getattr(cv2.dnn, "DNN_TARGET_CPU", 0)

        try:
            cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
        except Exception:
            cuda_devices = 0

        if cuda_devices <= 0:
            return backend_id, target_id

        cuda_backend = getattr(cv2.dnn, "DNN_BACKEND_CUDA", None)
        cuda_target = getattr(cv2.dnn, "DNN_TARGET_CUDA", None)
        if cuda_backend is None or cuda_target is None:
            return backend_id, target_id

        self.logger.info("OpenCV CUDA DNN backend is available for YuNet.")
        return int(cuda_backend), int(cuda_target)

    def _configure_torch_gpu(self) -> bool:
        try:
            import torch
        except Exception as exc:
            self.logger.warning(
                f"Could not import PyTorch for GPU detection: {exc}"
            )
            return False

        if not torch.cuda.is_available():
            return False

        gpu_name = torch.cuda.get_device_name(0)
        capability = torch.cuda.get_device_capability(0)
        self.logger.info(
            f"PyTorch CUDA device available: {gpu_name} (capability {capability[0]}.{capability[1]})"
        )
        return True

    def _has_ultralytics(self) -> bool:
        try:
            from ultralytics import YOLO  # noqa: F401
        except Exception as exc:
            self.logger.warning(
                f"Ultralytics is not available for YOLO face detection: {exc}"
            )
            return False
        return True

    def _build_direct_yolo_detector(self):
        if not self.detector_backend.startswith("yolo"):
            return

        weights_path = self._resolve_yolo_weights_path(self.detector_backend)
        if weights_path is None:
            self.logger.warning(
                "Direct Ultralytics face detector requested, but no local face "
                f"weights were found for '{self.detector_backend}'. Falling back "
                "to DeepFace detector dispatch."
            )
            return

        try:
            from ultralytics import YOLO

            self._yolo_detector = YOLO(weights_path)
            self._yolo_device = self._resolve_yolo_device()
            self._warm_up_direct_yolo_detector()
            if self._yolo_detector is None:
                return
        except Exception as exc:
            self._yolo_detector = None
            self._yolo_device = None
            self.logger.warning(
                "Could not initialize direct Ultralytics face detector. "
                f"Falling back to DeepFace detector dispatch: {exc}"
            )
            return

        self.logger.info(
            "Using direct Ultralytics face detector "
            f"{weights_path} on {self._yolo_device} "
            f"(imgsz={self.face_detection_imgsz}, "
            f"conf={self.face_detection_confidence:.2f})"
        )

    def _resolve_yolo_weights_path(self, detector_backend: str) -> str | None:
        configured_path = Path(detector_backend).expanduser()
        if configured_path.exists():
            return str(configured_path)

        weights_dir = Path.home() / ".deepface" / "weights"
        candidates = [
            weights_dir / f"{detector_backend}-face.pt",
        ]
        if detector_backend == "yolov8n":
            candidates.append(weights_dir / "yolov8n-face.pt")

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return None

    def _resolve_yolo_device(self):
        try:
            import torch
        except Exception:
            return "cpu"

        if torch.cuda.is_available():
            return 0
        return "cpu"

    def _warm_up_direct_yolo_detector(self):
        if self._yolo_detector is None:
            return

        try:
            warmup_size = self.face_detection_imgsz or 640
            warmup_img = np.zeros((warmup_size, warmup_size, 3), dtype=np.uint8)
            self._run_direct_yolo_detector(warmup_img)
        except Exception as exc:
            self._yolo_detector = None
            self._yolo_device = None
            self.logger.warning(
                "Direct Ultralytics detector warmup failed. "
                f"Falling back to DeepFace detector dispatch: {exc}"
            )

    def _run_direct_yolo_detector(self, img):
        detector = self._yolo_detector
        if detector is None:
            return []

        predict_args = {
            "source": img,
            "conf": self.face_detection_confidence,
            "verbose": False,
        }
        if self.face_detection_imgsz > 0:
            predict_args["imgsz"] = self.face_detection_imgsz
        if self._yolo_device is not None:
            predict_args["device"] = self._yolo_device

        return detector.predict(**predict_args)

    def _extract_faces_with_direct_yolo(self, img):
        if self._yolo_detector is None:
            return None

        try:
            results = self._run_direct_yolo_detector(img)
        except Exception as exc:
            self._yolo_detector = None
            self._yolo_device = None
            self.logger.warning(
                "Direct Ultralytics detector failed at runtime. "
                f"Falling back to DeepFace detector dispatch: {exc}"
            )
            return None

        height, width = img.shape[:2]
        face_objs = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            xyxy = boxes.xyxy.detach().cpu().numpy()
            confidence_tensor = boxes.conf
            confidences = (
                confidence_tensor.detach().cpu().numpy()
                if confidence_tensor is not None
                else [1.0] * len(xyxy)
            )

            for coords, confidence in zip(xyxy, confidences, strict=False):
                x1, y1, x2, y2 = self._clamp_yolo_box(coords, width, height)
                if x2 <= x1 or y2 <= y1:
                    continue

                face_objs.append(
                    {
                        "face": img[y1:y2, x1:x2].copy(),
                        "facial_area": {
                            "x": x1,
                            "y": y1,
                            "w": x2 - x1,
                            "h": y2 - y1,
                            "left_eye": None,
                            "right_eye": None,
                        },
                        "confidence": float(confidence),
                    }
                )

        return face_objs

    def _extract_faces_with_direct_yunet(self, img):
        detector = self._yunet_detector
        if detector is None:
            return None

        height, width = img.shape[:2]
        input_size = (width, height)
        try:
            if input_size != self._yunet_input_size:
                detector.setInputSize(input_size)
                self._yunet_input_size = input_size
            _, faces = detector.detect(img)
        except Exception as exc:
            self._yunet_detector = None
            self.logger.warning(
                "Direct OpenCV YuNet detector failed at runtime. "
                f"Falling back to DeepFace detector dispatch: {exc}"
            )
            return None

        if faces is None:
            return []

        face_objs = []
        for face in faces:
            x1, y1, x2, y2 = self._clamp_yunet_box(face, width, height)
            if x2 <= x1 or y2 <= y1:
                continue
            face_img = img[y1:y2, x1:x2].copy()
            aligned_face = self._align_sface_yunet_crop(img, face)
            if aligned_face is not None:
                face_img = aligned_face

            face_objs.append(
                {
                    "face": face_img,
                    "facial_area": {
                        "x": x1,
                        "y": y1,
                        "w": x2 - x1,
                        "h": y2 - y1,
                        "right_eye": self._clamp_point(face[4], face[5], width, height),
                        "left_eye": self._clamp_point(face[6], face[7], width, height),
                    },
                    "confidence": float(face[-1]),
                }
            )

        return face_objs

    def _align_sface_yunet_crop(self, img, face):
        recognizer = self._sface_recognizer
        if recognizer is None:
            return None
        try:
            return recognizer.alignCrop(img, face)
        except Exception:
            return None

    @staticmethod
    def _clamp_yunet_box(face, width: int, height: int) -> Tuple[int, int, int, int]:
        raw_x, raw_y, raw_w, raw_h = face[:4]
        x1 = max(0, min(int(round(float(raw_x))), width - 1))
        y1 = max(0, min(int(round(float(raw_y))), height - 1))
        x2 = max(0, min(int(round(float(raw_x + raw_w))), width))
        y2 = max(0, min(int(round(float(raw_y + raw_h))), height))
        return x1, y1, x2, y2

    @staticmethod
    def _clamp_point(x_value, y_value, width: int, height: int) -> Tuple[int, int]:
        x = max(0, min(int(round(float(x_value))), width - 1))
        y = max(0, min(int(round(float(y_value))), height - 1))
        return x, y

    @staticmethod
    def _clamp_yolo_box(coords, width: int, height: int) -> Tuple[int, int, int, int]:
        raw_x1, raw_y1, raw_x2, raw_y2 = coords
        x1 = max(0, min(int(round(float(raw_x1))), width - 1))
        y1 = max(0, min(int(round(float(raw_y1))), height - 1))
        x2 = max(0, min(int(round(float(raw_x2))), width))
        y2 = max(0, min(int(round(float(raw_y2))), height))
        return x1, y1, x2, y2

    def _filter_face_objects(self, face_objs, img):
        return [
            face_obj
            for face_obj in face_objs
            if face_obj["facial_area"]["w"] < img.shape[0] * 0.8
        ]

    def extract_faces(self, img):
        """
        Extract faces from image. Discards small faces.
        Returns:
        results (List[Dict[str, Any]]): A list of dictionaries, where each dictionary contains:

        - "face" (np.ndarray): The detected face as a NumPy array.

        - "facial_area" (Dict[str, Any]): The detected face's regions as a dictionary containing:
            - keys 'x', 'y', 'w', 'h' with int values
            - keys 'left_eye', 'right_eye' with a tuple of 2 ints as values

        - "confidence" (float): The confidence score associated with the detected face.
        """
        direct_yunet_faces = self._extract_faces_with_direct_yunet(img)
        if direct_yunet_faces is not None:
            return self._filter_face_objects(direct_yunet_faces, img)

        direct_yolo_faces = self._extract_faces_with_direct_yolo(img)
        if direct_yolo_faces is not None:
            return self._filter_face_objects(direct_yolo_faces, img)

        try:
            deepface = self._ensure_deepface()
            face_objs = deepface.extract_faces(
                img_path=img,
                detector_backend=self.detector_backend,
                enforce_detection=False,
            )
        except Exception as exc:
            if self.detector_backend == self.cpu_detector_backend:
                raise

            failed_backend = self.detector_backend
            self.detector_backend = self.cpu_detector_backend
            self.logger.warning(
                f"DeepFace detector '{failed_backend}' failed at runtime. "
                f"Falling back to '{self.detector_backend}': {exc}"
            )
            deepface = self._ensure_deepface()
            face_objs = deepface.extract_faces(
                img_path=img,
                detector_backend=self.detector_backend,
                enforce_detection=False,
            )
        return self._filter_face_objects(face_objs, img)
    
    def represent(self, img):
        """
        This function calculates vector representation for one face

        Returns representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).
        """
        if self._sface_recognizer is not None:
            return self._represent_with_direct_sface(img)

        deepface = self._ensure_deepface()
        target_embedding_obj = deepface.represent(
            img_path=img,
            model_name=self.model_name,
            detector_backend="skip",
            )
        return target_embedding_obj[0]["embedding"]

    def _represent_with_direct_sface(self, img) -> List[float]:
        recognizer = self._sface_recognizer
        if recognizer is None:
            raise RuntimeError("Direct SFace recognizer was not initialized.")

        if img.shape[0] != 112 or img.shape[1] != 112:
            img = cv2.resize(img, (112, 112), interpolation=cv2.INTER_AREA)

        embedding = recognizer.feature(img)
        return embedding.reshape(-1).astype(float).tolist()
