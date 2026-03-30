from typing import Tuple

class FaceRecognizer(object):

    def __init__(
        self,
        logger,
        model_name,
        detector_backend,
        prefer_gpu=True,
        gpu_face_recognition_model="ArcFace",
        gpu_face_detection_model="yolov8n",
    ):
        """
        Initialize face recognizer, and create embeddings in the intialization
        """
        self.logger = logger
        self.prefer_gpu = prefer_gpu
        self.model_name, self.detector_backend = self._resolve_runtime_models(
            model_name=model_name,
            detector_backend=detector_backend,
            gpu_face_recognition_model=gpu_face_recognition_model,
            gpu_face_detection_model=gpu_face_detection_model,
        )

        from deepface import DeepFace

        self._deepface = DeepFace

        # build models once to store them in the memory
        self.model = self._deepface.build_model(model_name=self.model_name)

        logger.info(f"facial recognition model {self.model_name} is just built")
        logger.info(f"face detection backend is '{self.detector_backend}'")

        self.logger.info("FaceRecognizer initialized!")

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
            import ultralytics  # noqa: F401
        except Exception as exc:
            self.logger.warning(
                f"Ultralytics is not available for YOLO face detection: {exc}"
            )
            return False
        return True
    
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
        face_objs = self._deepface.extract_faces(
            img_path=img,
            detector_backend=self.detector_backend,
            enforce_detection=False,
        )
        return [face_obj for face_obj in face_objs if face_obj["facial_area"]["w"] < img.shape[0] * 0.8]
    
    def represent(self, img):
        """
        This function calculates vector representation for one face

        Returns representation (List[float]): Multidimensional vector representing facial features.
            The number of dimensions varies based on the reference model
            (e.g., FaceNet returns 128 dimensions, VGG-Face returns 4096 dimensions).
        """
        target_embedding_obj = self._deepface.represent(
            img_path=img,
            model_name=self.model_name,
            detector_backend="skip",
            )
        return target_embedding_obj[0]["embedding"]
