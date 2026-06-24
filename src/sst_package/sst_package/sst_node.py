from __future__ import annotations

import threading

import numpy as np
import rclpy
import sounddevice as sd
import webrtcvad
from faster_whisper import WhisperModel
from rclpy.node import Node
from std_msgs.msg import String

SAMPLE_RATE = 16_000
FRAME_MS = 30
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1_000
SILENCE_FRAMES = 720 // FRAME_MS  # end an utterance after ~0.7 s of silence
MIN_SPEECH_FRAMES = 180 // FRAME_MS  # ignore utterances shorter than ~0.2 s
MAX_SPEECH_FRAMES = 30 * 1_000 // FRAME_MS  # cap utterances at 30 s

MODEL_NAME = "RASMUS/whisper-large-v3-turbo-finnish-ct2"
LANGUAGE = "fi"
TOPIC = "recognized_speech"


def pcm_to_float32(pcm: bytes) -> np.ndarray:
    """Convert signed 16-bit little-endian PCM to normalized float32 samples."""
    return np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0


class SST(Node):
    def __init__(self):
        super().__init__("sst_node")
        self.publisher = self.create_publisher(String, TOPIC, 10)

        self.get_logger().info(f"Loading {MODEL_NAME} on the GPU...")
        self.model = WhisperModel(MODEL_NAME, device="cuda", compute_type="float16")
        self.vad = webrtcvad.Vad(3)

        self.running = True
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.thread.start()
        self.get_logger().info("Listening for Finnish speech.")

    def listen(self) -> None:
        frames: list[bytes] = []
        silence = 0
        with sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=SAMPLES_PER_FRAME,
            channels=1,
            dtype="int16",
        ) as stream:
            while self.running:
                frame, _ = stream.read(SAMPLES_PER_FRAME)
                frame = bytes(frame)
                speaking = self.vad.is_speech(frame, SAMPLE_RATE)

                if speaking:
                    frames.append(frame)
                    silence = 0
                elif frames:
                    frames.append(frame)
                    silence += 1

                done = silence >= SILENCE_FRAMES or len(frames) >= MAX_SPEECH_FRAMES
                if frames and done:
                    if len(frames) - silence >= MIN_SPEECH_FRAMES:
                        self.transcribe(b"".join(frames))
                    frames, silence = [], 0

    def transcribe(self, pcm: bytes) -> None:
        audio = pcm_to_float32(pcm)
        segments, _ = self.model.transcribe(
            audio,
            language=LANGUAGE,
            beam_size=5,
            condition_on_previous_text=False,
            vad_filter=True,
        )
        text = " ".join(segment.text.strip() for segment in segments).strip()
        if text:
            self.get_logger().info(text)
            self.publisher.publish(String(data=text))

    def destroy_node(self) -> None:
        self.running = False
        self.thread.join(timeout=2)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SST()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
