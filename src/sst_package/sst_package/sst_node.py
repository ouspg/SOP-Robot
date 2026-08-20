import contextlib
import threading
import time
from queue import Empty, Full, Queue

import numpy as np
import pyaudio
import rclpy
import webrtcvad
from faster_whisper import WhisperModel
from rclpy.node import Node
from std_msgs.msg import Bool, String

SAMPLE_RATE = 16_000
FRAME_MS = 30
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1_000
SILENCE_FRAMES = 720 // FRAME_MS
MIN_SPEECH_FRAMES = 180 // FRAME_MS
MAX_SPEECH_FRAMES = 30 * 1_000 // FRAME_MS

MODEL_NAME = 'RASMUS/whisper-large-v3-turbo-finnish-ct2'
LANGUAGE = 'fi'


class SystemMicrophone:
    def __init__(self):
        self.audio = None
        self.stream = None

    def __enter__(self):
        self.audio = pyaudio.PyAudio()
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=SAMPLES_PER_FRAME,
            )
        except Exception:
            self.audio.terminate()
            self.audio = None
            raise
        return self

    def __exit__(self, exception_type, exception, traceback):
        if self.stream is not None:
            if self.stream.is_active():
                self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        if self.audio is not None:
            self.audio.terminate()
            self.audio = None

    def read_frame(self):
        if self.stream is None:
            raise RuntimeError('The system microphone is not open.')
        return self.stream.read(
            SAMPLES_PER_FRAME,
            exception_on_overflow=False,
        )


def pcm_to_float32(pcm):
    return np.frombuffer(pcm, dtype='<i2').astype(np.float32) / 32768.0


class SSTNode(Node):
    def __init__(self):
        super().__init__('sst_node')

        self.speech_publisher = self.create_publisher(
            String,
            'recognized_speech',
            10,
        )
        self.can_listen_subscription = self.create_subscription(
            Bool,
            'can_listen',
            self.can_listen_callback,
            10,
        )

        self.can_listen = True

        self.get_logger().info(f'Loading {MODEL_NAME} on the GPU...')
        self.model = WhisperModel(
            MODEL_NAME,
            device='cuda',
            compute_type='float16',
        )
        self.vad = webrtcvad.Vad(3)

        self.running = True
        self.utterances = Queue(maxsize=1)

        self.listen_thread = threading.Thread(
            target=self.listen,
            daemon=True,
        )
        self.transcribe_thread = threading.Thread(
            target=self.transcribe_worker,
            daemon=True,
        )

        self.listen_thread.start()
        self.transcribe_thread.start()

    def can_listen_callback(self, message):
        if self.can_listen == message.data:
            return

        self.can_listen = message.data
        state = 'listening' if self.can_listen else 'paused'
        self.get_logger().info(f'Microphone {state}.')

    def listen(self):
        frames = []
        silence = 0
        with SystemMicrophone() as microphone:
            self.get_logger().info('Capturing from PyAudio')
            while self.running:
                frame = microphone.read_frame()
                if not self.can_listen:
                    frames = []
                    silence = 0
                    continue

                if self.vad.is_speech(frame, SAMPLE_RATE):
                    frames.append(frame)
                    silence = 0
                elif frames:
                    frames.append(frame)
                    silence += 1

                utterance_complete = silence >= SILENCE_FRAMES or len(frames) >= MAX_SPEECH_FRAMES
                if not frames or not utterance_complete:
                    continue

                if len(frames) - silence >= MIN_SPEECH_FRAMES:
                    self.queue_utterance(b''.join(frames))
                frames = []
                silence = 0

    def queue_utterance(self, pcm):
        try:
            self.utterances.put_nowait(pcm)
        except Full:
            self.get_logger().warning('Faster Whisper is busy; dropping an utterance.')

    def transcribe_worker(self):
        while self.running:
            try:
                pcm = self.utterances.get(timeout=0.2)
            except Empty:
                continue
            if pcm is None:
                break

            started = time.perf_counter()
            try:
                text = self.transcribe_local(pcm)
                latency_ms = (time.perf_counter() - started) * 1_000
                if text:
                    self.get_logger().info(f'[Faster Whisper] {text} ({latency_ms:.0f} ms)')
                    self.speech_publisher.publish(String(data=text))
            except Exception as error:
                self.get_logger().error(f'Faster Whisper transcription failed: {error}')
            finally:
                self.utterances.task_done()

    def transcribe_local(self, pcm):
        segments, _ = self.model.transcribe(
            pcm_to_float32(pcm),
            language=LANGUAGE,
            beam_size=5,
            condition_on_previous_text=False,
            vad_filter=True,
        )
        return ' '.join(segment.text.strip() for segment in segments).strip()

    def destroy_node(self):
        self.running = False
        with contextlib.suppress(Full):
            self.utterances.put_nowait(None)

        self.listen_thread.join(timeout=2)
        self.transcribe_thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SSTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
