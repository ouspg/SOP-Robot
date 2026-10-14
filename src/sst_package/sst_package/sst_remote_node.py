import contextlib
import io
import os
import threading
import time
import wave
from pathlib import Path
from queue import Empty, Full, Queue

import rclpy
import webrtcvad
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .microphone import SystemMicrophone

SAMPLE_RATE = 16_000
FRAME_MS = 30
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1_000
SILENCE_FRAMES = 720 // FRAME_MS
MIN_SPEECH_FRAMES = 180 // FRAME_MS
MAX_SPEECH_FRAMES = 30 * 1_000 // FRAME_MS
DEFAULT_BASE_URL = 'https://api.lehmus-ai.oulu.fi/v1'
DEFAULT_MODEL = 'whisper-large-v3'
DEFAULT_API_KEY = 'your-lehmusai-key'
LANGUAGE = 'fi'

def load_local_environment():
    project_root = Path(os.environ.get('SOP_ROBOT_ROOT', Path.cwd())).resolve()
    load_dotenv(project_root / '.env.local')


class SSTRemoteNode(Node):
    def __init__(self):
        super().__init__('sst_remote_node')

        load_local_environment()
        self.base_url = (
            os.environ.get(
                'LLM_BASE_URL',
                DEFAULT_BASE_URL,
            )
            or DEFAULT_BASE_URL
        )
        self.model = os.environ.get('SST_MODEL') or DEFAULT_MODEL
        api_key = os.environ.get('LLM_API_KEY') or DEFAULT_API_KEY


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

        self.client = OpenAI(
            base_url=self.base_url,
            api_key=api_key,
            timeout=30.0,
            max_retries=0,
        )

        self.get_logger().info(f'Lehmus AI transcription client ready with {self.model}.')
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
            self.get_logger().warning('Transcription is busy; dropping an utterance.')

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
                text = self.transcribe(pcm)
                latency_ms = (time.perf_counter() - started) * 1_000
                if text:
                    self.get_logger().info(f'[Whisper] {text} ({latency_ms:.0f} ms)')
                    self.speech_publisher.publish(String(data=text))
            except Exception as error:
                self.get_logger().error(f'Whisper transcription failed: {error}')
            finally:
                self.utterances.task_done()

    def transcribe(self, pcm):
        with io.BytesIO() as audio:
            with wave.open(audio, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(SAMPLE_RATE)
                wav.writeframes(pcm)

            response = self.client.audio.transcriptions.create(
                model=self.model,
                file=('utterance.wav', audio.getvalue(), 'audio/wav'),
                language=LANGUAGE,
            )
        return response.text.strip()


    def destroy_node(self):
        self.running = False
        with contextlib.suppress(Full):
            self.utterances.put_nowait(None)

        self.listen_thread.join(timeout=2)
        self.transcribe_thread.join(timeout=2)
        self.client.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SSTRemoteNode()
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
