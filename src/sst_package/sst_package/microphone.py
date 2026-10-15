import pyaudio

SAMPLE_RATE = 16_000
FRAME_MS = 30
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1_000

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
