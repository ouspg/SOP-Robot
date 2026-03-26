"""
Voice Chatbot – headless command-line interface.

Runs the complete speech-to-speech pipeline in a single process.
Press Ctrl+C to quit.

Usage::

    python -m voice_chatbot.chatbot
"""

from .platform_setup import setup_cuda

setup_cuda()

from .audio_io import AudioIO
from .config import Config
from .llm import ChatLLM
from .stt import SpeechToText
from .tts_engine import TextToSpeech
from .vad import VoiceActivityDetector


class VoiceChatbot:
    """Synchronous voice chatbot — loads all models and runs an audio loop."""

    def __init__(self) -> None:
        self._config = Config.load()

        print("=" * 50)
        print("  Äänichatbot - Paikallinen GPU-kiihdytetty")
        print("=" * 50)
        print()

        print("[Init] Setting up audio I/O...")
        self._audio = AudioIO(self._config)

        print("[Init] Loading VAD model...")
        self._vad = VoiceActivityDetector(self._config)
        print("[Init] VAD ready.")

        self._stt = SpeechToText(self._config)
        self._llm = ChatLLM(self._config)
        self._tts = TextToSpeech(self._config)

        print()
        print("Kaikki mallit ladattu onnistuneesti!")
        print()

    def run(self) -> None:
        """Start the blocking audio capture → process → speak loop."""
        self._audio.start_capture()
        print("Kuunnellaan... (paina Ctrl+C lopettaaksesi)")
        print("-" * 50)

        try:
            while True:
                chunk = self._audio.get_audio_chunk(timeout=0.1)
                if chunk is None:
                    continue

                event, audio_data = self._vad.process_chunk(chunk)

                if event == "speech_start":
                    print("\n[Kuunnellaan...]")
                elif event == "speech_end" and audio_data is not None:
                    print("[Käsitellään...]")
                    text = self._stt.transcribe(audio_data)
                    if not text or text.isspace():
                        print("[Puhetta ei havaittu]")
                        continue

                    print(f"Sinä: {text}")
                    response = self._llm.chat(text)
                    print(f"Botti: {response}")

                    audio_out, sr = self._tts.synthesize(response)
                    self._audio.play_audio(audio_out, sr)
                    self._audio.clear_queue()
                    self._vad.reset()

        except KeyboardInterrupt:
            print("\n\nSammutetaan...")

        self._audio.close()
        print("Näkemiin!")


def main() -> None:
    chatbot = VoiceChatbot()
    chatbot.run()


if __name__ == "__main__":
    main()
