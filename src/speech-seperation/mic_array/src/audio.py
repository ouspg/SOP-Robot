#!/usr/bin/env python

from tuning import Tuning
import usb.core
import usb.util
import pyaudio
import wave
import numpy as np

class Audio:
    RESPEAKER_RATE = 16000
    RESPEAKER_CHANNELS = 1
    RESPEAKER_WIDTH = 2
    # import sounddevice as sd
    # print sd.query_devices()
    RESPEAKER_INDEX = 2  # refer to input device id
    CHUNK = 1024
    RECORD_SECONDS = 5
    WAVE_OUTPUT_FILENAME = "sc.wav"

    def __init__(self):
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.mic_tuning = Tuning(dev)
        self.mic_tuning.set_vad_threshold(3.5)
        self.recording = False

    def readDOA(self):
        return self.mic_tuning.direction

    def readVAD(self):
        return self.mic_tuning.is_voice()

    @staticmethod
    def getSampleSize():
        p = pyaudio.PyAudio()
        return p.get_sample_size(p.get_format_from_width(Audio.RESPEAKER_WIDTH))

    def record(self):
        p = pyaudio.PyAudio()

        try:
            stream = p.open(
            rate=Audio.RESPEAKER_RATE,
            format=p.get_format_from_width(Audio.RESPEAKER_WIDTH),
            channels=Audio.RESPEAKER_CHANNELS,
            input=True
            )
        except IOError:
            return

        frames = []

        for i in range(0, int(Audio.RESPEAKER_RATE / Audio.CHUNK * Audio.RECORD_SECONDS)):
            data = stream.read(Audio.CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        wf = wave.open(Audio.WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(p.get_format_from_width(Audio.RESPEAKER_WIDTH)))
        wf.setframerate(Audio.RESPEAKER_RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
    