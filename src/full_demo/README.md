# Robot AI demo

The current voice pipeline is a single turn-taking loop:

```text
Default microphone -> WebRTC VAD -> Finnish Whisper -> Ahma LLM -> Finnish TTS
```

SST publishes `recognized_speech`, the LLM publishes `chatbot_response`, and
TTS publishes `can_listen`. Listening is paused while the LLM thinks and while
the robot speaks, preventing the robot from answering its own voice.

Run only the integrated voice chatbot:

```console
pixi run chatbot
```

SST always captures from the PipeWire system-default microphone selected in
the desktop sound settings.

The full physical-robot demo remains available through `pixi run robot-demo`.

## Legacy demo coordinator

This package contains a node that combines various parts of the robot to be used in demo situations.

Currently, the parts/packages used by this node are:

- **speech_recognizer** - to hear when people talk to the robot
- **qabot** - to find responses for the recognized speech
- **tts_package** - to speak out the response produced by qabot
- **face_tracker** - to find faces and say hi when finding one
- **Hands** - to wave to the person found by face tracking

## Usage

The demo is simply run by going to the root directory of the project and running
```console
bash launch_demo.sh
```
This will start all the required nodes and services
