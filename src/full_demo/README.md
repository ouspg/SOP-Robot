# Robot AI demo

The current voice pipeline is a single turn-taking loop:

```text
Default microphone -> WebRTC VAD -> Finnish Whisper -> LLM correction -> LLM answer -> Finnish TTS
```

SST runs Faster Whisper locally and publishes its result on
`recognized_speech`. On the first turn, the LLM answers the Faster Whisper
transcript directly. Starting from the second turn, it first uses the
conversation history to correct likely recognition errors, then sends the
corrected utterance with the same history to generate `chatbot_response`. TTS
publishes `can_listen`. Listening is paused while the LLM thinks and while the
robot speaks, preventing the robot from answering its own voice.

Run only the integrated voice chatbot:

```console
pixi run chatbot
```

The standard OpenAI-compatible client is configured through `.env.local`:

```dotenv
LLM_BASE_URL=https://openrouter.ai/api/v1
LLM_API_KEY=your-provider-key
LLM_MODEL=openai/gpt-oss-120b
```


The complete demo has separate launch files for real and fake robot hardware:

```console
pixi run robot-demo-real
pixi run robot-demo-fake
```

These tasks run `real_robot.launch.py` and `fake_robot.launch.py` from this
package. Each ROS node runs as its own process, all output stays in the current
terminal, and one Ctrl+C stops the launch service and its nodes.

## Demo coordinator

This package contains a node that combines various parts of the robot to be used in demo situations.

Currently, the parts/packages used by this node are:

- **sst_package** - to hear when people talk to the robot
- **llm_package** - to generate responses for the recognized speech
- **tts_package** - to speak out the response produced by the LLM
- **face_tracker** - to find faces and say hi when finding one
- **unified_arms** - to coordinate arm and hand gestures

## Usage

Launch the physical robot from the repository root with:

```console
pixi run robot-demo-real
```

Use `pixi run robot-demo-fake` for the same demo with fake robot hardware.
