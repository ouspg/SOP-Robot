This package contains the runtime TTS node for the SOP Robot voice stack.

The node integrates with the in-tree `/voice_chatbot` contract.

Canonical project bring-up:

- `ros2 launch robot robot.launch.py`
- `ros2 launch robot robot.fake.launch.py`

Canonical executable:

- `ros2 run tts_package tts_node`

# Usage

## Before
Check that `model.pth` and `config.json` are located in `src/sop_robot_voice/tts_package/resource/`.
The root pixi workflow can download these assets with:

```bash
pixi run setup-models
```

## Dependencies

* `TTS`
* `espeak-ng`
* `simpleaudio`

When using the root pixi environment, the Python dependencies are installed by
`pixi install`. On Ubuntu 22.04, install `espeak-ng` separately if it is missing.

## Run TTS node
```bash
ros2 run tts_package tts_node
```

This subscribes to `/voice_chatbot/assistant_text` and the legacy
`chatbot_response` topic, publishes `/voice_chatbot/tts_done`, keeps `/can_listen`
for compatibility, and keeps `jaw_topic` for jaw animation.

## Using the service
Service can be exercised from the CLI publisher:

```bash
ros2 run tts_package tts_client
```

The node synthesizes the sentence into a temporary wav file and plays it
automatically.

## Potential future improvements

* Remove the `chatbot_response` compatibility subscription after the old packages
  are retired from the workspace.
