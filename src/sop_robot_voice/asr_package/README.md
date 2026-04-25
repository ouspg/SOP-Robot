# asr_package

ROS2 ASR package for the SOP Robot voice stack.

Responsibilities:

- microphone capture with `sounddevice`
- VAD with `silero-vad`
- ASR with selectable `faster-whisper`, `pywhispercpp`, or `crispasr` backends
- `/voice_chatbot/can_listen` gating
- transcript and user text publishing

Canonical executable:

- `ros2 run asr_package asr_node`

Canonical project bring-up:

- `ros2 launch robot robot.launch.py`
- `ros2 launch robot robot.fake.launch.py`

Subsystem-only debugging launch:

- `ros2 launch chatbot_app voice_stack.launch.py`

Configuration:

- `whisper_backend`: `faster-whisper`, `pywhispercpp`, or `crispasr`
- `whisper_model`: faster-whisper model name, for example `medium`
- `whisper_cpp_repo_id`: Hugging Face repository for GGML models
- `whisper_cpp_model_filename`: GGML file to download, for example `ggml-model-fi-medium.bin`
- `whisper_cpp_model_path`: local GGML model path used by `pywhispercpp`
- `crispasr_python_path`: path to the local CrispASR Python bindings
- `crispasr_lib_path`: path to the compiled `libcrispasr.so`
- `crispasr_model_path`: path to the CrispASR GGUF model, for example `/home/aapot/CrispASR/parakeet.gguf`
- `crispasr_backend`: CrispASR backend hint, normally `parakeet`

Install or repair the local CrispASR build with:

```bash
pixi run install-crispasr
```

The task clones `https://github.com/CrispStrobe/CrispASR` when needed, downloads
the Parakeet GGUF model, builds CUDA when available, and verifies the Python
binding against the configured shared library and model path.

Historical reference sources were migrated from the old voice chatbot projects
into this package and the shared `voice_stack_common` contracts.
