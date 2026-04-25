# Voice Chatbot Contract

This document is the source of truth for the in-tree voice stack contract.

Historical references:

- <https://github.com/Aapo2001/voice-chatbot-ros>
- <https://github.com/Aapo2001/python-chatbot>

## Scope

The in-tree voice stack is split across these ROS2 packages:

- `asr_package`: microphone capture, VAD, selectable ASR backends, and listen gating
- `llm_package`: selectable LLM backend, SQLite local data retrieval, and conversation history
- `tts_package`: text-to-speech playback and jaw signaling
- `chatbot_app`: PySide6 GUI and ROS bridge

The former `voice_chatbot_ros` submodule has been migrated into these packages
and removed from the workspace.

## Namespace

Primary namespace:

- `/voice_chatbot`

Root-managed draft config file:

- editable runtime config: `config/voice_chatbot.json`
- shipped defaults: `config/voice_chatbot.defaults.json`

## Topic And Service Matrix

| Interface | Type | Owner | Producers | Consumers | Notes |
| --- | --- | --- | --- | --- | --- |
| `/voice_chatbot/user_text` | `std_msgs/msg/String` | `asr_package` / `chatbot_app` | `asr_package`, `chatbot_app` | `llm_package` | Canonical user text input to the LLM |
| `/voice_chatbot/transcript` | `std_msgs/msg/String` | `asr_package` | `asr_package` | `chatbot_app`, `full_demo` | Published after successful speech recognition |
| `/voice_chatbot/assistant_text` | `std_msgs/msg/String` | `llm_package` | `llm_package`, `full_demo` | `tts_package`, `chatbot_app` | Canonical assistant reply channel |
| `/voice_chatbot/status` | `std_msgs/msg/String` | shared | `asr_package`, `llm_package`, `tts_package` | `chatbot_app`, `full_demo` | Shared status bus, same status vocabulary across nodes |
| `/voice_chatbot/log` | `std_msgs/msg/String` | shared | `asr_package`, `llm_package`, `tts_package`, `chatbot_app` | `chatbot_app` | Human-readable operational logs |
| `/voice_chatbot/tts_done` | `std_msgs/msg/String` | `tts_package` | `tts_package` | `asr_package`, `full_demo` | Used to reset ASR and resume listening after playback |
| `/voice_chatbot/can_listen` | `std_msgs/msg/Bool` | compatibility | `full_demo`, `tts_package` | `asr_package` | Preserve this legacy gating topic during migration |
| `/can_listen` | `std_msgs/msg/Bool` | compatibility | `tts_package` | `full_demo`, legacy nodes | Preserve global legacy output until old consumers are retired |
| `jaw_topic` | `std_msgs/msg/String` | compatibility | `tts_package` | `jaw_movement` and related consumers | Preserve existing jaw animation trigger |
| `/voice_chatbot/clear_history` | `std_srvs/srv/Trigger` | `llm_package` | `chatbot_app` | `llm_package` | Clears LLM conversation history |

## Status Vocabulary

The split nodes should use this shared status vocabulary:

- `initializing`
- `ready`
- `listening`
- `speech_detected`
- `transcribing`
- `llm_responding`
- `speaking`
- `error`

## Ownership Boundaries

### `asr_package`

Owns:

- audio input
- VAD state
- ASR model loading through the configured `faster-whisper`, `pywhispercpp`, or `crispasr` backend
- ASR listen gating
- reset-on-`tts_done`

Does not own:

- LLM inference
- GUI
- TTS playback

### `llm_package`

Owns:

- LLM model loading
- local SQLite knowledge-base ingestion and retrieval from `legacy/chatbot/chatbot/data`
- conversation history
- `clear_history`
- assistant text generation

Does not own:

- audio capture
- VAD
- TTS playback
- GUI rendering

### `tts_package`

Owns:

- speech synthesis
- audio playback
- `tts_done`
- `jaw_topic`
- `/can_listen` compatibility output

Does not own:

- STT
- LLM
- GUI

### `chatbot_app`

Owns:

- PySide6 GUI
- config editing UX
- ROS bridge for chat, status, logs, and history clear

Does not own:

- microphone capture
- model loading
- speech synthesis

## Migration Rules

1. Preserve the `/voice_chatbot/*` contract used by `full_demo`.
2. Keep `tts_package` backward-compatible with `chatbot_response` until the old chatbot packages are retired.
3. Do not add new runtime dependencies on the `voice-chatbot` pip package.
4. Treat `robot.launch.py` and `robot.fake.launch.py` as the canonical operator entrypoints for the voice stack when it is used as part of the full robot.
