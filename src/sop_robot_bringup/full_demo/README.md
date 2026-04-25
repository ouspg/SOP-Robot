# full_demo

`full_demo` is the high-level demo coordinator for the active SOP-Robot stack.

It now integrates with:

- `face_tracker`
- `face_tracker_movement`
- `asr_package`
- `llm_package`
- `tts_package`
- `jaw_movement`
- `hand_gestures`

The node uses the `/voice_chatbot/*` contract and the shared topic contracts in
`sop_robot_common`.

## Canonical Bring-Up

`full_demo` is normally started by the canonical robot launch files:

- `ros2 launch robot robot.launch.py`
- `ros2 launch robot robot.fake.launch.py`

It should not be treated as a separate project entrypoint anymore.

## Executable

```bash
ros2 run full_demo full_demo_node
```
