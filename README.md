# SOP-Robot

SOP-Robot is a ROS 2 Humble workspace for a Dynamixel-based robot with a unified
bring-up path, split voice stack, face tracking, and demo orchestration.

## Canonical Entrypoints

The project now has one canonical launch surface:

- `robot.launch.py` for the real robot
- `robot.fake.launch.py` for the fake/simulated robot

Use one of these three equivalent entry styles:

```bash
pixi run launch-robot
pixi run launch-robot-fake
```

```bash
bash tools/launch_robot.sh
bash tools/launch_robot_fake.sh
```

```bash
ros2 launch robot robot.launch.py
ros2 launch robot robot.fake.launch.py
```

Subsystem launch files remain available for focused debugging, but they are no
longer the primary way the project is started.

## Quick Start

For a fresh Ubuntu 22.04 machine:

```bash
sudo apt update
sudo apt install -y git curl python3-rosdep
sudo rosdep init || true
rosdep update
curl -fsSL https://pixi.sh/install.sh | bash
git clone --recurse-submodules https://github.com/ouspg/SOP-Robot.git
cd SOP-Robot
pixi run bootstrap
pixi run launch-robot-fake
```

`pixi run bootstrap` installs the Pixi environment, initializes submodules,
installs the Ubuntu-only dependencies via `rosdep`, rebuilds
`llama-cpp-python` with CUDA offload for the local NVIDIA GPU, repairs or
rebuilds pywhispercpp CUDA support for Whisper ASR, downloads the voice models,
builds the workspace, and runs the fast unit tests.

## Source Of Truth

These files define the runtime layout:

- `config/robot_stack.yaml`: top-level feature toggles and mode-specific defaults
- `config/voice_chatbot.json`: editable runtime voice stack config
- `config/voice_chatbot.defaults.json`: shipped voice defaults
- `src/sop_robot_bringup/robot/launch/robot.launch.py`: canonical real-robot entrypoint
- `src/sop_robot_bringup/robot/launch/robot.fake.launch.py`: canonical fake-robot entrypoint
- `src/sop_robot_voice/voice_stack_common/voice_stack_common/contracts.py`: shared voice stack topics and services
- `src/sop_robot_common/sop_robot_common/contracts.py`: shared non-voice robot topics

## Repository Layout

- `config/`: runtime configuration used by the canonical robot launches
- `docs/`: setup, bring-up, testing, and architecture notes
- `src/`: ROS 2 packages and submodules
- `tools/`: launch wrappers plus setup, device, monitoring, and runtime helpers
- `testing/`: project-level system tests and cross-package benchmark harnesses
- `benchmarks/`, `reports/`, `test-results/`: generated output directories

## Active Runtime Packages

- `robot`: stack orchestration, controller startup, and launch arguments
- `robot_hardware`: `ros2_control` hardware interface
- `voice_stack_common`: shared voice config, contracts, and runtime helpers
- `sop_robot_common`: shared non-voice topic contracts
- `asr_package`: audio capture, VAD, and Whisper STT
- `llm_package`: GGUF-backed conversation inference
- `tts_package`: speech synthesis, playback, and jaw/listen compatibility outputs
- `chatbot_app`: GUI bridge and unified operator window
- `face_tracker`: camera ingest and face detection
- `face_tracker_movement`: head and eye tracking
- `jaw_movement`: jaw motion follower
- `hand_gestures`: hand gesture action bridge
- `full_demo`: high-level demo state machine

The old `chatbot`, `qabot`, and `speech_recognizer` packages are archived
in-place for reference and are excluded from the active workspace graph. The
former `voice_chatbot_ros` submodule has been migrated into the split in-tree
voice packages and removed.

## Servo Table

**Note: Servos can be configured using [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)**

Baud rate: 57600, Voltage: 7.5

**Note: You can use Dynamixel Wizard 2.0 to check that servos have enough voltage to operate normally.**

| Servo ID | Model | Joint name                  | Description           |
| -------- | ----- | --------------------------- | --------------------- |
| 1        | XL430 | head_tilt_right_joint       | Head tilt right-side  |
| 2        | XL430 | head_tilt_vertical_joint    | Head tilt up/down     |
| 3        | XL430 | head_tilt_left_joint        | Head tilt left-side   |
| 4        | XL430 | head_pan_joint              | Head turn left/right  |
| 9        | XL320 | eyes_shift_horizontal_joint | Eyes shift left/right |
| 11       | XL320 | eyes_shift_vertical_joint   | Eyes shift up/down    |
| 12       | XL320 | head_jaw_joint              | Open/close jaw        |
| 31       | XL320 | r_thumb_joint               | Open/close thumb      |
| 34       | XL320 | r_index1_joint              | Open/close index      |
| 37       | XL320 | r_middle1_joint             | Open/close middle     |
| 40       | XL320 | r_ring_joint                | Open/close ring       |
| 44       | XL320 | r_pinky_joint               | Open/close pinky      |
| 47       | XL320 | unnamed joint               | Rotate wrist          |

## Cautions
**Note: head tilt range of motion is poor**

**Note: wrist joint seems to overload very easily**

- Reason for this detected during inspection of the mechanical assembly. The hand is installed wrong way to the wrist.

**Note: servo angle limits are not configured**

**Note: Be careful not to move joints too much, limits are not set yet**

## Documentation

- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md): Ubuntu 22.04 setup, Pixi/rosdep ownership, and developer workflow
- [docs/BRINGUP.md](docs/BRINGUP.md): runtime entrypoints, launch arguments, control flow, and ROS topic flow
- [docs/PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md): unified package and node layout conventions
- [docs/TESTING.md](docs/TESTING.md): test tasks, scopes, and validation order
- [docs/VOICE_CHATBOT_CONTRACT.md](docs/VOICE_CHATBOT_CONTRACT.md): voice-stack topic and service contract
