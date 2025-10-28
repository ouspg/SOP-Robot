# Unified Robot Control

A comprehensive ROS2 node that provides centralized control for the entire robot system, integrating face tracking, speech recognition, dialogue management, and robot movement coordination.

## Features

- **State Machine**: Manages robot behavior states (IDLE, LISTENING, THINKING, SPEAKING, TRACKING, GESTURING)
- **Face Tracking Integration**: Coordinates head and eye movement to track faces
- **Speech & Dialogue**: Manages speech recognition, QA bot integration, and TTS
- **Arm & Hand Control**: Coordinates gestures and arm movements
- **Modular Architecture**: Can be configured to enable/disable different subsystems

## Architecture

The unified robot control node **only uses topics** to communicate with other nodes. It never directly controls hardware.

### Subscriptions (Inputs)
- `/face_tracker/face_topic` - Detected faces from face tracker node
- `/face_tracker_movement/focused_face` - Focused face from face tracker movement node
- `speech_recognizer/recognized_speech` - Recognized speech text
- `chatbot/chatbot_response` - Responses from QA bot node
- `can_listen` - TTS ready status

### Publications (Outputs to existing control nodes)
- `speech_recognizer/can_listen` - Control when speech recognition listens
- `chatbot/recognized_speech` - Send recognized speech to chatbot
- `chatbot_response` - Text for TTS service to speak
- `/arms/arm_action` - Arm gesture commands to unified_arms_client
- `/l_hand/l_hand_topic` - Left hand gesture commands to hand_gestures node
- `/r_hand/r_hand_topic` - Right hand gesture commands to hand_gestures node
- `/face_tracker_movement/head_gesture_topic` - Head gesture commands to face_tracker_movement node

### Direct Control?
**NO!** This node never directly controls hardware. It coordinates with existing nodes:
- **face_tracker_movement node** - handles head/eye movement
- **hand_gestures node** - handles hand finger movements
- **unified_arms_client** - handles arm and gesture sequences
- **tts_package** - handles text-to-speech
- **speech_recognizer** - handles speech recognition
- **qabot** - handles dialogue management

## Usage

### Launch the Complete System

Launch the entire robot system with all subsystems:

```bash
# Simulation mode
ros2 launch unified_robot_control unified_robot.launch.py simulation:=true

# Hardware mode
ros2 launch unified_robot_control unified_robot.launch.py

# With custom configuration
ros2 launch unified_robot_control unified_robot.launch.py \
    simulation:=false \
    enable_face_tracking:=true \
    enable_speech:=true \
    enable_arms:=true \
    robot_parts:=head
```

### Launch Arguments

- `simulation` (default: false): Run in simulation mode
- `enable_face_tracking` (default: true): Enable face tracking
- `enable_speech` (default: true): Enable speech recognition and dialogue
- `enable_arms` (default: true): Enable arm and hand control
- `robot_parts` (default: head): Which robot parts to launch (head/arm/both)

### Run Node Only

If other nodes are already running:

```bash
ros2 run unified_robot_control unified_robot_control_node
```

With parameters:

```bash
ros2 run unified_robot_control unified_robot_control_node \
    --ros-args \
    -p simulation:=true \
    -p enable_face_tracking:=true \
    -p enable_speech:=true \
    -p enable_arms:=true
```

## State Machine

The robot follows this state flow:

```
IDLE
  ↓ (face detected)
LISTENING
  ↓ (speech recognized)
THINKING
  ↓ (chatbot responds)
SPEAKING
  ↓ (TTS complete)
LISTENING
  ↓ (timeout 30s)
IDLE
```

## Behavior

### Face Detection
- When a new face is detected, the robot greets the person
- Tracks faces with head and eye movements
- Maintains a cooldown period (2 minutes) before re-greeting the same person

### Speech Interaction
- Listens for speech when in LISTENING state
- Forwards recognized speech to QA bot
- Speaks responses via TTS
- Coordinates jaw movement with speech

### Gestures
- Performs gesture commands (hold, zero, wave) based on interaction state
- Coordinates arm and hand movements

## Dependencies

- rclpy
- std_msgs
- control_msgs
- trajectory_msgs
- face_tracker_msgs
- Robot hardware/controllers
- Face tracker node
- Speech recognizer node
- QA bot node
- TTS service node
- Hand gestures node

## Configuration

The node automatically configures movement parameters based on simulation vs hardware mode:

- **Simulation**: Relaxed movement limits for testing
- **Hardware**: Actual servo limits and positions

## Troubleshooting

### Node Not Connecting to Controllers
- Ensure robot hardware/simulation is launched first
- Check that action servers are available: `ros2 action list`

### Speech Not Working
- Verify microphone is connected and working
- Check speech_recognizer node is running
- Ensure TTS service is active

### Face Tracking Not Working
- Verify camera is connected
- Check face_tracker node is running
- Ensure camera permissions are set

## Building

Build the package:

```bash
cd /workspace
colcon build --packages-select unified_robot_control
source install/setup.bash
```

## License

MIT
