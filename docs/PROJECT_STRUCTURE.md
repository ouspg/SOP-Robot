# Project Structure

The workspace is grouped by subsystem while preserving standard ROS 2 package
metadata (`package.xml`, `setup.py`/`CMakeLists.txt`, `resource/`, `test/`, and
package-local `config/` or `launch/` directories). Colcon still discovers
packages recursively from `src/`.

## Top-Level Layout

- `config/`: operator-facing runtime config consumed by canonical launches
- `docs/`: setup, bring-up, testing, and contract documentation
- `src/`: active ROS 2 packages, grouped by subsystem
- `src/vendor/`: third-party ROS packages/submodules
- `tools/`: Pixi/ROS launch wrappers plus setup, device, monitoring, and runtime helpers
- `testing/`: project-level system tests and cross-package benchmark harnesses
- `benchmarks/`, `reports/`, `test-results/`: generated output directories

## Source Tree

```text
src/
  sop_robot_bringup/
    robot/                  # canonical launch orchestration and robot action package
    full_demo/              # high-level demo coordinator
  sop_robot_common/         # shared robot contracts/calibration/action helpers
  sop_robot_description/
    inmoov_description/     # URDF, RViz, controller description assets
    inmoov_meshes/          # mesh package
  sop_robot_gui/
    chatbot_app/            # Qt/ROS UI packages
  sop_robot_motion/
    hand_gestures/
    jaw_movement/
    robot_hardware/
    shoulder_controller/
  sop_robot_perception/
    face_tracker/
    face_tracker_movement/
    face_tracker_msgs/
  sop_robot_voice/
    asr_package/
    llm_package/
    tts_package/
    voice_stack_common/
  vendor/
    dynamixel-workbench/
    dynamixel-workbench-msgs/
    ros2graph_explorer/
```

## Python ROS 2 Package Layout

Active Python packages now use package-root modules as the canonical import and
console-entrypoint surface. Compatibility wrapper directories such as `nodes/`
and `features/` have been removed from first-party packages.

```text
src/<subsystem>/<package>/
  package.xml
  setup.py
  setup.cfg
  resource/<package>
  launch/                  # optional package-local launch files
  test/                    # node/package-level tests
  benchmarks/              # optional node/package-level benchmark sources
  config/                  # optional package-shipped defaults
  <package>/
    __init__.py
    <feature>.py
    <node>_node.py
```

Examples:

- `src/sop_robot_voice/asr_package/asr_package/asr_node.py`
- `src/sop_robot_voice/llm_package/llm_package/llm_node.py`
- `src/sop_robot_gui/chatbot_app/chatbot_app/unified_app.py`
- `src/sop_robot_voice/tts_package/tts_package/tts_node.py`
- `src/sop_robot_perception/face_tracker/face_tracker/face_tracker_node.py`

## Message Package Layout

ROS interface packages keep the standard structure:

```text
src/<subsystem>/<msg_package>/
  package.xml
  CMakeLists.txt
  msg/
  srv/                    # optional
  action/                 # optional
```

Example:

- `src/sop_robot_perception/face_tracker_msgs/msg/`

## Config Conventions

Use these config locations consistently:

- repo root `config/`: operator-facing runtime config used by canonical launches
- package `config/`: package-shipped defaults or reusable examples
- shared common packages: contract and config-loading helpers, not per-machine state

Examples:

- `config/robot_stack.yaml`
- `config/voice_chatbot.json`
- `src/sop_robot_bringup/robot/config/robot_stack.defaults.yaml`
- `src/sop_robot_voice/voice_stack_common/config/voice_chatbot.defaults.json`

## Running And Launching

Runtime ownership is split like this:

- `robot` owns project bring-up and launch orchestration
- subsystem packages may ship focused launch files for debugging
- package console entrypoints are for package-local execution and tests, not for replacing
  the canonical `robot` bring-up path

Canonical launch entrypoints:

- `src/sop_robot_bringup/robot/launch/robot.launch.py`
- `src/sop_robot_bringup/robot/launch/robot.fake.launch.py`

`robot.launch.py` is intentionally thin; implementation is split under
`src/sop_robot_bringup/robot/launch/robot_launch/`.

## Creating New Nodes

When adding a new Python runtime package:

1. Create the ROS package under the matching `src/sop_robot_<subsystem>/` group.
2. Add the canonical executable as a package-root `<name>_node.py` module.
3. Put reusable non-ROS logic in package-root modules and keep imports explicit.
4. Add a package-local launch file only if the package needs focused standalone debugging.
5. If the package participates in project bring-up, wire it into the `robot` launch modules
   instead of inventing a new top-level project entrypoint.
6. Centralize shared topic/service names in a common contracts module when more than one
   package depends on them.

## Shared Contracts

Cross-package topic and service names must be centralized instead of hard-coded:

- `src/sop_robot_voice/voice_stack_common/voice_stack_common/contracts.py`
- `src/sop_robot_common/sop_robot_common/contracts.py`

This keeps topic names consistent across ASR, LLM, TTS, GUI, face tracking, and
demo orchestration.
