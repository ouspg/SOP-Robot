#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

PYTEST_ARGS=(
    src/sop_robot_bringup/robot/launch/test/test_launch_utils.py
    src/sop_robot_voice/asr_package/test/test_stt_config.py
    src/sop_robot_perception/face_tracker_movement/test/test_face_tracker_movement_logic.py
    src/sop_robot_voice/llm_package/test/test_answer_selection.py
    src/sop_robot_voice/llm_package/test/test_knowledge_eval.py
    src/sop_robot_voice/llm_package/test/test_knowledge_base.py
    src/sop_robot_voice/llm_package/test/test_llm_engine.py
    src/sop_robot_common/test/test_arm_actions.py
    src/sop_robot_common/test/test_joint_calibration.py
    src/sop_robot_voice/voice_stack_common/test/test_config.py
    src/sop_robot_common/test/test_contracts.py
)

exec bash tools/with_ros_env.sh pytest "${PYTEST_ARGS[@]}"
