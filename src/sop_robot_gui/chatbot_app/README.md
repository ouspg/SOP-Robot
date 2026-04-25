# chatbot_app

ROS2 GUI package for the SOP Robot voice stack.

This package contains:

- a plain chat window connected to the ROS2 voice stack
- a unified SOP-Robot window that combines chat, RViz, image viewing, and arm/hand controls
- shared PySide6 widgets and ROS bridge logic

It does not load STT, LLM, or TTS models locally.

Executables:

- `ros2 run chatbot_app chatbot_app`
- `ros2 run chatbot_app chatbot_app_unified`

The unified UI is normally launched from `robot.launch.py` with
`enable_chatbot_ui:=true`.

The unified UI publishes `/arms/arm_action` commands and includes the in-tree
arm action bridge that forwards hand gestures to the hand topics and shoulder
positions to the configured serial servo controller when available.
