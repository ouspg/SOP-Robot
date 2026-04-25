"""Convenience fake-hardware launch entry point for SOP Robot bring-up."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = Path(__file__).resolve().parent
for _candidate in (_LAUNCH_DIR, _LAUNCH_DIR.parent):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from controllers.launch_utils import STACK_CONFIG  # noqa: E402
from robot_launch.arguments import launch_arguments  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    robot_launch = os.path.join(os.path.dirname(__file__), "robot.launch.py")
    argument_actions = launch_arguments(str(STACK_CONFIG), use_fake_default="true")
    forwarded_args: dict[str, LaunchConfiguration | str] = {
        action.name: LaunchConfiguration(action.name) for action in argument_actions
    }
    forwarded_args["use_fake_hardware"] = "true"

    return LaunchDescription(
        [
            *argument_actions,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments=forwarded_args.items(),
            ),
        ]
    )
