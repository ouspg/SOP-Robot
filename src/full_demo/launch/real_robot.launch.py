from full_demo.robot_demo_launch import (
    generate_robot_demo_launch_description,
)


def generate_launch_description():
    return generate_robot_demo_launch_description(use_fake_hardware=False)
