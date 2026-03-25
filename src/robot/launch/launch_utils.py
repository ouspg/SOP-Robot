from pathlib import Path
import tempfile


CONFIG_ROOT = Path(__file__).resolve().parents[3] / "config"
HEAD_CONFIG = CONFIG_ROOT / "dynamixel_head.yaml"
ARM_CONFIG = CONFIG_ROOT / "dynamixel_arm.yaml"
ALL_CONFIGS = [HEAD_CONFIG, ARM_CONFIG]


def resolve_included_config_files(robot_parts):
    normalized = (robot_parts or "all").lower()
    included_files = []

    if normalized in ("all", "head,arm", "arm,head"):
        included_files = ALL_CONFIGS
    else:
        if "head" in normalized:
            included_files.append(HEAD_CONFIG)
        if "arm" in normalized:
            included_files.append(ARM_CONFIG)

    return included_files or ALL_CONFIGS


def create_dynamixel_config_file(robot_parts):
    included_files = resolve_included_config_files(robot_parts)
    output_path = Path(tempfile.gettempdir()) / "sop_robot_dynamixel_launch.yaml"

    header = (
        "# Temporary file generated during launch.\n"
        "# Includes only the Dynamixel configuration files needed for the selected robot parts.\n\n"
    )
    with output_path.open("w", encoding="utf-8") as outfile:
        outfile.write(header)
        for filename in included_files:
            outfile.write(filename.read_text(encoding="utf-8"))
            outfile.write("\n")

    return str(output_path)


def parse_controller_argument(raw_value):
    return [controller.strip() for controller in raw_value.split(",") if controller.strip()]
