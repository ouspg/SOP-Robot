from pathlib import Path
import re
import tempfile
from typing import Any

import yaml


def _find_project_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "config" / "robot_stack.yaml").is_file():
            return parent
    return Path(__file__).resolve().parents[3]


PROJECT_ROOT = _find_project_root()
CONFIG_ROOT = PROJECT_ROOT / "config"
HEAD_CONFIG = CONFIG_ROOT / "dynamixel_head.yaml"
ARM_CONFIG = CONFIG_ROOT / "dynamixel_arm.yaml"
ALL_CONFIGS = [HEAD_CONFIG, ARM_CONFIG]
STACK_CONFIG = CONFIG_ROOT / "robot_stack.yaml"
DEFAULT_STACK_CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "robot_stack.defaults.yaml"
)


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


def load_stack_config(config_path: str | None = None) -> dict[str, Any]:
    candidate = Path(config_path) if config_path else STACK_CONFIG
    if not candidate.is_file():
        candidate = DEFAULT_STACK_CONFIG
    data = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


def resolve_launch_value(raw_value: str, config_value: Any, fallback: Any) -> Any:
    return raw_value if raw_value != "" else config_value if config_value is not None else fallback


def resolve_project_path(raw_value: str | Path) -> str:
    path = Path(raw_value).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return str(path)


def parse_bool(raw_value: Any, default: bool = False) -> bool:
    if isinstance(raw_value, bool):
        return raw_value
    if raw_value is None:
        return default
    normalized = str(raw_value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return default


def normalize_cpu_cores(raw_value: Any) -> str:
    """Normalize a Linux taskset CPU-list value.

    Accepted examples: "4", "2,3", "2-5", "0,2-3".
    """
    if raw_value is None:
        return ""

    normalized = str(raw_value).strip().replace(" ", "")
    if not normalized:
        return ""

    cpu_item = r"\d+(?:-\d+)?"
    if not re.fullmatch(rf"{cpu_item}(?:,{cpu_item})*", normalized):
        raise ValueError(
            f"Invalid CPU affinity value {raw_value!r}. "
            "Use a Linux CPU list such as '4', '2,3', or '2-5'."
        )
    return normalized


def cpu_affinity_prefix(raw_value: Any) -> str | None:
    cores = normalize_cpu_cores(raw_value)
    if not cores:
        return None
    return f"taskset -c {cores}"
