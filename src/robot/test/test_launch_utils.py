from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).resolve().parents[1] / "launch" / "launch_utils.py"
SPEC = importlib.util.spec_from_file_location("launch_utils", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_resolve_included_config_files_supports_head_and_arm():
    assert MODULE.resolve_included_config_files("head") == [MODULE.HEAD_CONFIG]
    assert MODULE.resolve_included_config_files("arm") == [MODULE.ARM_CONFIG]
    assert MODULE.resolve_included_config_files("all") == MODULE.ALL_CONFIGS


def test_create_dynamixel_config_file_merges_requested_configs():
    output_path = Path(MODULE.create_dynamixel_config_file("head"))
    contents = output_path.read_text(encoding="utf-8")

    assert output_path.exists()
    assert "Temporary file generated during launch" in contents
    assert "head_pan_joint" in contents
