from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).resolve().parents[2] / "controllers" / "launch_utils.py"
SPEC = importlib.util.spec_from_file_location("controllers.launch_utils", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
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


def test_cpu_affinity_prefix_accepts_linux_cpu_lists():
    assert MODULE.cpu_affinity_prefix("") is None
    assert MODULE.cpu_affinity_prefix(None) is None
    assert MODULE.cpu_affinity_prefix("4") == "taskset -c 4"
    assert MODULE.cpu_affinity_prefix("2,3") == "taskset -c 2,3"
    assert MODULE.cpu_affinity_prefix("2-5") == "taskset -c 2-5"
    assert MODULE.cpu_affinity_prefix("0, 2-3") == "taskset -c 0,2-3"


def test_cpu_affinity_prefix_rejects_shell_syntax():
    try:
        MODULE.cpu_affinity_prefix("2; echo bad")
    except ValueError as exc:
        assert "Invalid CPU affinity value" in str(exc)
    else:
        raise AssertionError("Expected invalid CPU affinity to raise ValueError")
