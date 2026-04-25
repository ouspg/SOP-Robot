import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from voice_stack_common.config import CONFIG_ENV_VAR, VoiceChatConfig, default_config_path


def test_voice_chat_config_load_filters_unknown_fields(tmp_path):
    config_path = tmp_path / "voice_chatbot.json"
    config_path.write_text(
        json.dumps(
            {
                "language": "en",
                "llm_max_tokens": 512,
                "unused_field": "ignored",
            }
        ),
        encoding="utf-8",
    )

    config = VoiceChatConfig.load(path=config_path)

    assert config.language == "en"
    assert config.llm_max_tokens == 512
    assert not hasattr(config, "unused_field")


def test_default_config_path_respects_environment_override(monkeypatch, tmp_path):
    config_path = tmp_path / "custom_voice_config.json"
    monkeypatch.setenv(CONFIG_ENV_VAR, str(config_path))

    assert default_config_path() == config_path
