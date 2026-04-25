"""GUI-facing config wrapper around the shared voice stack config."""

from voice_stack_common.config import (
    VoiceChatConfig as Config,
    default_config_path,
    resolve_config_path,
)

__all__ = ["Config", "default_config_path", "resolve_config_path"]
