"""Shared ROS helpers for ASR nodes."""

from voice_stack_common.config import VoiceChatConfig
from voice_stack_common.ros_helpers import VoiceNodeBase

DEFAULT_SAMPLE_RATE = VoiceChatConfig.sample_rate

class AsrNodeBase(VoiceNodeBase):
    """ASR-specific alias for the shared voice stack ROS base."""
