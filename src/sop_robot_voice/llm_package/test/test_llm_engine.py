import sys
from pathlib import Path
from typing import Any, cast

import pytest


COMMON_ROOT = Path(__file__).resolve().parents[2] / "voice_stack_common"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
for path in (COMMON_ROOT, PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from llm_package.llm_engine import ChatLLM, LLM_BACKENDS, normalize_llm_backend
from voice_stack_common.config import VoiceChatConfig


def test_llm_backend_aliases_normalize():
    assert normalize_llm_backend(None) == "llama-cpp"
    assert normalize_llm_backend("") == "llama-cpp"
    assert normalize_llm_backend("llama_cpp") == "llama-cpp"
    assert normalize_llm_backend("llama.cpp") == "llama-cpp"
    assert normalize_llm_backend("llama-cpp-python") == "llama-cpp"


def test_chat_llm_selects_normalized_backend(monkeypatch):
    class StubBackend:
        def __init__(self, config):
            self.config = config
            self.cleared = False

        def chat(self, user_message):
            return f"echo:{user_message}:{self.config.llm_backend}"

        def clear_history(self):
            self.cleared = True

    monkeypatch.setitem(LLM_BACKENDS, "llama-cpp", StubBackend)

    config = VoiceChatConfig()
    config.llm_backend = "llama_cpp"
    llm = ChatLLM(config)

    assert llm.chat("hello") == "echo:hello:llama_cpp"
    llm.clear_history()
    assert cast(Any, llm._impl).cleared is True


def test_chat_llm_rejects_unknown_backend():
    config = VoiceChatConfig()
    config.llm_backend = "unknown"

    with pytest.raises(ValueError, match="Unsupported llm_backend"):
        ChatLLM(config)
