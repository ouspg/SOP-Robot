"""Configuration-selected LLM backend facade."""

from __future__ import annotations

from collections.abc import Callable

from voice_stack_common.config import VoiceChatConfig

from .backends import ChatBackend, LlamaCppChatBackend


class ChatLLM:
    """Configuration-selected multi-turn chat engine."""

    def __init__(self, config: VoiceChatConfig):
        raw_backend = getattr(config, "llm_backend", "llama-cpp")
        backend = normalize_llm_backend(raw_backend)
        try:
            backend_factory = LLM_BACKENDS[backend]
        except KeyError:
            raise ValueError(
                f"Unsupported llm_backend {raw_backend!r}; use 'llama-cpp'."
            ) from None
        self._impl: ChatBackend = backend_factory(config)

    def chat(self, user_message: str) -> str:
        return self._impl.chat(user_message)

    def clear_history(self) -> None:
        self._impl.clear_history()


def normalize_llm_backend(raw_backend: str | None) -> str:
    if raw_backend is None or not raw_backend.strip():
        return "llama-cpp"
    normalized = raw_backend.strip().lower().replace("_", "-")
    aliases = {
        "llama": "llama-cpp",
        "llama.cpp": "llama-cpp",
        "llamacpp": "llama-cpp",
        "llama-cpp-python": "llama-cpp",
    }
    return aliases.get(normalized, normalized)


LLM_BACKENDS: dict[str, Callable[[VoiceChatConfig], ChatBackend]] = {
    "llama-cpp": LlamaCppChatBackend,
}


__all__ = [
    "ChatLLM",
    "LLM_BACKENDS",
    "LlamaCppChatBackend",
    "normalize_llm_backend",
]
