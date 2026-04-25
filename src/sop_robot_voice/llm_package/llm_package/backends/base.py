"""Common LLM backend interfaces."""

from __future__ import annotations

from typing import Protocol


class ChatBackend(Protocol):
    def chat(self, user_message: str) -> str:
        """Return the assistant response for a user message."""
        ...

    def clear_history(self) -> None:
        """Clear any backend conversation history."""
        ...
