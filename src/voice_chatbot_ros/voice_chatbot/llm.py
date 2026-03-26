"""
LLM chat inference using `llama-cpp-python <https://github.com/abetlen/llama-cpp-python>`_.

Wraps a GGUF model loaded through ``llama-cpp-python`` with a simple
multi-turn conversation interface.  A system prompt is prepended to
every request and the conversation history is automatically trimmed
to the configured maximum number of turns.

Usage::

    llm = ChatLLM(config)
    reply = llm.chat("Hei, kuka olet?")
    llm.clear_history()
"""

from typing import cast

from llama_cpp import Llama
from llama_cpp.llama_types import (
    ChatCompletionRequestAssistantMessage,
    ChatCompletionRequestMessage,
    ChatCompletionRequestSystemMessage,
    ChatCompletionRequestUserMessage,
    CreateChatCompletionResponse,
)

from .config import Config


class ChatLLM:
    """Multi-turn chat wrapper around a GGUF LLM.

    Maintains an in-memory conversation history (list of
    ``{"role": ..., "content": ...}`` dicts).  The system prompt is
    **not** stored in the history — it is prepended on every call so
    that trimming old turns never accidentally removes it.

    Args:
        config: Shared :class:`Config` – reads model path, GPU layers,
                context size, generation parameters, system prompt, and
                turn limit.
    """

    def __init__(self, config: Config):
        print(f"[LLM] Loading model from '{config.llm_model_path}'...")
        self._llm = Llama(
            model_path=config.llm_model_path,
            n_gpu_layers=config.llm_n_gpu_layers,
            n_ctx=config.llm_n_ctx,
            verbose=False,
        )
        self._system_prompt = config.llm_system_prompt
        self._max_turns = config.max_conversation_turns
        self._max_tokens = config.llm_max_tokens
        self._temperature = config.llm_temperature
        self._conversation_history: list[ChatCompletionRequestMessage] = []
        print("[LLM] Model loaded.")

    def chat(self, user_message: str) -> str:
        """Send a user message and return the assistant's reply.

        The full conversation history (trimmed to
        ``max_conversation_turns`` turn pairs) plus the system prompt is
        sent to the model on every call.

        Args:
            user_message: The latest user utterance.

        Returns:
            The assistant's reply text, or ``""`` if the model produced
            no output.
        """
        user_entry: ChatCompletionRequestUserMessage = {
            "role": "user",
            "content": user_message,
        }
        self._conversation_history.append(user_entry)

        # Each "turn" is one user + one assistant message (2 entries).
        # Trim from the front to keep the most recent context.
        max_messages = self._max_turns * 2
        if len(self._conversation_history) > max_messages:
            self._conversation_history = self._conversation_history[-max_messages:]

        # Prepend the system prompt (not part of trimmed history).
        system_entry: ChatCompletionRequestSystemMessage = {
            "role": "system",
            "content": self._system_prompt,
        }
        messages: list[ChatCompletionRequestMessage] = [
            system_entry,
            *self._conversation_history,
        ]

        response = cast(
            CreateChatCompletionResponse,
            self._llm.create_chat_completion(
                messages=messages,
                max_tokens=self._max_tokens,
                temperature=self._temperature,
                stream=False,
            ),
        )

        assistant_text = response["choices"][0]["message"].get("content") or ""
        if assistant_text:
            assistant_entry: ChatCompletionRequestAssistantMessage = {
                "role": "assistant",
                "content": assistant_text,
            }
            self._conversation_history.append(assistant_entry)
            return assistant_text
        return ""

    def clear_history(self):
        """Erase all stored conversation turns."""
        self._conversation_history = []
