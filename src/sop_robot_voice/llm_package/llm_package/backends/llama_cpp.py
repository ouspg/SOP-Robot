"""llama-cpp-python chat backend."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, cast

from voice_stack_common.config import VoiceChatConfig
from voice_stack_common.platform_setup import setup_cuda

from ..answer_selection import choose_closest_answer
from ..knowledge_base import (
    DEFAULT_DATA_DIR,
    KnowledgeBase,
    RetrievalResult,
    format_retrieval_context,
)

ChatMessage = dict[str, str]


class LlamaCppChatBackend:
    """Multi-turn chat backend around a GGUF llama-cpp-python model."""

    def __init__(self, config: VoiceChatConfig):
        setup_cuda()

        from llama_cpp import Llama

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
        self._conversation_history: list[ChatMessage] = []
        self._knowledge_base_direct_answer = bool(
            getattr(config, "llm_knowledge_base_direct_answer", True)
        )
        self._knowledge_base_search_limit = int(
            getattr(config, "llm_knowledge_base_search_limit", 3)
        )
        self._knowledge_base = self._load_knowledge_base(config)
        print("[LLM] Model loaded.")

    def chat(self, user_message: str) -> str:
        user_entry: ChatMessage = {
            "role": "user",
            "content": user_message,
        }

        system_prompt = self._system_prompt
        retrieval_results: list[RetrievalResult] = []
        if self._knowledge_base is not None:
            retrieval_results = self._knowledge_base.search(
                user_message,
                limit=self._knowledge_base_search_limit,
            )
            if retrieval_results:
                context = format_retrieval_context(retrieval_results)
                system_prompt = (
                    f"{self._system_prompt}\n\n"
                    "Local knowledge base candidates:\n"
                    f"{context}\n\n"
                    "Use the local knowledge base candidates only when they directly "
                    "answer the user's question."
                )

        system_entry: ChatMessage = {
            "role": "system",
            "content": system_prompt,
        }
        messages: list[ChatMessage] = [
            system_entry,
            *self._conversation_history,
            user_entry,
        ]

        response: Any = self._llm.create_chat_completion(
            messages=cast(Any, messages),
            max_tokens=self._max_tokens,
            temperature=self._temperature,
            stream=False,
        )
        assistant_text = response["choices"][0]["message"].get("content") or ""
        selected = choose_closest_answer(
            user_message=user_message,
            llm_answer=assistant_text,
            retrieval_results=retrieval_results,
            allow_knowledge_answer=self._knowledge_base_direct_answer,
        )
        if not selected.answer:
            return ""

        self._remember_turn(user_entry, selected.answer)
        return selected.answer

    def _remember_turn(self, user_entry: ChatMessage, assistant_text: str) -> None:
        assistant_entry: ChatMessage = {
            "role": "assistant",
            "content": assistant_text,
        }
        self._conversation_history.extend([user_entry, assistant_entry])
        max_messages = self._max_turns * 2
        if len(self._conversation_history) > max_messages:
            self._conversation_history = self._conversation_history[-max_messages:]

    def clear_history(self) -> None:
        self._conversation_history = []

    def _load_knowledge_base(self, config: VoiceChatConfig) -> KnowledgeBase | None:
        disabled = os.environ.get("SOP_ROBOT_LLM_KB_DISABLE", "").casefold()
        if disabled in {"1", "true", "yes", "on"}:
            return None
        if not bool(getattr(config, "llm_knowledge_base_enabled", True)):
            return None

        data_dir = Path(
            getattr(
                config,
                "llm_knowledge_base_data_dir",
                os.environ.get("SOP_ROBOT_LLM_KB_DATA_DIR", str(DEFAULT_DATA_DIR)),
            )
        ).expanduser()
        if not data_dir.is_dir():
            print(f"[LLM] Knowledge base data directory not found: {data_dir}")
            return None

        db_path = getattr(
            config,
            "llm_knowledge_base_path",
            os.environ.get("SOP_ROBOT_LLM_KB_PATH", ":memory:"),
        )
        try:
            knowledge_base = KnowledgeBase(db_path)
            if bool(getattr(config, "llm_knowledge_base_recreate", False)):
                knowledge_base.clear()
            inserted = knowledge_base.ingest_directory(data_dir)
            count = knowledge_base.count()
            if count == 0:
                knowledge_base.close()
                return None
            print(
                "[LLM] Knowledge base ready: "
                f"{count} entries ({inserted} new), "
                f"fts={'yes' if knowledge_base.fts_available else 'no'}."
            )
            return knowledge_base
        except Exception as exc:
            print(f"[LLM] Knowledge base initialization skipped: {exc}")
            return None
