"""Answer arbitration between generated text and local retrieved knowledge."""

from __future__ import annotations

import re
from dataclasses import dataclass
from difflib import SequenceMatcher
from typing import Literal

from .knowledge_base import RetrievalResult, normalize_question

AnswerSource = Literal["llm", "knowledge_base"]

_TOKEN_RE = re.compile(r"[\w]+", re.UNICODE)


@dataclass(frozen=True)
class AnswerSelection:
    """Chosen answer plus the scores used to make the decision."""

    answer: str
    source: AnswerSource
    query_similarity: float
    answer_similarity: float


def choose_closest_answer(
    *,
    user_message: str,
    llm_answer: str,
    retrieval_results: list[RetrievalResult],
    allow_knowledge_answer: bool = True,
) -> AnswerSelection:
    """Choose between the model answer and the strongest local KB candidate.

    The KB answer wins when the user's question is close to the retrieved KB
    question and the generated answer is not already close to the KB answer.
    This keeps exact known facts deterministic while still allowing the LLM to
    use its own phrasing when it agrees with the local data.
    """

    normalized_llm_answer = llm_answer.strip()
    if not retrieval_results or not allow_knowledge_answer:
        return AnswerSelection(
            answer=normalized_llm_answer,
            source="llm",
            query_similarity=0.0,
            answer_similarity=0.0,
        )

    best_result = retrieval_results[0]
    query_similarity = _question_similarity(user_message, best_result.question)
    answer_similarity = _text_similarity(normalized_llm_answer, best_result.answer)

    if not normalized_llm_answer:
        return AnswerSelection(
            answer=best_result.answer,
            source="knowledge_base",
            query_similarity=query_similarity,
            answer_similarity=answer_similarity,
        )

    exact_or_near_exact = best_result.method == "exact" or query_similarity >= 0.92
    high_confidence_retrieval = query_similarity >= 0.72
    llm_already_matches_kb = answer_similarity >= 0.82

    if (exact_or_near_exact or high_confidence_retrieval) and not llm_already_matches_kb:
        return AnswerSelection(
            answer=best_result.answer,
            source="knowledge_base",
            query_similarity=query_similarity,
            answer_similarity=answer_similarity,
        )

    return AnswerSelection(
        answer=normalized_llm_answer,
        source="llm",
        query_similarity=query_similarity,
        answer_similarity=answer_similarity,
    )


def _question_similarity(left: str, right: str) -> float:
    left_normalized = normalize_question(left)
    right_normalized = normalize_question(right)
    if not left_normalized or not right_normalized:
        return 0.0
    if left_normalized == right_normalized:
        return 1.0
    return max(
        _sequence_similarity(left_normalized, right_normalized),
        _token_containment(left_normalized, right_normalized),
    )


def _text_similarity(left: str, right: str) -> float:
    left_normalized = normalize_question(left)
    right_normalized = normalize_question(right)
    if not left_normalized or not right_normalized:
        return 0.0
    if left_normalized == right_normalized:
        return 1.0
    return max(
        _sequence_similarity(left_normalized, right_normalized),
        _token_f1(left_normalized, right_normalized),
    )


def _sequence_similarity(left: str, right: str) -> float:
    return SequenceMatcher(None, left, right).ratio()


def _token_containment(left: str, right: str) -> float:
    left_tokens = set(_TOKEN_RE.findall(left))
    right_tokens = set(_TOKEN_RE.findall(right))
    if not left_tokens or not right_tokens:
        return 0.0
    return len(left_tokens & right_tokens) / min(len(left_tokens), len(right_tokens))


def _token_f1(left: str, right: str) -> float:
    left_tokens = set(_TOKEN_RE.findall(left))
    right_tokens = set(_TOKEN_RE.findall(right))
    if not left_tokens or not right_tokens:
        return 0.0
    overlap = len(left_tokens & right_tokens)
    if overlap == 0:
        return 0.0
    precision = overlap / len(left_tokens)
    recall = overlap / len(right_tokens)
    return (2 * precision * recall) / (precision + recall)
