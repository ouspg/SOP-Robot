"""Deterministic evaluation data and scoring for LLM knowledge retrieval."""

from __future__ import annotations

import json
import os
import re
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Literal

from .answer_selection import choose_closest_answer
from .knowledge_base import (
    KnowledgeBase,
    RetrievalResult,
    format_retrieval_context,
    normalize_question,
)

EvalKind = Literal["exact", "similar_question", "mixed_answer", "very_different"]
EvalMode = Literal["without_kb", "with_kb"]

_TOKEN_RE = re.compile(r"[\w]+", re.UNICODE)
_LLAMA_CPP_CACHE: dict[tuple[str, str, int, int], Any] = {}


@dataclass(frozen=True)
class KnowledgeEvalCase:
    """A single generated QA comparison case."""

    id: str
    kind: EvalKind
    question: str
    expected_answer: str
    simulated_llm_answer: str
    source_question: str
    source_answer: str
    negative_answer: str
    should_use_knowledge_base: bool


@dataclass(frozen=True)
class KnowledgeEvalRun:
    """Result from one simulated or model-backed answer mode."""

    answer: str
    source: str
    correct: bool
    ms: float
    query_similarity: float
    answer_similarity: float
    retrieval_count: int


@dataclass(frozen=True)
class KnowledgeEvalComparison:
    """Side-by-side result for one eval case."""

    case: KnowledgeEvalCase
    without_kb: KnowledgeEvalRun
    with_kb: KnowledgeEvalRun


@dataclass(frozen=True)
class ModelSpec:
    """Named model command used for evaluation."""

    name: str
    command: str | None = None


@dataclass(frozen=True)
class KnowledgeModelComparison:
    """All KB/no-KB comparisons for one model."""

    model: ModelSpec
    summary: dict[str, Any]
    comparisons: list[KnowledgeEvalComparison]


@dataclass(frozen=True)
class KnowledgeLearningStep:
    """Before/after result for one correction learned into the KB."""

    case: KnowledgeEvalCase
    before: KnowledgeEvalRun
    learned: bool
    after: KnowledgeEvalRun


@dataclass(frozen=True)
class KnowledgeLearningRun:
    """Sequential run where corrections are written into the KB."""

    model: ModelSpec
    initial_count: int
    final_count: int
    summary: dict[str, Any]
    steps: list[KnowledgeLearningStep]


def generate_eval_cases(kb: KnowledgeBase, *, base_limit: int = 20) -> list[KnowledgeEvalCase]:
    """Create exact, similar, mixed, and unrelated cases from KB entries."""

    entries = kb.entries(limit=base_limit)
    if not entries:
        return []

    cases: list[KnowledgeEvalCase] = []
    for index, entry in enumerate(entries):
        negative = _different_answer(entries, index)
        base_id = f"{index + 1:03d}"
        cases.extend(
            [
                KnowledgeEvalCase(
                    id=f"{base_id}-exact",
                    kind="exact",
                    question=entry.question,
                    expected_answer=entry.answer,
                    simulated_llm_answer=negative,
                    source_question=entry.question,
                    source_answer=entry.answer,
                    negative_answer=negative,
                    should_use_knowledge_base=True,
                ),
                KnowledgeEvalCase(
                    id=f"{base_id}-similar",
                    kind="similar_question",
                    question=_similar_question(entry.question, index),
                    expected_answer=entry.answer,
                    simulated_llm_answer="En ole varma oikeasta vastauksesta.",
                    source_question=entry.question,
                    source_answer=entry.answer,
                    negative_answer=negative,
                    should_use_knowledge_base=True,
                ),
                KnowledgeEvalCase(
                    id=f"{base_id}-mixed",
                    kind="mixed_answer",
                    question=entry.question,
                    expected_answer=entry.answer,
                    simulated_llm_answer=negative,
                    source_question=entry.question,
                    source_answer=entry.answer,
                    negative_answer=negative,
                    should_use_knowledge_base=True,
                ),
                KnowledgeEvalCase(
                    id=f"{base_id}-different",
                    kind="very_different",
                    question=_very_different_question(index),
                    expected_answer="",
                    simulated_llm_answer="Tähän ei ole paikallista tietokantavastausta.",
                    source_question=entry.question,
                    source_answer=entry.answer,
                    negative_answer=negative,
                    should_use_knowledge_base=False,
                ),
            ]
        )
    return cases


def compare_knowledge_base_runs(
    kb: KnowledgeBase,
    cases: Iterable[KnowledgeEvalCase],
    *,
    limit: int = 3,
    model_command: str | None = None,
    model_name: str = "model",
    model_timeout: float = 60.0,
) -> list[KnowledgeEvalComparison]:
    """Compare answers with retrieval disabled and enabled."""

    comparisons: list[KnowledgeEvalComparison] = []
    for case in cases:
        without_kb = _run_case(
            kb,
            case,
            mode="without_kb",
            limit=limit,
            model_name=model_name,
            model_command=model_command,
            model_timeout=model_timeout,
        )
        with_kb = _run_case(
            kb,
            case,
            mode="with_kb",
            limit=limit,
            model_name=model_name,
            model_command=model_command,
            model_timeout=model_timeout,
        )
        comparisons.append(
            KnowledgeEvalComparison(
                case=case,
                without_kb=without_kb,
                with_kb=with_kb,
            )
        )
    return comparisons


def compare_model_runs(
    kb: KnowledgeBase,
    cases: Iterable[KnowledgeEvalCase],
    models: Iterable[ModelSpec],
    *,
    limit: int = 3,
    model_timeout: float = 60.0,
) -> list[KnowledgeModelComparison]:
    """Compare multiple models with KB disabled and enabled."""

    materialized_cases = list(cases)
    output: list[KnowledgeModelComparison] = []
    for model in models:
        comparisons = compare_knowledge_base_runs(
            kb,
            materialized_cases,
            limit=limit,
            model_command=model.command,
            model_name=model.name,
            model_timeout=model_timeout,
        )
        output.append(
            KnowledgeModelComparison(
                model=model,
                summary=summarize_comparisons(comparisons),
                comparisons=comparisons,
            )
        )
    return output


def run_learning_eval(
    kb: KnowledgeBase,
    cases: Iterable[KnowledgeEvalCase],
    model: ModelSpec,
    *,
    limit: int = 3,
    model_timeout: float = 60.0,
    learn_source_path: str = "runtime:llm_correction",
) -> KnowledgeLearningRun:
    """Run correction cases, write expected answers to the KB, and rerun them."""

    initial_count = kb.count()
    steps: list[KnowledgeLearningStep] = []
    for case in cases:
        before = _run_case(
            kb,
            case,
            mode="with_kb",
            limit=limit,
            model_name=model.name,
            model_command=model.command,
            model_timeout=model_timeout,
        )
        learned = False
        learn_question = case.source_question or case.question
        existing_answer = kb.direct_answer(learn_question)
        needs_learning = bool(case.expected_answer) and normalize_question(
            existing_answer or ""
        ) != normalize_question(case.expected_answer)
        if needs_learning:
            learned = kb.learn_answer(
                learn_question,
                case.expected_answer,
                source_path=learn_source_path,
                source_format="learned",
                category="llm_correction",
                metadata={
                    "case_id": case.id,
                    "case_kind": case.kind,
                    "model": model.name,
                },
            )
        after = _run_case(
            kb,
            case,
            mode="with_kb",
            limit=limit,
            model_name=model.name,
            model_command=model.command,
            model_timeout=model_timeout,
        )
        steps.append(
            KnowledgeLearningStep(
                case=case,
                before=before,
                learned=learned,
                after=after,
            )
        )

    return KnowledgeLearningRun(
        model=model,
        initial_count=initial_count,
        final_count=kb.count(),
        summary=summarize_learning_steps(steps),
        steps=steps,
    )


def answer_model_prompt(
    prompt: str,
    model: ModelSpec,
    *,
    model_timeout: float = 60.0,
) -> str:
    """Answer one prompt with the model only, without KB retrieval or arbitration."""

    case = KnowledgeEvalCase(
        id="manual-prompt",
        kind="very_different",
        question=prompt,
        expected_answer="",
        simulated_llm_answer="",
        source_question=prompt,
        source_answer="",
        negative_answer="",
        should_use_knowledge_base=False,
    )
    return _model_answer(
        case,
        mode="without_kb",
        model_name=model.name,
        retrieval=[],
        model_command=model.command,
        model_timeout=model_timeout,
    )


def summarize_comparisons(comparisons: list[KnowledgeEvalComparison]) -> dict[str, Any]:
    """Return aggregate metrics for a comparison run."""

    total = len(comparisons)
    without_correct = sum(1 for item in comparisons if item.without_kb.correct)
    with_correct = sum(1 for item in comparisons if item.with_kb.correct)
    changed = sum(1 for item in comparisons if item.without_kb.answer != item.with_kb.answer)
    improved = sum(
        1 for item in comparisons if not item.without_kb.correct and item.with_kb.correct
    )
    regressed = sum(
        1 for item in comparisons if item.without_kb.correct and not item.with_kb.correct
    )
    return {
        "cases": total,
        "without_kb_correct": without_correct,
        "with_kb_correct": with_correct,
        "without_kb_accuracy": _ratio(without_correct, total),
        "with_kb_accuracy": _ratio(with_correct, total),
        "changed_answers": changed,
        "improved": improved,
        "regressed": regressed,
    }


def summarize_model_comparisons(
    model_comparisons: list[KnowledgeModelComparison],
) -> dict[str, Any]:
    """Return compact per-model metrics."""

    return {
        item.model.name: {
            **item.summary,
            "without_kb_mean_ms": _mean(run.without_kb.ms for run in item.comparisons),
            "with_kb_mean_ms": _mean(run.with_kb.ms for run in item.comparisons),
            "without_kb_total_ms": round(
                sum(run.without_kb.ms for run in item.comparisons),
                3,
            ),
            "with_kb_total_ms": round(
                sum(run.with_kb.ms for run in item.comparisons),
                3,
            ),
        }
        for item in model_comparisons
    }


def summarize_learning_steps(steps: list[KnowledgeLearningStep]) -> dict[str, Any]:
    """Return aggregate before/after metrics for a learning run."""

    total = len(steps)
    before_correct = sum(1 for step in steps if step.before.correct)
    after_correct = sum(1 for step in steps if step.after.correct)
    learned = sum(1 for step in steps if step.learned)
    return {
        "cases": total,
        "learned": learned,
        "before_correct": before_correct,
        "after_correct": after_correct,
        "before_accuracy": _ratio(before_correct, total),
        "after_accuracy": _ratio(after_correct, total),
        "improved": sum(
            1 for step in steps if not step.before.correct and step.after.correct
        ),
        "regressed": sum(
            1 for step in steps if step.before.correct and not step.after.correct
        ),
        "before_mean_ms": _mean(step.before.ms for step in steps),
        "after_mean_ms": _mean(step.after.ms for step in steps),
    }


def write_cases_jsonl(cases: Iterable[KnowledgeEvalCase], path: str | Path) -> Path:
    """Write generated cases as JSONL."""

    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        for case in cases:
            handle.write(json.dumps(asdict(case), ensure_ascii=False, sort_keys=True))
            handle.write("\n")
    return target


def read_cases_jsonl(path: str | Path) -> list[KnowledgeEvalCase]:
    """Read generated cases from JSONL."""

    cases: list[KnowledgeEvalCase] = []
    for line in Path(path).expanduser().read_text(encoding="utf-8-sig").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        data = json.loads(stripped)
        cases.append(KnowledgeEvalCase(**data))
    return cases


def comparisons_to_json(comparisons: list[KnowledgeEvalComparison]) -> list[dict[str, Any]]:
    """Convert comparison dataclasses into JSON-serializable dictionaries."""

    return [
        {
            "case": asdict(item.case),
            "without_kb": asdict(item.without_kb),
            "with_kb": asdict(item.with_kb),
        }
        for item in comparisons
    ]


def model_comparisons_to_json(
    model_comparisons: list[KnowledgeModelComparison],
) -> list[dict[str, Any]]:
    """Convert model comparison dataclasses into JSON-serializable dictionaries."""

    return [
        {
            "model": asdict(item.model),
            "summary": item.summary,
            "comparisons": comparisons_to_json(item.comparisons),
        }
        for item in model_comparisons
    ]


def learning_run_to_json(run: KnowledgeLearningRun) -> dict[str, Any]:
    """Convert a learning run into a JSON-serializable dictionary."""

    return {
        "model": asdict(run.model),
        "initial_count": run.initial_count,
        "final_count": run.final_count,
        "summary": run.summary,
        "steps": [
            {
                "case": asdict(step.case),
                "before": asdict(step.before),
                "learned": step.learned,
                "after": asdict(step.after),
            }
            for step in run.steps
        ],
    }


def _run_case(
    kb: KnowledgeBase,
    case: KnowledgeEvalCase,
    *,
    mode: EvalMode,
    limit: int,
    model_name: str,
    model_command: str | None,
    model_timeout: float,
) -> KnowledgeEvalRun:
    start = time.perf_counter()
    retrieval = kb.search(case.question, limit=limit, method="auto") if mode == "with_kb" else []
    model_answer = _model_answer(
        case,
        mode=mode,
        model_name=model_name,
        retrieval=retrieval,
        model_command=model_command,
        model_timeout=model_timeout,
    )
    selection = choose_closest_answer(
        user_message=case.question,
        llm_answer=model_answer,
        retrieval_results=retrieval,
        allow_knowledge_answer=mode == "with_kb",
    )
    return KnowledgeEvalRun(
        answer=selection.answer,
        source=selection.source,
        correct=_is_correct(case, selection.answer),
        ms=round((time.perf_counter() - start) * 1000, 3),
        query_similarity=round(selection.query_similarity, 3),
        answer_similarity=round(selection.answer_similarity, 3),
        retrieval_count=len(retrieval),
    )


def _model_answer(
    case: KnowledgeEvalCase,
    *,
    mode: EvalMode,
    model_name: str,
    retrieval: list[RetrievalResult],
    model_command: str | None,
    model_timeout: float,
) -> str:
    if not model_command:
        return case.simulated_llm_answer
    if model_command.startswith("simulated:"):
        return _simulated_model_answer(case, model_command)
    if model_command.startswith("llama-cpp:"):
        return _llama_cpp_model_answer(
            case,
            mode=mode,
            retrieval=retrieval,
            model_command=model_command,
        )
    if model_command.startswith("llama-cpp-hf:"):
        return _llama_cpp_hf_model_answer(
            case,
            mode=mode,
            retrieval=retrieval,
            model_command=model_command,
        )

    payload = {
        "model": model_name,
        "mode": mode,
        "case": asdict(case),
        "question": case.question,
        "retrieval": [result.__dict__ for result in retrieval],
    }
    completed = subprocess.run(
        model_command,
        input=json.dumps(payload, ensure_ascii=False),
        text=True,
        shell=True,
        capture_output=True,
        timeout=model_timeout,
        check=False,
    )
    if completed.returncode != 0:
        return case.simulated_llm_answer
    return completed.stdout.strip() or case.simulated_llm_answer


def _llama_cpp_model_answer(
    case: KnowledgeEvalCase,
    *,
    mode: EvalMode,
    retrieval: list[RetrievalResult],
    model_command: str,
) -> str:
    from llama_cpp import Llama

    model_path = model_command.partition(":")[2].strip()
    if not model_path:
        return case.simulated_llm_answer

    _setup_llama_cpp_benchmark_runtime()
    n_gpu_layers, n_ctx, max_tokens, temperature = _llama_cpp_benchmark_options()
    cache_key = ("path", str(Path(model_path).expanduser()), n_gpu_layers, n_ctx)
    llm = _LLAMA_CPP_CACHE.get(cache_key)
    if llm is None:
        llm = Llama(
            model_path=cache_key[1],
            n_gpu_layers=n_gpu_layers,
            n_ctx=n_ctx,
            verbose=False,
        )
        _LLAMA_CPP_CACHE[cache_key] = llm

    return _llama_cpp_chat_answer(
        llm,
        case,
        mode=mode,
        retrieval=retrieval,
        max_tokens=max_tokens,
        temperature=temperature,
    )


def _llama_cpp_hf_model_answer(
    case: KnowledgeEvalCase,
    *,
    mode: EvalMode,
    retrieval: list[RetrievalResult],
    model_command: str,
) -> str:
    from llama_cpp import Llama

    repo_id, filename = parse_llama_cpp_hf_command(model_command)
    _setup_llama_cpp_benchmark_runtime()
    n_gpu_layers, n_ctx, max_tokens, temperature = _llama_cpp_benchmark_options()
    cache_key = ("hf", f"{repo_id}:{filename}", n_gpu_layers, n_ctx)
    llm = _LLAMA_CPP_CACHE.get(cache_key)
    if llm is None:
        llm = Llama.from_pretrained(
            repo_id=repo_id,
            filename=filename,
            n_gpu_layers=n_gpu_layers,
            n_ctx=n_ctx,
            verbose=False,
        )
        _LLAMA_CPP_CACHE[cache_key] = llm

    return _llama_cpp_chat_answer(
        llm,
        case,
        mode=mode,
        retrieval=retrieval,
        max_tokens=max_tokens,
        temperature=temperature,
    )


def parse_llama_cpp_hf_command(model_command: str) -> tuple[str, str]:
    """Parse llama-cpp Hugging Face benchmark commands.

    Command format: llama-cpp-hf:repo_id:filename
    """

    payload = model_command.removeprefix("llama-cpp-hf:")
    repo_id, separator, filename = payload.partition(":")
    if not separator or not repo_id.strip() or not filename.strip():
        raise ValueError(
            "llama-cpp-hf commands must use "
            "llama-cpp-hf:repo_id:filename, for example "
            "llama-cpp-hf:unsloth/gemma-4-E4B-it-GGUF:"
            "gemma-4-E4B-it-Q4_K_S.gguf"
        )
    return repo_id.strip(), filename.strip()


def _setup_llama_cpp_benchmark_runtime() -> None:
    import llama_cpp.llama_cpp as llama_cpp_backend

    from voice_stack_common.platform_setup import setup_cuda

    setup_cuda()
    require_gpu = os.environ.get("SOP_ROBOT_LLM_BENCH_REQUIRE_GPU", "1").casefold()
    if require_gpu in {"0", "false", "no", "off"}:
        return
    if not llama_cpp_backend.llama_supports_gpu_offload():
        raise RuntimeError(
            "llama-cpp-python was built without CUDA GPU offload support. "
            "Run `pixi run install-llama-cuda`, or set "
            "SOP_ROBOT_LLM_BENCH_REQUIRE_GPU=0 to allow CPU benchmarking."
        )


def _llama_cpp_benchmark_options() -> tuple[int, int, int, float]:
    n_gpu_layers = int(os.environ.get("SOP_ROBOT_LLM_BENCH_N_GPU_LAYERS", "-1"))
    n_ctx = int(os.environ.get("SOP_ROBOT_LLM_BENCH_N_CTX", "2048"))
    max_tokens = int(os.environ.get("SOP_ROBOT_LLM_BENCH_MAX_TOKENS", "256"))
    temperature = float(os.environ.get("SOP_ROBOT_LLM_BENCH_TEMPERATURE", "0.1"))
    return n_gpu_layers, n_ctx, max_tokens, temperature


def _llama_cpp_chat_answer(
    llm: Any,
    case: KnowledgeEvalCase,
    *,
    mode: EvalMode,
    retrieval: list[RetrievalResult],
    max_tokens: int,
    temperature: float,
) -> str:
    system_prompt = (
        "You are a Finnish-speaking voice assistant. Vastaa aina suomeksi, "
        "lyhyesti ja selkeasti."
    )
    if mode == "with_kb" and retrieval:
        system_prompt = (
            f"{system_prompt}\n\n"
            "Local knowledge base candidates:\n"
            f"{format_retrieval_context(retrieval)}\n\n"
            "Use the local knowledge base candidates only when they directly "
            "answer the user's question."
        )

    response: Any = llm.create_chat_completion(
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": case.question},
        ],
        max_tokens=max_tokens,
        temperature=temperature,
        stream=False,
    )
    return str(response["choices"][0]["message"].get("content") or "").strip()


def _simulated_model_answer(case: KnowledgeEvalCase, model_command: str) -> str:
    model = model_command.partition(":")[2].strip().casefold()
    if model in {"oracle", "perfect"}:
        return case.expected_answer or case.simulated_llm_answer
    if model in {"weak", "baseline", "simulated"}:
        return case.simulated_llm_answer
    if model == "echo":
        return case.question
    if model in {"mixed", "exact-only"}:
        if case.kind == "exact" and case.expected_answer:
            return case.expected_answer
        return case.simulated_llm_answer
    if model in {"wrong", "negative"}:
        return case.negative_answer or case.simulated_llm_answer
    return case.simulated_llm_answer


def _is_correct(case: KnowledgeEvalCase, answer: str) -> bool:
    if not case.expected_answer:
        return answer.strip() == case.simulated_llm_answer
    expected_tokens = set(_tokens(case.expected_answer))
    answer_tokens = set(_tokens(answer))
    if not expected_tokens or not answer_tokens:
        return normalize_question(answer) == normalize_question(case.expected_answer)
    overlap = len(expected_tokens & answer_tokens) / len(expected_tokens)
    return overlap >= 0.64


def _different_answer(entries: list[RetrievalResult], index: int) -> str:
    source = entries[index]
    best_answer = ""
    best_overlap = 1.0
    for offset in range(1, len(entries)):
        candidate = entries[(index + offset) % len(entries)]
        if normalize_question(candidate.answer) == normalize_question(source.answer):
            continue
        overlap = _answer_overlap(source.answer, candidate.answer)
        if overlap < best_overlap:
            best_overlap = overlap
            best_answer = candidate.answer
    return best_answer or "En tiedä varmasti."


def _similar_question(question: str, index: int) -> str:
    cleaned = question.strip().rstrip("?.! ")
    templates = [
        "Voisitko vastata tähän: {question}?",
        "Kerro lyhyesti, {lower_question}?",
        "Haluaisin tietää: {question}?",
        "{question}, voitko selittää?",
    ]
    template = templates[index % len(templates)]
    return template.format(question=cleaned, lower_question=cleaned[:1].lower() + cleaned[1:])


def _very_different_question(index: int) -> str:
    questions = [
        "Mikä sää on huomenna Helsingissä?",
        "Kuinka monta kahvikuppia keittiössä on juuri nyt?",
        "Milloin seuraava paikallisjuna lähtee asemalta?",
        "Mikä on tämän testin satunnainen tunnussana?",
    ]
    return questions[index % len(questions)]


def _tokens(text: str) -> list[str]:
    return _TOKEN_RE.findall(normalize_question(text))


def _answer_overlap(left: str, right: str) -> float:
    left_tokens = set(_tokens(left))
    right_tokens = set(_tokens(right))
    if not left_tokens or not right_tokens:
        return 0.0
    return len(left_tokens & right_tokens) / min(len(left_tokens), len(right_tokens))


def _ratio(value: int, total: int) -> float:
    return round(value / total, 4) if total else 0.0


def _mean(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    return round(sum(materialized) / len(materialized), 3)
