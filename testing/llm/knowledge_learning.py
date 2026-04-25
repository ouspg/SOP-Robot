"""Run an adaptive KB learning test with Oulu and university correction data."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
LLM_SRC = REPO_ROOT / "src" / "llm_package"
if str(LLM_SRC) not in sys.path:
    sys.path.insert(0, str(LLM_SRC))

from llm_package.knowledge_base import KnowledgeBase, KnowledgeEntry, load_entries
from llm_package.knowledge_eval import (
    KnowledgeEvalCase,
    ModelSpec,
    learning_run_to_json,
    run_learning_eval,
)

DEFAULT_SEED_DATA = REPO_ROOT / "testing" / "fixtures" / "llm" / "oulu_university_seed.yml"
DEFAULT_CORRECTIONS = (
    REPO_ROOT / "testing" / "fixtures" / "llm" / "oulu_university_corrections.yml"
)
DEFAULT_OUTPUT = REPO_ROOT / "test-results" / "llm_knowledge_learning.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-data", default=str(DEFAULT_SEED_DATA))
    parser.add_argument("--corrections", default=str(DEFAULT_CORRECTIONS))
    parser.add_argument("--db", default=":memory:")
    parser.add_argument("--limit", type=int, default=3)
    parser.add_argument("--model-name", default="simulated_weak")
    parser.add_argument("--model-command", default="simulated:weak")
    parser.add_argument("--model-timeout", type=float, default=60.0)
    parser.add_argument("--expect-learned", type=int)
    parser.add_argument("--expect-before-accuracy", type=float)
    parser.add_argument("--expect-after-accuracy", type=float)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = ModelSpec(name=args.model_name, command=args.model_command)
    with KnowledgeBase(args.db) as kb:
        seed_inserted = kb.ingest_path(args.seed_data)
        cases = correction_entries_to_cases(load_entries(args.corrections))
        run = run_learning_eval(
            kb,
            cases,
            model,
            limit=args.limit,
            model_timeout=args.model_timeout,
            learn_source_path=str(Path(args.corrections).expanduser()),
        )
        payload: dict[str, Any] = {
            "seed_data": args.seed_data,
            "corrections": args.corrections,
            "db": args.db,
            "seed_inserted": seed_inserted,
            "case_count": len(cases),
            "model": model.name,
            "summary": run.summary,
            "initial_count": run.initial_count,
            "final_count": run.final_count,
        }
        if not args.summary_only:
            payload["learning_run"] = learning_run_to_json(run)

    validate_expectations(payload["summary"], args)
    output_path = Path(args.output).expanduser() if args.output else DEFAULT_OUTPUT
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    payload["output"] = str(output_path)
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


def validate_expectations(summary: dict[str, Any], args: argparse.Namespace) -> None:
    errors: list[str] = []
    if args.expect_learned is not None and summary["learned"] != args.expect_learned:
        errors.append(
            f"expected learned={args.expect_learned}, got {summary['learned']}"
        )
    if args.expect_before_accuracy is not None and not _near(
        float(summary["before_accuracy"]),
        args.expect_before_accuracy,
    ):
        errors.append(
            "expected before_accuracy="
            f"{args.expect_before_accuracy}, got {summary['before_accuracy']}"
        )
    if args.expect_after_accuracy is not None and not _near(
        float(summary["after_accuracy"]),
        args.expect_after_accuracy,
    ):
        errors.append(
            "expected after_accuracy="
            f"{args.expect_after_accuracy}, got {summary['after_accuracy']}"
        )
    if errors:
        raise SystemExit("; ".join(errors))


def _near(left: float, right: float, *, tolerance: float = 0.0001) -> bool:
    return abs(left - right) <= tolerance


def correction_entries_to_cases(entries: list[KnowledgeEntry]) -> list[KnowledgeEvalCase]:
    cases: list[KnowledgeEvalCase] = []
    for index, entry in enumerate(entries, start=1):
        question = entry.question.strip()
        answer = entry.answer.strip()
        if not question or not answer:
            continue
        cases.append(
            KnowledgeEvalCase(
                id=f"oulu-learning-{index:03d}",
                kind="exact",
                question=question,
                expected_answer=answer,
                simulated_llm_answer="En ole varma oikeasta vastauksesta.",
                source_question=question,
                source_answer=answer,
                negative_answer="En ole varma oikeasta vastauksesta.",
                should_use_knowledge_base=True,
            )
        )
    return cases


if __name__ == "__main__":
    raise SystemExit(main())
