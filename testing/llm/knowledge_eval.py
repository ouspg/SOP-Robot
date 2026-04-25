"""Generate and compare LLM knowledge-base evaluation data."""

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

from llm_package.knowledge_base import DEFAULT_DATA_DIR, KnowledgeBase
from llm_package.knowledge_eval import (
    ModelSpec,
    compare_model_runs,
    compare_knowledge_base_runs,
    comparisons_to_json,
    generate_eval_cases,
    model_comparisons_to_json,
    read_cases_jsonl,
    summarize_comparisons,
    summarize_model_comparisons,
    write_cases_jsonl,
)

DEFAULT_OUTPUT = REPO_ROOT / "test-results" / "llm_knowledge_eval.jsonl"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="Generate JSONL evaluation cases")
    add_common_args(generate)
    generate.add_argument("--output", default=str(DEFAULT_OUTPUT))

    compare = subparsers.add_parser("compare", help="Compare without-KB and with-KB answers")
    add_common_args(compare)
    compare.add_argument("--cases-file")
    compare.add_argument("--output")
    compare.add_argument("--model-command")
    compare.add_argument("--model-timeout", type=float, default=60.0)
    compare.add_argument("--summary-only", action="store_true")

    compare_models = subparsers.add_parser(
        "compare-models",
        help="Compare multiple model commands with KB disabled and enabled",
    )
    add_common_args(compare_models)
    compare_models.add_argument("--cases-file")
    compare_models.add_argument("--output")
    compare_models.add_argument(
        "--model",
        action="append",
        default=[],
        metavar="NAME=COMMAND",
        help=(
            "Named model command. Use simulated:weak, simulated:mixed, or "
            "simulated:oracle for deterministic local tests."
        ),
    )
    compare_models.add_argument("--model-timeout", type=float, default=60.0)
    compare_models.add_argument("--summary-only", action="store_true")
    return parser.parse_args()


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--data-dir", default=str(DEFAULT_DATA_DIR))
    parser.add_argument("--db", default=":memory:")
    parser.add_argument("--base-limit", type=int, default=20)
    parser.add_argument("--limit", type=int, default=3)


def main() -> int:
    args = parse_args()
    with KnowledgeBase(args.db) as kb:
        inserted = kb.ingest_directory(args.data_dir)
        if args.command == "generate":
            cases = generate_eval_cases(kb, base_limit=args.base_limit)
            output = write_cases_jsonl(cases, args.output)
            payload = {
                "data_dir": args.data_dir,
                "db": args.db,
                "inserted": inserted,
                "count": kb.count(),
                "cases": len(cases),
                "output": str(output),
            }
            print(json.dumps(payload, indent=2, ensure_ascii=False))
            return 0

        if args.cases_file:
            cases = read_cases_jsonl(args.cases_file)
        else:
            cases = generate_eval_cases(kb, base_limit=args.base_limit)

        if args.command == "compare-models":
            model_results = compare_model_runs(
                kb,
                cases,
                parse_model_specs(args.model),
                limit=args.limit,
                model_timeout=args.model_timeout,
            )
            payload = {
                "data_dir": args.data_dir,
                "db": args.db,
                "inserted": inserted,
                "count": kb.count(),
                "summary": summarize_model_comparisons(model_results),
            }
            if not args.summary_only:
                payload["models"] = model_comparisons_to_json(model_results)
            write_optional_output(payload, args.output)
            print(json.dumps(payload, indent=2, ensure_ascii=False))
            return 0

        comparisons = compare_knowledge_base_runs(
            kb,
            cases,
            limit=args.limit,
            model_command=args.model_command,
            model_timeout=args.model_timeout,
        )
        payload = {
            "data_dir": args.data_dir,
            "db": args.db,
            "inserted": inserted,
            "count": kb.count(),
            "summary": summarize_comparisons(comparisons),
        }
        if not args.summary_only:
            payload["comparisons"] = comparisons_to_json(comparisons)
        write_optional_output(payload, args.output)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0


def parse_model_specs(raw_models: list[str]) -> list[ModelSpec]:
    if not raw_models:
        return [
            ModelSpec(name="simulated_weak", command="simulated:weak"),
            ModelSpec(name="simulated_mixed", command="simulated:mixed"),
            ModelSpec(name="simulated_oracle", command="simulated:oracle"),
        ]

    models: list[ModelSpec] = []
    for raw_model in raw_models:
        name, separator, command = raw_model.partition("=")
        if not separator or not name.strip() or not command.strip():
            raise SystemExit(
                "--model must use NAME=COMMAND, for example "
                "--model gemma4='llama-cpp-hf:unsloth/gemma-4-E4B-it-GGUF:"
                "gemma-4-E4B-it-Q4_K_S.gguf'"
            )
        models.append(ModelSpec(name=name.strip(), command=command.strip()))
    return models


def write_optional_output(payload: dict[str, Any], output_path: str | None) -> None:
    if not output_path:
        return
    output = Path(output_path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    payload["output"] = str(output)


if __name__ == "__main__":
    raise SystemExit(main())
