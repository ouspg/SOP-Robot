"""Send one prompt directly to one LLM model without KB retrieval."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
LLM_SRC = REPO_ROOT / "src" / "llm_package"
if str(LLM_SRC) not in sys.path:
    sys.path.insert(0, str(LLM_SRC))

from llm_package.knowledge_eval import ModelSpec, answer_model_prompt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        help="Model shorthand as NAME=COMMAND, for example ahma=llama-cpp:models/a.gguf",
    )
    parser.add_argument("--model-name", default="model")
    parser.add_argument("--model-command")
    parser.add_argument("--model-timeout", type=float, default=60.0)
    parser.add_argument("--prompt")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--allow-empty", action="store_true")
    parser.add_argument("--output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = model_from_args(args)
    prompt = prompt_from_args(args)

    started = time.perf_counter()
    answer = answer_model_prompt(prompt, model, model_timeout=args.model_timeout)
    elapsed_ms = round((time.perf_counter() - started) * 1000, 3)

    payload: dict[str, Any] = {
        "model": model.name,
        "model_command": model.command,
        "prompt": prompt,
        "answer": answer,
        "elapsed_ms": elapsed_ms,
        "kb_enabled": False,
        "retrieval": False,
        "answer_selection": False,
        "learning": False,
    }

    if args.output:
        output_path = Path(args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        payload["output"] = str(output_path)

    if args.json or args.output:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        print(answer)

    if not answer.strip() and not args.allow_empty:
        print("Model returned an empty answer.", file=sys.stderr)
        return 2
    return 0


def model_from_args(args: argparse.Namespace) -> ModelSpec:
    name = args.model_name
    command = args.model_command
    if args.model:
        parsed_name, separator, parsed_command = args.model.partition("=")
        if not separator or not parsed_name.strip() or not parsed_command.strip():
            raise SystemExit("--model must use NAME=COMMAND")
        name = parsed_name.strip()
        command = parsed_command.strip()
    if not command:
        raise SystemExit("--model or --model-command is required")
    return ModelSpec(name=name, command=command)


def prompt_from_args(args: argparse.Namespace) -> str:
    if args.prompt is not None:
        prompt = args.prompt
    elif not sys.stdin.isatty():
        prompt = sys.stdin.read()
    else:
        raise SystemExit("--prompt or stdin is required")
    prompt = prompt.strip()
    if not prompt:
        raise SystemExit("Prompt is empty")
    return prompt


if __name__ == "__main__":
    raise SystemExit(main())
