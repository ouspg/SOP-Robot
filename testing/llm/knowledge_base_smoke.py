"""Deterministic smoke harness for the local LLM SQLite knowledge base."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
LLM_SRC = REPO_ROOT / "src" / "llm_package"
if str(LLM_SRC) not in sys.path:
    sys.path.insert(0, str(LLM_SRC))

from llm_package.knowledge_base import DEFAULT_DATA_DIR, KnowledgeBase


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", default=str(DEFAULT_DATA_DIR))
    parser.add_argument("--db", default=":memory:")
    parser.add_argument("--query", action="append", default=[])
    parser.add_argument("--limit", type=int, default=3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with KnowledgeBase(args.db) as kb:
        inserted = kb.ingest_directory(args.data_dir)
        queries = args.query or kb.questions(limit=args.limit)
        payload = {
            "data_dir": args.data_dir,
            "db": args.db,
            "inserted": inserted,
            "count": kb.count(),
            "fts_available": kb.fts_available,
            "queries": [
                {
                    "query": query,
                    "direct_answer": kb.direct_answer(query),
                    "retrieval": [
                        result.__dict__
                        for result in kb.search(query, limit=args.limit, method="auto")
                    ],
                }
                for query in queries
            ],
        }
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
