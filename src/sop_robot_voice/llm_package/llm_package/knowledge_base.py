"""SQLite-backed local question-answer knowledge base for LLM responses."""

from __future__ import annotations

import argparse
import csv
import io
import json
import os
import re
import sqlite3
import tempfile
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Literal

SUPPORTED_SUFFIXES = {".json", ".jsonl", ".txt", ".yaml", ".yml"}
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[1] / "testing" / "fixtures" / "data"
DEFAULT_DB_FILENAME = "sop_robot_llm_knowledge_base.sqlite3"

SearchMethod = Literal["auto", "exact", "fts", "like"]

_WHITESPACE_RE = re.compile(r"\s+")
_TOKEN_RE = re.compile(r"[\w]+", re.UNICODE)
_QA_LINE_RE = re.compile(
    r"^(?P<label>q|question|kysymys|a|answer|vastaus)\s*[:\-]\s*(?P<text>.+)$",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class KnowledgeEntry:
    """A question-answer pair parsed from a local data source."""

    question: str
    answer: str
    source_path: str = ""
    source_format: str = ""
    category: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RetrievalResult:
    """A retrieved answer candidate."""

    id: int
    question: str
    answer: str
    source_path: str
    source_format: str
    category: str | None
    score: float
    method: str


def default_db_path() -> Path:
    """Return the default on-disk SQLite path for local interactive use."""

    configured = os.environ.get("SOP_ROBOT_LLM_KB_PATH")
    if configured:
        return Path(configured).expanduser()
    return Path(tempfile.gettempdir()) / DEFAULT_DB_FILENAME


def normalize_question(text: str) -> str:
    """Normalize user and corpus questions for deterministic exact matching."""

    normalized = unicodedata.normalize("NFKC", str(text))
    normalized = _WHITESPACE_RE.sub(" ", normalized.casefold()).strip()
    return normalized.rstrip("?!。！？. ").strip()


def format_retrieval_context(results: Iterable[RetrievalResult]) -> str:
    """Format retrieval results as compact context for an LLM prompt."""

    lines: list[str] = []
    for index, result in enumerate(results, start=1):
        lines.append(f"{index}. Q: {result.question}\n   A: {result.answer}")
    return "\n".join(lines)


class KnowledgeBase:
    """Small SQLite store for exact and lexical QA retrieval."""

    def __init__(self, db_path: str | Path = ":memory:") -> None:
        self.db_path = str(db_path)
        if self.db_path not in {":memory:", ""} and not self.db_path.startswith("file:"):
            Path(self.db_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(self.db_path)
        self._conn.row_factory = sqlite3.Row
        self._fts_available = False
        self._create_schema()

    @property
    def fts_available(self) -> bool:
        return self._fts_available

    def close(self) -> None:
        self._conn.close()

    def __enter__(self) -> "KnowledgeBase":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        del exc_type, exc, traceback
        self.close()

    def _create_schema(self) -> None:
        self._conn.execute(
            """
            CREATE TABLE IF NOT EXISTS qa_pairs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                question TEXT NOT NULL,
                normalized_question TEXT NOT NULL,
                answer TEXT NOT NULL,
                source_path TEXT NOT NULL DEFAULT '',
                source_format TEXT NOT NULL DEFAULT '',
                category TEXT,
                metadata_json TEXT NOT NULL DEFAULT '{}'
            )
            """
        )
        self._conn.execute(
            """
            CREATE UNIQUE INDEX IF NOT EXISTS uq_qa_pairs_source_question_answer
            ON qa_pairs(source_path, normalized_question, answer)
            """
        )
        self._conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_qa_pairs_normalized_question
            ON qa_pairs(normalized_question)
            """
        )
        self._conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_qa_pairs_source_path
            ON qa_pairs(source_path)
            """
        )
        self._create_fts_schema()
        self._conn.commit()

    def _create_fts_schema(self) -> None:
        try:
            self._conn.execute(
                """
                CREATE VIRTUAL TABLE IF NOT EXISTS qa_pairs_fts
                USING fts5(
                    question,
                    answer,
                    content='qa_pairs',
                    content_rowid='id'
                )
                """
            )
            self._conn.execute(
                """
                CREATE TRIGGER IF NOT EXISTS qa_pairs_ai
                AFTER INSERT ON qa_pairs BEGIN
                    INSERT INTO qa_pairs_fts(rowid, question, answer)
                    VALUES (new.id, new.question, new.answer);
                END
                """
            )
            self._conn.execute(
                """
                CREATE TRIGGER IF NOT EXISTS qa_pairs_ad
                AFTER DELETE ON qa_pairs BEGIN
                    INSERT INTO qa_pairs_fts(qa_pairs_fts, rowid, question, answer)
                    VALUES('delete', old.id, old.question, old.answer);
                END
                """
            )
            self._conn.execute(
                """
                CREATE TRIGGER IF NOT EXISTS qa_pairs_au
                AFTER UPDATE ON qa_pairs BEGIN
                    INSERT INTO qa_pairs_fts(qa_pairs_fts, rowid, question, answer)
                    VALUES('delete', old.id, old.question, old.answer);
                    INSERT INTO qa_pairs_fts(rowid, question, answer)
                    VALUES (new.id, new.question, new.answer);
                END
                """
            )
            self._conn.execute("INSERT INTO qa_pairs_fts(qa_pairs_fts) VALUES('rebuild')")
            self._fts_available = True
        except sqlite3.OperationalError:
            self._fts_available = False

    def clear(self) -> None:
        """Remove all indexed data while preserving a valid schema."""

        self._conn.execute("DROP TRIGGER IF EXISTS qa_pairs_ai")
        self._conn.execute("DROP TABLE IF EXISTS qa_pairs_fts")
        self._conn.execute("DROP TABLE IF EXISTS qa_pairs")
        self._conn.commit()
        self._create_schema()

    def count(self) -> int:
        row = self._conn.execute("SELECT COUNT(*) AS count FROM qa_pairs").fetchone()
        return int(row["count"])

    def insert_entries(self, entries: Iterable[KnowledgeEntry]) -> int:
        inserted = 0
        with self._conn:
            for entry in entries:
                question = entry.question.strip()
                answer = entry.answer.strip()
                if not question or not answer:
                    continue
                metadata_json = json.dumps(entry.metadata, ensure_ascii=False, sort_keys=True)
                cursor = self._conn.execute(
                    """
                    INSERT OR IGNORE INTO qa_pairs (
                        question,
                        normalized_question,
                        answer,
                        source_path,
                        source_format,
                        category,
                        metadata_json
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        question,
                        normalize_question(question),
                        answer,
                        entry.source_path,
                        entry.source_format,
                        entry.category,
                        metadata_json,
                    ),
                )
                inserted += cursor.rowcount
        return inserted

    def learn_answer(
        self,
        question: str,
        answer: str,
        *,
        source_path: str = "runtime:correction",
        source_format: str = "learned",
        category: str | None = "learned",
        metadata: dict[str, Any] | None = None,
    ) -> bool:
        """Insert or replace the answer for a question learned from correction data."""

        clean_question = question.strip()
        clean_answer = answer.strip()
        if not clean_question or not clean_answer:
            return False

        normalized = normalize_question(clean_question)
        metadata_json = json.dumps(metadata or {}, ensure_ascii=False, sort_keys=True)
        with self._conn:
            row = self._conn.execute(
                """
                SELECT *
                FROM qa_pairs
                WHERE normalized_question = ?
                ORDER BY
                    CASE source_format
                        WHEN 'learned' THEN 0
                        WHEN 'correction' THEN 1
                        ELSE 2
                    END,
                    id ASC
                LIMIT 1
                """,
                (normalized,),
            ).fetchone()
            if row is None:
                cursor = self._conn.execute(
                    """
                    INSERT INTO qa_pairs (
                        question,
                        normalized_question,
                        answer,
                        source_path,
                        source_format,
                        category,
                        metadata_json
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        clean_question,
                        normalized,
                        clean_answer,
                        source_path,
                        source_format,
                        category,
                        metadata_json,
                    ),
                )
                return cursor.rowcount > 0

            existing_id = int(row["id"])
            changed = (
                str(row["question"]) != clean_question
                or str(row["answer"]) != clean_answer
                or str(row["source_path"]) != source_path
                or str(row["source_format"]) != source_format
                or row["category"] != category
                or str(row["metadata_json"]) != metadata_json
            )
            self._conn.execute(
                """
                UPDATE qa_pairs
                SET question = ?,
                    answer = ?,
                    source_path = ?,
                    source_format = ?,
                    category = ?,
                    metadata_json = ?
                WHERE id = ?
                """,
                (
                    clean_question,
                    clean_answer,
                    source_path,
                    source_format,
                    category,
                    metadata_json,
                    existing_id,
                ),
            )
            self._conn.execute(
                """
                DELETE FROM qa_pairs
                WHERE normalized_question = ?
                  AND id != ?
                """,
                (normalized, existing_id),
            )
            return changed

    def ingest_path(self, path: str | Path) -> int:
        """Ingest a single supported file or a directory of supported files."""

        source = Path(path).expanduser()
        if source.is_dir():
            return self.ingest_directory(source)
        return self.insert_entries(load_entries(source))

    def ingest_directory(self, data_dir: str | Path) -> int:
        root = Path(data_dir).expanduser()
        total = 0
        for path in sorted(root.rglob("*")):
            if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES:
                total += self.ingest_path(path)
        return total

    def exact_match(self, question: str) -> RetrievalResult | None:
        row = self._conn.execute(
            """
            SELECT *
            FROM qa_pairs
            WHERE normalized_question = ?
            ORDER BY
                CASE source_format
                    WHEN 'learned' THEN 0
                    WHEN 'correction' THEN 1
                    ELSE 2
                END,
                id ASC
            LIMIT 1
            """,
            (normalize_question(question),),
        ).fetchone()
        if row is None:
            return None
        return _row_to_result(row, score=1.0, method="exact")

    def direct_answer(self, question: str) -> str | None:
        result = self.exact_match(question)
        return result.answer if result is not None else None

    def search(
        self,
        query: str,
        *,
        limit: int = 5,
        method: SearchMethod = "auto",
    ) -> list[RetrievalResult]:
        """Retrieve answer candidates with exact, FTS, or LIKE matching."""

        if limit <= 0:
            return []
        if method == "exact":
            result = self.exact_match(query)
            return [result] if result is not None else []
        if method == "fts":
            return self._search_fts(query, limit=limit)
        if method == "like":
            return self._search_like(query, limit=limit)
        if method != "auto":
            raise ValueError(f"Unsupported knowledge base search method: {method}")

        result = self.exact_match(query)
        if result is not None:
            return [result]

        results = self._search_fts(query, limit=limit)
        if results:
            return results
        return self._search_like(query, limit=limit)

    def questions(self, *, limit: int | None = None) -> list[str]:
        sql = "SELECT question FROM qa_pairs ORDER BY id ASC"
        params: tuple[int, ...] = ()
        if limit is not None:
            sql += " LIMIT ?"
            params = (limit,)
        return [str(row["question"]) for row in self._conn.execute(sql, params).fetchall()]

    def entries(self, *, limit: int | None = None) -> list[RetrievalResult]:
        sql = "SELECT * FROM qa_pairs ORDER BY id ASC"
        params: tuple[int, ...] = ()
        if limit is not None:
            sql += " LIMIT ?"
            params = (limit,)
        rows = self._conn.execute(sql, params).fetchall()
        return [_row_to_result(row, score=1.0, method="entry") for row in rows]

    def _search_fts(self, query: str, *, limit: int) -> list[RetrievalResult]:
        if not self._fts_available:
            return []
        fts_query = _to_fts_query(query)
        if not fts_query:
            return []
        try:
            rows = self._conn.execute(
                """
                SELECT qa_pairs.*, bm25(qa_pairs_fts) AS rank
                FROM qa_pairs_fts
                JOIN qa_pairs ON qa_pairs_fts.rowid = qa_pairs.id
                WHERE qa_pairs_fts MATCH ?
                ORDER BY rank ASC, qa_pairs.id ASC
                LIMIT ?
                """,
                (fts_query, limit),
            ).fetchall()
        except sqlite3.OperationalError:
            return []
        return [_row_to_result(row, score=float(row["rank"]), method="fts") for row in rows]

    def _search_like(self, query: str, *, limit: int) -> list[RetrievalResult]:
        terms = _tokens(query)
        if not terms:
            return []

        clauses: list[str] = []
        params: list[str] = []
        for term in terms[:8]:
            pattern = f"%{_escape_like(term)}%"
            clauses.append(
                """
                (
                    normalized_question LIKE ? ESCAPE '\\'
                    OR question LIKE ? ESCAPE '\\'
                    OR answer LIKE ? ESCAPE '\\'
                )
                """
            )
            params.extend([pattern, pattern, pattern])

        rows = self._conn.execute(
            f"""
            SELECT *
            FROM qa_pairs
            WHERE {" OR ".join(clauses)}
            ORDER BY id ASC
            LIMIT 500
            """,
            tuple(params),
        ).fetchall()
        scored = [_row_to_result(row, score=_like_score(query, row), method="like") for row in rows]
        scored.sort(key=lambda result: (-result.score, result.id))
        return scored[:limit]


def build_knowledge_base(
    *,
    db_path: str | Path = ":memory:",
    data_dir: str | Path = DEFAULT_DATA_DIR,
    recreate: bool = False,
) -> KnowledgeBase:
    """Create a KB and ingest a directory of local QA files."""

    kb = KnowledgeBase(db_path)
    if recreate:
        kb.clear()
    kb.ingest_directory(data_dir)
    return kb


def load_entries(path: str | Path) -> list[KnowledgeEntry]:
    """Parse a supported local QA file into normalized entries."""

    source = Path(path).expanduser()
    suffix = source.suffix.lower()
    if suffix not in SUPPORTED_SUFFIXES:
        return []

    text = _read_text(source)
    if suffix in {".yaml", ".yml"}:
        return _load_yaml_entries(text, source)
    if suffix == ".json":
        data = json.loads(text)
        return list(_entries_from_data(data, source, suffix.lstrip(".")))
    if suffix == ".jsonl":
        return list(_load_jsonl_entries(text, source))
    if suffix == ".txt":
        return list(_load_txt_entries(text, source))
    return []


def _load_yaml_entries(text: str, source: Path) -> list[KnowledgeEntry]:
    data: Any
    try:
        import yaml  # type: ignore[import-untyped]
    except Exception:
        data = _parse_simple_chatterbot_yaml(text)
    else:
        try:
            data = yaml.safe_load(text)
        except Exception:
            data = _parse_simple_chatterbot_yaml(text)
    return list(_entries_from_data(data, source, source.suffix.lstrip(".")))


def _read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "ISO-8859-1"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    return data.decode("utf-8", errors="replace")


def _load_jsonl_entries(text: str, source: Path) -> Iterator[KnowledgeEntry]:
    for line_number, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        data = json.loads(stripped)
        yield from _entries_from_data(
            data,
            source,
            "jsonl",
            metadata={"line": line_number},
        )


def _load_txt_entries(text: str, source: Path) -> Iterator[KnowledgeEntry]:
    delimited_entries = list(_load_delimited_txt_entries(text, source))
    if delimited_entries:
        yield from delimited_entries
        return

    labeled_entries = list(_load_labeled_txt_entries(text, source))
    if labeled_entries:
        yield from labeled_entries
        return

    yield from _load_alternating_txt_entries(text, source)


def _load_delimited_txt_entries(text: str, source: Path) -> Iterator[KnowledgeEntry]:
    first_line = next((line for line in text.splitlines() if line.strip()), "")
    if not first_line:
        return

    delimiter = "\t" if "\t" in first_line else "," if "," in first_line else None
    if delimiter is None:
        return

    reader = csv.DictReader(io.StringIO(text), delimiter=delimiter)
    if reader.fieldnames is None:
        return
    lower_fields = {field.casefold(): field for field in reader.fieldnames}
    question_field = _first_existing_field(lower_fields, ["question", "query", "prompt", "kysymys"])
    answer_field = _first_existing_field(
        lower_fields, ["answer", "response", "completion", "vastaus"]
    )
    if question_field is None or answer_field is None:
        return

    for row_number, row in enumerate(reader, start=2):
        yield _make_entry(
            row.get(question_field, ""),
            row.get(answer_field, ""),
            source,
            "txt",
            metadata={"row": row_number},
        )


def _load_labeled_txt_entries(text: str, source: Path) -> Iterator[KnowledgeEntry]:
    current_question: str | None = None
    for line_number, line in enumerate(text.splitlines(), start=1):
        match = _QA_LINE_RE.match(line.strip())
        if match is None:
            continue
        label = match.group("label").casefold()
        value = match.group("text").strip()
        if label in {"q", "question", "kysymys"}:
            current_question = value
            continue
        if current_question and label in {"a", "answer", "vastaus"}:
            yield _make_entry(
                current_question,
                value,
                source,
                "txt",
                metadata={"line": line_number},
            )
            current_question = None


def _load_alternating_txt_entries(text: str, source: Path) -> Iterator[KnowledgeEntry]:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if any(_QA_LINE_RE.match(line) for line in lines):
        return
    if not lines or len(lines) % 2 != 0:
        return
    for index in range(0, len(lines), 2):
        yield _make_entry(
            lines[index],
            lines[index + 1],
            source,
            "txt",
            metadata={"line": index + 1},
        )


def _entries_from_data(
    data: Any,
    source: Path,
    source_format: str,
    *,
    category: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> Iterator[KnowledgeEntry]:
    metadata = metadata or {}
    if isinstance(data, dict):
        local_category = _category_text(data.get("categories")) or category

        if "conversations" in data:
            yield from _entries_from_conversations(
                data["conversations"],
                source,
                source_format,
                local_category,
                metadata,
            )

        if "annotations" in data:
            yield from _entries_from_annotations(
                data,
                source,
                source_format,
                local_category,
                metadata,
            )

        yield from _entries_from_mapping(
            data,
            source,
            source_format,
            local_category,
            metadata,
        )

        for nested_key in ("data", "items", "records", "paragraphs", "qas", "qaPairs"):
            nested = data.get(nested_key)
            if nested is not None:
                yield from _entries_from_data(
                    nested,
                    source,
                    source_format,
                    category=local_category,
                    metadata=metadata,
                )
        return

    if isinstance(data, list):
        if _is_text_sequence(data):
            yield from _entries_from_conversations(
                [data],
                source,
                source_format,
                category,
                metadata,
            )
            return
        for item in data:
            yield from _entries_from_data(
                item,
                source,
                source_format,
                category=category,
                metadata=metadata,
            )


def _entries_from_conversations(
    conversations: Any,
    source: Path,
    source_format: str,
    category: str | None,
    metadata: dict[str, Any],
) -> Iterator[KnowledgeEntry]:
    if not isinstance(conversations, list):
        return
    for conversation_index, conversation in enumerate(conversations):
        if isinstance(conversation, dict):
            yield from _entries_from_mapping(
                conversation,
                source,
                source_format,
                category,
                {**metadata, "conversation": conversation_index},
            )
            continue
        if not isinstance(conversation, list):
            continue
        texts = [str(item).strip() for item in conversation if _is_scalar_text(item)]
        for offset, (question, answer) in enumerate(zip(texts, texts[1:], strict=False)):
            yield _make_entry(
                question,
                answer,
                source,
                source_format,
                category=category,
                metadata={**metadata, "conversation": conversation_index, "turn": offset},
            )


def _entries_from_annotations(
    data: dict[str, Any],
    source: Path,
    source_format: str,
    category: str | None,
    metadata: dict[str, Any],
) -> Iterator[KnowledgeEntry]:
    parent_question = _first_text(data, ["question", "query", "prompt"])
    annotations = data.get("annotations")
    if not isinstance(annotations, list):
        return

    for annotation_index, annotation in enumerate(annotations):
        if not isinstance(annotation, dict):
            continue
        annotation_metadata = {**metadata, "annotation": annotation_index}
        qa_pairs = annotation.get("qaPairs")
        if isinstance(qa_pairs, list):
            for pair_index, pair in enumerate(qa_pairs):
                if not isinstance(pair, dict):
                    continue
                question = _first_text(pair, ["question", "query", "prompt"])
                answers = _answers_from_mapping(pair)
                for answer_index, answer in enumerate(answers):
                    yield _make_entry(
                        question,
                        answer,
                        source,
                        source_format,
                        category=category,
                        metadata={
                            **annotation_metadata,
                            "qa_pair": pair_index,
                            "answer": answer_index,
                        },
                    )
            continue

        if parent_question:
            for answer_index, answer in enumerate(_answers_from_mapping(annotation)):
                yield _make_entry(
                    parent_question,
                    answer,
                    source,
                    source_format,
                    category=category,
                    metadata={**annotation_metadata, "answer": answer_index},
                )


def _entries_from_mapping(
    data: dict[str, Any],
    source: Path,
    source_format: str,
    category: str | None,
    metadata: dict[str, Any],
) -> Iterator[KnowledgeEntry]:
    question = _first_text(data, ["question", "query", "prompt"])
    if question is None:
        return
    for answer_index, answer in enumerate(_answers_from_mapping(data)):
        yield _make_entry(
            question,
            answer,
            source,
            source_format,
            category=category,
            metadata={**metadata, "answer": answer_index},
        )


def _answers_from_mapping(data: dict[str, Any]) -> list[str]:
    for key in ("answer", "answers", "response", "responses", "completion"):
        if key in data:
            return _text_values(data[key])
    return []


def _text_values(value: Any) -> list[str]:
    if _is_scalar_text(value):
        return [str(value).strip()]
    if isinstance(value, dict):
        text = _first_text(value, ["text", "answer", "response", "value"])
        return [text] if text else []
    if isinstance(value, list):
        values: list[str] = []
        for item in value:
            values.extend(_text_values(item))
        return values
    return []


def _make_entry(
    question: Any,
    answer: Any,
    source: Path,
    source_format: str,
    *,
    category: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> KnowledgeEntry:
    return KnowledgeEntry(
        question="" if question is None else str(question).strip(),
        answer="" if answer is None else str(answer).strip(),
        source_path=str(source),
        source_format=source_format,
        category=category,
        metadata=metadata or {},
    )


def _parse_simple_chatterbot_yaml(text: str) -> dict[str, Any]:
    categories: list[str] = []
    conversations: list[list[str]] = []
    section: str | None = None
    current_conversation: list[str] | None = None

    def finish_conversation() -> None:
        nonlocal current_conversation
        if current_conversation is not None:
            conversations.append(current_conversation)
        current_conversation = None

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "categories:":
            finish_conversation()
            section = "categories"
            continue
        if stripped == "conversations:":
            finish_conversation()
            section = "conversations"
            continue

        if section == "categories" and stripped.startswith("- "):
            categories.append(stripped[2:].strip())
            continue

        if section != "conversations":
            continue
        if stripped.startswith("- - "):
            finish_conversation()
            current_conversation = [stripped[4:].strip()]
            continue
        if raw_line.startswith("  - ") and current_conversation is not None:
            current_conversation.append(stripped[2:].strip())
            continue
        if current_conversation:
            current_conversation[-1] = f"{current_conversation[-1]} {stripped}".strip()

    finish_conversation()
    return {"categories": categories, "conversations": conversations}


def _row_to_result(sql_row: sqlite3.Row, *, score: float, method: str) -> RetrievalResult:
    return RetrievalResult(
        id=int(sql_row["id"]),
        question=str(sql_row["question"]),
        answer=str(sql_row["answer"]),
        source_path=str(sql_row["source_path"]),
        source_format=str(sql_row["source_format"]),
        category=sql_row["category"],
        score=score,
        method=method,
    )


def _to_fts_query(query: str) -> str:
    return " OR ".join(f'"{token}"' for token in _tokens(query)[:12])


def _tokens(text: str) -> list[str]:
    return _TOKEN_RE.findall(normalize_question(text))


def _like_score(query: str, row: sqlite3.Row) -> float:
    terms = _tokens(query)
    if not terms:
        return 0.0
    normalized_question = normalize_question(row["question"])
    normalized_answer = normalize_question(row["answer"])
    normalized_query = normalize_question(query)
    score = 0.0
    if normalized_query and normalized_query in normalized_question:
        score += 5.0
    for term in terms:
        if term in normalized_question:
            score += 1.0
        if term in normalized_answer:
            score += 0.25
    return score / len(terms)


def _escape_like(value: str) -> str:
    return value.replace("\\", "\\\\").replace("%", "\\%").replace("_", "\\_")


def _first_existing_field(fields: dict[str, str], candidates: list[str]) -> str | None:
    for candidate in candidates:
        field = fields.get(candidate)
        if field is not None:
            return field
    return None


def _first_text(data: dict[str, Any], keys: list[str]) -> str | None:
    for key in keys:
        if key in data and _is_scalar_text(data[key]):
            value = str(data[key]).strip()
            if value:
                return value
    return None


def _category_text(value: Any) -> str | None:
    values = _text_values(value)
    return ", ".join(values) if values else None


def _is_scalar_text(value: Any) -> bool:
    return isinstance(value, (str, int, float)) and str(value).strip() != ""


def _is_text_sequence(value: list[Any]) -> bool:
    return bool(value) and all(_is_scalar_text(item) for item in value)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build and query the local LLM QA KB.")
    parser.add_argument("--data-dir", default=str(DEFAULT_DATA_DIR))
    parser.add_argument("--db", default=str(default_db_path()))
    parser.add_argument("--recreate", action="store_true")
    parser.add_argument("--query", action="append", default=[])
    parser.add_argument("--method", choices=["auto", "exact", "fts", "like"], default="auto")
    parser.add_argument("--limit", type=int, default=5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    with build_knowledge_base(
        db_path=args.db,
        data_dir=args.data_dir,
        recreate=args.recreate,
    ) as kb:
        output: dict[str, Any] = {
            "db": args.db,
            "data_dir": args.data_dir,
            "count": kb.count(),
            "fts_available": kb.fts_available,
            "queries": [],
        }
        for query in args.query:
            output["queries"].append(
                {
                    "query": query,
                    "results": [
                        result.__dict__
                        for result in kb.search(query, limit=args.limit, method=args.method)
                    ],
                }
            )
        print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
