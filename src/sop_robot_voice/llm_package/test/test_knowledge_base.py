from __future__ import annotations

import json

from llm_package.knowledge_base import (
    KnowledgeBase,
    KnowledgeEntry,
    load_entries,
    normalize_question,
)


def test_yaml_ingest_supports_exact_direct_answer(tmp_path):
    data_file = tmp_path / "robot.yml"
    data_file.write_text(
        "\n".join(
            [
                "categories:",
                "- robotics",
                "conversations:",
                "- - What is ROS?",
                "  - Robot Operating System.",
                "- - How many wheels does the base have?",
                "  - Four.",
            ]
        ),
        encoding="utf-8",
    )

    with KnowledgeBase(tmp_path / "kb.sqlite3") as kb:
        inserted = kb.ingest_path(data_file)

        assert inserted == 2
        assert kb.count() == 2
        assert kb.direct_answer(" what is ros ") == "Robot Operating System."
        result = kb.exact_match("What is ROS???")
        assert result is not None
        assert result.category == "robotics"
        assert result.method == "exact"


def test_json_jsonl_and_txt_ingestion_are_idempotent(tmp_path):
    (tmp_path / "faq.json").write_text(
        json.dumps(
            {
                "conversations": [
                    ["How do I charge the battery?", "Use the charging dock."]
                ]
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "qa.jsonl").write_text(
        json.dumps({"query": "Who built the demo?", "answers": ["The SOP team."]})
        + "\n",
        encoding="utf-8",
    )
    (tmp_path / "notes.txt").write_text(
        "Q: Where is the lab?\nA: In the robotics wing.\n",
        encoding="utf-8",
    )

    with KnowledgeBase(":memory:") as kb:
        assert kb.ingest_directory(tmp_path) == 3
        assert kb.ingest_directory(tmp_path) == 0

        assert kb.count() == 3
        assert kb.direct_answer("Who built the demo?") == "The SOP team."
        assert kb.direct_answer("Where is the lab?") == "In the robotics wing."


def test_like_and_fts_retrieval_are_deterministic(tmp_path):
    data_file = tmp_path / "faq.json"
    data_file.write_text(
        json.dumps(
            [
                {
                    "question": "How do I charge the battery?",
                    "answer": "Use the charging dock.",
                },
                {
                    "question": "How do I reset the arm?",
                    "answer": "Use the reset switch.",
                },
            ]
        ),
        encoding="utf-8",
    )

    with KnowledgeBase(":memory:") as kb:
        kb.ingest_path(data_file)

        like_results = kb.search("battery charging", method="like")
        assert like_results
        assert like_results[0].question == "How do I charge the battery?"
        assert like_results[0].method == "like"

        if kb.fts_available:
            fts_results = kb.search("battery", method="fts")
            assert fts_results
            assert fts_results[0].method == "fts"


def test_load_entries_supports_annotation_json(tmp_path):
    data_file = tmp_path / "annotations.json"
    data_file.write_text(
        json.dumps(
            [
                {
                    "question": "When is the lab open?",
                    "annotations": [
                        {"type": "singleAnswer", "answer": ["Weekdays."]},
                        {
                            "type": "multipleQAs",
                            "qaPairs": [
                                {
                                    "question": "When is the lab open on Friday?",
                                    "answer": ["Until noon."],
                                }
                            ],
                        },
                    ],
                }
            ]
        ),
        encoding="utf-8",
    )

    entries = load_entries(data_file)

    assert [entry.question for entry in entries] == [
        "When is the lab open?",
        "When is the lab open on Friday?",
    ]
    assert [entry.answer for entry in entries] == ["Weekdays.", "Until noon."]


def test_learn_answer_replaces_wrong_exact_answer_and_updates_search():
    with KnowledgeBase(":memory:") as kb:
        kb.insert_entries(
            [
                KnowledgeEntry(
                    question="Missä Linnanmaan kampus sijaitsee?",
                    answer="Linnanmaa sijaitsee Tampereella.",
                )
            ]
        )

        learned = kb.learn_answer(
            "Missä Linnanmaan kampus sijaitsee?",
            "Linnanmaan kampus sijaitsee Oulussa.",
        )

        assert learned is True
        assert kb.count() == 1
        assert (
            kb.direct_answer("Missä Linnanmaan kampus sijaitsee?")
            == "Linnanmaan kampus sijaitsee Oulussa."
        )
        results = kb.search("Linnanmaan kampus Oulu", method="auto")
        assert results
        assert results[0].answer == "Linnanmaan kampus sijaitsee Oulussa."


def test_normalize_question_collapses_case_spacing_and_terminal_punctuation():
    assert normalize_question("  What   IS ROS???  ") == "what is ros"
