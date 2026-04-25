from llm_package.knowledge_base import KnowledgeBase
from llm_package.knowledge_eval import (
    KnowledgeEvalCase,
    ModelSpec,
    answer_model_prompt,
    compare_model_runs,
    compare_knowledge_base_runs,
    generate_eval_cases,
    parse_llama_cpp_hf_command,
    read_cases_jsonl,
    run_learning_eval,
    summarize_comparisons,
    summarize_model_comparisons,
    write_cases_jsonl,
)


def build_test_kb(tmp_path):
    data_file = tmp_path / "faq.yml"
    data_file.write_text(
        "\n".join(
            [
                "categories:",
                "- test",
                "conversations:",
                "- - Mikä on ROS?",
                "  - ROS on robottien ohjelmistokehys.",
                "- - Miten akku ladataan?",
                "  - Akku ladataan lataustelakassa.",
                "- - Mitä tekoäly tarkoittaa?",
                "  - Tekoäly tarkoittaa järjestelmiä, jotka voivat oppia.",
            ]
        ),
        encoding="utf-8",
    )
    kb = KnowledgeBase(":memory:")
    kb.ingest_path(data_file)
    return kb


def test_generate_eval_cases_mixes_known_similar_wrong_and_different_questions(tmp_path):
    with build_test_kb(tmp_path) as kb:
        cases = generate_eval_cases(kb, base_limit=2)

    kinds = {case.kind for case in cases}

    assert kinds == {"exact", "similar_question", "mixed_answer", "very_different"}
    assert len(cases) == 8
    assert any(case.question != case.source_question for case in cases)
    assert any(
        case.simulated_llm_answer != case.expected_answer
        for case in cases
        if case.should_use_knowledge_base
    )
    assert any(not case.expected_answer for case in cases)


def test_eval_cases_roundtrip_jsonl(tmp_path):
    with build_test_kb(tmp_path) as kb:
        cases = generate_eval_cases(kb, base_limit=1)

    output = write_cases_jsonl(cases, tmp_path / "cases.jsonl")

    assert read_cases_jsonl(output) == cases


def test_compare_knowledge_base_runs_shows_improvement_with_kb(tmp_path):
    with build_test_kb(tmp_path) as kb:
        cases = generate_eval_cases(kb, base_limit=3)
        comparisons = compare_knowledge_base_runs(kb, cases, limit=3)

    summary = summarize_comparisons(comparisons)

    assert summary["without_kb_correct"] < summary["with_kb_correct"]
    assert summary["improved"] > 0
    assert summary["regressed"] == 0
    assert any(item.without_kb.answer != item.with_kb.answer for item in comparisons)


def test_compare_model_runs_reports_each_model_with_kb_and_without(tmp_path):
    with build_test_kb(tmp_path) as kb:
        cases = generate_eval_cases(kb, base_limit=2)
        model_results = compare_model_runs(
            kb,
            cases,
            [
                ModelSpec("weak", "simulated:weak"),
                ModelSpec("mixed", "simulated:mixed"),
                ModelSpec("oracle", "simulated:oracle"),
            ],
            limit=3,
        )

    summary = summarize_model_comparisons(model_results)

    assert [result.model.name for result in model_results] == ["weak", "mixed", "oracle"]
    assert summary["weak"]["with_kb_correct"] > summary["weak"]["without_kb_correct"]
    assert summary["mixed"]["without_kb_correct"] > summary["weak"]["without_kb_correct"]
    assert summary["oracle"]["without_kb_accuracy"] == 1.0
    assert all(
        "with_kb_mean_ms" in model_summary
        for model_summary in summary.values()
    )


def test_parse_llama_cpp_hf_command_accepts_repo_and_filename():
    repo_id, filename = parse_llama_cpp_hf_command(
        "llama-cpp-hf:unsloth/gemma-4-E4B-it-GGUF:gemma-4-E4B-it-Q4_K_S.gguf"
    )

    assert repo_id == "unsloth/gemma-4-E4B-it-GGUF"
    assert filename == "gemma-4-E4B-it-Q4_K_S.gguf"


def test_learning_eval_corrects_seed_knowledge_base(tmp_path):
    data_file = tmp_path / "oulu.yml"
    data_file.write_text(
        "\n".join(
            [
                "categories:",
                "- oulu",
                "conversations:",
                "- - Missä Linnanmaan kampus sijaitsee?",
                "  - Se sijaitsee Tampereen Hervannassa.",
            ]
        ),
        encoding="utf-8",
    )
    case = KnowledgeEvalCase(
        id="oulu-correction",
        kind="exact",
        question="Missä Linnanmaan kampus sijaitsee?",
        expected_answer="Se sijaitsee Oulun Linnanmaalla.",
        simulated_llm_answer="En ole varma oikeasta vastauksesta.",
        source_question="Missä Linnanmaan kampus sijaitsee?",
        source_answer="Se sijaitsee Oulun Linnanmaalla.",
        negative_answer="Se sijaitsee Tampereen Hervannassa.",
        should_use_knowledge_base=True,
    )

    with KnowledgeBase(":memory:") as kb:
        kb.ingest_path(data_file)
        run = run_learning_eval(kb, [case], ModelSpec("weak", "simulated:weak"))

        assert run.summary["before_correct"] == 0
        assert run.summary["after_correct"] == 1
        assert run.summary["learned"] == 1
        assert (
            kb.direct_answer("Missä Linnanmaan kampus sijaitsee?")
            == "Se sijaitsee Oulun Linnanmaalla."
        )


def test_learning_eval_leaves_correct_seed_knowledge_base_unchanged(tmp_path):
    data_file = tmp_path / "oulu_correct.yml"
    data_file.write_text(
        "\n".join(
            [
                "categories:",
                "- oulu",
                "conversations:",
                "- - Missä Linnanmaan kampus sijaitsee?",
                "  - Se sijaitsee Oulun Linnanmaalla.",
            ]
        ),
        encoding="utf-8",
    )
    case = KnowledgeEvalCase(
        id="oulu-correct",
        kind="exact",
        question="Missä Linnanmaan kampus sijaitsee?",
        expected_answer="Se sijaitsee Oulun Linnanmaalla.",
        simulated_llm_answer="En ole varma oikeasta vastauksesta.",
        source_question="Missä Linnanmaan kampus sijaitsee?",
        source_answer="Se sijaitsee Oulun Linnanmaalla.",
        negative_answer="Se sijaitsee Tampereen Hervannassa.",
        should_use_knowledge_base=True,
    )

    with KnowledgeBase(":memory:") as kb:
        kb.ingest_path(data_file)
        run = run_learning_eval(kb, [case], ModelSpec("weak", "simulated:weak"))

        assert run.summary["before_correct"] == 1
        assert run.summary["after_correct"] == 1
        assert run.summary["learned"] == 0
        assert kb.count() == 1


def test_answer_model_prompt_bypasses_knowledge_base():
    answer = answer_model_prompt("Mikä on Oulu?", ModelSpec("echo", "simulated:echo"))

    assert answer == "Mikä on Oulu?"
