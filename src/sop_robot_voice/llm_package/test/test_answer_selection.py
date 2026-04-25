from llm_package.answer_selection import choose_closest_answer
from llm_package.knowledge_base import RetrievalResult


def result(
    *,
    question: str,
    answer: str,
    method: str = "exact",
    score: float = 1.0,
) -> RetrievalResult:
    return RetrievalResult(
        id=1,
        question=question,
        answer=answer,
        source_path="test",
        source_format="json",
        category=None,
        score=score,
        method=method,
    )


def test_exact_knowledge_answer_wins_when_llm_disagrees():
    selection = choose_closest_answer(
        user_message="Kuka kehitti ensimmäisen robottia?",
        llm_answer="Ensimmäinen robotti kehitettiin 1950-luvulla.",
        retrieval_results=[
            result(
                question="Kuka kehitti ensimmäisen robottia?",
                answer="Ensimmäisen robottin kehitti Isaac Asimov vuonna 1942.",
            )
        ],
    )

    assert selection.source == "knowledge_base"
    assert selection.answer == "Ensimmäisen robottin kehitti Isaac Asimov vuonna 1942."


def test_llm_answer_wins_when_it_matches_knowledge_answer():
    selection = choose_closest_answer(
        user_message="Miten akku ladataan?",
        llm_answer="Akku ladataan lataustelakassa.",
        retrieval_results=[
            result(
                question="Miten akku ladataan?",
                answer="Akku ladataan lataustelakassa.",
            )
        ],
    )

    assert selection.source == "llm"
    assert selection.answer == "Akku ladataan lataustelakassa."


def test_knowledge_answer_wins_when_llm_only_partly_overlaps():
    selection = choose_closest_answer(
        user_message="Millaista tietotekniikan opiskelu on?",
        llm_answer="Tietotekniikan opiskelu on mielenkiintoista.",
        retrieval_results=[
            result(
                question="Millaista tietotekniikan opiskelu on?",
                answer="Tietotekniikan opiskelu on kivaa mutta joskus vaikeaa.",
            )
        ],
    )

    assert selection.source == "knowledge_base"
    assert selection.answer == "Tietotekniikan opiskelu on kivaa mutta joskus vaikeaa."


def test_weak_knowledge_match_does_not_override_llm_answer():
    selection = choose_closest_answer(
        user_message="Mikä sää tänään on?",
        llm_answer="En näe reaaliaikaista säätä.",
        retrieval_results=[
            result(
                question="Miten akku ladataan?",
                answer="Akku ladataan lataustelakassa.",
                method="like",
                score=0.1,
            )
        ],
    )

    assert selection.source == "llm"
    assert selection.answer == "En näe reaaliaikaista säätä."


def test_knowledge_answer_can_be_disabled():
    selection = choose_closest_answer(
        user_message="Kuka kehitti ensimmäisen robottia?",
        llm_answer="LLM vastaus.",
        retrieval_results=[
            result(
                question="Kuka kehitti ensimmäisen robottia?",
                answer="Tietokannan vastaus.",
            )
        ],
        allow_knowledge_answer=False,
    )

    assert selection.source == "llm"
    assert selection.answer == "LLM vastaus."
