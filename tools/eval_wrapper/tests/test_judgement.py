from __future__ import annotations

from evalwrap.metrics.judgement import judge_domain


def test_judgement_rejects_unfinished() -> None:
    assert judge_domain(False, None, 0, 0) == "reject"


def test_judgement_marks_clean_finish_stable() -> None:
    assert judge_domain(True, 100.0, 0, 0) == "stable_candidate"


def test_judgement_marks_penalty_as_more_eval() -> None:
    assert judge_domain(True, 90.0, 1, 0) == "needs_more_eval"
