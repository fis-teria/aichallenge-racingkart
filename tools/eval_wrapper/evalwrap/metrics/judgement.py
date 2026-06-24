from __future__ import annotations


def judge_domain(
    finish: bool | None,
    total_time_sec: float | None,
    penalty_count: int | None,
    collision_count: int | None,
    baseline_total_time_sec: float | None = None,
) -> str:
    if finish is False:
        return "reject"
    if finish is None:
        return "needs_more_eval"

    penalties = penalty_count or 0
    collisions = collision_count or 0
    if penalties > 0 or collisions > 0:
        if baseline_total_time_sec is not None and total_time_sec is not None and total_time_sec < baseline_total_time_sec:
            return "attack_candidate"
        return "needs_more_eval"

    if baseline_total_time_sec is None or total_time_sec is None:
        return "stable_candidate"
    if total_time_sec < baseline_total_time_sec:
        return "candidate"
    return "stable_candidate"
