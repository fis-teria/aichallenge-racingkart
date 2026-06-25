from __future__ import annotations

from typing import Any


def judge_overtake_run(metrics: dict[str, object], config: dict[str, Any]) -> str:
    collision_count = _int(metrics.get("collision_count"))
    penalty_count = _int(metrics.get("penalty_count"))
    infeasible_count = _int(metrics.get("mpc_infeasible_count"))
    success_rate = _float(metrics.get("success_rate")) or 0.0
    max_slack = _float(metrics.get("max_cbf_slack")) or 0.0
    total_delta = _float(metrics.get("total_time_delta_sec"))
    blocked_delta = _float(metrics.get("blocked_time_delta_sec"))
    attempt_count = _int(metrics.get("attempt_count"))

    if (
        collision_count > 0
        or penalty_count > 0
        or infeasible_count > _cfg(config, "judgement.infeasible_limit", 2.0)
        or (total_delta is not None and total_delta > _cfg(config, "judgement.regression_limit_sec", 1.5))
    ):
        return "reject"

    if attempt_count <= 0:
        return "needs_more_eval"

    if (
        success_rate >= _cfg(config, "judgement.candidate_success_rate", 0.60)
        and (blocked_delta is None or blocked_delta < 0.0)
        and max_slack <= 0.10
        and infeasible_count == 0
    ):
        return "candidate"

    if (
        total_delta is not None
        and total_delta < 0.0
        and success_rate >= _cfg(config, "judgement.attack_success_rate", 0.50)
        and max_slack <= 0.25
    ):
        return "attack_candidate"

    if (
        collision_count == 0
        and penalty_count == 0
        and max_slack <= 0.05
        and (total_delta is None or total_delta <= _cfg(config, "judgement.small_regression_allowance_sec", 0.5))
    ):
        return "stable_candidate"

    return "needs_more_eval"


def _int(value: object) -> int:
    if isinstance(value, bool) or value in (None, ""):
        return 0
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _float(value: object) -> float | None:
    if isinstance(value, bool) or value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _cfg(config: dict[str, Any], dotted: str, default: float) -> float:
    current: Any = config
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return float(current) if isinstance(current, (int, float)) else default
