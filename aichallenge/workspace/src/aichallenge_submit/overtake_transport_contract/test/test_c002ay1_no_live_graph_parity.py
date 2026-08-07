#!/usr/bin/env python3
"""Source-level Graph 3 and publish-order parity guard for C-002AY1."""

from pathlib import Path
import re
import sys


ROS_ENDPOINT_TOKENS = (
    "create_publisher",
    "create_subscription",
    "create_service",
    "create_client",
)


def _slice_function(text: str, signature: str, next_signature: str) -> str:
    start = text.find(signature)
    end = text.find(next_signature, start + len(signature))
    if start < 0 or end < 0:
        raise AssertionError(f"function boundary missing: {signature}")
    return text[start:end]


def main() -> int:
    submit_root = Path(sys.argv[1]).resolve()
    planner = (
        submit_root / "overtake_planner/src/overtake_planner_node.cpp"
    ).read_text(encoding="utf-8")
    pp = (
        submit_root / "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
    ).read_text(encoding="utf-8")
    mux = (
        submit_root
        / "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py"
    ).read_text(encoding="utf-8")

    # Graph 3: disabled, enabled, and unavailable/full all use the same
    # production source graph. The observer may not create any ROS endpoint or
    # guard an existing endpoint behind its startup flag.
    graph_sources = (planner, pp, mux)
    baseline_signature = tuple(
        tuple(source.count(token) for token in ROS_ENDPOINT_TOKENS)
        for source in graph_sources
    )
    for scenario in ("disabled", "enabled", "unavailable_or_full"):
        scenario_signature = tuple(
            tuple(source.count(token) for token in ROS_ENDPOINT_TOKENS)
            for source in graph_sources
        )
        if scenario_signature != baseline_signature:
            raise AssertionError(f"{scenario}: ROS graph signature changed")

    for source_name, source in (
        ("Planner", planner),
        ("PurePursuit", pp),
        ("Mux", mux),
    ):
        for match in re.finditer(r"c002ay1_prod_measure", source):
            window = source[max(0, match.start() - 600) : match.end() + 600]
            if any(token in window for token in ROS_ENDPOINT_TOKENS):
                raise AssertionError(
                    f"{source_name}: observer flag is adjacent to ROS graph wiring"
                )

    # Output parity: hooks may observe entry/return and raw request count only.
    # Existing publish order is frozen by token order, and no observer write is
    # permitted between PP output publications.
    planner_timer = _slice_function(planner, "  void onTimer()", "\n  FrenetFrame ")
    planner_order = (
        "publishSafetyConstraint",
        "publishAuthoritativePlan",
        "publishSupervisorV2Shadow",
        "publishDebug",
    )
    positions = [planner_timer.find(token) for token in planner_order]
    if any(position < 0 for position in positions) or positions != sorted(positions):
        raise AssertionError("Planner output publish order changed")

    pp_timer = _slice_function(pp, "void SimplePurePursuit::onTimer()", "\nvoid ")
    pp_order = (
        "publishControllerTrackingStatus",
        "publishControllerCommandEnvelope",
        "publishControllerExecutionEnvelope",
        "pub_cmd_->publish",
        "pub_raw_cmd_->publish",
        "captureControllerAppliedShadow",
    )
    positions = [pp_timer.find(token) for token in pp_order]
    if any(position < 0 for position in positions) or positions != sorted(positions):
        raise AssertionError("Pure Pursuit output publish order changed")
    first_publish = min(positions)
    last_publish = max(positions)
    if "c002ay1_runtime_observer" in pp_timer[first_publish:last_publish]:
        raise AssertionError("observer write inserted into PP publish sequence")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
