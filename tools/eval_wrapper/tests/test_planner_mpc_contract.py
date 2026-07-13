from evalwrap.analysis.planner_mpc_contract import (
    compare_planner_and_mpc_horizons,
    contract_metrics,
)


def _inputs(offset: float = 0.5):
    return {
        "overrides": [
            {
                "time_sec": 1.0,
                "active": True,
                "mode_id": 7,
                "override_generation": 42,
                "lateral_offsets_m": [offset, offset],
            }
        ],
        "horizons": [
            {
                "time_sec": 1.1,
                "stamp_sec": 10,
                "stamp_nanosec": 20,
                "points": [(0.0, 0.5, None), (1.0, 0.5, None)],
            }
        ],
        "contracts": [
            {
                "time_sec": 1.1,
                "horizon_stamp_sec": 10,
                "horizon_stamp_nanosec": 20,
                "source": "solver_prediction",
                "mode_id": 7,
                "override_generation": 42,
            }
        ],
        "reference_history": [
            {
                "time_sec": 1.0,
                "topic": "/planning/scenario_planning/trajectory",
                "points": [(0.0, 0.0, None), (1.0, 0.0, None), (2.0, 0.0, None)],
            }
        ],
        "pure_pursuit_debug": [
            {
                "time_sec": 1.1,
                "mpc_horizon_applied": True,
                "trajectory_source": "mpc_horizon",
                "mpc_horizon_reject_reason": "fresh",
                "applied_horizon_stamp_sec": 10,
                "applied_horizon_stamp_nanosec": 20,
                "applied_horizon_source": "solver_prediction",
                "applied_horizon_mode_id": 7,
                "applied_horizon_generation": 42,
                "evaluated_horizon_stamp_sec": 10,
                "evaluated_horizon_stamp_nanosec": 20,
                "command_stamp_sec": 30,
                "command_stamp_nanosec": 40,
            }
        ],
        "mux_debug": [
            {
                "time_sec": 1.1,
                "source": "pure_pursuit",
                "selected_source": "pure_pursuit",
                "reason": "primary_pure_pursuit",
                "selected_input_stamp_sec": 30,
                "selected_input_stamp_nanosec": 40,
                "output_stamp_sec": 31,
                "output_stamp_nanosec": 50,
                "output_speed_mps": 4.0,
                "output_accel_mps2": -0.5,
                "output_steer_rad": 0.1,
            }
        ],
        "final_control_commands": [
            {
                "time_sec": 1.1,
                "command_stamp_sec": 31,
                "command_stamp_nanosec": 50,
                "target_speed_mps": 4.0,
                "accel_mps2": -0.5,
                "steer_rad": 0.1,
            }
        ],
    }


def test_pointwise_contract_comparison_matches_solver_horizon() -> None:
    summaries, points = compare_planner_and_mpc_horizons(**_inputs())

    assert len(summaries) == 1
    assert summaries[0]["status"] == "matched"
    assert summaries[0]["execution_status"] == "verified"
    assert summaries[0]["mux_output_stamp_sec"] == 31
    assert summaries[0]["compared_points"] == 2
    assert summaries[0]["max_abs_lateral_error_m"] == 0.0
    assert summaries[0]["mux_source"] == "pure_pursuit"
    assert [row["within_tolerance"] for row in points] == [True, True]
    assert contract_metrics(summaries)["planner_mpc_contract_mismatch_count"] == 0


def test_pointwise_contract_comparison_reports_lateral_mismatch() -> None:
    summaries, points = compare_planner_and_mpc_horizons(**_inputs(offset=0.1))

    assert summaries[0]["status"] == "mismatch"
    assert summaries[0]["first_mismatch_point_index"] == 0
    assert points[0]["abs_lateral_error_m"] == 0.4
    assert contract_metrics(summaries)["planner_mpc_contract_mismatch_count"] == 1


def test_contract_comparison_refuses_to_call_missing_reference_a_match() -> None:
    inputs = _inputs()
    inputs["reference_history"] = []

    summaries, points = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "unverifiable_reference_missing"
    assert points == []


def test_published_match_is_not_called_executed_when_pp_rejects_horizon() -> None:
    inputs = _inputs()
    inputs["pure_pursuit_debug"][0]["mpc_horizon_applied"] = False
    inputs["pure_pursuit_debug"][0]["trajectory_source"] = "trajectory_overtake_override"

    summaries, points = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "published_match_not_executed"
    assert summaries[0]["execution_reason"] == "pure_pursuit_rejected_horizon"
    assert points[0]["summary_status"] == "published_match_not_executed"
    assert contract_metrics(summaries)["planner_mpc_contract_execution_match_count"] == 0


def test_published_match_is_not_called_executed_when_mux_selects_mpc() -> None:
    inputs = _inputs()
    inputs["mux_debug"][0]["source"] = "mpc"

    summaries, _ = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "published_match_not_executed"
    assert summaries[0]["execution_reason"] == "hybrid_mux_selected_other_source"


def test_published_match_is_unverifiable_without_exact_mux_stamp_join() -> None:
    inputs = _inputs()
    inputs["mux_debug"][0]["selected_input_stamp_nanosec"] = 41

    summaries, _ = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "unverifiable_execution"
    assert summaries[0]["execution_reason"] == "hybrid_mux_exact_join_missing"


def test_published_match_is_unverifiable_without_exact_final_command_join() -> None:
    inputs = _inputs()
    inputs["final_control_commands"] = []

    summaries, _ = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "unverifiable_execution"
    assert summaries[0]["execution_reason"] == "final_control_exact_join_missing"


def test_published_match_is_unverifiable_when_final_command_payload_differs() -> None:
    inputs = _inputs()
    inputs["final_control_commands"][0]["steer_rad"] = 0.2

    summaries, _ = compare_planner_and_mpc_horizons(**inputs)

    assert summaries[0]["status"] == "unverifiable_execution"
    assert summaries[0]["execution_reason"] == "final_control_payload_mismatch"
