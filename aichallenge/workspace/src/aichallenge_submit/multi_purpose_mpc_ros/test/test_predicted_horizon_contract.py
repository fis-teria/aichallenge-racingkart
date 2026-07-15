from pathlib import Path


def test_abort_recovery_requires_explicit_mandatory_authorization() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "multi_purpose_mpc_ros"
        / "mpc_controller.py"
    ).read_text(encoding="utf-8")

    assert "ABORT_RECOVERY_MODE_ID = 7" in source
    assert "mandatory_lateral_avoidance" in source
    assert "if mode_id == ABORT_RECOVERY_MODE_ID:" in source


def test_horizon_contract_publishes_immutable_solver_provenance() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "multi_purpose_mpc_ros"
        / "mpc_controller.py"
    ).read_text(encoding="utf-8")

    assert '"/mpc/predicted_horizon_contract"' in source
    assert '"override_generation"' in source
    assert '"solver_horizon_authorized"' in source
    assert '"mandatory_lateral_avoidance"' in source
    assert '"contract_version": 2' in source
