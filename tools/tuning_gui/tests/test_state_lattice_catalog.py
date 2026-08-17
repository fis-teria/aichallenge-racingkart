from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


APP_PATH = Path(__file__).resolve().parents[1] / "app.py"
SPEC = importlib.util.spec_from_file_location("tuning_gui_app", APP_PATH)
assert SPEC is not None and SPEC.loader is not None
APP = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = APP
SPEC.loader.exec_module(APP)


def test_state_lattice_pure_pursuit_catalog_covers_runtime_inputs() -> None:
    files = APP.catalog_files("state_lattice_pure_pursuit")
    paths = {item["path"] for item in files}
    expected = {
        str(
            APP.LAUNCH_ROOT
            / "launch/control/state_lattice_pure_pursuit.launch.xml"
        ),
        str(
            APP.LAUNCH_ROOT
            / "launch/control/pure_pursuit_mpc_horizon.launch.xml"
        ),
        str(APP.LAUNCH_ROOT / "launch/control/pure_pursuit.launch.xml"),
        str(
            APP.STATE_LATTICE_ROOT
            / "config/state_lattice_overtake_planner.param.yaml"
        ),
        str(
            APP.STATE_LATTICE_ROOT
            / "launch/state_lattice_overtake_planner.launch.xml"
        ),
        str(APP.STATE_LATTICE_PERMISSION_CSV_PATH),
        str(APP.STATE_LATTICE_REFERENCE_CSV_PATH),
        str(APP.STATE_LATTICE_WALL_MAP_PATH),
        str(
            APP.HYBRID_CONTROL_MUX_ROOT
            / "config/pure_pursuit_mpc_horizon.param.yaml"
        ),
        str(
            APP.HYBRID_CONTROL_MUX_ROOT
            / "launch/hybrid_control_mux.launch.xml"
        ),
        str(
            APP.WALL_RECOVERY_ROOT
            / "config/wall_recovery_planner.param.yaml"
        ),
        str(
            APP.WALL_RECOVERY_ROOT
            / "launch/wall_recovery_planner.launch.xml"
        ),
        str(APP.OVERTAKE_ROOT / "config/overtake_planner.param.yaml"),
    }

    assert expected <= paths
    assert all(item["exists"] == "true" for item in files)


def test_state_lattice_permission_profile_uses_permission_csv_contract() -> None:
    assert APP.is_overtake_permission_csv(
        APP.STATE_LATTICE_PERMISSION_CSV_PATH
    )
    content = APP.abs_path(
        APP.STATE_LATTICE_PERMISSION_CSV_PATH
    ).read_text(encoding="utf-8")
    APP.validate_overtake_permission_csv(content)


def test_state_lattice_catalog_parameters_have_specific_descriptions() -> None:
    for item in APP.catalog_files("state_lattice_pure_pursuit"):
        path = item["path"]
        suffix = Path(path).suffix.lower()
        if suffix not in {".yaml", ".yml", ".xml"}:
            continue
        content = APP.abs_path(path).read_text(encoding="utf-8")
        if suffix == ".xml":
            kind = "xml"
            rows = APP._xml_rows(content)
        else:
            kind = "yaml"
            rows = APP._yaml_rows(content)
        for row in rows:
            if kind == "xml" and not (row.get("attrs") or {}).get("name"):
                continue
            description = APP.default_description(kind, row)
            assert description.strip(), (path, row)
            assert not APP.is_generic_description(description), (
                path,
                row,
                description,
            )


def test_mode_descriptions_name_supported_profiles_and_backends() -> None:
    control_method = APP.PROFILE_PARAMETER_DESCRIPTION_DEFAULTS[
        "control_method"
    ]
    control_route = APP.PROFILE_PARAMETER_DESCRIPTION_DEFAULTS["control_route"]
    trajectory_backend = APP.PROFILE_PARAMETER_DESCRIPTION_DEFAULTS[
        "trajectory_backend"
    ]

    assert "state_lattice_pure_pursuit" in control_method
    assert "pure_pursuit_mpc_horizon" in control_method
    assert "current" in control_route
    assert "state_lattice_pure_pursuit" in control_route
    assert "state_lattice_instant_mux" in control_route
    assert "current" in trajectory_backend
    assert "state_lattice_shadow" in trajectory_backend
    assert "state_lattice_candidate" in trajectory_backend


def test_catalog_xml_structured_round_trip_is_byte_identical() -> None:
    for item in APP.catalog_files("state_lattice_pure_pursuit"):
        path = item["path"]
        if Path(path).suffix.lower() != ".xml":
            continue
        content = APP.abs_path(path).read_text(encoding="utf-8")
        rows = APP._xml_rows(content)
        assert APP._apply_xml_rows(content, rows) == content, path


def test_xml_structured_edit_decodes_and_escapes_entities_once() -> None:
    content = (
        '<launch><let name="route_valid" '
        'value="$(eval &quot;1 == 1&quot;)"/></launch>\n'
    )
    rows = APP._xml_rows(content)
    assert rows[0]["attrs"]["value"] == '$(eval "1 == 1")'
    rows[0]["attrs"]["value"] = '$(eval "2 == 2")'

    updated = APP._apply_xml_rows(content, rows)

    assert 'value="$(eval &quot;2 == 2&quot;)"' in updated
    assert "&amp;quot;" not in updated


def test_state_lattice_launch_expressions_are_not_double_escaped() -> None:
    for relative in (
        APP.LAUNCH_ROOT
        / "launch/control/state_lattice_pure_pursuit.launch.xml",
        APP.LAUNCH_ROOT
        / "launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ):
        content = APP.abs_path(relative).read_text(encoding="utf-8")
        assert "&amp;quot;" not in content
        assert "&quot;" in content


def test_state_lattice_pure_pursuit_route_owner_is_valid() -> None:
    config = APP.abs_path(
        APP.OVERTAKE_ROOT / "config/overtake_planner.param.yaml"
    ).read_text(encoding="utf-8")
    assert "control_route: state_lattice_pure_pursuit" in config
    assert "trajectory_backend: current" in config
    assert "control_route: tate_lattice" not in config
    APP.validate_content(
        str(APP.OVERTAKE_ROOT / "config/overtake_planner.param.yaml"),
        config,
    )


def test_state_lattice_pure_pursuit_command_env_enables_v4_poc_sideband(
    monkeypatch,
) -> None:
    monkeypatch.delenv("STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED", raising=False)
    env = APP.command_env("state_lattice_pure_pursuit")
    assert env["STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED"] == "true"
    assert env["STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED"] == "true"
    assert env["STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED"] == "true"
    assert env["STATE_LATTICE_V2_PRODUCER_INSTANCE_ID"] == "4101"
    assert env["STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID"] == "4201"
    assert env["STATE_LATTICE_V2_SESSION_ID"] == "1"

    baseline = APP.command_env("pure_pursuit")
    assert "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED" not in baseline
    assert "STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED" not in baseline


def test_dev_command_keeps_selected_control_method() -> None:
    command = APP.command_for(
        "dev",
        "state_lattice_pure_pursuit",
        False,
        simulator_options={},
    )

    assert "CONTROL_METHOD='state_lattice_pure_pursuit'" in command
    assert command.endswith("make dev")


def test_gate2_uses_official_wrapper_without_mutable_gate_overrides(
    monkeypatch,
) -> None:
    for key in APP.GATE2_MAKE_DISPATCH_OWNED_ENV_KEYS:
        monkeypatch.setenv(key, "caller-controlled")

    command = APP.command_for(
        "gate",
        "state_lattice_pure_pursuit",
        False,
        headless=False,
        simulator_options={"timeout": 999, "raw_args": ""},
        safety_gate="gate2",
    )
    env = APP.command_env(
        "state_lattice_pure_pursuit", "gate", "gate2"
    )

    assert command == (
        "./aic-test run safegate2-stopped-overtake "
        "--runtime-timeout 120 --json"
    )
    assert all(
        key not in env for key in APP.GATE2_MAKE_DISPATCH_OWNED_ENV_KEYS
    )
    assert "AWSIM_TIMEOUT" not in command
    assert "CONTROL_METHOD" not in command


def test_gate2_build_first_cannot_rebuild_reviewed_artifact() -> None:
    command = APP.command_for(
        "gate",
        "state_lattice_pure_pursuit",
        True,
        simulator_options={},
        safety_gate="gate2",
    )

    assert command == (
        "./aic-test run safegate2-stopped-overtake "
        "--runtime-timeout 120 --json"
    )


def test_overtake_route_config_rejects_unknown_modes() -> None:
    path = str(APP.OVERTAKE_ROOT / "config/overtake_planner.param.yaml")
    content = APP.abs_path(path).read_text(encoding="utf-8")

    invalid_route = content.replace(
        "control_route: state_lattice_pure_pursuit",
        "control_route: tate_lattice_instant_mux",
        1,
    )
    with pytest.raises(ValueError, match="control_route must be one of"):
        APP.validate_content(path, invalid_route)

    invalid_backend = content.replace(
        "trajectory_backend: current",
        "trajectory_backend: state_lattice_instant_mux",
        1,
    )
    with pytest.raises(ValueError, match="trajectory_backend must be one of"):
        APP.validate_content(path, invalid_backend)
