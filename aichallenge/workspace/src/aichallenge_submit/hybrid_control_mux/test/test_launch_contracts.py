from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _aichallenge_submit_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _parse_launch(relative_path: str) -> ET.Element:
    return ET.parse(_aichallenge_submit_root() / relative_path).getroot()


def test_fail_safe_caps_cannot_fall_through_to_race_speed():
    config_path = (
        _aichallenge_submit_root()
        / "overtake_planner/config/overtake_planner.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "overtake_planner_node"
    ]["ros__parameters"]

    assert config["normal_recovery_speed_only_v_max_mps"] == 10.0
    assert config["speed_only_fallback_v_max_mps"] == 0.5
    assert config["opponent_collision_fallback_v_max_mps"] == 0.5
    assert config["side_by_side_leader_priority_v_max_mps"] <= 3.0
    assert config["side_by_side_speed_cap_mps"] == 7.5
    assert config["corner_yield_v_max_mps"] == 10.0
    assert config["large_lateral_error_v_max_mps"] <= 3.0
    assert config["reentry_mpc_degraded_hold_v_max_mps"] <= 3.0
    assert config["post_abort_curve_hold_v_max_mps"] <= 3.0
    assert config["recovery_v_max_mps"] <= 3.0
    assert config["wall_margin_recovery_v_max_mps"] == 0.5
    assert config["wall_risk_v_max_mps"] == 0.5
    assert config["mpc_health_v_max_mps"] == 0.5
    assert config["recovery_speed_guard_v_max_mps"] == 0.5


def test_follow_gap_closing_has_a_reachable_follow_window():
    config_path = (
        _aichallenge_submit_root()
        / "overtake_planner/config/overtake_planner.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["overtake_planner_node"][
        "ros__parameters"
    ]

    assert config["follow_gap_closing_enabled"] is True
    assert config["follow_gap_closing_target_gap_m"] >= config["safety_ellipse_a_m"]
    assert (
        config["follow_gap_closing_target_gap_m"]
        <= config["follow_gap_closing_engage_gap_m"]
        < config["follow_trigger_s_m"]
    )


def test_pure_pursuit_launch_wires_timing_and_vehicle_geometry():
    root = _parse_launch("aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml")
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.iter("param")
    }

    assert params["use_sim_time"] == "$(var use_sim_time)"
    assert params["wheel_base"] == "$(var wheel_base)"
    assert params["max_odom_age_sec"] == "$(var max_odom_age_sec)"
    assert params["max_trajectory_age_sec"] == "$(var max_trajectory_age_sec)"
    assert params["max_override_age_sec"] == "$(var max_override_age_sec)"
    assert params["stop_on_stale_input"] == "$(var stop_on_stale_input)"
    assert params["use_mpc_predicted_horizon"] == "$(var use_mpc_predicted_horizon)"
    assert params["max_mpc_horizon_age_sec"] == "$(var max_mpc_horizon_age_sec)"
    assert params["min_mpc_horizon_points"] == "$(var min_mpc_horizon_points)"
    assert (
        params["require_solved_mpc_health_for_horizon"]
        == "$(var require_solved_mpc_health_for_horizon)"
    )
    assert params["max_mpc_health_age_sec"] == "$(var max_mpc_health_age_sec)"
    assert (
        params["require_matching_overtake_horizon_contract"]
        == "$(var require_matching_overtake_horizon_contract)"
    )
    assert params["recovery_mode"] == "$(var recovery_mode)"
    assert params["recovery_status_timeout_sec"] == "$(var recovery_status_timeout_sec)"
    assert params["pp_control_delay_sec"] == "$(var pp_control_delay_sec)"
    assert params["steering_time_constant_sec"] == "$(var steering_time_constant_sec)"
    assert params["steering_status_timeout_sec"] == "$(var steering_status_timeout_sec)"
    assert (
        params["min_velocity_for_delay_compensation_mps"]
        == "$(var min_velocity_for_delay_compensation_mps)"
    )
    assert (
        params["horizon_curvature_feedforward_gain"]
        == "$(var horizon_curvature_feedforward_gain)"
    )

    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in root.iter("remap")
    }
    assert remaps["input/mpc_predicted_horizon"] == "$(var input_mpc_predicted_horizon)"
    assert (
        remaps["input/mpc_predicted_horizon_contract"]
        == "$(var input_mpc_predicted_horizon_contract)"
    )
    assert remaps["input/mpc_health"] == "$(var input_mpc_health)"
    assert remaps["input/steering_status"] == "$(var input_steering_status)"
    assert remaps["input/recovery_status"] == "$(var input_recovery_status)"
    assert remaps["output/recovery_control_cmd"] == "$(var output_recovery_control_cmd)"
    assert remaps["/pure_pursuit/debug"] == "$(var output_debug)"


def test_pure_pursuit_launch_uses_mpc_consistent_steering_gain():
    root = _parse_launch("aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml")
    steering_gain_values = [
        element.attrib["value"]
        for element in root.iter("let")
        if element.attrib.get("name") == "steering_tire_angle_gain_var"
    ]

    assert steering_gain_values == ["1.639", "1.639"]


def test_delay_aware_mpc_launch_wires_raw_command_history_input():
    root = _parse_launch("aichallenge_submit_launch/launch/control/delay_aware_mpc.launch.xml")
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.iter("param")
    }

    assert params["input_control_cmd_raw_topic"] == "$(var input_control_cmd_raw)"

    mpc_node = next(
        element
        for element in root.iter("node")
        if element.attrib.get("name") == "delay_aware_mpc_controller"
    )
    assert "--config_path $(var config_path)" in mpc_node.attrib["args"]
    assert "--ref_vel_path $(var ref_vel_path)" in mpc_node.attrib["args"]


def test_hybrid_delay_aware_mpc_explicitly_passes_pure_pursuit_wheel_base():
    root = _parse_launch(
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml"
    )
    include_paths = [
        element.attrib.get("file", "")
        for element in root.iter("include")
    ]
    pure_pursuit_include = next(
        path for path in include_paths if "pure_pursuit.launch.xml" in path
    )
    include = next(
        element
        for element in root.iter("include")
        if element.attrib.get("file") == pure_pursuit_include
    )
    args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in include.iter("arg")
    }

    assert args["wheel_base"] == "1.087"


def test_hybrid_delay_aware_mpc_wires_mpc_horizon_to_pure_pursuit():
    root = _parse_launch(
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml"
    )
    includes = list(root.iter("include"))
    delay_mpc_include = next(
        element
        for element in includes
        if "delay_aware_mpc.launch.xml" in element.attrib.get("file", "")
    )
    pure_pursuit_include = next(
        element
        for element in includes
        if "pure_pursuit.launch.xml" in element.attrib.get("file", "")
    )
    delay_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in delay_mpc_include.iter("arg")
    }
    pp_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in pure_pursuit_include.iter("arg")
    }

    assert delay_args["output_mpc_predicted_horizon"] == "/hybrid_control/mpc/predicted_horizon"
    assert (
        delay_args["output_mpc_predicted_horizon_contract"]
        == "/hybrid_control/mpc/predicted_horizon_contract"
    )
    assert delay_args["input_control_cmd_raw"] == "/hybrid_control/mpc/control_cmd_raw"
    assert pp_args["input_mpc_predicted_horizon"] == "/hybrid_control/mpc/predicted_horizon"
    assert (
        pp_args["input_mpc_predicted_horizon_contract"]
        == "/hybrid_control/mpc/predicted_horizon_contract"
    )
    assert pp_args["input_steering_status"] == "/vehicle/status/steering_status"
    assert pp_args["use_mpc_predicted_horizon"] == "true"
    assert pp_args["max_mpc_horizon_age_sec"] == "0.15"
    assert pp_args["require_matching_overtake_horizon_contract"] == "true"
    assert pp_args["pp_control_delay_sec"] == "0.0"
    assert pp_args["steering_time_constant_sec"] == "0.30"
    assert pp_args["horizon_curvature_feedforward_gain"] == "0.0"


def test_hybrid_delay_aware_mpc_can_realize_planner_brake_assumption():
    root = _aichallenge_submit_root()
    with (root / "delay_aware_mpc_ros/config/delay_aware_config.yaml").open() as stream:
        mpc_config = yaml.safe_load(stream)["mpc"]
    with (root / "overtake_planner/config/overtake_planner.param.yaml").open() as stream:
        planner_config = yaml.safe_load(stream)["overtake_planner_node"][
            "ros__parameters"
        ]

    # hybrid_delay_aware_mpc passes the MPC command through the mux when it is
    # healthy. Its deceleration limit must therefore be at least the braking
    # capability assumed by stationary-obstacle safety prediction.
    assert mpc_config["a_min"] <= -planner_config["max_brake_decel_mps2"]


def test_pure_pursuit_mpc_horizon_keeps_mpc_as_horizon_generator():
    root = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml"
    )
    launch_args = {
        element.attrib.get("name"): element.attrib
        for element in root.findall("arg")
    }
    includes = list(root.iter("include"))
    delay_mpc_include = next(
        element
        for element in includes
        if "delay_aware_mpc.launch.xml" in element.attrib.get("file", "")
    )
    pure_pursuit_include = next(
        element
        for element in includes
        if "pure_pursuit.launch.xml" in element.attrib.get("file", "")
    )
    mux_include = next(
        element
        for element in includes
        if "hybrid_control_mux.launch.xml" in element.attrib.get("file", "")
    )
    delay_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in delay_mpc_include.iter("arg")
    }
    pp_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in pure_pursuit_include.iter("arg")
    }
    mux_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in mux_include.iter("arg")
    }

    assert launch_args["use_obstacle_avoidance"]["default"] == "true"
    assert delay_args["config_path"].endswith("pure_pursuit_mpc_horizon_config.yaml")
    assert delay_args["use_obstacle_avoidance"] == "$(var use_obstacle_avoidance)"
    assert delay_args["input_control_cmd_raw"] == (
        "/pure_pursuit_mpc_horizon/pure_pursuit/control_cmd_raw"
    )
    assert delay_args["output_mpc_predicted_horizon"] == (
        "/pure_pursuit_mpc_horizon/mpc/predicted_horizon"
    )
    assert delay_args["output_mpc_predicted_horizon_contract"] == (
        "/pure_pursuit_mpc_horizon/mpc/predicted_horizon_contract"
    )
    assert delay_args["overtake_horizon_points"] == "40"
    assert delay_args["overtake_mpc_health_solve_time_warn_ms"] == "200.0"
    assert pp_args["input_mpc_predicted_horizon"] == (
        "/pure_pursuit_mpc_horizon/mpc/predicted_horizon"
    )
    assert pp_args["input_mpc_predicted_horizon_contract"] == (
        "/pure_pursuit_mpc_horizon/mpc/predicted_horizon_contract"
    )
    assert pp_args["require_solved_mpc_health_for_horizon"] == "true"
    assert pp_args["max_mpc_health_age_sec"] == "0.35"
    assert pp_args["require_matching_overtake_horizon_contract"] == "true"
    assert mux_args["param_file"].endswith("pure_pursuit_mpc_horizon.param.yaml")
    assert mux_args["input_pure_pursuit_cmd"] == (
        "/pure_pursuit_mpc_horizon/pure_pursuit/control_cmd"
    )
    assert mux_args["require_safety_constraint"] == "$(var use_overtake_planner)"
    assert mux_args["input_safety_constraint"] == "/overtake/safety_constraint"
    assert mux_args["input_overtake_plan"] == "/overtake/plan"


def test_hybrid_mux_exposes_typed_safety_constraint_contract():
    root = _parse_launch("hybrid_control_mux/launch/hybrid_control_mux.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.iter("param")
    }
    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in root.iter("remap")
    }

    assert launch_args["require_safety_constraint"] == "false"
    assert launch_args["input_safety_constraint"] == "/overtake/safety_constraint"
    assert launch_args["input_overtake_plan"] == "/overtake/plan"
    assert params["require_safety_constraint"] == "$(var require_safety_constraint)"
    assert remaps["input/safety_constraint"] == "$(var input_safety_constraint)"
    assert remaps["input/overtake_plan"] == "$(var input_overtake_plan)"


def test_v2_shadow_topics_are_not_controller_inputs():
    submit_root = _aichallenge_submit_root()
    controller_paths = [
        submit_root / "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py",
        submit_root / "simple_pure_pursuit/src/simple_pure_pursuit.cpp",
        submit_root / "multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py",
    ]

    for path in controller_paths:
        assert "/overtake/v2/shadow/" not in path.read_text(encoding="utf-8")

    for launch_path in submit_root.rglob("*.launch.xml"):
        root = ET.parse(launch_path).getroot()
        for element in root.iter():
            if element.tag not in {"arg", "remap"}:
                continue
            if element.attrib.get("name") in {
                "input_overtake_plan",
                "input_safety_constraint",
            } or element.attrib.get("from") in {
                "input/overtake_plan",
                "input/safety_constraint",
            }:
                serialized = " ".join(element.attrib.values())
                assert "/overtake/v2/shadow/" not in serialized


def test_hybrid_profiles_have_only_mux_wired_to_final_control_command():
    for relative_path in (
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml",
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ):
        root = _parse_launch(relative_path)
        final_output_wires = [
            element
            for element in root.iter("arg")
            if element.attrib.get("name") == "output_control_cmd"
            and element.attrib.get("value") == "/control/command/control_cmd"
        ]

        assert len(final_output_wires) == 1
        parent = next(
            include
            for include in root.iter("include")
            if final_output_wires[0] in list(include)
        )
        assert "hybrid_control_mux.launch.xml" in parent.attrib.get("file", "")


def test_pure_pursuit_mpc_horizon_allows_free_run_acceleration_without_changing_braking():
    config_path = (
        _aichallenge_submit_root()
        / "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["hybrid_control_mux_node"][
        "ros__parameters"
    ]

    # primary Pure Pursuit also traverses this clamp.  Keep the free-run
    # acceleration below MPC a_max=3.0 without weakening the 1.5 m/s^2
    # braking/stop paths.
    assert config["primary_source"] == "pure_pursuit"
    assert config["fallback_accel_max_mps2"] == 2.0
    assert config["fallback_decel_min_mps2"] == -1.5
    assert config["stop_decel_mps2"] == -1.5


def test_pure_pursuit_mpc_horizon_config_publishes_neutral_outside_overtake():
    config_path = (
        _aichallenge_submit_root()
        / "delay_aware_mpc_ros/config/pure_pursuit_mpc_horizon_config.yaml"
    )
    with config_path.open() as stream:
        config = yaml.safe_load(stream)

    mpc_config = config["mpc"]
    assert mpc_config["predicted_horizon_publish_mode"] == "overtake_or_neutral"
    assert mpc_config["neutral_horizon_publish_period_sec"] == 0.05


def test_wall_recovery_launch_contract_is_fail_closed_by_default():
    config_path = (
        _aichallenge_submit_root()
        / "wall_recovery_planner/config/wall_recovery_planner.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "wall_recovery_planner_node"
    ]["ros__parameters"]

    assert config["wall_recovery_enabled"] is False
    assert config["allow_unverified_recovery"] is False
    assert config["require_recovery_permit"] is True
    assert config["recovery_v_max_mps"] == 0.7


def test_hybrid_control_mux_wires_recovery_inputs():
    root = _parse_launch("hybrid_control_mux/launch/hybrid_control_mux.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.iter("param")
    }
    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in root.iter("remap")
    }

    assert launch_args["recovery_enabled"] == "false"
    assert params["recovery_enabled"] == "$(var recovery_enabled)"
    assert remaps["input/recovery_control_cmd"] == "$(var input_recovery_cmd)"
    assert remaps["input/recovery_status"] == "$(var input_recovery_status)"
    assert remaps["input/recovery_permit"] == "$(var input_recovery_permit)"


def test_pure_pursuit_mpc_horizon_wires_recovery_pp_without_overtake_or_mpc_horizon():
    root = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml"
    )
    includes = [
        element
        for element in root.iter("include")
        if "pure_pursuit.launch.xml" in element.attrib.get("file", "")
    ]
    recovery_include = next(
        element
        for element in includes
        if any(
            arg.attrib.get("name") == "node_name"
            and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
            for arg in element.iter("arg")
        )
    )
    args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in recovery_include.iter("arg")
    }

    assert args["input_trajectory"] == "/wall_recovery/trajectory"
    assert args["input_recovery_status"] == "/wall_recovery/status"
    assert args["output_recovery_control_cmd"] == "/hybrid_control/recovery/control_cmd"
    assert args["use_mpc_predicted_horizon"] == "false"
    assert args["use_overtake_reference_override"] == "false"
    assert args["recovery_mode"] == "true"


def test_reference_launch_exposes_pure_pursuit_mpc_horizon_control_method():
    root = _parse_launch("aichallenge_submit_launch/launch/reference.launch.xml")
    control_arg = next(
        element
        for element in root.iter("arg")
        if element.attrib.get("name") == "control_method"
    )
    assert "pure_pursuit_mpc_horizon" in control_arg.attrib["description"]

    includes = [
        element.attrib.get("file", "")
        for group in root.iter("group")
        if "'$(var control_method)' == 'pure_pursuit_mpc_horizon'" in group.attrib.get("if", "")
        for element in group.iter("include")
    ]
    assert any("pure_pursuit_mpc_horizon.launch.xml" in path for path in includes)


def test_reference_launch_uses_mincurv_manual_trajectory_source():
    root = _parse_launch("aichallenge_submit_launch/launch/reference.launch.xml")
    generator = next(
        element
        for element in root.iter("node")
        if element.attrib.get("pkg") == "simple_trajectory_generator"
    )
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in generator.iter("param")
    }

    assert params["csv_path"] == (
        "$(find-pkg-share multi_purpose_mpc_ros)/env/final_ver3/"
        "traj_mincurv_manual.csv"
    )
