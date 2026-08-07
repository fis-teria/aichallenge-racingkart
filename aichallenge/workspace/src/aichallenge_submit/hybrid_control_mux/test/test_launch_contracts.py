import csv
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest
import yaml


def _aichallenge_submit_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _parse_launch(relative_path: str) -> ET.Element:
    return ET.parse(_aichallenge_submit_root() / relative_path).getroot()


def _matching_closing_brace(source: str, opening_brace: int) -> int:
    brace_depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            brace_depth += 1
        elif source[index] == "}":
            brace_depth -= 1
            if brace_depth == 0:
                return index
    raise AssertionError("unterminated C++ brace block")


def _cpp_function_body(source: str, signature: str) -> str:
    signature_start = source.index(signature)
    body_start = source.index("{", signature_start)
    body_end = _matching_closing_brace(source, body_start)
    return source[body_start + 1 : body_end]


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

    assert config["braking_follow_enabled"] is True
    assert config["braking_follow_max_distance_m"] > config["lookahead_s_m"]
    # Race-speedの遅い前走車も、相対速度/TTC/物理制動距離が危険なら早期
    # braking-followへ入れる。停止車限定の1.0 m/sではD2相当(約2.5 m/s)を
    # 制動包絡から落としてしまう。
    assert config["braking_follow_max_target_speed_mps"] == 10.0
    # TTC包絡は遠距離FOLLOWの早期発動だけを制限する。本番値を変える時は
    # 物理制動距離（Coreで独立にmaxを取る）を置換しないことを明示する。
    assert config["braking_follow_ttc_threshold_sec"] == 3.0
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
    assert (
        params["overtake_short_spatial_horizon_v_max_mps"]
        == "$(var overtake_short_spatial_horizon_v_max_mps)"
    )
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
    assert remaps["input/overtake_plan"] == "$(var input_overtake_plan)"
    assert remaps["output/recovery_control_cmd"] == "$(var output_recovery_control_cmd)"
    assert (
        remaps["output/controller_execution_envelope"]
        == "$(var output_controller_execution_envelope)"
    )
    assert remaps["/pure_pursuit/debug"] == "$(var output_debug)"


def test_aw1_execution_envelope_topics_are_role_separated_in_hybrid_profiles():
    for relative_path in (
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml",
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ):
        root = _parse_launch(relative_path)
        pure_pursuit_includes = [
            element
            for element in root.iter("include")
            if element.attrib.get("file", "").endswith("pure_pursuit.launch.xml")
        ]
        assert len(pure_pursuit_includes) == 2

        include_args = [
            {
                element.attrib.get("name"): element.attrib.get("value")
                for element in include.iter("arg")
            }
            for include in pure_pursuit_includes
        ]
        primary_args = next(
            args for args in include_args if args.get("recovery_mode") != "true"
        )
        recovery_args = next(
            args for args in include_args if args.get("recovery_mode") == "true"
        )

        primary_topic = primary_args["output_controller_execution_envelope"]
        recovery_topic = recovery_args["output_controller_execution_envelope"]
        assert primary_topic == "/hybrid_control/pure_pursuit/execution_envelope"
        assert recovery_topic == "/wall_recovery/pure_pursuit/execution_envelope"
        assert primary_topic != recovery_topic


def test_aw1_execution_envelope_has_no_mux_or_planner_launch_consumer():
    for relative_path in (
        "hybrid_control_mux/launch/hybrid_control_mux.launch.xml",
        "overtake_planner/launch/overtake_planner.launch.xml",
    ):
        root = _parse_launch(relative_path)
        launch_contract_elements = [
            element
            for element in root.iter()
            if element.tag in {"arg", "remap"}
        ]

        assert all(
            "execution_envelope"
            not in " ".join(element.attrib.values())
            for element in launch_contract_elements
        )


def test_aw2_shadow_authority_stays_separate_from_typed_motion_grant():
    submit_root = _aichallenge_submit_root()
    live_authority_sources = (
        submit_root
        / "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py",
    )
    shadow_only_fields = (
        "CandidateExecutionRequest",
        "plan_sample_record_sha256",
    )

    for source_path in live_authority_sources:
        source = source_path.read_text(encoding="utf-8")
        assert all(field not in source for field in shadow_only_fields)
        assert "MotionAuthorityGrant" in source
        assert "_motion_authority_grant_eligible" in source
        assert "candidate_content_sha256" in source

    planner_source = (
        submit_root / "overtake_planner/src/overtake_planner_node.cpp"
    ).read_text(encoding="utf-8")
    assert "plan_msg.aw2_identity_schema_version = 0U" in planner_source
    assert "canonicalizeV2SourceWire" in planner_source
    assert (
        '"/overtake/continuation/shadow/candidate_execution_request"'
        in planner_source
    )
    assert "msg.authority_eligible = false" in planner_source
    assert "rclcpp::QoS(rclcpp::KeepLast(8))" in planner_source
    assert ".best_effort()" in planner_source
    assert ".durability_volatile()" in planner_source
    assert "const std::uint64_t v2_planner_instance_id_" in planner_source
    assert "plan_msg.planner_instance_id = v2_planner_instance_id_" in planner_source
    # Diagnostic-only parallel FOLLOW recheck evidence must remain visible in
    # the planner JSON without becoming an authority input to the Mux.
    assert '\\"parallel_follow_recheck_attempted\\"' in planner_source
    assert '\\"parallel_follow_recheck_reason\\"' in planner_source
    mux_source = (
        submit_root / "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py"
    ).read_text(encoding="utf-8")
    assert '"free_run_source_gap_hold_episode_count"' in mux_source
    assert '"free_run_source_gap_exact_promotion_count"' in mux_source
    assert '"free_run_source_gap_lease_expiry_count"' in mux_source


def test_motion_grant_is_published_before_bound_positive_command():
    source = (
        _aichallenge_submit_root()
        / "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py"
    ).read_text(encoding="utf-8")
    commit_publish = source.index(
        "self._publish_motion_authority_grant(\n"
        "                valid=True,"
    )
    command_publish = source.index(
        "self.control_pub.publish(cmd)", commit_publish
    )
    assert commit_publish < command_publish
    assert "pass_motion_grant_required" in source
    assert "pass_motion_grant_valid" in source
    assert "motion_authority_grant_" in source


def test_aw2_applied_transport_is_shadow_only_and_recovery_gated():
    submit_root = _aichallenge_submit_root()
    supervisor = (
        submit_root
        / "simple_pure_pursuit/src/controller_applied_shadow_supervisor.cpp"
    ).read_text(encoding="utf-8")
    pure_pursuit = (
        submit_root / "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
    ).read_text(encoding="utf-8")
    launch = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    )

    assert "create_subscription" not in supervisor
    assert supervisor.count("fork()") == 1
    assert "authority_eligible = false" in supervisor
    assert "rclcpp::KeepLast(8)" in supervisor
    assert ".best_effort()" in supervisor
    assert ".durability_volatile()" in supervisor
    assert "captureControllerAppliedShadow(" in pure_pursuit
    assert "aw2_shadow_transport_enabled_ && !recovery_mode_" in pure_pursuit
    assert "create_publisher<ControllerAppliedEnvelope>" not in pure_pursuit
    capture_body = _cpp_function_body(
        pure_pursuit,
        "void SimplePurePursuit::captureControllerAppliedShadow(",
    )
    for forbidden in (
        "publish(",
        "RCLCPP_",
        "aw2Sha256",
        "mutex",
        "sleep",
        "wait",
        "socket(",
        "send(",
        "recv(",
        "poll(",
        "new ",
    ):
        assert forbidden not in capture_body
    timer_body = _cpp_function_body(
        pure_pursuit, "void SimplePurePursuit::onTimer()"
    )
    stop_body = _cpp_function_body(
        pure_pursuit,
        "void SimplePurePursuit::publishStopForStaleInput(",
    )
    for body in (timer_body, stop_body):
        raw_publish = (
            "pub_raw_cmd_->publish(raw_cmd);"
            if "pub_raw_cmd_->publish(raw_cmd);" in body
            else "pub_raw_cmd_->publish(cmd);"
        )
        assert body.index("pub_cmd_->publish(cmd);") < body.index(raw_publish)
        assert body.index(raw_publish) < body.index(
            "captureControllerAppliedShadow("
        )
    assert any(
        element.attrib.get("name") == "aw2_shadow_transport_enabled"
        for element in launch.iter("param")
    )

    for relative_path in (
        "hybrid_control_mux/hybrid_control_mux/hybrid_control_mux_node.py",
        "overtake_planner/src/overtake_planner_node.cpp",
    ):
        source = (submit_root / relative_path).read_text(encoding="utf-8")
        assert "controller_applied_envelope" not in source
        assert "controller_applied_transport_status" not in source


def test_aw2_transport_status_schema_is_fully_bounded():
    schema = (
        _aichallenge_submit_root()
        / "multi_purpose_mpc_ros_msgs/msg/ControllerAppliedTransportStatus.msg"
    ).read_text(encoding="utf-8")
    data_lines = [
        line.split("#", 1)[0].strip()
        for line in schema.splitlines()
        if line.split("#", 1)[0].strip()
        and "=" not in line.split("#", 1)[0].strip().split()[-1]
    ]
    assert all(not line.startswith("string ") for line in data_lines)
    assert all("[]" not in line for line in data_lines)
    assert "string<=128 detail" in data_lines
    assert "uint64[<=16] first_dropped_sequence" in data_lines
    assert (
        "multi_purpose_mpc_ros_msgs/ControllerSampleKey[<=16] first_dropped_key"
        in data_lines
    )
    assert "bool authority_eligible" in data_lines


def test_aw2_candidate_execution_request_is_fully_bounded_and_shadow_only():
    submit_root = _aichallenge_submit_root()
    request_schema = (
        submit_root
        / "multi_purpose_mpc_ros_msgs/msg/CandidateExecutionRequest.msg"
    ).read_text(encoding="utf-8")
    data_lines = [
        line.split("#", 1)[0].strip()
        for line in request_schema.splitlines()
        if line.split("#", 1)[0].strip()
        and "=" not in line.split("#", 1)[0].strip().split()[-1]
    ]

    assert all(not line.startswith("string ") for line in data_lines)
    assert all("[]" not in line for line in data_lines)
    assert "string<=128 plan_frame_id" in data_lines
    assert "string<=64 target_vehicle_id" in data_lines
    assert "uint8[<=4096] canonical_source_wire" in data_lines
    assert (
        "multi_purpose_mpc_ros_msgs/CandidateExecutionPoint[<=100] geometry_points"
        in data_lines
    )
    assert "bool authority_eligible" in data_lines


def test_aw2_shadow_request_payload_is_non_authoritative_and_schema_gated():
    planner_source = (
        _aichallenge_submit_root()
        / "overtake_planner/src/overtake_planner_node.cpp"
    ).read_text(encoding="utf-8")
    function_body = _cpp_function_body(
        planner_source, "void publishCandidateExecutionRequest("
    )

    invalid_initialization = (
        "msg.schema_version = Request::SCHEMA_INVALID;"
    )
    schema_v1_assignment = "msg.schema_version = Request::SCHEMA_V1;"
    publish_call = "candidate_execution_request_pub_->publish(msg);"
    assert re.findall(
        r"msg\.authority_eligible\s*=\s*([^;]+);", function_body
    ) == ["false"]
    assert re.findall(
        r"msg\.schema_version\s*=\s*([^;]+);",
        function_body,
    ) == ["Request::SCHEMA_INVALID", "Request::SCHEMA_V1"]
    assert function_body.count(publish_call) == 1

    schema_assignment_index = function_body.index(schema_v1_assignment)
    schema_if_start = function_body.rfind("if (", 0, schema_assignment_index)
    schema_if_body_start = function_body.index("{", schema_if_start)
    assert schema_if_start >= 0
    assert schema_if_body_start < schema_assignment_index
    schema_condition = " ".join(
        function_body[schema_if_start:schema_if_body_start].split()
    )
    expected_schema_condition = (
        "if (plan_msg.aw2_identity_schema_version == 1U && "
        "(observation == aw2::DeliveryObservation::ACCEPTED || "
        "observation == aw2::DeliveryObservation::CONSISTENT_DUPLICATE))"
    )
    assert schema_condition == expected_schema_condition

    canonical_if = "if (canonical.valid())"
    canonical_if_start = function_body.rfind(
        canonical_if, 0, schema_if_start
    )
    assert canonical_if_start >= 0
    canonical_body_start = function_body.index("{", canonical_if_start)
    canonical_body_end = _matching_closing_brace(
        function_body, canonical_body_start
    )
    assert canonical_body_start < schema_if_start
    assert schema_assignment_index < canonical_body_end

    delivery_observe_call = (
        "const auto observation = delivery_tracker.observe("
    )
    delivery_observation_assignment = (
        "msg.delivery_observation = static_cast<std::uint8_t>(observation);"
    )
    delivery_observe_index = function_body.index(delivery_observe_call)
    delivery_observation_index = function_body.index(
        delivery_observation_assignment
    )
    assert canonical_body_start < delivery_observe_index
    assert delivery_observe_index < delivery_observation_index
    assert delivery_observation_index < schema_if_start
    assert schema_assignment_index < canonical_body_end
    assert function_body.index(invalid_initialization) < schema_if_start
    assert schema_assignment_index < function_body.index(publish_call)


def test_aw2_shadow_request_has_no_repo_production_subscriber():
    submit_root = _aichallenge_submit_root()
    planner_source_path = (
        submit_root / "overtake_planner/src/overtake_planner_node.cpp"
    )
    production_suffixes = {
        ".cc",
        ".cpp",
        ".cxx",
        ".h",
        ".hpp",
        ".py",
        ".xml",
        ".yaml",
        ".yml",
    }
    excluded_components = {
        "build",
        "docs",
        "install",
        "log",
        "output",
        "test",
        "tests",
    }
    production_sources = [
        path
        for path in submit_root.rglob("*")
        if path.is_file()
        and path.suffix in production_suffixes
        and not excluded_components.intersection(path.relative_to(submit_root).parts)
    ]

    request_type_users = []
    shadow_topic_users = []
    cpp_subscription_pattern = re.compile(
        r"(?:create_subscription|Subscription)\s*<"
        r"[^>]*CandidateExecutionRequest",
        re.DOTALL,
    )
    python_subscription_pattern = re.compile(
        r"create_subscription\s*\(\s*"
        r"(?:[A-Za-z_][A-Za-z0-9_.]*\.)?CandidateExecutionRequest\b"
    )
    generic_subscription_pattern = re.compile(r"\bcreate_generic_subscription\s*\(")
    shadow_topic = "/overtake/continuation/shadow/candidate_execution_request"

    for source_path in production_sources:
        source = source_path.read_text(encoding="utf-8")
        if "CandidateExecutionRequest" in source:
            request_type_users.append(source_path)
        if shadow_topic in source:
            shadow_topic_users.append(source_path)
        assert cpp_subscription_pattern.search(source) is None, source_path
        assert python_subscription_pattern.search(source) is None, source_path
        assert generic_subscription_pattern.search(source) is None, source_path

    assert request_type_users == [planner_source_path]
    assert shadow_topic_users == [planner_source_path]
    planner_source = planner_source_path.read_text(encoding="utf-8")
    publisher_pattern = re.compile(
        r"create_publisher\s*<[^>]*CandidateExecutionRequest\s*>",
        re.DOTALL,
    )
    assert len(publisher_pattern.findall(planner_source)) == 1


def test_pure_pursuit_launch_uses_mpc_consistent_steering_gain():
    root = _parse_launch("aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml")
    steering_gain_values = [
        element.attrib["value"]
        for element in root.iter("let")
        if element.attrib.get("name") == "steering_tire_angle_gain_var"
    ]

    assert steering_gain_values == ["1.54", "1.54"]


def test_overtake_pass_trackability_uses_active_controller_contract():
    submit_root = _aichallenge_submit_root()
    planner_config = yaml.safe_load(
        (
            submit_root
            / "overtake_planner/config/overtake_planner.param.yaml"
        ).read_text(encoding="utf-8")
    )["overtake_planner_node"]["ros__parameters"]
    mux_config = yaml.safe_load(
        (
            submit_root
            / "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml"
        ).read_text(encoding="utf-8")
    )["hybrid_control_mux_node"]["ros__parameters"]

    pure_pursuit_root = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    )
    pure_pursuit_launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in pure_pursuit_root.findall("arg")
    }
    steering_gains = {
        float(element.attrib["value"])
        for element in pure_pursuit_root.iter("let")
        if element.attrib.get("name") == "steering_tire_angle_gain_var"
    }
    active_control_root = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml"
    )
    pure_pursuit_include = next(
        element
        for element in active_control_root.iter("include")
        if "pure_pursuit.launch.xml" in element.attrib.get("file", "")
        and element.attrib.get("file", "").endswith("pure_pursuit.launch.xml")
    )
    pure_pursuit_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in pure_pursuit_include.iter("arg")
    }

    assert planner_config["attack_follow_tracking_wheelbase_m"] == float(
        pure_pursuit_args["wheel_base"]
    )
    assert steering_gains == {
        planner_config["attack_follow_steering_tire_angle_gain"]
    }
    assert planner_config["attack_follow_max_steering_rate_radps"] == 128.0
    assert planner_config["attack_follow_max_steering_angle_rad"] == 0.64
    assert mux_config["max_steering_angle_rad"] == 0.64
    assert mux_config["tracking_usable_max_steering_angle_rad"] == 0.64
    assert float(
        pure_pursuit_launch_args["free_run_live_exact_hard_steering_limit_rad"]
    ) == 0.64
    assert float(
        pure_pursuit_launch_args[
            "free_run_live_exact_hard_steering_rate_limit_radps"
        ]
    ) == 0.5
    assert planner_config["lateral_override_lookahead_gain"] == float(
        pure_pursuit_args["lookahead_gain"]
    )
    assert planner_config["lateral_override_lookahead_min_distance_m"] == float(
        pure_pursuit_args["lookahead_min_distance"]
    )
    assert pure_pursuit_args["curvature_adaptive_lookahead_enabled"] == "true"
    assert planner_config["lateral_override_lookahead_min_distance_m"] == float(
        pure_pursuit_args["curvature_lookahead_min_distance"]
    )
    assert pure_pursuit_args["horizon_curvature_feedforward_gain"] == "0.0"
    assert pure_pursuit_args["pp_control_delay_sec"] == "0.0"
    assert planner_config["moving_pass_reachability_enabled"] is True
    assert planner_config["moving_pass_max_completion_time_sec"] > 0.0
    assert planner_config["moving_pass_min_closing_speed_mps"] >= 0.0


def test_state_lattice_pp_proxy_matches_actual_pp_and_keeps_instant_independent():
    submit_root = _aichallenge_submit_root()
    state_lattice_config = yaml.safe_load(
        (
            submit_root
            / "state_lattice_overtake_planner/config/"
            "state_lattice_overtake_planner.param.yaml"
        ).read_text(encoding="utf-8")
    )["state_lattice_overtake_planner_node"]["ros__parameters"]
    mux_config = yaml.safe_load(
        (
            submit_root
            / "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml"
        ).read_text(encoding="utf-8")
    )["hybrid_control_mux_node"]["ros__parameters"]
    pure_pursuit_root = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    )
    pure_pursuit_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in pure_pursuit_root.findall("arg")
    }

    assert state_lattice_config["hard_max_steer_rad"] == 0.64
    assert state_lattice_config["planner_max_steer_rad"] == 0.64
    assert state_lattice_config["max_steer_rate_radps"] == 0.5
    assert float(
        pure_pursuit_args["free_run_live_exact_hard_steering_limit_rad"]
    ) == 0.64
    assert float(
        pure_pursuit_args[
            "free_run_live_exact_hard_steering_rate_limit_radps"
        ]
    ) == 0.5
    assert mux_config["max_steering_angle_rad"] == 0.64
    assert mux_config["max_steering_rate_radps"] == 0.5

    # The direct State Lattice command source is a separate, default-off
    # control route. This PP-proxy alignment must not broaden its authority.
    assert state_lattice_config["instant_control_max_steering_angle_rad"] == 0.5236
    assert state_lattice_config["instant_control_max_steering_rate_radps"] == 0.35
    assert mux_config["state_lattice_instant_control_enabled"] is False
    assert mux_config["state_lattice_max_steering_angle_rad"] == 0.5236
    assert mux_config["state_lattice_max_steering_rate_radps"] == 0.35


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
    assert pp_args["output_controller_tracking_status"] == (
        "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert mux_args["input_pure_pursuit_tracking_status"] == (
        "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert mux_args["output_controller_tracking_status"] == (
        "/hybrid_control/controller_tracking_status"
    )
    assert mux_args["output_debug"] == "/hybrid_control_mux/debug"


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
    assert launch_args["mpc_enabled"]["default"] == "true"
    delay_group = next(
        element
        for element in root.findall("group")
        if delay_mpc_include in list(element)
    )
    assert delay_group.attrib.get("if") == "$(var mpc_enabled)"
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
    assert pp_args["use_mpc_predicted_horizon"] == "$(var mpc_enabled)"
    assert pp_args["require_solved_mpc_health_for_horizon"] == (
        "$(var mpc_enabled)"
    )
    assert pp_args["max_mpc_health_age_sec"] == "0.35"
    assert pp_args["require_matching_overtake_horizon_contract"] == (
        "$(var mpc_enabled)"
    )
    assert mux_args["param_file"].endswith("pure_pursuit_mpc_horizon.param.yaml")
    assert mux_args["input_pure_pursuit_cmd"] == (
        "/pure_pursuit_mpc_horizon/pure_pursuit/control_cmd"
    )
    assert mux_args["require_safety_constraint"] == "$(var require_safety_constraint)"
    assert mux_args["input_safety_constraint"] == "/overtake/safety_constraint"
    assert mux_args["input_overtake_plan"] == "/overtake/plan"
    assert pp_args["output_controller_tracking_status"] == (
        "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert mux_args["input_pure_pursuit_tracking_status"] == (
        "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert mux_args["output_controller_tracking_status"] == (
        "/hybrid_control/controller_tracking_status"
    )
    assert mux_args["output_debug"] == "/hybrid_control_mux/debug"


def test_hybrid_mux_exposes_typed_safety_constraint_contract():
    root = _parse_launch("hybrid_control_mux/launch/hybrid_control_mux.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    assert launch_args["free_run_source_provenance_timeout_sec"] == "1.5"
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


def test_vehicle_steering_hard_limit_and_map_cover_064_rad():
    workspace_src = _aichallenge_submit_root().parent
    submit_map = (
        _aichallenge_submit_root()
        / "aichallenge_submit_launch/data/steer_map.csv"
    )
    system_map = (
        workspace_src
        / "aichallenge_system/aichallenge_awsim_adapter/data/steer_map.csv"
    )
    assert submit_map.read_bytes() == system_map.read_bytes()

    with submit_map.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    assert rows
    assert float(rows[0][1]) == -0.64
    assert float(rows[0][-1]) == 0.64
    assert all(len(row) == len(rows[0]) for row in rows)
    for row in rows[1:]:
        values = [float(value) for value in row[1:]]
        assert math.isclose(
            values[0],
            values[1] + 0.4 * (values[1] - values[2]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        assert math.isclose(
            values[-1],
            values[-2] + 0.4 * (values[-2] - values[-3]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )

    adapter_source = (
        workspace_src
        / "aichallenge_system/aichallenge_awsim_adapter/src/"
        "actuation_cmd_converter.cpp"
    ).read_text(encoding="utf-8")
    assert "kMaxSteeringTireAngleRad = 0.64" in adapter_source
    assert "std::clamp(" in adapter_source


def test_hybrid_mux_finish_stop_uses_per_domain_official_state_contract():
    root = _parse_launch("hybrid_control_mux/launch/hybrid_control_mux.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in root.iter("remap")
    }

    assert launch_args["input_race_armed"] == "/overtake/race_armed"
    assert launch_args["input_awsim_state"] == "/awsim/state"
    assert remaps["input/race_armed"] == "$(var input_race_armed)"
    assert remaps["input/awsim_state"] == "$(var input_awsim_state)"

    submit_root = _aichallenge_submit_root()
    for relative_path in (
        "hybrid_control_mux/config/hybrid_control_mux.param.yaml",
        "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml",
    ):
        params = yaml.safe_load(
            (submit_root / relative_path).read_text(encoding="utf-8")
        )["hybrid_control_mux_node"]["ros__parameters"]
        assert params["finish_stop_enabled"] is True
        assert params["finish_stop_decel_mps2"] == -3.2
        assert params["finish_terminal_reference_timeout_sec"] == 0.20
        assert params["finish_stop_max_steering_rad"] == 0.20
        assert params["finish_stop_steering_guard_trigger_rad"] == 0.35
        assert params["lateral_stop_steering_hold_timeout_sec"] == 0.12
        assert params["race_arm_required"] is True


def test_safety_constraint_requirement_has_single_launch_owner():
    root = _aichallenge_submit_root()

    for relative_path in (
        "hybrid_control_mux/config/hybrid_control_mux.param.yaml",
        "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml",
    ):
        config = yaml.safe_load((root / relative_path).read_text(encoding="utf-8"))
        params = config["hybrid_control_mux_node"]["ros__parameters"]

        # A node-specific YAML value wins over the wildcard parameter emitted by
        # the included launch when global vehicle parameters are also present.
        # Keep this key launch-owned so use_overtake_planner is honored in the
        # full aichallenge_system launch, not only in an isolated control launch.
        assert "require_safety_constraint" not in params


def test_hybrid_mux_exposes_typed_controller_tracking_status():
    root = _parse_launch("hybrid_control_mux/launch/hybrid_control_mux.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in root.iter("remap")
    }

    assert (
        launch_args["output_controller_tracking_status"]
        == "/hybrid_control/controller_tracking_status"
    )
    assert (
        launch_args["input_pure_pursuit_tracking_status"]
        == "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert (
        launch_args["input_pure_pursuit_command_envelope"]
        == "/hybrid_control/pure_pursuit/command_envelope"
    )
    assert (
        remaps["input/pure_pursuit_tracking_status"]
        == "$(var input_pure_pursuit_tracking_status)"
    )
    assert (
        remaps["input/pure_pursuit_command_envelope"]
        == "$(var input_pure_pursuit_command_envelope)"
    )
    assert (
        remaps["output/controller_tracking_status"]
        == "$(var output_controller_tracking_status)"
    )

    pure_pursuit_launch = _parse_launch(
        "aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    )
    pure_pursuit_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in pure_pursuit_launch.findall("arg")
    }
    pure_pursuit_remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in pure_pursuit_launch.iter("remap")
    }
    assert (
        pure_pursuit_args["output_controller_tracking_status"]
        == "/hybrid_control/pure_pursuit/tracking_status"
    )
    assert (
        pure_pursuit_remaps["output/controller_tracking_status"]
        == "$(var output_controller_tracking_status)"
    )
    pure_pursuit_source = (
        _aichallenge_submit_root()
        / "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
    ).read_text(encoding="utf-8")
    assert "publishControllerTrackingStatus" in pure_pursuit_source
    assert "last_valid_override_contract_generation_" in pure_pursuit_source
    assert "context->overtake_override_applied" in pure_pursuit_source
    assert "context->applied_horizon_generation" in pure_pursuit_source


def test_overtake_startup_and_free_run_safety_contracts_are_fail_closed():
    submit_root = _aichallenge_submit_root()
    planner_config = yaml.safe_load(
        (submit_root / "overtake_planner/config/overtake_planner.param.yaml").read_text(
            encoding="utf-8"
        )
    )["overtake_planner_node"]["ros__parameters"]
    planner_launch = _parse_launch("overtake_planner/launch/overtake_planner.launch.xml")
    launch_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in planner_launch.findall("arg")
    }
    remaps = {
        element.attrib.get("from"): element.attrib.get("to")
        for element in planner_launch.iter("remap")
    }

    assert planner_config["race_arm_required"] is True
    assert planner_config["race_arm_topic"] == "/overtake/race_armed"
    assert planner_config["input_future_stamp_tolerance_sec"] == 0.05
    assert planner_config["supervisor_v2_target_missing_hold_cycles"] == 2
    assert planner_config["supervisor_v2_tracking_unusable_hold_cycles"] == 2
    assert planner_config["start_grid_tracking_release_timeout_cycles"] == 60
    assert (
        planner_config[
            "start_grid_tracking_continuity_max_mpc_solve_time_ms"
        ]
        == 120.0
    )
    assert planner_config["start_grid_target_enabled"] is True
    assert planner_config["start_grid_target_window_sec"] > 0.0
    assert planner_config["start_grid_target_window_distance_m"] > 0.0
    assert planner_config["start_grid_target_min_delta_s_m"] < 0.0
    assert planner_config["start_grid_target_max_delta_s_m"] > 0.0
    assert planner_config["start_grid_attack_follow_v_max_mps"] <= 3.0
    assert planner_config["reentry_completion_lateral_error_m"] == 0.20
    assert planner_config["reentry_completion_rearm_lateral_error_m"] == 0.30
    assert (
        planner_config["reentry_completion_rearm_lateral_error_m"]
        > planner_config["reentry_completion_lateral_error_m"]
    )
    assert planner_config["mpc_health_clean_free_run_soft_guard_bypass_enabled"] is False
    assert planner_config["mpc_health_inactive_mpc_free_run_bypass_enabled"] is True
    assert launch_args["input_race_arm"] == "/overtake/race_armed"
    assert (
        launch_args["input_controller_tracking_status"]
        == "/hybrid_control/controller_tracking_status"
    )
    assert remaps["/overtake/race_armed"] == "$(var input_race_arm)"
    assert (
        remaps["/hybrid_control/controller_tracking_status"]
        == "$(var input_controller_tracking_status)"
    )


def test_autostart_initializes_before_ready_but_normal_race_arms_only_at_start():
    system_root = _aichallenge_submit_root().parent / "aichallenge_system"
    config = yaml.safe_load(
        (
            system_root
            / "autostart_orchestrator_py/config/autostart_orchestrator.param.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    assert config["race_arm_topic"] == "/overtake/race_armed"
    assert config["initialization_ready_topic"] == (
        "/autostart/initialization_ready"
    )
    assert set(config["start_on_vehicle_state"].split(",")) == {
        "Grounded",
        "Ready",
    }
    assert "Start" not in config["start_on_vehicle_state"].split(",")
    assert config["race_arm_on_vehicle_state"] == "Start"
    assert set(config["race_arm_neutral_vehicle_states"].split(",")) == {"Ready"}
    disarm_states = set(config["race_disarm_on_vehicle_state"].split(","))
    assert disarm_states >= {
        "Spawned",
        "Grounded",
        "Finish",
    }
    assert "Ready" not in disarm_states
    source = (
        system_root
        / "autostart_orchestrator_py/autostart_orchestrator_py/autostart_orchestrator_node.py"
    ).read_text(encoding="utf-8")
    assert "def _do_start_initialization" in source
    assert ") -> bool:" in source
    assert "class _RaceArmLatch:" in source
    assert "initialization_accepted, initialization_generation" in source
    assert "if initialization_accepted:" in source
    assert "race initialization skipped after terminal/reset state" in source
    assert "self._complete_race_start_initialization(" in source
    assert "return self._armed" in source
    assert "discarding stale race initialization result" in source
    assert "race arm remains false" in source

    evaluation_launch = ET.parse(
        system_root
        / "aichallenge_system_launch/launch/evaluation.launch.xml"
    ).getroot()
    evaluation_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in evaluation_launch.findall("arg")
    }
    assert evaluation_args["race_arm_on_vehicle_state"] == "Start"
    evaluation_system_include = next(
        element
        for element in evaluation_launch.iter("include")
        if element.attrib.get("file", "").endswith(
            "/launch/aichallenge_system.launch.xml"
        )
    )
    evaluation_include_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in evaluation_system_include.findall("arg")
    }
    assert evaluation_include_args["race_arm_on_vehicle_state"] == (
        "$(var race_arm_on_vehicle_state)"
    )
    state_manager_include = next(
        element
        for element in evaluation_launch.iter("include")
        if element.attrib.get("file", "").endswith(
            "/launch/mode/awsim_state_manager.launch.xml"
        )
    )
    state_manager_include_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in state_manager_include.findall("arg")
    }
    assert state_manager_include_args["admin_start_enabled"] == "false"
    start_helper = next(
        element
        for element in evaluation_launch.iter("executable")
        if element.attrib.get("name") == "request_awsim_start"
    )
    helper_command = start_helper.attrib["cmd"]
    assert "ROS_DOMAIN_ID=0" in helper_command
    assert "AWSIM_READY_DOMAINS=$(var domain_id)" in helper_command
    assert "AWSIM_START_MODE=$(var awsim_start_mode)" in helper_command
    assert "/aichallenge/request_awsim_start.bash" in helper_command

    state_manager_launch = ET.parse(
        system_root
        / "aichallenge_system_launch/launch/mode/awsim_state_manager.launch.xml"
    ).getroot()
    state_manager_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in state_manager_launch.findall("arg")
    }
    assert state_manager_args["admin_start_enabled"] == "true"
    state_manager_params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in state_manager_launch.iter("param")
    }
    assert state_manager_params["admin_start_enabled"] == (
        "$(var admin_start_enabled)"
    )

    system_launch = ET.parse(
        system_root
        / "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    ).getroot()
    system_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in system_launch.findall("arg")
    }
    assert system_args["race_arm_on_vehicle_state"] == "Start"
    awsim_include = next(
        element
        for element in system_launch.iter("include")
        if element.attrib.get("file", "").endswith("/launch/mode/awsim.launch.xml")
    )
    include_args = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in awsim_include.findall("arg")
    }
    assert include_args["race_arm_on_vehicle_state"] == (
        "$(var race_arm_on_vehicle_state)"
    )

    awsim_launch = ET.parse(
        system_root / "aichallenge_system_launch/launch/mode/awsim.launch.xml"
    ).getroot()
    awsim_args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in awsim_launch.findall("arg")
    }
    assert awsim_args["race_arm_on_vehicle_state"] == "Start"
    node_params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in awsim_launch.iter("param")
    }
    assert node_params["race_arm_on_vehicle_state"] == (
        "$(var race_arm_on_vehicle_state)"
    )

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


def test_state_lattice_instant_source_is_typed_wired_and_default_off():
    submit_root = _aichallenge_submit_root()
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
    assert launch_args["state_lattice_instant_control_enabled"] == "false"
    assert (
        launch_args["input_state_lattice_control_cmd"]
        == "/hybrid_control/state_lattice/control_cmd"
    )
    assert (
        params["state_lattice_instant_control_enabled"]
        == "$(var state_lattice_instant_control_enabled)"
    )
    assert (
        remaps["input/state_lattice_control_cmd"]
        == "$(var input_state_lattice_control_cmd)"
    )

    for relative_path in (
        "hybrid_control_mux/config/hybrid_control_mux.param.yaml",
        "hybrid_control_mux/config/pure_pursuit_mpc_horizon.param.yaml",
    ):
        config = yaml.safe_load(
            (submit_root / relative_path).read_text(encoding="utf-8")
        )["hybrid_control_mux_node"]["ros__parameters"]
        assert config["state_lattice_instant_control_enabled"] is False
        assert config["state_lattice_cmd_timeout_sec"] == 0.12
        assert config["state_lattice_max_steering_angle_rad"] == 0.5236
        assert config["state_lattice_max_steering_rate_radps"] == 0.35


def test_submit_launch_does_not_expose_overtake_control_route():
    submit = _parse_launch(
        "aichallenge_submit_launch/launch/aichallenge_submit.launch.xml"
    )
    assert not any(
        arg.attrib.get("name") == "overtake_control_route"
        for arg in submit.findall("arg")
    )
    reference_include = next(
        include
        for include in submit.iter("include")
        if "reference.launch.xml" in include.attrib.get("file", "")
    )
    reference_args = {
        arg.attrib.get("name"): arg.attrib.get("value")
        for arg in reference_include.findall("arg")
    }
    assert "overtake_control_route" not in reference_args

    reference = _parse_launch(
        "aichallenge_submit_launch/launch/reference.launch.xml"
    )
    route_let = next(
        item
        for item in reference.findall("let")
        if item.attrib.get("name") == "overtake_control_route"
    )
    assert "resolve_overtake_control_route" in route_let.attrib["value"]
    for profile in (
        "hybrid_delay_aware_mpc.launch.xml",
        "pure_pursuit_mpc_horizon.launch.xml",
    ):
        include = next(
            candidate
            for candidate in reference.iter("include")
            if profile in candidate.attrib.get("file", "")
        )
        args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in include.findall("arg")
        }
        assert args["overtake_control_route"] == "$(var overtake_control_route)"


def test_system_entrypoint_does_not_expose_overtake_control_route():
    workspace_src = Path(__file__).resolve().parents[3]
    system_launch = ET.parse(
        workspace_src
        / "aichallenge_system"
        / "aichallenge_system_launch"
        / "launch"
        / "aichallenge_system.launch.xml"
    ).getroot()
    assert not any(
        arg.attrib.get("name") == "overtake_control_route"
        for arg in system_launch.findall("arg")
    )
    submit_include = next(
        include
        for include in system_launch.iter("include")
        if "aichallenge_submit.launch.xml" in include.attrib.get("file", "")
    )
    submit_args = {
        arg.attrib.get("name"): arg.attrib.get("value")
        for arg in submit_include.findall("arg")
    }
    assert "overtake_control_route" not in submit_args


@pytest.mark.parametrize(
    "relative_path",
    (
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml",
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ),
)
def test_three_way_overtake_route_keeps_single_writer_and_mux_contract(
    relative_path,
):
    root = _parse_launch(relative_path)
    lets = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.findall("let")
    }
    assert "'$(var overtake_control_route)' == 'current'" in (
        lets["current_overtake_planner_enabled"]
    )
    assert "'$(var overtake_control_route)' == 'state_lattice_pure_pursuit'" in (
        lets["state_lattice_pure_pursuit_enabled"]
    )
    assert "'$(var overtake_control_route)' == 'state_lattice_instant_mux'" in (
        lets["state_lattice_instant_enabled"]
    )

    state_lattice_bindings = set()
    for include in root.iter("include"):
        if "state_lattice_overtake_planner.launch.xml" not in include.attrib.get(
            "file", ""
        ):
            continue
        args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in include.findall("arg")
        }
        state_lattice_bindings.add(
            (
                args["live_control_output_enabled"],
                args["instant_control_enabled"],
                args["controller_trackability_profile"],
            )
        )
    assert state_lattice_bindings == {
        ("true", "false", "pure_pursuit"),
    }
    simple_includes = [
        include
        for include in root.iter("include")
        if "simple_state_lattice_planner.launch.xml"
        in include.attrib.get("file", "")
    ]
    assert len(simple_includes) == 1
    simple_args = {
        arg.attrib.get("name"): arg.attrib.get("value")
        for arg in simple_includes[0].findall("arg")
    }
    assert simple_args["live_control_output_enabled"] == "true"
    assert simple_args["safety_evaluation_enabled"] == "true"
    assert simple_args["output_control_cmd"] == (
        "/hybrid_control/state_lattice/control_cmd"
    )

    pure_pursuit_include = next(
        include
        for include in root.iter("include")
        if "pure_pursuit.launch.xml" in include.attrib.get("file", "")
        and not any(
            arg.attrib.get("name") == "node_name"
            and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
            for arg in include.findall("arg")
        )
    )
    pure_pursuit_args = {
        arg.attrib.get("name"): arg.attrib.get("value")
        for arg in pure_pursuit_include.findall("arg")
    }
    assert pure_pursuit_args["use_overtake_reference_override"] == (
        "$(var reference_override_enabled)"
    )
    assert pure_pursuit_args["input_overtake_reference_override"] == (
        "/overtake/reference_override"
    )

    mux_include = next(
        include
        for include in root.iter("include")
        if "hybrid_control_mux.launch.xml" in include.attrib.get("file", "")
    )
    mux_args = {
        arg.attrib.get("name"): arg.attrib.get("value")
        for arg in mux_include.findall("arg")
    }
    expected_safety_binding = (
        "$(var require_safety_constraint)"
        if relative_path.endswith("pure_pursuit_mpc_horizon.launch.xml")
        else "$(var current_overtake_planner_enabled)"
    )
    assert mux_args["require_safety_constraint"] == expected_safety_binding
    assert mux_args["state_lattice_instant_control_enabled"] == (
        "$(var state_lattice_instant_enabled)"
    )
    assert mux_args["input_state_lattice_control_cmd"] == (
        "/hybrid_control/state_lattice/control_cmd"
    )


def test_state_lattice_standalone_launch_is_shadow_only_by_default():
    root = _parse_launch(
        "state_lattice_overtake_planner/launch/"
        "state_lattice_overtake_planner.launch.xml"
    )
    args = {
        element.attrib.get("name"): element.attrib.get("default")
        for element in root.findall("arg")
    }
    params = {
        element.attrib.get("name"): element.attrib.get("value")
        for element in root.iter("param")
    }

    assert args["live_control_output_enabled"] == "false"
    assert args["instant_control_enabled"] == "false"
    assert args["controller_trackability_profile"] == "shadow_only"
    assert params["controller_trackability_profile"] == (
        "$(var controller_trackability_profile)"
    )


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


def test_hybrid_profiles_isolate_primary_and_recovery_command_envelopes():
    for relative_path in (
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml",
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ):
        root = _parse_launch(relative_path)
        includes = [
            element
            for element in root.iter("include")
            if "pure_pursuit.launch.xml" in element.attrib.get("file", "")
        ]
        primary_include = next(
            element
            for element in includes
            if not any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in element.iter("arg")
            )
        )
        recovery_include = next(
            element
            for element in includes
            if any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in element.iter("arg")
            )
        )
        primary_args = {
            element.attrib.get("name"): element.attrib.get("value")
            for element in primary_include.iter("arg")
        }
        recovery_args = {
            element.attrib.get("name"): element.attrib.get("value")
            for element in recovery_include.iter("arg")
        }

        assert primary_args["output_controller_command_envelope"] == (
            "/hybrid_control/pure_pursuit/command_envelope"
        )
        assert recovery_args["output_controller_command_envelope"] == (
            "/wall_recovery/pure_pursuit/command_envelope"
        )
        mux_include = next(
            element
            for element in root.iter("include")
            if "hybrid_control_mux.launch.xml" in element.attrib.get("file", "")
        )
        mux_args = {
            element.attrib.get("name"): element.attrib.get("value")
            for element in mux_include.iter("arg")
        }
        assert mux_args["input_pure_pursuit_command_envelope"] == (
            "/hybrid_control/pure_pursuit/command_envelope"
        )


def test_hybrid_profiles_forward_free_run_live_exact_flags_and_topics():
    for relative_path in (
        "aichallenge_submit_launch/launch/control/hybrid_delay_aware_mpc.launch.xml",
        "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml",
    ):
        root = _parse_launch(relative_path)
        launch_args = {
            element.attrib.get("name"): element.attrib.get("default")
            for element in root.findall("arg")
        }
        assert launch_args["free_run_live_exact_observe_enabled"] == "true"
        assert (
            launch_args["free_run_live_exact_pre_ack_hold_enabled"] == "true"
        )

        pure_pursuit_includes = [
            element
            for element in root.iter("include")
            if element.attrib.get("file", "").endswith("pure_pursuit.launch.xml")
        ]
        primary_include = next(
            element
            for element in pure_pursuit_includes
            if not any(
                arg.attrib.get("name") == "recovery_mode"
                and arg.attrib.get("value") == "true"
                for arg in element.iter("arg")
            )
        )
        primary_args = {
            element.attrib.get("name"): element.attrib.get("value")
            for element in primary_include.iter("arg")
        }
        assert primary_args["free_run_live_exact_ack_enabled"] == (
            "$(var free_run_live_exact_observe_enabled)"
        )
        assert primary_args["output_free_run_execution_ack"] == (
            "/hybrid_control/pure_pursuit/free_run_execution_ack"
        )

        mux_include = next(
            element
            for element in root.iter("include")
            if element.attrib.get("file", "").endswith(
                "hybrid_control_mux.launch.xml"
            )
        )
        mux_args = {
            element.attrib.get("name"): element.attrib.get("value")
            for element in mux_include.iter("arg")
        }
        assert mux_args["free_run_live_exact_observe_enabled"] == (
            "$(var free_run_live_exact_observe_enabled)"
        )
        assert mux_args["free_run_live_exact_pre_ack_hold_enabled"] == (
            "$(var free_run_live_exact_pre_ack_hold_enabled)"
        )
        assert mux_args["input_free_run_execution_ack"] == (
            "/hybrid_control/pure_pursuit/free_run_execution_ack"
        )


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
