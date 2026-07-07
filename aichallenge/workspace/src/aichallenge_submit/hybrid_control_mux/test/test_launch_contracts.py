from pathlib import Path
import xml.etree.ElementTree as ET


def _aichallenge_submit_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _parse_launch(relative_path: str) -> ET.Element:
    return ET.parse(_aichallenge_submit_root() / relative_path).getroot()


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
    assert remaps["input/steering_status"] == "$(var input_steering_status)"


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
    assert delay_args["input_control_cmd_raw"] == "/hybrid_control/mpc/control_cmd_raw"
    assert pp_args["input_mpc_predicted_horizon"] == "/hybrid_control/mpc/predicted_horizon"
    assert pp_args["input_steering_status"] == "/vehicle/status/steering_status"
    assert pp_args["use_mpc_predicted_horizon"] == "true"
    assert pp_args["max_mpc_horizon_age_sec"] == "0.15"
    assert pp_args["pp_control_delay_sec"] == "0.0"
    assert pp_args["steering_time_constant_sec"] == "0.30"
    assert pp_args["horizon_curvature_feedforward_gain"] == "0.0"
