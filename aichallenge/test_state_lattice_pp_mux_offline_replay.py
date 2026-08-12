import importlib.util
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

import pytest


HERE = Path(__file__).resolve().parent
REPO = HERE.parent
LAUNCH = HERE / "workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/test_only/state_lattice_pp_mux_offline_replay.launch.xml"
HARNESS = HERE / "run_state_lattice_pp_mux_offline_replay.bash"
CONTRACT = HERE / "state_lattice_pp_mux_offline_contract.py"
MIRROR = REPO / "tools/scripts/headless_overrides"

spec = importlib.util.spec_from_file_location("offline_contract", CONTRACT)
offline_contract = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(offline_contract)


def write_metadata(path: Path, topics: dict[str, tuple[str, int]]) -> None:
    lines = ["rosbag2_bagfile_information:", "  topics_with_message_count:"]
    for name, (msg_type, count) in topics.items():
        lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {msg_type}",
                "      message_count: " + str(count),
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_headless_mirrors_are_exact() -> None:
    pairs = [
        (HARNESS, MIRROR / "aichallenge/run_state_lattice_pp_mux_offline_replay.bash"),
        (CONTRACT, MIRROR / "aichallenge/state_lattice_pp_mux_offline_contract.py"),
        (LAUNCH, MIRROR / "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/test_only/state_lattice_pp_mux_offline_replay.launch.xml"),
        (REPO / "Makefile", MIRROR / "Makefile"),
    ]
    for canonical, mirror in pairs:
        assert canonical.read_bytes() == mirror.read_bytes()


def test_launch_is_private_and_uses_production_nodes() -> None:
    root = ET.parse(LAUNCH).getroot()
    group = next(root.iter("group"))
    assert group.findall("remap") == []
    group_remaps = {
        item.attrib["from"]: item.attrib["to"]
        for item in group.findall("set_remap")
    }
    assert group_remaps["/clock"] == "$(var root)/input/clock"
    assert group_remaps["/tf"] == "$(var root)/input/tf"
    assert group_remaps["/tf_static"] == "$(var root)/input/tf_static"
    nodes = {node.attrib["name"]: node for node in root.iter("node")}
    lattice = nodes["state_lattice_overtake_planner_node"]
    assert lattice.attrib["pkg"] == "state_lattice_overtake_planner"
    lattice_params = {p.attrib.get("name"): p.attrib.get("value") for p in lattice.findall("param")}
    assert lattice_params["live_control_output_enabled"] == "$(var exact_cartesian_enabled)"
    assert lattice_params["experimental_spatial_reference_override_live_publish_enabled"] == "true"
    assert lattice_params["instant_control_enabled"] == "false"
    assert lattice_params["controller_trackability_profile"] == "$(var controller_trackability_profile)"
    assert lattice_params["own_vehicle_id"] == "$(var own_vehicle_id)"
    assert lattice_params["state_lattice_v2_live_proposal_publish_enabled"] == "$(var exact_cartesian_enabled)"
    assert lattice_params["state_lattice_v2_base_attestation_accept_enabled"] == "$(var exact_cartesian_enabled)"
    assert lattice_params["ego_state_topic"].startswith("$(var root)/")
    assert lattice_params["opponent_topic"].startswith("$(var root)/")
    assert lattice_params["mpc_health_topic"].startswith("$(var root)/")
    lattice_remaps = {
        item.attrib["from"]: item.attrib["to"]
        for item in lattice.findall("remap")
    }
    assert lattice_remaps["/overtake/reference_override"] == (
        "$(var root)/planner/reference_override"
    )

    planner = nodes["test_only_offline_overtake_planner"]
    assert planner.attrib["pkg"] == "overtake_planner"
    assert planner.attrib["if"] == "$(var use_current_authority)"
    planner_params = {
        item.attrib.get("name"): item.attrib.get("value")
        for item in planner.findall("param")
    }
    assert planner_params["own_vehicle_id"] == "$(var own_vehicle_id)"
    planner_destinations = {r.attrib["to"] for r in planner.findall("remap")}
    assert "$(var root)/planner/plan" in planner_destinations
    assert "$(var root)/planner/safety_constraint" in planner_destinations

    actuator = nodes["test_only_offline_steering_actuator"]
    assert actuator.attrib["pkg"] == "aichallenge_submit_launch"
    assert actuator.attrib["if"] == "$(var steering_feedback_enabled)"
    actuator_params = {
        item.attrib.get("name"): item.attrib.get("value")
        for item in actuator.findall("param")
    }
    assert actuator_params["input_control_topic"] == "$(var root)/output/control_cmd"
    assert actuator_params["output_steering_topic"] == "$(var root)/input/steering_status"
    assert float(actuator_params["max_steering_rate_radps"]) > 0.0
    assert float(actuator_params["max_steering_angle_rad"]) > 0.0

    mux = nodes["test_only_offline_hybrid_control_mux"]
    assert mux.attrib["pkg"] == "hybrid_control_mux"
    mux_destinations = {r.attrib["to"] for r in mux.findall("remap")}
    assert "$(var root)/mux/motion_authority_grant" in mux_destinations
    assert "$(var root)/output/control_cmd" in mux_destinations

    pp = next(root.iter("include"))
    pp_args = {arg.attrib["name"]: arg.attrib["value"] for arg in pp.findall("arg")}
    assert pp_args["input_overtake_reference_override"] == "$(var root)/planner/reference_override"
    assert pp_args["output_free_run_execution_ack"] == "$(var root)/pp/free_run_execution_ack"
    assert pp_args["use_sim_time"] == "$(var use_sim_time)"

    for element in root.iter():
        destination = element.attrib.get("to") or element.attrib.get("value", "")
        if destination.startswith("/"):
            assert destination.startswith("/test_only/offline_replay/")


def test_ros2_launch_frontend_accepts_combined_xml_without_starting_nodes() -> None:
    ros_setup = Path("/opt/ros/humble/setup.bash")
    workspace_setup = HERE / "install/setup.bash"
    if not ros_setup.is_file() or not workspace_setup.is_file():
        pytest.skip("ROS 2 installed workspace is required for frontend parse test")
    command = " && ".join(
        (
            f"source {shlex.quote(str(ros_setup))}",
            f"source {shlex.quote(str(workspace_setup))}",
            "ros2 launch --noninteractive --show-args "
            + shlex.quote(str(LAUNCH)),
        )
    )
    result = subprocess.run(
        ["bash", "-lc", command],
        capture_output=True,
        check=False,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "topic_token" in result.stdout


def test_harness_is_bounded_localhost_only_and_exact() -> None:
    source = HARNESS.read_text(encoding="utf-8")
    for required in (
        "cyclonedds_loopback_profile_required",
        "cyclonedds_interface_count_not_one",
        "cyclonedds_not_loopback_only",
        "export ROS_LOCALHOST_ONLY=0",
        "replay_domain_overlaps_live_domain",
        "dedicated_domain_not_unused",
        "ros_graph_before.txt",
        "ros_graph_active.txt",
        "ros_graph_after.txt",
        "ros_graph_recording.txt",
        "ros_graph_playing.txt",
        "setsid timeout --kill-after=2",
        "process_group_residue",
        "assert_exact_nodes launch",
        "assert_exact_nodes recording",
        "assert_exact_nodes playing",
        "assert_all_topics_private",
        "ros2 service list -t",
        "ros2 action list -t",
        "live_${kind}_interface",
        "publisher_identity_mismatch",
        "subscriber_identity_mismatch",
        'assert_topic_contract plan "${PRIVATE_ROOT}/planner/plan" 0',
        'assert_topic_contract safety "${PRIVATE_ROOT}/planner/safety_constraint" 0',
        'assert_topic_contract pp_ack',
        'assert_topic_contract mux_grant',
        'assert_topic_contract final_control',
        'assert_topic_contract steering_feedback',
        'TEST_ONLY_REPLAY_STEERING_FEEDBACK_ENABLED',
        'steering feedback requires exact Cartesian mode',
        'expected_nodes+=(/test_only_offline_steering_actuator)',
        "recorder_node=/rosbag2_recorder",
        "assert_all_topics_private /events/write_split",
        '"/tf:=${PRIVATE_ROOT}/input/tf"',
        '"/tf_static:=${PRIVATE_ROOT}/input/tf_static"',
        '"__node:=test_only_offline_player"',
        '"__ns:=${PRIVATE_ROOT}/player"',
        '"/events/read_split:=${PRIVATE_ROOT}/player/events/read_split"',
        '"/rosbag2_player/status:=${PRIVATE_ROOT}/player/status"',
        "player_exited_before_graph_audit",
        'play_command+=(--remap)',
        'play_command=(ros2 bag play "${SOURCE_BAG}" --topics)',
    ):
        assert required in source
    assert "aic tool" not in source
    assert "ros2 topic pub" not in source
    assert "ros2 service call" not in source


def test_contract_baseline_and_regenerated_require_real_chain(tmp_path: Path) -> None:
    source_topics = {
        topic: (msg_type, 0 if not required else 1)
        for topic, (msg_type, required) in offline_contract.SOURCE_TOPICS.items()
    }
    source_metadata = tmp_path / "source.yaml"
    source_mcap = tmp_path / "source.mcap"
    config = tmp_path / "config.yaml"
    write_metadata(source_metadata, source_topics)
    source_mcap.write_bytes(b"immutable bag")
    config.write_text("config", encoding="utf-8")
    validated = offline_contract.validate_source(source_metadata, source_mcap, {"config": config})
    assert validated["errors"] == []
    assert validated["source_hashes"]["mcap_sha256"]

    root = "/test_only/offline_replay/run_unit"
    names = offline_contract.output_topic_names(root)
    baseline_metadata = tmp_path / "baseline.yaml"
    write_metadata(
        baseline_metadata,
        {names["planner_reference_override"]: ("std_msgs/msg/String", 1)},
    )
    assert offline_contract.validate_output(baseline_metadata, root, "baseline")["errors"] == []

    regenerated_metadata = tmp_path / "regenerated.yaml"
    write_metadata(
        regenerated_metadata,
        {topic: ("std_msgs/msg/String", 1) for topic in names.values()},
    )
    assert offline_contract.validate_output(regenerated_metadata, root, "regenerated")["errors"] == []

    missing_execution_envelope = dict(
        (topic, ("std_msgs/msg/String", 1)) for topic in names.values()
    )
    missing_execution_envelope[names["pp_execution_envelope"]] = (
        "std_msgs/msg/String",
        0,
    )
    write_metadata(regenerated_metadata, missing_execution_envelope)
    errors = offline_contract.validate_output(regenerated_metadata, root, "regenerated")["errors"]
    assert "required_regenerated_output_absent:pp_execution_envelope" in errors
