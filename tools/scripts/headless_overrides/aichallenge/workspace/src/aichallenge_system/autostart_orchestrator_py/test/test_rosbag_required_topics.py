import ast
import inspect
import textwrap
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml


def _load_orchestrator_class():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    )
    spec = spec_from_file_location("autostart_orchestrator_node", node_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.AutostartOrchestrator


AutostartOrchestrator = _load_orchestrator_class()


def test_required_topics_parameter_is_explicitly_a_string_array():
    tree = ast.parse(textwrap.dedent(inspect.getsource(AutostartOrchestrator.__init__)))
    matching_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and len(node.args) >= 2
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "rosbag_required_nonempty_topics"
    ]

    assert len(matching_calls) == 1
    parameter_type = matching_calls[0].args[1]
    assert isinstance(parameter_type, ast.Attribute)
    assert parameter_type.attr == "STRING_ARRAY"
    assert isinstance(parameter_type.value, ast.Attribute)
    assert parameter_type.value.attr == "Type"
    assert isinstance(parameter_type.value.value, ast.Name)
    assert parameter_type.value.value.id == "Parameter"


def test_required_rosbag_topics_accept_nonempty_topic():
    missing = AutostartOrchestrator._missing_required_rosbag_topics(
        ["/hybrid_control/controller_tracking_status"],
        {"/hybrid_control/controller_tracking_status": 42},
    )

    assert missing == []


def test_required_rosbag_topics_reject_missing_or_empty_topics():
    missing = AutostartOrchestrator._missing_required_rosbag_topics(
        [
            "/hybrid_control/controller_tracking_status",
            "/overtake/v2/shadow/plan",
        ],
        {"/hybrid_control/controller_tracking_status": 0},
    )

    assert missing == [
        "/hybrid_control/controller_tracking_status",
        "/overtake/v2/shadow/plan",
    ]


def test_default_required_topics_are_a_nonempty_subset_of_recorded_topics():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config/autostart_orchestrator.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]

    required = config["rosbag_required_nonempty_topics"]
    recorded = config["rosbag_topics"]
    assert required == [
        "/hybrid_control/controller_tracking_status",
        "/hybrid_control/pure_pursuit/command_envelope",
        "/hybrid_control/pure_pursuit/execution_envelope",
        "/hybrid_control_mux/debug",
        "/debug/overtake/metrics",
    ]
    assert set(required).issubset(recorded)
    assert "/hybrid_control/pure_pursuit/execution_envelope" in recorded
    assert "/hybrid_control/motion_authority_grant" in recorded
    assert "/hybrid_control/motion_authority_grant" not in required
    assert (
        "/hybrid_control/pure_pursuit/controller_applied_envelope"
        in recorded
    )
    assert (
        "/hybrid_control/pure_pursuit/controller_applied_transport_status"
        in recorded
    )
    assert "/hybrid_control/pure_pursuit/free_run_execution_ack" in recorded
    assert "/hybrid_control/pure_pursuit/free_run_source_key" in recorded
    assert (
        "/hybrid_control/pure_pursuit/controller_applied_envelope"
        not in required
    )
    assert (
        "/hybrid_control/pure_pursuit/controller_applied_transport_status"
        not in required
    )
    assert "/wall_recovery/pure_pursuit/execution_envelope" not in required
    assert "/debug/overtake/metrics" in recorded
    assert (
        "/overtake/continuation/shadow/candidate_execution_request"
        in recorded
    )
    assert "/sensing/lidar/scan" in recorded


def test_default_topics_capture_debug_wall_contact_and_pose_evidence_only():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config/autostart_orchestrator.param.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    recorded = set(config["rosbag_topics"])

    assert {
        "/awsim/ground_truth/on_collision",
        "/awsim/ground_truth/vehicle/pose",
        "/delay_aware_mpc/localization/kinematic_state",
        "/localization/kinematic_state",
        "/localization/pose_with_covariance",
        "/sensing/gnss/pose",
        "/sensing/gnss/pose_with_covariance",
        "/sensing/gnss/nav_sat_fix",
        "/localization/imu_gnss_poser/pose_with_covariance",
        "/tf",
        "/tf_static",
    }.issubset(recorded)
    assert "/awsim/ground_truth/vehicle/pose" not in config[
        "rosbag_required_nonempty_topics"
    ]
