#!/usr/bin/env python3
"""Static NO-LIVE guards for C-002AY-0 and C-002AY1 observers."""

from pathlib import Path
import sys


FORBIDDEN = (
    "AuthorizedCartesianTrajectory",
    "ControllerBaseTrajectorySnapshot",
    "CartesianTrajectoryApplicationStatus",
    "/control/overtake/cartesian",
)
LIVE_SUFFIXES = {".cpp", ".hpp", ".py", ".xml"}
C002AY1_IDENTIFIERS = (
    "c002ay1_prod_measure",
    "C002ay1Runtime",
    "C002AY1_RUNTIME",
    "c002ay1_runtime_observer",
)
C002AY1_AUTHORITY_ROOTS = (
    ("aichallenge_submit", "hybrid_control_mux"),
    ("aichallenge_submit", "multi_purpose_mpc_ros"),
    ("aichallenge_submit", "wall_recovery_planner"),
    ("aichallenge_system", "autostart_orchestrator_py"),
    ("aichallenge_system", "aichallenge_awsim_adapter"),
)
C002AY1_HOT_PATH_FORBIDDEN = (
    "create_publisher",
    "create_subscription",
    "create_service",
    "create_client",
    "rclcpp::Publisher",
    "rclcpp::Subscription",
    "SHA256",
    "EVP_Digest",
    "serialize",
    "publish(",
    "std::mutex",
    "condition_variable",
    "sleep_for",
    "waitpid",
    "fork(",
    "posix_spawn",
    "kill(",
)
C002AY0_HOT_PATH_FORBIDDEN = (
    "SHA256",
    "EVP_Digest",
    "serialize",
    "create_publisher",
    "publish(",
    "std::mutex",
    "condition_variable",
    "sleep_for",
    "waitpid",
    "posix_spawn",
    "kill(",
)


def main() -> int:
    submit_root = Path(sys.argv[1]).resolve()
    workspace_src = submit_root.parent
    contract_root = submit_root / "overtake_transport_contract"
    allowed_roots = (
        submit_root / "multi_purpose_mpc_ros_msgs",
        contract_root,
    )
    allowed_files = {
        submit_root
        / "aichallenge_submit_launch/test/c002ay0_pp_runtime_measurement.py",
        submit_root
        / "aichallenge_submit_launch/test/test_c002ay0_reanchor_contract.py",
        submit_root
        / "simple_pure_pursuit/include/simple_pure_pursuit/"
        "state_lattice_shadow_binding.hpp",
        submit_root
        / "simple_pure_pursuit/include/simple_pure_pursuit/"
        "simple_pure_pursuit.hpp",
        submit_root
        / "simple_pure_pursuit/src/state_lattice_shadow_binding.cpp",
        submit_root
        / "simple_pure_pursuit/src/simple_pure_pursuit.cpp",
        # Phase-1 V2 uses a bounded typed, non-authoritative live attestation.
        # The legacy fixed-record shadow transport remains covered separately.
        submit_root
        / "simple_pure_pursuit/include/simple_pure_pursuit/ay0_base_capture.hpp",
        submit_root / "simple_pure_pursuit/src/ay0_base_capture.cpp",
        submit_root / "simple_pure_pursuit/test/test_lookahead.cpp",
        submit_root
        / "simple_pure_pursuit/test/test_state_lattice_shadow_binding.cpp",
        submit_root
        / "state_lattice_overtake_planner/include/"
        "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp",
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal.cpp",
        submit_root
        / "state_lattice_overtake_planner/include/"
        "state_lattice_overtake_planner/c002ay0_shadow_proposal_capture.hpp",
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal_capture.cpp",
        submit_root
        / "state_lattice_overtake_planner/include/"
        "state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp",
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal_record.cpp",
        submit_root
        / "state_lattice_overtake_planner/include/"
        "state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp",
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal_worker.cpp",
        submit_root
        / "state_lattice_overtake_planner/src/"
        "c002ay0_state_lattice_shadow_worker.cpp",
        submit_root
        / "state_lattice_overtake_planner/src/"
        "state_lattice_overtake_planner_node.cpp",
        submit_root
        / "state_lattice_overtake_planner/test/"
        "test_c002ay0_shadow_proposal.cpp",
    }
    violations: list[str] = []
    for path in submit_root.rglob("*"):
        if (
            not path.is_file()
            or (
                path.suffix not in LIVE_SUFFIXES
                and path.suffix not in {".yaml", ".yml"}
                and path.name != "CMakeLists.txt"
            )
            or any(path.is_relative_to(root) for root in allowed_roots)
            or path in allowed_files
        ):
            continue
        text = path.read_text(encoding="utf-8")
        for token in FORBIDDEN:
            if token in text:
                violations.append(f"{path}:{token}")

    measurement_topic = "/c002ay0_measurement/"
    for root_name in ("src", "include", "launch"):
        root = contract_root / root_name
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and measurement_topic in path.read_text(
                encoding="utf-8"
            ):
                violations.append(f"{path}:test-only measurement topic")

    cmake_text = (contract_root / "CMakeLists.txt").read_text(encoding="utf-8")
    testing_index = cmake_text.find("if(BUILD_TESTING)")
    for executable in (
        "c002ay0_phase1_rmw_measurement",
        "c002ay0_worker_rmw_measurement",
        "c002ay0_exact_proposal_relay",
        "c002ay1_cadence_rmw_measurement",
    ):
        executable_index = cmake_text.find(executable)
        if (
            testing_index < 0
            or executable_index < testing_index
            or f"install(TARGETS {executable}" in cmake_text
        ):
            violations.append(
                f"{contract_root / 'CMakeLists.txt'}:"
                f"{executable} must remain BUILD_TESTING-only and uninstalled"
            )

    # Static 9.1-4: the observer has no authority consumer, ROS endpoint, or
    # feedback path in Mux/MPC/SafetyConstraint/watchdog/E-stop/Finish owners.
    for root_parts in C002AY1_AUTHORITY_ROOTS:
        root = workspace_src.joinpath(*root_parts)
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in LIVE_SUFFIXES:
                continue
            text = path.read_text(encoding="utf-8")
            for token in C002AY1_IDENTIFIERS:
                if token in text:
                    violations.append(f"{path}:{token} authority wiring")

    observer_header = (
        contract_root
        / "include/overtake_transport_contract/c002ay1_runtime_observer.hpp"
    )
    observer_source = contract_root / "src/c002ay1_runtime_observer.cpp"
    observer_test = contract_root / "test/test_c002ay1_runtime_observer.cpp"
    collector_source = contract_root / "test/c002ay1_runtime_collector.cpp"
    required_files = (
        observer_header,
        observer_source,
        observer_test,
        collector_source,
    )
    for path in required_files:
        if not path.is_file():
            violations.append(f"{path}:required C-002AY1 implementation missing")

    # Static 9.2-7: production observer code is local fixed-record IPC only.
    for path in (observer_header, observer_source):
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        for token in C002AY1_HOT_PATH_FORBIDDEN:
            if token in text:
                violations.append(f"{path}:{token} forbidden observer operation")
        if "authority_eligible" in text:
            violations.append(f"{path}:observer must not expose authority")

    # Static 9.8: startup-only, default-off. Runtime parameter callbacks are
    # forbidden; recovery PP must be guarded in the node and launch wiring.
    planner_source = submit_root / "overtake_planner/src/overtake_planner_node.cpp"
    pp_source = submit_root / "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
    planner_launch = submit_root / "overtake_planner/launch/overtake_planner.launch.xml"
    pp_launch = (
        submit_root
        / "aichallenge_submit_launch/launch/control/pure_pursuit.launch.xml"
    )
    horizon_launch = (
        submit_root
        / "aichallenge_submit_launch/launch/control/pure_pursuit_mpc_horizon.launch.xml"
    )
    for path in (planner_source, pp_source, planner_launch, pp_launch, horizon_launch):
        if not path.is_file():
            violations.append(f"{path}:required startup forwarding surface missing")
            continue
        text = path.read_text(encoding="utf-8")
        if "c002ay1_prod_measure_enabled" not in text:
            violations.append(f"{path}:missing default-off C-002AY1 guard")
    for path in (planner_source, pp_source):
        if path.is_file() and "add_on_set_parameters_callback" in path.read_text(
            encoding="utf-8"
        ):
            violations.append(f"{path}:runtime observer parameter mutation forbidden")
    if pp_source.is_file():
        pp_text = pp_source.read_text(encoding="utf-8")
        if "recovery_mode_ && c002ay1_prod_measure_enabled_" not in pp_text:
            violations.append(f"{pp_source}:recovery observer hard-disable missing")
        if "recovery_mode_ && c002ay0_shadow_capture_enabled_" not in pp_text:
            violations.append(f"{pp_source}:AY0 recovery hard-disable missing")
        worker_start = pp_text.find("FixedBaseWorkerSession::start")
        worker_end = pp_text.find(
            "aw2_shadow_transport_enabled_",
            worker_start,
        )
        if worker_start < 0 or worker_end <= worker_start:
            violations.append(f"{pp_source}:AY0 worker startup guard missing")
        else:
            worker_startup = pp_text[worker_start:worker_end]
            if "!c002ay0_worker_session_->ready()" not in worker_startup:
                violations.append(
                    f"{pp_source}:AY0 non-ready cleanup owner guard missing"
                )
            if "c002ay0_shadow_capture_enabled_ = false;" not in worker_startup:
                violations.append(
                    f"{pp_source}:AY0 unavailable capture disable missing"
                )
        capture_start = pp_text.find("void SimplePurePursuit::captureAy0BaseShadow")
        capture_end = pp_text.find(
            "void SimplePurePursuit::captureControllerAppliedShadow",
            capture_start,
        )
        if capture_start < 0 or capture_end <= capture_start:
            violations.append(f"{pp_source}:AY0 fixed capture hook missing")
        else:
            capture_body = pp_text[capture_start:capture_end]
            for token in C002AY0_HOT_PATH_FORBIDDEN:
                if token in capture_body:
                    violations.append(
                        f"{pp_source}:{token} forbidden AY0 timer operation"
                    )
            if "state_lattice_shadow_race_arm_epoch_.epoch()" not in capture_body:
                violations.append(
                    f"{pp_source}:observer-only race segment missing"
                )
            if "overtake_plan_->race_arm_epoch" in capture_body:
                violations.append(
                    f"{pp_source}:AY0 observation still depends on plan publish"
                )
            if "buildBaseSnapshotFromFixedRecord" in capture_body:
                violations.append(
                    f"{pp_source}:snapshot construction must remain worker-only"
                )
        for live_token in (
            "/overtake/authorized_cartesian_trajectory",
            "/control/overtake/cartesian",
            "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
            "/debug/overtake/state_lattice/source_binding",
        ):
            if live_token in pp_text:
                violations.append(
                    f"{pp_source}:{live_token} PP endpoint forbidden"
                )
    if pp_launch.is_file():
        launch_text = pp_launch.read_text(encoding="utf-8")
        if (
            'name="c002ay0_shadow_capture_enabled" default="false"'
            not in launch_text
        ):
            violations.append(f"{pp_launch}:AY0 capture must be default-off")
        if (
            'name="input_race_armed" default="/overtake/race_armed"'
            not in launch_text
            or 'from="input/race_armed" to="$(var input_race_armed)"'
            not in launch_text
        ):
            violations.append(
                f"{pp_launch}:shadow race segment input/remap missing"
            )
    if horizon_launch.is_file():
        horizon_text = horizon_launch.read_text(encoding="utf-8")
        if (
            "c002ay1_prod_measure_enabled" not in horizon_text
            or "value=\"false\"" not in horizon_text
        ):
            violations.append(
                f"{horizon_launch}:recovery observer false forwarding missing"
            )
        if (
            '<arg name="c002ay0_shadow_capture_enabled" value="false"/>'
            not in horizon_text
        ):
            violations.append(
                f"{horizon_launch}:recovery AY0 capture false forwarding missing"
            )
        if (
            '<arg name="state_lattice_source_binding_shadow_enabled" value="false"/>'
            not in horizon_text
        ):
            violations.append(
                f"{horizon_launch}:recovery source binding false forwarding missing"
            )

    shadow_epoch_token = "StateLatticeShadowRaceArmEpoch"
    for root_parts in C002AY1_AUTHORITY_ROOTS:
        root = workspace_src.joinpath(*root_parts)
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if (
                path.is_file()
                and path.suffix in LIVE_SUFFIXES
                and shadow_epoch_token in path.read_text(encoding="utf-8")
            ):
                violations.append(
                    f"{path}:shadow race epoch entered authority owner"
                )

    # AY-0L exposes the authority-free base snapshot from its isolated worker.
    # C-002SL-AY0-PROPOSAL-WORKER-004 may observe that snapshot only when its
    # default-off Node capture is enabled. The planning callback may only copy
    # an already-fixed record to its SPSC queue; the child worker alone may
    # canonicalize, serialize, and publish the authority-free debug result.
    worker_source = contract_root / "src/c002ay0_worker_ipc.cpp"
    if worker_source.is_file():
        worker_text = worker_source.read_text(encoding="utf-8")
        shadow_topic = "/control/overtake/base_trajectory_snapshot"
        if worker_text.count(shadow_topic) != 1:
            violations.append(
                f"{worker_source}:exactly one AY0L shadow publisher required"
            )
        if "create_subscription<ControllerBaseTrajectorySnapshot" in worker_text:
            violations.append(
                f"{worker_source}:AY0L shadow worker must not consume authority"
            )
        binding_topic = "/debug/overtake/state_lattice/source_binding"
        proposal_topic = (
            "/debug/overtake/state_lattice/authorized_cartesian_trajectory"
        )
        if worker_text.count(binding_topic) != 1:
            violations.append(
                f"{worker_source}:exactly one worker binding publisher required"
            )
        if worker_text.count(proposal_topic) != 1:
            violations.append(
                f"{worker_source}:exactly one worker proposal observer required"
            )
        if '"lateral_authority_eligible\\":false' not in worker_text:
            violations.append(
                f"{worker_source}:worker binding authority must be false"
            )
        binding_audit_topic = (
            "/test/c002ay0/state_lattice/binding_callback_terminal"
        )
        if worker_text.count(binding_audit_topic) != 1:
            violations.append(
                f"{worker_source}:exactly one test-only binding audit topic required"
            )
        if (
            "C002AY0_TEST_RELIABLE_BINDING_AUDIT" not in worker_text
            or "std::getenv(kTestReliableBindingAuditEnvironment)"
            not in worker_text
        ):
            violations.append(
                f"{worker_source}:binding audit must remain explicit-env-only"
            )
    else:
        violations.append(f"{worker_source}:AY0L shadow worker missing")

    state_lattice_node = (
        submit_root
        / "state_lattice_overtake_planner/src/"
        "state_lattice_overtake_planner_node.cpp"
    )
    state_lattice_builder = (
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal.cpp"
    )
    state_lattice_worker = (
        submit_root
        / "state_lattice_overtake_planner/src/c002ay0_shadow_proposal_worker.cpp"
    )
    if (
        not state_lattice_node.is_file()
        or not state_lattice_builder.is_file()
        or not state_lattice_worker.is_file()
    ):
        violations.append(
            "state_lattice_overtake_planner:AY0 shadow proposal files missing"
        )
    else:
        node_text = state_lattice_node.read_text(encoding="utf-8")
        builder_text = state_lattice_builder.read_text(encoding="utf-8")
        state_lattice_worker_text = state_lattice_worker.read_text(encoding="utf-8")
        debug_topic = "/debug/overtake/state_lattice/authorized_cartesian_trajectory"
        if debug_topic in node_text:
            violations.append(
                f"{state_lattice_node}:proposal publish requires isolated worker"
            )
        if "/overtake/authorized_cartesian_trajectory" in node_text:
            violations.append(
                f"{state_lattice_node}:live authorized topic forbidden"
            )
        # Phase 2 permits one main-process canonical build only for the new,
        # startup-only, non-authoritative V2 edge. The legacy AY0 debug
        # proposal remains isolated in its worker and is still forbidden here.
        if node_text.count("buildAy0ShadowProposal(") != 1:
            violations.append(
                f"{state_lattice_node}:exactly one V2 canonical build required"
            )
        v2_build_start = node_text.find("void publishStateLatticeV2Proposal")
        v2_build_end = node_text.find(
            "captureAy0ShadowProposal(const PlannerOutput", v2_build_start
        )
        if v2_build_start < 0 or v2_build_end <= v2_build_start:
            violations.append(f"{state_lattice_node}:V2 publish guard missing")
        else:
            v2_build = node_text[v2_build_start:v2_build_end]
            for token in (
                "state_lattice_v2_live_proposal_publish_enabled_",
                "output.active",
                "output.emergency_stop",
                "WireKind::LATERAL_AND_SPEED_V3",
                "last_payload_",
                "selected_mode_matches",
                "buildAy0ShadowProposal(",
                "SCHEMA_V2_NON_AUTHORITATIVE",
            ):
                if token not in v2_build:
                    violations.append(
                        f"{state_lattice_node}:{token} missing from V2 publish edge"
                    )
        if (
            'declare_parameter<bool>("c002ay0_state_lattice_shadow_enabled", false)'
            not in node_text
        ):
            violations.append(
                f"{state_lattice_node}:AY0 proposal capture must default off"
            )
        if (
            node_text.count("/control/overtake/base_trajectory_snapshot") != 1
            or node_text.count(
                "/control/overtake/state_lattice/v2_base_attestation"
            )
            != 1
            or "onStateLatticeV2BaseAttestation" not in node_text
        ):
            violations.append(
                f"{state_lattice_node}:shadow and direct base surfaces must stay isolated"
            )
        if (
            'declare_parameter<bool>(\n        "state_lattice_v2_live_proposal_publish_enabled", false)'
            not in node_text
            or node_text.count("/planning/overtake/state_lattice/v2_proposal") != 1
        ):
            violations.append(
                f"{state_lattice_node}:V2 publisher must be startup-only/default-off"
            )
        if state_lattice_worker_text.count(debug_topic) != 1:
            violations.append(
                f"{state_lattice_worker}:exactly one shadow proposal publisher required"
            )
        if "proposal.trajectory.authority_eligible" not in state_lattice_worker_text:
            violations.append(
                f"{state_lattice_worker}:shadow publish must reject authority eligible payloads"
            )
        if "/overtake/authorized_cartesian_trajectory" in state_lattice_worker_text:
            violations.append(
                f"{state_lattice_worker}:live authorized topic forbidden"
            )
        if "trajectory.authority_eligible = false;" not in builder_text:
            violations.append(
                f"{state_lattice_builder}:authority must be hard-coded false"
            )
        if "trajectory.authority_eligible = true" in builder_text:
            violations.append(
                f"{state_lattice_builder}:authority promotion forbidden"
            )

        if pp_source.is_file():
            pp_text = pp_source.read_text(encoding="utf-8")
            if (
                'declare_parameter<bool>(\n      "state_lattice_v2_live_proposal_accept_enabled", false)'
                not in pp_text
                or pp_text.count("/planning/overtake/state_lattice/v2_proposal")
                != 1
                or "Intentionally discard v2_result.accepted" not in pp_text
                or '"state_lattice_v2_base_attestation_publish_enabled", false'
                not in pp_text
                or pp_text.count(
                    "/control/overtake/state_lattice/v2_base_attestation"
                )
                != 1
                or "kStatusQosDepth" not in pp_text
                or "v2_status_qos" not in pp_text
            ):
                violations.append(
                    f"{pp_source}:V2 uptake must be startup-only and non-authoritative"
                )

    # Static 9.9: collector and its producer fixture remain test-only and
    # cannot enter the production install surface.
    for test_only_name in (
        "c002ay1_runtime_collector",
        "c002ay1_runtime_producer_fixture",
        "c002ay1_runtime_attach_probe",
    ):
        test_only_index = cmake_text.find(test_only_name)
        if (
            testing_index < 0
            or test_only_index < testing_index
            or f"install(TARGETS {test_only_name}" in cmake_text
        ):
            violations.append(
                f"{contract_root / 'CMakeLists.txt'}:"
                f"{test_only_name} must remain BUILD_TESTING-only and "
                "uninstalled"
            )
    if violations:
        print("C-002AY NO-LIVE contract violation:", file=sys.stderr)
        print("\n".join(violations), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
