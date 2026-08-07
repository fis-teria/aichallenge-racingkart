#!/usr/bin/env python3
"""Static guardrails for the default-off State Lattice live source."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


SUBMIT_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_ROOT = SUBMIT_ROOT / "aichallenge_submit_launch" / "launch"
PLANNER_ROOT = SUBMIT_ROOT / "overtake_planner"
PLANNER_NODE = PLANNER_ROOT / "src" / "overtake_planner_node.cpp"
PLANNER_CORE = PLANNER_ROOT / "src" / "overtake_planner_core.cpp"
PLANNER_PARAMS = PLANNER_ROOT / "config" / "overtake_planner.param.yaml"
ROUTE_RESOLVER = PLANNER_ROOT / "scripts" / "resolve_overtake_control_route"
STATE_LATTICE_ROOT = SUBMIT_ROOT / "state_lattice_overtake_planner"
STATE_LATTICE_NODE = (
    STATE_LATTICE_ROOT / "src" / "state_lattice_overtake_planner_node.cpp"
)
STATE_LATTICE_PARAMS = (
    STATE_LATTICE_ROOT / "config" / "state_lattice_overtake_planner.param.yaml"
)
AWSIM_MODE_LAUNCH = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "launch"
    / "mode"
    / "awsim.launch.xml"
)
RVIZ_CONFIG_ROOT = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "config"
)
SYSTEM_LAUNCH = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "launch"
    / "aichallenge_system.launch.xml"
)
AICHALLENGE_ROOT = next(
    (
        parent
        for parent in SUBMIT_ROOT.parents
        if (parent / "run_autoware.bash").is_file()
    ),
    None,
)
RUN_AUTOWARE = (
    AICHALLENGE_ROOT / "run_autoware.bash" if AICHALLENGE_ROOT else None
)
RUN_EVALUATION = (
    AICHALLENGE_ROOT / "run_evaluation.bash" if AICHALLENGE_ROOT else None
)
EVALUATION_LAUNCH = SYSTEM_LAUNCH.with_name("evaluation.launch.xml")
DOCKER_COMPOSE = (
    AICHALLENGE_ROOT.parent / "docker-compose.yml"
    if AICHALLENGE_ROOT is not None
    else None
)

EXPECTED_AUTHORITY_PUBLISHERS = {
    "/overtake/reference_override": "std_msgs::msg::Float32MultiArray",
    "/overtake/safety_constraint": (
        "multi_purpose_mpc_ros_msgs::msg::SafetyConstraint"
    ),
    "/overtake/plan": "multi_purpose_mpc_ros_msgs::msg::OvertakePlan",
}

V2_EVIDENCE_TOPICS = (
    "/planning/overtake/state_lattice/v2_proposal",
    "/debug/overtake/state_lattice/v2_binding_status",
    "/control/overtake/state_lattice/v2_base_attestation",
)
V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS = (
    SUBMIT_ROOT / "hybrid_control_mux",
    SUBMIT_ROOT / "multi_purpose_mpc_ros",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def braced_block(source: str, marker: str) -> str:
    start = source.index(marker)
    open_brace = source.index("{", start)
    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1 : index]
    raise AssertionError(f"unterminated block after {marker!r}")


class StateLatticeNonLiveContractTest(unittest.TestCase):
    def test_v2_final_fence_is_private_diagnostic_and_default_off(self) -> None:
        parameter = "state_lattice_v2_final_fence_enabled"
        node_source = read(STATE_LATTICE_NODE)
        self.assertRegex(
            node_source,
            rf'declare_parameter<bool>\(\s*"{parameter}",\s*false\)',
        )
        planner_launch = STATE_LATTICE_ROOT / "launch/state_lattice_overtake_planner.launch.xml"
        root = ET.fromstring(read(planner_launch))
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in root.findall("arg")
        }
        self.assertEqual(defaults[parameter], "false")
        self.assertIn("/test/m4/state_lattice/v2_quiesce", node_source)
        self.assertIn("/test/m4/state_lattice/v2_final_fence", node_source)
        self.assertRegex(
            node_source,
            r"final_fence_admission\s*=\s*"
            r"state_lattice_v2_final_fence_enabled_\s*\?\s*"
            r"state_lattice_v2_final_fence_state_->tryAdmit",
        )
        for path in LAUNCH_ROOT.rglob("*.xml"):
            self.assertNotIn(parameter, read(path), msg=str(path))
        forbidden_roots = V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS + (
            SUBMIT_ROOT / "simple_pure_pursuit",
        )
        for package_root in forbidden_roots:
            for path in package_root.rglob("*"):
                if path.is_file() and path.suffix in {".cpp", ".hpp", ".py", ".xml"}:
                    text = read(path)
                    self.assertNotIn("/test/m4/state_lattice/v2_quiesce", text)
                    self.assertNotIn("/test/m4/state_lattice/v2_final_fence", text)

    def test_exact_spatial_generation_is_not_live_publication_authority(self) -> None:
        params = yaml.safe_load(read(STATE_LATTICE_PARAMS))
        node_params = params["state_lattice_overtake_planner_node"]["ros__parameters"]
        self.assertNotIn(
            "experimental_exact_spatial_follow_shadow_enabled",
            node_params,
        )
        node_source = read(
            STATE_LATTICE_ROOT / "src/state_lattice_overtake_planner_node.cpp"
        )
        self.assertRegex(
            node_source,
            r'declare_parameter<bool>\(\s*'
            r'"experimental_exact_spatial_follow_shadow_enabled",\s*true\)',
        )
        self.assertRegex(
            node_source,
            r'declare_parameter<bool>\(\s*'
            r'"experimental_spatial_reference_override_live_publish_enabled",\s*'
            r'false\)',
        )

        launch = ET.fromstring(
            read(STATE_LATTICE_ROOT / "launch/state_lattice_overtake_planner.launch.xml")
        )
        args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in launch.findall("arg")
        }
        self.assertEqual(
            args["experimental_exact_spatial_follow_shadow_enabled"], "true"
        )
        self.assertEqual(
            args["experimental_spatial_reference_override_live_publish_enabled"],
            "false",
        )
        node = launch.find("node")
        self.assertIsNotNone(node)
        overrides = [
            param
            for param in node.findall("param")
            if param.attrib.get("name")
            == "experimental_exact_spatial_follow_shadow_enabled"
        ]
        self.assertEqual(len(overrides), 1)
        self.assertEqual(
            overrides[0].attrib.get("value"),
            "$(var experimental_exact_spatial_follow_shadow_enabled)",
        )
        live_overrides = [
            param
            for param in node.findall("param")
            if param.attrib.get("name")
            == "experimental_spatial_reference_override_live_publish_enabled"
        ]
        self.assertEqual(len(live_overrides), 1)
        self.assertEqual(
            live_overrides[0].attrib.get("value"),
            "$(var experimental_spatial_reference_override_live_publish_enabled)",
        )

    def test_v2_poc_identity_sideband_is_explicit_default_off_and_route_scoped(
        self,
    ) -> None:
        names = (
            "state_lattice_v2_live_proposal_publish_enabled",
            "state_lattice_v2_live_proposal_accept_enabled",
            "state_lattice_v2_producer_instance_id",
            "state_lattice_v2_pp_producer_instance_id",
            "state_lattice_v2_session_id",
        )
        expected_defaults = {
            names[0]: "false",
            names[1]: "false",
            names[2]: "0",
            names[3]: "",
            names[4]: "",
        }

        submit = ET.fromstring(read(LAUNCH_ROOT / "aichallenge_submit.launch.xml"))
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        for root in (submit, reference):
            defaults = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            self.assertEqual(
                {name: defaults.get(name) for name in names},
                expected_defaults,
            )

        submit_reference = submit.find("include")
        self.assertIsNotNone(submit_reference)
        submit_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in submit_reference.findall("arg")
        }
        for name in names:
            self.assertEqual(submit_values[name], f"$(var {name})")

        groups = {
            group.attrib.get("if", ""): group
            for group in reference.findall("group")
        }
        state_lattice_group = next(
            group
            for condition, group in groups.items()
            if "state_lattice_pure_pursuit" in condition
        )
        state_lattice_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_lattice_group.find("include").findall("arg")
        }
        for name in names:
            self.assertEqual(state_lattice_values[name], f"$(var {name})")

        horizon_group = next(
            group
            for condition, group in groups.items()
            if "pure_pursuit_mpc_horizon" in condition
        )
        horizon_names = {
            arg.attrib.get("name")
            for arg in horizon_group.find("include").findall("arg")
        }
        self.assertTrue(set(names).isdisjoint(horizon_names))

        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        profile_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in profile.findall("arg")
        }
        self.assertEqual(
            {name: profile_defaults.get(name) for name in names},
            expected_defaults,
        )
        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        route_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in route_group.find("include").findall("arg")
        }
        self.assertEqual(
            {name: route_values[name] for name in names},
            {
                names[0]: "false",
                names[1]: "false",
                names[2]: "4101",
                names[3]: "4201",
                names[4]: "1",
            },
        )

        system = ET.fromstring(read(SYSTEM_LAUNCH))
        system_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in system.findall("arg")
        }
        self.assertEqual(
            {name: system_defaults.get(name) for name in names},
            expected_defaults,
        )
        submit_include = next(
            include
            for include in system.findall("include")
            if "aichallenge_submit.launch.xml" in include.attrib.get("file", "")
        )
        system_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in submit_include.findall("arg")
        }
        for name in names:
            self.assertEqual(system_values[name], f"$(var {name})")

        horizon = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        state_group = next(
            group
            for group in horizon.findall("group")
            if group.attrib.get("if") == "$(var state_lattice_pure_pursuit_enabled)"
        )
        planner_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_group.find("include").findall("arg")
        }
        self.assertEqual(
            planner_values["experimental_spatial_reference_override_live_publish_enabled"],
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )
        pp_include = next(
            include
            for include in horizon.findall("include")
            if "pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        pp_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in pp_include.findall("arg")
        }
        self.assertEqual(
            pp_values["state_lattice_v4_poc_identity_gate_enabled"],
            "$(var state_lattice_v4_poc_identity_gate_enabled)",
        )
        self.assertEqual(
            pp_values["state_lattice_v4_poc_command_activation_enabled"],
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )

        env_names = (
            "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED",
            "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID",
            "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID",
            "STATE_LATTICE_V2_SESSION_ID",
        )
        defaults = ("false", "false", "0")
        # Package-only Docker/CI mounts stop at the ROS workspace, so the
        # repository-level runners and compose file are intentionally absent.
        # The host-level invocation covers the complete environment chain.
        for runner_path in (
            path for path in (RUN_AUTOWARE, RUN_EVALUATION) if path is not None
        ):
            runner = read(runner_path)
            for name, env_name, default in zip(
                names[:3], env_names[:3], defaults
            ):
                self.assertIn(
                    f'"{name}:=${{{env_name}:-{default}}}"', runner
                )
            for name, env_name in zip(names[3:], env_names[3:]):
                self.assertIn(f'if [[ -n "${{{env_name}:-}}" ]]; then', runner)
                self.assertIn(
                    f'"{name}:=${{{env_name}}}"', runner
                )
                self.assertNotIn(
                    f'"{name}:=${{{env_name}:-}}"', runner
                )
        if DOCKER_COMPOSE is not None and DOCKER_COMPOSE.is_file():
            compose = read(DOCKER_COMPOSE)
            for env_name, default in zip(env_names, defaults):
                self.assertIn(
                    f"- {env_name}=${{{env_name}:-{default}}}", compose
                )

        evaluation = ET.fromstring(read(EVALUATION_LAUNCH))
        evaluation_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in evaluation.findall("arg")
        }
        self.assertEqual(
            {name: evaluation_defaults.get(name) for name in names},
            expected_defaults,
        )
        evaluation_system_include = next(
            include
            for group in evaluation.findall("group")
            for include in group.findall("include")
            if "aichallenge_system.launch.xml" in include.attrib.get("file", "")
        )
        evaluation_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in evaluation_system_include.findall("arg")
        }
        for name in names:
            self.assertEqual(evaluation_values[name], f"$(var {name})")

    def test_v2_string_parameters_are_explicit_at_leaf_node_boundaries(
        self,
    ) -> None:
        pure_pursuit = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit.launch.xml")
        )
        pure_pursuit_params = {
            param.attrib.get("name"): param
            for param in pure_pursuit.find("node").findall("param")
        }
        for name in (
            "state_lattice_v2_expected_producer_instance_id",
            "state_lattice_v2_base_attestation_producer_instance_id",
            "state_lattice_v2_base_attestation_session_id",
        ):
            with self.subTest(node="pure_pursuit", name=name):
                self.assertEqual(pure_pursuit_params[name].attrib.get("type"), "str")

        state_lattice = ET.fromstring(
            read(
                STATE_LATTICE_ROOT
                / "launch/state_lattice_overtake_planner.launch.xml"
            )
        )
        state_lattice_params = {
            param.attrib.get("name"): param
            for param in state_lattice.find("node").findall("param")
        }
        for name in (
            "state_lattice_v2_expected_pp_producer_instance_id",
            "state_lattice_v2_expected_pp_session_id",
        ):
            with self.subTest(node="state_lattice", name=name):
                self.assertEqual(state_lattice_params[name].attrib.get("type"), "str")
                self.assertEqual(
                    state_lattice_params[name].attrib.get("value"), f"$(var {name})"
                )

        # The planner producer ID is intentionally numeric, unlike the
        # numeric-looking producer/session identities above.
        self.assertIsNone(
            state_lattice_params[
                "state_lattice_v2_producer_instance_id"
            ].attrib.get("type")
        )

    def test_v4_command_activation_is_dedicated_and_v2_direct_stays_off(
        self,
    ) -> None:
        activation_name = "state_lattice_v4_poc_command_activation_enabled"
        submit = ET.fromstring(read(LAUNCH_ROOT / "aichallenge_submit.launch.xml"))
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        for root in (submit, reference):
            defaults = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            self.assertEqual(defaults.get(activation_name), "false")

        submit_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in submit.find("include").findall("arg")
        }
        self.assertEqual(submit_values.get(activation_name), f"$(var {activation_name})")
        state_profile_include = next(
            group.find("include")
            for group in reference.findall("group")
            if "state_lattice_pure_pursuit" in group.attrib.get("if", "")
        )
        reference_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_profile_include.findall("arg")
        }
        self.assertEqual(
            reference_values.get(activation_name),
            "$(var state_lattice_v4_awsim_poc_enabled)",
        )

        leaf = ET.fromstring(read(LAUNCH_ROOT / "control/pure_pursuit.launch.xml"))
        leaf_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in leaf.findall("arg")
        }
        self.assertEqual(
            leaf_defaults.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            leaf_defaults.get("state_lattice_v4_poc_command_activation_enabled"),
            "false",
        )
        leaf_params = {
            param.attrib.get("name"): param.attrib.get("value")
            for param in leaf.find("node").findall("param")
        }
        self.assertEqual(
            leaf_params.get("state_lattice_v2_command_activation_enabled"),
            "$(var state_lattice_v2_command_activation_enabled)",
        )
        self.assertEqual(
            leaf_params.get("state_lattice_v4_poc_command_activation_enabled"),
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )

        shared = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        shared_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in shared.findall("arg")
        }
        self.assertEqual(
            shared_defaults.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            shared_defaults.get("state_lattice_v4_poc_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            shared_defaults.get("state_lattice_v4_poc_identity_gate_enabled"),
            "false",
        )

        dedicated = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        dedicated_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in dedicated.findall("arg")
        }
        self.assertEqual(dedicated_defaults.get(activation_name), "false")
        shared_include = dedicated.find("group/include")
        self.assertIsNotNone(shared_include)
        dedicated_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in shared_include.findall("arg")
        }
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v4_poc_identity_gate_enabled"),
            "false",
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v4_poc_command_activation_enabled"),
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )
        self.assertEqual(
            dedicated_values.get("require_safety_constraint"),
            "false",
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_live_proposal_publish_enabled"),
            "false",
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_live_proposal_accept_enabled"),
            "false",
        )

        require_safety_default = shared_defaults.get("require_safety_constraint")
        self.assertIsNotNone(require_safety_default)
        self.assertIn("$(var use_overtake_planner)", require_safety_default)
        self.assertIn("$(var overtake_control_route)", require_safety_default)
        self.assertIn("current", require_safety_default)
        mux_include = next(
            include
            for include in shared.findall("include")
            if "hybrid_control_mux.launch.xml" in include.attrib.get("file", "")
        )
        mux_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in mux_include.findall("arg")
        }
        self.assertEqual(
            mux_values.get("require_safety_constraint"),
            "$(var require_safety_constraint)",
        )

        pure_pursuit_includes = [
            include
            for include in shared.findall("include")
            if "pure_pursuit.launch.xml" in include.attrib.get("file", "")
        ]
        recovery_include = next(
            include
            for include in pure_pursuit_includes
            if any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in include.findall("arg")
            )
        )
        recovery_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in recovery_include.findall("arg")
        }
        for name in (
            "state_lattice_v2_live_proposal_accept_enabled",
            "state_lattice_v2_command_activation_enabled",
            "state_lattice_v4_poc_identity_gate_enabled",
            "state_lattice_v4_poc_command_activation_enabled",
        ):
            with self.subTest(recovery_arg=name):
                self.assertEqual(recovery_values.get(name), "false")

    def test_state_lattice_debug_layers_are_non_authoritative_publishers(
        self,
    ) -> None:
        node = read(STATE_LATTICE_NODE)
        for topic in (
            "/debug/overtake/planning_geometry",
            "/debug/overtake/trajectory_candidates",
            "/debug/overtake/selected_trajectory",
            "/debug/overtake/opponent_costmap",
            "/debug/overtake/wall_costmap",
        ):
            self.assertIn(f'"{topic}"', node)

        publish_costmap = braced_block(node, "void publishCostmap()")
        self.assertIn("map_.wallLevels()", publish_costmap)
        self.assertIn("objectCostLevel(", publish_costmap)
        self.assertIn("wall_costmap_pub_->publish", publish_costmap)
        self.assertIn("opponent_costmap_pub_->publish", publish_costmap)
        self.assertIn("inputsFresh(stamp.seconds())", publish_costmap)
        self.assertNotIn("override_pub_->publish", publish_costmap)
        self.assertNotIn("instant_control_pub_->publish", publish_costmap)

    def test_rviz_presets_expose_all_state_lattice_debug_layers(self) -> None:
        expected_topics = {
            "/debug/overtake/planning_geometry",
            "/debug/overtake/trajectory_candidates",
            "/debug/overtake/selected_trajectory",
            "/debug/overtake/opponent_costmap",
            "/debug/overtake/wall_costmap",
        }

        def all_displays(displays: list[dict]) -> list[dict]:
            flattened: list[dict] = []
            for display in displays:
                flattened.append(display)
                nested = display.get("Displays")
                if isinstance(nested, list):
                    flattened.extend(all_displays(nested))
            return flattened

        for filename in ("autoware.rviz", "autoware_vehicle.rviz"):
            config = yaml.safe_load(read(RVIZ_CONFIG_ROOT / filename))
            displays = all_displays(config["Visualization Manager"]["Displays"])
            state_lattice_group = next(
                display
                for display in displays
                if display.get("Name") == "State Lattice Debug"
            )
            self.assertTrue(state_lattice_group["Enabled"])
            topics = {
                display.get("Topic", {}).get("Value")
                for display in state_lattice_group["Displays"]
            }
            self.assertEqual(topics, expected_topics)
            cost_displays = [
                display
                for display in state_lattice_group["Displays"]
                if display.get("Class") == "rviz_default_plugins/Map"
            ]
            self.assertEqual(len(cost_displays), 2)
            self.assertTrue(
                all(display["Color Scheme"] == "costmap" for display in cost_displays)
            )
            wall_display = next(
                display
                for display in cost_displays
                if display["Topic"]["Value"] == "/debug/overtake/wall_costmap"
            )
            self.assertEqual(
                wall_display["Topic"]["Durability Policy"], "Transient Local"
            )

    def test_planner_config_is_the_route_owner(self) -> None:
        params = yaml.safe_load(read(PLANNER_PARAMS))["overtake_planner_node"][
            "ros__parameters"
        ]
        self.assertIn(
            params["control_route"],
            {
                "current",
                "state_lattice_pure_pursuit",
                "state_lattice_instant_mux",
            },
        )
        self.assertEqual(
            params["control_route"],
            "state_lattice_instant_mux",
        )
        self.assertEqual(params["trajectory_backend"], "current")
        resolver = read(ROUTE_RESOLVER)
        self.assertIn('"state_lattice_instant_mux"', resolver)
        self.assertIn('DEFAULT_ROUTE = "current"', resolver)

        node = read(PLANNER_NODE)
        self.assertRegex(
            node,
            r'declare_parameter<std::string>\(\s*"trajectory_backend",\s*"current"\s*\)',
        )

    def test_awsim_adapter_does_not_own_overtake_control_route(self) -> None:
        awsim_launch = read(AWSIM_MODE_LAUNCH)
        self.assertNotIn("overtake_control_route", awsim_launch)
        self.assertNotIn("state_lattice_instant_mux", awsim_launch)
        self.assertNotIn("/hybrid_control/state_lattice/control_cmd", awsim_launch)

    def test_runtime_route_is_resolved_from_planner_config(self) -> None:
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        self.assertFalse(
            any(
                arg.attrib.get("name") == "overtake_control_route"
                for arg in reference.findall("arg")
            )
        )
        route_let = next(
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "overtake_control_route"
        )
        self.assertIn("resolve_overtake_control_route", route_let.attrib["value"])
        route_config_lets = [
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "overtake_control_route_config"
        ]
        self.assertEqual(len(route_config_lets), 2)
        default_route_config = next(
            item for item in route_config_lets if "if" not in item.attrib
        )
        poc_route_config = next(
            item for item in route_config_lets if "if" in item.attrib
        )
        self.assertIn("overtake_planner.param.yaml", default_route_config.attrib["value"])
        self.assertIn("v4_live_poc_route.param.yaml", poc_route_config.attrib["value"])
        self.assertEqual(
            poc_route_config.attrib["if"],
            "$(var state_lattice_v4_awsim_poc_enabled)",
        )
        effective_let = next(
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "state_lattice_v4_awsim_poc_enabled"
        )
        self.assertIn("state_lattice_v4_poc_command_activation_enabled", effective_let.attrib["value"])
        self.assertIn("simulation", effective_let.attrib["value"])

        for relative in (
            "control/hybrid_delay_aware_mpc.launch.xml",
            "control/pure_pursuit_mpc_horizon.launch.xml",
        ):
            launch_file = LAUNCH_ROOT / relative
            root = ET.fromstring(read(launch_file))
            args = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            with self.subTest(launch_file=launch_file):
                self.assertEqual(
                    args.get("overtake_control_route"), "current"
                )
                route_arg = next(
                    arg
                    for arg in root.findall("arg")
                    if arg.attrib.get("name") == "overtake_control_route"
                )
                self.assertEqual(
                    [choice.attrib.get("value") for choice in route_arg.findall("choice")],
                    [
                        "current",
                        "state_lattice_pure_pursuit",
                        "state_lattice_instant_mux",
                    ],
                )
                pp_groups = [
                    group
                    for group in root.findall("group")
                    if group.attrib.get("if")
                    == "$(var state_lattice_pure_pursuit_enabled)"
                ]
                instant_groups = [
                    group
                    for group in root.findall("group")
                    if group.attrib.get("if")
                    == "$(var state_lattice_instant_enabled)"
                ]
                self.assertEqual(len(pp_groups), 1)
                self.assertEqual(len(instant_groups), 1)
                pp_include = pp_groups[0].find("include")
                instant_include = instant_groups[0].find("include")
                self.assertIsNotNone(pp_include)
                self.assertIsNotNone(instant_include)
                self.assertIn(
                    "state_lattice_overtake_planner",
                    pp_include.attrib.get("file", ""),
                )
                self.assertIn(
                    "simple_state_lattice_planner",
                    instant_include.attrib.get("file", ""),
                )
                pp_args = {
                    arg.attrib.get("name"): arg.attrib.get("value")
                    for arg in pp_include.findall("arg")
                }
                instant_args = {
                    arg.attrib.get("name"): arg.attrib.get("value")
                    for arg in instant_include.findall("arg")
                }
                self.assertEqual(pp_args["instant_control_enabled"], "false")
                self.assertEqual(pp_args["live_control_output_enabled"], "true")
                self.assertEqual(
                    pp_args["output_reference_override"],
                    "/overtake/reference_override",
                )
                self.assertEqual(
                    pp_args["safety_evaluation_enabled"], "true"
                )
                self.assertEqual(
                    instant_args["live_control_output_enabled"], "true"
                )
                self.assertEqual(
                    instant_args["output_control_cmd"],
                    "/hybrid_control/state_lattice/control_cmd",
                )
                self.assertEqual(
                    instant_args["safety_evaluation_enabled"], "true"
                )

    def test_state_lattice_pure_pursuit_profile_has_no_live_mpc(self) -> None:
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        profile_groups = [
            group
            for group in reference.findall("group")
            if "state_lattice_pure_pursuit" in group.attrib.get("if", "")
        ]
        self.assertEqual(len(profile_groups), 1)
        profile_include = profile_groups[0].find("include")
        self.assertIsNotNone(profile_include)
        self.assertTrue(
            profile_include.attrib["file"].endswith(
                "/launch/control/state_lattice_pure_pursuit.launch.xml"
            )
        )

        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        base_include = route_group.find("include")
        self.assertIsNotNone(base_include)
        profile_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in base_include.findall("arg")
        }
        self.assertEqual(profile_args["mpc_enabled"], "false")
        self.assertEqual(
            profile_args["state_lattice_mpc_health_speed_guard_enabled"],
            "false",
        )
        state_lattice_params = yaml.safe_load(read(STATE_LATTICE_PARAMS))[
            "state_lattice_overtake_planner_node"
        ]["ros__parameters"]
        self.assertNotIn(
            "mpc_health_speed_guard_enabled", state_lattice_params
        )
        self.assertEqual(
            profile_args["require_state_lattice_override_fresh"], "true"
        )

        base = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        delay_groups = [
            group
            for group in base.findall("group")
            if group.attrib.get("if") == "$(var mpc_enabled)"
            and any(
                "delay_aware_mpc.launch.xml"
                in include.attrib.get("file", "")
                for include in group.findall("include")
            )
        ]
        self.assertEqual(len(delay_groups), 1)
        pure_pursuit_include = next(
            include
            for include in base.findall("include")
            if include.attrib.get("file", "").endswith(
                "/launch/control/pure_pursuit.launch.xml"
            )
        )
        pure_pursuit_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in pure_pursuit_include.findall("arg")
        }
        self.assertEqual(
            pure_pursuit_args["use_mpc_predicted_horizon"],
            "$(var mpc_enabled)",
        )
        self.assertEqual(
            pure_pursuit_args["require_overtake_reference_override_fresh"],
            "$(var require_state_lattice_override_fresh)",
        )

    def test_state_lattice_profile_wires_primary_shadow_workers_only(self) -> None:
        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        profile_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in profile.findall("arg")
        }
        self.assertEqual(profile_args["c002ay0_shadow_capture_enabled"], "true")
        self.assertEqual(
            profile_args["state_lattice_source_binding_shadow_enabled"], "true"
        )
        self.assertEqual(
            profile_args["c002ay0_state_lattice_shadow_enabled"], "true"
        )
        for name in (
            "c002ay0_shadow_session_generation",
            "c002ay0_shadow_session_nonce",
            "c002ay0_state_lattice_shadow_session_generation",
            "c002ay0_state_lattice_shadow_session_nonce",
        ):
            self.assertGreater(int(profile_args[name]), 0)
        self.assertIn(
            "c002ay0_shadow_worker",
            profile_args["c002ay0_shadow_worker_path"],
        )
        self.assertIn(
            "c002ay0_state_lattice_shadow_worker",
            profile_args["c002ay0_state_lattice_shadow_worker_path"],
        )

        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        horizon_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in route_group.find("include").findall("arg")
        }
        for name in (
            "c002ay0_shadow_capture_enabled",
            "state_lattice_source_binding_shadow_enabled",
            "c002ay0_shadow_worker_path",
            "c002ay0_shadow_session_generation",
            "c002ay0_shadow_session_nonce",
            "c002ay0_state_lattice_shadow_enabled",
            "c002ay0_state_lattice_shadow_worker_path",
            "c002ay0_state_lattice_shadow_session_generation",
            "c002ay0_state_lattice_shadow_session_nonce",
        ):
            self.assertEqual(horizon_args[name], "$(var " + name + ")")

        horizon = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        primary = next(
            include
            for include in horizon.findall("include")
            if include.attrib.get("file", "").endswith(
                "/launch/control/pure_pursuit.launch.xml"
            )
            and not any(
                arg.attrib.get("name") == "node_name" for arg in include.findall("arg")
            )
        )
        primary_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in primary.findall("arg")
        }
        self.assertEqual(
            primary_args["state_lattice_source_binding_shadow_enabled"],
            "$(var state_lattice_source_binding_shadow_enabled)",
        )

        recovery = next(
            include
            for include in horizon.findall("include")
            if any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in include.findall("arg")
            )
        )
        recovery_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in recovery.findall("arg")
        }
        self.assertEqual(recovery_args["c002ay0_shadow_worker_path"], "")
        self.assertEqual(recovery_args["c002ay0_shadow_session_generation"], "0")
        self.assertEqual(recovery_args["c002ay0_shadow_session_nonce"], "0")

    def test_container_and_public_launches_cannot_override_route(self) -> None:
        workspace_src = SUBMIT_ROOT.parent
        paths = [
            workspace_src
            / "aichallenge_system"
            / "aichallenge_system_launch"
            / "launch"
            / "aichallenge_system.launch.xml",
            LAUNCH_ROOT / "aichallenge_submit.launch.xml",
        ]
        submodule_root = next(
            (
                parent
                for parent in SUBMIT_ROOT.parents
                if (parent / "docker-compose.yml").is_file()
            ),
            None,
        )
        if submodule_root is not None:
            paths.extend(
                (
                    submodule_root / "Makefile",
                    submodule_root / "docker-compose.yml",
                    submodule_root / "aichallenge" / "run_autoware.bash",
                )
            )
        for path in paths:
            source = read(path)
            with self.subTest(path=path):
                self.assertNotIn("OVERTAKE_CONTROL_ROUTE", source)
                self.assertNotIn("overtake_control_route", source)

    def test_route_resolver_allowlists_and_fails_closed(self) -> None:
        def resolve(route_yaml: str) -> str:
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "route.yaml"
                config.write_text(route_yaml, encoding="utf-8")
                result = subprocess.run(
                    [sys.executable, str(ROUTE_RESOLVER), str(config)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                return result.stdout.strip()

        valid = """
overtake_planner_node:
  ros__parameters:
    control_route: state_lattice_instant_mux
"""
        invalid = """
overtake_planner_node:
  ros__parameters:
    control_route: unsafe_unknown_route
"""
        missing = """
overtake_planner_node:
  ros__parameters: {}
"""
        self.assertEqual(resolve(valid), "state_lattice_instant_mux")
        self.assertEqual(resolve(invalid), "current")
        self.assertEqual(resolve(missing), "current")

    def test_live_state_lattice_output_fails_closed_if_safety_is_disabled(
        self,
    ) -> None:
        params = read(STATE_LATTICE_PARAMS)
        self.assertNotRegex(params, r"(?m)^\s*live_control_output_enabled:")
        self.assertNotRegex(params, r"(?m)^\s*instant_control_enabled:")
        self.assertNotRegex(params, r"(?m)^\s*safety_evaluation_enabled:")
        node = read(STATE_LATTICE_NODE)
        self.assertIn(
            "live control output requires safety_evaluation_enabled=true",
            node,
        )
        self.assertRegex(
            node,
            r"if\s*\(\s*!live_control_output_enabled_\s*\|\|\s*"
            r"!instant_control_enabled_\s*\)\s*\{\s*return;\s*\}",
        )

    def test_authority_publisher_topics_and_types_remain_single_writer(self) -> None:
        node = read(PLANNER_NODE)
        for topic, message_type in EXPECTED_AUTHORITY_PUBLISHERS.items():
            pattern = re.compile(
                r"create_publisher\s*<\s*"
                + re.escape(message_type)
                + r"\s*>\s*\(\s*\""
                + re.escape(topic)
                + r"\"",
                re.MULTILINE,
            )
            with self.subTest(topic=topic, message_type=message_type):
                self.assertEqual(len(pattern.findall(node)), 1)

    def test_v2_evidence_topics_have_no_mux_or_mpc_authority_consumer(self) -> None:
        for root in V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS:
            source = "\n".join(
                path.read_text(encoding="utf-8", errors="replace")
                for path in root.rglob("*")
                if path.is_file()
                and path.suffix in {".cpp", ".hpp", ".py", ".xml", ".yaml"}
            )
            for topic in V2_EVIDENCE_TOPICS:
                with self.subTest(root=root.name, topic=topic):
                    self.assertNotIn(topic, source)

    def test_backend_validation_accepts_shadow_and_reserved_candidate(self) -> None:
        node = read(PLANNER_NODE)
        marker = 'if (config.trajectory_backend != "current"'
        validation_start = node.index(marker)
        validation = node[validation_start : validation_start + 600]
        self.assertIn('config.trajectory_backend != "state_lattice_shadow"', validation)
        self.assertIn('config.trajectory_backend != "state_lattice_candidate"', validation)

    def test_reserved_candidate_branch_only_warns(self) -> None:
        node = read(PLANNER_NODE)
        candidate_branch = braced_block(
            node, 'if (config.trajectory_backend == "state_lattice_candidate")'
        )
        self.assertIn("RCLCPP_WARN", candidate_branch)
        self.assertRegex(
            candidate_branch,
            r"reserved; live\s+\"\s*\"candidates remain current",
        )
        self.assertNotRegex(
            candidate_branch,
            r"\b(?:StateLatticeShadowAdapter|ShadowGeometryGenerator)\b",
        )
        self.assertNotRegex(candidate_branch, r"\bcore_?\s*(?:->|\.)")

    def test_adapter_and_generator_are_confined_to_core_diagnostic(self) -> None:
        core = read(PLANNER_CORE)
        diagnostic = braced_block(
            core,
            "OvertakePlannerCore::evaluateStateLatticeShadowComparison",
        )
        self.assertIn("StateLatticeShadowAdapter", diagnostic)
        self.assertIn("ShadowGeometryGenerator", diagnostic)
        self.assertEqual(
            core.count("evaluateStateLatticeShadowComparison("),
            2,
            "one definition plus one diagnostic-only update callsite",
        )
        core_without_diagnostic = core.replace(diagnostic, "", 1)
        self.assertNotIn("StateLatticeShadowAdapter", core_without_diagnostic)
        self.assertNotIn("ShadowGeometryGenerator", core_without_diagnostic)

        node = read(PLANNER_NODE)
        self.assertNotIn("StateLatticeShadowAdapter", node)
        self.assertNotIn("ShadowGeometryGenerator", node)
        debug = braced_block(node, "void publishDebug(")
        self.assertIn("state_lattice_shadow_comparison", debug)
        self.assertNotIn(
            "state_lattice_shadow_comparison",
            node.replace(debug, "", 1),
            "shadow record may only be serialized by the debug path",
        )


if __name__ == "__main__":
    unittest.main()
