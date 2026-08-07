#!/usr/bin/env python3
"""Static no-authority contract for the State Lattice observer launcher."""

from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


SUBMIT_ROOT = Path(__file__).resolve().parents[2]
LAUNCH = (
    SUBMIT_ROOT
    / "aichallenge_submit_launch"
    / "launch"
    / "test_only"
    / "state_lattice_observer.launch.xml"
)
HARNESS = SUBMIT_ROOT.parents[2] / "run_state_lattice_test_only_observer.bash"

FORBIDDEN = {
    "/control/command/control_cmd",
    "/control/command/control_cmd_raw",
    "/overtake/reference_override",
    "/hybrid_control/state_lattice/control_cmd",
    "/hybrid_control/motion_authority_grant",
    "/overtake/race_armed",
    "/admin/awsim/start",
    "/admin/awsim/reset",
}


class StateLatticeTestOnlyObserverContractTest(unittest.TestCase):
    def test_no_authority_parameters_are_hard_coded(self) -> None:
        node = ET.parse(LAUNCH).getroot().find("node")
        self.assertIsNotNone(node)
        assert node is not None
        self.assertEqual(node.attrib["name"], "test_only_state_lattice_observer")
        args = {arg.attrib["name"] for arg in ET.parse(LAUNCH).getroot().findall("arg")}
        self.assertIn("run_id", args)
        self.assertIn("topic_token", args)
        params = {
            param.attrib["name"]: param.attrib.get("value")
            for param in node.findall("param")
            if "name" in param.attrib
        }
        self.assertEqual(params["live_control_output_enabled"], "false")
        self.assertEqual(params["instant_control_enabled"], "false")
        self.assertEqual(params["safety_evaluation_enabled"], "true")
        self.assertEqual(params["c002ay0_state_lattice_shadow_enabled"], "false")
        self.assertTrue(params["instant_control_topic"].startswith("/test_only/"))
        self.assertTrue(params["reference_override_topic"].startswith("/test_only/"))

    def test_all_node_outputs_are_remapped_to_test_only(self) -> None:
        node = ET.parse(LAUNCH).getroot().find("node")
        self.assertIsNotNone(node)
        assert node is not None
        remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in node.findall("remap")
        }
        debug_topics = {
            "/debug/overtake/mode",
            "/debug/overtake/metrics",
            "/debug/overtake/wall_map",
            "/debug/overtake/wall_costmap",
            "/debug/overtake/opponent_costmap",
            "/debug/overtake/costmap",
            "/debug/overtake/trajectory_candidates",
            "/debug/overtake/selected_trajectory",
            "/debug/overtake/target_states",
            "/debug/overtake/rear_safety_paths",
            "/debug/overtake/planning_geometry",
        }
        self.assertTrue(debug_topics <= remaps.keys())
        for topic in debug_topics:
            self.assertTrue(remaps[topic].startswith("/test_only/"))
        launch_text = LAUNCH.read_text(encoding="utf-8")
        for topic in FORBIDDEN:
            self.assertNotIn(topic, launch_text)

    def test_harness_is_bounded_and_preserves_live_authority_graph(self) -> None:
        source = HARNESS.read_text(encoding="utf-8")
        for required in (
            "TEST_ONLY_DURATION_SEC",
            "GRAPH_QUERY_TIMEOUT_SEC=5",
            "ros_graph_before.txt",
            "ros_graph_active.txt",
            "ros_graph_after.txt",
            "rosbag2_test_only",
            "-name '*.mcap*'",
            "observer_node_info.txt",
            "test_only_output_has_subscriber_before_recording",
            "ros_graph_before_after.diff",
            "set +u",
            "set -u",
            "live_control_output_enabled=false",
            "instant_control_enabled=false",
        ):
            self.assertIn(required, source)
        self.assertNotIn("make down", source)
        self.assertNotIn("pkill", source)


if __name__ == "__main__":
    unittest.main()
