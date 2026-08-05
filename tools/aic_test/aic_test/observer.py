#!/usr/bin/env python3
"""Read-only ROS observer for the control-smoke scenario.

This node has subscriptions only.  It never publishes control, race-arm, AWSIM
admin, reference, or planner messages.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from typing import Any


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


FORBIDDEN_AUTHORITY_TOPICS = {
    "/awsim/admin/command",
    "/awsim/control_cmd",
    "/control/command/actuation_cmd",
    "/control/command/control_cmd",
    "/control/command/emergency_cmd",
    "/control/trajectory",
    "/overtake/reference_override",
    "/planning/scenario_planning/trajectory",
    "/race/arm",
}


def observe(
    duration_s: float,
    expected_domain_id: int,
    live_control_output_enabled: bool = False,
    instant_control_enabled: bool = False,
) -> dict[str, Any]:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=None)
    node = Node("aic_test_control_smoke_observer")
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    clocks: list[int] = []
    odom_stamps: list[int] = []
    frames: set[str] = set()
    invalid_reasons: list[str] = []

    def on_clock(message: Clock) -> None:
        value = _stamp_ns(message.clock)
        if clocks and value < clocks[-1]:
            invalid_reasons.append("clock_regressed")
        clocks.append(value)

    def on_odom(message: Odometry) -> None:
        stamp_ns = _stamp_ns(message.header.stamp)
        if odom_stamps and stamp_ns < odom_stamps[-1]:
            invalid_reasons.append("odometry_stamp_regressed")
        odom_stamps.append(stamp_ns)
        frames.add(message.header.frame_id)
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            invalid_reasons.append("odometry_nonfinite")

    node.create_subscription(Clock, "/clock", on_clock, qos)
    node.create_subscription(
        Odometry, "/localization/kinematic_state", on_odom, qos
    )

    started_monotonic_ns = time.monotonic_ns()
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(clocks) >= 3 and len(odom_stamps) >= 3:
            break

    graph_nodes = sorted(name for name, _ in node.get_node_names_and_namespaces())
    graph_topics = {
        name: sorted(types) for name, types in node.get_topic_names_and_types()
    }
    own_publishers = {
        name: sorted(types)
        for name, types in node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace()
        )
    }
    own_subscriptions = {
        name: sorted(types)
        for name, types in node.get_subscriber_names_and_types_by_node(
            node.get_name(), node.get_namespace()
        )
    }
    node.destroy_node()
    rclpy.shutdown()
    finished_monotonic_ns = time.monotonic_ns()

    required_nodes = {
        "hybrid_control_mux_node",
        "simple_pure_pursuit_node",
        "state_lattice_overtake_planner_node",
    }
    missing_nodes = sorted(required_nodes.difference(graph_nodes))
    if len(clocks) < 3:
        invalid_reasons.append("insufficient_clock_samples")
    if len(odom_stamps) < 3:
        invalid_reasons.append("insufficient_odometry_samples")
    if clocks and clocks[-1] <= clocks[0]:
        invalid_reasons.append("clock_not_progressing")
    if odom_stamps and odom_stamps[-1] <= odom_stamps[0]:
        invalid_reasons.append("odometry_stamp_not_progressing")
    if not frames or not frames.issubset({"map", "odom"}):
        invalid_reasons.append("unexpected_odometry_frame")
    if missing_nodes:
        invalid_reasons.append("required_nodes_missing")
    required_topic_types = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/localization/kinematic_state": "nav_msgs/msg/Odometry",
    }
    missing_topic_types = sorted(
        f"{topic}:{message_type}"
        for topic, message_type in required_topic_types.items()
        if message_type not in graph_topics.get(topic, [])
    )
    if missing_topic_types:
        invalid_reasons.append("required_topic_type_missing")
    authority_publishers = sorted(FORBIDDEN_AUTHORITY_TOPICS.intersection(own_publishers))
    if authority_publishers:
        invalid_reasons.append("observer_has_authority_publisher")
    authority_subscriptions = sorted(
        FORBIDDEN_AUTHORITY_TOPICS.intersection(own_subscriptions)
    )
    if authority_subscriptions:
        invalid_reasons.append("observer_has_authority_subscription")
    actual_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if actual_domain_id != expected_domain_id:
        invalid_reasons.append("unexpected_ros_domain_id")
    if live_control_output_enabled or instant_control_enabled:
        invalid_reasons.append("observer_control_output_enabled")

    reasons = sorted(set(invalid_reasons))
    return {
        "observer": "read_only_control_smoke",
        "evidence_scope": "phase1_non_authoritative_runtime_smoke_only",
        "outcome": "PASS" if not reasons else "INVALID_EVIDENCE",
        "reasons": reasons,
        "started_monotonic_ns": started_monotonic_ns,
        "finished_monotonic_ns": finished_monotonic_ns,
        "clock": {
            "samples": len(clocks),
            "first_ns": clocks[0] if clocks else None,
            "last_ns": clocks[-1] if clocks else None,
        },
        "odometry": {
            "topic": "/localization/kinematic_state",
            "samples": len(odom_stamps),
            "first_stamp_ns": odom_stamps[0] if odom_stamps else None,
            "last_stamp_ns": odom_stamps[-1] if odom_stamps else None,
            "frames": sorted(frames),
        },
        "graph": {
            "required_nodes": sorted(required_nodes),
            "missing_nodes": missing_nodes,
            "required_topic_types": required_topic_types,
            "missing_topic_types": missing_topic_types,
            "observer_publishers": own_publishers,
            "observer_subscriptions": own_subscriptions,
        },
        "authority": {
            "forbidden_topics": sorted(FORBIDDEN_AUTHORITY_TOPICS),
            "observer_authority_publishers": authority_publishers,
            "observer_authority_subscriptions": authority_subscriptions,
            "live_control_output_enabled": live_control_output_enabled,
            "instant_control_enabled": instant_control_enabled,
        },
        "ros_domain_id": {
            "expected": expected_domain_id,
            "actual": actual_domain_id,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--expected-domain-id", type=int, required=True)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 60.0:
        parser.error("--duration must be between 1 and 60 seconds")
    payload = observe(args.duration, args.expected_domain_id)
    print(json.dumps(payload, ensure_ascii=False, allow_nan=False))
    return 0 if payload["outcome"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
