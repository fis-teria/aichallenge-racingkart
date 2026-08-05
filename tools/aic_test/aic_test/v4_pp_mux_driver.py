#!/usr/bin/env python3
"""Bounded, private inputs for the V4 -> Pure Pursuit -> Mux Stage-2 fixture."""

from __future__ import annotations

import argparse
import time


PRIVATE_ROOT_PREFIX = "/aic_test/v4_pp_mux_stage2/"
PUBLISH_PERIOD_S = 0.02
DEFAULT_GENERATION = 41
VALID_V4_VERSION = 4.0
DEFAULT_POST_INVALID_DWELL_S = 1.0


def _validate_private_root(root: str) -> bool:
    suffix = root.removeprefix(PRIVATE_ROOT_PREFIX)
    return bool(
        root.startswith(PRIVATE_ROOT_PREFIX)
        and suffix
        and all(character.isalnum() or character in "_-" for character in suffix)
        and "/" not in suffix
    )


def _stamp(stamp: object, nanoseconds: int) -> None:
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000


def _v4_payload(generation: int, *, valid: bool) -> list[float]:
    """Return a valid positive-left spatial V4, or a representative reject."""
    if not valid:
        return []
    distances_m = [0.4 * index for index in range(21)]
    lateral_offsets_m = [min(0.60, 0.12 * index) for index in range(21)]
    speed_caps_mps = [1.50] * len(distances_m)
    return [
        1.0,
        7.0,
        float(len(distances_m)),
        *lateral_offsets_m,
        *speed_caps_mps,
        *distances_m,
        VALID_V4_VERSION,
        float(generation),
        2.0,
    ]


def _phase(
    elapsed_s: float, baseline_duration_s: float, valid_duration_s: float
) -> str:
    if elapsed_s < baseline_duration_s:
        return "baseline"
    if elapsed_s < baseline_duration_s + valid_duration_s:
        return "valid"
    return "invalid"


def _effective_duration_s(
    baseline_duration_s: float, valid_duration_s: float, post_invalid_dwell_s: float
) -> float:
    return baseline_duration_s + valid_duration_s + post_invalid_dwell_s


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--private-root", required=True)
    parser.add_argument("--duration", type=float, default=7.0)
    parser.add_argument("--baseline-duration", type=float, default=1.5)
    parser.add_argument("--valid-duration", type=float, default=3.5)
    parser.add_argument(
        "--post-invalid-dwell", type=float, default=DEFAULT_POST_INVALID_DWELL_S
    )
    parser.add_argument("--generation", type=int, default=DEFAULT_GENERATION)
    args = parser.parse_args()
    if not (
        _validate_private_root(args.private_root)
        and 2.0 <= args.duration <= 30.0
        and 0.5 <= args.baseline_duration <= 5.0
        and 1.0 <= args.valid_duration
        and 0.75 <= args.post_invalid_dwell <= 5.0
        and args.baseline_duration + args.valid_duration + args.post_invalid_dwell
        <= args.duration
        and 1 <= args.generation <= 16_777_215
    ):
        parser.error("bounded private Stage-2 configuration required")

    import rclpy
    from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
    from multi_purpose_mpc_ros_msgs.msg import OvertakePlan, SafetyConstraint
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray, String

    rclpy.init(args=None)
    node = rclpy.create_node("aic_test_v4_pp_mux_driver")
    root = args.private_root.rstrip("/")
    odom_pub = node.create_publisher(Odometry, f"{root}/input/kinematics", 10)
    trajectory_pub = node.create_publisher(
        Trajectory, f"{root}/input/trajectory", 10
    )
    v4_pub = node.create_publisher(
        Float32MultiArray, f"{root}/input/reference_override", 10
    )
    plan_pub = node.create_publisher(OvertakePlan, f"{root}/input/plan", 10)
    constraint_pub = node.create_publisher(
        SafetyConstraint, f"{root}/input/safety_constraint", 10
    )
    arm_qos = QoSProfile(depth=1)
    arm_qos.reliability = ReliabilityPolicy.RELIABLE
    arm_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    armed_pub = node.create_publisher(Bool, f"{root}/input/race_armed", arm_qos)
    awsim_pub = node.create_publisher(String, f"{root}/input/awsim_state", 10)

    trajectory = Trajectory()
    trajectory.header.frame_id = "map"
    for index in range(81):
        point = TrajectoryPoint()
        point.pose.position.x = 0.20 * index
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 2.0
        point.time_from_start.nanosec = (index * 10_000_000) % 1_000_000_000
        point.time_from_start.sec = (index * 10_000_000) // 1_000_000_000
        trajectory.points.append(point)

    odometry = Odometry()
    odometry.header.frame_id = "map"
    odometry.child_frame_id = "base_link"
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 0.5

    start_s = time.monotonic()
    effective_duration_s = _effective_duration_s(
        args.baseline_duration, args.valid_duration, args.post_invalid_dwell
    )
    next_publish_s = start_s
    armed_sent = False
    try:
        while rclpy.ok():
            now_s = time.monotonic()
            elapsed_s = now_s - start_s
            if elapsed_s >= effective_duration_s:
                break
            if now_s < next_publish_s:
                rclpy.spin_once(node, timeout_sec=min(0.01, next_publish_s - now_s))
                continue
            next_publish_s = now_s + PUBLISH_PERIOD_S
            stamp_ns = node.get_clock().now().nanoseconds
            _stamp(odometry.header.stamp, stamp_ns)
            _stamp(trajectory.header.stamp, stamp_ns)

            plan = OvertakePlan()
            _stamp(plan.header.stamp, stamp_ns)
            plan.header.frame_id = "map"
            plan.phase = OvertakePlan.FREE_RUN
            plan.plan_generation = args.generation
            plan.safety_inputs_complete = True
            plan.tracking_usable = True
            plan.trajectory_publishable = True

            constraint = SafetyConstraint()
            _stamp(constraint.header.stamp, stamp_ns)
            constraint.header.frame_id = "map"
            constraint.constraint_generation = args.generation
            constraint.plan_generation = args.generation
            constraint.valid = True
            constraint.release_authorized = True
            constraint.stop_requested = False
            constraint.speed_limit_mps = 2.0
            constraint.required_brake_decel_mps2 = 1.0
            constraint.reason = "aic_test_v4_pp_mux_stage2"

            v4 = Float32MultiArray()
            v4.data = _v4_payload(
                args.generation,
                valid=_phase(
                    elapsed_s, args.baseline_duration, args.valid_duration
                ) == "valid",
            )
            odom_pub.publish(odometry)
            trajectory_pub.publish(trajectory)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            v4_pub.publish(v4)

            if elapsed_s < 0.25:
                armed_pub.publish(Bool(data=False))
                awsim_pub.publish(String(data="Ready"))
            else:
                armed_pub.publish(Bool(data=True))
                awsim_pub.publish(String(data="Start"))
                armed_sent = True
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        if armed_sent:
            armed_pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
