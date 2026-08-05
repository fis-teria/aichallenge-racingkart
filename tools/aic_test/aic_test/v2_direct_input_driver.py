#!/usr/bin/env python3
"""Minimal private synthetic inputs for the direct observer-only V2 smoke."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time


CLOCK_PERIOD_S = 0.001
MAX_CLOCK_STEP_NS = 10_000_000
ODOMETRY_PERIOD_S = 0.010
TRAJECTORY_PERIOD_S = 0.050
V2X_PERIOD_S = 0.050
RACE_STATE_PERIOD_S = 0.050
INPUT_PERIODS_S = {
    "odometry": ODOMETRY_PERIOD_S,
    "trajectory": TRAJECTORY_PERIOD_S,
    "v2x": V2X_PERIOD_S,
    "race_state": RACE_STATE_PERIOD_S,
}


def _schedule_due(now_s: float, next_due_s: float, period_s: float) -> tuple[bool, float]:
    """Return one bounded publication opportunity; never catch up in a burst."""
    if now_s < next_due_s:
        return False, next_due_s
    return True, now_s + period_s


def _due_input_channels(
    now_s: float, next_due_s: dict[str, float]
) -> tuple[tuple[str, ...], dict[str, float]]:
    updated = dict(next_due_s)
    due: list[str] = []
    for channel, period_s in INPUT_PERIODS_S.items():
        channel_due, updated[channel] = _schedule_due(
            now_s, updated[channel], period_s
        )
        if channel_due:
            due.append(channel)
    return tuple(due), updated


def _input_stamp_ns(clock_stamp_ns: int | None, last_clock_ns: int) -> int:
    stamp_ns = clock_stamp_ns if clock_stamp_ns is not None else last_clock_ns
    if stamp_ns <= 0:
        raise RuntimeError("input_publish_without_clock")
    return stamp_ns


def _validate_clock_step(previous_ns: int, current_ns: int) -> None:
    if previous_ns > 0 and not (previous_ns < current_ns <= previous_ns + MAX_CLOCK_STEP_NS):
        raise RuntimeError("clock_step_out_of_bounds")


def _stamp(message: object, nanoseconds: int) -> None:
    stamp = getattr(message, "stamp", message)
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000


def _write_marker_atomic(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write("complete\n")
        stream.flush()
        os.fsync(stream.fileno())
        os.replace(temporary, path)


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o644)
            json.dump(payload, stream, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        if (path.stat().st_mode & 0o777) != 0o644:
            raise OSError(f"host-readable mode mismatch: {path}")
        with path.open(encoding="utf-8") as stream:
            json.load(stream)
    except Exception:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        raise


def _process_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--input-root", required=True)
    parser.add_argument("--start-gate-file")
    parser.add_argument("--primed-file")
    parser.add_argument("--run-nonce")
    parser.add_argument("--capture-complete-file")
    parser.add_argument("--terminal-graph-file")
    parser.add_argument("--terminal-graph-timeout", type=float, default=5.0)
    parser.add_argument("--supervisor-pid", type=int)
    args = parser.parse_args()
    if not (
        1.0 <= args.duration <= 30.0
        and args.input_root.startswith("/aic_test/v2/input")
        and 0.1 <= args.terminal_graph_timeout <= 30.0
        and (args.supervisor_pid is None or args.supervisor_pid > 0)
    ):
        parser.error("bounded private input configuration required")
    if args.start_gate_file and args.supervisor_pid is None:
        parser.error("--start-gate-file requires --supervisor-pid")

    import rclpy
    from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool
    from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=None)
    node = rclpy.create_node("aic_test_v2_direct_input_driver")
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    odom_pub = node.create_publisher(
        Odometry, f"{args.input_root}/kinematics", 10
    )
    v2x_pub = node.create_publisher(
        V2XVehiclePositionArray, f"{args.input_root}/v2x", 10
    )
    trajectory_pub = node.create_publisher(
        Trajectory, f"{args.input_root}/trajectory", 10
    )
    arm_qos = QoSProfile(depth=1)
    arm_qos.reliability = ReliabilityPolicy.RELIABLE
    arm_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    disarm_pub = node.create_publisher(Bool, f"{args.input_root}/race_armed", arm_qos)
    ego_x_m = 89633.30679316094
    ego_y_m = 43131.17568487456
    ego_yaw_rad = 2.216058185742633
    trajectory = Trajectory()
    trajectory.header.frame_id = "map"
    for index in range(64):
        distance_m = 0.20 * index
        point = TrajectoryPoint()
        point.pose.position.x = ego_x_m + math.cos(ego_yaw_rad) * distance_m
        point.pose.position.y = ego_y_m + math.sin(ego_yaw_rad) * distance_m
        point.pose.orientation.z = math.sin(ego_yaw_rad * 0.5)
        point.pose.orientation.w = math.cos(ego_yaw_rad * 0.5)
        point.longitudinal_velocity_mps = 2.0
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = index * 10_000_000
        trajectory.points.append(point)
    v2x = V2XVehiclePositionArray()
    v2x.header.frame_id = "map"
    opponent = V2XVehiclePosition()
    opponent.header.frame_id = "map"
    opponent.vehicle_id = "d3"
    opponent.position.x = ego_x_m + math.cos(ego_yaw_rad) * 9.0
    opponent.position.y = ego_y_m + math.sin(ego_yaw_rad) * 9.0
    opponent.covariance.x = 0.0
    opponent.covariance.y = 0.0
    v2x.vehicles.append(opponent)
    exit_code = 0
    capture_started = False

    last_ros_ns = 0
    last_published_clock_ns = 0

    def next_ros_time_ns() -> int:
        nonlocal last_ros_ns
        now_ns = max(time.monotonic_ns(), last_ros_ns + 1)
        last_ros_ns = now_ns
        return now_ns

    def publish_clock() -> int:
        nonlocal last_published_clock_ns
        now_ns = next_ros_time_ns()
        _validate_clock_step(last_published_clock_ns, now_ns)
        clock = Clock()
        _stamp(clock.clock, now_ns)
        clock_pub.publish(clock)
        last_published_clock_ns = now_ns
        return now_ns

    def publish_odometry(now_ns: int) -> None:
        odom = Odometry()
        odom.header.frame_id = "map"
        _stamp(odom.header.stamp, now_ns)
        odom.pose.pose.position.x = ego_x_m
        odom.pose.pose.position.y = ego_y_m
        odom.pose.pose.orientation.z = math.sin(ego_yaw_rad * 0.5)
        odom.pose.pose.orientation.w = math.cos(ego_yaw_rad * 0.5)
        odom.twist.twist.linear.x = 0.0
        odom_pub.publish(odom)

    def publish_trajectory(now_ns: int) -> None:
        _stamp(trajectory.header.stamp, now_ns)
        trajectory_pub.publish(trajectory)

    def publish_v2x(now_ns: int) -> None:
        _stamp(v2x.header.stamp, now_ns)
        _stamp(opponent.header.stamp, now_ns)
        v2x_pub.publish(v2x)

    def publish_race_state(armed: bool) -> None:
        disarm_pub.publish(Bool(data=armed))

    def run_scheduled_tick(
        now_s: float, next_clock_s: float, next_input_s: dict[str, float], armed: bool
    ) -> tuple[float, dict[str, float], int | None, tuple[str, ...]]:
        clock_due, next_clock_s = _schedule_due(
            now_s, next_clock_s, CLOCK_PERIOD_S
        )
        due_channels, next_input_s = _due_input_channels(now_s, next_input_s)
        clock_ns = publish_clock() if clock_due else None
        if due_channels:
            input_stamp_ns = _input_stamp_ns(clock_ns, last_published_clock_ns)
            if "odometry" in due_channels:
                publish_odometry(input_stamp_ns)
            if "trajectory" in due_channels:
                publish_trajectory(input_stamp_ns)
            if "v2x" in due_channels:
                publish_v2x(input_stamp_ns)
            if "race_state" in due_channels:
                publish_race_state(armed=armed)
        rclpy.spin_once(node, timeout_sec=0.0)
        return next_clock_s, next_input_s, clock_ns, due_channels

    try:
        gate_file = Path(args.start_gate_file) if args.start_gate_file else None
        primed_file = Path(args.primed_file) if args.primed_file else None
        priming_clock_ns: list[int] = []
        next_clock_s = time.monotonic()
        next_input_s = {channel: next_clock_s for channel in INPUT_PERIODS_S}
        while gate_file is not None and not gate_file.is_file():
            # Startup timing belongs to the runner. Keep the private baseline
            # disarmed until its gate arrives, but fail closed if that runner
            # disappears instead of inventing a second local startup deadline.
            if not _process_is_alive(args.supervisor_pid):
                exit_code = 2
                break
            now_s = time.monotonic()
            next_clock_s, next_input_s, clock_ns, due_channels = run_scheduled_tick(
                now_s, next_clock_s, next_input_s, armed=False
            )
            if "race_state" in due_channels and clock_ns is not None:
                priming_clock_ns.append(clock_ns)
            if primed_file is not None and not primed_file.exists() and len(priming_clock_ns) >= 3:
                _write_json_atomic(
                    primed_file,
                    {
                        "schema_version": 1,
                        "run_nonce": args.run_nonce,
                        "input_root": args.input_root,
                        "sample_count": len(priming_clock_ns),
                        "first_clock_ns": priming_clock_ns[0],
                        "last_clock_ns": priming_clock_ns[-1],
                        "strictly_increasing_clock": all(
                            earlier < later
                            for earlier, later in zip(
                                priming_clock_ns, priming_clock_ns[1:]
                            )
                        ),
                        "race_armed": False,
                        "baseline_published": {
                            "clock": True,
                            "odometry": True,
                            "trajectory": True,
                            "v2x": True,
                        },
                        "frames": {"odometry": "map", "trajectory": "map", "v2x": "map"},
                    },
                )
            time.sleep(min(0.001, max(0.0, next_clock_s - time.monotonic())))
        if exit_code == 0:
            capture_started = True
        started = time.monotonic()
        deadline = started + args.duration
        next_clock_s = started
        next_input_s = {channel: started for channel in INPUT_PERIODS_S}
        while capture_started and rclpy.ok() and time.monotonic() < deadline:
            now_s = time.monotonic()
            remaining_s = deadline - now_s
            armed = now_s - started >= 1.0 and remaining_s > 0.25
            next_clock_s, next_input_s, _, _ = run_scheduled_tick(
                now_s, next_clock_s, next_input_s, armed=armed
            )
            time.sleep(min(0.001, max(0.0, next_clock_s - time.monotonic())))
    finally:
        # Reserve a bounded tail for a repeated private disarm before teardown.
        for _ in range(3):
            disarm_pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.02)
        if capture_started and args.capture_complete_file:
            _write_json_atomic(
                Path(args.capture_complete_file),
                {
                    "schema_version": 1,
                    "run_nonce": args.run_nonce,
                    "capture_complete": True,
                    "duration_s": args.duration,
                    "terminal_graph_file": args.terminal_graph_file,
                },
            )
        if capture_started and args.terminal_graph_file:
            terminal_deadline = time.monotonic() + args.terminal_graph_timeout
            terminal_file = Path(args.terminal_graph_file)
            while not terminal_file.is_file() and time.monotonic() < terminal_deadline:
                disarm_pub.publish(Bool(data=False))
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.05)
            if not terminal_file.is_file():
                exit_code = 2
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
