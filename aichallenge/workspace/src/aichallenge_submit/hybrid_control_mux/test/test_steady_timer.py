import time
import json

import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from multi_purpose_mpc_ros_msgs.msg import (
    OvertakePlan,
    RecoveryControlCommand,
    RecoveryPermit,
    RecoveryStatus,
    SafetyConstraint,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from hybrid_control_mux.hybrid_control_mux_node import HybridControlMuxNode


def _set_stamp(stamp, sec: int) -> None:
    stamp.sec = sec
    stamp.nanosec = 0


def test_nonadvancing_input_stamps_cannot_overwrite_newer_payloads():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        newer_pp = AckermannControlCommand()
        _set_stamp(newer_pp.stamp, 2)
        newer_pp.longitudinal.speed = 1.0
        mux.on_pure_pursuit_cmd(newer_pp)
        pp_receipt_time = mux.pure_pursuit_cmd_time_sec

        stale_pp = AckermannControlCommand()
        _set_stamp(stale_pp.stamp, 1)
        stale_pp.longitudinal.speed = 9.0
        mux.on_pure_pursuit_cmd(stale_pp)
        same_stamp_pp = AckermannControlCommand()
        _set_stamp(same_stamp_pp.stamp, 2)
        same_stamp_pp.longitudinal.speed = 8.0
        mux.on_pure_pursuit_cmd(same_stamp_pp)
        assert mux.pure_pursuit_cmd.longitudinal.speed == 1.0
        assert mux.pure_pursuit_cmd_time_sec == pp_receipt_time

        newer_mpc = AckermannControlCommand()
        _set_stamp(newer_mpc.stamp, 4)
        newer_mpc.longitudinal.speed = 2.0
        mux.on_mpc_cmd(newer_mpc)
        stale_mpc = AckermannControlCommand()
        _set_stamp(stale_mpc.stamp, 3)
        stale_mpc.longitudinal.speed = 10.0
        mux.on_mpc_cmd(stale_mpc)
        assert mux.mpc_cmd.longitudinal.speed == 2.0

        newer_recovery = RecoveryControlCommand()
        _set_stamp(newer_recovery.header.stamp, 6)
        newer_recovery.command.longitudinal.speed = 0.5
        mux.on_recovery_cmd(newer_recovery)
        stale_recovery = RecoveryControlCommand()
        _set_stamp(stale_recovery.header.stamp, 5)
        stale_recovery.command.longitudinal.speed = 4.0
        mux.on_recovery_cmd(stale_recovery)
        assert mux.recovery_cmd.command.longitudinal.speed == 0.5

        newer_status = RecoveryStatus()
        _set_stamp(newer_status.header.stamp, 8)
        newer_status.state = RecoveryStatus.ACTIVE
        mux.on_recovery_status(newer_status)
        stale_status = RecoveryStatus()
        _set_stamp(stale_status.header.stamp, 7)
        stale_status.state = RecoveryStatus.INACTIVE
        mux.on_recovery_status(stale_status)
        assert mux.recovery_status.state == RecoveryStatus.ACTIVE

        newer_permit = RecoveryPermit()
        _set_stamp(newer_permit.header.stamp, 10)
        newer_permit.recovery_allowed = False
        mux.on_recovery_permit(newer_permit)
        stale_permit = RecoveryPermit()
        _set_stamp(stale_permit.header.stamp, 9)
        stale_permit.recovery_allowed = True
        mux.on_recovery_permit(stale_permit)
        assert mux.recovery_permit.recovery_allowed is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_mux_publishes_stop_while_sim_clock_is_frozen():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "control_rate_hz:=50.0",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_steady_timer_test_observer")
    received = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: received.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 0.40
        while time.monotonic() < deadline and len(received) < 3:
            executor.spin_once(timeout_sec=0.02)

        assert len(received) >= 2
        assert all(msg.longitudinal.speed == 0.0 for msg in received)
        assert all(msg.longitudinal.acceleration < 0.0 for msg in received)
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_same_stamp_republication_cannot_hide_ros_clock_stall():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "control_rate_hz:=50.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "ros_clock_stall_timeout_sec:=0.15",
            "-p",
            "control_fault_clear_safe_cycles:=1",
            "-p",
            "debug_publish_period_sec:=0.01",
        ]
    )
    mux = HybridControlMuxNode()
    peer = Node("hybrid_control_mux_clock_stall_peer")
    commands = []
    debug = []
    peer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    peer.create_subscription(
        String,
        "output/debug",
        lambda msg: debug.append(json.loads(msg.data)),
        10,
    )
    command_pub = peer.create_publisher(
        AckermannControlCommand, "input/pure_pursuit_control_cmd", 10
    )
    plan_pub = peer.create_publisher(OvertakePlan, "input/overtake_plan", 10)
    constraint_pub = peer.create_publisher(
        SafetyConstraint, "input/safety_constraint", 10
    )

    command = AckermannControlCommand()
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.5
    plan = OvertakePlan()
    plan.header.frame_id = "map"
    plan.plan_generation = 1
    constraint = SafetyConstraint()
    constraint.header.frame_id = "map"
    constraint.constraint_generation = 1
    constraint.plan_generation = 1
    constraint.valid = True
    constraint.release_authorized = True
    constraint.speed_limit_mps = 3.0
    mux.on_pure_pursuit_cmd(command)
    mux.on_overtake_plan(plan)
    mux.on_safety_constraint(constraint)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(peer)
    try:
        deadline = time.monotonic() + 0.40
        while time.monotonic() < deadline:
            # All headers intentionally remain at ROS stamp zero while no
            # /clock is published. Receipt traffic must not keep motion fresh.
            command_pub.publish(command)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            executor.spin_once(timeout_sec=0.01)

        assert any(msg.longitudinal.speed > 0.0 for msg in commands)
        assert commands[-1].longitudinal.speed == 0.0
        assert any(item.get("ros_clock_stalled") is True for item in debug)
    finally:
        executor.remove_node(peer)
        executor.remove_node(mux)
        peer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()
