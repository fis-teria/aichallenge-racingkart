#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import math
from typing import Optional

import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from multi_purpose_mpc_ros_msgs.msg import (
    OvertakePlan,
    RecoveryControlCommand,
    RecoveryPermit,
    RecoveryStatus,
    SafetyConstraint,
    SafetyStopStatus,
)
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from hybrid_control_mux.core import (
    ControlLoopWatchdog,
    HybridMuxConfig,
    HybridMuxCore,
    MpcHealth,
    RecoveryMuxState,
    RosClockProgressWatchdog,
    SafetyConstraintAuthority,
    SafetyConstraintDecision,
    SafetyConstraintState,
    SteeringLimitResult,
    SteeringLimiter,
    SteeringLimiterConfig,
    apply_safety_constraint,
)


class HybridControlMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_control_mux_node")

        self.enabled = self.declare_parameter("enabled", True).value
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.mpc_cmd_timeout_sec = float(
            self.declare_parameter("mpc_cmd_timeout_sec", 0.20).value
        )
        self.pure_pursuit_cmd_timeout_sec = float(
            self.declare_parameter("pure_pursuit_cmd_timeout_sec", 0.20).value
        )
        self.mpc_health_timeout_sec = float(
            self.declare_parameter("mpc_health_timeout_sec", 0.75).value
        )
        self.safety_constraint_enabled = bool(
            self.declare_parameter("safety_constraint_enabled", True).value
        )
        self.require_safety_constraint = bool(
            self.declare_parameter("require_safety_constraint", False).value
        )
        self.safety_constraint_timeout_sec = float(
            self.declare_parameter("safety_constraint_timeout_sec", 0.20).value
        )
        self.overtake_plan_timeout_sec = float(
            self.declare_parameter("overtake_plan_timeout_sec", 0.20).value
        )
        self.safety_constraint_max_speed_mps = float(
            self.declare_parameter("safety_constraint_max_speed_mps", 15.0).value
        )
        self.safety_constraint_max_brake_decel_mps2 = float(
            self.declare_parameter(
                "safety_constraint_max_brake_decel_mps2", 1.5
            ).value
        )
        self.fallback_speed_mps = float(
            self.declare_parameter("fallback_speed_mps", 2.0).value
        )
        self.fallback_accel_max_mps2 = float(
            self.declare_parameter("fallback_accel_max_mps2", 0.8).value
        )
        self.fallback_decel_min_mps2 = float(
            self.declare_parameter("fallback_decel_min_mps2", -1.5).value
        )
        self.recovery_enabled = bool(
            self.declare_parameter("recovery_enabled", False).value
        )
        self.recovery_cmd_timeout_sec = float(
            self.declare_parameter("recovery_cmd_timeout_sec", 0.15).value
        )
        self.recovery_status_timeout_sec = float(
            self.declare_parameter("recovery_status_timeout_sec", 0.20).value
        )
        self.recovery_permit_timeout_sec = float(
            self.declare_parameter("recovery_permit_timeout_sec", 0.30).value
        )
        self.external_safety_timeout_sec = float(
            self.declare_parameter("external_safety_timeout_sec", 0.20).value
        )
        self.require_recovery_external_safety_status = bool(
            self.declare_parameter("require_recovery_external_safety_status", True).value
        )
        self.recovery_speed_mps = float(
            self.declare_parameter("recovery_speed_mps", 0.7).value
        )
        self.recovery_accel_max_mps2 = float(
            self.declare_parameter("recovery_accel_max_mps2", 0.5).value
        )
        self.recovery_decel_min_mps2 = float(
            self.declare_parameter("recovery_decel_min_mps2", -1.5).value
        )
        self.stop_decel_mps2 = float(self.declare_parameter("stop_decel_mps2", -1.5).value)
        self.debug_publish_period_sec = float(
            self.declare_parameter("debug_publish_period_sec", 0.25).value
        )
        self.steering_log_throttle_sec = float(
            self.declare_parameter("steering_log_throttle_sec", 1.0).value
        )
        self.last_steering_limit_log_sec = -1.0e9
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        config = HybridMuxConfig(
            primary_source=str(
                self.declare_parameter("primary_source", "mpc").value
            ),
            fallback_trigger_infeasible_count=int(
                self.declare_parameter("fallback_trigger_infeasible_count", 2).value
            ),
            fallback_release_solved_cycles=int(
                self.declare_parameter("fallback_release_solved_cycles", 3).value
            ),
            fallback_min_hold_sec=float(
                self.declare_parameter("fallback_min_hold_sec", 1.0).value
            ),
            use_pure_pursuit_on_mpc_cmd_timeout=bool(
                self.declare_parameter("use_pure_pursuit_on_mpc_cmd_timeout", True).value
            ),
            use_pure_pursuit_on_mpc_health_timeout=bool(
                self.declare_parameter("use_pure_pursuit_on_mpc_health_timeout", False).value
            ),
            use_mpc_on_pure_pursuit_cmd_timeout=bool(
                self.declare_parameter("use_mpc_on_pure_pursuit_cmd_timeout", False).value
            ),
            recovery_max_duration_sec=float(
                self.declare_parameter("recovery_max_duration_sec", 3.0).value
            ),
        )
        self.core = HybridMuxCore(config)
        self.safety_authority = SafetyConstraintAuthority(
            required=self.safety_constraint_enabled and self.require_safety_constraint,
            timeout_sec=self.safety_constraint_timeout_sec,
            maximum_speed_limit_mps=self.safety_constraint_max_speed_mps,
            maximum_brake_decel_mps2=self.safety_constraint_max_brake_decel_mps2,
        )
        period = 1.0 / max(1.0, self.control_rate_hz)
        self.control_loop_watchdog = ControlLoopWatchdog(
            float(
                self.declare_parameter(
                    "control_loop_max_gap_sec", max(0.10, period * 3.0)
                ).value
            )
        )
        self.ros_clock_watchdog = RosClockProgressWatchdog(
            float(
                self.declare_parameter(
                    "ros_clock_stall_timeout_sec", 0.20
                ).value
            )
        )
        self.control_fault_clear_safe_cycles = max(
            1,
            int(
                self.declare_parameter(
                    "control_fault_clear_safe_cycles", 3
                ).value
            ),
        )
        self.control_fault_latched = False
        self.control_fault_clear_cycles = 0
        self.control_fault_reason = ""
        self.control_fault_last_release_stamp_ns: Optional[int] = None
        self.steering_limiter = SteeringLimiter(
            SteeringLimiterConfig(
                enabled=bool(self.declare_parameter("enable_steering_rate_limit", True).value),
                max_steering_angle_rad=float(
                    self.declare_parameter("max_steering_angle_rad", 0.5585053606381855).value
                ),
                max_steering_rate_radps=float(
                    self.declare_parameter("max_steering_rate_radps", 8.0).value
                ),
                max_steering_delta_per_cycle=float(
                    self.declare_parameter("max_steering_delta_per_cycle", 0.0).value
                ),
                reset_dt_threshold_sec=float(
                    self.declare_parameter("steering_limiter_reset_dt_sec", 0.50).value
                ),
                reset_on_mode_change=bool(
                    self.declare_parameter("reset_steering_limiter_on_mode_change", False).value
                ),
            )
        )

        self.mpc_cmd: Optional[AckermannControlCommand] = None
        self.mpc_cmd_time_sec: Optional[float] = None
        self.mpc_cmd_stamp_ns: Optional[int] = None
        self.pure_pursuit_cmd: Optional[AckermannControlCommand] = None
        self.pure_pursuit_cmd_time_sec: Optional[float] = None
        self.pure_pursuit_cmd_stamp_ns: Optional[int] = None
        self.recovery_cmd: Optional[RecoveryControlCommand] = None
        self.recovery_cmd_time_sec: Optional[float] = None
        self.recovery_cmd_stamp_ns: Optional[int] = None
        self.recovery_status: Optional[RecoveryStatus] = None
        self.recovery_status_time_sec: Optional[float] = None
        self.recovery_status_stamp_ns: Optional[int] = None
        self.recovery_permit: Optional[RecoveryPermit] = None
        self.recovery_permit_time_sec: Optional[float] = None
        self.recovery_permit_stamp_ns: Optional[int] = None
        self.external_safety_status: Optional[SafetyStopStatus] = None
        self.external_safety_time_sec: Optional[float] = None
        self.external_safety_stamp_ns: Optional[int] = None
        self.external_stop_latched = False
        self.safety_constraint: Optional[SafetyConstraintState] = None
        self.safety_constraint_time_sec: Optional[float] = None
        self.safety_constraint_header_stamp_ns: Optional[int] = None
        self.overtake_plan_generation: Optional[int] = None
        self.overtake_plan_time_sec: Optional[float] = None
        self.overtake_plan_header_stamp_ns: Optional[int] = None
        self.overtake_plan_valid = False
        self.mpc_health = MpcHealth()
        self.mpc_health_time_sec: Optional[float] = None
        self.last_debug_publish_sec = -1.0e9
        self.last_source = ""

        self.create_subscription(
            AckermannControlCommand,
            "input/mpc_control_cmd",
            self.on_mpc_cmd,
            1,
        )
        self.create_subscription(
            AckermannControlCommand,
            "input/pure_pursuit_control_cmd",
            self.on_pure_pursuit_cmd,
            1,
        )
        self.create_subscription(
            RecoveryControlCommand,
            "input/recovery_control_cmd",
            self.on_recovery_cmd,
            1,
        )
        self.create_subscription(
            RecoveryStatus,
            "input/recovery_status",
            self.on_recovery_status,
            1,
        )
        self.create_subscription(
            RecoveryPermit,
            "input/recovery_permit",
            self.on_recovery_permit,
            1,
        )
        self.create_subscription(
            SafetyStopStatus,
            "input/external_safety_status",
            self.on_external_safety_status,
            1,
        )
        self.create_subscription(
            SafetyConstraint,
            "input/safety_constraint",
            self.on_safety_constraint,
            1,
        )
        self.create_subscription(
            OvertakePlan,
            "input/overtake_plan",
            self.on_overtake_plan,
            1,
        )
        self.create_subscription(String, "input/mpc_health", self.on_mpc_health, 1)
        self.control_pub = self.create_publisher(AckermannControlCommand, "output/control_cmd", 1)
        self.debug_pub = self.create_publisher(String, "output/debug", 1)

        self.timer = self.create_timer(
            period, self.on_timer, clock=self.steady_clock
        )

    def now_sec(self) -> float:
        return self.steady_clock.now().nanoseconds / 1.0e9

    def on_mpc_cmd(self, msg: AckermannControlCommand) -> None:
        receipt_time_sec, stamp_ns, timestamp_advanced = self._advance_receipt_time(
            self.mpc_cmd_time_sec,
            self.mpc_cmd_stamp_ns,
            self._stamp_ns(msg.stamp),
        )
        if not timestamp_advanced:
            return
        self.mpc_cmd = msg
        self.mpc_cmd_time_sec = receipt_time_sec
        self.mpc_cmd_stamp_ns = stamp_ns

    def on_pure_pursuit_cmd(self, msg: AckermannControlCommand) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.pure_pursuit_cmd_time_sec,
            self.pure_pursuit_cmd_stamp_ns,
            self._stamp_ns(msg.stamp),
        )
        if not timestamp_advanced:
            return
        self.pure_pursuit_cmd = msg
        self.pure_pursuit_cmd_time_sec = receipt_time_sec
        self.pure_pursuit_cmd_stamp_ns = stamp_ns

    def on_recovery_cmd(self, msg: RecoveryControlCommand) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_cmd_time_sec,
            self.recovery_cmd_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_cmd = msg
        self.recovery_cmd_time_sec = receipt_time_sec
        self.recovery_cmd_stamp_ns = stamp_ns

    def on_recovery_status(self, msg: RecoveryStatus) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_status_time_sec,
            self.recovery_status_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_status = msg
        self.recovery_status_time_sec = receipt_time_sec
        self.recovery_status_stamp_ns = stamp_ns

    def on_recovery_permit(self, msg: RecoveryPermit) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_permit_time_sec,
            self.recovery_permit_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_permit = msg
        self.recovery_permit_time_sec = receipt_time_sec
        self.recovery_permit_stamp_ns = stamp_ns

    def on_external_safety_status(self, msg: SafetyStopStatus) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.external_safety_time_sec,
            self.external_safety_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not bool(msg.valid) or bool(msg.stop_requested):
            self.external_safety_status = msg
            self.external_stop_latched = True
            if timestamp_advanced:
                self.external_safety_time_sec = receipt_time_sec
                self.external_safety_stamp_ns = stamp_ns
            return
        if not timestamp_advanced:
            return
        self.external_safety_status = msg
        self.external_safety_time_sec = receipt_time_sec
        self.external_safety_stamp_ns = stamp_ns
        self.external_stop_latched = False

    def on_safety_constraint(self, msg: SafetyConstraint) -> None:
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        self.safety_constraint = SafetyConstraintState(
            constraint_generation=int(msg.constraint_generation),
            plan_generation=int(msg.plan_generation),
            valid=bool(msg.valid),
            stop_requested=bool(msg.stop_requested),
            release_authorized=bool(msg.release_authorized),
            speed_limit_mps=float(msg.speed_limit_mps),
            required_brake_decel_mps2=float(msg.required_brake_decel_mps2),
            header_stamp_ns=header_stamp_ns,
            frame_id=str(msg.header.frame_id),
            reason=str(msg.reason),
        )
        (
            self.safety_constraint_time_sec,
            self.safety_constraint_header_stamp_ns,
            _,
        ) = self._advance_receipt_time(
            self.safety_constraint_time_sec,
            self.safety_constraint_header_stamp_ns,
            header_stamp_ns,
        )

    def on_overtake_plan(self, msg: OvertakePlan) -> None:
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        timestamp_monotonic = (
            self.overtake_plan_header_stamp_ns is None
            or header_stamp_ns >= self.overtake_plan_header_stamp_ns
        )
        self.overtake_plan_generation = int(msg.plan_generation)
        (
            self.overtake_plan_time_sec,
            advanced_stamp_ns,
            _,
        ) = self._advance_receipt_time(
            self.overtake_plan_time_sec,
            self.overtake_plan_header_stamp_ns,
            header_stamp_ns,
        )
        trajectory_header_matches = (
            str(msg.trajectory.header.frame_id) == str(msg.header.frame_id)
            and self._stamp_ns(msg.trajectory.header.stamp) == header_stamp_ns
        )
        trajectory_points_valid = bool(msg.trajectory.points) and all(
            math.isfinite(float(point.pose.position.x))
            and math.isfinite(float(point.pose.position.y))
            and math.isfinite(float(point.longitudinal_velocity_mps))
            and float(point.longitudinal_velocity_mps) >= 0.0
            for point in msg.trajectory.points
        )
        trajectory_contract_required = bool(msg.trajectory_authorized) or bool(
            msg.lateral_maneuver_required
        )
        trajectory_contract_valid = (
            not trajectory_contract_required
            or (
                bool(msg.trajectory_authorized)
                and trajectory_header_matches
                and trajectory_points_valid
            )
        )
        self.overtake_plan_valid = (
            str(msg.header.frame_id) == "map"
            and header_stamp_ns >= 0
            and timestamp_monotonic
            and int(msg.phase)
            in (
                int(OvertakePlan.FREE_RUN),
                int(OvertakePlan.ATTACK_FOLLOW),
                int(OvertakePlan.PASSING),
                int(OvertakePlan.ABORT_HOLD),
            )
            and trajectory_contract_valid
        )
        if timestamp_monotonic:
            self.overtake_plan_header_stamp_ns = advanced_stamp_ns

    def on_mpc_health(self, msg: String) -> None:
        now_sec = self.now_sec()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.mpc_health = MpcHealth(valid=False, status="parse_error", age_sec=0.0)
            self.mpc_health_time_sec = now_sec
            return

        self.mpc_health = MpcHealth(
            valid=True,
            status=str(payload.get("mpc_status", "unknown")),
            infeasible_count=int(payload.get("mpc_infeasible_count", 0)),
            age_sec=0.0,
        )
        self.mpc_health_time_sec = now_sec

    def on_timer(self) -> None:
        now_sec = self.now_sec()
        watchdog = self.control_loop_watchdog.update(now_sec)
        ros_clock_watchdog = self.ros_clock_watchdog.update(
            int(self.get_clock().now().nanoseconds), now_sec
        )
        plan_fresh = self._fresh(
            self.overtake_plan_time_sec, now_sec, self.overtake_plan_timeout_sec
        )
        active_plan_generation = None
        if self.require_safety_constraint:
            active_plan_generation = (
                self.overtake_plan_generation
                if plan_fresh
                and self.overtake_plan_valid
                and self.overtake_plan_generation is not None
                else -1
            )
        constraint_decision = self.safety_authority.evaluate(
            self.safety_constraint if self.safety_constraint_enabled else None,
            received_time_sec=(
                self.safety_constraint_time_sec
                if self.safety_constraint_enabled
                else None
            ),
            now_sec=now_sec,
            active_plan_generation=active_plan_generation,
        )
        active_control_fault_reason = ""
        if watchdog.deadline_missed:
            active_control_fault_reason = "control_loop_deadline_missed"
        elif ros_clock_watchdog.stalled:
            active_control_fault_reason = ros_clock_watchdog.reason
        if active_control_fault_reason:
            self.control_fault_latched = True
            self.control_fault_clear_cycles = 0
            self.control_fault_reason = active_control_fault_reason
            self.control_fault_last_release_stamp_ns = None
        elif self.control_fault_latched:
            explicit_safe_release = (
                not constraint_decision.stop_required
                and self.safety_constraint is not None
                and self.safety_constraint.release_authorized
            )
            if explicit_safe_release:
                release_stamp_ns = self.safety_constraint.header_stamp_ns
                if (
                    self.control_fault_last_release_stamp_ns is None
                    or release_stamp_ns
                    > self.control_fault_last_release_stamp_ns
                ):
                    self.control_fault_clear_cycles += 1
                    self.control_fault_last_release_stamp_ns = release_stamp_ns
            else:
                self.control_fault_clear_cycles = 0
                self.control_fault_last_release_stamp_ns = None
            if (
                self.control_fault_clear_cycles
                >= self.control_fault_clear_safe_cycles
            ):
                self.control_fault_latched = False
                self.control_fault_clear_cycles = 0
                self.control_fault_reason = ""
                self.control_fault_last_release_stamp_ns = None
        mpc_cmd_fresh = self._fresh(self.mpc_cmd_time_sec, now_sec, self.mpc_cmd_timeout_sec)
        pp_cmd_fresh = self._fresh(
            self.pure_pursuit_cmd_time_sec, now_sec, self.pure_pursuit_cmd_timeout_sec
        )
        health = self._current_health(now_sec)
        recovery_state = self._current_recovery_state(now_sec)

        if self.external_stop_latched:
            decision_source = "stop"
            reason = "external_safety_stop"
            fallback_active = True
            solved_cycles = 0
        elif self.control_fault_latched:
            decision_source = "stop"
            reason = self.control_fault_reason or "control_fault_latched"
            fallback_active = True
            solved_cycles = 0
        elif not self.enabled:
            decision_source = "mpc" if mpc_cmd_fresh else "stop"
            reason = "disabled"
            fallback_active = False
            solved_cycles = 0
        else:
            decision = self.core.update(
                now_sec,
                mpc_cmd_fresh=mpc_cmd_fresh,
                pure_pursuit_cmd_fresh=pp_cmd_fresh,
                mpc_health=health,
                recovery=recovery_state,
            )
            decision_source = decision.source
            reason = decision.reason
            fallback_active = decision.fallback_active
            solved_cycles = decision.solved_cycles

        selected_input_cmd = self._selected_input_command(decision_source)
        cmd = self._select_command(
            decision_source, now_sec, selected_input_cmd)
        if self.safety_constraint_enabled:
            cmd, decision_source, reason = self._apply_safety_constraint(
                cmd, decision_source, reason, constraint_decision, now_sec
            )
        steering_result = self._limit_steering(cmd, decision_source, now_sec)
        self.control_pub.publish(cmd)
        self._publish_debug(
            now_sec=now_sec,
            source=decision_source,
            reason=reason,
            fallback_active=fallback_active,
            solved_cycles=solved_cycles,
            mpc_cmd_fresh=mpc_cmd_fresh,
            pure_pursuit_cmd_fresh=pp_cmd_fresh,
            health=health,
            recovery_state=recovery_state,
            selected_input_cmd=selected_input_cmd,
            output_cmd=cmd,
            steering_result=steering_result,
            constraint_decision=constraint_decision,
            control_loop_deadline_missed=watchdog.deadline_missed,
            control_publish_gap_sec=watchdog.publish_gap_sec,
            ros_clock_stalled=ros_clock_watchdog.stalled,
            ros_clock_stagnant_duration_sec=(
                ros_clock_watchdog.stagnant_duration_sec
            ),
            control_fault_latched=self.control_fault_latched,
            control_fault_clear_cycles=self.control_fault_clear_cycles,
            overtake_plan_fresh=plan_fresh,
            overtake_plan_valid=self.overtake_plan_valid,
            active_plan_generation=active_plan_generation,
        )

        if decision_source != self.last_source:
            self.get_logger().warn(
                f"hybrid control source={decision_source} reason={reason} "
                f"mpc_status={health.status} mpc_infeasible={health.infeasible_count}"
            )
            self.last_source = decision_source

    def _apply_safety_constraint(
        self,
        cmd: AckermannControlCommand,
        source: str,
        reason: str,
        decision: SafetyConstraintDecision,
        now_sec: float,
    ) -> tuple[AckermannControlCommand, str, str]:
        if not (
            math.isfinite(float(cmd.longitudinal.speed))
            and math.isfinite(float(cmd.longitudinal.acceleration))
        ):
            return self._stop_command(now_sec), "stop", "invalid_tracking_command"
        if decision.stop_required:
            stop_cmd = self._stop_command(now_sec)
            if decision.required_brake_decel_mps2 > 0.0:
                stop_cmd.longitudinal.acceleration = min(
                    float(stop_cmd.longitudinal.acceleration),
                    -decision.required_brake_decel_mps2,
                )
            return stop_cmd, "stop", decision.reason
        speed_mps, acceleration_mps2 = apply_safety_constraint(
            float(cmd.longitudinal.speed),
            float(cmd.longitudinal.acceleration),
            decision,
        )
        cmd.longitudinal.speed = speed_mps
        cmd.longitudinal.acceleration = acceleration_mps2
        if decision.reason == "safety_constraint_relaxation_not_authorized":
            reason = decision.reason
        return cmd, source, reason

    def _select_command(
        self,
        source: str,
        now_sec: float,
        selected_input_cmd: Optional[AckermannControlCommand],
    ) -> AckermannControlCommand:
        if source == "mpc" and selected_input_cmd is not None:
            return self._stamp(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "pure_pursuit" and selected_input_cmd is not None:
            return self._fallback_command(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "recovery" and selected_input_cmd is not None:
            return self._recovery_command(copy.deepcopy(selected_input_cmd), now_sec)
        return self._stop_command(now_sec)

    def _selected_input_command(
        self, source: str
    ) -> Optional[AckermannControlCommand]:
        if source == "mpc":
            return self.mpc_cmd
        if source == "pure_pursuit":
            return self.pure_pursuit_cmd
        if source == "recovery" and self.recovery_cmd is not None:
            return self.recovery_cmd.command
        return None

    def _fallback_command(
        self, cmd: AckermannControlCommand, now_sec: float
    ) -> AckermannControlCommand:
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = self._finite_clamp(
            cmd.longitudinal.speed, 0.0, max(0.0, self.fallback_speed_mps)
        )
        cmd.longitudinal.acceleration = self._finite_clamp(
            cmd.longitudinal.acceleration,
            self.fallback_decel_min_mps2,
            self.fallback_accel_max_mps2,
        )
        return cmd

    def _recovery_command(
        self, cmd: AckermannControlCommand, now_sec: float
    ) -> AckermannControlCommand:
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = self._finite_clamp(
            cmd.longitudinal.speed, 0.0, max(0.0, self.recovery_speed_mps)
        )
        cmd.longitudinal.acceleration = self._finite_clamp(
            cmd.longitudinal.acceleration,
            self.recovery_decel_min_mps2,
            self.recovery_accel_max_mps2,
        )
        return cmd

    def _limit_steering(
        self, cmd: AckermannControlCommand, source: str, now_sec: float
    ) -> SteeringLimitResult:
        if source == "stop":
            result = self.steering_limiter.reset(0.0, now_sec, source)
            cmd.lateral.steering_tire_angle = 0.0
            return result

        result = self.steering_limiter.update(
            float(cmd.lateral.steering_tire_angle), now_sec, source
        )
        cmd.lateral.steering_tire_angle = result.limited_steering_rad
        if result.angle_limited or result.rate_limited:
            if now_sec - self.last_steering_limit_log_sec >= self.steering_log_throttle_sec:
                self.get_logger().warn(
                    "hybrid steering limited "
                    f"source={source} raw={result.raw_steering_rad:.3f} "
                    f"limited={result.limited_steering_rad:.3f} "
                    f"delta={result.steering_delta_rad:.3f} "
                    f"angle_limited={result.angle_limited} "
                    f"rate_limited={result.rate_limited}"
                )
                self.last_steering_limit_log_sec = now_sec
        return result

    def _stop_command(self, now_sec: float) -> AckermannControlCommand:
        cmd = AckermannControlCommand()
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = 0.0
        cmd.longitudinal.acceleration = self.stop_decel_mps2
        cmd.lateral.steering_tire_angle = 0.0
        return cmd

    def _stamp(self, cmd: AckermannControlCommand, now_sec: float) -> AckermannControlCommand:
        stamp = self.get_clock().now().to_msg()
        cmd.stamp = stamp
        cmd.longitudinal.stamp = stamp
        cmd.lateral.stamp = stamp
        return cmd

    def _current_health(self, now_sec: float) -> MpcHealth:
        if self.mpc_health_time_sec is None:
            return MpcHealth(valid=False, status="missing", age_sec=float("inf"))
        age_sec = now_sec - self.mpc_health_time_sec
        if age_sec > self.mpc_health_timeout_sec:
            return MpcHealth(
                valid=False,
                status="stale",
                infeasible_count=self.mpc_health.infeasible_count,
                age_sec=age_sec,
            )
        return MpcHealth(
            valid=self.mpc_health.valid,
            status=self.mpc_health.status,
            infeasible_count=self.mpc_health.infeasible_count,
            age_sec=age_sec,
        )

    def _current_recovery_state(self, now_sec: float) -> RecoveryMuxState:
        status_fresh = self._fresh(
            self.recovery_status_time_sec, now_sec, self.recovery_status_timeout_sec
        )
        command_fresh = self._fresh(
            self.recovery_cmd_time_sec, now_sec, self.recovery_cmd_timeout_sec
        )
        permit_fresh = self._fresh(
            self.recovery_permit_time_sec, now_sec, self.recovery_permit_timeout_sec
        )
        safety_fresh = self._fresh(
            self.external_safety_time_sec, now_sec, self.external_safety_timeout_sec
        )
        if self.require_recovery_external_safety_status:
            external_safety_ok = (
                safety_fresh
                and self.external_safety_status is not None
                and bool(self.external_safety_status.valid)
                and not bool(self.external_safety_status.stop_requested)
            )
        else:
            external_safety_ok = (
                self.external_safety_status is None
                or not safety_fresh
                or (bool(self.external_safety_status.valid)
                    and not bool(self.external_safety_status.stop_requested))
            )

        status = self.recovery_status
        command = self.recovery_cmd
        permit = self.recovery_permit
        status_state = self._recovery_state_name(status.state) if status else "missing"
        ids_match = (
            status is not None
            and command is not None
            and permit is not None
            and status.attempt_id == command.attempt_id == permit.attempt_id
            and status.trajectory_generation
            == command.trajectory_generation
            == permit.trajectory_generation
        )
        trajectory_header_match = (
            status is not None
            and command is not None
            and status.trajectory_header.frame_id == command.trajectory_header.frame_id
            and status.trajectory_header.stamp.sec == command.trajectory_header.stamp.sec
            and status.trajectory_header.stamp.nanosec
            == command.trajectory_header.stamp.nanosec
        )
        command_valid = False
        command_forward_only = False
        if command is not None:
            cmd = command.command
            command_valid = (
                math.isfinite(float(cmd.longitudinal.speed))
                and math.isfinite(float(cmd.longitudinal.acceleration))
                and math.isfinite(float(cmd.lateral.steering_tire_angle))
            )
            command_forward_only = command_valid and float(cmd.longitudinal.speed) >= 0.0

        return RecoveryMuxState(
            enabled=self.recovery_enabled,
            status_state=status_state,
            status_fresh=status_fresh,
            command_fresh=command_fresh,
            permit_fresh=permit_fresh,
            external_safety_ok=external_safety_ok,
            input_complete=bool(status.input_complete) if status else False,
            trajectory_valid=bool(status.trajectory_valid) if status else False,
            trajectory_safe=bool(status.trajectory_safe) if status else False,
            stop_required=bool(status.stop_required) if status else False,
            handoff_ready=bool(status.handoff_ready) if status else False,
            recovery_allowed=bool(permit.recovery_allowed) if permit else False,
            handoff_allowed=bool(permit.handoff_allowed) if permit else False,
            ids_match=ids_match,
            trajectory_header_match=trajectory_header_match,
            command_valid=command_valid,
            command_forward_only=command_forward_only,
        )

    def _publish_debug(
        self,
        *,
        now_sec: float,
        source: str,
        reason: str,
        fallback_active: bool,
        solved_cycles: int,
        mpc_cmd_fresh: bool,
        pure_pursuit_cmd_fresh: bool,
        health: MpcHealth,
        recovery_state: RecoveryMuxState,
        selected_input_cmd: Optional[AckermannControlCommand],
        output_cmd: AckermannControlCommand,
        steering_result: SteeringLimitResult,
        constraint_decision: SafetyConstraintDecision,
        control_loop_deadline_missed: bool,
        control_publish_gap_sec: float,
        ros_clock_stalled: bool,
        ros_clock_stagnant_duration_sec: float,
        control_fault_latched: bool,
        control_fault_clear_cycles: int,
        overtake_plan_fresh: bool,
        overtake_plan_valid: bool,
        active_plan_generation: Optional[int],
    ) -> None:
        if self.debug_publish_period_sec <= 0.0:
            return
        if now_sec - self.last_debug_publish_sec < self.debug_publish_period_sec:
            return
        self.last_debug_publish_sec = now_sec

        msg = String()
        msg.data = json.dumps(
            {
                "controller": "hybrid_control_mux",
                "primary_source": self.core.config.primary_source,
                "source": source,
                "selected_source": source,
                "fallback_active": fallback_active,
                "reason": reason,
                "solved_cycles": solved_cycles,
                "mpc_cmd_fresh": mpc_cmd_fresh,
                "pure_pursuit_cmd_fresh": pure_pursuit_cmd_fresh,
                "mpc_health_valid": health.valid,
                "mpc_status": health.status,
                "mpc_infeasible_count": health.infeasible_count,
                "mpc_health_age_sec": health.age_sec if math.isfinite(health.age_sec) else None,
                "recovery_enabled": recovery_state.enabled,
                "recovery_latched": self.core.recovery_episode_latched,
                "recovery_status_state": recovery_state.status_state,
                "recovery_status_fresh": recovery_state.status_fresh,
                "recovery_command_fresh": recovery_state.command_fresh,
                "recovery_permit_fresh": recovery_state.permit_fresh,
                "recovery_external_safety_ok": recovery_state.external_safety_ok,
                "recovery_ids_match": recovery_state.ids_match,
                "recovery_trajectory_header_match": recovery_state.trajectory_header_match,
                "selected_input_stamp_sec": self._stamp_sec(selected_input_cmd),
                "selected_input_stamp_nanosec": self._stamp_nanosec(selected_input_cmd),
                "output_stamp_sec": self._stamp_sec(output_cmd),
                "output_stamp_nanosec": self._stamp_nanosec(output_cmd),
                "output_speed_mps": output_cmd.longitudinal.speed,
                "output_accel_mps2": output_cmd.longitudinal.acceleration,
                "output_steer_rad": output_cmd.lateral.steering_tire_angle,
                "raw_steer_rad": steering_result.raw_steering_rad,
                "limited_steer_rad": steering_result.limited_steering_rad,
                "steering_delta_rad": steering_result.steering_delta_rad,
                "steering_angle_limited": steering_result.angle_limited,
                "steering_rate_limited": steering_result.rate_limited,
                "steering_limiter_reset": steering_result.limiter_reset,
                "safety_constraint_enabled": self.safety_constraint_enabled,
                "safety_constraint_required": self.require_safety_constraint,
                "safety_constraint_reason": constraint_decision.reason,
                "safety_constraint_stop_required": constraint_decision.stop_required,
                "safety_constraint_generation": constraint_decision.constraint_generation,
                "safety_constraint_plan_generation": constraint_decision.plan_generation,
                "safety_constraint_speed_limit_mps": constraint_decision.speed_limit_mps,
                "safety_constraint_required_brake_decel_mps2": (
                    constraint_decision.required_brake_decel_mps2
                ),
                "overtake_plan_fresh": overtake_plan_fresh,
                "overtake_plan_valid": overtake_plan_valid,
                "active_plan_generation": active_plan_generation,
                "external_stop_latched": self.external_stop_latched,
                "control_loop_deadline_missed": control_loop_deadline_missed,
                "ros_clock_stalled": ros_clock_stalled,
                "ros_clock_stagnant_duration_sec": (
                    ros_clock_stagnant_duration_sec
                    if math.isfinite(ros_clock_stagnant_duration_sec)
                    else None
                ),
                "control_fault_latched": control_fault_latched,
                "control_fault_clear_cycles": control_fault_clear_cycles,
                "control_publish_gap_sec": (
                    control_publish_gap_sec
                    if math.isfinite(control_publish_gap_sec)
                    else None
                ),
            },
            separators=(",", ":"),
        )
        self.debug_pub.publish(msg)

    @staticmethod
    def _fresh(last_time_sec: Optional[float], now_sec: float, timeout_sec: float) -> bool:
        if last_time_sec is None:
            return False
        age_sec = now_sec - last_time_sec
        return math.isfinite(age_sec) and 0.0 <= age_sec <= timeout_sec

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _advance_receipt_time(
        self,
        previous_receipt_time_sec: Optional[float],
        maximum_stamp_ns: Optional[int],
        incoming_stamp_ns: int,
    ) -> tuple[Optional[float], Optional[int], bool]:
        if maximum_stamp_ns is None or incoming_stamp_ns > maximum_stamp_ns:
            return self.now_sec(), incoming_stamp_ns, True
        return previous_receipt_time_sec, maximum_stamp_ns, False

    @staticmethod
    def _stamp_sec(cmd: Optional[AckermannControlCommand]) -> Optional[int]:
        if cmd is None:
            return None
        return int(cmd.stamp.sec)

    @staticmethod
    def _stamp_nanosec(cmd: Optional[AckermannControlCommand]) -> Optional[int]:
        if cmd is None:
            return None
        return int(cmd.stamp.nanosec)

    @staticmethod
    def _finite_clamp(value: float, lower: float, upper: float) -> float:
        if not math.isfinite(value):
            return lower
        return min(max(value, lower), upper)

    @staticmethod
    def _recovery_state_name(state: int) -> str:
        mapping = {
            int(RecoveryStatus.INACTIVE): "inactive",
            int(RecoveryStatus.STUCK_CONFIRMING): "stuck_confirming",
            int(RecoveryStatus.STOP_HOLD): "stop_hold",
            int(RecoveryStatus.ACTIVE): "active",
            int(RecoveryStatus.HANDOFF_VERIFY): "handoff_verify",
            int(RecoveryStatus.COMPLETE): "complete",
            int(RecoveryStatus.ABORT): "abort",
            int(RecoveryStatus.LOCKOUT): "lockout",
        }
        return mapping.get(int(state), "unknown")


def main() -> None:
    rclpy.init()
    node = HybridControlMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
