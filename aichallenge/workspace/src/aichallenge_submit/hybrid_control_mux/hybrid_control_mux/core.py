from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class MpcHealth:
    valid: bool = False
    status: str = "unknown"
    infeasible_count: int = 0
    age_sec: float = float("inf")


@dataclass
class HybridMuxConfig:
    primary_source: str = "mpc"
    fallback_trigger_infeasible_count: int = 2
    fallback_release_solved_cycles: int = 3
    fallback_min_hold_sec: float = 1.0
    use_pure_pursuit_on_mpc_cmd_timeout: bool = True
    use_pure_pursuit_on_mpc_health_timeout: bool = False
    use_mpc_on_pure_pursuit_cmd_timeout: bool = False
    recovery_max_duration_sec: float = 3.0


@dataclass
class MuxDecision:
    source: str
    fallback_active: bool
    reason: str
    solved_cycles: int


@dataclass(frozen=True)
class SafetyConstraintState:
    constraint_generation: int
    plan_generation: int
    valid: bool
    stop_requested: bool
    release_authorized: bool
    speed_limit_mps: float
    required_brake_decel_mps2: float
    header_stamp_ns: int = 0
    frame_id: str = "map"
    reason: str = ""


@dataclass(frozen=True)
class SafetyConstraintDecision:
    stop_required: bool
    speed_limit_mps: float
    required_brake_decel_mps2: float
    constraint_generation: int
    plan_generation: int
    reason: str


class SafetyConstraintAuthority:
    """Fail-closed, monotonic safety-constraint latch for the final mux."""

    def __init__(
        self,
        *,
        required: bool,
        timeout_sec: float,
        maximum_speed_limit_mps: float,
        maximum_brake_decel_mps2: float,
        fault_clear_safe_cycles: int = 3,
    ) -> None:
        self.required = required
        self.timeout_sec = max(0.0, timeout_sec)
        self.maximum_speed_limit_mps = max(0.0, maximum_speed_limit_mps)
        self.maximum_brake_decel_mps2 = max(0.0, maximum_brake_decel_mps2)
        self.fault_clear_safe_cycles = max(1, int(fault_clear_safe_cycles))
        self._latched: SafetyConstraintState | None = None
        self._last_payload: tuple[object, ...] | None = None
        self._last_header_stamp_ns: int | None = None
        self._fault_latched = False
        self._fault_clear_cycles = 0
        self._fault_last_release_stamp_ns: int | None = None
        self._fault_reason = ""

    def evaluate(
        self,
        constraint: SafetyConstraintState | None,
        *,
        received_time_sec: float | None,
        now_sec: float,
        active_plan_generation: int | None = None,
    ) -> SafetyConstraintDecision:
        if constraint is None or received_time_sec is None:
            if self.required:
                return self._stop("safety_constraint_missing")
            return self._unconstrained("safety_constraint_optional_missing")

        age_sec = now_sec - received_time_sec
        if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > self.timeout_sec:
            return self._stop("safety_constraint_stale")
        if not self._valid(constraint):
            return self._stop("safety_constraint_invalid")
        if (
            active_plan_generation is not None
            and constraint.plan_generation != active_plan_generation
        ):
            return self._stop("safety_constraint_plan_generation_mismatch")

        payload = self._payload(constraint)
        previous = self._latched
        if previous is not None:
            if (
                self._last_header_stamp_ns is not None
                and constraint.header_stamp_ns < self._last_header_stamp_ns
            ):
                return self._stop("safety_constraint_timestamp_regression")
            if constraint.constraint_generation < previous.constraint_generation:
                return self._stop("safety_constraint_generation_regression")
            if constraint.plan_generation < previous.plan_generation:
                return self._stop("safety_constraint_plan_generation_regression")

        if self._fault_latched:
            if constraint.release_authorized:
                if (
                    self._fault_last_release_stamp_ns is None
                    or constraint.header_stamp_ns
                    > self._fault_last_release_stamp_ns
                ):
                    self._fault_clear_cycles += 1
                    self._fault_last_release_stamp_ns = (
                        constraint.header_stamp_ns
                    )
            else:
                self._fault_clear_cycles = 0
                self._fault_last_release_stamp_ns = None
            if self._fault_clear_cycles < self.fault_clear_safe_cycles:
                return self._stop(
                    "safety_constraint_fault_latched", latch_fault=False
                )
            self._fault_latched = False
            self._fault_clear_cycles = 0
            self._fault_last_release_stamp_ns = None
            self._fault_reason = ""

        if previous is not None:
            if constraint.constraint_generation == previous.constraint_generation:
                if payload != self._last_payload:
                    return self._stop("safety_constraint_same_generation_conflict")
                self._last_header_stamp_ns = constraint.header_stamp_ns
                return self._decision(previous, "safety_constraint_retransmit")

            relaxes = (
                constraint.speed_limit_mps > previous.speed_limit_mps
                or constraint.required_brake_decel_mps2
                < previous.required_brake_decel_mps2
                or (previous.stop_requested and not constraint.stop_requested)
            )
            if relaxes and not constraint.release_authorized:
                constraint = SafetyConstraintState(
                    constraint_generation=constraint.constraint_generation,
                    plan_generation=constraint.plan_generation,
                    valid=True,
                    stop_requested=previous.stop_requested or constraint.stop_requested,
                    release_authorized=False,
                    speed_limit_mps=min(
                        previous.speed_limit_mps, constraint.speed_limit_mps
                    ),
                    required_brake_decel_mps2=max(
                        previous.required_brake_decel_mps2,
                        constraint.required_brake_decel_mps2,
                    ),
                    header_stamp_ns=constraint.header_stamp_ns,
                    frame_id=constraint.frame_id,
                    reason=constraint.reason,
                )
                self._latched = constraint
                self._last_payload = payload
                self._last_header_stamp_ns = constraint.header_stamp_ns
                return self._decision(
                    constraint, "safety_constraint_relaxation_not_authorized"
                )

        self._latched = constraint
        self._last_payload = payload
        self._last_header_stamp_ns = constraint.header_stamp_ns
        return self._decision(constraint, "safety_constraint_applied")

    def _valid(self, constraint: SafetyConstraintState) -> bool:
        return (
            constraint.valid
            and constraint.constraint_generation >= 0
            and constraint.plan_generation >= 0
            and constraint.header_stamp_ns >= 0
            and constraint.frame_id == "map"
            and math.isfinite(constraint.speed_limit_mps)
            and 0.0 < constraint.speed_limit_mps <= self.maximum_speed_limit_mps
            and math.isfinite(constraint.required_brake_decel_mps2)
            and 0.0
            <= constraint.required_brake_decel_mps2
            <= self.maximum_brake_decel_mps2
        )

    @staticmethod
    def _payload(constraint: SafetyConstraintState) -> tuple[object, ...]:
        return (
            constraint.plan_generation,
            constraint.valid,
            constraint.stop_requested,
            constraint.release_authorized,
            constraint.speed_limit_mps,
            constraint.required_brake_decel_mps2,
        )

    def _decision(
        self, constraint: SafetyConstraintState, reason: str
    ) -> SafetyConstraintDecision:
        return SafetyConstraintDecision(
            stop_required=constraint.stop_requested,
            speed_limit_mps=constraint.speed_limit_mps,
            required_brake_decel_mps2=constraint.required_brake_decel_mps2,
            constraint_generation=constraint.constraint_generation,
            plan_generation=constraint.plan_generation,
            reason=reason,
        )

    def _stop(
        self, reason: str, *, latch_fault: bool = True
    ) -> SafetyConstraintDecision:
        # Before the first accepted contract, required-input absence is a
        # fail-closed startup stop, not a historical low-cap release.  Once a
        # valid contract has been accepted, faults latch until explicit safe
        # release so a stale/invalid interval cannot silently raise speed.
        if latch_fault and self._latched is not None:
            self._fault_latched = True
            self._fault_clear_cycles = 0
            self._fault_last_release_stamp_ns = None
            self._fault_reason = reason
        generation = self._latched.constraint_generation if self._latched else 0
        plan_generation = self._latched.plan_generation if self._latched else 0
        return SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=0.0,
            constraint_generation=generation,
            plan_generation=plan_generation,
            reason=reason,
        )

    def _unconstrained(self, reason: str) -> SafetyConstraintDecision:
        return SafetyConstraintDecision(
            stop_required=False,
            speed_limit_mps=self.maximum_speed_limit_mps,
            required_brake_decel_mps2=0.0,
            constraint_generation=0,
            plan_generation=0,
            reason=reason,
        )


@dataclass(frozen=True)
class ControlLoopWatchdogResult:
    deadline_missed: bool
    publish_gap_sec: float


class ControlLoopWatchdog:
    def __init__(self, maximum_gap_sec: float) -> None:
        self.maximum_gap_sec = max(0.0, maximum_gap_sec)
        self._last_tick_sec: float | None = None

    def update(self, now_sec: float) -> ControlLoopWatchdogResult:
        if self._last_tick_sec is None:
            self._last_tick_sec = now_sec
            return ControlLoopWatchdogResult(False, 0.0)
        gap_sec = now_sec - self._last_tick_sec
        self._last_tick_sec = now_sec
        missed = (
            not math.isfinite(gap_sec)
            or gap_sec < 0.0
            or gap_sec > self.maximum_gap_sec
        )
        return ControlLoopWatchdogResult(missed, gap_sec)


@dataclass(frozen=True)
class RosClockProgressWatchdogResult:
    stalled: bool
    stagnant_duration_sec: float
    reason: str


class RosClockProgressWatchdog:
    """Detect a ROS clock that stops or moves backwards using steady time."""

    def __init__(self, maximum_stall_sec: float) -> None:
        self.maximum_stall_sec = max(0.0, maximum_stall_sec)
        self._maximum_ros_time_ns: int | None = None
        self._last_progress_steady_sec: float | None = None

    def update(
        self, ros_time_ns: int, steady_time_sec: float
    ) -> RosClockProgressWatchdogResult:
        if (
            not isinstance(ros_time_ns, int)
            or ros_time_ns < 0
            or not math.isfinite(steady_time_sec)
        ):
            return RosClockProgressWatchdogResult(
                True, float("inf"), "ros_clock_invalid"
            )

        if self._maximum_ros_time_ns is None:
            self._maximum_ros_time_ns = ros_time_ns
            self._last_progress_steady_sec = steady_time_sec
            return RosClockProgressWatchdogResult(False, 0.0, "ros_clock_initial")

        if ros_time_ns > self._maximum_ros_time_ns:
            self._maximum_ros_time_ns = ros_time_ns
            self._last_progress_steady_sec = steady_time_sec
            return RosClockProgressWatchdogResult(False, 0.0, "ros_clock_progressing")

        if ros_time_ns < self._maximum_ros_time_ns:
            return RosClockProgressWatchdogResult(
                True, float("inf"), "ros_clock_regression"
            )

        if self._last_progress_steady_sec is None:
            return RosClockProgressWatchdogResult(
                True, float("inf"), "ros_clock_progress_missing"
            )
        stagnant_duration_sec = steady_time_sec - self._last_progress_steady_sec
        stalled = (
            not math.isfinite(stagnant_duration_sec)
            or stagnant_duration_sec < 0.0
            or stagnant_duration_sec > self.maximum_stall_sec
        )
        return RosClockProgressWatchdogResult(
            stalled,
            stagnant_duration_sec,
            "ros_clock_stalled" if stalled else "ros_clock_within_stall_budget",
        )


def apply_safety_constraint(
    speed_mps: float,
    acceleration_mps2: float,
    decision: SafetyConstraintDecision,
) -> tuple[float, float]:
    """Apply an already-validated constraint to one longitudinal command."""
    if decision.stop_required:
        stop_acceleration = acceleration_mps2
        if decision.required_brake_decel_mps2 > 0.0:
            stop_acceleration = min(
                acceleration_mps2, -decision.required_brake_decel_mps2
            )
        return 0.0, stop_acceleration
    limited_speed = min(max(0.0, speed_mps), decision.speed_limit_mps)
    limited_acceleration = acceleration_mps2
    if decision.required_brake_decel_mps2 > 0.0:
        limited_acceleration = min(
            acceleration_mps2, -decision.required_brake_decel_mps2
        )
    return limited_speed, limited_acceleration


@dataclass
class RecoveryMuxState:
    enabled: bool = False
    status_state: str = "inactive"
    status_fresh: bool = False
    command_fresh: bool = False
    permit_fresh: bool = False
    external_safety_ok: bool = False
    input_complete: bool = False
    trajectory_valid: bool = False
    trajectory_safe: bool = False
    stop_required: bool = False
    handoff_ready: bool = False
    recovery_allowed: bool = False
    handoff_allowed: bool = False
    ids_match: bool = False
    trajectory_header_match: bool = False
    command_valid: bool = False
    command_forward_only: bool = False


@dataclass
class SteeringLimiterConfig:
    enabled: bool = True
    max_steering_angle_rad: float = 0.5585053606381855
    max_steering_rate_radps: float = 8.0
    max_steering_delta_per_cycle: float = 0.0
    reset_dt_threshold_sec: float = 0.50
    reset_on_mode_change: bool = False


@dataclass
class SteeringLimitResult:
    raw_steering_rad: float
    limited_steering_rad: float
    steering_delta_rad: float
    angle_limited: bool
    rate_limited: bool
    limiter_reset: bool


class SteeringLimiter:
    def __init__(self, config: SteeringLimiterConfig) -> None:
        self.config = config
        self.has_last_steering = False
        self.last_steering_rad = 0.0
        self.last_time_sec = 0.0
        self.last_source = ""

    def update(self, raw_steering_rad: float, now_sec: float, source: str) -> SteeringLimitResult:
        finite_raw = raw_steering_rad if math.isfinite(raw_steering_rad) else 0.0
        abs_limit = max(0.0, self.config.max_steering_angle_rad)
        clamped = self._clamp(finite_raw, -abs_limit, abs_limit)
        angle_limited = not math.isclose(clamped, finite_raw, abs_tol=1.0e-12)

        mode_changed = self.has_last_steering and source != self.last_source
        dt = now_sec - self.last_time_sec
        reset = (
            not self.config.enabled
            or not self.has_last_steering
            or not math.isfinite(dt)
            or dt <= 0.0
            or dt > self.config.reset_dt_threshold_sec
            or (self.config.reset_on_mode_change and mode_changed)
        )
        limited = clamped
        rate_limited = False

        if self.config.enabled and not reset:
            max_delta = max(0.0, self.config.max_steering_rate_radps) * dt
            if self.config.max_steering_delta_per_cycle > 0.0:
                max_delta = min(max_delta, self.config.max_steering_delta_per_cycle)
            lower = self.last_steering_rad - max_delta
            upper = self.last_steering_rad + max_delta
            limited = self._clamp(clamped, lower, upper)
            rate_limited = not math.isclose(limited, clamped, abs_tol=1.0e-12)
            limited = self._clamp(limited, -abs_limit, abs_limit)

        steering_delta = limited - self.last_steering_rad if self.has_last_steering else 0.0
        self.last_steering_rad = limited
        self.last_time_sec = now_sec
        self.last_source = source
        self.has_last_steering = True

        return SteeringLimitResult(
            raw_steering_rad=finite_raw,
            limited_steering_rad=limited,
            steering_delta_rad=steering_delta,
            angle_limited=angle_limited,
            rate_limited=rate_limited,
            limiter_reset=reset,
        )

    def reset(self, raw_steering_rad: float, now_sec: float, source: str) -> SteeringLimitResult:
        finite_raw = raw_steering_rad if math.isfinite(raw_steering_rad) else 0.0
        abs_limit = max(0.0, self.config.max_steering_angle_rad)
        clamped = self._clamp(finite_raw, -abs_limit, abs_limit)
        angle_limited = not math.isclose(clamped, finite_raw, abs_tol=1.0e-12)
        steering_delta = clamped - self.last_steering_rad if self.has_last_steering else 0.0

        self.last_steering_rad = clamped
        self.last_time_sec = now_sec
        self.last_source = source
        self.has_last_steering = True

        return SteeringLimitResult(
            raw_steering_rad=finite_raw,
            limited_steering_rad=clamped,
            steering_delta_rad=steering_delta,
            angle_limited=angle_limited,
            rate_limited=False,
            limiter_reset=True,
        )

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)


class HybridMuxCore:
    def __init__(self, config: HybridMuxConfig) -> None:
        self.config = config
        self.fallback_active = False
        self.fallback_enter_time_sec = 0.0
        self.solved_cycles = 0
        self.last_reason = "startup"
        self.recovery_episode_latched = False
        self.recovery_enter_time_sec = 0.0

    def update(
        self,
        now_sec: float,
        *,
        mpc_cmd_fresh: bool,
        pure_pursuit_cmd_fresh: bool,
        mpc_health: MpcHealth,
        recovery: RecoveryMuxState | None = None,
    ) -> MuxDecision:
        recovery_decision = self._update_recovery(
            now_sec,
            recovery=recovery,
            normal_cmd_fresh=mpc_cmd_fresh or pure_pursuit_cmd_fresh,
        )
        if recovery_decision is not None:
            return recovery_decision

        if self._primary_source() == "pure_pursuit":
            return self._update_pure_pursuit_primary(
                now_sec,
                mpc_cmd_fresh=mpc_cmd_fresh,
                pure_pursuit_cmd_fresh=pure_pursuit_cmd_fresh,
                mpc_health=mpc_health,
            )

        trigger, trigger_reason = self._fallback_trigger(
            mpc_cmd_fresh=mpc_cmd_fresh,
            mpc_health=mpc_health,
        )
        mpc_healthy = self._mpc_healthy(
            mpc_cmd_fresh=mpc_cmd_fresh,
            mpc_health=mpc_health,
        )

        if self.fallback_active:
            if mpc_healthy:
                self.solved_cycles += 1
            else:
                self.solved_cycles = 0
            hold_done = now_sec - self.fallback_enter_time_sec >= self.config.fallback_min_hold_sec
            release_ready = (
                hold_done
                and self.solved_cycles >= max(1, self.config.fallback_release_solved_cycles)
            )
            if release_ready:
                self.fallback_active = False
                self.last_reason = "mpc_recovered"
            elif trigger:
                self.last_reason = trigger_reason
        else:
            self.solved_cycles = 1 if mpc_healthy else 0
            if trigger:
                self.fallback_active = True
                self.fallback_enter_time_sec = now_sec
                self.solved_cycles = 0
                self.last_reason = trigger_reason

        if self.fallback_active:
            if pure_pursuit_cmd_fresh:
                return MuxDecision("pure_pursuit", True, self.last_reason, self.solved_cycles)
            return MuxDecision("stop", True, "fallback_without_pure_pursuit_cmd", self.solved_cycles)

        if mpc_cmd_fresh:
            return MuxDecision("mpc", False, self.last_reason, self.solved_cycles)

        if pure_pursuit_cmd_fresh and self.config.use_pure_pursuit_on_mpc_cmd_timeout:
            self.fallback_active = True
            self.fallback_enter_time_sec = now_sec
            self.solved_cycles = 0
            self.last_reason = "mpc_cmd_timeout"
            return MuxDecision("pure_pursuit", True, self.last_reason, self.solved_cycles)

        return MuxDecision("stop", False, "no_fresh_control_cmd", self.solved_cycles)

    def _update_recovery(
        self,
        now_sec: float,
        *,
        recovery: RecoveryMuxState | None,
        normal_cmd_fresh: bool,
    ) -> MuxDecision | None:
        if recovery is None or not recovery.enabled:
            self.recovery_episode_latched = False
            return None

        state = str(recovery.status_state or "inactive").strip().lower()
        starts_episode = state in {"stop_hold", "active", "handoff_verify"}
        if recovery.status_fresh and starts_episode and not self.recovery_episode_latched:
            self.recovery_episode_latched = True
            self.recovery_enter_time_sec = now_sec

        if not self.recovery_episode_latched:
            return None

        if not recovery.status_fresh:
            return MuxDecision("stop", True, "recovery_status_timeout", self.solved_cycles)
        if not recovery.external_safety_ok:
            return MuxDecision("stop", True, "recovery_external_safety_stop", self.solved_cycles)

        release_ready = (
            state == "complete"
            and recovery.handoff_ready
            and recovery.permit_fresh
            and recovery.handoff_allowed
            and normal_cmd_fresh
        )
        if release_ready:
            self.recovery_episode_latched = False
            self.last_reason = "recovery_handoff_complete"
            return None

        if state in {"stop_hold", "abort", "lockout"}:
            return MuxDecision("stop", True, f"recovery_{state}", self.solved_cycles)

        if state in {"active", "handoff_verify"}:
            if now_sec - self.recovery_enter_time_sec > self.config.recovery_max_duration_sec:
                return MuxDecision("stop", True, "recovery_duration_timeout", self.solved_cycles)
            gates_ok = (
                recovery.command_fresh
                and recovery.permit_fresh
                and recovery.input_complete
                and recovery.trajectory_valid
                and recovery.trajectory_safe
                and not recovery.stop_required
                and recovery.recovery_allowed
                and recovery.ids_match
                and recovery.trajectory_header_match
                and recovery.command_valid
                and recovery.command_forward_only
            )
            if gates_ok:
                return MuxDecision("recovery", True, f"recovery_{state}", self.solved_cycles)
            return MuxDecision("stop", True, "recovery_gate_not_satisfied", self.solved_cycles)

        return MuxDecision("stop", True, f"recovery_latched_{state}", self.solved_cycles)

    def _update_pure_pursuit_primary(
        self,
        now_sec: float,
        *,
        mpc_cmd_fresh: bool,
        pure_pursuit_cmd_fresh: bool,
        mpc_health: MpcHealth,
    ) -> MuxDecision:
        mpc_healthy = self._mpc_healthy(
            mpc_cmd_fresh=mpc_cmd_fresh,
            mpc_health=mpc_health,
        )
        self.solved_cycles = 1 if mpc_healthy else 0

        if pure_pursuit_cmd_fresh:
            self.fallback_active = False
            self.last_reason = "primary_pure_pursuit"
            return MuxDecision("pure_pursuit", False, self.last_reason, self.solved_cycles)

        self.fallback_active = True
        self.fallback_enter_time_sec = now_sec
        self.last_reason = "pure_pursuit_cmd_timeout"
        if self.config.use_mpc_on_pure_pursuit_cmd_timeout and mpc_healthy:
            return MuxDecision("mpc", True, self.last_reason, self.solved_cycles)
        return MuxDecision("stop", True, self.last_reason, self.solved_cycles)

    def _fallback_trigger(self, *, mpc_cmd_fresh: bool, mpc_health: MpcHealth) -> tuple[bool, str]:
        if not mpc_cmd_fresh:
            return self.config.use_pure_pursuit_on_mpc_cmd_timeout, "mpc_cmd_timeout"

        if not mpc_health.valid:
            if self.config.use_pure_pursuit_on_mpc_health_timeout:
                return True, "mpc_health_timeout"
            return False, ""

        threshold = max(1, self.config.fallback_trigger_infeasible_count)
        if mpc_health.infeasible_count >= threshold:
            return True, "mpc_infeasible"
        if threshold <= 1 and mpc_health.status == "infeasible":
            return True, "mpc_infeasible"
        return False, ""

    def _primary_source(self) -> str:
        source = str(self.config.primary_source or "mpc").strip().lower()
        return "pure_pursuit" if source in {"pure_pursuit", "pp"} else "mpc"

    @staticmethod
    def _mpc_healthy(*, mpc_cmd_fresh: bool, mpc_health: MpcHealth) -> bool:
        return (
            mpc_cmd_fresh
            and mpc_health.valid
            and mpc_health.status == "solved"
            and mpc_health.infeasible_count <= 0
        )
