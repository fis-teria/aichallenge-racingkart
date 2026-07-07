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
    fallback_trigger_infeasible_count: int = 2
    fallback_release_solved_cycles: int = 3
    fallback_min_hold_sec: float = 1.0
    use_pure_pursuit_on_mpc_cmd_timeout: bool = True
    use_pure_pursuit_on_mpc_health_timeout: bool = False


@dataclass
class MuxDecision:
    source: str
    fallback_active: bool
    reason: str
    solved_cycles: int


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

    def update(
        self,
        now_sec: float,
        *,
        mpc_cmd_fresh: bool,
        pure_pursuit_cmd_fresh: bool,
        mpc_health: MpcHealth,
    ) -> MuxDecision:
        trigger, trigger_reason = self._fallback_trigger(
            mpc_cmd_fresh=mpc_cmd_fresh,
            mpc_health=mpc_health,
        )
        mpc_healthy = (
            mpc_cmd_fresh
            and mpc_health.valid
            and mpc_health.status == "solved"
            and mpc_health.infeasible_count <= 0
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
