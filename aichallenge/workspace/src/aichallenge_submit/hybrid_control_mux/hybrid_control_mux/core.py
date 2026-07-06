from __future__ import annotations

from dataclasses import dataclass


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
