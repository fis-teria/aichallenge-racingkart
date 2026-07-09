#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import math
from typing import Optional

import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.node import Node
from std_msgs.msg import String

from hybrid_control_mux.core import (
    HybridMuxConfig,
    HybridMuxCore,
    MpcHealth,
    SteeringLimitResult,
    SteeringLimiter,
    SteeringLimiterConfig,
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
        self.fallback_speed_mps = float(
            self.declare_parameter("fallback_speed_mps", 2.0).value
        )
        self.fallback_accel_max_mps2 = float(
            self.declare_parameter("fallback_accel_max_mps2", 0.8).value
        )
        self.fallback_decel_min_mps2 = float(
            self.declare_parameter("fallback_decel_min_mps2", -1.5).value
        )
        self.stop_decel_mps2 = float(self.declare_parameter("stop_decel_mps2", -1.5).value)
        self.debug_publish_period_sec = float(
            self.declare_parameter("debug_publish_period_sec", 0.25).value
        )
        self.steering_log_throttle_sec = float(
            self.declare_parameter("steering_log_throttle_sec", 1.0).value
        )
        self.last_steering_limit_log_sec = -1.0e9

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
        )
        self.core = HybridMuxCore(config)
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
        self.pure_pursuit_cmd: Optional[AckermannControlCommand] = None
        self.pure_pursuit_cmd_time_sec: Optional[float] = None
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
        self.create_subscription(String, "input/mpc_health", self.on_mpc_health, 1)
        self.control_pub = self.create_publisher(AckermannControlCommand, "output/control_cmd", 1)
        self.debug_pub = self.create_publisher(String, "output/debug", 1)

        period = 1.0 / max(1.0, self.control_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def on_mpc_cmd(self, msg: AckermannControlCommand) -> None:
        self.mpc_cmd = msg
        self.mpc_cmd_time_sec = self.now_sec()

    def on_pure_pursuit_cmd(self, msg: AckermannControlCommand) -> None:
        self.pure_pursuit_cmd = msg
        self.pure_pursuit_cmd_time_sec = self.now_sec()

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
        mpc_cmd_fresh = self._fresh(self.mpc_cmd_time_sec, now_sec, self.mpc_cmd_timeout_sec)
        pp_cmd_fresh = self._fresh(
            self.pure_pursuit_cmd_time_sec, now_sec, self.pure_pursuit_cmd_timeout_sec
        )
        health = self._current_health(now_sec)

        if not self.enabled:
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
            )
            decision_source = decision.source
            reason = decision.reason
            fallback_active = decision.fallback_active
            solved_cycles = decision.solved_cycles

        cmd = self._select_command(decision_source, now_sec)
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
            output_cmd=cmd,
            steering_result=steering_result,
        )

        if decision_source != self.last_source:
            self.get_logger().warn(
                f"hybrid control source={decision_source} reason={reason} "
                f"mpc_status={health.status} mpc_infeasible={health.infeasible_count}"
            )
            self.last_source = decision_source

    def _select_command(self, source: str, now_sec: float) -> AckermannControlCommand:
        if source == "mpc" and self.mpc_cmd is not None:
            return self._stamp(copy.deepcopy(self.mpc_cmd), now_sec)
        if source == "pure_pursuit" and self.pure_pursuit_cmd is not None:
            return self._fallback_command(copy.deepcopy(self.pure_pursuit_cmd), now_sec)
        return self._stop_command(now_sec)

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
        output_cmd: AckermannControlCommand,
        steering_result: SteeringLimitResult,
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
                "fallback_active": fallback_active,
                "reason": reason,
                "solved_cycles": solved_cycles,
                "mpc_cmd_fresh": mpc_cmd_fresh,
                "pure_pursuit_cmd_fresh": pure_pursuit_cmd_fresh,
                "mpc_health_valid": health.valid,
                "mpc_status": health.status,
                "mpc_infeasible_count": health.infeasible_count,
                "mpc_health_age_sec": health.age_sec if math.isfinite(health.age_sec) else None,
                "output_speed_mps": output_cmd.longitudinal.speed,
                "output_accel_mps2": output_cmd.longitudinal.acceleration,
                "output_steer_rad": output_cmd.lateral.steering_tire_angle,
                "raw_steer_rad": steering_result.raw_steering_rad,
                "limited_steer_rad": steering_result.limited_steering_rad,
                "steering_delta_rad": steering_result.steering_delta_rad,
                "steering_angle_limited": steering_result.angle_limited,
                "steering_rate_limited": steering_result.rate_limited,
                "steering_limiter_reset": steering_result.limiter_reset,
            },
            separators=(",", ":"),
        )
        self.debug_pub.publish(msg)

    @staticmethod
    def _fresh(last_time_sec: Optional[float], now_sec: float, timeout_sec: float) -> bool:
        return last_time_sec is not None and now_sec - last_time_sec <= timeout_sec

    @staticmethod
    def _finite_clamp(value: float, lower: float, upper: float) -> float:
        if not math.isfinite(value):
            return lower
        return min(max(value, lower), upper)


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
