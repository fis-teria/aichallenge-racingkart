#!/usr/bin/env python3
"""Bounded steering actuator model for the private offline replay only."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def bounded_steering_step(
    current: float, target: float, max_rate: float, period: float, max_angle: float
) -> float:
    if not all(
        math.isfinite(value)
        for value in (current, target, max_rate, period, max_angle)
    ) or max_rate <= 0.0 or period <= 0.0 or max_angle <= 0.0:
        raise ValueError("invalid steering step input")
    bounded_target = max(-max_angle, min(max_angle, target))
    max_step = max_rate * period
    delta = max(-max_step, min(max_step, bounded_target - current))
    return max(-max_angle, min(max_angle, current + delta))


class TestOnlySteeringActuator(Node):
    def __init__(self) -> None:
        super().__init__("test_only_offline_steering_actuator")
        input_topic = self.declare_parameter("input_control_topic", "").value
        output_topic = self.declare_parameter("output_steering_topic", "").value
        self._period = float(self.declare_parameter("period_sec", 0.01).value)
        self._max_rate = float(
            self.declare_parameter("max_steering_rate_radps", 0.75).value
        )
        self._max_angle = float(
            self.declare_parameter("max_steering_angle_rad", 0.48).value
        )
        self._command_timeout = float(
            self.declare_parameter("command_timeout_sec", 0.20).value
        )
        if (
            not input_topic
            or not output_topic
            or not math.isfinite(self._period)
            or self._period <= 0.0
            or not math.isfinite(self._max_rate)
            or self._max_rate <= 0.0
            or not math.isfinite(self._max_angle)
            or self._max_angle <= 0.0
            or not math.isfinite(self._command_timeout)
            or self._command_timeout <= 0.0
        ):
            raise ValueError("invalid test-only steering actuator configuration")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._angle = 0.0
        self._target = 0.0
        self._last_command_ns: int | None = None
        self._publisher = self.create_publisher(SteeringReport, output_topic, qos)
        self._subscription = self.create_subscription(
            AckermannControlCommand, input_topic, self._on_command, qos
        )
        self._timer = self.create_timer(self._period, self._on_timer)

    def _on_command(self, message: AckermannControlCommand) -> None:
        requested = float(message.lateral.steering_tire_angle)
        if not math.isfinite(requested):
            self._target = 0.0
        else:
            self._target = max(-self._max_angle, min(self._max_angle, requested))
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        target = self._target
        if (
            self._last_command_ns is None
            or (now.nanoseconds - self._last_command_ns) * 1.0e-9
            > self._command_timeout
        ):
            target = 0.0
        self._angle = bounded_steering_step(
            self._angle, target, self._max_rate, self._period, self._max_angle
        )
        report = SteeringReport()
        report.stamp = now.to_msg()
        report.steering_tire_angle = float(self._angle)
        self._publisher.publish(report)


def main() -> None:
    rclpy.init()
    node = TestOnlySteeringActuator()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception:
        # SIGINT may invalidate the context between executor checks and wait
        # set construction. Preserve real runtime failures while treating only
        # that already-shutdown race as a clean launch termination.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
