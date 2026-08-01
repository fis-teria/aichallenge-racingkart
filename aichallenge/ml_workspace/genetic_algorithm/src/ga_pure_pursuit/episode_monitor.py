from __future__ import annotations

import json
import math
import os
import statistics
import time
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)]


def finite_mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def run() -> int:
    try:
        import rclpy
        from autoware_auto_control_msgs.msg import AckermannControlCommand
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray, Int32, String
        from tier4_vehicle_msgs.msg import ActuationCommandStamped
    except ImportError as error:
        raise SystemExit(f"ROS 2 Python environment is required: {error}")

    request = json.loads(Path(os.environ["GA_EPISODE_REQUEST"]).read_text(encoding="utf-8"))
    episode_options = request.get("episode_options", {})
    target_laps = max(1, int(episode_options.get("target_laps", 1)))
    standing_lap_weight = float(episode_options.get("standing_lap_weight", 1.0))
    flying_lap_weight = float(episode_options.get("flying_lap_weight", 0.0))
    result_path = Path(os.environ["GA_EPISODE_RESULT"])
    timeout_sec = float(os.environ.get("GA_EPISODE_TIMEOUT_SEC", "180"))
    stuck_timeout_sec = float(os.environ.get("GA_STUCK_TIMEOUT_SEC", "20"))
    minimum_valid_lap_time = float(os.environ.get("GA_MINIMUM_VALID_LAP_TIME_SEC", "25.0"))
    minimum_valid_lap_distance = float(
        os.environ.get("GA_MINIMUM_VALID_LAP_DISTANCE_M", "300.0")
    )
    diagnostic_sample_interval = float(
        os.environ.get("GA_DIAGNOSTIC_SAMPLE_INTERVAL_SEC", "0.2")
    )

    class Monitor(Node):
        def __init__(self):
            super().__init__("ga_episode_monitor")
            self.started = time.monotonic()
            self.last_progress = self.started
            self.initial_lap = None
            self.lap = None
            self.section = 0
            self.lap_time = 0.0
            self.completed_lap_time = 0.0
            self.completed_lap_times: list[float] = []
            self.last_session_time = 0.0
            self.lateral_errors: list[float] = []
            self.steering_deltas: list[float] = []
            self.rate_limit_samples: list[float] = []
            self.previous_steering = None
            self.steering_angles: list[float] = []
            self.previous_position = None
            self.distance_traveled_m = 0.0
            self.initial_speed_mps = None
            self.latest_actual_speed_mps = None
            self.latest_target_speed_mps = None
            self.latest_curvature_speed_limit_mps = None
            self.latest_commanded_acceleration_mps2 = None
            self.latest_actual_acceleration_mps2 = None
            self.latest_speed_limited = None
            self.latest_accel_cmd = None
            self.latest_brake_cmd = None
            self.latest_nearest_trajectory_index = None
            self.actual_speeds_mps: list[float] = []
            self.target_speeds_mps: list[float] = []
            self.absolute_speed_errors_mps: list[float] = []
            self.commanded_accelerations_mps2: list[float] = []
            self.actual_accelerations_mps2: list[float] = []
            self.speed_limited_samples: list[float] = []
            self.speed_threshold_times = {
                "10_kmh": None,
                "20_kmh": None,
                "30_kmh": None,
                "34_kmh": None,
            }
            self.speed_thresholds_mps = {
                "10_kmh": 10.0 / 3.6,
                "20_kmh": 20.0 / 3.6,
                "30_kmh": 30.0 / 3.6,
                "34_kmh": 34.0 / 3.6,
            }
            self.section_splits: list[dict[str, float | int | None]] = []
            self.diagnostic_trace: list[dict[str, float | int | bool | None]] = []
            self.last_diagnostic_sample = self.started - diagnostic_sample_interval
            self.lap_completion_speed_mps = None
            self.previous_speed_sample = None
            self.collision_count = 0
            self.last_condition = None
            self.done = False
            self.exit_reason = "running"
            self.create_subscription(Float32MultiArray, "/awsim/status", self.on_status, 10)
            self.create_subscription(String, "/pure_pursuit/debug", self.on_debug, 20)
            best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                Odometry, "/localization/kinematic_state", self.on_odometry, best_effort
            )
            self.create_subscription(
                AckermannControlCommand, "/control/command/control_cmd", self.on_command, 20
            )
            self.create_subscription(
                ActuationCommandStamped,
                "/control/command/actuation_cmd",
                self.on_actuation,
                20,
            )
            self.create_subscription(Int32, "/aichallenge/pitstop/condition", self.on_condition, 10)
            self.create_timer(0.2, self.check_timeout)

        def on_status(self, message):
            if len(message.data) < 4:
                return
            session_time = float(message.data[0])
            lap = int(message.data[1])
            section = int(message.data[3])
            lap_time = float(message.data[2])
            if self.initial_lap is None:
                self.initial_lap = lap
            if self.lap != lap or self.section != section:
                self.last_progress = time.monotonic()
                previous = self.section_splits[-1] if self.section_splits else None
                section_duration = None
                if (
                    previous is not None
                    and previous["lap"] == lap
                    and lap_time >= float(previous["lap_time_seconds"])
                ):
                    section_duration = lap_time - float(previous["lap_time_seconds"])
                self.section_splits.append(
                    {
                        "lap": lap,
                        "section": section,
                        "lap_time_seconds": lap_time,
                        "session_time_seconds": session_time,
                        "section_duration_seconds": section_duration,
                    }
                )
            valid_lap_increment = (
                self.lap is not None
                and lap > self.lap
                and self.lap_time >= minimum_valid_lap_time
                and self.distance_traveled_m >= minimum_valid_lap_distance
            )
            if valid_lap_increment:
                self.completed_lap_time = self.lap_time
                self.completed_lap_times.append(self.lap_time)
                self.lap_completion_speed_mps = self.latest_actual_speed_mps
            elif lap > self.initial_lap and self.lap_time < minimum_valid_lap_time:
                # A reset can briefly expose the previous lap counter with a
                # zero timer. Rebase instead of awarding a zero-second lap.
                self.initial_lap = lap
            self.lap = lap
            self.section = section
            self.lap_time = lap_time
            self.last_session_time = session_time
            if valid_lap_increment and len(self.completed_lap_times) >= target_laps:
                self.done = True
                self.exit_reason = "completed"

        def on_debug(self, message):
            try:
                data = json.loads(message.data)
                if (
                    data.get("candidate_id")
                    and data["candidate_id"] != request["candidate_id"]
                ):
                    return
                if (
                    data.get("parameter_hash")
                    and data["parameter_hash"] != request["parameter_hash"]
                ):
                    return
                lateral = abs(float(data.get("lateral_error_m", 0.0)))
                if math.isfinite(lateral):
                    self.lateral_errors.append(lateral)
                nearest_index = data.get("nearest_trajectory_index")
                if nearest_index is not None:
                    self.latest_nearest_trajectory_index = int(nearest_index)
                target_speed = float(data.get("target_velocity_mps", 0.0))
                if math.isfinite(target_speed):
                    self.latest_target_speed_mps = target_speed
                    self.target_speeds_mps.append(target_speed)
                    if self.latest_actual_speed_mps is not None:
                        self.absolute_speed_errors_mps.append(
                            abs(target_speed - self.latest_actual_speed_mps)
                        )
                curvature_limit = float(data.get("curvature_speed_limit_mps", 0.0))
                if math.isfinite(curvature_limit):
                    self.latest_curvature_speed_limit_mps = curvature_limit
                commanded_acceleration = float(
                    data.get("commanded_acceleration_mps2", 0.0)
                )
                if math.isfinite(commanded_acceleration):
                    self.latest_commanded_acceleration_mps2 = commanded_acceleration
                    self.commanded_accelerations_mps2.append(commanded_acceleration)
                self.latest_speed_limited = bool(data.get("speed_limited", False))
                self.speed_limited_samples.append(float(self.latest_speed_limited))
                self.rate_limit_samples.append(float(bool(data.get("steering_rate_limited", False))))
            except (ValueError, TypeError, json.JSONDecodeError):
                self.done = True
                self.exit_reason = "invalid_command"

        def on_command(self, message):
            steering = float(message.lateral.steering_tire_angle)
            if not math.isfinite(steering):
                self.done = True
                self.exit_reason = "invalid_command"
                return
            if self.previous_steering is not None:
                self.steering_deltas.append(steering - self.previous_steering)
            self.previous_steering = steering
            self.steering_angles.append(steering)

        def on_actuation(self, message):
            accel = float(message.actuation.accel_cmd)
            brake = float(message.actuation.brake_cmd)
            if math.isfinite(accel):
                self.latest_accel_cmd = accel
            if math.isfinite(brake):
                self.latest_brake_cmd = brake

        def on_odometry(self, message):
            speed = max(0.0, float(message.twist.twist.linear.x))
            if math.isfinite(speed):
                wall_time = time.monotonic()
                stamp = message.header.stamp
                odometry_time = (
                    float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
                )
                use_odometry_time = math.isfinite(odometry_time) and odometry_time > 0.0
                sample_time = odometry_time if use_odometry_time else wall_time
                if self.previous_speed_sample is not None:
                    previous_time, previous_speed, previous_used_odometry_time = (
                        self.previous_speed_sample
                    )
                    delta_time = sample_time - previous_time
                    same_time_source = (
                        use_odometry_time == previous_used_odometry_time
                    )
                    if same_time_source and delta_time >= 0.2:
                        acceleration = (speed - previous_speed) / delta_time
                        if math.isfinite(acceleration) and abs(acceleration) <= 30.0:
                            self.latest_actual_acceleration_mps2 = acceleration
                            self.actual_accelerations_mps2.append(acceleration)
                        self.previous_speed_sample = (
                            sample_time,
                            speed,
                            use_odometry_time,
                        )
                    elif not same_time_source or delta_time <= 0.0:
                        self.previous_speed_sample = (
                            sample_time,
                            speed,
                            use_odometry_time,
                        )
                else:
                    self.previous_speed_sample = (
                        sample_time,
                        speed,
                        use_odometry_time,
                    )
                if self.initial_speed_mps is None:
                    self.initial_speed_mps = speed
                self.latest_actual_speed_mps = speed
                self.actual_speeds_mps.append(speed)
                elapsed = time.monotonic() - self.started
                for name, threshold in self.speed_thresholds_mps.items():
                    if self.speed_threshold_times[name] is None and speed >= threshold:
                        self.speed_threshold_times[name] = elapsed
            position = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
            if self.previous_position is not None:
                step = math.hypot(
                    position[0] - self.previous_position[0],
                    position[1] - self.previous_position[1],
                )
                if math.isfinite(step) and step <= 5.0:
                    self.distance_traveled_m += step
                    if step >= 0.05:
                        self.last_progress = time.monotonic()
            self.previous_position = position

        def on_condition(self, message):
            value = int(message.data)
            if self.last_condition is not None and value - self.last_condition > 30:
                self.collision_count += 1
            self.last_condition = value

        def check_timeout(self):
            now = time.monotonic()
            if now - self.last_diagnostic_sample >= diagnostic_sample_interval:
                self.last_diagnostic_sample = now
                self.diagnostic_trace.append(
                    {
                        "elapsed_seconds": now - self.started,
                        "session_time_seconds": self.last_session_time,
                        "lap": self.lap,
                        "section": self.section,
                        "lap_time_seconds": self.lap_time,
                        "actual_speed_mps": self.latest_actual_speed_mps,
                        "target_speed_mps": self.latest_target_speed_mps,
                        "curvature_speed_limit_mps": self.latest_curvature_speed_limit_mps,
                        "commanded_acceleration_mps2":
                            self.latest_commanded_acceleration_mps2,
                        "actual_acceleration_mps2": self.latest_actual_acceleration_mps2,
                        "steering_angle_rad": self.previous_steering,
                        "accel_cmd": self.latest_accel_cmd,
                        "brake_cmd": self.latest_brake_cmd,
                        "speed_limited": self.latest_speed_limited,
                        "nearest_trajectory_index":
                            self.latest_nearest_trajectory_index,
                        "position_x_m": (
                            self.previous_position[0] if self.previous_position else None
                        ),
                        "position_y_m": (
                            self.previous_position[1] if self.previous_position else None
                        ),
                    }
                )
            if now - self.started >= timeout_sec:
                self.done = True
                self.exit_reason = "timeout"
            elif self.initial_lap is not None and now - self.last_progress >= stuck_timeout_sec:
                self.done = True
                self.exit_reason = "stuck"

        def metrics(self):
            elapsed = time.monotonic() - self.started
            rms = (
                math.sqrt(statistics.fmean(value * value for value in self.steering_deltas))
                if self.steering_deltas
                else 0.0
            )
            steering_angle_rms = (
                math.sqrt(statistics.fmean(value * value for value in self.steering_angles))
                if self.steering_angles
                else 0.0
            )
            progress = 0.0
            if self.initial_lap is not None and self.lap is not None:
                progress = min(
                    1.0, max(0.0, (self.lap - self.initial_lap) / target_laps)
                )
                if not progress:
                    progress = min(0.99, max(0.0, self.section / 100.0))
            actual_speed_mean = finite_mean(self.actual_speeds_mps)
            target_speed_mean = finite_mean(self.target_speeds_mps)
            speed_limited_ratio = finite_mean(self.speed_limited_samples)
            standing_lap = self.completed_lap_times[0] if self.completed_lap_times else None
            flying_lap = (
                self.completed_lap_times[1] if len(self.completed_lap_times) > 1 else None
            )
            weighted_lap = self.completed_lap_time
            if flying_lap is not None:
                total_weight = standing_lap_weight + flying_lap_weight
                if total_weight > 0.0:
                    weighted_lap = (
                        standing_lap_weight * standing_lap +
                        flying_lap_weight * flying_lap
                    ) / total_weight
            return {
                "candidate_id": request["candidate_id"],
                "parameter_hash": request["parameter_hash"],
                "completed": self.exit_reason == "completed",
                "invalid": self.exit_reason == "invalid_command",
                "exit_reason": self.exit_reason,
                "lap_time_seconds": (
                    weighted_lap if self.exit_reason == "completed" else elapsed
                ),
                "lap_times_seconds": self.completed_lap_times,
                "standing_lap_time_seconds": standing_lap,
                "flying_lap_time_seconds": flying_lap,
                "elapsed_seconds": elapsed,
                "progress": progress,
                "distance_traveled_m": self.distance_traveled_m,
                "distance_average_speed_mps": (
                    self.distance_traveled_m / elapsed if elapsed > 0.0 else 0.0
                ),
                "initial_speed_mps": self.initial_speed_mps,
                "actual_speed_mean_mps": actual_speed_mean,
                "actual_speed_max_mps": max(self.actual_speeds_mps, default=0.0),
                "target_speed_mean_mps": target_speed_mean,
                "target_speed_min_mps": min(self.target_speeds_mps, default=0.0),
                "target_speed_max_mps": max(self.target_speeds_mps, default=0.0),
                "absolute_speed_error_mean_mps": finite_mean(
                    self.absolute_speed_errors_mps
                ),
                "absolute_speed_error_p95_mps": percentile(
                    self.absolute_speed_errors_mps, 0.95
                ),
                "commanded_acceleration_mean_mps2": finite_mean(
                    self.commanded_accelerations_mps2
                ),
                "commanded_acceleration_max_mps2": max(
                    self.commanded_accelerations_mps2, default=0.0
                ),
                "actual_acceleration_mean_mps2": finite_mean(
                    self.actual_accelerations_mps2
                ),
                "actual_acceleration_max_mps2": max(
                    self.actual_accelerations_mps2, default=0.0
                ),
                "speed_limited_ratio": speed_limited_ratio,
                "speed_limited_estimated_seconds": elapsed * speed_limited_ratio,
                "time_to_10_kmh_seconds": self.speed_threshold_times["10_kmh"],
                "time_to_20_kmh_seconds": self.speed_threshold_times["20_kmh"],
                "time_to_30_kmh_seconds": self.speed_threshold_times["30_kmh"],
                "time_to_34_kmh_seconds": self.speed_threshold_times["34_kmh"],
                "lap_completion_speed_mps": self.lap_completion_speed_mps,
                "section_splits": self.section_splits,
                "diagnostic_trace": self.diagnostic_trace,
                "collision_count": self.collision_count,
                "wall_count": 0,
                "over_count": 0,
                "penalty_seconds": 0.0,
                "lateral_error_p95_m": percentile(self.lateral_errors, 0.95),
                "lateral_error_max_m": max(self.lateral_errors, default=0.0),
                "steering_delta_rms": rms,
                "steering_angle_rms": steering_angle_rms,
                "steering_rate_limit_ratio": (
                    statistics.fmean(self.rate_limit_samples) if self.rate_limit_samples else 0.0
                ),
            }

    rclpy.init()
    monitor = Monitor()
    try:
        while rclpy.ok() and not monitor.done:
            rclpy.spin_once(monitor, timeout_sec=0.2)
        result = monitor.metrics()
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(result, sort_keys=True))
        return 0
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
