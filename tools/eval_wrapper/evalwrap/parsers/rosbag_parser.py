from __future__ import annotations

import json
import math
import struct
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEFAULT_THRESHOLDS = {
    "low_speed_mps": 0.5,
    "stuck_duration_sec": 3.0,
    "hard_brake_mps2": -1.5,
    "high_accel_mps2": 1.0,
    "steer_oscillation_warn": 0.8,
    "section_count": 10.0,
    "sync_tolerance_sec": 0.5,
    "path_error_warn_m": 1.5,
    "corner_curvature_min_1pm": 0.035,
    "corner_min_length_m": 2.0,
    "corner_merge_gap_m": 2.0,
    "corner_padding_m": 1.0,
    "corner_id_rotation": 0.0,
    "grade_min_speed_mps": 1.0,
    "grade_min_distance_delta_m": 0.5,
    "grade_smoothing_window_m": 2.0,
}

ODOMETRY_TOPICS = {
    "/localization/kinematic_state",
    "/Odometry",
}
ACCEL_TOPICS = {
    "/localization/acceleration",
}
CONTROL_TOPICS = {
    "/control/command/control_cmd",
}
ACTUATION_TOPICS = {
    "/control/command/actuation_cmd",
}
VELOCITY_STATUS_TOPICS = {
    "/vehicle/status/velocity_status",
}
STEERING_STATUS_TOPICS = {
    "/vehicle/status/steering_status",
}
TRAJECTORY_TOPICS = {
    "/planning/scenario_planning/trajectory",
}
AWSIM_STATUS_TOPICS = {
    "/awsim/status",
}
DELAY_DEBUG_TOPICS = {
    "/delay_aware_mpc/debug",
}
SPEED_PROFILE_DEBUG_TOPICS = {
    "/mpc/speed_profile_debug",
}
ACKERMANN_CONTROL_COMMAND_TYPE = "autoware_auto_control_msgs/msg/AckermannControlCommand"
RawDecoder = Callable[[bytes, float], dict[str, float | None] | None]
PathPoint = tuple[float, float, float | None]


@dataclass
class ParsedRosbag:
    available: bool = False
    reason: str | None = None
    warnings: list[str] = field(default_factory=list)
    metrics: dict[str, float | int | None] = field(default_factory=dict)
    vehicle_timeseries: list[dict[str, float | int | None]] = field(default_factory=list)
    control_timeseries: list[dict[str, float | None]] = field(default_factory=list)
    delay_debug_timeseries: list[dict[str, object]] = field(default_factory=list)
    speed_profile_debug_timeseries: list[dict[str, object]] = field(default_factory=list)
    section_summary: list[dict[str, float | int | None]] = field(default_factory=list)
    awsim_section_summary: list[dict[str, float | int | None]] = field(default_factory=list)
    corner_summary: list[dict[str, float | int | str | None]] = field(default_factory=list)
    trajectory_reference: list[dict[str, float | int | str | None]] = field(default_factory=list)
    trajectory_source: str | None = None
    events: list[dict[str, float | int | str | None]] = field(default_factory=list)


def parse_rosbag(
    domain_dir: Path,
    thresholds: dict[str, float] | None = None,
    fallback_trajectory_points: list[Sequence[float | None]] | None = None,
    fallback_trajectory_source: str | None = None,
) -> ParsedRosbag:
    bag_uri = _find_rosbag_uri(domain_dir)
    if bag_uri is None:
        return ParsedRosbag(available=False, reason="rosbag2_autoware storage not found")

    try:
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except Exception as exc:  # noqa: BLE001 - optional dependency
        return ParsedRosbag(available=False, reason=f"rosbag parser unavailable: {exc}")

    storage_id, input_format, output_format = _infer_storage_options(bag_uri)
    if storage_id is None:
        return ParsedRosbag(available=False, reason=f"unsupported rosbag storage: {bag_uri}")

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(bag_uri), storage_id=storage_id),
            ConverterOptions(input_serialization_format=input_format, output_serialization_format=output_format),
        )
    except Exception as exc:  # noqa: BLE001
        return ParsedRosbag(available=False, reason=f"failed to open rosbag: {exc}")

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    target_topics = _target_topics(topic_types)
    if not target_topics:
        return ParsedRosbag(available=False, reason="rosbag has no supported evaluation topics")
    message_types = {}
    raw_decoders: dict[str, RawDecoder] = {}
    warnings: list[str] = []
    for topic_name in target_topics:
        try:
            message_types[topic_name] = get_message(topic_types[topic_name])
        except Exception as exc:  # noqa: BLE001
            raw_decoder = _raw_decoder_for(topic_types[topic_name])
            if raw_decoder is None:
                warnings.append(f"unsupported message type for {topic_name}: {topic_types[topic_name]} ({exc})")
            else:
                raw_decoders[topic_name] = raw_decoder
                warnings.append(f"using raw decoder for {topic_name}: {topic_types[topic_name]} ({exc})")

    odometry: list[dict[str, float | None]] = []
    accel: list[dict[str, float | None]] = []
    velocity_status: list[dict[str, float | None]] = []
    steering_status: list[dict[str, float | None]] = []
    control: list[dict[str, float | None]] = []
    actuation: list[dict[str, float | None]] = []
    awsim_status: list[dict[str, float | int | None]] = []
    delay_debug: list[dict[str, object]] = []
    speed_profile_debug: list[dict[str, object]] = []
    trajectory_points: list[PathPoint] = []
    trajectory_source: str | None = None
    raw_decode_failed_topics: set[str] = set()

    while reader.has_next():
        topic_name, raw, stamp = reader.read_next()
        time_sec = stamp * 1e-9
        if topic_name in raw_decoders and topic_name not in message_types:
            decoded = raw_decoders[topic_name](raw, time_sec)
            if decoded is None:
                if topic_name not in raw_decode_failed_topics:
                    warnings.append(f"failed to raw-decode {topic_name}")
                    raw_decode_failed_topics.add(topic_name)
                continue
            if topic_name in CONTROL_TOPICS:
                control.append(decoded)
            continue
        if topic_name not in message_types:
            continue
        try:
            msg = deserialize_message(raw, message_types[topic_name])
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"failed to deserialize {topic_name}: {exc}")
            continue
        if topic_name in ODOMETRY_TOPICS:
            odometry.append(_extract_odometry(msg, time_sec))
        elif topic_name in ACCEL_TOPICS:
            accel.append({"time_sec": time_sec, "acceleration_mps2": _get_nested_float(msg, "accel.accel.linear.x")})
        elif topic_name in VELOCITY_STATUS_TOPICS:
            velocity_status.append({"time_sec": time_sec, "speed_mps": _first_float(msg, VELOCITY_FIELDS)})
        elif topic_name in STEERING_STATUS_TOPICS:
            steering_status.append({"time_sec": time_sec, "steering_rad": _first_float(msg, STEERING_FIELDS)})
        elif topic_name in CONTROL_TOPICS:
            control.append(_extract_control_cmd(msg, time_sec))
        elif topic_name in ACTUATION_TOPICS:
            actuation.append(_extract_actuation_cmd(msg, time_sec))
        elif topic_name in AWSIM_STATUS_TOPICS:
            status = _extract_awsim_status(msg, time_sec)
            if status is not None:
                awsim_status.append(status)
        elif topic_name in DELAY_DEBUG_TOPICS:
            row = _extract_json_debug(msg, time_sec)
            if row is not None:
                delay_debug.append(row)
        elif topic_name in SPEED_PROFILE_DEBUG_TOPICS:
            row = _extract_json_debug(msg, time_sec)
            if row is not None:
                speed_profile_debug.append(row)
        elif topic_name in TRAJECTORY_TOPICS:
            points = _extract_trajectory_points(msg)
            if points:
                trajectory_points = points
                trajectory_source = topic_name

    if not trajectory_points and fallback_trajectory_points:
        trajectory_points = _normalize_trajectory_points(fallback_trajectory_points)
        trajectory_source = fallback_trajectory_source or "fallback"

    return build_analysis_from_series(
        odometry=odometry,
        acceleration=accel,
        velocity_status=velocity_status,
        steering_status=steering_status,
        control=control,
        actuation=actuation,
        awsim_status=awsim_status,
        delay_debug=delay_debug,
        speed_profile_debug=speed_profile_debug,
        trajectory_points=trajectory_points,
        trajectory_source=trajectory_source,
        thresholds=thresholds,
        warnings=warnings,
    )


def build_analysis_from_series(
    *,
    odometry: list[dict[str, float | None]] | None = None,
    acceleration: list[dict[str, float | None]] | None = None,
    velocity_status: list[dict[str, float | None]] | None = None,
    steering_status: list[dict[str, float | None]] | None = None,
    control: list[dict[str, float | None]] | None = None,
    actuation: list[dict[str, float | None]] | None = None,
    awsim_status: list[dict[str, float | int | None]] | None = None,
    delay_debug: list[dict[str, object]] | None = None,
    speed_profile_debug: list[dict[str, object]] | None = None,
    trajectory_points: list[Sequence[float | None]] | None = None,
    trajectory_source: str | None = None,
    thresholds: dict[str, float] | None = None,
    warnings: list[str] | None = None,
) -> ParsedRosbag:
    merged_thresholds = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
    sync_tolerance = float(merged_thresholds["sync_tolerance_sec"])
    odometry = _sorted_rows(odometry or [])
    acceleration = _sorted_rows(acceleration or [])
    velocity_status = _sorted_rows(velocity_status or [])
    steering_status = _sorted_rows(steering_status or [])
    control = _sorted_rows(control or [])
    actuation = _sorted_rows(actuation or [])
    awsim_status = _sorted_rows(awsim_status or [])
    delay_debug = _sorted_debug_rows(delay_debug or [])
    speed_profile_debug = _sorted_debug_rows(speed_profile_debug or [])
    trajectory_points = _normalize_trajectory_points(trajectory_points or [])

    vehicle_rows = _merge_vehicle_rows(odometry, velocity_status, acceleration, steering_status, sync_tolerance)
    control_rows = _merge_control_rows(control, actuation, sync_tolerance)
    if trajectory_points and vehicle_rows:
        trajectory_profile = _build_trajectory_profile(trajectory_points, merged_thresholds)
        _attach_trajectory_projection(vehicle_rows, trajectory_profile)
        trajectory_reference = _build_trajectory_reference_rows(trajectory_profile, trajectory_source)
    else:
        trajectory_profile = None
        trajectory_reference = []

    if not vehicle_rows and not control_rows and not awsim_status:
        return ParsedRosbag(
            available=False,
            reason="rosbag parsed but no usable time-series samples were found",
            warnings=warnings or [],
        )

    _attach_distance_and_sections(vehicle_rows, int(merged_thresholds["section_count"]))
    _attach_odometry_grade(vehicle_rows, merged_thresholds)
    metrics = _compute_metrics(vehicle_rows, control_rows, merged_thresholds)
    events = _detect_events(vehicle_rows, control_rows, metrics, merged_thresholds)
    sections = _build_section_summary(vehicle_rows, events)
    awsim_sections = _build_awsim_section_summary(awsim_status, vehicle_rows)
    corners = _build_corner_summary(vehicle_rows, events, trajectory_profile)
    return ParsedRosbag(
        available=True,
        warnings=warnings or [],
        metrics=metrics,
        vehicle_timeseries=vehicle_rows,
        control_timeseries=control_rows,
        delay_debug_timeseries=delay_debug,
        speed_profile_debug_timeseries=speed_profile_debug,
        section_summary=sections,
        awsim_section_summary=awsim_sections,
        corner_summary=corners,
        trajectory_reference=trajectory_reference,
        trajectory_source=trajectory_source,
        events=events,
    )


VELOCITY_FIELDS = (
    "longitudinal_velocity",
    "longitudinal_velocity_mps",
    "velocity",
    "speed",
    "twist.twist.linear.x",
)
STEERING_FIELDS = (
    "steering_tire_angle",
    "steering_angle",
    "steering_wheel_angle",
)


def _find_rosbag_uri(domain_dir: Path) -> Path | None:
    candidates = [
        domain_dir / "rosbag2_autoware",
        domain_dir / "rosbag2_autoware.mcap",
        domain_dir / "rosbag2_autoware_0.mcap",
        domain_dir / "rosbag.mcap",
    ]
    for candidate in candidates:
        if not candidate.exists():
            continue
        if candidate.is_file() and candidate.suffix in {".mcap", ".db3"}:
            return candidate
        if candidate.is_dir():
            if (candidate / "metadata.yaml").exists():
                return candidate
            if any(path.suffix in {".mcap", ".db3"} for path in candidate.iterdir()):
                return candidate
    return None


def _infer_storage_options(uri: Path) -> tuple[str | None, str, str]:
    storage_by_suffix = {".mcap": ("mcap", "", ""), ".db3": ("sqlite3", "cdr", "cdr")}
    if uri.is_file():
        return storage_by_suffix.get(uri.suffix, (None, "", ""))
    for child in uri.iterdir():
        if child.suffix in storage_by_suffix:
            return storage_by_suffix[child.suffix]
    return None, "", ""


def _target_topics(topic_types: dict[str, str]) -> set[str]:
    supported = (
        ODOMETRY_TOPICS
        | ACCEL_TOPICS
        | CONTROL_TOPICS
        | ACTUATION_TOPICS
        | VELOCITY_STATUS_TOPICS
        | STEERING_STATUS_TOPICS
        | TRAJECTORY_TOPICS
        | AWSIM_STATUS_TOPICS
        | DELAY_DEBUG_TOPICS
        | SPEED_PROFILE_DEBUG_TOPICS
    )
    return set(topic_types).intersection(supported)


def _extract_odometry(msg: Any, time_sec: float) -> dict[str, float | None]:
    return {
        "time_sec": time_sec,
        "x_m": _get_nested_float(msg, "pose.pose.position.x"),
        "y_m": _get_nested_float(msg, "pose.pose.position.y"),
        "z_m": _get_nested_float(msg, "pose.pose.position.z"),
        "speed_mps": _get_nested_float(msg, "twist.twist.linear.x"),
        "yaw_rate_rps": _get_nested_float(msg, "twist.twist.angular.z"),
    }


def _extract_control_cmd(msg: Any, time_sec: float) -> dict[str, float | None]:
    return {
        "time_sec": time_sec,
        "target_speed_mps": _get_nested_float(msg, "longitudinal.speed"),
        "accel_mps2": _get_nested_float(msg, "longitudinal.acceleration"),
        "steer_rad": _get_nested_float(msg, "lateral.steering_tire_angle"),
        "throttle": None,
        "brake": None,
    }


def _raw_decoder_for(message_type: str) -> RawDecoder | None:
    if message_type == ACKERMANN_CONTROL_COMMAND_TYPE:
        return _decode_ackermann_control_command
    return None


def _decode_ackermann_control_command(raw: bytes, time_sec: float) -> dict[str, float | None] | None:
    # ROS 2 CDR: 4-byte encapsulation header, then AckermannControlCommand fields.
    if len(raw) < 48:
        return None
    try:
        steering = struct.unpack_from("<f", raw, 20)[0]
        target_speed = struct.unpack_from("<f", raw, 36)[0]
        acceleration = struct.unpack_from("<f", raw, 40)[0]
    except struct.error:
        return None
    return {
        "time_sec": time_sec,
        "target_speed_mps": target_speed,
        "accel_mps2": acceleration,
        "steer_rad": steering,
        "throttle": None,
        "brake": None,
    }


def _extract_actuation_cmd(msg: Any, time_sec: float) -> dict[str, float | None]:
    return {
        "time_sec": time_sec,
        "target_speed_mps": None,
        "accel_mps2": _first_float(msg, ("accel_cmd", "acceleration", "actuation.accel_cmd")),
        "steer_rad": _first_float(msg, ("steer_cmd", "steering", "actuation.steer_cmd")),
        "throttle": _first_float(msg, ("throttle", "throttle_cmd", "actuation.accel_cmd")),
        "brake": _first_float(msg, ("brake", "brake_cmd", "actuation.brake_cmd")),
    }


def _extract_awsim_status(msg: Any, time_sec: float) -> dict[str, float | int | None] | None:
    data = getattr(msg, "data", None)
    if data is None or len(data) < 4:
        return None
    try:
        return {
            "time_sec": time_sec,
            "vehicle_index": int(data[0]),
            "lap": int(data[1]),
            "lap_time_sec": float(data[2]),
            "section": int(data[3]),
        }
    except (TypeError, ValueError):
        return None


def _extract_json_debug(msg: Any, time_sec: float) -> dict[str, object] | None:
    data = getattr(msg, "data", None)
    if not isinstance(data, str) or not data:
        return None
    try:
        payload = json.loads(data)
    except json.JSONDecodeError:
        return {"time_sec": time_sec, "raw": data}
    if not isinstance(payload, dict):
        return {"time_sec": time_sec, "raw": data}
    row: dict[str, object] = {"time_sec": time_sec}
    _flatten_debug_payload(row, "", payload)
    return row


def _flatten_debug_payload(output: dict[str, object], prefix: str, payload: dict[str, object]) -> None:
    for key, value in payload.items():
        name = f"{prefix}_{key}" if prefix else str(key)
        if isinstance(value, dict):
            _flatten_debug_payload(output, name, value)
        elif isinstance(value, (int, float, str, bool)) or value is None:
            output[name] = value


def _extract_trajectory_points(msg: Any) -> list[PathPoint]:
    points = getattr(msg, "points", None)
    if not points:
        return []
    result: list[PathPoint] = []
    for point in points:
        x = _get_nested_float(point, "pose.position.x")
        y = _get_nested_float(point, "pose.position.y")
        z = _get_nested_float(point, "pose.position.z")
        if x is not None and y is not None:
            result.append((x, y, z))
    return result


def _normalize_trajectory_points(points: list[Sequence[float | None]]) -> list[PathPoint]:
    normalized: list[PathPoint] = []
    for point in points:
        if len(point) < 2:
            continue
        x = _finite_float(point[0])
        y = _finite_float(point[1])
        z = _finite_float(point[2]) if len(point) >= 3 else None
        if x is None or y is None:
            continue
        normalized.append((x, y, z))
    return normalized


def _finite_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _merge_vehicle_rows(
    odometry: list[dict[str, float | None]],
    velocity_status: list[dict[str, float | None]],
    acceleration: list[dict[str, float | None]],
    steering_status: list[dict[str, float | None]],
    sync_tolerance: float,
) -> list[dict[str, float | int | str | None]]:
    primary = odometry or velocity_status
    rows: list[dict[str, float | int | str | None]] = []
    for source in primary:
        time_sec = source["time_sec"]
        speed = source.get("speed_mps")
        if speed is None and velocity_status:
            speed = _nearest_value(velocity_status, float(time_sec), "speed_mps", sync_tolerance)
        rows.append(
            {
                "time_sec": float(time_sec),
                "x_m": source.get("x_m"),
                "y_m": source.get("y_m"),
                "z_m": source.get("z_m"),
                "distance_m": None,
                "section": None,
                "track_s_m": None,
                "trajectory_curvature_1pm": None,
                "corner_id": None,
                "trajectory_z_m": None,
                "trajectory_grade_rad": None,
                "trajectory_grade_percent": None,
                "grade_rad": None,
                "grade_percent": None,
                "grade_source": None,
                "speed_mps": speed,
                "acceleration_mps2": _nearest_value(acceleration, float(time_sec), "acceleration_mps2", sync_tolerance),
                "steering_rad": _nearest_value(steering_status, float(time_sec), "steering_rad", sync_tolerance),
                "yaw_rate_rps": source.get("yaw_rate_rps"),
                "path_error_m": None,
            }
        )
    return rows


def _merge_control_rows(
    control: list[dict[str, float | None]],
    actuation: list[dict[str, float | None]],
    sync_tolerance: float,
) -> list[dict[str, float | None]]:
    primary = control or actuation
    rows: list[dict[str, float | None]] = []
    for source in primary:
        time_sec = float(source["time_sec"])
        rows.append(
            {
                "time_sec": time_sec,
                "target_speed_mps": source.get("target_speed_mps"),
                "accel_mps2": source.get("accel_mps2"),
                "steer_rad": source.get("steer_rad"),
                "throttle": source.get("throttle")
                if source.get("throttle") is not None
                else _nearest_value(actuation, time_sec, "throttle", sync_tolerance),
                "brake": source.get("brake")
                if source.get("brake") is not None
                else _nearest_value(actuation, time_sec, "brake", sync_tolerance),
            }
        )
    return rows


def _attach_trajectory_projection(
    rows: list[dict[str, float | int | str | None]],
    trajectory_profile: dict[str, object],
) -> None:
    points = trajectory_profile["points"]
    s_values = trajectory_profile["s_values"]
    curvatures = trajectory_profile["curvatures"]
    corner_by_index = trajectory_profile["corner_by_index"]
    z_values = trajectory_profile["z_values"]
    grade_rads = trajectory_profile["grade_rads"]
    grade_percents = trajectory_profile["grade_percents"]
    for row in rows:
        x = row.get("x_m")
        y = row.get("y_m")
        if x is None or y is None:
            continue
        nearest_idx, min_sq = min(
            enumerate((float(x) - tx) ** 2 + (float(y) - ty) ** 2 for tx, ty, _tz in points),
            key=lambda item: item[1],
        )
        row["path_error_m"] = math.sqrt(min_sq)
        row["track_s_m"] = s_values[nearest_idx]
        row["trajectory_curvature_1pm"] = curvatures[nearest_idx]
        row["corner_id"] = corner_by_index[nearest_idx]
        row["trajectory_z_m"] = z_values[nearest_idx]
        row["trajectory_grade_rad"] = grade_rads[nearest_idx]
        row["trajectory_grade_percent"] = grade_percents[nearest_idx]
        if grade_rads[nearest_idx] is not None:
            row["grade_rad"] = grade_rads[nearest_idx]
            row["grade_percent"] = grade_percents[nearest_idx]
            row["grade_source"] = "trajectory"


def _build_trajectory_profile(
    trajectory_points: list[PathPoint],
    thresholds: dict[str, float],
) -> dict[str, object]:
    s_values = _trajectory_s_values(trajectory_points)
    curvatures = _trajectory_curvatures(trajectory_points)
    z_values = [point[2] for point in trajectory_points]
    grade_rads, grade_percents = _trajectory_grades(s_values, z_values)
    segments = _detect_corner_segments(trajectory_points, s_values, curvatures, thresholds)
    segments = _apply_corner_id_rotation(segments, int(thresholds.get("corner_id_rotation", 0.0)))
    corner_by_index: list[str | None] = [None] * len(trajectory_points)
    for segment in segments:
        for index in range(int(segment["start_idx"]), int(segment["end_idx"]) + 1):
            corner_by_index[index] = str(segment["corner_id"])
    return {
        "points": trajectory_points,
        "s_values": s_values,
        "curvatures": curvatures,
        "z_values": z_values,
        "grade_rads": grade_rads,
        "grade_percents": grade_percents,
        "segments": segments,
        "corner_by_index": corner_by_index,
    }


def _build_trajectory_reference_rows(
    trajectory_profile: dict[str, object],
    trajectory_source: str | None,
) -> list[dict[str, float | int | str | None]]:
    points = trajectory_profile["points"]
    s_values = trajectory_profile["s_values"]
    curvatures = trajectory_profile["curvatures"]
    z_values = trajectory_profile["z_values"]
    grade_rads = trajectory_profile["grade_rads"]
    grade_percents = trajectory_profile["grade_percents"]
    corner_by_index = trajectory_profile["corner_by_index"]
    output: list[dict[str, float | int | str | None]] = []
    for index, ((x, y, _z), s_value, curvature, z_value, grade_rad, grade_percent, corner_id) in enumerate(
        zip(points, s_values, curvatures, z_values, grade_rads, grade_percents, corner_by_index)
    ):
        output.append(
            {
                "point_index": index,
                "x_m": x,
                "y_m": y,
                "z_m": z_value,
                "track_s_m": s_value,
                "grade_rad": grade_rad,
                "grade_percent": grade_percent,
                "trajectory_curvature_1pm": curvature,
                "corner_id": corner_id,
                "trajectory_source": trajectory_source,
            }
        )
    return output


def _trajectory_s_values(points: list[PathPoint]) -> list[float]:
    if not points:
        return []
    values = [0.0]
    total = 0.0
    for prev, current in zip(points, points[1:]):
        total += math.hypot(current[0] - prev[0], current[1] - prev[1])
        values.append(total)
    return values


def _trajectory_curvatures(points: list[PathPoint]) -> list[float]:
    if len(points) < 3:
        return [0.0] * len(points)
    curvatures = [0.0] * len(points)
    for index in range(1, len(points) - 1):
        p0 = points[index - 1]
        p1 = points[index]
        p2 = points[index + 1]
        a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        c = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
        area2 = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]))
        denom = a * b * c
        curvatures[index] = (2.0 * area2 / denom) if denom > 1e-9 else 0.0
    if len(points) >= 2:
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]
    return curvatures


def _trajectory_grades(
    s_values: list[float],
    z_values: list[float | None],
    min_distance_delta_m: float = 0.5,
    smoothing_window_m: float = 2.0,
    valid_mask: list[bool] | None = None,
) -> tuple[list[float | None], list[float | None]]:
    if len(s_values) != len(z_values) or len(s_values) < 2:
        empty = [None] * len(s_values)
        return empty, empty.copy()
    valid = [
        value is not None and (valid_mask[index] if valid_mask is not None else True)
        for index, value in enumerate(z_values)
    ]
    finite_z = [float(value) for index, value in enumerate(z_values) if valid[index] and value is not None]
    if len(finite_z) < 2 or max(finite_z) - min(finite_z) <= 1e-4:
        empty = [None] * len(s_values)
        return empty, empty.copy()

    grade_rads: list[float | None] = []
    grade_percents: list[float | None] = []
    for index, z_value in enumerate(z_values):
        if z_value is None or not valid[index]:
            grade_rads.append(None)
            grade_percents.append(None)
            continue
        bounds = _grade_bounds(
            index,
            s_values,
            valid,
            max(0.0, float(min_distance_delta_m)),
            max(0.0, float(smoothing_window_m)),
        )
        if bounds is None:
            grade_rads.append(None)
            grade_percents.append(None)
            continue
        left, right = bounds
        ds = float(s_values[right]) - float(s_values[left])
        if abs(ds) < max(float(min_distance_delta_m), 1e-9):
            grade_rads.append(None)
            grade_percents.append(None)
            continue
        left_z = z_values[left]
        right_z = z_values[right]
        if left_z is None or right_z is None:
            grade_rads.append(None)
            grade_percents.append(None)
            continue
        dz = float(right_z) - float(left_z)
        grade_rad = math.atan2(dz, ds)
        grade_rads.append(grade_rad)
        grade_percents.append(dz / ds * 100.0)
    return grade_rads, grade_percents


def _grade_bounds(
    index: int,
    s_values: list[float],
    valid: list[bool],
    min_distance_delta_m: float,
    smoothing_window_m: float,
) -> tuple[int, int] | None:
    left = index
    right = index
    half_window = max(smoothing_window_m * 0.5, min_distance_delta_m * 0.5)
    while True:
        previous = _previous_valid_index(valid, left)
        if previous is None:
            break
        left = previous
        if float(s_values[index]) - float(s_values[left]) >= half_window:
            break
    while True:
        next_index = _next_valid_index(valid, right)
        if next_index is None:
            break
        right = next_index
        if float(s_values[right]) - float(s_values[index]) >= half_window:
            break
    while float(s_values[right]) - float(s_values[left]) < min_distance_delta_m:
        previous = _previous_valid_index(valid, left)
        next_index = _next_valid_index(valid, right)
        if previous is None and next_index is None:
            return None
        if previous is not None and (
            next_index is None
            or abs(float(s_values[index]) - float(s_values[previous]))
            <= abs(float(s_values[next_index]) - float(s_values[index]))
        ):
            left = previous
        elif next_index is not None:
            right = next_index
    if left == right:
        return None
    return left, right


def _previous_valid_index(valid: list[bool], start: int) -> int | None:
    for index in range(start - 1, -1, -1):
        if valid[index]:
            return index
    return None


def _next_valid_index(valid: list[bool], start: int) -> int | None:
    for index in range(start + 1, len(valid)):
        if valid[index]:
            return index
    return None


def _detect_corner_segments(
    points: list[PathPoint],
    s_values: list[float],
    curvatures: list[float],
    thresholds: dict[str, float],
) -> list[dict[str, float | int | str]]:
    if len(points) < 3:
        return []
    curvature_min = thresholds["corner_curvature_min_1pm"]
    min_length = thresholds["corner_min_length_m"]
    merge_gap = thresholds["corner_merge_gap_m"]
    padding = thresholds["corner_padding_m"]
    raw: list[tuple[int, int]] = []
    start: int | None = None
    for index, curvature in enumerate(curvatures):
        if abs(curvature) >= curvature_min:
            if start is None:
                start = index
        elif start is not None:
            raw.append((start, index - 1))
            start = None
    if start is not None:
        raw.append((start, len(curvatures) - 1))

    padded = [
        (_index_at_or_before(s_values, s_values[start] - padding), _index_at_or_after(s_values, s_values[end] + padding))
        for start, end in raw
    ]
    merged: list[tuple[int, int]] = []
    for start, end in padded:
        if not merged:
            merged.append((start, end))
            continue
        prev_start, prev_end = merged[-1]
        if s_values[start] - s_values[prev_end] <= merge_gap:
            merged[-1] = (prev_start, max(prev_end, end))
        else:
            merged.append((start, end))

    segments: list[dict[str, float | int | str]] = []
    for start, end in merged:
        length = s_values[end] - s_values[start]
        if length < min_length:
            continue
        peak = max(abs(curvature) for curvature in curvatures[start : end + 1])
        segments.append(
            {
                "corner_id": f"corner_{len(segments) + 1:02d}",
                "start_idx": start,
                "end_idx": end,
                "start_s_m": s_values[start],
                "end_s_m": s_values[end],
                "length_m": length,
                "peak_curvature_1pm": peak,
            }
        )
    return segments


def _apply_corner_id_rotation(
    segments: list[dict[str, float | int | str]],
    rotation: int,
) -> list[dict[str, float | int | str]]:
    if not segments:
        return segments
    count = len(segments)
    shift = rotation % count
    if shift == 0:
        return segments
    width = max(2, len(str(count)))
    output: list[dict[str, float | int | str]] = []
    for index, segment in enumerate(segments):
        new_number = ((index - shift) % count) + 1
        updated = dict(segment)
        updated["corner_id"] = f"corner_{new_number:0{width}d}"
        output.append(updated)
    return output


def _index_at_or_before(values: list[float], target: float) -> int:
    selected = 0
    for index, value in enumerate(values):
        if value <= target:
            selected = index
        else:
            break
    return selected


def _index_at_or_after(values: list[float], target: float) -> int:
    for index, value in enumerate(values):
        if value >= target:
            return index
    return len(values) - 1


def _attach_distance_and_sections(rows: list[dict[str, float | int | None]], section_count: int) -> None:
    if not rows:
        return
    distance = 0.0
    prev_x = rows[0].get("x_m")
    prev_y = rows[0].get("y_m")
    rows[0]["distance_m"] = 0.0
    for row in rows[1:]:
        x = row.get("x_m")
        y = row.get("y_m")
        if x is not None and y is not None and prev_x is not None and prev_y is not None:
            distance += math.hypot(float(x) - float(prev_x), float(y) - float(prev_y))
        row["distance_m"] = distance
        prev_x = x if x is not None else prev_x
        prev_y = y if y is not None else prev_y

    total_distance = max(float(rows[-1].get("distance_m") or 0.0), 1e-9)
    count = max(1, section_count)
    for row in rows:
        section = int(float(row.get("distance_m") or 0.0) / total_distance * count) + 1
        row["section"] = min(section, count)


def _attach_odometry_grade(rows: list[dict[str, float | int | str | None]], thresholds: dict[str, float]) -> None:
    if not rows:
        return
    distances = [float(row.get("distance_m") or 0.0) for row in rows]
    z_values = [
        float(row["z_m"]) if isinstance(row.get("z_m"), (int, float)) else None
        for row in rows
    ]
    min_speed_mps = float(thresholds.get("grade_min_speed_mps", 1.0))
    valid_mask = [
        z_value is not None
        and isinstance(row.get("speed_mps"), (int, float))
        and abs(float(row.get("speed_mps") or 0.0)) >= min_speed_mps
        for row, z_value in zip(rows, z_values)
    ]
    grade_rads, grade_percents = _trajectory_grades(
        distances,
        z_values,
        min_distance_delta_m=float(thresholds.get("grade_min_distance_delta_m", 0.5)),
        smoothing_window_m=float(thresholds.get("grade_smoothing_window_m", 2.0)),
        valid_mask=valid_mask,
    )
    for row, grade_rad, grade_percent in zip(rows, grade_rads, grade_percents):
        if grade_rad is None or row.get("grade_rad") is not None:
            continue
        row["grade_rad"] = grade_rad
        row["grade_percent"] = grade_percent
        row["grade_source"] = "odometry"


def _compute_metrics(
    vehicle_rows: list[dict[str, float | int | None]],
    control_rows: list[dict[str, float | None]],
    thresholds: dict[str, float],
) -> dict[str, float | int | None]:
    speed_rows = [(float(row["time_sec"]), row.get("speed_mps")) for row in vehicle_rows]
    speeds = _values(speed_rows)
    vehicle_accels = _values((float(row["time_sec"]), row.get("acceleration_mps2")) for row in vehicle_rows)
    control_accels = _values((float(row["time_sec"]), row.get("accel_mps2")) for row in control_rows)
    steer_rows = [(float(row["time_sec"]), row.get("steering_rad")) for row in vehicle_rows]
    if not _values(steer_rows):
        steer_rows = [(float(row["time_sec"]), row.get("steer_rad")) for row in control_rows]
    steers = _values(steer_rows)
    path_errors = _values((float(row["time_sec"]), row.get("path_error_m")) for row in vehicle_rows)
    accel_source = vehicle_accels or control_accels
    low_speed_time, stuck_count = _low_speed_segments(vehicle_rows, thresholds)
    return {
        "rosbag_available": 1,
        "low_speed_time_sec": low_speed_time,
        "stuck_count": stuck_count,
        "max_speed_mps": max(speeds) if speeds else None,
        "avg_speed_mps": _time_weighted_average(speed_rows),
        "max_abs_steer_rad": max((abs(value) for value in steers), default=None),
        "steer_oscillation_score": _oscillation_score(steer_rows),
        "max_accel_mps2": max(accel_source) if accel_source else None,
        "max_decel_mps2": min(accel_source) if accel_source else None,
        "avg_path_error_m": _mean(path_errors),
        "max_path_error_m": max(path_errors) if path_errors else None,
    }


def _detect_events(
    vehicle_rows: list[dict[str, float | int | None]],
    control_rows: list[dict[str, float | None]],
    metrics: dict[str, float | int | None],
    thresholds: dict[str, float],
) -> list[dict[str, float | int | str | None]]:
    events: list[dict[str, float | int | str | None]] = []
    events.extend(_low_speed_events(vehicle_rows, thresholds))
    accel_rows = [
        (float(row["time_sec"]), row.get("acceleration_mps2"), row.get("section")) for row in vehicle_rows
    ]
    if not _values((time_sec, value) for time_sec, value, _section in accel_rows):
        accel_rows = [(float(row["time_sec"]), row.get("accel_mps2"), None) for row in control_rows]

    for time_sec, accel, section in accel_rows:
        if accel is None:
            continue
        if accel <= thresholds["hard_brake_mps2"]:
            events.append(
                {
                    "time_sec": time_sec,
                    "lap": None,
                    "section": section,
                    "event_type": "hard_brake",
                    "severity": "warn",
                    "description": f"acceleration {accel:.3f} m/s^2 <= {thresholds['hard_brake_mps2']:.3f}",
                }
            )
        elif accel >= thresholds["high_accel_mps2"]:
            events.append(
                {
                    "time_sec": time_sec,
                    "lap": None,
                    "section": section,
                    "event_type": "high_accel",
                    "severity": "warn",
                    "description": f"acceleration {accel:.3f} m/s^2 >= {thresholds['high_accel_mps2']:.3f}",
                }
            )

    steer_score = metrics.get("steer_oscillation_score")
    if isinstance(steer_score, (int, float)) and steer_score >= thresholds["steer_oscillation_warn"]:
        events.append(
            {
                "time_sec": None,
                "lap": None,
                "section": None,
                "event_type": "steer_oscillation",
                "severity": "warn",
                "description": f"steer oscillation score {steer_score:.3f} >= {thresholds['steer_oscillation_warn']:.3f}",
            }
        )

    max_path_error = metrics.get("max_path_error_m")
    if isinstance(max_path_error, (int, float)) and max_path_error >= thresholds["path_error_warn_m"]:
        worst = max(
            (row for row in vehicle_rows if row.get("path_error_m") is not None),
            key=lambda row: float(row.get("path_error_m") or 0.0),
            default=None,
        )
        events.append(
            {
                "time_sec": float(worst["time_sec"]) if worst else None,
                "lap": None,
                "section": worst.get("section") if worst else None,
                "event_type": "path_deviation",
                "severity": "warn",
                "description": f"path error {max_path_error:.3f} m >= {thresholds['path_error_warn_m']:.3f}",
            }
        )
    return events


def _build_section_summary(
    vehicle_rows: list[dict[str, float | int | None]],
    events: list[dict[str, float | int | str | None]],
) -> list[dict[str, float | int | None]]:
    sections = sorted({int(row["section"]) for row in vehicle_rows if row.get("section") is not None})
    output: list[dict[str, float | int | None]] = []
    for section in sections:
        rows = [row for row in vehicle_rows if row.get("section") == section]
        speeds = [float(row["speed_mps"]) for row in rows if row.get("speed_mps") is not None]
        path_errors = [float(row["path_error_m"]) for row in rows if row.get("path_error_m") is not None]
        event_count = sum(1 for event in events if event.get("section") == section)
        output.append(
            {
                "lap": None,
                "section": section,
                "entry_time_sec": float(rows[0]["time_sec"]) if rows else None,
                "exit_time_sec": float(rows[-1]["time_sec"]) if rows else None,
                "duration_sec": float(rows[-1]["time_sec"]) - float(rows[0]["time_sec"]) if len(rows) >= 2 else 0.0,
                "avg_speed_mps": _mean(speeds),
                "max_speed_mps": max(speeds) if speeds else None,
                "min_speed_mps": min(speeds) if speeds else None,
                "event_count": event_count,
                "avg_path_error_m": _mean(path_errors),
                "max_path_error_m": max(path_errors) if path_errors else None,
                "distance_m": (float(rows[-1].get("distance_m") or 0.0) - float(rows[0].get("distance_m") or 0.0))
                if rows
                else None,
            }
        )
    return output


def _build_awsim_section_summary(
    awsim_status: list[dict[str, float | int | None]],
    vehicle_rows: list[dict[str, float | int | None]],
) -> list[dict[str, float | int | None]]:
    rows = [
        row
        for row in awsim_status
        if row.get("time_sec") is not None and row.get("lap") is not None and row.get("section") is not None
    ]
    if not rows:
        return []

    segments: list[
        tuple[
            dict[str, float | int | None],
            dict[str, float | int | None],
            dict[str, float | int | None],
        ]
    ] = []
    start = rows[0]
    previous = rows[0]
    previous_key = (int(start["lap"]), int(start["section"]))
    for row in rows[1:]:
        key = (int(row["lap"]), int(row["section"]))
        if key != previous_key:
            segments.append((start, row, previous))
            start = row
            previous_key = key
        previous = row
    segments.append((start, previous, previous))

    time_zero = float(rows[0]["time_sec"])
    output: list[dict[str, float | int | None]] = []
    for start_row, exit_row, previous_row in segments:
        entry_abs = float(start_row["time_sec"])
        exit_abs = float(exit_row["time_sec"])
        lap = int(start_row["lap"])
        section = int(start_row["section"])
        vehicle_slice = [
            vehicle
            for vehicle in vehicle_rows
            if vehicle.get("time_sec") is not None and entry_abs <= float(vehicle["time_sec"]) <= exit_abs
        ]
        speeds = [float(row["speed_mps"]) for row in vehicle_slice if row.get("speed_mps") is not None]
        path_errors = [float(row["path_error_m"]) for row in vehicle_slice if row.get("path_error_m") is not None]
        output.append(
            {
                "lap": lap,
                "section": section,
                "entry_time_sec": entry_abs - time_zero,
                "exit_time_sec": exit_abs - time_zero,
                "entry_lap_time_sec": float(start_row["lap_time_sec"])
                if start_row.get("lap_time_sec") is not None
                else None,
                "exit_lap_time_sec": _awsim_exit_lap_time(lap, exit_row, previous_row),
                "duration_sec": max(0.0, exit_abs - entry_abs),
                "avg_speed_mps": _mean(speeds),
                "max_speed_mps": max(speeds) if speeds else None,
                "min_speed_mps": min(speeds) if speeds else None,
                "avg_path_error_m": _mean(path_errors),
                "max_path_error_m": max(path_errors) if path_errors else None,
                "sample_count": len(vehicle_slice),
            }
        )
    return output


def _awsim_exit_lap_time(
    lap: int,
    exit_row: dict[str, float | int | None],
    previous_row: dict[str, float | int | None],
) -> float | None:
    if int(exit_row["lap"]) == lap and exit_row.get("lap_time_sec") is not None:
        return float(exit_row["lap_time_sec"])
    if int(previous_row["lap"]) != lap or previous_row.get("lap_time_sec") is None:
        return None
    previous_time = previous_row.get("time_sec")
    exit_time = exit_row.get("time_sec")
    if previous_time is None or exit_time is None:
        return float(previous_row["lap_time_sec"])
    return float(previous_row["lap_time_sec"]) + max(0.0, float(exit_time) - float(previous_time))


def _build_corner_summary(
    vehicle_rows: list[dict[str, float | int | None]],
    events: list[dict[str, float | int | str | None]],
    trajectory_profile: dict[str, object] | None,
) -> list[dict[str, float | int | str | None]]:
    if not vehicle_rows or trajectory_profile is None:
        return []
    segments = {str(segment["corner_id"]): segment for segment in trajectory_profile["segments"]}
    passes: list[tuple[str, list[dict[str, float | int | None]]]] = []
    current_corner: str | None = None
    current_rows: list[dict[str, float | int | None]] = []
    for row in vehicle_rows:
        corner = row.get("corner_id")
        corner_id = str(corner) if corner is not None else None
        if corner_id is None:
            if current_corner is not None and current_rows:
                passes.append((current_corner, current_rows))
            current_corner = None
            current_rows = []
            continue
        if corner_id != current_corner:
            if current_corner is not None and current_rows:
                passes.append((current_corner, current_rows))
            current_corner = corner_id
            current_rows = [row]
        else:
            current_rows.append(row)
    if current_corner is not None and current_rows:
        passes.append((current_corner, current_rows))

    pass_counts: dict[str, int] = {}
    output: list[dict[str, float | int | str | None]] = []
    time_zero = float(vehicle_rows[0]["time_sec"])
    for corner_id, rows in passes:
        pass_counts[corner_id] = pass_counts.get(corner_id, 0) + 1
        speeds = [float(row["speed_mps"]) for row in rows if row.get("speed_mps") is not None]
        path_errors = [float(row["path_error_m"]) for row in rows if row.get("path_error_m") is not None]
        curvatures = [
            abs(float(row["trajectory_curvature_1pm"]))
            for row in rows
            if row.get("trajectory_curvature_1pm") is not None
        ]
        entry_time_abs = float(rows[0]["time_sec"])
        exit_time_abs = float(rows[-1]["time_sec"])
        segment = segments.get(corner_id, {})
        output.append(
            {
                "corner_id": corner_id,
                "pass": pass_counts[corner_id],
                "entry_time_sec": entry_time_abs - time_zero,
                "exit_time_sec": exit_time_abs - time_zero,
                "duration_sec": exit_time_abs - entry_time_abs if len(rows) >= 2 else 0.0,
                "entry_speed_mps": _first_present_speed(rows),
                "min_speed_mps": min(speeds) if speeds else None,
                "avg_speed_mps": _mean(speeds),
                "max_speed_mps": max(speeds) if speeds else None,
                "exit_speed_mps": _last_present_speed(rows),
                "avg_path_error_m": _mean(path_errors),
                "max_path_error_m": max(path_errors) if path_errors else None,
                "entry_distance_m": rows[0].get("distance_m"),
                "exit_distance_m": rows[-1].get("distance_m"),
                "start_track_s_m": segment.get("start_s_m"),
                "end_track_s_m": segment.get("end_s_m"),
                "corner_length_m": segment.get("length_m"),
                "peak_curvature_1pm": segment.get("peak_curvature_1pm") or (max(curvatures) if curvatures else None),
                "event_count": sum(
                    1
                    for event in events
                    if isinstance(event.get("time_sec"), (int, float))
                    and entry_time_abs <= float(event["time_sec"]) <= exit_time_abs
                ),
            }
        )
    return output


def _first_present_speed(rows: list[dict[str, float | int | None]]) -> float | None:
    for row in rows:
        if row.get("speed_mps") is not None:
            return float(row["speed_mps"])
    return None


def _last_present_speed(rows: list[dict[str, float | int | None]]) -> float | None:
    for row in reversed(rows):
        if row.get("speed_mps") is not None:
            return float(row["speed_mps"])
    return None


def _low_speed_segments(
    rows: list[dict[str, float | int | None]],
    thresholds: dict[str, float],
) -> tuple[float | None, int]:
    if len(rows) < 2:
        return None, 0
    low_time = 0.0
    current_segment = 0.0
    stuck_count = 0
    for prev, current in zip(rows, rows[1:]):
        prev_speed = prev.get("speed_mps")
        if prev_speed is None:
            continue
        dt = max(0.0, float(current["time_sec"]) - float(prev["time_sec"]))
        if prev_speed < thresholds["low_speed_mps"]:
            low_time += dt
            current_segment += dt
        else:
            if current_segment >= thresholds["stuck_duration_sec"]:
                stuck_count += 1
            current_segment = 0.0
    if current_segment >= thresholds["stuck_duration_sec"]:
        stuck_count += 1
    return low_time, stuck_count


def _low_speed_events(
    rows: list[dict[str, float | int | None]],
    thresholds: dict[str, float],
) -> list[dict[str, float | int | str | None]]:
    events: list[dict[str, float | int | str | None]] = []
    if len(rows) < 2:
        return events
    segment_start: dict[str, float | int | None] | None = None
    segment_duration = 0.0
    for prev, current in zip(rows, rows[1:]):
        speed = prev.get("speed_mps")
        if speed is None:
            continue
        dt = max(0.0, float(current["time_sec"]) - float(prev["time_sec"]))
        if speed < thresholds["low_speed_mps"]:
            if segment_start is None:
                segment_start = prev
            segment_duration += dt
        else:
            _append_low_speed_event(events, segment_start, segment_duration, thresholds)
            segment_start = None
            segment_duration = 0.0
    _append_low_speed_event(events, segment_start, segment_duration, thresholds)
    return events


def _append_low_speed_event(
    events: list[dict[str, float | int | str | None]],
    segment_start: dict[str, float | int | None] | None,
    duration_sec: float,
    thresholds: dict[str, float],
) -> None:
    if segment_start is None or duration_sec <= 0.0:
        return
    event_type = "stuck" if duration_sec >= thresholds["stuck_duration_sec"] else "low_speed"
    severity = "error" if event_type == "stuck" else "info"
    events.append(
        {
            "time_sec": float(segment_start["time_sec"]),
            "lap": None,
            "section": segment_start.get("section"),
            "event_type": event_type,
            "severity": severity,
            "description": f"speed below {thresholds['low_speed_mps']:.3f} m/s for {duration_sec:.3f} sec",
        }
    )


def _nearest_value(
    rows: list[dict[str, float | None]],
    time_sec: float,
    key: str,
    tolerance_sec: float,
) -> float | None:
    nearest: dict[str, float | None] | None = None
    nearest_dt = tolerance_sec
    for row in rows:
        value = row.get(key)
        if value is None:
            continue
        dt = abs(float(row["time_sec"]) - time_sec)
        if dt <= nearest_dt:
            nearest = row
            nearest_dt = dt
    return float(nearest[key]) if nearest is not None and nearest.get(key) is not None else None


def _oscillation_score(rows: list[tuple[float, float | int | None]]) -> float | None:
    valid = [(time_sec, float(value)) for time_sec, value in rows if value is not None]
    if len(valid) < 2:
        return None
    scores: list[float] = []
    for (prev_time, prev_value), (time_sec, value) in zip(valid, valid[1:]):
        dt = time_sec - prev_time
        if dt <= 1e-6:
            continue
        scores.append(abs(value - prev_value) / dt)
    return _mean(scores)


def _time_weighted_average(rows: list[tuple[float, float | int | None]]) -> float | None:
    valid = [(time_sec, float(value)) for time_sec, value in rows if value is not None]
    if not valid:
        return None
    if len(valid) == 1:
        return valid[0][1]
    total_weight = 0.0
    weighted = 0.0
    for (time_sec, value), (next_time, _next_value) in zip(valid, valid[1:]):
        dt = max(0.0, next_time - time_sec)
        weighted += value * dt
        total_weight += dt
    if total_weight <= 1e-9:
        return _mean([value for _time_sec, value in valid])
    return weighted / total_weight


def _values(rows: Any) -> list[float]:
    return [float(value) for _time_sec, value in rows if value is not None]


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _sorted_rows(rows: list[dict[str, float | None]]) -> list[dict[str, float | None]]:
    return sorted(rows, key=lambda row: float(row.get("time_sec") or 0.0))


def _sorted_debug_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    return sorted(rows, key=lambda row: float(row.get("time_sec") or 0.0))


def _first_float(msg: Any, candidates: tuple[str, ...]) -> float | None:
    for candidate in candidates:
        value = _get_nested_float(msg, candidate)
        if value is not None:
            return value
    return None


def _get_nested_float(obj: Any, path: str) -> float | None:
    current = obj
    for part in path.split("."):
        if not hasattr(current, part):
            return None
        current = getattr(current, part)
    if isinstance(current, bool):
        return None
    if isinstance(current, (int, float)):
        if math.isfinite(float(current)):
            return float(current)
    return None
