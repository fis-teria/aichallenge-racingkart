#!/usr/bin/env python3
"""Aggregate every shared-AWSIM GA vehicle domain into one RViz MarkerArray."""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import queue
import signal
import time
from collections import deque
import json
from pathlib import Path


ODOMETRY_TOPIC = "/localization/kinematic_state"
MARKER_TOPIC = "/ga/ghost4/markers"
CONTROLLER_NODE = "/simple_pure_pursuit_node"
DEFAULT_POOL_CONFIG = "/output/ga-pool/experiment.json"
SLOT_COLORS = (
    (0.95, 0.20, 0.20),
    (0.20, 0.55, 1.00),
    (0.20, 0.90, 0.35),
    (1.00, 0.75, 0.10),
)


def _load_vehicle_specs(config_path: str) -> list[dict[str, object]]:
    """Return environment-aware vehicle specs, with legacy domains as fallback."""
    try:
        data = json.loads(Path(config_path).read_text(encoding="utf-8"))
        environments = data["evaluator"]["environments"]
        if not environments:
            raise ValueError("evaluator.environments is empty")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(
            f"[ga-rviz] cannot read pool config {config_path}: {error}; "
            "falling back to ROS domains 1-4",
            flush=True,
        )
        environments = [{"name": "env1", "vehicle_domain_ids": [1, 2, 3, 4]}]

    specs = []
    seen_domains = set()
    for environment_index, environment in enumerate(environments, start=1):
        environment_name = str(environment.get("name") or f"env{environment_index}")
        domains = environment.get("vehicle_domain_ids", [])
        for slot_index, value in enumerate(domains):
            domain_id = int(value)
            if domain_id in seen_domains:
                raise ValueError(f"duplicate vehicle ROS domain {domain_id}")
            seen_domains.add(domain_id)
            brightness = max(0.55, 1.0 - 0.22 * (environment_index - 1))
            base_color = SLOT_COLORS[slot_index % len(SLOT_COLORS)]
            specs.append(
                {
                    "domain_id": domain_id,
                    "environment_index": environment_index,
                    "environment_name": environment_name,
                    "slot_index": slot_index + 1,
                    "color": tuple(component * brightness for component in base_color),
                    "z_offset": 0.12 * (environment_index - 1),
                }
            )
    if not specs:
        raise ValueError("pool config contains no vehicle domains")
    return specs


def _replace_latest(output: mp.Queue, value: tuple) -> None:
    try:
        output.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        output.get_nowait()
    except queue.Empty:
        pass
    try:
        output.put_nowait(value)
    except queue.Full:
        pass


def _collect_domain(domain_id: int, output: mp.Queue, stop: mp.Event) -> None:
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters

    rclpy.init(domain_id=domain_id)
    node = rclpy.create_node(f"ga_ghost4_collector_d{domain_id}")
    candidate_id = "waiting"
    last_parameter_request = 0.0
    parameter_future = None
    client = node.create_client(GetParameters, f"{CONTROLLER_NODE}/get_parameters")

    def on_odometry(message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        stamp = message.header.stamp
        _replace_latest(
            output,
            (
                domain_id,
                stamp.sec,
                stamp.nanosec,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                candidate_id,
                time.monotonic(),
            ),
        )

    node.create_subscription(Odometry, ODOMETRY_TOPIC, on_odometry, 10)
    try:
        while rclpy.ok() and not stop.is_set():
            now = time.monotonic()
            if parameter_future is not None and parameter_future.done():
                try:
                    response = parameter_future.result()
                    if response.values:
                        candidate_id = response.values[0].string_value or "idle"
                except Exception:
                    candidate_id = "unavailable"
                parameter_future = None
            if (
                parameter_future is None
                and now - last_parameter_request >= 1.0
                and client.service_is_ready()
            ):
                request = GetParameters.Request()
                request.names = ["ga_candidate_id"]
                parameter_future = client.call_async(request)
                last_parameter_request = now
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _make_color(color, alpha):
    from std_msgs.msg import ColorRGBA

    return ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)


def main() -> None:
    mp.set_start_method("spawn")
    config_path = os.environ.get("GA_POOL_CONFIG", DEFAULT_POOL_CONFIG)
    vehicle_specs = _load_vehicle_specs(config_path)
    domains = tuple(int(spec["domain_id"]) for spec in vehicle_specs)
    specs_by_domain = {int(spec["domain_id"]): spec for spec in vehicle_specs}
    publisher_domain = domains[0]
    print(
        f"[ga-rviz] visualizing {len(domains)} vehicles from {config_path}: {domains}",
        flush=True,
    )
    stop = mp.Event()
    outputs = {domain: mp.Queue(maxsize=1) for domain in domains}
    collectors = [
        mp.Process(target=_collect_domain, args=(domain, outputs[domain], stop), daemon=True)
        for domain in domains
    ]
    for collector in collectors:
        collector.start()

    os.environ["ROS_DOMAIN_ID"] = str(publisher_domain)
    import rclpy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(domain_id=publisher_domain)
    node = rclpy.create_node("ga_pool_visualizer")
    publisher = node.create_publisher(MarkerArray, MARKER_TOPIC, 1)
    latest = {}
    histories = {domain: deque(maxlen=300) for domain in domains}
    last_history_stamp = {domain: None for domain in domains}

    def publish_markers() -> None:
        for domain, output in outputs.items():
            try:
                while True:
                    latest[domain] = output.get_nowait()
            except queue.Empty:
                pass

        markers = MarkerArray()
        now_ros = node.get_clock().now().to_msg()
        now_wall = time.monotonic()
        for global_index, domain in enumerate(domains, start=1):
            state = latest.get(domain)
            if state is None:
                continue
            spec = specs_by_domain[domain]
            z_offset = float(spec["z_offset"])
            (
                _, stamp_sec, stamp_nanosec, x, y, z, qx, qy, qz, qw,
                speed, candidate_id, received_at,
            ) = state
            stamp_key = (stamp_sec, stamp_nanosec)
            history = histories[domain]
            if last_history_stamp[domain] != stamp_key:
                if history and math.hypot(x - history[-1].x, y - history[-1].y) > 15.0:
                    history.clear()
                point = Point(x=x, y=y, z=z + 0.15 + z_offset)
                if not history or math.hypot(x - history[-1].x, y - history[-1].y) >= 0.15:
                    history.append(point)
                last_history_stamp[domain] = stamp_key

            color = spec["color"]
            environment_index = int(spec["environment_index"])
            environment_name = str(spec["environment_name"])
            stale = now_wall - received_at > 1.0
            alpha = 0.25 if stale else 0.88
            base_id = global_index * 10
            namespace = f"ga_pool_{environment_name}"

            vehicle = Marker()
            vehicle.header.frame_id = "map"
            vehicle.header.stamp = now_ros
            vehicle.ns = namespace
            vehicle.id = base_id
            vehicle.type = Marker.CUBE
            vehicle.action = Marker.ADD
            vehicle.pose.position.x = x
            vehicle.pose.position.y = y
            vehicle.pose.position.z = z + 0.32 + z_offset
            vehicle.pose.orientation.x = qx
            vehicle.pose.orientation.y = qy
            vehicle.pose.orientation.z = qz
            vehicle.pose.orientation.w = qw
            vehicle.scale.x = 2.0
            vehicle.scale.y = 1.05
            vehicle.scale.z = 0.55
            vehicle.color = _make_color(color, alpha)
            markers.markers.append(vehicle)

            trail = Marker()
            trail.header = vehicle.header
            trail.ns = namespace
            trail.id = base_id + 1
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = 0.18
            trail.color = _make_color(color, 0.75 if not stale else 0.25)
            trail.points = list(history)
            markers.markers.append(trail)

            label = Marker()
            label.header = vehicle.header
            label.ns = namespace
            label.id = base_id + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.6 + z_offset
            label.pose.orientation.w = 1.0
            label.scale.z = 0.65
            label.color = _make_color(color, 1.0 if not stale else 0.4)
            suffix = " [STALE]" if stale else ""
            label.text = (
                f"E{environment_index}-D{domain}  {candidate_id}  "
                f"{speed * 3.6:.1f} km/h{suffix}"
            )
            markers.markers.append(label)

        publisher.publish(markers)

    node.create_timer(0.1, publish_markers)

    def request_stop(*_args) -> None:
        stop.set()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        rclpy.spin(node)
    finally:
        stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for collector in collectors:
            collector.join(timeout=2.0)
            if collector.is_alive():
                collector.terminate()


if __name__ == "__main__":
    main()
