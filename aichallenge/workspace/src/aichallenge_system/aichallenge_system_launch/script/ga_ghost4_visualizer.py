#!/usr/bin/env python3
"""Aggregate four isolated GA vehicle domains into one RViz MarkerArray."""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import queue
import signal
import time
from collections import deque


ODOMETRY_TOPIC = "/localization/kinematic_state"
MARKER_TOPIC = "/ga/ghost4/markers"
CONTROLLER_NODE = "/simple_pure_pursuit_node"
DOMAINS = (1, 2, 3, 4)
COLORS = {
    1: (0.95, 0.20, 0.20),
    2: (0.20, 0.55, 1.00),
    3: (0.20, 0.90, 0.35),
    4: (1.00, 0.75, 0.10),
}


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
    stop = mp.Event()
    outputs = {domain: mp.Queue(maxsize=1) for domain in DOMAINS}
    collectors = [
        mp.Process(target=_collect_domain, args=(domain, outputs[domain], stop), daemon=True)
        for domain in DOMAINS
    ]
    for collector in collectors:
        collector.start()

    os.environ["ROS_DOMAIN_ID"] = "1"
    import rclpy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(domain_id=1)
    node = rclpy.create_node("ga_ghost4_visualizer")
    publisher = node.create_publisher(MarkerArray, MARKER_TOPIC, 1)
    latest = {}
    histories = {domain: deque(maxlen=300) for domain in DOMAINS}
    last_history_stamp = {domain: None for domain in DOMAINS}

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
        for domain in DOMAINS:
            state = latest.get(domain)
            if state is None:
                continue
            (
                _, stamp_sec, stamp_nanosec, x, y, z, qx, qy, qz, qw,
                speed, candidate_id, received_at,
            ) = state
            stamp_key = (stamp_sec, stamp_nanosec)
            history = histories[domain]
            if last_history_stamp[domain] != stamp_key:
                if history and math.hypot(x - history[-1].x, y - history[-1].y) > 15.0:
                    history.clear()
                point = Point(x=x, y=y, z=z + 0.15)
                if not history or math.hypot(x - history[-1].x, y - history[-1].y) >= 0.15:
                    history.append(point)
                last_history_stamp[domain] = stamp_key

            color = COLORS[domain]
            stale = now_wall - received_at > 1.0
            alpha = 0.25 if stale else 0.88
            base_id = domain * 10

            vehicle = Marker()
            vehicle.header.frame_id = "map"
            vehicle.header.stamp = now_ros
            vehicle.ns = "ga_ghost4_vehicle"
            vehicle.id = base_id
            vehicle.type = Marker.CUBE
            vehicle.action = Marker.ADD
            vehicle.pose.position.x = x
            vehicle.pose.position.y = y
            vehicle.pose.position.z = z + 0.32
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
            trail.ns = "ga_ghost4_trail"
            trail.id = base_id + 1
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = 0.18
            trail.color = _make_color(color, 0.75 if not stale else 0.25)
            trail.points = list(history)
            markers.markers.append(trail)

            label = Marker()
            label.header = vehicle.header
            label.ns = "ga_ghost4_label"
            label.id = base_id + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.65
            label.color = _make_color(color, 1.0 if not stale else 0.4)
            suffix = " [STALE]" if stale else ""
            label.text = f"D{domain}  {candidate_id}  {speed * 3.6:.1f} km/h{suffix}"
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
