#!/usr/bin/env python3
import os
import time

import rclpy
import rclpy.node
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from v2x_msgs.msg import V2XVehiclePositionArray


VEHICLE_COLORS = {
    "d1": (0.2, 0.4, 1.0),
    "d2": (1.0, 0.9, 0.2),
    "d3": (0.2, 1.0, 0.2),
    "d4": (1.0, 0.2, 0.2),
}
DEFAULT_COLOR = (1.0, 1.0, 1.0)
OVERTAKE_MODE_COLORS = {
    "FOLLOW_BLOCKED": (1.0, 0.70, 0.10),
    "PREPARE_OVERTAKE_LEFT": (0.0, 0.90, 1.0),
    "PREPARE_OVERTAKE_RIGHT": (0.0, 0.90, 1.0),
    "OVERTAKE_LEFT": (0.0, 1.0, 0.25),
    "OVERTAKE_RIGHT": (0.0, 1.0, 0.25),
    "MERGE_BACK": (0.65, 0.35, 1.0),
    "YIELD_BEHIND": (1.0, 0.45, 0.0),
    "ABORT_RECOVERY": (1.0, 0.05, 0.05),
    "SAFE_STOP": (0.55, 0.0, 0.0),
    "SPEED_GUARD": (0.85, 0.95, 1.0),
}
SPHERE_DIAMETER = 1.5
ALPHA = 0.9
LIFETIME_SEC = 1


def vehicle_id_from_ros_domain_id() -> str | None:
    raw_domain_id = os.environ.get("ROS_DOMAIN_ID")
    if not raw_domain_id:
        return None
    try:
        domain_id = int(raw_domain_id)
    except ValueError:
        return None
    if domain_id <= 0:
        return None
    return f"d{domain_id}"


class V2XMarkerPublisherNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("v2x_marker_publisher")
        self.declare_parameter("own_vehicle_id", "auto")
        self.declare_parameter("overtake_mode_topic", "/debug/overtake/mode")
        self.declare_parameter("overtake_mode_stale_timeout_sec", 1.0)

        configured_vehicle_id = self.get_parameter(
            "own_vehicle_id").get_parameter_value().string_value
        self.own_vehicle_id = self._resolve_own_vehicle_id(configured_vehicle_id)
        self.overtake_mode_topic = self.get_parameter(
            "overtake_mode_topic").get_parameter_value().string_value
        self.overtake_mode_stale_timeout_sec = self.get_parameter(
            "overtake_mode_stale_timeout_sec").get_parameter_value().double_value
        self.overtake_mode = ""
        self.overtake_mode_received_at = 0.0

        self.sub = self.create_subscription(
            V2XVehiclePositionArray, "/v2x/vehicle_positions", self.callback, 1)
        self.mode_sub = self.create_subscription(
            String, self.overtake_mode_topic, self.mode_callback, 1)
        self.pub = self.create_publisher(
            MarkerArray, "/v2x/vehicle_positions/markers", 1)

        if self.own_vehicle_id:
            self.get_logger().info(
                f"V2X self marker color follows {self.overtake_mode_topic} "
                f"for own_vehicle_id={self.own_vehicle_id}")
        else:
            self.get_logger().warn(
                "own_vehicle_id could not be resolved; V2X markers will keep "
                "vehicle-id colors. Set own_vehicle_id or ROS_DOMAIN_ID to "
                "enable overtake state coloring.")

    def mode_callback(self, msg: String) -> None:
        self.overtake_mode = msg.data.strip()
        self.overtake_mode_received_at = time.monotonic()

    def callback(self, msg: V2XVehiclePositionArray) -> None:
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, vehicle in enumerate(msg.vehicles):
            markers.markers.append(self._build_marker(msg, vehicle, index))

        self.pub.publish(markers)

    def _build_marker(self, array_msg, vehicle, index: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = vehicle.header.frame_id or array_msg.header.frame_id or "map"
        marker.header.stamp = vehicle.header.stamp
        marker.ns = "v2x_vehicles"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = vehicle.position.x
        marker.pose.position.y = vehicle.position.y
        marker.pose.position.z = vehicle.position.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = SPHERE_DIAMETER
        marker.scale.y = SPHERE_DIAMETER
        marker.scale.z = SPHERE_DIAMETER
        r, g, b = self._color_for_vehicle(vehicle.vehicle_id)
        marker.color = ColorRGBA(r=r, g=g, b=b, a=ALPHA)
        marker.lifetime = Duration(sec=LIFETIME_SEC, nanosec=0)
        return marker

    def _resolve_own_vehicle_id(self, configured_vehicle_id: str) -> str | None:
        if configured_vehicle_id and configured_vehicle_id != "auto":
            return configured_vehicle_id
        return vehicle_id_from_ros_domain_id()

    def _color_for_vehicle(self, vehicle_id: str) -> tuple[float, float, float]:
        base_color = VEHICLE_COLORS.get(vehicle_id, DEFAULT_COLOR)
        if vehicle_id != self.own_vehicle_id:
            return base_color
        mode_color = self._fresh_overtake_mode_color()
        return mode_color if mode_color is not None else base_color

    def _fresh_overtake_mode_color(self) -> tuple[float, float, float] | None:
        if not self.overtake_mode:
            return None
        if self.overtake_mode_stale_timeout_sec > 0.0:
            age_sec = time.monotonic() - self.overtake_mode_received_at
            if age_sec > self.overtake_mode_stale_timeout_sec:
                return None
        return OVERTAKE_MODE_COLORS.get(self.overtake_mode)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(V2XMarkerPublisherNode())
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
