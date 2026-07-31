from __future__ import annotations

import time


def main() -> int:
    try:
        import rclpy
        from autoware_auto_planning_msgs.msg import Trajectory
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray
    except ImportError as error:
        raise SystemExit(f"ROS 2 Python environment is required: {error}")

    class ReadyProbe(Node):
        def __init__(self):
            super().__init__("ga_reset_ready_probe")
            self.status = False
            self.stopped_odom = False
            self.trajectory = False
            best_effort = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(Float32MultiArray, "/awsim/status", self.on_status, 1)
            self.create_subscription(
                Odometry, "/localization/kinematic_state", self.on_odom, best_effort
            )
            self.create_subscription(
                Trajectory,
                "/planning/scenario_planning/trajectory",
                self.on_trajectory,
                best_effort,
            )

        def on_status(self, message):
            self.status = len(message.data) >= 4

        def on_odom(self, message):
            self.stopped_odom = abs(float(message.twist.twist.linear.x)) <= 0.2

        def on_trajectory(self, message):
            self.trajectory = bool(message.points)

        @property
        def ready(self):
            return self.status and self.stopped_odom and self.trajectory

    rclpy.init()
    probe = ReadyProbe()
    deadline = time.monotonic() + 40.0
    try:
        while rclpy.ok() and time.monotonic() < deadline and not probe.ready:
            rclpy.spin_once(probe, timeout_sec=0.2)
        if not probe.ready:
            probe.get_logger().error(
                "reset readiness timeout: "
                f"status={probe.status} stopped_odom={probe.stopped_odom} "
                f"trajectory={probe.trajectory}"
            )
            return 1
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
