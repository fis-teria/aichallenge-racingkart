#!/usr/bin/env python3

import json
import yaml
from typing import List, Tuple, Optional, NamedTuple
import dataclasses
from scipy import sparse
from scipy.sparse import dia_matrix
import numpy as np
import copy
import os
import shutil
from datetime import datetime
from time import perf_counter

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, Bool, Float32MultiArray, Int32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose2D, Point, Vector3
from std_msgs.msg import ColorRGBA

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

# autoware
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros.v2x_vehicle_tracker import (
    V2XVehicleTracker,
    predictions_to_obstacles,
)

# Multi_Purpose_MPC
from multi_purpose_mpc_ros.core.map import Map, Obstacle
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path

# Project
from multi_purpose_mpc_ros.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros.obstacle_manager import ObstacleManager
from multi_purpose_mpc_ros.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros.speed_profile import (
    SpeedProfilePoint,
    apply_combined_speed_profile,
    combine_speed_profile,
)
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand, PathConstraints, BorderCells
from multi_purpose_mpc_ros.tools.reference_velocity_configulator import ReferenceVelocityConfigulator


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)
GRAVITY_MPS2 = 9.80665

def array_to_ackermann_control_command(stamp, u: np.ndarray, acc: float) -> AckermannControlCommand:
    msg = AckermannControlCommand()
    msg.stamp = stamp
    msg.lateral.stamp = stamp
    msg.lateral.steering_tire_angle = u[1]
    msg.lateral.steering_tire_rotation_rate = 2.0
    msg.longitudinal.stamp = stamp
    msg.longitudinal.speed = u[0]
    msg.longitudinal.acceleration = acc
    return msg

def yaw_from_quaternion(q: Quaternion):
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z
    sqw = q.w * q.w

    # Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw) # normalization added from urdfom_headers

    if sarg <= -0.99999:
        yaw = -2. * np.arctan2(q.y, q.x)
    elif sarg >= 0.99999:
        yaw = 2. * np.arctan2(q.y, q.x)
    else:
        yaw = np.arctan2(2. * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz)

    return yaw

def odom_to_pose_2d(odom: Odometry) -> Pose2D:
    pose = Pose2D()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = yaw_from_quaternion(odom.pose.pose.orientation)

    return pose


def cfg_bool(config, name: str, default: bool) -> bool:
    return bool(getattr(config, name, default))


def cfg_float(config, name: str, default: float) -> float:
    return float(getattr(config, name, default))


def cfg_str(config, name: str, default: str) -> str:
    return str(getattr(config, name, default))

@dataclasses.dataclass
class MPCConfig:
    N: int
    Q: dia_matrix
    R: dia_matrix
    QN: dia_matrix
    v_max: float
    a_min: float
    a_max: float
    ay_max: float
    delta_max: float
    steer_rate_max: float
    control_rate: float
    steering_tire_angle_gain_var: float
    accel_low_pass_gain: float
    steer_low_pass_gain: float
    wp_id_offset: int
    use_max_kappa_pred: bool
    lateral_target_mode: str
    wall_margin_m: float
    use_curvature_speed_profile: bool
    use_ref_vel_as_speed_cap: bool
    speed_profile_debug_publish_period_sec: float
    use_grade_accel_feedforward: bool
    grade_ff_gain: float
    grade_ff_max_accel_mps2: float
    grade_ff_window_m: float
    grade_ff_min_distance_delta_m: float
    grade_ff_min_speed_mps: float
    grade_ff_low_pass_gain: float
    grade_ff_only_when_not_decelerating: bool
    grade_ff_speed_error_deadband_mps: float


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h
    BUG_ACC = 400.0

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 100.0

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str]) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        self.declare_parameter("use_obstacle_avoidance", False)
        self.declare_parameter("use_stats", False)

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.USE_OBSTACLE_AVOIDANCE = self.get_parameter("use_obstacle_avoidance").get_parameter_value().bool_value
        self.use_stats = self.get_parameter("use_stats").get_parameter_value().bool_value

        self._config_path = config_path
        self._ref_vel_config_path: Optional[str] = ref_vel_config_path
        self._cfg = self._load_config()
        self._odom: Optional[Odometry] = None
        self._enable_control = True
        self._initialize()
        self._setup_parameters_callback()
        self._setup_pub_sub()

        if self.use_sim_time:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("use_sim_time is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_BUG_ACC:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_BUG_ACC is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_OBSTACLE_AVOIDANCE:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_OBSTACLE_AVOIDANCE is enabled!")
            self.get_logger().warn("------------------------------------")

    def _load_config(self) -> NamedTuple:

        # logging content
        with open(self._config_path, "r") as f:
            config_content = f.read()
            self.get_logger().info(
                "\n" +
                "----- config.yaml -----\n"+
                config_content + "\n" +
                "-----------------------")

        if self._ref_vel_config_path is not None:
            with open(self._ref_vel_config_path, "r") as f:
                ref_vel_config_content = f.read()
                self.get_logger().info(
                    "\n" +
                    "----- ref_vel.yaml -----\n"+
                    ref_vel_config_content + "\n" +
                    "-----------------------")

        with open(self._config_path, "r") as f:
            cfg: NamedTuple = convert_to_namedtuple(yaml.safe_load(f)) # type: ignore

        # Check if the files exist
        mandatory_files = [cfg.map.yaml_path, cfg.waypoints.csv_path] # type: ignore
        for file_path in mandatory_files:
            file_exists(self.in_pkg_share(file_path))
        return cfg

    def _create_reference_path_from_autoware_trajectory(self, trajectory: Trajectory) -> Optional[ReferencePath]:
        wp_x = [0] * len(trajectory.points)
        wp_y = [0] * len(trajectory.points)
        for i, p in enumerate(trajectory.points):
            wp_x[i] = p.pose.position.x
            wp_y[i] = p.pose.position.y

        cfg_ref_path = self._cfg.reference_path # type: ignore
        reference_path = ReferencePath(
            self._map,
            wp_x,
            wp_y,
            cfg_ref_path.resolution,
            cfg_ref_path.smoothing_distance,
            cfg_ref_path.max_width,
            cfg_ref_path.circular)

        mpc_config = self._mpc_cfg
        speed_profile_constraints = {
            "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
            "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}

        if not reference_path.compute_speed_profile(speed_profile_constraints):
            return None

        return reference_path

    def _setup_parameters_callback(self) -> None:
        def declatre_parameters():
            cfg_mpc = self._cfg.mpc
            self.declare_parameter("v_max", cfg_mpc.v_max)
            self.declare_parameter("steering_tire_angle_gain_var", cfg_mpc.steering_tire_angle_gain_var)
            self.declare_parameter("Q0", cfg_mpc.Q[0])
            self.declare_parameter("Q1", cfg_mpc.Q[1])
            self.declare_parameter("Q2", cfg_mpc.Q[2])
            self.declare_parameter("R0", cfg_mpc.R[0])
            self.declare_parameter("R1", cfg_mpc.R[1])
            self.declare_parameter("QN0", cfg_mpc.QN[0])
            self.declare_parameter("QN1", cfg_mpc.QN[1])
            self.declare_parameter("QN2", cfg_mpc.QN[2])

            mpc_cfg = self._mpc_cfg
            self.declare_parameter("ay_max", mpc_cfg.ay_max)
            self.declare_parameter("accel_low_pass_gain", mpc_cfg.accel_low_pass_gain)
            self.declare_parameter("steer_low_pass_gain", mpc_cfg.steer_low_pass_gain)
            self.declare_parameter("wp_id_offset", mpc_cfg.wp_id_offset)
            self.declare_parameter("lateral_target_mode", mpc_cfg.lateral_target_mode)
            self.declare_parameter("wall_margin_m", mpc_cfg.wall_margin_m)
            self.declare_parameter("use_curvature_speed_profile", mpc_cfg.use_curvature_speed_profile)
            self.declare_parameter("use_ref_vel_as_speed_cap", mpc_cfg.use_ref_vel_as_speed_cap)
            self.declare_parameter(
                "speed_profile_debug_publish_period_sec",
                mpc_cfg.speed_profile_debug_publish_period_sec)
            self.declare_parameter("use_grade_accel_feedforward", mpc_cfg.use_grade_accel_feedforward)
            self.declare_parameter("grade_ff_gain", mpc_cfg.grade_ff_gain)
            self.declare_parameter("grade_ff_max_accel_mps2", mpc_cfg.grade_ff_max_accel_mps2)
            self.declare_parameter("grade_ff_window_m", mpc_cfg.grade_ff_window_m)
            self.declare_parameter("grade_ff_min_distance_delta_m", mpc_cfg.grade_ff_min_distance_delta_m)
            self.declare_parameter("grade_ff_min_speed_mps", mpc_cfg.grade_ff_min_speed_mps)
            self.declare_parameter("grade_ff_low_pass_gain", mpc_cfg.grade_ff_low_pass_gain)
            self.declare_parameter("grade_ff_only_when_not_decelerating", mpc_cfg.grade_ff_only_when_not_decelerating)
            self.declare_parameter("grade_ff_speed_error_deadband_mps", mpc_cfg.grade_ff_speed_error_deadband_mps)

        def param_cb(parameters):
            cfg_mpc = self._cfg.mpc # type: ignore
            mpc_cfg = self._mpc_cfg

            def update_Q(index: int, value: float):
                cfg_mpc.Q[index] = value
                mpc_cfg.Q = sparse.diags(cfg_mpc.Q)
                self._mpc.update_Q(mpc_cfg.Q)
                self.get_logger().warn(f"Q[{index}] was updated to '{value}'")

            def update_R(index: int, value: float):
                cfg_mpc.R[index] = value
                mpc_cfg.R = sparse.diags(cfg_mpc.R)
                self._mpc.update_R(mpc_cfg.R)
                self.get_logger().warn(f"R[{index}] was updated to '{value}'")

            def update_QN(index: int, value: float):
                cfg_mpc.QN[index] = value
                mpc_cfg.QN = sparse.diags(cfg_mpc.QN)
                self._mpc.update_QN(mpc_cfg.QN)
                self.get_logger().warn(f"QN[{index}] was updated to '{value}'")

            for param in parameters:
                if param.name == "v_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.v_max = kmh_to_m_per_sec(param.value)
                    self._mpc.update_v_max(mpc_cfg.v_max)
                    self._recompute_curvature_speed_profile()
                    self._apply_speed_profile(self._mpc.model.wp_id)
                    self.get_logger().warn(f"v_max was updated to '{param.value}' [km/h]")

                elif param.name == "steering_tire_angle_gain_var" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steering_tire_angle_gain_var = param.value
                    self.get_logger().warn(f"steering_tire_angle_gain_var was updated to '{param.value}'")

                elif param.name == "Q0" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(0, param.value)
                elif param.name == "Q1" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(1, param.value)
                elif param.name == "Q2" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(2, param.value)


                elif param.name == "R0" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(0, param.value)
                elif param.name == "R1" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(1, param.value)

                elif param.name == "QN0" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(0, param.value)
                elif param.name == "QN1" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(1, param.value)
                elif param.name == "QN2" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(2, param.value)

                elif param.name == "ay_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.ay_max = param.value
                    self._mpc.update_ay_max(param.value)
                    self._recompute_curvature_speed_profile()
                    self._apply_speed_profile(self._mpc.model.wp_id)
                    self.get_logger().warn(f"ay_max was updated to '{param.value}'")

                elif param.name == "accel_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.accel_low_pass_gain = param.value
                    self.get_logger().warn(f"accel_low_pass_gain was updated to '{param.value}'")

                elif param.name == "steer_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steer_low_pass_gain = param.value
                    self.get_logger().warn(f"steer_low_pass_gain was updated to '{param.value}'")

                elif param.name == "wp_id_offset" and param.type_ == Parameter.Type.INTEGER:
                    mpc_cfg.wp_id_offset = param.value
                    self._mpc.update_wp_id_offset(param.value)
                    self.get_logger().warn(f"wp_id_offset was updated to '{param.value}'")

                elif param.name == "lateral_target_mode" and param.type_ == Parameter.Type.STRING:
                    mpc_cfg.lateral_target_mode = param.value
                    self._mpc.update_lateral_target_mode(param.value)
                    self.get_logger().warn(f"lateral_target_mode was updated to '{param.value}'")

                elif param.name == "wall_margin_m" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.wall_margin_m = param.value
                    self._mpc.update_wall_margin_m(param.value)
                    self.get_logger().warn(f"wall_margin_m was updated to '{param.value}'")

                elif param.name == "use_curvature_speed_profile" and param.type_ == Parameter.Type.BOOL:
                    mpc_cfg.use_curvature_speed_profile = param.value
                    self._recompute_curvature_speed_profile()
                    self._apply_speed_profile(self._mpc.model.wp_id)
                    self.get_logger().warn(f"use_curvature_speed_profile was updated to '{param.value}'")

                elif param.name == "use_ref_vel_as_speed_cap" and param.type_ == Parameter.Type.BOOL:
                    mpc_cfg.use_ref_vel_as_speed_cap = param.value
                    self._apply_speed_profile(self._mpc.model.wp_id)
                    self.get_logger().warn(f"use_ref_vel_as_speed_cap was updated to '{param.value}'")

                elif param.name == "speed_profile_debug_publish_period_sec" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.speed_profile_debug_publish_period_sec = param.value
                    self.get_logger().warn(
                        f"speed_profile_debug_publish_period_sec was updated to '{param.value}'")

                elif param.name == "use_grade_accel_feedforward" and param.type_ == Parameter.Type.BOOL:
                    mpc_cfg.use_grade_accel_feedforward = param.value
                    self.get_logger().warn(f"use_grade_accel_feedforward was updated to '{param.value}'")

                elif param.name == "grade_ff_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_gain = param.value
                    self.get_logger().warn(f"grade_ff_gain was updated to '{param.value}'")

                elif param.name == "grade_ff_max_accel_mps2" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_max_accel_mps2 = param.value
                    self.get_logger().warn(f"grade_ff_max_accel_mps2 was updated to '{param.value}'")

                elif param.name == "grade_ff_window_m" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_window_m = param.value
                    self.get_logger().warn(f"grade_ff_window_m was updated to '{param.value}'")

                elif param.name == "grade_ff_min_distance_delta_m" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_min_distance_delta_m = param.value
                    self.get_logger().warn(f"grade_ff_min_distance_delta_m was updated to '{param.value}'")

                elif param.name == "grade_ff_min_speed_mps" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_min_speed_mps = param.value
                    self.get_logger().warn(f"grade_ff_min_speed_mps was updated to '{param.value}'")

                elif param.name == "grade_ff_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_low_pass_gain = param.value
                    self.get_logger().warn(f"grade_ff_low_pass_gain was updated to '{param.value}'")

                elif param.name == "grade_ff_only_when_not_decelerating" and param.type_ == Parameter.Type.BOOL:
                    mpc_cfg.grade_ff_only_when_not_decelerating = param.value
                    self.get_logger().warn(f"grade_ff_only_when_not_decelerating was updated to '{param.value}'")

                elif param.name == "grade_ff_speed_error_deadband_mps" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.grade_ff_speed_error_deadband_mps = param.value
                    self.get_logger().warn(f"grade_ff_speed_error_deadband_mps was updated to '{param.value}'")

            return SetParametersResult(successful=True)

        declatre_parameters()
        self.add_on_set_parameters_callback(param_cb)

    def _initialize(self) -> None:
        def create_map() -> Map:
            return Map(self.in_pkg_share(self._cfg.map.yaml_path)) # type: ignore

        def create_ref_path(map: Map) -> ReferencePath:
            cfg_ref_path = self._cfg.reference_path # type: ignore

            is_ref_path_given = cfg_ref_path.csv_path != "" # type: ignore
            if is_ref_path_given:
                print("Using given reference path")
                wp_x, wp_y, _, _ = load_ref_path(self.in_pkg_share(self._cfg.reference_path.csv_path)) # type: ignore
                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)

            else:
                print("Using waypoints to create reference path")
                wp_x, wp_y = load_waypoints(self.in_pkg_share(self._cfg.waypoints.csv_path)) # type: ignore

                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)


        def create_obstacles() -> List[Obstacle]:
            use_csv_obstacles = self._cfg.obstacles.csv_path != "" # type: ignore
            if use_csv_obstacles:
                obstacles_file_path = self.in_pkg_share(self._cfg.obstacles.csv_path) # type: ignore
                obs_x, obs_y = load_waypoints(obstacles_file_path)
                obstacles = []
                for cx, cy in zip(obs_x, obs_y):
                    obstacles.append(Obstacle(cx=cx, cy=cy, radius=self._cfg.obstacles.radius)) # type: ignore
                self._obstacle_manager = ObstacleManager(self._map, obstacles)
                return obstacles
            else:
                return []

        def create_car(ref_path: ReferencePath) -> BicycleModel:
            cfg_model = self._cfg.bicycle_model # type: ignore
            return BicycleModel(
                ref_path,
                cfg_model.length,
                cfg_model.width,
                1.0 / self._cfg.mpc.control_rate) # type: ignore

        def create_mpc(car: BicycleModel) -> Tuple[MPCConfig, MPC]:
            cfg_mpc = self._cfg.mpc # type: ignore

            mpc_cfg = MPCConfig(
                cfg_mpc.N,
                sparse.diags(cfg_mpc.Q),
                sparse.diags(cfg_mpc.R),
                sparse.diags(cfg_mpc.QN),
                kmh_to_m_per_sec(self.BUG_VEL if self.USE_BUG_ACC else cfg_mpc.v_max),
                cfg_mpc.a_min,
                cfg_mpc.a_max,
                cfg_mpc.ay_max,
                np.deg2rad(cfg_mpc.delta_max_deg),
                cfg_mpc.steer_rate_max,
                cfg_mpc.control_rate,
                cfg_mpc.steering_tire_angle_gain_var,
                cfg_mpc.accel_low_pass_gain,
                cfg_mpc.steer_low_pass_gain,
                cfg_mpc.wp_id_offset,
                cfg_mpc.use_max_kappa_pred,
                cfg_str(cfg_mpc, "lateral_target_mode", "center_of_corridor"),
                cfg_float(cfg_mpc, "wall_margin_m", 0.0),
                cfg_bool(cfg_mpc, "use_curvature_speed_profile", True),
                cfg_bool(cfg_mpc, "use_ref_vel_as_speed_cap", True),
                cfg_float(cfg_mpc, "speed_profile_debug_publish_period_sec", 0.25),
                cfg_bool(cfg_mpc, "use_grade_accel_feedforward", False),
                cfg_float(cfg_mpc, "grade_ff_gain", 1.0),
                cfg_float(cfg_mpc, "grade_ff_max_accel_mps2", 0.35),
                cfg_float(cfg_mpc, "grade_ff_window_m", 2.0),
                cfg_float(cfg_mpc, "grade_ff_min_distance_delta_m", 0.5),
                cfg_float(cfg_mpc, "grade_ff_min_speed_mps", 0.3),
                cfg_float(cfg_mpc, "grade_ff_low_pass_gain", 0.25),
                cfg_bool(cfg_mpc, "grade_ff_only_when_not_decelerating", True),
                cfg_float(cfg_mpc, "grade_ff_speed_error_deadband_mps", 0.1))

            state_constraints = {
                "xmin": np.array([-np.inf, -np.inf, -np.inf]),
                "xmax": np.array([np.inf, np.inf, np.inf])}
            input_constraints = {
                "umin": np.array([0.0, -np.tan(mpc_cfg.delta_max) / car.length]),
                "umax": np.array([mpc_cfg.v_max, np.tan(mpc_cfg.delta_max) / car.length])}

            # mpcからのsteer指令出力は、gainを掛けて出力され、その状態で車体のsteer rate limit が適用されるため、
            # mpcの制御計算におけるsteer_rate_maxは、実際のsteer_rate_maxをgainで除した値で設定する
            scaled_steer_rate_max = mpc_cfg.steer_rate_max / mpc_cfg.steering_tire_angle_gain_var

            mpc = MPC(
                car,
                mpc_cfg.N,
                mpc_cfg.Q,
                mpc_cfg.R,
                mpc_cfg.QN,
                state_constraints,
                input_constraints,
                mpc_cfg.ay_max,
                scaled_steer_rate_max,
                mpc_cfg.wp_id_offset,
                self.USE_OBSTACLE_AVOIDANCE,
                self._cfg.reference_path.use_path_constraints_topic,
                mpc_cfg.use_max_kappa_pred,
                mpc_cfg.lateral_target_mode,
                mpc_cfg.wall_margin_m)

            return mpc_cfg, mpc

        def compute_speed_profile(car: BicycleModel, mpc_config: MPCConfig) -> None:
            speed_profile_constraints = {
                "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
                "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}
            car.reference_path.compute_speed_profile(speed_profile_constraints)

        def create_ref_vel_configulator() -> Optional[ReferenceVelocityConfigulator]:
            if self._ref_vel_config_path is None:
                return None
            return ReferenceVelocityConfigulator(self, self._config_path, self._ref_vel_config_path)

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg, self._mpc = create_mpc(self._car)
        self._curvature_speed_profile_mps: List[float] = []
        self._combined_speed_profile: List[SpeedProfilePoint] = []
        self._last_speed_profile_debug_publish_sec = -1.0e9
        self._last_mpc_solve_time_ms = 0.0
        self._last_mpc_status = "unknown"
        self._last_mpc_infeasible_count = 0
        self._reset_grade_estimator()
        compute_speed_profile(self._car, self._mpc_cfg)
        self._curvature_speed_profile_mps = self._read_reference_speed_profile()

        self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()
        self._apply_speed_profile(0)

        self._trajectory: Optional[Trajectory] = None
        self._path_constraints = None
        self._last_overtake_override_sec: Optional[float] = None
        self._overtake_override_timeout_sec = 0.50

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = bool(self._static_obstacles)
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)  # type: ignore
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(mpc_N)
            ]
            # コリドー外の V2X 障害物で MPC のコリドー狭窄/反転が起きないよう、
            # ref-path 近傍のみに絞り込む。閾値 = max_width/2 + vehicle_radius + 余白。
            ref_max_width = float(self._cfg.reference_path.max_width)  # type: ignore
            self._v2x_corridor_threshold_sq = (
                ref_max_width / 2.0 + self._v2x_vehicle_radius + 0.5
            ) ** 2
            wps = self._reference_path.waypoints
            self._waypoint_xy = np.asarray(
                [(wp.x, wp.y) for wp in wps], dtype=np.float64)

        # Laps
        self._current_laps = 1
        self._last_lap_time = 0.0
        self._lap_times = [None] * (self.MAX_LAPS + 1) # +1 means include lap 0

        # condition
        self._last_condition = None
        self._last_colliding_time = None

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

        # save config
        if self._cfg.common.save_config:
            self._save_config()

    def _read_reference_speed_profile(self) -> List[float]:
        fallback = self._mpc_cfg.v_max
        if not self._mpc_cfg.use_curvature_speed_profile:
            return [fallback] * len(self._reference_path.waypoints)
        return [
            fallback if wp.v_ref is None else float(wp.v_ref)
            for wp in self._reference_path.waypoints
        ]

    def _recompute_curvature_speed_profile(self) -> None:
        if self._mpc_cfg.use_curvature_speed_profile:
            speed_profile_constraints = {
                "a_min": self._mpc_cfg.a_min,
                "a_max": self._mpc_cfg.a_max,
                "v_min": 0.0,
                "v_max": self._mpc_cfg.v_max,
                "ay_max": self._mpc_cfg.ay_max,
            }
            self._reference_path.compute_speed_profile(speed_profile_constraints)
        self._curvature_speed_profile_mps = self._read_reference_speed_profile()

    def _section_speed_cap_mps(self, wp_id: int) -> Optional[float]:
        if not self._mpc_cfg.use_ref_vel_as_speed_cap or self._ref_vel_configulator is None:
            return None
        try:
            return kmh_to_m_per_sec(self._ref_vel_configulator.get_ref_vel(wp_id))
        except ValueError:
            return None

    def _apply_speed_profile(self, current_wp_id: int) -> None:
        self._combined_speed_profile = combine_speed_profile(
            self._curvature_speed_profile_mps,
            self._mpc_cfg.v_max,
            self._section_speed_cap_mps,
        )
        apply_combined_speed_profile(self._reference_path, self._combined_speed_profile)
        current_point = self._combined_speed_profile[
            current_wp_id % len(self._combined_speed_profile)
        ] if self._combined_speed_profile else None
        if current_point is not None:
            self._mpc.update_v_max(current_point.global_cap_mps)

    def _publish_speed_profile_debug(self, now, current_wp_id: int, actual_speed_mps: float, command_speed_mps: float) -> None:
        period = self._mpc_cfg.speed_profile_debug_publish_period_sec
        if period <= 0.0:
            return
        t = now.nanoseconds / 1e9
        if t - self._last_speed_profile_debug_publish_sec < period:
            return
        self._last_speed_profile_debug_publish_sec = t
        if not self._combined_speed_profile:
            return

        point = self._combined_speed_profile[current_wp_id % len(self._combined_speed_profile)]
        msg = String()
        msg.data = json.dumps(
            {
                "wp_id": point.wp_id,
                "source": point.source,
                "target_speed_mps": point.target_speed_mps,
                "curvature_speed_mps": point.curvature_speed_mps,
                "section_cap_mps": point.section_cap_mps,
                "global_cap_mps": point.global_cap_mps,
                "actual_speed_mps": actual_speed_mps,
                "command_speed_mps": command_speed_mps,
                "use_curvature_speed_profile": self._mpc_cfg.use_curvature_speed_profile,
                "use_ref_vel_as_speed_cap": self._mpc_cfg.use_ref_vel_as_speed_cap,
                "lateral_target_mode": self._mpc_cfg.lateral_target_mode,
                "wall_margin_m": self._mpc_cfg.wall_margin_m,
                "use_grade_accel_feedforward": self._mpc_cfg.use_grade_accel_feedforward,
                "grade_percent": self._last_grade_percent,
                "grade_accel_base_mps2": self._last_grade_accel_base_mps2,
                "grade_accel_ff_mps2": self._last_grade_accel_ff_mps2,
                "mpc_status": self._last_mpc_status,
                "mpc_solve_time_ms": self._last_mpc_solve_time_ms,
                "mpc_infeasible_count": self._last_mpc_infeasible_count,
                "overtake_mode_id": getattr(self._mpc, "overtake_mode_id", 0),
                "overtake_override_active": bool(getattr(self._mpc, "overtake_lateral_offsets", None) is not None),
            },
            separators=(",", ":"),
        )
        self._speed_profile_debug_pub.publish(msg)

    def _reset_grade_estimator(self) -> None:
        self._grade_samples: List[Tuple[float, float, float, float]] = []
        self._grade_distance_m = 0.0
        self._grade_last_pose_xyz: Optional[Tuple[float, float, float]] = None
        self._filtered_grade_fraction = 0.0
        self._last_grade_percent = 0.0
        self._last_grade_accel_base_mps2 = 0.0
        self._last_grade_accel_ff_mps2 = 0.0

    def _update_grade_estimate(self, odom: Odometry, speed_mps: float) -> float:
        pos = odom.pose.pose.position
        x = float(pos.x)
        y = float(pos.y)
        z = float(pos.z)
        if not np.isfinite([x, y, z]).all():
            return self._filtered_grade_fraction

        if self._grade_last_pose_xyz is None:
            self._grade_last_pose_xyz = (x, y, z)
            self._grade_samples = [(self._grade_distance_m, x, y, z)]
            return self._filtered_grade_fraction

        last_x, last_y, _ = self._grade_last_pose_xyz
        distance_delta = float(np.hypot(x - last_x, y - last_y))
        min_step_m = max(0.02, self._mpc_cfg.grade_ff_min_distance_delta_m * 0.05)
        if distance_delta >= min_step_m:
            self._grade_distance_m += distance_delta
            self._grade_last_pose_xyz = (x, y, z)
            self._grade_samples.append((self._grade_distance_m, x, y, z))
            self._trim_grade_samples()

        if abs(speed_mps) < self._mpc_cfg.grade_ff_min_speed_mps:
            return self._filtered_grade_fraction

        raw_grade = self._estimate_raw_grade_fraction()
        if raw_grade is None:
            return self._filtered_grade_fraction

        gain = float(np.clip(self._mpc_cfg.grade_ff_low_pass_gain, 0.0, 1.0))
        self._filtered_grade_fraction += (raw_grade - self._filtered_grade_fraction) * gain
        self._last_grade_percent = self._filtered_grade_fraction * 100.0
        return self._filtered_grade_fraction

    def _trim_grade_samples(self) -> None:
        if len(self._grade_samples) <= 2:
            return
        history_m = max(
            self._mpc_cfg.grade_ff_window_m * 4.0,
            self._mpc_cfg.grade_ff_min_distance_delta_m * 4.0,
            5.0)
        current_distance = self._grade_samples[-1][0]
        while len(self._grade_samples) > 2 and current_distance - self._grade_samples[0][0] > history_m:
            self._grade_samples.pop(0)

    def _estimate_raw_grade_fraction(self) -> Optional[float]:
        if len(self._grade_samples) < 2:
            return None

        current_distance, _, _, current_z = self._grade_samples[-1]
        target_window_m = max(
            self._mpc_cfg.grade_ff_window_m,
            self._mpc_cfg.grade_ff_min_distance_delta_m)
        ref_sample = self._grade_samples[0]
        for sample in reversed(self._grade_samples[:-1]):
            if current_distance - sample[0] >= target_window_m:
                ref_sample = sample
                break

        ref_distance, _, _, ref_z = ref_sample
        distance_delta = current_distance - ref_distance
        if distance_delta < self._mpc_cfg.grade_ff_min_distance_delta_m:
            return None

        grade_fraction = (current_z - ref_z) / distance_delta
        if not np.isfinite(grade_fraction):
            return None
        return float(grade_fraction)

    def _compute_grade_accel_ff(self, target_speed_mps: float, actual_speed_mps: float) -> float:
        self._last_grade_accel_ff_mps2 = 0.0
        if not self._mpc_cfg.use_grade_accel_feedforward:
            return 0.0

        speed_error = target_speed_mps - actual_speed_mps
        if (
            self._mpc_cfg.grade_ff_only_when_not_decelerating
            and speed_error < -self._mpc_cfg.grade_ff_speed_error_deadband_mps
        ):
            return 0.0

        uphill_grade = max(0.0, self._filtered_grade_fraction)
        ff_acc = GRAVITY_MPS2 * uphill_grade * self._mpc_cfg.grade_ff_gain
        ff_acc = float(np.clip(ff_acc, 0.0, self._mpc_cfg.grade_ff_max_accel_mps2))
        self._last_grade_accel_ff_mps2 = ff_acc
        return ff_acc

    def _save_config(self) -> None:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst_dir = self.PKG_PATH + f"log/{now}"
        os.makedirs(dst_dir, exist_ok=True)
        shutil.copy(self._config_path, os.path.join(dst_dir, "config.yaml"))

    def _setup_pub_sub(self) -> None:
        # Publishers
        if self.USE_BUG_ACC:
          self._command_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command", 1)
        else:
          self._command_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd_raw", 1)
          print("use normal ackermann control command")

        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._mpc_pred_pub = self.create_publisher(
            MarkerArray, "/mpc/prediction", 1)
        self._mpc_pred_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall", 1)

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._ref_path_pub = self.create_publisher(
            MarkerArray, "/mpc/ref_path", latching_qos)
        self._ref_path_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound", latching_qos)
        self._speed_profile_debug_pub = self.create_publisher(
            String, "/mpc/speed_profile_debug", 1)

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)
        self._control_mode_request_sub = self.create_subscription(
            Bool, "control/control_mode_request_topic", self._control_mode_request_callback, 1)
        # simple_trajectory_generator publishes with BEST_EFFORT/KEEP_LAST(1) — match it
        # so the subscription is QoS-compatible (rclpy default is RELIABLE).
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._trajectory_sub = self.create_subscription(
            Trajectory, "planning/scenario_planning/trajectory", self._trajectory_callback, trajectory_qos)
        self._stop_request_sub = self.create_subscription(
            Empty, "/control/mpc/stop_request", self._stop_request_callback, 1)
        self._overtake_override_sub = self.create_subscription(
            Float32MultiArray, "/overtake/reference_override", self._overtake_override_callback, 1)

        if self.use_sim_time:
            self._awsim_status_sub = self.create_subscription(
                Float32MultiArray, "/awsim/status", self._awsim_status_callback, 1)
            self._condition_sub = self.create_subscription(
                Int32, "/aichallenge/pitstop/condition", self._condition_callback, 1)

        if self.USE_OBSTACLE_AVOIDANCE:
            if self._cfg.reference_path.use_path_constraints_topic: # type: ignore
                self._path_constraints_sub = self.create_subscription(
                    PathConstraints, "/path_constraints_provider/path_constraints", self._path_constraints_callback, 1)

            if self._cfg.reference_path.use_border_cells_topic: # type: ignore
                self._border_cells_sub = self.create_subscription(
                    BorderCells, "/path_constraints_provider/border_cells", self._border_cells_callback, 1)

            self._v2x_sub = self.create_subscription(
                V2XVehiclePositionArray,
                "/v2x/vehicle_positions",
                self._v2x_callback,
                1)

    def _create_ackerman_control_command(self, stamp, u, acc, bug_acc_enabled):
        v_cmd = u[0]
        steer_cmd = u[1]

        ackerman_cmd = array_to_ackermann_control_command(stamp.to_msg(), [v_cmd, steer_cmd], acc)

        if not self.USE_BUG_ACC:
            return ackerman_cmd

        ackerman_boost_cmd = AckermannControlBoostCommand()
        ackerman_boost_cmd.command = ackerman_cmd
        ackerman_boost_cmd.boost_mode = bug_acc_enabled
        return ackerman_boost_cmd

    def _publish_control_command(self, stamp, u, acc, bug_acc_enabled):
        cmd = self._create_ackerman_control_command(stamp, u, acc, bug_acc_enabled)

        # publish raw control command
        self._command_raw_pub.publish(cmd)

        # compensate steering angle for the real vehicle
        # AWSIMにおいても後段のactuation_cmd_converter でgainを考慮した指令を生成するため、実機/sim問わず
        # gain を掛ける
        cmd.lateral.steering_tire_angle *= self._mpc_cfg.steering_tire_angle_gain_var
        self._command_pub.publish(cmd)


    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _control_mode_request_callback(self, msg):
        if msg.data and not self._enable_control:
            self.get_logger().info("Control mode request received")
            self._enable_control = True

    def _path_constraints_callback(self, msg: PathConstraints):
        self._reference_path.set_path_constraints(
            msg.upper_bounds, msg.lower_bounds, msg.rows, msg.cols)

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x_tracker.update(msg)
        predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
        self._dynamic_obstacles = predictions_to_obstacles(
            predictions, self._v2x_vehicle_radius)
        self._obstacles_updated = True

    def _filter_obstacles_to_corridor(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        if not obstacles or self._waypoint_xy.size == 0:
            return obstacles
        thr_sq = self._v2x_corridor_threshold_sq
        wps = self._waypoint_xy
        kept: List[Obstacle] = []
        for ob in obstacles:
            dxy = wps - np.array([ob.cx, ob.cy], dtype=np.float64)
            if np.min(np.einsum('ij,ij->i', dxy, dxy)) <= thr_sq:
                kept.append(ob)
        return kept

    def _border_cells_callback(self, msg: BorderCells):
        self._reference_path.set_border_cells(
            msg.dynamic_upper_bounds, msg.dynamic_lower_bounds, msg.rows, msg.cols)

    def _trajectory_callback(self, msg):
        self._trajectory = msg

    def _overtake_override_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 3 or int(data[0]) != 1:
            self._mpc.clear_overtake_reference_override()
            self._last_overtake_override_sec = None
            return

        mode_id = int(data[1])
        n = int(data[2])
        expected = 3 + 2 * n
        if n <= 0 or mode_id == 0:
            self._mpc.clear_overtake_reference_override()
            self._last_overtake_override_sec = self.get_clock().now().nanoseconds / 1e9
            return
        if len(data) < expected:
            self.get_logger().warn(
                f"Malformed overtake override: len={len(data)} expected={expected}",
                throttle_duration_sec=1.0)
            return

        lateral_offsets = data[3:3 + n]
        speed_caps = data[3 + n:3 + 2 * n]
        self._mpc.set_overtake_reference_override(lateral_offsets, speed_caps, mode_id)
        self._last_overtake_override_sec = self.get_clock().now().nanoseconds / 1e9

    def _clear_stale_overtake_override(self, now) -> None:
        if self._last_overtake_override_sec is None:
            return
        now_sec = now.nanoseconds / 1e9
        if now_sec - self._last_overtake_override_sec > self._overtake_override_timeout_sec:
            self._mpc.clear_overtake_reference_override()
            self._last_overtake_override_sec = None

    def _awsim_status_callback(self, msg):
        laps = int(msg.data[1])
        lap_time = msg.data[2]
        # section = int(msg.data[3])

        if self._current_laps is None:
            self._current_laps = 1 if laps == 0 else laps

        if laps > self._current_laps:
            self.get_logger().info(f'\033[32mLap {self._current_laps} completed! Lap time: {self._last_lap_time} s\033[0m')
            self._lap_times[self._current_laps] = self._last_lap_time
            self._current_laps = laps

        self._last_lap_time = lap_time

    def _condition_callback(self, msg: Int32):
        if self._last_condition is None:
            self._last_condition = msg.data

        diff_condition = msg.data - self._last_condition
        if diff_condition > 30.0:
            self._last_colliding_time = self.get_clock().now()
            self.get_logger().warning(f"Collision detected!")
        self._last_condition = msg.data

    def _stop_request_callback(self, msg: Empty) -> None:
        if self._enable_control:
            self.get_logger().warn(f"Stop request received {self._enable_control}")
            self._enable_control = False

    def _wait_until_clock_received(self) -> None:
        if self.use_sim_time:
            self.get_logger().info(f"wait until clock received...")
            rate = self.create_rate(10)
            rate.sleep()
            self.get_logger().info(f">> OK!")

    def _wait_until_message_received(self, message_getter, message_name: str, timeout: float, rate_hz: int = 30) -> None:

        t_start = self.get_clock().now()
        rate = self.create_rate(rate_hz)

        self.get_logger().info(f"wait until {message_name} received...")

        while message_getter() is None:
            now = self.get_clock().now()
            if (now - t_start).nanoseconds > timeout * 1e9:
                self.get_logger().info(f"now: {now}, t_start: {t_start}")
                raise TimeoutError(f"Timeout while waiting for {message_name} message")
            rate.sleep()

        self.get_logger().info(f">> OK!")

    def _wait_until_odom_received(self, timeout: float = 30.) -> None:
        self._wait_until_message_received(lambda: self._odom, 'odometry', timeout)

    def _wait_until_trajectory_received(self, timeout: float = 30.) -> None:
        if self._cfg.reference_path.update_by_topic:
            self._wait_until_message_received(lambda: self._trajectory, 'trajectory', timeout)

    def _wait_until_path_constraints_received(self, timeout: float = 30.) -> None:
        if self.USE_OBSTACLE_AVOIDANCE and self._cfg.reference_path.use_path_constraints_topic: # type: ignore
            self._wait_until_message_received(lambda: self._reference_path.path_constraints, 'path constraints', timeout)

    def _publish_mpc_pred_marker(self, x_pred, y_pred):
        pred_marker_array = MarkerArray()
        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "mpc_pred"
        m_base.type = Marker.SPHERE
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale = Vector3(x=0.5, y=0.5, z=0.5)
        m_base.color = self._pred_marker_color
        for i in range(len(x_pred)):
            m = copy.deepcopy(m_base)
            m.id = i
            m.pose.position.x = x_pred[i]
            m.pose.position.y = y_pred[i]
            pred_marker_array.markers.append(m) # type: ignore
        self._mpc_pred_pub.publish(pred_marker_array)
        self._mpc_pred_pub_dummy.publish(pred_marker_array)

    def _publish_ref_path_marker(self, ref_path: ReferencePath):
        WP_SPHERE_ENABLED = False

        ref_path_marker_array = MarkerArray()

        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "ref_path"
        m_base.type = Marker.LINE_STRIP
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale.x = 0.2
        m_base.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)

        for i in range(len(ref_path.waypoints) - 1):
            m = copy.deepcopy(m_base)
            m.id = i
            start = Point()
            start.x = ref_path.waypoints[i].x
            start.y = ref_path.waypoints[i].y
            end = Point()
            end.x = ref_path.waypoints[i + 1].x
            end.y = ref_path.waypoints[i + 1].y
            m.points.append(start) # type: ignore
            m.points.append(end) # type: ignore
            ref_path_marker_array.markers.append(m) # type: ignore

        if WP_SPHERE_ENABLED:
            spheres = Marker()
            spheres.header.frame_id = "map"
            spheres.ns = "ref_path_point"
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            radius = 0.2
            spheres.scale = Vector3(x=radius, y=radius, z=radius)
            spheres.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            for i in range(len(ref_path.waypoints) - 1):
                p = Point()
                p.x = ref_path.waypoints[i].x
                p.y = ref_path.waypoints[i].y
                p.z = 0.
                spheres.points.append(p) #type: ignore
            ref_path_marker_array.markers.append(spheres) # type: ignore

        self._ref_path_pub.publish(ref_path_marker_array)
        self._ref_path_pub_dummy.publish(ref_path_marker_array)

    def _control(self):
        now = self.get_clock().now()
        t = (now - self._t_start).nanoseconds / 1e9
        dt = (now - self._last_t).nanoseconds / 1e9

        self._last_t = now
        self._loop += 1

        # record and print execution stats
        if self.use_stats:
            self._stats.record()

        # self.get_logger().info("loop")
        self._control_rate.sleep()

        if self._loop % 100 == 0:
            # update reference path
            if self._cfg.reference_path.update_by_topic: # type: ignore
                new_referece_path = self._create_reference_path_from_autoware_trajectory(self._trajectory)
                if new_referece_path is not None:
                    self._reference_path = new_referece_path
                    self._car.reference_path = self._reference_path
                    self._car.update_reference_path(self._car.reference_path)
                    self._curvature_speed_profile_mps = self._read_reference_speed_profile()
                    self._apply_speed_profile(self._car.wp_id)

            def plot_reference_path(car):
                import matplotlib.pyplot as plt
                import sys
                fig, ax = plt.subplots(1, 1)
                car.reference_path.show(ax)
                plt.show()
                sys.exit(1)
            # plot_reference_path(self._car)

        if self.USE_OBSTACLE_AVOIDANCE and self._obstacles_updated:
            self._obstacles_updated = False
            self._map.reset_map()
            filtered_dynamic = self._filter_obstacles_to_corridor(self._dynamic_obstacles)
            self._map.add_obstacles(self._static_obstacles + filtered_dynamic)
            self._reference_path.reset_dynamic_constraints()

        is_colliding = False
        if self._last_colliding_time is not None:
            elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
            if elapsed_from_last_colliding < 5.0:
                is_colliding = True

        pose = odom_to_pose_2d(self._odom) # type: ignore
        v = self._odom.twist.twist.linear.x
        self._update_grade_estimate(self._odom, v) # type: ignore

        self._car.update_states(pose.x, pose.y, pose.theta)
        # print(f"car x: {self._car.temporal_state.x}, y: {self._car.temporal_state.y}, psi: {self._car.temporal_state.psi}")
        # print(f"mpc x: {self._mpc.model.temporal_state.x}, y: {self._mpc.model.temporal_state.y}, psi: {self._mpc.model.temporal_state.psi}")
        self._car.get_current_waypoint()
        self._apply_speed_profile(self._car.wp_id)
        self._clear_stale_overtake_override(now)

        with self._stats.time_block("control"):
            solve_started = perf_counter()
            u, max_delta = self._mpc.get_control()
            self._last_mpc_solve_time_ms = (perf_counter() - solve_started) * 1000.0
            self._last_mpc_infeasible_count = int(getattr(self._mpc, "infeasibility_counter", 0))
            self._last_mpc_status = "infeasible" if self._last_mpc_infeasible_count > 0 else "solved"
            # self.get_logger().info(f"u: {u}")

        # override by brake command if control is disabled
        if not self._enable_control:
            last_v_cmd = self._last_u[0]
            if last_v_cmd < 0.5:
                u[0] = 0.0
            else:
                decel_v = last_v_cmd + self._mpc_cfg.a_min * dt
                u[0] = np.clip(decel_v, 0.0, self._mpc_cfg.v_max)

        if len(u) == 0:
            self.get_logger().error("No control signal", throttle_duration_sec=1)
            u = [0.0, 0.0]
            # continue

        acc = 0.
        bug_acc_enabled = False
        self._last_grade_accel_base_mps2 = 0.0
        self._last_grade_accel_ff_mps2 = 0.0
        if self.USE_BUG_ACC:
            def deg2rad(deg):
                return deg * np.pi / 180.0

            if abs(v) > kmh_to_m_per_sec(44.0) or \
             (abs(v) > kmh_to_m_per_sec(38.0) and abs(max_delta) > deg2rad(12.0)):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_min / 3.0 * 2.0
                self._pred_marker_color = RED
            elif abs(v) > kmh_to_m_per_sec(41.0) or abs(u[1]) > deg2rad(10.0):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_max
                self._pred_marker_color = YELLOW
            else:
                bug_acc_enabled = True
                acc = 500.0
                self._pred_marker_color = CYAN
            self._last_grade_accel_base_mps2 = acc
        else:
            base_acc = self.KP * (u[0] - v)
            # print(f"v: {v}, u[0]: {u[0]}, acc: {base_acc}")
            base_acc = float(np.clip(base_acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max))
            self._last_grade_accel_base_mps2 = base_acc
            acc = base_acc + self._compute_grade_accel_ff(float(u[0]), float(v))
            acc = float(np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max))
        # u[0] = np.clip(last_u[0] + acc * dt, 0.0, self._mpc_cfg.v_max)

        # apply low pass filter to control signal
        acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
        u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain

        self._last_acc = acc
        self._last_u[0] = u[0]
        self._last_u[1] = u[1]

        # update car state (use v for feedback actual speed)
        self._car.drive([v, u[1]])

        # Publish control command
        self._publish_control_command(now, u, acc, bug_acc_enabled)
        self._publish_speed_profile_debug(now, self._mpc.model.wp_id, v, float(u[0]))

        # Log states
        self._sim_logger.log(self._car, u, t)
        self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)

        # 約 0.25 秒ごとに予測結果を表示
        if (self._mpc.current_prediction is not None) and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
            self._publish_mpc_pred_marker(self._mpc.current_prediction[0], self._mpc.current_prediction[1]) # type: ignore

    def run(self) -> None:
        self._wait_until_clock_received()
        self._wait_until_odom_received()
        self._wait_until_trajectory_received()
        self._wait_until_path_constraints_received()

        # initialize car states
        pose = odom_to_pose_2d(self._odom) # type: ignore
        self._car.update_states(pose.x, pose.y, pose.theta)
        self._car.update_reference_path(self._car.reference_path)

        if self._ref_vel_configulator is None:
            self._publish_ref_path_marker(self._car.reference_path)

        self._pred_marker_color = CYAN

        # for i in range(10):
        #     self._obstacle_manager.push_next_obstacle()

        # initialize control states
        self._control_rate = self.create_rate(self._mpc_cfg.control_rate)
        self._sim_logger = SimulationLogger(
            self.get_logger(),
            self._car.temporal_state.x, self._car.temporal_state.y, self._cfg.sim_logger.animation_enabled, self.SHOW_PLOT_ANIMATION, self.PLOT_RESULTS, self.ANIMATION_INTERVAL) # type: ignore

        self._loop = 0
        self._last_acc = 0.0
        self._last_u = np.array([0.0, 0.0])
        self._reset_grade_estimator()
        self._t_start = self.get_clock().now()
        self._last_t = self._t_start

        self.get_logger().info("----------------------")
        self.get_logger().info("START!")
        self.get_logger().info("----------------------")

        while rclpy.ok() and (not self._sim_logger.stop_requested()):
            self._control()

    def stop(self):
        # Wait for stopping
        self.get_logger().warn("----------------------")
        self.get_logger().warn("Stopping...")
        self.get_logger().warn("----------------------")
        timeout_time = self.get_clock().now() + rclpy.time.Duration(seconds=5)
        while self._odom.twist.twist.linear.x > 0.1 and self.get_clock().now() < timeout_time:
            self._enable_control = False
            self._control()

        # Publish zero command to stop the car completely
        zero_cmd = self._create_ackerman_control_command(self.get_clock().now(), [0.0, 0.0], 0.0, False)
        self._command_pub.publish(zero_cmd)

        self.get_logger().warn(">> Stop Completed!")

        # show results
        self._sim_logger.show_results(self._current_laps, self._lap_times, self._car)

    @classmethod
    def in_pkg_share(cls, file_path: str) -> str:
        return cls.PKG_PATH + file_path
