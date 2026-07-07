#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>

namespace simple_pure_pursuit {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
    : Node("simple_pure_pursuit"),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
      lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
      lookahead_min_distance_(
          declare_parameter<float>("lookahead_min_distance", 1.0)),
      speed_proportional_gain_(
          declare_parameter<float>("speed_proportional_gain", 1.0)),
      use_external_target_vel_(
          declare_parameter<bool>("use_external_target_vel", false)),
      external_target_vel_(
          declare_parameter<float>("external_target_vel", 0.0)),
      steering_tire_angle_gain_(
          declare_parameter<float>("steering_tire_angle_gain", 1.0)),
      debug_publish_period_sec_(
          declare_parameter<float>("debug_publish_period_sec", 0.25)),
      use_overtake_reference_override_(
          declare_parameter<bool>("use_overtake_reference_override", false)),
      overtake_override_timeout_sec_(
          declare_parameter<float>("overtake_override_timeout_sec", 0.50)),
      curvature_adaptive_lookahead_enabled_(declare_parameter<bool>(
          "curvature_adaptive_lookahead_enabled", true)),
      curvature_lookahead_min_distance_(
          declare_parameter<float>("curvature_lookahead_min_distance", 3.5)),
      curvature_lookahead_sensitivity_(
          declare_parameter<float>("curvature_lookahead_sensitivity", 8.0)),
      curvature_lookahead_window_ratio_(
          declare_parameter<float>("curvature_lookahead_window_ratio", 2.0)),
      curvature_lookahead_max_window_distance_(declare_parameter<float>(
          "curvature_lookahead_max_window_distance", 10.0)),
      curvature_lookahead_min_arc_length_(
          declare_parameter<float>("curvature_lookahead_min_arc_length", 1.0)),
      curvature_lookahead_smoothing_alpha_(declare_parameter<float>(
          "curvature_lookahead_smoothing_alpha", 0.35)) {
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ =
      create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ =
      create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<String>("/pure_pursuit/debug", 1);

  const auto bv_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", bv_qos,
      [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", bv_qos,
      [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  sub_overtake_override_ = create_subscription<Float32MultiArray>(
      "input/overtake_reference_override", rclcpp::QoS(1),
      [this](const Float32MultiArray::SharedPtr msg) {
        onOvertakeOverride(msg);
      });

  using namespace std::literals::chrono_literals;
  timer_ =
      create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp) {
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer() {
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }
  const auto stamp = get_clock()->now();
  const double now_sec = stamp.seconds();

  size_t closet_traj_point_idx =
      findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  const Trajectory *control_trajectory = trajectory_.get();
  Trajectory adjusted_trajectory;
  bool overtake_override_applied = false;
  if (overtakeOverrideFresh(now_sec)) {
    adjusted_trajectory = *trajectory_;
    overtake_override_applied = applyOvertakeOverride(
        adjusted_trajectory, closet_traj_point_idx, now_sec);
    if (overtake_override_applied) {
      control_trajectory = &adjusted_trajectory;
    }
  } else if (overtake_override_active_) {
    clearOvertakeOverride();
  }

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(stamp);

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point =
      control_trajectory->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_
                               : closet_traj_point.longitudinal_velocity_mps;
  const auto overtake_speed_cap = overtakeSpeedCap(0, now_sec);
  if (overtake_speed_cap.has_value()) {
    target_longitudinal_vel =
        std::min(target_longitudinal_vel, overtake_speed_cap.value());
  }
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;
  const double current_yaw = tf2::getYaw(odometry_->pose.pose.orientation);

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
      speed_proportional_gain_ *
      (target_longitudinal_vel - current_longitudinal_vel);

  // calc lateral control
  //// calc lookahead distance
  const LookaheadParams lookahead_params{
      lookahead_gain_, lookahead_min_distance_,
      curvature_adaptive_lookahead_enabled_, curvature_lookahead_min_distance_,
      curvature_lookahead_sensitivity_};
  const double base_lookahead_distance = speedBasedLookaheadDistance(
      target_longitudinal_vel, current_longitudinal_vel, lookahead_params);
  const double curvature_window_ratio =
      std::isfinite(curvature_lookahead_window_ratio_)
          ? std::max(1.0, curvature_lookahead_window_ratio_)
          : 1.0;
  const double curvature_min_arc_m =
      std::isfinite(curvature_lookahead_min_arc_length_)
          ? std::max(1.0e-3, curvature_lookahead_min_arc_length_)
          : 1.0e-3;
  const double curvature_window_max_m =
      std::isfinite(curvature_lookahead_max_window_distance_)
          ? std::max(curvature_min_arc_m,
                     curvature_lookahead_max_window_distance_)
          : base_lookahead_distance;
  const double curvature_window_distance = std::min(
      curvature_window_max_m, base_lookahead_distance * curvature_window_ratio);
  const double path_curvature = estimateTrajectoryCurvature(
      *trajectory_, closet_traj_point_idx, curvature_window_distance,
      curvature_min_arc_m);
  const double desired_lookahead_distance = adaptiveLookaheadDistance(
      target_longitudinal_vel, current_longitudinal_vel, path_curvature,
      lookahead_params);
  double lookahead_distance = desired_lookahead_distance;
  if (curvature_adaptive_lookahead_enabled_) {
    lookahead_distance = smoothLookaheadDistance(
        desired_lookahead_distance, smoothed_lookahead_distance_,
        has_smoothed_lookahead_distance_, curvature_lookahead_smoothing_alpha_);
    smoothed_lookahead_distance_ = lookahead_distance;
    has_smoothed_lookahead_distance_ = true;
  } else {
    has_smoothed_lookahead_distance_ = false;
  }
  //// calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(current_yaw);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(current_yaw);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
      control_trajectory->points.begin() + closet_traj_point_idx,
      control_trajectory->points.end(), [&](const TrajectoryPoint &point) {
        return std::hypot(point.pose.position.x - rear_x,
                          point.pose.position.y - rear_y) >= lookahead_distance;
      });
  if (lookahead_point_itr == control_trajectory->points.end()) {
    lookahead_point_itr = std::prev(control_trajectory->points.end());
  }
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha =
      std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
      current_yaw;
  const double raw_steering_tire_angle =
      std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  cmd.lateral.steering_tire_angle =
      steering_tire_angle_gain_ * raw_steering_tire_angle;

  publishDebug(cmd.stamp, *control_trajectory, closet_traj_point_idx,
               target_longitudinal_vel, current_longitudinal_vel,
               cmd.longitudinal.acceleration, base_lookahead_distance,
               desired_lookahead_distance, lookahead_distance, path_curvature,
               curvature_window_distance, lookahead_point_x, lookahead_point_y,
               rear_x, rear_y, alpha, raw_steering_tire_angle,
               cmd.lateral.steering_tire_angle, overtake_override_applied,
               overtakeLateralOffset(0), overtake_speed_cap.value_or(0.0));

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle = raw_steering_tire_angle;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable() {
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,
                         "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,
                         "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,
                         "trajectory points is empty");
    return false;
  }
  return true;
}

void SimplePurePursuit::onOvertakeOverride(
    const Float32MultiArray::SharedPtr msg) {
  if (!use_overtake_reference_override_) {
    return;
  }

  const double now_sec = get_clock()->now().seconds();
  const auto &data = msg->data;
  if (data.size() < 3 || static_cast<int>(data[0]) != 1) {
    clearOvertakeOverride();
    return;
  }

  const int mode_id = static_cast<int>(data[1]);
  const int n = static_cast<int>(data[2]);
  if (n <= 0 || mode_id == 0) {
    clearOvertakeOverride();
    last_overtake_override_sec_ = now_sec;
    return;
  }

  const std::size_t count = static_cast<std::size_t>(n);
  const std::size_t expected = 3 + 2 * count;
  if (data.size() < expected) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Malformed overtake reference override: len=%zu expected=%zu",
        data.size(), expected);
    clearOvertakeOverride();
    return;
  }

  overtake_lateral_offsets_.clear();
  overtake_speed_caps_.clear();
  overtake_lateral_offsets_.reserve(count);
  overtake_speed_caps_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    overtake_lateral_offsets_.push_back(static_cast<double>(data[3 + i]));
  }
  for (std::size_t i = 0; i < count; ++i) {
    overtake_speed_caps_.push_back(static_cast<double>(data[3 + count + i]));
  }
  overtake_mode_id_ = mode_id;
  overtake_override_active_ = true;
  last_overtake_override_sec_ = now_sec;
}

void SimplePurePursuit::clearOvertakeOverride() {
  overtake_override_active_ = false;
  overtake_mode_id_ = 0;
  overtake_lateral_offsets_.clear();
  overtake_speed_caps_.clear();
  last_overtake_override_sec_ = -1.0e9;
}

bool SimplePurePursuit::overtakeOverrideFresh(double now_sec) const {
  return use_overtake_reference_override_ && overtake_override_active_ &&
         !overtake_lateral_offsets_.empty() &&
         now_sec - last_overtake_override_sec_ <=
             overtake_override_timeout_sec_;
}

bool SimplePurePursuit::applyOvertakeOverride(
    Trajectory &trajectory, std::size_t nearest_traj_point_idx,
    double now_sec) {
  if (!overtakeOverrideFresh(now_sec) ||
      nearest_traj_point_idx >= trajectory.points.size()) {
    return false;
  }

  const std::size_t count =
      std::min(overtake_lateral_offsets_.size(),
               trajectory.points.size() - nearest_traj_point_idx);
  for (std::size_t i = 0; i < count; ++i) {
    const double offset_m = overtake_lateral_offsets_[i];
    if (!std::isfinite(offset_m)) {
      continue;
    }

    auto &point = trajectory.points[nearest_traj_point_idx + i];
    const double yaw = tf2::getYaw(point.pose.orientation);
    point.pose.position.x = point.pose.position.x - offset_m * std::sin(yaw);
    point.pose.position.y = point.pose.position.y + offset_m * std::cos(yaw);

    const auto speed_cap = overtakeSpeedCap(i, now_sec);
    if (speed_cap.has_value()) {
      point.longitudinal_velocity_mps =
          std::min(point.longitudinal_velocity_mps,
                   static_cast<float>(speed_cap.value()));
    }
  }
  return count > 0;
}

std::optional<double>
SimplePurePursuit::overtakeSpeedCap(std::size_t horizon_index,
                                    double now_sec) const {
  if (!overtakeOverrideFresh(now_sec) ||
      horizon_index >= overtake_speed_caps_.size()) {
    return std::nullopt;
  }
  const double speed_cap_mps = overtake_speed_caps_[horizon_index];
  if (!std::isfinite(speed_cap_mps) || speed_cap_mps <= 0.0) {
    return std::nullopt;
  }
  return speed_cap_mps;
}

double
SimplePurePursuit::overtakeLateralOffset(std::size_t horizon_index) const {
  if (!overtake_override_active_ ||
      horizon_index >= overtake_lateral_offsets_.size()) {
    return 0.0;
  }
  const double offset_m = overtake_lateral_offsets_[horizon_index];
  return std::isfinite(offset_m) ? offset_m : 0.0;
}

void SimplePurePursuit::publishDebug(
    const rclcpp::Time &stamp, const Trajectory &control_trajectory,
    std::size_t nearest_traj_point_idx, double target_longitudinal_vel,
    double current_longitudinal_vel, double command_accel,
    double base_lookahead_distance, double desired_lookahead_distance,
    double lookahead_distance, double path_curvature,
    double curvature_window_distance, double lookahead_point_x,
    double lookahead_point_y, double rear_x, double rear_y, double alpha,
    double raw_steering_tire_angle, double steering_tire_angle,
    bool overtake_override_applied, double overtake_lateral_offset_m,
    double overtake_speed_cap_mps) {
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }

  const double now_sec = stamp.seconds();
  if (now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = now_sec;

  const auto &nearest = control_trajectory.points.at(nearest_traj_point_idx);
  const double ego_x = odometry_->pose.pose.position.x;
  const double ego_y = odometry_->pose.pose.position.y;
  const double ref_x = nearest.pose.position.x;
  const double ref_y = nearest.pose.position.y;
  const double ref_yaw = tf2::getYaw(nearest.pose.orientation);
  const double ego_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double dx = ego_x - ref_x;
  const double dy = ego_y - ref_y;
  const double lateral_error_m =
      -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
  const double yaw_error_rad =
      std::atan2(std::sin(ego_yaw - ref_yaw), std::cos(ego_yaw - ref_yaw));

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"nearest_trajectory_index\":" << nearest_traj_point_idx << ","
       << "\"target_speed_mps\":" << target_longitudinal_vel << ","
       << "\"current_speed_mps\":" << current_longitudinal_vel << ","
       << "\"command_accel_mps2\":" << command_accel << ","
       << "\"base_lookahead_distance_m\":" << base_lookahead_distance << ","
       << "\"desired_lookahead_distance_m\":" << desired_lookahead_distance
       << ","
       << "\"lookahead_distance_m\":" << lookahead_distance << ","
       << "\"path_curvature_1pm\":" << path_curvature << ","
       << "\"curvature_window_distance_m\":" << curvature_window_distance << ","
       << "\"curvature_adaptive_lookahead_enabled\":"
       << (curvature_adaptive_lookahead_enabled_ ? "true" : "false") << ","
       << "\"lookahead_point_x\":" << lookahead_point_x << ","
       << "\"lookahead_point_y\":" << lookahead_point_y << ","
       << "\"rear_x\":" << rear_x << ","
       << "\"rear_y\":" << rear_y << ","
       << "\"alpha_rad\":" << alpha << ","
       << "\"raw_steering_tire_angle_rad\":" << raw_steering_tire_angle << ","
       << "\"steering_tire_angle_rad\":" << steering_tire_angle << ","
       << "\"lateral_error_m\":" << lateral_error_m << ","
       << "\"yaw_error_rad\":" << yaw_error_rad << ","
       << "\"use_external_target_vel\":"
       << (use_external_target_vel_ ? "true" : "false") << ","
       << "\"overtake_override_applied\":"
       << (overtake_override_applied ? "true" : "false") << ","
       << "\"overtake_mode_id\":" << overtake_mode_id_ << ","
       << "\"overtake_lateral_offset_m\":" << overtake_lateral_offset_m << ","
       << "\"overtake_speed_cap_mps\":" << overtake_speed_cap_mps << "}";

  String msg;
  msg.data = json.str();
  pub_debug_->publish(msg);
}
} // namespace simple_pure_pursuit

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
