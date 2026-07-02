#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  debug_publish_period_sec_(declare_parameter<float>("debug_publish_period_sec", 0.25))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<String>("/pure_pursuit/debug", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

  // calc lateral control
  //// calc lookahead distance
  double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
  //// calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
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
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 tf2::getYaw(odometry_->pose.pose.orientation);
  const double raw_steering_tire_angle =
    std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  cmd.lateral.steering_tire_angle = steering_tire_angle_gain_ * raw_steering_tire_angle;

  publishDebug(
    cmd.stamp, closet_traj_point_idx, target_longitudinal_vel, current_longitudinal_vel,
    cmd.longitudinal.acceleration, lookahead_distance, lookahead_point_x, lookahead_point_y, rear_x,
    rear_y, alpha, raw_steering_tire_angle, cmd.lateral.steering_tire_angle);

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle = raw_steering_tire_angle;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}

void SimplePurePursuit::publishDebug(
  const rclcpp::Time & stamp, std::size_t nearest_traj_point_idx, double target_longitudinal_vel,
  double current_longitudinal_vel, double command_accel, double lookahead_distance,
  double lookahead_point_x, double lookahead_point_y, double rear_x, double rear_y, double alpha,
  double raw_steering_tire_angle, double steering_tire_angle)
{
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }

  const double now_sec = stamp.seconds();
  if (now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = now_sec;

  const auto & nearest = trajectory_->points.at(nearest_traj_point_idx);
  const double ego_x = odometry_->pose.pose.position.x;
  const double ego_y = odometry_->pose.pose.position.y;
  const double ref_x = nearest.pose.position.x;
  const double ref_y = nearest.pose.position.y;
  const double ref_yaw = tf2::getYaw(nearest.pose.orientation);
  const double ego_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double dx = ego_x - ref_x;
  const double dy = ego_y - ref_y;
  const double lateral_error_m = -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;
  const double yaw_error_rad = std::atan2(std::sin(ego_yaw - ref_yaw), std::cos(ego_yaw - ref_yaw));

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"nearest_trajectory_index\":" << nearest_traj_point_idx << ","
       << "\"target_speed_mps\":" << target_longitudinal_vel << ","
       << "\"current_speed_mps\":" << current_longitudinal_vel << ","
       << "\"command_accel_mps2\":" << command_accel << ","
       << "\"lookahead_distance_m\":" << lookahead_distance << ","
       << "\"lookahead_point_x\":" << lookahead_point_x << ","
       << "\"lookahead_point_y\":" << lookahead_point_y << ","
       << "\"rear_x\":" << rear_x << ","
       << "\"rear_y\":" << rear_y << ","
       << "\"alpha_rad\":" << alpha << ","
       << "\"raw_steering_tire_angle_rad\":" << raw_steering_tire_angle << ","
       << "\"steering_tire_angle_rad\":" << steering_tire_angle << ","
       << "\"lateral_error_m\":" << lateral_error_m << ","
       << "\"yaw_error_rad\":" << yaw_error_rad << ","
       << "\"use_external_target_vel\":" << (use_external_target_vel_ ? "true" : "false") << "}";

  String msg;
  msg.data = json.str();
  pub_debug_->publish(msg);
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
