#include "simple_delay_aware_control/simple_delay_aware_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>

namespace simple_delay_aware_control
{
namespace
{

using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

AckermannControlCommand zeroCommand(const rclcpp::Time & stamp)
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

std::string jsonBool(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

SimpleDelayAwareControlNode::SimpleDelayAwareControlNode()
: Node("simple_delay_aware_control_node")
{
  wheel_base_ = declare_parameter<double>("wheel_base", wheel_base_);
  lookahead_gain_ = declare_parameter<double>("lookahead_gain", lookahead_gain_);
  lookahead_min_distance_ =
    declare_parameter<double>("lookahead_min_distance", lookahead_min_distance_);
  speed_proportional_gain_ =
    declare_parameter<double>("speed_proportional_gain", speed_proportional_gain_);
  max_acceleration_mps2_ = declare_parameter<double>("max_acceleration_mps2", max_acceleration_mps2_);
  max_deceleration_mps2_ = declare_parameter<double>("max_deceleration_mps2", max_deceleration_mps2_);
  steering_tire_angle_gain_ =
    declare_parameter<double>("steering_tire_angle_gain", steering_tire_angle_gain_);
  max_steering_tire_angle_rad_ =
    declare_parameter<double>("max_steering_tire_angle_rad", max_steering_tire_angle_rad_);
  control_period_ms_ = declare_parameter<double>("control_period_ms", control_period_ms_);

  delay_params_.enabled = declare_parameter<bool>("delay_enabled", delay_params_.enabled);
  delay_params_.use_steering_lag =
    declare_parameter<bool>("use_steering_lag", delay_params_.use_steering_lag);
  delay_params_.delay_sec = declare_parameter<double>("steering_delay_sec", delay_params_.delay_sec);
  delay_params_.prediction_dt = declare_parameter<double>("prediction_dt", delay_params_.prediction_dt);
  delay_params_.steering_time_constant_sec = declare_parameter<double>(
    "steering_time_constant_sec", delay_params_.steering_time_constant_sec);
  delay_params_.wheel_base = wheel_base_;
  delay_params_.max_steering_rad = max_steering_tire_angle_rad_;

  use_steering_status_ = declare_parameter<bool>("use_steering_status", use_steering_status_);
  steering_status_timeout_sec_ =
    declare_parameter<double>("steering_status_timeout_sec", steering_status_timeout_sec_);
  use_control_cmd_input_ = declare_parameter<bool>("use_control_cmd_input", use_control_cmd_input_);
  control_cmd_timeout_sec_ =
    declare_parameter<double>("control_cmd_timeout_sec", control_cmd_timeout_sec_);
  use_yaw_rate_fallback_ = declare_parameter<bool>("use_yaw_rate_fallback", use_yaw_rate_fallback_);
  min_velocity_for_yaw_prediction_ =
    declare_parameter<double>("min_velocity_for_yaw_prediction", min_velocity_for_yaw_prediction_);

  speed_params_.enabled = declare_parameter<bool>("curvature_speed_enabled", speed_params_.enabled);
  speed_params_.use_trajectory_velocity =
    declare_parameter<bool>("use_trajectory_velocity", speed_params_.use_trajectory_velocity);
  speed_params_.max_speed_mps = declare_parameter<double>("max_speed_mps", speed_params_.max_speed_mps);
  speed_params_.min_speed_mps = declare_parameter<double>("min_speed_mps", speed_params_.min_speed_mps);
  speed_params_.lateral_accel_limit_mps2 = declare_parameter<double>(
    "lateral_accel_limit_mps2", speed_params_.lateral_accel_limit_mps2);
  speed_params_.curvature_epsilon =
    declare_parameter<double>("curvature_epsilon", speed_params_.curvature_epsilon);
  speed_params_.curvature_lookahead_points = static_cast<std::size_t>(
    std::max<int>(2, declare_parameter<int>("curvature_lookahead_points", 8)));
  trajectory_is_closed_ = declare_parameter<bool>("trajectory_is_closed", trajectory_is_closed_);
  publish_delay_aware_odometry_ =
    declare_parameter<bool>("publish_delay_aware_odometry", publish_delay_aware_odometry_);
  write_target_speed_to_output_twist_ = declare_parameter<bool>(
    "write_target_speed_to_output_twist", write_target_speed_to_output_twist_);
  debug_publish_period_sec_ =
    declare_parameter<double>("debug_publish_period_sec", debug_publish_period_sec_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", qos, [this](const Odometry::ConstSharedPtr msg) { onOdometry(msg); });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", qos, [this](const Trajectory::ConstSharedPtr msg) { onTrajectory(msg); });
  sub_steering_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", qos,
    [this](const SteeringReport::ConstSharedPtr msg) { onSteeringReport(msg); });
  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "input/control_cmd", qos,
    [this](const AckermannControlCommand::ConstSharedPtr msg) { onControlCommand(msg); });

  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_delay_aware_odometry_ = create_publisher<Odometry>("output/kinematics", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<std_msgs::msg::String>("/simple_delay_aware_control/debug", 1);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(std::max(1.0, control_period_ms_))),
    std::bind(&SimpleDelayAwareControlNode::onTimer, this));
}

void SimpleDelayAwareControlNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  trajectory_.clear();
  trajectory_.reserve(msg->points.size());
  for (const auto & point : msg->points) {
    trajectory_.push_back(TrajectorySample{
      point.pose.position.x, point.pose.position.y, yawFromQuaternion(point.pose.orientation),
      point.longitudinal_velocity_mps});
  }
}

void SimpleDelayAwareControlNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  if (!publish_delay_aware_odometry_) {
    return;
  }

  const double now_sec = nowSec();
  const VehicleState measured_state = makeVehicleState(*msg);
  const double current_steering = estimateCurrentSteering(measured_state, now_sec);
  const double target_steering = targetSteeringForPrediction(now_sec);
  delay_params_.wheel_base = wheel_base_;
  delay_params_.max_steering_rad = max_steering_tire_angle_rad_;

  const DelayPrediction delay_prediction =
    predictDelay(measured_state, current_steering, target_steering, delay_params_);

  std::optional<SpeedPlan> speed_plan;
  if (write_target_speed_to_output_twist_ && !trajectory_.empty()) {
    speed_plan = planCurvatureSpeed(
      trajectory_, delay_prediction.predicted, speed_params_, trajectory_is_closed_);
  }

  publishDelayAwareOdometry(*msg, delay_prediction, speed_plan);
}

void SimpleDelayAwareControlNode::onSteeringReport(const SteeringReport::ConstSharedPtr msg)
{
  latest_steering_status_ = {nowSec(), static_cast<double>(msg->steering_tire_angle)};
}

void SimpleDelayAwareControlNode::onControlCommand(const AckermannControlCommand::ConstSharedPtr msg)
{
  latest_control_cmd_steering_ = {nowSec(), static_cast<double>(msg->lateral.steering_tire_angle)};
}

bool SimpleDelayAwareControlNode::ready()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "odometry is not available");
    return false;
  }
  if (trajectory_.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "trajectory is not available");
    return false;
  }
  return true;
}

VehicleState SimpleDelayAwareControlNode::makeVehicleState(const Odometry & odometry) const
{
  return VehicleState{
    odometry.pose.pose.position.x, odometry.pose.pose.position.y,
    yawFromQuaternion(odometry.pose.pose.orientation), odometry.twist.twist.linear.x,
    odometry.twist.twist.angular.z};
}

double SimpleDelayAwareControlNode::estimateCurrentSteering(
  const VehicleState & state, const double now_sec) const
{
  if (use_steering_status_ && latest_steering_status_.has_value()) {
    const double age = now_sec - latest_steering_status_->first;
    if (age >= 0.0 && age <= steering_status_timeout_sec_) {
      return latest_steering_status_->second;
    }
  }

  if (use_yaw_rate_fallback_ && std::abs(state.velocity) >= min_velocity_for_yaw_prediction_) {
    return std::atan2(wheel_base_ * state.yaw_rate, state.velocity);
  }

  return last_command_steering_rad_;
}

double SimpleDelayAwareControlNode::targetSteeringForPrediction(const double now_sec) const
{
  if (use_control_cmd_input_ && latest_control_cmd_steering_.has_value()) {
    const double age = now_sec - latest_control_cmd_steering_->first;
    if (age >= 0.0 && age <= control_cmd_timeout_sec_) {
      return latest_control_cmd_steering_->second;
    }
  }
  return last_command_steering_rad_;
}

void SimpleDelayAwareControlNode::publishDelayAwareOdometry(
  const Odometry & source_odometry, const DelayPrediction & delay_prediction,
  const std::optional<SpeedPlan> & speed_plan)
{
  auto output = source_odometry;
  if (delay_prediction.shifted) {
    output.pose.pose.position.x = delay_prediction.predicted.x;
    output.pose.pose.position.y = delay_prediction.predicted.y;
    output.pose.pose.orientation = quaternionFromYaw(delay_prediction.predicted.yaw);
  }
  if (write_target_speed_to_output_twist_ && speed_plan.has_value()) {
    output.twist.twist.linear.x = speed_plan->target_speed_mps;
  }
  pub_delay_aware_odometry_->publish(output);
}

std::size_t SimpleDelayAwareControlNode::findLookaheadIndex(
  const VehicleState & control_state, const std::size_t nearest_index,
  const double lookahead_distance) const
{
  const double rear_x = control_state.x - wheel_base_ * 0.5 * std::cos(control_state.yaw);
  const double rear_y = control_state.y - wheel_base_ * 0.5 * std::sin(control_state.yaw);

  for (std::size_t offset = 0; offset < trajectory_.size(); ++offset) {
    const std::size_t index =
      trajectory_is_closed_ ? (nearest_index + offset) % trajectory_.size()
                            : std::min(nearest_index + offset, trajectory_.size() - 1);
    const double dx = trajectory_[index].x - rear_x;
    const double dy = trajectory_[index].y - rear_y;
    if (std::hypot(dx, dy) >= lookahead_distance) {
      return index;
    }
    if (!trajectory_is_closed_ && index + 1 >= trajectory_.size()) {
      return index;
    }
  }

  return nearest_index;
}

void SimpleDelayAwareControlNode::publishLookaheadPoint(const TrajectorySample & sample)
{
  PointStamped point;
  point.header.stamp = get_clock()->now();
  point.header.frame_id = "map";
  point.point.x = sample.x;
  point.point.y = sample.y;
  point.point.z = 0.0;
  pub_lookahead_point_->publish(point);
}

void SimpleDelayAwareControlNode::onTimer()
{
  if (!ready()) {
    return;
  }

  const auto stamp = get_clock()->now();
  const double now_sec = nowSec();
  const VehicleState measured_state = makeVehicleState(*odometry_);
  const double current_steering = estimateCurrentSteering(measured_state, now_sec);
  const double target_steering = targetSteeringForPrediction(now_sec);
  delay_params_.wheel_base = wheel_base_;
  delay_params_.max_steering_rad = max_steering_tire_angle_rad_;

  const DelayPrediction delay_prediction =
    predictDelay(measured_state, current_steering, target_steering, delay_params_);
  const VehicleState & control_state = delay_prediction.predicted;
  const SpeedPlan speed_plan =
    planCurvatureSpeed(trajectory_, control_state, speed_params_, trajectory_is_closed_);

  const double target_speed = speed_plan.target_speed_mps;
  const double current_speed = measured_state.velocity;
  const double acceleration = clamp(
    speed_proportional_gain_ * (target_speed - current_speed), -max_deceleration_mps2_,
    max_acceleration_mps2_);

  const double lookahead_distance =
    std::max(lookahead_min_distance_, lookahead_gain_ * std::max(target_speed, current_speed));
  const std::size_t lookahead_index =
    findLookaheadIndex(control_state, speed_plan.nearest_index, lookahead_distance);
  const auto & lookahead = trajectory_[lookahead_index];
  publishLookaheadPoint(lookahead);

  const double rear_x = control_state.x - wheel_base_ * 0.5 * std::cos(control_state.yaw);
  const double rear_y = control_state.y - wheel_base_ * 0.5 * std::sin(control_state.yaw);
  const double alpha =
    normalizeAngle(std::atan2(lookahead.y - rear_y, lookahead.x - rear_x) - control_state.yaw);
  const double raw_steering =
    std::atan2(2.0 * wheel_base_ * std::sin(alpha), std::max(lookahead_distance, 1.0e-3));
  const double command_steering = clamp(
    steering_tire_angle_gain_ * raw_steering, -max_steering_tire_angle_rad_,
    max_steering_tire_angle_rad_);

  auto cmd = zeroCommand(stamp);
  cmd.longitudinal.speed = target_speed;
  cmd.longitudinal.acceleration = acceleration;
  cmd.lateral.steering_tire_angle = command_steering;
  pub_cmd_->publish(cmd);

  auto raw_cmd = cmd;
  raw_cmd.lateral.steering_tire_angle = raw_steering;
  pub_raw_cmd_->publish(raw_cmd);
  last_command_steering_rad_ = command_steering;

  publishDebug(now_sec, delay_prediction, speed_plan, target_steering, raw_steering, command_steering);
}

void SimpleDelayAwareControlNode::publishDebug(
  const double now_sec, const DelayPrediction & delay_prediction, const SpeedPlan & speed_plan,
  const double target_steering_rad, const double raw_steering_rad, const double command_steering_rad)
{
  if (debug_publish_period_sec_ > 0.0 &&
    now_sec - last_debug_publish_sec_ < debug_publish_period_sec_)
  {
    return;
  }
  last_debug_publish_sec_ = now_sec;

  std_msgs::msg::String msg;
  std::ostringstream json;
  json << "{"
       << "\"delay_shifted\":" << jsonBool(delay_prediction.shifted) << ","
       << "\"prediction_steps\":" << delay_prediction.prediction_steps << ","
       << "\"current_steering_rad\":" << delay_prediction.current_steering_rad << ","
       << "\"target_steering_rad\":" << target_steering_rad << ","
       << "\"applied_steering_rad\":" << delay_prediction.applied_steering_rad << ","
       << "\"raw_steering_rad\":" << raw_steering_rad << ","
       << "\"command_steering_rad\":" << command_steering_rad << ","
       << "\"nearest_index\":" << speed_plan.nearest_index << ","
       << "\"curvature\":" << speed_plan.curvature << ","
       << "\"curvature_speed_limit_mps\":" << speed_plan.curvature_speed_limit_mps << ","
       << "\"trajectory_speed_mps\":" << speed_plan.trajectory_speed_mps << ","
       << "\"target_speed_mps\":" << speed_plan.target_speed_mps << ","
       << "\"input_pose\":{"
       << "\"x\":" << delay_prediction.input.x << ","
       << "\"y\":" << delay_prediction.input.y << ","
       << "\"yaw\":" << delay_prediction.input.yaw << "},"
       << "\"control_pose\":{"
       << "\"x\":" << delay_prediction.predicted.x << ","
       << "\"y\":" << delay_prediction.predicted.y << ","
       << "\"yaw\":" << delay_prediction.predicted.yaw << "}"
       << "}";
  msg.data = json.str();
  pub_debug_->publish(msg);
}

double SimpleDelayAwareControlNode::nowSec()
{
  return static_cast<double>(get_clock()->now().nanoseconds()) / 1.0e9;
}

}  // namespace simple_delay_aware_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_delay_aware_control::SimpleDelayAwareControlNode>());
  rclcpp::shutdown();
  return 0;
}
