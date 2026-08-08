#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/lookahead.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>

namespace simple_pure_pursuit
{

namespace
{

AckermannControlCommand zeroCommand(const rclcpp::Time & stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.lateral.stamp = stamp;
  return cmd;
}

bool finiteInRange(const double value, const double minimum, const double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      out << '\\';
    }
    out << character;
  }
  return out.str();
}

}  // namespace

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  wheel_base_(declare_parameter<double>("wheel_base", 2.14)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  ga_experiment_mode_(declare_parameter<bool>("ga_experiment_mode", false)),
  controller_enabled_(!ga_experiment_mode_)
{
  tuning_.lookahead_gain = declare_parameter<double>("lookahead_gain", 0.5);
  tuning_.lookahead_min_distance = declare_parameter<double>("lookahead_min_distance", 3.5);
  tuning_.actual_lookahead_distance_blend =
    declare_parameter<double>("actual_lookahead_distance_blend", 0.0);
  tuning_.dual_preview_near_ratio =
    declare_parameter<double>("dual_preview_near_ratio", 0.5);
  tuning_.dual_preview_blend =
    declare_parameter<double>("dual_preview_blend", 0.0);
  tuning_.steering_tire_angle_gain =
    declare_parameter<double>("steering_tire_angle_gain", 1.5);
  tuning_.curvature_lookahead_min_distance =
    declare_parameter<double>("curvature_lookahead_min_distance", 2.0);
  tuning_.curvature_lookahead_sensitivity =
    declare_parameter<double>("curvature_lookahead_sensitivity", 8.0);
  tuning_.curvature_lookahead_smoothing_alpha =
    declare_parameter<double>("curvature_lookahead_smoothing_alpha", 0.35);
  tuning_.max_lateral_acceleration =
    declare_parameter<double>("max_lateral_acceleration", 6.0);
  tuning_.minimum_corner_speed =
    declare_parameter<double>("minimum_corner_speed", 5.0);
  tuning_.corner_speed_retention =
    declare_parameter<double>("corner_speed_retention", 0.0);
  tuning_.lateral_error_speed_gate_enabled =
    declare_parameter<bool>("lateral_error_speed_gate_enabled", true);
  tuning_.corner_speed_retention_lateral_error_soft =
    declare_parameter<double>("corner_speed_retention_lateral_error_soft", 0.5);
  tuning_.corner_speed_retention_lateral_error_hard =
    declare_parameter<double>("corner_speed_retention_lateral_error_hard", 1.0);
  tuning_.curvature_speed_preview_distance =
    declare_parameter<double>("curvature_speed_preview_distance", 12.0);
  tuning_.speed_proportional_gain =
    declare_parameter<double>("speed_proportional_gain", 1.0);
  tuning_.longitudinal_acceleration_limit =
    declare_parameter<double>("longitudinal_acceleration_limit", 1.0);
  tuning_.external_target_vel =
    declare_parameter<double>("external_target_vel", 9.722222222222);
  tuning_.continuous_preview_interpolation_enabled =
    declare_parameter<bool>("continuous_preview_interpolation_enabled", false);
  tuning_.delay_compensation_enabled =
    declare_parameter<bool>("delay_compensation_enabled", false);
  tuning_.pp_control_delay_sec =
    declare_parameter<double>("pp_control_delay_sec", 0.2);
  tuning_.pp_prediction_dt_sec =
    declare_parameter<double>("pp_prediction_dt_sec", 0.02);
  tuning_.steering_time_constant_sec =
    declare_parameter<double>("steering_time_constant_sec", 0.30);
  tuning_.steering_status_timeout_sec =
    declare_parameter<double>("steering_status_timeout_sec", 0.20);
  tuning_.min_velocity_for_delay_compensation_mps =
    declare_parameter<double>("min_velocity_for_delay_compensation_mps", 0.20);
  tuning_.curvature_feedforward_enabled =
    declare_parameter<bool>("curvature_feedforward_enabled", false);
  tuning_.curvature_feedforward_gain =
    declare_parameter<double>("curvature_feedforward_gain", 0.20);
  tuning_.exit_unwind_enabled =
    declare_parameter<bool>("exit_unwind_enabled", false);
  tuning_.exit_unwind_far_preview_boost =
    declare_parameter<double>("exit_unwind_far_preview_boost", 0.20);
  tuning_.exit_unwind_curvature_drop_threshold =
    declare_parameter<double>("exit_unwind_curvature_drop_threshold", 0.02);
  tuning_.rotation_gate_enabled =
    declare_parameter<bool>("rotation_gate_enabled", false);
  tuning_.rotation_gate_min_curvature =
    declare_parameter<double>("rotation_gate_min_curvature", 0.06);
  tuning_.rotation_gate_min_steering_angle =
    declare_parameter<double>("rotation_gate_min_steering_angle", 0.18);
  tuning_.rotation_gate_yaw_rate_error_threshold =
    declare_parameter<double>("rotation_gate_yaw_rate_error_threshold", 0.30);
  tuning_.rotation_gate_yaw_rate_error_release_ratio =
    declare_parameter<double>("rotation_gate_yaw_rate_error_release_ratio", 0.60);
  tuning_.rotation_gate_max_lateral_error =
    declare_parameter<double>("rotation_gate_max_lateral_error", 1.00);
  tuning_.rotation_gate_min_duration_sec =
    declare_parameter<double>("rotation_gate_min_duration_sec", 0.05);
  tuning_.rotation_gate_max_duration_sec =
    declare_parameter<double>("rotation_gate_max_duration_sec", 0.20);
  tuning_.rotation_gate_cooldown_sec =
    declare_parameter<double>("rotation_gate_cooldown_sec", 0.50);
  ga_run_id_ = declare_parameter<std::string>("ga_run_id", "");
  ga_candidate_id_ = declare_parameter<std::string>("ga_candidate_id", "");
  ga_parameter_hash_ = declare_parameter<std::string>("ga_parameter_hash", "");

  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<std_msgs::msg::String>("/pure_pursuit/debug", 10);

  const auto best_effort =
    rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", best_effort,
    [this](const Odometry::SharedPtr message) {odometry_ = message;});
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", best_effort,
    [this](const Trajectory::SharedPtr message) {trajectory_ = message;});
  sub_steering_status_ = create_subscription<SteeringReport>(
    "input/steering_status", best_effort,
    [this](const SteeringReport::SharedPtr message) {steering_status_ = message;});

  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&SimplePurePursuit::onSetParameters, this, std::placeholders::_1));
  srv_set_enabled_ = create_service<std_srvs::srv::SetBool>(
    "~/ga/set_enabled",
    std::bind(
      &SimplePurePursuit::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2));
  srv_reset_state_ = create_service<std_srvs::srv::Trigger>(
    "~/ga/reset_state",
    std::bind(
      &SimplePurePursuit::onResetState, this, std::placeholders::_1, std::placeholders::_2));

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult SimplePurePursuit::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  if (!ga_experiment_mode_) {
    result.reason = "runtime tuning requires ga_experiment_mode=true";
    return result;
  }
  if (controller_enabled_) {
    result.reason = "disable the controller before changing a candidate";
    return result;
  }

  TuningParameters candidate = tuning_;
  std::string run_id = ga_run_id_;
  std::string candidate_id = ga_candidate_id_;
  std::string parameter_hash = ga_parameter_hash_;
  try {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "lookahead_gain") {
        candidate.lookahead_gain = parameter.as_double();
      } else if (name == "lookahead_min_distance") {
        candidate.lookahead_min_distance = parameter.as_double();
      } else if (name == "actual_lookahead_distance_blend") {
        candidate.actual_lookahead_distance_blend = parameter.as_double();
      } else if (name == "dual_preview_near_ratio") {
        candidate.dual_preview_near_ratio = parameter.as_double();
      } else if (name == "dual_preview_blend") {
        candidate.dual_preview_blend = parameter.as_double();
      } else if (name == "steering_tire_angle_gain") {
        candidate.steering_tire_angle_gain = parameter.as_double();
      } else if (name == "curvature_lookahead_min_distance") {
        candidate.curvature_lookahead_min_distance = parameter.as_double();
      } else if (name == "curvature_lookahead_sensitivity") {
        candidate.curvature_lookahead_sensitivity = parameter.as_double();
      } else if (name == "curvature_lookahead_smoothing_alpha") {
        candidate.curvature_lookahead_smoothing_alpha = parameter.as_double();
      } else if (name == "max_lateral_acceleration") {
        candidate.max_lateral_acceleration = parameter.as_double();
      } else if (name == "minimum_corner_speed") {
        candidate.minimum_corner_speed = parameter.as_double();
      } else if (name == "corner_speed_retention") {
        candidate.corner_speed_retention = parameter.as_double();
      } else if (name == "lateral_error_speed_gate_enabled") {
        candidate.lateral_error_speed_gate_enabled = parameter.as_bool();
      } else if (name == "corner_speed_retention_lateral_error_soft") {
        candidate.corner_speed_retention_lateral_error_soft = parameter.as_double();
      } else if (name == "corner_speed_retention_lateral_error_hard") {
        candidate.corner_speed_retention_lateral_error_hard = parameter.as_double();
      } else if (name == "curvature_speed_preview_distance") {
        candidate.curvature_speed_preview_distance = parameter.as_double();
      } else if (name == "speed_proportional_gain") {
        candidate.speed_proportional_gain = parameter.as_double();
      } else if (name == "longitudinal_acceleration_limit") {
        candidate.longitudinal_acceleration_limit = parameter.as_double();
      } else if (name == "external_target_vel") {
        candidate.external_target_vel = parameter.as_double();
      } else if (name == "continuous_preview_interpolation_enabled") {
        candidate.continuous_preview_interpolation_enabled = parameter.as_bool();
      } else if (name == "delay_compensation_enabled") {
        candidate.delay_compensation_enabled = parameter.as_bool();
      } else if (name == "pp_control_delay_sec") {
        candidate.pp_control_delay_sec = parameter.as_double();
      } else if (name == "pp_prediction_dt_sec") {
        candidate.pp_prediction_dt_sec = parameter.as_double();
      } else if (name == "steering_time_constant_sec") {
        candidate.steering_time_constant_sec = parameter.as_double();
      } else if (name == "steering_status_timeout_sec") {
        candidate.steering_status_timeout_sec = parameter.as_double();
      } else if (name == "min_velocity_for_delay_compensation_mps") {
        candidate.min_velocity_for_delay_compensation_mps = parameter.as_double();
      } else if (name == "curvature_feedforward_enabled") {
        candidate.curvature_feedforward_enabled = parameter.as_bool();
      } else if (name == "curvature_feedforward_gain") {
        candidate.curvature_feedforward_gain = parameter.as_double();
      } else if (name == "exit_unwind_enabled") {
        candidate.exit_unwind_enabled = parameter.as_bool();
      } else if (name == "exit_unwind_far_preview_boost") {
        candidate.exit_unwind_far_preview_boost = parameter.as_double();
      } else if (name == "exit_unwind_curvature_drop_threshold") {
        candidate.exit_unwind_curvature_drop_threshold = parameter.as_double();
      } else if (name == "rotation_gate_enabled") {
        candidate.rotation_gate_enabled = parameter.as_bool();
      } else if (name == "rotation_gate_min_curvature") {
        candidate.rotation_gate_min_curvature = parameter.as_double();
      } else if (name == "rotation_gate_min_steering_angle") {
        candidate.rotation_gate_min_steering_angle = parameter.as_double();
      } else if (name == "rotation_gate_yaw_rate_error_threshold") {
        candidate.rotation_gate_yaw_rate_error_threshold = parameter.as_double();
      } else if (name == "rotation_gate_yaw_rate_error_release_ratio") {
        candidate.rotation_gate_yaw_rate_error_release_ratio = parameter.as_double();
      } else if (name == "rotation_gate_max_lateral_error") {
        candidate.rotation_gate_max_lateral_error = parameter.as_double();
      } else if (name == "rotation_gate_min_duration_sec") {
        candidate.rotation_gate_min_duration_sec = parameter.as_double();
      } else if (name == "rotation_gate_max_duration_sec") {
        candidate.rotation_gate_max_duration_sec = parameter.as_double();
      } else if (name == "rotation_gate_cooldown_sec") {
        candidate.rotation_gate_cooldown_sec = parameter.as_double();
      } else if (name == "ga_run_id") {
        run_id = parameter.as_string();
      } else if (name == "ga_candidate_id") {
        candidate_id = parameter.as_string();
      } else if (name == "ga_parameter_hash") {
        parameter_hash = parameter.as_string();
      } else {
        result.reason = "parameter is immutable during a GA run: " + name;
        return result;
      }
    }
  } catch (const rclcpp::ParameterTypeException & error) {
    result.reason = error.what();
    return result;
  }

  if (!finiteInRange(candidate.lookahead_gain, 0.05, 3.0) ||
    !finiteInRange(candidate.lookahead_min_distance, 0.5, 20.0) ||
    !finiteInRange(candidate.actual_lookahead_distance_blend, 0.0, 1.0) ||
    !finiteInRange(candidate.dual_preview_near_ratio, 0.1, 1.0) ||
    !finiteInRange(candidate.dual_preview_blend, 0.0, 1.0) ||
    !finiteInRange(candidate.steering_tire_angle_gain, 0.1, 3.0) ||
    !finiteInRange(candidate.curvature_lookahead_min_distance, 0.5, 20.0) ||
    !finiteInRange(candidate.curvature_lookahead_sensitivity, 0.0, 100.0) ||
    !finiteInRange(candidate.curvature_lookahead_smoothing_alpha, 0.0, 1.0) ||
    !finiteInRange(candidate.max_lateral_acceleration, 0.5, 20.0) ||
    !finiteInRange(candidate.minimum_corner_speed, 0.0, candidate.external_target_vel) ||
    !finiteInRange(candidate.corner_speed_retention, 0.0, 1.0) ||
    !finiteInRange(candidate.corner_speed_retention_lateral_error_soft, 0.0, 5.0) ||
    !finiteInRange(candidate.corner_speed_retention_lateral_error_hard, 0.0, 5.0) ||
    candidate.corner_speed_retention_lateral_error_hard <=
    candidate.corner_speed_retention_lateral_error_soft ||
    !finiteInRange(candidate.curvature_speed_preview_distance, 1.0, 50.0) ||
    !finiteInRange(candidate.speed_proportional_gain, 0.0, 10.0) ||
    !finiteInRange(candidate.longitudinal_acceleration_limit, 0.0, 1.0) ||
    !finiteInRange(candidate.external_target_vel, 0.0, 30.0) ||
    !finiteInRange(candidate.pp_control_delay_sec, 0.0, 1.0) ||
    !finiteInRange(candidate.pp_prediction_dt_sec, 0.001, 0.10) ||
    !finiteInRange(candidate.steering_time_constant_sec, 0.001, 2.0) ||
    !finiteInRange(candidate.steering_status_timeout_sec, 0.01, 2.0) ||
    !finiteInRange(candidate.min_velocity_for_delay_compensation_mps, 0.0, 10.0) ||
    !finiteInRange(candidate.curvature_feedforward_gain, 0.0, 2.0) ||
    !finiteInRange(candidate.exit_unwind_far_preview_boost, 0.0, 1.0) ||
    !finiteInRange(candidate.exit_unwind_curvature_drop_threshold, 0.001, 1.0) ||
    !finiteInRange(candidate.rotation_gate_min_curvature, 0.001, 1.0) ||
    !finiteInRange(candidate.rotation_gate_min_steering_angle, 0.01, 1.0) ||
    !finiteInRange(candidate.rotation_gate_yaw_rate_error_threshold, 0.01, 10.0) ||
    !finiteInRange(candidate.rotation_gate_yaw_rate_error_release_ratio, 0.1, 1.0) ||
    !finiteInRange(candidate.rotation_gate_max_lateral_error, 0.1, 5.0) ||
    !finiteInRange(candidate.rotation_gate_min_duration_sec, 0.01, 1.0) ||
    !finiteInRange(candidate.rotation_gate_max_duration_sec, 0.01, 2.0) ||
    candidate.rotation_gate_max_duration_sec < candidate.rotation_gate_min_duration_sec ||
    !finiteInRange(candidate.rotation_gate_cooldown_sec, 0.0, 5.0))
  {
    result.reason = "candidate contains a non-finite or out-of-range value";
    return result;
  }

  tuning_ = candidate;
  ga_run_id_ = run_id;
  ga_candidate_id_ = candidate_id;
  ga_parameter_hash_ = parameter_hash;
  result.successful = true;
  result.reason = "candidate applied atomically";
  return result;
}

void SimplePurePursuit::onSetEnabled(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!ga_experiment_mode_) {
    response->success = false;
    response->message = "service requires ga_experiment_mode=true";
    return;
  }
  controller_enabled_ = request->data;
  if (!controller_enabled_) {
    pub_cmd_->publish(zeroCommand(get_clock()->now()));
  }
  response->success = true;
  response->message = controller_enabled_ ? "controller enabled" : "controller disabled";
}

void SimplePurePursuit::onResetState(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!ga_experiment_mode_ || controller_enabled_) {
    response->success = false;
    response->message = "reset requires GA mode with the controller disabled";
    return;
  }
  resetExperimentState();
  response->success = true;
  response->message = "controller history reset";
}

void SimplePurePursuit::resetExperimentState()
{
  smoothed_curvature_ = 0.0;
  curvature_initialized_ = false;
  previous_steering_ = 0.0;
  rotation_gate_active_ = false;
  rotation_gate_started_sec_ = 0.0;
  rotation_gate_cooldown_until_sec_ = 0.0;
  ++reset_epoch_;
}

bool SimplePurePursuit::subscribeMessageAvailable() const
{
  return odometry_ && trajectory_ && !trajectory_->points.empty();
}

double SimplePurePursuit::estimateCurvature(const std::size_t nearest_index) const
{
  const auto & points = trajectory_->points;
  if (points.size() < 3) {
    return 0.0;
  }
  const std::size_t first = nearest_index > 1 ? nearest_index - 1 : 0;
  const std::size_t last = std::min(points.size() - 1, nearest_index + 4);
  const std::size_t middle = (first + last) / 2;
  const auto & a = points[first].pose.position;
  const auto & b = points[middle].pose.position;
  const auto & c = points[last].pose.position;
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double denominator = ab * bc * ca;
  if (denominator <= 1.0e-6) {
    return 0.0;
  }
  const double twice_area =
    std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  return 2.0 * twice_area / denominator;
}

double SimplePurePursuit::estimateSignedCurvature(const std::size_t nearest_index) const
{
  const auto & points = trajectory_->points;
  if (points.size() < 3) {
    return 0.0;
  }
  const std::size_t first = nearest_index > 1 ? nearest_index - 1 : 0;
  const std::size_t last = std::min(points.size() - 1, nearest_index + 4);
  const std::size_t middle = (first + last) / 2;
  const auto & a = points[first].pose.position;
  const auto & b = points[middle].pose.position;
  const auto & c = points[last].pose.position;
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double denominator = ab * bc * ca;
  if (denominator <= 1.0e-6) {
    return 0.0;
  }
  const double twice_signed_area =
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return 2.0 * twice_signed_area / denominator;
}

double SimplePurePursuit::estimatePreviewCurvature(
  const std::size_t nearest_index, const double preview_distance) const
{
  const auto & points = trajectory_->points;
  double maximum_curvature = estimateCurvature(nearest_index);
  double distance = 0.0;
  for (std::size_t index = nearest_index + 1; index < points.size(); ++index) {
    const auto & previous = points[index - 1].pose.position;
    const auto & current = points[index].pose.position;
    distance += std::hypot(current.x - previous.x, current.y - previous.y);
    if (distance > preview_distance) {
      break;
    }
    maximum_curvature = std::max(maximum_curvature, estimateCurvature(index));
  }
  return maximum_curvature;
}

void SimplePurePursuit::onTimer()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!controller_enabled_) {
    pub_cmd_->publish(zeroCommand(get_clock()->now()));
    return;
  }
  if (!subscribeMessageAvailable()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "odometry or trajectory is not available");
    return;
  }

  const double current_velocity = odometry_->twist.twist.linear.x;
  const double current_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double current_rear_x =
    odometry_->pose.pose.position.x - wheel_base_ * 0.5 * std::cos(current_yaw);
  const double current_rear_y =
    odometry_->pose.pose.position.y - wheel_base_ * 0.5 * std::sin(current_yaw);
  double control_yaw = current_yaw;
  double control_rear_x = current_rear_x;
  double control_rear_y = current_rear_y;
  bool steering_status_fresh = false;
  bool delay_compensation_used = false;
  const rclcpp::Time now = get_clock()->now();
  if (steering_status_ && std::isfinite(steering_status_->steering_tire_angle)) {
    const rclcpp::Time steering_stamp(steering_status_->stamp);
    const double steering_age_sec = (now - steering_stamp).seconds();
    steering_status_fresh = steering_stamp.nanoseconds() > 0 &&
      steering_age_sec >= 0.0 && steering_age_sec <= tuning_.steering_status_timeout_sec;
  }

  if (tuning_.delay_compensation_enabled &&
    tuning_.pp_control_delay_sec > 0.0 &&
    current_velocity >= tuning_.min_velocity_for_delay_compensation_mps &&
    steering_status_fresh)
  {
    const EgoControlState current_state{
      current_rear_x, current_rear_y, current_yaw, current_velocity};
    const DelayedPosePrediction prediction = predictDelayedPose(
      current_state, steering_status_->steering_tire_angle, previous_steering_,
      tuning_.pp_control_delay_sec, tuning_.pp_prediction_dt_sec,
      tuning_.steering_time_constant_sec, wheel_base_);
    if (prediction.shifted) {
      control_rear_x = prediction.x;
      control_rear_y = prediction.y;
      control_yaw = prediction.yaw;
      delay_compensation_used = true;
    }
  }

  geometry_msgs::msg::Point control_position = odometry_->pose.pose.position;
  if (delay_compensation_used) {
    control_position.x = control_rear_x + wheel_base_ * 0.5 * std::cos(control_yaw);
    control_position.y = control_rear_y + wheel_base_ * 0.5 * std::sin(control_yaw);
  }
  const std::size_t nearest_index =
    motion_utils::findNearestIndex(trajectory_->points, control_position);
  const TrajectoryPoint & nearest = trajectory_->points.at(nearest_index);
  const double maximum_velocity =
    (ga_experiment_mode_ || use_external_target_vel_) ?
    tuning_.external_target_vel : nearest.longitudinal_velocity_mps;

  const double curvature = estimatePreviewCurvature(
    nearest_index, tuning_.curvature_speed_preview_distance);
  const double signed_curvature = estimateSignedCurvature(nearest_index);
  const std::size_t exit_preview_index = selectForwardTrajectoryIndex(
    *trajectory_, nearest_index, tuning_.curvature_speed_preview_distance);
  const double exit_signed_curvature = estimateSignedCurvature(exit_preview_index);
  const double alpha = tuning_.curvature_lookahead_smoothing_alpha;
  if (!curvature_initialized_) {
    smoothed_curvature_ = curvature;
    curvature_initialized_ = true;
  } else {
    smoothed_curvature_ = alpha * curvature + (1.0 - alpha) * smoothed_curvature_;
  }
  const double curvature_speed_limit = std::sqrt(
    tuning_.max_lateral_acceleration / std::max(smoothed_curvature_, 1.0e-4));
  const double curvature_target_velocity = std::min(
    maximum_velocity, std::max(tuning_.minimum_corner_speed, curvature_speed_limit));
  const double lateral_error = tier4_autoware_utils::calcLateralDeviation(
    nearest.pose, control_position);
  const double absolute_lateral_error = std::abs(lateral_error);
  const double retention_gate = tuning_.lateral_error_speed_gate_enabled ? std::clamp(
    (tuning_.corner_speed_retention_lateral_error_hard - absolute_lateral_error) /
    (tuning_.corner_speed_retention_lateral_error_hard -
    tuning_.corner_speed_retention_lateral_error_soft),
    0.0, 1.0) : 1.0;
  const double effective_corner_speed_retention =
    tuning_.corner_speed_retention * retention_gate;
  const double target_velocity =
    curvature_target_velocity +
    effective_corner_speed_retention * (maximum_velocity - curvature_target_velocity);
  const double speed_lookahead =
    tuning_.lookahead_gain * std::abs(target_velocity) + tuning_.lookahead_min_distance;
  const double lookahead_distance = std::max(
    tuning_.curvature_lookahead_min_distance,
    speed_lookahead /
    (1.0 + tuning_.curvature_lookahead_sensitivity * smoothed_curvature_));

  struct PreviewTarget
  {
    TrajectoryPoint point;
    std::size_t lower_index;
    std::size_t upper_index;
    bool endpoint_fallback;
  };
  const auto find_preview_point = [&](const double distance) {
      if (tuning_.continuous_preview_interpolation_enabled) {
        const auto interpolated = interpolateForwardTrajectoryPoint(
          *trajectory_, nearest_index, distance);
        return PreviewTarget{
          interpolated.point, interpolated.lower_index, interpolated.upper_index,
          interpolated.endpoint_fallback};
      }
      const auto point = std::find_if(
        trajectory_->points.begin() + static_cast<std::ptrdiff_t>(nearest_index),
        trajectory_->points.end(), [&](const TrajectoryPoint & candidate) {
          return std::hypot(
            candidate.pose.position.x - control_rear_x,
            candidate.pose.position.y - control_rear_y) >= distance;
        });
      const auto selected = point == trajectory_->points.end() ?
        std::prev(trajectory_->points.end()) : point;
      const std::size_t index = static_cast<std::size_t>(
        std::distance(trajectory_->points.begin(), selected));
      return PreviewTarget{*selected, index, index, point == trajectory_->points.end()};
    };
  const PreviewTarget lookahead = find_preview_point(lookahead_distance);
  const double near_preview_distance = std::max(
    0.5, lookahead_distance * tuning_.dual_preview_near_ratio);
  const PreviewTarget near_preview = find_preview_point(near_preview_distance);
  const double curvature_drop = std::max(
    0.0, std::abs(signed_curvature) - std::abs(exit_signed_curvature));
  const double exit_unwind_progress = tuning_.exit_unwind_enabled ? std::clamp(
    curvature_drop / tuning_.exit_unwind_curvature_drop_threshold, 0.0, 1.0) : 0.0;
  const double effective_dual_preview_blend = std::clamp(
    tuning_.dual_preview_blend -
    tuning_.exit_unwind_far_preview_boost * exit_unwind_progress,
    0.0, 1.0);

  AckermannControlCommand command = zeroCommand(get_clock()->now());
  command.longitudinal.speed = target_velocity;
  command.longitudinal.acceleration = std::clamp(
    tuning_.speed_proportional_gain * (target_velocity - current_velocity), 0.0,
    tuning_.longitudinal_acceleration_limit);
  const double heading_error =
    std::atan2(
    lookahead.point.pose.position.y - control_rear_y,
    lookahead.point.pose.position.x - control_rear_x) - control_yaw;
  const double actual_lookahead_distance = std::hypot(
    lookahead.point.pose.position.x - control_rear_x,
    lookahead.point.pose.position.y - control_rear_y);
  const double steering_lookahead_distance =
    (1.0 - tuning_.actual_lookahead_distance_blend) * lookahead_distance +
    tuning_.actual_lookahead_distance_blend * actual_lookahead_distance;
  const double near_heading_error =
    std::atan2(
    near_preview.point.pose.position.y - control_rear_y,
    near_preview.point.pose.position.x - control_rear_x) - control_yaw;
  const double actual_near_preview_distance = std::hypot(
    near_preview.point.pose.position.x - control_rear_x,
    near_preview.point.pose.position.y - control_rear_y);
  const double steering_near_preview_distance =
    (1.0 - tuning_.actual_lookahead_distance_blend) * near_preview_distance +
    tuning_.actual_lookahead_distance_blend * actual_near_preview_distance;
  const double far_steering_angle = std::atan2(
    2.0 * wheel_base_ * std::sin(heading_error), steering_lookahead_distance);
  const double near_steering_angle = std::atan2(
    2.0 * wheel_base_ * std::sin(near_heading_error), steering_near_preview_distance);
  const double pure_pursuit_steering_angle =
    tuning_.steering_tire_angle_gain *
    ((1.0 - effective_dual_preview_blend) * far_steering_angle +
    effective_dual_preview_blend * near_steering_angle);
  const double curvature_feedforward_angle = tuning_.curvature_feedforward_enabled ?
    tuning_.curvature_feedforward_gain * std::atan(wheel_base_ * signed_curvature) : 0.0;
  command.lateral.steering_tire_angle =
    pure_pursuit_steering_angle + curvature_feedforward_angle;

  const double measured_yaw_rate = odometry_->twist.twist.angular.z;
  const double reference_yaw_rate = current_velocity * signed_curvature;
  const double signed_yaw_rate_error =
    std::copysign(1.0, signed_curvature == 0.0 ? 1.0 : signed_curvature) *
    (reference_yaw_rate - measured_yaw_rate);
  const double gate_steering_angle = steering_status_fresh ?
    steering_status_->steering_tire_angle : 0.0;
  const bool gate_safe = steering_status_fresh &&
    absolute_lateral_error <= tuning_.rotation_gate_max_lateral_error;
  const bool gate_entry_condition = tuning_.rotation_gate_enabled && gate_safe &&
    std::abs(signed_curvature) >= tuning_.rotation_gate_min_curvature &&
    std::abs(gate_steering_angle) >= tuning_.rotation_gate_min_steering_angle &&
    signed_yaw_rate_error >= tuning_.rotation_gate_yaw_rate_error_threshold;
  const bool gate_hold_condition = gate_safe &&
    std::abs(signed_curvature) >= 0.8 * tuning_.rotation_gate_min_curvature &&
    std::abs(gate_steering_angle) >= 0.8 * tuning_.rotation_gate_min_steering_angle &&
    signed_yaw_rate_error >=
    tuning_.rotation_gate_yaw_rate_error_threshold *
    tuning_.rotation_gate_yaw_rate_error_release_ratio;
  const double now_sec = now.seconds();
  if (rotation_gate_active_) {
    const double elapsed_sec = now_sec - rotation_gate_started_sec_;
    const bool min_hold_active = elapsed_sec < tuning_.rotation_gate_min_duration_sec && gate_safe;
    if (elapsed_sec >= tuning_.rotation_gate_max_duration_sec ||
      (!min_hold_active && !gate_hold_condition))
    {
      rotation_gate_active_ = false;
      rotation_gate_cooldown_until_sec_ = now_sec + tuning_.rotation_gate_cooldown_sec;
    }
  }
  if (!rotation_gate_active_ && gate_entry_condition &&
    now_sec >= rotation_gate_cooldown_until_sec_)
  {
    rotation_gate_active_ = true;
    rotation_gate_started_sec_ = now_sec;
  }
  if (rotation_gate_active_) {
    // A rotation gate is coast only. It never requests braking.
    command.longitudinal.acceleration = 0.0;
  }

  PointStamped lookahead_message;
  lookahead_message.header.stamp = get_clock()->now();
  lookahead_message.header.frame_id = "map";
  lookahead_message.point = lookahead.point.pose.position;
  pub_lookahead_point_->publish(lookahead_message);
  pub_cmd_->publish(command);
  AckermannControlCommand raw_command = command;
  raw_command.lateral.steering_tire_angle /= tuning_.steering_tire_angle_gain;
  pub_raw_cmd_->publish(raw_command);

  std_msgs::msg::String debug;
  std::ostringstream json;
  json << "{\"candidate_id\":\"" << jsonEscape(ga_candidate_id_)
       << "\",\"parameter_hash\":\"" << jsonEscape(ga_parameter_hash_)
       << "\",\"reset_epoch\":" << reset_epoch_
       << ",\"nearest_trajectory_index\":" << nearest_index
       << ",\"continuous_preview_interpolation_enabled\":" <<
    (tuning_.continuous_preview_interpolation_enabled ? "true" : "false")
       << ",\"lookahead_lower_trajectory_index\":" << lookahead.lower_index
       << ",\"lookahead_upper_trajectory_index\":" << lookahead.upper_index
       << ",\"delay_compensation_enabled\":" <<
    (tuning_.delay_compensation_enabled ? "true" : "false")
       << ",\"delay_compensation_used\":" <<
    (delay_compensation_used ? "true" : "false")
       << ",\"steering_status_fresh\":" <<
    (steering_status_fresh ? "true" : "false")
       << ",\"control_rear_x\":" << control_rear_x
       << ",\"control_rear_y\":" << control_rear_y
       << ",\"control_yaw\":" << control_yaw
       << ",\"lateral_error_m\":" << lateral_error
       << ",\"lookahead_distance_m\":" << lookahead_distance
       << ",\"actual_lookahead_distance_m\":" << actual_lookahead_distance
       << ",\"steering_lookahead_distance_m\":" << steering_lookahead_distance
       << ",\"near_preview_distance_m\":" << near_preview_distance
       << ",\"actual_near_preview_distance_m\":" << actual_near_preview_distance
       << ",\"dual_preview_near_ratio\":" << tuning_.dual_preview_near_ratio
       << ",\"dual_preview_blend\":" << tuning_.dual_preview_blend
       << ",\"effective_dual_preview_blend\":" << effective_dual_preview_blend
       << ",\"exit_unwind_enabled\":" <<
    (tuning_.exit_unwind_enabled ? "true" : "false")
       << ",\"exit_unwind_progress\":" << exit_unwind_progress
       << ",\"far_steering_angle_rad\":" << far_steering_angle
       << ",\"near_steering_angle_rad\":" << near_steering_angle
       << ",\"pure_pursuit_steering_angle_rad\":" << pure_pursuit_steering_angle
       << ",\"curvature_feedforward_enabled\":" <<
    (tuning_.curvature_feedforward_enabled ? "true" : "false")
       << ",\"signed_curvature_1pm\":" << signed_curvature
       << ",\"curvature_feedforward_angle_rad\":" << curvature_feedforward_angle
       << ",\"curvature\":" << smoothed_curvature_
       << ",\"target_velocity_mps\":" << target_velocity
       << ",\"maximum_velocity_mps\":" << maximum_velocity
       << ",\"curvature_speed_limit_mps\":" << curvature_speed_limit
       << ",\"curvature_target_velocity_mps\":" << curvature_target_velocity
       << ",\"corner_speed_retention\":" << tuning_.corner_speed_retention
       << ",\"lateral_error_speed_gate_enabled\":" <<
    (tuning_.lateral_error_speed_gate_enabled ? "true" : "false")
       << ",\"effective_corner_speed_retention\":" << effective_corner_speed_retention
       << ",\"corner_speed_retention_gate\":" << retention_gate
       << ",\"commanded_acceleration_mps2\":" << command.longitudinal.acceleration
       << ",\"measured_yaw_rate_radps\":" << measured_yaw_rate
       << ",\"reference_yaw_rate_radps\":" << reference_yaw_rate
       << ",\"signed_yaw_rate_error_radps\":" << signed_yaw_rate_error
       << ",\"rotation_gate_enabled\":" <<
    (tuning_.rotation_gate_enabled ? "true" : "false")
       << ",\"rotation_gate_active\":" <<
    (rotation_gate_active_ ? "true" : "false")
       << ",\"rotation_gate_entry_condition\":" <<
    (gate_entry_condition ? "true" : "false")
       << ",\"speed_limited\":" << (target_velocity < maximum_velocity)
       << ",\"steering_rate_limited\":false}";
  debug.data = json.str();
  pub_debug_->publish(debug);
  previous_steering_ = command.lateral.steering_tire_angle;
}

}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
