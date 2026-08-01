#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

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
    !finiteInRange(candidate.external_target_vel, 0.0, 30.0))
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

  const std::size_t nearest_index =
    motion_utils::findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  const TrajectoryPoint & nearest = trajectory_->points.at(nearest_index);
  const double maximum_velocity =
    (ga_experiment_mode_ || use_external_target_vel_) ?
    tuning_.external_target_vel : nearest.longitudinal_velocity_mps;
  const double current_velocity = odometry_->twist.twist.linear.x;

  const double curvature = estimatePreviewCurvature(
    nearest_index, tuning_.curvature_speed_preview_distance);
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
    nearest.pose, odometry_->pose.pose.position);
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

  const double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  const double rear_x = odometry_->pose.pose.position.x - wheel_base_ * 0.5 * std::cos(yaw);
  const double rear_y = odometry_->pose.pose.position.y - wheel_base_ * 0.5 * std::sin(yaw);
  const auto find_preview_point = [&](const double distance) {
      auto point = std::find_if(
        trajectory_->points.begin() + static_cast<std::ptrdiff_t>(nearest_index),
        trajectory_->points.end(), [&](const TrajectoryPoint & candidate) {
          return std::hypot(
            candidate.pose.position.x - rear_x, candidate.pose.position.y - rear_y) >= distance;
        });
      return point == trajectory_->points.end() ? std::prev(trajectory_->points.end()) : point;
    };
  const auto lookahead = find_preview_point(lookahead_distance);
  const double near_preview_distance = std::max(
    0.5, lookahead_distance * tuning_.dual_preview_near_ratio);
  const auto near_preview = find_preview_point(near_preview_distance);

  AckermannControlCommand command = zeroCommand(get_clock()->now());
  command.longitudinal.speed = target_velocity;
  command.longitudinal.acceleration = std::clamp(
    tuning_.speed_proportional_gain * (target_velocity - current_velocity), 0.0,
    tuning_.longitudinal_acceleration_limit);
  const double heading_error =
    std::atan2(lookahead->pose.position.y - rear_y, lookahead->pose.position.x - rear_x) - yaw;
  const double actual_lookahead_distance = std::hypot(
    lookahead->pose.position.x - rear_x, lookahead->pose.position.y - rear_y);
  const double steering_lookahead_distance =
    (1.0 - tuning_.actual_lookahead_distance_blend) * lookahead_distance +
    tuning_.actual_lookahead_distance_blend * actual_lookahead_distance;
  const double near_heading_error =
    std::atan2(
    near_preview->pose.position.y - rear_y, near_preview->pose.position.x - rear_x) - yaw;
  const double actual_near_preview_distance = std::hypot(
    near_preview->pose.position.x - rear_x, near_preview->pose.position.y - rear_y);
  const double steering_near_preview_distance =
    (1.0 - tuning_.actual_lookahead_distance_blend) * near_preview_distance +
    tuning_.actual_lookahead_distance_blend * actual_near_preview_distance;
  const double far_steering_angle = std::atan2(
    2.0 * wheel_base_ * std::sin(heading_error), steering_lookahead_distance);
  const double near_steering_angle = std::atan2(
    2.0 * wheel_base_ * std::sin(near_heading_error), steering_near_preview_distance);
  command.lateral.steering_tire_angle =
    tuning_.steering_tire_angle_gain *
    ((1.0 - tuning_.dual_preview_blend) * far_steering_angle +
    tuning_.dual_preview_blend * near_steering_angle);

  PointStamped lookahead_message;
  lookahead_message.header.stamp = get_clock()->now();
  lookahead_message.header.frame_id = "map";
  lookahead_message.point = lookahead->pose.position;
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
       << ",\"lateral_error_m\":" << lateral_error
       << ",\"lookahead_distance_m\":" << lookahead_distance
       << ",\"actual_lookahead_distance_m\":" << actual_lookahead_distance
       << ",\"steering_lookahead_distance_m\":" << steering_lookahead_distance
       << ",\"near_preview_distance_m\":" << near_preview_distance
       << ",\"actual_near_preview_distance_m\":" << actual_near_preview_distance
       << ",\"dual_preview_near_ratio\":" << tuning_.dual_preview_near_ratio
       << ",\"dual_preview_blend\":" << tuning_.dual_preview_blend
       << ",\"far_steering_angle_rad\":" << far_steering_angle
       << ",\"near_steering_angle_rad\":" << near_steering_angle
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
