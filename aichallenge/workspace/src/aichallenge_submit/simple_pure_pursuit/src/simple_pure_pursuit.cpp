#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include "simple_pure_pursuit/delay_compensation.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iterator>
#include <limits>
#include <sstream>
#include <utility>

namespace simple_pure_pursuit {

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

namespace {
constexpr std::size_t kMaxMpcHorizonNearestIndex = 3;
constexpr double kMpcHorizonVelocityCapForwardArcM = 0.25;
constexpr std::int64_t kMaxOvertakeOverrideGeneration = 16777215;

std::optional<std::int64_t> finiteIntegerInRange(
    double value, std::int64_t minimum, std::int64_t maximum) {
  if (!std::isfinite(value) || std::trunc(value) != value ||
      value < static_cast<double>(minimum) ||
      value > static_cast<double>(maximum)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(value);
}

double trajectoryArcLength(const Trajectory &trajectory) {
  if (trajectory.points.size() < 2) {
    return 0.0;
  }

  double length_m = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto &prev = trajectory.points[i - 1].pose.position;
    const auto &curr = trajectory.points[i].pose.position;
    length_m += std::hypot(curr.x - prev.x, curr.y - prev.y);
  }
  return length_m;
}

bool trajectoryValuesValid(const Trajectory &trajectory) {
  for (const auto &point : trajectory.points) {
    const auto &pos = point.pose.position;
    const auto &orientation = point.pose.orientation;
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) ||
        !std::isfinite(pos.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        point.longitudinal_velocity_mps < 0.0) {
      return false;
    }
  }
  return true;
}

double firstPointDistanceToEgo(const Trajectory &trajectory,
                               const Odometry &odometry) {
  if (trajectory.points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &first = trajectory.points.front().pose.position;
  const auto &ego = odometry.pose.pose.position;
  return std::hypot(first.x - ego.x, first.y - ego.y);
}

std::optional<std::string> jsonStringField(const std::string &json,
                                           const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto value_start = json.find('"', colon_pos + 1);
  if (value_start == std::string::npos) {
    return std::nullopt;
  }
  const auto value_end = json.find('"', value_start + 1);
  if (value_end == std::string::npos || value_end <= value_start) {
    return std::nullopt;
  }
  return json.substr(value_start + 1, value_end - value_start - 1);
}

std::optional<double> jsonNumberField(const std::string &json,
                                      const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }

  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() &&
         std::isspace(static_cast<unsigned char>(json[value_start]))) {
    ++value_start;
  }
  std::size_t value_end = value_start;
  while (value_end < json.size()) {
    const char c = json[value_end];
    if (!(std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' ||
          c == '.' || c == 'e' || c == 'E')) {
      break;
    }
    ++value_end;
  }
  if (value_end == value_start) {
    return std::nullopt;
  }

  try {
    return std::stod(json.substr(value_start, value_end - value_start));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::int64_t> jsonIntegerField(const std::string &json,
                                             const std::string &key) {
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  std::size_t value_start = colon_pos + 1;
  while (value_start < json.size() &&
         std::isspace(static_cast<unsigned char>(json[value_start]))) {
    ++value_start;
  }
  std::size_t value_end = value_start;
  if (value_end < json.size() && json[value_end] == '-') {
    ++value_end;
  }
  while (value_end < json.size() &&
         std::isdigit(static_cast<unsigned char>(json[value_end]))) {
    ++value_end;
  }
  if (value_end == value_start ||
      (json[value_start] == '-' && value_end == value_start + 1)) {
    return std::nullopt;
  }
  if (value_end < json.size() &&
      (json[value_end] == '.' || json[value_end] == 'e' ||
       json[value_end] == 'E')) {
    return std::nullopt;
  }
  try {
    return std::stoll(json.substr(value_start, value_end - value_start));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

} // namespace

SimplePurePursuit::SimplePurePursuit()
    : Node("simple_pure_pursuit"),
      // initialize parameters
      wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
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
      max_odom_age_sec_(declare_parameter<float>("max_odom_age_sec", 0.20)),
      max_trajectory_age_sec_(
          declare_parameter<float>("max_trajectory_age_sec", 0.50)),
      max_override_age_sec_(
          declare_parameter<float>("max_override_age_sec", 0.50)),
      stop_on_stale_input_(
          declare_parameter<bool>("stop_on_stale_input", true)),
      diagnostic_throttle_sec_(
          declare_parameter<float>("diagnostic_throttle_sec", 1.0)),
      use_mpc_predicted_horizon_(
          declare_parameter<bool>("use_mpc_predicted_horizon", false)),
      max_mpc_horizon_age_sec_(
          declare_parameter<float>("max_mpc_horizon_age_sec", 0.15)),
      min_mpc_horizon_points_(
          declare_parameter<int>("min_mpc_horizon_points", 5)),
      max_mpc_horizon_start_distance_m_(
          declare_parameter<float>("max_mpc_horizon_start_distance_m", 2.0)),
      min_mpc_horizon_arc_length_m_(
          declare_parameter<float>("min_mpc_horizon_arc_length_m", 2.0)),
      require_solved_mpc_health_for_horizon_(declare_parameter<bool>(
          "require_solved_mpc_health_for_horizon", false)),
      max_mpc_health_age_sec_(
          declare_parameter<float>("max_mpc_health_age_sec", 0.30)),
      require_matching_overtake_horizon_contract_(declare_parameter<bool>(
          "require_matching_overtake_horizon_contract", false)),
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
          "curvature_lookahead_smoothing_alpha", 0.35)),
      pp_control_delay_sec_(
          declare_parameter<float>("pp_control_delay_sec", 0.0)),
      pp_prediction_dt_sec_(
          declare_parameter<float>("pp_prediction_dt_sec", 0.02)),
      steering_time_constant_sec_(
          declare_parameter<float>("steering_time_constant_sec", 0.30)),
      steering_status_timeout_sec_(
          declare_parameter<float>("steering_status_timeout_sec", 0.20)),
      min_velocity_for_delay_compensation_mps_(declare_parameter<float>(
          "min_velocity_for_delay_compensation_mps", 0.20)),
      horizon_curvature_feedforward_gain_(
          declare_parameter<float>("horizon_curvature_feedforward_gain", 0.0)),
      horizon_curvature_feedforward_max_rad_(declare_parameter<float>(
          "horizon_curvature_feedforward_max_rad", 0.08)) {
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ =
      create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ =
      create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  pub_debug_ = create_publisher<String>("/pure_pursuit/debug", 1);

  const auto bv_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
      "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) {
        odometry_ = msg;
        last_odometry_receive_sec_ = steadyNowSec();
      });
  sub_trajectory_ = create_subscription<Trajectory>(
      "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) {
        trajectory_ = msg;
        last_trajectory_receive_sec_ = steadyNowSec();
      });
  sub_mpc_predicted_horizon_ = create_subscription<Trajectory>(
      "input/mpc_predicted_horizon", bv_qos,
      [this](const Trajectory::SharedPtr msg) {
        mpc_predicted_horizon_ = msg;
        last_mpc_predicted_horizon_receive_sec_ = steadyNowSec();
      });
  sub_mpc_predicted_horizon_contract_ = create_subscription<String>(
      "input/mpc_predicted_horizon_contract", bv_qos,
      [this](const String::SharedPtr msg) {
        onMpcPredictedHorizonContract(msg);
      });
  sub_overtake_override_ = create_subscription<Float32MultiArray>(
      "input/overtake_reference_override", rclcpp::QoS(1),
      [this](const Float32MultiArray::SharedPtr msg) {
        onOvertakeOverride(msg);
      });
  sub_steering_status_ = create_subscription<SteeringReport>(
      "input/steering_status", bv_qos,
      [this](const SteeringReport::SharedPtr msg) {
        latest_steering_status_rad_ = msg->steering_tire_angle;
        last_steering_status_receive_sec_ = steadyNowSec();
      });
  sub_mpc_health_ = create_subscription<String>(
      "input/mpc_health", bv_qos,
      [this](const String::SharedPtr msg) { onMpcHealth(msg); });

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
  const auto stamp = get_clock()->now();
  const double now_sec = steadyNowSec();

  // 1. 入力が古い場合は、制御計算へ進まず停止/diagnosticだけを出す。
  const auto freshness = evaluateInputFreshness(now_sec);
  if (handleInvalidFreshness(stamp, freshness)) {
    return;
  }

  // 2. 古いovertake overrideはこの周期の参照生成へ混ぜない。
  clearStaleOvertakeOverride(now_sec);
  const auto control_pose = predictControlPose(now_sec);
  const auto mpc_horizon_freshness = evaluateMpcPredictedHorizon(now_sec);

  // 3. 通常trajectory / MPC horizon / overtake override適用後trajectoryを
  //    1つのcontrol trajectoryへ正規化する。
  const auto context =
      selectControlTrajectory(control_pose, now_sec, mpc_horizon_freshness);
  if (!context.valid || context.trajectory == nullptr ||
      context.trajectory->points.empty()) {
    FreshnessResult invalid;
    invalid.reason = context.invalid_reason;
    publishStaleDebug(stamp, invalid);
    if (stop_on_stale_input_) {
      publishStopForStaleInput(stamp, invalid);
    }
    return;
  }

  AckermannControlCommand cmd = zeroAckermannControlCommand(stamp);
  const auto &nearest_traj_point =
      context.trajectory->points.at(context.nearest_index);

  // 4. 速度と横制御を別々に計算し、最後にAckermann commandへ詰める。
  const auto longitudinal = computeLongitudinalCommand(context, now_sec);
  cmd.longitudinal.speed = longitudinal.target_speed_mps;
  cmd.longitudinal.acceleration = longitudinal.acceleration_mps2;

  const auto lateral =
      computeLateralCommand(context, control_pose, longitudinal);
  publishLookaheadPoint(lateral.lookahead_point_x, lateral.lookahead_point_y,
                        nearest_traj_point.pose.position.z);
  cmd.lateral.steering_tire_angle = lateral.steering_tire_angle_rad;

  publishDebug(
      cmd.stamp, *context.trajectory, context.nearest_index,
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      longitudinal.acceleration_mps2, lateral.base_lookahead_distance_m,
      lateral.desired_lookahead_distance_m, lateral.lookahead_distance_m,
      lateral.path_curvature_1pm, lateral.signed_path_curvature_1pm,
      lateral.curvature_window_distance_m, lateral.lookahead_point_x,
      lateral.lookahead_point_y, lateral.rear_x, lateral.rear_y,
      lateral.alpha_rad, lateral.pure_pursuit_steering_tire_angle_rad,
      lateral.curvature_feedforward_steering_rad,
      lateral.raw_steering_tire_angle_rad, lateral.steering_tire_angle_rad,
      context.overtake_override_applied, overtakeLateralOffset(0),
      longitudinal.overtake_speed_cap_mps, now_sec, context.mpc_horizon_applied,
      longitudinal.mpc_horizon_velocity_cap_applied,
      longitudinal.mpc_horizon_velocity_cap_mps,
      context.evaluated_horizon_stamp_sec,
      context.evaluated_horizon_stamp_nanosec,
      context.applied_horizon_stamp_sec,
      context.applied_horizon_stamp_nanosec, context.applied_horizon_source,
      context.applied_horizon_mode_id, context.applied_horizon_generation,
      context.mpc_horizon_freshness,
      context.source, control_pose);

  pub_cmd_->publish(cmd);
  last_commanded_steering_tire_angle_ = cmd.lateral.steering_tire_angle;
  cmd.lateral.steering_tire_angle = lateral.raw_steering_tire_angle_rad;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::handleInvalidFreshness(
    const rclcpp::Time &stamp, const FreshnessResult &freshness) {
  if (freshness.fresh) {
    return false;
  }

  has_smoothed_lookahead_distance_ = false;
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      static_cast<int>(std::max(0.1, diagnostic_throttle_sec_) * 1000.0),
      "PurePursuit input invalid: %s odom_age=%.3f trajectory_age=%.3f",
      freshness.reason.c_str(), freshness.ages.odom_age_sec,
      freshness.ages.trajectory_age_sec);
  publishStaleDebug(stamp, freshness);
  if (stop_on_stale_input_) {
    publishStopForStaleInput(stamp, freshness);
  }
  return true;
}

void SimplePurePursuit::clearStaleOvertakeOverride(double now_sec) {
  if (!overtake_override_active_ || overtakeOverrideFresh(now_sec)) {
    return;
  }

  const double override_age_sec =
      inputAgeSec(last_overtake_override_sec_, now_sec);
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      static_cast<int>(std::max(0.1, diagnostic_throttle_sec_) * 1000.0),
      "PurePursuit overtake override stale: age=%.3f", override_age_sec);
  clearOvertakeOverride();
}

SimplePurePursuit::ControlTrajectoryContext
SimplePurePursuit::selectControlTrajectory(
    const ControlPosePrediction &control_pose, double now_sec,
    const HorizonFreshnessResult &mpc_horizon_freshness) {
  ControlTrajectoryContext context;
  context.mpc_horizon_freshness = mpc_horizon_freshness;
  context.mpc_horizon_applied = mpc_horizon_freshness.usable;
  context.trajectory = context.mpc_horizon_applied
                           ? mpc_predicted_horizon_.get()
                           : trajectory_.get();
  context.source = context.mpc_horizon_applied ? "mpc_horizon" : "trajectory";
  if (mpc_predicted_horizon_ != nullptr) {
    context.evaluated_horizon_stamp_sec =
        mpc_predicted_horizon_->header.stamp.sec;
    context.evaluated_horizon_stamp_nanosec =
        mpc_predicted_horizon_->header.stamp.nanosec;
  }
  if (context.mpc_horizon_applied && mpc_predicted_horizon_ != nullptr) {
    context.applied_horizon_stamp_sec = context.evaluated_horizon_stamp_sec;
    context.applied_horizon_stamp_nanosec =
        context.evaluated_horizon_stamp_nanosec;
    if (mpc_horizon_contract_received_ &&
        context.applied_horizon_stamp_sec == mpc_horizon_contract_stamp_sec_ &&
        context.applied_horizon_stamp_nanosec ==
            mpc_horizon_contract_stamp_nanosec_) {
      context.applied_horizon_source = mpc_horizon_contract_source_;
      context.applied_horizon_mode_id = mpc_horizon_contract_mode_id_;
      context.applied_horizon_generation =
          mpc_horizon_contract_generation_;
    }
  }

  if (context.trajectory == nullptr || context.trajectory->points.empty()) {
    context.invalid_reason =
        context.mpc_horizon_applied ? "empty_mpc_horizon" : "empty_trajectory";
    return context;
  }

  context.nearest_index =
      findNearestIndex(context.trajectory->points, control_pose.position);

  // MPC horizonを使っている周期は、horizon自体を優先する。
  // 通常trajectory周期だけ、plannerから来るovertake overrideを重ねる。
  if (!context.mpc_horizon_applied && overtakeOverrideFresh(now_sec)) {
    context.owned_trajectory = std::make_shared<Trajectory>(*trajectory_);
    context.overtake_override_applied = applyOvertakeOverride(
        *context.owned_trajectory, context.nearest_index, now_sec);
    if (context.overtake_override_applied) {
      context.trajectory = context.owned_trajectory.get();
      context.source = "trajectory_overtake_override";
      context.nearest_index =
          findNearestIndex(context.trajectory->points, control_pose.position);
    }
  } else if (!context.mpc_horizon_applied && overtake_override_active_) {
    clearOvertakeOverride();
  }

  context.valid = true;
  return context;
}

SimplePurePursuit::LongitudinalCommand
SimplePurePursuit::computeLongitudinalCommand(
    const ControlTrajectoryContext &context, double now_sec) const {
  LongitudinalCommand result;
  const auto &nearest = context.trajectory->points.at(context.nearest_index);

  result.target_speed_mps = use_external_target_vel_
                                ? external_target_vel_
                                : nearest.longitudinal_velocity_mps;
  result.current_speed_mps = odometry_->twist.twist.linear.x;

  // neutral horizonの先頭点は、現在姿勢アンカーとして低速/0mpsに
  // なることがあるため、少し前方の速度capを見る。
  const bool skip_mpc_horizon_anchor_speed =
      mpc_predicted_horizon_source_ == "neutral_reference" ||
      mpc_predicted_horizon_source_ == "fixed_neutral_reference";
  const auto cap_index = context.mpc_horizon_applied
                             ? selectMpcHorizonVelocityCapIndex(
                                   *context.trajectory, context.nearest_index,
                                   kMpcHorizonVelocityCapForwardArcM,
                                   skip_mpc_horizon_anchor_speed)
                             : context.nearest_index;
  result.mpc_horizon_velocity_cap_mps =
      context.mpc_horizon_applied
          ? context.trajectory->points.at(cap_index).longitudinal_velocity_mps
          : -1.0;
  if (context.mpc_horizon_applied &&
      std::isfinite(result.mpc_horizon_velocity_cap_mps) &&
      result.mpc_horizon_velocity_cap_mps >= 0.0) {
    result.mpc_horizon_velocity_cap_applied =
        result.mpc_horizon_velocity_cap_mps < result.target_speed_mps;
    result.target_speed_mps =
        std::min(result.target_speed_mps, result.mpc_horizon_velocity_cap_mps);
  }

  if (const auto overtake_speed_cap = overtakeSpeedCap(0, now_sec)) {
    result.overtake_speed_cap_mps = overtake_speed_cap.value();
    result.target_speed_mps =
        applyOvertakeSpeedCap(result.target_speed_mps, overtake_speed_cap);
  }

  result.acceleration_mps2 =
      proportionalLongitudinalAcceleration(result.target_speed_mps,
                                            result.current_speed_mps,
                                            speed_proportional_gain_);
  return result;
}

SimplePurePursuit::LateralCommand SimplePurePursuit::computeLateralCommand(
    const ControlTrajectoryContext &context,
    const ControlPosePrediction &control_pose,
    const LongitudinalCommand &longitudinal) {
  LateralCommand result;
  const auto &trajectory = *context.trajectory;

  const LookaheadParams lookahead_params{
      lookahead_gain_, lookahead_min_distance_,
      curvature_adaptive_lookahead_enabled_, curvature_lookahead_min_distance_,
      curvature_lookahead_sensitivity_};
  result.base_lookahead_distance_m = speedBasedLookaheadDistance(
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      lookahead_params);

  // 曲率を見る距離は、速度由来のlookaheadを基準にしつつ上限を持たせる。
  // 直線では遠く、コーナーでは手前を見るための前処理。
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
          : result.base_lookahead_distance_m;
  result.curvature_window_distance_m =
      std::min(curvature_window_max_m,
               result.base_lookahead_distance_m * curvature_window_ratio);
  result.path_curvature_1pm = estimateTrajectoryCurvature(
      trajectory, context.nearest_index, result.curvature_window_distance_m,
      curvature_min_arc_m);
  result.signed_path_curvature_1pm = estimateSignedTrajectoryCurvature(
      trajectory, context.nearest_index, result.curvature_window_distance_m,
      curvature_min_arc_m);
  result.desired_lookahead_distance_m = adaptiveLookaheadDistance(
      longitudinal.target_speed_mps, longitudinal.current_speed_mps,
      result.path_curvature_1pm, lookahead_params);
  result.lookahead_distance_m = result.desired_lookahead_distance_m;
  if (curvature_adaptive_lookahead_enabled_) {
    result.lookahead_distance_m = smoothLookaheadDistance(
        result.desired_lookahead_distance_m, smoothed_lookahead_distance_,
        has_smoothed_lookahead_distance_, curvature_lookahead_smoothing_alpha_);
    smoothed_lookahead_distance_ = result.lookahead_distance_m;
    has_smoothed_lookahead_distance_ = true;
  } else {
    has_smoothed_lookahead_distance_ = false;
  }

  // PurePursuitは後輪中心からlookahead点を見るため、制御姿勢から後輪中心へ戻す。
  result.rear_x =
      control_pose.position.x - wheel_base_ / 2.0 * std::cos(control_pose.yaw);
  result.rear_y =
      control_pose.position.y - wheel_base_ / 2.0 * std::sin(control_pose.yaw);
  auto lookahead_point_itr =
      std::find_if(trajectory.points.begin() + context.nearest_index,
                   trajectory.points.end(), [&](const TrajectoryPoint &point) {
                     return std::hypot(point.pose.position.x - result.rear_x,
                                       point.pose.position.y - result.rear_y) >=
                            result.lookahead_distance_m;
                   });
  if (lookahead_point_itr == trajectory.points.end()) {
    lookahead_point_itr = std::prev(trajectory.points.end());
  }
  result.lookahead_point_x = lookahead_point_itr->pose.position.x;
  result.lookahead_point_y = lookahead_point_itr->pose.position.y;

  result.alpha_rad =
      normalizeAngle(std::atan2(result.lookahead_point_y - result.rear_y,
                                result.lookahead_point_x - result.rear_x) -
                     control_pose.yaw);
  result.pure_pursuit_steering_tire_angle_rad =
      std::atan2(2.0 * wheel_base_ * std::sin(result.alpha_rad),
                 result.lookahead_distance_m);
  result.curvature_feedforward_steering_rad =
      std::clamp(horizon_curvature_feedforward_gain_ *
                     std::atan(wheel_base_ * result.signed_path_curvature_1pm),
                 -std::max(0.0, horizon_curvature_feedforward_max_rad_),
                 std::max(0.0, horizon_curvature_feedforward_max_rad_));
  result.raw_steering_tire_angle_rad =
      result.pure_pursuit_steering_tire_angle_rad +
      result.curvature_feedforward_steering_rad;
  result.steering_tire_angle_rad =
      steering_tire_angle_gain_ * result.raw_steering_tire_angle_rad;
  return result;
}

void SimplePurePursuit::publishLookaheadPoint(double x, double y, double z) {
  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = x;
  lookahead_point_msg.point.y = y;
  lookahead_point_msg.point.z = z;
  pub_lookahead_point_->publish(lookahead_point_msg);
}

double SimplePurePursuit::steadyNowSec() const {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

SimplePurePursuit::ControlPosePrediction
SimplePurePursuit::predictControlPose(double now_sec) const {
  ControlPosePrediction result;
  if (!odometry_) {
    result.steering_source = "missing_odom";
    return result;
  }

  result.position = odometry_->pose.pose.position;
  result.yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  result.velocity_mps = odometry_->twist.twist.linear.x;

  double steering_age_sec = -1.0;
  const auto [current_steering_rad, steering_source] =
      estimateCurrentSteering(now_sec, &steering_age_sec);
  result.current_steering_rad = current_steering_rad;
  result.applied_steering_rad = current_steering_rad;
  result.steering_age_sec = steering_age_sec;
  result.steering_source = steering_source;

  const bool delay_enabled =
      std::isfinite(pp_control_delay_sec_) && pp_control_delay_sec_ > 0.0;
  const bool steering_fresh = steering_source == "steering_status";
  const double min_compensation_velocity_mps =
      std::max(0.0, min_velocity_for_delay_compensation_mps_);
  const bool velocity_valid =
      std::isfinite(result.velocity_mps) &&
      result.velocity_mps >= min_compensation_velocity_mps;
  if (!delay_enabled || !steering_fresh || !velocity_valid) {
    return result;
  }

  const EgoControlState state{result.position.x, result.position.y, result.yaw,
                              result.velocity_mps};
  const auto predicted = predictDelayedPose(
      state, current_steering_rad, last_commanded_steering_tire_angle_,
      pp_control_delay_sec_, pp_prediction_dt_sec_, steering_time_constant_sec_,
      wheel_base_);
  result.position.x = predicted.x;
  result.position.y = predicted.y;
  result.yaw = predicted.yaw;
  result.applied_steering_rad = predicted.applied_steering_rad;
  result.prediction_steps = predicted.prediction_steps;
  result.shifted = predicted.shifted;
  return result;
}

std::pair<double, std::string>
SimplePurePursuit::estimateCurrentSteering(double now_sec,
                                           double *steering_age_sec) const {
  if (last_steering_status_receive_sec_.has_value()) {
    const double age_sec =
        inputAgeSec(last_steering_status_receive_sec_, now_sec);
    if (steering_age_sec != nullptr) {
      *steering_age_sec = age_sec;
    }
    if (ageFresh(age_sec, steering_status_timeout_sec_) &&
        std::isfinite(latest_steering_status_rad_)) {
      return {latest_steering_status_rad_, "steering_status"};
    }
    return {last_commanded_steering_tire_angle_, "stale_steering_status"};
  }

  if (steering_age_sec != nullptr) {
    *steering_age_sec = -1.0;
  }
  return {last_commanded_steering_tire_angle_, "missing_steering_status"};
}

FreshnessResult
SimplePurePursuit::evaluateInputFreshness(double now_sec) const {
  FreshnessResult result;
  result.ages.odom_age_sec = inputAgeSec(last_odometry_receive_sec_, now_sec);
  result.ages.trajectory_age_sec =
      inputAgeSec(last_trajectory_receive_sec_, now_sec);
  result.ages.override_age_sec =
      overtake_override_active_
          ? inputAgeSec(last_overtake_override_sec_, now_sec)
          : -1.0;

  if (!last_odometry_receive_sec_.has_value() || !odometry_) {
    result.reason = "missing_odom";
    return result;
  }
  if (!ageFresh(result.ages.odom_age_sec, max_odom_age_sec_)) {
    result.reason = "stale_odom";
    return result;
  }

  const bool trajectory_time_fresh =
      last_trajectory_receive_sec_.has_value() &&
      ageFresh(result.ages.trajectory_age_sec, max_trajectory_age_sec_);
  if (trajectory_ && !trajectory_->points.empty() && trajectory_time_fresh) {
    result.fresh = true;
    result.reason = "fresh";
    return result;
  }

  const auto horizon = evaluateMpcPredictedHorizon(now_sec);
  if (horizon.usable) {
    result.fresh = true;
    result.reason = "fresh_mpc_horizon";
    return result;
  }

  if (!last_trajectory_receive_sec_.has_value() || !trajectory_) {
    result.reason = "missing_trajectory";
    return result;
  }
  if (trajectory_->points.empty()) {
    result.reason = "empty_trajectory";
    return result;
  }
  if (!trajectory_time_fresh) {
    result.reason = "stale_trajectory";
    return result;
  }
  return result;
}

HorizonFreshnessResult
SimplePurePursuit::evaluateMpcPredictedHorizon(double now_sec) const {
  if (!odometry_) {
    HorizonFreshnessResult result;
    result.reason = "missing_odom";
    result.age_sec =
        inputAgeSec(last_mpc_predicted_horizon_receive_sec_, now_sec);
    return result;
  }

  const std::size_t point_count =
      mpc_predicted_horizon_ ? mpc_predicted_horizon_->points.size() : 0;
  const std::size_t min_points =
      static_cast<std::size_t>(std::max(1, min_mpc_horizon_points_));
  const bool frame_valid = mpc_predicted_horizon_ &&
                           mpc_predicted_horizon_->header.frame_id == "map";
  const bool values_valid =
      mpc_predicted_horizon_ && trajectoryValuesValid(*mpc_predicted_horizon_);
  const double start_distance_m =
      mpc_predicted_horizon_
          ? firstPointDistanceToEgo(*mpc_predicted_horizon_, *odometry_)
          : -1.0;
  const double arc_length_m = mpc_predicted_horizon_
                                  ? trajectoryArcLength(*mpc_predicted_horizon_)
                                  : -1.0;

  auto result = evaluateMpcHorizonFreshness(
      use_mpc_predicted_horizon_, last_mpc_predicted_horizon_receive_sec_,
      now_sec, max_mpc_horizon_age_sec_, point_count, min_points,
      start_distance_m, max_mpc_horizon_start_distance_m_, arc_length_m,
      min_mpc_horizon_arc_length_m_, values_valid, frame_valid);

  if (result.usable && require_solved_mpc_health_for_horizon_) {
    const double health_age_sec = mpcHealthAgeSec(now_sec);
    if (!last_mpc_health_receive_sec_.has_value()) {
      result.usable = false;
      result.reason = "mpc_health_missing";
    } else if (!ageFresh(health_age_sec, max_mpc_health_age_sec_)) {
      result.usable = false;
      result.reason = "mpc_health_stale";
    } else if (mpc_health_status_ != "solved" ||
               mpc_health_infeasible_count_ > 0) {
      result.usable = false;
      result.reason = mpc_health_infeasible_count_ > 0
                          ? "mpc_health_infeasible"
                          : "mpc_health_" + mpc_health_status_;
    }
  }

  if (result.usable) {
    const bool override_active = overtakeOverrideFresh(now_sec);
    const bool stamp_matches =
        mpc_predicted_horizon_ && mpc_horizon_contract_received_ &&
        mpc_predicted_horizon_->header.stamp.sec ==
            mpc_horizon_contract_stamp_sec_ &&
        mpc_predicted_horizon_->header.stamp.nanosec ==
            mpc_horizon_contract_stamp_nanosec_;
    const auto contract = evaluateMpcHorizonContract(
        require_matching_overtake_horizon_contract_, override_active,
        overtake_mode_id_, overtake_override_generation_,
        mpc_horizon_contract_received_,
        last_mpc_predicted_horizon_contract_receive_sec_, now_sec,
        max_mpc_horizon_age_sec_, stamp_matches,
        mpc_horizon_contract_source_, mpc_horizon_contract_mode_id_,
        mpc_horizon_contract_generation_);
    if (!contract.usable) {
      result.usable = false;
      result.reason = contract.reason;
    }
  }

  if (result.usable) {
    const auto nearest_idx = findNearestIndex(mpc_predicted_horizon_->points,
                                              odometry_->pose.pose.position);
    if (nearest_idx > kMaxMpcHorizonNearestIndex) {
      result.usable = false;
      result.reason = "nearest_index";
    }
  }

  return result;
}

double SimplePurePursuit::mpcHealthAgeSec(double now_sec) const {
  return inputAgeSec(last_mpc_health_receive_sec_, now_sec);
}

void SimplePurePursuit::publishStopForStaleInput(
    const rclcpp::Time &stamp, const FreshnessResult &freshness) {
  (void)freshness;
  auto cmd = zeroAckermannControlCommand(stamp);
  cmd.longitudinal.acceleration = -1.5;
  pub_cmd_->publish(cmd);
  pub_raw_cmd_->publish(cmd);
}

void SimplePurePursuit::publishStaleDebug(const rclcpp::Time &stamp,
                                          const FreshnessResult &freshness) {
  (void)stamp;
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }
  const double now_sec = steadyNowSec();
  if (now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = now_sec;
  const auto horizon = evaluateMpcPredictedHorizon(now_sec);

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"stale_input\":true,"
       << "\"stale_reason\":\"" << freshness.reason << "\","
       << "\"stop_on_stale_input\":"
       << (stop_on_stale_input_ ? "true" : "false") << ","
       << "\"odom_age_sec\":" << freshness.ages.odom_age_sec << ","
       << "\"trajectory_age_sec\":" << freshness.ages.trajectory_age_sec << ","
       << "\"override_age_sec\":" << freshness.ages.override_age_sec << ","
       << "\"mpc_horizon_age_sec\":" << horizon.age_sec << ","
       << "\"mpc_horizon_points\":" << horizon.point_count << ","
       << "\"mpc_health_required_for_horizon\":"
       << (require_solved_mpc_health_for_horizon_ ? "true" : "false") << ","
       << "\"mpc_health_status\":\"" << mpc_health_status_ << "\","
       << "\"mpc_health_age_sec\":" << mpcHealthAgeSec(now_sec) << ","
       << "\"mpc_infeasible_count\":" << mpc_health_infeasible_count_ << ","
       << "\"mpc_predicted_horizon_source\":\"" << mpc_predicted_horizon_source_
       << "\","
       << "\"mpc_horizon_contract_required\":"
       << (require_matching_overtake_horizon_contract_ ? "true" : "false")
       << ","
       << "\"mpc_horizon_contract_received\":"
       << (mpc_horizon_contract_received_ ? "true" : "false") << ","
       << "\"mpc_horizon_contract_age_sec\":"
       << inputAgeSec(last_mpc_predicted_horizon_contract_receive_sec_, now_sec)
       << ","
       << "\"mpc_horizon_contract_source\":\""
       << mpc_horizon_contract_source_ << "\","
       << "\"mpc_horizon_contract_mode_id\":"
       << mpc_horizon_contract_mode_id_ << ","
       << "\"mpc_horizon_contract_generation\":"
       << mpc_horizon_contract_generation_ << ","
       << "\"overtake_override_generation\":"
       << overtake_override_generation_ << ","
       << "\"mpc_horizon_reject_reason\":\"" << horizon.reason << "\"}";

  String msg;
  msg.data = json.str();
  pub_debug_->publish(msg);
}

void SimplePurePursuit::onOvertakeOverride(
    const Float32MultiArray::SharedPtr msg) {
  if (!use_overtake_reference_override_) {
    return;
  }

  const double now_sec = steadyNowSec();
  const auto &data = msg->data;
  if (data.size() < 3) {
    clearOvertakeOverride();
    return;
  }

  const auto valid = finiteIntegerInRange(data[0], 1, 1);
  const auto mode_id = finiteIntegerInRange(data[1], 0, 255);
  const auto count_value = finiteIntegerInRange(data[2], 0, 1000);
  if (!valid.has_value() || !mode_id.has_value() ||
      !count_value.has_value()) {
    clearOvertakeOverride();
    return;
  }
  const int mode_id_int = static_cast<int>(mode_id.value());
  const int n = static_cast<int>(count_value.value());
  if (n <= 0 || mode_id_int == 0) {
    clearOvertakeOverride();
    last_overtake_override_sec_ = now_sec;
    return;
  }

  const std::size_t count = static_cast<std::size_t>(n);
  const std::size_t expected = 3 + 2 * count;
  if (data.size() != expected && data.size() != expected + 2) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Malformed overtake reference override: len=%zu expected=%zu or %zu",
        data.size(), expected, expected + 2);
    clearOvertakeOverride();
    return;
  }

  overtake_lateral_offsets_.clear();
  overtake_speed_caps_.clear();
  overtake_lateral_offsets_.reserve(count);
  overtake_speed_caps_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double lateral_offset = static_cast<double>(data[3 + i]);
    const double speed_cap = static_cast<double>(data[3 + count + i]);
    if (!std::isfinite(lateral_offset) || !std::isfinite(speed_cap)) {
      clearOvertakeOverride();
      return;
    }
    overtake_lateral_offsets_.push_back(lateral_offset);
    overtake_speed_caps_.push_back(speed_cap);
  }
  overtake_override_generation_ = 0;
  if (data.size() == expected + 2) {
    const auto contract_version = finiteIntegerInRange(data[expected], 1, 1);
    const auto generation = finiteIntegerInRange(
        data[expected + 1], 1, kMaxOvertakeOverrideGeneration);
    if (!contract_version.has_value() || !generation.has_value()) {
      clearOvertakeOverride();
      return;
    }
    overtake_override_generation_ =
        static_cast<std::uint32_t>(generation.value());
  }
  overtake_mode_id_ = mode_id_int;
  overtake_override_active_ = true;
  last_overtake_override_sec_ = now_sec;
}

void SimplePurePursuit::onMpcPredictedHorizonContract(
    const String::SharedPtr msg) {
  const auto version = jsonIntegerField(msg->data, "contract_version");
  const auto stamp_sec = jsonIntegerField(msg->data, "horizon_stamp_sec");
  const auto stamp_nanosec =
      jsonIntegerField(msg->data, "horizon_stamp_nanosec");
  const auto source = jsonStringField(msg->data, "source");
  const auto mode_id = jsonIntegerField(msg->data, "mode_id");
  const auto generation =
      jsonIntegerField(msg->data, "override_generation");
  if (!version.has_value() || version.value() != 1 || !stamp_sec.has_value() ||
      !stamp_nanosec.has_value() || !source.has_value() ||
      !mode_id.has_value() || !generation.has_value() ||
      stamp_sec.value() < std::numeric_limits<std::int32_t>::min() ||
      stamp_sec.value() > std::numeric_limits<std::int32_t>::max() ||
      stamp_nanosec.value() < 0 || stamp_nanosec.value() >= 1000000000LL ||
      mode_id.value() < 0 || mode_id.value() > 255 ||
      generation.value() < 0 ||
      generation.value() > kMaxOvertakeOverrideGeneration) {
    mpc_horizon_contract_received_ = false;
    return;
  }
  mpc_horizon_contract_stamp_sec_ =
      static_cast<std::int32_t>(stamp_sec.value());
  mpc_horizon_contract_stamp_nanosec_ =
      static_cast<std::uint32_t>(stamp_nanosec.value());
  mpc_horizon_contract_source_ = *source;
  mpc_horizon_contract_mode_id_ = static_cast<int>(mode_id.value());
  mpc_horizon_contract_generation_ =
      static_cast<std::uint32_t>(generation.value());
  mpc_horizon_contract_received_ = true;
  last_mpc_predicted_horizon_contract_receive_sec_ = steadyNowSec();
}

void SimplePurePursuit::onMpcHealth(const String::SharedPtr msg) {
  last_mpc_health_receive_sec_ = steadyNowSec();

  if (const auto status = jsonStringField(msg->data, "mpc_status")) {
    mpc_health_status_ = *status;
  } else {
    mpc_health_status_ = "parse_error";
  }

  if (const auto infeasible_count =
          jsonNumberField(msg->data, "mpc_infeasible_count")) {
    mpc_health_infeasible_count_ =
        std::max(0, static_cast<int>(std::llround(*infeasible_count)));
  } else {
    mpc_health_infeasible_count_ = 0;
  }

  if (const auto horizon_source =
          jsonStringField(msg->data, "mpc_predicted_horizon_source")) {
    mpc_predicted_horizon_source_ = *horizon_source;
  } else {
    mpc_predicted_horizon_source_ = "unknown";
  }
}

void SimplePurePursuit::clearOvertakeOverride() {
  overtake_override_active_ = false;
  overtake_mode_id_ = 0;
  overtake_override_generation_ = 0;
  overtake_lateral_offsets_.clear();
  overtake_speed_caps_.clear();
  last_overtake_override_sec_ = -1.0e9;
}

bool SimplePurePursuit::overtakeOverrideFresh(double now_sec) const {
  if (!use_overtake_reference_override_ || !overtake_override_active_ ||
      overtake_lateral_offsets_.empty()) {
    return false;
  }
  const double age_sec = inputAgeSec(last_overtake_override_sec_, now_sec);
  return ageFresh(age_sec, overtake_override_timeout_sec_) &&
         ageFresh(age_sec, max_override_age_sec_);
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
    double signed_path_curvature, double curvature_window_distance,
    double lookahead_point_x, double lookahead_point_y, double rear_x,
    double rear_y, double alpha, double pure_pursuit_steering_tire_angle,
    double curvature_feedforward_steering_rad, double raw_steering_tire_angle,
    double steering_tire_angle, bool overtake_override_applied,
    double overtake_lateral_offset_m, double overtake_speed_cap_mps,
    double freshness_now_sec, bool mpc_horizon_applied,
    bool mpc_horizon_velocity_cap_applied, double mpc_horizon_velocity_cap_mps,
    std::int32_t evaluated_horizon_stamp_sec,
    std::uint32_t evaluated_horizon_stamp_nanosec,
    std::int32_t applied_horizon_stamp_sec,
    std::uint32_t applied_horizon_stamp_nanosec,
    const std::string &applied_horizon_source, int applied_horizon_mode_id,
    std::uint32_t applied_horizon_generation,
    const HorizonFreshnessResult &mpc_horizon_freshness,
    const std::string &trajectory_source,
    const ControlPosePrediction &control_pose) {
  if (debug_publish_period_sec_ <= 0.0 || !pub_debug_) {
    return;
  }

  if (!mpc_horizon_applied &&
      freshness_now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
    return;
  }
  last_debug_publish_sec_ = freshness_now_sec;

  // rclcpp::Time does not expose ROS message fields directly. Preserve the
  // command timestamp exactly as it will appear on the wire in debug output.
  const builtin_interfaces::msg::Time stamp_msg = stamp;

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
  const double control_dx = control_pose.position.x - ref_x;
  const double control_dy = control_pose.position.y - ref_y;
  const double control_lateral_error_m =
      -std::sin(ref_yaw) * control_dx + std::cos(ref_yaw) * control_dy;
  const double control_yaw_error_rad =
      std::atan2(std::sin(control_pose.yaw - ref_yaw),
                 std::cos(control_pose.yaw - ref_yaw));
  const double odom_age_sec =
      inputAgeSec(last_odometry_receive_sec_, freshness_now_sec);
  const double trajectory_age_sec =
      inputAgeSec(last_trajectory_receive_sec_, freshness_now_sec);
  const double override_age_sec =
      overtake_override_active_
          ? inputAgeSec(last_overtake_override_sec_, freshness_now_sec)
          : -1.0;

  std::ostringstream json;
  json << "{"
       << "\"controller\":\"simple_pure_pursuit\","
       << "\"stale_input\":false,"
       << "\"odom_age_sec\":" << odom_age_sec << ","
       << "\"trajectory_age_sec\":" << trajectory_age_sec << ","
       << "\"override_age_sec\":" << override_age_sec << ","
       << "\"trajectory_source\":\"" << trajectory_source << "\","
       << "\"mpc_horizon_applied\":" << (mpc_horizon_applied ? "true" : "false")
       << ","
       << "\"mpc_horizon_velocity_cap_applied\":"
       << (mpc_horizon_velocity_cap_applied ? "true" : "false") << ","
       << "\"mpc_horizon_velocity_cap_mps\":" << mpc_horizon_velocity_cap_mps
       << ","
       << "\"command_stamp_sec\":" << stamp_msg.sec << ","
       << "\"command_stamp_nanosec\":" << stamp_msg.nanosec << ","
       << "\"evaluated_horizon_stamp_sec\":"
       << evaluated_horizon_stamp_sec << ","
       << "\"evaluated_horizon_stamp_nanosec\":"
       << evaluated_horizon_stamp_nanosec << ","
       << "\"applied_horizon_stamp_sec\":" << applied_horizon_stamp_sec
       << ","
       << "\"applied_horizon_stamp_nanosec\":"
       << applied_horizon_stamp_nanosec << ","
       << "\"applied_horizon_source\":\"" << applied_horizon_source
       << "\","
       << "\"applied_horizon_mode_id\":" << applied_horizon_mode_id
       << ","
       << "\"applied_horizon_generation\":"
       << applied_horizon_generation << ","
       << "\"mpc_horizon_age_sec\":" << mpc_horizon_freshness.age_sec << ","
       << "\"mpc_horizon_points\":" << mpc_horizon_freshness.point_count << ","
       << "\"mpc_horizon_start_distance_m\":"
       << mpc_horizon_freshness.start_distance_m << ","
       << "\"mpc_horizon_arc_length_m\":" << mpc_horizon_freshness.arc_length_m
       << ","
       << "\"mpc_health_required_for_horizon\":"
       << (require_solved_mpc_health_for_horizon_ ? "true" : "false") << ","
       << "\"mpc_health_status\":\"" << mpc_health_status_ << "\","
       << "\"mpc_health_age_sec\":" << mpcHealthAgeSec(freshness_now_sec) << ","
       << "\"mpc_infeasible_count\":" << mpc_health_infeasible_count_ << ","
       << "\"mpc_predicted_horizon_source\":\"" << mpc_predicted_horizon_source_
       << "\","
       << "\"mpc_horizon_contract_required\":"
       << (require_matching_overtake_horizon_contract_ ? "true" : "false")
       << ","
       << "\"mpc_horizon_contract_received\":"
       << (mpc_horizon_contract_received_ ? "true" : "false") << ","
       << "\"mpc_horizon_contract_age_sec\":"
       << inputAgeSec(last_mpc_predicted_horizon_contract_receive_sec_,
                      freshness_now_sec)
       << ","
       << "\"mpc_horizon_contract_source\":\""
       << mpc_horizon_contract_source_ << "\","
       << "\"mpc_horizon_contract_mode_id\":"
       << mpc_horizon_contract_mode_id_ << ","
       << "\"mpc_horizon_contract_generation\":"
       << mpc_horizon_contract_generation_ << ","
       << "\"overtake_override_generation\":"
       << overtake_override_generation_ << ","
       << "\"mpc_horizon_reject_reason\":\"" << mpc_horizon_freshness.reason
       << "\","
       << "\"nearest_trajectory_index\":" << nearest_traj_point_idx << ","
       << "\"target_speed_mps\":" << target_longitudinal_vel << ","
       << "\"current_speed_mps\":" << current_longitudinal_vel << ","
       << "\"command_accel_mps2\":" << command_accel << ","
       << "\"base_lookahead_distance_m\":" << base_lookahead_distance << ","
       << "\"desired_lookahead_distance_m\":" << desired_lookahead_distance
       << ","
       << "\"lookahead_distance_m\":" << lookahead_distance << ","
       << "\"path_curvature_1pm\":" << path_curvature << ","
       << "\"signed_path_curvature_1pm\":" << signed_path_curvature << ","
       << "\"curvature_window_distance_m\":" << curvature_window_distance << ","
       << "\"curvature_adaptive_lookahead_enabled\":"
       << (curvature_adaptive_lookahead_enabled_ ? "true" : "false") << ","
       << "\"lookahead_point_x\":" << lookahead_point_x << ","
       << "\"lookahead_point_y\":" << lookahead_point_y << ","
       << "\"rear_x\":" << rear_x << ","
       << "\"rear_y\":" << rear_y << ","
       << "\"alpha_rad\":" << alpha << ","
       << "\"pure_pursuit_steering_tire_angle_rad\":"
       << pure_pursuit_steering_tire_angle << ","
       << "\"curvature_feedforward_steering_rad\":"
       << curvature_feedforward_steering_rad << ","
       << "\"raw_steering_tire_angle_rad\":" << raw_steering_tire_angle << ","
       << "\"steering_tire_angle_rad\":" << steering_tire_angle << ","
       << "\"lateral_error_m\":" << lateral_error_m << ","
       << "\"yaw_error_rad\":" << yaw_error_rad << ","
       << "\"control_pose_x\":" << control_pose.position.x << ","
       << "\"control_pose_y\":" << control_pose.position.y << ","
       << "\"control_pose_yaw_rad\":" << control_pose.yaw << ","
       << "\"control_pose_shifted\":"
       << (control_pose.shifted ? "true" : "false") << ","
       << "\"control_pose_prediction_steps\":" << control_pose.prediction_steps
       << ","
       << "\"control_pose_lateral_error_m\":" << control_lateral_error_m << ","
       << "\"control_pose_yaw_error_rad\":" << control_yaw_error_rad << ","
       << "\"pp_control_delay_sec\":" << pp_control_delay_sec_ << ","
       << "\"pp_prediction_dt_sec\":" << pp_prediction_dt_sec_ << ","
       << "\"steering_source\":\"" << control_pose.steering_source << "\","
       << "\"steering_age_sec\":" << control_pose.steering_age_sec << ","
       << "\"current_steering_rad\":" << control_pose.current_steering_rad
       << ","
       << "\"predicted_applied_steering_rad\":"
       << control_pose.applied_steering_rad << ","
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
