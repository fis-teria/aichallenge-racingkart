#include "overtake_planner/aw2_plan_sample_contract.hpp"
#include "overtake_planner/overtake_planner_core.hpp"
#include "overtake_planner/planner_output_builder.hpp"
#include "overtake_planner/planner_publication_state.hpp"
#include "overtake_planner/pp_core_exact_snapshot.hpp"
#include "overtake_planner/reference_override_contract.hpp"
#include "overtake_planner/safety_constraint_authority.hpp"
#include "overtake_transport_contract/c002ay0_canonical.hpp"
#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/candidate_execution_request.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_execution_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_tracking_status.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/motion_authority_grant.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/overtake_plan.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/safety_constraint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace overtake_planner {

namespace {

// 入力: ROS geometry_msgsのQuaternion。
// 出力: yaw角[rad]。
// 処理概要: 2D走行で使うz軸周りの姿勢だけを取り出す。
double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw_rad) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);
  return q;
}

std::uint64_t makePlannerInstanceId() {
  static std::atomic<std::uint64_t> instance_counter{1U};
  const auto steady_ticks =
      std::chrono::steady_clock::now().time_since_epoch().count();
  std::uint64_t instance_id =
      static_cast<std::uint64_t>(steady_ticks) ^
      instance_counter.fetch_add(1U, std::memory_order_relaxed);
  if (instance_id == 0U) {
    instance_id = instance_counter.fetch_add(1U, std::memory_order_relaxed);
  }
  return instance_id == 0U ? 1U : instance_id;
}

std::uint64_t makeDistinctPlannerInstanceId(std::uint64_t excluded_id) {
  const std::uint64_t candidate = makePlannerInstanceId();
  if (candidate != excluded_id) {
    return candidate;
  }
  return excluded_id == std::numeric_limits<std::uint64_t>::max()
             ? 1U
             : excluded_id + 1U;
}

// 入力: パッケージ名とCSVパス。
// 出力: 絶対パス。csv_pathが絶対パスならそのまま返す。
// 処理概要: launch/configでは短い相対パスを書けるよう、package
// share配下へ解決する。
std::string resolveReferencePath(const std::string &package_name,
                                 const std::string &csv_path) {
  // 相対パスならパッケージshare配下として解決し、configから短いパスで指定できるようにする。
  if (csv_path.empty() || csv_path.front() == '/') {
    return csv_path;
  }
  return ament_index_cpp::get_package_share_directory(package_name) + "/" +
         csv_path;
}

// 入力: 文字列。
// 出力: 前後の空白を除いた文字列。
// 処理概要: CSV列やbool文字列を読む前に不要な空白を落とす。
std::string trim(std::string value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

// 入力: CSVの1行。
// 出力: trim済みの列文字列配列。
// 処理概要: 許可CSVを読むための単純なカンマ分割を行う。
std::vector<std::string> splitCsvLine(const std::string &line) {
  std::vector<std::string> columns;
  std::stringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ',')) {
    columns.push_back(trim(column));
  }
  return columns;
}

// 入力: 整数文字列。
// 出力: 変換できたint64。失敗時はnullopt。
// 処理概要: 許可CSVのwaypoint idを安全に数値化する。
std::optional<std::int64_t> parseInt64(const std::string &value) {
  errno = 0;
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(parsed);
}

// 入力: true/false相当の文字列。
// 出力: bool値。未対応文字列ならnullopt。
// 処理概要: CSVでallow/denyなども使えるよう表記揺れを吸収する。
std::optional<bool> parseBool(const std::string &value) {
  std::string lowered;
  lowered.reserve(value.size());
  std::transform(
      value.begin(), value.end(), std::back_inserter(lowered),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "true" || lowered == "1" || lowered == "yes" ||
      lowered == "allow") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" ||
      lowered == "deny") {
    return false;
  }
  return std::nullopt;
}

// 入力: debug JSONへ出したいdouble。
// 出力: JSON数値文字列。NaN/Infならnull。
// 処理概要: 非有限値をJSONとして壊さず、後段解析で欠損として扱えるようにする。
std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  return std::to_string(value);
}

const char *
passTransitionDeadlineSourceString(PassTransitionDeadlineSource source) {
  switch (source) {
  case PassTransitionDeadlineSource::PROFILE_MARKER:
    return "profile_marker";
  case PassTransitionDeadlineSource::TARGET_CLEARANCE:
    return "target_clearance";
  case PassTransitionDeadlineSource::NONE:
    return "none";
  }
  return "none";
}

std::vector<std::string>
supervisorV2AuthorizationFailureReasons(std::uint32_t mask,
                                        const std::string &tracking_reason) {
  std::vector<std::string> reasons;
  const auto append = [&reasons, mask](std::uint32_t bit, const char *reason) {
    if ((mask & bit) != 0U) {
      reasons.emplace_back(reason);
    }
  };
  append(SUPERVISOR_V2_AUTH_CANDIDATE_MISSING, "candidate_missing");
  append(SUPERVISOR_V2_AUTH_CANDIDATE_NOT_SAFETY_EVALUATED,
         "candidate_not_safety_evaluated");
  append(SUPERVISOR_V2_AUTH_CANDIDATE_REJECTED, "candidate_rejected");
  append(SUPERVISOR_V2_AUTH_EGO_STALE, "ego_stale");
  append(SUPERVISOR_V2_AUTH_V2X_STALE, "v2x_stale");
  append(SUPERVISOR_V2_AUTH_OPPONENT_STALE, "opponent_stale");
  append(SUPERVISOR_V2_AUTH_OPPONENT_EXCLUDED, "opponent_excluded");
  append(SUPERVISOR_V2_AUTH_REFERENCE_INVALID, "reference_invalid");
  append(SUPERVISOR_V2_AUTH_PREDICTION_INCOMPLETE, "prediction_incomplete");
  if ((mask & SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE) != 0U) {
    reasons.push_back(tracking_reason.empty()
                          ? "controller_tracking_unusable"
                          : "controller_tracking_" + tracking_reason);
  }
  append(SUPERVISOR_V2_AUTH_MPC_HEALTH_STALE, "mpc_health_stale");
  append(SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE, "mpc_hard_failure");
  append(SUPERVISOR_V2_AUTH_TRAJECTORY_NOT_PUBLISHABLE,
         "trajectory_not_publishable");
  append(SUPERVISOR_V2_AUTH_CONSTRAINT_STOP_REQUESTED,
         "constraint_stop_requested");
  append(SUPERVISOR_V2_AUTH_SAFETY_INPUTS_INCOMPLETE,
         "safety_inputs_incomplete");
  append(SUPERVISOR_V2_AUTH_PROFILE_PROGRESS_INVALID,
         "profile_progress_invalid");
  return reasons;
}

std::string joinStrings(const std::vector<std::string> &values,
                        const char *separator) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      stream << separator;
    }
    stream << values[i];
  }
  return stream.str();
}

// 入力: 簡易JSON文字列と数値field名。
// 出力: field値をdouble化したもの。無い/不正ならnullopt。
// 処理概要: MPC health debugから必要な数値だけを軽量に抜き出す。
std::optional<double> jsonNumberField(const std::string &json,
                                      const std::string &field) {
  const std::string key = "\"" + field + "\":";
  const auto key_pos = json.find(key);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  std::size_t pos = key_pos + key.size();
  while (pos < json.size() &&
         std::isspace(static_cast<unsigned char>(json[pos])) != 0) {
    ++pos;
  }
  if (pos >= json.size()) {
    return std::nullopt;
  }

  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(json.c_str() + pos, &end);
  if (errno != 0 || end == json.c_str() + pos || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

// 入力: ROS_DOMAIN_ID文字列。
// 出力: 対応するV2X vehicle_id(dN)。変換できなければnullopt。
// 処理概要:
// 複数台評価でdomain_idと車両IDを揃え、自車を相手車リストから除外する。
std::optional<std::string> vehicleIdFromRosDomainId(const char *raw_domain_id) {
  // AI Challengeのdomain番号とV2X vehicle_id(d1,d2,...)を対応させる。
  if (raw_domain_id == nullptr || raw_domain_id[0] == '\0') {
    return std::nullopt;
  }

  errno = 0;
  char *end = nullptr;
  const long domain_id = std::strtol(raw_domain_id, &end, 10);
  if (errno != 0 || end == raw_domain_id || *end != '\0' || domain_id <= 0) {
    return std::nullopt;
  }

  return "d" + std::to_string(domain_id);
}

// 入力: パラメータで指定された自車IDとlogger。
// 出力: 実際に使う自車vehicle_id。
// 処理概要:
// 明示IDを優先し、auto時はROS_DOMAIN_IDから推定、失敗時はd1へフォールバックする。
std::string resolveOwnVehicleId(const std::string &configured_id,
                                const rclcpp::Logger &logger) {
  // own_vehicle_id=autoならROS_DOMAIN_IDから自車IDを推定し、V2X上の自車を除外する。
  if (!configured_id.empty() && configured_id != "auto") {
    return configured_id;
  }

  const char *raw_domain_id = std::getenv("ROS_DOMAIN_ID");
  const auto resolved_id = vehicleIdFromRosDomainId(raw_domain_id);
  if (resolved_id.has_value()) {
    RCLCPP_INFO(logger, "resolved own_vehicle_id=%s from ROS_DOMAIN_ID=%s",
                resolved_id->c_str(), raw_domain_id);
    return resolved_id.value();
  }

  RCLCPP_WARN(logger, "own_vehicle_id is auto, but ROS_DOMAIN_ID is unset or "
                      "invalid; falling back to d1");
  return "d1";
}

// 入力: 起動時に読み込んだPlannerConfig。
// 出力: 安全にoverrideを有効化できない理由。問題がなければnullopt。
// 処理概要: 縦安全予測がplannerの保守的制動上限・有限horizon内にあることを、
// 起動前に検査して不正設定での介入を止める。実制御器の能力はactive launchで
// 別途この仮定以上であることを確認する。
std::optional<std::string>
invalidLongitudinalSafetyConfig(const PlannerConfig &config) {
  if (config.horizon_points < 2) {
    return "horizon_points must be >= 2";
  }
  if (!std::isfinite(config.horizon_dt_sec) || config.horizon_dt_sec <= 0.0) {
    return "horizon_dt_sec must be finite and > 0";
  }
  if (!std::isfinite(config.max_brake_decel_mps2) ||
      config.max_brake_decel_mps2 <= 0.0 || config.max_brake_decel_mps2 > 1.5) {
    return "max_brake_decel_mps2 must be finite and in (0, 1.5]";
  }
  if (!std::isfinite(config.longitudinal_response_delay_sec) ||
      config.longitudinal_response_delay_sec < 0.0) {
    return "longitudinal_response_delay_sec must be finite and >= 0";
  }
  if (!std::isfinite(config.recovery_assumed_accel_mps2) ||
      config.recovery_assumed_accel_mps2 <= 0.0 ||
      config.recovery_assumed_accel_mps2 > 3.0) {
    return "recovery_assumed_accel_mps2 must be finite and in (0, 3.0]";
  }
  if (!std::isfinite(config.stationary_obstacle_speed_threshold_mps) ||
      config.stationary_obstacle_speed_threshold_mps < 0.0) {
    return "stationary_obstacle_speed_threshold_mps must be finite and >= 0";
  }
  if (!std::isfinite(config.moving_pass_max_completion_time_sec) ||
      config.moving_pass_max_completion_time_sec <= 0.0 ||
      config.moving_pass_max_completion_time_sec > 60.0 ||
      !std::isfinite(config.moving_pass_min_closing_speed_mps) ||
      config.moving_pass_min_closing_speed_mps < 0.0) {
    return "invalid moving pass reachability safety parameter";
  }
  if (config.reentry_gate_enabled &&
      (config.reentry_safe_cycles < 1 ||
       !std::isfinite(config.reentry_min_safety_margin_h) ||
       config.reentry_min_safety_margin_h < config.min_ellipse_h ||
       !std::isfinite(config.reentry_evaluation_horizon_sec) ||
       config.reentry_evaluation_horizon_sec <= 0.0 ||
       !std::isfinite(config.reentry_v2x_snapshot_stale_time_sec) ||
       config.reentry_v2x_snapshot_stale_time_sec <= 0.0 ||
       !std::isfinite(config.reentry_hold_v_max_mps) ||
       config.reentry_hold_v_max_mps <= 0.0 ||
       !std::isfinite(config.post_abort_curve_hold_v_max_mps) ||
       config.post_abort_curve_hold_v_max_mps <= 0.0 ||
       !std::isfinite(config.reentry_completion_lateral_error_m) ||
       config.reentry_completion_lateral_error_m <= 0.0 ||
       !std::isfinite(config.reentry_completion_rearm_lateral_error_m) ||
       config.reentry_completion_rearm_lateral_error_m <=
           config.reentry_completion_lateral_error_m)) {
    return "invalid reentry gate safety parameter";
  }
  if (config.start_grid_target_enabled &&
      (!std::isfinite(config.start_grid_target_window_sec) ||
       config.start_grid_target_window_sec <= 0.0 ||
       !std::isfinite(config.start_grid_target_window_distance_m) ||
       config.start_grid_target_window_distance_m <= 0.0 ||
       !std::isfinite(config.start_grid_target_max_ego_speed_mps) ||
       config.start_grid_target_max_ego_speed_mps < 0.0 ||
       !std::isfinite(config.start_grid_target_min_delta_s_m) ||
       !std::isfinite(config.start_grid_target_max_delta_s_m) ||
       config.start_grid_target_max_delta_s_m <
           config.start_grid_target_min_delta_s_m ||
       !std::isfinite(config.start_grid_target_lateral_width_m) ||
       config.start_grid_target_lateral_width_m <= 0.0 ||
       !std::isfinite(config.start_grid_stationary_confirmation_sec) ||
       config.start_grid_stationary_confirmation_sec < 0.0 ||
       !std::isfinite(config.start_grid_stationary_confirmation_distance_m) ||
       config.start_grid_stationary_confirmation_distance_m < 0.0 ||
       !std::isfinite(config.start_grid_attack_follow_v_max_mps) ||
       config.start_grid_attack_follow_v_max_mps <= 0.0 ||
       !std::isfinite(config.start_grid_uncommitted_hold_max_lateral_drift_m) ||
       config.start_grid_uncommitted_hold_max_lateral_drift_m <= 0.0 ||
       !std::isfinite(config.start_grid_hold_correction_distance_m) ||
       config.start_grid_hold_correction_distance_m <= 0.0 ||
       !std::isfinite(
           config.start_grid_moving_pass_max_lateral_displacement_m) ||
       config.start_grid_moving_pass_max_lateral_displacement_m <= 0.0)) {
    return "invalid start grid target safety parameter";
  }
  if (!std::isfinite(config.normal_recovery_speed_only_v_max_mps) ||
      config.normal_recovery_speed_only_v_max_mps <= 0.0) {
    return "normal_recovery_speed_only_v_max_mps must be finite and > 0";
  }
  if (!std::isfinite(config.attack_follow_tracking_wheelbase_m) ||
      config.attack_follow_tracking_wheelbase_m <= 0.0 ||
      !std::isfinite(config.attack_follow_max_steering_angle_rad) ||
      config.attack_follow_max_steering_angle_rad <= 0.0 ||
      config.attack_follow_max_steering_angle_rad >
          kVehicleHardSteeringTireAngleRad ||
      !std::isfinite(config.attack_follow_max_steering_rate_radps) ||
      config.attack_follow_max_steering_rate_radps <= 0.0 ||
      !std::isfinite(config.attack_follow_steering_tire_angle_gain) ||
      config.attack_follow_steering_tire_angle_gain <= 0.0 ||
      !std::isfinite(config.attack_follow_steering_rate_reserve_ratio) ||
      config.attack_follow_steering_rate_reserve_ratio <= 0.0 ||
      config.attack_follow_steering_rate_reserve_ratio > 1.0 ||
      !std::isfinite(config.attack_follow_min_spatial_horizon_m) ||
      config.attack_follow_min_spatial_horizon_m < 0.50 ||
      !std::isfinite(config.attack_follow_max_evaluation_horizon_sec) ||
      config.attack_follow_max_evaluation_horizon_sec <=
          static_cast<double>(config.horizon_points - 1U) *
              config.horizon_dt_sec ||
      config.attack_follow_max_evaluation_horizon_sec > 20.0 ||
      !std::isfinite(config.lateral_override_lookahead_gain) ||
      config.lateral_override_lookahead_gain < 0.0 ||
      !std::isfinite(config.lateral_override_lookahead_min_distance_m) ||
      config.lateral_override_lookahead_min_distance_m < 0.50 ||
      !std::isfinite(
          config.moving_lateral_override_max_evaluation_horizon_sec) ||
      config.moving_lateral_override_max_evaluation_horizon_sec <=
          static_cast<double>(config.horizon_points - 1U) *
              config.horizon_dt_sec ||
      config.moving_lateral_override_max_evaluation_horizon_sec > 20.0 ||
      config.moving_lateral_override_max_evaluation_horizon_sec + 1.0e-9 <
          config.attack_follow_max_evaluation_horizon_sec ||
      !std::isfinite(config.lateral_override_max_evaluation_horizon_sec) ||
      config.lateral_override_max_evaluation_horizon_sec <=
          static_cast<double>(config.horizon_points - 1U) *
              config.horizon_dt_sec ||
      config.lateral_override_max_evaluation_horizon_sec > 20.0 ||
      config.lateral_override_max_evaluation_horizon_sec + 1.0e-9 <
          config.moving_lateral_override_max_evaluation_horizon_sec ||
      !std::isfinite(config.lateral_override_execution_speed_reserve_sec) ||
      config.lateral_override_execution_speed_reserve_sec < 0.05 ||
      config.lateral_override_execution_speed_reserve_sec > 0.50) {
    return "invalid attack follow tracking safety parameter";
  }
  if (config.unstarted_pass_target_release_enabled &&
      (!std::isfinite(config.unstarted_pass_target_release_min_gap_m) ||
       config.unstarted_pass_target_release_min_gap_m <= 0.0 ||
       !std::isfinite(
           config.unstarted_pass_target_release_min_opening_speed_mps) ||
       config.unstarted_pass_target_release_min_opening_speed_mps <= 0.0 ||
       !std::isfinite(
           config.unstarted_pass_target_release_max_lateral_progress_m) ||
       config.unstarted_pass_target_release_max_lateral_progress_m < 0.0 ||
       config.unstarted_pass_target_release_required_cycles < 2)) {
    return "invalid unstarted pass target release safety parameter";
  }
  if (config.gentle_curve_safe_pass_enabled &&
      (!std::isfinite(config.gentle_curve_safe_pass_max_curvature_m_inv) ||
       config.gentle_curve_safe_pass_max_curvature_m_inv <= 0.0 ||
       !std::isfinite(config.gentle_curve_safe_pass_v_max_mps) ||
       config.gentle_curve_safe_pass_v_max_mps <= 0.0 ||
       !std::isfinite(
           config.gentle_curve_safe_pass_max_lateral_displacement_m) ||
       config.gentle_curve_safe_pass_max_lateral_displacement_m <= 0.0 ||
       !std::isfinite(config.gentle_curve_safe_pass_max_lateral_accel_mps2) ||
       config.gentle_curve_safe_pass_max_lateral_accel_mps2 <= 0.0)) {
    return "invalid gentle curve safe pass safety parameter";
  }
  if (config.stationary_no_pass_safe_pass_enabled &&
      (!std::isfinite(
           config.stationary_no_pass_safe_pass_max_curvature_m_inv) ||
       config.stationary_no_pass_safe_pass_max_curvature_m_inv <= 0.0 ||
       !std::isfinite(config.stationary_no_pass_safe_pass_v_max_mps) ||
       config.stationary_no_pass_safe_pass_v_max_mps <= 0.0 ||
       !std::isfinite(
           config.stationary_no_pass_safe_pass_max_lateral_displacement_m) ||
       config.stationary_no_pass_safe_pass_max_lateral_displacement_m <= 0.0 ||
       !std::isfinite(
           config.stationary_no_pass_safe_pass_max_lateral_accel_mps2) ||
       config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 <= 0.0 ||
       !std::isfinite(config.stationary_no_pass_safe_pass_max_cbf_slack) ||
       config.stationary_no_pass_safe_pass_max_cbf_slack < 0.0)) {
    return "invalid stationary no-pass safe pass safety parameter";
  }
  if (config.braking_follow_enabled &&
      (!std::isfinite(config.braking_follow_max_distance_m) ||
       config.braking_follow_max_distance_m <= 0.0 ||
       !std::isfinite(config.braking_follow_max_target_speed_mps) ||
       config.braking_follow_max_target_speed_mps < 0.0 ||
       !std::isfinite(config.braking_follow_trigger_margin_m) ||
       config.braking_follow_trigger_margin_m < 0.0 ||
       !std::isfinite(config.braking_follow_ttc_threshold_sec) ||
       config.braking_follow_ttc_threshold_sec < 0.0)) {
    return "invalid braking follow safety parameter";
  }
  if (config.follow_gap_closing_enabled &&
      (!std::isfinite(config.follow_gap_closing_target_gap_m) ||
       config.follow_gap_closing_target_gap_m < config.safety_ellipse_a_m ||
       !std::isfinite(config.follow_gap_closing_engage_gap_m) ||
       config.follow_gap_closing_engage_gap_m <
           config.follow_gap_closing_target_gap_m ||
       !std::isfinite(config.follow_trigger_s_m) ||
       config.follow_trigger_s_m <= config.follow_gap_closing_engage_gap_m ||
       !std::isfinite(config.follow_gap_closing_speed_gain_per_m) ||
       config.follow_gap_closing_speed_gain_per_m <= 0.0 ||
       !std::isfinite(config.follow_gap_closing_max_speed_bonus_mps) ||
       config.follow_gap_closing_max_speed_bonus_mps <= 0.0 ||
       !std::isfinite(config.follow_gap_closing_assumed_accel_mps2) ||
       config.follow_gap_closing_assumed_accel_mps2 <= 0.0 ||
       config.follow_gap_closing_assumed_accel_mps2 > 3.0)) {
    return "invalid follow gap closing safety parameter";
  }
  if (!std::isfinite(config.pass_speed_cap_mps) ||
      config.pass_speed_cap_mps <= 0.0 ||
      !std::isfinite(config.pass_assumed_accel_mps2) ||
      config.pass_assumed_accel_mps2 <= 0.0 ||
      config.pass_assumed_accel_mps2 > 3.0 ||
      !std::isfinite(config.pass_target_lateral_margin_m) ||
      config.pass_target_lateral_margin_m < 0.0 ||
      !std::isfinite(config.pass_lateral_first_stationary_creep_v_max_mps) ||
      config.pass_lateral_first_stationary_creep_v_max_mps <= 0.0 ||
      config.pass_lateral_first_stationary_creep_v_max_mps > 1.0 ||
      !std::isfinite(config.pass_lateral_tracking_lag_threshold_m) ||
      config.pass_lateral_tracking_lag_threshold_m <= 0.0 ||
      !std::isfinite(config.pass_lateral_tracking_lag_speed_cap_mps) ||
      config.pass_lateral_tracking_lag_speed_cap_mps <= 0.0 ||
      config.pass_lateral_tracking_lag_speed_cap_mps >
          config.pass_lateral_first_stationary_creep_v_max_mps) {
    return "invalid pass acceleration safety parameter";
  }
  if (config.pass_target_policy != "legacy_fixed_offset" &&
      config.pass_target_policy != "minimum_clearance") {
    return "pass_target_policy must be legacy_fixed_offset or "
           "minimum_clearance";
  }
  if (config.early_stationary_parallel_pass_enabled &&
      (!std::isfinite(config.early_stationary_parallel_pass_distance_m) ||
       config.early_stationary_parallel_pass_distance_m <= 0.0 ||
       !std::isfinite(config.early_stationary_parallel_pass_lateral_width_m) ||
       config.early_stationary_parallel_pass_lateral_width_m <=
           config.same_corridor_width_m ||
       config.early_stationary_parallel_pass_lateral_width_m >
           config.parallel_side_margin_m)) {
    return "invalid early stationary parallel pass parameter";
  }
  return std::nullopt;
}

} // namespace

class OvertakePlannerNode : public rclcpp::Node {
public:
  // 入力: ROS parameter、参照CSV、V2X/odom/MPC health topic。
  // 出力: publisher/subscriber/timerを持つROS node。
  // 処理概要:
  // PlannerConfigとFrenetFrameを初期化し、周期timerでcoreを更新する実行環境を作る。
  OvertakePlannerNode() : Node("overtake_planner_node") {
    // 参照線、V2Xフィルタ、候補生成/安全評価のしきい値をROSパラメータから読む。
    const auto reference_package = declare_parameter<std::string>(
        "reference_package", "multi_purpose_mpc_ros");
    const auto reference_csv = declare_parameter<std::string>(
        "reference_csv", "env/final_ver3/traj_mincurv_manual.csv");
    const auto overtake_permission_package = declare_parameter<std::string>(
        "overtake_permission_package", "overtake_planner");
    const auto overtake_permission_csv = declare_parameter<std::string>(
        "overtake_permission_csv", "config/overtake_permission.csv");
    const auto drivable_corridor_package = declare_parameter<std::string>(
        "drivable_corridor_package", "overtake_planner");
    const auto drivable_corridor_csv = declare_parameter<std::string>(
        "drivable_corridor_csv", "config/final_ver3_drivable_corridor.csv");
    const bool drivable_corridor_enabled =
        declare_parameter<bool>("drivable_corridor_enabled", true);
    own_vehicle_id_ = resolveOwnVehicleId(
        declare_parameter<std::string>("own_vehicle_id", "auto"), get_logger());
    c002ay1_prod_measure_enabled_ =
        declare_parameter<bool>("c002ay1_prod_measure_enabled", false);
    const auto c002ay1_socket_path =
        declare_parameter<std::string>("c002ay1_prod_measure_socket_path", "");
    const auto c002ay1_run_id =
        declare_parameter<std::string>("c002ay1_prod_measure_run_id", "");
    const auto c002ay1_session_nonce = declare_parameter<std::int64_t>(
        "c002ay1_prod_measure_session_nonce", 0);
    const auto c002ay1_instance_id =
        declare_parameter<std::int64_t>("c002ay1_prod_measure_instance_id", 0);
    const auto c002ay1_selected_stream = declare_parameter<std::string>(
        "c002ay1_prod_measure_selected_stream", "legacy");
    if (c002ay1_selected_stream == "legacy") {
      c002ay1_configured_stream_ = overtake_transport_contract::c002ay1::
          ConfiguredStreamKind::kLegacyReferenceOverride;
    } else if (c002ay1_selected_stream == "v2") {
      c002ay1_configured_stream_ = overtake_transport_contract::c002ay1::
          ConfiguredStreamKind::kV2Trajectory;
    } else {
      c002ay1_configured_stream_ =
          overtake_transport_contract::c002ay1::ConfiguredStreamKind::kNone;
      c002ay1_prod_measure_enabled_ = false;
    }
    overtake_transport_contract::c002ay1::RuntimeObserverConfig
        c002ay1_observer_config;
    c002ay1_observer_config.enabled = c002ay1_prod_measure_enabled_ &&
                                      c002ay1_session_nonce > 0 &&
                                      c002ay1_instance_id > 0;
    c002ay1_observer_config.role =
        overtake_transport_contract::c002ay1::ProducerRole::kPlanner;
    c002ay1_observer_config.socket_path = c002ay1_socket_path;
    c002ay1_observer_config.run_id = c002ay1_run_id;
    c002ay1_observer_config.session_nonce =
        c002ay1_session_nonce > 0
            ? static_cast<std::uint64_t>(c002ay1_session_nonce)
            : 0U;
    c002ay1_observer_config.producer_instance_id =
        c002ay1_instance_id > 0
            ? static_cast<std::uint64_t>(c002ay1_instance_id)
            : 0U;
    c002ay1_runtime_observer_ =
        overtake_transport_contract::c002ay1::RuntimeObservationWriter::attach(
            c002ay1_observer_config);
    ignore_near_ego_m_ = declare_parameter<double>("ignore_near_ego_m", 1.0);
    position_jump_threshold_m_ =
        declare_parameter<double>("position_jump_threshold_m", 5.0);
    ego_stale_time_sec_ = declare_parameter<double>("ego_stale_time_sec", 0.50);
    input_future_stamp_tolerance_sec_ =
        declare_parameter<double>("input_future_stamp_tolerance_sec", 0.05);
    if (!std::isfinite(input_future_stamp_tolerance_sec_) ||
        input_future_stamp_tolerance_sec_ < 0.0) {
      RCLCPP_WARN(get_logger(),
                  "invalid input_future_stamp_tolerance_sec; using 0.0");
      input_future_stamp_tolerance_sec_ = 0.0;
    }
    race_arm_required_ = declare_parameter<bool>("race_arm_required", true);
    race_arm_topic_ = declare_parameter<std::string>("race_arm_topic",
                                                     "/overtake/race_armed");
    race_armed_ = !race_arm_required_;
    if (race_armed_) {
      race_arm_epoch_ = 1U;
      connector_transaction_sequencer_.resetForRaceEpoch(race_arm_epoch_);
      v2_connector_transaction_sequencer_.resetForRaceEpoch(race_arm_epoch_);
    }
    if (race_arm_required_ && race_arm_topic_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "race_arm_topic is empty; planner will remain disarmed");
    }
    controller_tracking_status_timeout_sec_ = declare_parameter<double>(
        "controller_tracking_status_timeout_sec", 0.30);
    pp_core_exact_snapshot_enabled_ =
        declare_parameter<bool>("pp_core_exact_snapshot_enabled", false);
    if (!std::isfinite(controller_tracking_status_timeout_sec_) ||
        controller_tracking_status_timeout_sec_ <= 0.0) {
      RCLCPP_ERROR(get_logger(),
                   "controller_tracking_status_timeout_sec must be finite and "
                   "> 0; typed Pure Pursuit confirmation will remain false");
    }

    PlannerConfig config;
    config.enabled = declare_parameter<bool>("enabled", true);
    config.trajectory_backend =
        declare_parameter<std::string>("trajectory_backend", "current");
    if (config.trajectory_backend != "current" &&
        config.trajectory_backend != "state_lattice_shadow" &&
        config.trajectory_backend != "state_lattice_candidate") {
      RCLCPP_ERROR(
          get_logger(),
          "trajectory_backend must be current, state_lattice_shadow, or "
          "state_lattice_candidate; using current");
      config.trajectory_backend = "current";
    }
    if (config.trajectory_backend == "state_lattice_candidate") {
      RCLCPP_WARN(get_logger(), "state_lattice_candidate is reserved; live "
                                "candidates remain current");
    }
    config.supervisor_v2_shadow_enabled =
        declare_parameter<bool>("supervisor_v2_shadow_enabled", false);
    config.supervisor_v2_abort_release_cycles =
        declare_parameter<int>("supervisor_v2_abort_release_cycles", 3);
    config.supervisor_v2_pass_completion_cycles =
        declare_parameter<int>("supervisor_v2_pass_completion_cycles", 2);
    config.supervisor_v2_target_missing_hold_cycles =
        declare_parameter<int>("supervisor_v2_target_missing_hold_cycles", 2);
    config.supervisor_v2_tracking_unusable_hold_cycles = declare_parameter<int>(
        "supervisor_v2_tracking_unusable_hold_cycles", 2);
    config.start_grid_tracking_probe_required_cycles =
        declare_parameter<int>("start_grid_tracking_probe_required_cycles", 2);
    if (config.start_grid_tracking_probe_required_cycles <= 0) {
      RCLCPP_ERROR(
          get_logger(),
          "start_grid_tracking_probe_required_cycles must be > 0; using 2");
      config.start_grid_tracking_probe_required_cycles = 2;
    }
    config.start_grid_tracking_release_timeout_cycles = declare_parameter<int>(
        "start_grid_tracking_release_timeout_cycles", 60);
    if (config.start_grid_tracking_release_timeout_cycles <= 0) {
      RCLCPP_ERROR(
          get_logger(),
          "start_grid_tracking_release_timeout_cycles must be > 0; using 60");
      config.start_grid_tracking_release_timeout_cycles = 60;
    }
    config.start_grid_tracking_continuity_max_mpc_solve_time_ms =
        declare_parameter<double>(
            "start_grid_tracking_continuity_max_mpc_solve_time_ms", 120.0);
    if (!std::isfinite(
            config.start_grid_tracking_continuity_max_mpc_solve_time_ms) ||
        config.start_grid_tracking_continuity_max_mpc_solve_time_ms <= 0.0) {
      RCLCPP_ERROR(
          get_logger(),
          "start_grid_tracking_continuity_max_mpc_solve_time_ms must be "
          "finite and > 0; using 120 ms");
      config.start_grid_tracking_continuity_max_mpc_solve_time_ms = 120.0;
    }
    supervisor_v2_shadow_enabled_ = config.supervisor_v2_shadow_enabled;
    const int horizon_points_param =
        declare_parameter<int>("horizon_points", 20);
    config.horizon_points = horizon_points_param > 0
                                ? static_cast<std::size_t>(horizon_points_param)
                                : 0U;
    config.horizon_dt_sec = declare_parameter<double>("horizon_dt_sec", 0.025);
    config.lookahead_s_m = declare_parameter<double>("lookahead_s_m", 10.0);
    config.follow_trigger_s_m =
        declare_parameter<double>("follow_trigger_s_m", 12.0);
    config.same_corridor_width_m =
        declare_parameter<double>("same_corridor_width_m", 0.90);
    config.same_direction_filter_enabled =
        declare_parameter<bool>("same_direction_filter_enabled", true);
    config.same_direction_min_speed_mps =
        declare_parameter<double>("same_direction_min_speed_mps", 0.30);
    config.same_direction_min_s_dot_mps =
        declare_parameter<double>("same_direction_min_s_dot_mps", 0.05);
    config.future_side_prediction_enabled =
        declare_parameter<bool>("future_side_prediction_enabled", true);
    config.future_side_prediction_horizon_sec =
        declare_parameter<double>("future_side_prediction_horizon_sec", 1.2);
    config.future_side_prediction_dt_sec =
        declare_parameter<double>("future_side_prediction_dt_sec", 0.3);
    config.future_side_yield_wall_clearance_m =
        declare_parameter<double>("future_side_yield_wall_clearance_m", 0.35);
    config.dv_block_threshold_mps =
        declare_parameter<double>("dv_block_threshold_mps", 0.20);
    config.opponent_stale_time_sec =
        declare_parameter<double>("opponent_stale_time_sec", 0.50);
    config.input_future_stamp_tolerance_sec = input_future_stamp_tolerance_sec_;
    config.side_by_side_s_m =
        declare_parameter<double>("side_by_side_s_m", 4.0);
    config.side_margin_m = declare_parameter<double>("side_margin_m", 1.2);
    config.parallel_side_detection_enabled =
        declare_parameter<bool>("parallel_side_detection_enabled", true);
    config.parallel_side_s_m =
        declare_parameter<double>("parallel_side_s_m", 12.0);
    config.parallel_side_margin_m =
        declare_parameter<double>("parallel_side_margin_m", 4.0);
    config.parallel_follow_enabled =
        declare_parameter<bool>("parallel_follow_enabled", false);
    config.parallel_follow_s_m =
        declare_parameter<double>("parallel_follow_s_m", 12.0);
    config.parallel_follow_lateral_width_m =
        declare_parameter<double>("parallel_follow_lateral_width_m", 1.20);
    config.side_yield_s_m = declare_parameter<double>("side_yield_s_m", 0.30);
    config.side_by_side_target_gap_m =
        declare_parameter<double>("side_by_side_target_gap_m", 0.75);
    config.side_by_side_shift_distance_m =
        declare_parameter<double>("side_by_side_shift_distance_m", 7.0);
    config.side_by_side_speed_cap_mps =
        declare_parameter<double>("side_by_side_speed_cap_mps", 7.5);
    config.corner_side_yield_curvature_m_inv =
        declare_parameter<double>("corner_side_yield_curvature_m_inv", 0.05);
    config.corner_side_yield_lookahead_m =
        declare_parameter<double>("corner_side_yield_lookahead_m", 10.0);
    config.corner_side_yield_wall_clearance_m =
        declare_parameter<double>("corner_side_yield_wall_clearance_m", 0.55);
    config.corner_yield_target_d_m =
        declare_parameter<double>("corner_yield_target_d_m", 0.0);
    config.corner_yield_rejoin_gap_m =
        declare_parameter<double>("corner_yield_rejoin_gap_m", 5.5);
    config.yield_rejoin_wall_clearance_m =
        declare_parameter<double>("yield_rejoin_wall_clearance_m", 0.25);
    config.recovery_release_lateral_error_m =
        declare_parameter<double>("recovery_release_lateral_error_m", 0.60);
    config.yield_release_lateral_error_m =
        declare_parameter<double>("yield_release_lateral_error_m", 0.60);
    config.corner_follow_speed_margin_mps =
        declare_parameter<double>("corner_follow_speed_margin_mps", 0.20);
    config.corner_yield_v_max_mps =
        declare_parameter<double>("corner_yield_v_max_mps", 3.0);
    config.straight_only_overtake_enabled =
        declare_parameter<bool>("straight_only_overtake_enabled", true);
    config.straight_overtake_max_curvature_m_inv = declare_parameter<double>(
        "straight_overtake_max_curvature_m_inv", 0.025);
    config.straight_overtake_lookahead_m =
        declare_parameter<double>("straight_overtake_lookahead_m", 12.0);
    config.straight_overtake_release_hysteresis_m_inv =
        declare_parameter<double>("straight_overtake_release_hysteresis_m_inv",
                                  0.005);
    config.gentle_curve_safe_pass_enabled =
        declare_parameter<bool>("gentle_curve_safe_pass_enabled", false);
    config.gentle_curve_safe_pass_max_curvature_m_inv =
        declare_parameter<double>("gentle_curve_safe_pass_max_curvature_m_inv",
                                  0.0);
    config.gentle_curve_safe_pass_v_max_mps =
        declare_parameter<double>("gentle_curve_safe_pass_v_max_mps", 0.0);
    config.gentle_curve_safe_pass_max_lateral_displacement_m =
        declare_parameter<double>(
            "gentle_curve_safe_pass_max_lateral_displacement_m", 0.0);
    config.gentle_curve_safe_pass_max_lateral_accel_mps2 =
        declare_parameter<double>(
            "gentle_curve_safe_pass_max_lateral_accel_mps2", 0.0);
    config.gentle_curve_safe_pass_max_cbf_slack =
        declare_parameter<double>("gentle_curve_safe_pass_max_cbf_slack", 0.0);
    config.gentle_curve_safe_pass_bypass_mode_hold_enabled =
        declare_parameter<bool>(
            "gentle_curve_safe_pass_bypass_mode_hold_enabled", false);
    config.overtake_permission_profile_enabled =
        declare_parameter<bool>("overtake_permission_profile_enabled", true);
    config.default_overtake_allowed =
        declare_parameter<bool>("default_overtake_allowed", true);
    config.overtake_permission_lookahead_m =
        declare_parameter<double>("overtake_permission_lookahead_m", 8.0);
    config.slow_front_exception_enabled =
        declare_parameter<bool>("slow_front_exception_enabled", true);
    config.slow_front_permission_exception_enabled = declare_parameter<bool>(
        "slow_front_permission_exception_enabled", false);
    config.slow_front_exception_speed_mps =
        declare_parameter<double>("slow_front_exception_speed_mps", 1.0);
    config.slow_front_exception_distance_m =
        declare_parameter<double>("slow_front_exception_distance_m", 8.0);
    config.slow_front_exception_required_cycles =
        declare_parameter<int>("slow_front_exception_required_cycles", 3);
    config.slow_front_exception_max_start_curvature_m_inv =
        declare_parameter<double>(
            "slow_front_exception_max_start_curvature_m_inv", 0.0);
    config.slow_obstacle_chain_enabled =
        declare_parameter<bool>("slow_obstacle_chain_enabled", true);
    config.slow_obstacle_chain_distance_m =
        declare_parameter<double>("slow_obstacle_chain_distance_m", 12.0);
    config.early_stationary_parallel_pass_enabled = declare_parameter<bool>(
        "early_stationary_parallel_pass_enabled", false);
    config.early_stationary_parallel_permission_exception_enabled =
        declare_parameter<bool>(
            "early_stationary_parallel_permission_exception_enabled", false);
    config.early_stationary_parallel_pass_distance_m =
        declare_parameter<double>("early_stationary_parallel_pass_distance_m",
                                  8.0);
    config.early_stationary_parallel_pass_lateral_width_m =
        declare_parameter<double>(
            "early_stationary_parallel_pass_lateral_width_m", 1.5);
    config.start_grid_target_enabled =
        declare_parameter<bool>("start_grid_target_enabled", false);
    config.start_grid_target_window_sec =
        declare_parameter<double>("start_grid_target_window_sec", 5.0);
    config.start_grid_target_window_distance_m =
        declare_parameter<double>("start_grid_target_window_distance_m", 8.0);
    config.start_grid_target_max_ego_speed_mps =
        declare_parameter<double>("start_grid_target_max_ego_speed_mps", 3.0);
    config.start_grid_target_min_delta_s_m =
        declare_parameter<double>("start_grid_target_min_delta_s_m", -1.0);
    config.start_grid_target_max_delta_s_m =
        declare_parameter<double>("start_grid_target_max_delta_s_m", 8.0);
    config.start_grid_target_lateral_width_m =
        declare_parameter<double>("start_grid_target_lateral_width_m", 1.5);
    config.start_grid_stationary_confirmation_sec = declare_parameter<double>(
        "start_grid_stationary_confirmation_sec", 1.0);
    config.start_grid_stationary_confirmation_distance_m =
        declare_parameter<double>(
            "start_grid_stationary_confirmation_distance_m", 3.0);
    config.start_grid_attack_follow_v_max_mps =
        declare_parameter<double>("start_grid_attack_follow_v_max_mps", 3.0);
    config.start_grid_uncommitted_hold_max_lateral_drift_m =
        declare_parameter<double>(
            "start_grid_uncommitted_hold_max_lateral_drift_m", 0.15);
    config.start_grid_hold_correction_distance_m =
        declare_parameter<double>("start_grid_hold_correction_distance_m", 3.0);
    config.start_grid_moving_pass_max_lateral_displacement_m =
        declare_parameter<double>(
            "start_grid_moving_pass_max_lateral_displacement_m", 2.2);
    config.stationary_no_pass_safe_pass_enabled =
        declare_parameter<bool>("stationary_no_pass_safe_pass_enabled", false);
    config.stationary_no_pass_safe_pass_max_curvature_m_inv =
        declare_parameter<double>(
            "stationary_no_pass_safe_pass_max_curvature_m_inv", 0.0);
    config.stationary_no_pass_safe_pass_v_max_mps = declare_parameter<double>(
        "stationary_no_pass_safe_pass_v_max_mps", 0.0);
    config.stationary_no_pass_safe_pass_max_lateral_displacement_m =
        declare_parameter<double>(
            "stationary_no_pass_safe_pass_max_lateral_displacement_m", 0.0);
    config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 =
        declare_parameter<double>(
            "stationary_no_pass_safe_pass_max_lateral_accel_mps2", 0.0);
    config.stationary_no_pass_safe_pass_max_cbf_slack =
        declare_parameter<double>("stationary_no_pass_safe_pass_max_cbf_slack",
                                  0.0);
    config.braking_follow_enabled =
        declare_parameter<bool>("braking_follow_enabled", false);
    config.braking_follow_max_distance_m =
        declare_parameter<double>("braking_follow_max_distance_m", 0.0);
    config.braking_follow_max_target_speed_mps =
        declare_parameter<double>("braking_follow_max_target_speed_mps", 10.0);
    config.braking_follow_trigger_margin_m =
        declare_parameter<double>("braking_follow_trigger_margin_m", 0.0);
    config.braking_follow_ttc_threshold_sec =
        declare_parameter<double>("braking_follow_ttc_threshold_sec", 0.0);
    config.large_lateral_error_threshold_m =
        declare_parameter<double>("large_lateral_error_threshold_m", 0.60);
    config.large_lateral_error_v_max_mps =
        declare_parameter<double>("large_lateral_error_v_max_mps", 2.5);
    config.min_pass_gap_m = declare_parameter<double>("min_pass_gap_m", 1.80);
    config.pass_gap_hysteresis_m =
        declare_parameter<double>("pass_gap_hysteresis_m", 0.25);
    config.dynamic_pass_candidate_enabled =
        declare_parameter<bool>("dynamic_pass_candidate_enabled", true);
    config.yield_speed_margin_mps =
        declare_parameter<double>("yield_speed_margin_mps", 0.60);
    config.yield_min_speed_cap_mps =
        declare_parameter<double>("yield_min_speed_cap_mps", 0.50);
    config.yield_rejoin_gap_m =
        declare_parameter<double>("yield_rejoin_gap_m", 3.0);
    config.max_brake_decel_mps2 =
        declare_parameter<double>("max_brake_decel_mps2", 1.0);
    config.longitudinal_response_delay_sec =
        declare_parameter<double>("longitudinal_response_delay_sec", 0.25);
    config.stationary_obstacle_speed_threshold_mps = declare_parameter<double>(
        "stationary_obstacle_speed_threshold_mps", 0.30);
    config.moving_pass_reachability_enabled =
        declare_parameter<bool>("moving_pass_reachability_enabled", true);
    config.moving_pass_max_completion_time_sec =
        declare_parameter<double>("moving_pass_max_completion_time_sec", 20.0);
    config.moving_pass_min_closing_speed_mps =
        declare_parameter<double>("moving_pass_min_closing_speed_mps", 0.05);
    config.reentry_gate_enabled =
        declare_parameter<bool>("reentry_gate_enabled", true);
    config.reentry_safe_cycles =
        declare_parameter<int>("reentry_safe_cycles", 5);
    config.reentry_min_safety_margin_h =
        declare_parameter<double>("reentry_min_safety_margin_h", 0.30);
    config.reentry_evaluation_horizon_sec =
        declare_parameter<double>("reentry_evaluation_horizon_sec", 4.0);
    config.reentry_v2x_snapshot_stale_time_sec =
        declare_parameter<double>("reentry_v2x_snapshot_stale_time_sec", 0.50);
    config.reentry_completion_lateral_error_m =
        declare_parameter<double>("reentry_completion_lateral_error_m", 0.20);
    config.reentry_completion_rearm_lateral_error_m = declare_parameter<double>(
        "reentry_completion_rearm_lateral_error_m", 0.30);
    config.reentry_hold_v_max_mps =
        declare_parameter<double>("reentry_hold_v_max_mps", 0.50);
    config.reentry_mpc_degraded_hold_v_max_mps =
        declare_parameter<double>("reentry_mpc_degraded_hold_v_max_mps", 3.0);
    config.post_abort_curve_hold_v_max_mps =
        declare_parameter<double>("post_abort_curve_hold_v_max_mps", 3.0);
    config.reentry_mpc_latency_degraded_enter_samples =
        declare_parameter<int>("reentry_mpc_latency_degraded_enter_samples", 1);
    config.reentry_mpc_unhealthy_enter_samples =
        declare_parameter<int>("reentry_mpc_unhealthy_enter_samples", 2);
    config.reentry_mpc_healthy_release_samples =
        declare_parameter<int>("reentry_mpc_healthy_release_samples", 3);
    config.reentry_require_mpc_health =
        declare_parameter<bool>("reentry_require_mpc_health", true);
    config.left_offset_m = declare_parameter<double>("left_offset_m", 0.70);
    config.right_offset_m = declare_parameter<double>("right_offset_m", -0.70);
    config.pass_target_policy = declare_parameter<std::string>(
        "pass_target_policy", "legacy_fixed_offset");
    config.overtake_lateral_profile_mode = declare_parameter<std::string>(
        "overtake_lateral_profile_mode", "legacy");
    config.pass_horizon_publish_mode = declare_parameter<std::string>(
        "pass_horizon_publish_mode", "prepare_and_overtake");
    config.localized_avoidance_start_before_target_m =
        declare_parameter<double>("localized_avoidance_start_before_target_m",
                                  6.0);
    config.localized_avoidance_full_offset_before_target_m =
        declare_parameter<double>(
            "localized_avoidance_full_offset_before_target_m", 2.0);
    config.localized_avoidance_hold_after_target_m = declare_parameter<double>(
        "localized_avoidance_hold_after_target_m", 5.0);
    config.localized_avoidance_merge_distance_m =
        declare_parameter<double>("localized_avoidance_merge_distance_m", 8.0);
    config.attack_follow_tracking_wheelbase_m =
        declare_parameter<double>("attack_follow_tracking_wheelbase_m", 1.087);
    config.attack_follow_max_steering_angle_rad =
        declare_parameter<double>("attack_follow_max_steering_angle_rad",
                                  kVehicleHardSteeringTireAngleRad);
    config.attack_follow_max_steering_rate_radps = declare_parameter<double>(
        "attack_follow_max_steering_rate_radps", kVehicleHardSteeringRateRadps);
    config.attack_follow_steering_tire_angle_gain = declare_parameter<double>(
        "attack_follow_steering_tire_angle_gain", 1.639);
    config.attack_follow_steering_rate_reserve_ratio =
        declare_parameter<double>("attack_follow_steering_rate_reserve_ratio",
                                  0.80);
    config.attack_follow_min_spatial_horizon_m =
        declare_parameter<double>("attack_follow_min_spatial_horizon_m", 0.55);
    config.attack_follow_max_evaluation_horizon_sec = declare_parameter<double>(
        "attack_follow_max_evaluation_horizon_sec", 4.0);
    config.lateral_override_lookahead_gain =
        declare_parameter<double>("lateral_override_lookahead_gain", 0.5);
    config.lateral_override_lookahead_min_distance_m =
        declare_parameter<double>("lateral_override_lookahead_min_distance_m",
                                  3.5);
    config.moving_lateral_override_max_evaluation_horizon_sec =
        declare_parameter<double>(
            "moving_lateral_override_max_evaluation_horizon_sec", 6.0);
    config.lateral_override_max_evaluation_horizon_sec =
        declare_parameter<double>("lateral_override_max_evaluation_horizon_sec",
                                  20.0);
    config.lateral_override_execution_speed_reserve_sec =
        declare_parameter<double>(
            "lateral_override_execution_speed_reserve_sec", 0.10);
    config.maneuver_latch_min_hold_sec =
        declare_parameter<double>("maneuver_latch_min_hold_sec", 1.0);
    config.maneuver_latch_target_update_alpha =
        declare_parameter<double>("maneuver_latch_target_update_alpha", 1.0);
    config.unstarted_pass_target_release_enabled =
        declare_parameter<bool>("unstarted_pass_target_release_enabled", true);
    config.unstarted_pass_target_release_min_gap_m = declare_parameter<double>(
        "unstarted_pass_target_release_min_gap_m", 8.0);
    config.unstarted_pass_target_release_min_opening_speed_mps =
        declare_parameter<double>(
            "unstarted_pass_target_release_min_opening_speed_mps", 1.0);
    config.unstarted_pass_target_release_max_lateral_progress_m =
        declare_parameter<double>(
            "unstarted_pass_target_release_max_lateral_progress_m", 0.10);
    config.unstarted_pass_target_release_required_cycles =
        declare_parameter<int>("unstarted_pass_target_release_required_cycles",
                               5);
    config.prepare_distance_m =
        declare_parameter<double>("prepare_distance_m", 8.0);
    config.merge_distance_m =
        declare_parameter<double>("merge_distance_m", 12.0);
    config.follow_speed_margin_mps =
        declare_parameter<double>("follow_speed_margin_mps", 0.20);
    config.follow_gap_closing_enabled =
        declare_parameter<bool>("follow_gap_closing_enabled", false);
    config.follow_gap_closing_target_gap_m =
        declare_parameter<double>("follow_gap_closing_target_gap_m", 5.0);
    config.follow_gap_closing_engage_gap_m =
        declare_parameter<double>("follow_gap_closing_engage_gap_m", 6.0);
    config.follow_gap_closing_speed_gain_per_m =
        declare_parameter<double>("follow_gap_closing_speed_gain_per_m", 0.10);
    config.follow_gap_closing_max_speed_bonus_mps = declare_parameter<double>(
        "follow_gap_closing_max_speed_bonus_mps", 0.30);
    config.follow_gap_closing_assumed_accel_mps2 =
        declare_parameter<double>("follow_gap_closing_assumed_accel_mps2", 3.0);
    config.pass_speed_cap_mps =
        declare_parameter<double>("pass_speed_cap_mps", 10.0);
    config.pass_assumed_accel_mps2 =
        declare_parameter<double>("pass_assumed_accel_mps2", 3.0);
    config.pass_target_lateral_margin_m =
        declare_parameter<double>("pass_target_lateral_margin_m", 0.10);
    config.pass_lateral_first_stationary_creep_v_max_mps =
        declare_parameter<double>(
            "pass_lateral_first_stationary_creep_v_max_mps", 0.75);
    config.pass_lateral_tracking_lag_threshold_m = declare_parameter<double>(
        "pass_lateral_tracking_lag_threshold_m", 0.05);
    config.pass_lateral_tracking_lag_speed_cap_mps = declare_parameter<double>(
        "pass_lateral_tracking_lag_speed_cap_mps", 0.35);
    config.recovery_v_max_mps =
        declare_parameter<double>("recovery_v_max_mps", 3.0);
    config.recovery_assumed_accel_mps2 =
        declare_parameter<double>("recovery_assumed_accel_mps2", 3.0);
    config.wall_margin_recovery_v_max_mps =
        declare_parameter<double>("wall_margin_recovery_v_max_mps", 0.5);
    config.outside_corridor_recovery_centering_time_sec =
        declare_parameter<double>(
            "outside_corridor_recovery_centering_time_sec", 1.0);
    config.v_passthrough_mps =
        declare_parameter<double>("v_passthrough_mps", 50.0);
    config.d_min_m = declare_parameter<double>("d_min_m", -1.35);
    config.d_max_m = declare_parameter<double>("d_max_m", 1.35);
    config.min_wall_margin_m =
        declare_parameter<double>("min_wall_margin_m", 0.50);
    config.wall_footprint_check_enabled =
        declare_parameter<bool>("wall_footprint_check_enabled", true);
    config.ego_front_extent_m =
        declare_parameter<double>("ego_front_extent_m", 1.554);
    config.ego_rear_extent_m =
        declare_parameter<double>("ego_rear_extent_m", 0.510);
    config.ego_half_width_m =
        declare_parameter<double>("ego_half_width_m", 0.650);
    config.wall_localization_uncertainty_m =
        declare_parameter<double>("wall_localization_uncertainty_m", 0.250);
    config.wall_footprint_max_sample_distance_m = declare_parameter<double>(
        "wall_footprint_max_sample_distance_m", 0.250);
    config.wall_footprint_max_sample_yaw_rad =
        declare_parameter<double>("wall_footprint_max_sample_yaw_rad", 0.050);
    config.safety_ellipse_a_m =
        declare_parameter<double>("safety_ellipse_a_m", 3.0);
    config.safety_ellipse_b_m =
        declare_parameter<double>("safety_ellipse_b_m", 1.8);
    config.min_ellipse_h = declare_parameter<double>("min_ellipse_h", 0.20);
    config.pass_safe_required_cycles =
        declare_parameter<double>("pass_safe_required_cycles", 5.0);
    config.merge_front_gap_m =
        declare_parameter<double>("merge_front_gap_m", 6.0);
    config.abort_timeout_sec =
        declare_parameter<double>("abort_timeout_sec", 5.0);
    config.min_mode_hold_time_sec =
        declare_parameter<double>("min_mode_hold_time_sec", 0.60);
    config.keep_mode_bonus = declare_parameter<double>("keep_mode_bonus", 25.0);
    config.lateral_target_max_step_m =
        declare_parameter<double>("lateral_target_max_step_m", 0.25);
    config.high_speed_curve_lateral_hold_enabled =
        declare_parameter<bool>("high_speed_curve_lateral_hold_enabled", true);
    config.high_speed_curve_lateral_hold_min_speed_mps =
        declare_parameter<double>("high_speed_curve_lateral_hold_min_speed_mps",
                                  4.0);
    config.high_speed_curve_lateral_hold_release_speed_mps =
        declare_parameter<double>(
            "high_speed_curve_lateral_hold_release_speed_mps", 2.5);
    config.high_speed_curve_lateral_hold_release_curvature_m_inv =
        declare_parameter<double>(
            "high_speed_curve_lateral_hold_release_curvature_m_inv", 0.025);
    config.preemptive_wall_recovery_min_curvature_m_inv =
        declare_parameter<double>(
            "preemptive_wall_recovery_min_curvature_m_inv", 0.50);
    config.speed_only_fallback_enabled =
        declare_parameter<bool>("speed_only_fallback_enabled", true);
    config.speed_only_fallback_v_max_mps =
        declare_parameter<double>("speed_only_fallback_v_max_mps", 0.5);
    config.normal_recovery_speed_only_v_max_mps =
        declare_parameter<double>("normal_recovery_speed_only_v_max_mps", 10.0);
    config.opponent_collision_fallback_v_max_mps =
        declare_parameter<double>("opponent_collision_fallback_v_max_mps", 0.5);
    config.side_by_side_leader_priority_enabled =
        declare_parameter<bool>("side_by_side_leader_priority_enabled", true);
    config.side_by_side_leader_priority_enter_s_m = declare_parameter<double>(
        "side_by_side_leader_priority_enter_s_m", 1.0);
    config.side_by_side_leader_priority_release_s_m = declare_parameter<double>(
        "side_by_side_leader_priority_release_s_m", 0.3);
    config.side_by_side_leader_priority_hold_sec =
        declare_parameter<double>("side_by_side_leader_priority_hold_sec", 1.0);
    config.side_by_side_leader_priority_v_max_mps = declare_parameter<double>(
        "side_by_side_leader_priority_v_max_mps", 3.0);
    config.wall_risk_speed_guard_enabled =
        declare_parameter<bool>("wall_risk_speed_guard_enabled", true);
    config.wall_soft_margin_m =
        declare_parameter<double>("wall_soft_margin_m", 0.25);
    config.wall_risk_v_max_mps =
        declare_parameter<double>("wall_risk_v_max_mps", 0.5);
    config.mpc_health_speed_guard_enabled =
        declare_parameter<bool>("mpc_health_speed_guard_enabled", true);
    config.mpc_health_clean_free_run_soft_guard_bypass_enabled =
        declare_parameter<bool>(
            "mpc_health_clean_free_run_soft_guard_bypass_enabled", false);
    config.mpc_health_inactive_mpc_free_run_bypass_enabled =
        declare_parameter<bool>(
            "mpc_health_inactive_mpc_free_run_bypass_enabled", false);
    config.mpc_health_infeasible_count_threshold =
        declare_parameter<int>("mpc_health_infeasible_count_threshold", 1);
    config.mpc_health_solve_time_warn_ms =
        declare_parameter<double>("mpc_health_solve_time_warn_ms", 80.0);
    config.mpc_health_v_max_mps =
        declare_parameter<double>("mpc_health_v_max_mps", 0.5);
    config.mpc_health_stale_time_sec =
        declare_parameter<double>("mpc_health_stale_time_sec", 0.60);
    config.recovery_speed_guard_enabled =
        declare_parameter<bool>("recovery_speed_guard_enabled", true);
    config.recovery_speed_guard_v_max_mps =
        declare_parameter<double>("recovery_speed_guard_v_max_mps", 0.5);
    config.section_safety_profile_enabled =
        declare_parameter<bool>("section_safety_profile_enabled", true);
    config.safe_stop_enabled =
        declare_parameter<bool>("safe_stop_enabled", true);
    config.safe_stop_v_mps = declare_parameter<double>("safe_stop_v_mps", 0.20);
    config.safe_stop_trigger_cycles =
        declare_parameter<int>("safe_stop_trigger_cycles", 1);
    config.start_grace_safe_stop_enabled =
        declare_parameter<bool>("start_grace_safe_stop_enabled", true);
    config.start_grace_duration_sec =
        declare_parameter<double>("start_grace_duration_sec", 8.0);
    config.start_grace_max_speed_mps =
        declare_parameter<double>("start_grace_max_speed_mps", 1.5);
    config.safe_stop_release_cycles =
        declare_parameter<int>("safe_stop_release_cycles", 5);
    config.safe_stop_release_front_gap_m =
        declare_parameter<double>("safe_stop_release_front_gap_m", 5.0);
    config.safe_stop_release_wall_clearance_m =
        declare_parameter<double>("safe_stop_release_wall_clearance_m", 0.20);
    config.safe_stop_lateral_error_threshold_m =
        declare_parameter<double>("safe_stop_lateral_error_threshold_m", 0.40);
    config.safe_stop_release_speed_mps =
        declare_parameter<double>("safe_stop_release_speed_mps", 0.50);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    config.control_rate_hz = control_rate_hz_;
    safety_constraint_normal_speed_limit_mps_ = declare_parameter<double>(
        "safety_constraint_normal_speed_limit_mps", 10.0);
    const int safety_constraint_release_safe_cycles =
        declare_parameter<int>("safety_constraint_release_safe_cycles", 3);
    safety_constraint_release_safe_cycles_ =
        std::max(1, safety_constraint_release_safe_cycles);
    // FOLLOW/PASSの安全速度上限は、更新された相手車両観測ごとに微小に
    // 変化する。完全一致を要求すると、安全な解除候補が連続していても
    // release counterが毎周期resetされ、停止したままになる。確認窓内の
    // 最も厳しいconstraintを保持しつつ連続確認する。
    safety_constraint_release_gate_ = SafetyConstraintReleaseGate(
        safety_constraint_release_safe_cycles, true);
    supervisor_v2_constraint_release_gate_ = SafetyConstraintReleaseGate(
        safety_constraint_release_safe_cycles, true);
    safety_constraint_maximum_brake_decel_mps2_ = config.max_brake_decel_mps2;
    // race arm前はproposal診断だけを許し、Planner/Mux境界にも正速度capを
    // 渡さない。通常SAFE_STOPのcreep速度とは別の外部authority停止契約である。
    pre_arm_stop_speed_mps_ = 0.0;
    horizon_dt_sec_ = config.horizon_dt_sec;

    if (const auto invalid_reason = invalidLongitudinalSafetyConfig(config);
        invalid_reason.has_value()) {
      RCLCPP_ERROR(
          get_logger(),
          "invalid overtake longitudinal safety configuration: %s; disabling "
          "planner override so the baseline watchdog/controller remains active",
          invalid_reason->c_str());
      config.enabled = false;
    }

    FrenetFrame frame;
    std::string error;
    const auto reference_path =
        resolveReferencePath(reference_package, reference_csv);
    if (!frame.loadCsv(reference_path, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      config.enabled = false;
    } else {
      RCLCPP_INFO(
          get_logger(), "loaded overtake reference: %s (%zu points, %.2f m)",
          reference_path.c_str(), frame.reference().size(), frame.length());
      if (drivable_corridor_enabled) {
        const auto corridor_path = resolveReferencePath(
            drivable_corridor_package, drivable_corridor_csv);
        if (!frame.loadCorridorCsv(corridor_path, &error)) {
          RCLCPP_ERROR(
              get_logger(),
              "failed to load verified drivable corridor; disabling planner "
              "override: %s",
              error.c_str());
          config.enabled = false;
        } else {
          RCLCPP_INFO(get_logger(), "loaded overtake drivable corridor: %s",
                      corridor_path.c_str());
        }
      }
    }
    frame_ = frame;
    config.section_safety_rules = readSectionSafetyRules(frame_);
    config.overtake_permission_rules = readOvertakePermissionRules(
        frame_, overtake_permission_package, overtake_permission_csv);
    planner_config_ = config;
    mpc_health_stale_time_sec_ = config.mpc_health_stale_time_sec;
    opponent_stale_time_sec_ = config.opponent_stale_time_sec;
    reentry_v2x_snapshot_stale_time_sec_ =
        config.reentry_v2x_snapshot_stale_time_sec;
    mpc_health_infeasible_count_threshold_ =
        config.mpc_health_infeasible_count_threshold;
    mpc_health_solve_time_warn_ms_ = config.mpc_health_solve_time_warn_ms;
    core_ = std::make_unique<OvertakePlannerCore>(frame_, config);
    prearm_proposal_core_ =
        std::make_unique<OvertakePlannerCore>(frame_, config);

    // MPCへ渡すoverride配列と、evalwrapで拾うdebugトピックをpublishする。
    override_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/overtake/reference_override", rclcpp::QoS(1));
    safety_constraint_pub_ =
        create_publisher<multi_purpose_mpc_ros_msgs::msg::SafetyConstraint>(
            "/overtake/safety_constraint", rclcpp::QoS(1));
    authoritative_plan_pub_ =
        create_publisher<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>(
            "/overtake/plan", rclcpp::QoS(1));
    candidate_execution_request_pub_ = create_publisher<
        multi_purpose_mpc_ros_msgs::msg::CandidateExecutionRequest>(
        "/overtake/continuation/shadow/candidate_execution_request",
        rclcpp::QoS(rclcpp::KeepLast(8)).best_effort().durability_volatile());
    supervisor_v2_plan_pub_ =
        create_publisher<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>(
            "/overtake/v2/shadow/plan", rclcpp::QoS(1));
    supervisor_v2_constraint_pub_ =
        create_publisher<multi_purpose_mpc_ros_msgs::msg::SafetyConstraint>(
            "/overtake/v2/shadow/safety_constraint", rclcpp::QoS(1));
    mode_pub_ = create_publisher<std_msgs::msg::String>("/debug/overtake/mode",
                                                        rclcpp::QoS(1));
    metrics_pub_ = create_publisher<std_msgs::msg::String>(
        "/debug/overtake/metrics", rclcpp::QoS(1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom_ = *msg;
        });
    v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        "/v2x/vehicle_positions", rclcpp::QoS(1),
        [this](
            const v2x_msgs::msg::V2XVehiclePositionArray::ConstSharedPtr msg) {
          updateOpponents(*msg);
        });
    mpc_health_sub_ = create_subscription<std_msgs::msg::String>(
        "/mpc/speed_profile_debug", rclcpp::QoS(1),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          updateMpcHealth(*msg);
        });
    controller_tracking_status_sub_ = create_subscription<
        multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus>(
        "/hybrid_control/controller_tracking_status", rclcpp::QoS(1),
        [this](const multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus::
                   ConstSharedPtr msg) {
          updateControllerTrackingStatus(*msg);
        });
    if (pp_core_exact_snapshot_enabled_) {
      controller_execution_envelope_sub_ = create_subscription<
          multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope>(
          "/hybrid_control/pure_pursuit/execution_envelope", rclcpp::QoS(1),
          [this](const multi_purpose_mpc_ros_msgs::msg::
                     ControllerExecutionEnvelope::ConstSharedPtr msg) {
            updatePurePursuitExactSnapshot(*msg);
          });
    }
    motion_authority_grant_sub_ = create_subscription<
        multi_purpose_mpc_ros_msgs::msg::MotionAuthorityGrant>(
        "/hybrid_control/motion_authority_grant", rclcpp::QoS(1),
        [this](const multi_purpose_mpc_ros_msgs::msg::MotionAuthorityGrant::
                   ConstSharedPtr msg) { updateMotionAuthorityGrant(*msg); });
    if (race_arm_required_ && !race_arm_topic_.empty()) {
      auto race_arm_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      race_arm_qos.reliable().transient_local();
      race_arm_sub_ = create_subscription<std_msgs::msg::Bool>(
          race_arm_topic_, race_arm_qos,
          [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
            updateRaceArm(msg->data);
          });
    }
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
        [this]() { onTimer(); });
  }

private:
  struct TrackSample {
    // V2X位置の前回値。位置差分から他車速度を推定する。
    double stamp_sec{0.0};
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    bool has_prev{false};
  };

  struct DecisionLogSnapshot {
    BehaviorMode mode{BehaviorMode::FREE_RUN};
    CandidateType selected{CandidateType::FASTEST};
    bool blocked{false};
    bool side_by_side{false};
    bool corner_side_by_side{false};
    bool parallel_side_candidate{false};
    bool parallel_follow_candidate{false};
    bool parallel_follow_feasible{false};
    bool future_side_by_side{false};
    bool future_corner_side_by_side{false};
    bool future_yield_required{false};
    bool future_outer_wall_risk{false};
    bool can_pass_left{false};
    bool can_pass_right{false};
    bool pass_left_candidate_generated{false};
    bool pass_left_candidate_feasible{false};
    std::string pass_left_candidate_reject_reason{};
    bool pass_right_candidate_generated{false};
    bool pass_right_candidate_feasible{false};
    std::string pass_right_candidate_reject_reason{};
    bool maneuver_transaction_incomplete{false};
    CandidateType maneuver_transaction_pass_type{CandidateType::FASTEST};
    bool maneuver_transaction_safe_lateral_hold_active{false};
    std::string maneuver_target_id{};
    double maneuver_target_relative_s_m{
        std::numeric_limits<double>::infinity()};
    std::string maneuver_chain_tail_id{};
    double maneuver_chain_tail_relative_s_m{
        std::numeric_limits<double>::infinity()};
    bool active_override{false};
    double corner_abs_curvature{0.0};
    bool straight_overtake_start_allowed{true};
    double overtake_start_abs_curvature{0.0};
    std::string overtake_start_gate_reason{};
    bool gentle_curve_fresh_dynamic_gap_target{false};
    bool gentle_curve_safe_pass_start_approved{false};
    std::string gentle_curve_safe_pass_block_reason{};
    int pass_left_safe_cycles{0};
    int pass_right_safe_cycles{0};
    std::string pass_safe_cycle_reset_reason{};
    double follow_candidate_speed_cap_mps{
        std::numeric_limits<double>::quiet_NaN()};
    bool overtake_permission_allowed{true};
    std::string overtake_permission_section_name{};
    std::string overtake_permission_reason{};
    bool front_vehicle_low_speed{false};
    bool slow_front_exception_active{false};
    int slow_front_exception_count{0};
    bool slow_obstacle_chain_active{false};
    std::string slow_obstacle_chain_id{};
    double front_vehicle_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    bool pass_decision_frozen{false};
    std::string pass_decision_freeze_reason{};
    double future_wall_clearance_m{std::numeric_limits<double>::infinity()};
    double ego_wall_clearance_m{std::numeric_limits<double>::infinity()};
    bool early_wall_recovery_probe_requested{false};
    bool early_wall_recovery_probe_generated{false};
    bool early_wall_recovery_probe_feasible{false};
    std::string early_wall_recovery_probe_first_false{"not_evaluated"};
    std::string early_wall_recovery_reject_reason{};
    std::string front_vehicle_id{};
    std::string side_vehicle_id{};
    std::string parallel_side_vehicle_id{};
    std::string parallel_follow_vehicle_id{};
    bool leader_priority_active{false};
    bool leader_priority_latched{false};
    std::string leader_priority_id{};
    double leader_priority_delta_s{std::numeric_limits<double>::infinity()};
    std::string leader_priority_reason{};
    std::string pass_gap_reason{};
    std::string yield_reason{};
    std::string reason{};
    bool safe_stop_triggered{false};
    bool start_grace_active{false};
    bool safe_stop_release_ready{false};
    std::string safe_stop_reason{};
    std::string safe_stop_reject_reason{};
    int safe_stop_trigger_count{0};
    int safe_stop_hold_count{0};
    int safe_stop_release_count{0};
    bool speed_only_fallback_active{false};
    bool wall_risk_speed_guard_active{false};
    bool mpc_health_speed_guard_active{false};
    bool recovery_speed_guard_active{false};
    bool reentry_requested{false};
    bool reentry_permitted{false};
    int reentry_clear_cycles{0};
    std::string reentry_reason{};
    std::string reentry_primary_blocker_id{};
    bool lateral_target_hold_active{false};
    std::string lateral_target_hold_reason{};
    std::string speed_cap_reason{};
    std::string active_section_name{};
    std::string active_section_profile{};
    std::string active_section_role_policy{};
    double applied_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
    double wall_soft_margin_m{std::numeric_limits<double>::quiet_NaN()};
    bool mpc_health_valid{false};
    int mpc_infeasible_count{0};
    double mpc_solve_time_ms{std::numeric_limits<double>::quiet_NaN()};
  };

  // 入力: ROS builtin_interfaces::msg::Time。
  // 出力: 秒単位のdouble時刻。
  // 処理概要: odom/V2X stampの鮮度判定で扱いやすい形式へ変換する。
  double stampToSec(const builtin_interfaces::msg::Time &stamp) const {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  // 入力: FrenetFrameとwaypoint id。
  // 出力: waypointに対応するs[m]。範囲外ならnullopt。
  // 処理概要: section設定をwp番号で書いた場合に、参照線のsへ変換する。
  std::optional<double> sectionSFromWp(const FrenetFrame &frame,
                                       std::int64_t wp_id) const {
    if (frame.reference().empty() || wp_id < 0) {
      return std::nullopt;
    }
    const auto index = static_cast<std::size_t>(wp_id);
    if (index >= frame.reference().size()) {
      return std::nullopt;
    }
    return frame.reference()[index].s;
  }

  // 入力: 参照線frameとROS parameter群。
  // 出力: section safety rule配列。
  // 処理概要:
  // YAMLの区間安全設定を読み、wp指定またはs指定を統一したs区間へ変換する。
  std::vector<SectionSafetyRule>
  readSectionSafetyRules(const FrenetFrame &frame) {
    const auto names = declare_parameter<std::vector<std::string>>(
        "section_safety_names", std::vector<std::string>{});
    const auto profiles = declare_parameter<std::vector<std::string>>(
        "section_safety_profiles", std::vector<std::string>{});
    const auto role_policies = declare_parameter<std::vector<std::string>>(
        "section_safety_role_policies", std::vector<std::string>{});
    const auto start_s = declare_parameter<std::vector<double>>(
        "section_safety_start_s_m", std::vector<double>{});
    const auto end_s = declare_parameter<std::vector<double>>(
        "section_safety_end_s_m", std::vector<double>{});
    const auto start_wp = declare_parameter<std::vector<std::int64_t>>(
        "section_safety_start_wp", std::vector<std::int64_t>{});
    const auto end_wp = declare_parameter<std::vector<std::int64_t>>(
        "section_safety_end_wp", std::vector<std::int64_t>{});

    // 処理ブロック: 複数parameter配列の最大長を基準にrule候補を走査する。
    // 設計意図:
    // 一部の配列だけ短い設定でも、欠損を警告しながら有効なruleだけ採用する。
    const std::size_t n = std::max(
        {names.size(), profiles.size(), role_policies.size(), start_s.size(),
         end_s.size(), start_wp.size(), end_wp.size()});
    std::vector<SectionSafetyRule> rules;
    rules.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      std::optional<double> start;
      std::optional<double> end;
      if (i < start_s.size() && i < end_s.size()) {
        start = start_s[i];
        end = end_s[i];
      } else if (i < start_wp.size() && i < end_wp.size()) {
        start = sectionSFromWp(frame, start_wp[i]);
        end = sectionSFromWp(frame, end_wp[i]);
      }
      if (!start.has_value() || !end.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "skipping section safety rule %zu because start/end are "
                    "missing or invalid",
                    i);
        continue;
      }
      SectionSafetyRule rule;
      rule.name = i < names.size() && !names[i].empty()
                      ? names[i]
                      : "section_" + std::to_string(i);
      rule.s_start_m = start.value();
      rule.s_end_m = end.value();
      rule.profile =
          i < profiles.size() && !profiles[i].empty() ? profiles[i] : "default";
      rule.role_policy = i < role_policies.size() && !role_policies[i].empty()
                             ? role_policies[i]
                             : "default";
      rules.push_back(std::move(rule));
    }
    if (!rules.empty()) {
      RCLCPP_INFO(get_logger(), "loaded %zu overtake section safety rules",
                  rules.size());
    }
    return rules;
  }

  // 入力: 参照線frame、CSVのパッケージ名、CSVパス。
  // 出力: 追い越し許可区間rule配列。
  // 処理概要:
  // name,start_wp,end_wp,allow_overtake形式のCSVを読み、wp範囲をs範囲へ変換する。
  std::vector<OvertakePermissionRule>
  readOvertakePermissionRules(const FrenetFrame &frame,
                              const std::string &package_name,
                              const std::string &csv_path) {
    std::vector<OvertakePermissionRule> rules;
    if (csv_path.empty()) {
      return rules;
    }

    const auto resolved_path = resolveReferencePath(package_name, csv_path);
    std::ifstream file(resolved_path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(),
                  "overtake permission csv could not be opened: %s",
                  resolved_path.c_str());
      return rules;
    }

    std::string line;
    std::size_t line_number = 0;
    // 処理ブロック: CSVを1行ずつ検証してrule化する。
    // 設計意図:
    // 1行が壊れていてもnode全体は止めず、残りの有効な区間設定で走れるようにする。
    while (std::getline(file, line)) {
      ++line_number;
      line = trim(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }

      const auto columns = splitCsvLine(line);
      if (!columns.empty() && columns[0] == "name") {
        continue;
      }
      if (columns.size() < 4) {
        RCLCPP_WARN(get_logger(),
                    "skipping overtake permission line %zu: expected "
                    "name,start_wp,end_wp,allow_overtake",
                    line_number);
        continue;
      }

      const auto start_wp = parseInt64(columns[1]);
      const auto end_wp = parseInt64(columns[2]);
      const auto allow_overtake = parseBool(columns[3]);
      if (!start_wp.has_value() || !end_wp.has_value() ||
          !allow_overtake.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "skipping overtake permission line %zu: invalid wp or "
                    "allow_overtake value",
                    line_number);
        continue;
      }

      const auto start_s = sectionSFromWp(frame, start_wp.value());
      const auto end_s = sectionSFromWp(frame, end_wp.value());
      if (!start_s.has_value() || !end_s.has_value()) {
        RCLCPP_WARN(get_logger(),
                    "skipping overtake permission line %zu: wp out of "
                    "reference range",
                    line_number);
        continue;
      }

      OvertakePermissionRule rule;
      rule.name = columns[0].empty()
                      ? "permission_" + std::to_string(rules.size())
                      : columns[0];
      rule.s_start_m = start_s.value();
      rule.s_end_m = end_s.value();
      rule.allow_overtake = allow_overtake.value();
      rules.push_back(std::move(rule));
    }

    RCLCPP_INFO(get_logger(), "loaded %zu overtake permission rules from %s",
                rules.size(), resolved_path.c_str());
    return rules;
  }

  // 入力: /mpc/speed_profile_debug のJSON文字列。
  // 出力: なし。内部mpc_health_と受信時刻を更新する。
  // 処理概要: MPC infeasible回数とsolve
  // timeを抜き出し、planner側の速度guardへ渡す。
  void updateMpcHealth(const std_msgs::msg::String &msg) {
    MpcHealthStatus health;
    const auto infeasible_count =
        jsonNumberField(msg.data, "mpc_infeasible_count");
    const auto solve_time_ms = jsonNumberField(msg.data, "mpc_solve_time_ms");
    if (!infeasible_count.has_value() && !solve_time_ms.has_value()) {
      return;
    }
    health.valid = true;
    health.infeasible_count =
        infeasible_count.has_value()
            ? std::max(0,
                       static_cast<int>(std::llround(infeasible_count.value())))
            : 0;
    health.solve_time_ms =
        solve_time_ms.value_or(std::numeric_limits<double>::quiet_NaN());
    health.age_sec = 0.0;
    health.sample_sequence =
        mpc_health_sample_sequence_ == std::numeric_limits<std::uint64_t>::max()
            ? 1U
            : mpc_health_sample_sequence_ + 1U;
    mpc_health_sample_sequence_ = health.sample_sequence;
    mpc_health_ = health;
    last_mpc_health_sec_ = now().seconds();
  }

  // 入力: 現在時刻[sec]。
  // 出力: age_secを更新したMpcHealthStatus。
  // 処理概要:
  // health情報が無い場合はinvalidにし、古さはcore側のguard条件で判断できるようにする。
  MpcHealthStatus currentMpcHealth(double now_sec) const {
    auto health = mpc_health_;
    if (!health.valid || !last_mpc_health_sec_.has_value()) {
      health.valid = false;
      return health;
    }
    health.age_sec = now_sec - last_mpc_health_sec_.value();
    return health;
  }

  void updateControllerTrackingStatus(
      const multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus &msg) {
    controller_tracking_mpc_horizon_usable_ = msg.mpc_horizon_usable;
    controller_tracking_pp_command_fresh_ = msg.pp_command_fresh;
    controller_tracking_usable_ = msg.trajectory_tracking_usable;
    controller_tracking_safety_constraint_release_ready_ =
        msg.safety_constraint_release_ready;
    controller_tracking_attack_follow_stop_transport_release_ready_ =
        msg.attack_follow_stop_transport_release_ready;
    controller_tracking_pass_probe_transport_evidence_ =
        msg.pass_probe_transport_evidence;
    controller_tracking_pass_probe_exact_current_usable_ =
        msg.pass_probe_exact_current_usable;
    controller_tracking_pass_probe_lateral_stop_authority_token_ =
        msg.pass_probe_lateral_stop_authority_token;
    controller_tracking_plan_generation_ = msg.plan_generation;
    controller_tracking_reason_ = msg.reason;
    controller_tracking_command_age_sec_ = msg.command_age_sec;
    // 受信時刻でfresh扱いすると、古いPP commandをmuxが再publishしただけでも
    // controller proofが若返ってしまう。型付きstatusのheaderは実際に照合した
    // PP command stampなので、その時刻を鮮度契約へ使う。zero/NaN/futureは
    // controllerTrackingStatusFresh()がfail-closedで拒否する。
    last_controller_tracking_status_sec_ = stampToSec(msg.header.stamp);
  }

  void updatePurePursuitExactSnapshot(
      const multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope &msg) {
    const auto previous = pure_pursuit_exact_snapshot_;
    pure_pursuit_exact_snapshot_.reset();
    PurePursuitCandidateBinding expected;
    expected.race_arm_epoch = race_arm_epoch_;
    expected.planner_instance_id = planner_instance_id_;
    expected.attempt_id = last_authoritative_plan_attempt_id_;
    expected.target_vehicle_id = last_authoritative_plan_target_id_;
    expected.pass_direction = last_authoritative_plan_pass_direction_;
    expected.connector_transaction_id =
        last_authoritative_plan_connector_transaction_id_;
    expected.plan_stamp_sec = last_authoritative_plan_stamp_sec_;
    expected.plan_generation = last_authoritative_plan_generation_;
    expected.candidate_revision = last_authoritative_plan_candidate_revision_;
    expected.candidate_content_sha256 =
        last_authoritative_plan_candidate_content_sha256_;
    auto validated = validatePurePursuitExactSnapshotV2(
        msg, expected, now().seconds(), input_future_stamp_tolerance_sec_);
    if (!validated.valid) {
      pure_pursuit_exact_snapshot_reason_ = validated.reason;
      return;
    }
    if (pure_pursuit_exact_producer_instance_id_ ==
            validated.producer_instance_id &&
        validated.command_sequence <= pure_pursuit_exact_command_sequence_) {
      const bool same_tuple_mutated =
          previous.has_value() &&
          previous->producer_instance_id == validated.producer_instance_id &&
          previous->command_sequence == validated.command_sequence &&
          previous->bounded_geometry_sha256 !=
              validated.bounded_geometry_sha256;
      pure_pursuit_exact_snapshot_reason_ =
          same_tuple_mutated ? "same_tuple_geometry_mutation"
                             : "command_sequence_not_advanced";
      return;
    }
    pure_pursuit_exact_producer_instance_id_ = validated.producer_instance_id;
    pure_pursuit_exact_command_sequence_ = validated.command_sequence;
    pure_pursuit_exact_snapshot_reason_ = "complete";
    pure_pursuit_exact_snapshot_ = std::move(validated);
  }

  void updateMotionAuthorityGrant(
      const multi_purpose_mpc_ros_msgs::msg::MotionAuthorityGrant &msg) {
    const auto &key = msg.plan_sample_key;
    const auto &warmup_key = msg.warmup_plan_sample_key;
    const bool sequence_advanced =
        msg.grant_issuer_instance_id != motion_grant_issuer_instance_id_ ||
        msg.grant_sequence > motion_grant_sequence_;
    const std::uint32_t predecessor_generation =
        key.plan_generation <= 1U ? 16777215U : key.plan_generation - 1U;
    const bool warmup_same_sample =
        last_pass_probe_warmup_valid_ && warmup_key == key &&
        msg.warmup_candidate_revision == msg.candidate_revision &&
        msg.warmup_candidate_content_sha256 == msg.candidate_content_sha256 &&
        warmup_key.race_arm_epoch == last_pass_probe_warmup_race_arm_epoch_ &&
        warmup_key.planner_instance_id ==
            last_pass_probe_warmup_planner_instance_id_ &&
        warmup_key.plan_generation == last_pass_probe_warmup_plan_generation_ &&
        warmup_key.attempt_id == last_pass_probe_warmup_attempt_id_ &&
        warmup_key.target_vehicle_id ==
            last_pass_probe_warmup_target_vehicle_id_ &&
        warmup_key.pass_direction == last_pass_probe_warmup_pass_direction_ &&
        warmup_key.connector_transaction_id ==
            last_pass_probe_warmup_connector_transaction_id_ &&
        stampToSec(warmup_key.plan_stamp) ==
            last_pass_probe_warmup_stamp_sec_ &&
        msg.warmup_candidate_revision ==
            last_pass_probe_warmup_candidate_revision_ &&
        msg.warmup_candidate_content_sha256 ==
            last_pass_probe_warmup_candidate_content_sha256_ &&
        msg.warmup_lateral_stop_authority_token ==
            last_pass_probe_warmup_authority_token_;
    const bool warmup_direct_transaction_predecessor =
        last_pass_probe_warmup_valid_ &&
        warmup_key.race_arm_epoch == key.race_arm_epoch &&
        warmup_key.planner_instance_id == key.planner_instance_id &&
        warmup_key.plan_generation == predecessor_generation &&
        warmup_key.attempt_id == key.attempt_id &&
        warmup_key.target_vehicle_id == key.target_vehicle_id &&
        warmup_key.pass_direction == key.pass_direction &&
        warmup_key.connector_transaction_id == key.connector_transaction_id &&
        stampToSec(warmup_key.plan_stamp) < stampToSec(key.plan_stamp) &&
        msg.warmup_candidate_revision < msg.candidate_revision &&
        msg.warmup_candidate_content_sha256 == msg.candidate_content_sha256 &&
        warmup_key.race_arm_epoch == last_pass_probe_warmup_race_arm_epoch_ &&
        warmup_key.planner_instance_id ==
            last_pass_probe_warmup_planner_instance_id_ &&
        warmup_key.plan_generation == last_pass_probe_warmup_plan_generation_ &&
        warmup_key.attempt_id == last_pass_probe_warmup_attempt_id_ &&
        warmup_key.target_vehicle_id ==
            last_pass_probe_warmup_target_vehicle_id_ &&
        warmup_key.pass_direction == last_pass_probe_warmup_pass_direction_ &&
        warmup_key.connector_transaction_id ==
            last_pass_probe_warmup_connector_transaction_id_ &&
        stampToSec(warmup_key.plan_stamp) ==
            last_pass_probe_warmup_stamp_sec_ &&
        msg.warmup_candidate_revision ==
            last_pass_probe_warmup_candidate_revision_ &&
        msg.warmup_candidate_content_sha256 ==
            last_pass_probe_warmup_candidate_content_sha256_ &&
        msg.warmup_lateral_stop_authority_token ==
            last_pass_probe_warmup_authority_token_;
    const bool exact_authoritative_sample =
        msg.schema_version == 1U && msg.valid && sequence_advanced &&
        msg.grant_issuer_instance_id != 0U && msg.grant_sequence != 0U &&
        key.race_arm_epoch == race_arm_epoch_ &&
        key.planner_instance_id == planner_instance_id_ &&
        key.plan_generation == last_authoritative_plan_generation_ &&
        key.attempt_id == last_authoritative_plan_attempt_id_ &&
        key.target_vehicle_id == last_authoritative_plan_target_id_ &&
        key.pass_direction == last_authoritative_plan_pass_direction_ &&
        key.connector_transaction_id ==
            last_authoritative_plan_connector_transaction_id_ &&
        msg.candidate_revision == last_authoritative_plan_candidate_revision_ &&
        msg.candidate_content_sha256 ==
            last_authoritative_plan_candidate_content_sha256_ &&
        (warmup_same_sample || warmup_direct_transaction_predecessor) &&
        msg.warmup_lateral_stop_authority_token != 0U &&
        msg.warmup_pp_producer_instance_id == msg.pp_producer_instance_id &&
        msg.warmup_pp_command_sequence != 0U &&
        msg.warmup_pp_command_sequence < msg.pp_command_sequence &&
        stampToSec(key.plan_stamp) == last_authoritative_plan_stamp_sec_ &&
        msg.phase == multi_purpose_mpc_ros_msgs::msg::OvertakePlan::PASSING &&
        std::isfinite(msg.lease_duration_sec) &&
        msg.lease_duration_sec > 0.0F && msg.lease_duration_sec <= 0.10F;
    motion_grant_issuer_instance_id_ = msg.grant_issuer_instance_id;
    motion_grant_sequence_ = msg.grant_sequence;
    motion_grant_observed_ = exact_authoritative_sample;
    motion_grant_reason_ = exact_authoritative_sample ? "observed" : msg.reason;
    last_motion_grant_status_sec_ = stampToSec(msg.header.stamp);
  }

  void drainLatestControllerTrackingStatus() {
    if (!controller_tracking_status_sub_) {
      return;
    }
    // SafetyEvaluatorが20 Hz周期を超える計算時間を要した場合、single-thread
    // executorではtimer callbackが連続して選ばれ、QoS depth 1の最新statusが
    // subscription callbackへ渡る前に次のCore更新へ入ることがある。Core入力を
    // 確定する境界で待機中の最新1件を直接取り込み、計算負荷だけを理由に正常な
    // PASS proofがN-2へ見えることを防ぐ。takeできない周期は既存statusを使い、
    // 鮮度・generation・fault判定は後段で従来どおり行う。
    multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus latest;
    rclcpp::MessageInfo message_info;
    bool received = false;
    while (controller_tracking_status_sub_->take(latest, message_info)) {
      received = true;
    }
    if (received) {
      updateControllerTrackingStatus(latest);
    }
  }

  bool controllerTrackingStatusFresh(double now_sec) const {
    return last_controller_tracking_status_sec_.has_value() &&
           inputTimestampFresh(now_sec,
                               last_controller_tracking_status_sec_.value(),
                               controller_tracking_status_timeout_sec_,
                               input_future_stamp_tolerance_sec_) &&
           std::isfinite(controller_tracking_command_age_sec_) &&
           controller_tracking_command_age_sec_ >=
               -input_future_stamp_tolerance_sec_ &&
           controller_tracking_command_age_sec_ <=
               controller_tracking_status_timeout_sec_;
  }

  std::string controllerTrackingStatusReason(double now_sec) const {
    if (!last_controller_tracking_status_sec_.has_value()) {
      return "missing";
    }
    if (!controllerTrackingStatusFresh(now_sec)) {
      return "stale";
    }
    if (!controller_tracking_pp_command_fresh_) {
      return "pp_command_stale";
    }
    if (!controller_tracking_usable_) {
      return controller_tracking_reason_.empty() ? "unusable"
                                                 : controller_tracking_reason_;
    }
    if (controller_tracking_plan_generation_ != override_generation_) {
      return "generation_mismatch";
    }
    return "ready";
  }

  bool purePursuitPrimaryAndFresh(double now_sec) const {
    return controllerTrackingStatusReason(now_sec) == "ready";
  }

  bool purePursuitAttackFollowTransportUsable(double now_sec) const {
    // Muxが型付きで検証したprepared ATTACK_FOLLOW transport専用。commit後の
    // N-1配送継続はpurePursuitTrackingContinuityUsable()とtyped identityが
    // 所有し、初回PASS ACKやSTOP解除へこの診断reasonを流用しない。
    return controllerTrackingStatusFresh(now_sec) &&
           controller_tracking_pp_command_fresh_ &&
           controller_tracking_usable_ &&
           controller_tracking_reason_ ==
               "attack_follow_transport_continuity" &&
           purePursuitTransportTrackingUsable(now_sec);
  }

  const OverrideTrackingIdentity *controllerTrackingManeuverIdentity() const {
    return overrideTrackingIdentityForProof(
        current_override_tracking_identity_,
        previous_override_tracking_identity_,
        controller_tracking_plan_generation_, override_generation_,
        attempt_active_ ? current_attempt_id_ : 0U, last_override_target_id_,
        last_override_pass_type_);
  }

  bool purePursuitTransportTrackingUsable(double now_sec) const {
    if (!controllerTrackingStatusFresh(now_sec) ||
        !controller_tracking_pp_command_fresh_ ||
        controller_tracking_plan_generation_ == 0U ||
        override_generation_ == 0U) {
      return false;
    }
    const std::uint32_t previous_generation =
        override_generation_ <= 1U ? 16777215U : override_generation_ - 1U;
    const bool generation_continuous =
        controller_tracking_plan_generation_ == override_generation_ ||
        controller_tracking_plan_generation_ == previous_generation;
    if (!generation_continuous) {
      return false;
    }
    // muxのplan_generation_mismatchは、final
    // sourceがPPでcommand自体はfreshだが、
    // plannerの次generationが先着したtransport差を表す。この理由に限り高々
    // 1 generationを継続証跡として扱う。STOP/watchdog/command-stale等の理由は
    // ここへ入らず、従来どおり即座に不使用となる。
    return controller_tracking_usable_ ||
           (controller_tracking_plan_generation_ == previous_generation &&
            controller_tracking_reason_ == "plan_generation_mismatch");
  }

  bool purePursuitTrackingContinuityUsable(double now_sec) const {
    if (!purePursuitTransportTrackingUsable(now_sec)) {
      return false;
    }
    const std::uint32_t previous_generation =
        override_generation_ <= 1U ? 16777215U : override_generation_ - 1U;
    const bool generation_continuous =
        controller_tracking_plan_generation_ == override_generation_ ||
        controller_tracking_plan_generation_ == previous_generation;
    // generation差だけでは、その世代がcurrent-d stop/FOLLOWだったのか
    // 同じtransactionのPASS/ATTACK_FOLLOWだったのか分からない。実際に
    // publishした同一attemptの型付き履歴まで一致する時だけcontinuityを返す。
    return generation_continuous &&
           controllerTrackingManeuverIdentity() != nullptr;
  }

  bool purePursuitReleaseReady(double now_sec) const {
    const bool current_generation_ready =
        controllerTrackingStatusFresh(now_sec) &&
        controller_tracking_pp_command_fresh_ &&
        controller_tracking_safety_constraint_release_ready_ &&
        controller_tracking_plan_generation_ == override_generation_;
    const bool stopped_attack_follow_transport_ready =
        attackFollowStopTransportReleaseProofReady(
            tracking_release_ack_expected_,
            controllerTrackingStatusFresh(now_sec),
            controller_tracking_pp_command_fresh_,
            controller_tracking_safety_constraint_release_ready_,
            controller_tracking_attack_follow_stop_transport_release_ready_,
            controller_tracking_plan_generation_, override_generation_);
    return trackingReleaseProofReady(
        current_generation_ready || stopped_attack_follow_transport_ready,
        tracking_release_ack_expected_, tracking_release_expected_token_,
        tracking_release_expected_generation_, override_generation_,
        last_override_latch_active_, last_override_target_id_,
        tracking_release_expected_target_id_, last_override_pass_type_,
        tracking_release_expected_pass_type_);
  }

  bool verifiedNonMpcPurePursuit(double now_sec) const {
    const bool stopped_attack_follow_transport_ready =
        attackFollowStopTransportReleaseProofReady(
            tracking_release_ack_expected_,
            controllerTrackingStatusFresh(now_sec),
            controller_tracking_pp_command_fresh_,
            controller_tracking_safety_constraint_release_ready_,
            controller_tracking_attack_follow_stop_transport_release_ready_,
            controller_tracking_plan_generation_, override_generation_);
    return (purePursuitTransportTrackingUsable(now_sec) ||
            stopped_attack_follow_transport_ready) &&
           !controller_tracking_mpc_horizon_usable_;
  }

  void updateRaceArm(bool armed) {
    if (!race_arm_required_ || armed == race_armed_) {
      return;
    }
    race_armed_ = armed;
    // arm前proposalは診断専用の別Coreで評価する。arm/disarm epoch境界では
    // 必ず捨て、safe-cycle・target latch・RECOVERY状態をlive Coreへ
    // 持ち越さない。
    prearm_proposal_core_ =
        std::make_unique<OvertakePlannerCore>(frame_, planner_config_);
    safety_constraint_release_gate_ = SafetyConstraintReleaseGate(
        safety_constraint_release_safe_cycles_, true);
    supervisor_v2_constraint_release_gate_ = SafetyConstraintReleaseGate(
        safety_constraint_release_safe_cycles_, true);
    attempt_active_ = false;
    last_mode_ = BehaviorMode::FREE_RUN;
    has_decision_log_snapshot_ = false;
    last_controller_tracking_status_sec_.reset();
    controller_tracking_command_age_sec_ =
        std::numeric_limits<double>::infinity();
    controller_tracking_mpc_horizon_usable_ = false;
    controller_tracking_pp_command_fresh_ = false;
    controller_tracking_usable_ = false;
    controller_tracking_safety_constraint_release_ready_ = false;
    controller_tracking_attack_follow_stop_transport_release_ready_ = false;
    controller_tracking_pass_probe_transport_evidence_ =
        static_cast<std::uint8_t>(StartGridProbeTransportEvidence::UNKNOWN);
    controller_tracking_plan_generation_ = 0U;
    controller_tracking_reason_.clear();
    motion_grant_issuer_instance_id_ = 0U;
    motion_grant_sequence_ = 0U;
    motion_grant_observed_ = false;
    motion_grant_reason_ = "race_epoch_reset";
    last_motion_grant_status_sec_.reset();
    current_override_tracking_identity_ = OverrideTrackingIdentity{};
    previous_override_tracking_identity_ = OverrideTrackingIdentity{};
    has_last_override_identity_ = false;
    last_override_authoritative_target_id_.clear();
    has_last_tracking_release_token_ = false;
    last_tracking_release_token_ = 0U;
    tracking_release_ack_expected_ = false;
    tracking_release_expected_token_ = 0U;
    tracking_release_expected_generation_ = 0U;
    tracking_release_expected_target_id_.clear();
    tracking_release_expected_pass_type_ = CandidateType::FASTEST;
    last_pass_probe_warmup_valid_ = false;
    last_pass_probe_warmup_authority_token_ = 0U;
    last_supervisor_v2_prefilter_constraint_reason_.clear();
    last_supervisor_v2_filtered_constraint_reason_.clear();
    last_supervisor_v2_authorization_failure_mask_ = 0U;
    last_supervisor_v2_authorization_failure_reasons_.clear();
    last_supervisor_v2_trajectory_publishable_ = false;
    last_supervisor_v2_effective_trajectory_authorized_ = false;
    last_authoritative_plan_feedback_valid_ = false;
    last_authoritative_plan_stop_requested_ = false;
    last_authoritative_plan_trajectory_authorized_ = false;
    last_authoritative_plan_target_id_.clear();
    last_authoritative_plan_pass_type_ = CandidateType::FASTEST;
    if (race_armed_) {
      // core内の多数の戦術/復帰ラッチはarm epochごと再生成し、arm前の判断を
      // 列挙resetの漏れで持ち越さない。
      core_ = std::make_unique<OvertakePlannerCore>(frame_, planner_config_);
      const auto next_epoch = aw2::nextRaceArmEpoch(race_arm_epoch_);
      if (next_epoch.has_value()) {
        race_arm_epoch_ = next_epoch.value();
        aw2_identity_exhausted_ =
            !connector_transaction_sequencer_.resetForRaceEpoch(
                race_arm_epoch_) ||
            !v2_connector_transaction_sequencer_.resetForRaceEpoch(
                race_arm_epoch_);
      } else {
        aw2_identity_exhausted_ = true;
        RCLCPP_ERROR(
            get_logger(),
            "AW2 race arm epoch exhausted; shadow identity remains schema 0");
      }
      aw2_candidate_binding_tracker_.reset();
      aw2_v2_candidate_binding_tracker_.reset();
      aw2_delivery_record_tracker_.reset();
      aw2_v2_delivery_record_tracker_.reset();
      RCLCPP_INFO(get_logger(), "overtake race armed (epoch=%llu)",
                  static_cast<unsigned long long>(race_arm_epoch_));
    } else {
      RCLCPP_WARN(get_logger(), "overtake race disarmed; publishing stop");
    }
  }

  // 入力: V2X車両位置配列。
  // 出力: なし。車両IDごとの最新位置と推定速度をsamples_へ保存する。
  // 処理概要:
  // 位置差分からvx/vyを推定し、ジャンプが大きい時は速度を0として外れ値を抑える。
  void updateOpponents(const v2x_msgs::msg::V2XVehiclePositionArray &msg) {
    // 各車両の最新位置を保持し、ジャンプが小さい時だけ速度推定を更新する。
    for (const auto &vehicle : msg.vehicles) {
      const double stamp_sec = stampToSec(vehicle.header.stamp);
      const double x = vehicle.position.x;
      const double y = vehicle.position.y;
      if (vehicle.vehicle_id.empty() || !std::isfinite(stamp_sec) ||
          !std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      auto &sample = samples_[vehicle.vehicle_id];
      if (sample.has_prev && stamp_sec <= sample.stamp_sec + 1.0e-9) {
        // duplicate/out-of-order観測で最新cacheと速度を巻き戻さない。heartbeatは
        // 配列受信として下で更新するが、この古いsampleをfresh target進捗や
        // PASS完了debounceへ再利用しない。
        continue;
      }
      if (sample.has_prev) {
        const double dt = stamp_sec - sample.stamp_sec;
        const double jump = std::hypot(x - sample.x, y - sample.y);
        if (dt > 0.0 && jump <= position_jump_threshold_m_) {
          // V2X位置差分から等速予測用のvx/vyを作る。
          sample.vx = (x - sample.x) / dt;
          sample.vy = (y - sample.y) / dt;
        } else {
          sample.vx = 0.0;
          sample.vy = 0.0;
        }
      }
      sample.stamp_sec = stamp_sec;
      sample.x = x;
      sample.y = y;
      sample.has_prev = true;
    }
    // 個々の車両stampだけでは「空配列」と「受信停止」を区別できないため、
    // 復帰ゲート用に配列heartbeatも保持する。
    last_v2x_snapshot_sec_ = now().seconds();
  }

  // 入力: 現在の自車状態と時刻。
  // 出力: coreへ渡すOpponentState配列。
  // 処理概要:
  // 自車ID、近すぎる点、無効な自車状態を除外し、相手車をFrenet座標つきへ変換する。
  std::vector<OpponentState> collectOpponents(const EgoState &ego,
                                              double now_sec) const {
    // 自車IDと近すぎる点を除外し、Frenet座標つきの他車リストへ変換する。
    std::vector<OpponentState> opponents;
    if (!ego.valid || frame_.empty()) {
      return opponents;
    }
    for (const auto &item : samples_) {
      const auto &id = item.first;
      const auto &sample = item.second;
      if (!sample.has_prev || id == own_vehicle_id_) {
        continue;
      }
      const double ego_dist = std::hypot(sample.x - ego.x, sample.y - ego.y);
      if (ego_dist < ignore_near_ego_m_) {
        continue;
      }
      OpponentState opp;
      opp.id = id;
      opp.stamp_sec = sample.stamp_sec;
      opp.x = sample.x;
      opp.y = sample.y;
      opp.vx = sample.vx;
      opp.vy = sample.vy;
      opp.v = std::hypot(sample.vx, sample.vy);
      opp.frenet = frame_.cartesianToFrenet(opp.x, opp.y, 0.0);
      opp.valid = inputTimestampFresh(now_sec, opp.stamp_sec, 2.0,
                                      input_future_stamp_tolerance_sec_);
      opponents.push_back(opp);
    }
    return opponents;
  }

  // 入力: 現在時刻、自車、最新MPC health。
  // 出力: 通常ライン復帰ゲートが使える入力完全性。
  // 処理概要:
  // V2Xの受信停止を「相手なし」と扱わず、既知の他車の古いstampもfail-closedにする。
  ReentryInputStatus reentryInputStatus(double now_sec, const EgoState &ego,
                                        const MpcHealthStatus &health) const {
    ReentryInputStatus status;
    const auto fresh = [this, now_sec](double stamp_sec, double timeout_sec) {
      return inputTimestampFresh(now_sec, stamp_sec, timeout_sec,
                                 input_future_stamp_tolerance_sec_);
    };
    status.ego_fresh = ego.valid && fresh(ego.stamp_sec, ego_stale_time_sec_);
    status.v2x_snapshot_fresh = last_v2x_snapshot_sec_.has_value() &&
                                fresh(last_v2x_snapshot_sec_.value(),
                                      reentry_v2x_snapshot_stale_time_sec_);
    status.all_observed_opponents_fresh = true;
    status.all_observed_opponents_included = true;
    for (const auto &item : samples_) {
      const auto &id = item.first;
      const auto &sample = item.second;
      if (id == own_vehicle_id_) {
        continue;
      }
      if (!sample.has_prev || !std::isfinite(sample.x) ||
          !std::isfinite(sample.y) ||
          !fresh(sample.stamp_sec, opponent_stale_time_sec_)) {
        status.all_observed_opponents_fresh = false;
        break;
      }
      // 通常のplanner入力では近すぎる観測をself重複対策として除外する。
      // 復帰だけは未評価の相手を安全とみなさず、gateを閉じる。
      if (std::hypot(sample.x - ego.x, sample.y - ego.y) < ignore_near_ego_m_) {
        status.all_observed_opponents_included = false;
      }
    }
    status.reference_valid = !frame_.empty();
    status.mpc_health_fresh = health.valid && fresh(now_sec - health.age_sec,
                                                    mpc_health_stale_time_sec_);
    status.mpc_hard_failure =
        status.mpc_health_fresh &&
        health.infeasible_count >= mpc_health_infeasible_count_threshold_;
    status.mpc_latency_warning =
        status.mpc_health_fresh && !status.mpc_hard_failure &&
        std::isfinite(health.solve_time_ms) &&
        health.solve_time_ms >= mpc_health_solve_time_warn_ms_;
    status.mpc_healthy = status.mpc_health_fresh && !status.mpc_hard_failure &&
                         !status.mpc_latency_warning;
    status.pure_pursuit_primary_and_fresh = purePursuitPrimaryAndFresh(now_sec);
    status.controller_tracking_status_received =
        last_controller_tracking_status_sec_.has_value();
    status.controller_tracking_plan_generation =
        controller_tracking_plan_generation_;
    status.controller_tracking_expected_generation = override_generation_;
    status.controller_tracking_status_reason =
        controllerTrackingStatusReason(now_sec);
    status.controller_tracking_mpc_horizon_usable =
        controller_tracking_mpc_horizon_usable_;
    status.controller_tracking_continuity_usable =
        purePursuitTrackingContinuityUsable(now_sec);
    constexpr auto kMaxProbeTransportEvidence = static_cast<std::uint8_t>(
        StartGridProbeTransportEvidence::REGRESSION_OR_GAP);
    status.start_grid_pass_probe_transport_evidence =
        controllerTrackingStatusFresh(now_sec) &&
                controller_tracking_pass_probe_transport_evidence_ <=
                    kMaxProbeTransportEvidence
            ? static_cast<StartGridProbeTransportEvidence>(
                  controller_tracking_pass_probe_transport_evidence_)
            : StartGridProbeTransportEvidence::STALE;
    status.pass_probe_exact_current_usable =
        controllerTrackingStatusFresh(now_sec) &&
        controller_tracking_pass_probe_exact_current_usable_;
    status.pass_probe_lateral_stop_authority_token =
        status.pass_probe_exact_current_usable
            ? controller_tracking_pass_probe_lateral_stop_authority_token_
            : 0U;
    status.pass_probe_exact_sample_stamp_sec =
        status.pass_probe_exact_current_usable &&
                last_controller_tracking_status_sec_.has_value()
            ? last_controller_tracking_status_sec_.value()
            : std::numeric_limits<double>::quiet_NaN();
    status.pass_probe_exact_plan_generation =
        status.pass_probe_exact_current_usable
            ? controller_tracking_plan_generation_
            : 0U;
    status.pure_pursuit_attack_follow_transport_usable =
        purePursuitAttackFollowTransportUsable(now_sec);
    status.pure_pursuit_tracking_continuity_usable =
        purePursuitTrackingContinuityUsable(now_sec);
    if (status.pure_pursuit_tracking_continuity_usable) {
      const auto *identity = controllerTrackingManeuverIdentity();
      if (identity != nullptr) {
        status.pure_pursuit_tracking_target_id = identity->target_id;
        status.pure_pursuit_tracking_pass_type = identity->pass_type;
      }
    }
    status.pure_pursuit_release_ready = purePursuitReleaseReady(now_sec);
    status.verified_non_mpc_pure_pursuit = verifiedNonMpcPurePursuit(now_sec);
    status.previous_authoritative_plan_feedback_valid =
        last_authoritative_plan_feedback_valid_;
    status.previous_authoritative_plan_stop_requested =
        last_authoritative_plan_stop_requested_;
    status.previous_authoritative_plan_trajectory_authorized =
        last_authoritative_plan_trajectory_authorized_;
    status.previous_authoritative_plan_target_id =
        last_authoritative_plan_target_id_;
    status.previous_authoritative_plan_pass_type =
        last_authoritative_plan_pass_type_;
    status.mpc_health_sample_sequence = health.sample_sequence;
    return status;
  }

  PlannerPublicationState plannerPublicationState() const {
    PlannerPublicationState state;
    state.last_override_wire_payload = last_override_wire_payload_;
    state.override_generation = override_generation_;
    state.has_last_override_identity = has_last_override_identity_;
    state.last_override_latch_active = last_override_latch_active_;
    state.last_override_target_id = last_override_target_id_;
    state.last_override_authoritative_target_id =
        last_override_authoritative_target_id_;
    state.last_override_pass_type = last_override_pass_type_;
    state.current_override_tracking_identity =
        current_override_tracking_identity_;
    state.previous_override_tracking_identity =
        previous_override_tracking_identity_;
    state.has_last_tracking_release_token = has_last_tracking_release_token_;
    state.last_tracking_release_token = last_tracking_release_token_;
    state.tracking_release_ack_expected = tracking_release_ack_expected_;
    state.tracking_release_expected_token = tracking_release_expected_token_;
    state.tracking_release_expected_generation =
        tracking_release_expected_generation_;
    state.tracking_release_expected_target_id =
        tracking_release_expected_target_id_;
    state.tracking_release_expected_pass_type =
        tracking_release_expected_pass_type_;
    state.last_safety_constraint_command = last_safety_constraint_command_;
    state.safety_constraint_generation = safety_constraint_generation_;
    state.last_safety_constraint_plan_generation =
        last_safety_constraint_plan_generation_;
    return state;
  }

  void applyPlannerPublicationState(const PlannerPublicationState &state) {
    last_override_wire_payload_ = state.last_override_wire_payload;
    override_generation_ = state.override_generation;
    has_last_override_identity_ = state.has_last_override_identity;
    last_override_latch_active_ = state.last_override_latch_active;
    last_override_target_id_ = state.last_override_target_id;
    last_override_authoritative_target_id_ =
        state.last_override_authoritative_target_id;
    last_override_pass_type_ = state.last_override_pass_type;
    current_override_tracking_identity_ =
        state.current_override_tracking_identity;
    previous_override_tracking_identity_ =
        state.previous_override_tracking_identity;
    has_last_tracking_release_token_ = state.has_last_tracking_release_token;
    last_tracking_release_token_ = state.last_tracking_release_token;
    tracking_release_ack_expected_ = state.tracking_release_ack_expected;
    tracking_release_expected_token_ = state.tracking_release_expected_token;
    tracking_release_expected_generation_ =
        state.tracking_release_expected_generation;
    tracking_release_expected_target_id_ =
        state.tracking_release_expected_target_id;
    tracking_release_expected_pass_type_ =
        state.tracking_release_expected_pass_type;
    last_safety_constraint_command_ = state.last_safety_constraint_command;
    safety_constraint_generation_ = state.safety_constraint_generation;
    last_safety_constraint_plan_generation_ =
        state.last_safety_constraint_plan_generation;
  }

  // 入力: coreが返したPlannerOutput。
  // 出力: /overtake/reference_override へFloat32MultiArrayをpublishする。
  // 処理概要: 安全な横列がある時は従来v1、横列なしの速度guard時はv2へ詰める。
  void publishOverride(const PlannerOutput &output, std::uint64_t attempt_id) {
    std_msgs::msg::Float32MultiArray msg;
    const auto publication = makeReferenceOverridePublication(
        plannerPublicationState(), output, attempt_id);
    applyPlannerPublicationState(publication.next_state);
    msg.data = publication.wire_payload.data;
    aw2::Float32SourceWire aw2_source_wire;
    aw2_source_wire.data_offset = msg.layout.data_offset;
    aw2_source_wire.values = msg.data;
    aw2_source_wire.dimensions.reserve(msg.layout.dim.size());
    for (const auto &dimension : msg.layout.dim) {
      aw2_source_wire.dimensions.push_back(
          {dimension.label, dimension.size, dimension.stride});
    }
    last_aw2_source_wire_ = aw2::canonicalizeFloat32SourceWire(aw2_source_wire);
    override_pub_->publish(msg);
  }

  // 入力: 最終PlannerOutput、自車状態、入力鮮度。
  // 出力: muxがsource選択後に強制する型付きSafetyConstraint。
  // 処理概要: legacy overrideと同じplan generationへ紐付け、低速/停止の解除を
  // 別generationの明示releaseとしてpublishする。
  multi_purpose_mpc_ros_msgs::msg::SafetyConstraint
  publishSafetyConstraint(const SafetyConstraintCommand &command,
                          const rclcpp::Time &contract_stamp) {
    builtin_interfaces::msg::Time contract_stamp_msg = contract_stamp;
    const auto publication = makeSafetyConstraintPublication(
        plannerPublicationState(), command, contract_stamp_msg);
    applyPlannerPublicationState(publication.next_state);
    safety_constraint_pub_->publish(publication.message);
    return publication.message;
  }

  void publishCandidateExecutionRequest(
      const multi_purpose_mpc_ros_msgs::msg::OvertakePlan &plan_msg,
      const multi_purpose_mpc_ros_msgs::msg::SafetyConstraint &constraint_msg,
      std::int8_t transaction_pass_direction, std::uint8_t candidate_type,
      aw2::CandidateSourceKind source_kind,
      const std::optional<std::vector<std::uint8_t>> &canonical_source_wire,
      double required_controller_spatial_horizon_m, double planned_target_d_m,
      double committed_target_d_m,
      aw2::SafetyEvaluationResult safety_evaluation_result,
      const std::string &safety_evaluation_reason,
      std::uint64_t safety_snapshot_id,
      aw2::DeliveryRecordTracker &delivery_tracker) {
    using Request = multi_purpose_mpc_ros_msgs::msg::CandidateExecutionRequest;
    Request msg;
    msg.schema_version = Request::SCHEMA_INVALID;
    msg.authority_eligible = false;
    msg.plan_stamp = plan_msg.header.stamp;
    if (plan_msg.header.frame_id.size() <= aw2::kMaxFrameIdBytes) {
      msg.plan_frame_id = plan_msg.header.frame_id;
    }
    msg.planner_instance_id = plan_msg.planner_instance_id;
    msg.race_arm_epoch = plan_msg.race_arm_epoch;
    msg.attempt_id = plan_msg.attempt_id;
    if (plan_msg.target_vehicle_id.size() <= aw2::kMaxTargetIdBytes) {
      msg.target_vehicle_id = plan_msg.target_vehicle_id;
    }
    msg.transaction_pass_direction = transaction_pass_direction;
    msg.published_pass_direction = plan_msg.pass_direction;
    msg.connector_transaction_id = plan_msg.connector_transaction_id;
    msg.plan_generation = plan_msg.plan_generation;
    msg.candidate_revision = plan_msg.candidate_revision;
    msg.candidate_content_sha256 = plan_msg.candidate_content_sha256;
    msg.phase = plan_msg.phase;
    msg.authorization_state = plan_msg.trajectory_authorized
                                  ? Request::AUTHORIZATION_AUTHORIZED
                                  : Request::AUTHORIZATION_NOT_AUTHORIZED;
    msg.geometry_requirement = !plan_msg.lateral_maneuver_required
                                   ? Request::GEOMETRY_LONGITUDINAL_ONLY
                                   : (plan_msg.trajectory.points.empty()
                                          ? Request::GEOMETRY_UNKNOWN
                                          : Request::GEOMETRY_LATERAL_REQUIRED);
    msg.candidate_type = candidate_type;
    msg.trajectory_authorized_legacy = plan_msg.trajectory_authorized;
    msg.lateral_maneuver_required_legacy = plan_msg.lateral_maneuver_required;
    msg.typed_trajectory_present = !plan_msg.trajectory.points.empty();
    msg.source_kind = static_cast<std::uint8_t>(source_kind);
    msg.source_generation = plan_msg.plan_generation;
    if (canonical_source_wire.has_value() &&
        canonical_source_wire->size() <= aw2::kMaxSourceWireBytes) {
      msg.source_original_size_bytes =
          static_cast<std::uint32_t>(canonical_source_wire->size());
      msg.canonical_source_wire.assign(canonical_source_wire->begin(),
                                       canonical_source_wire->end());
    }
    msg.constraint_stamp = constraint_msg.header.stamp;
    if (constraint_msg.header.frame_id.size() <= aw2::kMaxFrameIdBytes) {
      msg.constraint_frame_id = constraint_msg.header.frame_id;
    }
    msg.constraint_generation = constraint_msg.constraint_generation;
    msg.constraint_plan_generation = constraint_msg.plan_generation;
    msg.constraint_valid = constraint_msg.valid;
    msg.constraint_stop_requested = constraint_msg.stop_requested;
    msg.constraint_release_authorized = constraint_msg.release_authorized;
    msg.constraint_speed_limit_mps = constraint_msg.speed_limit_mps;
    msg.constraint_required_brake_decel_mps2 =
        constraint_msg.required_brake_decel_mps2;
    if (constraint_msg.reason.size() <= aw2::kMaxReasonBytes) {
      msg.constraint_reason = constraint_msg.reason;
    }
    msg.required_controller_spatial_horizon_m =
        required_controller_spatial_horizon_m;
    msg.planned_target_d_m = planned_target_d_m;
    msg.committed_target_d_m = committed_target_d_m;
    msg.safety_snapshot_id = safety_snapshot_id;
    msg.safety_evaluation_result =
        static_cast<std::uint8_t>(safety_evaluation_result);
    if (safety_evaluation_reason.size() <= aw2::kMaxReasonBytes) {
      msg.safety_evaluation_reason = safety_evaluation_reason;
    }

    aw2::CandidateExecutionRecord record;
    record.key.transaction.race_arm_epoch = plan_msg.race_arm_epoch;
    record.key.transaction.planner_instance_id = plan_msg.planner_instance_id;
    record.key.transaction.attempt_id = plan_msg.attempt_id;
    record.key.transaction.target_vehicle_id = plan_msg.target_vehicle_id;
    record.key.transaction.pass_direction = transaction_pass_direction;
    record.key.transaction.connector_transaction_id =
        plan_msg.connector_transaction_id;
    record.key.plan_stamp_sec = plan_msg.header.stamp.sec;
    record.key.plan_stamp_nanosec = plan_msg.header.stamp.nanosec;
    record.key.plan_generation = plan_msg.plan_generation;
    record.candidate_revision = plan_msg.candidate_revision;
    record.candidate_content_sha256 = plan_msg.candidate_content_sha256;
    record.plan_frame_id = plan_msg.header.frame_id;
    record.phase = plan_msg.phase;
    record.authorization_state = plan_msg.trajectory_authorized
                                     ? aw2::AuthorizationState::AUTHORIZED
                                     : aw2::AuthorizationState::NOT_AUTHORIZED;
    record.geometry_requirement =
        !plan_msg.lateral_maneuver_required
            ? aw2::GeometryRequirement::NOT_REQUIRED
            : (plan_msg.trajectory.points.empty()
                   ? aw2::GeometryRequirement::UNKNOWN
                   : aw2::GeometryRequirement::REQUIRED);
    record.candidate_type = candidate_type;
    record.trajectory_authorized_legacy = plan_msg.trajectory_authorized;
    record.lateral_maneuver_required_legacy =
        plan_msg.lateral_maneuver_required;
    record.published_pass_direction = plan_msg.pass_direction;
    record.typed_trajectory_present = !plan_msg.trajectory.points.empty();
    if (plan_msg.trajectory.points.size() <= aw2::kMaxGeometryPoints) {
      record.geometry_points.reserve(plan_msg.trajectory.points.size());
      msg.geometry_points.reserve(plan_msg.trajectory.points.size());
      for (const auto &point : plan_msg.trajectory.points) {
        aw2::CandidateExecutionPoint record_point;
        record_point.time_sec = point.time_from_start.sec;
        record_point.time_nanosec = point.time_from_start.nanosec;
        record_point.position_x_m = point.pose.position.x;
        record_point.position_y_m = point.pose.position.y;
        record_point.position_z_m = point.pose.position.z;
        record_point.orientation_x = point.pose.orientation.x;
        record_point.orientation_y = point.pose.orientation.y;
        record_point.orientation_z = point.pose.orientation.z;
        record_point.orientation_w = point.pose.orientation.w;
        record_point.longitudinal_velocity_mps =
            point.longitudinal_velocity_mps;
        record_point.lateral_velocity_mps = point.lateral_velocity_mps;
        record_point.acceleration_mps2 = point.acceleration_mps2;
        record_point.heading_rate_rps = point.heading_rate_rps;
        record_point.front_wheel_angle_rad = point.front_wheel_angle_rad;
        record_point.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
        record.geometry_points.push_back(record_point);

        multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint msg_point;
        msg_point.time_from_start = point.time_from_start;
        msg_point.position_x_m = record_point.position_x_m;
        msg_point.position_y_m = record_point.position_y_m;
        msg_point.position_z_m = record_point.position_z_m;
        msg_point.orientation_x = record_point.orientation_x;
        msg_point.orientation_y = record_point.orientation_y;
        msg_point.orientation_z = record_point.orientation_z;
        msg_point.orientation_w = record_point.orientation_w;
        msg_point.longitudinal_velocity_mps =
            record_point.longitudinal_velocity_mps;
        msg_point.lateral_velocity_mps = record_point.lateral_velocity_mps;
        msg_point.acceleration_mps2 = record_point.acceleration_mps2;
        msg_point.heading_rate_rps = record_point.heading_rate_rps;
        msg_point.front_wheel_angle_rad = record_point.front_wheel_angle_rad;
        msg_point.rear_wheel_angle_rad = record_point.rear_wheel_angle_rad;
        msg.geometry_points.push_back(msg_point);
      }
    }
    record.source_kind = source_kind;
    record.source_generation = plan_msg.plan_generation;
    if (canonical_source_wire.has_value()) {
      record.source_original_size_bytes =
          static_cast<std::uint32_t>(canonical_source_wire->size());
      record.canonical_source_wire = canonical_source_wire.value();
    }
    record.constraint_stamp_sec = constraint_msg.header.stamp.sec;
    record.constraint_stamp_nanosec = constraint_msg.header.stamp.nanosec;
    record.constraint_frame_id = constraint_msg.header.frame_id;
    record.constraint_generation = constraint_msg.constraint_generation;
    record.constraint_plan_generation = constraint_msg.plan_generation;
    record.constraint_valid = constraint_msg.valid;
    record.constraint_stop_requested = constraint_msg.stop_requested;
    record.constraint_release_authorized = constraint_msg.release_authorized;
    record.constraint_speed_limit_mps = constraint_msg.speed_limit_mps;
    record.constraint_required_brake_decel_mps2 =
        constraint_msg.required_brake_decel_mps2;
    record.constraint_reason = constraint_msg.reason;
    record.required_controller_spatial_horizon_m =
        required_controller_spatial_horizon_m;
    record.planned_target_d_m = planned_target_d_m;
    record.committed_target_d_m = committed_target_d_m;
    record.safety_snapshot_id = safety_snapshot_id;
    record.safety_evaluation_result = safety_evaluation_result;
    record.safety_evaluation_reason = safety_evaluation_reason;

    const auto canonical = aw2::canonicalizeCandidateExecutionRecordV1(record);
    if (canonical.valid()) {
      msg.delivered_geometry_sha256 = canonical.delivered_geometry_sha256;
      msg.canonical_source_sha256 = canonical.canonical_source_sha256;
      msg.plan_sample_record_sha256 = canonical.plan_sample_record_sha256;
      const auto observation = delivery_tracker.observe(
          record.key, canonical.plan_sample_record_sha256);
      msg.delivery_observation = static_cast<std::uint8_t>(observation);
      if (plan_msg.aw2_identity_schema_version == 1U &&
          (observation == aw2::DeliveryObservation::ACCEPTED ||
           observation == aw2::DeliveryObservation::CONSISTENT_DUPLICATE)) {
        msg.schema_version = Request::SCHEMA_V1;
      }
    } else {
      msg.delivery_observation = Request::DELIVERY_INVALID;
    }
    candidate_execution_request_pub_->publish(msg);
    if (c002ay1_active_observation_ != nullptr) {
      const bool legacy =
          source_kind == aw2::CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE;
      const bool v2 = source_kind == aw2::CandidateSourceKind::V2_TRAJECTORY;
      if (legacy && c002ay1_active_observation_->legacy_publish_count !=
                        std::numeric_limits<std::uint8_t>::max()) {
        ++c002ay1_active_observation_->legacy_publish_count;
      }
      if (v2 && c002ay1_active_observation_->v2_publish_count !=
                    std::numeric_limits<std::uint8_t>::max()) {
        ++c002ay1_active_observation_->v2_publish_count;
      }
      const bool selected =
          (legacy && c002ay1_configured_stream_ ==
                         overtake_transport_contract::c002ay1::
                             ConfiguredStreamKind::kLegacyReferenceOverride) ||
          (v2 && c002ay1_configured_stream_ ==
                     overtake_transport_contract::c002ay1::
                         ConfiguredStreamKind::kV2Trajectory);
      if (selected) {
        if (c002ay1_active_observation_->selected_proposal_count !=
            std::numeric_limits<std::uint8_t>::max()) {
          ++c002ay1_active_observation_->selected_proposal_count;
        }
        if (c002ay1_selected_proposal_sequence_ !=
            std::numeric_limits<std::uint64_t>::max()) {
          ++c002ay1_selected_proposal_sequence_;
        }
        c002ay1_active_observation_->selected_proposal_sequence =
            c002ay1_selected_proposal_sequence_;
        c002ay1_active_observation_->flags |=
            overtake_transport_contract::c002ay1::kFlagEmitted;
      }
    }
  }

  void publishAuthoritativePlan(
      const PlannerOutput &output, const EgoState &ego,
      std::uint64_t attempt_id, const SafetyConstraintCommand &constraint,
      const multi_purpose_mpc_ros_msgs::msg::SafetyConstraint &constraint_msg,
      const rclcpp::Time &contract_stamp) {
    multi_purpose_mpc_ros_msgs::msg::OvertakePlan msg;
    msg.header.stamp = contract_stamp;
    msg.header.frame_id = "map";
    msg.plan_generation = override_generation_;
    msg.attempt_id = static_cast<std::uint32_t>(std::min<std::uint64_t>(
        attempt_id, std::numeric_limits<std::uint32_t>::max()));

    switch (output.mode) {
    case BehaviorMode::FREE_RUN:
      msg.phase = multi_purpose_mpc_ros_msgs::msg::OvertakePlan::FREE_RUN;
      break;
    case BehaviorMode::FOLLOW_BLOCKED:
      msg.phase = multi_purpose_mpc_ros_msgs::msg::OvertakePlan::ATTACK_FOLLOW;
      break;
    case BehaviorMode::PREPARE_OVERTAKE_LEFT:
    case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
    case BehaviorMode::OVERTAKE_LEFT:
    case BehaviorMode::OVERTAKE_RIGHT:
    case BehaviorMode::MERGE_BACK:
      msg.phase = multi_purpose_mpc_ros_msgs::msg::OvertakePlan::PASSING;
      break;
    default:
      msg.phase = multi_purpose_mpc_ros_msgs::msg::OvertakePlan::ABORT_HOLD;
      break;
    }

    msg.target_vehicle_id = authoritativeTargetVehicleId(output.blocked_info);
    const bool current_d_tracking_stop = isSafetyEvaluatedCurrentDTrackingStop(
        output, ego, constraint.valid, constraint.stop_requested,
        constraint.release_authorized, constraint.speed_limit_mps,
        constraint.reason);
    const bool current_d_tracking_stop_intent =
        constraint.stop_requested && isTrackingStopStructuralIntent(output);
    const bool current_d_tracking_stop_wire_ready =
        !current_d_tracking_stop_intent ||
        trackingStopLateralWireMatchesAuthorization(
            output, last_override_wire_payload_, override_generation_);
    msg.pass_direction =
        authoritativePlanPassDirection(output, current_d_tracking_stop);
    const bool left = msg.pass_direction > 0;
    const bool right = msg.pass_direction < 0;
    const bool legacy_lateral_maneuver_required =
        output.active_override &&
        (output.solver_horizon_intent !=
             PlannerOutput::SolverHorizonIntent::NONE ||
         output.lateral_tracking_authorized_during_stop) &&
        current_d_tracking_stop_wire_ready;
    msg.trajectory.header = msg.header;

    const std::size_t point_count =
        std::min(output.lateral_offsets.size(), output.speed_caps.size());
    const bool spatial_axis_complete =
        output.longitudinal_offsets_m.size() == point_count;
    const bool spatial_axis_valid =
        spatial_axis_complete && point_count > 1U &&
        std::isfinite(output.longitudinal_offsets_m.front()) &&
        std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-6 &&
        std::isfinite(output.longitudinal_offsets_m.back()) &&
        output.longitudinal_offsets_m.back() > 1.0e-6 &&
        std::adjacent_find(output.longitudinal_offsets_m.begin(),
                           output.longitudinal_offsets_m.end(),
                           [](double previous, double next) {
                             return !std::isfinite(previous) ||
                                    !std::isfinite(next) ||
                                    next + 1.0e-9 < previous;
                           }) == output.longitudinal_offsets_m.end();
    const bool complete_trajectory =
        ego.valid && !frame_.empty() && point_count > 0U &&
        point_count == output.lateral_offsets.size() &&
        point_count == output.speed_caps.size() && spatial_axis_valid;
    const bool committed_release_pending_pass_lateral_stop =
        isReleasePendingPassLateralTrackingAuthorized(
            output, constraint.valid, constraint.stop_requested,
            constraint.release_authorized, constraint.reason);
    const bool initial_prepared_pass_lateral_stop =
        isInitialPreparedPassWarmupWireAuthorized(
            output, constraint.valid, constraint.stop_requested,
            constraint.release_authorized, constraint.reason,
            last_override_wire_payload_, override_generation_, attempt_id);
    const bool release_pending_pass_lateral_stop =
        committed_release_pending_pass_lateral_stop ||
        initial_prepared_pass_lateral_stop;
    const bool lateral_tracking_authorized_during_stop =
        constraint.stop_requested && output.lateral_stop_inputs_complete &&
        (output.lateral_tracking_authorized_during_stop ||
         release_pending_pass_lateral_stop);
    msg.trajectory_authorized = output.active_override && complete_trajectory &&
                                constraint.valid &&
                                (!constraint.stop_requested ||
                                 lateral_tracking_authorized_during_stop) &&
                                current_d_tracking_stop_wire_ready;
    if (complete_trajectory) {
      msg.trajectory.points.reserve(point_count);
      double s_m = ego.frenet.s;
      for (std::size_t i = 0; i < point_count; ++i) {
        const double d_m = output.lateral_offsets[i];
        const double speed_mps = output.speed_caps[i];
        if (!std::isfinite(d_m) || !std::isfinite(speed_mps) ||
            speed_mps < 0.0 ||
            !std::isfinite(output.longitudinal_offsets_m[i]) ||
            output.longitudinal_offsets_m[i] < 0.0) {
          msg.trajectory.points.clear();
          msg.trajectory_authorized = false;
          break;
        }
        s_m = frame_.wrapS(ego.frenet.s + output.longitudinal_offsets_m[i]);
        const auto point_on_track = frame_.frenetToCartesian(s_m, d_m);
        autoware_auto_planning_msgs::msg::TrajectoryPoint point;
        point.pose.position.x = point_on_track.x;
        point.pose.position.y = point_on_track.y;
        point.pose.orientation = quaternionFromYaw(point_on_track.yaw);
        point.longitudinal_velocity_mps = static_cast<float>(speed_mps);
        point.acceleration_mps2 = 0.0F;
        msg.trajectory.points.push_back(point);
      }
    }
    const bool inward_connector_category =
        isAttackFollowInwardConnectorOutput(output);
    const bool inward_connector_lateral_authorized =
        isPublishedAttackFollowInwardConnectorLateralRequired(
            output, ego, current_override_tracking_identity_,
            last_override_wire_payload_, override_generation_, attempt_id,
            msg.target_vehicle_id, msg.pass_direction,
            msg.trajectory_authorized);
    msg.lateral_maneuver_required = composeLateralManeuverRequired(
        legacy_lateral_maneuver_required, inward_connector_category,
        inward_connector_lateral_authorized);
    if (!authoritativePlanTrajectoryPayloadRequired(
            msg.trajectory_authorized, msg.lateral_maneuver_required)) {
      // Motion authorityを持たないego起点geometryを同一generationで
      // 再構築すると、Muxのimmutable plan fingerprintを汚染する。
      // 非認可geometryはplan payloadへ載せず、戦略状態と横trajectory要求は
      // typed fieldsで伝える。bootstrap HOLDはMux契約どおりempty
      // trajectoryにする。
      msg.trajectory.points.clear();
    }
    populateAw2AuthoritativePlanIdentity(msg, output, point_count);
    const std::int8_t latched_transaction_pass_direction =
        output.blocked_info.maneuver_transaction_pass_type ==
                CandidateType::PASS_LEFT
            ? 1
            : (output.blocked_info.maneuver_transaction_pass_type ==
                       CandidateType::PASS_RIGHT
                   ? -1
                   : 0);
    const std::int8_t transaction_pass_direction =
        aw2::transactionPassDirection(
            msg.pass_direction,
            output.blocked_info.maneuver_transaction_incomplete,
            latched_transaction_pass_direction);
    msg.lateral_stop_authority_kind =
        multi_purpose_mpc_ros_msgs::msg::OvertakePlan::LATERAL_STOP_NONE;
    msg.lateral_stop_transaction_pass_direction = 0;
    msg.lateral_stop_authority_token = 0U;
    if (msg.trajectory_authorized && msg.lateral_maneuver_required &&
        current_d_tracking_stop && latched_transaction_pass_direction != 0 &&
        msg.attempt_id != 0U) {
      msg.lateral_stop_authority_kind = multi_purpose_mpc_ros_msgs::msg::
          OvertakePlan::LATERAL_STOP_CURRENT_D_HOLD;
      msg.lateral_stop_transaction_pass_direction =
          latched_transaction_pass_direction;
      msg.lateral_stop_authority_token =
          (static_cast<std::uint64_t>(msg.attempt_id) << 32U) |
          static_cast<std::uint64_t>(msg.plan_generation);
    } else if (msg.trajectory_authorized && msg.lateral_maneuver_required &&
               release_pending_pass_lateral_stop &&
               transaction_pass_direction != 0 &&
               output.tracking_release_token != 0U && msg.attempt_id != 0U) {
      msg.lateral_stop_authority_kind = multi_purpose_mpc_ros_msgs::msg::
          OvertakePlan::LATERAL_STOP_PASS_WARMUP;
      msg.lateral_stop_transaction_pass_direction = transaction_pass_direction;
      msg.lateral_stop_authority_token = output.tracking_release_token;
    }
    last_authoritative_plan_generation_ = msg.plan_generation;
    last_authoritative_plan_attempt_id_ = msg.attempt_id;
    last_authoritative_plan_target_id_ = msg.target_vehicle_id;
    last_authoritative_plan_pass_direction_ = msg.pass_direction;
    last_authoritative_plan_connector_transaction_id_ =
        msg.connector_transaction_id;
    last_authoritative_plan_candidate_revision_ = msg.candidate_revision;
    last_authoritative_plan_candidate_content_sha256_ =
        msg.candidate_content_sha256;
    last_authoritative_plan_stamp_sec_ = stampToSec(msg.header.stamp);
    if (msg.lateral_stop_authority_kind ==
            multi_purpose_mpc_ros_msgs::msg::OvertakePlan::
                LATERAL_STOP_PASS_WARMUP &&
        msg.lateral_stop_authority_token != 0U) {
      last_pass_probe_warmup_valid_ = true;
      last_pass_probe_warmup_race_arm_epoch_ = msg.race_arm_epoch;
      last_pass_probe_warmup_planner_instance_id_ = msg.planner_instance_id;
      last_pass_probe_warmup_plan_generation_ = msg.plan_generation;
      last_pass_probe_warmup_attempt_id_ = msg.attempt_id;
      last_pass_probe_warmup_target_vehicle_id_ = msg.target_vehicle_id;
      last_pass_probe_warmup_pass_direction_ = msg.pass_direction;
      last_pass_probe_warmup_connector_transaction_id_ =
          msg.connector_transaction_id;
      last_pass_probe_warmup_stamp_sec_ = stampToSec(msg.header.stamp);
      last_pass_probe_warmup_candidate_revision_ = msg.candidate_revision;
      last_pass_probe_warmup_candidate_content_sha256_ =
          msg.candidate_content_sha256;
      last_pass_probe_warmup_authority_token_ =
          msg.lateral_stop_authority_token;
    } else if (!output.blocked_info
                    .maneuver_transaction_tracking_release_pending) {
      last_pass_probe_warmup_valid_ = false;
      last_pass_probe_warmup_authority_token_ = 0U;
    }
    (void)overtake_transport_contract::c002ay0::populateFreeRunPlanIdentityV1(
        msg);
    authoritative_plan_pub_->publish(msg);
    const double committed_target_d_m =
        std::isfinite(output.maneuver_latch_last_waypoint_d_m)
            ? output.maneuver_latch_last_waypoint_d_m
            : output.target_lateral_offset_m;
    const auto safety_evaluation_result =
        output.selected_lateral_profile_safety_verified
            ? aw2::SafetyEvaluationResult::PASSED
            : ((!output.raw_selected_reject_reason.empty() ||
                output.published_lateral_safety_rejected)
                   ? aw2::SafetyEvaluationResult::REJECTED
                   : aw2::SafetyEvaluationResult::NOT_EVALUATED);
    const std::string safety_evaluation_reason =
        !output.raw_selected_reject_reason.empty()
            ? output.raw_selected_reject_reason
            : (!output.safe_stop_reject_reason.empty()
                   ? output.safe_stop_reject_reason
                   : output.reason);
    publishCandidateExecutionRequest(
        msg, constraint_msg, transaction_pass_direction,
        static_cast<std::uint8_t>(output.selected),
        aw2::CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE,
        last_aw2_source_wire_, output.required_controller_spatial_horizon_m,
        output.target_lateral_offset_m, committed_target_d_m,
        safety_evaluation_result, safety_evaluation_reason,
        current_safety_snapshot_id_, aw2_delivery_record_tracker_);
    if (current_d_tracking_stop) {
      // 旧PASS payloadは既にreference overrideから消え、現在generationには
      // SafetyEvaluator済みcurrent-d STOPだけが載っている。旧token/genのACKを
      // 待ち続けるとSTOP proofを一段目へ使えず自己デッドロックするため、ここで
      // 期待identityだけを破棄する。縦STOPは維持され、次のPASS再生成時には
      // publishOverride()が必ず新token/generationを設定してexact
      // ACKを要求する。
      tracking_release_ack_expected_ = false;
      tracking_release_expected_token_ = 0U;
      tracking_release_expected_generation_ = 0U;
      tracking_release_expected_target_id_.clear();
      tracking_release_expected_pass_type_ = CandidateType::FASTEST;
    }
    last_authoritative_plan_feedback_valid_ = constraint.valid;
    last_authoritative_plan_stop_requested_ = constraint.stop_requested;
    last_authoritative_plan_trajectory_authorized_ = msg.trajectory_authorized;
    last_authoritative_plan_target_id_ = msg.target_vehicle_id;
    last_authoritative_plan_pass_type_ =
        left ? CandidateType::PASS_LEFT
             : (right ? CandidateType::PASS_RIGHT : CandidateType::FASTEST);
  }

  void publishSupervisorV2Shadow(const PlannerOutput &output,
                                 const EgoState &ego,
                                 const ReentryInputStatus &inputs) {
    if (!supervisor_v2_shadow_enabled_) {
      return;
    }
    const auto &decision = output.supervisor_v2;
    const bool trajectory_publishable =
        supervisorV2TrajectoryPublishable(decision);
    const bool candidate_trajectory_authorized =
        supervisorV2EffectivelyAuthorized(decision);
    std::uint32_t authorization_failure_mask =
        decision.authorization_failure_mask;
    if (!trajectory_publishable) {
      authorization_failure_mask |=
          SUPERVISOR_V2_AUTH_TRAJECTORY_NOT_PUBLISHABLE;
    }
    const auto tracking_reason =
        controllerTrackingStatusReason(now().seconds());
    auto authorization_failure_reasons =
        supervisorV2AuthorizationFailureReasons(authorization_failure_mask,
                                                tracking_reason);
    // shadow constraintへlegacy state/recovery guardを持ち込まない。V2が選んだ
    // 同一候補と入力鮮度だけから独立してconstraintを生成する。
    PlannerOutput shadow_output;
    switch (decision.phase) {
    case TacticalPhase::FREE_RUN:
      shadow_output.mode = BehaviorMode::FREE_RUN;
      break;
    case TacticalPhase::ATTACK_FOLLOW:
      shadow_output.mode = BehaviorMode::FOLLOW_BLOCKED;
      break;
    case TacticalPhase::PASSING:
      shadow_output.mode = decision.selected == CandidateType::PASS_RIGHT
                               ? BehaviorMode::OVERTAKE_RIGHT
                               : BehaviorMode::OVERTAKE_LEFT;
      break;
    case TacticalPhase::ABORT_HOLD:
      shadow_output.mode = BehaviorMode::ABORT_RECOVERY;
      break;
    }
    shadow_output.selected = decision.selected;
    shadow_output.blocked_info = output.blocked_info;
    shadow_output.reason = decision.reason;
    shadow_output.lateral_offsets = decision.trajectory.d;
    shadow_output.speed_caps = decision.trajectory.v_ref;
    shadow_output.longitudinal_offsets_m =
        decision.trajectory.longitudinal_offsets_m;
    shadow_output.active_override = candidate_trajectory_authorized;
    shadow_output.longitudinal_speed_cap_active =
        candidate_trajectory_authorized && !decision.trajectory.v_ref.empty();
    shadow_output.applied_speed_cap_mps =
        std::numeric_limits<double>::quiet_NaN();
    shadow_output.target_lateral_offset_m =
        decision.trajectory.d.empty() ? 0.0 : decision.trajectory.d.back();
    shadow_output.min_cbf_h = decision.trajectory.min_safety_margin;
    shadow_output.cbf_slack = decision.trajectory.cbf_slack;
    shadow_output.active_cbf_constraint_count =
        decision.trajectory.active_safety_constraint_count;
    for (const double cap_mps : decision.trajectory.v_ref) {
      if (std::isfinite(cap_mps) && cap_mps > 0.0) {
        shadow_output.applied_speed_cap_mps =
            std::isfinite(shadow_output.applied_speed_cap_mps)
                ? std::min(shadow_output.applied_speed_cap_mps, cap_mps)
                : cap_mps;
      }
    }
    if (!candidate_trajectory_authorized ||
        decision.selected == CandidateType::SAFE_STOP) {
      shadow_output.safe_stop_triggered = true;
      if (decision.selected == CandidateType::SAFE_STOP) {
        shadow_output.safe_stop_reason = "v2_abort_safe_stop";
      } else if (!authorization_failure_reasons.empty()) {
        shadow_output.safe_stop_reason =
            "v2_" + authorization_failure_reasons.front();
      } else {
        shadow_output.safe_stop_reason = "v2_trajectory_not_authorized";
      }
    }
    const auto prefilter_constraint = makeSafetyConstraint(
        shadow_output, ego, inputs, safety_constraint_normal_speed_limit_mps_,
        safety_constraint_maximum_brake_decel_mps2_);
    const auto constraint =
        supervisor_v2_constraint_release_gate_.filter(prefilter_constraint);
    if (constraint.stop_requested) {
      authorization_failure_mask |=
          SUPERVISOR_V2_AUTH_CONSTRAINT_STOP_REQUESTED;
      authorization_failure_reasons = supervisorV2AuthorizationFailureReasons(
          authorization_failure_mask, tracking_reason);
    }
    last_supervisor_v2_prefilter_constraint_reason_ =
        prefilter_constraint.reason;
    last_supervisor_v2_filtered_constraint_reason_ = constraint.reason;
    last_supervisor_v2_authorization_failure_mask_ = authorization_failure_mask;
    last_supervisor_v2_authorization_failure_reasons_ =
        authorization_failure_reasons;
    last_supervisor_v2_trajectory_publishable_ = trajectory_publishable;
    const bool effective_trajectory_authorized =
        candidate_trajectory_authorized && constraint.valid &&
        !constraint.stop_requested;
    last_supervisor_v2_effective_trajectory_authorized_ =
        effective_trajectory_authorized;
    if (!safetyConstraintSemanticallyEqual(
            constraint, last_supervisor_v2_constraint_command_) ||
        last_supervisor_v2_constraint_plan_generation_ !=
            decision.plan_generation) {
      supervisor_v2_constraint_generation_ =
          supervisor_v2_constraint_generation_ ==
                  std::numeric_limits<std::uint32_t>::max()
              ? 1U
              : supervisor_v2_constraint_generation_ + 1U;
    }
    last_supervisor_v2_constraint_command_ = constraint;
    last_supervisor_v2_constraint_plan_generation_ = decision.plan_generation;

    const auto stamp = now();
    multi_purpose_mpc_ros_msgs::msg::SafetyConstraint constraint_msg;
    constraint_msg.header.stamp = stamp;
    constraint_msg.header.frame_id = "map";
    constraint_msg.constraint_generation = supervisor_v2_constraint_generation_;
    constraint_msg.plan_generation = decision.plan_generation;
    constraint_msg.valid = constraint.valid;
    constraint_msg.stop_requested = constraint.stop_requested;
    constraint_msg.release_authorized = constraint.release_authorized;
    constraint_msg.speed_limit_mps =
        static_cast<float>(constraint.speed_limit_mps);
    constraint_msg.required_brake_decel_mps2 =
        static_cast<float>(constraint.required_brake_decel_mps2);
    constraint_msg.reason = constraint.reason;
    supervisor_v2_constraint_pub_->publish(constraint_msg);

    multi_purpose_mpc_ros_msgs::msg::OvertakePlan plan_msg;
    plan_msg.header.stamp = stamp;
    plan_msg.header.frame_id = "map";
    plan_msg.phase = static_cast<std::uint8_t>(decision.phase);
    plan_msg.plan_generation = decision.plan_generation;
    plan_msg.attempt_id = static_cast<std::uint32_t>(std::min<std::uint64_t>(
        decision.attempt_id, std::numeric_limits<std::uint32_t>::max()));
    plan_msg.target_vehicle_id = decision.target_vehicle_id;
    plan_msg.pass_direction = static_cast<std::int8_t>(decision.pass_direction);
    plan_msg.trajectory_authorized = effective_trajectory_authorized;
    plan_msg.lateral_maneuver_required = decision.lateral_maneuver_required;
    plan_msg.decision_reason = decision.reason;
    plan_msg.authorization_failure_mask = authorization_failure_mask;
    plan_msg.authorization_failure_reasons = authorization_failure_reasons;
    plan_msg.candidate_reject_reason = decision.candidate_reject_reason;
    plan_msg.safety_inputs_complete = decision.safety_inputs_complete;
    plan_msg.tracking_usable = decision.tracking_usable;
    plan_msg.trajectory_publishable = trajectory_publishable;
    plan_msg.constraint_reason = constraint.reason;
    plan_msg.trajectory.header = plan_msg.header;
    // LegacyとV2は同じNodeからpublishされるが、独立したgeneration/source
    // namespaceを持つproducerである。同一sim stamp、attempt、target、side、
    // generationが偶然一致してもPlanSampleKeyを衝突させない。
    plan_msg.planner_instance_id = v2_planner_instance_id_;
    plan_msg.race_arm_epoch = race_arm_epoch_;
    plan_msg.candidate_revision = plan_msg.plan_generation;
    plan_msg.aw2_identity_schema_version = 0U;
    const auto &trajectory = decision.trajectory;
    const std::size_t point_count = trajectory.x.size();
    if (trajectory_publishable) {
      plan_msg.trajectory.points.reserve(point_count);
      for (std::size_t i = 0; i < point_count; ++i) {
        autoware_auto_planning_msgs::msg::TrajectoryPoint point;
        point.pose.position.x = trajectory.x[i];
        point.pose.position.y = trajectory.y[i];
        point.pose.orientation = quaternionFromYaw(trajectory.yaw[i]);
        point.longitudinal_velocity_mps =
            static_cast<float>(trajectory.v_ref[i]);
        point.acceleration_mps2 = 0.0F;
        plan_msg.trajectory.points.push_back(point);
      }
    }
    aw2::V2CanonicalSource v2_source;
    v2_source.candidate_type = static_cast<std::uint8_t>(trajectory.type);
    v2_source.t = trajectory.t;
    v2_source.longitudinal_offsets_m = trajectory.longitudinal_offsets_m;
    v2_source.s = trajectory.s;
    v2_source.d = trajectory.d;
    v2_source.x = trajectory.x;
    v2_source.y = trajectory.y;
    v2_source.yaw = trajectory.yaw;
    v2_source.longitudinal_initial_measured_speed_mps =
        trajectory.longitudinal_initial_measured_speed_mps;
    v2_source.predicted_speed_mps = trajectory.predicted_speed_mps;
    v2_source.v_ref = trajectory.v_ref;
    v2_source.safety_evaluated = trajectory.safety_evaluated;
    v2_source.feasible = trajectory.feasible;
    v2_source.pass_target_corridor_valid =
        trajectory.pass_target_corridor_valid;
    v2_source.controller_tracking_profile_valid =
        trajectory.controller_tracking_profile_valid;
    v2_source.desired_path_trackable = trajectory.desired_path_trackable;
    v2_source.pure_pursuit_command_trackable =
        trajectory.pure_pursuit_command_trackable;
    v2_source.moving_target_relatively_reachable =
        trajectory.moving_target_relatively_reachable;
    v2_source.planned_target_d_m = trajectory.planned_target_d_m;
    v2_source.committed_attack_follow_target_d_m =
        trajectory.committed_attack_follow_target_d_m;
    v2_source.attack_follow_safe_lateral_hold =
        trajectory.attack_follow_safe_lateral_hold;
    v2_source.attack_follow_opponent_collision_current_d_hold =
        trajectory.attack_follow_opponent_collision_current_d_hold;
    v2_source.attack_follow_opponent_collision_inward_connector =
        trajectory.attack_follow_opponent_collision_inward_connector;
    v2_source.required_controller_spatial_horizon_m =
        trajectory.required_controller_spatial_horizon_m;
    v2_source.controller_spatial_horizon_proof_valid =
        trajectory.controller_spatial_horizon_proof_valid;
    v2_source.score = trajectory.score;
    v2_source.min_safety_margin = trajectory.min_safety_margin;
    v2_source.cbf_slack = trajectory.cbf_slack;
    v2_source.active_safety_constraint_count =
        trajectory.active_safety_constraint_count;
    v2_source.longitudinal_profile_valid =
        trajectory.longitudinal_profile_valid;
    v2_source.assumed_brake_decel_mps2 = trajectory.assumed_brake_decel_mps2;
    v2_source.response_delay_sec = trajectory.response_delay_sec;
    v2_source.required_brake_distance_m = trajectory.required_brake_distance_m;
    v2_source.available_brake_distance_m =
        trajectory.available_brake_distance_m;
    v2_source.reject_reason = trajectory.reject_reason;
    const auto v2_source_wire = aw2::canonicalizeV2SourceWire(v2_source);

    const bool transaction_active =
        !aw2_identity_exhausted_ && race_armed_ &&
        plan_msg.plan_generation != 0U && plan_msg.attempt_id != 0U &&
        !plan_msg.target_vehicle_id.empty() &&
        (plan_msg.pass_direction == -1 || plan_msg.pass_direction == 1);
    const auto connector_transaction_id =
        v2_connector_transaction_sequencer_.update(
            race_arm_epoch_, plan_msg.attempt_id, plan_msg.target_vehicle_id,
            plan_msg.pass_direction, transaction_active);
    if (connector_transaction_id.has_value() && v2_source_wire.has_value()) {
      plan_msg.connector_transaction_id = connector_transaction_id.value();
      aw2::CandidateContent content;
      content.key.transaction.race_arm_epoch = race_arm_epoch_;
      content.key.transaction.planner_instance_id = v2_planner_instance_id_;
      content.key.transaction.attempt_id = plan_msg.attempt_id;
      content.key.transaction.target_vehicle_id = plan_msg.target_vehicle_id;
      content.key.transaction.pass_direction = plan_msg.pass_direction;
      content.key.transaction.connector_transaction_id =
          connector_transaction_id.value();
      content.key.plan_stamp_sec = plan_msg.header.stamp.sec;
      content.key.plan_stamp_nanosec = plan_msg.header.stamp.nanosec;
      content.key.plan_generation = plan_msg.plan_generation;
      content.candidate_revision = plan_msg.candidate_revision;
      content.frame_id = plan_msg.header.frame_id;
      content.source_kind = aw2::CandidateSourceKind::V2_TRAJECTORY;
      content.geometry_point_count = plan_msg.trajectory.points.size();
      content.source_wire = v2_source_wire.value();
      const auto canonical = aw2::canonicalizeCandidateContentV1(content);
      if (canonical.valid()) {
        plan_msg.candidate_content_sha256 = canonical.sha256;
        aw2::CandidateBinding binding;
        binding.race_arm_epoch = race_arm_epoch_;
        binding.planner_instance_id = v2_planner_instance_id_;
        binding.plan_generation = plan_msg.plan_generation;
        binding.candidate_revision = plan_msg.candidate_revision;
        binding.candidate_content_sha256 = canonical.sha256;
        const auto observation =
            aw2_v2_candidate_binding_tracker_.observe(binding);
        plan_msg.aw2_identity_schema_version = aw2::identitySchemaVersion(
            true, connector_transaction_id, observation);
      }
    }
    (void)overtake_transport_contract::c002ay0::populateFreeRunPlanIdentityV1(
        plan_msg);
    supervisor_v2_plan_pub_->publish(plan_msg);
    const auto safety_evaluation_result =
        !trajectory.safety_evaluated
            ? aw2::SafetyEvaluationResult::NOT_EVALUATED
            : (trajectory.feasible ? aw2::SafetyEvaluationResult::PASSED
                                   : aw2::SafetyEvaluationResult::REJECTED);
    const double committed_target_d_m =
        std::isfinite(trajectory.committed_attack_follow_target_d_m)
            ? trajectory.committed_attack_follow_target_d_m
            : trajectory.planned_target_d_m;
    publishCandidateExecutionRequest(
        plan_msg, constraint_msg, plan_msg.pass_direction,
        static_cast<std::uint8_t>(decision.selected),
        aw2::CandidateSourceKind::V2_TRAJECTORY, v2_source_wire,
        trajectory.required_controller_spatial_horizon_m,
        trajectory.planned_target_d_m, committed_target_d_m,
        safety_evaluation_result,
        trajectory.reject_reason.empty() ? decision.reason
                                         : trajectory.reject_reason,
        current_safety_snapshot_id_, aw2_v2_delivery_record_tracker_);
  }

  void populateAw2AuthoritativePlanIdentity(
      multi_purpose_mpc_ros_msgs::msg::OvertakePlan &msg,
      const PlannerOutput &output, std::size_t semantic_geometry_point_count) {
    msg.aw2_identity_schema_version = 0U;
    msg.planner_instance_id = planner_instance_id_;
    msg.race_arm_epoch = race_arm_epoch_;
    msg.connector_transaction_id = 0U;
    msg.candidate_revision = msg.plan_generation;
    msg.candidate_content_sha256.fill(0U);

    const std::int8_t latched_pass_direction =
        output.blocked_info.maneuver_transaction_pass_type ==
                CandidateType::PASS_LEFT
            ? 1
            : (output.blocked_info.maneuver_transaction_pass_type ==
                       CandidateType::PASS_RIGHT
                   ? -1
                   : 0);
    const std::int8_t transaction_pass_direction =
        aw2::transactionPassDirection(
            msg.pass_direction,
            output.blocked_info.maneuver_transaction_incomplete,
            latched_pass_direction);
    const bool transaction_active =
        !aw2_identity_exhausted_ && race_armed_ && msg.attempt_id != 0U &&
        !msg.target_vehicle_id.empty() &&
        (transaction_pass_direction == -1 || transaction_pass_direction == 1);
    const auto connector_transaction_id =
        connector_transaction_sequencer_.update(
            race_arm_epoch_, msg.attempt_id, msg.target_vehicle_id,
            transaction_pass_direction, transaction_active);
    if (!connector_transaction_id.has_value()) {
      return;
    }
    msg.connector_transaction_id = connector_transaction_id.value();

    if (!last_aw2_source_wire_.has_value()) {
      return;
    }

    aw2::CandidateContent content;
    content.key.transaction.race_arm_epoch = race_arm_epoch_;
    content.key.transaction.planner_instance_id = planner_instance_id_;
    content.key.transaction.attempt_id = msg.attempt_id;
    content.key.transaction.target_vehicle_id = msg.target_vehicle_id;
    content.key.transaction.pass_direction = transaction_pass_direction;
    content.key.transaction.connector_transaction_id =
        connector_transaction_id.value();
    content.key.plan_stamp_sec = msg.header.stamp.sec;
    content.key.plan_stamp_nanosec = msg.header.stamp.nanosec;
    content.key.plan_generation = msg.plan_generation;
    content.candidate_revision = msg.candidate_revision;
    content.frame_id = msg.header.frame_id;
    content.source_kind = aw2::CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE;
    content.geometry_point_count = semantic_geometry_point_count;
    content.source_wire = last_aw2_source_wire_.value();
    const auto canonical = aw2::canonicalizeCandidateContentV1(content);
    if (!canonical.valid()) {
      return;
    }
    msg.candidate_content_sha256 = canonical.sha256;

    aw2::CandidateBinding binding;
    binding.race_arm_epoch = race_arm_epoch_;
    binding.planner_instance_id = planner_instance_id_;
    binding.plan_generation = msg.plan_generation;
    binding.candidate_revision = msg.candidate_revision;
    binding.candidate_content_sha256 = canonical.sha256;
    const auto observation = aw2_candidate_binding_tracker_.observe(binding);
    msg.aw2_identity_schema_version =
        aw2::identitySchemaVersion(true, connector_transaction_id, observation);
  }

  // 入力: 今周期のPlannerOutput。
  // 出力: debugで使うattempt_id。追い越し試行外なら0。
  // 処理概要: PREPARE/OVERTAKE開始から復帰/譲り完了まで同じIDを維持する。
  std::uint64_t updateAttemptId(const PlannerOutput &output) {
    const BehaviorMode mode = output.mode;
    // evalwrapでattemptを追跡できるよう、追い越し準備開始から復帰完了まで同じIDを出す。
    const bool starts_attempt = mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                                mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
                                mode == BehaviorMode::OVERTAKE_LEFT ||
                                mode == BehaviorMode::OVERTAKE_RIGHT ||
                                output.tracking_release_pass_warmup;
    if (!attempt_active_ && starts_attempt) {
      ++current_attempt_id_;
      attempt_active_ = true;
    }

    std::uint64_t publish_id = attempt_active_ ? current_attempt_id_ : 0;
    const bool yield_finishes_attempt =
        mode == BehaviorMode::YIELD_BEHIND &&
        !output.blocked_info.maneuver_transaction_incomplete &&
        (last_mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         last_mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         last_mode_ == BehaviorMode::OVERTAKE_LEFT ||
         last_mode_ == BehaviorMode::OVERTAKE_RIGHT);
    const bool recovery_handoff_finishes_attempt =
        (mode == BehaviorMode::FREE_RUN ||
         mode == BehaviorMode::FOLLOW_BLOCKED) &&
        !output.blocked_info.maneuver_transaction_incomplete &&
        (last_mode_ == BehaviorMode::MERGE_BACK ||
         last_mode_ == BehaviorMode::ABORT_RECOVERY ||
         last_mode_ == BehaviorMode::YIELD_BEHIND ||
         last_mode_ == BehaviorMode::SAFE_STOP);
    if (attempt_active_ &&
        (recovery_handoff_finishes_attempt || yield_finishes_attempt)) {
      publish_id = current_attempt_id_;
      attempt_active_ = false;
    }
    last_mode_ = mode;
    return publish_id;
  }

  // 入力: PlannerOutput、自車状態、attempt_id。
  // 出力: /debug/overtake/mode と /debug/overtake/metrics をpublishする。
  // 処理概要: 軽量mode文字列と、eval/report向けの詳細JSONを分けて出す。
  void publishDebug(const PlannerOutput &output, const EgoState &ego,
                    std::uint64_t attempt_id) {
    // modeだけの軽量トピックと、解析用の詳細JSONを分けてpublishする。
    std_msgs::msg::String mode_msg;
    mode_msg.data = toString(output.mode);
    mode_pub_->publish(mode_msg);

    const double debug_now_sec = now().seconds();
    const double controller_tracking_age_sec =
        last_controller_tracking_status_sec_.has_value()
            ? debug_now_sec - last_controller_tracking_status_sec_.value()
            : std::numeric_limits<double>::quiet_NaN();
    const std::string controller_tracking_status_reason =
        controllerTrackingStatusReason(debug_now_sec);
    const auto *controller_tracking_pass_identity =
        controllerTrackingManeuverIdentity();
    std::string controller_tracking_snapshot_classification = "none";
    if (last_controller_tracking_status_sec_.has_value()) {
      if (controller_tracking_plan_generation_ == override_generation_) {
        controller_tracking_snapshot_classification =
            controller_tracking_pass_identity != nullptr ? "exact" : "current";
      } else if (controller_tracking_pass_identity != nullptr) {
        controller_tracking_snapshot_classification = "N-1";
      }
    }

    const auto &inner_band = output.attack_follow_inner_band_diagnostic;
    std::ostringstream inner_band_probes_json;
    inner_band_probes_json << "[";
    const int inner_band_probe_count = std::clamp(
        inner_band.evaluated_probe_count, 0,
        static_cast<int>(AttackFollowInnerBandDiagnostic::kMaxProbeCount));
    for (int i = 0; i < inner_band_probe_count; ++i) {
      if (i > 0) {
        inner_band_probes_json << ",";
      }
      const auto &probe = inner_band.probes[static_cast<std::size_t>(i)];
      inner_band_probes_json
          << "{\"inward_delta_m\":" << jsonNumber(probe.inward_delta_m)
          << ",\"terminal_d_m\":" << jsonNumber(probe.terminal_d_m)
          << ",\"longitudinal_contract_unchanged\":"
          << (probe.longitudinal_contract_unchanged ? "true" : "false")
          << ",\"safety_evaluated\":"
          << (probe.safety_evaluated ? "true" : "false")
          << ",\"corridor_valid\":" << (probe.corridor_valid ? "true" : "false")
          << ",\"controller_tracking_profile_valid\":"
          << (probe.controller_tracking_profile_valid ? "true" : "false")
          << ",\"desired_path_trackable\":"
          << (probe.desired_path_trackable ? "true" : "false")
          << ",\"pure_pursuit_command_trackable\":"
          << (probe.pure_pursuit_command_trackable ? "true" : "false")
          << ",\"all_checks_pass\":"
          << (probe.all_checks_pass ? "true" : "false")
          << ",\"reject_reason\":\"" << probe.reject_reason << "\""
          << ",\"min_safety_margin\":" << jsonNumber(probe.min_safety_margin)
          << ",\"corridor_min_margin_m\":"
          << jsonNumber(probe.corridor_min_margin_m) << "}";
    }
    inner_band_probes_json << "]";

    const auto &state_lattice = output.state_lattice_shadow_comparison;
    const auto shadow_candidate_json =
        [](const StateLatticeShadowCandidateMetrics &metrics) {
          std::ostringstream json;
          json << "{\"safety_evaluated\":"
               << (metrics.safety_evaluated ? "true" : "false")
               << ",\"feasible\":" << (metrics.feasible ? "true" : "false")
               << ",\"wall_rejected\":"
               << (metrics.wall_rejected ? "true" : "false")
               << ",\"wall_clearance_m\":"
               << jsonNumber(metrics.wall_clearance_m)
               << ",\"cbf_min_margin\":" << jsonNumber(metrics.cbf_min_margin)
               << ",\"cbf_slack\":" << jsonNumber(metrics.cbf_slack)
               << ",\"cbf_blocking_opponent_id\":\""
               << metrics.cbf_blocking_opponent_id << "\""
               << ",\"cbf_blocking_time_sec\":"
               << jsonNumber(metrics.cbf_blocking_time_sec)
               << ",\"pure_pursuit_required_arc_m\":"
               << jsonNumber(metrics.pure_pursuit_required_arc_m)
               << ",\"pure_pursuit_available_arc_m\":"
               << jsonNumber(metrics.pure_pursuit_available_arc_m)
               << ",\"deadline_required_arc_m\":"
               << jsonNumber(metrics.deadline_required_arc_m)
               << ",\"deadline_available_arc_m\":"
               << jsonNumber(metrics.deadline_available_arc_m)
               << ",\"deadline_slack_m\":"
               << jsonNumber(metrics.deadline_slack_m)
               << ",\"first_reject_reason\":\"" << metrics.first_reject_reason
               << "\"}";
          return json.str();
        };
    std::ostringstream state_lattice_json;
    state_lattice_json
        << "{\"requested\":" << (state_lattice.requested ? "true" : "false")
        << ",\"snapshot_complete\":"
        << (state_lattice.snapshot_complete ? "true" : "false")
        << ",\"geometry_generated\":"
        << (state_lattice.geometry_generated ? "true" : "false")
        << ",\"adapter_valid\":"
        << (state_lattice.adapter_valid ? "true" : "false")
        << ",\"cartesian_trackability_valid\":"
        << (state_lattice.cartesian_trackability_valid ? "true" : "false")
        << ",\"cartesian_trackable\":"
        << (state_lattice.cartesian_trackable ? "true" : "false")
        << ",\"current_pp_exact_snapshot_valid\":"
        << (state_lattice.current_pp_exact_snapshot_valid ? "true" : "false")
        << ",\"current_pp_exact_snapshot_reason\":\""
        << state_lattice.current_pp_exact_snapshot_reason << "\""
        << ",\"current_pp_command_sequence\":"
        << state_lattice.current_pp_command_sequence
        << ",\"current_pp_valid_until_sec\":"
        << jsonNumber(state_lattice.current_pp_valid_until_sec)
        << ",\"current_pp_resolved_lookahead_m\":"
        << jsonNumber(state_lattice.current_pp_resolved_lookahead_m)
        << ",\"current_pp_requested_steering_rad\":"
        << jsonNumber(state_lattice.current_pp_requested_steering_rad)
        << ",\"current_pp_bounded_steering_rad\":"
        << jsonNumber(state_lattice.current_pp_bounded_steering_rad)
        << ",\"evaluated\":" << (state_lattice.evaluated ? "true" : "false")
        << ",\"status_reason\":\"" << state_lattice.status_reason << "\""
        << ",\"target_id\":\"" << state_lattice.target_id << "\""
        << ",\"pass_type\":\"" << toString(state_lattice.pass_type) << "\""
        << ",\"pass_side\":" << state_lattice.pass_side
        << ",\"snapshot_cycle\":" << state_lattice.snapshot_cycle
        << ",\"snapshot_stamp_sec\":"
        << jsonNumber(state_lattice.snapshot_stamp_sec)
        << ",\"ego_stamp_sec\":" << jsonNumber(state_lattice.ego_stamp_sec)
        << ",\"opponent_count\":" << state_lattice.opponent_count
        << ",\"snapshot_hash\":\"" << state_lattice.snapshot_hash << "\""
        << ",\"current\":" << shadow_candidate_json(state_lattice.current)
        << ",\"lattice\":" << shadow_candidate_json(state_lattice.lattice)
        << "}";

    std_msgs::msg::String metrics_msg;
    std::ostringstream oss;
    oss << "{"
        << "\"mode\":\"" << toString(output.mode) << "\","
        << "\"race_arm_required\":" << (race_arm_required_ ? "true" : "false")
        << ","
        << "\"race_armed\":" << (race_armed_ ? "true" : "false") << ","
        << "\"race_arm_epoch\":" << race_arm_epoch_ << ","
        << "\"overtake_state\":\"" << toString(output.mode) << "\","
        << "\"attempt_id\":" << attempt_id << ","
        << "\"selected\":\"" << toString(output.selected) << "\","
        << "\"planner_update_steady_duration_ms\":"
        << jsonNumber(planner_update_steady_duration_ms_) << ","
        << "\"planner_proposal_interval_ms\":"
        << jsonNumber(planner_proposal_interval_ms_) << ","
        << "\"state_lattice_shadow_comparison\":" << state_lattice_json.str()
        << ","
        << "\"start_grid_target_active\":"
        << (output.blocked_info.start_grid_target_active ? "true" : "false")
        << ","
        << "\"start_grid_uncommitted_hold_active\":"
        << (output.blocked_info.start_grid_uncommitted_hold_active ? "true"
                                                                   : "false")
        << ","
        << "\"start_grid_lateral_release_pending\":"
        << (output.blocked_info.start_grid_lateral_release_pending ? "true"
                                                                   : "false")
        << ","
        << "\"start_grid_hold_target_d_m\":"
        << jsonNumber(output.blocked_info.start_grid_hold_target_d_m) << ","
        << "\"start_grid_target_confirmation_pending\":"
        << (output.blocked_info.start_grid_target_confirmation_pending
                ? "true"
                : "false")
        << ","
        << "\"start_grid_target_confirmed_stationary\":"
        << (output.blocked_info.start_grid_target_confirmed_stationary
                ? "true"
                : "false")
        << ","
        << "\"start_grid_target_superseded_by_blocked_front\":"
        << (output.blocked_info.start_grid_target_superseded_by_blocked_front
                ? "true"
                : "false")
        << ","
        << "\"start_grid_target_reselection_suppressed\":"
        << (output.blocked_info.start_grid_target_reselection_suppressed
                ? "true"
                : "false")
        << ","
        << "\"start_grid_superseded_target_id\":\""
        << output.blocked_info.start_grid_superseded_target_id << "\","
        << "\"start_grid_replacement_target_id\":\""
        << output.blocked_info.start_grid_replacement_target_id << "\","
        << "\"start_grid_target_id\":\""
        << output.blocked_info.start_grid_target_id << "\","
        << "\"start_grid_target_delta_s\":"
        << jsonNumber(output.blocked_info.start_grid_target_delta_s) << ","
        << "\"start_grid_target_delta_d\":"
        << jsonNumber(output.blocked_info.start_grid_target_delta_d) << ","
        << "\"start_grid_target_age_sec\":"
        << jsonNumber(output.blocked_info.start_grid_target_age_sec) << ","
        << "\"start_grid_ego_progress_m\":"
        << jsonNumber(output.blocked_info.start_grid_ego_progress_m) << ","
        << "\"blocked\":" << (output.blocked_info.blocked ? "true" : "false")
        << ","
        << "\"side_by_side\":"
        << (output.blocked_info.side_by_side ? "true" : "false") << ","
        << "\"corner_side_by_side\":"
        << (output.blocked_info.corner_side_by_side ? "true" : "false") << ","
        << "\"parallel_side_candidate\":"
        << (output.blocked_info.parallel_side_candidate ? "true" : "false")
        << ","
        << "\"parallel_yield_hold_lateral\":"
        << (output.blocked_info.parallel_yield_hold_lateral ? "true" : "false")
        << ","
        << "\"corner_abs_curvature\":"
        << jsonNumber(output.blocked_info.corner_abs_curvature) << ","
        << "\"straight_overtake_start_allowed\":"
        << (output.blocked_info.straight_overtake_start_allowed ? "true"
                                                                : "false")
        << ","
        << "\"overtake_start_abs_curvature\":"
        << jsonNumber(output.blocked_info.overtake_start_abs_curvature) << ","
        << "\"overtake_start_gate_reason\":\""
        << output.blocked_info.overtake_start_gate_reason << "\","
        << "\"gentle_curve_target_identity_valid\":"
        << (output.blocked_info.gentle_curve_target_identity_valid ? "true"
                                                                   : "false")
        << ","
        << "\"gentle_curve_direct_normal_target\":"
        << (output.blocked_info.gentle_curve_direct_normal_target ? "true"
                                                                  : "false")
        << ","
        << "\"gentle_curve_direct_braking_target\":"
        << (output.blocked_info.gentle_curve_direct_braking_target ? "true"
                                                                   : "false")
        << ","
        << "\"gentle_curve_fresh_dynamic_gap_target\":"
        << (output.blocked_info.gentle_curve_fresh_dynamic_gap_target ? "true"
                                                                      : "false")
        << ","
        << "\"gentle_curve_dynamic_target_speed_reachable\":"
        << (output.blocked_info.gentle_curve_dynamic_target_speed_reachable
                ? "true"
                : "false")
        << ","
        << "\"gentle_curve_target_context_reason\":\""
        << output.blocked_info.gentle_curve_target_context_reason << "\","
        << "\"gentle_curve_context_clean\":"
        << (output.blocked_info.gentle_curve_context_clean ? "true" : "false")
        << ","
        << "\"gentle_curve_lateral_capacity_sufficient\":"
        << (output.blocked_info.gentle_curve_lateral_capacity_sufficient
                ? "true"
                : "false")
        << ","
        << "\"gentle_curve_dynamic_speed_cap_valid\":"
        << (output.blocked_info.gentle_curve_dynamic_speed_cap_valid ? "true"
                                                                     : "false")
        << ","
        << "\"gentle_curve_curvature_within_limit\":"
        << (output.blocked_info.gentle_curve_curvature_within_limit ? "true"
                                                                    : "false")
        << ","
        << "\"gentle_curve_observation_inputs_complete\":"
        << (output.blocked_info.gentle_curve_observation_inputs_complete
                ? "true"
                : "false")
        << ","
        << "\"gentle_curve_prediction_complete\":"
        << (output.blocked_info.gentle_curve_prediction_complete ? "true"
                                                                 : "false")
        << ","
        << "\"gentle_curve_tracking_usable\":"
        << (output.blocked_info.gentle_curve_tracking_usable ? "true" : "false")
        << ","
        << "\"gentle_curve_mpc_ready\":"
        << (output.blocked_info.gentle_curve_mpc_ready ? "true" : "false")
        << ","
        << "\"gentle_curve_safe_pass_found\":"
        << (output.blocked_info.gentle_curve_safe_pass_found ? "true" : "false")
        << ","
        << "\"gentle_curve_safe_pass_side\":\""
        << toString(output.blocked_info.gentle_curve_safe_pass_side) << "\","
        << "\"gentle_curve_safe_pass_cbf_slack\":"
        << jsonNumber(output.blocked_info.gentle_curve_safe_pass_cbf_slack)
        << ","
        << "\"gentle_curve_safe_pass_block_reason\":\""
        << output.blocked_info.gentle_curve_safe_pass_block_reason << "\","
        << "\"overtake_permission_allowed\":"
        << (output.blocked_info.overtake_permission_allowed ? "true" : "false")
        << ","
        << "\"overtake_permission_section_name\":\""
        << output.blocked_info.overtake_permission_section_name << "\","
        << "\"overtake_permission_reason\":\""
        << output.blocked_info.overtake_permission_reason << "\","
        << "\"confirmed_stationary_parallel_permission_exception\":"
        << (output.blocked_info
                    .confirmed_stationary_parallel_permission_exception
                ? "true"
                : "false")
        << ","
        << "\"permission_start_exception_active\":"
        << (output.blocked_info.permission_start_exception_active ? "true"
                                                                  : "false")
        << ","
        << "\"front_vehicle_low_speed\":"
        << (output.blocked_info.front_vehicle_low_speed ? "true" : "false")
        << ","
        << "\"slow_front_exception_active\":"
        << (output.blocked_info.slow_front_exception_active ? "true" : "false")
        << ","
        << "\"slow_front_exception_count\":"
        << output.blocked_info.slow_front_exception_count << ","
        << "\"slow_obstacle_chain_active\":"
        << (output.blocked_info.slow_obstacle_chain_active ? "true" : "false")
        << ","
        << "\"slow_obstacle_chain_id\":\""
        << output.blocked_info.slow_obstacle_chain_id << "\","
        << "\"slow_obstacle_chain_delta_s\":"
        << jsonNumber(output.blocked_info.slow_obstacle_chain_delta_s) << ","
        << "\"slow_obstacle_chain_delta_d\":"
        << jsonNumber(output.blocked_info.slow_obstacle_chain_delta_d) << ","
        << "\"slow_obstacle_chain_speed_mps\":"
        << jsonNumber(output.blocked_info.slow_obstacle_chain_speed_mps) << ","
        << "\"stationary_front_obstacle\":"
        << (output.blocked_info.stationary_front_obstacle ? "true" : "false")
        << ","
        << "\"stationary_front_id\":\""
        << output.blocked_info.stationary_front_id << "\","
        << "\"stationary_front_ttc_sec\":"
        << jsonNumber(output.blocked_info.stationary_front_ttc_sec) << ","
        << "\"stationary_front_brake_feasible\":"
        << (output.blocked_info.stationary_front_brake_feasible ? "true"
                                                                : "false")
        << ","
        << "\"stationary_front_required_brake_distance_m\":"
        << jsonNumber(
               output.blocked_info.stationary_front_required_brake_distance_m)
        << ","
        << "\"stationary_front_available_brake_distance_m\":"
        << jsonNumber(
               output.blocked_info.stationary_front_available_brake_distance_m)
        << ","
        << "\"braking_follow_active\":"
        << (output.blocked_info.braking_follow_active ? "true" : "false") << ","
        << "\"braking_follow_feasible\":"
        << (output.blocked_info.braking_follow_feasible ? "true" : "false")
        << ","
        << "\"braking_follow_candidate_reject_reason\":\""
        << output.blocked_info.braking_follow_candidate_reject_reason << "\","
        << "\"braking_follow_candidate_tracking_profile_valid\":"
        << (output.blocked_info.braking_follow_candidate_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"braking_follow_candidate_min_safety_margin\":"
        << jsonNumber(
               output.blocked_info.braking_follow_candidate_min_safety_margin)
        << ","
        << "\"braking_follow_id\":\"" << output.blocked_info.braking_follow_id
        << "\","
        << "\"braking_follow_delta_s\":"
        << jsonNumber(output.blocked_info.braking_follow_delta_s) << ","
        << "\"braking_follow_required_distance_m\":"
        << jsonNumber(output.blocked_info.braking_follow_required_distance_m)
        << ","
        << "\"braking_follow_trigger_distance_m\":"
        << jsonNumber(output.blocked_info.braking_follow_trigger_distance_m)
        << ","
        << "\"braking_follow_available_distance_m\":"
        << jsonNumber(output.blocked_info.braking_follow_available_distance_m)
        << ","
        << "\"braking_follow_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.braking_follow_relative_speed_mps)
        << ","
        << "\"braking_follow_target_speed_mps\":"
        << jsonNumber(output.blocked_info.braking_follow_target_speed_mps)
        << ","
        << "\"braking_follow_ttc_sec\":"
        << jsonNumber(output.blocked_info.braking_follow_ttc_sec) << ","
        << "\"braking_follow_speed_cap_mps\":"
        << jsonNumber(output.blocked_info.braking_follow_speed_cap_mps) << ","
        << "\"stationary_no_pass_safe_pass_eligible\":"
        << (output.blocked_info.stationary_no_pass_safe_pass_eligible ? "true"
                                                                      : "false")
        << ","
        << "\"stationary_no_pass_safe_pass_start_approved\":"
        << (output.blocked_info.stationary_no_pass_safe_pass_start_approved
                ? "true"
                : "false")
        << ","
        << "\"stationary_no_pass_safe_pass_speed_cap_mps\":"
        << jsonNumber(
               output.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps)
        << ","
        << "\"future_side_by_side\":"
        << (output.blocked_info.future_side_by_side ? "true" : "false") << ","
        << "\"future_corner_side_by_side\":"
        << (output.blocked_info.future_corner_side_by_side ? "true" : "false")
        << ","
        << "\"future_delta_s\":"
        << jsonNumber(output.blocked_info.future_delta_s) << ","
        << "\"future_delta_d\":"
        << jsonNumber(output.blocked_info.future_delta_d) << ","
        << "\"future_wall_clearance_m\":"
        << jsonNumber(output.blocked_info.future_wall_clearance_m) << ","
        << "\"future_abs_curvature\":"
        << jsonNumber(output.blocked_info.future_abs_curvature) << ","
        << "\"future_outer_wall_risk\":"
        << (output.blocked_info.future_outer_wall_risk ? "true" : "false")
        << ","
        << "\"future_yield_required\":"
        << (output.blocked_info.future_yield_required ? "true" : "false") << ","
        << "\"predictive_pass_target_shadow_evaluated\":"
        << (output.blocked_info.predictive_pass_target_shadow.evaluated
                ? "true"
                : "false")
        << ","
        << "\"predictive_pass_target_shadow_inputs_complete\":"
        << (output.blocked_info.predictive_pass_target_shadow.inputs_complete
                ? "true"
                : "false")
        << ","
        << "\"predictive_pass_target_shadow_valid\":"
        << (output.blocked_info.predictive_pass_target_shadow.valid ? "true"
                                                                    : "false")
        << ","
        << "\"predictive_pass_target_shadow_id\":\""
        << output.blocked_info.predictive_pass_target_shadow.target_id << "\","
        << "\"predictive_pass_target_shadow_source\":\""
        << output.blocked_info.predictive_pass_target_shadow.source << "\","
        << "\"predictive_pass_target_shadow_reason\":\""
        << output.blocked_info.predictive_pass_target_shadow.reason << "\","
        << "\"predictive_pass_target_shadow_current_delta_s_m\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .current_delta_s_m)
        << ","
        << "\"predictive_pass_target_shadow_current_delta_d_m\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .current_delta_d_m)
        << ","
        << "\"predictive_pass_target_shadow_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .relative_speed_mps)
        << ","
        << "\"predictive_pass_target_shadow_earliest_time_sec\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .earliest_blocking_time_sec)
        << ","
        << "\"predictive_pass_target_shadow_opponent_s_m\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .predicted_opponent_s_m)
        << ","
        << "\"predictive_pass_target_shadow_opponent_d_m\":"
        << jsonNumber(output.blocked_info.predictive_pass_target_shadow
                          .predicted_opponent_d_m)
        << ","
        << "\"future_parallel_interaction\":"
        << (output.blocked_info.future_parallel_interaction ? "true" : "false")
        << ","
        << "\"future_prediction_time_sec\":"
        << jsonNumber(output.blocked_info.future_prediction_time_sec) << ","
        << "\"predicted_opponent_s\":"
        << jsonNumber(output.blocked_info.predicted_opponent_s) << ","
        << "\"predicted_opponent_d\":"
        << jsonNumber(output.blocked_info.predicted_opponent_d) << ","
        << "\"yield_reason\":\"" << output.blocked_info.yield_reason << "\","
        << "\"front_vehicle_id\":\"" << output.blocked_info.nearest_id << "\","
        << "\"target_vehicle_id\":\""
        << authoritativeTargetVehicleId(output.blocked_info) << "\","
        << "\"front_delta_s\":" << jsonNumber(output.blocked_info.front_delta_s)
        << ","
        << "\"front_distance_m\":"
        << jsonNumber(output.blocked_info.front_delta_s) << ","
        << "\"front_delta_d\":" << jsonNumber(output.blocked_info.front_delta_d)
        << ","
        << "\"front_rel_v\":" << jsonNumber(output.blocked_info.front_rel_v)
        << ","
        << "\"front_vehicle_speed_mps\":"
        << jsonNumber(output.blocked_info.front_vehicle_speed_mps) << ","
        << "\"front_s_dot_mps\":"
        << jsonNumber(output.blocked_info.front_s_dot_mps) << ","
        << "\"front_same_direction\":"
        << (output.blocked_info.front_same_direction ? "true" : "false") << ","
        << "\"front_direction_known\":"
        << (output.blocked_info.front_direction_known ? "true" : "false") << ","
        << "\"relative_speed_mps\":"
        << jsonNumber(output.blocked_info.front_rel_v) << ","
        << "\"side_vehicle_id\":\"" << output.blocked_info.side_id << "\","
        << "\"side_delta_s\":" << jsonNumber(output.blocked_info.side_delta_s)
        << ","
        << "\"side_delta_d\":" << jsonNumber(output.blocked_info.side_delta_d)
        << ","
        << "\"side_lateral_gap_m\":"
        << jsonNumber(std::abs(output.blocked_info.side_delta_d)) << ","
        << "\"side_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.side_rel_v) << ","
        << "\"side_s_dot_mps\":"
        << jsonNumber(output.blocked_info.side_s_dot_mps) << ","
        << "\"side_same_direction\":"
        << (output.blocked_info.side_same_direction ? "true" : "false") << ","
        << "\"side_direction_known\":"
        << (output.blocked_info.side_direction_known ? "true" : "false") << ","
        << "\"parallel_side_vehicle_id\":\""
        << output.blocked_info.parallel_side_id << "\","
        << "\"parallel_side_delta_s\":"
        << jsonNumber(output.blocked_info.parallel_side_delta_s) << ","
        << "\"parallel_side_delta_d\":"
        << jsonNumber(output.blocked_info.parallel_side_delta_d) << ","
        << "\"parallel_side_lateral_gap_m\":"
        << jsonNumber(std::abs(output.blocked_info.parallel_side_delta_d))
        << ","
        << "\"parallel_side_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.parallel_side_rel_v) << ","
        << "\"parallel_side_s_dot_mps\":"
        << jsonNumber(output.blocked_info.parallel_side_s_dot_mps) << ","
        << "\"parallel_side_same_direction\":"
        << (output.blocked_info.parallel_side_same_direction ? "true" : "false")
        << ","
        << "\"parallel_side_direction_known\":"
        << (output.blocked_info.parallel_side_direction_known ? "true"
                                                              : "false")
        << ","
        << "\"parallel_follow_candidate\":"
        << (output.blocked_info.parallel_follow_candidate ? "true" : "false")
        << ","
        << "\"parallel_follow_feasible\":"
        << (output.blocked_info.parallel_follow_feasible ? "true" : "false")
        << ","
        << "\"parallel_follow_recheck_attempted\":"
        << (output.blocked_info.parallel_follow_recheck_attempted ? "true"
                                                                  : "false")
        << ","
        << "\"parallel_follow_recheck_reason\":\""
        << output.blocked_info.parallel_follow_recheck_reason << "\","
        << "\"attack_follow_hold_pass_side\":"
        << (output.blocked_info.attack_follow_hold_pass_side ? "true" : "false")
        << ","
        << "\"attack_follow_acceleration_allowed\":"
        << (output.blocked_info.attack_follow_acceleration_allowed ? "true"
                                                                   : "false")
        << ","
        << "\"pass_acceleration_allowed\":"
        << (output.blocked_info.pass_acceleration_allowed ? "true" : "false")
        << ","
        << "\"pass_lateral_first_speed_gate_active\":"
        << (output.blocked_info.pass_lateral_first_speed_gate_active ? "true"
                                                                     : "false")
        << ","
        << "\"pass_lateral_clearance_ready\":"
        << (output.blocked_info.pass_lateral_clearance_ready ? "true" : "false")
        << ","
        << "\"pass_lateral_first_target_id\":\""
        << output.blocked_info.pass_lateral_first_target_id << "\","
        << "\"pass_lateral_first_target_relative_s_m\":"
        << jsonNumber(
               output.blocked_info.pass_lateral_first_target_relative_s_m)
        << ","
        << "\"pass_lateral_separation_actual_m\":"
        << jsonNumber(output.blocked_info.pass_lateral_separation_actual_m)
        << ","
        << "\"pass_lateral_separation_required_m\":"
        << jsonNumber(output.blocked_info.pass_lateral_separation_required_m)
        << ","
        << "\"pass_lateral_first_target_speed_mps\":"
        << jsonNumber(output.blocked_info.pass_lateral_first_target_speed_mps)
        << ","
        << "\"pass_lateral_first_speed_cap_mps\":"
        << jsonNumber(output.blocked_info.pass_lateral_first_speed_cap_mps)
        << ","
        << "\"attack_follow_target_d_m\":"
        << jsonNumber(output.blocked_info.attack_follow_target_d_m) << ","
        << "\"prestart_attack_follow_hold_lateral\":"
        << (output.blocked_info.prestart_attack_follow_hold_lateral ? "true"
                                                                    : "false")
        << ","
        << "\"attack_follow_candidate_generated\":"
        << (output.blocked_info.attack_follow_candidate_generated ? "true"
                                                                  : "false")
        << ","
        << "\"attack_follow_candidate_feasible\":"
        << (output.blocked_info.attack_follow_candidate_feasible ? "true"
                                                                 : "false")
        << ","
        << "\"attack_follow_candidate_safe_lateral_hold\":"
        << (output.blocked_info.attack_follow_candidate_safe_lateral_hold
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_candidate_opponent_collision_current_d_hold\":"
        << (output.blocked_info
                    .attack_follow_candidate_opponent_collision_current_d_hold
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_candidate_opponent_collision_inward_connector\":"
        << (output.blocked_info
                    .attack_follow_candidate_opponent_collision_inward_connector
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_current_d_hold_variant_generated\":"
        << (output.blocked_info.attack_follow_current_d_hold_variant_generated
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_current_d_hold_variant_feasible\":"
        << (output.blocked_info.attack_follow_current_d_hold_variant_feasible
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_current_d_hold_variant_used\":"
        << (output.blocked_info.attack_follow_current_d_hold_variant_used
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_current_d_hold_variant_reject_reason\":\""
        << output.blocked_info
               .attack_follow_current_d_hold_variant_reject_reason
        << "\","
        << "\"attack_follow_inward_connector_variant_generated\":"
        << (output.blocked_info.attack_follow_inward_connector_variant_generated
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_inward_connector_variant_feasible\":"
        << (output.blocked_info.attack_follow_inward_connector_variant_feasible
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_inward_connector_variant_used\":"
        << (output.blocked_info.attack_follow_inward_connector_variant_used
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_inward_connector_direction_sign\":"
        << output.blocked_info.attack_follow_inward_connector_direction_sign
        << ","
        << "\"attack_follow_inward_connector_terminal_d_m\":"
        << jsonNumber(
               output.blocked_info.attack_follow_inward_connector_terminal_d_m)
        << ","
        << "\"attack_follow_inward_connector_variant_reject_reason\":\""
        << output.blocked_info
               .attack_follow_inward_connector_variant_reject_reason
        << "\","
        << "\"attack_follow_current_d_hold_source_blocking_opponent_id\":\""
        << output.blocked_info
               .attack_follow_current_d_hold_source_blocking_opponent_id
        << "\","
        << "\"attack_follow_current_d_hold_source_blocking_time_sec\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_source_blocking_time_sec)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_footprint_valid\":"
        << (output.blocked_info
                    .attack_follow_current_d_hold_blocking_wall_footprint_valid
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_segment_index\":"
        << (output.blocked_info
                    .attack_follow_current_d_hold_blocking_wall_footprint_valid
                ? std::to_string(
                      output.blocked_info
                          .attack_follow_current_d_hold_blocking_wall_segment_index)
                : "null")
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_segment_ratio\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_segment_ratio)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corner_index\":"
        << (output.blocked_info
                    .attack_follow_current_d_hold_blocking_wall_footprint_valid
                ? std::to_string(
                      output.blocked_info
                          .attack_follow_current_d_hold_blocking_wall_corner_index)
                : "null")
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_time_sec\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_current_d_hold_blocking_wall_time_sec)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_candidate_x_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_candidate_x_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_candidate_y_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_candidate_y_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_candidate_yaw_rad\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_candidate_yaw_rad)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_candidate_s_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_candidate_s_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_candidate_d_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_candidate_d_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corner_x_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corner_x_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corner_y_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corner_y_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corner_s_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corner_s_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corner_d_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corner_d_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corridor_d_min_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corridor_d_min_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_corridor_d_max_m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_corridor_d_max_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_physical_clearance_"
           "m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_physical_clearance_m)
        << ","
        << "\"attack_follow_current_d_hold_blocking_wall_effective_clearance_"
           "m\":"
        << jsonNumber(
               output.blocked_info
                   .attack_follow_current_d_hold_blocking_wall_effective_clearance_m)
        << ","
        << "\"attack_follow_inner_band_evaluated\":"
        << (inner_band.evaluated ? "true" : "false") << ","
        << "\"attack_follow_inner_band_rate_limited\":"
        << (inner_band.rate_limited ? "true" : "false") << ","
        << "\"attack_follow_inner_band_complete\":"
        << (inner_band.complete ? "true" : "false") << ","
        << "\"attack_follow_inner_band_status_reason\":\""
        << inner_band.status_reason << "\","
        << "\"attack_follow_inner_band_source_stamp_sec\":"
        << jsonNumber(inner_band.source_stamp_sec) << ","
        << "\"attack_follow_inner_band_target_id\":\"" << inner_band.target_id
        << "\","
        << "\"attack_follow_inner_band_pass_type\":\""
        << toString(inner_band.pass_type) << "\","
        << "\"attack_follow_inner_band_source_current_d_m\":"
        << jsonNumber(inner_band.source_current_d_m) << ","
        << "\"attack_follow_inner_band_committed_target_d_m\":"
        << jsonNumber(inner_band.committed_target_d_m) << ","
        << "\"attack_follow_inner_band_inward_direction_sign\":"
        << inner_band.inward_direction_sign << ","
        << "\"attack_follow_inner_band_evaluated_probe_count\":"
        << inner_band.evaluated_probe_count << ","
        << "\"attack_follow_inner_band_feasible_probe_count\":"
        << inner_band.feasible_probe_count << ","
        << "\"attack_follow_inner_band_wall_reject_count\":"
        << inner_band.wall_reject_count << ","
        << "\"attack_follow_inner_band_opponent_reject_count\":"
        << inner_band.opponent_reject_count << ","
        << "\"attack_follow_inner_band_other_reject_count\":"
        << inner_band.other_reject_count << ","
        << "\"attack_follow_inner_band_outermost_feasible_d_m\":"
        << jsonNumber(inner_band.outermost_feasible_d_m) << ","
        << "\"attack_follow_inner_band_innermost_feasible_d_m\":"
        << jsonNumber(inner_band.innermost_feasible_d_m) << ","
        << "\"attack_follow_inner_band_probes\":"
        << inner_band_probes_json.str() << ","
        << "\"attack_follow_candidate_tracking_profile_valid\":"
        << (output.blocked_info.attack_follow_candidate_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"attack_follow_candidate_reject_reason\":\""
        << output.blocked_info.attack_follow_candidate_reject_reason << "\","
        << "\"attack_follow_candidate_planned_target_d_m\":"
        << jsonNumber(
               output.blocked_info.attack_follow_candidate_planned_target_d_m)
        << ","
        << "\"attack_follow_candidate_committed_target_d_m\":"
        << jsonNumber(
               output.blocked_info.attack_follow_candidate_committed_target_d_m)
        << ","
        << "\"attack_follow_candidate_corridor_min_margin_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_corridor_min_margin_m)
        << ","
        << "\"attack_follow_candidate_min_safety_margin\":"
        << jsonNumber(
               output.blocked_info.attack_follow_candidate_min_safety_margin)
        << ","
        << "\"attack_follow_candidate_blocking_opponent_id\":\""
        << output.blocked_info.attack_follow_candidate_blocking_opponent_id
        << "\","
        << "\"attack_follow_candidate_blocking_time_sec\":"
        << jsonNumber(
               output.blocked_info.attack_follow_candidate_blocking_time_sec)
        << ","
        << "\"attack_follow_candidate_blocking_candidate_x_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_candidate_x_m)
        << ","
        << "\"attack_follow_candidate_blocking_candidate_y_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_candidate_y_m)
        << ","
        << "\"attack_follow_candidate_blocking_candidate_yaw_rad\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_candidate_yaw_rad)
        << ","
        << "\"attack_follow_candidate_blocking_candidate_s_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_candidate_s_m)
        << ","
        << "\"attack_follow_candidate_blocking_candidate_d_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_candidate_d_m)
        << ","
        << "\"attack_follow_candidate_blocking_opponent_x_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_opponent_x_m)
        << ","
        << "\"attack_follow_candidate_blocking_opponent_y_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_opponent_y_m)
        << ","
        << "\"attack_follow_candidate_blocking_opponent_s_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_opponent_s_m)
        << ","
        << "\"attack_follow_candidate_blocking_opponent_d_m\":"
        << jsonNumber(output.blocked_info
                          .attack_follow_candidate_blocking_opponent_d_m)
        << ","
        << "\"maneuver_transaction_incomplete\":"
        << (output.blocked_info.maneuver_transaction_incomplete ? "true"
                                                                : "false")
        << ","
        << "\"maneuver_transaction_prepared\":"
        << (output.blocked_info.maneuver_transaction_prepared ? "true"
                                                              : "false")
        << ","
        << "\"maneuver_transaction_pass_type\":\""
        << toString(output.blocked_info.maneuver_transaction_pass_type) << "\","
        << "\"pass_start_target_continuity_active\":"
        << (output.blocked_info.pass_start_target_continuity_active ? "true"
                                                                    : "false")
        << ","
        << "\"pass_start_target_continuity_cycles\":"
        << output.blocked_info.pass_start_target_continuity_cycles << ","
        << "\"pass_start_target_continuity_budget_cycles\":"
        << output.blocked_info.pass_start_target_continuity_budget_cycles << ","
        << "\"pass_start_target_continuity_expired\":"
        << (output.blocked_info.pass_start_target_continuity_expired ? "true"
                                                                     : "false")
        << ","
        << "\"pass_start_target_continuity_reason\":\""
        << output.blocked_info.pass_start_target_continuity_reason << "\","
        << "\"pass_start_tracking_diagnostic_evaluated\":"
        << (output.blocked_info.pass_start_tracking_diagnostic.evaluated
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_diagnostic_candidate_type\":\""
        << toString(output.blocked_info.pass_start_tracking_diagnostic
                        .candidate_type)
        << "\","
        << "\"pass_start_tracking_actual_pose_start\":"
        << (output.blocked_info.pass_start_tracking_diagnostic.actual_pose_start
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_endpoint_arc_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .endpoint_arc_m)
        << ","
        << "\"pass_start_tracking_required_arc_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .required_arc_m)
        << ","
        << "\"pass_start_tracking_transition_deadline_presence\":\""
        << (output.blocked_info.pass_start_tracking_diagnostic
                    .transition_deadline_present
                ? "EVALUATED"
                : "NOT_EVALUATED")
        << "\","
        << "\"pass_start_tracking_transition_start_s_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .transition_deadline.transition_start_s_m)
        << ","
        << "\"pass_start_tracking_transition_end_s_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .transition_deadline.transition_end_s_m)
        << ","
        << "\"pass_start_tracking_transition_required_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .transition_deadline.required_transition_m)
        << ","
        << "\"pass_start_tracking_transition_available_m\":"
        << jsonNumber(output.blocked_info.pass_start_tracking_diagnostic
                          .transition_deadline.available_deadline_m)
        << ","
        << "\"pass_start_tracking_desired_path_trackable\":"
        << (output.blocked_info.pass_start_tracking_diagnostic
                    .desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_pp_command_trackable\":"
        << (output.blocked_info.pass_start_tracking_diagnostic
                    .pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_controller_generation\":"
        << output.blocked_info.pass_start_tracking_diagnostic
               .controller_plan_generation
        << ","
        << "\"pass_start_tracking_expected_generation\":"
        << output.blocked_info.pass_start_tracking_diagnostic
               .controller_expected_generation
        << ","
        << "\"pass_start_tracking_controller_status_reason\":\""
        << output.blocked_info.pass_start_tracking_diagnostic
               .controller_status_reason
        << "\","
        << "\"pass_start_tracking_mpc_horizon_usable\":"
        << (output.blocked_info.pass_start_tracking_diagnostic
                    .controller_mpc_horizon_usable
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_continuity_usable\":"
        << (output.blocked_info.pass_start_tracking_diagnostic
                    .controller_continuity_usable
                ? "true"
                : "false")
        << ","
        << "\"pass_start_tracking_first_false\":\""
        << output.blocked_info.pass_start_tracking_diagnostic.first_false
        << "\","
        << "\"pass_left_safe_cycles\":"
        << output.blocked_info.pass_left_safe_cycles << ","
        << "\"pass_right_safe_cycles\":"
        << output.blocked_info.pass_right_safe_cycles << ","
        << "\"pass_safe_cycle_reset_reason\":\""
        << output.blocked_info.pass_safe_cycle_reset_reason << "\","
        << "\"follow_candidate_speed_cap_mps\":"
        << jsonNumber(output.blocked_info.follow_candidate_speed_cap_mps) << ","
        << "\"follow_candidate_terminal_speed_mps\":"
        << jsonNumber(output.blocked_info.follow_candidate_terminal_speed_mps)
        << ","
        << "\"maneuver_transaction_safe_lateral_hold_active\":"
        << (output.blocked_info.maneuver_transaction_safe_lateral_hold_active
                ? "true"
                : "false")
        << ","
        << "\"authorized_pass_current_d_hold_active\":"
        << (output.blocked_info.authorized_pass_current_d_hold_active ? "true"
                                                                      : "false")
        << ","
        << "\"maneuver_transaction_tracking_stop_active\":"
        << (output.blocked_info.maneuver_transaction_tracking_stop_active
                ? "true"
                : "false")
        << ","
        << "\"maneuver_transaction_tracking_release_pending\":"
        << (output.blocked_info.maneuver_transaction_tracking_release_pending
                ? "true"
                : "false")
        << ","
        << "\"maneuver_transaction_tracking_release_confirmed\":"
        << (output.blocked_info.maneuver_transaction_tracking_release_confirmed
                ? "true"
                : "false")
        << ","
        << "\"maneuver_transaction_tracking_release_cycles\":"
        << output.blocked_info.maneuver_transaction_tracking_release_cycles
        << ","
        << "\"tracking_release_pass_warmup\":"
        << (output.tracking_release_pass_warmup ? "true" : "false") << ","
        << "\"tracking_release_token\":" << output.tracking_release_token << ","
        << "\"maneuver_transaction_tracking_continuity_armed\":"
        << (output.blocked_info.maneuver_transaction_tracking_continuity_armed
                ? "true"
                : "false")
        << ","
        << "\"parallel_follow_vehicle_id\":\""
        << output.blocked_info.parallel_follow_id << "\","
        << "\"parallel_follow_delta_s\":"
        << jsonNumber(output.blocked_info.parallel_follow_delta_s) << ","
        << "\"parallel_follow_delta_d\":"
        << jsonNumber(output.blocked_info.parallel_follow_delta_d) << ","
        << "\"parallel_follow_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.parallel_follow_rel_v) << ","
        << "\"parallel_follow_s_dot_mps\":"
        << jsonNumber(output.blocked_info.parallel_follow_s_dot_mps) << ","
        << "\"parallel_follow_same_direction\":"
        << (output.blocked_info.parallel_follow_same_direction ? "true"
                                                               : "false")
        << ","
        << "\"parallel_follow_direction_known\":"
        << (output.blocked_info.parallel_follow_direction_known ? "true"
                                                                : "false")
        << ","
        << "\"leader_priority_active\":"
        << (output.blocked_info.leader_priority_active ? "true" : "false")
        << ","
        << "\"leader_priority_latched\":"
        << (output.blocked_info.leader_priority_latched ? "true" : "false")
        << ","
        << "\"leader_priority_id\":\"" << output.blocked_info.leader_priority_id
        << "\","
        << "\"leader_priority_delta_s\":"
        << jsonNumber(output.blocked_info.leader_priority_delta_s) << ","
        << "\"leader_priority_reason\":\""
        << output.blocked_info.leader_priority_reason << "\","
        << "\"ignored_opposite_direction_count\":"
        << output.blocked_info.ignored_opposite_direction_count << ","
        << "\"left_pass_gap_m\":"
        << jsonNumber(output.blocked_info.left_pass_gap_m) << ","
        << "\"right_pass_gap_m\":"
        << jsonNumber(output.blocked_info.right_pass_gap_m) << ","
        << "\"can_pass_left\":"
        << (output.blocked_info.can_pass_left ? "true" : "false") << ","
        << "\"can_pass_right\":"
        << (output.blocked_info.can_pass_right ? "true" : "false") << ","
        << "\"pass_left_candidate_generated\":"
        << (output.blocked_info.pass_left_candidate_generated ? "true"
                                                              : "false")
        << ","
        << "\"pass_right_candidate_generated\":"
        << (output.blocked_info.pass_right_candidate_generated ? "true"
                                                               : "false")
        << ","
        << "\"pass_left_candidate_feasible\":"
        << (output.blocked_info.pass_left_candidate_feasible ? "true" : "false")
        << ","
        << "\"pass_right_candidate_feasible\":"
        << (output.blocked_info.pass_right_candidate_feasible ? "true"
                                                              : "false")
        << ","
        << "\"pass_left_candidate_tracking_profile_valid\":"
        << (output.blocked_info.pass_left_candidate_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"pass_right_candidate_tracking_profile_valid\":"
        << (output.blocked_info.pass_right_candidate_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"pass_left_candidate_desired_path_trackable\":"
        << (output.blocked_info.pass_left_candidate_desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"pass_right_candidate_desired_path_trackable\":"
        << (output.blocked_info.pass_right_candidate_desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"pass_left_candidate_pure_pursuit_command_trackable\":"
        << (output.blocked_info
                    .pass_left_candidate_pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"pass_right_candidate_pure_pursuit_command_trackable\":"
        << (output.blocked_info
                    .pass_right_candidate_pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"committed_pass_snapshot_continuity_used\":"
        << (output.blocked_info.committed_pass_snapshot_continuity_used
                ? "true"
                : "false")
        << ","
        << "\"committed_pass_spatial_profile_continuity_used\":"
        << (output.blocked_info.committed_pass_spatial_profile_continuity_used
                ? "true"
                : "false")
        << ","
        << "\"committed_pass_spatial_profile_tracking_error_m\":"
        << jsonNumber(output.blocked_info
                          .committed_pass_spatial_profile_tracking_error_m)
        << ","
        << "\"committed_pass_spatial_profile_source_age_sec\":"
        << jsonNumber(output.blocked_info
                          .committed_pass_spatial_profile_source_age_sec)
        << ","
        << "\"pass_left_candidate_endpoint_arc_m\":"
        << jsonNumber(output.blocked_info.pass_left_candidate_endpoint_arc_m)
        << ","
        << "\"pass_right_candidate_endpoint_arc_m\":"
        << jsonNumber(output.blocked_info.pass_right_candidate_endpoint_arc_m)
        << ","
        << "\"pass_left_candidate_required_arc_m\":"
        << jsonNumber(output.blocked_info.pass_left_candidate_required_arc_m)
        << ","
        << "\"pass_right_candidate_required_arc_m\":"
        << jsonNumber(output.blocked_info.pass_right_candidate_required_arc_m)
        << ","
        << "\"pass_left_candidate_target_d_m\":"
        << jsonNumber(output.blocked_info.pass_left_candidate_target_d_m) << ","
        << "\"pass_right_candidate_target_d_m\":"
        << jsonNumber(output.blocked_info.pass_right_candidate_target_d_m)
        << ","
        << "\"pass_left_candidate_corridor_min_margin_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_corridor_min_margin_m)
        << ","
        << "\"pass_right_candidate_corridor_min_margin_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_corridor_min_margin_m)
        << ","
        << "\"pass_left_transition_deadline_evaluated\":"
        << (output.blocked_info.pass_left_candidate_transition_deadline
                    .evaluated
                ? "true"
                : "false")
        << ","
        << "\"pass_left_transition_deadline_input_valid\":"
        << (output.blocked_info.pass_left_candidate_transition_deadline
                    .input_valid
                ? "true"
                : "false")
        << ","
        << "\"pass_left_transition_deadline_reachable\":"
        << (output.blocked_info.pass_left_candidate_transition_deadline
                    .reachable
                ? "true"
                : "false")
        << ","
        << "\"pass_left_transition_deadline_source\":\""
        << passTransitionDeadlineSourceString(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .source)
        << "\","
        << "\"pass_left_transition_lateral_shift_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .lateral_shift_m)
        << ","
        << "\"pass_left_transition_start_s_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .transition_start_s_m)
        << ","
        << "\"pass_left_transition_end_s_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .transition_end_s_m)
        << ","
        << "\"pass_left_transition_required_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .required_transition_m)
        << ","
        << "\"pass_left_transition_available_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .available_deadline_m)
        << ","
        << "\"pass_left_transition_slack_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .deadline_slack_m)
        << ","
        << "\"pass_left_transition_tracking_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .evaluated_tracking_speed_mps)
        << ","
        << "\"pass_left_transition_deadline_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .deadline_speed_cap_mps)
        << ","
        << "\"pass_left_transition_proposal_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .proposal_speed_cap_mps)
        << ","
        << "\"pass_left_transition_proposal_horizon_sec\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .proposal_horizon_sec)
        << ","
        << "\"pass_left_transition_proposal_endpoint_arc_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .proposal_endpoint_arc_m)
        << ","
        << "\"pass_left_transition_proposal_end_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .proposal_end_speed_mps)
        << ","
        << "\"pass_left_transition_pp_required_arc_m\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .pp_required_arc_m)
        << ","
        << "\"pass_left_transition_time_to_required_sec\":"
        << jsonNumber(
               output.blocked_info.pass_left_candidate_transition_deadline
                   .proposal_time_to_required_transition_sec)
        << ","
        << "\"pass_right_transition_deadline_evaluated\":"
        << (output.blocked_info.pass_right_candidate_transition_deadline
                    .evaluated
                ? "true"
                : "false")
        << ","
        << "\"pass_right_transition_deadline_input_valid\":"
        << (output.blocked_info.pass_right_candidate_transition_deadline
                    .input_valid
                ? "true"
                : "false")
        << ","
        << "\"pass_right_transition_deadline_reachable\":"
        << (output.blocked_info.pass_right_candidate_transition_deadline
                    .reachable
                ? "true"
                : "false")
        << ","
        << "\"pass_right_transition_deadline_source\":\""
        << passTransitionDeadlineSourceString(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .source)
        << "\","
        << "\"pass_right_transition_lateral_shift_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .lateral_shift_m)
        << ","
        << "\"pass_right_transition_start_s_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .transition_start_s_m)
        << ","
        << "\"pass_right_transition_end_s_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .transition_end_s_m)
        << ","
        << "\"pass_right_transition_required_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .required_transition_m)
        << ","
        << "\"pass_right_transition_available_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .available_deadline_m)
        << ","
        << "\"pass_right_transition_slack_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .deadline_slack_m)
        << ","
        << "\"pass_right_transition_tracking_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .evaluated_tracking_speed_mps)
        << ","
        << "\"pass_right_transition_deadline_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .deadline_speed_cap_mps)
        << ","
        << "\"pass_right_transition_proposal_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .proposal_speed_cap_mps)
        << ","
        << "\"pass_right_transition_proposal_horizon_sec\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .proposal_horizon_sec)
        << ","
        << "\"pass_right_transition_proposal_endpoint_arc_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .proposal_endpoint_arc_m)
        << ","
        << "\"pass_right_transition_proposal_end_speed_mps\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .proposal_end_speed_mps)
        << ","
        << "\"pass_right_transition_pp_required_arc_m\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .pp_required_arc_m)
        << ","
        << "\"pass_right_transition_time_to_required_sec\":"
        << jsonNumber(
               output.blocked_info.pass_right_candidate_transition_deadline
                   .proposal_time_to_required_transition_sec)
        << ","
        << "\"pass_left_candidate_reject_reason\":\""
        << output.blocked_info.pass_left_candidate_reject_reason << "\","
        << "\"pass_right_candidate_reject_reason\":\""
        << output.blocked_info.pass_right_candidate_reject_reason << "\","
        << "\"pass_gap_required_m\":"
        << jsonNumber(output.blocked_info.pass_gap_required_m) << ","
        << "\"pass_decision_frozen\":"
        << (output.blocked_info.pass_decision_frozen ? "true" : "false") << ","
        << "\"pass_decision_freeze_reason\":\""
        << output.blocked_info.pass_decision_freeze_reason << "\","
        << "\"maneuver_transaction_retry_active\":"
        << (output.blocked_info.maneuver_transaction_retry_active ? "true"
                                                                  : "false")
        << ","
        << "\"pass_gap_reason\":\"" << output.blocked_info.pass_gap_reason
        << "\","
        << "\"ego_x\":" << jsonNumber(ego.x) << ","
        << "\"ego_y\":" << jsonNumber(ego.y) << ","
        << "\"ego_s\":" << jsonNumber(ego.frenet.s) << ","
        << "\"ego_lateral_offset\":" << jsonNumber(ego.frenet.d) << ","
        << "\"ego_wall_clearance_m\":"
        << jsonNumber(output.blocked_info.ego_wall_clearance_m) << ","
        << "\"early_wall_recovery_probe_requested\":"
        << (output.blocked_info.early_wall_recovery_probe_requested ? "true"
                                                                    : "false")
        << ","
        << "\"early_wall_recovery_probe_generated\":"
        << (output.blocked_info.early_wall_recovery_probe_generated ? "true"
                                                                    : "false")
        << ","
        << "\"early_wall_recovery_probe_feasible\":"
        << (output.blocked_info.early_wall_recovery_probe_feasible ? "true"
                                                                   : "false")
        << ","
        << "\"early_wall_recovery_clearance_threshold_m\":"
        << jsonNumber(
               output.blocked_info.early_wall_recovery_clearance_threshold_m)
        << ","
        << "\"early_wall_recovery_probe_first_false\":\""
        << output.blocked_info.early_wall_recovery_probe_first_false << "\","
        << "\"early_wall_recovery_reject_reason\":\""
        << output.blocked_info.early_wall_recovery_reject_reason << "\","
        << "\"ego_speed_mps\":" << jsonNumber(ego.v) << ","
        << "\"target_lateral_offset_m\":"
        << jsonNumber(output.target_lateral_offset_m) << ","
        << "\"min_cbf_h\":" << jsonNumber(output.min_cbf_h) << ","
        << "\"cbf_slack\":" << jsonNumber(output.cbf_slack) << ","
        << "\"active_cbf_constraint_count\":"
        << output.active_cbf_constraint_count << ","
        << "\"safe_stop_triggered\":"
        << (output.safe_stop_triggered ? "true" : "false") << ","
        << "\"start_grace_active\":"
        << (output.start_grace_active ? "true" : "false") << ","
        << "\"safe_stop_release_ready\":"
        << (output.safe_stop_release_ready ? "true" : "false") << ","
        << "\"safe_stop_reason\":\"" << output.safe_stop_reason << "\","
        << "\"safe_stop_reject_reason\":\"" << output.safe_stop_reject_reason
        << "\","
        << "\"safe_stop_v_mps\":" << jsonNumber(output.safe_stop_v_mps) << ","
        << "\"safe_stop_trigger_count\":" << output.safe_stop_trigger_count
        << ","
        << "\"safe_stop_hold_count\":" << output.safe_stop_hold_count << ","
        << "\"safe_stop_release_count\":" << output.safe_stop_release_count
        << ","
        << "\"speed_only_fallback_active\":"
        << (output.speed_only_fallback_active ? "true" : "false") << ","
        << "\"wall_risk_speed_guard_active\":"
        << (output.wall_risk_speed_guard_active ? "true" : "false") << ","
        << "\"mpc_health_speed_guard_active\":"
        << (output.mpc_health_speed_guard_active ? "true" : "false") << ","
        << "\"recovery_speed_guard_active\":"
        << (output.recovery_speed_guard_active ? "true" : "false") << ","
        << "\"lateral_target_hold_active\":"
        << (output.lateral_target_hold_active ? "true" : "false") << ","
        << "\"lateral_target_hold_reason\":\""
        << output.lateral_target_hold_reason << "\","
        << "\"published_lateral_safety_rejected\":"
        << (output.published_lateral_safety_rejected ? "true" : "false") << ","
        << "\"transition_previous_mode\":\""
        << toString(output.transition_previous_mode) << "\","
        << "\"state_machine_mode\":\"" << toString(output.state_machine_mode)
        << "\","
        << "\"post_reentry_arbitration_mode\":\""
        << toString(output.post_reentry_arbitration_mode) << "\","
        << "\"raw_selected\":\"" << toString(output.raw_selected) << "\","
        << "\"raw_selected_feasible\":"
        << (output.raw_selected_feasible ? "true" : "false") << ","
        << "\"raw_selected_reject_reason\":\""
        << output.raw_selected_reject_reason << "\","
        << "\"reentry_phase_before_update\":"
        << (output.reentry_phase_before_update ? "true" : "false") << ","
        << "\"reentry_lockout_before_update\":"
        << (output.reentry_lockout_before_update ? "true" : "false") << ","
        << "\"generic_recovery_before_update\":"
        << (output.generic_recovery_before_update ? "true" : "false") << ","
        << "\"reentry_phase_after_update\":"
        << (output.reentry_phase_after_update ? "true" : "false") << ","
        << "\"reentry_lockout_after_update\":"
        << (output.reentry_lockout_after_update ? "true" : "false") << ","
        << "\"generic_recovery_after_update\":"
        << (output.generic_recovery_after_update ? "true" : "false") << ","
        << "\"reentry_requested\":"
        << (output.reentry_gate.requested ? "true" : "false") << ","
        << "\"reentry_permitted\":"
        << (output.reentry_gate.permitted ? "true" : "false") << ","
        << "\"reentry_input_complete\":"
        << (output.reentry_gate.input_complete ? "true" : "false") << ","
        << "\"reentry_clear_cycles\":" << output.reentry_gate.clear_cycles
        << ","
        << "\"reentry_evaluated_opponent_count\":"
        << output.reentry_gate.evaluated_opponent_count << ","
        << "\"reentry_reason\":\"" << output.reentry_gate.reason << "\","
        << "\"reentry_hold_speed_cap_mps\":"
        << jsonNumber(output.blocked_info.reentry_hold_speed_cap_mps) << ","
        << "\"reentry_primary_blocker_id\":\""
        << output.reentry_gate.blocking_vehicle_id << "\","
        << "\"reentry_min_safety_margin\":"
        << jsonNumber(output.reentry_gate.min_safety_margin) << ","
        << "\"reentry_cbf_slack\":" << jsonNumber(output.reentry_gate.cbf_slack)
        << ","
        << "\"reentry_blocking_time_sec\":"
        << jsonNumber(output.reentry_gate.blocking_time_sec) << ","
        << "\"lateral_profile_mode\":\"" << output.lateral_profile_mode << "\","
        << "\"maneuver_latch_active\":"
        << (output.maneuver_latch_active ? "true" : "false") << ","
        << "\"maneuver_latch_target_id\":\"" << output.maneuver_latch_target_id
        << "\","
        << "\"maneuver_target_latched\":"
        << (output.blocked_info.maneuver_target_latched ? "true" : "false")
        << ","
        << "\"maneuver_target_id\":\"" << output.blocked_info.maneuver_target_id
        << "\","
        << "\"maneuver_target_observed\":"
        << (output.blocked_info.maneuver_target_observed ? "true" : "false")
        << ","
        << "\"maneuver_target_fresh\":"
        << (output.blocked_info.maneuver_target_fresh ? "true" : "false") << ","
        << "\"maneuver_target_age_sec\":"
        << jsonNumber(output.blocked_info.maneuver_target_age_sec) << ","
        << "\"maneuver_target_relative_s_m\":"
        << jsonNumber(output.blocked_info.maneuver_target_relative_s_m) << ","
        << "\"maneuver_target_relative_d_m\":"
        << jsonNumber(output.blocked_info.maneuver_target_relative_d_m) << ","
        << "\"maneuver_target_relative_speed_mps\":"
        << jsonNumber(output.blocked_info.maneuver_target_relative_speed_mps)
        << ","
        << "\"maneuver_unstarted_target_released\":"
        << (output.blocked_info.maneuver_unstarted_target_released ? "true"
                                                                   : "false")
        << ","
        << "\"maneuver_unstarted_target_reacquire_suppressed\":"
        << (output.blocked_info.maneuver_unstarted_target_reacquire_suppressed
                ? "true"
                : "false")
        << ","
        << "\"maneuver_unstarted_target_pulling_away_cycles\":"
        << output.blocked_info.maneuver_unstarted_target_pulling_away_cycles
        << ","
        << "\"maneuver_pass_lateral_progress_m\":"
        << jsonNumber(output.blocked_info.maneuver_pass_lateral_progress_m)
        << ","
        << "\"maneuver_target_pass_geometric_complete\":"
        << (output.blocked_info.maneuver_target_pass_geometric_complete
                ? "true"
                : "false")
        << ","
        << "\"maneuver_target_pass_safety_approved\":"
        << (output.blocked_info.maneuver_target_pass_safety_approved ? "true"
                                                                     : "false")
        << ","
        << "\"maneuver_target_pass_complete\":"
        << (output.blocked_info.maneuver_target_pass_complete ? "true"
                                                              : "false")
        << ","
        << "\"maneuver_target_pass_candidate_feasible\":"
        << (output.blocked_info.maneuver_target_pass_candidate_feasible
                ? "true"
                : "false")
        << ","
        << "\"maneuver_target_pass_min_safety_margin\":"
        << jsonNumber(
               output.blocked_info.maneuver_target_pass_min_safety_margin)
        << ","
        << "\"maneuver_target_pass_reject_reason\":\""
        << output.blocked_info.maneuver_target_pass_reject_reason << "\","
        << "\"maneuver_target_previous_id\":\""
        << output.blocked_info.maneuver_target_previous_id << "\","
        << "\"maneuver_target_new_id\":\""
        << output.blocked_info.maneuver_target_new_id << "\","
        << "\"maneuver_target_change_reason\":\""
        << output.blocked_info.maneuver_target_change_reason << "\","
        << "\"maneuver_chain_tail_id\":\""
        << output.blocked_info.maneuver_chain_tail_id << "\","
        << "\"maneuver_chain_tail_index\":"
        << output.blocked_info.maneuver_chain_tail_index << ","
        << "\"maneuver_chain_target_count\":"
        << output.blocked_info.maneuver_chain_target_count << ","
        << "\"maneuver_chain_tail_observed\":"
        << (output.blocked_info.maneuver_chain_tail_observed ? "true" : "false")
        << ","
        << "\"maneuver_chain_tail_relative_s_m\":"
        << jsonNumber(output.blocked_info.maneuver_chain_tail_relative_s_m)
        << ","
        << "\"maneuver_chain_tail_relative_speed_mps\":"
        << jsonNumber(
               output.blocked_info.maneuver_chain_tail_relative_speed_mps)
        << ","
        << "\"maneuver_latch_target_s\":"
        << jsonNumber(output.maneuver_latch_target_s_m) << ","
        << "\"maneuver_latch_avoid_start_s\":"
        << jsonNumber(output.maneuver_latch_avoid_start_s_m) << ","
        << "\"maneuver_latch_full_offset_start_s\":"
        << jsonNumber(output.maneuver_latch_full_offset_start_s_m) << ","
        << "\"maneuver_latch_full_offset_end_s\":"
        << jsonNumber(output.maneuver_latch_full_offset_end_s_m) << ","
        << "\"maneuver_latch_merge_end_s\":"
        << jsonNumber(output.maneuver_latch_merge_end_s_m) << ","
        << "\"maneuver_latch_waypoint_count\":"
        << output.maneuver_latch_waypoint_count << ","
        << "\"maneuver_latch_last_waypoint_id\":\""
        << output.maneuver_latch_last_waypoint_id << "\","
        << "\"maneuver_latch_last_waypoint_s_m\":"
        << jsonNumber(output.maneuver_latch_last_waypoint_s_m) << ","
        << "\"maneuver_latch_last_waypoint_d_m\":"
        << jsonNumber(output.maneuver_latch_last_waypoint_d_m) << ","
        << "\"speed_cap_reason\":\"" << output.speed_cap_reason << "\","
        << "\"applied_speed_cap_mps\":"
        << jsonNumber(output.applied_speed_cap_mps) << ","
        << "\"wall_soft_margin_m\":" << jsonNumber(output.wall_soft_margin_m)
        << ","
        << "\"active_section_name\":\"" << output.active_section.name << "\","
        << "\"active_section_profile\":\"" << output.active_section.profile
        << "\","
        << "\"active_section_role_policy\":\""
        << output.active_section.role_policy << "\","
        << "\"mpc_health_valid\":"
        << (output.mpc_health.valid ? "true" : "false") << ","
        << "\"mpc_infeasible_count\":" << output.mpc_health.infeasible_count
        << ","
        << "\"mpc_solve_time_ms\":"
        << jsonNumber(output.mpc_health.solve_time_ms) << ","
        << "\"mpc_health_age_sec\":" << jsonNumber(output.mpc_health.age_sec)
        << ","
        << "\"closest_vehicle_id\":\"" << output.blocked_info.nearest_id
        << "\","
        << "\"closest_vehicle_distance_m\":"
        << jsonNumber(
               !output.blocked_info.nearest_id.empty() &&
                       std::isfinite(output.blocked_info.front_delta_s) &&
                       std::isfinite(output.blocked_info.front_delta_d)
                   ? std::hypot(output.blocked_info.front_delta_s,
                                output.blocked_info.front_delta_d)
                   : std::numeric_limits<double>::quiet_NaN())
        << ","
        << "\"active_override\":" << (output.active_override ? "true" : "false")
        << ","
        << "\"selected_lateral_profile_safety_verified\":"
        << (output.selected_lateral_profile_safety_verified ? "true" : "false")
        << ","
        << "\"lateral_stop_inputs_complete\":"
        << (output.lateral_stop_inputs_complete ? "true" : "false") << ","
        << "\"lateral_tracking_authorized_during_stop\":"
        << (output.lateral_tracking_authorized_during_stop ? "true" : "false")
        << ","
        << "\"solver_horizon_intent\":"
        << static_cast<int>(output.solver_horizon_intent) << ","
        << "\"abort_reason\":\""
        << (output.mode == BehaviorMode::ABORT_RECOVERY ? output.reason : "")
        << "\","
        << "\"v2_phase\":" << static_cast<int>(output.supervisor_v2.phase)
        << ","
        << "\"v2_decision_reason\":\"" << output.supervisor_v2.reason << "\","
        << "\"v2_selected\":\"" << toString(output.supervisor_v2.selected)
        << "\","
        << "\"v2_decision_trajectory_authorized\":"
        << (output.supervisor_v2.trajectory_authorized ? "true" : "false")
        << ","
        << "\"v2_effective_trajectory_authorized\":"
        << (last_supervisor_v2_effective_trajectory_authorized_ ? "true"
                                                                : "false")
        << ","
        << "\"v2_trajectory_safety_evaluated\":"
        << (output.supervisor_v2.trajectory.safety_evaluated ? "true" : "false")
        << ","
        << "\"v2_trajectory_publishable\":"
        << (last_supervisor_v2_trajectory_publishable_ ? "true" : "false")
        << ","
        << "\"v2_safety_inputs_complete\":"
        << (output.supervisor_v2.safety_inputs_complete ? "true" : "false")
        << ","
        << "\"v2_tracking_usable\":"
        << (output.supervisor_v2.tracking_usable ? "true" : "false") << ","
        << "\"v2_authorization_failure_mask\":"
        << last_supervisor_v2_authorization_failure_mask_ << ","
        << "\"v2_authorization_failure_reasons\":\""
        << joinStrings(last_supervisor_v2_authorization_failure_reasons_, "|")
        << "\","
        << "\"v2_candidate_reject_reason\":\""
        << output.supervisor_v2.candidate_reject_reason << "\","
        << "\"v2_follow_candidate_generated\":"
        << (output.supervisor_v2.follow_candidate.generated ? "true" : "false")
        << ","
        << "\"v2_follow_candidate_feasible\":"
        << (output.supervisor_v2.follow_candidate.feasible ? "true" : "false")
        << ","
        << "\"v2_follow_candidate_tracking_profile_valid\":"
        << (output.supervisor_v2.follow_candidate
                    .controller_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"v2_follow_candidate_desired_path_trackable\":"
        << (output.supervisor_v2.follow_candidate.desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_follow_candidate_pure_pursuit_command_trackable\":"
        << (output.supervisor_v2.follow_candidate.pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_follow_candidate_moving_target_reachable\":"
        << (output.supervisor_v2.follow_candidate
                    .moving_target_relatively_reachable
                ? "true"
                : "false")
        << ","
        << "\"v2_follow_candidate_reject_reason\":\""
        << output.supervisor_v2.follow_candidate.reject_reason << "\","
        << "\"v2_follow_candidate_endpoint_arc_m\":"
        << jsonNumber(output.supervisor_v2.follow_candidate.endpoint_arc_m)
        << ","
        << "\"v2_follow_candidate_required_arc_m\":"
        << jsonNumber(output.supervisor_v2.follow_candidate.required_arc_m)
        << ","
        << "\"v2_follow_candidate_target_d_m\":"
        << jsonNumber(output.supervisor_v2.follow_candidate.planned_target_d_m)
        << ","
        << "\"v2_follow_candidate_min_safety_margin\":"
        << jsonNumber(output.supervisor_v2.follow_candidate.min_safety_margin)
        << ","
        << "\"v2_follow_candidate_blocking_opponent_id\":\""
        << output.supervisor_v2.follow_candidate.blocking_opponent_id << "\","
        << "\"v2_follow_candidate_blocking_time_sec\":"
        << jsonNumber(output.supervisor_v2.follow_candidate.blocking_time_sec)
        << ","
        << "\"v2_pass_left_candidate_generated\":"
        << (output.supervisor_v2.pass_left_candidate.generated ? "true"
                                                               : "false")
        << ","
        << "\"v2_pass_left_candidate_feasible\":"
        << (output.supervisor_v2.pass_left_candidate.feasible ? "true"
                                                              : "false")
        << ","
        << "\"v2_pass_left_candidate_tracking_profile_valid\":"
        << (output.supervisor_v2.pass_left_candidate
                    .controller_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_left_candidate_desired_path_trackable\":"
        << (output.supervisor_v2.pass_left_candidate.desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_left_candidate_pure_pursuit_command_trackable\":"
        << (output.supervisor_v2.pass_left_candidate
                    .pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_left_candidate_moving_target_reachable\":"
        << (output.supervisor_v2.pass_left_candidate
                    .moving_target_relatively_reachable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_left_candidate_reject_reason\":\""
        << output.supervisor_v2.pass_left_candidate.reject_reason << "\","
        << "\"v2_pass_left_candidate_endpoint_arc_m\":"
        << jsonNumber(output.supervisor_v2.pass_left_candidate.endpoint_arc_m)
        << ","
        << "\"v2_pass_left_candidate_required_arc_m\":"
        << jsonNumber(output.supervisor_v2.pass_left_candidate.required_arc_m)
        << ","
        << "\"v2_pass_left_candidate_target_d_m\":"
        << jsonNumber(
               output.supervisor_v2.pass_left_candidate.planned_target_d_m)
        << ","
        << "\"v2_pass_left_candidate_min_safety_margin\":"
        << jsonNumber(
               output.supervisor_v2.pass_left_candidate.min_safety_margin)
        << ","
        << "\"v2_pass_left_candidate_blocking_opponent_id\":\""
        << output.supervisor_v2.pass_left_candidate.blocking_opponent_id
        << "\","
        << "\"v2_pass_left_candidate_blocking_time_sec\":"
        << jsonNumber(
               output.supervisor_v2.pass_left_candidate.blocking_time_sec)
        << ","
        << "\"v2_pass_right_candidate_generated\":"
        << (output.supervisor_v2.pass_right_candidate.generated ? "true"
                                                                : "false")
        << ","
        << "\"v2_pass_right_candidate_feasible\":"
        << (output.supervisor_v2.pass_right_candidate.feasible ? "true"
                                                               : "false")
        << ","
        << "\"v2_pass_right_candidate_tracking_profile_valid\":"
        << (output.supervisor_v2.pass_right_candidate
                    .controller_tracking_profile_valid
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_right_candidate_desired_path_trackable\":"
        << (output.supervisor_v2.pass_right_candidate.desired_path_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_right_candidate_pure_pursuit_command_trackable\":"
        << (output.supervisor_v2.pass_right_candidate
                    .pure_pursuit_command_trackable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_right_candidate_moving_target_reachable\":"
        << (output.supervisor_v2.pass_right_candidate
                    .moving_target_relatively_reachable
                ? "true"
                : "false")
        << ","
        << "\"v2_pass_right_candidate_reject_reason\":\""
        << output.supervisor_v2.pass_right_candidate.reject_reason << "\","
        << "\"v2_pass_right_candidate_endpoint_arc_m\":"
        << jsonNumber(output.supervisor_v2.pass_right_candidate.endpoint_arc_m)
        << ","
        << "\"v2_pass_right_candidate_required_arc_m\":"
        << jsonNumber(output.supervisor_v2.pass_right_candidate.required_arc_m)
        << ","
        << "\"v2_pass_right_candidate_target_d_m\":"
        << jsonNumber(
               output.supervisor_v2.pass_right_candidate.planned_target_d_m)
        << ","
        << "\"v2_pass_right_candidate_min_safety_margin\":"
        << jsonNumber(
               output.supervisor_v2.pass_right_candidate.min_safety_margin)
        << ","
        << "\"v2_pass_right_candidate_blocking_opponent_id\":\""
        << output.supervisor_v2.pass_right_candidate.blocking_opponent_id
        << "\","
        << "\"v2_pass_right_candidate_blocking_time_sec\":"
        << jsonNumber(
               output.supervisor_v2.pass_right_candidate.blocking_time_sec)
        << ","
        << "\"v2_constraint_prefilter_reason\":\""
        << last_supervisor_v2_prefilter_constraint_reason_ << "\","
        << "\"v2_constraint_filtered_reason\":\""
        << last_supervisor_v2_filtered_constraint_reason_ << "\","
        << "\"controller_tracking_status_received\":"
        << (last_controller_tracking_status_sec_.has_value() ? "true" : "false")
        << ","
        << "\"controller_tracking_status_age_sec\":"
        << jsonNumber(controller_tracking_age_sec) << ","
        << "\"controller_tracking_status_header_stamp_sec\":"
        << jsonNumber(last_controller_tracking_status_sec_.value_or(
               std::numeric_limits<double>::quiet_NaN()))
        << ","
        << "\"controller_tracking_status_reason\":\""
        << controller_tracking_status_reason << "\","
        << "\"controller_tracking_status_message_reason\":\""
        << controller_tracking_reason_ << "\","
        << "\"controller_tracking_snapshot_classification\":\""
        << controller_tracking_snapshot_classification << "\","
        << "\"controller_tracking_source_reason\":\""
        << controller_tracking_reason_ << "\","
        << "\"controller_tracking_pp_command_fresh\":"
        << (controller_tracking_pp_command_fresh_ ? "true" : "false") << ","
        << "\"controller_tracking_usable\":"
        << (controller_tracking_usable_ ? "true" : "false") << ","
        << "\"controller_tracking_safety_constraint_release_ready\":"
        << (controller_tracking_safety_constraint_release_ready_ ? "true"
                                                                 : "false")
        << ","
        << "\"controller_tracking_attack_follow_stop_transport_release_ready\":"
        << (controller_tracking_attack_follow_stop_transport_release_ready_
                ? "true"
                : "false")
        << ","
        << "\"controller_tracking_mpc_horizon_usable\":"
        << (controller_tracking_mpc_horizon_usable_ ? "true" : "false") << ","
        << "\"verified_non_mpc_pure_pursuit\":"
        << (verifiedNonMpcPurePursuit(debug_now_sec) ? "true" : "false") << ","
        << "\"controller_tracking_continuity_usable\":"
        << (purePursuitTrackingContinuityUsable(debug_now_sec) ? "true"
                                                               : "false")
        << ","
        << "\"controller_tracking_continuity_target_id\":\""
        << (controller_tracking_pass_identity != nullptr
                ? controller_tracking_pass_identity->target_id
                : std::string{})
        << "\","
        << "\"controller_tracking_continuity_pass_type\":\""
        << (controller_tracking_pass_identity != nullptr
                ? toString(controller_tracking_pass_identity->pass_type)
                : toString(CandidateType::FASTEST))
        << "\","
        << "\"controller_tracking_plan_generation\":"
        << controller_tracking_plan_generation_ << ","
        << "\"controller_tracking_expected_generation\":"
        << override_generation_ << ","
        << "\"controller_tracking_snapshot_override_generation\":"
        << override_generation_ << ","
        << "\"controller_tracking_snapshot_attempt_id\":" << attempt_id << ","
        << "\"controller_tracking_snapshot_release_token\":"
        << output.tracking_release_token << ","
        << "\"decision_freeze_lateral_error_m\":"
        << jsonNumber(output.blocked_info.decision_freeze_lateral_error_m)
        << ","
        << "\"pass_reauthorization_lockout_active\":"
        << (output.blocked_info.pass_reauthorization_lockout_active ? "true"
                                                                    : "false")
        << ","
        << "\"pass_authorized_envelope_min_d_m\":"
        << jsonNumber(output.blocked_info.pass_authorized_envelope_min_d_m)
        << ","
        << "\"pass_authorized_envelope_max_d_m\":"
        << jsonNumber(output.blocked_info.pass_authorized_envelope_max_d_m)
        << ","
        << "\"pass_authorized_envelope_error_m\":"
        << jsonNumber(output.blocked_info.pass_authorized_envelope_error_m)
        << ","
        << "\"pass_reauthorization_recovery_target_d_m\":"
        << jsonNumber(
               output.blocked_info.pass_reauthorization_recovery_target_d_m)
        << ","
        << "\"pass_reauthorization_clear_cycles\":"
        << output.blocked_info.pass_reauthorization_clear_cycles << ","
        << "\"recovery_tracking_error_m\":"
        << jsonNumber(output.recovery_tracking_error_m) << ","
        << "\"recovery_tracking_target_d_m\":"
        << jsonNumber(output.recovery_tracking_target_d_m) << ","
        << "\"reason\":\"" << output.reason << "\""
        << "}";
    metrics_msg.data = oss.str();
    metrics_pub_->publish(metrics_msg);
  }

  // 入力: planner coreが返したPlannerOutput。
  // 出力: ログ変化検出用に主要フィールドだけを抜き出したDecisionLogSnapshot。
  // 処理概要:
  // 巨大な出力全体ではなく、判断の変化に効く値だけを比較できる形に詰め替える。
  DecisionLogSnapshot
  makeDecisionLogSnapshot(const PlannerOutput &output) const {
    DecisionLogSnapshot snapshot;
    snapshot.mode = output.mode;
    snapshot.selected = output.selected;
    snapshot.blocked = output.blocked_info.blocked;
    snapshot.side_by_side = output.blocked_info.side_by_side;
    snapshot.corner_side_by_side = output.blocked_info.corner_side_by_side;
    snapshot.parallel_side_candidate =
        output.blocked_info.parallel_side_candidate;
    snapshot.parallel_follow_candidate =
        output.blocked_info.parallel_follow_candidate;
    snapshot.parallel_follow_feasible =
        output.blocked_info.parallel_follow_feasible;
    snapshot.future_side_by_side = output.blocked_info.future_side_by_side;
    snapshot.future_corner_side_by_side =
        output.blocked_info.future_corner_side_by_side;
    snapshot.future_yield_required = output.blocked_info.future_yield_required;
    snapshot.future_outer_wall_risk =
        output.blocked_info.future_outer_wall_risk;
    snapshot.can_pass_left = output.blocked_info.can_pass_left;
    snapshot.can_pass_right = output.blocked_info.can_pass_right;
    snapshot.pass_left_candidate_generated =
        output.blocked_info.pass_left_candidate_generated;
    snapshot.pass_left_candidate_feasible =
        output.blocked_info.pass_left_candidate_feasible;
    snapshot.pass_left_candidate_reject_reason =
        output.blocked_info.pass_left_candidate_reject_reason;
    snapshot.pass_right_candidate_generated =
        output.blocked_info.pass_right_candidate_generated;
    snapshot.pass_right_candidate_feasible =
        output.blocked_info.pass_right_candidate_feasible;
    snapshot.pass_right_candidate_reject_reason =
        output.blocked_info.pass_right_candidate_reject_reason;
    snapshot.maneuver_transaction_incomplete =
        output.blocked_info.maneuver_transaction_incomplete;
    snapshot.maneuver_transaction_pass_type =
        output.blocked_info.maneuver_transaction_pass_type;
    snapshot.maneuver_transaction_safe_lateral_hold_active =
        output.blocked_info.maneuver_transaction_safe_lateral_hold_active;
    snapshot.maneuver_target_id = output.blocked_info.maneuver_target_id;
    snapshot.maneuver_target_relative_s_m =
        output.blocked_info.maneuver_target_relative_s_m;
    snapshot.maneuver_chain_tail_id =
        output.blocked_info.maneuver_chain_tail_id;
    snapshot.maneuver_chain_tail_relative_s_m =
        output.blocked_info.maneuver_chain_tail_relative_s_m;
    snapshot.active_override = output.active_override;
    snapshot.corner_abs_curvature = output.blocked_info.corner_abs_curvature;
    snapshot.straight_overtake_start_allowed =
        output.blocked_info.straight_overtake_start_allowed;
    snapshot.overtake_start_abs_curvature =
        output.blocked_info.overtake_start_abs_curvature;
    snapshot.overtake_start_gate_reason =
        output.blocked_info.overtake_start_gate_reason;
    snapshot.gentle_curve_fresh_dynamic_gap_target =
        output.blocked_info.gentle_curve_fresh_dynamic_gap_target;
    snapshot.gentle_curve_safe_pass_start_approved =
        output.blocked_info.gentle_curve_safe_pass_start_approved;
    snapshot.gentle_curve_safe_pass_block_reason =
        output.blocked_info.gentle_curve_safe_pass_block_reason;
    snapshot.pass_left_safe_cycles = output.blocked_info.pass_left_safe_cycles;
    snapshot.pass_right_safe_cycles =
        output.blocked_info.pass_right_safe_cycles;
    snapshot.pass_safe_cycle_reset_reason =
        output.blocked_info.pass_safe_cycle_reset_reason;
    snapshot.follow_candidate_speed_cap_mps =
        output.blocked_info.follow_candidate_speed_cap_mps;
    snapshot.overtake_permission_allowed =
        output.blocked_info.overtake_permission_allowed;
    snapshot.overtake_permission_section_name =
        output.blocked_info.overtake_permission_section_name;
    snapshot.overtake_permission_reason =
        output.blocked_info.overtake_permission_reason;
    snapshot.front_vehicle_low_speed =
        output.blocked_info.front_vehicle_low_speed;
    snapshot.slow_front_exception_active =
        output.blocked_info.slow_front_exception_active;
    snapshot.slow_front_exception_count =
        output.blocked_info.slow_front_exception_count;
    snapshot.slow_obstacle_chain_active =
        output.blocked_info.slow_obstacle_chain_active;
    snapshot.slow_obstacle_chain_id =
        output.blocked_info.slow_obstacle_chain_id;
    snapshot.front_vehicle_speed_mps =
        output.blocked_info.front_vehicle_speed_mps;
    snapshot.pass_decision_frozen = output.blocked_info.pass_decision_frozen;
    snapshot.pass_decision_freeze_reason =
        output.blocked_info.pass_decision_freeze_reason;
    snapshot.future_wall_clearance_m =
        output.blocked_info.future_wall_clearance_m;
    snapshot.ego_wall_clearance_m = output.blocked_info.ego_wall_clearance_m;
    snapshot.early_wall_recovery_probe_requested =
        output.blocked_info.early_wall_recovery_probe_requested;
    snapshot.early_wall_recovery_probe_generated =
        output.blocked_info.early_wall_recovery_probe_generated;
    snapshot.early_wall_recovery_probe_feasible =
        output.blocked_info.early_wall_recovery_probe_feasible;
    snapshot.early_wall_recovery_probe_first_false =
        output.blocked_info.early_wall_recovery_probe_first_false;
    snapshot.early_wall_recovery_reject_reason =
        output.blocked_info.early_wall_recovery_reject_reason;
    snapshot.front_vehicle_id = output.blocked_info.nearest_id;
    snapshot.side_vehicle_id = output.blocked_info.side_id;
    snapshot.parallel_side_vehicle_id = output.blocked_info.parallel_side_id;
    snapshot.parallel_follow_vehicle_id =
        output.blocked_info.parallel_follow_id;
    snapshot.leader_priority_active =
        output.blocked_info.leader_priority_active;
    snapshot.leader_priority_latched =
        output.blocked_info.leader_priority_latched;
    snapshot.leader_priority_id = output.blocked_info.leader_priority_id;
    snapshot.leader_priority_delta_s =
        output.blocked_info.leader_priority_delta_s;
    snapshot.leader_priority_reason =
        output.blocked_info.leader_priority_reason;
    snapshot.pass_gap_reason = output.blocked_info.pass_gap_reason;
    snapshot.yield_reason = output.blocked_info.yield_reason;
    snapshot.reason = output.reason;
    snapshot.safe_stop_triggered = output.safe_stop_triggered;
    snapshot.start_grace_active = output.start_grace_active;
    snapshot.safe_stop_release_ready = output.safe_stop_release_ready;
    snapshot.safe_stop_reason = output.safe_stop_reason;
    snapshot.safe_stop_reject_reason = output.safe_stop_reject_reason;
    snapshot.safe_stop_trigger_count = output.safe_stop_trigger_count;
    snapshot.safe_stop_hold_count = output.safe_stop_hold_count;
    snapshot.safe_stop_release_count = output.safe_stop_release_count;
    snapshot.speed_only_fallback_active = output.speed_only_fallback_active;
    snapshot.wall_risk_speed_guard_active = output.wall_risk_speed_guard_active;
    snapshot.mpc_health_speed_guard_active =
        output.mpc_health_speed_guard_active;
    snapshot.recovery_speed_guard_active = output.recovery_speed_guard_active;
    snapshot.reentry_requested = output.reentry_gate.requested;
    snapshot.reentry_permitted = output.reentry_gate.permitted;
    snapshot.reentry_clear_cycles = output.reentry_gate.clear_cycles;
    snapshot.reentry_reason = output.reentry_gate.reason;
    snapshot.reentry_primary_blocker_id =
        output.reentry_gate.blocking_vehicle_id;
    snapshot.lateral_target_hold_active = output.lateral_target_hold_active;
    snapshot.lateral_target_hold_reason = output.lateral_target_hold_reason;
    snapshot.speed_cap_reason = output.speed_cap_reason;
    snapshot.active_section_name = output.active_section.name;
    snapshot.active_section_profile = output.active_section.profile;
    snapshot.active_section_role_policy = output.active_section.role_policy;
    snapshot.applied_speed_cap_mps = output.applied_speed_cap_mps;
    snapshot.wall_soft_margin_m = output.wall_soft_margin_m;
    snapshot.mpc_health_valid = output.mpc_health.valid;
    snapshot.mpc_infeasible_count = output.mpc_health.infeasible_count;
    snapshot.mpc_solve_time_ms = output.mpc_health.solve_time_ms;
    return snapshot;
  }

  // 入力: 1周期分のDecisionLogSnapshot。
  // 出力: autoware.logへ残す価値がある判断イベントならtrue。
  // 処理概要: FREE_RUN/FASTESTだけの平常周期を抑制し、閉塞/横並び/safe
  // stopなどの文脈を抽出する。
  bool isInterestingDecisionEvent(const DecisionLogSnapshot &snapshot) const {
    const bool has_pass_gap_context = !snapshot.pass_gap_reason.empty() &&
                                      snapshot.pass_gap_reason != "no_target";
    return snapshot.mode != BehaviorMode::FREE_RUN ||
           snapshot.selected != CandidateType::FASTEST || snapshot.blocked ||
           snapshot.side_by_side || snapshot.corner_side_by_side ||
           snapshot.parallel_side_candidate ||
           snapshot.parallel_follow_candidate || snapshot.future_side_by_side ||
           snapshot.future_corner_side_by_side ||
           snapshot.future_yield_required || snapshot.active_override ||
           snapshot.safe_stop_triggered || snapshot.start_grace_active ||
           snapshot.speed_only_fallback_active ||
           snapshot.wall_risk_speed_guard_active ||
           snapshot.mpc_health_speed_guard_active ||
           snapshot.recovery_speed_guard_active || snapshot.reentry_requested ||
           snapshot.lateral_target_hold_active ||
           !snapshot.straight_overtake_start_allowed ||
           !snapshot.overtake_permission_allowed ||
           snapshot.slow_front_exception_active ||
           snapshot.slow_obstacle_chain_active ||
           snapshot.pass_decision_frozen ||
           !snapshot.active_section_name.empty() ||
           !snapshot.front_vehicle_id.empty() ||
           !snapshot.side_vehicle_id.empty() ||
           !snapshot.parallel_side_vehicle_id.empty() ||
           !snapshot.parallel_follow_vehicle_id.empty() ||
           snapshot.leader_priority_active || has_pass_gap_context ||
           !snapshot.reason.empty();
  }

  // 入力: 現在周期のDecisionLogSnapshot。
  // 出力: 前回との差分としてログ出力すべきならtrue。
  // 処理概要:
  // 数値は小さな揺れを無視し、状態・理由・対象車両が変わった時だけ記録する。
  bool shouldLogDecisionEvent(const DecisionLogSnapshot &current) const {
    if (!has_decision_log_snapshot_) {
      return isInterestingDecisionEvent(current);
    }

    const auto &previous = last_decision_log_snapshot_;
    const bool changed =
        current.mode != previous.mode ||
        current.selected != previous.selected ||
        current.blocked != previous.blocked ||
        current.side_by_side != previous.side_by_side ||
        current.corner_side_by_side != previous.corner_side_by_side ||
        current.parallel_side_candidate != previous.parallel_side_candidate ||
        current.parallel_follow_candidate !=
            previous.parallel_follow_candidate ||
        current.parallel_follow_feasible != previous.parallel_follow_feasible ||
        current.future_side_by_side != previous.future_side_by_side ||
        current.future_corner_side_by_side !=
            previous.future_corner_side_by_side ||
        current.future_yield_required != previous.future_yield_required ||
        current.future_outer_wall_risk != previous.future_outer_wall_risk ||
        current.can_pass_left != previous.can_pass_left ||
        current.can_pass_right != previous.can_pass_right ||
        current.pass_left_candidate_generated !=
            previous.pass_left_candidate_generated ||
        current.pass_left_candidate_feasible !=
            previous.pass_left_candidate_feasible ||
        current.pass_left_candidate_reject_reason !=
            previous.pass_left_candidate_reject_reason ||
        current.pass_right_candidate_generated !=
            previous.pass_right_candidate_generated ||
        current.pass_right_candidate_feasible !=
            previous.pass_right_candidate_feasible ||
        current.pass_right_candidate_reject_reason !=
            previous.pass_right_candidate_reject_reason ||
        current.maneuver_transaction_incomplete !=
            previous.maneuver_transaction_incomplete ||
        current.maneuver_transaction_pass_type !=
            previous.maneuver_transaction_pass_type ||
        current.maneuver_transaction_safe_lateral_hold_active !=
            previous.maneuver_transaction_safe_lateral_hold_active ||
        current.maneuver_target_id != previous.maneuver_target_id ||
        std::abs(current.maneuver_target_relative_s_m -
                 previous.maneuver_target_relative_s_m) > 0.25 ||
        current.maneuver_chain_tail_id != previous.maneuver_chain_tail_id ||
        std::abs(current.maneuver_chain_tail_relative_s_m -
                 previous.maneuver_chain_tail_relative_s_m) > 0.25 ||
        current.active_override != previous.active_override ||
        std::abs(current.corner_abs_curvature - previous.corner_abs_curvature) >
            0.02 ||
        current.straight_overtake_start_allowed !=
            previous.straight_overtake_start_allowed ||
        std::abs(current.overtake_start_abs_curvature -
                 previous.overtake_start_abs_curvature) > 0.02 ||
        current.overtake_start_gate_reason !=
            previous.overtake_start_gate_reason ||
        current.gentle_curve_fresh_dynamic_gap_target !=
            previous.gentle_curve_fresh_dynamic_gap_target ||
        current.gentle_curve_safe_pass_start_approved !=
            previous.gentle_curve_safe_pass_start_approved ||
        current.gentle_curve_safe_pass_block_reason !=
            previous.gentle_curve_safe_pass_block_reason ||
        current.pass_left_safe_cycles != previous.pass_left_safe_cycles ||
        current.pass_right_safe_cycles != previous.pass_right_safe_cycles ||
        current.pass_safe_cycle_reset_reason !=
            previous.pass_safe_cycle_reset_reason ||
        std::abs(current.follow_candidate_speed_cap_mps -
                 previous.follow_candidate_speed_cap_mps) > 0.05 ||
        current.overtake_permission_allowed !=
            previous.overtake_permission_allowed ||
        current.overtake_permission_section_name !=
            previous.overtake_permission_section_name ||
        current.overtake_permission_reason !=
            previous.overtake_permission_reason ||
        current.front_vehicle_low_speed != previous.front_vehicle_low_speed ||
        current.slow_front_exception_active !=
            previous.slow_front_exception_active ||
        current.slow_front_exception_count !=
            previous.slow_front_exception_count ||
        current.slow_obstacle_chain_active !=
            previous.slow_obstacle_chain_active ||
        current.slow_obstacle_chain_id != previous.slow_obstacle_chain_id ||
        std::abs(current.front_vehicle_speed_mps -
                 previous.front_vehicle_speed_mps) > 0.05 ||
        current.pass_decision_frozen != previous.pass_decision_frozen ||
        current.pass_decision_freeze_reason !=
            previous.pass_decision_freeze_reason ||
        std::abs(current.future_wall_clearance_m -
                 previous.future_wall_clearance_m) > 0.05 ||
        std::abs(current.ego_wall_clearance_m - previous.ego_wall_clearance_m) >
            0.05 ||
        current.early_wall_recovery_probe_requested !=
            previous.early_wall_recovery_probe_requested ||
        current.early_wall_recovery_probe_generated !=
            previous.early_wall_recovery_probe_generated ||
        current.early_wall_recovery_probe_feasible !=
            previous.early_wall_recovery_probe_feasible ||
        current.early_wall_recovery_probe_first_false !=
            previous.early_wall_recovery_probe_first_false ||
        current.early_wall_recovery_reject_reason !=
            previous.early_wall_recovery_reject_reason ||
        current.front_vehicle_id != previous.front_vehicle_id ||
        current.side_vehicle_id != previous.side_vehicle_id ||
        current.parallel_side_vehicle_id != previous.parallel_side_vehicle_id ||
        current.parallel_follow_vehicle_id !=
            previous.parallel_follow_vehicle_id ||
        current.leader_priority_active != previous.leader_priority_active ||
        current.leader_priority_latched != previous.leader_priority_latched ||
        current.leader_priority_id != previous.leader_priority_id ||
        std::abs(current.leader_priority_delta_s -
                 previous.leader_priority_delta_s) > 0.05 ||
        current.leader_priority_reason != previous.leader_priority_reason ||
        current.pass_gap_reason != previous.pass_gap_reason ||
        current.yield_reason != previous.yield_reason ||
        current.safe_stop_triggered != previous.safe_stop_triggered ||
        current.start_grace_active != previous.start_grace_active ||
        current.safe_stop_release_ready != previous.safe_stop_release_ready ||
        current.safe_stop_reason != previous.safe_stop_reason ||
        current.safe_stop_reject_reason != previous.safe_stop_reject_reason ||
        current.safe_stop_trigger_count != previous.safe_stop_trigger_count ||
        current.safe_stop_hold_count != previous.safe_stop_hold_count ||
        current.safe_stop_release_count != previous.safe_stop_release_count ||
        current.speed_only_fallback_active !=
            previous.speed_only_fallback_active ||
        current.wall_risk_speed_guard_active !=
            previous.wall_risk_speed_guard_active ||
        current.mpc_health_speed_guard_active !=
            previous.mpc_health_speed_guard_active ||
        current.recovery_speed_guard_active !=
            previous.recovery_speed_guard_active ||
        current.reentry_requested != previous.reentry_requested ||
        current.reentry_permitted != previous.reentry_permitted ||
        current.reentry_clear_cycles != previous.reentry_clear_cycles ||
        current.reentry_reason != previous.reentry_reason ||
        current.reentry_primary_blocker_id !=
            previous.reentry_primary_blocker_id ||
        current.lateral_target_hold_active !=
            previous.lateral_target_hold_active ||
        current.lateral_target_hold_reason !=
            previous.lateral_target_hold_reason ||
        current.speed_cap_reason != previous.speed_cap_reason ||
        current.active_section_name != previous.active_section_name ||
        current.active_section_profile != previous.active_section_profile ||
        current.active_section_role_policy !=
            previous.active_section_role_policy ||
        std::abs(current.applied_speed_cap_mps -
                 previous.applied_speed_cap_mps) > 0.05 ||
        std::abs(current.wall_soft_margin_m - previous.wall_soft_margin_m) >
            0.05 ||
        current.mpc_health_valid != previous.mpc_health_valid ||
        current.mpc_infeasible_count != previous.mpc_infeasible_count ||
        std::abs(current.mpc_solve_time_ms - previous.mpc_solve_time_ms) >
            5.0 ||
        current.reason != previous.reason;
    return changed && (isInterestingDecisionEvent(current) ||
                       isInterestingDecisionEvent(previous));
  }

  // 入力: planner出力、自車状態、追い越し試行ID。
  // 出力: なし。必要な時だけRCLCPP_INFOで判断ログを出す。
  // 処理概要: debug topicを見返せない環境でも、mode遷移やsafe
  // stop理由をautoware.logから追えるようにする。
  void logDecisionEvent(const PlannerOutput &output, const EgoState &ego,
                        std::uint64_t attempt_id) {
    // 毎周期ではなく、判断入力や出力が変わった時だけautoware.logへ要約を残す。
    const auto current = makeDecisionLogSnapshot(output);
    const bool should_log = shouldLogDecisionEvent(current);
    last_decision_log_snapshot_ = current;
    has_decision_log_snapshot_ = true;
    if (!should_log) {
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "overtake decision: own_vehicle_id=%s attempt_id=%lu mode=%s "
        "selected=%s "
        "blocked=%d side_by_side=%d corner_side_by_side=%d parallel_side=%d "
        "future_side_by_side=%d "
        "future_corner_side_by_side=%d future_yield_required=%d "
        "active_override=%d "
        "front_id=%s front_ds=%.2f "
        "front_dd=%.2f rel_v=%.2f side_id=%s side_ds=%.2f side_dd=%.2f "
        "side_s_dot=%.2f parallel_id=%s parallel_ds=%.2f parallel_dd=%.2f "
        "parallel_follow=%d parallel_follow_feasible=%d "
        "parallel_follow_id=%s parallel_follow_ds=%.2f "
        "parallel_follow_dd=%.2f "
        "leader_priority=%d leader_latched=%d leader_id=%s "
        "leader_ds=%.2f leader_reason=%s "
        "can_left=%d can_right=%d pass_gap_reason=%s "
        "pass_left_gen=%d pass_left_ok=%d pass_left_reject=%s "
        "pass_right_gen=%d pass_right_ok=%d pass_right_reject=%s "
        "transaction_prepared=%d transaction_incomplete=%d "
        "transaction_side=%s "
        "maneuver_target_id=%s maneuver_target_ds=%.2f "
        "chain_tail_id=%s chain_tail_ds=%.2f safe_lateral_hold=%d "
        "tracking_stop=%d "
        "tracking_probe=%d/%d tracking_probe_reason=%s "
        "corner_abs_curvature=%.3f straight_start_allowed=%d "
        "overtake_start_abs_curvature=%.3f overtake_start_gate_reason=%s "
        "permission_allowed=%d permission_section=%s permission_reason=%s "
        "front_low_speed=%d slow_exception=%d slow_exception_count=%d "
        "slow_chain=%d slow_chain_id=%s front_speed=%.2f "
        "early_stationary_parallel=%d early_stationary_parallel_id=%s "
        "early_stationary_parallel_count=%d "
        "early_stationary_parallel_permission_exception=%d "
        "permission_start_exception=%d "
        "future_wall_clearance=%.2f yield_reason=%s "
        "pass_frozen=%d pass_freeze_reason=%s "
        "ego_s=%.2f ego_d=%.2f ego_wall_clearance=%.2f target_d=%.2f "
        "early_wall=%d/%d/%d early_wall_first_false=%s "
        "early_wall_reject=%s "
        "min_cbf_h=%.3f cbf_slack=%.3f "
        "safe_stop_triggered=%d start_grace=%d safe_stop_reason=%s "
        "safe_stop_reject_reason=%s "
        "safe_stop_trigger_count=%d safe_stop_hold_count=%d "
        "safe_stop_release_count=%d "
        "safe_stop_release_ready=%d speed_only=%d wall_guard=%d mpc_guard=%d "
        "recovery_guard=%d "
        "reentry_requested=%d reentry_permitted=%d reentry_clear_cycles=%d "
        "reentry_reason=%s reentry_blocker=%s "
        "speed_cap_reason=%s speed_cap=%.2f section=%s/%s mpc_infeasible=%d "
        "mpc_solve_ms=%.2f reason=%s",
        own_vehicle_id_.c_str(), static_cast<unsigned long>(attempt_id),
        toString(output.mode), toString(output.selected),
        output.blocked_info.blocked, output.blocked_info.side_by_side,
        output.blocked_info.corner_side_by_side,
        output.blocked_info.parallel_side_candidate,
        output.blocked_info.future_side_by_side,
        output.blocked_info.future_corner_side_by_side,
        output.blocked_info.future_yield_required, output.active_override,
        output.blocked_info.nearest_id.c_str(),
        output.blocked_info.front_delta_s, output.blocked_info.front_delta_d,
        output.blocked_info.front_rel_v, output.blocked_info.side_id.c_str(),
        output.blocked_info.side_delta_s, output.blocked_info.side_delta_d,
        output.blocked_info.side_s_dot_mps,
        output.blocked_info.parallel_side_id.c_str(),
        output.blocked_info.parallel_side_delta_s,
        output.blocked_info.parallel_side_delta_d,
        output.blocked_info.parallel_follow_candidate,
        output.blocked_info.parallel_follow_feasible,
        output.blocked_info.parallel_follow_id.c_str(),
        output.blocked_info.parallel_follow_delta_s,
        output.blocked_info.parallel_follow_delta_d,
        output.blocked_info.leader_priority_active,
        output.blocked_info.leader_priority_latched,
        output.blocked_info.leader_priority_id.c_str(),
        output.blocked_info.leader_priority_delta_s,
        output.blocked_info.leader_priority_reason.c_str(),
        output.blocked_info.can_pass_left, output.blocked_info.can_pass_right,
        output.blocked_info.pass_gap_reason.c_str(),
        output.blocked_info.pass_left_candidate_generated,
        output.blocked_info.pass_left_candidate_feasible,
        output.blocked_info.pass_left_candidate_reject_reason.c_str(),
        output.blocked_info.pass_right_candidate_generated,
        output.blocked_info.pass_right_candidate_feasible,
        output.blocked_info.pass_right_candidate_reject_reason.c_str(),
        output.blocked_info.maneuver_transaction_prepared,
        output.blocked_info.maneuver_transaction_incomplete,
        toString(output.blocked_info.maneuver_transaction_pass_type),
        output.blocked_info.maneuver_target_id.c_str(),
        output.blocked_info.maneuver_target_relative_s_m,
        output.blocked_info.maneuver_chain_tail_id.c_str(),
        output.blocked_info.maneuver_chain_tail_relative_s_m,
        output.blocked_info.maneuver_transaction_safe_lateral_hold_active,
        output.blocked_info.maneuver_transaction_tracking_stop_active,
        output.blocked_info.maneuver_transaction_tracking_probe_cycles,
        output.blocked_info.maneuver_transaction_tracking_probe_required_cycles,
        output.blocked_info.maneuver_transaction_tracking_probe_reset_reason
            .c_str(),
        output.blocked_info.corner_abs_curvature,
        output.blocked_info.straight_overtake_start_allowed,
        output.blocked_info.overtake_start_abs_curvature,
        output.blocked_info.overtake_start_gate_reason.c_str(),
        output.blocked_info.overtake_permission_allowed,
        output.blocked_info.overtake_permission_section_name.c_str(),
        output.blocked_info.overtake_permission_reason.c_str(),
        output.blocked_info.front_vehicle_low_speed,
        output.blocked_info.slow_front_exception_active,
        output.blocked_info.slow_front_exception_count,
        output.blocked_info.slow_obstacle_chain_active,
        output.blocked_info.slow_obstacle_chain_id.c_str(),
        output.blocked_info.front_vehicle_speed_mps,
        output.blocked_info.early_stationary_parallel_pass_target,
        output.blocked_info.early_stationary_parallel_pass_id.c_str(),
        output.blocked_info.early_stationary_parallel_pass_count,
        output.blocked_info.confirmed_stationary_parallel_permission_exception,
        output.blocked_info.permission_start_exception_active,
        output.blocked_info.future_wall_clearance_m,
        output.blocked_info.yield_reason.c_str(),
        output.blocked_info.pass_decision_frozen,
        output.blocked_info.pass_decision_freeze_reason.c_str(), ego.frenet.s,
        ego.frenet.d, output.blocked_info.ego_wall_clearance_m,
        output.target_lateral_offset_m,
        output.blocked_info.early_wall_recovery_probe_requested,
        output.blocked_info.early_wall_recovery_probe_generated,
        output.blocked_info.early_wall_recovery_probe_feasible,
        output.blocked_info.early_wall_recovery_probe_first_false.c_str(),
        output.blocked_info.early_wall_recovery_reject_reason.c_str(),
        output.min_cbf_h, output.cbf_slack, output.safe_stop_triggered,
        output.start_grace_active, output.safe_stop_reason.c_str(),
        output.safe_stop_reject_reason.c_str(), output.safe_stop_trigger_count,
        output.safe_stop_hold_count, output.safe_stop_release_count,
        output.safe_stop_release_ready, output.speed_only_fallback_active,
        output.wall_risk_speed_guard_active,
        output.mpc_health_speed_guard_active,
        output.recovery_speed_guard_active, output.reentry_gate.requested,
        output.reentry_gate.permitted, output.reentry_gate.clear_cycles,
        output.reentry_gate.reason.c_str(),
        output.reentry_gate.blocking_vehicle_id.c_str(),
        output.speed_cap_reason.c_str(), output.applied_speed_cap_mps,
        output.active_section.name.c_str(),
        output.active_section.profile.c_str(),
        output.mpc_health.infeasible_count, output.mpc_health.solve_time_ms,
        output.reason.c_str());
    const auto log_deadline =
        [&](const char *side, const PassTransitionDeadlineDiagnostic &diag) {
          if (!diag.evaluated) {
            return;
          }
          RCLCPP_INFO(
              get_logger(),
              "overtake pass deadline: side=%s source=%s input_valid=%d "
              "reachable=%d lateral_shift_m=%.3f transition_start_s_m=%.3f "
              "transition_end_s_m=%.3f required_m=%.3f available_m=%.3f "
              "slack_m=%.3f deadline_speed_mps=%.3f tracking_speed_mps=%.3f "
              "proposal_speed_mps=%.3f proposal_horizon_sec=%.3f "
              "proposal_endpoint_arc_m=%.3f pp_required_arc_m=%.3f "
              "time_to_required_sec=%.3f",
              side, passTransitionDeadlineSourceString(diag.source),
              diag.input_valid, diag.reachable, diag.lateral_shift_m,
              diag.transition_start_s_m, diag.transition_end_s_m,
              diag.required_transition_m, diag.available_deadline_m,
              diag.deadline_slack_m, diag.deadline_speed_cap_mps,
              diag.evaluated_tracking_speed_mps, diag.proposal_speed_cap_mps,
              diag.proposal_horizon_sec, diag.proposal_endpoint_arc_m,
              diag.pp_required_arc_m,
              diag.proposal_time_to_required_transition_sec);
        };
    log_deadline("left",
                 output.blocked_info.pass_left_candidate_transition_deadline);
    log_deadline("right",
                 output.blocked_info.pass_right_candidate_transition_deadline);
    RCLCPP_INFO(
        get_logger(),
        "overtake gate detail: target_identity=%d direct_normal=%d "
        "direct_braking=%d fresh_dynamic=%d dynamic_speed_reachable=%d "
        "target_context=%s context_clean=%d lateral_capacity=%d "
        "dynamic_speed_cap=%d curvature_ok=%d observation_complete=%d "
        "prediction_complete=%d tracking_usable=%d mpc_ready=%d "
        "safe_pass=%d safe_side=%s safe_cbf=%.3f approved=%d blocker=%s "
        "safe_cycles_left=%d safe_cycles_right=%d safe_cycle_reset=%s "
        "follow_cap=%.2f follow_terminal=%.2f",
        output.blocked_info.gentle_curve_target_identity_valid,
        output.blocked_info.gentle_curve_direct_normal_target,
        output.blocked_info.gentle_curve_direct_braking_target,
        output.blocked_info.gentle_curve_fresh_dynamic_gap_target,
        output.blocked_info.gentle_curve_dynamic_target_speed_reachable,
        output.blocked_info.gentle_curve_target_context_reason.c_str(),
        output.blocked_info.gentle_curve_context_clean,
        output.blocked_info.gentle_curve_lateral_capacity_sufficient,
        output.blocked_info.gentle_curve_dynamic_speed_cap_valid,
        output.blocked_info.gentle_curve_curvature_within_limit,
        output.blocked_info.gentle_curve_observation_inputs_complete,
        output.blocked_info.gentle_curve_prediction_complete,
        output.blocked_info.gentle_curve_tracking_usable,
        output.blocked_info.gentle_curve_mpc_ready,
        output.blocked_info.gentle_curve_safe_pass_found,
        toString(output.blocked_info.gentle_curve_safe_pass_side),
        output.blocked_info.gentle_curve_safe_pass_cbf_slack,
        output.blocked_info.gentle_curve_safe_pass_start_approved,
        output.blocked_info.gentle_curve_safe_pass_block_reason.c_str(),
        output.blocked_info.pass_left_safe_cycles,
        output.blocked_info.pass_right_safe_cycles,
        output.blocked_info.pass_safe_cycle_reset_reason.c_str(),
        output.blocked_info.follow_candidate_speed_cap_mps,
        output.blocked_info.follow_candidate_terminal_speed_mps);
  }

  // 入力: ROS timer周期。
  // 出力: override topic、mode topic、debug metrics、必要に応じた判断ログ。
  // 処理概要:
  // odomをFrenet自車状態へ変換し、相手車収集からcore更新、publishまでを1周期で実行する。
  void publishRaceNotArmed(const EgoState &ego,
                           const PlannerOutput *proposal_diagnostic = nullptr) {
    PlannerOutput output =
        proposal_diagnostic != nullptr ? *proposal_diagnostic : PlannerOutput{};
    output.mode = BehaviorMode::FREE_RUN;
    output.selected = CandidateType::FASTEST;
    output.reason = "race_not_armed";
    // pre-arm Coreは候補生成・安全診断だけに使う。横/縦payload、token、
    // generation、motion authorityは一切publishせず、従来のtyped STOPを
    // 唯一のauthorityにする。
    output.active_override = false;
    output.lateral_offsets.clear();
    output.longitudinal_offsets_m.clear();
    output.tracking_release_pass_warmup = false;
    output.tracking_release_token = 0U;
    output.selected_lateral_profile_safety_verified = false;
    output.safe_stop_triggered = true;
    output.safe_stop_reason = "race_not_armed";
    output.safe_stop_v_mps = pre_arm_stop_speed_mps_;
    output.longitudinal_speed_cap_active = true;
    output.speed_caps.assign(
        std::max<std::size_t>(1U, planner_config_.horizon_points),
        pre_arm_stop_speed_mps_);
    output.applied_speed_cap_mps = pre_arm_stop_speed_mps_;
    output.speed_cap_reason = "race_not_armed";
    publishOverride(output, 0U);

    SafetyConstraintCommand constraint;
    constraint.valid = true;
    constraint.stop_requested = true;
    constraint.release_authorized = false;
    constraint.speed_limit_mps = pre_arm_stop_speed_mps_;
    constraint.required_brake_decel_mps2 =
        safety_constraint_maximum_brake_decel_mps2_;
    constraint.reason = "race_not_armed";
    const auto contract_stamp = now();
    const auto constraint_msg =
        publishSafetyConstraint(constraint, contract_stamp);
    publishAuthoritativePlan(output, ego, 0U, constraint, constraint_msg,
                             contract_stamp);
    publishDebug(output, ego, 0U);
    logDecisionEvent(output, ego, 0U);
  }

  void onTimer() {
    overtake_transport_contract::c002ay1::PlannerObservationScope
        c002ay1_observation(c002ay1_runtime_observer_, 0, 0U,
                            c002ay1_configured_stream_);
    c002ay1_active_observation_ = &c002ay1_observation.record();
    // 最新odomを自車状態へ変換し、他車収集 -> コア更新 -> override/debug
    // publishを1周期で行う。
    drainLatestControllerTrackingStatus();
    if (current_safety_snapshot_id_ !=
        std::numeric_limits<std::uint64_t>::max()) {
      ++current_safety_snapshot_id_;
    } else {
      current_safety_snapshot_id_ = 0U;
      aw2_identity_exhausted_ = true;
    }
    const auto planner_now = now();
    const double now_sec = planner_now.seconds();
    const builtin_interfaces::msg::Time planner_now_msg = planner_now;
    c002ay1_observation.record().ros_sec = planner_now_msg.sec;
    c002ay1_observation.record().ros_nanosec = planner_now_msg.nanosec;
    EgoState ego;
    if (odom_.has_value() && !frame_.empty()) {
      const auto &odom = *odom_;
      ego.stamp_sec = stampToSec(odom.header.stamp);
      if (inputTimestampFresh(now_sec, ego.stamp_sec, ego_stale_time_sec_,
                              input_future_stamp_tolerance_sec_)) {
        // 古いodomで追い越し判断すると危ないので、新鮮な時だけvalidにする。
        ego.x = odom.pose.pose.position.x;
        ego.y = odom.pose.pose.position.y;
        ego.yaw = yawFromQuaternion(odom.pose.pose.orientation);
        ego.v = odom.twist.twist.linear.x;
        ego.frenet = frame_.cartesianToFrenet(ego.x, ego.y, ego.yaw);
        ego.valid = true;
      }
    }

    const auto opponents = collectOpponents(ego, now_sec);
    const auto mpc_health = currentMpcHealth(now_sec);
    const auto reentry_input = reentryInputStatus(now_sec, ego, mpc_health);
    if (race_arm_required_ && !race_armed_) {
      // live Coreはarm前に進めない。一方、別Coreでstart-grid候補を
      // PROPOSAL_SAFETY_EVALUATIONまで準備し、D1/D2配置の最初の物理rejectを
      // debugへ残す。publishRaceNotArmed()がtrajectory/tokenを消去し、
      // typed STOP authorityだけを送るため、正式Start前の走行権限は0のまま。
      std::optional<PlannerOutput> proposal_diagnostic;
      if (prearm_proposal_core_ != nullptr && ego.valid) {
        proposal_diagnostic = prearm_proposal_core_->update(
            now_sec, ego, opponents, mpc_health, reentry_input, nullptr);
      }
      publishRaceNotArmed(ego, proposal_diagnostic.has_value()
                                   ? &*proposal_diagnostic
                                   : nullptr);
      c002ay1_observation.setReturnReason(
          overtake_transport_contract::c002ay1::ReturnReason::
              kExternalAuthorityStop);
      c002ay1_active_observation_ = nullptr;
      return;
    }

    const PurePursuitExactSnapshot *pp_exact_snapshot =
        pp_core_exact_snapshot_enabled_ &&
                pure_pursuit_exact_snapshot_.has_value()
            ? &pure_pursuit_exact_snapshot_.value()
            : nullptr;
    const auto core_started = std::chrono::steady_clock::now();
    if (last_core_update_started_.has_value()) {
      planner_proposal_interval_ms_ =
          std::chrono::duration<double, std::milli>(
              core_started - last_core_update_started_.value())
              .count();
    }
    last_core_update_started_ = core_started;
    auto output = core_->update(now_sec, ego, opponents, mpc_health,
                                reentry_input, pp_exact_snapshot);
    planner_update_steady_duration_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - core_started)
            .count();
    const auto attempt_id = updateAttemptId(output);
    publishOverride(output, attempt_id);
    if (output.blocked_info.maneuver_transaction_tracking_release_pending &&
        !reentry_input.pure_pursuit_primary_and_fresh &&
        !purePursuitReleaseReady(now_sec)) {
      // coreが2段目proofを確認していても、この周期のpayload差分でgenerationが
      // 更新された場合は解除しない。publish後の実generationとPP proofが一致する
      // 次周期までstopを保持し、TOCTOUで未追従PASSを動かさない。
      output.blocked_info.maneuver_transaction_tracking_stop_active = true;
    }
    const auto constraint = safety_constraint_release_gate_.filter(
        makeSafetyConstraint(output, ego, reentry_input,
                             safety_constraint_normal_speed_limit_mps_,
                             safety_constraint_maximum_brake_decel_mps2_));
    const auto contract_stamp = now();
    // tighter/STOP authorityをtyped planより先にpublishする。release側はMuxが
    // exactなplan/constraint/PP proofを揃えるまで保留するため、topic間の
    // delivery gapで新PASSへ旧non-STOP authorityを組み合わせない。
    const auto constraint_msg =
        publishSafetyConstraint(constraint, contract_stamp);
    publishAuthoritativePlan(output, ego, attempt_id, constraint,
                             constraint_msg, contract_stamp);
    publishSupervisorV2Shadow(output, ego, reentry_input);
    publishDebug(output, ego, attempt_id);
    logDecisionEvent(output, ego, attempt_id);
    c002ay1_observation.setReturnReason(
        c002ay1_observation.record().selected_proposal_count > 0U
            ? overtake_transport_contract::c002ay1::ReturnReason::kCompletedEmit
            : overtake_transport_contract::c002ay1::ReturnReason::
                  kCompletedNoEmit);
    c002ay1_active_observation_ = nullptr;
  }

  FrenetFrame frame_;
  PlannerConfig planner_config_;
  std::unique_ptr<OvertakePlannerCore> core_;
  std::unique_ptr<OvertakePlannerCore> prearm_proposal_core_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::unordered_map<std::string, TrackSample> samples_;
  std::string own_vehicle_id_;
  double ignore_near_ego_m_{1.0};
  double position_jump_threshold_m_{5.0};
  double ego_stale_time_sec_{0.50};
  double input_future_stamp_tolerance_sec_{0.05};
  double opponent_stale_time_sec_{0.50};
  double reentry_v2x_snapshot_stale_time_sec_{0.50};
  double control_rate_hz_{20.0};
  double mpc_health_stale_time_sec_{0.60};
  int mpc_health_infeasible_count_threshold_{1};
  double mpc_health_solve_time_warn_ms_{80.0};
  double controller_tracking_status_timeout_sec_{0.30};
  double planner_update_steady_duration_ms_{
      std::numeric_limits<double>::quiet_NaN()};
  double planner_proposal_interval_ms_{
      std::numeric_limits<double>::quiet_NaN()};
  std::optional<std::chrono::steady_clock::time_point>
      last_core_update_started_{};
  bool pp_core_exact_snapshot_enabled_{false};
  std::optional<PurePursuitExactSnapshot> pure_pursuit_exact_snapshot_{};
  std::string pure_pursuit_exact_snapshot_reason_{"disabled"};
  std::uint64_t pure_pursuit_exact_producer_instance_id_{0U};
  std::uint64_t pure_pursuit_exact_command_sequence_{0U};
  double safety_constraint_normal_speed_limit_mps_{10.0};
  double safety_constraint_maximum_brake_decel_mps2_{1.0};
  double horizon_dt_sec_{0.025};
  double pre_arm_stop_speed_mps_{0.0};
  bool race_arm_required_{true};
  bool race_armed_{false};
  std::string race_arm_topic_{"/overtake/race_armed"};
  std::uint64_t race_arm_epoch_{0U};
  bool c002ay1_prod_measure_enabled_{false};
  overtake_transport_contract::c002ay1::ConfiguredStreamKind
      c002ay1_configured_stream_{
          overtake_transport_contract::c002ay1::ConfiguredStreamKind::
              kLegacyReferenceOverride};
  overtake_transport_contract::c002ay1::RuntimeObservationWriter
      c002ay1_runtime_observer_{};
  overtake_transport_contract::c002ay1::PlannerCallbackObservationV1
      *c002ay1_active_observation_{nullptr};
  std::uint64_t c002ay1_selected_proposal_sequence_{0U};
  const std::uint64_t planner_instance_id_{makePlannerInstanceId()};
  const std::uint64_t v2_planner_instance_id_{
      makeDistinctPlannerInstanceId(planner_instance_id_)};
  aw2::ConnectorTransactionSequencer connector_transaction_sequencer_{};
  aw2::ConnectorTransactionSequencer v2_connector_transaction_sequencer_{};
  aw2::SameGenerationBindingTracker aw2_candidate_binding_tracker_{};
  aw2::SameGenerationBindingTracker aw2_v2_candidate_binding_tracker_{};
  aw2::DeliveryRecordTracker aw2_delivery_record_tracker_{};
  aw2::DeliveryRecordTracker aw2_v2_delivery_record_tracker_{};
  bool aw2_identity_exhausted_{false};
  std::uint64_t current_safety_snapshot_id_{0U};
  std::optional<std::vector<std::uint8_t>> last_aw2_source_wire_{};
  int safety_constraint_release_safe_cycles_{3};
  bool supervisor_v2_shadow_enabled_{false};
  SafetyConstraintReleaseGate safety_constraint_release_gate_{3};
  SafetyConstraintReleaseGate supervisor_v2_constraint_release_gate_{3, true};
  MpcHealthStatus mpc_health_{};
  std::uint64_t mpc_health_sample_sequence_{0U};
  std::optional<double> last_mpc_health_sec_;
  std::optional<double> last_v2x_snapshot_sec_;
  std::optional<double> last_controller_tracking_status_sec_;
  double controller_tracking_command_age_sec_{
      std::numeric_limits<double>::infinity()};
  bool controller_tracking_mpc_horizon_usable_{false};
  bool controller_tracking_pp_command_fresh_{false};
  bool controller_tracking_usable_{false};
  bool controller_tracking_safety_constraint_release_ready_{false};
  bool controller_tracking_attack_follow_stop_transport_release_ready_{false};
  std::uint8_t controller_tracking_pass_probe_transport_evidence_{
      static_cast<std::uint8_t>(StartGridProbeTransportEvidence::UNKNOWN)};
  bool controller_tracking_pass_probe_exact_current_usable_{false};
  std::uint64_t controller_tracking_pass_probe_lateral_stop_authority_token_{
      0U};
  std::uint32_t controller_tracking_plan_generation_{0U};
  std::string controller_tracking_reason_{};
  std::uint64_t motion_grant_issuer_instance_id_{0U};
  std::uint64_t motion_grant_sequence_{0U};
  bool motion_grant_observed_{false};
  std::string motion_grant_reason_{"missing"};
  std::optional<double> last_motion_grant_status_sec_;
  std::uint32_t last_authoritative_plan_generation_{0U};
  std::uint32_t last_authoritative_plan_attempt_id_{0U};
  std::int8_t last_authoritative_plan_pass_direction_{0};
  std::uint64_t last_authoritative_plan_connector_transaction_id_{0U};
  std::uint32_t last_authoritative_plan_candidate_revision_{0U};
  std::array<std::uint8_t, 32>
      last_authoritative_plan_candidate_content_sha256_{};
  bool last_pass_probe_warmup_valid_{false};
  std::uint64_t last_pass_probe_warmup_race_arm_epoch_{0U};
  std::uint64_t last_pass_probe_warmup_planner_instance_id_{0U};
  std::uint32_t last_pass_probe_warmup_plan_generation_{0U};
  std::uint32_t last_pass_probe_warmup_attempt_id_{0U};
  std::string last_pass_probe_warmup_target_vehicle_id_{};
  std::int8_t last_pass_probe_warmup_pass_direction_{0};
  std::uint64_t last_pass_probe_warmup_connector_transaction_id_{0U};
  double last_pass_probe_warmup_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  std::uint32_t last_pass_probe_warmup_candidate_revision_{0U};
  std::array<std::uint8_t, 32>
      last_pass_probe_warmup_candidate_content_sha256_{};
  std::uint64_t last_pass_probe_warmup_authority_token_{0U};
  double last_authoritative_plan_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  BehaviorMode last_mode_{BehaviorMode::FREE_RUN};
  DecisionLogSnapshot last_decision_log_snapshot_{};
  bool has_decision_log_snapshot_{false};
  bool attempt_active_{false};
  std::uint64_t current_attempt_id_{0};
  ReferenceOverrideWirePayload last_override_wire_payload_;
  std::uint32_t override_generation_{0};
  bool has_last_override_identity_{false};
  bool last_override_latch_active_{false};
  std::string last_override_target_id_{};
  std::string last_override_authoritative_target_id_{};
  CandidateType last_override_pass_type_{CandidateType::FASTEST};
  OverrideTrackingIdentity current_override_tracking_identity_{};
  OverrideTrackingIdentity previous_override_tracking_identity_{};
  bool has_last_tracking_release_token_{false};
  std::uint64_t last_tracking_release_token_{0U};
  bool tracking_release_ack_expected_{false};
  std::uint64_t tracking_release_expected_token_{0U};
  std::uint32_t tracking_release_expected_generation_{0U};
  std::string tracking_release_expected_target_id_{};
  CandidateType tracking_release_expected_pass_type_{CandidateType::FASTEST};
  SafetyConstraintCommand last_safety_constraint_command_{};
  std::uint32_t safety_constraint_generation_{0};
  std::uint32_t last_safety_constraint_plan_generation_{0};
  SafetyConstraintCommand last_supervisor_v2_constraint_command_{};
  std::uint32_t supervisor_v2_constraint_generation_{0};
  std::uint32_t last_supervisor_v2_constraint_plan_generation_{0};
  std::string last_supervisor_v2_prefilter_constraint_reason_{};
  std::string last_supervisor_v2_filtered_constraint_reason_{};
  std::uint32_t last_supervisor_v2_authorization_failure_mask_{0U};
  std::vector<std::string> last_supervisor_v2_authorization_failure_reasons_{};
  bool last_supervisor_v2_trajectory_publishable_{false};
  bool last_supervisor_v2_effective_trajectory_authorized_{false};
  bool last_authoritative_plan_feedback_valid_{false};
  bool last_authoritative_plan_stop_requested_{false};
  bool last_authoritative_plan_trajectory_authorized_{false};
  std::string last_authoritative_plan_target_id_{};
  CandidateType last_authoritative_plan_pass_type_{CandidateType::FASTEST};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr override_pub_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::SafetyConstraint>::
      SharedPtr safety_constraint_pub_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>::SharedPtr
      authoritative_plan_pub_;
  rclcpp::Publisher<
      multi_purpose_mpc_ros_msgs::msg::CandidateExecutionRequest>::SharedPtr
      candidate_execution_request_pub_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::OvertakePlan>::SharedPtr
      supervisor_v2_plan_pub_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::SafetyConstraint>::
      SharedPtr supervisor_v2_constraint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr
      v2x_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpc_health_sub_;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus>::SharedPtr
      controller_tracking_status_sub_;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope>::SharedPtr
      controller_execution_envelope_sub_;
  rclcpp::Subscription<multi_purpose_mpc_ros_msgs::msg::MotionAuthorityGrant>::
      SharedPtr motion_authority_grant_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr race_arm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace overtake_planner

// 入力: ROS 2プロセス引数。
// 出力: 終了コード。正常終了時は0。
// 処理概要:
// OvertakePlannerNodeを生成してspinし、シャットダウン時にrclcppを閉じる。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<overtake_planner::OvertakePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
