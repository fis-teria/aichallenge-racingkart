#include "overtake_planner/overtake_planner_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
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

// 入力: パッケージ名とCSVパス。
// 出力: 絶対パス。csv_pathが絶対パスならそのまま返す。
// 処理概要: launch/configでは短い相対パスを書けるよう、package share配下へ解決する。
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
  std::transform(value.begin(), value.end(), std::back_inserter(lowered),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
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
// 処理概要: 複数台評価でdomain_idと車両IDを揃え、自車を相手車リストから除外する。
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
// 処理概要: 明示IDを優先し、auto時はROS_DOMAIN_IDから推定、失敗時はd1へフォールバックする。
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
      config.max_brake_decel_mps2 <= 0.0 ||
      config.max_brake_decel_mps2 > 1.5) {
    return "max_brake_decel_mps2 must be finite and in (0, 1.5]";
  }
  if (!std::isfinite(config.longitudinal_response_delay_sec) ||
      config.longitudinal_response_delay_sec < 0.0) {
    return "longitudinal_response_delay_sec must be finite and >= 0";
  }
  if (!std::isfinite(config.stationary_obstacle_speed_threshold_mps) ||
      config.stationary_obstacle_speed_threshold_mps < 0.0) {
    return "stationary_obstacle_speed_threshold_mps must be finite and >= 0";
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
       config.reentry_hold_v_max_mps <= 0.0)) {
    return "invalid reentry gate safety parameter";
  }
  return std::nullopt;
}

} // namespace

class OvertakePlannerNode : public rclcpp::Node {
public:
  // 入力: ROS parameter、参照CSV、V2X/odom/MPC health topic。
  // 出力: publisher/subscriber/timerを持つROS node。
  // 処理概要: PlannerConfigとFrenetFrameを初期化し、周期timerでcoreを更新する実行環境を作る。
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
    own_vehicle_id_ = resolveOwnVehicleId(
        declare_parameter<std::string>("own_vehicle_id", "auto"), get_logger());
    ignore_near_ego_m_ = declare_parameter<double>("ignore_near_ego_m", 1.0);
    position_jump_threshold_m_ =
        declare_parameter<double>("position_jump_threshold_m", 5.0);
    ego_stale_time_sec_ = declare_parameter<double>("ego_stale_time_sec", 0.50);

    PlannerConfig config;
    config.enabled = declare_parameter<bool>("enabled", true);
    const int horizon_points_param = declare_parameter<int>("horizon_points", 20);
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
    config.side_by_side_s_m =
        declare_parameter<double>("side_by_side_s_m", 4.0);
    config.side_margin_m = declare_parameter<double>("side_margin_m", 1.2);
    config.parallel_side_detection_enabled =
        declare_parameter<bool>("parallel_side_detection_enabled", true);
    config.parallel_side_s_m =
        declare_parameter<double>("parallel_side_s_m", 12.0);
    config.parallel_side_margin_m =
        declare_parameter<double>("parallel_side_margin_m", 4.0);
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
    config.overtake_permission_profile_enabled =
        declare_parameter<bool>("overtake_permission_profile_enabled", true);
    config.default_overtake_allowed =
        declare_parameter<bool>("default_overtake_allowed", true);
    config.overtake_permission_lookahead_m =
        declare_parameter<double>("overtake_permission_lookahead_m", 8.0);
    config.slow_front_exception_enabled =
        declare_parameter<bool>("slow_front_exception_enabled", true);
    config.slow_front_exception_speed_mps =
        declare_parameter<double>("slow_front_exception_speed_mps", 1.0);
    config.slow_front_exception_distance_m =
        declare_parameter<double>("slow_front_exception_distance_m", 8.0);
    config.slow_front_exception_required_cycles =
        declare_parameter<int>("slow_front_exception_required_cycles", 3);
    config.slow_obstacle_chain_enabled =
        declare_parameter<bool>("slow_obstacle_chain_enabled", true);
    config.slow_obstacle_chain_distance_m =
        declare_parameter<double>("slow_obstacle_chain_distance_m", 12.0);
    config.large_lateral_error_threshold_m =
        declare_parameter<double>("large_lateral_error_threshold_m", 0.60);
    config.large_lateral_error_v_max_mps =
        declare_parameter<double>("large_lateral_error_v_max_mps", 2.5);
    config.min_pass_gap_m = declare_parameter<double>("min_pass_gap_m", 1.80);
    config.pass_gap_hysteresis_m =
        declare_parameter<double>("pass_gap_hysteresis_m", 0.15);
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
    config.reentry_gate_enabled =
        declare_parameter<bool>("reentry_gate_enabled", true);
    config.reentry_safe_cycles =
        declare_parameter<int>("reentry_safe_cycles", 5);
    config.reentry_min_safety_margin_h =
        declare_parameter<double>("reentry_min_safety_margin_h", 0.30);
    config.reentry_evaluation_horizon_sec =
        declare_parameter<double>("reentry_evaluation_horizon_sec", 4.0);
    config.reentry_v2x_snapshot_stale_time_sec = declare_parameter<double>(
        "reentry_v2x_snapshot_stale_time_sec", 0.50);
    config.reentry_hold_v_max_mps =
        declare_parameter<double>("reentry_hold_v_max_mps", 0.50);
    config.reentry_require_mpc_health =
        declare_parameter<bool>("reentry_require_mpc_health", true);
    config.left_offset_m = declare_parameter<double>("left_offset_m", 0.80);
    config.right_offset_m = declare_parameter<double>("right_offset_m", -0.80);
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
    config.localized_avoidance_hold_after_target_m =
        declare_parameter<double>("localized_avoidance_hold_after_target_m",
                                  5.0);
    config.localized_avoidance_merge_distance_m =
        declare_parameter<double>("localized_avoidance_merge_distance_m", 8.0);
    config.maneuver_latch_min_hold_sec =
        declare_parameter<double>("maneuver_latch_min_hold_sec", 1.0);
    config.maneuver_latch_target_update_alpha =
        declare_parameter<double>("maneuver_latch_target_update_alpha", 0.0);
    config.prepare_distance_m =
        declare_parameter<double>("prepare_distance_m", 8.0);
    config.merge_distance_m =
        declare_parameter<double>("merge_distance_m", 12.0);
    config.follow_speed_margin_mps =
        declare_parameter<double>("follow_speed_margin_mps", 0.20);
    config.recovery_v_max_mps =
        declare_parameter<double>("recovery_v_max_mps", 8.5);
    config.wall_margin_recovery_v_max_mps =
        declare_parameter<double>("wall_margin_recovery_v_max_mps", 8.5);
    config.outside_corridor_recovery_centering_time_sec =
        declare_parameter<double>("outside_corridor_recovery_centering_time_sec",
                                  1.0);
    config.v_passthrough_mps =
        declare_parameter<double>("v_passthrough_mps", 50.0);
    config.d_min_m = declare_parameter<double>("d_min_m", -1.35);
    config.d_max_m = declare_parameter<double>("d_max_m", 1.35);
    config.min_wall_margin_m =
        declare_parameter<double>("min_wall_margin_m", 0.50);
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
    config.speed_only_fallback_enabled =
        declare_parameter<bool>("speed_only_fallback_enabled", true);
    config.speed_only_fallback_v_max_mps =
        declare_parameter<double>("speed_only_fallback_v_max_mps", 3.0);
    config.opponent_collision_fallback_v_max_mps = declare_parameter<double>(
        "opponent_collision_fallback_v_max_mps", 0.5);
    config.side_by_side_leader_priority_enabled =
        declare_parameter<bool>("side_by_side_leader_priority_enabled", true);
    config.side_by_side_leader_priority_enter_s_m = declare_parameter<double>(
        "side_by_side_leader_priority_enter_s_m", 1.0);
    config.side_by_side_leader_priority_release_s_m = declare_parameter<double>(
        "side_by_side_leader_priority_release_s_m", 0.3);
    config.side_by_side_leader_priority_hold_sec = declare_parameter<double>(
        "side_by_side_leader_priority_hold_sec", 1.0);
    config.side_by_side_leader_priority_v_max_mps = declare_parameter<double>(
        "side_by_side_leader_priority_v_max_mps", 3.0);
    config.wall_risk_speed_guard_enabled =
        declare_parameter<bool>("wall_risk_speed_guard_enabled", true);
    config.wall_soft_margin_m =
        declare_parameter<double>("wall_soft_margin_m", 0.25);
    config.wall_risk_v_max_mps =
        declare_parameter<double>("wall_risk_v_max_mps", 5.0);
    config.mpc_health_speed_guard_enabled =
        declare_parameter<bool>("mpc_health_speed_guard_enabled", true);
    config.mpc_health_infeasible_count_threshold =
        declare_parameter<int>("mpc_health_infeasible_count_threshold", 1);
    config.mpc_health_solve_time_warn_ms =
        declare_parameter<double>("mpc_health_solve_time_warn_ms", 80.0);
    config.mpc_health_v_max_mps =
        declare_parameter<double>("mpc_health_v_max_mps", 3.0);
    config.mpc_health_stale_time_sec =
        declare_parameter<double>("mpc_health_stale_time_sec", 0.60);
    config.recovery_speed_guard_enabled =
        declare_parameter<bool>("recovery_speed_guard_enabled", true);
    config.recovery_speed_guard_v_max_mps =
        declare_parameter<double>("recovery_speed_guard_v_max_mps", 3.0);
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
    } else {
      RCLCPP_INFO(
          get_logger(), "loaded overtake reference: %s (%zu points, %.2f m)",
          reference_path.c_str(), frame.reference().size(), frame.length());
    }
    frame_ = frame;
    config.section_safety_rules = readSectionSafetyRules(frame_);
    config.overtake_permission_rules = readOvertakePermissionRules(
        frame_, overtake_permission_package, overtake_permission_csv);
    mpc_health_stale_time_sec_ = config.mpc_health_stale_time_sec;
    opponent_stale_time_sec_ = config.opponent_stale_time_sec;
    reentry_v2x_snapshot_stale_time_sec_ =
        config.reentry_v2x_snapshot_stale_time_sec;
    mpc_health_infeasible_count_threshold_ =
        config.mpc_health_infeasible_count_threshold;
    mpc_health_solve_time_warn_ms_ = config.mpc_health_solve_time_warn_ms;
    core_ = std::make_unique<OvertakePlannerCore>(frame_, config);

    // MPCへ渡すoverride配列と、evalwrapで拾うdebugトピックをpublishする。
    override_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/overtake/reference_override", rclcpp::QoS(1));
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
    bool future_side_by_side{false};
    bool future_corner_side_by_side{false};
    bool future_yield_required{false};
    bool future_outer_wall_risk{false};
    bool can_pass_left{false};
    bool can_pass_right{false};
    bool active_override{false};
    double corner_abs_curvature{0.0};
    bool straight_overtake_start_allowed{true};
    double overtake_start_abs_curvature{0.0};
    std::string overtake_start_gate_reason{};
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
    std::string front_vehicle_id{};
    std::string side_vehicle_id{};
    std::string parallel_side_vehicle_id{};
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
  // 処理概要: YAMLの区間安全設定を読み、wp指定またはs指定を統一したs区間へ変換する。
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
    // 設計意図: 一部の配列だけ短い設定でも、欠損を警告しながら有効なruleだけ採用する。
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
  // 処理概要: name,start_wp,end_wp,allow_overtake形式のCSVを読み、wp範囲をs範囲へ変換する。
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
    // 設計意図: 1行が壊れていてもnode全体は止めず、残りの有効な区間設定で走れるようにする。
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
      rule.name = columns[0].empty() ? "permission_" + std::to_string(rules.size())
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
  // 処理概要: MPC infeasible回数とsolve timeを抜き出し、planner側の速度guardへ渡す。
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
    mpc_health_ = health;
    last_mpc_health_sec_ = now().seconds();
  }

  // 入力: 現在時刻[sec]。
  // 出力: age_secを更新したMpcHealthStatus。
  // 処理概要: health情報が無い場合はinvalidにし、古さはcore側のguard条件で判断できるようにする。
  MpcHealthStatus currentMpcHealth(double now_sec) const {
    auto health = mpc_health_;
    if (!health.valid || !last_mpc_health_sec_.has_value()) {
      health.valid = false;
      return health;
    }
    health.age_sec = now_sec - last_mpc_health_sec_.value();
    return health;
  }

  // 入力: V2X車両位置配列。
  // 出力: なし。車両IDごとの最新位置と推定速度をsamples_へ保存する。
  // 処理概要: 位置差分からvx/vyを推定し、ジャンプが大きい時は速度を0として外れ値を抑える。
  void updateOpponents(const v2x_msgs::msg::V2XVehiclePositionArray &msg) {
    // 各車両の最新位置を保持し、ジャンプが小さい時だけ速度推定を更新する。
    for (const auto &vehicle : msg.vehicles) {
      const double stamp_sec = stampToSec(vehicle.header.stamp);
      auto &sample = samples_[vehicle.vehicle_id];
      const double x = vehicle.position.x;
      const double y = vehicle.position.y;
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
  // 処理概要: 自車ID、近すぎる点、無効な自車状態を除外し、相手車をFrenet座標つきへ変換する。
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
      opp.valid = now_sec - opp.stamp_sec <= 2.0;
      opponents.push_back(opp);
    }
    return opponents;
  }

  // 入力: 現在時刻、自車、最新MPC health。
  // 出力: 通常ライン復帰ゲートが使える入力完全性。
  // 処理概要: V2Xの受信停止を「相手なし」と扱わず、既知の他車の古いstampもfail-closedにする。
  ReentryInputStatus reentryInputStatus(double now_sec, const EgoState &ego,
                                        const MpcHealthStatus &health) const {
    ReentryInputStatus status;
    const auto fresh = [now_sec](double stamp_sec, double timeout_sec) {
      return std::isfinite(stamp_sec) && std::isfinite(timeout_sec) &&
             timeout_sec > 0.0 && now_sec >= stamp_sec &&
             now_sec - stamp_sec <= timeout_sec;
    };
    status.ego_fresh = ego.valid && fresh(ego.stamp_sec, ego_stale_time_sec_);
    status.v2x_snapshot_fresh =
        last_v2x_snapshot_sec_.has_value() &&
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
      if (std::hypot(sample.x - ego.x, sample.y - ego.y) <
          ignore_near_ego_m_) {
        status.all_observed_opponents_included = false;
      }
    }
    status.reference_valid = !frame_.empty();
    status.mpc_healthy = health.valid &&
                        health.infeasible_count <
                            mpc_health_infeasible_count_threshold_ &&
                        (!std::isfinite(health.solve_time_ms) ||
                         health.solve_time_ms < mpc_health_solve_time_warn_ms_) &&
                        fresh(now_sec - health.age_sec,
                              mpc_health_stale_time_sec_);
    return status;
  }

  // 入力: coreが返したPlannerOutput。
  // 出力: /overtake/reference_override へFloat32MultiArrayをpublishする。
  // 処理概要: MPC側の簡易プロトコルに合わせ、mode、点数、横offset列、速度列を1配列に詰める。
  void publishOverride(const PlannerOutput &output) {
    // Float32MultiArrayの簡易プロトコル: [valid, mode_id, n, d[0..n),
    // v_ref[0..n), contract_version, override_generation]。
    // 末尾2要素を読まない旧consumerは従来どおり先頭部分だけを使える。
    std_msgs::msg::Float32MultiArray msg;
    const int mode_id = static_cast<int>(output.mode);
    const int n = output.active_override
                      ? static_cast<int>(output.lateral_offsets.size())
                      : 0;
    const bool payload_changed =
        output.active_override != last_override_active_ ||
        mode_id != last_override_mode_id_ ||
        output.lateral_offsets != last_override_lateral_offsets_ ||
        output.speed_caps != last_override_speed_caps_;
    if (payload_changed) {
      // Float32が全整数を正確に保持できる範囲に収める。0はinactive/旧形式用。
      override_generation_ = override_generation_ >= 16777215U
                                 ? 1U
                                 : override_generation_ + 1U;
      last_override_active_ = output.active_override;
      last_override_mode_id_ = mode_id;
      last_override_lateral_offsets_ = output.lateral_offsets;
      last_override_speed_caps_ = output.speed_caps;
    }
    msg.data.push_back(1.0F);
    msg.data.push_back(static_cast<float>(mode_id));
    msg.data.push_back(static_cast<float>(n));
    for (int i = 0; i < n; ++i) {
      msg.data.push_back(static_cast<float>(
          output.lateral_offsets[static_cast<std::size_t>(i)]));
    }
    for (int i = 0; i < n; ++i) {
      msg.data.push_back(
          static_cast<float>(output.speed_caps[static_cast<std::size_t>(i)]));
    }
    msg.data.push_back(1.0F);  // contract_version
    msg.data.push_back(static_cast<float>(override_generation_));
    override_pub_->publish(msg);
  }

  // 入力: 今周期のBehaviorMode。
  // 出力: debugで使うattempt_id。追い越し試行外なら0。
  // 処理概要: PREPARE/OVERTAKE開始から復帰/譲り完了まで同じIDを維持する。
  std::uint64_t updateAttemptId(BehaviorMode mode) {
    // evalwrapでattemptを追跡できるよう、追い越し準備開始から復帰完了まで同じIDを出す。
    const bool starts_attempt = mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                                mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
                                mode == BehaviorMode::OVERTAKE_LEFT ||
                                mode == BehaviorMode::OVERTAKE_RIGHT;
    if (!attempt_active_ && starts_attempt) {
      ++current_attempt_id_;
      attempt_active_ = true;
    }

    std::uint64_t publish_id = attempt_active_ ? current_attempt_id_ : 0;
    const bool yield_finishes_attempt =
        mode == BehaviorMode::YIELD_BEHIND &&
        (last_mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         last_mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         last_mode_ == BehaviorMode::OVERTAKE_LEFT ||
         last_mode_ == BehaviorMode::OVERTAKE_RIGHT);
    if (attempt_active_ && (((mode == BehaviorMode::FREE_RUN ||
                              mode == BehaviorMode::FOLLOW_BLOCKED) &&
                             (last_mode_ == BehaviorMode::MERGE_BACK ||
                              last_mode_ == BehaviorMode::ABORT_RECOVERY ||
                              last_mode_ == BehaviorMode::YIELD_BEHIND ||
                              last_mode_ == BehaviorMode::SAFE_STOP)) ||
                            yield_finishes_attempt)) {
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

    std_msgs::msg::String metrics_msg;
    std::ostringstream oss;
    oss << "{"
        << "\"mode\":\"" << toString(output.mode) << "\","
        << "\"overtake_state\":\"" << toString(output.mode) << "\","
        << "\"attempt_id\":" << attempt_id << ","
        << "\"selected\":\"" << toString(output.selected) << "\","
        << "\"blocked\":" << (output.blocked_info.blocked ? "true" : "false")
        << ","
        << "\"side_by_side\":"
        << (output.blocked_info.side_by_side ? "true" : "false") << ","
        << "\"corner_side_by_side\":"
        << (output.blocked_info.corner_side_by_side ? "true" : "false") << ","
        << "\"parallel_side_candidate\":"
        << (output.blocked_info.parallel_side_candidate ? "true" : "false")
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
        << "\"overtake_permission_allowed\":"
        << (output.blocked_info.overtake_permission_allowed ? "true"
                                                            : "false")
        << ","
        << "\"overtake_permission_section_name\":\""
        << output.blocked_info.overtake_permission_section_name << "\","
        << "\"overtake_permission_reason\":\""
        << output.blocked_info.overtake_permission_reason << "\","
        << "\"front_vehicle_low_speed\":"
        << (output.blocked_info.front_vehicle_low_speed ? "true" : "false")
        << ","
        << "\"slow_front_exception_active\":"
        << (output.blocked_info.slow_front_exception_active ? "true"
                                                            : "false")
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
        << "\"target_vehicle_id\":\"" << output.blocked_info.nearest_id << "\","
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
        << "\"leader_priority_active\":"
        << (output.blocked_info.leader_priority_active ? "true" : "false")
        << ","
        << "\"leader_priority_latched\":"
        << (output.blocked_info.leader_priority_latched ? "true" : "false")
        << ","
        << "\"leader_priority_id\":\""
        << output.blocked_info.leader_priority_id << "\","
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
        << (output.blocked_info.pass_left_candidate_feasible ? "true"
                                                              : "false")
        << ","
        << "\"pass_right_candidate_feasible\":"
        << (output.blocked_info.pass_right_candidate_feasible ? "true"
                                                               : "false")
        << ","
        << "\"pass_gap_required_m\":"
        << jsonNumber(output.blocked_info.pass_gap_required_m) << ","
        << "\"pass_decision_frozen\":"
        << (output.blocked_info.pass_decision_frozen ? "true" : "false")
        << ","
        << "\"pass_decision_freeze_reason\":\""
        << output.blocked_info.pass_decision_freeze_reason << "\","
        << "\"pass_gap_reason\":\"" << output.blocked_info.pass_gap_reason
        << "\","
        << "\"ego_x\":" << jsonNumber(ego.x) << ","
        << "\"ego_y\":" << jsonNumber(ego.y) << ","
        << "\"ego_s\":" << jsonNumber(ego.frenet.s) << ","
        << "\"ego_lateral_offset\":" << jsonNumber(ego.frenet.d) << ","
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
        << (output.published_lateral_safety_rejected ? "true" : "false")
        << ","
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
        << "\"reentry_primary_blocker_id\":\""
        << output.reentry_gate.blocking_vehicle_id << "\","
        << "\"reentry_min_safety_margin\":"
        << jsonNumber(output.reentry_gate.min_safety_margin) << ","
        << "\"reentry_cbf_slack\":"
        << jsonNumber(output.reentry_gate.cbf_slack) << ","
        << "\"reentry_blocking_time_sec\":"
        << jsonNumber(output.reentry_gate.blocking_time_sec) << ","
        << "\"lateral_profile_mode\":\"" << output.lateral_profile_mode
        << "\","
        << "\"maneuver_latch_active\":"
        << (output.maneuver_latch_active ? "true" : "false") << ","
        << "\"maneuver_latch_target_id\":\""
        << output.maneuver_latch_target_id << "\","
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
        << jsonNumber(output.blocked_info.front_delta_s) << ","
        << "\"active_override\":" << (output.active_override ? "true" : "false")
        << ","
        << "\"abort_reason\":\""
        << (output.mode == BehaviorMode::ABORT_RECOVERY ? output.reason : "")
        << "\","
        << "\"reason\":\"" << output.reason << "\""
        << "}";
    metrics_msg.data = oss.str();
    metrics_pub_->publish(metrics_msg);
  }

  // 入力: planner coreが返したPlannerOutput。
  // 出力: ログ変化検出用に主要フィールドだけを抜き出したDecisionLogSnapshot。
  // 処理概要: 巨大な出力全体ではなく、判断の変化に効く値だけを比較できる形に詰め替える。
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
    snapshot.future_side_by_side = output.blocked_info.future_side_by_side;
    snapshot.future_corner_side_by_side =
        output.blocked_info.future_corner_side_by_side;
    snapshot.future_yield_required = output.blocked_info.future_yield_required;
    snapshot.future_outer_wall_risk =
        output.blocked_info.future_outer_wall_risk;
    snapshot.can_pass_left = output.blocked_info.can_pass_left;
    snapshot.can_pass_right = output.blocked_info.can_pass_right;
    snapshot.active_override = output.active_override;
    snapshot.corner_abs_curvature = output.blocked_info.corner_abs_curvature;
    snapshot.straight_overtake_start_allowed =
        output.blocked_info.straight_overtake_start_allowed;
    snapshot.overtake_start_abs_curvature =
        output.blocked_info.overtake_start_abs_curvature;
    snapshot.overtake_start_gate_reason =
        output.blocked_info.overtake_start_gate_reason;
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
    snapshot.front_vehicle_id = output.blocked_info.nearest_id;
    snapshot.side_vehicle_id = output.blocked_info.side_id;
    snapshot.parallel_side_vehicle_id = output.blocked_info.parallel_side_id;
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
  // 処理概要: FREE_RUN/FASTESTだけの平常周期を抑制し、閉塞/横並び/safe stopなどの文脈を抽出する。
  bool isInterestingDecisionEvent(const DecisionLogSnapshot &snapshot) const {
    const bool has_pass_gap_context = !snapshot.pass_gap_reason.empty() &&
                                      snapshot.pass_gap_reason != "no_target";
    return snapshot.mode != BehaviorMode::FREE_RUN ||
           snapshot.selected != CandidateType::FASTEST || snapshot.blocked ||
           snapshot.side_by_side || snapshot.corner_side_by_side ||
           snapshot.parallel_side_candidate || snapshot.future_side_by_side ||
           snapshot.future_corner_side_by_side ||
           snapshot.future_yield_required || snapshot.active_override ||
           snapshot.safe_stop_triggered ||
           snapshot.start_grace_active ||
           snapshot.speed_only_fallback_active ||
           snapshot.wall_risk_speed_guard_active ||
           snapshot.mpc_health_speed_guard_active ||
           snapshot.recovery_speed_guard_active ||
           snapshot.reentry_requested ||
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
           snapshot.leader_priority_active || has_pass_gap_context ||
           !snapshot.reason.empty();
  }

  // 入力: 現在周期のDecisionLogSnapshot。
  // 出力: 前回との差分としてログ出力すべきならtrue。
  // 処理概要: 数値は小さな揺れを無視し、状態・理由・対象車両が変わった時だけ記録する。
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
        current.future_side_by_side != previous.future_side_by_side ||
        current.future_corner_side_by_side !=
            previous.future_corner_side_by_side ||
        current.future_yield_required != previous.future_yield_required ||
        current.future_outer_wall_risk != previous.future_outer_wall_risk ||
        current.can_pass_left != previous.can_pass_left ||
        current.can_pass_right != previous.can_pass_right ||
        current.active_override != previous.active_override ||
        std::abs(current.corner_abs_curvature - previous.corner_abs_curvature) >
            0.02 ||
        current.straight_overtake_start_allowed !=
            previous.straight_overtake_start_allowed ||
        std::abs(current.overtake_start_abs_curvature -
                 previous.overtake_start_abs_curvature) > 0.02 ||
        current.overtake_start_gate_reason !=
            previous.overtake_start_gate_reason ||
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
        current.front_vehicle_id != previous.front_vehicle_id ||
        current.side_vehicle_id != previous.side_vehicle_id ||
        current.parallel_side_vehicle_id != previous.parallel_side_vehicle_id ||
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
  // 処理概要: debug topicを見返せない環境でも、mode遷移やsafe stop理由をautoware.logから追えるようにする。
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
        "leader_priority=%d leader_latched=%d leader_id=%s "
        "leader_ds=%.2f leader_reason=%s "
        "can_left=%d can_right=%d pass_gap_reason=%s "
        "corner_abs_curvature=%.3f straight_start_allowed=%d "
        "overtake_start_abs_curvature=%.3f overtake_start_gate_reason=%s "
        "permission_allowed=%d permission_section=%s permission_reason=%s "
        "front_low_speed=%d slow_exception=%d slow_exception_count=%d "
        "slow_chain=%d slow_chain_id=%s front_speed=%.2f "
        "future_wall_clearance=%.2f yield_reason=%s "
        "pass_frozen=%d pass_freeze_reason=%s "
        "ego_s=%.2f ego_d=%.2f target_d=%.2f min_cbf_h=%.3f cbf_slack=%.3f "
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
        output.blocked_info.leader_priority_active,
        output.blocked_info.leader_priority_latched,
        output.blocked_info.leader_priority_id.c_str(),
        output.blocked_info.leader_priority_delta_s,
        output.blocked_info.leader_priority_reason.c_str(),
        output.blocked_info.can_pass_left, output.blocked_info.can_pass_right,
        output.blocked_info.pass_gap_reason.c_str(),
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
        output.blocked_info.future_wall_clearance_m,
        output.blocked_info.yield_reason.c_str(),
        output.blocked_info.pass_decision_frozen,
        output.blocked_info.pass_decision_freeze_reason.c_str(), ego.frenet.s,
        ego.frenet.d,
        output.target_lateral_offset_m, output.min_cbf_h, output.cbf_slack,
        output.safe_stop_triggered, output.start_grace_active,
        output.safe_stop_reason.c_str(), output.safe_stop_reject_reason.c_str(),
        output.safe_stop_trigger_count, output.safe_stop_hold_count,
        output.safe_stop_release_count,
        output.safe_stop_release_ready, output.speed_only_fallback_active,
        output.wall_risk_speed_guard_active,
        output.mpc_health_speed_guard_active,
        output.recovery_speed_guard_active,
        output.reentry_gate.requested, output.reentry_gate.permitted,
        output.reentry_gate.clear_cycles, output.reentry_gate.reason.c_str(),
        output.reentry_gate.blocking_vehicle_id.c_str(),
        output.speed_cap_reason.c_str(),
        output.applied_speed_cap_mps, output.active_section.name.c_str(),
        output.active_section.profile.c_str(),
        output.mpc_health.infeasible_count, output.mpc_health.solve_time_ms,
        output.reason.c_str());
  }

  // 入力: ROS timer周期。
  // 出力: override topic、mode topic、debug metrics、必要に応じた判断ログ。
  // 処理概要: odomをFrenet自車状態へ変換し、相手車収集からcore更新、publishまでを1周期で実行する。
  void onTimer() {
    // 最新odomを自車状態へ変換し、他車収集 -> コア更新 -> override/debug
    // publishを1周期で行う。
    const double now_sec = now().seconds();
    EgoState ego;
    if (odom_.has_value() && !frame_.empty()) {
      const auto &odom = *odom_;
      ego.stamp_sec = stampToSec(odom.header.stamp);
      if (now_sec - ego.stamp_sec <= ego_stale_time_sec_) {
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
    const auto output =
        core_->update(now_sec, ego, opponents, mpc_health, reentry_input);
    const auto attempt_id = updateAttemptId(output.mode);
    publishOverride(output);
    publishDebug(output, ego, attempt_id);
    logDecisionEvent(output, ego, attempt_id);
  }

  FrenetFrame frame_;
  std::unique_ptr<OvertakePlannerCore> core_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::unordered_map<std::string, TrackSample> samples_;
  std::string own_vehicle_id_;
  double ignore_near_ego_m_{1.0};
  double position_jump_threshold_m_{5.0};
  double ego_stale_time_sec_{0.50};
  double opponent_stale_time_sec_{0.50};
  double reentry_v2x_snapshot_stale_time_sec_{0.50};
  double control_rate_hz_{20.0};
  double mpc_health_stale_time_sec_{0.60};
  int mpc_health_infeasible_count_threshold_{1};
  double mpc_health_solve_time_warn_ms_{80.0};
  MpcHealthStatus mpc_health_{};
  std::optional<double> last_mpc_health_sec_;
  std::optional<double> last_v2x_snapshot_sec_;
  BehaviorMode last_mode_{BehaviorMode::FREE_RUN};
  DecisionLogSnapshot last_decision_log_snapshot_{};
  bool has_decision_log_snapshot_{false};
  bool attempt_active_{false};
  std::uint64_t current_attempt_id_{0};
  bool last_override_active_{false};
  int last_override_mode_id_{0};
  std::vector<double> last_override_lateral_offsets_;
  std::vector<double> last_override_speed_caps_;
  std::uint32_t override_generation_{0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr override_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr
      v2x_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpc_health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace overtake_planner

// 入力: ROS 2プロセス引数。
// 出力: 終了コード。正常終了時は0。
// 処理概要: OvertakePlannerNodeを生成してspinし、シャットダウン時にrclcppを閉じる。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<overtake_planner::OvertakePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
