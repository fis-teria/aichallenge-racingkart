#ifndef SIMPLE_PURE_PURSUIT_SAFETY_HPP_
#define SIMPLE_PURE_PURSUIT_SAFETY_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string>

namespace simple_pure_pursuit {

struct FreshnessAges {
  double odom_age_sec{-1.0};
  double trajectory_age_sec{-1.0};
  double override_age_sec{-1.0};
};

struct FreshnessResult {
  bool fresh{false};
  std::string reason{"missing_input"};
  FreshnessAges ages{};
};

struct HorizonFreshnessResult {
  bool usable{false};
  std::string reason{"disabled"};
  double age_sec{-1.0};
  std::size_t point_count{0};
  double start_distance_m{-1.0};
  double arc_length_m{-1.0};
};

inline double inputAgeSec(std::optional<double> receive_time_sec,
                          double now_sec);
inline bool ageFresh(double age_sec, double max_age_sec);

// MPC horizonの内容とplanner overrideの世代が同じかを、ROS依存なしで判定する。
// 不一致時はhorizonを使わず、既存の通常trajectory + override fallbackへ戻す。
struct HorizonContractResult {
  bool usable{false};
  std::string reason{"mpc_horizon_contract_missing"};
};

inline HorizonContractResult evaluateMpcHorizonContract(
    bool required, bool override_active, int override_mode_id,
    std::uint32_t override_generation, bool metadata_received,
    std::optional<double> metadata_receive_sec, double now_sec,
    double max_age_sec, bool stamp_matches, const std::string &source,
    int contract_mode_id, std::uint32_t contract_generation) {
  HorizonContractResult result;
  if (!required) {
    result.usable = true;
    result.reason = "fresh";
    return result;
  }
  if (!metadata_received || !metadata_receive_sec.has_value()) {
    return result;
  }
  if (!ageFresh(inputAgeSec(metadata_receive_sec, now_sec), max_age_sec)) {
    result.reason = "mpc_horizon_contract_stale";
    return result;
  }
  if (!stamp_matches) {
    result.reason = "mpc_horizon_contract_stamp_mismatch";
    return result;
  }
  if (!override_active) {
    if (contract_mode_id != 0 || contract_generation != 0U) {
      result.reason = "mpc_horizon_contract_inactive_nonzero";
      return result;
    }
    result.usable = true;
    result.reason = "fresh";
    return result;
  }
  if (source != "solver_prediction") {
    result.reason = "mpc_horizon_contract_source";
    return result;
  }
  if (override_mode_id <= 0 || contract_mode_id != override_mode_id) {
    result.reason = "mpc_horizon_contract_mode_mismatch";
    return result;
  }
  if (override_generation == 0U ||
      contract_generation != override_generation) {
    result.reason = "mpc_horizon_contract_generation_mismatch";
    return result;
  }
  result.usable = true;
  result.reason = "fresh";
  return result;
}

// 入力: 既存軌道からの目標速度と、追い越しplannerが出した任意の速度cap。
// 出力: capを超えない目標速度。無効なcapは停止要求として0 m/sへ倒す。
// 処理概要: planner overrideは各control周期でindex 0を即時に消費する契約を、
// ROS node本体と単体試験で共通化する。
inline double applyOvertakeSpeedCap(double target_speed_mps,
                                    std::optional<double> speed_cap_mps) {
  if (!speed_cap_mps.has_value()) {
    return target_speed_mps;
  }
  if (!std::isfinite(speed_cap_mps.value())) {
    return 0.0;
  }
  return std::min(target_speed_mps, std::max(0.0, speed_cap_mps.value()));
}

// 入力: 目標速度、現在速度、既存Pゲイン。
// 出力: Pure Pursuitが出す縦加速度要求[m/s^2]。
// 処理概要: override capが現在速度より低ければ、同周期から負の加速度要求になる
// ことを明示する。最終的なactuator/MPC制限は既存の下流safety clampが担う。
inline double proportionalLongitudinalAcceleration(double target_speed_mps,
                                                    double current_speed_mps,
                                                    double gain) {
  return gain * (target_speed_mps - current_speed_mps);
}

inline double inputAgeSec(std::optional<double> receive_time_sec,
                          double now_sec) {
  if (!receive_time_sec.has_value() || !std::isfinite(now_sec) ||
      !std::isfinite(receive_time_sec.value())) {
    return -1.0;
  }
  return now_sec - receive_time_sec.value();
}

inline bool ageFresh(double age_sec, double max_age_sec) {
  if (!std::isfinite(max_age_sec) || max_age_sec < 0.0) {
    return true;
  }
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= max_age_sec;
}

inline FreshnessResult evaluateRequiredInputFreshness(
    std::optional<double> odom_receive_sec,
    std::optional<double> trajectory_receive_sec, double now_sec,
    double max_odom_age_sec, double max_trajectory_age_sec) {
  FreshnessResult result;
  result.ages.odom_age_sec = inputAgeSec(odom_receive_sec, now_sec);
  result.ages.trajectory_age_sec =
      inputAgeSec(trajectory_receive_sec, now_sec);

  if (!odom_receive_sec.has_value()) {
    result.reason = "missing_odom";
    return result;
  }
  if (!trajectory_receive_sec.has_value()) {
    result.reason = "missing_trajectory";
    return result;
  }
  if (!ageFresh(result.ages.odom_age_sec, max_odom_age_sec)) {
    result.reason = "stale_odom";
    return result;
  }
  if (!ageFresh(result.ages.trajectory_age_sec, max_trajectory_age_sec)) {
    result.reason = "stale_trajectory";
    return result;
  }

  result.fresh = true;
  result.reason = "fresh";
  return result;
}

inline HorizonFreshnessResult evaluateMpcHorizonFreshness(
    bool enabled, std::optional<double> horizon_receive_sec, double now_sec,
    double max_age_sec, std::size_t point_count, std::size_t min_points,
    double start_distance_m, double max_start_distance_m, double arc_length_m,
    double min_arc_length_m, bool values_finite, bool frame_valid) {
  HorizonFreshnessResult result;
  result.age_sec = inputAgeSec(horizon_receive_sec, now_sec);
  result.point_count = point_count;
  result.start_distance_m = start_distance_m;
  result.arc_length_m = arc_length_m;

  if (!enabled) {
    result.reason = "disabled";
    return result;
  }
  if (!horizon_receive_sec.has_value()) {
    result.reason = "missing";
    return result;
  }
  if (!ageFresh(result.age_sec, max_age_sec)) {
    result.reason = "stale";
    return result;
  }
  if (point_count == 0) {
    result.reason = "empty";
    return result;
  }
  if (point_count < min_points) {
    result.reason = "short";
    return result;
  }
  if (!frame_valid) {
    result.reason = "frame_mismatch";
    return result;
  }
  if (!values_finite) {
    result.reason = "nonfinite";
    return result;
  }
  if (std::isfinite(max_start_distance_m) && max_start_distance_m >= 0.0 &&
      (!std::isfinite(start_distance_m) ||
       start_distance_m > max_start_distance_m)) {
    result.reason = "start_distance";
    return result;
  }
  if (std::isfinite(min_arc_length_m) && min_arc_length_m > 0.0 &&
      (!std::isfinite(arc_length_m) || arc_length_m < min_arc_length_m)) {
    result.reason = "short_arc";
    return result;
  }

  result.usable = true;
  result.reason = "fresh";
  return result;
}

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_SAFETY_HPP_
