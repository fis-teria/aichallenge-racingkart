#ifndef SIMPLE_PURE_PURSUIT_SAFETY_HPP_
#define SIMPLE_PURE_PURSUIT_SAFETY_HPP_

#include <cstddef>
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
