#include "state_lattice_overtake_planner/instant_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace state_lattice_overtake_planner {

InstantController::InstantController(InstantControllerConfig config)
    : config_(config) {}

InstantControlResult
InstantController::update(const EgoState &ego,
                          const std::vector<TrajectoryPoint> &trajectory,
                          double now_sec) {
  InstantControlResult result;
  if (!ego.valid || !std::isfinite(ego.x) || !std::isfinite(ego.y) ||
      !std::isfinite(ego.yaw) || !std::isfinite(ego.speed_mps) ||
      !std::isfinite(now_sec) || trajectory.size() < 2U) {
    result.reason = "input_invalid";
    return result;
  }

  std::size_t nearest_index = 0U;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < trajectory.size(); ++index) {
    const auto &point = trajectory[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.speed_mps)) {
      result.reason = "trajectory_non_finite";
      return result;
    }
    const double dx = point.x - ego.x;
    const double dy = point.y - ego.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_index = index;
    }
  }

  const double lookahead_m =
      std::max(config_.lookahead_min_m,
               config_.lookahead_gain_sec * std::max(0.0, ego.speed_mps));
  std::size_t target_index = nearest_index;
  double available_arc_m = 0.0;
  while (target_index + 1U < trajectory.size() &&
         available_arc_m < lookahead_m) {
    const auto &from = trajectory[target_index];
    const auto &to = trajectory[target_index + 1U];
    available_arc_m += std::hypot(to.x - from.x, to.y - from.y);
    ++target_index;
  }
  if (!std::isfinite(available_arc_m) ||
      available_arc_m + 1.0e-6 < lookahead_m) {
    result.reason = "required_arc_unavailable";
    return result;
  }

  const auto &target = trajectory[target_index];
  const double dx = target.x - ego.x;
  const double dy = target.y - ego.y;
  const double target_distance_m = std::hypot(dx, dy);
  if (!std::isfinite(target_distance_m) || target_distance_m < 1.0e-3) {
    result.reason = "lookahead_degenerate";
    return result;
  }
  const double alpha = normalizeAngle(std::atan2(dy, dx) - ego.yaw);
  const double raw_steering_rad = std::atan2(
      2.0 * config_.wheel_base_m * std::sin(alpha), target_distance_m);
  if (!std::isfinite(raw_steering_rad) ||
      std::abs(raw_steering_rad) >
          config_.maximum_steering_angle_rad + 1.0e-9) {
    result.reason = "steering_angle_untrackable";
    return result;
  }

  double previous_steering_rad = last_steering_rad_;
  double dt_sec = now_sec - last_time_sec_;
  if (!initialized_) {
    previous_steering_rad =
        std::clamp(std::atan(config_.wheel_base_m * ego.curvature),
                   -config_.maximum_steering_angle_rad,
                   config_.maximum_steering_angle_rad);
    dt_sec = 1.0 / 20.0;
  }
  if (!std::isfinite(previous_steering_rad) || !std::isfinite(dt_sec) ||
      dt_sec <= 0.0 || dt_sec > 0.20) {
    result.reason = "controller_time_invalid";
    reset();
    return result;
  }
  const double maximum_delta_rad = config_.maximum_steering_rate_radps * dt_sec;
  const double steering_rad =
      std::clamp(raw_steering_rad, previous_steering_rad - maximum_delta_rad,
                 previous_steering_rad + maximum_delta_rad);
  const double steering_rate_radps =
      (steering_rad - previous_steering_rad) / dt_sec;

  const double target_speed_mps = std::max(0.0, target.speed_mps);
  const double acceleration_mps2 = std::clamp(
      config_.speed_proportional_gain * (target_speed_mps - ego.speed_mps),
      config_.minimum_acceleration_mps2, config_.maximum_acceleration_mps2);
  if (!std::isfinite(target_speed_mps) || !std::isfinite(acceleration_mps2) ||
      !std::isfinite(steering_rad) || !std::isfinite(steering_rate_radps)) {
    result.reason = "output_non_finite";
    return result;
  }

  initialized_ = true;
  last_steering_rad_ = steering_rad;
  last_time_sec_ = now_sec;
  result.valid = true;
  result.speed_mps = target_speed_mps;
  result.acceleration_mps2 = acceleration_mps2;
  result.steering_angle_rad = steering_rad;
  result.steering_rate_radps = steering_rate_radps;
  result.reason = "valid";
  return result;
}

void InstantController::reset() {
  initialized_ = false;
  last_steering_rad_ = 0.0;
  last_time_sec_ = 0.0;
}

} // namespace state_lattice_overtake_planner
