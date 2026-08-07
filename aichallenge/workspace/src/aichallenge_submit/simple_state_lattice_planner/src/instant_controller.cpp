#include "simple_state_lattice_planner/instant_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace simple_state_lattice_planner {
namespace {

bool finite(double value) { return std::isfinite(value); }

double normalizeAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

bool validConfig(const InstantControllerConfig &config) {
  return finite(config.wheel_base_m) && finite(config.lookahead_min_m) &&
         finite(config.lookahead_gain_sec) &&
         finite(config.maximum_steering_angle_rad) &&
         finite(config.maximum_steering_rate_radps) &&
         finite(config.speed_proportional_gain) &&
         finite(config.minimum_acceleration_mps2) &&
         finite(config.maximum_acceleration_mps2) &&
         finite(config.maximum_dt_sec) && config.wheel_base_m > 0.0 &&
         config.lookahead_min_m > 0.0 && config.lookahead_gain_sec >= 0.0 &&
         config.maximum_steering_angle_rad > 0.0 &&
         config.maximum_steering_rate_radps > 0.0 &&
         config.speed_proportional_gain >= 0.0 &&
         config.minimum_acceleration_mps2 <= 0.0 &&
         config.maximum_acceleration_mps2 >= 0.0 &&
         config.minimum_acceleration_mps2 <= config.maximum_acceleration_mps2 &&
         config.maximum_dt_sec > 0.0;
}

}  // namespace

InstantController::InstantController(InstantControllerConfig config)
    : config_(config) {}

InstantControlResult InstantController::invalid(const char *reason) {
  reset();
  InstantControlResult result;
  result.acceleration_mps2 = std::min(config_.minimum_acceleration_mps2, 0.0);
  result.reason = reason;
  return result;
}

InstantControlResult InstantController::update(const InstantControlInput &input) {
  if (!validConfig(config_)) {
    return invalid("config_invalid");
  }
  if (input.ego == nullptr || input.selected_candidate == nullptr ||
      !input.inputs_fresh || !input.planner_output_fresh ||
      input.planner_snapshot_id == 0U || input.monotonic_now_ns <= 0) {
    return invalid("input_invalid_or_stale");
  }
  const auto &ego = *input.ego;
  const auto &candidate = *input.selected_candidate;
  if (ego.snapshot_id != input.planner_snapshot_id ||
      candidate.snapshot_id != input.planner_snapshot_id ||
      !finite(ego.x_m) || !finite(ego.y_m) || !finite(ego.yaw_rad) ||
      !finite(ego.speed_mps) || ego.speed_mps < 0.0 ||
      candidate.points.size() < 2U) {
    return invalid("snapshot_or_ego_invalid");
  }

  std::size_t nearest = 0U;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
    const auto &point = candidate.points[index];
    if (!finite(point.x_m) || !finite(point.y_m) || !finite(point.yaw_rad) ||
        !finite(point.speed_mps) || point.speed_mps < 0.0) {
      return invalid("trajectory_invalid");
    }
    const double dx = point.x_m - ego.x_m;
    const double dy = point.y_m - ego.y_m;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = index;
    }
  }

  const double lookahead_m =
      std::max(config_.lookahead_min_m,
               config_.lookahead_gain_sec * ego.speed_mps);
  std::size_t target = nearest;
  double available_arc_m = 0.0;
  while (target + 1U < candidate.points.size() &&
         available_arc_m < lookahead_m) {
    available_arc_m +=
        std::hypot(candidate.points[target + 1U].x_m -
                       candidate.points[target].x_m,
                   candidate.points[target + 1U].y_m -
                       candidate.points[target].y_m);
    ++target;
  }
  if (!finite(available_arc_m) || available_arc_m + 1.0e-9 < lookahead_m) {
    return invalid("required_arc_unavailable");
  }
  const auto &target_point = candidate.points[target];
  const double dx = target_point.x_m - ego.x_m;
  const double dy = target_point.y_m - ego.y_m;
  const double target_distance_m = std::hypot(dx, dy);
  if (!finite(target_distance_m) || target_distance_m < 1.0e-3) {
    return invalid("lookahead_degenerate");
  }
  const double alpha = normalizeAngle(std::atan2(dy, dx) - ego.yaw_rad);
  const double raw_steering_rad = std::atan2(
      2.0 * config_.wheel_base_m * std::sin(alpha), target_distance_m);
  if (!finite(raw_steering_rad) ||
      std::abs(raw_steering_rad) > config_.maximum_steering_angle_rad) {
    return invalid("steering_angle_untrackable");
  }

  double dt_sec = 0.05;
  double previous_steering_rad = 0.0;
  if (initialized_) {
    const std::int64_t delta_ns = input.monotonic_now_ns - last_monotonic_ns_;
    dt_sec = static_cast<double>(delta_ns) * 1.0e-9;
    previous_steering_rad = last_steering_rad_;
    if (!finite(dt_sec) || dt_sec <= 0.0 || dt_sec > config_.maximum_dt_sec) {
      return invalid("controller_time_invalid");
    }
  }
  const double maximum_delta_rad = config_.maximum_steering_rate_radps * dt_sec;
  const double steering_rad =
      std::clamp(raw_steering_rad, previous_steering_rad - maximum_delta_rad,
                 previous_steering_rad + maximum_delta_rad);
  const double steering_rate_radps =
      (steering_rad - previous_steering_rad) / dt_sec;
  const double acceleration_mps2 = std::clamp(
      config_.speed_proportional_gain *
          (target_point.speed_mps - ego.speed_mps),
      config_.minimum_acceleration_mps2,
      config_.maximum_acceleration_mps2);
  if (!finite(steering_rad) || !finite(steering_rate_radps) ||
      !finite(acceleration_mps2)) {
    return invalid("output_non_finite");
  }

  initialized_ = true;
  last_steering_rad_ = steering_rad;
  last_monotonic_ns_ = input.monotonic_now_ns;
  InstantControlResult result;
  result.valid = true;
  result.stop_required = false;
  result.speed_mps = target_point.speed_mps;
  result.acceleration_mps2 = acceleration_mps2;
  result.steering_angle_rad = steering_rad;
  result.steering_rate_radps = steering_rate_radps;
  result.reason = "valid";
  return result;
}

void InstantController::reset() {
  initialized_ = false;
  last_steering_rad_ = 0.0;
  last_monotonic_ns_ = 0;
}

}  // namespace simple_state_lattice_planner
