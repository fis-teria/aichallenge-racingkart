#include "simple_state_lattice_planner/candidate_evaluator.hpp"

#include <cmath>

namespace simple_state_lattice_planner {
namespace {

bool finite(double value) { return std::isfinite(value); }

Candidate rejected(Candidate candidate, RejectReason reason) {
  candidate.valid = false;
  candidate.reject_reason = reason;
  candidate.total_cost = 0.0;
  return candidate;
}

bool validConfig(const CandidateEvaluatorConfig &config) {
  return finite(config.wheel_base_m) &&
         finite(config.maximum_curvature_radpm) &&
         finite(config.maximum_steering_angle_rad) &&
         finite(config.maximum_steering_rate_radps) &&
         finite(config.minimum_rate_evaluation_speed_mps) &&
         finite(config.maximum_segment_length_m) &&
         finite(config.obstacle_cost_weight) &&
         finite(config.reference_deviation_weight) &&
         finite(config.curvature_weight) &&
         finite(config.steering_change_weight) && config.wheel_base_m > 0.0 &&
         config.maximum_curvature_radpm > 0.0 &&
         config.maximum_steering_angle_rad > 0.0 &&
         config.maximum_steering_rate_radps > 0.0 &&
         config.minimum_rate_evaluation_speed_mps > 0.0 &&
         config.maximum_segment_length_m > 0.0 &&
         config.maximum_segment_length_m <= 0.05 &&
         config.obstacle_cost_weight >= 0.0 &&
         config.reference_deviation_weight >= 0.0 &&
         config.curvature_weight >= 0.0 &&
         config.steering_change_weight >= 0.0;
}

}  // namespace

Candidate evaluateCandidate(Candidate candidate, const Costmap2D &costmap,
                            const CandidateEvaluatorConfig &config) {
  if (!costmap.valid()) {
    return rejected(std::move(candidate), RejectReason::INVALID_COSTMAP);
  }
  if (!validConfig(config)) {
    return rejected(std::move(candidate), RejectReason::INVALID_EVALUATOR_CONFIG);
  }
  if (candidate.snapshot_id == 0U ||
      candidate.snapshot_id != costmap.snapshot_id) {
    return rejected(std::move(candidate), RejectReason::SNAPSHOT_MISMATCH);
  }
  if (candidate.points.size() < 2U) {
    return rejected(std::move(candidate), RejectReason::TOO_FEW_POINTS);
  }
  double total_cost = 0.0;
  double previous_s_m = candidate.points.front().s_m - 1.0;
  double previous_steering_rad = 0.0;
  for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
    const auto &point = candidate.points[index];
    if (!finite(point.x_m) || !finite(point.y_m) || !finite(point.yaw_rad) ||
        !finite(point.curvature_radpm) || !finite(point.s_m) ||
        !finite(point.d_m) || !finite(point.speed_mps)) {
      return rejected(std::move(candidate), RejectReason::NON_FINITE_POINT);
    }
    if (point.speed_mps < 0.0) {
      return rejected(std::move(candidate), RejectReason::NEGATIVE_SPEED);
    }
    if (point.s_m <= previous_s_m) {
      return rejected(std::move(candidate), RejectReason::REVERSE_S);
    }
    previous_s_m = point.s_m;
    std::uint8_t maximum_cell_cost = costmap.costAt(point.x_m, point.y_m);
    double segment_length_m = 0.0;
    if (index > 0U) {
      const auto &previous = candidate.points[index - 1U];
      segment_length_m = std::hypot(point.x_m - previous.x_m,
                                    point.y_m - previous.y_m);
      if (!finite(segment_length_m)) {
        return rejected(std::move(candidate), RejectReason::NON_FINITE_POINT);
      }
      const std::size_t pieces = static_cast<std::size_t>(
          std::max(1.0, std::ceil(segment_length_m /
                                 config.maximum_segment_length_m)));
      for (std::size_t piece = 1U; piece <= pieces; ++piece) {
        const double ratio = static_cast<double>(piece) /
                             static_cast<double>(pieces);
        const double x_m = previous.x_m + ratio * (point.x_m - previous.x_m);
        const double y_m = previous.y_m + ratio * (point.y_m - previous.y_m);
        maximum_cell_cost =
            std::max(maximum_cell_cost, costmap.costAt(x_m, y_m));
      }
    }
    if (maximum_cell_cost >= kOccupiedCost) {
      return rejected(std::move(candidate), RejectReason::COSTMAP_OCCUPIED);
    }
    const double curvature = std::abs(point.curvature_radpm);
    if (curvature > config.maximum_curvature_radpm) {
      return rejected(std::move(candidate), RejectReason::CURVATURE_LIMIT);
    }
    const double steering_rad =
        std::atan(config.wheel_base_m * point.curvature_radpm);
    if (!finite(steering_rad) ||
        std::abs(steering_rad) > config.maximum_steering_angle_rad) {
      return rejected(std::move(candidate), RejectReason::STEERING_ANGLE_LIMIT);
    }
    if (index > 0U) {
      const double rate_dt_sec =
          segment_length_m /
          std::max(config.minimum_rate_evaluation_speed_mps, point.speed_mps);
      if (!finite(rate_dt_sec) || rate_dt_sec <= 0.0 ||
          std::abs(steering_rad - previous_steering_rad) /
                  rate_dt_sec >
              config.maximum_steering_rate_radps) {
        return rejected(std::move(candidate), RejectReason::STEERING_RATE_LIMIT);
      }
    }
    total_cost += config.obstacle_cost_weight *
                      (static_cast<double>(maximum_cell_cost) / 100.0) +
                  config.reference_deviation_weight * std::abs(point.d_m) +
                  config.curvature_weight * curvature +
                  config.steering_change_weight *
                      std::abs(steering_rad - previous_steering_rad);
    previous_steering_rad = steering_rad;
  }
  if (!finite(total_cost)) {
    return rejected(std::move(candidate), RejectReason::COST_OVERFLOW);
  }
  candidate.valid = true;
  candidate.reject_reason = RejectReason::NONE;
  candidate.total_cost = total_cost;
  return candidate;
}

}  // namespace simple_state_lattice_planner
