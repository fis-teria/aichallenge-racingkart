#include "state_lattice_overtake_planner/lattice_planner.hpp"

#include "state_lattice_overtake_planner/cost_model.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <string_view>
#include <tuple>
#include <utility>

namespace state_lattice_overtake_planner {
namespace {
double lerp(double a, double b, double ratio) { return a + (b - a) * ratio; }

double effectiveMaximumSteer(const PlannerConfig &config) {
  return std::min(config.planner_max_steer_rad, config.hard_max_steer_rad);
}

double maximumPlannerCurvature(const PlannerConfig &config) {
  return std::abs(
      curvatureForSteering(effectiveMaximumSteer(config), config.wheel_base_m));
}

double sanitizedCurvature(double curvature, const PlannerConfig &config) {
  if (!std::isfinite(curvature)) {
    return 0.0;
  }
  const double limit = std::min(maximumPlannerCurvature(config),
                                config.reference_curvature_sanity_limit_radpm);
  return std::clamp(curvature, -limit, limit);
}

double referenceSpeedLimit(const FrenetFrame *frame, double s,
                           const PlannerConfig &config) {
  if (!config.reference_speed_limit_enabled || frame == nullptr ||
      frame->empty()) {
    return config.normal_speed_mps;
  }
  const double speed = frame->interpolate(s).speed_mps;
  return std::isfinite(speed) && speed > 0.0
             ? std::min(speed, config.normal_speed_mps)
             : config.normal_speed_mps;
}

std::optional<double>
recedingHorizonSpeedCap(const std::vector<double> &speed_caps_mps,
                        const std::vector<double> &longitudinal_offsets_m) {
  if (speed_caps_mps.empty() ||
      speed_caps_mps.size() != longitudinal_offsets_m.size()) {
    return std::nullopt;
  }
  double previous_offset_m = -1.0e-9;
  for (std::size_t i = 0; i < speed_caps_mps.size(); ++i) {
    const double speed_mps = speed_caps_mps[i];
    const double offset_m = longitudinal_offsets_m[i];
    if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
        !std::isfinite(offset_m) || offset_m < previous_offset_m) {
      return std::nullopt;
    }
    previous_offset_m = offset_m;
    // outputHorizon() already applies the backward braking and forward
    // acceleration passes. Index zero is tied to measured ego speed, so use
    // the first point ahead as this receding-horizon cycle's scalar target.
    if (offset_m > 1.0e-6) {
      return speed_mps;
    }
  }
  return std::nullopt;
}

double requiredPassLateralSeparation(const PlannerConfig &config,
                                     const OpponentState &opponent,
                                     bool ego_on_left) {
  const double ego_inner_extent =
      ego_on_left ? config.right_extent_m : config.left_extent_m;
  const double opponent_inner_extent =
      ego_on_left ? config.left_extent_m : config.right_extent_m;
  return ego_inner_extent + config.opponent_lateral_tracking_margin_m +
         opponent_inner_extent + opponent.uncertainty_y_m +
         config.opponent_hard_clearance_m + config.pass_lateral_extra_margin_m;
}

double
requiredPreObstacleLongitudinalSeparation(const PlannerConfig &config,
                                          const OpponentState &opponent) {
  return config.wheel_base_m + config.front_overhang_m +
         config.opponent_longitudinal_tracking_margin_m +
         config.rear_overhang_m + opponent.uncertainty_x_m +
         config.opponent_hard_clearance_m + config.pass_lateral_extra_margin_m;
}

bool passCapableGoal(double goal_d_m, const OpponentState &opponent,
                     const PlannerConfig &config) {
  const double left_clearance =
      requiredPassLateralSeparation(config, opponent, true);
  const double right_clearance =
      requiredPassLateralSeparation(config, opponent, false);
  return goal_d_m - opponent.frenet.d >= left_clearance - 1.0e-6 ||
         opponent.frenet.d - goal_d_m >= right_clearance - 1.0e-6;
}

PassClearanceDiagnostic
evaluatePassClearanceDiagnostic(double goal_d_m, const OpponentState &opponent,
                                const PlannerConfig &config) {
  PassClearanceDiagnostic diagnostic;
  const std::size_t target_id_size =
      std::min(opponent.id.size(), diagnostic.target_id.size() - 1U);
  std::copy_n(opponent.id.data(), target_id_size, diagnostic.target_id.data());
  diagnostic.target_d_m = opponent.frenet.d;
  diagnostic.target_uncertainty_y_m = opponent.uncertainty_y_m;
  diagnostic.target_observation_stamp_sec = opponent.stamp_sec;
  diagnostic.candidate_goal_d_m = goal_d_m;
  const double left_clearance =
      requiredPassLateralSeparation(config, opponent, true);
  const double right_clearance =
      requiredPassLateralSeparation(config, opponent, false);
  const double left_separation = goal_d_m - opponent.frenet.d;
  const double right_separation = opponent.frenet.d - goal_d_m;
  if (left_separation > 0.0) {
    diagnostic.side = 1;
    diagnostic.actual_separation_m = left_separation;
    diagnostic.required_separation_m = left_clearance;
  } else if (right_separation > 0.0) {
    diagnostic.side = -1;
    diagnostic.actual_separation_m = right_separation;
    diagnostic.required_separation_m = right_clearance;
  }
  diagnostic.margin_m =
      diagnostic.actual_separation_m - diagnostic.required_separation_m;
  diagnostic.observed_predicate = passCapableGoal(goal_d_m, opponent, config);
  diagnostic.evaluated = true;
  diagnostic.inputs_valid = opponent.valid && std::isfinite(goal_d_m) &&
                            std::isfinite(opponent.frenet.d) &&
                            std::isfinite(opponent.uncertainty_y_m) &&
                            std::isfinite(left_clearance) &&
                            std::isfinite(right_clearance);
  if (!diagnostic.inputs_valid) {
    diagnostic.target_d_m = std::numeric_limits<double>::quiet_NaN();
    diagnostic.target_uncertainty_y_m =
        std::numeric_limits<double>::quiet_NaN();
    diagnostic.target_observation_stamp_sec =
        std::numeric_limits<double>::quiet_NaN();
    diagnostic.candidate_goal_d_m = std::numeric_limits<double>::quiet_NaN();
    diagnostic.side = 0;
    diagnostic.actual_separation_m = std::numeric_limits<double>::quiet_NaN();
    diagnostic.required_separation_m = std::numeric_limits<double>::quiet_NaN();
    diagnostic.margin_m = std::numeric_limits<double>::quiet_NaN();
  }
  return diagnostic;
}

double timeAtU(const std::vector<TrajectoryPoint> &points, double u) {
  if (points.empty()) {
    return 0.0;
  }
  if (u <= points.front().u) {
    return points.front().time_sec;
  }
  const auto upper =
      std::upper_bound(points.begin(), points.end(), u,
                       [](double value, const TrajectoryPoint &point) {
                         return value < point.u;
                       });
  if (upper == points.end()) {
    return points.back().time_sec;
  }
  const auto &a = *(upper - 1);
  const auto &b = *upper;
  const double ratio = (u - a.u) / std::max(1.0e-9, b.u - a.u);
  return lerp(a.time_sec, b.time_sec, ratio);
}

double sAtU(const std::vector<TrajectoryPoint> &points, double u) {
  if (points.empty()) {
    return 0.0;
  }
  if (u <= points.front().u) {
    return points.front().s;
  }
  const auto upper =
      std::upper_bound(points.begin(), points.end(), u,
                       [](double value, const TrajectoryPoint &point) {
                         return value < point.u;
                       });
  if (upper == points.end()) {
    return points.back().s;
  }
  const auto &a = *(upper - 1);
  const auto &b = *upper;
  const double ratio = (u - a.u) / std::max(1.0e-9, b.u - a.u);
  return lerp(a.s, b.s, ratio);
}

double interpolateProfile(const std::vector<TrajectoryPoint> &points, double s,
                          bool speed) {
  if (points.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (s <= points.front().s) {
    return speed ? points.front().speed_mps : points.front().d;
  }
  auto upper = std::upper_bound(points.begin(), points.end(), s,
                                [](double value, const TrajectoryPoint &point) {
                                  return value < point.s;
                                });
  if (upper == points.end()) {
    return speed ? points.back().speed_mps : points.back().d;
  }
  const auto &a = *(upper - 1);
  const auto &b = *upper;
  const double ratio = (s - a.s) / std::max(1.0e-9, b.s - a.s);
  return speed ? lerp(a.speed_mps, b.speed_mps, ratio) : lerp(a.d, b.d, ratio);
}

bool finitePoint(const TrajectoryPoint &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.yaw) && std::isfinite(point.kappa);
}

double orientation(const TrajectoryPoint &a, const TrajectoryPoint &b,
                   const TrajectoryPoint &c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool properSegmentIntersection(const TrajectoryPoint &a,
                               const TrajectoryPoint &b,
                               const TrajectoryPoint &c,
                               const TrajectoryPoint &d) {
  constexpr double epsilon = 1.0e-10;
  const double ab_c = orientation(a, b, c);
  const double ab_d = orientation(a, b, d);
  const double cd_a = orientation(c, d, a);
  const double cd_b = orientation(c, d, b);
  return ((ab_c > epsilon && ab_d < -epsilon) ||
          (ab_c < -epsilon && ab_d > epsilon)) &&
         ((cd_a > epsilon && cd_b < -epsilon) ||
          (cd_a < -epsilon && cd_b > epsilon));
}

bool hasSelfIntersection(const std::vector<TrajectoryPoint> &points) {
  for (std::size_t first = 1; first < points.size(); ++first) {
    for (std::size_t second = first + 2U; second < points.size(); ++second) {
      if (properSegmentIntersection(points[first - 1U], points[first],
                                    points[second - 1U], points[second])) {
        return true;
      }
    }
  }
  return false;
}

void copyDiagnosticId(
    const std::string &source,
    std::array<char, kFrontDetectionDiagnosticIdCapacity> *destination) {
  destination->fill('\0');
  const std::size_t copy_size =
      std::min(source.size(), destination->size() - 1U);
  std::copy_n(source.data(), copy_size, destination->data());
}

struct LongitudinalTargetRelation {
  FrontTargetClass target_class{FrontTargetClass::NONE};
  double forward_gap_m{std::numeric_limits<double>::infinity()};
  double rear_gap_m{std::numeric_limits<double>::infinity()};
};

double parallelRearEnvelope(const PlannerConfig &config,
                            const OpponentState &opponent) {
  const double ego_rear_extent =
      config.rear_overhang_m + config.opponent_longitudinal_tracking_margin_m;
  const double opponent_front_extent =
      config.wheel_base_m + config.front_overhang_m + opponent.uncertainty_x_m;
  return ego_rear_extent + opponent_front_extent +
         config.opponent_hard_clearance_m;
}

double candidateLateralDetectionRadius(const PlannerConfig &config,
                                       const OpponentState &opponent) {
  const double ego_half_width =
      std::max(config.left_extent_m, config.right_extent_m);
  const double opponent_half_width =
      std::max(config.left_extent_m, config.right_extent_m);
  return std::max(config.front_detection_radius_m,
                  ego_half_width + config.opponent_lateral_tracking_margin_m +
                      opponent_half_width + opponent.uncertainty_y_m +
                      config.opponent_hard_clearance_m);
}

LongitudinalTargetRelation
classifyLongitudinalTarget(double ego_s, const OpponentState &opponent,
                           const FrenetFrame &frame,
                           const PlannerConfig &config) {
  LongitudinalTargetRelation relation;
  relation.forward_gap_m = frame.forwardDeltaS(ego_s, opponent.frenet.s);
  relation.rear_gap_m = frame.forwardDeltaS(opponent.frenet.s, ego_s);
  const double half_lap = frame.length() * 0.5;
  if (relation.forward_gap_m > config.frontmost_s_tolerance_m &&
      relation.forward_gap_m < half_lap) {
    relation.target_class = FrontTargetClass::FORWARD;
  } else if (relation.forward_gap_m <= config.frontmost_s_tolerance_m ||
             relation.rear_gap_m <= parallelRearEnvelope(config, opponent)) {
    relation.target_class = FrontTargetClass::PARALLEL;
  }
  return relation;
}

enum class CandidateRejectBucket {
  WALL,
  OPPONENT,
  CURVATURE,
  TRACKABILITY,
  OTHER,
};

CandidateRejectBucket candidateRejectBucket(const std::string &reason) {
  if (reason == "wall_collision") {
    return CandidateRejectBucket::WALL;
  }
  if (reason == "opponent_collision") {
    return CandidateRejectBucket::OPPONENT;
  }
  if (reason == "maximum_curvature" ||
      reason == "curve_speed_below_safe_stop") {
    return CandidateRejectBucket::CURVATURE;
  }
  if (reason == "frenet_projection" || reason == "representative_projection" ||
      reason == "zero_length_steering_change" ||
      reason == "steering_rate_below_safe_stop" ||
      reason == "deceleration_profile" ||
      reason == "entry_speed_exceeds_candidate_limit") {
    return CandidateRejectBucket::TRACKABILITY;
  }
  return CandidateRejectBucket::OTHER;
}
} // namespace

double requiredLateralTransitionDistance(const PlannerConfig &config,
                                         double speed_mps, double current_d_m,
                                         double goal_d_m) {
  const double base_distance =
      forwardTerminalDistance(std::abs(speed_mps), config);
  const double lateral_change = std::abs(goal_d_m - current_d_m);
  const double transition_distance =
      lateral_change > 0.05
          ? config.minimum_lateral_transition_distance_m +
                config.lateral_transition_distance_gain * lateral_change
          : base_distance;
  return std::clamp(std::max(base_distance, transition_distance),
                    config.base_distance_m, config.max_distance_m);
}

double derivedFrontDetectionRadius(const PlannerConfig &config,
                                   const OpponentState &opponent) {
  const double ego_longitudinal_extent =
      std::max(config.wheel_base_m + config.front_overhang_m,
               config.rear_overhang_m) +
      config.opponent_longitudinal_tracking_margin_m;
  const double ego_lateral_extent =
      std::max(config.left_extent_m, config.right_extent_m) +
      config.opponent_lateral_tracking_margin_m;
  const double opponent_longitudinal_extent =
      std::max(config.wheel_base_m + config.front_overhang_m,
               config.rear_overhang_m) +
      opponent.uncertainty_x_m;
  const double opponent_lateral_extent =
      std::max(config.left_extent_m, config.right_extent_m) +
      opponent.uncertainty_y_m;
  const double body_envelope_radius =
      std::hypot(ego_longitudinal_extent, ego_lateral_extent) +
      std::hypot(opponent_longitudinal_extent, opponent_lateral_extent) +
      config.opponent_hard_clearance_m;
  return std::max(config.front_detection_radius_m, body_envelope_radius);
}

ParametricQuintic::Polynomial ParametricQuintic::solve(double p0, double v0,
                                                       double acceleration0,
                                                       double p1, double v1,
                                                       double acceleration1) {
  Polynomial p;
  p.a[0] = p0;
  p.a[1] = v0;
  p.a[2] = 0.5 * acceleration0;
  const double c0 = p1 - p.a[0] - p.a[1] - p.a[2];
  const double c1 = v1 - p.a[1] - 2.0 * p.a[2];
  const double c2 = acceleration1 - 2.0 * p.a[2];
  p.a[3] = 10.0 * c0 - 4.0 * c1 + 0.5 * c2;
  p.a[4] = -15.0 * c0 + 7.0 * c1 - c2;
  p.a[5] = 6.0 * c0 - 3.0 * c1 + 0.5 * c2;
  return p;
}

void ParametricQuintic::evaluate(const Polynomial &p, double u, double *value,
                                 double *derivative, double *second) {
  *value =
      p.a[0] +
      u * (p.a[1] + u * (p.a[2] + u * (p.a[3] + u * (p.a[4] + u * p.a[5]))));
  *derivative =
      p.a[1] + u * (2.0 * p.a[2] +
                    u * (3.0 * p.a[3] + u * (4.0 * p.a[4] + u * 5.0 * p.a[5])));
  *second = 2.0 * p.a[2] +
            u * (6.0 * p.a[3] + u * (12.0 * p.a[4] + u * 20.0 * p.a[5]));
}

bool ParametricQuintic::configure(const Pose2d &start, double start_curvature,
                                  const Pose2d &goal, double goal_curvature,
                                  double tangent_scale) {
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const double h = tangent_scale * distance;
  if (!std::isfinite(distance) || distance < 1.0e-3 ||
      !std::isfinite(start_curvature) || !std::isfinite(goal_curvature) ||
      !std::isfinite(h)) {
    return false;
  }
  const double start_ax = h * h * start_curvature * -std::sin(start.yaw);
  const double start_ay = h * h * start_curvature * std::cos(start.yaw);
  const double goal_ax = h * h * goal_curvature * -std::sin(goal.yaw);
  const double goal_ay = h * h * goal_curvature * std::cos(goal.yaw);
  x_ = solve(start.x, h * std::cos(start.yaw), start_ax, goal.x,
             h * std::cos(goal.yaw), goal_ax);
  y_ = solve(start.y, h * std::sin(start.yaw), start_ay, goal.y,
             h * std::sin(goal.yaw), goal_ay);
  valid_ = std::all_of(x_.a.begin(), x_.a.end(),
                       [](double v) { return std::isfinite(v); }) &&
           std::all_of(y_.a.begin(), y_.a.end(),
                       [](double v) { return std::isfinite(v); });
  return valid_;
}

TrajectoryPoint ParametricQuintic::sample(double u) const {
  TrajectoryPoint point;
  point.u = std::clamp(u, 0.0, 1.0);
  double dx = 0.0, ddx = 0.0, dy = 0.0, ddy = 0.0;
  evaluate(x_, point.u, &point.x, &dx, &ddx);
  evaluate(y_, point.u, &point.y, &dy, &ddy);
  point.yaw = std::atan2(dy, dx);
  const double speed_sq = dx * dx + dy * dy;
  point.kappa = speed_sq > 1.0e-12
                    ? (dx * ddy - dy * ddx) / std::pow(speed_sq, 1.5)
                    : std::numeric_limits<double>::quiet_NaN();
  return point;
}

std::optional<std::string>
FrontDetector::update(const EgoState &ego,
                      const std::vector<OpponentState> &opponents,
                      const FrenetFrame &frame) {
  diagnostic_ = FrontDetectionDiagnostic{};
  diagnostic_.evaluated = true;
  previous_terminals_ = current_terminals_;
  current_terminals_.clear();
  maximum_transition_distance_m_ = 0.0;
  for (const double d : config_.lateral_targets_m) {
    const double distance = requiredLateralTransitionDistance(
        config_, ego.speed_mps, ego.frenet.d, d);
    maximum_transition_distance_m_ =
        std::max(maximum_transition_distance_m_, distance);
    const auto point = frame.frenetToCartesian(ego.frenet.s + distance, d);
    current_terminals_.push_back({point.x, point.y, point.yaw});
  }
  sweep_starts_.clear();
  sweep_starts_.reserve(current_terminals_.size());
  for (std::size_t i = 0; i < current_terminals_.size(); ++i) {
    (void)i;
    // Re-evaluate the complete transition corridor every cycle. A sweep from
    // the previous terminal can skip an opponent that entered near the ego.
    sweep_starts_.push_back(static_cast<const Pose2d &>(ego));
  }
  std::optional<std::string> sweep_forward_id;
  std::optional<std::string> legacy_forward_id;
  std::optional<std::string> parallel_id;
  bool forward_sweep_order_valid = true;
  double nearest_forward_gap = std::numeric_limits<double>::infinity();
  double nearest_forward_distance = std::numeric_limits<double>::infinity();
  double nearest_legacy_forward_gap = std::numeric_limits<double>::infinity();
  double nearest_parallel_gap = std::numeric_limits<double>::infinity();
  double nearest_parallel_distance = std::numeric_limits<double>::infinity();
  detection_radius_m_ = config_.front_detection_radius_m;
  for (const auto &opponent : opponents) {
    // Diagnostic slots intentionally preserve the detector's evaluation
    // order; they are not sorted and never feed selection.
    FrontDetectionOpponentDiagnostic *opponent_diagnostic = nullptr;
    if (diagnostic_.opponent_count < diagnostic_.opponents.size()) {
      opponent_diagnostic =
          &diagnostic_.opponents[diagnostic_.opponent_count++];
      copyDiagnosticId(opponent.id, &opponent_diagnostic->opponent_id);
    } else {
      ++diagnostic_.dropped_opponent_count;
    }
    if (!opponent.valid) {
      if (opponent_diagnostic != nullptr) {
        opponent_diagnostic->first_false =
            FrontDetectionFailure::INVALID_OPPONENT;
      }
      continue;
    }
    const double detection_radius =
        derivedFrontDetectionRadius(config_, opponent);
    detection_radius_m_ = std::max(detection_radius_m_, detection_radius);
    const auto relation =
        classifyLongitudinalTarget(ego.frenet.s, opponent, frame, config_);
    if (opponent_diagnostic != nullptr) {
      opponent_diagnostic->target_class = relation.target_class;
      opponent_diagnostic->forward_gap_m = relation.forward_gap_m;
      opponent_diagnostic->rear_gap_m = relation.rear_gap_m;
      opponent_diagnostic->detection_radius_m = detection_radius;
    }
    if (relation.target_class == FrontTargetClass::NONE) {
      if (opponent_diagnostic != nullptr) {
        opponent_diagnostic->first_false =
            FrontDetectionFailure::LONGITUDINAL_TARGET;
      }
      continue;
    }
    double minimum_sweep_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < current_terminals_.size(); ++i) {
      const auto &current = current_terminals_[i];
      const auto &start = sweep_starts_[i];
      const double distance_to_sweep = pointToSegmentDistance(
          opponent.x, opponent.y, start.x, start.y, current.x, current.y);
      minimum_sweep_distance =
          std::min(minimum_sweep_distance, distance_to_sweep);
    }
    if (opponent_diagnostic != nullptr) {
      opponent_diagnostic->minimum_sweep_distance_m = minimum_sweep_distance;
    }
    if (minimum_sweep_distance > detection_radius) {
      if (opponent_diagnostic != nullptr) {
        opponent_diagnostic->first_false =
            FrontDetectionFailure::TRANSITION_ENVELOPE;
      }
      continue;
    }
    if (opponent_diagnostic != nullptr) {
      opponent_diagnostic->first_false =
          FrontDetectionFailure::SELECTION_ARBITRATION;
    }
    if (relation.target_class == FrontTargetClass::FORWARD) {
      const bool finite_forward_gap = std::isfinite(relation.forward_gap_m) &&
                                      relation.forward_gap_m >= 0.0;
      const bool finite_sweep_distance =
          std::isfinite(minimum_sweep_distance) &&
          minimum_sweep_distance >= 0.0;
      if (!finite_forward_gap) {
        // A forward relation without a finite longitudinal gap cannot be used
        // by either ranking. Leave target selection fail-closed for that
        // opponent instead of allowing floating-point ordering to decide it.
        forward_sweep_order_valid = false;
        continue;
      }
      const bool smaller_legacy_forward_gap =
          relation.forward_gap_m <
          nearest_legacy_forward_gap - config_.tie_break_epsilon;
      const bool same_legacy_forward_gap =
          std::abs(relation.forward_gap_m - nearest_legacy_forward_gap) <=
          config_.tie_break_epsilon;
      const bool better_legacy =
          !legacy_forward_id.has_value() || smaller_legacy_forward_gap ||
          (same_legacy_forward_gap && opponent.id < legacy_forward_id.value());
      if (better_legacy) {
        legacy_forward_id = opponent.id;
        nearest_legacy_forward_gap = relation.forward_gap_m;
      }

      // If any admissible forward sweep is non-finite, retain the former
      // deterministic Frenet-gap ranking for the complete forward set. Do not
      // rank an unknown sweep last: that could silently remove a nearby
      // opponent from the longitudinal target decision.
      if (!finite_sweep_distance) {
        forward_sweep_order_valid = false;
        continue;
      }
      // The target used for longitudinal FOLLOW must be the forward opponent
      // closest to the transition corridor, not merely the one with the
      // smallest Frenet-s separation. At the start grid a laterally adjacent
      // kart can project to a smaller s while the kart directly along the
      // ego's admissible transition is farther along the reference. The
      // non-selected opponent remains in the all-opponent safety evaluation.
      const bool closer_to_transition =
          minimum_sweep_distance <
          nearest_forward_distance - config_.tie_break_epsilon;
      const bool same_transition_distance =
          std::abs(minimum_sweep_distance - nearest_forward_distance) <=
          config_.tie_break_epsilon;
      const bool smaller_forward_gap =
          relation.forward_gap_m <
          nearest_forward_gap - config_.tie_break_epsilon;
      const bool same_forward_gap =
          std::abs(relation.forward_gap_m - nearest_forward_gap) <=
          config_.tie_break_epsilon;
      const bool better =
          !sweep_forward_id.has_value() || closer_to_transition ||
          (same_transition_distance &&
           (smaller_forward_gap ||
            (same_forward_gap && opponent.id < sweep_forward_id.value())));
      if (better) {
        sweep_forward_id = opponent.id;
        nearest_forward_gap = relation.forward_gap_m;
        nearest_forward_distance = minimum_sweep_distance;
      }
    } else {
      const double parallel_gap =
          std::min(relation.forward_gap_m, relation.rear_gap_m);
      const bool better =
          !parallel_id.has_value() || parallel_gap < nearest_parallel_gap ||
          (parallel_gap == nearest_parallel_gap &&
           (minimum_sweep_distance < nearest_parallel_distance ||
            (minimum_sweep_distance == nearest_parallel_distance &&
             opponent.id < parallel_id.value())));
      if (better) {
        parallel_id = opponent.id;
        nearest_parallel_gap = parallel_gap;
        nearest_parallel_distance = minimum_sweep_distance;
      }
    }
  }
  const std::optional<std::string> forward_id =
      forward_sweep_order_valid ? sweep_forward_id : legacy_forward_id;
  const std::optional<std::string> detected_id =
      forward_id.has_value() ? forward_id : parallel_id;
  if (detected_id.has_value()) {
    copyDiagnosticId(detected_id.value(), &diagnostic_.selected_target_id);
  }
  initial_sweep_ = false;
  if (detected_id.has_value()) {
    ++enter_cycles_;
    if (enter_cycles_ >= config_.front_enter_cycles) {
      detected_ = true;
    }
    clear_cycles_ = 0;
  } else if (detected_) {
    enter_cycles_ = 0;
    ++clear_cycles_;
    if (clear_cycles_ >= config_.front_release_cycles) {
      detected_ = false;
      clear_cycles_ = 0;
    }
  } else {
    enter_cycles_ = 0;
  }
  diagnostic_.enter_cycles = enter_cycles_;
  diagnostic_.clear_cycles = clear_cycles_;
  diagnostic_.detected_latched = detected_;
  if (detected_id.has_value()) {
    for (std::size_t i = 0U; i < diagnostic_.opponent_count; ++i) {
      auto &entry = diagnostic_.opponents[i];
      if (std::string_view(entry.opponent_id.data()) == detected_id.value()) {
        entry.first_false = detected_ ? FrontDetectionFailure::NONE
                                      : FrontDetectionFailure::ENTER_HYSTERESIS;
        break;
      }
    }
  }
  return detected_ ? detected_id : std::nullopt;
}

void EarlyAwareSelector::clear(const std::string &reason) {
  target_id_.clear();
  last_ego_stamp_sec_ = -1.0;
  last_target_stamp_sec_ = -1.0;
  distinct_fresh_stamp_count_ = 0;
  diagnostic_ = EarlyAwareDiagnostic{};
  diagnostic_.reason = reason;
}

void EarlyAwareSelector::reset() { clear("reset"); }

void EarlyAwareSelector::update(const EgoState &ego,
                                const std::vector<OpponentState> &opponents,
                                const FrenetFrame &frame, bool inputs_fresh,
                                double now_sec) {
  diagnostic_ = EarlyAwareDiagnostic{};
  diagnostic_.evaluated = true;
  if (!config_.early_aware_enabled) {
    clear("disabled");
    diagnostic_.evaluated = true;
    diagnostic_.reason = "disabled";
    return;
  }
  const bool finite_ego =
      ego.valid && ego.frenet.valid && std::isfinite(ego.x) &&
      std::isfinite(ego.y) && std::isfinite(ego.yaw) &&
      std::isfinite(ego.speed_mps) && std::isfinite(ego.stamp_sec) &&
      std::isfinite(ego.frenet.s);
  const bool ego_stamp_fresh =
      finite_ego && inputs_fresh &&
      ego.stamp_sec <= now_sec + config_.tie_break_epsilon &&
      now_sec - ego.stamp_sec <= config_.ego_stale_sec;
  if (!ego_stamp_fresh || frame.empty()) {
    clear("invalid_or_stale_ego");
    diagnostic_.evaluated = true;
    diagnostic_.reason = "invalid_or_stale_ego";
    return;
  }
  const auto ego_reference = frame.interpolate(ego.frenet.s);
  const double ego_heading_error_rad =
      std::abs(normalizeAngle(ego.yaw - ego_reference.yaw));
  const double ego_tangent_speed_mps =
      ego.speed_mps * std::cos(normalizeAngle(ego.yaw - ego_reference.yaw));
  if (!std::isfinite(ego_reference.yaw) ||
      !std::isfinite(ego_tangent_speed_mps) ||
      !std::isfinite(ego_heading_error_rad) ||
      ego_heading_error_rad > config_.early_aware_max_track_heading_error_rad ||
      ego_tangent_speed_mps < config_.early_aware_min_tangent_speed_mps) {
    clear("reverse_or_stationary_ego");
    diagnostic_.evaluated = true;
    diagnostic_.reason = "reverse_or_stationary_ego";
    return;
  }

  const OpponentState *best = nullptr;
  double best_forward_delta_s_m = std::numeric_limits<double>::infinity();
  double best_target_tangent_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  for (const auto &opponent : opponents) {
    const bool finite_opponent =
        opponent.valid && opponent.frenet.valid && !opponent.id.empty() &&
        std::isfinite(opponent.x) && std::isfinite(opponent.y) &&
        std::isfinite(opponent.yaw) && std::isfinite(opponent.stamp_sec) &&
        std::isfinite(opponent.vx_mps) && std::isfinite(opponent.vy_mps) &&
        std::isfinite(opponent.frenet.s);
    if (!finite_opponent ||
        opponent.stamp_sec > now_sec + config_.tie_break_epsilon ||
        now_sec - opponent.stamp_sec > config_.opponent_stale_sec) {
      continue;
    }
    const auto reference = frame.interpolate(opponent.frenet.s);
    const double heading_error_rad =
        std::abs(normalizeAngle(opponent.yaw - reference.yaw));
    const double tangent_speed_mps = opponent.vx_mps * std::cos(reference.yaw) +
                                     opponent.vy_mps * std::sin(reference.yaw);
    const double forward_delta_s_m =
        frame.forwardDeltaS(ego.frenet.s, opponent.frenet.s);
    const double closing_speed_mps =
        std::max(ego_tangent_speed_mps - tangent_speed_mps, 0.0);
    if (!std::isfinite(reference.yaw) || !std::isfinite(tangent_speed_mps) ||
        !std::isfinite(heading_error_rad) ||
        !std::isfinite(forward_delta_s_m) ||
        heading_error_rad > config_.early_aware_max_track_heading_error_rad ||
        tangent_speed_mps <
            -config_.early_aware_reverse_tangent_tolerance_mps ||
        forward_delta_s_m < config_.early_aware_min_forward_delta_s_m ||
        closing_speed_mps <= 0.0) {
      continue;
    }
    if (best == nullptr ||
        forward_delta_s_m <
            best_forward_delta_s_m - config_.tie_break_epsilon ||
        (std::abs(forward_delta_s_m - best_forward_delta_s_m) <=
             config_.tie_break_epsilon &&
         opponent.id < best->id)) {
      best = &opponent;
      best_forward_delta_s_m = forward_delta_s_m;
      best_target_tangent_speed_mps = tangent_speed_mps;
    }
  }
  if (best == nullptr) {
    clear("no_same_direction_closing_target");
    diagnostic_.evaluated = true;
    diagnostic_.reason = "no_same_direction_closing_target";
    return;
  }
  const double closing_speed_mps =
      std::max(ego_tangent_speed_mps - best_target_tangent_speed_mps, 0.0);
  const double dynamic_distance_m = std::clamp(
      config_.early_aware_base_distance_m +
          ego_tangent_speed_mps * config_.early_aware_speed_horizon_sec +
          closing_speed_mps * closing_speed_mps /
              (2.0 * config_.early_aware_max_deceleration_mps2),
      config_.early_aware_min_distance_m, config_.early_aware_max_distance_m);
  diagnostic_.target_observation_stamp_sec = best->stamp_sec;
  diagnostic_.forward_delta_s_m = best_forward_delta_s_m;
  diagnostic_.dynamic_distance_m = dynamic_distance_m;
  diagnostic_.ego_tangent_speed_mps = ego_tangent_speed_mps;
  diagnostic_.target_tangent_speed_mps = best_target_tangent_speed_mps;
  diagnostic_.projected_closing_speed_mps = closing_speed_mps;
  if (!std::isfinite(dynamic_distance_m) ||
      best_forward_delta_s_m > dynamic_distance_m) {
    clear("outside_dynamic_distance");
    diagnostic_.evaluated = true;
    diagnostic_.reason = "outside_dynamic_distance";
    return;
  }

  const bool same_target = best->id == target_id_;
  const bool distinct_fresh_stamp =
      same_target &&
      ego.stamp_sec > last_ego_stamp_sec_ + config_.tie_break_epsilon &&
      best->stamp_sec > last_target_stamp_sec_ + config_.tie_break_epsilon;
  if (!same_target) {
    target_id_ = best->id;
    last_ego_stamp_sec_ = ego.stamp_sec;
    last_target_stamp_sec_ = best->stamp_sec;
    distinct_fresh_stamp_count_ = 1;
  } else if (distinct_fresh_stamp) {
    last_ego_stamp_sec_ = ego.stamp_sec;
    last_target_stamp_sec_ = best->stamp_sec;
    ++distinct_fresh_stamp_count_;
  }
  copyDiagnosticId(target_id_, &diagnostic_.target_id);
  diagnostic_.distinct_fresh_stamp_count = distinct_fresh_stamp_count_;
  diagnostic_.active =
      distinct_fresh_stamp_count_ >= config_.early_aware_required_fresh_stamps;
  diagnostic_.state =
      diagnostic_.active ? EarlyAwareState::ACTIVE : EarlyAwareState::CANDIDATE;
  diagnostic_.reason = diagnostic_.active ? "active" : "awaiting_fresh_stamp";
}

LatticePlanner::LatticePlanner(PlannerConfig config, const FrenetFrame *frame,
                               const GridMap *map,
                               const FrenetFrame *output_frame)
    : config_(std::move(config)), frame_(frame),
      output_frame_(output_frame != nullptr ? output_frame : frame), map_(map),
      detector_(config_), early_aware_selector_(config_) {}

std::vector<TrajectoryPoint>
LatticePlanner::denseSamples(const ParametricQuintic &polynomial,
                             int coarse_intervals) const {
  coarse_intervals = std::max(1, coarse_intervals);
  std::vector<TrajectoryPoint> coarse;
  coarse.reserve(static_cast<std::size_t>(coarse_intervals + 1));
  for (int i = 0; i <= coarse_intervals; ++i) {
    coarse.push_back(polynomial.sample(static_cast<double>(i) /
                                       static_cast<double>(coarse_intervals)));
  }
  std::vector<TrajectoryPoint> dense;
  dense.push_back(coarse.front());
  for (std::size_t i = 1; i < coarse.size(); ++i) {
    const double distance = std::hypot(coarse[i].x - coarse[i - 1U].x,
                                       coarse[i].y - coarse[i - 1U].y);
    const double yaw_change =
        std::abs(normalizeAngle(coarse[i].yaw - coarse[i - 1U].yaw));
    const int pieces =
        std::max(1, static_cast<int>(std::ceil(std::max(
                        distance / config_.collision_max_step_m,
                        yaw_change / config_.collision_max_yaw_step_rad))));
    for (int piece = 1; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / pieces;
      dense.push_back(
          polynomial.sample(lerp(coarse[i - 1U].u, coarse[i].u, ratio)));
    }
  }
  return dense;
}

std::vector<CandidateTrajectory> LatticePlanner::generateCandidates(
    const EgoState &ego, const std::vector<OpponentState> &opponents) const {
  return generateCandidatesInternal(ego, opponents);
}

std::vector<CandidateTrajectory> LatticePlanner::generateCandidatesInternal(
    const EgoState &ego, const std::vector<OpponentState> &opponents) const {
  std::vector<CandidateTrajectory> candidates;
  const double reference_start_curvature =
      sanitizedCurvature(frame_->interpolate(ego.frenet.s).kappa, config_);
  const double maximum_curvature = maximumPlannerCurvature(config_);
  const bool ego_curvature_usable =
      std::isfinite(ego.curvature) &&
      std::abs(ego.curvature) <= maximum_curvature * 1.25;
  const double start_curvature = sanitizedCurvature(
      ego_curvature_usable ? ego.curvature : reference_start_curvature,
      config_);

  const OpponentState *forward_opponent = nullptr;
  const OpponentState *parallel_opponent = nullptr;
  double nearest_forward_gap = std::numeric_limits<double>::infinity();
  double nearest_parallel_gap = std::numeric_limits<double>::infinity();
  for (const auto &opponent : opponents) {
    if (!opponent.valid) {
      continue;
    }
    if (!target_id_.empty() && opponent.id != target_id_) {
      continue;
    }
    const auto relation =
        classifyLongitudinalTarget(ego.frenet.s, opponent, *frame_, config_);
    if (relation.target_class == FrontTargetClass::NONE) {
      continue;
    }
    if (target_id_.empty() &&
        std::abs(opponent.frenet.d - ego.frenet.d) >
            candidateLateralDetectionRadius(config_, opponent)) {
      continue;
    }
    if (relation.target_class == FrontTargetClass::FORWARD) {
      if (relation.forward_gap_m < nearest_forward_gap) {
        nearest_forward_gap = relation.forward_gap_m;
        forward_opponent = &opponent;
      }
    } else {
      const double parallel_gap =
          std::min(relation.forward_gap_m, relation.rear_gap_m);
      if (parallel_gap < nearest_parallel_gap) {
        nearest_parallel_gap = parallel_gap;
        parallel_opponent = &opponent;
      }
    }
  }
  const bool blocking_is_forward = forward_opponent != nullptr;
  const OpponentState *blocking_opponent =
      blocking_is_forward ? forward_opponent : parallel_opponent;
  const double blocking_forward_gap =
      blocking_is_forward ? nearest_forward_gap : 0.0;
  const double largest_nominal_offset = std::accumulate(
      config_.lateral_targets_m.begin(), config_.lateral_targets_m.end(), 0.0,
      [](double value, double offset) {
        return std::max(value, std::abs(offset));
      });
  const bool obstacle_within_lattice =
      blocking_opponent != nullptr &&
      blocking_forward_gap <=
          config_.max_distance_m +
              requiredPreObstacleLongitudinalSeparation(config_,
                                                        *blocking_opponent) +
              candidateLateralDetectionRadius(config_, *blocking_opponent);
  bool already_pass_clear = false;
  double pre_obstacle_terminal_distance =
      std::numeric_limits<double>::infinity();
  if (obstacle_within_lattice) {
    already_pass_clear =
        passCapableGoal(ego.frenet.d, *blocking_opponent, config_);
    const std::size_t nearest = frame_->nearestIndex(ego.frenet.s);
    const double midpoint_s =
        0.5 * (frame_->unwrappedIndexS(static_cast<long long>(nearest), nearest,
                                       ego.frenet.s) +
               frame_->unwrappedIndexS(static_cast<long long>(nearest) +
                                           config_.mpc_wp_id_offset,
                                       nearest, ego.frenet.s));
    const double furthest_receiver_s = frame_->unwrappedIndexS(
        static_cast<long long>(nearest) + config_.mpc_wp_id_offset +
            config_.nearest_index_uncertainty,
        nearest, ego.frenet.s);
    const double receiver_alignment_lead =
        std::max(0.0, furthest_receiver_s - midpoint_s);
    if (blocking_is_forward) {
      pre_obstacle_terminal_distance =
          blocking_forward_gap -
          requiredPreObstacleLongitudinalSeparation(config_,
                                                    *blocking_opponent) -
          receiver_alignment_lead;
    }
  }

  for (std::size_t lateral = 0; lateral < config_.lateral_targets_m.size();
       ++lateral) {
    const double nominal_goal_d = config_.lateral_targets_m[lateral];
    double goal_d = nominal_goal_d;
    const bool outer_candidate =
        std::abs(nominal_goal_d) >=
        largest_nominal_offset - config_.tie_break_epsilon;
    if (config_.adaptive_pass_offset_enabled && obstacle_within_lattice &&
        outer_candidate && !already_pass_clear) {
      if (nominal_goal_d > 0.0) {
        goal_d = std::max(nominal_goal_d,
                          blocking_opponent->frenet.d +
                              requiredPassLateralSeparation(
                                  config_, *blocking_opponent, true));
      } else if (nominal_goal_d < 0.0) {
        goal_d = std::min(nominal_goal_d,
                          blocking_opponent->frenet.d -
                              requiredPassLateralSeparation(
                                  config_, *blocking_opponent, false));
      }
      goal_d = std::clamp(goal_d, -config_.max_adaptive_lateral_offset_m,
                          config_.max_adaptive_lateral_offset_m);
    }
    const double nominal_terminal_distance = requiredLateralTransitionDistance(
        config_, ego.speed_mps, ego.frenet.d, goal_d);
    double terminal_distance = nominal_terminal_distance;
    if (obstacle_within_lattice && !already_pass_clear &&
        std::isfinite(pre_obstacle_terminal_distance)) {
      terminal_distance =
          std::min(terminal_distance,
                   std::max(config_.minimum_obstacle_transition_distance_m,
                            pre_obstacle_terminal_distance));
    }
    const auto reference_goal =
        frame_->interpolate(ego.frenet.s + terminal_distance);
    const double denominator = 1.0 - goal_d * reference_goal.kappa;
    if (!std::isfinite(denominator) || denominator <= 1.0e-3) {
      continue;
    }
    const auto goal_point =
        frame_->frenetToCartesian(ego.frenet.s + terminal_distance, goal_d);
    const Pose2d goal{goal_point.x, goal_point.y, reference_goal.yaw};
    const double goal_curvature =
        sanitizedCurvature(reference_goal.kappa / denominator, config_);
    if (config_.safety_evaluation_enabled &&
        map_->footprintHitsWall(goal, wallFootprint(config_, true))) {
      for (std::size_t tangent = 0; tangent < config_.tangent_scales.size();
           ++tangent) {
        CandidateTrajectory candidate;
        candidate.lateral_index = lateral;
        candidate.tangent_index = tangent;
        candidate.goal_d_m = goal_d;
        candidate.tangent_scale = config_.tangent_scales[tangent];
        candidate.required_arc_m = terminal_distance;
        candidate.rejection_reason = "wall_collision";
        candidates.push_back(std::move(candidate));
      }
      continue;
    }
    for (std::size_t tangent = 0; tangent < config_.tangent_scales.size();
         ++tangent) {
      CandidateTrajectory candidate;
      candidate.lateral_index = lateral;
      candidate.tangent_index = tangent;
      candidate.goal_d_m = goal_d;
      candidate.tangent_scale = config_.tangent_scales[tangent];
      candidate.required_arc_m = terminal_distance;
      ParametricQuintic polynomial;
      if (!polynomial.configure(ego, start_curvature, goal, goal_curvature,
                                candidate.tangent_scale)) {
        candidate.rejection_reason = "quintic_generation";
        candidates.push_back(std::move(candidate));
        continue;
      }
      for (int i = 0; i < config_.sampling_points; ++i) {
        candidate.representative.push_back(polynomial.sample(
            static_cast<double>(i) / (config_.sampling_points - 1)));
      }
      candidate.dense = denseSamples(polynomial);
      evaluateTrajectory(&candidate, opponents, std::abs(ego.speed_mps));
      const bool legacy_trackability_failure =
          candidate.rejection_reason == "maximum_curvature" ||
          candidate.rejection_reason == "curve_speed_below_safe_stop" ||
          candidate.rejection_reason == "steering_rate_below_safe_stop";
      // A continuation latch identifies the selected lattice branch, not the
      // intermediate quintic that preceded deterministic clearance-profile
      // recovery. Re-run the same transition-distance and recovery pipeline
      // on every cycle so a latched recovered branch is evaluated with the
      // same geometry and safety checks as its first selection.
      const bool may_try_clearance_profile =
          !candidate.feasible && legacy_trackability_failure &&
          blocking_is_forward && obstacle_within_lattice &&
          blocking_opponent != nullptr && !already_pass_clear &&
          passCapableGoal(goal_d, *blocking_opponent, config_) &&
          std::isfinite(blocking_forward_gap) &&
          blocking_forward_gap >
              config_.minimum_obstacle_transition_distance_m +
                  config_.tie_break_epsilon &&
          nominal_terminal_distance >
              blocking_forward_gap + config_.tie_break_epsilon;
      if (may_try_clearance_profile) {
        std::optional<CandidateTrajectory> best_clearance_profile;
        const double lateral_direction = goal_d >= ego.frenet.d ? 1.0 : -1.0;
        for (const double join_fraction : {0.50, 0.65, 0.80}) {
          if (best_clearance_profile.has_value()) {
            break;
          }
          const double join_distance = blocking_forward_gap * join_fraction;
          const double join_d =
              ego.frenet.d + join_fraction * (goal_d - ego.frenet.d);
          const auto join_reference =
              frame_->interpolate(ego.frenet.s + join_distance);
          const auto join_point = frame_->frenetToCartesian(
              ego.frenet.s + join_distance, join_d);
          const double join_denominator =
              1.0 - join_d * join_reference.kappa;
          if (!std::isfinite(join_denominator) ||
              join_denominator <= 1.0e-3) {
            continue;
          }
          const double join_curvature = sanitizedCurvature(
              join_reference.kappa / join_denominator, config_);
          for (const double yaw_magnitude : {0.4, 0.6, 0.8}) {
            if (best_clearance_profile.has_value()) {
              break;
            }
            const Pose2d join_pose{
                join_point.x, join_point.y,
                join_reference.yaw + lateral_direction * yaw_magnitude};
            const auto clearance_reference =
                frame_->interpolate(ego.frenet.s + blocking_forward_gap);
            const auto clearance_point = frame_->frenetToCartesian(
                ego.frenet.s + blocking_forward_gap, goal_d);
            const double clearance_denominator =
                1.0 - goal_d * clearance_reference.kappa;
            if (!std::isfinite(clearance_denominator) ||
                clearance_denominator <= 1.0e-3) {
              continue;
            }
            const Pose2d clearance_pose{clearance_point.x, clearance_point.y,
                                        clearance_reference.yaw};
            const double clearance_curvature = sanitizedCurvature(
                clearance_reference.kappa / clearance_denominator, config_);
            const auto hold_reference = frame_->interpolate(
                ego.frenet.s + nominal_terminal_distance);
            const auto hold_point = frame_->frenetToCartesian(
                ego.frenet.s + nominal_terminal_distance, goal_d);
            const double hold_denominator =
                1.0 - goal_d * hold_reference.kappa;
            if (!std::isfinite(hold_denominator) ||
                hold_denominator <= 1.0e-3) {
              continue;
            }
            const Pose2d hold_pose{hold_point.x, hold_point.y,
                                   hold_reference.yaw};
            const double hold_curvature = sanitizedCurvature(
                hold_reference.kappa / hold_denominator, config_);

            ParametricQuintic approach;
            ParametricQuintic clearance;
            ParametricQuintic hold;
            if (!approach.configure(ego, start_curvature, join_pose,
                                    join_curvature,
                                    candidate.tangent_scale) ||
                !clearance.configure(join_pose, join_curvature, clearance_pose,
                                     clearance_curvature,
                                     candidate.tangent_scale) ||
                !hold.configure(clearance_pose, clearance_curvature, hold_pose,
                                hold_curvature, candidate.tangent_scale)) {
              continue;
            }

            CandidateTrajectory trial;
            trial.lateral_index = lateral;
            trial.tangent_index = tangent;
            trial.goal_d_m = goal_d;
            trial.tangent_scale = candidate.tangent_scale;
            trial.required_arc_m = nominal_terminal_distance;
            // Preserve the legacy 100-interval geometric sampling budget over
            // the complete path. Giving every segment 100 intervals creates
            // more than the fixed 256-point Cartesian transport can carry,
            // even though the path itself is only one planning horizon. The
            // existing distance/yaw refinement below denseSamples() remains
            // authoritative and can still add samples wherever required.
            constexpr int kWholePathCoarseIntervals = 100;
            constexpr int kMinimumSegmentIntervals = 8;
            const double hold_distance =
                nominal_terminal_distance - blocking_forward_gap;
            const int hold_intervals = std::clamp(
                static_cast<int>(std::lround(
                    kWholePathCoarseIntervals * hold_distance /
                    nominal_terminal_distance)),
                kMinimumSegmentIntervals,
                kWholePathCoarseIntervals - 2 * kMinimumSegmentIntervals);
            const int passing_intervals =
                kWholePathCoarseIntervals - hold_intervals;
            const int approach_intervals = std::clamp(
                static_cast<int>(std::lround(
                    passing_intervals * join_distance /
                    blocking_forward_gap)),
                kMinimumSegmentIntervals,
                passing_intervals - kMinimumSegmentIntervals);
            const int clearance_intervals =
                passing_intervals - approach_intervals;
            trial.dense = denseSamples(approach, approach_intervals);
            auto clearance_dense =
                denseSamples(clearance, clearance_intervals);
            auto hold_dense = denseSamples(hold, hold_intervals);
            if (trial.dense.empty() || clearance_dense.size() < 2U ||
                hold_dense.size() < 2U) {
              continue;
            }
            trial.dense.insert(trial.dense.end(),
                               std::next(clearance_dense.begin()),
                               clearance_dense.end());
            trial.dense.insert(trial.dense.end(), std::next(hold_dense.begin()),
                               hold_dense.end());
            for (std::size_t i = 0U; i < trial.dense.size(); ++i) {
              trial.dense[i].u =
                  static_cast<double>(i) /
                  static_cast<double>(trial.dense.size() - 1U);
            }
            for (int i = 0; i < config_.sampling_points; ++i) {
              const std::size_t index =
                  static_cast<std::size_t>(i) * (trial.dense.size() - 1U) /
                  static_cast<std::size_t>(config_.sampling_points - 1);
              trial.representative.push_back(trial.dense[index]);
            }
            evaluateTrajectory(&trial, opponents, std::abs(ego.speed_mps));
            if (!trial.feasible) {
              continue;
            }
            // The search order is deliberate: earlier join fractions and
            // smaller heading offsets establish lateral clearance sooner.
            // Retain the first fully evaluated feasible profile rather than
            // trading clearance timing for a higher speed limit.
            if (!best_clearance_profile.has_value()) {
              best_clearance_profile = std::move(trial);
            }
          }
        }
        if (best_clearance_profile.has_value()) {
          candidate = std::move(best_clearance_profile.value());
        }
      }
      candidates.push_back(std::move(candidate));
    }
  }
  return candidates;
}

std::optional<CandidateTrajectory> LatticePlanner::roleCorridorCandidate(
    const EgoState &ego, double goal_d_m,
    const std::vector<OpponentState> &opponents) const {
  if (!ego.valid || !ego.frenet.valid || !std::isfinite(goal_d_m) ||
      frame_ == nullptr || frame_->empty() || map_ == nullptr ||
      !map_->initialized() || config_.tangent_scales.empty()) {
    return std::nullopt;
  }
  const double terminal_distance_m = std::clamp(
      std::max(requiredLateralTransitionDistance(config_, ego.speed_mps,
                                                 ego.frenet.d, goal_d_m),
               config_.max_distance_m),
      config_.minimum_lateral_transition_distance_m, config_.max_distance_m);
  const auto reference_goal =
      frame_->interpolate(ego.frenet.s + terminal_distance_m);
  const double denominator = 1.0 - goal_d_m * reference_goal.kappa;
  if (!std::isfinite(denominator) || denominator <= 1.0e-3) {
    return std::nullopt;
  }
  const auto goal_point =
      frame_->frenetToCartesian(ego.frenet.s + terminal_distance_m, goal_d_m);
  const Pose2d goal{goal_point.x, goal_point.y, reference_goal.yaw};
  const double goal_curvature =
      sanitizedCurvature(reference_goal.kappa / denominator, config_);
  const double start_curvature = sanitizedCurvature(ego.curvature, config_);
  const auto tangent = std::min_element(
      config_.tangent_scales.cbegin(), config_.tangent_scales.cend(),
      [](double lhs, double rhs) {
        return std::abs(lhs - 1.0) < std::abs(rhs - 1.0);
      });
  ParametricQuintic polynomial;
  if (tangent == config_.tangent_scales.cend() ||
      !polynomial.configure(ego, start_curvature, goal, goal_curvature,
                            *tangent)) {
    return std::nullopt;
  }
  CandidateTrajectory candidate;
  candidate.lateral_index = config_.lateral_targets_m.size() + 1U;
  candidate.tangent_index = static_cast<std::size_t>(
      std::distance(config_.tangent_scales.cbegin(), tangent));
  candidate.goal_d_m = goal_d_m;
  candidate.tangent_scale = *tangent;
  candidate.required_arc_m = terminal_distance_m;
  for (int i = 0; i < config_.sampling_points; ++i) {
    candidate.representative.push_back(polynomial.sample(
        static_cast<double>(i) / (config_.sampling_points - 1)));
  }
  candidate.dense = denseSamples(polynomial);
  evaluateTrajectory(&candidate, opponents, std::abs(ego.speed_mps));
  return candidate;
}

std::optional<CandidateTrajectory> LatticePlanner::movingTargetFollowCandidate(
    const EgoState &ego, const OpponentState &target,
    const std::vector<OpponentState> &opponents) const {
  if (!target.valid ||
      target.speed_mps <= config_.safe_stop_speed_mps + 1.0e-6) {
    return std::nullopt;
  }

  // Moving is not safety evidence. Every fallback is re-evaluated with the
  // selected target stopped at its measured pose, so an immediate target stop
  // cannot turn a longitudinal FOLLOW into an unverified collision path.
  auto worst_case_opponents = opponents;
  for (auto &opponent : worst_case_opponents) {
    if (opponent.id == target.id) {
      opponent.speed_mps = 0.0;
      opponent.vx_mps = 0.0;
      opponent.vy_mps = 0.0;
    }
  }

  std::optional<double> next_centerward_goal;
  for (const double goal_d_m : config_.lateral_targets_m) {
    const bool same_side = std::abs(goal_d_m) <= config_.tie_break_epsilon ||
                           std::signbit(goal_d_m) == std::signbit(ego.frenet.d);
    const bool centerward =
        std::abs(goal_d_m) + config_.tie_break_epsilon < std::abs(ego.frenet.d);
    if (same_side && centerward &&
        (!next_centerward_goal.has_value() ||
         std::abs(goal_d_m) > std::abs(next_centerward_goal.value()))) {
      next_centerward_goal = goal_d_m;
    }
  }
  std::vector<double> goals;
  goals.reserve(2U);
  bool front_clear_for_stage = false;
  if (next_centerward_goal.has_value()) {
    const double stage_distance = std::clamp(
        std::max(requiredLateralTransitionDistance(
                     config_, ego.speed_mps, ego.frenet.d,
                     next_centerward_goal.value()),
                 config_.max_distance_m),
        config_.minimum_lateral_transition_distance_m, config_.max_distance_m);
    const std::size_t nearest = frame_->nearestIndex(ego.frenet.s);
    const double midpoint_s =
        0.5 * (frame_->unwrappedIndexS(static_cast<long long>(nearest), nearest,
                                       ego.frenet.s) +
               frame_->unwrappedIndexS(static_cast<long long>(nearest) +
                                           config_.mpc_wp_id_offset,
                                       nearest, ego.frenet.s));
    const double furthest_receiver_s = frame_->unwrappedIndexS(
        static_cast<long long>(nearest) + config_.mpc_wp_id_offset +
            config_.nearest_index_uncertainty,
        nearest, ego.frenet.s);
    const double receiver_alignment_lead =
        std::max(0.0, furthest_receiver_s - midpoint_s);
    const double forward_gap =
        frame_->forwardDeltaS(ego.frenet.s, target.frenet.s);
    front_clear_for_stage =
        forward_gap >=
        stage_distance +
            requiredPreObstacleLongitudinalSeparation(config_, target) +
            receiver_alignment_lead;
  }
  if (!front_clear_for_stage) {
    goals.push_back(ego.frenet.d);
  }
  if (next_centerward_goal.has_value()) {
    goals.push_back(next_centerward_goal.value());
  }
  if (front_clear_for_stage) {
    goals.push_back(ego.frenet.d);
  }

  const double reference_start_curvature =
      sanitizedCurvature(frame_->interpolate(ego.frenet.s).kappa, config_);
  const double maximum_curvature = maximumPlannerCurvature(config_);
  const bool ego_curvature_usable =
      std::isfinite(ego.curvature) &&
      std::abs(ego.curvature) <= maximum_curvature * 1.25;
  const double start_curvature = sanitizedCurvature(
      ego_curvature_usable ? ego.curvature : reference_start_curvature,
      config_);
  const auto tangent = std::min_element(
      config_.tangent_scales.begin(), config_.tangent_scales.end(),
      [](double lhs, double rhs) {
        return std::abs(lhs - 1.0) < std::abs(rhs - 1.0);
      });
  if (tangent == config_.tangent_scales.end()) {
    return std::nullopt;
  }

  for (const double goal_d_m : goals) {
    double terminal_distance = requiredLateralTransitionDistance(
        config_, ego.speed_mps, ego.frenet.d, goal_d_m);
    // This fallback exists specifically because the obstacle-truncated normal
    // transition is too short. A real lateral stage uses the full bounded
    // lattice distance so it remains gradual and trackable. The measured
    // current-d hold has no lateral transition: keep its normal forward
    // horizon instead of either truncating it to the 1 m obstacle deadline or
    // extending it through the forward target.
    terminal_distance = std::max(terminal_distance, config_.max_distance_m);
    terminal_distance = std::clamp(
        terminal_distance, config_.minimum_lateral_transition_distance_m,
        config_.max_distance_m);
    const auto reference_goal =
        frame_->interpolate(ego.frenet.s + terminal_distance);
    const double denominator = 1.0 - goal_d_m * reference_goal.kappa;
    if (!std::isfinite(denominator) || denominator <= 1.0e-3) {
      continue;
    }
    const auto goal_point =
        frame_->frenetToCartesian(ego.frenet.s + terminal_distance, goal_d_m);
    const Pose2d goal{goal_point.x, goal_point.y, reference_goal.yaw};
    const double goal_curvature =
        sanitizedCurvature(reference_goal.kappa / denominator, config_);
    if (config_.safety_evaluation_enabled &&
        map_->footprintHitsWall(goal, wallFootprint(config_, true))) {
      continue;
    }

    ParametricQuintic polynomial;
    if (!polynomial.configure(ego, start_curvature, goal, goal_curvature,
                              *tangent)) {
      continue;
    }
    CandidateTrajectory candidate;
    candidate.lateral_index = config_.lateral_targets_m.size();
    candidate.tangent_index = static_cast<std::size_t>(
        std::distance(config_.tangent_scales.begin(), tangent));
    candidate.goal_d_m = goal_d_m;
    candidate.tangent_scale = *tangent;
    candidate.required_arc_m = terminal_distance;
    for (int i = 0; i < config_.sampling_points; ++i) {
      candidate.representative.push_back(polynomial.sample(
          static_cast<double>(i) / (config_.sampling_points - 1)));
    }
    candidate.dense = denseSamples(polynomial);
    evaluateTrajectory(&candidate, worst_case_opponents,
                       std::abs(ego.speed_mps));
    if (candidate.feasible) {
      return candidate;
    }
  }
  return std::nullopt;
}

bool LatticePlanner::sideRolePeerSeparationAtPoint(
    const TrajectoryPoint &point, const SideRoleSeparationReference &reference,
    int sample_index, bool interpolated_sample) const {
  if (reference.peer == nullptr || !reference.peer->valid ||
      !std::isfinite(point.time_sec) || point.time_sec < 0.0 ||
      !std::isfinite(point.d)) {
    if (reference.diagnostic != nullptr &&
        reference.diagnostic->failure ==
            PreventiveSideRoleCandidateFailure::NONE) {
      reference.diagnostic->failure = reference.failure;
      reference.diagnostic->violating_sample_index = sample_index;
      reference.diagnostic->violating_interpolated_sample = interpolated_sample;
    }
    return false;
  }
  OpponentState predicted_peer = *reference.peer;
  predicted_peer.x += predicted_peer.vx_mps * point.time_sec;
  predicted_peer.y += predicted_peer.vy_mps * point.time_sec;
  predicted_peer.frenet =
      frame_->project(predicted_peer.x, predicted_peer.y, predicted_peer.yaw);
  predicted_peer.valid = predicted_peer.frenet.valid;
  // The collision diagnostic applies the same constant-velocity prediction
  // internally. Use the current peer there to avoid predicting it twice;
  // predicted_peer above is only for the Frenet lateral-gap projection.
  const auto clearance = opponentCollisionDiagnostic(point, {*reference.peer});
  const double lateral_gap_m = predicted_peer.valid
                                   ? std::abs(predicted_peer.frenet.d - point.d)
                                   : std::numeric_limits<double>::quiet_NaN();
  const double epsilon_m = config_.preventive_side_role_separation_epsilon_m;
  const bool cartesian_clearance_safe =
      std::isfinite(clearance.clearance_m) &&
      clearance.clearance_m + epsilon_m >= config_.opponent_hard_clearance_m;
  if (cartesian_clearance_safe) {
    return true;
  }
  if (reference.diagnostic != nullptr &&
      reference.diagnostic->failure ==
          PreventiveSideRoleCandidateFailure::NONE) {
    reference.diagnostic->failure = reference.failure;
    reference.diagnostic->violating_sample_index = sample_index;
    reference.diagnostic->violating_interpolated_sample = interpolated_sample;
    reference.diagnostic->observed_lateral_gap_m = lateral_gap_m;
    reference.diagnostic->observed_clearance_m = clearance.clearance_m;
  }
  return false;
}

bool LatticePlanner::validateSideRolePeerSeparation(
    const EgoState &ego, const OpponentState &peer,
    const std::vector<TrajectoryPoint> &trajectory,
    PreventiveSideRoleCandidateDiagnostic *diagnostic) const {
  TrajectoryPoint initial_pose;
  initial_pose.x = ego.x;
  initial_pose.y = ego.y;
  initial_pose.yaw = ego.yaw;
  initial_pose.s = ego.frenet.s;
  initial_pose.d = ego.frenet.d;
  initial_pose.kappa = sanitizedCurvature(ego.curvature, config_);
  initial_pose.time_sec = 0.0;
  const double initial_lateral_gap_m = std::abs(peer.frenet.d - ego.frenet.d);
  const double initial_clearance_m =
      opponentCollisionDiagnostic(initial_pose, {peer}).clearance_m;
  if (diagnostic != nullptr) {
    diagnostic->peer_id = peer.id;
    diagnostic->initial_lateral_gap_m = initial_lateral_gap_m;
    diagnostic->initial_clearance_m = initial_clearance_m;
  }
  if (!ego.valid || !peer.valid || trajectory.empty() ||
      !std::isfinite(initial_lateral_gap_m) ||
      !std::isfinite(initial_clearance_m)) {
    if (diagnostic != nullptr) {
      diagnostic->failure =
          PreventiveSideRoleCandidateFailure::DENSE_PEER_SEPARATION;
    }
    return false;
  }
  const SideRoleSeparationReference reference{
      &peer, initial_lateral_gap_m, initial_clearance_m, diagnostic,
      PreventiveSideRoleCandidateFailure::DENSE_PEER_SEPARATION};
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    if (!sideRolePeerSeparationAtPoint(trajectory[i], reference,
                                       static_cast<int>(i), false)) {
      return false;
    }
  }
  return true;
}

LatticePlanner::PreventiveSideRoleDecision
LatticePlanner::detectPreventiveSideRole(
    const EgoState &ego, const std::vector<OpponentState> &observed_opponents,
    const std::vector<OpponentState> &planning_opponents, double now_sec) {
  PreventiveSideRoleDecision decision;
  TrajectoryPoint current_pose;
  current_pose.x = ego.x;
  current_pose.y = ego.y;
  current_pose.yaw = ego.yaw;
  current_pose.s = ego.frenet.s;
  current_pose.d = ego.frenet.d;
  current_pose.kappa = sanitizedCurvature(ego.curvature, config_);
  current_pose.time_sec = 0.0;

  const auto ego_observation_valid = [this, &ego, now_sec]() {
    return std::isfinite(now_sec) && std::isfinite(ego.stamp_sec) &&
           ego.stamp_sec <= now_sec + config_.tie_break_epsilon &&
           now_sec - ego.stamp_sec <= config_.ego_stale_sec && ego.valid &&
           std::isfinite(ego.x) && std::isfinite(ego.y) &&
           std::isfinite(ego.yaw) && std::isfinite(ego.speed_mps) &&
           ego.speed_mps >= 0.0 && ego.frenet.valid &&
           std::isfinite(ego.frenet.s) && std::isfinite(ego.frenet.d) &&
           std::isfinite(ego.frenet.yaw_error);
  };
  const auto peer_observation_valid = [this,
                                       now_sec](const OpponentState &opponent) {
    return opponent.valid && opponent.frenet.valid &&
           std::isfinite(opponent.stamp_sec) &&
           opponent.stamp_sec <= now_sec + config_.tie_break_epsilon &&
           now_sec - opponent.stamp_sec <= config_.opponent_stale_sec &&
           std::isfinite(opponent.x) && std::isfinite(opponent.y) &&
           std::isfinite(opponent.yaw) && std::isfinite(opponent.speed_mps) &&
           std::isfinite(opponent.vx_mps) && std::isfinite(opponent.vy_mps) &&
           std::isfinite(opponent.uncertainty_x_m) &&
           std::isfinite(opponent.uncertainty_y_m) &&
           opponent.uncertainty_x_m >= 0.0 && opponent.uncertainty_y_m >= 0.0 &&
           std::isfinite(opponent.frenet.s) &&
           std::isfinite(opponent.frenet.d) &&
           std::isfinite(opponent.frenet.yaw_error);
  };
  struct SidePeer {
    std::size_t index{0U};
    double clearance_m{std::numeric_limits<double>::infinity()};
    PreventiveSideRoleEligibilityDiagnostic diagnostic;
  };
  const auto predict_clearance =
      [this, &ego](const OpponentState &planned, double horizon_sec,
                   PreventiveSideRoleEligibilityDiagnostic *diagnostic) {
        double minimum_clearance_m = std::numeric_limits<double>::infinity();
        double time_to_hard_sec = std::numeric_limits<double>::infinity();
        const double ego_vx_mps = ego.speed_mps * std::cos(ego.yaw);
        const double ego_vy_mps = ego.speed_mps * std::sin(ego.yaw);
        const double relative_speed_mps = std::hypot(
            planned.vx_mps - ego_vx_mps, planned.vy_mps - ego_vy_mps);
        const double adaptive_step_sec =
            relative_speed_mps > config_.tie_break_epsilon
                ? std::min(config_.preventive_side_role_prediction_step_sec,
                           config_.collision_max_step_m / relative_speed_mps)
                : config_.preventive_side_role_prediction_step_sec;
        constexpr int kMaximumPredictiveSamples = 2048;
        const double required_steps =
            std::ceil(horizon_sec / adaptive_step_sec);
        if (!std::isfinite(adaptive_step_sec) || adaptive_step_sec <= 0.0 ||
            !std::isfinite(required_steps) ||
            required_steps > kMaximumPredictiveSamples) {
          if (diagnostic != nullptr) {
            diagnostic->predicted_min_clearance_m =
                -std::numeric_limits<double>::infinity();
            diagnostic->predicted_time_to_hard_sec = 0.0;
          }
          return std::make_pair(-std::numeric_limits<double>::infinity(), 0.0);
        }
        const int steps = std::max(1, static_cast<int>(required_steps));
        for (int step = 0; step <= steps; ++step) {
          const double time_sec =
              std::min(horizon_sec, step * adaptive_step_sec);
          TrajectoryPoint predicted_ego;
          predicted_ego.x =
              ego.x + ego.speed_mps * std::cos(ego.yaw) * time_sec;
          predicted_ego.y =
              ego.y + ego.speed_mps * std::sin(ego.yaw) * time_sec;
          predicted_ego.yaw = ego.yaw;
          predicted_ego.time_sec = time_sec;
          const double clearance_m =
              opponentCollisionDiagnostic(predicted_ego, {planned}).clearance_m;
          if (clearance_m < minimum_clearance_m) {
            minimum_clearance_m = clearance_m;
          }
          if (!std::isfinite(time_to_hard_sec) &&
              clearance_m <= config_.opponent_hard_clearance_m) {
            time_to_hard_sec = time_sec;
          }
        }
        if (diagnostic != nullptr) {
          diagnostic->predicted_min_clearance_m = minimum_clearance_m;
          diagnostic->predicted_time_to_hard_sec = time_to_hard_sec;
        }
        return std::make_pair(minimum_clearance_m, time_to_hard_sec);
      };
  const auto eligibility = [this, &ego, &ego_observation_valid,
                            &peer_observation_valid, &predict_clearance,
                            now_sec](const OpponentState &planned,
                                     const OpponentState *observed,
                                     std::size_t matching_ids,
                                     double clearance_m, bool entry_checks) {
    PreventiveSideRoleEligibilityDiagnostic diagnostic;
    diagnostic.peer_id = planned.id;
    diagnostic.matching_peer_ids = static_cast<int>(matching_ids);
    diagnostic.clearance_m = clearance_m;
    diagnostic.synchronized_ego_stamp_sec = ego.stamp_sec;
    diagnostic.synchronized_peer_stamp_sec = planned.stamp_sec;
    diagnostic.synchronized_stamp_sec =
        std::max(ego.stamp_sec, planned.stamp_sec);
    const double ego_dt_sec =
        std::max(0.0, diagnostic.synchronized_stamp_sec - ego.stamp_sec);
    const double peer_dt_sec =
        std::max(0.0, diagnostic.synchronized_stamp_sec - planned.stamp_sec);
    const double ego_vx_mps = ego.speed_mps * std::cos(ego.yaw);
    const double ego_vy_mps = ego.speed_mps * std::sin(ego.yaw);
    const double ego_x = ego.x + ego_vx_mps * ego_dt_sec;
    const double ego_y = ego.y + ego_vy_mps * ego_dt_sec;
    const double peer_x = planned.x + planned.vx_mps * peer_dt_sec;
    const double peer_y = planned.y + planned.vy_mps * peer_dt_sec;
    const double forward_x = std::cos(ego.yaw) + std::cos(planned.yaw);
    const double forward_y = std::sin(ego.yaw) + std::sin(planned.yaw);
    const double forward_norm = std::hypot(forward_x, forward_y);
    if (std::isfinite(forward_norm) &&
        forward_norm > config_.tie_break_epsilon) {
      const double common_forward_x = forward_x / forward_norm;
      const double common_forward_y = forward_y / forward_norm;
      const double relative_x = peer_x - ego_x;
      const double relative_y = peer_y - ego_y;
      diagnostic.cartesian_longitudinal_m =
          relative_x * common_forward_x + relative_y * common_forward_y;
      diagnostic.cartesian_lateral_m =
          -relative_x * common_forward_y + relative_y * common_forward_x;
    }
    diagnostic.delta_s_m = diagnostic.cartesian_longitudinal_m;
    diagnostic.lateral_separation_m = std::abs(diagnostic.cartesian_lateral_m);
    diagnostic.ego_tangent_progress_mps =
        ego.speed_mps * std::cos(ego.frenet.yaw_error);
    const double peer_reference_yaw =
        planned.frenet.valid ? frame_->interpolate(planned.frenet.s).yaw
                             : std::numeric_limits<double>::quiet_NaN();
    diagnostic.peer_tangent_progress_mps =
        planned.vx_mps * std::cos(peer_reference_yaw) +
        planned.vy_mps * std::sin(peer_reference_yaw);
    diagnostic.ego_track_heading_error_rad = std::abs(ego.frenet.yaw_error);
    diagnostic.peer_track_heading_error_rad =
        std::abs(planned.frenet.yaw_error);
    diagnostic.relative_heading_error_rad =
        std::abs(normalizeAngle(planned.yaw - ego.yaw));
    diagnostic.ego_relation = classifyCurrentPoseOpponentRelation(ego, planned);

    EgoState peer_view;
    peer_view.valid = planned.valid;
    peer_view.x = planned.x;
    peer_view.y = planned.y;
    peer_view.yaw = planned.yaw;
    OpponentState ego_view;
    ego_view.valid = ego.valid;
    ego_view.x = ego.x;
    ego_view.y = ego.y;
    ego_view.yaw = ego.yaw;
    ego_view.uncertainty_x_m = 0.0;
    ego_view.uncertainty_y_m = 0.0;
    diagnostic.peer_relation =
        classifyCurrentPoseOpponentRelation(peer_view, ego_view);

    const auto reject =
        [&diagnostic](PreventiveSideRoleEligibilityFailure failure) {
          diagnostic.failure = failure;
          return diagnostic;
        };
    if (!ego_observation_valid()) {
      return reject(
          PreventiveSideRoleEligibilityFailure::INVALID_EGO_OBSERVATION);
    }
    if (matching_ids == 0U || observed == nullptr ||
        !peer_observation_valid(*observed) || !planned.valid ||
        !planned.frenet.valid) {
      return reject(
          PreventiveSideRoleEligibilityFailure::INVALID_PEER_OBSERVATION);
    }
    if (matching_ids != 1U) {
      return reject(PreventiveSideRoleEligibilityFailure::DUPLICATE_PEER_ID);
    }
    if (config_.own_vehicle_id.empty() || config_.own_vehicle_id == "auto" ||
        planned.id.empty() || planned.id == config_.own_vehicle_id) {
      return reject(PreventiveSideRoleEligibilityFailure::INVALID_VEHICLE_ID);
    }
    if (!std::isfinite(diagnostic.ego_track_heading_error_rad) ||
        diagnostic.ego_track_heading_error_rad >
            config_.preventive_side_role_max_track_heading_error_rad) {
      return reject(PreventiveSideRoleEligibilityFailure::EGO_TRACK_HEADING);
    }
    if (!std::isfinite(diagnostic.peer_track_heading_error_rad) ||
        diagnostic.peer_track_heading_error_rad >
            config_.preventive_side_role_max_track_heading_error_rad) {
      return reject(PreventiveSideRoleEligibilityFailure::PEER_TRACK_HEADING);
    }
    if (!std::isfinite(diagnostic.synchronized_stamp_sec) ||
        !std::isfinite(diagnostic.cartesian_longitudinal_m) ||
        !std::isfinite(diagnostic.cartesian_lateral_m)) {
      return reject(PreventiveSideRoleEligibilityFailure::SYNCHRONIZATION);
    }
    if (!std::isfinite(diagnostic.relative_heading_error_rad) ||
        diagnostic.relative_heading_error_rad >
            config_.preventive_side_role_max_relative_heading_error_rad) {
      return reject(PreventiveSideRoleEligibilityFailure::RELATIVE_HEADING);
    }
    predict_clearance(planned,
                      entry_checks
                          ? config_.preventive_side_role_prediction_horizon_sec
                          : config_.preventive_side_role_release_prediction_sec,
                      &diagnostic);
    if (std::isfinite(diagnostic.predicted_time_to_hard_sec)) {
      diagnostic.decision_deadline_sec =
          now_sec + diagnostic.predicted_time_to_hard_sec -
          config_.preventive_side_role_deadline_response_margin_sec;
    }
    if (!entry_checks) {
      return diagnostic;
    }
    if (!std::isfinite(diagnostic.delta_s_m) ||
        std::abs(diagnostic.delta_s_m) >
            config_.preventive_side_role_max_abs_delta_s_m) {
      return reject(PreventiveSideRoleEligibilityFailure::DELTA_S);
    }
    if (!std::isfinite(diagnostic.ego_tangent_progress_mps) ||
        diagnostic.ego_tangent_progress_mps <
            config_.preventive_side_role_min_tangent_progress_mps) {
      return reject(PreventiveSideRoleEligibilityFailure::EGO_TANGENT_PROGRESS);
    }
    if (!std::isfinite(diagnostic.peer_tangent_progress_mps) ||
        diagnostic.peer_tangent_progress_mps <
            config_.preventive_side_role_min_tangent_progress_mps) {
      return reject(
          PreventiveSideRoleEligibilityFailure::PEER_TANGENT_PROGRESS);
    }
    if (!std::isfinite(diagnostic.lateral_separation_m) ||
        diagnostic.lateral_separation_m <
            config_.preventive_side_role_min_lateral_separation_m) {
      return reject(PreventiveSideRoleEligibilityFailure::LATERAL_SEPARATION);
    }
    if (!std::isfinite(diagnostic.clearance_m) ||
        diagnostic.clearance_m < config_.opponent_hard_clearance_m) {
      return reject(PreventiveSideRoleEligibilityFailure::HARD_CLEARANCE);
    }
    if (diagnostic.clearance_m >=
        config_.preventive_side_role_trigger_clearance_m) {
      return reject(PreventiveSideRoleEligibilityFailure::ENTRY_ENVELOPE);
    }
    if (!std::isfinite(diagnostic.predicted_time_to_hard_sec)) {
      return reject(
          PreventiveSideRoleEligibilityFailure::PREDICTED_HARD_CLEARANCE);
    }
    if (!std::isfinite(diagnostic.decision_deadline_sec) ||
        diagnostic.decision_deadline_sec <= now_sec) {
      return reject(PreventiveSideRoleEligibilityFailure::DECISION_DEADLINE);
    }
    diagnostic.entry_reason = "cv_predicted_hard_clearance";
    return diagnostic;
  };
  const auto ambiguous_predictive_peer =
      [this, &predict_clearance](
          const OpponentState &planned,
          PreventiveSideRoleEligibilityDiagnostic *diagnostic) {
        if (diagnostic == nullptr ||
            (diagnostic->failure !=
                 PreventiveSideRoleEligibilityFailure::DUPLICATE_PEER_ID &&
             diagnostic->failure !=
                 PreventiveSideRoleEligibilityFailure::INVALID_VEHICLE_ID &&
             diagnostic->failure !=
                 PreventiveSideRoleEligibilityFailure::DECISION_DEADLINE)) {
          return false;
        }
        const bool bounded_side_approach =
            planned.valid && planned.frenet.valid &&
            std::isfinite(diagnostic->delta_s_m) &&
            std::abs(diagnostic->delta_s_m) <=
                config_.preventive_side_role_max_abs_delta_s_m &&
            std::isfinite(diagnostic->lateral_separation_m) &&
            diagnostic->lateral_separation_m >=
                config_.preventive_side_role_min_lateral_separation_m &&
            std::isfinite(diagnostic->clearance_m) &&
            diagnostic->clearance_m >= config_.opponent_hard_clearance_m &&
            diagnostic->clearance_m <
                config_.preventive_side_role_trigger_clearance_m &&
            std::isfinite(diagnostic->ego_tangent_progress_mps) &&
            diagnostic->ego_tangent_progress_mps >=
                config_.preventive_side_role_min_tangent_progress_mps &&
            std::isfinite(diagnostic->peer_tangent_progress_mps) &&
            diagnostic->peer_tangent_progress_mps >=
                config_.preventive_side_role_min_tangent_progress_mps &&
            std::isfinite(diagnostic->ego_track_heading_error_rad) &&
            diagnostic->ego_track_heading_error_rad <=
                config_.preventive_side_role_max_track_heading_error_rad &&
            std::isfinite(diagnostic->peer_track_heading_error_rad) &&
            diagnostic->peer_track_heading_error_rad <=
                config_.preventive_side_role_max_track_heading_error_rad &&
            std::isfinite(diagnostic->relative_heading_error_rad) &&
            diagnostic->relative_heading_error_rad <=
                config_.preventive_side_role_max_relative_heading_error_rad;
        if (!bounded_side_approach) {
          return false;
        }
        predict_clearance(planned,
                          config_.preventive_side_role_prediction_horizon_sec,
                          diagnostic);
        return std::isfinite(diagnostic->predicted_time_to_hard_sec);
      };

  if (preventive_side_role_latch_.has_value()) {
    auto &latch = *preventive_side_role_latch_;
    const auto planned =
        std::find_if(planning_opponents.cbegin(), planning_opponents.cend(),
                     [&latch](const OpponentState &candidate) {
                       return candidate.id == latch.peer_id;
                     });
    const auto observed =
        std::find_if(observed_opponents.cbegin(), observed_opponents.cend(),
                     [&latch](const OpponentState &candidate) {
                       return candidate.id == latch.peer_id;
                     });
    const std::size_t matching_ids = static_cast<std::size_t>(
        std::count_if(observed_opponents.cbegin(), observed_opponents.cend(),
                      [&latch](const OpponentState &candidate) {
                        return candidate.id == latch.peer_id;
                      }));
    if (planned == planning_opponents.cend() ||
        observed == observed_opponents.cend()) {
      decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
      decision.reason = "preventive_side_role_peer_dropout";
      decision.diagnostic.failure =
          PreventiveSideRoleEligibilityFailure::PEER_DROPOUT;
      return decision;
    }
    const auto collision =
        opponentCollisionDiagnostic(current_pose, {*planned});
    decision.diagnostic = eligibility(*planned, &*observed, matching_ids,
                                      collision.clearance_m, false);
    if (matching_ids != 1U) {
      decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
      decision.reason = "preventive_side_role_ambiguous_peer";
      decision.diagnostic.failure =
          PreventiveSideRoleEligibilityFailure::DUPLICATE_PEER_ID;
      return decision;
    }
    if (decision.diagnostic.failure !=
        PreventiveSideRoleEligibilityFailure::NONE) {
      decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
      decision.reason = "preventive_side_role_release_unverified";
      return decision;
    }
    for (const auto &other_planned : planning_opponents) {
      if (other_planned.id == latch.peer_id) {
        continue;
      }
      const auto other_observed =
          std::find_if(observed_opponents.cbegin(), observed_opponents.cend(),
                       [&other_planned](const OpponentState &candidate) {
                         return candidate.id == other_planned.id;
                       });
      const std::size_t other_matching_ids = static_cast<std::size_t>(
          std::count_if(observed_opponents.cbegin(), observed_opponents.cend(),
                        [&other_planned](const OpponentState &candidate) {
                          return candidate.id == other_planned.id;
                        }));
      const auto other_collision =
          opponentCollisionDiagnostic(current_pose, {other_planned});
      auto other_diagnostic = eligibility(
          other_planned,
          other_observed == observed_opponents.cend() ? nullptr
                                                      : &*other_observed,
          other_matching_ids, other_collision.clearance_m, true);
      const bool second_eligible = other_diagnostic.failure ==
                                   PreventiveSideRoleEligibilityFailure::NONE;
      const bool ambiguous_other =
          ambiguous_predictive_peer(other_planned, &other_diagnostic);
      if (second_eligible || ambiguous_other) {
        decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
        decision.reason = second_eligible
                              ? "preventive_side_role_multiple_peers"
                              : "preventive_side_role_ambiguous_peer";
        decision.diagnostic = other_diagnostic;
        if (second_eligible) {
          decision.diagnostic.failure =
              PreventiveSideRoleEligibilityFailure::MULTIPLE_PEERS;
        }
        return decision;
      }
    }

    const bool distinct_observation =
        observed->stamp_sec >
        latch.last_observation_stamp_sec + config_.tie_break_epsilon;
    if (distinct_observation && std::isfinite(latch.previous_peer_x_m) &&
        std::hypot(observed->x - latch.previous_peer_x_m,
                   observed->y - latch.previous_peer_y_m) >
            config_.opponent_max_position_jump_m) {
      decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
      decision.reason = "preventive_side_role_position_jump";
      decision.diagnostic.failure =
          PreventiveSideRoleEligibilityFailure::POSITION_JUMP;
      return decision;
    }
    if (distinct_observation) {
      latch.last_observation_stamp_sec = observed->stamp_sec;
      latch.previous_peer_x_m = observed->x;
      latch.previous_peer_y_m = observed->y;
      const bool role_reversed =
          (latch.role == PreventiveSideRoleKind::FOLLOWER &&
           decision.diagnostic.cartesian_longitudinal_m <
               -config_.preventive_side_role_tie_band_m) ||
          (latch.role == PreventiveSideRoleKind::LEADER &&
           decision.diagnostic.cartesian_longitudinal_m >
               config_.preventive_side_role_tie_band_m);
      if (role_reversed) {
        decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
        decision.reason = "preventive_side_role_geometry_changed";
        decision.diagnostic.failure =
            PreventiveSideRoleEligibilityFailure::ROLE_CHANGED;
        return decision;
      }
      if (latch.role == PreventiveSideRoleKind::NEUTRAL) {
        PreventiveSideRoleKind observed_role = PreventiveSideRoleKind::NEUTRAL;
        if (decision.diagnostic.cartesian_longitudinal_m >
            config_.preventive_side_role_tie_band_m) {
          observed_role = PreventiveSideRoleKind::FOLLOWER;
        } else if (decision.diagnostic.cartesian_longitudinal_m <
                   -config_.preventive_side_role_tie_band_m) {
          observed_role = PreventiveSideRoleKind::LEADER;
        }
        if (observed_role == PreventiveSideRoleKind::NEUTRAL) {
          latch.pending_role = PreventiveSideRoleKind::NONE;
          latch.role_confirmation_count = 0;
        } else {
          if (latch.pending_role != observed_role) {
            latch.pending_role = observed_role;
            latch.role_confirmation_count = 0;
          }
          ++latch.role_confirmation_count;
          if (latch.role_confirmation_count >=
              config_.preventive_side_role_role_confirm_samples) {
            latch.role = observed_role;
            latch.phase = observed_role == PreventiveSideRoleKind::LEADER
                              ? PreventiveSideRolePhase::LEADER_PROCEED
                              : PreventiveSideRolePhase::FOLLOWER_YIELD;
            latch.exit_sample_count = 0;
          }
        }
      }
    }

    if (latch.role == PreventiveSideRoleKind::NEUTRAL &&
        std::isfinite(latch.decision_deadline_sec) &&
        now_sec >= latch.decision_deadline_sec) {
      decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
      decision.reason = "preventive_side_role_decision_deadline";
      decision.diagnostic.failure =
          PreventiveSideRoleEligibilityFailure::DECISION_DEADLINE;
      return decision;
    }
    const double input_age_sec = std::max(0.0, now_sec - observed->stamp_sec);
    const double relative_speed_mps =
        std::max(0.0, decision.diagnostic.ego_tangent_progress_mps -
                          decision.diagnostic.peer_tangent_progress_mps);
    const double braking_gap_m = relative_speed_mps * relative_speed_mps /
                                 std::max(config_.tie_break_epsilon,
                                          -2.0 * config_.min_acceleration_mps2);
    decision.required_gap_m =
        config_.preventive_side_role_follower_min_gap_m +
        relative_speed_mps *
            (input_age_sec +
             config_.preventive_side_role_controller_response_sec) +
        braking_gap_m + observed->uncertainty_x_m +
        config_.preventive_side_role_release_margin_m;
    const bool release_clearance =
        collision.clearance_m >= config_.preventive_side_role_exit_clearance_m;
    const bool short_horizon_safe =
        decision.diagnostic.predicted_min_clearance_m >=
        config_.opponent_hard_clearance_m;
    const bool geometry_exited =
        release_clearance && short_horizon_safe &&
        std::isfinite(decision.diagnostic.cartesian_longitudinal_m) &&
        std::abs(decision.diagnostic.cartesian_longitudinal_m) >=
            decision.required_gap_m &&
        decision.diagnostic.ego_relation !=
            CurrentPoseOpponentRelation::SIDE_OVERLAP &&
        decision.diagnostic.peer_relation !=
            CurrentPoseOpponentRelation::SIDE_OVERLAP;
    if (distinct_observation && latch.role != PreventiveSideRoleKind::NEUTRAL) {
      latch.exit_sample_count =
          geometry_exited ? latch.exit_sample_count + 1 : 0;
    }
    if (latch.exit_sample_count >= config_.preventive_side_role_exit_samples) {
      decision.status = PreventiveSideRoleDecisionStatus::RELEASED;
      decision.peer_index = static_cast<std::size_t>(
          std::distance(planning_opponents.cbegin(), planned));
      decision.clearance_m = collision.clearance_m;
      return decision;
    }
    decision.status = PreventiveSideRoleDecisionStatus::ACTIVE;
    decision.role = latch.role;
    decision.peer_index = static_cast<std::size_t>(
        std::distance(planning_opponents.cbegin(), planned));
    decision.clearance_m = collision.clearance_m;
    return decision;
  }

  std::vector<SidePeer> eligible_peers;
  bool ambiguous_peer = false;
  PreventiveSideRoleEligibilityDiagnostic first_ambiguous_peer;
  double closest_clearance_m = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < planning_opponents.size(); ++i) {
    const auto &planned = planning_opponents[i];
    const auto observed =
        std::find_if(observed_opponents.cbegin(), observed_opponents.cend(),
                     [&planned](const OpponentState &candidate) {
                       return candidate.id == planned.id;
                     });
    const std::size_t matching_ids = static_cast<std::size_t>(
        std::count_if(observed_opponents.cbegin(), observed_opponents.cend(),
                      [&planned](const OpponentState &candidate) {
                        return candidate.id == planned.id;
                      }));
    const auto collision = opponentCollisionDiagnostic(current_pose, {planned});
    auto diagnostic = eligibility(
        planned, observed == observed_opponents.cend() ? nullptr : &*observed,
        matching_ids, collision.clearance_m, true);
    if (collision.clearance_m < closest_clearance_m) {
      closest_clearance_m = collision.clearance_m;
      decision.diagnostic = diagnostic;
    }
    if (diagnostic.failure == PreventiveSideRoleEligibilityFailure::NONE) {
      eligible_peers.push_back(SidePeer{i, collision.clearance_m, diagnostic});
    } else if (ambiguous_predictive_peer(planned, &diagnostic)) {
      if (!ambiguous_peer) {
        first_ambiguous_peer = diagnostic;
      }
      ambiguous_peer = true;
    }
  }

  if (eligible_peers.size() > 1U || ambiguous_peer) {
    decision.status = PreventiveSideRoleDecisionStatus::AMBIGUOUS;
    decision.reason = eligible_peers.size() > 1U
                          ? "preventive_side_role_multiple_peers"
                          : "preventive_side_role_ambiguous_peer";
    decision.diagnostic = eligible_peers.size() > 1U
                              ? eligible_peers.front().diagnostic
                              : first_ambiguous_peer;
    if (eligible_peers.size() > 1U) {
      decision.diagnostic.failure =
          PreventiveSideRoleEligibilityFailure::MULTIPLE_PEERS;
      decision.diagnostic.matching_peer_ids =
          static_cast<int>(eligible_peers.size());
    }
    return decision;
  }
  if (eligible_peers.empty()) {
    return decision;
  }

  const auto &peer = eligible_peers.front();
  decision.status = PreventiveSideRoleDecisionStatus::ACTIVE;
  decision.peer_index = peer.index;
  decision.clearance_m = peer.clearance_m;
  decision.diagnostic = peer.diagnostic;
  if (peer.diagnostic.cartesian_longitudinal_m >
      config_.preventive_side_role_tie_band_m) {
    decision.role = PreventiveSideRoleKind::FOLLOWER;
  } else if (peer.diagnostic.cartesian_longitudinal_m <
             -config_.preventive_side_role_tie_band_m) {
    decision.role = PreventiveSideRoleKind::LEADER;
  } else {
    decision.role = PreventiveSideRoleKind::NEUTRAL;
  }
  return decision;
}

LatticePlanner::RoleCandidateEvaluation LatticePlanner::evaluateRoleCandidate(
    const EgoState &ego, const OpponentState &peer,
    const std::vector<OpponentState> &all_opponents,
    const CandidateTrajectory &candidate, double target_speed_mps,
    std::uint64_t generation) const {
  RoleCandidateEvaluation evaluation;
  evaluation.role_diagnostic.peer_id = peer.id;
  evaluation.role_diagnostic.evaluation_generation = generation;
  // These dense values are bounded observability only. They must not decide
  // admission: outputHorizon() and the role-aware resampled final horizon below
  // remain the authoritative wall/all-opponent/trackability checks.
  const double footprint_radius_m =
      std::hypot(std::max(config_.front_overhang_m + config_.wheel_base_m,
                          config_.rear_overhang_m),
                 std::max(config_.left_extent_m, config_.right_extent_m));
  for (const auto &point : candidate.dense) {
    const auto opponent_diagnostic =
        opponentCollisionDiagnostic(point, all_opponents);
    evaluation.role_diagnostic.minimum_opponent_clearance_m =
        std::min(evaluation.role_diagnostic.minimum_opponent_clearance_m,
                 opponent_diagnostic.clearance_m);
    int cell_x = 0;
    int cell_y = 0;
    if (map_->worldToCell(point.x, point.y, &cell_x, &cell_y)) {
      evaluation.role_diagnostic.minimum_wall_clearance_m =
          std::min(evaluation.role_diagnostic.minimum_wall_clearance_m,
                   map_->wallDistance(cell_x, cell_y) - footprint_radius_m);
    }
  }
  if (!candidate.feasible) {
    evaluation.role_diagnostic.failure =
        PreventiveSideRoleCandidateFailure::NO_BASE_FEASIBLE_CANDIDATE;
    evaluation.role_diagnostic.first_reject_reason = candidate.rejection_reason;
    return evaluation;
  }
  if (!outputHorizon(ego, candidate, all_opponents, target_speed_mps,
                     &evaluation.lateral_offsets_m, &evaluation.speed_caps_mps,
                     &evaluation.longitudinal_offsets_m,
                     &evaluation.output_diagnostic)) {
    evaluation.role_diagnostic.failure =
        PreventiveSideRoleCandidateFailure::OUTPUT_HORIZON;
    evaluation.role_diagnostic.first_reject_reason =
        toString(evaluation.output_diagnostic.failure);
    return evaluation;
  }
  PreventiveSideRoleCandidateDiagnostic final_diagnostic =
      evaluation.role_diagnostic;
  if (!validateSideRoleOutputHorizon(
          ego, peer, all_opponents, evaluation.lateral_offsets_m,
          evaluation.speed_caps_mps, evaluation.longitudinal_offsets_m,
          &evaluation.output_diagnostic, &final_diagnostic)) {
    evaluation.role_diagnostic = std::move(final_diagnostic);
    if (evaluation.role_diagnostic.failure ==
        PreventiveSideRoleCandidateFailure::NONE) {
      evaluation.role_diagnostic.failure =
          PreventiveSideRoleCandidateFailure::RESAMPLED_PEER_SEPARATION;
    }
    evaluation.role_diagnostic.first_reject_reason =
        toString(evaluation.role_diagnostic.failure);
    return evaluation;
  }
  evaluation.role_diagnostic = std::move(final_diagnostic);
  evaluation.role_diagnostic.failure = PreventiveSideRoleCandidateFailure::NONE;
  evaluation.safe = true;
  return evaluation;
}

bool LatticePlanner::evaluateTrajectory(
    CandidateTrajectory *candidate, const std::vector<OpponentState> &opponents,
    double initial_speed_mps) const {
  if (candidate != nullptr) {
    candidate->feasible = false;
    candidate->requires_entry_deceleration = false;
    candidate->dynamic_speed_limit_mps = 0.0;
    candidate->entry_speed_limit_mps = 0.0;
  }
  if (candidate == nullptr || candidate->representative.size() != 5U ||
      candidate->dense.size() < 2U) {
    return false;
  }
  if (config_.safety_evaluation_enabled &&
      hasSelfIntersection(candidate->dense)) {
    candidate->rejection_reason = "self_intersection";
    return false;
  }
  if (frame_ == nullptr || frame_->empty() || map_ == nullptr ||
      !map_->initialized()) {
    candidate->rejection_reason = "planner_environment";
    return false;
  }

  const std::size_t count = candidate->dense.size();
  std::vector<double> segment_distance(count, 0.0);
  std::vector<double> speed_limits(count, config_.normal_speed_mps);
  std::vector<double> expected_speeds(count, config_.safe_stop_speed_mps);
  double expected_s =
      frame_
          ->project(candidate->dense.front().x, candidate->dense.front().y,
                    candidate->dense.front().yaw)
          .s;
  double previous_steer =
      std::atan(config_.wheel_base_m * candidate->dense.front().kappa);
  double total_steer_change = 0.0;
  const double maximum_steer = effectiveMaximumSteer(config_);

  for (std::size_t i = 0; i < count; ++i) {
    auto &point = candidate->dense[i];
    if (!finitePoint(point)) {
      candidate->rejection_reason = "non_finite";
      return false;
    }
    const auto projected = frame_->projectContinuous(
        point.x, point.y, point.yaw, expected_s,
        i == 0U ? config_.projection_initial_half_width_m
                : config_.projection_follow_half_width_m);
    if (!projected.valid ||
        (i > 0U &&
         projected.s < expected_s - config_.projection_max_backward_m)) {
      candidate->rejection_reason = "frenet_projection";
      return false;
    }
    point.s = std::max(expected_s, projected.s);
    point.d = projected.d;
    expected_s = point.s;

    // Static wall clearance does not depend on the speed profile. Reject an
    // invalid corridor immediately instead of finishing every dynamic check.
    if (config_.safety_evaluation_enabled &&
        map_->footprintHitsWall(point, wallFootprint(config_, true))) {
      candidate->rejection_reason = "wall_collision";
      return false;
    }

    const double steer = std::atan(config_.wheel_base_m * point.kappa);
    if (!std::isfinite(steer) || std::abs(steer) > maximum_steer + 1.0e-6) {
      candidate->rejection_reason = "maximum_curvature";
      return false;
    }
    speed_limits[i] = std::min({config_.normal_speed_mps,
                                curveSpeedLimit(point.kappa, config_),
                                referenceSpeedLimit(frame_, point.s, config_)});
    if (!std::isfinite(speed_limits[i]) ||
        speed_limits[i] < config_.safe_stop_speed_mps - 1.0e-9) {
      candidate->rejection_reason = "curve_speed_below_safe_stop";
      return false;
    }

    if (i > 0U) {
      const auto &previous_point = candidate->dense[i - 1U];
      const double ds =
          std::hypot(point.x - previous_point.x, point.y - previous_point.y);
      segment_distance[i] = ds;
      const double steer_change = std::abs(steer - previous_steer);
      if (ds <= 1.0e-8 && steer_change > 1.0e-8) {
        candidate->rejection_reason = "zero_length_steering_change";
        return false;
      }
      if (steer_change > 1.0e-8) {
        const double steering_rate_speed_limit =
            config_.max_steer_rate_radps * ds / steer_change;
        if (!std::isfinite(steering_rate_speed_limit) ||
            steering_rate_speed_limit < config_.safe_stop_speed_mps - 1.0e-9) {
          candidate->rejection_reason = "steering_rate_below_safe_stop";
          return false;
        }
        speed_limits[i - 1U] =
            std::min(speed_limits[i - 1U], steering_rate_speed_limit);
        speed_limits[i] = std::min(speed_limits[i], steering_rate_speed_limit);
      }
      total_steer_change += steer_change;
    }
    previous_steer = steer;
  }

  // Propagate future steering/curvature limits backwards using the configured
  // deceleration bound. The old implementation rejected every candidate at a
  // fixed normal-speed assumption; this profile instead publishes the speed
  // at which the candidate satisfies steering and curvature constraints.
  for (std::size_t i = count - 1U; i > 0U; --i) {
    const double reachable_previous = std::sqrt(std::max(
        0.0, speed_limits[i] * speed_limits[i] -
                 2.0 * config_.min_acceleration_mps2 * segment_distance[i]));
    speed_limits[i - 1U] = std::min(speed_limits[i - 1U], reachable_previous);
  }
  candidate->entry_speed_limit_mps = speed_limits.front();
  candidate->dynamic_speed_limit_mps =
      *std::min_element(speed_limits.begin(), speed_limits.end());
  const bool requires_entry_deceleration =
      std::max(0.0, initial_speed_mps) >
      candidate->entry_speed_limit_mps +
          config_.candidate_entry_speed_tolerance_mps;

  const double initial_speed = std::max(0.0, initial_speed_mps);
  expected_speeds.front() =
      std::clamp(std::min(std::max(config_.safe_stop_speed_mps, initial_speed),
                          speed_limits.front()),
                 config_.safe_stop_speed_mps, speed_limits.front());
  candidate->dense.front().time_sec = 0.0;
  candidate->dense.front().speed_mps = speed_limits.front();

  for (std::size_t i = 1U; i < count; ++i) {
    const double ds = segment_distance[i];
    const double maximum_reachable = std::sqrt(
        std::max(0.0, expected_speeds[i - 1U] * expected_speeds[i - 1U] +
                          2.0 * config_.max_acceleration_mps2 * ds));
    const double minimum_reachable = std::sqrt(
        std::max(0.0, expected_speeds[i - 1U] * expected_speeds[i - 1U] +
                          2.0 * config_.min_acceleration_mps2 * ds));
    expected_speeds[i] = std::min(speed_limits[i], maximum_reachable);
    if (expected_speeds[i] + 1.0e-6 < minimum_reachable) {
      candidate->rejection_reason = "deceleration_profile";
      return false;
    }
    expected_speeds[i] =
        std::max(config_.safe_stop_speed_mps, expected_speeds[i]);
    const double average_speed =
        std::max(config_.safe_stop_speed_mps,
                 0.5 * (expected_speeds[i - 1U] + expected_speeds[i]));
    candidate->dense[i].time_sec =
        candidate->dense[i - 1U].time_sec + ds / average_speed;
    candidate->dense[i].speed_mps = speed_limits[i];
  }

  for (const auto &point : candidate->dense) {
    if (config_.safety_evaluation_enabled &&
        opponentCollision(point, opponents)) {
      // The generic diagnostic reports only the nearest opponent. A t=0
      // exemption is valid only if every simultaneous collision is a proven
      // rear-only exempt opponent.
      bool collision_exempt_at_current_pose = true;
      for (const auto &opponent : opponents) {
        if (opponentCollisionDiagnostic(point, {opponent}).collision() &&
            !currentPoseRearOnlyExemptionApplies(point, opponent)) {
          collision_exempt_at_current_pose = false;
          break;
        }
      }
      if (collision_exempt_at_current_pose) {
        continue;
      }
      candidate->rejection_reason = "opponent_collision";
      return false;
    }
  }

  std::vector<int> costs;
  std::vector<int> safety_costs;
  std::vector<int> reference_costs;
  std::vector<int> wall_costs;
  std::vector<int> object_costs;
  costs.reserve(candidate->representative.size());
  safety_costs.reserve(candidate->representative.size());
  reference_costs.reserve(candidate->representative.size());
  wall_costs.reserve(candidate->representative.size());
  object_costs.reserve(candidate->representative.size());
  for (std::size_t representative_index = 0U;
       representative_index < candidate->representative.size();
       ++representative_index) {
    auto &point = candidate->representative[representative_index];
    const double expected_representative_s = sAtU(candidate->dense, point.u);
    const auto projected = frame_->projectContinuous(
        point.x, point.y, point.yaw, expected_representative_s,
        config_.projection_follow_half_width_m);
    if (!projected.valid) {
      candidate->rejection_reason = "representative_projection";
      return false;
    }
    point.s = projected.s;
    point.d = projected.d;
    point.time_sec = timeAtU(candidate->dense, point.u);
    const auto cost = poseCostBreakdown(point, opponents);
    costs.push_back(cost.total_cost);
    safety_costs.push_back(cost.safety_cost);
    reference_costs.push_back(cost.reference_cost);
    wall_costs.push_back(cost.wall_cost);
    object_costs.push_back(cost.object_cost);
    if (representative_index <
        candidate->representative_object_diagnostics.size()) {
      auto &object_diagnostic =
          candidate->representative_object_diagnostics[representative_index];
      object_diagnostic.winner_valid = cost.representative_object_winner_valid;
      object_diagnostic.prediction_valid =
          cost.representative_object_prediction_valid;
      object_diagnostic.object_level = cost.representative_object_level;
      object_diagnostic.clearance_m = cost.representative_object_clearance_m;
      object_diagnostic.prediction_horizon_sec = point.time_sec;
      if (cost.representative_object_opponent != nullptr) {
        const auto &opponent_id = cost.representative_object_opponent->id;
        const std::size_t copy_size = std::min(
            opponent_id.size(), object_diagnostic.opponent_id.size() - 1U);
        std::copy_n(opponent_id.data(), copy_size,
                    object_diagnostic.opponent_id.data());
        object_diagnostic.opponent_id[copy_size] = '\0';
        object_diagnostic.opponent_id_truncated =
            opponent_id.size() >= object_diagnostic.opponent_id.size();
      }
      candidate->representative_object_diagnostic_count =
          representative_index + 1U;
    }
    if (cost.nominal_wall_clearance_proxy_valid &&
        (!candidate->representative_wall_diagnostic_valid ||
         cost.nominal_wall_clearance_proxy_m <
             candidate
                 ->representative_minimum_nominal_wall_clearance_proxy_m)) {
      candidate->representative_wall_diagnostic_valid = true;
      candidate->representative_minimum_nominal_wall_clearance_proxy_m =
          cost.nominal_wall_clearance_proxy_m;
      candidate->representative_minimum_nominal_wall_clearance_point_index =
          static_cast<int>(representative_index);
    }
    if (cost.nominal_wall_level >
        candidate->representative_maximum_nominal_wall_level) {
      candidate->representative_maximum_nominal_wall_level =
          cost.nominal_wall_level;
      candidate->representative_maximum_nominal_wall_level_point_index =
          static_cast<int>(representative_index);
    }
  }
  candidate->total_cost = trajectoryCost(costs);
  candidate->safety_cost = trajectoryCost(safety_costs);
  candidate->reference_cost = trajectoryCost(reference_costs);
  candidate->wall_cost = trajectoryCost(wall_costs);
  candidate->object_cost = trajectoryCost(object_costs);
  candidate->total_abs_steer_change = total_steer_change;
  candidate->requires_entry_deceleration = requires_entry_deceleration;
  candidate->feasible = !requires_entry_deceleration;
  candidate->rejection_reason =
      requires_entry_deceleration ? "entry_speed_exceeds_candidate_limit" : "";
  return candidate->feasible;
}

LatticePlanner::PoseCostBreakdown LatticePlanner::poseCostBreakdown(
    const TrajectoryPoint &pose,
    const std::vector<OpponentState> &opponents) const {
  const int reference_level = referenceCostLevel(pose.d, config_);
  PoseCostBreakdown breakdown;
  breakdown.reference_cost =
      config_.cost_levels[static_cast<std::size_t>(reference_level)];
  if (!config_.safety_evaluation_enabled) {
    breakdown.total_cost = breakdown.reference_cost;
    breakdown.safety_cost = config_.cost_levels.front();
    return breakdown;
  }
  const auto nominal_footprint = nominalFootprint(config_);
  const int wall_level = map_->maxWallLevelInFootprint(pose, nominal_footprint);
  breakdown.nominal_wall_level = wall_level;
  int cell_x = 0;
  int cell_y = 0;
  if (map_->worldToCell(pose.x, pose.y, &cell_x, &cell_y)) {
    const double footprint_radius_m = std::hypot(
        std::max(nominal_footprint.front_m, nominal_footprint.rear_m),
        std::max(nominal_footprint.left_m, nominal_footprint.right_m));
    breakdown.nominal_wall_clearance_proxy_valid = true;
    breakdown.nominal_wall_clearance_proxy_m =
        map_->wallDistance(cell_x, cell_y) - footprint_radius_m;
  }
  int object_level = 0;
  const auto ego_fp = nominalFootprint(config_);
  std::size_t opponent_index = 0U;
  std::size_t winning_opponent_index = 0U;
  for (const auto &opponent : opponents) {
    OpponentState predicted = opponent;
    predicted.x += opponent.vx_mps * pose.time_sec;
    predicted.y += opponent.vy_mps * pose.time_sec;
    Footprint opponent_fp = nominalFootprint(config_);
    opponent_fp.front_m += opponent.uncertainty_x_m;
    opponent_fp.rear_m += opponent.uncertainty_x_m;
    opponent_fp.left_m += opponent.uncertainty_y_m;
    opponent_fp.right_m += opponent.uncertainty_y_m;
    const double clearance_m =
        rectangleClearance(pose, ego_fp, predicted, opponent_fp);
    const int opponent_level = objectCostLevel(clearance_m, config_);
    const bool prediction_valid = std::isfinite(clearance_m);
    const auto *winner = breakdown.representative_object_opponent;
    const bool better_level = opponent_level > object_level;
    const bool equal_level = opponent_level == object_level;
    const bool nonfinite_preferred =
        equal_level && !prediction_valid &&
        breakdown.representative_object_prediction_valid;
    const bool smaller_clearance =
        equal_level && prediction_valid &&
        breakdown.representative_object_prediction_valid &&
        clearance_m < breakdown.representative_object_clearance_m;
    const bool equal_clearance =
        equal_level && prediction_valid &&
        breakdown.representative_object_prediction_valid &&
        clearance_m == breakdown.representative_object_clearance_m;
    const bool lexical_preferred =
        equal_clearance && winner != nullptr && opponent.id < winner->id;
    const bool input_index_preferred = equal_clearance && winner != nullptr &&
                                       opponent.id == winner->id &&
                                       opponent_index < winning_opponent_index;
    if (winner == nullptr || better_level || nonfinite_preferred ||
        smaller_clearance || lexical_preferred || input_index_preferred) {
      object_level = opponent_level;
      breakdown.representative_object_winner_valid = !opponent.id.empty();
      breakdown.representative_object_prediction_valid = prediction_valid;
      breakdown.representative_object_level = opponent_level;
      breakdown.representative_object_clearance_m =
          prediction_valid ? clearance_m
                           : std::numeric_limits<double>::quiet_NaN();
      breakdown.representative_object_opponent = &opponent;
      winning_opponent_index = opponent_index;
    }
    ++opponent_index;
  }
  const int clear_space_cost = config_.cost_levels.front();
  breakdown.wall_cost =
      config_.cost_levels[static_cast<std::size_t>(wall_level)] -
      clear_space_cost;
  breakdown.object_cost =
      config_.cost_levels[static_cast<std::size_t>(object_level)] -
      clear_space_cost;
  breakdown.total_cost =
      config_.cost_levels[static_cast<std::size_t>(mergeCostLevels(
          reference_level, mergeCostLevels(wall_level, object_level)))];
  breakdown.safety_cost =
      config_.cost_levels[static_cast<std::size_t>(
          mergeCostLevels(wall_level, object_level))];
  return breakdown;
}

int LatticePlanner::poseCost(
    const TrajectoryPoint &pose,
    const std::vector<OpponentState> &opponents) const {
  return poseCostBreakdown(pose, opponents).total_cost;
}

bool LatticePlanner::hardCollision(
    const TrajectoryPoint &pose,
    const std::vector<OpponentState> &opponents) const {
  if (!config_.safety_evaluation_enabled) {
    return false;
  }
  if (map_->footprintHitsWall(pose, wallFootprint(config_, true))) {
    return true;
  }
  return opponentCollision(pose, opponents);
}

bool LatticePlanner::opponentCollision(
    const TrajectoryPoint &pose,
    const std::vector<OpponentState> &opponents) const {
  return opponentCollisionDiagnostic(pose, opponents).collision();
}

CollisionDiagnostic LatticePlanner::opponentCollisionDiagnostic(
    const TrajectoryPoint &pose,
    const std::vector<OpponentState> &opponents) const {
  CollisionDiagnostic diagnostic;
  diagnostic.required_clearance_m = config_.opponent_hard_clearance_m;
  if (!config_.safety_evaluation_enabled) {
    return diagnostic;
  }
  Footprint ego_fp = nominalFootprint(config_);
  ego_fp.front_m += config_.opponent_longitudinal_tracking_margin_m;
  ego_fp.rear_m += config_.opponent_longitudinal_tracking_margin_m;
  ego_fp.left_m += config_.opponent_lateral_tracking_margin_m;
  ego_fp.right_m += config_.opponent_lateral_tracking_margin_m;
  for (const auto &opponent : opponents) {
    if (!opponent.valid) {
      continue;
    }
    OpponentState predicted = opponent;
    predicted.x += opponent.vx_mps * pose.time_sec;
    predicted.y += opponent.vy_mps * pose.time_sec;
    auto opponent_fp = nominalFootprint(config_);
    opponent_fp.front_m += opponent.uncertainty_x_m;
    opponent_fp.rear_m += opponent.uncertainty_x_m;
    opponent_fp.left_m += opponent.uncertainty_y_m;
    opponent_fp.right_m += opponent.uncertainty_y_m;
    const double clearance_m =
        rectangleClearance(pose, ego_fp, predicted, opponent_fp);
    if (clearance_m < diagnostic.clearance_m) {
      diagnostic.opponent_id = opponent.id;
      diagnostic.clearance_m = clearance_m;
    }
  }
  if (diagnostic.clearance_m < config_.opponent_hard_clearance_m) {
    diagnostic.kind = CollisionKind::OPPONENT;
  }
  return diagnostic;
}

CurrentPoseOpponentRelation LatticePlanner::classifyCurrentPoseOpponentRelation(
    const EgoState &ego, const OpponentState &opponent) const {
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!opponent.valid || !finite(ego.x) || !finite(ego.y) || !finite(ego.yaw) ||
      !finite(opponent.x) || !finite(opponent.y) || !finite(opponent.yaw) ||
      !finite(opponent.uncertainty_x_m) || !finite(opponent.uncertainty_y_m) ||
      opponent.uncertainty_x_m < 0.0 || opponent.uncertainty_y_m < 0.0) {
    return CurrentPoseOpponentRelation::UNKNOWN;
  }

  Footprint ego_fp = nominalFootprint(config_);
  ego_fp.front_m += config_.opponent_longitudinal_tracking_margin_m;
  ego_fp.rear_m += config_.opponent_longitudinal_tracking_margin_m;
  ego_fp.left_m += config_.opponent_lateral_tracking_margin_m;
  ego_fp.right_m += config_.opponent_lateral_tracking_margin_m;
  Footprint opponent_fp = nominalFootprint(config_);
  opponent_fp.front_m += opponent.uncertainty_x_m;
  opponent_fp.rear_m += opponent.uncertainty_x_m;
  opponent_fp.left_m += opponent.uncertainty_y_m;
  opponent_fp.right_m += opponent.uncertainty_y_m;

  // Project every inflated opponent corner into the ego body frame. Center
  // positions are intentionally not used: an opponent is rear-only only when
  // its entire conservative footprint is behind the ego's conservative rear.
  const double cos_yaw = std::cos(ego.yaw);
  const double sin_yaw = std::sin(ego.yaw);
  double min_x_m = std::numeric_limits<double>::infinity();
  double max_x_m = -std::numeric_limits<double>::infinity();
  double min_y_m = std::numeric_limits<double>::infinity();
  double max_y_m = -std::numeric_limits<double>::infinity();
  for (const auto &corner : footprintCorners(opponent, opponent_fp)) {
    const double dx = corner.x - ego.x;
    const double dy = corner.y - ego.y;
    const double body_x_m = cos_yaw * dx + sin_yaw * dy;
    const double body_y_m = -sin_yaw * dx + cos_yaw * dy;
    if (!finite(body_x_m) || !finite(body_y_m)) {
      return CurrentPoseOpponentRelation::UNKNOWN;
    }
    min_x_m = std::min(min_x_m, body_x_m);
    max_x_m = std::max(max_x_m, body_x_m);
    min_y_m = std::min(min_y_m, body_y_m);
    max_y_m = std::max(max_y_m, body_y_m);
  }
  if (!finite(min_x_m) || !finite(max_x_m) || !finite(min_y_m) ||
      !finite(max_y_m)) {
    return CurrentPoseOpponentRelation::UNKNOWN;
  }
  if (max_x_m < -ego_fp.rear_m) {
    return CurrentPoseOpponentRelation::REAR_ONLY;
  }
  if (min_x_m > ego_fp.front_m) {
    return CurrentPoseOpponentRelation::FRONT_OR_OVERLAP;
  }
  // Any longitudinal overlap is fail-closed, including nominally side-by-side
  // contact and a corner crossing the ego front or rear plane.
  return CurrentPoseOpponentRelation::SIDE_OVERLAP;
}

bool LatticePlanner::rearOnlyCurrentPoseExemptionProvenSafe(
    const EgoState &ego, const OpponentState &predicted_opponent,
    const OpponentState &observed_opponent, double now_sec) const {
  if (classifyCurrentPoseOpponentRelation(ego, predicted_opponent) !=
      CurrentPoseOpponentRelation::REAR_ONLY) {
    return false;
  }
  if (!std::isfinite(now_sec) || !std::isfinite(ego.stamp_sec) ||
      !std::isfinite(observed_opponent.stamp_sec) ||
      observed_opponent.stamp_sec > now_sec + config_.tie_break_epsilon ||
      now_sec - observed_opponent.stamp_sec > config_.opponent_stale_sec ||
      !std::isfinite(ego.speed_mps) || ego.speed_mps < 0.0 ||
      !std::isfinite(observed_opponent.vx_mps) ||
      !std::isfinite(observed_opponent.vy_mps)) {
    return false;
  }
  const double ego_vx_mps = ego.speed_mps * std::cos(ego.yaw);
  const double ego_vy_mps = ego.speed_mps * std::sin(ego.yaw);
  const double relative_forward_speed_mps =
      (observed_opponent.vx_mps - ego_vx_mps) * std::cos(ego.yaw) +
      (observed_opponent.vy_mps - ego_vy_mps) * std::sin(ego.yaw);
  // A rear vehicle that is gaining on ego is not exempt. Equality is safe for
  // this current-pose-only rule because all future samples remain hard-checked.
  return std::isfinite(relative_forward_speed_mps) &&
         relative_forward_speed_mps <= config_.tie_break_epsilon;
}

bool LatticePlanner::currentPoseRearOnlyExemptionApplies(
    const TrajectoryPoint &pose, const OpponentState &opponent) const {
  return std::abs(pose.time_sec) <= 1.0e-9 &&
         std::find(current_pose_rear_only_exempt_opponent_ids_.begin(),
                   current_pose_rear_only_exempt_opponent_ids_.end(),
                   opponent.id) !=
             current_pose_rear_only_exempt_opponent_ids_.end();
}

std::vector<TrajectoryPoint>
LatticePlanner::generateRearSafetyPath(const EgoState &ego,
                                       double goal_d_m) const {
  std::vector<TrajectoryPoint> path;
  path.reserve(static_cast<std::size_t>(config_.rear_sampling_points));
  for (int i = 0; i < config_.rear_sampling_points; ++i) {
    const double ratio =
        static_cast<double>(i) / (config_.rear_sampling_points - 1);
    const double s = ego.frenet.s - ratio * config_.rear_safety_distance_m;
    const double lateral_change = ego.frenet.d - goal_d_m;
    const double side = lateral_change > 0.0   ? 1.0
                        : lateral_change < 0.0 ? -1.0
                                               : 0.0;
    const double spread = side * config_.rear_terminal_distance_m *
                          std::tan(config_.rear_sampling_angle_rad);
    // This is a space-time corridor for traffic approaching the merge lane:
    // the rear endpoint is evaluated now, while the point next to the ego is
    // evaluated at the expected end of the merge.
    const double d = lerp(goal_d_m, goal_d_m + spread, ratio);
    const auto point = frame_->frenetToCartesian(s, d);
    TrajectoryPoint trajectory_point;
    trajectory_point.x = point.x;
    trajectory_point.y = point.y;
    trajectory_point.yaw = point.yaw;
    trajectory_point.u = ratio;
    trajectory_point.s = s;
    trajectory_point.d = d;
    trajectory_point.kappa = sanitizedCurvature(point.kappa, config_);
    trajectory_point.time_sec =
        (1.0 - ratio) * config_.rear_prediction_horizon_sec;
    trajectory_point.speed_mps = config_.target_missing_recovery_speed_mps;
    path.push_back(trajectory_point);
  }
  return path;
}

int LatticePlanner::rearCost(const std::vector<TrajectoryPoint> &path,
                             const std::vector<OpponentState> &opponents,
                             bool *hard_safe) const {
  if (!config_.safety_evaluation_enabled) {
    if (hard_safe != nullptr) {
      *hard_safe = true;
    }
    return 0;
  }
  *hard_safe = path.size() == 5U;
  std::vector<int> costs;
  for (std::size_t i = 0; i < path.size(); ++i) {
    costs.push_back(poseCost(path[i], opponents));
    if (hardCollision(path[i], opponents)) {
      *hard_safe = false;
    }
    if (i == 0U) {
      continue;
    }
    const double distance =
        std::hypot(path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y);
    const int pieces = std::max(
        1,
        static_cast<int>(std::ceil(distance / config_.collision_max_step_m)));
    for (int j = 1; j < pieces; ++j) {
      const double ratio = static_cast<double>(j) / pieces;
      TrajectoryPoint point;
      point.x = lerp(path[i - 1U].x, path[i].x, ratio);
      point.y = lerp(path[i - 1U].y, path[i].y, ratio);
      point.yaw = normalizeAngle(
          path[i - 1U].yaw +
          ratio * normalizeAngle(path[i].yaw - path[i - 1U].yaw));
      point.time_sec = lerp(path[i - 1U].time_sec, path[i].time_sec, ratio);
      if (hardCollision(point, opponents)) {
        *hard_safe = false;
      }
    }
  }
  return trajectoryCost(costs);
}

std::optional<CandidateTrajectory>
LatticePlanner::boundedExactCartesianExecution(
    const CandidateTrajectory &candidate,
    const std::vector<OpponentState> &opponents, std::size_t maximum_points,
    OutputHorizonDiagnostic *diagnostic) const {
  if (maximum_points < 2U || candidate.dense.size() < 2U) {
    if (diagnostic != nullptr) {
      *diagnostic = OutputHorizonDiagnostic{};
      diagnostic->failure = OutputHorizonFailure::INVALID_CONTRACT;
    }
    return std::nullopt;
  }

  CandidateTrajectory execution = candidate;
  if (execution.dense.size() > maximum_points) {
    const auto &source = candidate.dense;
    execution.dense.clear();
    execution.dense.reserve(maximum_points);
    for (std::size_t output_index = 0U; output_index < maximum_points;
         ++output_index) {
      const std::size_t source_index =
          output_index * (source.size() - 1U) / (maximum_points - 1U);
      execution.dense.push_back(source[source_index]);
    }
  }
  if (!validateExactCartesianHorizon(execution, opponents, diagnostic)) {
    return std::nullopt;
  }
  return execution;
}

bool LatticePlanner::validateExactCartesianHorizon(
    const CandidateTrajectory &candidate,
    const std::vector<OpponentState> &opponents,
    OutputHorizonDiagnostic *diagnostic) const {
  if (diagnostic != nullptr) {
    *diagnostic = OutputHorizonDiagnostic{};
  }
  if (!config_.safety_evaluation_enabled || !candidate.feasible ||
      candidate.dense.size() < 2U || map_ == nullptr ||
      !map_->initialized()) {
    if (diagnostic != nullptr) {
      diagnostic->failure = OutputHorizonFailure::INVALID_CONTRACT;
    }
    return false;
  }

  const double maximum_curvature = maximumPlannerCurvature(config_);
  for (std::size_t i = 0U; i < candidate.dense.size(); ++i) {
    const auto &point = candidate.dense[i];
    if (!finitePoint(point) || point.time_sec < -1.0e-9 ||
        (i > 0U &&
         point.time_sec < candidate.dense[i - 1U].time_sec - 1.0e-9)) {
      if (diagnostic != nullptr) {
        diagnostic->failure = i > 0U ? OutputHorizonFailure::NON_POSITIVE_DT
                                     : OutputHorizonFailure::INVALID_POINT;
        diagnostic->waypoint_index = static_cast<int>(i);
      }
      return false;
    }
    if (std::abs(point.kappa) > maximum_curvature + 1.0e-6) {
      if (diagnostic != nullptr) {
        diagnostic->failure = OutputHorizonFailure::CURVATURE;
        diagnostic->waypoint_index = static_cast<int>(i);
        diagnostic->curvature_radpm = point.kappa;
      }
      return false;
    }
    if (map_->footprintHitsWall(point, wallFootprint(config_, true))) {
      if (diagnostic != nullptr) {
        diagnostic->failure = OutputHorizonFailure::WAYPOINT_WALL_COLLISION;
        diagnostic->waypoint_index = static_cast<int>(i);
      }
      return false;
    }
    if (opponentCollision(point, opponents)) {
      bool exempt = i == 0U;
      for (const auto &opponent : opponents) {
        if (opponentCollisionDiagnostic(point, {opponent}).collision() &&
            !currentPoseRearOnlyExemptionApplies(point, opponent)) {
          exempt = false;
          break;
        }
      }
      if (!exempt) {
        if (diagnostic != nullptr) {
          diagnostic->failure =
              OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION;
          diagnostic->waypoint_index = static_cast<int>(i);
        }
        return false;
      }
    }
    if (i == 0U) {
      continue;
    }

    const auto &previous = candidate.dense[i - 1U];
    const double distance = std::hypot(point.x - previous.x,
                                       point.y - previous.y);
    const double yaw_delta =
        std::abs(normalizeAngle(point.yaw - previous.yaw));
    const int samples = std::max(
        1, static_cast<int>(std::ceil(std::max(
               distance / config_.collision_max_step_m,
               yaw_delta / config_.collision_max_yaw_step_rad))));
    const double dt = point.time_sec - previous.time_sec;
    if (dt <= 1.0e-9) {
      const bool identical_join = distance <= 1.0e-9 &&
                                  yaw_delta <= 1.0e-9 &&
                                  std::abs(point.kappa - previous.kappa) <=
                                      1.0e-9;
      if (identical_join) {
        continue;
      }
      if (diagnostic != nullptr) {
        diagnostic->failure = OutputHorizonFailure::NON_POSITIVE_DT;
        diagnostic->waypoint_index = static_cast<int>(i);
      }
      return false;
    }
    const double previous_steer =
        std::atan(config_.wheel_base_m * previous.kappa);
    const double current_steer = std::atan(config_.wheel_base_m * point.kappa);
    if (std::abs(current_steer - previous_steer) / dt >
        config_.max_steer_rate_radps + 1.0e-6) {
      if (diagnostic != nullptr) {
        diagnostic->failure = OutputHorizonFailure::STEERING_RATE;
        diagnostic->waypoint_index = static_cast<int>(i);
      }
      return false;
    }
    for (int sample = 1; sample < samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      TrajectoryPoint interpolated = previous;
      interpolated.x = lerp(previous.x, point.x, ratio);
      interpolated.y = lerp(previous.y, point.y, ratio);
      interpolated.yaw = normalizeAngle(
          previous.yaw +
          ratio * normalizeAngle(point.yaw - previous.yaw));
      interpolated.kappa = lerp(previous.kappa, point.kappa, ratio);
      interpolated.time_sec = lerp(previous.time_sec, point.time_sec, ratio);
      interpolated.speed_mps =
          lerp(previous.speed_mps, point.speed_mps, ratio);
      if (map_->footprintHitsWall(interpolated, wallFootprint(config_, true))) {
        if (diagnostic != nullptr) {
          diagnostic->failure =
              OutputHorizonFailure::INTERPOLATED_WALL_COLLISION;
          diagnostic->waypoint_index = static_cast<int>(i);
        }
        return false;
      }
      if (opponentCollision(interpolated, opponents)) {
        if (diagnostic != nullptr) {
          diagnostic->failure =
              OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION;
          diagnostic->waypoint_index = static_cast<int>(i);
        }
        return false;
      }
    }
  }
  return true;
}

bool LatticePlanner::outputHorizon(const EgoState &ego,
                                   const CandidateTrajectory &candidate,
                                   const std::vector<OpponentState> &opponents,
                                   double target_speed, std::vector<double> *d,
                                   std::vector<double> *speed,
                                   std::vector<double> *longitudinal_offsets_m,
                                   OutputHorizonDiagnostic *diagnostic) const {
  if (diagnostic != nullptr) {
    *diagnostic = OutputHorizonDiagnostic{};
  }
  if (d == nullptr || speed == nullptr || longitudinal_offsets_m == nullptr ||
      candidate.dense.size() < 2U || frame_ == nullptr || frame_->empty() ||
      output_frame_ == nullptr || output_frame_->empty()) {
    if (diagnostic != nullptr) {
      diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
      diagnostic->observed_value = static_cast<double>(candidate.dense.size());
      diagnostic->limit_value = 2.0;
    }
    return false;
  }
  std::vector<TrajectoryPoint> profile = candidate.dense;
  for (auto &point : profile) {
    const double candidate_limit =
        std::isfinite(point.speed_mps) && point.speed_mps > 0.0
            ? point.speed_mps
            : config_.normal_speed_mps;
    point.speed_mps =
        std::max(config_.safe_stop_speed_mps,
                 std::min({target_speed, candidate_limit,
                           curveSpeedLimit(point.kappa, config_),
                           referenceSpeedLimit(frame_, point.s, config_)}));
  }
  for (std::size_t i = profile.size() - 1U; i > 0U; --i) {
    const double ds = std::hypot(profile[i].x - profile[i - 1U].x,
                                 profile[i].y - profile[i - 1U].y);
    profile[i - 1U].speed_mps = std::min(
        profile[i - 1U].speed_mps,
        std::sqrt(std::max(0.0, profile[i].speed_mps * profile[i].speed_mps -
                                    2.0 * config_.min_acceleration_mps2 * ds)));
  }
  profile.front().speed_mps =
      std::min(profile.front().speed_mps,
               std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps)));
  for (std::size_t i = 1; i < profile.size(); ++i) {
    const double ds = std::hypot(profile[i].x - profile[i - 1U].x,
                                 profile[i].y - profile[i - 1U].y);
    profile[i].speed_mps = std::min(
        profile[i].speed_mps,
        std::sqrt(std::max(0.0, profile[i - 1U].speed_mps *
                                        profile[i - 1U].speed_mps +
                                    2.0 * config_.max_acceleration_mps2 * ds)));
  }

  const std::size_t nearest = frame_->nearestIndex(ego.frenet.s);
  d->clear();
  speed->clear();
  longitudinal_offsets_m->clear();
  d->reserve(static_cast<std::size_t>(config_.horizon_points));
  speed->reserve(static_cast<std::size_t>(config_.horizon_points));
  if (!config_.experimental_exact_spatial_follow_shadow_enabled) {
    for (int i = 0; i < config_.horizon_points; ++i) {
      const double pp_s = frame_->unwrappedIndexS(
          static_cast<long long>(nearest) + i, nearest, ego.frenet.s);
      const double mpc_s = frame_->unwrappedIndexS(
          static_cast<long long>(nearest) + config_.mpc_wp_id_offset + i,
          nearest, ego.frenet.s);
      const double out_s = 0.5 * (pp_s + mpc_s);
      d->push_back(i == 0 ? ego.frenet.d
                          : interpolateProfile(profile, out_s, false));
      speed->push_back(std::max(config_.safe_stop_speed_mps,
                                interpolateProfile(profile, out_s, true)));
    }
    return validateResampledHorizon(ego, opponents, *d, *speed,
                                    *longitudinal_offsets_m, diagnostic);
  }
  if (output_frame_ == frame_) {
    // Preserve the established single-reference exact-spatial encoding for
    // embedders and tests that use the backward-compatible constructor.
    longitudinal_offsets_m->reserve(
        static_cast<std::size_t>(config_.horizon_points));
    long long first_forward_source = static_cast<long long>(nearest);
    while (frame_->unwrappedIndexS(first_forward_source, nearest,
                                   ego.frenet.s) < ego.frenet.s - 1.0e-6) {
      ++first_forward_source;
    }
    double previous_x = ego.x;
    double previous_y = ego.y;
    double accumulated_arc_m = 0.0;
    for (int i = 0; i < config_.horizon_points; ++i) {
      const double out_s =
          i == 0 ? ego.frenet.s
                 : frame_->unwrappedIndexS(first_forward_source + i, nearest,
                                           ego.frenet.s);
      const double out_d =
          i == 0 ? ego.frenet.d : interpolateProfile(profile, out_s, false);
      d->push_back(out_d);
      speed->push_back(std::max(config_.safe_stop_speed_mps,
                                interpolateProfile(profile, out_s, true)));
      if (i > 0) {
        const auto point = frame_->frenetToCartesian(out_s, out_d);
        accumulated_arc_m +=
            std::hypot(point.x - previous_x, point.y - previous_y);
        previous_x = point.x;
        previous_y = point.y;
      }
      longitudinal_offsets_m->push_back(accumulated_arc_m);
    }
    return validateResampledHorizon(ego, opponents, *d, *speed,
                                    *longitudinal_offsets_m, diagnostic);
  }
  const auto output_ego = output_frame_->project(ego.x, ego.y, ego.yaw);
  if (!output_ego.valid) {
    if (diagnostic != nullptr) {
      diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
    }
    return false;
  }
  std::vector<double> profile_arc_m(profile.size(), 0.0);
  for (std::size_t i = 1U; i < profile.size(); ++i) {
    profile_arc_m[i] =
        profile_arc_m[i - 1U] + std::hypot(profile[i].x - profile[i - 1U].x,
                                           profile[i].y - profile[i - 1U].y);
  }
  const auto sample_profile = [&profile, &profile_arc_m](double arc_m) {
    const auto upper =
        std::upper_bound(profile_arc_m.begin(), profile_arc_m.end(), arc_m);
    if (upper == profile_arc_m.begin()) {
      return profile.front();
    }
    if (upper == profile_arc_m.end()) {
      return profile.back();
    }
    const std::size_t upper_index =
        static_cast<std::size_t>(std::distance(profile_arc_m.begin(), upper));
    const std::size_t lower_index = upper_index - 1U;
    const double span = profile_arc_m[upper_index] - profile_arc_m[lower_index];
    const double ratio =
        span > 1.0e-9 ? (arc_m - profile_arc_m[lower_index]) / span : 1.0;
    TrajectoryPoint sampled;
    sampled.x = lerp(profile[lower_index].x, profile[upper_index].x, ratio);
    sampled.y = lerp(profile[lower_index].y, profile[upper_index].y, ratio);
    sampled.yaw =
        normalizeAngle(profile[lower_index].yaw +
                       ratio * normalizeAngle(profile[upper_index].yaw -
                                              profile[lower_index].yaw));
    sampled.speed_mps = lerp(profile[lower_index].speed_mps,
                             profile[upper_index].speed_mps, ratio);
    return sampled;
  };
  const std::size_t output_nearest = output_frame_->nearestIndex(output_ego.s);
  longitudinal_offsets_m->reserve(
      static_cast<std::size_t>(config_.horizon_points));
  long long first_forward_source = static_cast<long long>(output_nearest);
  while (output_frame_->unwrappedIndexS(first_forward_source, output_nearest,
                                        output_ego.s) < output_ego.s - 1.0e-6) {
    ++first_forward_source;
  }
  double previous_reference_x = ego.x;
  double previous_reference_y = ego.y;
  double candidate_arc_m = 0.0;
  double previous_output_s = output_ego.s;
  double previous_projected_reference_x = ego.x;
  double previous_projected_reference_y = ego.y;
  std::optional<double> terminal_output_d;
  TrajectoryPoint previous_desired = profile.front();
  for (int i = 0; i < config_.horizon_points; ++i) {
    if (i == 0) {
      d->push_back(output_ego.d);
      speed->push_back(
          std::max(config_.safe_stop_speed_mps, profile.front().speed_mps));
      longitudinal_offsets_m->push_back(0.0);
      continue;
    }
    const double nominal_output_s = output_frame_->unwrappedIndexS(
        first_forward_source + i, output_nearest, output_ego.s);
    const auto nominal_reference = output_frame_->interpolate(nominal_output_s);
    const double nominal_step_m =
        std::hypot(nominal_reference.x - previous_reference_x,
                   nominal_reference.y - previous_reference_y);
    candidate_arc_m += nominal_step_m;
    previous_reference_x = nominal_reference.x;
    previous_reference_y = nominal_reference.y;
    const auto desired = sample_profile(candidate_arc_m);

    double accepted_output_s = previous_output_s;
    double output_d_m = 0.0;
    if (terminal_output_d.has_value()) {
      // Once the finite candidate transition ends, continue its accepted
      // terminal lateral offset along the controller reference.  Reusing the
      // terminal Cartesian point would create duplicate stations and only
      // appear to fill the controller horizon.
      if (!std::isfinite(nominal_step_m) || nominal_step_m <= 1.0e-6) {
        if (diagnostic != nullptr) {
          diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
          diagnostic->waypoint_index = i;
          diagnostic->observed_value = nominal_step_m;
          diagnostic->limit_value = 1.0e-6;
        }
        return false;
      }
      accepted_output_s = previous_output_s + nominal_step_m;
      output_d_m = terminal_output_d.value();
    } else {
      // Preserve the candidate's actual Cartesian arc, then continuously
      // project that point into the controller reference near its expected
      // forward station. Advancing both references by the same nominal arc
      // accumulates their different parameterizations and eventually compares
      // unrelated points (runtime-10 generation 489).
      const double candidate_step_m = std::hypot(
          desired.x - previous_desired.x, desired.y - previous_desired.y);
      if (!std::isfinite(candidate_step_m) || candidate_step_m <= 1.0e-6) {
        if (diagnostic != nullptr) {
          diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
          diagnostic->waypoint_index = i;
          diagnostic->observed_value = candidate_step_m;
          diagnostic->limit_value = 1.0e-6;
        }
        return false;
      }
      const double expected_output_s = previous_output_s + candidate_step_m;
      const auto projected = output_frame_->projectContinuousUnique(
          desired.x, desired.y, desired.yaw, expected_output_s,
          config_.projection_follow_half_width_m);
      if (!projected.valid) {
        if (diagnostic != nullptr) {
          diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
          diagnostic->waypoint_index = i;
          diagnostic->observed_value =
              std::numeric_limits<double>::quiet_NaN();
          diagnostic->limit_value = config_.projection_follow_half_width_m;
        }
        return false;
      }
      const double raw_forward_delta_m = projected.s - previous_output_s;
      if (!std::isfinite(raw_forward_delta_m) ||
          raw_forward_delta_m < -config_.projection_max_backward_m) {
        if (diagnostic != nullptr) {
          diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
          diagnostic->waypoint_index = i;
          diagnostic->observed_value = raw_forward_delta_m;
          diagnostic->limit_value = config_.projection_max_backward_m;
        }
        return false;
      }
      accepted_output_s = std::max(previous_output_s, projected.s);
      output_d_m = projected.d;
      if (candidate_arc_m >= profile_arc_m.back() - 1.0e-6) {
        terminal_output_d = output_d_m;
      }
    }
    const auto accepted_reference =
        output_frame_->interpolate(accepted_output_s);
    const double forward_arc_delta_m = std::hypot(
        accepted_reference.x - previous_projected_reference_x,
        accepted_reference.y - previous_projected_reference_y);
    if (!std::isfinite(forward_arc_delta_m)) {
      if (diagnostic != nullptr) {
        diagnostic->failure = OutputHorizonFailure::INVALID_INPUT;
        diagnostic->waypoint_index = i;
        diagnostic->observed_value = forward_arc_delta_m;
        diagnostic->limit_value = config_.projection_follow_half_width_m;
      }
      return false;
    }
    d->push_back(output_d_m);
    speed->push_back(std::max(config_.safe_stop_speed_mps, desired.speed_mps));
    longitudinal_offsets_m->push_back(longitudinal_offsets_m->back() +
                                      forward_arc_delta_m);
    previous_output_s = accepted_output_s;
    previous_projected_reference_x = accepted_reference.x;
    previous_projected_reference_y = accepted_reference.y;
    previous_desired = desired;
  }
  return validateResampledHorizon(ego, opponents, *d, *speed,
                                  *longitudinal_offsets_m, diagnostic);
}

bool LatticePlanner::validateResampledHorizon(
    const EgoState &ego, const std::vector<OpponentState> &opponents,
    const std::vector<double> &d, const std::vector<double> &speed,
    const std::vector<double> &longitudinal_offsets_m,
    OutputHorizonDiagnostic *diagnostic,
    const SideRoleSeparationReference *side_role_reference) const {
  const auto fail =
      [&](OutputHorizonFailure failure, int shift, int layout_offset,
          int waypoint_index, int interpolation_piece, double observed_value,
          double limit_value, const std::string &opponent_id = std::string{},
          std::int64_t reference_index = -1,
          const TrajectoryPoint *point = nullptr) {
        if (diagnostic != nullptr &&
            diagnostic->failure == OutputHorizonFailure::NONE) {
          diagnostic->failure = failure;
          diagnostic->nearest_shift = shift;
          diagnostic->layout_offset = layout_offset;
          diagnostic->waypoint_index = waypoint_index;
          diagnostic->interpolation_piece = interpolation_piece;
          diagnostic->reference_index = reference_index;
          diagnostic->observed_value = observed_value;
          diagnostic->limit_value = limit_value;
          diagnostic->opponent_id = opponent_id;
          if (point != nullptr) {
            diagnostic->s_m = point->s;
            diagnostic->d_m = point->d;
            diagnostic->x_m = point->x;
            diagnostic->y_m = point->y;
            diagnostic->curvature_radpm = point->kappa;
          }
        }
        return false;
      };

  const bool spatial_profile = !longitudinal_offsets_m.empty();
  const bool two_frame_spatial_profile =
      spatial_profile && output_frame_ != frame_;
  const FrenetFrame *const reconstruction_frame =
      two_frame_spatial_profile ? output_frame_ : frame_;
  if (reconstruction_frame == nullptr || reconstruction_frame->empty() ||
      map_ == nullptr || !map_->initialized()) {
    return fail(OutputHorizonFailure::INVALID_INPUT, 0, 0, -1, -1,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
  }
  if (d.size() != static_cast<std::size_t>(config_.horizon_points) ||
      d.size() != speed.size() ||
      (spatial_profile && d.size() != longitudinal_offsets_m.size())) {
    return fail(OutputHorizonFailure::INVALID_CONTRACT, 0, 0, -1, -1,
                static_cast<double>(d.size()),
                static_cast<double>(config_.horizon_points));
  }
  for (std::size_t i = 0U; i < d.size(); ++i) {
    if (!std::isfinite(d[i])) {
      return fail(OutputHorizonFailure::INVALID_CONTRACT, 0, 0,
                  static_cast<int>(i), -1, d[i],
                  std::numeric_limits<double>::quiet_NaN());
    }
    if (!std::isfinite(speed[i]) || speed[i] <= 0.0) {
      return fail(OutputHorizonFailure::INVALID_CONTRACT, 0, 0,
                  static_cast<int>(i), -1, speed[i], 0.0);
    }
    if (spatial_profile &&
        (!std::isfinite(longitudinal_offsets_m[i]) ||
         longitudinal_offsets_m[i] < 0.0 ||
         (i == 0U && std::abs(longitudinal_offsets_m[i]) > 1.0e-5) ||
         (i > 0U && longitudinal_offsets_m[i] + 1.0e-6 <
                        longitudinal_offsets_m[i - 1U]))) {
      return fail(OutputHorizonFailure::INVALID_CONTRACT, 0, 0,
                  static_cast<int>(i), -1, longitudinal_offsets_m[i], 0.0);
    }
  }

  if (diagnostic != nullptr) {
    diagnostic->failure = OutputHorizonFailure::NONE;
  }

  const auto output_ego =
      two_frame_spatial_profile
          ? reconstruction_frame->project(ego.x, ego.y, ego.yaw)
          : ego.frenet;
  if (!output_ego.valid) {
    return fail(OutputHorizonFailure::INVALID_INPUT, 0, 0, -1, -1,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN());
  }
  const std::size_t nearest = reconstruction_frame->nearestIndex(output_ego.s);
  const double maximum_curvature = maximumPlannerCurvature(config_);
  const auto sample_spatial_profile = [&longitudinal_offsets_m](
                                          const std::vector<double> &values,
                                          double arc_m) {
    if (arc_m <= longitudinal_offsets_m.front()) {
      return values.front();
    }
    const auto upper = std::upper_bound(longitudinal_offsets_m.begin(),
                                        longitudinal_offsets_m.end(), arc_m);
    if (upper == longitudinal_offsets_m.end()) {
      return values.back();
    }
    const std::size_t upper_index = static_cast<std::size_t>(
        std::distance(longitudinal_offsets_m.begin(), upper));
    const std::size_t lower_index = upper_index - 1U;
    const double span = longitudinal_offsets_m[upper_index] -
                        longitudinal_offsets_m[lower_index];
    if (span <= 1.0e-9) {
      return values[upper_index];
    }
    const double ratio = (arc_m - longitudinal_offsets_m[lower_index]) / span;
    return lerp(values[lower_index], values[upper_index], ratio);
  };
  const int minimum_shift =
      spatial_profile ? 0 : -config_.nearest_index_uncertainty;
  const int maximum_shift =
      spatial_profile ? 0 : config_.nearest_index_uncertainty;
  const std::vector<int> layout_offsets =
      spatial_profile ? std::vector<int>{0}
                      : std::vector<int>{0, config_.mpc_wp_id_offset};
  for (int shift = minimum_shift; shift <= maximum_shift; ++shift) {
    for (const int layout_offset : layout_offsets) {
      std::vector<TrajectoryPoint> reconstructed;
      reconstructed.reserve(static_cast<std::size_t>(config_.horizon_points));
      long long first_forward_source =
          static_cast<long long>(nearest) + shift + layout_offset;
      if (spatial_profile) {
        while (reconstruction_frame->unwrappedIndexS(first_forward_source,
                                                     nearest, output_ego.s) <
               output_ego.s - 1.0e-6) {
          ++first_forward_source;
        }
      }
      double source_arc_m = 0.0;
      ReferencePoint previous_reference =
          reconstruction_frame->interpolate(output_ego.s);
      if (two_frame_spatial_profile) {
        // Match outputHorizon(): controller-space offsets start at the actual
        // ego pose, then advance over controller reference waypoints.
        previous_reference.x = ego.x;
        previous_reference.y = ego.y;
      }
      for (int i = 0; i < config_.horizon_points; ++i) {
        const auto index =
            i == 0 ? first_forward_source : first_forward_source + i;
        const double s = i == 0 ? output_ego.s
                                : reconstruction_frame->unwrappedIndexS(
                                      index, nearest, output_ego.s);
        ReferencePoint point;
        double sampled_d = ego.frenet.d;
        double sampled_speed = speed.front();
        if (spatial_profile && i == 0) {
          point.x = ego.x;
          point.y = ego.y;
          point.yaw = ego.yaw;
          point.s = output_ego.s;
          point.kappa = sanitizedCurvature(ego.curvature, config_);
          point.speed_mps = speed.front();
        } else {
          const auto reference = reconstruction_frame->interpolate(s);
          source_arc_m += std::hypot(reference.x - previous_reference.x,
                                     reference.y - previous_reference.y);
          previous_reference = reference;
          sampled_d = spatial_profile ? sample_spatial_profile(d, source_arc_m)
                                      : d[static_cast<std::size_t>(i)];
          sampled_speed = spatial_profile
                              ? sample_spatial_profile(speed, source_arc_m)
                              : speed[static_cast<std::size_t>(i)];
          point = reconstruction_frame->frenetToCartesian(s, sampled_d);
        }
        TrajectoryPoint trajectory_point;
        trajectory_point.x = point.x;
        trajectory_point.y = point.y;
        trajectory_point.yaw = point.yaw;
        trajectory_point.s = s;
        trajectory_point.d = sampled_d;
        trajectory_point.kappa = point.kappa;
        trajectory_point.speed_mps = sampled_speed;
        if (i > 0) {
          const auto &previous = reconstructed.back();
          const double distance = std::hypot(trajectory_point.x - previous.x,
                                             trajectory_point.y - previous.y);
          const double average_speed =
              std::max(config_.safe_stop_speed_mps,
                       0.5 * (previous.speed_mps + trajectory_point.speed_mps));
          trajectory_point.time_sec =
              previous.time_sec + distance / average_speed;
          const double previous_steer =
              std::atan(config_.wheel_base_m * previous.kappa);
          const double steer =
              std::atan(config_.wheel_base_m * trajectory_point.kappa);
          const double dt = trajectory_point.time_sec - previous.time_sec;
          if (dt <= 0.0) {
            return fail(OutputHorizonFailure::NON_POSITIVE_DT, shift,
                        layout_offset, i, -1, dt, 0.0, std::string{}, index,
                        &trajectory_point);
          }
          const double steering_rate = std::abs(steer - previous_steer) / dt;
          if (steering_rate > config_.max_steer_rate_radps + 1.0e-6) {
            return fail(OutputHorizonFailure::STEERING_RATE, shift,
                        layout_offset, i, -1, steering_rate,
                        config_.max_steer_rate_radps, std::string{}, index,
                        &trajectory_point);
          }
        }
        if (!finitePoint(trajectory_point)) {
          return fail(OutputHorizonFailure::INVALID_POINT, shift, layout_offset,
                      i, -1, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(), std::string{},
                      index, &trajectory_point);
        }
        if (std::abs(trajectory_point.kappa) > maximum_curvature + 1.0e-6) {
          return fail(OutputHorizonFailure::CURVATURE, shift, layout_offset, i,
                      -1, std::abs(trajectory_point.kappa), maximum_curvature,
                      std::string{}, index, &trajectory_point);
        }
        if (config_.safety_evaluation_enabled &&
            map_->footprintHitsWall(trajectory_point,
                                    wallFootprint(config_, true))) {
          return fail(OutputHorizonFailure::WAYPOINT_WALL_COLLISION, shift,
                      layout_offset, i, -1,
                      std::numeric_limits<double>::quiet_NaN(),
                      config_.wall_hard_margin_m, std::string{}, index,
                      &trajectory_point);
        }
        const auto opponent_collision =
            opponentCollisionDiagnostic(trajectory_point, opponents);
        const bool exact_current_pose =
            i == 0 && std::abs(trajectory_point.time_sec) <= 1.0e-9 &&
            std::hypot(trajectory_point.x - ego.x,
                       trajectory_point.y - ego.y) <= 1.0e-6 &&
            std::abs(normalizeAngle(trajectory_point.yaw - ego.yaw)) <= 1.0e-6;
        bool collision_exempt_at_current_pose =
            opponent_collision.collision() && exact_current_pose;
        if (collision_exempt_at_current_pose) {
          for (const auto &opponent : opponents) {
            if (opponentCollisionDiagnostic(trajectory_point, {opponent})
                    .collision() &&
                !currentPoseRearOnlyExemptionApplies(trajectory_point,
                                                     opponent)) {
              collision_exempt_at_current_pose = false;
              break;
            }
          }
        }
        if (opponent_collision.collision() &&
            !collision_exempt_at_current_pose) {
          return fail(OutputHorizonFailure::WAYPOINT_OPPONENT_COLLISION, shift,
                      layout_offset, i, -1, opponent_collision.clearance_m,
                      opponent_collision.required_clearance_m,
                      opponent_collision.opponent_id, index, &trajectory_point);
        }
        if (side_role_reference != nullptr &&
            !sideRolePeerSeparationAtPoint(trajectory_point,
                                           *side_role_reference, i, false)) {
          const auto *role_diagnostic = side_role_reference->diagnostic;
          return fail(OutputHorizonFailure::WAYPOINT_SIDE_ROLE_SEPARATION,
                      shift, layout_offset, i, -1,
                      role_diagnostic != nullptr
                          ? role_diagnostic->observed_clearance_m
                          : std::numeric_limits<double>::quiet_NaN(),
                      side_role_reference->initial_clearance_m,
                      side_role_reference->peer != nullptr
                          ? side_role_reference->peer->id
                          : std::string{},
                      index, &trajectory_point);
        }
        reconstructed.push_back(trajectory_point);
      }
      for (std::size_t i = 1; i < reconstructed.size(); ++i) {
        const double distance =
            std::hypot(reconstructed[i].x - reconstructed[i - 1U].x,
                       reconstructed[i].y - reconstructed[i - 1U].y);
        const double yaw_change = std::abs(
            normalizeAngle(reconstructed[i].yaw - reconstructed[i - 1U].yaw));
        const int pieces =
            std::max(1, static_cast<int>(std::ceil(std::max(
                            distance / config_.collision_max_step_m,
                            yaw_change / config_.collision_max_yaw_step_rad))));
        for (int piece = 1; piece < pieces; ++piece) {
          const double ratio = static_cast<double>(piece) / pieces;
          TrajectoryPoint point;
          point.x = lerp(reconstructed[i - 1U].x, reconstructed[i].x, ratio);
          point.y = lerp(reconstructed[i - 1U].y, reconstructed[i].y, ratio);
          point.yaw =
              normalizeAngle(reconstructed[i - 1U].yaw +
                             ratio * normalizeAngle(reconstructed[i].yaw -
                                                    reconstructed[i - 1U].yaw));
          point.time_sec = lerp(reconstructed[i - 1U].time_sec,
                                reconstructed[i].time_sec, ratio);
          point.s = lerp(reconstructed[i - 1U].s, reconstructed[i].s, ratio);
          point.d = lerp(reconstructed[i - 1U].d, reconstructed[i].d, ratio);
          point.kappa =
              lerp(reconstructed[i - 1U].kappa, reconstructed[i].kappa, ratio);
          if (config_.safety_evaluation_enabled &&
              map_->footprintHitsWall(point, wallFootprint(config_, true))) {
            return fail(OutputHorizonFailure::INTERPOLATED_WALL_COLLISION,
                        shift, layout_offset, static_cast<int>(i), piece,
                        std::numeric_limits<double>::quiet_NaN(),
                        config_.wall_hard_margin_m, std::string{},
                        static_cast<std::int64_t>(nearest) + shift +
                            layout_offset + static_cast<int>(i),
                        &point);
          }
          const auto opponent_collision =
              opponentCollisionDiagnostic(point, opponents);
          if (opponent_collision.collision()) {
            return fail(OutputHorizonFailure::INTERPOLATED_OPPONENT_COLLISION,
                        shift, layout_offset, static_cast<int>(i), piece,
                        opponent_collision.clearance_m,
                        opponent_collision.required_clearance_m,
                        opponent_collision.opponent_id,
                        static_cast<std::int64_t>(nearest) + shift +
                            layout_offset + static_cast<int>(i),
                        &point);
          }
          if (side_role_reference != nullptr &&
              !sideRolePeerSeparationAtPoint(point, *side_role_reference,
                                             static_cast<int>(i), true)) {
            const auto *role_diagnostic = side_role_reference->diagnostic;
            return fail(OutputHorizonFailure::INTERPOLATED_SIDE_ROLE_SEPARATION,
                        shift, layout_offset, static_cast<int>(i), piece,
                        role_diagnostic != nullptr
                            ? role_diagnostic->observed_clearance_m
                            : std::numeric_limits<double>::quiet_NaN(),
                        side_role_reference->initial_clearance_m,
                        side_role_reference->peer != nullptr
                            ? side_role_reference->peer->id
                            : std::string{},
                        static_cast<std::int64_t>(nearest) + shift +
                            layout_offset + static_cast<int>(i),
                        &point);
          }
        }
      }
    }
  }
  return true;
}

bool LatticePlanner::validateOutputHorizon(
    const EgoState &ego, const std::vector<OpponentState> &opponents,
    const std::vector<double> &d, const std::vector<double> &speed,
    const std::vector<double> &longitudinal_offsets_m, double now_sec) const {
  return validateResampledHorizon(ego, extrapolateOpponents(opponents, now_sec),
                                  d, speed, longitudinal_offsets_m, nullptr);
}

bool LatticePlanner::validateSideRoleOutputHorizon(
    const EgoState &ego, const OpponentState &peer,
    const std::vector<OpponentState> &all_opponents,
    const std::vector<double> &d, const std::vector<double> &speed,
    const std::vector<double> &longitudinal_offsets_m,
    OutputHorizonDiagnostic *output_diagnostic,
    PreventiveSideRoleCandidateDiagnostic *role_diagnostic) const {
  TrajectoryPoint initial_pose;
  initial_pose.x = ego.x;
  initial_pose.y = ego.y;
  initial_pose.yaw = ego.yaw;
  initial_pose.s = ego.frenet.s;
  initial_pose.d = ego.frenet.d;
  initial_pose.kappa = sanitizedCurvature(ego.curvature, config_);
  initial_pose.time_sec = 0.0;
  const double initial_lateral_gap_m = std::abs(peer.frenet.d - ego.frenet.d);
  const double initial_clearance_m =
      opponentCollisionDiagnostic(initial_pose, {peer}).clearance_m;
  if (role_diagnostic != nullptr) {
    role_diagnostic->peer_id = peer.id;
    role_diagnostic->initial_lateral_gap_m = initial_lateral_gap_m;
    role_diagnostic->initial_clearance_m = initial_clearance_m;
  }
  if (!ego.valid || !peer.valid || !std::isfinite(initial_lateral_gap_m) ||
      !std::isfinite(initial_clearance_m)) {
    if (role_diagnostic != nullptr) {
      role_diagnostic->failure =
          PreventiveSideRoleCandidateFailure::RESAMPLED_PEER_SEPARATION;
    }
    return false;
  }
  const SideRoleSeparationReference reference{
      &peer, initial_lateral_gap_m, initial_clearance_m, role_diagnostic,
      PreventiveSideRoleCandidateFailure::RESAMPLED_PEER_SEPARATION};
  return validateResampledHorizon(ego, all_opponents, d, speed,
                                  longitudinal_offsets_m, output_diagnostic,
                                  &reference);
}

const OpponentState *
LatticePlanner::target(const std::vector<OpponentState> &opponents) const {
  const auto it = std::find_if(opponents.begin(), opponents.end(),
                               [this](const OpponentState &opponent) {
                                 return opponent.id == target_id_;
                               });
  return it == opponents.end() ? nullptr : &*it;
}

std::vector<OpponentState> LatticePlanner::extrapolateOpponents(
    const std::vector<OpponentState> &opponents, double now_sec) const {
  std::vector<OpponentState> predicted = opponents;
  for (auto &opponent : predicted) {
    if (!opponent.valid) {
      continue;
    }
    const double raw_dt = now_sec - opponent.stamp_sec;
    const double dt = std::isfinite(raw_dt)
                          ? std::clamp(raw_dt, 0.0, config_.opponent_stale_sec)
                          : 0.0;
    opponent.x += opponent.vx_mps * dt;
    opponent.y += opponent.vy_mps * dt;
    opponent.stamp_sec = now_sec;
    opponent.frenet = frame_->project(opponent.x, opponent.y, opponent.yaw);
    opponent.valid = opponent.frenet.valid;
  }
  return predicted;
}

bool LatticePlanner::permissionRuleContainsS(const OvertakePermissionRule &rule,
                                             double s) const {
  if (frame_ == nullptr || frame_->empty()) {
    return false;
  }
  const double start = frame_->wrapS(rule.s_start_m);
  const double end = frame_->wrapS(rule.s_end_m);
  const double wrapped_s = frame_->wrapS(s);
  if (start <= end) {
    return wrapped_s >= start && wrapped_s <= end;
  }
  return wrapped_s >= start || wrapped_s <= end;
}

ActiveOvertakePermission LatticePlanner::overtakePermissionAtS(double s) const {
  ActiveOvertakePermission active;
  if (!config_.safety_evaluation_enabled ||
      !config_.overtake_permission_profile_enabled) {
    active.allow_overtake = true;
    active.reason = config_.safety_evaluation_enabled
                        ? "permission_profile_disabled"
                        : "safety_evaluation_disabled";
    return active;
  }
  active.allow_overtake = config_.default_overtake_allowed;
  active.reason = config_.default_overtake_allowed ? "default_allowed"
                                                   : "default_disallowed";
  for (const auto &rule : config_.overtake_permission_rules) {
    if (!permissionRuleContainsS(rule, s)) {
      continue;
    }
    active.active = true;
    active.name = rule.name;
    active.allow_overtake = rule.allow_overtake;
    active.reason =
        rule.allow_overtake ? "section_allowed" : "section_disallowed";
    return active;
  }
  return active;
}

ActiveOvertakePermission
LatticePlanner::activeOvertakePermission(double s) const {
  ActiveOvertakePermission current = overtakePermissionAtS(s);
  if (!config_.overtake_permission_profile_enabled ||
      config_.overtake_permission_lookahead_m <= 0.0) {
    return current;
  }
  constexpr int sample_count = 8;
  const double ds = config_.overtake_permission_lookahead_m /
                    static_cast<double>(sample_count);
  for (int i = 1; i <= sample_count; ++i) {
    auto ahead = overtakePermissionAtS(s + ds * static_cast<double>(i));
    if (!ahead.allow_overtake) {
      ahead.reason = ahead.active ? "lookahead_section_disallowed"
                                  : "lookahead_default_disallowed";
      return ahead;
    }
  }
  return current;
}

bool LatticePlanner::updateMpcHealthGuard(const MpcHealthStatus &health,
                                          std::string *reason) {
  if (!config_.safety_evaluation_enabled ||
      !config_.mpc_health_speed_guard_enabled) {
    mpc_health_guard_latched_ = false;
    mpc_health_release_count_ = 0;
    mpc_health_guard_reason_.clear();
    if (reason != nullptr) {
      reason->clear();
    }
    return false;
  }

  std::string violation;
  if (health.valid) {
    if (!std::isfinite(health.age_sec) ||
        health.age_sec > config_.mpc_health_stale_time_sec) {
      violation = "mpc_health_stale_guard";
    } else if (health.infeasible_count >=
               config_.mpc_health_infeasible_count_threshold) {
      violation = "mpc_health_infeasible_guard";
    } else if (std::isfinite(health.solve_time_ms) &&
               health.solve_time_ms >= config_.mpc_health_solve_time_warn_ms) {
      violation = "mpc_health_solve_time_guard";
    }
  }

  const bool new_sample =
      health.valid && (health.sample_sequence == 0U ||
                       health.sample_sequence != last_mpc_health_sequence_);
  if (!violation.empty()) {
    mpc_health_guard_latched_ = true;
    mpc_health_release_count_ = 0;
    mpc_health_guard_reason_ = violation;
  } else if (mpc_health_guard_latched_ && new_sample) {
    ++mpc_health_release_count_;
    if (mpc_health_release_count_ >= config_.mpc_health_release_samples) {
      mpc_health_guard_latched_ = false;
      mpc_health_release_count_ = 0;
      mpc_health_guard_reason_.clear();
    }
  }
  if (health.valid && health.sample_sequence != 0U) {
    last_mpc_health_sequence_ = health.sample_sequence;
  }
  if (reason != nullptr) {
    *reason = mpc_health_guard_reason_;
  }
  return mpc_health_guard_latched_;
}

void LatticePlanner::applyMpcHealthGuard(PlannerOutput *output,
                                         const std::string &reason) const {
  if (output == nullptr || !mpc_health_guard_latched_ ||
      output->mode == BehaviorMode::SAFE_STOP) {
    return;
  }
  output->active = true;
  output->mode = BehaviorMode::SPEED_GUARD;
  output->mpc_health_guard_active = true;
  const double current_cap = output->speed_cap_mps > 0.0
                                 ? output->speed_cap_mps
                                 : config_.normal_speed_mps;
  output->speed_cap_mps = std::min(current_cap, config_.mpc_health_v_max_mps);
  if (output->safe_lateral) {
    for (auto &cap : output->speed_caps_mps) {
      cap = std::min(cap, config_.mpc_health_v_max_mps);
    }
  } else {
    output->intent = SolverHorizonIntent::NONE;
  }
  output->reason = reason.empty() ? "mpc_health_guard" : reason;
}

void LatticePlanner::clearPassContinuationLatch() {
  pass_continuation_latch_.reset();
  suspended_pass_continuation_latch_.reset();
}

void LatticePlanner::populatePassContinuationDiagnostic(PlannerOutput *output,
                                                        bool active) const {
  if (output == nullptr) {
    return;
  }
  const auto *latch = pass_continuation_latch_.has_value()
                          ? &pass_continuation_latch_.value()
                          : (suspended_pass_continuation_latch_.has_value()
                                 ? &suspended_pass_continuation_latch_.value()
                                 : nullptr);
  if (latch == nullptr) {
    return;
  }
  output->pass_continuation_latched = true;
  output->pass_continuation_active =
      active && pass_continuation_latch_.has_value();
  output->pass_continuation_suspended =
      suspended_pass_continuation_latch_.has_value();
  output->pass_continuation_target_id = latch->target_id;
  output->pass_continuation_side = latch->side;
  output->pass_continuation_lateral_index =
      static_cast<int>(latch->lateral_index);
  output->pass_continuation_tangent_index =
      static_cast<int>(latch->tangent_index);
  output->pass_continuation_goal_d_m = latch->goal_d_m;
  output->pass_continuation_required_arc_m = latch->required_arc_m;
  output->pass_continuation_target_observation_stamp_sec =
      latch->target_observation_stamp_sec;
}

void LatticePlanner::resetManeuverState() {
  active_ = false;
  safe_stop_latched_ = false;
  safe_stop_release_count_ = 0;
  return_ready_count_ = 0;
  target_id_.clear();
  previous_lateral_index_ = -1;
  previous_tangent_index_ = -1;
  last_mode_change_sec_ = -1.0;
  last_mode_ = BehaviorMode::FREE_RUN;
  speed_state_initialized_ = false;
  previous_command_speed_mps_ = 0.0;
  previous_acceleration_mps2_ = 0.0;
  candidates_.clear();
  selected_.reset();
  rear_path_.clear();
  last_target_state_.reset();
  last_target_observed_sec_ = -1.0;
  target_missing_since_sec_ = -1.0;
  clearPassContinuationLatch();
  preventive_side_role_latch_.reset();
  detector_.reset();
  early_aware_selector_.reset();
}

PlannerOutput LatticePlanner::update(
    const EgoState &ego, const std::vector<OpponentState> &opponents,
    bool inputs_fresh, double now_sec, const MpcHealthStatus &mpc_health,
    TrialSpeedEvidence *speed_evidence) {
  last_planning_cycle_metrics_ = PlanningCycleMetrics{};
  current_pose_rear_only_exempt_opponent_ids_.clear();
  struct CurrentPoseExemptionReset {
    std::vector<std::string> *ids{nullptr};
    ~CurrentPoseExemptionReset() {
      if (ids != nullptr) {
        ids->clear();
      }
    }
  } current_pose_exemption_reset{&current_pose_rear_only_exempt_opponent_ids_};
  const auto capture_speed_state = [this]() {
    SpeedTransitionStateEvidence state{};
    state.initialized = speed_state_initialized_ ? 1U : 0U;
    state.safe_stop_latched = safe_stop_latched_ ? 1U : 0U;
    state.safe_stop_release_count = safe_stop_release_count_;
    state.command_speed_mps = previous_command_speed_mps_;
    state.acceleration_mps2 = previous_acceleration_mps2_;
    return state;
  };
  if (speed_evidence != nullptr) {
    *speed_evidence = TrialSpeedEvidence{};
    speed_evidence->pre_state = capture_speed_state();
    speed_evidence->present_fields |= SPEED_EVIDENCE_PRE_STATE;
  }
  const auto record_speed_transition =
      [speed_evidence](SpeedTransitionBranch branch, double effective_dt_sec,
                       double requested_speed_mps, double lower_speed_limit_mps,
                       double upper_speed_limit_mps, bool logical_stop,
                       bool target_missing_recovery,
                       std::int32_t candidate_lateral_index = -1,
                       std::int32_t candidate_tangent_index = -1,
                       double candidate_cost = 0.0) {
        if (speed_evidence == nullptr) {
          return;
        }
        speed_evidence->branch = branch;
        speed_evidence->effective_dt_sec = effective_dt_sec;
        speed_evidence->requested_speed_mps = requested_speed_mps;
        speed_evidence->lower_speed_limit_mps = lower_speed_limit_mps;
        speed_evidence->upper_speed_limit_mps = upper_speed_limit_mps;
        speed_evidence->logical_stop = logical_stop ? 1U : 0U;
        speed_evidence->target_missing_recovery =
            target_missing_recovery ? 1U : 0U;
        speed_evidence->candidate_lateral_index = candidate_lateral_index;
        speed_evidence->candidate_tangent_index = candidate_tangent_index;
        speed_evidence->candidate_cost = candidate_cost;
        speed_evidence->present_fields |=
            SPEED_EVIDENCE_EFFECTIVE_DT | SPEED_EVIDENCE_TRANSITION_INPUT;
      };
  PlannerOutput output;
  const auto finalize_output = [this, speed_evidence,
                                &capture_speed_state](PlannerOutput candidate) {
    if (candidate.mode == BehaviorMode::SAFE_STOP) {
      early_aware_selector_.reset();
      candidate.early_aware_diagnostic = early_aware_selector_.diagnostic();
    }
    if (speed_evidence != nullptr) {
      speed_evidence->post_state = capture_speed_state();
      speed_evidence->present_fields |= SPEED_EVIDENCE_POST_STATE;
      speed_evidence->state_changed =
          sameSpeedTransitionState(speed_evidence->pre_state,
                                   speed_evidence->post_state)
              ? 0U
              : 1U;
      if (speed_evidence->state_changed != 0U &&
          speed_evidence->branch == SpeedTransitionBranch::NONE) {
        speed_evidence->capture_state =
            SpeedEvidenceCaptureState::INVALID_CAPTURE;
        speed_evidence->failure = SpeedEvidenceFailure::INCOMPLETE_TRANSITION;
      } else {
        speed_evidence->capture_state =
            SpeedEvidenceCaptureState::TRIAL_COMPLETE_UNSEALED;
      }
    }
    return candidate;
  };
  output.front_detection_radius_m = detector_.detectionRadiusM();
  output.front_detection_transition_distance_m =
      detector_.maximumTransitionDistanceM();
  std::string mpc_guard_reason;
  updateMpcHealthGuard(mpc_health, &mpc_guard_reason);

  if ((config_.safety_evaluation_enabled && !inputs_fresh) || !ego.valid ||
      frame_ == nullptr || frame_->empty() || map_ == nullptr ||
      !map_->initialized()) {
    clearPassContinuationLatch();
    preventive_side_role_latch_.reset();
    early_aware_selector_.reset();
    output.early_aware_diagnostic = early_aware_selector_.diagnostic();
    output.active = true;
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = "invalid_or_stale_input";
    return finalize_output(std::move(output));
  }
  if (config_.safety_evaluation_enabled && ego.speed_mps < -0.05 &&
      !config_.allow_reverse) {
    clearPassContinuationLatch();
    preventive_side_role_latch_.reset();
    early_aware_selector_.reset();
    output.early_aware_diagnostic = early_aware_selector_.diagnostic();
    output.active = true;
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = "reverse_motion_not_allowed";
    return finalize_output(std::move(output));
  }

  const auto permission = activeOvertakePermission(ego.frenet.s);
  output.overtake_permission_allowed = permission.allow_overtake;
  output.overtake_permission_section_name = permission.name;
  output.overtake_permission_reason = permission.reason;

  auto planning_opponents = extrapolateOpponents(opponents, now_sec);
  TrajectoryPoint current_pose;
  current_pose.x = ego.x;
  current_pose.y = ego.y;
  current_pose.yaw = ego.yaw;
  current_pose.s = ego.frenet.s;
  current_pose.d = ego.frenet.d;
  current_pose.kappa = sanitizedCurvature(ego.curvature, config_);
  current_pose.time_sec = 0.0;
  const bool current_wall_collision =
      config_.safety_evaluation_enabled &&
      map_->footprintHitsWall(current_pose, wallFootprint(config_, false));
  CollisionDiagnostic current_opponent_collision;
  current_opponent_collision.required_clearance_m =
      config_.opponent_hard_clearance_m;
  const bool rear_only_exemption_allowed =
      !pass_continuation_latch_.has_value() &&
      (last_mode_ == BehaviorMode::FREE_RUN ||
       last_mode_ == BehaviorMode::FOLLOW_BLOCKED);
  for (std::size_t i = 0U; i < planning_opponents.size(); ++i) {
    const auto &predicted_opponent = planning_opponents[i];
    const auto diagnostic =
        opponentCollisionDiagnostic(current_pose, {predicted_opponent});
    if (!diagnostic.collision()) {
      continue;
    }
    const auto relation =
        classifyCurrentPoseOpponentRelation(ego, predicted_opponent);
    const bool has_observed_opponent = i < opponents.size();
    const bool rear_only_exempt =
        rear_only_exemption_allowed && has_observed_opponent &&
        rearOnlyCurrentPoseExemptionProvenSafe(ego, predicted_opponent,
                                               opponents[i], now_sec);
    if (rear_only_exempt) {
      current_pose_rear_only_exempt_opponent_ids_.push_back(
          predicted_opponent.id);
      output.current_opponent_relation = relation;
      output.current_pose_rear_only_exempt = true;
      if (diagnostic.clearance_m <
          output.current_pose_rear_only_exempt_clearance_m) {
        output.current_pose_rear_only_exempt_opponent_id =
            predicted_opponent.id;
        output.current_pose_rear_only_exempt_clearance_m =
            diagnostic.clearance_m;
      }
      continue;
    }
    if (diagnostic.clearance_m < current_opponent_collision.clearance_m) {
      current_opponent_collision = diagnostic;
      current_opponent_collision.relation = relation;
    }
  }
  if (current_wall_collision || current_opponent_collision.collision()) {
    clearPassContinuationLatch();
    preventive_side_role_latch_.reset();
    early_aware_selector_.reset();
    output.early_aware_diagnostic = early_aware_selector_.diagnostic();
    output.active = true;
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = "current_pose_hard_collision";
    if (current_wall_collision) {
      output.current_collision_kind = CollisionKind::WALL;
    } else {
      output.current_collision_kind = CollisionKind::OPPONENT;
      output.current_opponent_relation = current_opponent_collision.relation;
      output.blocking_opponent_id = current_opponent_collision.opponent_id;
      output.blocking_clearance_m = current_opponent_collision.clearance_m;
      output.blocking_required_clearance_m =
          current_opponent_collision.required_clearance_m;
    }
    return finalize_output(std::move(output));
  }

  const auto side_role =
      detectPreventiveSideRole(ego, opponents, planning_opponents, now_sec);
  output.preventive_side_role_eligibility_diagnostic = side_role.diagnostic;
  if (side_role.status == PreventiveSideRoleDecisionStatus::RELEASED) {
    preventive_side_role_latch_.reset();
    if (suspended_pass_continuation_latch_.has_value()) {
      const auto &suspended = *suspended_pass_continuation_latch_;
      const auto observed_target =
          std::find_if(opponents.cbegin(), opponents.cend(),
                       [&suspended](const OpponentState &candidate) {
                         return candidate.id == suspended.target_id;
                       });
      const bool fresh_same_target =
          observed_target != opponents.cend() && observed_target->valid &&
          std::isfinite(observed_target->stamp_sec) &&
          observed_target->stamp_sec > suspended.target_observation_stamp_sec +
                                           config_.tie_break_epsilon &&
          observed_target->stamp_sec <= now_sec + config_.tie_break_epsilon &&
          now_sec - observed_target->stamp_sec <= config_.opponent_stale_sec &&
          suspended.side != 0 && std::isfinite(suspended.required_arc_m) &&
          suspended.required_arc_m >=
              config_.minimum_lateral_transition_distance_m;
      if (fresh_same_target) {
        pass_continuation_latch_ = suspended;
        pass_continuation_latch_->target_observation_stamp_sec =
            observed_target->stamp_sec;
      }
      suspended_pass_continuation_latch_.reset();
    }
  } else if (side_role.status == PreventiveSideRoleDecisionStatus::AMBIGUOUS) {
    clearPassContinuationLatch();
    preventive_side_role_latch_.reset();
    early_aware_selector_.reset();
    output.early_aware_diagnostic = early_aware_selector_.diagnostic();
    selected_.reset();
    candidates_.clear();
    output.active = true;
    output.mode = BehaviorMode::SAFE_STOP;
    output.intent = SolverHorizonIntent::NONE;
    output.emergency_stop = true;
    output.safe_lateral = false;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = side_role.reason.empty() ? "preventive_side_role_ambiguous"
                                             : side_role.reason;
    last_mode_ = output.mode;
    return finalize_output(std::move(output));
  } else if (side_role.status == PreventiveSideRoleDecisionStatus::ACTIVE) {
    early_aware_selector_.reset();
    output.early_aware_diagnostic = early_aware_selector_.diagnostic();
    const auto &peer = planning_opponents[side_role.peer_index];
    output.active = true;
    output.target_id = peer.id;
    output.preventive_side_role_active = true;
    output.preventive_side_role_leader =
        side_role.role == PreventiveSideRoleKind::LEADER;
    output.preventive_side_role_peer_id = peer.id;
    output.preventive_side_role_clearance_m = side_role.clearance_m;
    for (const auto &opponent : planning_opponents) {
      if (!output.preventive_side_role_included_opponent_ids.empty()) {
        output.preventive_side_role_included_opponent_ids += ",";
      }
      output.preventive_side_role_included_opponent_ids += opponent.id;
    }

    // Role arbitration temporarily owns output selection, but it must not
    // destroy the normal detector/target/candidate continuity state. The pass
    // latch alone moves to a zero-authority suspended slot and is revalidated
    // before it can become active again.
    if (!preventive_side_role_latch_.has_value()) {
      if (pass_continuation_latch_.has_value()) {
        suspended_pass_continuation_latch_ =
            std::move(pass_continuation_latch_);
        pass_continuation_latch_.reset();
      }
      PreventiveSideRoleLatch latch;
      latch.peer_id = peer.id;
      latch.role = side_role.role;
      latch.phase = side_role.role == PreventiveSideRoleKind::LEADER
                        ? PreventiveSideRolePhase::LEADER_PROCEED
                        : (side_role.role == PreventiveSideRoleKind::FOLLOWER
                               ? PreventiveSideRolePhase::FOLLOWER_YIELD
                               : PreventiveSideRolePhase::NEUTRAL_HOLD);
      latch.generation = ++preventive_side_role_generation_;
      latch.decision_deadline_sec = side_role.diagnostic.decision_deadline_sec;
      const auto observed_peer =
          std::find_if(opponents.cbegin(), opponents.cend(),
                       [&peer](const OpponentState &candidate) {
                         return candidate.id == peer.id;
                       });
      if (observed_peer != opponents.cend()) {
        latch.last_observation_stamp_sec = observed_peer->stamp_sec;
        latch.previous_peer_x_m = observed_peer->x;
        latch.previous_peer_y_m = observed_peer->y;
      }
      preventive_side_role_latch_ = std::move(latch);
    }
    auto &role_latch = *preventive_side_role_latch_;
    if (mpc_health_guard_latched_ || safe_stop_latched_) {
      // A health or safety-stop cycle cannot retain or grant yield/exit
      // evidence. Consume its peer stamp as the interruption boundary, then
      // require a completely new distinct-sample proof after recovery.
      role_latch.phase =
          role_latch.role == PreventiveSideRoleKind::LEADER
              ? PreventiveSideRolePhase::LEADER_PROCEED
              : (role_latch.role == PreventiveSideRoleKind::FOLLOWER
                     ? PreventiveSideRolePhase::FOLLOWER_YIELD
                     : PreventiveSideRolePhase::NEUTRAL_HOLD);
      role_latch.first_yield_observation_stamp_sec = -1.0;
      role_latch.yield_sample_count = 0;
      role_latch.exit_sample_count = 0;
      const auto observed_peer =
          std::find_if(opponents.cbegin(), opponents.cend(),
                       [&peer](const OpponentState &candidate) {
                         return candidate.id == peer.id;
                       });
      if (observed_peer != opponents.cend()) {
        role_latch.last_observation_stamp_sec = observed_peer->stamp_sec;
      }
    }
    output.preventive_side_role_phase = role_latch.phase;
    output.preventive_side_role_neutral =
        role_latch.role == PreventiveSideRoleKind::NEUTRAL;
    output.preventive_side_role_generation = role_latch.generation;
    output.preventive_side_role_required_gap_m = side_role.required_gap_m;
    output.preventive_side_role_current_gap_m =
        std::abs(side_role.diagnostic.cartesian_longitudinal_m);
    output.preventive_side_role_yield_sample_count =
        role_latch.yield_sample_count;
    output.preventive_side_role_yield_sample_span_sec =
        role_latch.yield_sample_count > 0
            ? std::max(0.0, role_latch.last_observation_stamp_sec -
                                role_latch.first_yield_observation_stamp_sec)
            : 0.0;
    output.preventive_side_role_peer_tangent_speed_mps =
        std::abs(side_role.diagnostic.peer_tangent_progress_mps);

    const auto role_stop = [&](const std::string &reason,
                               const OutputHorizonDiagnostic &diagnostic =
                                   OutputHorizonDiagnostic{}) {
      PlannerOutput stop = output;
      selected_.reset();
      candidates_.clear();
      stop.mode = BehaviorMode::SAFE_STOP;
      stop.preventive_side_role_action = PreventiveSideRoleAction::SAFE_STOP;
      stop.intent = SolverHorizonIntent::NONE;
      stop.emergency_stop = true;
      stop.safe_lateral = false;
      stop.lateral_offsets_m.clear();
      stop.speed_caps_mps.clear();
      stop.longitudinal_offsets_m.clear();
      const double dt_sec = 1.0 / config_.planner_rate_hz;
      const double previous_speed_mps =
          speed_state_initialized_
              ? std::clamp(previous_command_speed_mps_,
                           config_.safe_stop_speed_mps,
                           std::max(config_.safe_stop_speed_mps,
                                    std::abs(ego.speed_mps)))
              : std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps));
      const double previous_acceleration_mps2 =
          speed_state_initialized_ ? previous_acceleration_mps2_ : 0.0;
      const double acceleration_mps2 =
          std::min(0.0, jerkLimitedAcceleration(
                            config_.safe_stop_speed_mps, previous_speed_mps,
                            std::min(0.0, previous_acceleration_mps2), dt_sec,
                            false, config_));
      stop.speed_cap_mps =
          std::clamp(previous_speed_mps + acceleration_mps2 * dt_sec,
                     config_.safe_stop_speed_mps, previous_speed_mps);
      record_speed_transition(SpeedTransitionBranch::ROLE_STOP_JERK_LIMITED,
                              dt_sec, config_.safe_stop_speed_mps,
                              config_.safe_stop_speed_mps, previous_speed_mps,
                              true, false);
      speed_state_initialized_ = true;
      previous_command_speed_mps_ = stop.speed_cap_mps;
      previous_acceleration_mps2_ = acceleration_mps2;
      stop.output_horizon_diagnostic = diagnostic;
      stop.reason = reason;
      populatePassContinuationDiagnostic(&stop, false);
      last_mode_ = stop.mode;
      return finalize_output(std::move(stop));
    };
    if (mpc_health_guard_latched_) {
      auto stop = role_stop(mpc_guard_reason.empty()
                                ? "preventive_side_role_mpc_health_guard"
                                : mpc_guard_reason);
      suspended_pass_continuation_latch_.reset();
      preventive_side_role_latch_.reset();
      return finalize_output(std::move(stop));
    }
    if (safe_stop_latched_) {
      auto stop = role_stop("preventive_side_role_safe_stop_latched");
      suspended_pass_continuation_latch_.reset();
      preventive_side_role_latch_.reset();
      return finalize_output(std::move(stop));
    }
    PreventiveSideRoleCandidateDiagnostic side_candidate_diagnostic;
    side_candidate_diagnostic.peer_id = peer.id;
    const auto candidate_generation_started = std::chrono::steady_clock::now();
    std::vector<OpponentState> generation_opponents;
    generation_opponents.reserve(planning_opponents.size());
    std::copy_if(planning_opponents.cbegin(), planning_opponents.cend(),
                 std::back_inserter(generation_opponents),
                 [&peer](const OpponentState &opponent) {
                   return opponent.id != peer.id;
                 });
    candidates_ =
        generateCandidatesInternal(ego, generation_opponents);
    auto straight_corridor =
        roleCorridorCandidate(ego, ego.frenet.d, generation_opponents);
    if (straight_corridor.has_value()) {
      candidates_.push_back(std::move(straight_corridor.value()));
    }
    if (role_latch.role == PreventiveSideRoleKind::LEADER) {
      const double away_sign =
          side_role.diagnostic.cartesian_lateral_m > 0.0 ? -1.0 : 1.0;
      const double away_goal_d_m =
          std::clamp(ego.frenet.d + away_sign * 0.40,
                     -config_.max_adaptive_lateral_offset_m,
                     config_.max_adaptive_lateral_offset_m);
      auto away_corridor =
          roleCorridorCandidate(ego, away_goal_d_m, generation_opponents);
      if (away_corridor.has_value()) {
        candidates_.push_back(std::move(away_corridor.value()));
      }
    }
    if (role_latch.role == PreventiveSideRoleKind::FOLLOWER) {
      auto moving_follow =
          movingTargetFollowCandidate(ego, peer, planning_opponents);
      if (moving_follow.has_value()) {
        candidates_.push_back(std::move(moving_follow.value()));
      }
    }
    last_planning_cycle_metrics_.candidate_generation_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - candidate_generation_started)
            .count();
    output.generated_candidates = static_cast<int>(candidates_.size());
    side_candidate_diagnostic.generated_candidates =
        output.generated_candidates;
    if (candidates_.empty()) {
      side_candidate_diagnostic.failure =
          PreventiveSideRoleCandidateFailure::NO_GENERATED_CANDIDATE;
      output.preventive_side_role_candidate_diagnostic =
          side_candidate_diagnostic;
      return role_stop("preventive_side_role_leader_wait_no_safe_candidate");
    }
    std::vector<CandidateTrajectory *> separation_feasible;
    const auto candidate_body_left_displacement_m =
        [this, &ego](const CandidateTrajectory &candidate) {
          double selected_displacement_m = 0.0;
          for (const auto &point : candidate.dense) {
            const auto current_corridor =
                frame_->frenetToCartesian(point.s, ego.frenet.d);
            const double relative_x_m = point.x - current_corridor.x;
            const double relative_y_m = point.y - current_corridor.y;
            const double body_left_m = -relative_x_m * std::sin(ego.yaw) +
                                       relative_y_m * std::cos(ego.yaw);
            if (std::isfinite(body_left_m) &&
                std::abs(body_left_m) > std::abs(selected_displacement_m)) {
              selected_displacement_m = body_left_m;
            }
          }
          return selected_displacement_m;
        };
    for (auto &candidate : candidates_) {
      if (!candidate.feasible) {
        continue;
      }
      ++side_candidate_diagnostic.base_feasible_candidates;
      const double body_left_displacement_m =
          candidate_body_left_displacement_m(candidate);
      const bool straight = std::abs(body_left_displacement_m) <= 0.05;
      const bool away_from_peer = side_role.diagnostic.cartesian_lateral_m > 0.0
                                      ? body_left_displacement_m < 0.0
                                      : body_left_displacement_m > 0.0;
      const bool before_deadline =
          !std::isfinite(role_latch.decision_deadline_sec) ||
          now_sec < role_latch.decision_deadline_sec;
      const bool direction_allowed =
          straight || (role_latch.role == PreventiveSideRoleKind::LEADER &&
                       away_from_peer && before_deadline);
      if (!direction_allowed) {
        continue;
      }
      separation_feasible.push_back(&candidate);
    }
    side_candidate_diagnostic.separation_feasible_candidates =
        static_cast<int>(separation_feasible.size());
    output.feasible_candidates =
        side_candidate_diagnostic.separation_feasible_candidates;
    if (side_candidate_diagnostic.base_feasible_candidates == 0) {
      side_candidate_diagnostic.failure =
          PreventiveSideRoleCandidateFailure::NO_BASE_FEASIBLE_CANDIDATE;
      output.preventive_side_role_candidate_diagnostic =
          side_candidate_diagnostic;
      return role_stop("preventive_side_role_leader_wait_no_safe_candidate");
    }
    if (separation_feasible.empty()) {
      side_candidate_diagnostic.generated_candidates =
          output.generated_candidates;
      side_candidate_diagnostic.base_feasible_candidates = static_cast<int>(
          std::count_if(candidates_.cbegin(), candidates_.cend(),
                        [](const CandidateTrajectory &candidate) {
                          return candidate.feasible;
                        }));
      side_candidate_diagnostic.separation_feasible_candidates = 0;
      side_candidate_diagnostic.failure =
          PreventiveSideRoleCandidateFailure::DIRECTION;
      side_candidate_diagnostic.first_reject_reason = "role_direction";
      output.preventive_side_role_candidate_diagnostic =
          side_candidate_diagnostic;
      return role_stop("preventive_side_role_leader_wait_no_safe_candidate");
    }
    std::sort(
        separation_feasible.begin(), separation_feasible.end(),
        [&candidate_body_left_displacement_m](const CandidateTrajectory *lhs,
                                              const CandidateTrajectory *rhs) {
          return std::make_tuple(
                     std::abs(candidate_body_left_displacement_m(*lhs)),
                     lhs->total_cost, lhs->lateral_index, lhs->tangent_index) <
                 std::make_tuple(
                     std::abs(candidate_body_left_displacement_m(*rhs)),
                     rhs->total_cost, rhs->lateral_index, rhs->tangent_index);
        });
    auto *choice = separation_feasible.front();
    last_planning_cycle_metrics_.candidate_dense_point_count =
        choice->dense.size();

    const double current_speed_cap =
        std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps));
    const double previous_speed_mps =
        speed_state_initialized_
            ? std::clamp(previous_command_speed_mps_,
                         config_.safe_stop_speed_mps, current_speed_cap)
            : current_speed_cap;
    const double current_gap_m =
        std::abs(side_role.diagnostic.cartesian_longitudinal_m);
    const double required_gap_m =
        std::isfinite(side_role.required_gap_m)
            ? side_role.required_gap_m
            : config_.preventive_side_role_follower_min_gap_m;
    const double gap_error_m = std::max(0.0, required_gap_m - current_gap_m);
    double requested_speed_cap_mps = current_speed_cap;
    if (role_latch.role == PreventiveSideRoleKind::FOLLOWER) {
      requested_speed_cap_mps =
          std::max(0.0, side_role.diagnostic.peer_tangent_progress_mps) -
          config_.preventive_side_role_follower_speed_reduction_mps -
          config_.preventive_side_role_follower_gap_gain_per_s * gap_error_m;
    } else if (role_latch.role == PreventiveSideRoleKind::NEUTRAL) {
      requested_speed_cap_mps =
          current_speed_cap -
          config_.preventive_side_role_neutral_speed_reduction_mps;
    }
    requested_speed_cap_mps =
        std::clamp(requested_speed_cap_mps, config_.safe_stop_speed_mps,
                   std::min(choice->entry_speed_limit_mps,
                            choice->dynamic_speed_limit_mps));
    const double dt_sec = 1.0 / config_.planner_rate_hz;
    const double previous_acceleration_mps2 =
        speed_state_initialized_ ? previous_acceleration_mps2_ : 0.0;
    const double role_acceleration_mps2 = jerkLimitedAcceleration(
        requested_speed_cap_mps, previous_speed_mps, previous_acceleration_mps2,
        dt_sec, false, config_);
    double role_speed_cap =
        std::clamp(previous_speed_mps + role_acceleration_mps2 * dt_sec,
                   config_.safe_stop_speed_mps,
                   std::min(choice->entry_speed_limit_mps,
                            choice->dynamic_speed_limit_mps));
    output.preventive_side_role_requested_speed_cap_mps =
        requested_speed_cap_mps;
    output.preventive_side_role_applied_speed_cap_mps = role_speed_cap;

    const auto output_horizon_started = std::chrono::steady_clock::now();
    RoleCandidateEvaluation role_evaluation;
    PreventiveSideRoleCandidateDiagnostic first_reject;
    bool reject_recorded = false;
    for (auto *candidate : separation_feasible) {
      const double candidate_speed_cap =
          std::clamp(role_speed_cap, config_.safe_stop_speed_mps,
                     std::min(candidate->entry_speed_limit_mps,
                              candidate->dynamic_speed_limit_mps));
      auto evaluation =
          evaluateRoleCandidate(ego, peer, planning_opponents, *candidate,
                                candidate_speed_cap, role_latch.generation);
      ++last_planning_cycle_metrics_.output_horizon_call_count;
      if (evaluation.safe) {
        choice = candidate;
        role_speed_cap = candidate_speed_cap;
        role_evaluation = std::move(evaluation);
        break;
      }
      role_evaluation = evaluation;
      if (!reject_recorded) {
        first_reject = evaluation.role_diagnostic;
        reject_recorded = true;
      }
    }
    last_planning_cycle_metrics_.output_horizon_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - output_horizon_started)
            .count();
    if (!role_evaluation.safe) {
      output.preventive_side_role_candidate_diagnostic =
          reject_recorded ? first_reject : role_evaluation.role_diagnostic;
      return role_stop(std::string("preventive_side_role_output_unsafe_") +
                           toString(role_evaluation.output_diagnostic.failure),
                       role_evaluation.output_diagnostic);
    }

    selected_ = *choice;
    output.mode = BehaviorMode::SIDE_BY_SIDE_KEEP;
    const double body_left_displacement_m =
        candidate_body_left_displacement_m(*choice);
    output.intent = SolverHorizonIntent::MANDATORY_AVOIDANCE;
    output.safe_lateral = true;
    output.emergency_stop = false;
    output.lateral_offsets_m = std::move(role_evaluation.lateral_offsets_m);
    output.speed_caps_mps = std::move(role_evaluation.speed_caps_mps);
    output.longitudinal_offsets_m =
        std::move(role_evaluation.longitudinal_offsets_m);
    output.spatial_profile_shadow_only =
        config_.experimental_exact_spatial_follow_shadow_enabled &&
        !output.longitudinal_offsets_m.empty();
    output.speed_cap_mps = role_speed_cap;
    output.minimum_cost = choice->total_cost;
    output.candidate_speed_limit_mps = choice->dynamic_speed_limit_mps;
    if (role_latch.role == PreventiveSideRoleKind::NEUTRAL) {
      output.preventive_side_role_action =
          PreventiveSideRoleAction::NEUTRAL_STRAIGHT_HOLD;
    } else if (role_latch.role == PreventiveSideRoleKind::FOLLOWER) {
      output.preventive_side_role_action =
          choice->lateral_index == config_.lateral_targets_m.size()
              ? PreventiveSideRoleAction::FOLLOW_BEHIND
              : PreventiveSideRoleAction::STRAIGHT_YIELD;
    } else if (std::abs(body_left_displacement_m) <= 0.05) {
      output.preventive_side_role_action =
          PreventiveSideRoleAction::STRAIGHT_CURRENT_CORRIDOR;
    } else {
      output.preventive_side_role_action =
          body_left_displacement_m > 0.0
              ? PreventiveSideRoleAction::LEFT_ESCAPE
              : PreventiveSideRoleAction::RIGHT_ESCAPE;
    }
    output.preventive_side_role_candidate_diagnostic =
        role_evaluation.role_diagnostic;
    output.reason = toString(output.preventive_side_role_action);
    populatePassContinuationDiagnostic(&output, false);
    record_speed_transition(
        SpeedTransitionBranch::ROLE_ACTIVE_JERK_LIMITED, dt_sec,
        requested_speed_cap_mps, config_.safe_stop_speed_mps,
        std::min(choice->entry_speed_limit_mps,
                 choice->dynamic_speed_limit_mps),
        false, false, static_cast<std::int32_t>(choice->lateral_index),
        static_cast<std::int32_t>(choice->tangent_index), choice->total_cost);
    speed_state_initialized_ = true;
    previous_command_speed_mps_ = role_speed_cap;
    previous_acceleration_mps2_ = role_acceleration_mps2;
    if (last_mode_ != output.mode) {
      last_mode_change_sec_ = now_sec;
    }
    last_mode_ = output.mode;
    return finalize_output(std::move(output));
  }

  // This read-only selector runs only after the existing stale/reverse/current
  // hard-stop and preventive-role gates. It must never retain an observation
  // through any of those authority-bearing paths.
  early_aware_selector_.update(ego, opponents, *frame_, inputs_fresh, now_sec);
  output.early_aware_diagnostic = early_aware_selector_.diagnostic();

  const auto detection = detector_.update(ego, planning_opponents, *frame_);
  output.front_detection_radius_m = detector_.detectionRadiusM();
  output.front_detection_transition_distance_m =
      detector_.maximumTransitionDistanceM();
  output.front_detection_diagnostic = detector_.diagnostic();
  if (!active_ && detection.has_value()) {
    active_ = true;
    target_id_ = detection.value();
    last_mode_change_sec_ = now_sec;
  }
  if (!active_) {
    clearPassContinuationLatch();
    output.reason = "free_run";
    applyMpcHealthGuard(&output, mpc_guard_reason);
    return finalize_output(std::move(output));
  }

  output.active = true;
  output.target_id = target_id_;
  const auto live_target = std::find_if(
      opponents.cbegin(), opponents.cend(), [this](const auto &opponent) {
        return opponent.valid && opponent.id == target_id_;
      });
  const bool live_target_present = live_target != opponents.cend();
  const bool continuation_was_latched = pass_continuation_latch_.has_value();
  const bool continuation_target_matches =
      continuation_was_latched && live_target_present &&
      pass_continuation_latch_->target_id == target_id_ &&
      std::isfinite(live_target->stamp_sec) &&
      live_target->stamp_sec + config_.tie_break_epsilon >=
          pass_continuation_latch_->target_observation_stamp_sec;
  if (continuation_was_latched && !continuation_target_matches) {
    // A predicted target is allowed only for the existing recovery path, never
    // to continue a previously authorized pass.
    clearPassContinuationLatch();
  }
  bool target_missing_recovery = false;
  const OpponentState *overtake_target = target(planning_opponents);
  if (overtake_target != nullptr) {
    last_target_state_ = *overtake_target;
    last_target_observed_sec_ = now_sec;
    target_missing_since_sec_ = -1.0;
  } else {
    output.target_missing = true;
    if (target_missing_since_sec_ < 0.0) {
      target_missing_since_sec_ = now_sec;
    }
    const double missing_age =
        std::max(0.0, now_sec - target_missing_since_sec_);
    if (last_target_state_.has_value()) {
      OpponentState predicted = last_target_state_.value();
      const double dt = std::max(0.0, now_sec - last_target_observed_sec_);
      predicted.x += predicted.vx_mps * dt;
      predicted.y += predicted.vy_mps * dt;
      predicted.stamp_sec = now_sec;
      const double uncertainty_growth =
          config_.target_missing_uncertainty_growth_mps * dt;
      predicted.uncertainty_x_m =
          std::min(config_.sigma_max_margin_m,
                   predicted.uncertainty_x_m + uncertainty_growth);
      predicted.uncertainty_y_m =
          std::min(config_.sigma_max_margin_m,
                   predicted.uncertainty_y_m + uncertainty_growth);
      predicted.frenet =
          frame_->project(predicted.x, predicted.y, predicted.yaw);
      predicted.valid = predicted.frenet.valid;
      if (predicted.valid && missing_age <= config_.target_missing_forget_sec) {
        planning_opponents.push_back(predicted);
        if (missing_age <= config_.target_missing_prediction_grace_sec) {
          overtake_target = &planning_opponents.back();
        }
      }
    }
    if (missing_age > config_.target_missing_prediction_grace_sec ||
        overtake_target == nullptr) {
      target_missing_recovery = true;
    }
  }

  const auto candidate_generation_started = std::chrono::steady_clock::now();
  candidates_ = generateCandidatesInternal(ego, planning_opponents);
  std::vector<OpponentState> parallel_opponents;
  for (std::size_t i = 0U; i < output.front_detection_diagnostic.opponent_count;
       ++i) {
    const auto &diagnostic = output.front_detection_diagnostic.opponents[i];
    if (diagnostic.target_class != FrontTargetClass::PARALLEL) {
      continue;
    }
    const auto opponent =
        std::find_if(planning_opponents.cbegin(), planning_opponents.cend(),
                     [&diagnostic](const auto &candidate) {
                       return candidate.id == diagnostic.opponent_id.data();
                     });
    if (opponent != planning_opponents.cend()) {
      parallel_opponents.push_back(*opponent);
    }
  }
  for (auto &candidate : candidates_) {
    if (candidate.dense.empty() || parallel_opponents.empty()) {
      continue;
    }
    const double start_clearance =
        opponentCollisionDiagnostic(candidate.dense.front(), parallel_opponents)
            .clearance_m;
    const double terminal_clearance =
        opponentCollisionDiagnostic(candidate.dense.back(), parallel_opponents)
            .clearance_m;
    candidate.opponent_clearance_recovery_m =
        std::isfinite(start_clearance) && std::isfinite(terminal_clearance)
            ? terminal_clearance - start_clearance
            : 0.0;
  }
  last_planning_cycle_metrics_.candidate_generation_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - candidate_generation_started)
          .count();
  for (const auto &candidate : candidates_) {
    last_planning_cycle_metrics_.candidate_dense_point_count +=
        candidate.dense.size();
    if (last_planning_cycle_metrics_.candidate_diagnostic_count <
        last_planning_cycle_metrics_.candidate_diagnostics.size()) {
      auto &diagnostic =
          last_planning_cycle_metrics_.candidate_diagnostics
              [last_planning_cycle_metrics_.candidate_diagnostic_count++];
      diagnostic.lateral_index = candidate.lateral_index;
      diagnostic.tangent_index = candidate.tangent_index;
      diagnostic.goal_d_m = candidate.goal_d_m;
      diagnostic.tangent_scale = candidate.tangent_scale;
      diagnostic.required_arc_m = candidate.required_arc_m;
      diagnostic.total_cost = candidate.total_cost;
      diagnostic.reference_cost = candidate.reference_cost;
      diagnostic.wall_cost = candidate.wall_cost;
      diagnostic.object_cost = candidate.object_cost;
      diagnostic.representative_wall_diagnostic_valid =
          candidate.representative_wall_diagnostic_valid;
      diagnostic.representative_minimum_nominal_wall_clearance_proxy_m =
          candidate.representative_minimum_nominal_wall_clearance_proxy_m;
      diagnostic.representative_minimum_nominal_wall_clearance_point_index =
          candidate.representative_minimum_nominal_wall_clearance_point_index;
      diagnostic.representative_maximum_nominal_wall_level =
          candidate.representative_maximum_nominal_wall_level;
      diagnostic.representative_maximum_nominal_wall_level_point_index =
          candidate.representative_maximum_nominal_wall_level_point_index;
      diagnostic.representative_object_diagnostic_count =
          candidate.representative_object_diagnostic_count;
      diagnostic.representative_object_diagnostics =
          candidate.representative_object_diagnostics;
      diagnostic.opponent_clearance_recovery_m =
          candidate.opponent_clearance_recovery_m;
      diagnostic.start_x_m = ego.x;
      diagnostic.start_y_m = ego.y;
      diagnostic.start_yaw_rad = ego.yaw;
      diagnostic.start_s_m = ego.frenet.s;
      diagnostic.start_d_m = ego.frenet.d;
      const std::string_view reason =
          candidate.rejection_reason.empty()
              ? std::string_view{"feasible"}
              : std::string_view{candidate.rejection_reason};
      const std::size_t copy_size =
          std::min(reason.size(), diagnostic.first_reject_reason.size() - 1U);
      std::copy_n(reason.data(), copy_size,
                  diagnostic.first_reject_reason.data());
      diagnostic.first_reject_reason[copy_size] = '\0';
    }
  }
  output.generated_candidates = static_cast<int>(candidates_.size());
  std::vector<CandidateTrajectory *> feasible;
  std::vector<CandidateTrajectory *> preparable;
  for (auto &candidate : candidates_) {
    if (candidate.feasible) {
      feasible.push_back(&candidate);
    }
    if (candidate.requires_entry_deceleration) {
      preparable.push_back(&candidate);
    }
    if (candidate.feasible) {
      continue;
    }
    switch (candidateRejectBucket(candidate.rejection_reason)) {
    case CandidateRejectBucket::WALL:
      ++output.rejected_wall_candidates;
      break;
    case CandidateRejectBucket::OPPONENT:
      ++output.rejected_opponent_candidates;
      break;
    case CandidateRejectBucket::CURVATURE:
      ++output.rejected_curvature_candidates;
      break;
    case CandidateRejectBucket::TRACKABILITY:
      ++output.rejected_trackability_candidates;
      break;
    case CandidateRejectBucket::OTHER:
      ++output.rejected_other_candidates;
      break;
    }
  }
  output.feasible_candidates = static_cast<int>(feasible.size());

  double target_forward_gap = std::numeric_limits<double>::infinity();
  double target_rear_gap = std::numeric_limits<double>::infinity();
  bool target_ahead = false;
  bool target_passed = false;
  bool side_by_side_hold = false;
  if (overtake_target != nullptr && !target_missing_recovery) {
    target_forward_gap =
        frame_->forwardDeltaS(ego.frenet.s, overtake_target->frenet.s);
    target_rear_gap =
        frame_->forwardDeltaS(overtake_target->frenet.s, ego.frenet.s);
    target_ahead = target_forward_gap > config_.frontmost_s_tolerance_m &&
                   target_forward_gap < frame_->length() * 0.5;
    target_passed = target_rear_gap >= config_.passed_target_gap_m &&
                    target_rear_gap < frame_->length() * 0.5;
    side_by_side_hold = !target_ahead && !target_passed;
  }

  const auto is_sub_stop_cost = [this](const auto *candidate) {
    return candidate->safety_cost < config_.stop_cost;
  };
  const auto is_pass_class = [](const auto *candidate) {
    return std::abs(candidate->goal_d_m) > 0.05;
  };
  const auto pass_capable_goal = [this,
                                  overtake_target](const auto *candidate) {
    auto diagnostic = evaluatePassClearanceDiagnostic(
        candidate->goal_d_m, *overtake_target, config_);
    const bool observed_predicate = diagnostic.observed_predicate;
    auto &recorded = last_planning_cycle_metrics_.pass_clearance_diagnostic;
    if (!recorded.evaluated || !recorded.observed_predicate) {
      recorded = std::move(diagnostic);
    }
    return observed_predicate;
  };
  const bool prioritize_pass_class = !target_missing_recovery &&
                                     overtake_target != nullptr &&
                                     (target_ahead || side_by_side_hold);
  const auto same_maneuver_class = [prioritize_pass_class, &is_pass_class](
                                       const auto *lhs, const auto *rhs) {
    return !prioritize_pass_class || is_pass_class(lhs) == is_pass_class(rhs);
  };
  const auto normal_order = [this](const auto *lhs, const auto *rhs) {
    if (lhs->total_cost != rhs->total_cost) {
      return lhs->total_cost < rhs->total_cost;
    }
    if (std::abs(lhs->goal_d_m) != std::abs(rhs->goal_d_m)) {
      return std::abs(lhs->goal_d_m) < std::abs(rhs->goal_d_m);
    }
    if (std::abs(lhs->total_abs_steer_change - rhs->total_abs_steer_change) >
        config_.tie_break_epsilon) {
      return lhs->total_abs_steer_change < rhs->total_abs_steer_change;
    }
    return std::tie(lhs->lateral_index, lhs->tangent_index) <
           std::tie(rhs->lateral_index, rhs->tangent_index);
  };

  OutputHorizonDiagnostic first_output_horizon_failure;
  const auto remember_output_horizon_failure =
      [&](OutputHorizonDiagnostic diagnostic,
          const CandidateTrajectory &candidate) {
        if (first_output_horizon_failure.failure !=
                OutputHorizonFailure::NONE ||
            diagnostic.failure == OutputHorizonFailure::NONE) {
          return;
        }
        diagnostic.candidate_goal_d_m = candidate.goal_d_m;
        diagnostic.candidate_tangent_scale = candidate.tangent_scale;
        first_output_horizon_failure = std::move(diagnostic);
      };

  const auto timed_output_horizon =
      [&](const CandidateTrajectory &candidate, double target_speed_mps,
          std::vector<double> *d, std::vector<double> *speed,
          std::vector<double> *longitudinal,
          OutputHorizonDiagnostic *diagnostic) {
        const auto output_horizon_started = std::chrono::steady_clock::now();
        const bool valid =
            outputHorizon(ego, candidate, planning_opponents, target_speed_mps,
                          d, speed, longitudinal, diagnostic);
        last_planning_cycle_metrics_.output_horizon_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - output_horizon_started)
                .count();
        ++last_planning_cycle_metrics_.output_horizon_call_count;
        return valid;
      };

  const auto blocked_speed_cap = [&]() {
    const double buffer = overtake_target == nullptr
                              ? 0.0
                              : requiredPreObstacleLongitudinalSeparation(
                                    config_, *overtake_target);
    const double free_gap = std::max(0.0, target_forward_gap - buffer);
    const double target_speed = overtake_target == nullptr
                                    ? 0.0
                                    : std::clamp(overtake_target->speed_mps,
                                                 0.0, config_.normal_speed_mps);
    const double braking_allowance = std::sqrt(std::max(
        0.0, 2.0 * std::abs(config_.min_acceleration_mps2) * free_gap));
    const double braking_speed_cap = target_speed + braking_allowance;
    const double closing_speed =
        std::max(0.0, std::abs(ego.speed_mps) - target_speed);
    const double gap_error = free_gap - config_.follow_desired_extra_gap_m;
    const double regulated_speed_cap =
        target_speed + config_.follow_gap_gain_per_s * gap_error -
        config_.follow_closing_speed_gain * closing_speed;
    return std::clamp(std::min(braking_speed_cap, regulated_speed_cap),
                      config_.safe_stop_speed_mps, config_.normal_speed_mps);
  };

  const auto follow_blocked_output = [&](const std::string &reason) {
    clearPassContinuationLatch();
    PlannerOutput follow = output;
    if (config_.experimental_exact_spatial_follow_shadow_enabled &&
        target_ahead && overtake_target != nullptr) {
      const auto moving_follow = movingTargetFollowCandidate(
          ego, *overtake_target, planning_opponents);
      if (moving_follow.has_value()) {
        auto stopped_target_opponents = planning_opponents;
        for (auto &opponent : stopped_target_opponents) {
          if (opponent.id == overtake_target->id) {
            opponent.speed_mps = 0.0;
            opponent.vx_mps = 0.0;
            opponent.vy_mps = 0.0;
          }
        }
        std::vector<double> follow_d;
        std::vector<double> follow_speed;
        std::vector<double> follow_longitudinal;
        OutputHorizonDiagnostic diagnostic;
        const auto output_horizon_started = std::chrono::steady_clock::now();
        const double follow_speed_cap = std::clamp(
            std::min(blocked_speed_cap(), moving_follow->entry_speed_limit_mps),
            config_.safe_stop_speed_mps, config_.normal_speed_mps);
        const bool output_safe =
            outputHorizon(ego, moving_follow.value(), stopped_target_opponents,
                          follow_speed_cap, &follow_d, &follow_speed,
                          &follow_longitudinal, &diagnostic);
        last_planning_cycle_metrics_.output_horizon_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - output_horizon_started)
                .count();
        ++last_planning_cycle_metrics_.output_horizon_call_count;
        if (output_safe) {
          selected_ = moving_follow.value();
          previous_lateral_index_ =
              static_cast<int>(moving_follow->lateral_index);
          previous_tangent_index_ =
              static_cast<int>(moving_follow->tangent_index);
          follow.mode = BehaviorMode::FOLLOW_BLOCKED;
          follow.intent = SolverHorizonIntent::MANEUVER_AUTHORIZED;
          follow.safe_lateral = true;
          follow.emergency_stop = false;
          follow.lateral_offsets_m = std::move(follow_d);
          follow.speed_caps_mps = std::move(follow_speed);
          follow.longitudinal_offsets_m = std::move(follow_longitudinal);
          follow.spatial_profile_shadow_only = true;
          follow.speed_cap_mps = follow_speed_cap;
          follow.minimum_cost = moving_follow->total_cost;
          follow.candidate_speed_limit_mps =
              moving_follow->dynamic_speed_limit_mps;
          follow.output_horizon_diagnostic = OutputHorizonDiagnostic{};
          follow.reason = std::abs(moving_follow->goal_d_m - ego.frenet.d) <=
                                  config_.tie_break_epsilon
                              ? "moving_target_current_d_follow"
                              : "moving_target_staged_lateral_return";
          record_speed_transition(
              SpeedTransitionBranch::MOVING_FOLLOW_DIRECT, 0.0,
              follow_speed_cap, config_.safe_stop_speed_mps,
              config_.normal_speed_mps, false, false,
              static_cast<std::int32_t>(moving_follow->lateral_index),
              static_cast<std::int32_t>(moving_follow->tangent_index),
              moving_follow->total_cost);
          safe_stop_latched_ = false;
          safe_stop_release_count_ = 0;
          speed_state_initialized_ = true;
          previous_command_speed_mps_ = std::min(
              std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps)),
              follow_speed_cap);
          previous_acceleration_mps2_ = config_.min_acceleration_mps2;
          applyMpcHealthGuard(&follow, mpc_guard_reason);
          if (follow.mode != last_mode_) {
            last_mode_change_sec_ = now_sec;
          }
          last_mode_ = follow.mode;
          return finalize_output(std::move(follow));
        }
        remember_output_horizon_failure(std::move(diagnostic),
                                        moving_follow.value());
      }
    }
    selected_.reset();
    follow.active = true;
    follow.safe_lateral = false;
    follow.intent = SolverHorizonIntent::NONE;
    follow.lateral_offsets_m.clear();
    follow.speed_caps_mps.clear();
    follow.longitudinal_offsets_m.clear();
    follow.candidate_speed_limit_mps = 0.0;
    follow.output_horizon_diagnostic = first_output_horizon_failure;
    const double buffer = overtake_target == nullptr
                              ? 0.0
                              : requiredPreObstacleLongitudinalSeparation(
                                    config_, *overtake_target);
    follow.speed_cap_mps = blocked_speed_cap();
    if (target_forward_gap <= buffer + 1.0e-6) {
      follow.mode = BehaviorMode::SAFE_STOP;
      follow.emergency_stop = true;
      follow.speed_cap_mps = config_.safe_stop_speed_mps;
      follow.reason = "blocked_target_inside_stop_buffer";
      record_speed_transition(SpeedTransitionBranch::LATCH_ONLY_STOP, 0.0,
                              follow.speed_cap_mps, config_.safe_stop_speed_mps,
                              config_.normal_speed_mps, true, false);
      safe_stop_latched_ = true;
    } else {
      follow.mode = BehaviorMode::FOLLOW_BLOCKED;
      follow.reason = reason;
      record_speed_transition(SpeedTransitionBranch::LATCH_ONLY_CLEAR, 0.0,
                              follow.speed_cap_mps, config_.safe_stop_speed_mps,
                              config_.normal_speed_mps, false, false);
      safe_stop_latched_ = false;
      safe_stop_release_count_ = 0;
    }
    applyMpcHealthGuard(&follow, mpc_guard_reason);
    if (follow.mode != last_mode_ && follow.mode != BehaviorMode::SAFE_STOP) {
      last_mode_change_sec_ = now_sec;
    }
    last_mode_ = follow.mode;
    return finalize_output(std::move(follow));
  };

  // A rear-only exemption proves only that the current pose need not stop.
  // It never grants a new lateral maneuver in that same cycle: retain the
  // existing follow path and its braking/output-horizon checks instead.
  if (output.current_pose_rear_only_exempt) {
    return follow_blocked_output("rear_only_exemption_follow_hold");
  }

  const bool pass_maneuver_in_progress =
      std::abs(ego.frenet.d) > config_.return_lateral_error_m ||
      last_mode_ == BehaviorMode::OVERTAKE_LEFT ||
      last_mode_ == BehaviorMode::OVERTAKE_RIGHT ||
      last_mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP ||
      last_mode_ == BehaviorMode::MERGE_BACK ||
      last_mode_ == BehaviorMode::ABORT_RECOVERY;
  if (target_ahead && !target_missing_recovery && !permission.allow_overtake &&
      !pass_maneuver_in_progress) {
    return follow_blocked_output("follow_blocked_overtake_permission");
  }

  const auto target_missing_yield_output = [&](const std::string &reason) {
    clearPassContinuationLatch();
    PlannerOutput yield = output;
    selected_.reset();
    yield.active = true;
    yield.safe_lateral = false;
    yield.intent = SolverHorizonIntent::NONE;
    yield.lateral_offsets_m.clear();
    yield.speed_caps_mps.clear();
    yield.longitudinal_offsets_m.clear();
    yield.candidate_speed_limit_mps = 0.0;
    yield.mode = BehaviorMode::YIELD_BEHIND;
    yield.speed_cap_mps = config_.safe_stop_speed_mps;
    yield.reason = reason;
    record_speed_transition(SpeedTransitionBranch::TARGET_MISSING_YIELD_DIRECT,
                            0.0, config_.safe_stop_speed_mps,
                            config_.safe_stop_speed_mps,
                            config_.safe_stop_speed_mps, true, true);
    safe_stop_latched_ = false;
    safe_stop_release_count_ = 0;
    speed_state_initialized_ = true;
    previous_command_speed_mps_ = config_.safe_stop_speed_mps;
    previous_acceleration_mps2_ = config_.min_acceleration_mps2;
    applyMpcHealthGuard(&yield, mpc_guard_reason);
    if (yield.mode != last_mode_) {
      last_mode_change_sec_ = now_sec;
    }
    last_mode_ = yield.mode;
    return finalize_output(std::move(yield));
  };

  const auto prepare_overtake_output = [&]() -> std::optional<PlannerOutput> {
    if (!target_ahead || overtake_target == nullptr || preparable.empty()) {
      return std::nullopt;
    }
    std::vector<CandidateTrajectory *> pass_candidates;
    std::copy_if(
        preparable.begin(), preparable.end(),
        std::back_inserter(pass_candidates),
        [this, overtake_target, &pass_capable_goal](const auto *candidate) {
          return !config_.safety_evaluation_enabled
                     ? std::abs(candidate->goal_d_m -
                                overtake_target->frenet.d) > 0.05
                     : pass_capable_goal(candidate);
        });
    std::sort(pass_candidates.begin(), pass_candidates.end(), normal_order);
    for (auto *candidate : pass_candidates) {
      std::vector<double> candidate_d;
      std::vector<double> candidate_speed;
      std::vector<double> candidate_longitudinal;
      OutputHorizonDiagnostic diagnostic;
      const double preparation_cap = std::clamp(
          std::min(candidate->entry_speed_limit_mps, blocked_speed_cap()),
          config_.safe_stop_speed_mps, config_.normal_speed_mps);
      if (!timed_output_horizon(*candidate, preparation_cap, &candidate_d,
                                &candidate_speed, &candidate_longitudinal,
                                &diagnostic)) {
        remember_output_horizon_failure(std::move(diagnostic), *candidate);
        continue;
      }
      PlannerOutput prepare = output;
      clearPassContinuationLatch();
      selected_.reset();
      prepare.active = true;
      prepare.safe_lateral = false;
      prepare.intent = SolverHorizonIntent::NONE;
      prepare.lateral_offsets_m.clear();
      prepare.speed_caps_mps.clear();
      prepare.longitudinal_offsets_m.clear();
      prepare.minimum_cost = candidate->total_cost;
      prepare.candidate_speed_limit_mps = candidate->entry_speed_limit_mps;
      prepare.speed_cap_mps = preparation_cap;
      prepare.mode = candidate->goal_d_m > 0.0
                         ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                         : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      prepare.reason = candidate->goal_d_m > 0.0 ? "prepare_left_entry_speed"
                                                 : "prepare_right_entry_speed";
      record_speed_transition(
          SpeedTransitionBranch::PREPARE_OVERTAKE_DIRECT, 0.0, preparation_cap,
          config_.safe_stop_speed_mps, config_.normal_speed_mps, false, false,
          static_cast<std::int32_t>(candidate->lateral_index),
          static_cast<std::int32_t>(candidate->tangent_index),
          candidate->total_cost);
      safe_stop_latched_ = false;
      safe_stop_release_count_ = 0;
      speed_state_initialized_ = true;
      previous_command_speed_mps_ = std::min(
          std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps)),
          preparation_cap);
      previous_acceleration_mps2_ = config_.min_acceleration_mps2;
      applyMpcHealthGuard(&prepare, mpc_guard_reason);
      if (prepare.mode != last_mode_) {
        last_mode_change_sec_ = now_sec;
      }
      last_mode_ = prepare.mode;
      return finalize_output(std::move(prepare));
    }
    return std::nullopt;
  };

  if (feasible.empty()) {
    if (const auto prepare = prepare_overtake_output(); prepare.has_value()) {
      return prepare.value();
    }
    if (target_missing_recovery) {
      return target_missing_yield_output(
          "target_missing_yield_no_recovery_path");
    }
    if (target_ahead && overtake_target != nullptr) {
      return follow_blocked_output("follow_blocked_no_feasible_pass_path");
    }
    selected_.reset();
    clearPassContinuationLatch();
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = target_missing_recovery ? "no_feasible_recovery_candidate"
                                            : "no_feasible_candidate";
    record_speed_transition(
        SpeedTransitionBranch::LATCH_ONLY_STOP, 0.0,
        config_.safe_stop_speed_mps, config_.safe_stop_speed_mps,
        config_.normal_speed_mps, true, target_missing_recovery);
    safe_stop_latched_ = true;
    return finalize_output(std::move(output));
  }

  std::vector<CandidateTrajectory *> selectable = feasible;
  if (!target_missing_recovery && overtake_target != nullptr &&
      (target_ahead || side_by_side_hold)) {
    selectable.clear();
    std::copy_if(
        feasible.begin(), feasible.end(), std::back_inserter(selectable),
        [this, overtake_target, &pass_capable_goal](const auto *candidate) {
          return !config_.safety_evaluation_enabled
                     ? std::abs(candidate->goal_d_m -
                                overtake_target->frenet.d) > 0.05
                     : pass_capable_goal(candidate);
        });
    if (selectable.empty()) {
      if (target_ahead) {
        if (const auto prepare = prepare_overtake_output();
            prepare.has_value()) {
          return prepare.value();
        }
        return follow_blocked_output("follow_blocked_no_pass_clearance");
      }
      selected_.reset();
      clearPassContinuationLatch();
      output.mode = BehaviorMode::SAFE_STOP;
      output.emergency_stop = true;
      output.speed_cap_mps = config_.safe_stop_speed_mps;
      output.reason = "side_by_side_path_lost";
      record_speed_transition(SpeedTransitionBranch::LATCH_ONLY_STOP, 0.0,
                              config_.safe_stop_speed_mps,
                              config_.safe_stop_speed_mps,
                              config_.normal_speed_mps, true, false);
      safe_stop_latched_ = true;
      return finalize_output(std::move(output));
    }
  }

  const bool parallel_opponent_present = !parallel_opponents.empty();
  const auto selection_order =
      [this, parallel_opponent_present, prioritize_pass_class,
       &is_sub_stop_cost, &is_pass_class, &same_maneuver_class,
       &normal_order](const auto *lhs, const auto *rhs) {
        if (is_sub_stop_cost(lhs) != is_sub_stop_cost(rhs)) {
          return is_sub_stop_cost(lhs);
        }
        if (prioritize_pass_class && is_pass_class(lhs) != is_pass_class(rhs)) {
          return is_pass_class(lhs);
        }
        if (parallel_opponent_present && same_maneuver_class(lhs, rhs) &&
            std::abs(lhs->opponent_clearance_recovery_m -
                     rhs->opponent_clearance_recovery_m) >
                config_.tie_break_epsilon) {
          return lhs->opponent_clearance_recovery_m >
                 rhs->opponent_clearance_recovery_m;
        }
        return normal_order(lhs, rhs);
      };
  std::sort(selectable.begin(), selectable.end(), selection_order);
  CandidateTrajectory *preferred = selectable.front();
  std::vector<CandidateTrajectory *> attempts = selectable;

  if (target_missing_recovery) {
    std::vector<CandidateTrajectory *> centerward;
    std::copy_if(feasible.begin(), feasible.end(),
                 std::back_inserter(centerward), [&ego](const auto *candidate) {
                   return std::abs(candidate->goal_d_m) <=
                          std::abs(ego.frenet.d) + 0.05;
                 });
    if (centerward.empty()) {
      centerward = feasible;
    }
    std::sort(centerward.begin(), centerward.end(),
              [this](const auto *lhs, const auto *rhs) {
                if (std::abs(lhs->goal_d_m) != std::abs(rhs->goal_d_m)) {
                  return std::abs(lhs->goal_d_m) < std::abs(rhs->goal_d_m);
                }
                if (lhs->total_cost != rhs->total_cost) {
                  return lhs->total_cost < rhs->total_cost;
                }
                if (std::abs(lhs->total_abs_steer_change -
                             rhs->total_abs_steer_change) >
                    config_.tie_break_epsilon) {
                  return lhs->total_abs_steer_change <
                         rhs->total_abs_steer_change;
                }
                return std::tie(lhs->lateral_index, lhs->tangent_index) <
                       std::tie(rhs->lateral_index, rhs->tangent_index);
              });
    attempts = centerward;
    preferred = attempts.front();
  } else {
    const auto previous = std::find_if(
        selectable.begin(), selectable.end(), [this](const auto *candidate) {
          return static_cast<int>(candidate->lateral_index) ==
                     previous_lateral_index_ &&
                 static_cast<int>(candidate->tangent_index) ==
                     previous_tangent_index_;
        });
    if (previous != selectable.end() &&
        is_sub_stop_cost(*previous) == is_sub_stop_cost(preferred) &&
        same_maneuver_class(*previous, preferred) &&
        (*previous)->total_cost <=
            preferred->total_cost + config_.candidate_cost_hysteresis) {
      preferred = *previous;
    }

    const bool holding_side =
        now_sec - last_mode_change_sec_ < config_.normal_mode_min_hold_sec &&
        (last_mode_ == BehaviorMode::OVERTAKE_LEFT ||
         last_mode_ == BehaviorMode::OVERTAKE_RIGHT ||
         last_mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP);
    if (holding_side) {
      const bool keep_left =
          last_mode_ == BehaviorMode::OVERTAKE_LEFT ||
          (last_mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP &&
           previous_lateral_index_ >= 0 && selected_.has_value() &&
           selected_->goal_d_m > 0.0);
      std::stable_partition(attempts.begin(), attempts.end(),
                            [keep_left](const auto *candidate) {
                              return keep_left ? candidate->goal_d_m > 0.05
                                               : candidate->goal_d_m < -0.05;
                            });
      const auto same_side = std::find_if(
          attempts.begin(), attempts.end(),
          [keep_left, preferred, &is_sub_stop_cost](const auto *candidate) {
            const bool requested_side = keep_left ? candidate->goal_d_m > 0.05
                                                  : candidate->goal_d_m < -0.05;
            const bool same_stop_class =
                is_sub_stop_cost(candidate) == is_sub_stop_cost(preferred);
            return requested_side && same_stop_class;
          });
      if (same_side != attempts.end()) {
        preferred = *same_side;
      }
    }

    // A history preference may decide between candidates in the same safety
    // cost class, but it must not hide a currently output-safe candidate below
    // the stop threshold behind a candidate that would immediately stop.
    const auto first_stop_class = std::stable_partition(
        attempts.begin(), attempts.end(), is_sub_stop_cost);
    const bool has_sub_stop_candidate = first_stop_class != attempts.begin();
    const auto preferred_it =
        std::find(attempts.begin(), attempts.end(), preferred);
    if (preferred_it != attempts.end() &&
        (!has_sub_stop_candidate || is_sub_stop_cost(preferred))) {
      std::rotate(attempts.begin(), preferred_it, preferred_it + 1);
    }
  }

  if (pass_continuation_latch_.has_value()) {
    const auto &latch = pass_continuation_latch_.value();
    const auto exact = std::find_if(
        attempts.begin(), attempts.end(),
        [this, &latch, &ego](const auto *candidate) {
          const double full_required_arc = requiredLateralTransitionDistance(
              config_, ego.speed_mps, ego.frenet.d, candidate->goal_d_m);
          return candidate->lateral_index == latch.lateral_index &&
                 candidate->tangent_index == latch.tangent_index &&
                 std::abs(candidate->goal_d_m - latch.goal_d_m) <=
                     config_.tie_break_epsilon &&
                 candidate->required_arc_m >=
                     full_required_arc - config_.tie_break_epsilon;
        });
    if (exact == attempts.end()) {
      clearPassContinuationLatch();
      if (side_by_side_hold) {
        output.mode = BehaviorMode::SAFE_STOP;
        output.intent = SolverHorizonIntent::NONE;
        output.emergency_stop = true;
        output.safe_lateral = false;
        output.speed_cap_mps = config_.safe_stop_speed_mps;
        output.reason = "side_by_side_continuation_lost";
        record_speed_transition(SpeedTransitionBranch::LATCH_ONLY_STOP, 0.0,
                                config_.safe_stop_speed_mps,
                                config_.safe_stop_speed_mps,
                                config_.normal_speed_mps, true, false);
        safe_stop_latched_ = true;
        return finalize_output(std::move(output));
      }
      if (target_ahead && overtake_target != nullptr) {
        return follow_blocked_output("pass_continuation_invalidated");
      }
    } else {
      preferred = *exact;
      attempts.assign(1U, *exact);
    }
  }

  struct CandidatePlan {
    CandidateTrajectory *candidate{nullptr};
    bool safe_stop_latched{false};
    int safe_stop_release_count{0};
    bool logical_stop{false};
    double requested_speed_mps{0.0};
    double command_speed_mps{0.0};
    double acceleration_mps2{0.0};
    bool exact_cartesian_execution{false};
    std::vector<double> lateral_offsets_m;
    std::vector<double> speed_caps_mps;
    std::vector<double> longitudinal_offsets_m;
  };

  const double base_command_speed =
      speed_state_initialized_
          ? previous_command_speed_mps_
          : std::max(config_.safe_stop_speed_mps, std::abs(ego.speed_mps));
  const double base_acceleration =
      speed_state_initialized_ ? previous_acceleration_mps2_ : 0.0;
  const auto plan_candidate =
      [&](CandidateTrajectory *candidate) -> std::optional<CandidatePlan> {
    CandidatePlan plan;
    plan.candidate = candidate;
    plan.safe_stop_latched = safe_stop_latched_;
    plan.safe_stop_release_count = safe_stop_release_count_;
    const bool fresh_cost_stop = config_.safety_evaluation_enabled &&
                                 candidate->safety_cost >= config_.stop_cost;
    if (fresh_cost_stop) {
      plan.safe_stop_latched = true;
      plan.safe_stop_release_count = 0;
    }
    const auto populate_plan = [&](CandidatePlan *candidate_plan,
                                   bool logical_stop, bool remember_failure) {
      candidate_plan->logical_stop = logical_stop;
      double desired_speed =
          logical_stop ? config_.safe_stop_speed_mps
                       : targetSpeedForCost(candidate->safety_cost, config_);
      if (target_missing_recovery) {
        desired_speed =
            std::min(desired_speed, config_.target_missing_recovery_speed_mps);
      }
      candidate_plan->requested_speed_mps = desired_speed;
      candidate_plan->command_speed_mps = desired_speed;
      if (!logical_stop) {
        const double dt = 1.0 / config_.planner_rate_hz;
        candidate_plan->acceleration_mps2 =
            jerkLimitedAcceleration(desired_speed, base_command_speed,
                                    base_acceleration, dt, false, config_);
        candidate_plan->command_speed_mps = std::clamp(
            base_command_speed + candidate_plan->acceleration_mps2 * dt,
            config_.safe_stop_speed_mps, config_.normal_speed_mps);
      } else {
        candidate_plan->acceleration_mps2 = std::clamp(
            (config_.safe_stop_speed_mps - base_command_speed) *
                config_.planner_rate_hz,
            config_.min_acceleration_mps2, config_.max_acceleration_mps2);
        candidate_plan->command_speed_mps = config_.safe_stop_speed_mps;
      }
      OutputHorizonDiagnostic diagnostic;
      candidate_plan->exact_cartesian_execution =
          config_.exact_cartesian_execution_enabled && !logical_stop;
      const bool valid_output = candidate_plan->exact_cartesian_execution
                                    ? validateExactCartesianHorizon(
                                          *candidate, planning_opponents,
                                          &diagnostic)
                                    : timed_output_horizon(
                                          *candidate,
                                          candidate_plan->command_speed_mps,
                                          &candidate_plan->lateral_offsets_m,
                                          &candidate_plan->speed_caps_mps,
                                          &candidate_plan
                                               ->longitudinal_offsets_m,
                                          &diagnostic);
      if (!valid_output) {
        if (remember_failure) {
          remember_output_horizon_failure(std::move(diagnostic), *candidate);
        }
        return false;
      }
      if (candidate_plan->exact_cartesian_execution) {
        double exact_arc_m = 0.0;
        for (std::size_t i = 1U; i < candidate->dense.size(); ++i) {
          exact_arc_m += std::hypot(candidate->dense[i].x -
                                        candidate->dense[i - 1U].x,
                                    candidate->dense[i].y -
                                        candidate->dense[i - 1U].y);
        }
        if (!std::isfinite(exact_arc_m) || exact_arc_m <= 1.0e-6) {
          return false;
        }
        // V4 remains the authority heartbeat. Its bounded zero-lateral
        // profile is never execution geometry when the exact kind is set;
        // PP selects the same-generation BindingStore Cartesian payload.
        candidate_plan->lateral_offsets_m = {0.0, 0.0};
        candidate_plan->speed_caps_mps = {
            candidate_plan->command_speed_mps,
            candidate_plan->command_speed_mps};
        candidate_plan->longitudinal_offsets_m = {0.0, exact_arc_m};
      }
      return true;
    };

    // A previous-cycle planner latch is not current physical evidence.
    // Release it only after the current below-threshold candidate has passed
    // the complete output contract at the actual bounded forward command. A
    // fresh threshold crossing never enters this path.
    if (config_.safety_evaluation_enabled && safe_stop_latched_ &&
        !fresh_cost_stop) {
      CandidatePlan release = plan;
      release.safe_stop_latched = false;
      release.safe_stop_release_count = 0;
      if (populate_plan(&release, false, false)) {
        return release;
      }
    }
    if (config_.safety_evaluation_enabled && plan.safe_stop_latched) {
      if (candidate->safety_cost <= config_.safe_stop_release_cost) {
        ++plan.safe_stop_release_count;
      } else {
        plan.safe_stop_release_count = 0;
      }
      if (plan.safe_stop_release_count >= config_.safe_stop_release_cycles) {
        plan.safe_stop_latched = false;
        plan.safe_stop_release_count = 0;
      }
    }
    plan.logical_stop =
        config_.safety_evaluation_enabled &&
        (plan.safe_stop_latched || candidate->safety_cost >= config_.stop_cost);
    if (!populate_plan(&plan, plan.logical_stop, true)) {
      return std::nullopt;
    }
    return plan;
  };

  std::optional<CandidatePlan> chosen_plan;
  for (auto *candidate : attempts) {
    chosen_plan = plan_candidate(candidate);
    if (chosen_plan.has_value()) {
      break;
    }
  }
  if (!chosen_plan.has_value()) {
    if (target_ahead && overtake_target != nullptr) {
      return follow_blocked_output("follow_blocked_output_contract");
    }
    if (target_missing_recovery) {
      return target_missing_yield_output(
          "target_missing_yield_output_contract");
    }
    selected_.reset();
    clearPassContinuationLatch();
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = "unsafe_output_resampling";
    output.output_horizon_diagnostic = first_output_horizon_failure;
    record_speed_transition(
        SpeedTransitionBranch::LATCH_ONLY_STOP, 0.0,
        config_.safe_stop_speed_mps, config_.safe_stop_speed_mps,
        config_.normal_speed_mps, true, target_missing_recovery);
    safe_stop_latched_ = true;
    return finalize_output(std::move(output));
  }

  CandidateTrajectory *choice = chosen_plan->candidate;
  selected_ = *choice;
  previous_lateral_index_ = static_cast<int>(choice->lateral_index);
  previous_tangent_index_ = static_cast<int>(choice->tangent_index);
  output.minimum_cost = choice->total_cost;
  output.candidate_speed_limit_mps = choice->dynamic_speed_limit_mps;
  output.lateral_offsets_m = std::move(chosen_plan->lateral_offsets_m);
  output.speed_caps_mps = std::move(chosen_plan->speed_caps_mps);
  output.longitudinal_offsets_m =
      std::move(chosen_plan->longitudinal_offsets_m);
  output.execution_geometry_kind =
      chosen_plan->exact_cartesian_execution
          ? ExecutionGeometryKind::EXACT_CARTESIAN
          : ExecutionGeometryKind::LEGACY_OFFSETS;
  output.spatial_profile_shadow_only =
      config_.experimental_exact_spatial_follow_shadow_enabled &&
      !output.longitudinal_offsets_m.empty();
  record_speed_transition(
      chosen_plan->logical_stop ? SpeedTransitionBranch::CANDIDATE_LOGICAL_STOP
                                : SpeedTransitionBranch::CANDIDATE_JERK_LIMITED,
      1.0 / config_.planner_rate_hz, chosen_plan->requested_speed_mps,
      config_.safe_stop_speed_mps, config_.normal_speed_mps,
      chosen_plan->logical_stop, target_missing_recovery,
      static_cast<std::int32_t>(choice->lateral_index),
      static_cast<std::int32_t>(choice->tangent_index), choice->total_cost);
  safe_stop_latched_ = chosen_plan->safe_stop_latched;
  safe_stop_release_count_ = chosen_plan->safe_stop_release_count;
  speed_state_initialized_ = true;
  previous_command_speed_mps_ = chosen_plan->command_speed_mps;
  previous_acceleration_mps2_ = chosen_plan->acceleration_mps2;
  const bool logical_stop = chosen_plan->logical_stop;
  output.safe_lateral = true;
  if (logical_stop) {
    output.mode = BehaviorMode::SAFE_STOP;
    output.intent = SolverHorizonIntent::NONE;
    std::fill(output.speed_caps_mps.begin(), output.speed_caps_mps.end(),
              config_.safe_stop_speed_mps);
    output.emergency_stop = true;
    output.reason = choice->safety_cost >= config_.stop_cost
                        ? "cost_stop_threshold"
                        : "cost_stop_latched";
  } else if (target_missing_recovery) {
    output.mode = BehaviorMode::ABORT_RECOVERY;
    output.intent = SolverHorizonIntent::MANEUVER_AUTHORIZED;
    output.reason = "target_missing_recovery";
  } else if (side_by_side_hold && std::abs(choice->goal_d_m) > 0.05) {
    output.mode = BehaviorMode::SIDE_BY_SIDE_KEEP;
    output.intent = SolverHorizonIntent::MANDATORY_AVOIDANCE;
    output.reason = "side_by_side_keep_pass_lane";
  } else if (choice->goal_d_m > 0.05) {
    output.mode = BehaviorMode::OVERTAKE_LEFT;
    output.intent = SolverHorizonIntent::MANDATORY_AVOIDANCE;
    output.reason = output.target_missing
                        ? "selected_left_candidate_predicted_target"
                        : "selected_left_candidate";
  } else if (choice->goal_d_m < -0.05) {
    output.mode = BehaviorMode::OVERTAKE_RIGHT;
    output.intent = SolverHorizonIntent::MANDATORY_AVOIDANCE;
    output.reason = output.target_missing
                        ? "selected_right_candidate_predicted_target"
                        : "selected_right_candidate";
  } else {
    output.mode = BehaviorMode::MERGE_BACK;
    output.intent = SolverHorizonIntent::MANEUVER_AUTHORIZED;
    output.reason = output.target_missing
                        ? "selected_return_candidate_predicted_target"
                        : "selected_return_candidate";
  }
  const double conservative_profile_cap_mps = *std::min_element(
      output.speed_caps_mps.begin(), output.speed_caps_mps.end());
  output.speed_cap_mps = conservative_profile_cap_mps;
  if (!logical_stop && output.spatial_profile_shadow_only) {
    const auto local_cap = recedingHorizonSpeedCap(
        output.speed_caps_mps, output.longitudinal_offsets_m);
    if (local_cap.has_value()) {
      output.speed_cap_mps =
          std::clamp(local_cap.value(), config_.safe_stop_speed_mps,
                     config_.normal_speed_mps);
    }
  }

  const bool actual_overtake_output =
      output.mode == BehaviorMode::OVERTAKE_LEFT ||
      output.mode == BehaviorMode::OVERTAKE_RIGHT;
  const bool pass_continuation_mode =
      actual_overtake_output || output.mode == BehaviorMode::SIDE_BY_SIDE_KEEP;
  const bool exact_continuation_output =
      pass_continuation_latch_.has_value() && live_target_present &&
      pass_continuation_latch_->target_id == target_id_ &&
      ((choice->goal_d_m > 0.0 && pass_continuation_latch_->side == 1) ||
       (choice->goal_d_m < 0.0 && pass_continuation_latch_->side == -1)) &&
      choice->lateral_index == pass_continuation_latch_->lateral_index &&
      choice->tangent_index == pass_continuation_latch_->tangent_index &&
      std::abs(choice->goal_d_m - pass_continuation_latch_->goal_d_m) <=
          config_.tie_break_epsilon;
  if (pass_continuation_mode && exact_continuation_output) {
    pass_continuation_latch_->target_observation_stamp_sec =
        live_target->stamp_sec;
    pass_continuation_latch_->required_arc_m = choice->required_arc_m;
    populatePassContinuationDiagnostic(&output, true);
  } else if (actual_overtake_output && !continuation_was_latched &&
             live_target_present && permission.allow_overtake &&
             choice->required_arc_m >=
                 requiredLateralTransitionDistance(
                     config_, ego.speed_mps, ego.frenet.d, choice->goal_d_m) -
                     config_.tie_break_epsilon) {
    pass_continuation_latch_ =
        PassContinuationLatch{target_id_,
                              choice->goal_d_m > 0.0 ? 1 : -1,
                              choice->lateral_index,
                              choice->tangent_index,
                              choice->goal_d_m,
                              choice->required_arc_m,
                              live_target->stamp_sec};
    populatePassContinuationDiagnostic(&output, false);
  } else if (!pass_continuation_mode) {
    clearPassContinuationLatch();
  }

  rear_path_ = generateRearSafetyPath(ego, 0.0);
  bool rear_hard_safe = false;
  output.rear_cost = rearCost(rear_path_, planning_opponents, &rear_hard_safe);
  output.rear_hard_safe = rear_hard_safe;

  bool return_ready = false;
  if (target_missing_recovery) {
    return_ready =
        choice->total_cost <= config_.free_run_return_cost &&
        std::abs(choice->goal_d_m) <= 0.05 && rear_hard_safe &&
        output.rear_cost <= config_.rear_return_cost_threshold &&
        std::abs(ego.frenet.d) <= config_.return_lateral_error_m &&
        std::abs(ego.frenet.yaw_error) <= config_.return_heading_error_rad &&
        !detector_.detected();
  } else if (overtake_target != nullptr && !output.target_missing) {
    const double target_to_ego =
        frame_->forwardDeltaS(overtake_target->frenet.s, ego.frenet.s);
    const double predicted_gap =
        target_to_ego + (std::abs(ego.speed_mps) - overtake_target->speed_mps) *
                            config_.return_prediction_sec;
    return_ready =
        choice->total_cost <= config_.free_run_return_cost &&
        target_to_ego >= config_.passed_target_gap_m &&
        target_to_ego < frame_->length() * 0.5 &&
        predicted_gap >= config_.return_predicted_gap_m &&
        std::abs(choice->goal_d_m) <= 0.05 && rear_hard_safe &&
        output.rear_cost <= config_.rear_return_cost_threshold &&
        std::abs(ego.frenet.d) <= config_.return_lateral_error_m &&
        std::abs(ego.frenet.yaw_error) <= config_.return_heading_error_rad &&
        !detector_.detected();
  }
  return_ready_count_ = return_ready ? return_ready_count_ + 1 : 0;
  output.return_ready_cycles = return_ready_count_;

  const BehaviorMode maneuver_mode = output.mode;
  if (return_ready_count_ >= config_.return_required_cycles) {
    const bool recovered_missing_target = target_missing_recovery;
    resetManeuverState();
    output = PlannerOutput{};
    output.reason = recovered_missing_target
                        ? "target_missing_recovery_complete"
                        : "return_complete";
  } else {
    if (maneuver_mode != last_mode_ &&
        maneuver_mode != BehaviorMode::SAFE_STOP) {
      last_mode_change_sec_ = now_sec;
    }
    last_mode_ = maneuver_mode;
  }

  applyMpcHealthGuard(&output, mpc_guard_reason);
  if (output.mode != BehaviorMode::OVERTAKE_LEFT &&
      output.mode != BehaviorMode::OVERTAKE_RIGHT &&
      output.mode != BehaviorMode::SIDE_BY_SIDE_KEEP) {
    clearPassContinuationLatch();
    output.pass_continuation_latched = false;
    output.pass_continuation_active = false;
    output.pass_continuation_target_id.clear();
    output.pass_continuation_side = 0;
    output.pass_continuation_lateral_index = -1;
    output.pass_continuation_tangent_index = -1;
    output.pass_continuation_goal_d_m =
        std::numeric_limits<double>::quiet_NaN();
    output.pass_continuation_required_arc_m =
        std::numeric_limits<double>::quiet_NaN();
    output.pass_continuation_target_observation_stamp_sec =
        std::numeric_limits<double>::quiet_NaN();
  }
  return finalize_output(std::move(output));
}

} // namespace state_lattice_overtake_planner
