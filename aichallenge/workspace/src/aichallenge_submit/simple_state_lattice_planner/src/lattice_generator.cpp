#include "simple_state_lattice_planner/lattice_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace simple_state_lattice_planner {
namespace {

constexpr double kEpsilon = 1.0e-9;
constexpr double kMaximumSampleSpacingM = 0.05;
constexpr double kMaximumHorizonM = 50.0;
constexpr std::size_t kHardMaximumSampleCount = 4096U;

double normalizeAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

bool finite(double value) { return std::isfinite(value); }

LatticeResult rejected(AdmissionFailure failure, const char *reason) {
  LatticeResult result;
  result.failure = failure;
  result.reason = reason;
  return result;
}

bool validConfig(const LatticeConfig &config) {
  if (config.frame_id != "map" || !finite(config.horizon_m) ||
      !finite(config.sample_spacing_m) || !finite(config.pass_offset_m) ||
      !finite(config.target_speed_mps) ||
      !finite(config.maximum_input_age_sec) ||
      !finite(config.maximum_future_offset_sec) ||
      !finite(config.maximum_input_skew_sec) || config.horizon_m <= 0.0 ||
      config.horizon_m > kMaximumHorizonM || config.sample_spacing_m <= 0.0 ||
      config.sample_spacing_m > kMaximumSampleSpacingM ||
      config.sample_spacing_m > config.horizon_m ||
      config.pass_offset_m <= 0.0 || config.target_speed_mps < 0.0 ||
      config.maximum_input_age_sec < 0.0 ||
      config.maximum_future_offset_sec < 0.0 ||
      config.maximum_input_skew_sec < 0.0 || config.maximum_sample_count < 2U ||
      config.maximum_sample_count > kHardMaximumSampleCount) {
    return false;
  }
  const double sample_count =
      std::ceil(config.horizon_m / config.sample_spacing_m);
  if (!finite(sample_count) ||
      sample_count > static_cast<double>(config.maximum_sample_count - 1U)) {
    return false;
  }
  return std::all_of(config.transition_distances_m.begin(),
                     config.transition_distances_m.end(),
                     [&config](double distance_m) {
                       return finite(distance_m) && distance_m > 0.0 &&
                              distance_m <= config.horizon_m;
                     });
}

bool validReference(const ReferenceWindow &reference) {
  if (reference.frame_id != "map" || !finite(reference.stamp_sec) ||
      reference.points.size() < 2U) {
    return false;
  }
  double previous_s_m = -std::numeric_limits<double>::infinity();
  for (const auto &point : reference.points) {
    if (!finite(point.x_m) || !finite(point.y_m) || !finite(point.yaw_rad) ||
        !finite(point.s_m) || point.s_m <= previous_s_m) {
      return false;
    }
    previous_s_m = point.s_m;
  }
  return reference.points.back().s_m - reference.points.front().s_m > kEpsilon;
}

struct Projection {
  bool valid{false};
  double s_m{0.0};
  double d_m{0.0};
};

Projection projectToReference(const ReferenceWindow &reference,
                              const EgoState &ego) {
  Projection best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < reference.points.size(); ++index) {
    const auto &from = reference.points[index];
    const auto &to = reference.points[index + 1U];
    const double vx = to.x_m - from.x_m;
    const double vy = to.y_m - from.y_m;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= kEpsilon) {
      continue;
    }
    const double ratio = std::clamp(
        ((ego.x_m - from.x_m) * vx + (ego.y_m - from.y_m) * vy) / length_sq,
        0.0, 1.0);
    const double projected_x_m = from.x_m + ratio * vx;
    const double projected_y_m = from.y_m + ratio * vy;
    const double dx_m = ego.x_m - projected_x_m;
    const double dy_m = ego.y_m - projected_y_m;
    const double distance_sq = dx_m * dx_m + dy_m * dy_m;
    if (distance_sq + kEpsilon >= best_distance_sq) {
      continue;
    }
    const double segment_length_m = std::sqrt(length_sq);
    best_distance_sq = distance_sq;
    best.valid = true;
    best.s_m = from.s_m + ratio * (to.s_m - from.s_m);
    best.d_m = (-vy * dx_m + vx * dy_m) / segment_length_m;
  }
  return best;
}

std::vector<double> splineSecondDerivatives(const ReferenceWindow &reference,
                                            bool use_x) {
  const std::size_t count = reference.points.size();
  std::vector<double> second(count, 0.0);
  std::vector<double> work(count, 0.0);
  for (std::size_t index = 1U; index + 1U < count; ++index) {
    const double previous_interval =
        reference.points[index].s_m - reference.points[index - 1U].s_m;
    const double next_interval =
        reference.points[index + 1U].s_m - reference.points[index].s_m;
    const double span = previous_interval + next_interval;
    const double sigma = previous_interval / span;
    const double pivot = sigma * second[index - 1U] + 2.0;
    second[index] = (sigma - 1.0) / pivot;
    const auto coordinate = [use_x](const ReferencePoint &point) {
      return use_x ? point.x_m : point.y_m;
    };
    const double slope_change = (coordinate(reference.points[index + 1U]) -
                                 coordinate(reference.points[index])) /
                                    next_interval -
                                (coordinate(reference.points[index]) -
                                 coordinate(reference.points[index - 1U])) /
                                    previous_interval;
    work[index] =
        (6.0 * slope_change / span - sigma * work[index - 1U]) / pivot;
  }
  for (std::size_t index = count - 1U; index-- > 0U;) {
    second[index] = second[index] * second[index + 1U] + work[index];
  }
  return second;
}

class ReferenceSpline {
public:
  explicit ReferenceSpline(const ReferenceWindow &reference)
      : reference_(reference),
        x_second_(splineSecondDerivatives(reference, true)),
        y_second_(splineSecondDerivatives(reference, false)) {}

  ReferencePoint interpolate(double s_m) const {
    const auto &reference = reference_;
    const double bounded_s_m = std::clamp(s_m, reference.points.front().s_m,
                                          reference.points.back().s_m);
    auto upper = std::upper_bound(
        reference.points.begin(), reference.points.end(), bounded_s_m,
        [](double value, const ReferencePoint &point) {
          return value < point.s_m;
        });
    if (upper == reference.points.begin()) {
      return reference.points.front();
    }
    if (upper == reference.points.end()) {
      return reference.points.back();
    }
    const std::size_t to_index =
        static_cast<std::size_t>(upper - reference.points.begin());
    const std::size_t from_index = to_index - 1U;
    const auto &to = reference.points[to_index];
    const auto &from = reference.points[from_index];
    const double interval_m = to.s_m - from.s_m;
    const double a = (to.s_m - bounded_s_m) / interval_m;
    const double b = (bounded_s_m - from.s_m) / interval_m;
    const auto value = [a, b, interval_m, from_index,
                        to_index](double from_value, double to_value,
                                  const std::vector<double> &second) {
      return a * from_value + b * to_value +
             ((a * a * a - a) * second[from_index] +
              (b * b * b - b) * second[to_index]) *
                 interval_m * interval_m / 6.0;
    };
    const auto derivative = [a, b, interval_m, from_index,
                             to_index](double from_value, double to_value,
                                       const std::vector<double> &second) {
      return (to_value - from_value) / interval_m +
             ((-3.0 * a * a + 1.0) * second[from_index] +
              (3.0 * b * b - 1.0) * second[to_index]) *
                 interval_m / 6.0;
    };
    ReferencePoint point;
    point.s_m = bounded_s_m;
    point.x_m = value(from.x_m, to.x_m, x_second_);
    point.y_m = value(from.y_m, to.y_m, y_second_);
    point.yaw_rad = std::atan2(derivative(from.y_m, to.y_m, y_second_),
                               derivative(from.x_m, to.x_m, x_second_));
    return point;
  }

private:
  const ReferenceWindow &reference_;
  std::vector<double> x_second_;
  std::vector<double> y_second_;
};

double quinticBlend(double ratio) {
  const double u = std::clamp(ratio, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double inverseQuinticBlend(double value) {
  double lower = 0.0;
  double upper = 1.0;
  for (int iteration = 0; iteration < 48; ++iteration) {
    const double midpoint = 0.5 * (lower + upper);
    if (quinticBlend(midpoint) < value) {
      lower = midpoint;
    } else {
      upper = midpoint;
    }
  }
  return 0.5 * (lower + upper);
}

Candidate buildCandidate(
    const ReferenceWindow &reference, const ReferenceSpline &reference_spline,
    const LatticeConfig &config, const Projection &projection, std::uint32_t id,
    CandidateSide side, double goal_d_m, double transition_distance_m,
    double merge_start_forward_m = std::numeric_limits<double>::infinity()) {
  Candidate candidate;
  candidate.id = id;
  candidate.snapshot_id = reference.snapshot_id;
  candidate.side = side;
  candidate.transition_distance_m = transition_distance_m;

  double remaining_transition_distance_m = transition_distance_m;
  double initial_shift_phase = 0.0;
  const double current_abs_d_m = std::abs(projection.d_m);
  const double goal_abs_d_m = std::abs(goal_d_m);
  const bool continuing_outbound_shift =
      goal_abs_d_m > kEpsilon && projection.d_m * goal_d_m > 0.0 &&
      current_abs_d_m > kEpsilon && current_abs_d_m < goal_abs_d_m;
  if (continuing_outbound_shift) {
    const double completed_lateral_fraction =
        std::clamp(current_abs_d_m / goal_abs_d_m, 0.0, 1.0);
    initial_shift_phase = inverseQuinticBlend(completed_lateral_fraction);
    remaining_transition_distance_m =
        transition_distance_m * (1.0 - initial_shift_phase);
  }
  candidate.remaining_transition_distance_m = remaining_transition_distance_m;

  const std::size_t sample_count = static_cast<std::size_t>(
      std::ceil(config.horizon_m / config.sample_spacing_m));
  candidate.points.reserve(sample_count + 1U);
  for (std::size_t index = 0U; index <= sample_count; ++index) {
    const double forward_m = std::min(
        config.horizon_m, config.sample_spacing_m * static_cast<double>(index));
    const auto base = reference_spline.interpolate(projection.s_m + forward_m);
    double d_m = projection.d_m;
    if (continuing_outbound_shift) {
      d_m = goal_d_m * quinticBlend(initial_shift_phase +
                                    forward_m / transition_distance_m);
    } else {
      const double shift_blend =
          quinticBlend(forward_m / transition_distance_m);
      d_m = projection.d_m + (goal_d_m - projection.d_m) * shift_blend;
    }
    if (finite(merge_start_forward_m) && forward_m > merge_start_forward_m) {
      const double merge_blend = quinticBlend(
          (forward_m - merge_start_forward_m) / transition_distance_m);
      d_m = goal_d_m * (1.0 - merge_blend);
    }

    TrajectoryPoint point;
    point.s_m = projection.s_m + forward_m;
    point.d_m = d_m;
    point.x_m = base.x_m - std::sin(base.yaw_rad) * d_m;
    point.y_m = base.y_m + std::cos(base.yaw_rad) * d_m;
    point.speed_mps = config.target_speed_mps;
    candidate.points.push_back(point);
  }

  for (std::size_t index = 0U; index < candidate.points.size(); ++index) {
    const std::size_t previous = index == 0U ? index : index - 1U;
    const std::size_t next =
        index + 1U < candidate.points.size() ? index + 1U : index;
    const double dx =
        candidate.points[next].x_m - candidate.points[previous].x_m;
    const double dy =
        candidate.points[next].y_m - candidate.points[previous].y_m;
    candidate.points[index].yaw_rad = std::atan2(dy, dx);
  }
  for (std::size_t index = 1U; index + 1U < candidate.points.size(); ++index) {
    const auto &previous = candidate.points[index - 1U];
    const auto &next = candidate.points[index + 1U];
    const double ds_m =
        std::hypot(next.x_m - previous.x_m, next.y_m - previous.y_m);
    if (ds_m > kEpsilon) {
      candidate.points[index].curvature_radpm =
          normalizeAngle(next.yaw_rad - previous.yaw_rad) / ds_m;
    }
  }
  if (candidate.points.size() > 1U) {
    candidate.points.front().curvature_radpm =
        candidate.points[1U].curvature_radpm;
    candidate.points.back().curvature_radpm =
        candidate.points[candidate.points.size() - 2U].curvature_radpm;
  }
  return candidate;
}

} // namespace

LatticeResult generateLatticeCandidates(const ReferenceWindow &reference,
                                        const EgoState &ego,
                                        const LatticeConfig &config,
                                        double snapshot_time_sec) {
  if (reference.frame_id != "map" || ego.frame_id != "map" ||
      config.frame_id != "map" || reference.frame_id != ego.frame_id ||
      reference.frame_id != config.frame_id) {
    return rejected(AdmissionFailure::INVALID_FRAME, "frame_must_be_map");
  }
  if (reference.snapshot_id == 0U || ego.snapshot_id == 0U ||
      reference.snapshot_id != ego.snapshot_id) {
    return rejected(AdmissionFailure::INVALID_TIMESTAMP,
                    "snapshot_identity_mismatch");
  }
  if (!finite(snapshot_time_sec) || !finite(reference.stamp_sec) ||
      !finite(ego.stamp_sec)) {
    return rejected(AdmissionFailure::INVALID_TIMESTAMP,
                    "timestamp_non_finite");
  }
  if (!finite(ego.x_m) || !finite(ego.y_m) || !finite(ego.yaw_rad) ||
      !finite(ego.speed_mps)) {
    return rejected(AdmissionFailure::NON_FINITE_INPUT, "ego_non_finite");
  }
  if (ego.speed_mps < 0.0) {
    return rejected(AdmissionFailure::REVERSE_SPEED,
                    "reverse_speed_unsupported");
  }
  if (!validConfig(config)) {
    return rejected(AdmissionFailure::INVALID_CONFIG, "config_invalid");
  }
  if (!validReference(reference)) {
    return rejected(AdmissionFailure::INVALID_REFERENCE, "reference_invalid");
  }
  const double oldest_stamp_sec = std::min(reference.stamp_sec, ego.stamp_sec);
  const double newest_stamp_sec = std::max(reference.stamp_sec, ego.stamp_sec);
  if (snapshot_time_sec - oldest_stamp_sec > config.maximum_input_age_sec) {
    return rejected(AdmissionFailure::STALE_INPUT, "input_stale");
  }
  if (newest_stamp_sec - snapshot_time_sec > config.maximum_future_offset_sec) {
    return rejected(AdmissionFailure::FUTURE_INPUT, "input_from_future");
  }
  if (newest_stamp_sec - oldest_stamp_sec > config.maximum_input_skew_sec) {
    return rejected(AdmissionFailure::INVALID_TIMESTAMP, "input_skew_exceeded");
  }

  const Projection projection = projectToReference(reference, ego);
  if (!projection.valid || projection.s_m + config.horizon_m >
                               reference.points.back().s_m + kEpsilon) {
    return rejected(AdmissionFailure::INVALID_REFERENCE,
                    "reference_horizon_unavailable");
  }

  LatticeResult result;
  result.snapshot_id = ego.snapshot_id;
  const ReferenceSpline reference_spline(reference);
  const double center_transition_m = config.transition_distances_m[1U];
  result.candidates.push_back(
      buildCandidate(reference, reference_spline, config, projection, 0U,
                     CandidateSide::CENTER, 0.0, center_transition_m));
  for (std::size_t index = 0U; index < config.transition_distances_m.size();
       ++index) {
    result.candidates.push_back(buildCandidate(
        reference, reference_spline, config, projection,
        static_cast<std::uint32_t>(index + 1U), CandidateSide::LEFT,
        config.pass_offset_m, config.transition_distances_m[index]));
  }
  for (std::size_t index = 0U; index < config.transition_distances_m.size();
       ++index) {
    result.candidates.push_back(buildCandidate(
        reference, reference_spline, config, projection,
        static_cast<std::uint32_t>(index + 4U), CandidateSide::RIGHT,
        -config.pass_offset_m, config.transition_distances_m[index]));
  }
  return result;
}

LatticeResult generateOvertakeLatticeCandidates(
    const ReferenceWindow &reference, const EgoState &ego,
    const LatticeConfig &config, double snapshot_time_sec,
    double merge_start_forward_m) {
  auto admission =
      generateLatticeCandidates(reference, ego, config, snapshot_time_sec);
  if (!admission.valid()) {
    return admission;
  }
  if (!finite(merge_start_forward_m) || merge_start_forward_m < 0.0) {
    return rejected(AdmissionFailure::INVALID_CONFIG,
                    "merge_start_forward_invalid");
  }
  const Projection projection = projectToReference(reference, ego);
  LatticeResult result;
  result.snapshot_id = ego.snapshot_id;
  const ReferenceSpline reference_spline(reference);
  result.candidates.push_back(buildCandidate(
      reference, reference_spline, config, projection, 0U,
      CandidateSide::CENTER, 0.0, config.transition_distances_m[1U]));
  for (std::size_t index = 0U; index < config.transition_distances_m.size();
       ++index) {
    const double transition_m = config.transition_distances_m[index];
    if (merge_start_forward_m + transition_m > config.horizon_m) {
      continue;
    }
    result.candidates.push_back(
        buildCandidate(reference, reference_spline, config, projection,
                       static_cast<std::uint32_t>(index + 1U),
                       CandidateSide::LEFT, config.pass_offset_m, transition_m,
                       std::max(merge_start_forward_m, transition_m)));
  }
  for (std::size_t index = 0U; index < config.transition_distances_m.size();
       ++index) {
    const double transition_m = config.transition_distances_m[index];
    if (merge_start_forward_m + transition_m > config.horizon_m) {
      continue;
    }
    result.candidates.push_back(buildCandidate(
        reference, reference_spline, config, projection,
        static_cast<std::uint32_t>(index + 4U), CandidateSide::RIGHT,
        -config.pass_offset_m, transition_m,
        std::max(merge_start_forward_m, transition_m)));
  }
  if (result.candidates.size() == 1U) {
    return rejected(AdmissionFailure::INVALID_CONFIG,
                    "merge_horizon_unavailable");
  }
  return result;
}

} // namespace simple_state_lattice_planner
