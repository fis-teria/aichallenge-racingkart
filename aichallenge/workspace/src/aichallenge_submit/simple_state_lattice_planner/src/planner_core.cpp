#include "simple_state_lattice_planner/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace simple_state_lattice_planner {
namespace {

std::optional<double> projectS(const ReferenceWindow &reference, double x_m,
                               double y_m) {
  double best_distance_sq = std::numeric_limits<double>::infinity();
  std::optional<double> best_s;
  for (std::size_t index = 0U; index + 1U < reference.points.size(); ++index) {
    const auto &a = reference.points[index];
    const auto &b = reference.points[index + 1U];
    const double vx = b.x_m - a.x_m;
    const double vy = b.y_m - a.y_m;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= 1.0e-12) {
      continue;
    }
    const double ratio = std::clamp(
        ((x_m - a.x_m) * vx + (y_m - a.y_m) * vy) / length_sq, 0.0,
        1.0);
    const double px = a.x_m + ratio * vx;
    const double py = a.y_m + ratio * vy;
    const double distance_sq = (x_m - px) * (x_m - px) + (y_m - py) * (y_m - py);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_s = a.s_m + ratio * (b.s_m - a.s_m);
    }
  }
  return best_s;
}

const Candidate *bestCandidate(const std::vector<Candidate> &candidates,
                               CandidateSide side) {
  const Candidate *best = nullptr;
  for (const auto &candidate : candidates) {
    if (!candidate.valid || candidate.side != side) {
      continue;
    }
    if (best == nullptr || candidate.total_cost < best->total_cost - 1.0e-12 ||
        (std::abs(candidate.total_cost - best->total_cost) <= 1.0e-12 &&
         candidate.id < best->id)) {
      best = &candidate;
    }
  }
  return best;
}

PlannerOutput stopped(std::uint64_t snapshot_id, PlannerState state,
                      OutputReason reason, std::vector<Candidate> candidates = {}) {
  PlannerOutput output;
  output.snapshot_id = snapshot_id;
  output.state = state;
  output.reason = reason;
  output.candidates = std::move(candidates);
  return output;
}

}  // namespace

PlannerCore::PlannerCore(PlannerConfig config) : config_(std::move(config)) {}

PlannerOutput PlannerCore::plan(const PlannerInput &input,
                                double snapshot_time_sec) {
  if (input.snapshot_id == 0U || input.ego.snapshot_id != input.snapshot_id ||
      input.reference.snapshot_id != input.snapshot_id ||
      input.costmap.snapshot_id != input.snapshot_id || !input.costmap.valid() ||
      !std::isfinite(config_.pass_clearance_m) || config_.pass_clearance_m <= 0.0) {
    reset();
    return stopped(input.snapshot_id, state_, OutputReason::INPUT_INVALID);
  }
  if (input.opponent.has_value() &&
      (input.opponent->snapshot_id != input.snapshot_id ||
       !std::isfinite(input.opponent->stamp_sec) ||
       !std::isfinite(input.opponent->x_m) ||
       !std::isfinite(input.opponent->y_m) ||
       !std::isfinite(input.opponent->yaw_rad) ||
       !std::isfinite(input.opponent->speed_mps))) {
    reset();
    return stopped(input.snapshot_id, state_, OutputReason::INPUT_INVALID);
  }

  const auto ego_s = projectS(input.reference, input.ego.x_m, input.ego.y_m);
  std::optional<double> opponent_s;
  if (input.opponent.has_value()) {
    opponent_s = projectS(input.reference, input.opponent->x_m,
                          input.opponent->y_m);
  }
  if (!ego_s.has_value() || (input.opponent.has_value() && !opponent_s.has_value())) {
    reset();
    return stopped(input.snapshot_id, state_, OutputReason::INPUT_INVALID);
  }

  double merge_start_forward_m = config_.lattice.transition_distances_m[1U];
  if (opponent_s.has_value()) {
    merge_start_forward_m = std::max(
        merge_start_forward_m,
        *opponent_s - *ego_s + config_.pass_clearance_m);
  }
  auto generated = generateOvertakeLatticeCandidates(
      input.reference, input.ego, config_.lattice, snapshot_time_sec,
      std::max(0.0, merge_start_forward_m));
  if (!generated.valid()) {
    return stopped(input.snapshot_id, state_, OutputReason::INPUT_INVALID);
  }
  std::vector<Candidate> evaluated;
  evaluated.reserve(generated.candidates.size());
  for (auto candidate : generated.candidates) {
    evaluated.push_back(evaluateCandidate(std::move(candidate), input.costmap,
                                          config_.evaluator));
  }

  const Candidate *center = bestCandidate(evaluated, CandidateSide::CENTER);
  if (state_ == PlannerState::FREE_RUN) {
    if (center != nullptr) {
      const auto selected_id = center->id;
      const auto selected_points = center->points;
      PlannerOutput output;
      output.snapshot_id = input.snapshot_id;
      output.state = state_;
      output.candidates = std::move(evaluated);
      output.selected_id = selected_id;
      output.trajectory = selected_points;
      output.speed_limit_mps = config_.lattice.target_speed_mps;
      output.reason = OutputReason::CENTER_CLEAR;
      return output;
    }
    const auto center_it = std::find_if(
        evaluated.begin(), evaluated.end(),
        [](const Candidate &candidate) { return candidate.side == CandidateSide::CENTER; });
    if (center_it == evaluated.end() ||
        center_it->reject_reason != RejectReason::COSTMAP_OCCUPIED ||
        !input.opponent.has_value()) {
      return stopped(input.snapshot_id, state_, OutputReason::NO_VALID_CANDIDATE,
                     std::move(evaluated));
    }
    const Candidate *left = bestCandidate(evaluated, CandidateSide::LEFT);
    const Candidate *right = bestCandidate(evaluated, CandidateSide::RIGHT);
    const Candidate *selected = nullptr;
    if (left != nullptr && right != nullptr) {
      selected = left->total_cost <= right->total_cost ? left : right;
    } else {
      selected = left != nullptr ? left : right;
    }
    if (selected == nullptr) {
      return stopped(input.snapshot_id, state_, OutputReason::NO_VALID_CANDIDATE,
                     std::move(evaluated));
    }
    state_ = PlannerState::OVERTAKE;
    selected_side_ = selected->side;
    const auto selected_id = selected->id;
    const auto selected_points = selected->points;
    PlannerOutput output;
    output.snapshot_id = input.snapshot_id;
    output.state = state_;
    output.candidates = std::move(evaluated);
    output.selected_id = selected_id;
    output.trajectory = selected_points;
    output.speed_limit_mps = config_.lattice.target_speed_mps;
    output.reason = OutputReason::CENTER_BLOCKED_SIDE_SELECTED;
    return output;
  }

  if (!input.opponent.has_value() || !opponent_s.has_value()) {
    return stopped(input.snapshot_id, state_, OutputReason::OPPONENT_UNAVAILABLE,
                   std::move(evaluated));
  }
  if (*ego_s > *opponent_s + config_.pass_clearance_m && center != nullptr) {
    const auto selected_id = center->id;
    const auto selected_points = center->points;
    state_ = PlannerState::FREE_RUN;
    selected_side_.reset();
    PlannerOutput output;
    output.snapshot_id = input.snapshot_id;
    output.state = state_;
    output.candidates = std::move(evaluated);
    output.selected_id = selected_id;
    output.trajectory = selected_points;
    output.speed_limit_mps = config_.lattice.target_speed_mps;
    output.reason = OutputReason::OPPONENT_PASSED_CENTER_CLEAR;
    return output;
  }
  if (!selected_side_.has_value()) {
    return stopped(input.snapshot_id, state_, OutputReason::NO_VALID_CANDIDATE,
                   std::move(evaluated));
  }
  const Candidate *continuing = bestCandidate(evaluated, *selected_side_);
  if (continuing == nullptr) {
    return stopped(input.snapshot_id, state_, OutputReason::NO_VALID_CANDIDATE,
                   std::move(evaluated));
  }
  const auto selected_id = continuing->id;
  const auto selected_points = continuing->points;
  PlannerOutput output;
  output.snapshot_id = input.snapshot_id;
  output.state = state_;
  output.candidates = std::move(evaluated);
  output.selected_id = selected_id;
  output.trajectory = selected_points;
  output.speed_limit_mps = config_.lattice.target_speed_mps;
  output.reason = OutputReason::OVERTAKE_CONTINUING;
  return output;
}

void PlannerCore::reset() {
  state_ = PlannerState::FREE_RUN;
  selected_side_.reset();
}

}  // namespace simple_state_lattice_planner
