#include "overtake_planner/overtake_supervisor_v2.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

bool sameTrajectory(const CandidateTrajectory &lhs,
                    const CandidateTrajectory &rhs) {
  return lhs.type == rhs.type && lhs.feasible == rhs.feasible && lhs.d == rhs.d &&
         lhs.x == rhs.x && lhs.y == rhs.y && lhs.yaw == rhs.yaw &&
         lhs.v_ref == rhs.v_ref;
}

bool sameDecision(const SupervisorV2Decision &lhs,
                  const SupervisorV2Decision &rhs) {
  return lhs.phase == rhs.phase && lhs.attempt_id == rhs.attempt_id &&
         lhs.target_vehicle_id == rhs.target_vehicle_id &&
         lhs.pass_direction == rhs.pass_direction &&
         lhs.trajectory_authorized == rhs.trajectory_authorized &&
         lhs.lateral_maneuver_required == rhs.lateral_maneuver_required &&
         lhs.candidate_set_limited_by_legacy ==
             rhs.candidate_set_limited_by_legacy &&
         lhs.selected == rhs.selected && lhs.reason == rhs.reason &&
         sameTrajectory(lhs.trajectory, rhs.trajectory);
}

int passDirection(CandidateType type) {
  if (type == CandidateType::PASS_LEFT) {
    return 1;
  }
  if (type == CandidateType::PASS_RIGHT) {
    return -1;
  }
  return 0;
}

} // namespace

OvertakeSupervisorV2::OvertakeSupervisorV2(int abort_release_cycles)
    : abort_release_cycles_(std::max(1, abort_release_cycles)) {}

SupervisorV2Decision
OvertakeSupervisorV2::update(const SupervisorV2Input &input) {
  const bool complete_and_trackable =
      input.safety_inputs_complete && input.tracking_usable;

  if (phase_ == TacticalPhase::PASSING) {
    const CandidateTrajectory *same_side = findCandidate(input, pass_type_);
    const bool target_matches = input.target_present &&
                                input.target_vehicle_id == target_vehicle_id_;
    // permission/curve start gate is latched at PASS entry. Continuation uses
    // the same-side trajectory's current SafetyEvaluator result, freshness and
    // tracking health; a start-only gate closing must not cause an immediate
    // merge-back.
    if (complete_and_trackable && target_matches && same_side != nullptr &&
        same_side->feasible) {
      return makeDecision(input, TacticalPhase::PASSING, same_side,
                          "passing_same_generation_side");
    }
    phase_ = TacticalPhase::ABORT_HOLD;
    abort_clear_cycles_ = 0;
    return makeDecision(input, phase_, bestFallback(input),
                        "passing_safety_or_freshness_lost");
  }

  if (phase_ == TacticalPhase::ABORT_HOLD) {
    const CandidateTrajectory *hold = bestFallback(input);
    const bool release_cycle = input.abort_release_allowed &&
                               complete_and_trackable && hold != nullptr &&
                               hold->feasible;
    abort_clear_cycles_ = release_cycle ? abort_clear_cycles_ + 1 : 0;
    if (abort_clear_cycles_ < abort_release_cycles_) {
      return makeDecision(input, phase_, hold, "abort_hold_waiting_clear");
    }
    abort_clear_cycles_ = 0;
    target_vehicle_id_.clear();
    pass_type_ = CandidateType::FASTEST;
    phase_ = input.target_present ? TacticalPhase::ATTACK_FOLLOW
                                  : TacticalPhase::FREE_RUN;
    const auto *released =
        input.target_present ? findCandidate(input, CandidateType::FOLLOW)
                             : findCandidate(input, CandidateType::FASTEST);
    return makeDecision(input, phase_, released, "abort_hold_released");
  }

  const CandidateTrajectory *pass = bestPass(input);
  if (input.target_present && complete_and_trackable &&
      input.pass_start_allowed && pass != nullptr && pass->feasible) {
    phase_ = TacticalPhase::PASSING;
    target_vehicle_id_ = input.target_vehicle_id;
    pass_type_ = pass->type;
    ++attempt_id_;
    return makeDecision(input, phase_, pass, "gate2_pass_immediate");
  }

  if (input.target_present) {
    phase_ = TacticalPhase::ATTACK_FOLLOW;
    return makeDecision(input, phase_,
                        findCandidate(input, CandidateType::FOLLOW),
                        "pass_unavailable_attack_follow");
  }

  phase_ = TacticalPhase::FREE_RUN;
  target_vehicle_id_.clear();
  pass_type_ = CandidateType::FASTEST;
  return makeDecision(input, phase_, findCandidate(input, CandidateType::FASTEST),
                      "no_target_free_run");
}

const CandidateTrajectory *
OvertakeSupervisorV2::findCandidate(const SupervisorV2Input &input,
                                    CandidateType type) const {
  if (input.candidates == nullptr) {
    return nullptr;
  }
  const auto it = std::find_if(
      input.candidates->begin(), input.candidates->end(),
      [type](const CandidateTrajectory &candidate) {
        return candidate.type == type && candidate.feasible;
      });
  return it == input.candidates->end() ? nullptr : &*it;
}

const CandidateTrajectory *
OvertakeSupervisorV2::bestPass(const SupervisorV2Input &input) const {
  const auto *left = findCandidate(input, CandidateType::PASS_LEFT);
  const auto *right = findCandidate(input, CandidateType::PASS_RIGHT);
  if (left == nullptr) {
    return right;
  }
  if (right == nullptr) {
    return left;
  }
  return left->score <= right->score ? left : right;
}

const CandidateTrajectory *
OvertakeSupervisorV2::bestFallback(const SupervisorV2Input &input) const {
  if (input.abort_hold_candidate != nullptr &&
      input.abort_hold_candidate->feasible) {
    return input.abort_hold_candidate;
  }
  for (const auto type : {CandidateType::RECOVERY, CandidateType::SAFE_STOP,
                          CandidateType::YIELD_BEHIND,
                          CandidateType::FOLLOW}) {
    if (const auto *candidate = findCandidate(input, type)) {
      return candidate;
    }
  }
  return nullptr;
}

SupervisorV2Decision OvertakeSupervisorV2::makeDecision(
    const SupervisorV2Input &input, TacticalPhase phase,
    const CandidateTrajectory *candidate, const std::string &reason) {
  SupervisorV2Decision decision;
  decision.phase = phase;
  decision.attempt_id =
      phase == TacticalPhase::PASSING || phase == TacticalPhase::ABORT_HOLD
          ? attempt_id_
          : 0U;
  decision.target_vehicle_id = target_vehicle_id_.empty()
                                   ? input.target_vehicle_id
                                   : target_vehicle_id_;
  decision.pass_direction =
      phase == TacticalPhase::PASSING || phase == TacticalPhase::ABORT_HOLD
          ? passDirection(pass_type_)
          : 0;
  decision.trajectory_authorized = candidate != nullptr && candidate->feasible &&
                                   input.safety_inputs_complete &&
                                   input.tracking_usable;
  decision.lateral_maneuver_required = phase == TacticalPhase::PASSING ||
                                       phase == TacticalPhase::ABORT_HOLD;
  decision.candidate_set_limited_by_legacy =
      input.candidate_set_limited_by_legacy;
  decision.selected = candidate == nullptr ? CandidateType::FASTEST
                                           : candidate->type;
  if (candidate != nullptr) {
    decision.trajectory = *candidate;
  }
  decision.reason = reason;
  updateGeneration(decision);
  return decision;
}

void OvertakeSupervisorV2::updateGeneration(SupervisorV2Decision &decision) {
  if (!has_last_decision_ || !sameDecision(decision, last_decision_)) {
    plan_generation_ =
        plan_generation_ == std::numeric_limits<std::uint32_t>::max()
            ? 1U
            : plan_generation_ + 1U;
  }
  decision.plan_generation = plan_generation_;
  last_decision_ = decision;
  has_last_decision_ = true;
}

} // namespace overtake_planner
