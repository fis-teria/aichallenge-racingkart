#pragma once

#include "overtake_planner/behavior_state_machine.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner
{

class OvertakePlannerCore
{
public:
  OvertakePlannerCore(FrenetFrame frame, PlannerConfig config);

  PlannerOutput update(
    double now_sec,
    const EgoState & ego,
    const std::vector<OpponentState> & opponents);

  BehaviorMode mode() const { return mode_; }

private:
  BlockedInfo detectBlocked(
    const EgoState & ego,
    const std::vector<OpponentState> & opponents,
    double now_sec) const;
  std::vector<PredictedOpponent> predictOpponents(
    const std::vector<OpponentState> & opponents,
    double now_sec) const;
  CandidateTrajectory makeCandidate(
    CandidateType type,
    const EgoState & ego,
    const BlockedInfo & blocked_info,
    const std::vector<OpponentState> & opponents) const;
  CandidateTrajectory selectCandidate(
    std::vector<CandidateTrajectory> & candidates) const;
  double candidateScore(
    const CandidateTrajectory & candidate,
    const BlockedInfo & blocked_info) const;

  FrenetFrame frame_;
  PlannerConfig config_;
  SafetyEvaluator safety_;
  BehaviorStateMachine state_machine_;
  BehaviorMode mode_{BehaviorMode::FREE_RUN};
  double overtake_start_sec_{0.0};
};

}  // namespace overtake_planner
