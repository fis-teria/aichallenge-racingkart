#pragma once

#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner
{

class SafetyEvaluator
{
public:
  explicit SafetyEvaluator(PlannerConfig config);

  bool evaluate(
    CandidateTrajectory & candidate,
    const std::vector<PredictedOpponent> & predictions) const;

  double ellipseMargin(
    double ego_x, double ego_y, double ego_yaw,
    double opp_x, double opp_y) const;

private:
  PlannerConfig config_;
};

}  // namespace overtake_planner
