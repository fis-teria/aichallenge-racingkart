#pragma once

#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner
{

class SafetyEvaluator
{
public:
  explicit SafetyEvaluator(PlannerConfig config);

  // 候補軌道が壁マージンと他車楕円マージンを満たすかを評価する。
  bool evaluate(
    CandidateTrajectory & candidate,
    const std::vector<PredictedOpponent> & predictions) const;

  // 自車姿勢を基準にした楕円CBF風の安全余裕。0以下に近いほど接近している。
  double ellipseMargin(
    double ego_x, double ego_y, double ego_yaw,
    double opp_x, double opp_y) const;

private:
  PlannerConfig config_;
};

}  // namespace overtake_planner
