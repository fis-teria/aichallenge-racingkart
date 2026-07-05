#pragma once

#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner {

class FutureSideBySideRiskAnalyzer {
public:
  FutureSideBySideRiskAnalyzer(const FrenetFrame &frame,
                               const PlannerConfig &config,
                               const BlockedRiskAnalyzer &blocked_risk);

  BlockedInfo evaluate(const EgoState &ego, const BlockedInfo &blocked_info,
                       const std::vector<OpponentState> &opponents) const;

private:
  int sideRiskIndex(const BlockedInfo &info) const;
  double maxAbsCurvatureAhead(double s, double lookahead_m) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
  const BlockedRiskAnalyzer &blocked_risk_;
};

} // namespace overtake_planner
