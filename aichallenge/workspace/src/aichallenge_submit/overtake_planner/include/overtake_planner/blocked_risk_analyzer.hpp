#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner {

class BlockedRiskAnalyzer {
public:
  BlockedRiskAnalyzer(const FrenetFrame &frame, const PlannerConfig &config);

  BlockedInfo detectBlocked(const EgoState &ego,
                            const std::vector<OpponentState> &opponents,
                            double now_sec) const;

  BlockedInfo
  evaluatePassGap(const BlockedInfo &blocked_info,
                  const std::vector<OpponentState> &opponents,
                  const std::vector<PredictedOpponent> &predictions,
                  BehaviorMode mode) const;

  double wallClearance(double d) const;
  double opponentSDot(const OpponentState &opponent) const;

private:
  int sideRiskIndex(const BlockedInfo &info) const;
  int yieldTargetIndex(const BlockedInfo &info) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
};

} // namespace overtake_planner
