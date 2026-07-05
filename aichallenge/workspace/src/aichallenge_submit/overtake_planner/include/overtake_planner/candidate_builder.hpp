#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner {

class CandidateBuilder {
public:
  CandidateBuilder(const FrenetFrame &frame, const PlannerConfig &config);

  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents) const;

private:
  double wallClearance(double d) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
};

} // namespace overtake_planner
