#pragma once

#include "overtake_planner/types.hpp"

namespace overtake_planner {

struct PlannerOutputBuildInput {
  BehaviorMode mode;
  const EgoState &ego;
  const CandidateTrajectory &selected;
  const BlockedInfo &blocked_info;
  const SafeStopContext &safe_stop_context;
  const CandidateTrajectory &safe_stop_candidate;
  bool safe_stop_candidate_infeasible{false};
  int safe_stop_trigger_count{0};
  int safe_stop_hold_count{0};
  int safe_stop_release_count{0};
  double wall_soft_margin_m{0.0};
  const ActiveSectionSafety &active_section;
  const MpcHealthStatus &mpc_health;
};

class PlannerOutputBuilder {
public:
  explicit PlannerOutputBuilder(const PlannerConfig &config);

  PlannerOutput build(const PlannerOutputBuildInput &input) const;

private:
  double scaledSpeedCap(double speed_cap_mps,
                        const ActiveSectionSafety &section) const;

  const PlannerConfig &config_;
};

} // namespace overtake_planner
