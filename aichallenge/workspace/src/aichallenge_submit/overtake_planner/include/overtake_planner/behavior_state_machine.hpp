#pragma once

#include "overtake_planner/types.hpp"

namespace overtake_planner
{

class BehaviorStateMachine
{
public:
  explicit BehaviorStateMachine(PlannerConfig config);

  BehaviorMode update(
    double now_sec,
    BehaviorMode current,
    CandidateType selected,
    const BlockedInfo & blocked_info,
    bool selected_feasible);

  double modeEnterTime() const { return mode_enter_time_sec_; }

private:
  bool canSwitch(double now_sec) const;
  void markIfChanged(double now_sec, BehaviorMode before, BehaviorMode after);

  PlannerConfig config_;
  double mode_enter_time_sec_{0.0};
  int pass_safe_cycles_{0};
};

}  // namespace overtake_planner
