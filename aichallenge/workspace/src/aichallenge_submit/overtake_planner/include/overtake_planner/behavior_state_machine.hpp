#pragma once

#include "overtake_planner/types.hpp"

namespace overtake_planner {

class BehaviorStateMachine {
public:
  explicit BehaviorStateMachine(PlannerConfig config);

  // 選ばれた候補とblocked状態から、追従・追い越し・復帰のモードを更新する。
  BehaviorMode update(double now_sec, BehaviorMode current,
                      CandidateType selected, const BlockedInfo &blocked_info,
                      bool selected_feasible,
                      SafeStopContext safe_stop_context = SafeStopContext{});

  double modeEnterTime() const { return mode_enter_time_sec_; }
  int safeStopHoldCount() const { return safe_stop_hold_count_; }
  int safeStopReleaseCount() const { return safe_stop_release_count_; }
  int passLeftSafeCycles() const { return pass_left_safe_cycles_; }
  int passRightSafeCycles() const { return pass_right_safe_cycles_; }
  const std::string &passSafeCycleResetReason() const {
    return pass_safe_cycle_reset_reason_;
  }
  bool futureYieldHoldActive() const { return future_yield_hold_active_; }

private:
  // モードが短時間で振動しないよう、最低保持時間を満たしたかを見る。
  bool canSwitch(double now_sec) const;
  void markIfChanged(double now_sec, BehaviorMode before, BehaviorMode after);
  bool shouldHoldFutureYield(double now_sec,
                             const BlockedInfo &blocked_info) const;
  bool lateralReleaseReady(const BlockedInfo &blocked_info,
                           double threshold_m) const;

  PlannerConfig config_;
  double mode_enter_time_sec_{0.0};
  int pass_left_safe_cycles_{0};
  int pass_right_safe_cycles_{0};
  std::string pass_safe_target_id_{};
  CandidateType pass_safe_candidate_type_{CandidateType::FASTEST};
  std::string pass_safe_cycle_reset_reason_{};
  int safe_stop_hold_count_{0};
  int safe_stop_release_count_{0};
  bool future_yield_hold_active_{false};
};

} // namespace overtake_planner
