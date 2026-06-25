#include "overtake_planner/behavior_state_machine.hpp"

namespace overtake_planner
{

BehaviorStateMachine::BehaviorStateMachine(PlannerConfig config) : config_(config) {}

bool BehaviorStateMachine::canSwitch(double now_sec) const
{
  return now_sec - mode_enter_time_sec_ >= config_.min_mode_hold_time_sec;
}

void BehaviorStateMachine::markIfChanged(
  double now_sec, BehaviorMode before, BehaviorMode after)
{
  if (before != after) {
    mode_enter_time_sec_ = now_sec;
  }
}

BehaviorMode BehaviorStateMachine::update(
  double now_sec,
  BehaviorMode current,
  CandidateType selected,
  const BlockedInfo & blocked_info,
  bool selected_feasible)
{
  BehaviorMode next = current;

  if (!selected_feasible) {
    next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::ABORT_RECOVERY;
    markIfChanged(now_sec, current, next);
    return next;
  }

  if (selected == CandidateType::PASS_LEFT || selected == CandidateType::PASS_RIGHT) {
    ++pass_safe_cycles_;
  } else {
    pass_safe_cycles_ = 0;
  }

  switch (current) {
    case BehaviorMode::FREE_RUN:
      if (blocked_info.blocked) {
        if (pass_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles) && canSwitch(now_sec)) {
          next = selected == CandidateType::PASS_LEFT ?
            BehaviorMode::PREPARE_OVERTAKE_LEFT : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
        } else {
          next = BehaviorMode::FOLLOW_BLOCKED;
        }
      }
      break;
    case BehaviorMode::FOLLOW_BLOCKED:
      if (!blocked_info.blocked) {
        next = BehaviorMode::FREE_RUN;
      } else if (
        pass_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles) &&
        canSwitch(now_sec)) {
        next = selected == CandidateType::PASS_LEFT ?
          BehaviorMode::PREPARE_OVERTAKE_LEFT : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      }
      break;
    case BehaviorMode::PREPARE_OVERTAKE_LEFT:
      next = BehaviorMode::OVERTAKE_LEFT;
      break;
    case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
      next = BehaviorMode::OVERTAKE_RIGHT;
      break;
    case BehaviorMode::OVERTAKE_LEFT:
    case BehaviorMode::OVERTAKE_RIGHT:
      if (blocked_info.side_by_side) {
        next = current;
      } else if (blocked_info.front_delta_s > config_.merge_front_gap_m && !blocked_info.blocked) {
        next = BehaviorMode::MERGE_BACK;
      } else if (now_sec - mode_enter_time_sec_ > config_.abort_timeout_sec) {
        next = BehaviorMode::ABORT_RECOVERY;
      }
      break;
    case BehaviorMode::MERGE_BACK:
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::FREE_RUN;
      break;
    case BehaviorMode::ABORT_RECOVERY:
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::FREE_RUN;
      break;
  }

  markIfChanged(now_sec, current, next);
  return next;
}

}  // namespace overtake_planner
