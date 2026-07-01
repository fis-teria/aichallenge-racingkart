#include "overtake_planner/behavior_state_machine.hpp"

#include <algorithm>

namespace overtake_planner
{

BehaviorStateMachine::BehaviorStateMachine(PlannerConfig config) : config_(config) {}

bool BehaviorStateMachine::canSwitch(double now_sec) const
{
  // 候補が一瞬だけ安全に見えた場合のチャタリングを防ぐ。
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
  // コアが選んだ候補を、そのまま使うのではなく運転状態として安定化する。
  BehaviorMode next = current;

  if (!selected_feasible) {
    // 選択候補が危険なら、前方閉塞中は追従、それ以外は中心線へ復帰する。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side && selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::ABORT_RECOVERY;
    }
    markIfChanged(now_sec, current, next);
    return next;
  }

  if (selected == CandidateType::PASS_LEFT || selected == CandidateType::PASS_RIGHT) {
    // PASS候補が連続して安全なときだけ追い越し準備へ進む。
    if (selected == CandidateType::PASS_LEFT) {
      ++pass_left_safe_cycles_;
      pass_right_safe_cycles_ = 0;
    } else {
      ++pass_right_safe_cycles_;
      pass_left_safe_cycles_ = 0;
    }
  } else {
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
  }

  switch (current) {
    case BehaviorMode::FREE_RUN:
      // 通常走行中に前方閉塞を検出したら、まず追従しつつPASS安全周期を貯める。
      if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.side_by_side && selected == CandidateType::SIDE_BY_SIDE_KEEP) {
        next = BehaviorMode::SIDE_BY_SIDE_KEEP;
      } else if (blocked_info.blocked) {
        const bool left_ready =
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles);
        const bool right_ready =
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles);
        if ((left_ready || right_ready) && canSwitch(now_sec)) {
          next = selected == CandidateType::PASS_LEFT ?
            BehaviorMode::PREPARE_OVERTAKE_LEFT : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
        } else {
          next = BehaviorMode::FOLLOW_BLOCKED;
        }
      }
      break;
    case BehaviorMode::FOLLOW_BLOCKED:
      // 追従中に閉塞が解けたら通常走行へ、十分安全なら追い越し準備へ移る。
      if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.side_by_side && selected == CandidateType::SIDE_BY_SIDE_KEEP) {
        next = BehaviorMode::SIDE_BY_SIDE_KEEP;
      } else if (!blocked_info.blocked) {
        next = BehaviorMode::FREE_RUN;
      } else {
        const bool left_ready =
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles);
        const bool right_ready =
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >= static_cast<int>(config_.pass_safe_required_cycles);
        if ((left_ready || right_ready) && canSwitch(now_sec)) {
          next = selected == CandidateType::PASS_LEFT ?
            BehaviorMode::PREPARE_OVERTAKE_LEFT : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
        }
      }
      break;
    case BehaviorMode::PREPARE_OVERTAKE_LEFT:
      // 準備モードは1周期だけ使い、次周期から実際の追い越しオフセットを維持する。
      next = selected == CandidateType::YIELD_BEHIND ?
        BehaviorMode::YIELD_BEHIND : BehaviorMode::OVERTAKE_LEFT;
      break;
    case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
      // 左右どちらに避けるかをデバッグ上でも分けて追跡する。
      next = selected == CandidateType::YIELD_BEHIND ?
        BehaviorMode::YIELD_BEHIND : BehaviorMode::OVERTAKE_RIGHT;
      break;
    case BehaviorMode::OVERTAKE_LEFT:
    case BehaviorMode::OVERTAKE_RIGHT:
      // 横並び中は追い越しを継続し、前方ギャップが戻ったら中心線へ戻る。
      if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.side_by_side) {
        next = current;
      } else if (blocked_info.front_delta_s > config_.merge_front_gap_m && !blocked_info.blocked) {
        next = BehaviorMode::MERGE_BACK;
      } else if (now_sec - mode_enter_time_sec_ > config_.abort_timeout_sec) {
        next = BehaviorMode::ABORT_RECOVERY;
      }
      break;
    case BehaviorMode::MERGE_BACK:
      // 中心線復帰後、まだ前が詰まっていれば追従、空いていれば通常走行へ戻る。
      if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.side_by_side) {
        next = selected == CandidateType::SIDE_BY_SIDE_KEEP ?
          BehaviorMode::SIDE_BY_SIDE_KEEP : BehaviorMode::FREE_RUN;
      } else {
        next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::FREE_RUN;
      }
      break;
    case BehaviorMode::ABORT_RECOVERY:
      // 中止復帰も中心線へ戻す処理なので、復帰後の状態はMERGE_BACKと同じ判定にする。
      if (blocked_info.ego_wall_clearance_m < config_.yield_rejoin_wall_clearance_m) {
        next = BehaviorMode::ABORT_RECOVERY;
      } else if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.side_by_side) {
        next = selected == CandidateType::SIDE_BY_SIDE_KEEP ?
          BehaviorMode::SIDE_BY_SIDE_KEEP : BehaviorMode::FREE_RUN;
      } else {
        next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::FREE_RUN;
      }
      break;
    case BehaviorMode::SIDE_BY_SIDE_KEEP:
      // 横並び中は相手から離れる距離維持overrideを出し続け、解けたら通常の閉塞判定へ戻る。
      if (selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (!blocked_info.side_by_side || selected != CandidateType::SIDE_BY_SIDE_KEEP) {
        next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED : BehaviorMode::FREE_RUN;
      }
      break;
    case BehaviorMode::YIELD_BEHIND:
      // 相手の後ろに入れる距離が戻ったら、通常の追従状態へ戻す。
      {
        const bool lateral_ready =
          blocked_info.ego_wall_clearance_m >= config_.yield_rejoin_wall_clearance_m;
        const double rejoin_gap = blocked_info.corner_side_by_side ?
          std::max(config_.yield_rejoin_gap_m, config_.corner_yield_rejoin_gap_m) :
          config_.yield_rejoin_gap_m;
        if (!lateral_ready) {
          next = BehaviorMode::YIELD_BEHIND;
        } else if (blocked_info.nearest_index >= 0 && blocked_info.front_delta_s >= rejoin_gap) {
          next = BehaviorMode::FOLLOW_BLOCKED;
        } else if (!blocked_info.blocked && !blocked_info.side_by_side) {
          next = BehaviorMode::FREE_RUN;
        }
      }
      break;
  }

  markIfChanged(now_sec, current, next);
  return next;
}

}  // namespace overtake_planner
