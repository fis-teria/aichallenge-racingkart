#include "overtake_planner/behavior_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner {

namespace {

// 入力: PlannerConfig。
// 出力: 高速カーブ復帰holdを解除する曲率しきい値[1/m]。
// 処理概要: 明示設定があればそれを使い、未設定時はコーナー譲り曲率の半分を安全側の既定にする。
double releaseCurvature(const PlannerConfig &config) {
  if (config.high_speed_curve_lateral_hold_release_curvature_m_inv >= 0.0) {
    return config.high_speed_curve_lateral_hold_release_curvature_m_inv;
  }
  return std::max(0.0, config.corner_side_yield_curvature_m_inv * 0.5);
}

// 入力: PlannerConfigと現在のBlockedInfo。
// 出力: 高速カーブ中の復帰holdを継続すべきならtrue。
// 処理概要: 高速かつ曲率が残っている間は、中心復帰の短周期切替を抑える。
bool shouldHoldHighSpeedCurveRecovery(const PlannerConfig &config,
                                      const BlockedInfo &blocked_info) {
  if (!config.high_speed_curve_lateral_hold_enabled) {
    return false;
  }
  const double release_speed =
      config.high_speed_curve_lateral_hold_release_speed_mps >= 0.0
          ? config.high_speed_curve_lateral_hold_release_speed_mps
          : config.high_speed_curve_lateral_hold_min_speed_mps;
  return blocked_info.ego_speed_mps > release_speed &&
         blocked_info.corner_abs_curvature > releaseCurvature(config);
}

} // namespace

// 入力: planner設定。
// 出力: 挙動モード状態機械のインスタンス。
// 処理概要: hold時間や解除条件の設定を保持し、周期ごとにmode遷移を判断できるようにする。
BehaviorStateMachine::BehaviorStateMachine(PlannerConfig config)
    : config_(config) {}

// 入力: 現在時刻[sec]。
// 出力: 最小モード保持時間を満たし、別modeへ切替可能ならtrue。
// 処理概要: 候補が一瞬だけ変わった時にmodeが細かく揺れないようにする。
bool BehaviorStateMachine::canSwitch(double now_sec) const {
  // 候補が一瞬だけ安全に見えた場合のチャタリングを防ぐ。
  return now_sec - mode_enter_time_sec_ >= config_.min_mode_hold_time_sec;
}

// 入力: 現在時刻、切替前mode、切替後mode。
// 出力: なし。modeが変わった場合のみ内部の入場時刻を更新する。
// 処理概要: canSwitch()の基準時刻を、実際にmodeが変わった周期だけ更新する。
void BehaviorStateMachine::markIfChanged(double now_sec, BehaviorMode before,
                                         BehaviorMode after) {
  if (before != after) {
    mode_enter_time_sec_ = now_sec;
  }
}

// 入力: 現在の横位置情報と解除しきい値[m]。
// 出力: 横ずれが解除可能な範囲ならtrue。しきい値が負/NaNなら常にtrue。
// 処理概要: YIELD/RECOVERY/SAFE_STOP解除時に、横位置が十分戻ったかを共通判定する。
bool BehaviorStateMachine::lateralReleaseReady(
    const BlockedInfo &blocked_info, double threshold_m) const {
  if (!std::isfinite(threshold_m) || threshold_m < 0.0) {
    return true;
  }
  return std::abs(blocked_info.ego_lateral_offset_m) <= threshold_m;
}

// 入力: 現在時刻とBlockedInfo。
// 出力: 未来コーナー譲り状態をまだ保持すべきならtrue。
// 処理概要: 最小保持時間、壁余裕、横位置、前方ギャップ、コーナー継続をまとめて見る。
bool BehaviorStateMachine::shouldHoldFutureYield(
    double now_sec, const BlockedInfo &blocked_info) const {
  if (!future_yield_hold_active_) {
    return false;
  }

  const bool lateral_ready = blocked_info.ego_wall_clearance_m >=
                                 config_.yield_rejoin_wall_clearance_m &&
                             lateralReleaseReady(
                                 blocked_info,
                                 config_.yield_release_lateral_error_m);
  const bool corner_still_relevant =
      blocked_info.corner_side_by_side ||
      blocked_info.future_corner_side_by_side ||
      (config_.corner_side_yield_curvature_m_inv > 0.0 &&
       blocked_info.corner_abs_curvature >=
           config_.corner_side_yield_curvature_m_inv);
  const double rejoin_gap =
      std::max(config_.yield_rejoin_gap_m, config_.corner_yield_rejoin_gap_m);
  const bool front_gap_ready = blocked_info.nearest_index < 0 ||
                               blocked_info.front_delta_s >= rejoin_gap;
  const bool minimum_hold_done = canSwitch(now_sec);

  return !minimum_hold_done || !lateral_ready || corner_still_relevant ||
         !front_gap_ready;
}

// 入力: 現在時刻、現在mode、選択候補、リスク情報、候補安全性、safe stop文脈。
// 出力: 次周期に使うBehaviorMode。
// 処理概要: 候補選択をそのままmodeにせず、保持時間・連続安全回数・停止解除条件で安定化する。
BehaviorMode BehaviorStateMachine::update(double now_sec, BehaviorMode current,
                                          CandidateType selected,
                                          const BlockedInfo &blocked_info,
                                          bool selected_feasible,
                                          SafeStopContext safe_stop_context) {
  // コアが選んだ候補を、そのまま使うのではなく運転状態として安定化する。
  BehaviorMode next = current;
  const int release_cycles_required =
      std::max(1, config_.safe_stop_release_cycles);
  if (selected == CandidateType::YIELD_BEHIND &&
      blocked_info.future_yield_required) {
    future_yield_hold_active_ = true;
  }

  // 処理ブロック: SAFE_STOP中の専用解除ロジック。
  // 設計意図: 停止系は通常の追い越し遷移より安全側なので、解除にも連続条件と復帰条件を要求する。
  if (current == BehaviorMode::SAFE_STOP) {
    // SAFE_STOP中は解除条件が連続で満たされるまで低速停止overrideを維持する。
    if (!safe_stop_context.candidate_feasible) {
      if (selected_feasible && selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (selected_feasible && blocked_info.side_by_side &&
                 selected == CandidateType::SIDE_BY_SIDE_KEEP) {
        next = BehaviorMode::SIDE_BY_SIDE_KEEP;
      } else {
        next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                    : BehaviorMode::ABORT_RECOVERY;
      }
      safe_stop_hold_count_ = 0;
      safe_stop_release_count_ = 0;
      pass_left_safe_cycles_ = 0;
      pass_right_safe_cycles_ = 0;
      markIfChanged(now_sec, current, next);
      if (next != BehaviorMode::YIELD_BEHIND &&
          next != BehaviorMode::SAFE_STOP) {
        future_yield_hold_active_ = false;
      }
      return next;
    }

    ++safe_stop_hold_count_;
    const bool recovery_required =
        !safe_stop_context.requested && !blocked_info.blocked &&
        !blocked_info.side_by_side && !blocked_info.future_yield_required &&
        (blocked_info.ego_wall_clearance_m <
             config_.safe_stop_release_wall_clearance_m ||
         !lateralReleaseReady(blocked_info,
                              config_.safe_stop_lateral_error_threshold_m));
    if (recovery_required && canSwitch(now_sec)) {
      next = BehaviorMode::ABORT_RECOVERY;
      safe_stop_hold_count_ = 0;
      safe_stop_release_count_ = 0;
      pass_left_safe_cycles_ = 0;
      pass_right_safe_cycles_ = 0;
      markIfChanged(now_sec, current, next);
      future_yield_hold_active_ = false;
      return next;
    }

    if (safe_stop_context.release_ready) {
      ++safe_stop_release_count_;
    } else {
      safe_stop_release_count_ = 0;
    }

    if (safe_stop_release_count_ >= release_cycles_required) {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::FREE_RUN;
      safe_stop_hold_count_ = 0;
      safe_stop_release_count_ = 0;
    } else {
      next = BehaviorMode::SAFE_STOP;
    }
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    markIfChanged(now_sec, current, next);
    if (next != BehaviorMode::YIELD_BEHIND && next != BehaviorMode::SAFE_STOP) {
      future_yield_hold_active_ = false;
    }
    return next;
  }

  safe_stop_hold_count_ = 0;
  safe_stop_release_count_ = 0;

  // 処理ブロック: 選択候補がfeasibleでない時のfallback。
  // 設計意図: 危険な候補をmode遷移に採用せず、前方閉塞なら追従、それ以外は復帰へ倒す。
  if (!selected_feasible) {
    // 選択候補が危険なら、前方閉塞中は追従、それ以外は中心線へ復帰する。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::ABORT_RECOVERY;
    }
    markIfChanged(now_sec, current, next);
    if (next != BehaviorMode::YIELD_BEHIND) {
      future_yield_hold_active_ = false;
    }
    return next;
  }

  // 処理ブロック: 回避不能判定が確定した時だけSAFE_STOPへ入る。
  // 設計意図: SAFE_STOPは強い介入なので、通常候補が無い文脈でのみ発火させる。
  if (selected == CandidateType::SAFE_STOP && safe_stop_context.requested) {
    // 回避不能判定が確定した時だけ、通常回避の外側にある低速停止へ入る。
    next = BehaviorMode::SAFE_STOP;
    safe_stop_hold_count_ = 1;
    safe_stop_release_count_ = 0;
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    markIfChanged(now_sec, current, next);
    return next;
  }

  const bool pass_start_allowed =
      blocked_info.straight_overtake_start_allowed;
  // 処理ブロック: PASS候補の連続安全回数を数える。
  // 設計意図: 一瞬だけ空いた隙間では追い越し準備へ入らず、同じ方向の安全判定が続いた時だけ開始する。
  if ((selected == CandidateType::PASS_LEFT ||
       selected == CandidateType::PASS_RIGHT) &&
      pass_start_allowed) {
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

  // 処理ブロック: 現在modeごとの状態遷移。
  // 設計意図: 候補のスコアリングと運転状態の保持条件を分離し、チャタリングを抑える。
  switch (current) {
  case BehaviorMode::FREE_RUN:
    // 通常走行中に前方閉塞を検出したら、まず追従しつつPASS安全周期を貯める。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (blocked_info.blocked) {
      const bool left_ready =
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      const bool right_ready =
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      if (pass_start_allowed && (left_ready || right_ready) &&
          canSwitch(now_sec)) {
        next = selected == CandidateType::PASS_LEFT
                   ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                   : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      } else {
        next = BehaviorMode::FOLLOW_BLOCKED;
      }
    }
    break;
  case BehaviorMode::FOLLOW_BLOCKED:
    // 追従中に閉塞が解けたら通常走行へ、十分安全なら追い越し準備へ移る。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (!blocked_info.blocked) {
      next = BehaviorMode::FREE_RUN;
    } else {
      const bool left_ready =
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      const bool right_ready =
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      if (pass_start_allowed && (left_ready || right_ready) &&
          canSwitch(now_sec)) {
        next = selected == CandidateType::PASS_LEFT
                   ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                   : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      }
    }
    break;
  case BehaviorMode::PREPARE_OVERTAKE_LEFT:
    // 準備モードは1周期だけ使い、次周期から実際の追い越しオフセットを維持する。
    next = selected == CandidateType::YIELD_BEHIND
               ? BehaviorMode::YIELD_BEHIND
               : BehaviorMode::OVERTAKE_LEFT;
    break;
  case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
    // 左右どちらに避けるかをデバッグ上でも分けて追跡する。
    next = selected == CandidateType::YIELD_BEHIND
               ? BehaviorMode::YIELD_BEHIND
               : BehaviorMode::OVERTAKE_RIGHT;
    break;
  case BehaviorMode::OVERTAKE_LEFT:
  case BehaviorMode::OVERTAKE_RIGHT:
    // 横並び中は追い越しを継続し、前方ギャップが戻ったら中心線へ戻る。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side) {
      next = current;
    } else if (blocked_info.front_delta_s > config_.merge_front_gap_m &&
               !blocked_info.blocked) {
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
      next = selected == CandidateType::SIDE_BY_SIDE_KEEP
                 ? BehaviorMode::SIDE_BY_SIDE_KEEP
                 : BehaviorMode::FREE_RUN;
    } else {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::ABORT_RECOVERY:
    // 中止復帰も中心線へ戻す処理なので、復帰後の状態はMERGE_BACKと同じ判定にする。
    if (blocked_info.ego_wall_clearance_m <
            config_.yield_rejoin_wall_clearance_m ||
        !lateralReleaseReady(blocked_info,
                             config_.recovery_release_lateral_error_m) ||
        shouldHoldHighSpeedCurveRecovery(config_, blocked_info)) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side) {
      next = selected == CandidateType::SIDE_BY_SIDE_KEEP
                 ? BehaviorMode::SIDE_BY_SIDE_KEEP
                 : BehaviorMode::FREE_RUN;
    } else {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::SIDE_BY_SIDE_KEEP:
    // 横並び中は相手から離れる距離維持overrideを出し続け、解けたら通常の閉塞判定へ戻る。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (!blocked_info.side_by_side ||
               selected != CandidateType::SIDE_BY_SIDE_KEEP) {
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::YIELD_BEHIND:
    // 相手の後ろに入れる距離が戻ったら、通常の追従状態へ戻す。
    {
      const bool hold_future_yield =
          shouldHoldFutureYield(now_sec, blocked_info);
      const bool lateral_ready = blocked_info.ego_wall_clearance_m >=
                                     config_.yield_rejoin_wall_clearance_m &&
                                 lateralReleaseReady(
                                     blocked_info,
                                     config_.yield_release_lateral_error_m);
      const double rejoin_gap =
          blocked_info.corner_side_by_side
              ? std::max(config_.yield_rejoin_gap_m,
                         config_.corner_yield_rejoin_gap_m)
              : config_.yield_rejoin_gap_m;
      if (hold_future_yield || !lateral_ready) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (blocked_info.nearest_index >= 0 &&
                 blocked_info.front_delta_s >= rejoin_gap) {
        next = BehaviorMode::FOLLOW_BLOCKED;
      } else if (!blocked_info.blocked && !blocked_info.side_by_side) {
        next = BehaviorMode::FREE_RUN;
      }
    }
    break;
  case BehaviorMode::SAFE_STOP:
    break;
  case BehaviorMode::SPEED_GUARD:
    next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                : BehaviorMode::FREE_RUN;
    break;
  }

  // 処理ブロック: 遷移後の内部保持状態を整理する。
  // 設計意図: YIELD/SAFE_STOP以外へ戻ったら、未来譲りholdを残して次の判断を縛らない。
  markIfChanged(now_sec, current, next);
  if (next != BehaviorMode::YIELD_BEHIND && next != BehaviorMode::SAFE_STOP) {
    future_yield_hold_active_ = false;
  }
  return next;
}

} // namespace overtake_planner
