#include "overtake_planner/behavior_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner {

namespace {

// 入力: CandidateType。
// 出力: 左右PASS候補ならtrue。
// 処理概要: SAFE_STOP解除中でも低速車列を抜ける候補だけを限定的に通す。
bool isPassCandidate(CandidateType selected) {
  return selected == CandidateType::PASS_LEFT ||
         selected == CandidateType::PASS_RIGHT;
}

// 入力: 現在modeと選択候補。
// 出力: 現在の追い越し方向と同じPASS候補ならtrue。
// 処理概要:
// 複数の低速/停止車両を連続して抜く間、timeoutだけで中止復帰へ落とさない。
bool selectedMatchesCurrentPass(BehaviorMode current, CandidateType selected) {
  return (current == BehaviorMode::OVERTAKE_LEFT &&
          selected == CandidateType::PASS_LEFT) ||
         (current == BehaviorMode::OVERTAKE_RIGHT &&
          selected == CandidateType::PASS_RIGHT);
}

// 入力: 現在modeと閉塞情報。
// 出力: infeasible候補を受けてもABORT_RECOVERYを使うべき実復帰文脈ならtrue。
// 処理概要: 通常走行の横ずれ・壁寄り・予見対象を、PASS中止と同じ強い状態へ
// 落とさない。ABORTは実PASS/mergeの中止だけに限定し、その他は速度guardまたは
// 既存の追従/譲りで安全を保つ。
bool isAbortRecoveryContext(BehaviorMode current,
                            const BlockedInfo &blocked_info) {
  return current == BehaviorMode::OVERTAKE_LEFT ||
         current == BehaviorMode::OVERTAKE_RIGHT ||
         current == BehaviorMode::MERGE_BACK ||
         current == BehaviorMode::ABORT_RECOVERY ||
         (current == BehaviorMode::SAFE_STOP &&
          blocked_info.reentry_hold_active);
}

// 入力: BlockedInfo。
// 出力: FOLLOW_BLOCKEDで車間形成すべき対象があるならtrue。
// 処理概要:
// 通常front blockedに加え、明示的に安全評価へ載せたparallel FOLLOW対象も
// FOLLOW状態の入力として扱う。
bool hasFollowBlockedTarget(const BlockedInfo &blocked_info) {
  const bool valid_braking_follow_target =
      blocked_info.braking_follow_active &&
      blocked_info.braking_follow_feasible &&
      !blocked_info.braking_follow_id.empty() &&
      blocked_info.braking_follow_index >= 0;
  // CoreがPASS不成立時にだけ全SafetyEvaluatorを通したwide parallel recheck。
  // candidate_pending/衝突・wall・trackability rejectをFOLLOW状態へ昇格させない。
  const bool verified_parallel_follow_recheck =
      blocked_info.parallel_follow_recheck_attempted &&
      blocked_info.parallel_follow_recheck_reason == "safety_evaluated_feasible" &&
      !blocked_info.parallel_side_id.empty() &&
      std::isfinite(blocked_info.parallel_side_delta_s) &&
      blocked_info.parallel_side_delta_s > 0.0 &&
      blocked_info.parallel_side_direction_known &&
      blocked_info.parallel_side_same_direction;
  return blocked_info.blocked || blocked_info.start_grid_target_active ||
         blocked_info.parallel_follow_candidate || valid_braking_follow_target ||
         verified_parallel_follow_recheck;
}

// PASS開始だけに使う対象。停止した前方parallel車をSafetyEvaluatorへ早期に載せるが、
// FOLLOW対象にはせず、PASSが不成立なら従来のfallbackへ戻す。
bool hasPassStartTarget(const BlockedInfo &blocked_info) {
  const bool committed_retry_target =
      blocked_info.maneuver_transaction_incomplete &&
      blocked_info.maneuver_target_latched &&
      blocked_info.maneuver_target_observed &&
      blocked_info.maneuver_target_fresh &&
      blocked_info.maneuver_chain_tail_observed;
  return blocked_info.blocked ||
         blocked_info.pass_start_target_continuity_active ||
         committed_retry_target || blocked_info.start_grid_target_active ||
         blocked_info.early_stationary_parallel_pass_target ||
         blocked_info.braking_follow_active;
}

// 入力: PASS開始判定に使うBlockedInfo。
// 出力: safe-cycleを束縛するauthoritative target ID。診断fixtureでIDを省略した
// 場合だけ空文字を返す。
// 処理概要: classifierが一周期落ちてもmaneuver latchを最優先し、別targetへ
// safe-cycleを引き継がない。
std::string passStartTargetId(const BlockedInfo &blocked_info) {
  if (blocked_info.maneuver_target_latched &&
      !blocked_info.maneuver_target_id.empty()) {
    return blocked_info.maneuver_target_id;
  }
  if (blocked_info.start_grid_target_active &&
      !blocked_info.start_grid_target_id.empty()) {
    return blocked_info.start_grid_target_id;
  }
  if (blocked_info.braking_follow_active &&
      !blocked_info.braking_follow_id.empty()) {
    return blocked_info.braking_follow_id;
  }
  if (blocked_info.early_stationary_parallel_pass_target &&
      !blocked_info.early_stationary_parallel_pass_id.empty()) {
    return blocked_info.early_stationary_parallel_pass_id;
  }
  return blocked_info.nearest_id;
}

} // namespace

// 入力: planner設定。
// 出力: 挙動モード状態機械のインスタンス。
// 処理概要:
// hold時間や解除条件の設定を保持し、周期ごとにmode遷移を判断できるようにする。
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
// 処理概要:
// YIELD/RECOVERY/SAFE_STOP解除時に、横位置が十分戻ったかを共通判定する。
bool BehaviorStateMachine::lateralReleaseReady(const BlockedInfo &blocked_info,
                                               double threshold_m) const {
  if (!std::isfinite(threshold_m) || threshold_m < 0.0) {
    return true;
  }
  return std::abs(blocked_info.ego_lateral_offset_m) <= threshold_m;
}

// 入力: 現在時刻とBlockedInfo。
// 出力: 未来コーナー譲り状態をまだ保持すべきならtrue。
// 処理概要:
// 最小保持時間、壁余裕、横位置、前方ギャップ、コーナー継続をまとめて見る。
bool BehaviorStateMachine::shouldHoldFutureYield(
    double now_sec, const BlockedInfo &blocked_info) const {
  if (!future_yield_hold_active_) {
    return false;
  }

  const bool lateral_ready =
      blocked_info.ego_wall_clearance_m >=
          config_.yield_rejoin_wall_clearance_m &&
      lateralReleaseReady(blocked_info, config_.yield_release_lateral_error_m);
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
// 処理概要:
// 候補選択をそのままmodeにせず、保持時間・連続安全回数・停止解除条件で安定化する。
BehaviorMode BehaviorStateMachine::update(double now_sec, BehaviorMode current,
                                          CandidateType selected,
                                          const BlockedInfo &blocked_info,
                                          bool selected_feasible,
                                          SafeStopContext safe_stop_context) {
  // コアが選んだ候補を、そのまま使うのではなく運転状態として安定化する。
  BehaviorMode next = current;
  pass_safe_cycle_reset_reason_ = "mode_not_pass_start";
  const int release_cycles_required =
      std::max(1, config_.safe_stop_release_cycles);
  if (selected == CandidateType::YIELD_BEHIND &&
      blocked_info.future_yield_required) {
    future_yield_hold_active_ = true;
  }

  // 処理ブロック: SAFE_STOP中の専用解除ロジック。
  // 設計意図:
  // 停止系は通常の追い越し遷移より安全側なので、解除にも連続条件と復帰条件を要求する。
  if (current == BehaviorMode::SAFE_STOP) {
    // SAFE_STOP中は解除条件が連続で満たされるまで低速停止overrideを維持する。
    if (!safe_stop_context.candidate_feasible) {
      if (selected_feasible && selected == CandidateType::YIELD_BEHIND) {
        next = BehaviorMode::YIELD_BEHIND;
      } else if (selected_feasible && blocked_info.side_by_side &&
                 selected == CandidateType::SIDE_BY_SIDE_KEEP) {
        next = BehaviorMode::SIDE_BY_SIDE_KEEP;
      } else {
        next = hasFollowBlockedTarget(blocked_info)
                   ? BehaviorMode::FOLLOW_BLOCKED
                   : BehaviorMode::SPEED_GUARD;
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
        !safe_stop_context.requested && !hasFollowBlockedTarget(blocked_info) &&
        !blocked_info.side_by_side && !blocked_info.future_yield_required &&
        (blocked_info.ego_wall_clearance_m <
             config_.safe_stop_release_wall_clearance_m ||
         !lateralReleaseReady(blocked_info,
                              config_.safe_stop_lateral_error_threshold_m));
    if (recovery_required && canSwitch(now_sec)) {
      // SAFE_STOP解除待ちの壁/横ずれは、実際に通常ラインへ横断する文脈が
      // Coreで確認されるまでは速度guardで保持する。
      next = BehaviorMode::SPEED_GUARD;
      safe_stop_hold_count_ = 0;
      safe_stop_release_count_ = 0;
      pass_left_safe_cycles_ = 0;
      pass_right_safe_cycles_ = 0;
      markIfChanged(now_sec, current, next);
      future_yield_hold_active_ = false;
      return next;
    }

    if (!safe_stop_context.requested &&
        blocked_info.slow_obstacle_chain_active && selected_feasible &&
        isPassCandidate(selected) && canSwitch(now_sec)) {
      next = BehaviorMode::FOLLOW_BLOCKED;
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
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
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

  const bool pending_tracking_release_without_usable_pass =
      blocked_info.maneuver_transaction_tracking_release_pending &&
      (selected != blocked_info.maneuver_transaction_pass_type ||
       !selected_feasible);
  const bool released_tracking_without_safe_hold =
      !blocked_info.maneuver_transaction_tracking_release_pending &&
      blocked_info.maneuver_transaction_incomplete &&
      !blocked_info.maneuver_transaction_safe_lateral_hold_active;
  const bool maneuver_tracking_stop_transaction =
      blocked_info.maneuver_transaction_incomplete ||
      (blocked_info.start_grid_target_active &&
       blocked_info.maneuver_transaction_prepared);
  if (maneuver_tracking_stop_transaction &&
      blocked_info.maneuver_transaction_tracking_stop_active &&
      (pending_tracking_release_without_usable_pass ||
       released_tracking_without_safe_hold)) {
    // 未完了PASSでPASS snapshotまたは現在d SAFE_STOP軌道が不成立でも、
    // 不成立PASSや中心向きYIELD/RECOVERYへ切り替えない。Core/authorityが横列を
    // 認可せず縦停止する間、FSMは同じtarget/sideを再評価できるFOLLOWに留める。
    next = BehaviorMode::FOLLOW_BLOCKED;
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    markIfChanged(now_sec, current, next);
    future_yield_hold_active_ = false;
    return next;
  }

  // 処理ブロック: 選択候補がfeasibleでない時のfallback。
  // 設計意図:
  // 危険な候補をmode遷移に採用せず、前方閉塞なら追従、それ以外は復帰へ倒す。
  if (!selected_feasible) {
    // 選択候補が危険なら、前方閉塞中は追従、それ以外は中心線へ復帰する。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (isAbortRecoveryContext(current, blocked_info)) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else {
      // FREE_RUN/FOLLOWのgenericな横ずれや壁寄りは、中心線への強制復帰ではなく
      // 既存の速度制限・横目標holdへ委譲する。前方閉塞があれば追従を優先する。
      next = blocked_info.blocked ? BehaviorMode::FOLLOW_BLOCKED
                                  : BehaviorMode::SPEED_GUARD;
    }
    markIfChanged(now_sec, current, next);
    // PASSの連続安全回数は、どの不成立周期でも必ず途切れる。
    // 早期returnの前にresetしないと、後の安全1周期を連続と誤認する。
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    if (next != BehaviorMode::YIELD_BEHIND) {
      future_yield_hold_active_ = false;
    }
    return next;
  }

  if (selected == CandidateType::SAFE_STOP &&
      blocked_info.maneuver_transaction_incomplete &&
      blocked_info.maneuver_transaction_safe_lateral_hold_active) {
    // 認可済みPASSの実行契約が失効した時は、ABORT中心復帰や
    // 低速FOLLOWへ進まず、同周期にSafetyEvaluatorを通った現在d停止を選ぶ。
    // modeはtarget/sideを再評価できるFOLLOW_BLOCKEDに保ち、実停止は
    // SafetyConstraintAuthorityの専用stop要求で保証する。
    next = BehaviorMode::FOLLOW_BLOCKED;
    safe_stop_hold_count_ = 0;
    safe_stop_release_count_ = 0;
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    markIfChanged(now_sec, current, next);
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

  const bool pass_start_allowed = blocked_info.straight_overtake_start_allowed;
  const bool gentle_curve_safe_hold_bypass =
      config_.gentle_curve_safe_pass_bypass_mode_hold_enabled &&
      blocked_info.gentle_curve_safe_pass_start_approved;
  // 処理ブロック: PASS候補の連続安全回数を数える。
  // 設計意図:
  // 一瞬だけ空いた隙間では追い越し準備へ入らず、同じ方向の安全判定が続いた時だけ開始する。
  const bool normal_pass_start_context =
      current == BehaviorMode::FREE_RUN ||
      current == BehaviorMode::FOLLOW_BLOCKED ||
      (current == BehaviorMode::SPEED_GUARD &&
       gentle_curve_safe_hold_bypass &&
       !blocked_info.maneuver_transaction_incomplete &&
       !blocked_info.reentry_hold_active &&
       !blocked_info.reentry_centering_authorized &&
       !blocked_info.post_abort_curve_hold_active &&
       !blocked_info.pass_reauthorization_lockout_active &&
       !blocked_info.pass_decision_frozen);
  const std::string pass_start_target_id = passStartTargetId(blocked_info);
  if (normal_pass_start_context &&
      (selected == CandidateType::PASS_LEFT ||
       selected == CandidateType::PASS_RIGHT) &&
      selected_feasible && pass_start_allowed) {
    const bool side_changed = pass_safe_candidate_type_ != selected;
    const bool target_changed =
        !pass_safe_target_id_.empty() && !pass_start_target_id.empty() &&
        pass_safe_target_id_ != pass_start_target_id;
    if (side_changed || target_changed ||
        blocked_info.pass_start_target_continuity_expired) {
      pass_left_safe_cycles_ = 0;
      pass_right_safe_cycles_ = 0;
      pass_safe_cycle_reset_reason_ =
          blocked_info.pass_start_target_continuity_expired
              ? "continuity_expired"
              : target_changed ? "target_changed" : "side_changed";
    } else {
      pass_safe_cycle_reset_reason_.clear();
    }
    pass_safe_target_id_ = pass_start_target_id;
    pass_safe_candidate_type_ = selected;
    // PASS候補が連続して安全なときだけ追い越し準備へ進む。
    if (selected == CandidateType::PASS_LEFT) {
      ++pass_left_safe_cycles_;
      pass_right_safe_cycles_ = 0;
    } else {
      ++pass_right_safe_cycles_;
      pass_left_safe_cycles_ = 0;
    }
  } else {
    if (!normal_pass_start_context) {
      pass_safe_cycle_reset_reason_ = "mode_not_pass_start";
    } else if (!pass_start_allowed) {
      pass_safe_cycle_reset_reason_ =
          blocked_info.overtake_start_gate_reason.empty()
              ? "start_gate_closed"
              : "start_gate_closed:" +
                    blocked_info.overtake_start_gate_reason;
    } else if (!selected_feasible) {
      pass_safe_cycle_reset_reason_ = "selected_candidate_infeasible";
    } else {
      pass_safe_cycle_reset_reason_ = "pass_candidate_not_selected";
    }
    pass_left_safe_cycles_ = 0;
    pass_right_safe_cycles_ = 0;
    pass_safe_target_id_.clear();
    pass_safe_candidate_type_ = CandidateType::FASTEST;
  }
  const bool prepared_start_grid_tracking_transaction =
      blocked_info.start_grid_target_active &&
      blocked_info.start_grid_target_confirmed_stationary &&
      blocked_info.maneuver_transaction_prepared &&
      !blocked_info.maneuver_transaction_incomplete &&
      blocked_info.maneuver_transaction_tracking_release_pending &&
      blocked_info.maneuver_target_latched &&
      blocked_info.maneuver_target_observed &&
      blocked_info.maneuver_target_fresh &&
      blocked_info.maneuver_chain_tail_observed && selected_feasible &&
      selected == blocked_info.maneuver_transaction_pass_type &&
      isPassCandidate(selected) && pass_start_allowed &&
      (blocked_info.overtake_permission_allowed ||
       blocked_info.permission_start_exception_active);

  // 処理ブロック: 現在modeごとの状態遷移。
  // 設計意図:
  // 候補のスコアリングと運転状態の保持条件を分離し、チャタリングを抑える。
  switch (current) {
  case BehaviorMode::FREE_RUN:
    // 通常走行中に前方閉塞を検出したら、まず追従しつつPASS安全周期を貯める。
    if (selected == CandidateType::YIELD_BEHIND) {
      // Gate 2未認可のstart-grid current-d holdは、初回周期にYIELDが最良でも
      // PASS後の中心復帰ではない。SafetyEvaluator済み候補をFOLLOW文脈で
      // 維持し、ABORT/reentry phaseを開始しない。
      next = blocked_info.start_grid_uncommitted_hold_active &&
                     selected_feasible
                 ? BehaviorMode::FOLLOW_BLOCKED
                 : BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY &&
               blocked_info.braking_follow_hold_lateral) {
      // 制動距離FOLLOWが不成立でも、同周期にSafetyEvaluatorを通った現d
      // hold RECOVERYなら追従状態を維持する。通常ラインへ横切らない。
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if (selected == CandidateType::RECOVERY) {
      // RECOVERY候補が選ばれても、PASSを中止した事実がない通常走行では
      // ABORTへ遷移しない。速度guard付きの横補正として扱う。
      next = BehaviorMode::SPEED_GUARD;
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (prepared_start_grid_tracking_transaction) {
      // 停止targetのGate 2承認済みPASSは、一般走行用の5周期debounceを
      // 再適用しない。停止constraint下のPREPAREへ入り、同generationの
      // controller proofが揃うまでは横軌道だけwarm-upする。
      next = selected == CandidateType::PASS_LEFT
                 ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                 : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
    } else if (hasFollowBlockedTarget(blocked_info) ||
               blocked_info.early_stationary_parallel_pass_target) {
      const bool left_ready =
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      const bool right_ready =
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      if (hasPassStartTarget(blocked_info) && pass_start_allowed &&
          (left_ready || right_ready) &&
          (canSwitch(now_sec) || gentle_curve_safe_hold_bypass)) {
        next = selected == CandidateType::PASS_LEFT
                   ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                   : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      } else if (hasFollowBlockedTarget(blocked_info)) {
        next = BehaviorMode::FOLLOW_BLOCKED;
      } else {
        // early stationary parallelはPASS開始の事前評価だけであり、PASSがまだ
        // 許可されない周期にFOLLOWへ遷移して通常の車間制御を起動しない。
        next = BehaviorMode::FREE_RUN;
      }
    }
    break;
  case BehaviorMode::FOLLOW_BLOCKED:
    // 追従中に閉塞が解けたら通常走行へ、十分安全なら追い越し準備へ移る。
    if (blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_target_latched &&
        !blocked_info.maneuver_target_observed) {
      // stale/missingになったラッチ対象を除外した別車FOLLOWで継続しない。
      // 予測入力から対象が欠けた時点で明示ABORTへ閉じる。
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (prepared_start_grid_tracking_transaction) {
      next = selected == CandidateType::PASS_LEFT
                 ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                 : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      // 同一PASS取引のfresh対象を見たまま、SafetyEvaluator済みの現d holdで
      // 一時減速できる時は、YIELDを中心復帰の開始として扱わない。通常の
      // ATTACK_FOLLOW補正を次周期も再評価し、target ID/pass sideを保持する。
      // hold不成立・stale・別対象では従来どおりYIELD/ABORTへ閉じる。
      next =
          (blocked_info.start_grid_uncommitted_hold_active &&
           selected_feasible) ||
                  (blocked_info.maneuver_transaction_incomplete &&
                  blocked_info.maneuver_transaction_safe_lateral_hold_active &&
                  selected_feasible)
              ? BehaviorMode::FOLLOW_BLOCKED
              : BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY) {
      // 対象が残る間だけ追従を継続する。未完了PASSは上の専用分岐または
      // maneuver_transaction_incomplete分岐でtarget ID/sideを保持する。
      // 対象も取引も無いgeneric RECOVERYをFOLLOWへ貼り付けると、前方が空いても
      // 低速RECOVERYを選び続けるため、復帰gateが必要ならABORTへ、不要なら
      // FREE_RUNへ戻す。Coreはgeneric復帰を後段でSPEED_GUARDへ分離する。
      if (blocked_info.maneuver_transaction_incomplete &&
          blocked_info.authorized_pass_current_d_hold_active &&
          blocked_info.maneuver_transaction_safe_lateral_hold_active &&
          selected_feasible) {
        // Gate 2認可済みのPASS側dをSafetyEvaluator済みRECOVERYで保持できる間は、
        // 対象が自車の真横〜直後へ移ってfront分類から外れてもABORTしない。
        // target/sideは未完了transactionへ残し、幾何完了または同側PASS再成立を待つ。
        next = BehaviorMode::FOLLOW_BLOCKED;
      } else if (hasFollowBlockedTarget(blocked_info)) {
        next = BehaviorMode::FOLLOW_BLOCKED;
      } else if (blocked_info.maneuver_transaction_incomplete) {
        next = BehaviorMode::ABORT_RECOVERY;
      } else if (blocked_info.reentry_hold_active ||
                 blocked_info.reentry_centering_authorized) {
        next = BehaviorMode::ABORT_RECOVERY;
      } else {
        next = BehaviorMode::FREE_RUN;
      }
    } else if (blocked_info.side_by_side &&
               selected == CandidateType::SIDE_BY_SIDE_KEEP) {
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (blocked_info.maneuver_transaction_incomplete) {
      const bool committed_retry_ready =
          blocked_info.maneuver_transaction_retry_active &&
          blocked_info.maneuver_target_latched &&
          blocked_info.maneuver_target_observed &&
          blocked_info.maneuver_target_fresh &&
          blocked_info.maneuver_chain_tail_observed && selected_feasible &&
          selected == blocked_info.maneuver_transaction_pass_type &&
          isPassCandidate(selected);
      const bool left_ready =
          blocked_info.maneuver_transaction_pass_type ==
              CandidateType::PASS_LEFT &&
          selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      const bool right_ready =
          blocked_info.maneuver_transaction_pass_type ==
              CandidateType::PASS_RIGHT &&
          selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles);
      if (committed_retry_ready) {
        // これは新規PASS開始ではなく、同じtarget ID/side/dを保持した認可済み
        // transactionの再開である。同周期のSafetyEvaluatorを通ったPASSを、
        // 新規開始用の曲率gate・5周期待ちで不成立FOLLOWへ差し替えない。
        // 初回開始は下のpass_start_allowed分岐に残し、再開でもtarget/chainの
        // fresh性と同周期のPASS feasibleは省略しない。
        next = selected == CandidateType::PASS_LEFT
                   ? BehaviorMode::OVERTAKE_LEFT
                   : BehaviorMode::OVERTAKE_RIGHT;
      } else if (hasPassStartTarget(blocked_info) && pass_start_allowed &&
                 (left_ready || right_ready) &&
                 (canSwitch(now_sec) || gentle_curve_safe_hold_bypass)) {
        next = left_ready ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                          : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      } else {
        // front/parallel分類が一周期消えても、幾何+安全完了前にFREE_RUNへ
        // 抜けない。Coreは同周期にPASS側FOLLOW/HOLDを再評価してpublishする。
        next = BehaviorMode::FOLLOW_BLOCKED;
      }
    } else if (!hasFollowBlockedTarget(blocked_info) &&
               !blocked_info.early_stationary_parallel_pass_target &&
               !blocked_info.pass_start_target_continuity_active) {
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
      if (hasPassStartTarget(blocked_info) && pass_start_allowed &&
          (left_ready || right_ready) &&
          (canSwitch(now_sec) || gentle_curve_safe_hold_bypass)) {
        next = selected == CandidateType::PASS_LEFT
                   ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                   : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
      }
    }
    break;
  case BehaviorMode::PREPARE_OVERTAKE_LEFT:
    // 準備モードは1周期だけ使う。ただし禁止区間の停止parallel例外で入った
    // PREPAREは、同一対象への例外承認が次周期も残る場合だけOVERTAKEへ進める。
    // 例外対象の消失・別ID化・stale入力では通常PASSへ昇格させない。
    if (blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_target_latched &&
        !blocked_info.maneuver_target_observed) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (prepared_start_grid_tracking_transaction &&
               selected == CandidateType::PASS_LEFT) {
      // PREPARE payloadとOVERTAKE payloadはwire上で同じmode IDへ正規化する。
      // stop下でそのgenerationを追従したproofが来るまではPREPAREを維持し、
      // 確認済みの同じ候補だけを実行へ昇格する。
      next = blocked_info.maneuver_transaction_tracking_release_confirmed
                 ? BehaviorMode::OVERTAKE_LEFT
                 : BehaviorMode::PREPARE_OVERTAKE_LEFT;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (!blocked_info.overtake_permission_allowed &&
               !blocked_info.permission_start_exception_active) {
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    } else if (selected != CandidateType::PASS_LEFT || !selected_feasible ||
               !pass_start_allowed) {
      // PREPAREへ入った後も、実行直前の周期で同じ方向のPASS候補が
      // SafetyEvaluatorを通過して選ばれていることを要求する。FOLLOWなどへ
      // 優先候補が変わった状態でOVERTAKEだけ確定させると、横軌道を一度も
      // publishしないまま次周期のABORTへ落ちるため、横移動前の追従へ戻す。
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    } else {
      next = BehaviorMode::OVERTAKE_LEFT;
    }
    break;
  case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
    // 左右どちらに避けるかをデバッグ上でも分けて追跡する。
    if (blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_target_latched &&
        !blocked_info.maneuver_target_observed) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (prepared_start_grid_tracking_transaction &&
               selected == CandidateType::PASS_RIGHT) {
      next = blocked_info.maneuver_transaction_tracking_release_confirmed
                 ? BehaviorMode::OVERTAKE_RIGHT
                 : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (!blocked_info.overtake_permission_allowed &&
               !blocked_info.permission_start_exception_active) {
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    } else if (selected != CandidateType::PASS_RIGHT || !selected_feasible ||
               !pass_start_allowed) {
      // 左側と同じく、方向まで一致する評価済みPASSだけを実行へ昇格する。
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    } else {
      next = BehaviorMode::OVERTAKE_RIGHT;
    }
    break;
  case BehaviorMode::OVERTAKE_LEFT:
  case BehaviorMode::OVERTAKE_RIGHT:
    // 横並び中は追い越しを継続し、前方ギャップが戻ったら中心線へ戻る。
    if (blocked_info.maneuver_target_latched &&
        !blocked_info.maneuver_target_observed) {
      // ラッチ対象が規定freshnessを超えて消えた時だけ、明示的な安全理由で
      // ABORTする。別車がnearestになっただけならobservedはtrueのまま。
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (selected == CandidateType::FOLLOW && selected_feasible) {
      // 同側PASSのSafetyEvaluatorが不成立でも、現在dを保つFOLLOWが安全なら
      // 直ちに中心復帰へ向かわず、攻め追従へ戻る。FOLLOW候補はCoreで全horizon
      // 評価済みなので、単なるPASS失敗だけでABORT/停止へ落とさない。
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if ((selected == CandidateType::RECOVERY ||
                selected == CandidateType::YIELD_BEHIND) &&
               selected_feasible &&
               blocked_info.maneuver_transaction_incomplete &&
               blocked_info.maneuver_transaction_safe_lateral_hold_active) {
      // 同方向PASSが不成立でも、freshな同一targetを含む予測とSafetyEvaluatorで
      // 現在d保持を別途認可できた時だけ攻めFOLLOWへ戻す。target ID・PASS side・
      // target dはtransactionに残し、次周期もPASS再開を評価する。
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if (!blocked_info.overtake_permission_allowed &&
               blocked_info.stationary_no_pass_safe_pass_constraint_active &&
               !blocked_info.stationary_no_pass_safe_pass_start_approved) {
      // 禁止区間の停止障害物例外は毎周期Gate 2・freshness・同一対象を
      // 再承認する。失効したPASSを同側候補だけで継続せず、同周期にPASSを
      // 除外済みのYIELD、または安全評価済みRECOVERYへ閉じる。
      next = selected == CandidateType::YIELD_BEHIND
                 ? BehaviorMode::YIELD_BEHIND
                 : BehaviorMode::ABORT_RECOVERY;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side) {
      next = current;
    } else if (blocked_info.maneuver_target_pass_complete) {
      const bool next_target_waiting =
          blocked_info.blocked && !blocked_info.nearest_id.empty() &&
          blocked_info.nearest_id != blocked_info.maneuver_target_id;
      // relative s・relative speed・SafetyEvaluator余裕で前対象を抜き切った。
      // 次対象が既に前方にいる時は同じ側を保ち、次周期にだけtargetを張り替える。
      // それ以外は初めてMERGE_BACKへ進む。
      next =
          next_target_waiting && selectedMatchesCurrentPass(current, selected)
              ? current
              : BehaviorMode::MERGE_BACK;
    } else if (selectedMatchesCurrentPass(current, selected)) {
      // PASS候補が同じ方向でまだ安全なら、停止車列や長い回避区間の途中で
      // abort_timeout_secだけを理由にABORT_RECOVERYへ落とさない。
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
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::ABORT_RECOVERY:
    // 中止復帰も中心線へ戻す処理なので、復帰後の状態はMERGE_BACKと同じ判定にする。
    if (selected == CandidateType::FOLLOW && selected_feasible) {
      // ABORTへ入った後も、中心復帰より安全な現d保持FOLLOWが成立したなら
      // その候補へhandoffする。横復帰のlateral release条件を先に要求すると、
      // 追越不能を検知した車両が停止したままになる。
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if ((selected == CandidateType::RECOVERY ||
                selected == CandidateType::YIELD_BEHIND) &&
               selected_feasible &&
               blocked_info.maneuver_transaction_incomplete &&
               blocked_info.maneuver_transaction_safe_lateral_hold_active) {
      // 同一target/sideの未完了PASSで、最新予測を通した現d holdが成立した時は
      // 中心復帰を継続せず攻めFOLLOWへ戻す。PASS reject候補は使わず、Coreが
      // 独立評価したholdだけをhandoffする。
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if (blocked_info.maneuver_transaction_incomplete) {
      // 抜き切る前のtarget ID/sideを保持したままFREE_RUNへ抜けない。
      // FOLLOW/holdが成立しない周期はABORTでfail-closedを維持する。
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (blocked_info.ego_wall_clearance_m <
                   config_.yield_rejoin_wall_clearance_m ||
               !lateralReleaseReady(blocked_info,
                                    config_.recovery_release_lateral_error_m)) {
      next = BehaviorMode::ABORT_RECOVERY;
    } else if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (blocked_info.side_by_side) {
      next = selected == CandidateType::SIDE_BY_SIDE_KEEP
                 ? BehaviorMode::SIDE_BY_SIDE_KEEP
                 : BehaviorMode::FREE_RUN;
    } else {
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::SIDE_BY_SIDE_KEEP:
    // 横並び中は相手から離れる距離維持overrideを出し続け、解けたら通常の閉塞判定へ戻る。
    if (selected == CandidateType::YIELD_BEHIND) {
      next = BehaviorMode::YIELD_BEHIND;
    } else if (selected == CandidateType::RECOVERY) {
      // wide parallel観測だけで中心線へ横切らない。横並び保持を続ける。
      next = BehaviorMode::SIDE_BY_SIDE_KEEP;
    } else if (!blocked_info.side_by_side ||
               selected != CandidateType::SIDE_BY_SIDE_KEEP) {
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    }
    break;
  case BehaviorMode::YIELD_BEHIND:
    // 相手の後ろに入れる距離が戻ったら、通常の追従状態へ戻す。
    {
      const bool hold_future_yield =
          shouldHoldFutureYield(now_sec, blocked_info);
      const bool lateral_ready =
          blocked_info.ego_wall_clearance_m >=
              config_.yield_rejoin_wall_clearance_m &&
          lateralReleaseReady(blocked_info,
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
      } else if (blocked_info.parallel_follow_candidate) {
        next = BehaviorMode::FOLLOW_BLOCKED;
      } else if (!blocked_info.blocked && !blocked_info.side_by_side) {
        next = BehaviorMode::FREE_RUN;
      }
    }
    break;
  case BehaviorMode::SAFE_STOP:
    break;
  case BehaviorMode::SPEED_GUARD:
    // generic lateral/wall/health guardが続く間は状態も維持し、FREE_RUNと
    // SPEED_GUARDの1周期振動を避ける。
    if (gentle_curve_safe_hold_bypass && selected_feasible &&
        pass_start_allowed && hasPassStartTarget(blocked_info) &&
        ((selected == CandidateType::PASS_LEFT &&
          pass_left_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles)) ||
         (selected == CandidateType::PASS_RIGHT &&
          pass_right_safe_cycles_ >=
              static_cast<int>(config_.pass_safe_required_cycles)))) {
      // 実reentryではなく、同周期のSafetyEvaluator承認済みgentle PASSだけが
      // SPEED_GUARDへ残った場合はsafe-cycleを捨てずPREPAREへ進める。
      next = selected == CandidateType::PASS_LEFT
                 ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                 : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
    } else if (selected == CandidateType::FOLLOW && selected_feasible &&
        blocked_info.maneuver_transaction_incomplete) {
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if ((selected == CandidateType::RECOVERY ||
                selected == CandidateType::YIELD_BEHIND) &&
               selected_feasible &&
               blocked_info.maneuver_transaction_incomplete &&
               blocked_info.maneuver_transaction_safe_lateral_hold_active) {
      next = BehaviorMode::FOLLOW_BLOCKED;
    } else if (blocked_info.maneuver_transaction_incomplete) {
      next = BehaviorMode::SPEED_GUARD;
    } else if (blocked_info.pass_decision_frozen ||
               blocked_info.post_abort_curve_hold_active) {
      next = BehaviorMode::SPEED_GUARD;
    } else {
      next = hasFollowBlockedTarget(blocked_info) ? BehaviorMode::FOLLOW_BLOCKED
                                                  : BehaviorMode::FREE_RUN;
    }
    break;
  }

  // 処理ブロック: 遷移後の内部保持状態を整理する。
  // 設計意図:
  // YIELD/SAFE_STOP以外へ戻ったら、未来譲りholdを残して次の判断を縛らない。
  markIfChanged(now_sec, current, next);
  if (next != BehaviorMode::YIELD_BEHIND && next != BehaviorMode::SAFE_STOP) {
    future_yield_hold_active_ = false;
  }
  return next;
}

} // namespace overtake_planner
