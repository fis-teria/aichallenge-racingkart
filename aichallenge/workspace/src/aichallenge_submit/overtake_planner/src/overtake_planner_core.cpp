#include "overtake_planner/overtake_planner_core.hpp"

#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/planner_output_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace overtake_planner {

namespace {

// 入力: 現在mode。
// 出力: 左追い越し準備/実行中ならtrue。
// 処理概要: 左側pass gapの喪失判定や優先候補維持に使う。
bool isLeftPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_LEFT;
}

// 入力: 現在mode。
// 出力: 右追い越し準備/実行中ならtrue。
// 処理概要: 右側pass gapの喪失判定や優先候補維持に使う。
bool isRightPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

// 入力: 現在mode。
// 出力: 左右どちらかの追い越し準備/実行中ならtrue。
// 処理概要: 追い越し中だけ中心線基準ではなく、追い越し目標基準の横誤差も許容する。
bool isAnyPassMode(BehaviorMode mode) {
  return isLeftPassMode(mode) || isRightPassMode(mode);
}

// 入力: 現在modeと左右pass可否を含むBlockedInfo。
// 出力: 現在走っている側のpass gapが失われたならtrue。
// 処理概要: 追い越し中に走行中の側が塞がれた場合、YIELD/復帰候補を追加するトリガにする。
bool currentPassGapLost(BehaviorMode mode, const BlockedInfo &blocked_info) {
  if (isLeftPassMode(mode)) {
    return blocked_info.pass_left_candidate_generated
               ? !blocked_info.pass_left_candidate_feasible
               : !blocked_info.can_pass_left;
  }
  if (isRightPassMode(mode)) {
    return blocked_info.pass_right_candidate_generated
               ? !blocked_info.pass_right_candidate_feasible
               : !blocked_info.can_pass_right;
  }
  return false;
}

// 入力: CandidateType。
// 出力: 左右PASS候補ならtrue。
// 処理概要: safe stop判定や候補集合の有無チェックでPASSだけを抽出する。
bool isPassCandidate(CandidateType type) {
  return type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
}

// 入力: CandidateType。
// 出力: PASS以外の通常回避/追従候補ならtrue。
// 処理概要: PASSが使えない時に安全な代替候補があるかを調べる。
bool isFallbackCandidate(CandidateType type) {
  return type == CandidateType::FOLLOW || type == CandidateType::YIELD_BEHIND ||
         type == CandidateType::RECOVERY ||
         type == CandidateType::SIDE_BY_SIDE_KEEP;
}

// 入力: BehaviorMode。
// 出力: 横位置を安定化すべきmodeならtrue。
// 処理概要: YIELD/RECOVERY/SAFE_STOP/SPEED_GUARD中の横参照hold対象をまとめる。
bool isLateralStabilizationMode(BehaviorMode mode) {
  return mode == BehaviorMode::ABORT_RECOVERY ||
         mode == BehaviorMode::YIELD_BEHIND ||
         mode == BehaviorMode::SAFE_STOP || mode == BehaviorMode::SPEED_GUARD;
}

// 入力: 最終PlannerOutput。
// 出力: 横参照holdを発動するほどの危険文脈があるならtrue。
// 処理概要: コーナー横並び、未来譲り、速度ガードなど短周期で横目標を動かしたくない状態を集約する。
bool hasLateralStabilizationRisk(const PlannerOutput &output) {
  const auto &blocked = output.blocked_info;
  return blocked.corner_side_by_side || blocked.future_yield_required ||
         blocked.future_corner_side_by_side ||
         blocked.future_outer_wall_risk ||
         output.speed_only_fallback_active ||
         output.wall_risk_speed_guard_active ||
         output.mpc_health_speed_guard_active ||
         output.recovery_speed_guard_active ||
         (output.reentry_gate.requested && !output.reentry_gate.permitted);
}

// 入力: PlannerConfig。
// 出力: 高速カーブ横参照holdへ入る曲率しきい値[1/m]。
// 処理概要: コーナー譲り設定を優先し、無ければ直線追い越しgateの曲率を使う。
double highSpeedCurveHoldEnterCurvature(const PlannerConfig &config) {
  if (config.corner_side_yield_curvature_m_inv > 0.0) {
    return config.corner_side_yield_curvature_m_inv;
  }
  return std::max(0.0, config.straight_overtake_max_curvature_m_inv);
}

// 入力: PlannerConfig。
// 出力: 高速カーブ横参照holdを解除する曲率しきい値[1/m]。
// 処理概要: 明示解除しきい値が無い場合、入場曲率の半分をヒステリシスとして使う。
double highSpeedCurveHoldReleaseCurvature(const PlannerConfig &config) {
  if (config.high_speed_curve_lateral_hold_release_curvature_m_inv >= 0.0) {
    return config.high_speed_curve_lateral_hold_release_curvature_m_inv;
  }
  return highSpeedCurveHoldEnterCurvature(config) * 0.5;
}

// 入力: 保持済みoffset配列と参照index。
// 出力: indexに対応する保持offset。範囲外は最後の値。
// 処理概要: 新しいhorizon長が変わっても保持中の横参照を安全に再利用する。
double heldOffsetAt(const std::vector<double> &offsets, std::size_t index) {
  if (offsets.empty()) {
    return 0.0;
  }
  return offsets[std::min(index, offsets.size() - 1)];
}

// 入力: 候補集合と候補種別predicate。
// 出力: predicateに合うfeasible候補が1つでもあればtrue。
// 処理概要: safe stopへ落とす前に、PASSやfallback候補の有無を確認する。
bool hasFeasibleCandidate(const std::vector<CandidateTrajectory> &candidates,
                          bool (*predicate)(CandidateType)) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [predicate](const CandidateTrajectory &candidate) {
                       return candidate.feasible && predicate(candidate.type);
                     });
}

// 入力: 選択候補とBlockedInfo。
// 出力: SAFE_STOP解除時に通常復帰へ使える候補ならtrue。
// 処理概要: 停止解除後に危険候補へ飛ばないよう、解放に使える候補種別を制限する。
bool isFeasibleReleaseCandidate(const CandidateTrajectory &selected,
                                const BlockedInfo &blocked_info) {
  if (!selected.feasible || selected.type == CandidateType::SAFE_STOP) {
    return false;
  }
  if (!blocked_info.blocked && !blocked_info.side_by_side &&
      !blocked_info.future_yield_required) {
    return selected.type == CandidateType::FASTEST ||
           selected.type == CandidateType::RECOVERY;
  }
  return isPassCandidate(selected.type) ||
         selected.type == CandidateType::FOLLOW ||
         selected.type == CandidateType::YIELD_BEHIND ||
         selected.type == CandidateType::RECOVERY;
}

// 入力: BehaviorMode。
// 出力: PREPARE_OVERTAKE_*ならtrue。
// 処理概要: PREPARE中だけPASS horizon publishを抑制する設定に使う。
bool isPrepareOvertakeMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT;
}

// 入力: PlannerConfig。
// 出力: PREPARE中にもPASS horizonをpublishする設定ならtrue。
// 処理概要: 内部PASS評価とMPCへ出すhorizonを分離するための切替を読む。
bool publishPassHorizonInPrepare(const PlannerConfig &config) {
  return config.pass_horizon_publish_mode != "overtake_only";
}

// 入力: planner設定、現在mode、ラッチ済み局所プロファイル。
// 出力: 現在の追い越し方向が目標にしているd。非PASS文脈ならNaN。
// 処理概要: 横ずれ判定で「中心から離れたこと」自体を異常扱いしないため、PASS目標dを参照する。
double passTargetForMode(const PlannerConfig &config, BehaviorMode mode,
                         const LocalizedLateralProfile &profile) {
  if (!isAnyPassMode(mode)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (profile.active &&
      ((isLeftPassMode(mode) && profile.pass_type == CandidateType::PASS_LEFT) ||
       (isRightPassMode(mode) &&
        profile.pass_type == CandidateType::PASS_RIGHT)) &&
      std::isfinite(profile.target_d_m)) {
    return profile.target_d_m;
  }
  if (isLeftPassMode(mode)) {
    return config.left_offset_m;
  }
  if (isRightPassMode(mode)) {
    return config.right_offset_m;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// 入力: planner設定、現在mode、自車、ラッチ済み局所プロファイル。
// 出力: large_lateral_error判定に使う横誤差[m]。
// 処理概要: 非追い越し時は従来どおり中心線基準、追い越し中は中心線とPASS目標の近い方を使う。
double lateralErrorForPassAwareFreeze(const PlannerConfig &config,
                                      BehaviorMode mode, const EgoState &ego,
                                      const LocalizedLateralProfile &profile) {
  if (!std::isfinite(ego.frenet.d)) {
    return std::numeric_limits<double>::infinity();
  }
  double error = std::abs(ego.frenet.d);
  const double pass_target_d = passTargetForMode(config, mode, profile);
  if (std::isfinite(pass_target_d)) {
    error = std::min(error, std::abs(ego.frenet.d - pass_target_d));
  }
  return error;
}

// 入力: 候補集合とBlockedInfo。
// 出力: SAFE_STOP解除時に使える候補が1つでもあればtrue。
// 処理概要: SAFE_STOP release_readyの判定を候補集合全体に広げる。
bool hasFeasibleReleaseCandidate(
    const std::vector<CandidateTrajectory> &candidates,
    const BlockedInfo &blocked_info) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [&blocked_info](const CandidateTrajectory &candidate) {
                       return isFeasibleReleaseCandidate(candidate,
                                                         blocked_info);
                     });
}

constexpr double kPublishedLateralTargetMemorySec = 0.5;
constexpr double kStartGraceMotionSpeedMps = 0.20;

} // namespace

// 入力: Frenet参照線とplanner設定。
// 出力: overtake planner coreのインスタンス。
// 処理概要: リスク解析、候補生成、安全評価、状態機械を同じ参照線/設定で動かすために初期化する。
OvertakePlannerCore::OvertakePlannerCore(FrenetFrame frame,
                                         PlannerConfig config)
    : frame_(std::move(frame)), config_(config), blocked_risk_(frame_, config_),
      future_side_risk_(frame_, config_, blocked_risk_), safety_(config),
      state_machine_(config) {}

// 入力: 現在時刻、自車状態、相手車一覧、MPC health。
// 出力: MPC overrideとdebug情報を含むPlannerOutput。
// 処理概要: リスク判定、候補生成、安全評価、状態機械、publish用候補整形を1周期分実行する。
PlannerOutput
OvertakePlannerCore::update(double now_sec, const EgoState &ego,
                            const std::vector<OpponentState> &opponents,
                            const MpcHealthStatus &mpc_health,
                            const ReentryInputStatus &reentry_input) {
  // デフォルトはMPCの元参照をそのまま使う。安全に判断できる時だけoverrideを有効化する。
  PlannerOutput output;
  output.lateral_offsets.assign(config_.horizon_points, 0.0);
  output.speed_caps.assign(config_.horizon_points, config_.v_passthrough_mps);

  if (!config_.enabled || frame_.empty()) {
    // planner無効化または参照線欠損時は、既存契約どおりoverrideを出さない。
    mode_ = BehaviorMode::FREE_RUN;
    safe_stop_trigger_count_ = 0;
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = false;
    reentry_phase_active_ = false;
    slow_front_exception_count_ = 0;
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    clearLocalizedLateralProfile();
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }
  if (!ego.valid) {
    // 復帰/lockout中に自己状態が欠けても、overrideを消して通常ラインへ
    // fall throughしない。最後に安全にpublishした横列を低速で維持する。
    const bool reentry_context = config_.reentry_gate_enabled &&
                                 (reentry_lockout_active_ ||
                                  mode_ != BehaviorMode::FREE_RUN);
    if (reentry_context) {
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      reentry_clear_cycles_ = 0;
      mode_ = BehaviorMode::ABORT_RECOVERY;
      output.mode = mode_;
      output.selected = CandidateType::RECOVERY;
      output.blocked_info.reentry_hold_active = true;
      output.reentry_gate.requested = true;
      output.reentry_gate.permitted = false;
      output.reentry_gate.input_complete = false;
      output.reentry_gate.reason = "stale_ego";
      if (!last_published_lateral_offsets_.empty()) {
        output.active_override = true;
        output.lateral_offsets = last_published_lateral_offsets_;
        output.speed_caps.assign(
            output.lateral_offsets.size(),
            std::max(1.0e-3, config_.reentry_hold_v_max_mps));
        output.target_lateral_offset_m = output.lateral_offsets.back();
        output.applied_speed_cap_mps = config_.reentry_hold_v_max_mps;
        output.speed_cap_reason = "reentry_stale_ego_hold";
        output.reason = "reentry_stale_ego_hold";
      } else {
        // 初回の有効override前は保持すべき安全横列が無いため、値を捏造しない。
        // Node/controller側の既存stale watchdogへ明示的に委譲する。
        output.reason = "reentry_stale_ego_no_hold_reference";
      }
      return output;
    }

    // 復帰文脈でない自己状態欠損は従来どおりplanner overrideを無効化する。
    mode_ = BehaviorMode::FREE_RUN;
    safe_stop_trigger_count_ = 0;
    reentry_clear_cycles_ = 0;
    reentry_phase_active_ = false;
    slow_front_exception_count_ = 0;
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    clearLocalizedLateralProfile();
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }
  if (!std::isfinite(first_valid_update_sec_)) {
    first_valid_update_sec_ = now_sec;
  }
  if (!std::isfinite(first_motion_update_sec_) && std::isfinite(ego.v) &&
      ego.v >= kStartGraceMotionSpeedMps) {
    first_motion_update_sec_ = now_sec;
  }

  // 処理ブロック: コース区間設定、追い越し許可、現在相手車リスクを集約する。
  // 設計意図: 候補生成前に「今どの制約が有効か」をBlockedInfoへ一度まとめる。
  const ActiveSectionSafety active_section = activeSectionSafety(ego.frenet.s);
  const ActiveOvertakePermission active_overtake_permission =
      activeOvertakePermission(ego.frenet.s);
  const double wall_soft_margin = effectiveWallSoftMargin(active_section);
  BlockedInfo blocked = blocked_risk_.detectBlocked(ego, opponents, now_sec);
  const auto predictions = predictOpponents(opponents, now_sec);
  promoteSlowObstacleChain(blocked, opponents);
  classifyStationaryFrontObstacle(now_sec, ego, blocked, opponents);
  blocked =
      blocked_risk_.evaluatePassGap(blocked, opponents, predictions, mode_);
  blocked.overtake_permission_allowed =
      !config_.overtake_permission_profile_enabled ||
      active_overtake_permission.allow_overtake;
  blocked.overtake_permission_section_name = active_overtake_permission.name;
  if (!config_.overtake_permission_profile_enabled) {
    blocked.overtake_permission_reason = "permission_profile_disabled";
  } else if (active_overtake_permission.active) {
    blocked.overtake_permission_reason =
        active_overtake_permission.allow_overtake ? "section_allowed"
                                                  : "section_disallowed";
  } else {
    blocked.overtake_permission_reason =
        config_.default_overtake_allowed ? "default_allowed"
                                         : "default_disallowed";
  }
  const bool front_low_speed_by_distance =
      blocked.nearest_index >= 0 &&
      std::isfinite(blocked.front_vehicle_speed_mps) &&
      blocked.front_vehicle_speed_mps <=
          std::max(0.0, config_.slow_front_exception_speed_mps) &&
      (blocked.front_delta_s <=
           std::max(0.0, config_.slow_front_exception_distance_m) ||
       blocked.slow_obstacle_chain_active);
  blocked.front_vehicle_low_speed =
      config_.slow_front_exception_enabled && front_low_speed_by_distance;
  blocked.slow_front_exception_active = updateSlowFrontException(blocked);
  blocked.slow_front_exception_count = slow_front_exception_count_;
  // 処理ブロック: 追い越し開始gateとコーナー横並びリスクを判定する。
  // 設計意図: コーナー入口で新規追い越しを始めない一方、既に横並びなら譲りや維持へ誘導する。
  blocked.corner_abs_curvature =
      maxAbsCurvatureAhead(ego.frenet.s, config_.corner_side_yield_lookahead_m);
  blocked.corner_side_by_side =
      blocked.side_by_side && config_.corner_side_yield_curvature_m_inv > 0.0 &&
      blocked.corner_abs_curvature >= config_.corner_side_yield_curvature_m_inv;
  blocked.overtake_start_abs_curvature =
      maxAbsCurvatureAhead(ego.frenet.s, config_.straight_overtake_lookahead_m);
  blocked.straight_overtake_start_allowed = true;
  if (config_.straight_only_overtake_enabled &&
      config_.straight_overtake_max_curvature_m_inv > 0.0) {
    const double close_threshold =
        std::max(0.0, config_.straight_overtake_max_curvature_m_inv);
    const double open_threshold = std::max(
        0.0,
        close_threshold -
            std::max(0.0, config_.straight_overtake_release_hysteresis_m_inv));
    if (straight_overtake_start_allowed_) {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= close_threshold;
    } else {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= open_threshold;
    }
    blocked.straight_overtake_start_allowed = straight_overtake_start_allowed_;
    if (!blocked.straight_overtake_start_allowed) {
      blocked.overtake_start_gate_reason = "curve";
    }
  } else {
    straight_overtake_start_allowed_ = true;
  }
  const bool permission_start_allowed = blocked.overtake_permission_allowed;
  if (!permission_start_allowed) {
    straight_overtake_start_allowed_ = false;
    blocked.straight_overtake_start_allowed = false;
    if (blocked.overtake_start_gate_reason.empty()) {
      blocked.overtake_start_gate_reason =
          blocked.overtake_permission_reason.empty()
              ? "section_disallowed"
              : blocked.overtake_permission_reason;
    }
  }
  blocked.ego_lateral_offset_m = ego.frenet.d;
  blocked.ego_speed_mps = ego.v;
  blocked.ego_wall_clearance_m = blocked_risk_.wallClearance(ego.frenet.d);
  // 処理ブロック: 未来横並びとsection safetyを重ねて、事前譲りが必要かを決める。
  // 設計意図: 現在は接触していなくても、コーナーで外側車両が壁へ寄る場面を先に抑える。
  blocked = future_side_risk_.evaluate(ego, blocked, opponents);
  if (active_section.force_outer_yield &&
      (blocked.side_by_side || blocked.parallel_side_candidate ||
       blocked.future_side_by_side) &&
      blocked.ego_wall_clearance_m <= wall_soft_margin) {
    blocked.future_yield_required = true;
    blocked.future_corner_side_by_side = true;
    blocked.future_outer_wall_risk = true;
    if (blocked.yield_reason.empty()) {
      blocked.yield_reason = "section_outer_yield";
    }
  }
  updateLeaderPriority(now_sec, blocked);
  const double pass_aware_lateral_error =
      lateralErrorForPassAwareFreeze(config_, mode_, ego,
                                     localized_lateral_profile_);
  const bool large_lateral_error =
      config_.large_lateral_error_threshold_m >= 0.0 &&
      pass_aware_lateral_error > config_.large_lateral_error_threshold_m;
  const bool freeze_overtake_decisions =
      large_lateral_error &&
      (blocked.ego_wall_clearance_m < wall_soft_margin || blocked.blocked ||
       blocked.side_by_side || blocked.parallel_side_candidate ||
       blocked.future_side_by_side || blocked.future_yield_required ||
       currentPassGapLost(mode_, blocked));
  if (freeze_overtake_decisions) {
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason = "large_lateral_error";
    if (blocked.pass_gap_reason.empty() || blocked.pass_gap_reason == "ok") {
      blocked.pass_gap_reason = "large_lateral_error";
    }
  }
  // 通常ラインへの復帰は、d2を抜いた事実や単一front gapでは許可しない。
  // 実際にpublishするRECOVERY形状を全fresh相手車両へ評価し、未許可なら現在dを保持する。
  updateReentryPhase(ego);
  ReentryGateResult reentry_gate = evaluateReentryGate(
      now_sec, ego, blocked, opponents, mpc_health, reentry_input);
  if (reentry_gate.requested && !reentry_gate.permitted) {
    // dが中心付近まで到達する前にmodeだけFREE_RUNへ戻っても、次周期以降に
    // 復帰ゲートを必ず継続するためlockoutを残す。
    reentry_lockout_active_ = true;
    reentry_phase_active_ = true;
    blocked.reentry_hold_active = true;
  }
  // 処理ブロック: 局所回避プロファイルのラッチ状態を更新する。
  // 設計意図: PASS候補の横ラインが相手位置の短周期変動で揺れないよう、対象sと回避区間を保持する。
  updateLocalizedLateralProfile(now_sec, ego, blocked, opponents);

  // 処理ブロック: 状況に応じた候補集合を作る。
  // 設計意図: PASS候補は内部評価へ残しつつ、FOLLOW/YIELD/RECOVERY/SAFE_STOPの代替候補も同時に安全評価する。
  // まず全状況でFASTEST候補を作り、閉塞時だけ追従/左右追い越し候補を増やす。
  std::vector<CandidateTrajectory> candidates;
  candidates.push_back(
      makeCandidate(CandidateType::FASTEST, ego, blocked, opponents));
  if (freeze_overtake_decisions) {
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  } else {
    if (blocked.side_by_side && !blocked.corner_side_by_side &&
        !blocked.future_yield_required) {
      candidates.push_back(makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego,
                                         blocked, opponents));
    }
    if (shouldYieldBehindSideBySide(ego, blocked)) {
      candidates.push_back(
          makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
    }
    if (blocked.blocked) {
      candidates.push_back(
          makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents));
      // 静的gapは診断値として残し、左右の物理的安全性は各候補を同じSafetyEvaluatorで判定する。
      // 追い越し中は反対側のPASS候補を作らない。同じ側の安全性が失われたら
      // 横切って切り替えず、YIELD/RECOVERYへ戻す。
      const bool evaluate_left_pass =
          !isRightPassMode(mode_) &&
          (blocked.can_pass_left || config_.dynamic_pass_candidate_enabled);
      const bool evaluate_right_pass =
          !isLeftPassMode(mode_) &&
          (blocked.can_pass_right || config_.dynamic_pass_candidate_enabled);
      if (evaluate_left_pass) {
        candidates.push_back(
            makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
        blocked.pass_left_candidate_generated = true;
      }
      if (evaluate_right_pass) {
        candidates.push_back(
            makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
        blocked.pass_right_candidate_generated = true;
      }
      candidates.push_back(
          makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
    }
  }
  if (currentPassGapLost(mode_, blocked)) {
    candidates.push_back(
        makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
  }
  if (isPassMode(mode_) || mode_ == BehaviorMode::ABORT_RECOVERY) {
    // 追い越し中や中止中は、中心線へ戻るRECOVERY候補も常に評価する。
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }
  if (!freeze_overtake_decisions && isLeftPassMode(mode_) &&
      !blocked.pass_left_candidate_generated) {
    // 前方閉塞が一時的に解けても、走行中PASSの継続可否は同じ時系列安全評価で確認する。
    candidates.push_back(
        makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
    blocked.pass_left_candidate_generated = true;
  }
  if (!freeze_overtake_decisions && isRightPassMode(mode_) &&
      !blocked.pass_right_candidate_generated) {
    candidates.push_back(
        makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
    blocked.pass_right_candidate_generated = true;
  }
  const bool needs_safe_stop_fallback_check =
      config_.safe_stop_enabled &&
      (blocked.blocked || blocked.side_by_side ||
       blocked.parallel_side_candidate || blocked.future_yield_required ||
       currentPassGapLost(mode_, blocked));
  const bool has_recovery_candidate =
      std::any_of(candidates.begin(), candidates.end(),
                  [](const CandidateTrajectory &candidate) {
                    return candidate.type == CandidateType::RECOVERY;
                  });
  if (needs_safe_stop_fallback_check && !has_recovery_candidate) {
    // SAFE_STOP判定前に、通常fallbackであるRECOVERYも必ず安全評価へ含める。
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }

  // 処理ブロック: 全候補を安全評価してscoreを付ける。
  // 設計意図: 先にfeasibleを確定し、その後で追い越し意欲やfallback優先度を比較する。
  for (auto &candidate : candidates) {
    // 壁/他車との安全余裕を見てから、減速モデルが有効かも同時に確認する。
    safety_.evaluate(candidate, predictions);
    if (!candidate.longitudinal_profile_valid) {
      candidate.feasible = false;
      candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    const bool stationary_braking_shortfall =
        blocked.stationary_front_obstacle &&
        (candidate.type == CandidateType::FOLLOW ||
         candidate.type == CandidateType::YIELD_BEHIND) &&
        candidate.required_brake_distance_m >
            candidate.available_brake_distance_m;
    if (stationary_braking_shortfall) {
      // 評価horizon内だけ安全に見えても、停止余裕が尽きる追従/譲りは通常候補に採用しない。
      candidate.feasible = false;
      candidate.reject_reason = "insufficient_braking_distance";
    }
    if (candidate.type == CandidateType::PASS_LEFT) {
      blocked.pass_left_candidate_feasible = candidate.feasible;
    } else if (candidate.type == CandidateType::PASS_RIGHT) {
      blocked.pass_right_candidate_feasible = candidate.feasible;
    } else if (candidate.type == CandidateType::FOLLOW &&
               blocked.stationary_front_obstacle) {
      blocked.stationary_front_required_brake_distance_m =
          candidate.required_brake_distance_m;
      blocked.stationary_front_available_brake_distance_m =
          candidate.available_brake_distance_m;
      blocked.stationary_front_brake_feasible =
          candidate.longitudinal_profile_valid && candidate.feasible &&
          candidate.required_brake_distance_m <=
              candidate.available_brake_distance_m;
    }
  }
  for (auto &candidate : candidates) {
    candidate.score = candidateScore(candidate, blocked);
    if (freeze_overtake_decisions) {
      candidate.score =
          candidate.type == CandidateType::RECOVERY ? -100.0 : 1000.0;
    }
  }

  const bool no_feasible_pass =
      !hasFeasibleCandidate(candidates, isPassCandidate);
  const bool no_feasible_fallback =
      !hasFeasibleCandidate(candidates, isFallbackCandidate);
  const bool safe_stop_base_condition =
      config_.safe_stop_enabled &&
      (blocked.blocked || blocked.side_by_side ||
       blocked.parallel_side_candidate || blocked.future_yield_required ||
       currentPassGapLost(mode_, blocked)) &&
      no_feasible_pass && no_feasible_fallback;
  const bool start_grace_active =
      safe_stop_base_condition &&
      shouldSuppressSafeStopForStartGrace(now_sec, ego, blocked);
  const bool leader_priority_safe_stop_suppressed =
      safe_stop_base_condition && blocked.leader_priority_active;
  const bool effective_safe_stop_base_condition =
      safe_stop_base_condition && !start_grace_active &&
      !leader_priority_safe_stop_suppressed;

  // 処理ブロック: 回避不能状態が連続した時だけSAFE_STOP要求を作る。
  // 設計意図: 一瞬のinfeasibleで停止に入ると走行が固まるため、trigger_cyclesで確定させる。
  if (effective_safe_stop_base_condition) {
    ++safe_stop_trigger_count_;
  } else {
    safe_stop_trigger_count_ = 0;
  }

  const int safe_stop_trigger_cycles_required =
      std::max(1, config_.safe_stop_trigger_cycles);
  SafeStopContext safe_stop_context;
  safe_stop_context.requested =
      effective_safe_stop_base_condition &&
      safe_stop_trigger_count_ >= safe_stop_trigger_cycles_required;
  safe_stop_context.trigger_count = safe_stop_trigger_count_;
  safe_stop_context.ego_speed_mps = ego.v;
  const double safe_stop_clamped_d =
      std::clamp(ego.frenet.d, config_.d_min_m + config_.min_wall_margin_m,
                 config_.d_max_m - config_.min_wall_margin_m);
  safe_stop_context.lateral_error_m =
      std::abs(safe_stop_clamped_d);

  bool safe_stop_candidate_infeasible = false;
  CandidateTrajectory safe_stop_candidate;
  if (safe_stop_context.requested || mode_ == BehaviorMode::SAFE_STOP) {
    // STOP要求時とSTOP保持中は、停止候補自体の安全性も毎周期確認する。
    safe_stop_candidate =
        makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    safety_.evaluate(safe_stop_candidate, predictions);
    if (!safe_stop_candidate.longitudinal_profile_valid) {
      safe_stop_candidate.feasible = false;
      safe_stop_candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    safe_stop_candidate.score = candidateScore(safe_stop_candidate, blocked);
    safe_stop_context.candidate_feasible = safe_stop_candidate.feasible;
    safe_stop_context.reason = safe_stop_candidate.feasible
                                   ? "no_pass_and_no_safe_fallback"
                                   : "safe_stop_infeasible";
    if (safe_stop_context.requested && safe_stop_candidate.feasible) {
      candidates.push_back(safe_stop_candidate);
    } else if (!safe_stop_candidate.feasible) {
      safe_stop_candidate_infeasible = true;
    }
  }
  if (safe_stop_candidate_infeasible) {
    // 一度でも停止候補が不成立になった復帰文脈は、次周期に相手予測が欠けただけで
    // 通常ラインへ戻らないようlockoutする。解除は復帰ゲートの連続clear条件だけ。
    reentry_lockout_active_ = true;
    reentry_clear_cycles_ = 0;
    if (reentry_gate.requested) {
      reentry_gate.permitted = false;
      reentry_gate.clear_cycles = 0;
      reentry_gate.reason = "safe_stop_infeasible";
      reentry_gate.blocking_vehicle_id =
          safe_stop_candidate.blocking_opponent_id;
      reentry_gate.min_safety_margin = safe_stop_candidate.min_safety_margin;
      reentry_gate.cbf_slack = safe_stop_candidate.cbf_slack;
      reentry_gate.blocking_time_sec =
          safe_stop_candidate.blocking_time_sec;
      blocked.reentry_hold_active = true;
    }
  }

  // 処理ブロック: 内部候補を選び、状態機械で運転modeを安定化する。
  // 設計意図: score上の最良候補をそのままpublishせず、保持時間や連続安全回数を通してmodeを決める。
  CandidateTrajectory selected = selectCandidate(candidates);
  const bool release_front_gap_ready =
      blocked.nearest_index < 0 ||
      blocked.front_delta_s >= config_.safe_stop_release_front_gap_m;
  safe_stop_context.release_ready =
      ego.valid &&
      blocked.ego_wall_clearance_m >=
          config_.safe_stop_release_wall_clearance_m &&
      !blocked.side_by_side && !blocked.future_yield_required &&
      release_front_gap_ready &&
      hasFeasibleReleaseCandidate(candidates, blocked) &&
      ego.v <= config_.safe_stop_release_speed_mps &&
      safe_stop_context.lateral_error_m <=
          config_.safe_stop_lateral_error_threshold_m;

  // 候補選択だけで急にモードを切り替えず、状態機械で保持時間や継続条件をかける。
  mode_ = state_machine_.update(now_sec, mode_, selected.type, blocked,
                                selected.feasible, safe_stop_context);
  // PASS実行中は回避開始を復帰と誤認しない。一方この周期にMERGE/YIELD/RECOVERYへ
  // 遷移した場合は、中心方向の候補をpublishする前に同じgateを必ず通す。
  updateReentryPhase(ego);
  if (!reentry_gate.requested && reentryRequested(ego)) {
    reentry_gate = evaluateReentryGate(now_sec, ego, blocked, opponents,
                                       mpc_health, reentry_input);
    if (reentry_gate.requested && !reentry_gate.permitted) {
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      blocked.reentry_hold_active = true;
    }
  }
  if (reentry_gate.requested && !reentry_gate.permitted) {
    // 状態機械がblocked/frontだけを見てFREE_RUNへ戻る経路を、復帰ゲートで閉じる。
    // ABORT_RECOVERYのRECOVERY候補はreentry_hold_activeにより中心方向へ動かない。
    mode_ = BehaviorMode::ABORT_RECOVERY;
  }
  const bool yield_lateral_hold =
      config_.yield_release_lateral_error_m >= 0.0 &&
      std::abs(ego.frenet.d) > config_.yield_release_lateral_error_m;
  if (mode_ == BehaviorMode::YIELD_BEHIND &&
      state_machine_.futureYieldHoldActive()) {
    const bool corner_still_relevant =
        config_.corner_side_yield_curvature_m_inv > 0.0 &&
        blocked.corner_abs_curvature >=
            config_.corner_side_yield_curvature_m_inv;
    blocked.future_yield_required = true;
    blocked.future_corner_side_by_side = blocked.future_corner_side_by_side ||
                                         blocked.corner_side_by_side ||
                                         corner_still_relevant;
    if (blocked.yield_reason.empty()) {
      blocked.yield_reason = "future_yield_hold";
    }
  } else if (mode_ == BehaviorMode::YIELD_BEHIND && yield_lateral_hold &&
             blocked.yield_reason.empty()) {
    blocked.yield_reason = "yield_lateral_error_hold";
  }
  // 処理ブロック: modeに合わせてpublish用候補を再生成する。
  // 設計意図: 内部PASS評価は状態遷移へ使いつつ、FOLLOW中やPREPARE中のMPC horizonは設定に応じて安全側へ差し替える。
  CandidateTrajectory output_selected = selected;
  if (mode_ == BehaviorMode::SAFE_STOP) {
    output_selected =
        makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  } else if (mode_ == BehaviorMode::ABORT_RECOVERY) {
    // 中止時は必ず中心線へ戻す候補を再生成し、最新予測で安全評価する。
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  } else if (mode_ == BehaviorMode::MERGE_BACK) {
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  } else if (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP) {
    output_selected = makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego,
                                    blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  } else if (mode_ == BehaviorMode::YIELD_BEHIND) {
    output_selected =
        makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  } else if (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
             output_selected.type != CandidateType::FOLLOW) {
    // 追従モードでは速度上限だけを落とすFOLLOW候補を優先する。
    output_selected =
        makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
    if (!output_selected.feasible) {
      output_selected =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      safety_.evaluate(output_selected, predictions);
    }
  } else if (isPrepareOvertakeMode(mode_) &&
             !publishPassHorizonInPrepare(config_)) {
    // PASS候補は内部状態遷移に使うが、PREPARE中はMPCへ追い越しhorizonを
    // 出さず、OVERTAKEに入った周期から横オフセットをpublishする。
    output_selected =
        makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
    if (!output_selected.feasible) {
      output_selected =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      safety_.evaluate(output_selected, predictions);
    }
  } else if (mode_ == BehaviorMode::FREE_RUN) {
    output_selected =
        makeCandidate(CandidateType::FASTEST, ego, blocked, opponents);
    safety_.evaluate(output_selected, predictions);
  }

  // ROSノードがMPC overrideとdebug
  // JSONを作れるよう、選択結果を平坦な出力に詰める。
  // 処理ブロック: 出力形式へ変換し、rate limitと高速カーブholdを最後に適用する。
  // 設計意図: 候補生成後の急な横参照変化をpublish直前で抑え、MPCへの入力を安定させる。
  auto output_built =
      PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
          mode_, ego, output_selected, blocked, safe_stop_context,
          safe_stop_candidate, safe_stop_candidate_infeasible,
          safe_stop_trigger_count_,
          state_machine_.safeStopHoldCount(),
          state_machine_.safeStopReleaseCount(), wall_soft_margin,
          active_section, mpc_health});
  output_built.start_grace_active = start_grace_active;
  output_built.reentry_gate = reentry_gate;
  output_built.lateral_profile_mode = config_.overtake_lateral_profile_mode;
  output_built.maneuver_latch_active = localized_lateral_profile_.active;
  if (localized_lateral_profile_.active) {
    output_built.maneuver_latch_target_id =
        localized_lateral_profile_.target_id;
    output_built.maneuver_latch_target_s_m =
        localized_lateral_profile_.target_s_m;
    output_built.maneuver_latch_avoid_start_s_m =
        localized_lateral_profile_.avoid_start_s_m;
    output_built.maneuver_latch_full_offset_start_s_m =
        localized_lateral_profile_.full_offset_start_s_m;
    output_built.maneuver_latch_full_offset_end_s_m =
        localized_lateral_profile_.full_offset_end_s_m;
    output_built.maneuver_latch_merge_end_s_m =
        localized_lateral_profile_.merge_end_s_m;
  }
  if (start_grace_active) {
    output_built.reason = "start_grace_safe_stop_suppressed";
  } else if (leader_priority_safe_stop_suppressed &&
             output_built.reason.empty()) {
    output_built.reason = "leader_priority_safe_stop_suppressed";
  }
  applyLateralTargetRateLimit(now_sec, output_built);
  applyHighSpeedCurveLateralHold(now_sec, ego, output_built);
  if (!revalidatePublishedLateral(output_built, output_selected, predictions)) {
    // publish直前のrate limit/holdで形が変わったPASS軌道を、そのままMPCへ渡さない。
    // overrideを消して通常走行へ戻すと、停止障害物の前で再加速し得るため、
    // 同じ周期にRECOVERYを安全評価してpublishする。RECOVERYも不可なら
    // PlannerOutputBuilderの既存speed-only fallbackへ必ず渡す。
    mode_ = BehaviorMode::ABORT_RECOVERY;
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    // 既存のPASS再評価失敗は従来どおり中心へRECOVERYする。復帰ゲートが
    // 有効な通常ライン復帰だけ、横位置を保持して二次衝突を防ぐ。
    blocked.reentry_hold_active = reentry_gate.requested;
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = true;
    if (reentry_gate.requested) {
      reentry_gate.permitted = false;
      reentry_gate.clear_cycles = 0;
      reentry_gate.reason = "published_reentry_safety_reject";
    }
    CandidateTrajectory recovery =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(recovery, predictions);
    if (!recovery.longitudinal_profile_valid) {
      recovery.feasible = false;
      recovery.reject_reason = "invalid_longitudinal_brake_model";
    }
    output_built = PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
        mode_, ego, recovery, blocked, safe_stop_context, safe_stop_candidate,
        safe_stop_candidate_infeasible, safe_stop_trigger_count_,
        state_machine_.safeStopHoldCount(), state_machine_.safeStopReleaseCount(),
        wall_soft_margin, active_section, mpc_health});
    output_built.reason = recovery.feasible
                              ? "published_lateral_safety_reject_recovery"
                              : "published_lateral_safety_reject_speed_only";
    output_built.published_lateral_safety_rejected = true;
    output_built.reentry_gate = reentry_gate;
  }
  rememberPublishedLateralTarget(now_sec, output_built);
  return output_built;
}

// 入力: 相手車一覧と現在時刻。
// 出力: 各相手車の等速予測軌道。
// 処理概要: V2X速度推定を使い、planner horizon上の相手位置をFrenet/Cartesianで並べる。
std::vector<PredictedOpponent> OvertakePlannerCore::predictOpponents(
    const std::vector<OpponentState> &opponents, double now_sec,
    const std::vector<double> *time_points) const {
  // V2X位置から推定した速度を使い、短いhorizonでは等速直線運動として予測する。
  std::vector<PredictedOpponent> out;
  for (const auto &opp : opponents) {
    if (!opp.valid || !std::isfinite(opp.stamp_sec) || now_sec < opp.stamp_sec ||
        now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
      continue;
    }
    PredictedOpponent pred;
    pred.id = opp.id;
    const std::size_t count =
        time_points == nullptr ? config_.horizon_points : time_points->size();
    for (std::size_t i = 0; i < count; ++i) {
      const double t = time_points == nullptr
                           ? static_cast<double>(i) * config_.horizon_dt_sec
                           : (*time_points)[i];
      if (!std::isfinite(t) || t < 0.0) {
        continue;
      }
      const double x = opp.x + opp.vx * t;
      const double y = opp.y + opp.vy * t;
      const auto fr = frame_.cartesianToFrenet(x, y, 0.0);
      pred.t.push_back(t);
      pred.x.push_back(x);
      pred.y.push_back(y);
      pred.s.push_back(fr.s);
      pred.d.push_back(fr.d);
    }
    out.push_back(std::move(pred));
  }
  return out;
}

// 入力: 自車、現在のBlockedInfo、相手車一覧。
// 出力: publish用horizonより長い、通常ラインまで戻るRECOVERY評価軌道。
// 処理概要: 短いMPC horizonだけが安全でも、merge終端で他車へ入る復帰を許可しない。
CandidateTrajectory OvertakePlannerCore::makeReentryEvaluationCandidate(
    const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  PlannerConfig evaluation_config = config_;
  const double dt_sec = std::max(1.0e-3, config_.horizon_dt_sec);
  const double horizon_sec = std::max(
      static_cast<double>(config_.horizon_points) * dt_sec,
      std::max(0.0, config_.reentry_evaluation_horizon_sec));
  evaluation_config.horizon_points = std::max<std::size_t>(
      config_.horizon_points,
      static_cast<std::size_t>(std::ceil(horizon_sec / dt_sec)) + 1U);
  // 実行時のRECOVERYが横誤差のrelease閾値で一時的に保持されていても、
  // ゲート評価は「通常ラインへ実際に戻る」最悪ケースを必ず検査する。
  evaluation_config.recovery_release_lateral_error_m = 0.0;
  BlockedInfo evaluation_blocked = blocked_info;
  evaluation_blocked.reentry_hold_active = false;
  return CandidateBuilder(frame_, evaluation_config)
      .makeCandidate(CandidateType::RECOVERY, ego, evaluation_blocked,
                     opponents, nullptr);
}

// 入力: 自車状態。
// 出力: 現在のmodeで通常ラインへの横移動を始める/継続する必要があるならtrue。
// 処理概要: 通常走行や完全に中心へ戻った状態までV2X欠損で固定しないよう、復帰文脈だけを選ぶ。
bool OvertakePlannerCore::reentryRequested(const EgoState &ego) const {
  // PASSの横移動開始を中心線復帰と誤認しない。復帰phaseへ入った後はFREE_RUNへ
  // modeだけが変わっても、中心へ戻り切るまで同じgateを継続する。
  if (reentry_lockout_active_) {
    return true;
  }
  return reentry_phase_active_ && std::isfinite(ego.frenet.d) &&
         std::abs(ego.frenet.d) > 1.0e-3;
}

// 入力: 現在の自車横位置と直前/遷移後のbehavior mode。
// 出力: なし。通常ライン復帰を開始した文脈をラッチする。
// 処理概要: PASSの外向き横移動は除外し、YIELD/FOLLOW/RECOVERYからFREE_RUNへ
// 遷移しても中心復帰が終わるまでgate適用を失わない。
void OvertakePlannerCore::updateReentryPhase(const EgoState &ego) {
  if (reentry_lockout_active_) {
    reentry_phase_active_ = true;
    return;
  }
  if (!std::isfinite(ego.frenet.d) || std::abs(ego.frenet.d) <= 1.0e-3) {
    reentry_phase_active_ = false;
    return;
  }
  // PASS準備・実行中は回避側へ出る途中なので、ここでRECOVERY評価を開始しない。
  if (isPassMode(mode_)) {
    return;
  }
  // FREE_RUN外の横ずれはすべて通常ライン復帰の候補を持ち得る。FREE_RUNで
  // 横ずれが残る場合は、以前にlatchedしたphaseだけを継続する。
  if (mode_ != BehaviorMode::FREE_RUN) {
    reentry_phase_active_ = true;
  }
}

// 入力: 現在時刻、自車、周辺車、MPC health、Node側の入力鮮度状態。
// 出力: 通常ライン復帰を許可できるかと、拒否理由・ブロッカー。
// 処理概要: 全fresh相手の予測と復帰終端までの候補を同じSafetyEvaluatorで照合し、
// 連続安全周期を満たすまでfail-closedにする。
ReentryGateResult OvertakePlannerCore::evaluateReentryGate(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const MpcHealthStatus &mpc_health,
    const ReentryInputStatus &reentry_input) {
  (void)mpc_health;
  ReentryGateResult gate;
  gate.requested = config_.reentry_gate_enabled && reentryRequested(ego);
  if (!gate.requested) {
    reentry_clear_cycles_ = 0;
    return gate;
  }

  const bool mpc_input_ready =
      !config_.reentry_require_mpc_health || reentry_input.mpc_healthy;
  gate.input_complete = reentry_input.ego_fresh &&
                        reentry_input.v2x_snapshot_fresh &&
                        reentry_input.all_observed_opponents_fresh &&
                        reentry_input.all_observed_opponents_included &&
                        reentry_input.reference_valid && mpc_input_ready;
  if (!gate.input_complete) {
    reentry_clear_cycles_ = 0;
    gate.reason = !reentry_input.ego_fresh
                      ? "stale_ego"
                      : !reentry_input.v2x_snapshot_fresh
                            ? "stale_v2x_snapshot"
                            : !reentry_input.all_observed_opponents_fresh
                                  ? "stale_reentry_opponent"
                                  : !reentry_input
                                            .all_observed_opponents_included
                                        ? "untracked_reentry_opponent"
                                  : !reentry_input.reference_valid
                                        ? "invalid_reference"
                                        : "unhealthy_mpc";
    gate.clear_cycles = reentry_clear_cycles_;
    return gate;
  }

  const auto reentry_candidate =
      makeReentryEvaluationCandidate(ego, blocked_info, opponents);
  const auto predictions =
      predictOpponents(opponents, now_sec, &reentry_candidate.t);
  gate.evaluated_opponent_count = static_cast<int>(predictions.size());
  if (predictions.size() != opponents.size() ||
      reentry_candidate.t.empty() ||
      reentry_candidate.x.size() != reentry_candidate.t.size() ||
      reentry_candidate.y.size() != reentry_candidate.t.size() ||
      reentry_candidate.yaw.size() != reentry_candidate.t.size()) {
    reentry_clear_cycles_ = 0;
    gate.reason = "incomplete_reentry_prediction";
    gate.clear_cycles = reentry_clear_cycles_;
    return gate;
  }

  CandidateTrajectory evaluated = reentry_candidate;
  safety_.evaluate(evaluated, predictions);
  gate.min_safety_margin = evaluated.min_safety_margin;
  gate.cbf_slack = evaluated.cbf_slack;
  gate.blocking_vehicle_id = evaluated.blocking_opponent_id;
  gate.blocking_time_sec = evaluated.blocking_time_sec;
  if (!evaluated.longitudinal_profile_valid) {
    reentry_clear_cycles_ = 0;
    gate.reason = "invalid_longitudinal_brake_model";
  } else if (!evaluated.feasible) {
    reentry_clear_cycles_ = 0;
    gate.reason = evaluated.reject_reason;
  } else if (std::isnan(evaluated.min_safety_margin) ||
             evaluated.min_safety_margin <
                 config_.reentry_min_safety_margin_h) {
    reentry_clear_cycles_ = 0;
    gate.reason = "reentry_margin_below_threshold";
  } else if (evaluated.cbf_slack > 0.0) {
    reentry_clear_cycles_ = 0;
    gate.reason = "reentry_cbf_slack";
  } else {
    ++reentry_clear_cycles_;
    gate.clear_cycles = reentry_clear_cycles_;
    const int required_cycles = std::max(1, config_.reentry_safe_cycles);
    gate.permitted = reentry_clear_cycles_ >= required_cycles;
    gate.reason = gate.permitted ? "reentry_clear" : "reentry_clear_pending";
    if (gate.permitted) {
      reentry_lockout_active_ = false;
    }
    return gate;
  }

  gate.clear_cycles = reentry_clear_cycles_;
  if (reentry_lockout_active_ && gate.reason.empty()) {
    gate.reason = "reentry_lockout";
  }
  return gate;
}

// 入力: 候補種別、自車状態、BlockedInfo、相手車一覧。
// 出力: CandidateBuilderで生成した候補軌道。
// 処理概要: core側で保持している局所横プロファイルを必要に応じて候補生成へ渡す。
CandidateTrajectory OvertakePlannerCore::makeCandidate(
    CandidateType type, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  return CandidateBuilder(frame_, config_)
      .makeCandidate(type, ego, blocked_info, opponents,
                     localized_lateral_profile_.active
                         ? &localized_lateral_profile_
                         : nullptr);
}

// 入力: 安全評価済み候補とBlockedInfo。
// 出力: 小さいほど優先されるscore。
// 処理概要: 安全性を最優先にしつつ、状況に応じてPASS/FOLLOW/YIELD/RECOVERYの優先度を調整する。
double
OvertakePlannerCore::candidateScore(const CandidateTrajectory &candidate,
                                    const BlockedInfo &blocked_info) const {
  // 候補の優先順位を単純なコストへ落とし、まず安全性、その後に追い越し意欲を見る。
  if (!candidate.feasible) {
    if (candidate.reject_reason == "wall_margin" && blocked_info.side_by_side &&
        candidate.type == CandidateType::YIELD_BEHIND) {
      return blocked_info.corner_side_by_side ? -90.0 : -40.0;
    }
    if (candidate.reject_reason == "opponent_collision" &&
        blocked_info.side_by_side &&
        candidate.type == CandidateType::SIDE_BY_SIDE_KEEP &&
        !blocked_info.corner_side_by_side) {
      return -30.0;
    }
    if (candidate.reject_reason == "opponent_collision" &&
        candidate.type == CandidateType::YIELD_BEHIND) {
      return blocked_info.corner_side_by_side ? -70.0 : -50.0;
    }
    return 1.0e9;
  }
  double score = 0.0;
  // 処理ブロック: feasible候補の基本優先度を種別ごとに決める。
  // 設計意図: 通常はPASSを取りに行くが、横並びや未来譲りではYIELD/KEEPを優先できるようにする。
  switch (candidate.type) {
  case CandidateType::FASTEST:
    score = blocked_info.blocked ? 50.0 : 0.0;
    break;
  case CandidateType::FOLLOW:
    score = 15.0;
    break;
  case CandidateType::PASS_LEFT:
  case CandidateType::PASS_RIGHT:
    score = blocked_info.side_by_side ? 200.0 : -10.0;
    if (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
        blocked_info.straight_overtake_start_allowed &&
        blocked_info.blocked && !blocked_info.side_by_side &&
        !blocked_info.future_yield_required) {
      score -= 1.0;
    }
    break;
  case CandidateType::RECOVERY:
    score = 40.0;
    break;
  case CandidateType::SIDE_BY_SIDE_KEEP:
    score = blocked_info.side_by_side && !blocked_info.corner_side_by_side &&
                    !blocked_info.future_yield_required
                ? -30.0
                : 80.0;
    break;
  case CandidateType::YIELD_BEHIND:
    if (blocked_info.future_yield_required) {
      score = -80.0;
    } else if (blocked_info.corner_side_by_side) {
      score = -70.0;
    } else if (blocked_info.side_by_side &&
               blocked_info.side_delta_s > config_.side_yield_s_m) {
      score = -60.0;
    } else if (currentPassGapLost(mode_, blocked_info)) {
      score = -50.0;
    } else {
      score = (!blocked_info.can_pass_left && !blocked_info.can_pass_right)
                  ? 5.0
                  : 70.0;
    }
    break;
  case CandidateType::SAFE_STOP:
    score = -120.0;
    break;
  }
  if ((mode_ == BehaviorMode::OVERTAKE_LEFT &&
       candidate.type == CandidateType::PASS_LEFT) ||
      (mode_ == BehaviorMode::OVERTAKE_RIGHT &&
       candidate.type == CandidateType::PASS_RIGHT) ||
      (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
       candidate.type == CandidateType::FOLLOW) ||
      (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP &&
       candidate.type == CandidateType::SIDE_BY_SIDE_KEEP) ||
      (mode_ == BehaviorMode::YIELD_BEHIND &&
       candidate.type == CandidateType::YIELD_BEHIND) ||
      (mode_ == BehaviorMode::SAFE_STOP &&
       candidate.type == CandidateType::SAFE_STOP)) {
    // いまのモードに沿う候補を少し優遇し、左右や追従/追い越しが細かく揺れないようにする。
    score -= config_.keep_mode_bonus;
  }
  return score;
}

// 入力: 開始sとlookahead距離[m]。
// 出力: 区間内の最大絶対曲率[1/m]。
// 処理概要: 追い越し開始gateや横並びコーナー判定のため、参照線曲率を粗くサンプリングする。
double OvertakePlannerCore::maxAbsCurvatureAhead(double s,
                                                 double lookahead_m) const {
  if (frame_.empty() || lookahead_m <= 0.0) {
    return 0.0;
  }
  const int sample_count = 8;
  const double ds = lookahead_m / static_cast<double>(sample_count);
  double max_abs_kappa = 0.0;
  for (int i = 0; i <= sample_count; ++i) {
    const auto ref = frame_.interpolate(s + ds * static_cast<double>(i));
    max_abs_kappa = std::max(max_abs_kappa, std::abs(ref.kappa));
  }
  return max_abs_kappa;
}

// 入力: 現在s。
// 出力: 現在有効なsection safety設定。
// 処理概要: YAMLから読み込んだ区間ルールを探し、壁マージン/速度/外側譲りのスケールを返す。
ActiveSectionSafety OvertakePlannerCore::activeSectionSafety(double s) const {
  ActiveSectionSafety active;
  if (!config_.section_safety_profile_enabled) {
    return active;
  }
  for (const auto &rule : config_.section_safety_rules) {
    if (!sectionContainsS(rule, s)) {
      continue;
    }
    active.active = true;
    active.name = rule.name;
    active.profile = rule.profile.empty() ? "default" : rule.profile;
    active.role_policy =
        rule.role_policy.empty() ? "default" : rule.role_policy;
    if (active.profile == "wall_risk_moderate") {
      active.wall_margin_scale = 1.15;
      active.speed_cap_scale = 0.85;
    } else if (active.profile == "side_by_side_corner_strict") {
      active.wall_margin_scale = 1.35;
      active.speed_cap_scale = 0.65;
      active.force_outer_yield = true;
    }
    if (active.role_policy == "outer_yields") {
      active.force_outer_yield = true;
    }
    return active;
  }
  return active;
}

// 入力: 現在s。
// 出力: lookaheadも考慮した追い越し許可状態。
// 処理概要: 現在地点と前方区間を見て、近い将来の禁止区間へ入る前に追い越し開始を止める。
ActiveOvertakePermission
OvertakePlannerCore::activeOvertakePermission(double s) const {
  ActiveOvertakePermission current = overtakePermissionAtS(s);
  if (!config_.overtake_permission_profile_enabled ||
      config_.overtake_permission_lookahead_m <= 0.0) {
    return current;
  }

  const int sample_count = 8;
  const double ds = config_.overtake_permission_lookahead_m /
                    static_cast<double>(sample_count);
  for (int i = 1; i <= sample_count; ++i) {
    const auto ahead =
        overtakePermissionAtS(s + ds * static_cast<double>(i));
    if (!ahead.allow_overtake) {
      return ahead;
    }
  }
  return current;
}

// 入力: 評価対象s。
// 出力: そのs地点単体の追い越し許可状態。
// 処理概要: 許可CSVから該当区間を探し、見つからなければdefault設定を返す。
ActiveOvertakePermission
OvertakePlannerCore::overtakePermissionAtS(double s) const {
  ActiveOvertakePermission active;
  active.allow_overtake = config_.default_overtake_allowed;
  if (!config_.overtake_permission_profile_enabled) {
    active.allow_overtake = true;
    return active;
  }
  for (const auto &rule : config_.overtake_permission_rules) {
    if (!permissionRuleContainsS(rule, s)) {
      continue;
    }
    active.active = true;
    active.name = rule.name;
    active.allow_overtake = rule.allow_overtake;
    return active;
  }
  return active;
}

// 入力: section safety ruleと評価対象s。
// 出力: sがrule区間内ならtrue。
// 処理概要: 周回境界をまたぐ区間にも対応して、wrap済みsで包含判定する。
bool OvertakePlannerCore::sectionContainsS(const SectionSafetyRule &rule,
                                           double s) const {
  if (frame_.empty()) {
    return false;
  }
  const double start = frame_.wrapS(rule.s_start_m);
  const double end = frame_.wrapS(rule.s_end_m);
  const double wrapped_s = frame_.wrapS(s);
  if (start <= end) {
    return wrapped_s >= start && wrapped_s <= end;
  }
  return wrapped_s >= start || wrapped_s <= end;
}

// 入力: overtake permission ruleと評価対象s。
// 出力: sがrule区間内ならtrue。
// 処理概要: sectionContainsSと同じ閉ループ区間判定を追い越し許可CSVにも使う。
bool OvertakePlannerCore::permissionRuleContainsS(
    const OvertakePermissionRule &rule, double s) const {
  if (frame_.empty()) {
    return false;
  }
  const double start = frame_.wrapS(rule.s_start_m);
  const double end = frame_.wrapS(rule.s_end_m);
  const double wrapped_s = frame_.wrapS(s);
  if (start <= end) {
    return wrapped_s >= start && wrapped_s <= end;
  }
  return wrapped_s >= start || wrapped_s <= end;
}

// 入力: 現在有効なsection safety設定。
// 出力: section scaleを反映したsoft wall margin[m]。
// 処理概要: コーナー厳格区間などで壁に寄る判断を早めに危険扱いする。
double OvertakePlannerCore::effectiveWallSoftMargin(
    const ActiveSectionSafety &section) const {
  const double base = std::max(0.0, config_.wall_soft_margin_m);
  return base * std::max(1.0, section.wall_margin_scale);
}

// 入力: detectBlocked直後のBlockedInfoと相手車一覧。
// 出力: parallel-sideの低速/停止車を前方閉塞へ昇格したならtrue。
// 処理概要: 停止車列を1台ずつ抜く場面で、2台目以降がfrontではなく
// parallel-sideに見えても、既存PASS候補生成と状態機械に載せる。
bool OvertakePlannerCore::promoteSlowObstacleChain(
    BlockedInfo &blocked, const std::vector<OpponentState> &opponents) const {
  if (!config_.slow_obstacle_chain_enabled ||
      !config_.slow_front_exception_enabled || blocked.nearest_index >= 0 ||
      !blocked.parallel_side_candidate || blocked.parallel_side_index < 0 ||
      static_cast<std::size_t>(blocked.parallel_side_index) >=
          opponents.size()) {
    return false;
  }
  if (!std::isfinite(blocked.parallel_side_delta_s) ||
      blocked.parallel_side_delta_s <= 0.0 ||
      blocked.parallel_side_delta_s >
          std::max(0.0, config_.slow_obstacle_chain_distance_m)) {
    return false;
  }
  // 通常時は同じ走行コリドー内だけ昇格する。一方で既に追い越し側へ出ている時は、
  // 2台目以降が横に離れて見えるため、parallel検出幅まで停止車列として扱う。
  const bool pass_chain_context =
      isAnyPassMode(mode_) || localized_lateral_profile_.active ||
      reentry_lockout_active_;
  const double chain_lateral_limit =
      pass_chain_context ? config_.parallel_side_margin_m
                         : config_.same_corridor_width_m;
  if (std::abs(blocked.parallel_side_delta_d) >
          std::max(0.0, chain_lateral_limit) ||
      !std::isfinite(blocked.parallel_side_rel_v) ||
      blocked.parallel_side_rel_v <= 0.0) {
    return false;
  }

  const auto &target =
      opponents[static_cast<std::size_t>(blocked.parallel_side_index)];
  if (!std::isfinite(target.v) ||
      target.v > std::max(0.0, config_.slow_front_exception_speed_mps)) {
    return false;
  }

  blocked.slow_obstacle_chain_active = true;
  blocked.slow_obstacle_chain_id = target.id;
  blocked.slow_obstacle_chain_delta_s = blocked.parallel_side_delta_s;
  blocked.slow_obstacle_chain_delta_d = blocked.parallel_side_delta_d;
  blocked.slow_obstacle_chain_speed_mps = target.v;

  blocked.blocked = true;
  blocked.nearest_index = blocked.parallel_side_index;
  blocked.nearest_id = blocked.parallel_side_id;
  blocked.front_delta_s = blocked.parallel_side_delta_s;
  blocked.front_delta_d = blocked.parallel_side_delta_d;
  blocked.front_rel_v = blocked.parallel_side_rel_v;
  blocked.front_vehicle_speed_mps = target.v;
  blocked.front_s_dot_mps = blocked.parallel_side_s_dot_mps;
  blocked.front_direction_known = blocked.parallel_side_direction_known;
  blocked.front_same_direction = blocked.parallel_side_same_direction;
  return true;
}

// 入力: 現在時刻、自車、前方判定、相手車一覧。
// 出力: なし。停止/ほぼ停止した前方車だけをBlockedInfoへ明示する。
// 処理概要: parallel観測と分離し、fresh・同方向・前方・同一コリドー・有限TTCを満たす場合だけ分類する。
void OvertakePlannerCore::classifyStationaryFrontObstacle(
    double now_sec, const EgoState &ego, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.stationary_front_obstacle = false;
  blocked.stationary_front_id.clear();
  blocked.stationary_front_ttc_sec = std::numeric_limits<double>::infinity();
  if (blocked.nearest_index < 0 ||
      static_cast<std::size_t>(blocked.nearest_index) >= opponents.size() ||
      !std::isfinite(blocked.front_delta_s) || blocked.front_delta_s <= 0.0 ||
      !std::isfinite(blocked.front_delta_d) ||
      std::abs(blocked.front_delta_d) >
          std::max(0.0, config_.same_corridor_width_m) ||
      !std::isfinite(blocked.front_rel_v) || blocked.front_rel_v <= 0.0 ||
      (blocked.front_direction_known && !blocked.front_same_direction)) {
    return;
  }
  const auto &target =
      opponents[static_cast<std::size_t>(blocked.nearest_index)];
  if (!target.valid || !std::isfinite(target.stamp_sec) ||
      now_sec < target.stamp_sec ||
      now_sec - target.stamp_sec > config_.opponent_stale_time_sec ||
      !std::isfinite(target.v) ||
      target.v > std::max(0.0, config_.stationary_obstacle_speed_threshold_mps)) {
    return;
  }
  const double ttc_sec = blocked.front_delta_s / blocked.front_rel_v;
  if (!std::isfinite(ttc_sec) || ttc_sec <= 0.0) {
    return;
  }
  // egoを参照していることを明示し、NaN/負速度が分類を通らないようにする。
  if (!std::isfinite(ego.v) || ego.v < 0.0) {
    return;
  }
  blocked.stationary_front_obstacle = true;
  blocked.stationary_front_id = target.id;
  blocked.stationary_front_ttc_sec = ttc_sec;
}

// 入力: publish直前のPlannerOutput、基準候補、相手予測。
// 出力: 実際にpublishする横offset列が安全ならtrue。
// 処理概要: rate limit/hold後のd列をCartesianへ再構成し、候補評価時と同じ楕円・壁制約で再評価する。
bool OvertakePlannerCore::revalidatePublishedLateral(
    const PlannerOutput &output, const CandidateTrajectory &base_candidate,
    const std::vector<PredictedOpponent> &predictions) const {
  // PASSだけでなく、通常ラインへ戻るRECOVERYもrate limit/hold後の形を再評価する。
  // SAFE_STOP・速度のみfallbackはそれぞれ専用の安全/下流fallback経路を維持する。
  const bool reentry_recovery =
      output.reentry_gate.requested &&
      base_candidate.type == CandidateType::RECOVERY;
  if (!output.active_override ||
      (!isPassCandidate(base_candidate.type) && !reentry_recovery)) {
    return true;
  }
  if (!base_candidate.longitudinal_profile_valid) {
    return false;
  }
  if (output.lateral_offsets.empty() ||
      output.lateral_offsets.size() != base_candidate.s.size()) {
    return false;
  }
  CandidateTrajectory published = base_candidate;
  published.d = output.lateral_offsets;
  published.x.clear();
  published.y.clear();
  published.yaw.clear();
  published.x.reserve(published.s.size());
  published.y.reserve(published.s.size());
  published.yaw.reserve(published.s.size());
  for (std::size_t i = 0; i < published.s.size(); ++i) {
    const auto point = frame_.frenetToCartesian(published.s[i], published.d[i]);
    published.x.push_back(point.x);
    published.y.push_back(point.y);
    published.yaw.push_back(point.yaw);
  }
  return safety_.evaluate(published, predictions);
}

// 入力: 現在のBlockedInfo。
// 出力: 低速前走車例外が有効になったならtrue。
// 処理概要: 追い越し禁止区間でも、前走車が低速で一定周期続いた時だけ開始許可へ戻す。
bool OvertakePlannerCore::updateSlowFrontException(
    const BlockedInfo &blocked) {
  if (!config_.slow_front_exception_enabled || !blocked.front_vehicle_low_speed) {
    slow_front_exception_count_ = 0;
    return false;
  }
  ++slow_front_exception_count_;
  const int required_cycles =
      std::max(1, config_.slow_front_exception_required_cycles);
  return slow_front_exception_count_ >= required_cycles;
}

// 入力: 現在時刻、自車状態、BlockedInfo。
// 出力: スタート直後のsafe stopを抑制するならtrue。
// 処理概要: 低速スタート直後の横並び/未来譲りで、すぐ停止に落ちるのを短時間だけ避ける。
bool OvertakePlannerCore::shouldSuppressSafeStopForStartGrace(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked) const {
  if (!config_.start_grace_safe_stop_enabled ||
      config_.start_grace_duration_sec <= 0.0 ||
      !std::isfinite(first_valid_update_sec_) || !std::isfinite(now_sec) ||
      now_sec < first_valid_update_sec_) {
    return false;
  }
  const double start_grace_reference_sec =
      std::isfinite(first_motion_update_sec_) ? first_motion_update_sec_
                                              : now_sec;
  if (now_sec < start_grace_reference_sec ||
      now_sec - start_grace_reference_sec > config_.start_grace_duration_sec) {
    return false;
  }
  if (config_.start_grace_max_speed_mps >= 0.0 &&
      ego.v > config_.start_grace_max_speed_mps) {
    return false;
  }
  if (blocked.blocked) {
    return false;
  }
  return blocked.side_by_side || blocked.parallel_side_candidate ||
         blocked.future_side_by_side || blocked.future_yield_required;
}

// 入力: BlockedInfoと、先行扱いに必要な相手後方距離margin。
// 出力: 自車が横並び/並走相手より明確に前ならtrue。
// 処理概要: 別の前方車に塞がれていない場面で、対象相手が後ろにいることをs差で判定する。
bool OvertakePlannerCore::leaderPriorityCandidate(
    const BlockedInfo &blocked, double margin_m, std::string &target_id,
    double &target_delta_s, std::string &reason) const {
  if (!config_.side_by_side_leader_priority_enabled) {
    return false;
  }
  const double required_margin = std::max(0.0, margin_m);
  const auto is_leading = [required_margin](double delta_s) {
    return std::isfinite(delta_s) && delta_s <= -required_margin;
  };
  const auto blocked_by_other_front = [&blocked](const std::string &id) {
    return blocked.blocked && !blocked.nearest_id.empty() &&
           blocked.nearest_id != id;
  };
  if (blocked.side_by_side && blocked.side_same_direction &&
      !blocked.side_id.empty() && !blocked_by_other_front(blocked.side_id) &&
      is_leading(blocked.side_delta_s)) {
    target_id = blocked.side_id;
    target_delta_s = blocked.side_delta_s;
    reason = "side_by_side_leader";
    return true;
  }
  if (blocked.parallel_side_candidate && blocked.parallel_side_same_direction &&
      !blocked.parallel_side_id.empty() &&
      !blocked_by_other_front(blocked.parallel_side_id) &&
      is_leading(blocked.parallel_side_delta_s)) {
    target_id = blocked.parallel_side_id;
    target_delta_s = blocked.parallel_side_delta_s;
    reason = "parallel_side_leader";
    return true;
  }
  return false;
}

// 入力: 現在時刻とBlockedInfo。
// 出力: なし。BlockedInfoへleader priority情報を付与する。
// 処理概要: 先行/後続の優先権が毎周期入れ替わらないよう、解除側に小さなヒステリシスと保持時間を持たせる。
void OvertakePlannerCore::updateLeaderPriority(double now_sec,
                                               BlockedInfo &blocked) {
  blocked.leader_priority_active = false;
  blocked.leader_priority_latched = false;
  blocked.leader_priority_id.clear();
  blocked.leader_priority_delta_s = std::numeric_limits<double>::infinity();
  blocked.leader_priority_reason.clear();

  if (!config_.side_by_side_leader_priority_enabled) {
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }

  std::string target_id;
  double target_delta_s = std::numeric_limits<double>::infinity();
  std::string reason;
  bool active = leaderPriorityCandidate(
      blocked, config_.side_by_side_leader_priority_enter_s_m, target_id,
      target_delta_s, reason);
  bool latched = false;

  const bool hold_window_active =
      leader_priority_hold_active_ && std::isfinite(now_sec) &&
      std::isfinite(leader_priority_hold_until_sec_) &&
      now_sec <= leader_priority_hold_until_sec_;
  if (active && hold_window_active && target_id == leader_priority_hold_id_) {
    latched = true;
    reason = "leader_priority_hold";
  }

  if (!active && hold_window_active) {
    const double release_margin = std::min(
        std::max(0.0, config_.side_by_side_leader_priority_enter_s_m),
        std::max(0.0, config_.side_by_side_leader_priority_release_s_m));
    std::string held_id;
    double held_delta_s = std::numeric_limits<double>::infinity();
    std::string held_reason;
    if (leaderPriorityCandidate(blocked, release_margin, held_id, held_delta_s,
                                held_reason) &&
        held_id == leader_priority_hold_id_) {
      active = true;
      latched = true;
      target_id = held_id;
      target_delta_s = held_delta_s;
      reason = "leader_priority_hold";
    }
  }

  if (!active) {
    if (!std::isfinite(now_sec) || !std::isfinite(leader_priority_hold_until_sec_) ||
        now_sec > leader_priority_hold_until_sec_) {
      leader_priority_hold_active_ = false;
      leader_priority_hold_id_.clear();
      leader_priority_hold_until_sec_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    return;
  }

  blocked.leader_priority_active = true;
  blocked.leader_priority_latched = latched;
  blocked.leader_priority_id = target_id;
  blocked.leader_priority_delta_s = target_delta_s;
  blocked.leader_priority_reason = reason;
  leader_priority_hold_active_ = true;
  leader_priority_hold_id_ = target_id;
  leader_priority_hold_until_sec_ =
      now_sec + std::max(0.0, config_.side_by_side_leader_priority_hold_sec);
}

// 入力: なし。
// 出力: 局所横プロファイルモードが有効ならtrue。
// 処理概要: legacyとlocalized_latchedを切り替え、実験機能をパラメータで隔離する。
bool OvertakePlannerCore::localizedLateralProfileEnabled() const {
  return config_.overtake_lateral_profile_mode == "localized_latched";
}

// 入力: 現在のBlockedInfo。
// 出力: 優先するPASS候補種別。使えない場合はFASTEST。
// 処理概要: 既に追い越し中の方向を優先し、未開始なら通れる側を選ぶ。
CandidateType
OvertakePlannerCore::preferredPassType(const BlockedInfo &blocked) const {
  if (isLeftPassMode(mode_)) {
    return CandidateType::PASS_LEFT;
  }
  if (isRightPassMode(mode_)) {
    return CandidateType::PASS_RIGHT;
  }
  if (blocked.can_pass_left) {
    return CandidateType::PASS_LEFT;
  }
  if (blocked.can_pass_right) {
    return CandidateType::PASS_RIGHT;
  }
  if (blocked.blocked && blocked.straight_overtake_start_allowed &&
      blocked.overtake_permission_allowed) {
    // 静的gapが両側とも狭くても、候補安全評価用の局所プロファイルは広い診断側へ固定する。
    return blocked.left_pass_gap_m >= blocked.right_pass_gap_m
               ? CandidateType::PASS_LEFT
               : CandidateType::PASS_RIGHT;
  }
  return CandidateType::FASTEST;
}

// 入力: 現在のBlockedInfo。
// 出力: 局所回避プロファイルの対象相手index。無ければ-1。
// 処理概要: 前方閉塞車両、横並び車両、並走候補の順で対象を選ぶ。
int OvertakePlannerCore::localizedProfileTargetIndex(
    const BlockedInfo &blocked) const {
  if (blocked.nearest_index >= 0) {
    return blocked.nearest_index;
  }
  if (blocked.side_index >= 0) {
    return blocked.side_index;
  }
  return blocked.parallel_side_index;
}

// 入力: 現在時刻、自車状態、BlockedInfo、相手車一覧。
// 出力: なし。localized_lateral_profile_を更新またはクリアする。
// 処理概要: PASS文脈で対象車両と目標sをラッチし、短周期の相手位置揺れで横ラインが揺れないようにする。
void OvertakePlannerCore::updateLocalizedLateralProfile(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) {
  if (!localizedLateralProfileEnabled()) {
    clearLocalizedLateralProfile();
    return;
  }
  if (!blocked.blocked && !blocked.side_by_side && !isPassMode(mode_)) {
    // 広いparallel観測だけではPASS文脈を作らず、局所回避プロファイルもラッチしない。
    clearLocalizedLateralProfile();
    return;
  }

  const CandidateType pass_type = preferredPassType(blocked);
  const int target_index = localizedProfileTargetIndex(blocked);
  const bool target_valid =
      target_index >= 0 &&
      static_cast<std::size_t>(target_index) < opponents.size();
  const bool pass_context =
      pass_type == CandidateType::PASS_LEFT ||
      pass_type == CandidateType::PASS_RIGHT || isPassMode(mode_);
  const bool profile_min_hold_active =
      localized_lateral_profile_.active &&
      std::isfinite(localized_lateral_profile_.created_time_sec) &&
      now_sec - localized_lateral_profile_.created_time_sec <
          std::max(0.0, config_.maneuver_latch_min_hold_sec);

  if (!target_valid || (!pass_context && !profile_min_hold_active)) {
    clearLocalizedLateralProfile();
    return;
  }

  const auto &target = opponents[static_cast<std::size_t>(target_index)];
  if (localized_lateral_profile_.active) {
    const bool same_target =
        target.id == localized_lateral_profile_.target_id;
    const bool same_direction =
        pass_type == localized_lateral_profile_.pass_type ||
        pass_type == CandidateType::FASTEST;
    if ((!same_target || !same_direction) && !profile_min_hold_active) {
      clearLocalizedLateralProfile();
    }
  }

  if (!localized_lateral_profile_.active) {
    // 処理ブロック: 新規ラッチを作る。
    // 設計意図: 開始時の自車dと対象sを基準にし、以後の候補生成で同じ回避区間を使う。
    if (pass_type != CandidateType::PASS_LEFT &&
        pass_type != CandidateType::PASS_RIGHT) {
      return;
    }
    localized_lateral_profile_.active = true;
    localized_lateral_profile_.pass_type = pass_type;
    localized_lateral_profile_.target_id = target.id;
    localized_lateral_profile_.created_time_sec = now_sec;
    localized_lateral_profile_.anchor_s_m = ego.frenet.s;
    localized_lateral_profile_.start_d_m = ego.frenet.d;
    localized_lateral_profile_.target_d_m = targetOffsetForPass(pass_type);
    const double target_s_m =
        localized_lateral_profile_.anchor_s_m +
        frame_.deltaS(localized_lateral_profile_.anchor_s_m, target.frenet.s);
    setLocalizedProfileMarkers(target_s_m);
    return;
  }

  const double alpha =
      std::clamp(config_.maneuver_latch_target_update_alpha, 0.0, 1.0);
  if (alpha <= 0.0 || target.id != localized_lateral_profile_.target_id) {
    return;
  }
  // 処理ブロック: 任意設定時だけ対象sを低域更新する。
  // 設計意図: 完全固定が強すぎる場合でも、急変ではなくalphaで滑らかに追従させる。
  const double measured_target_s_m =
      localized_lateral_profile_.anchor_s_m +
      frame_.deltaS(localized_lateral_profile_.anchor_s_m, target.frenet.s);
  const double updated_target_s_m =
      localized_lateral_profile_.target_s_m * (1.0 - alpha) +
      measured_target_s_m * alpha;
  setLocalizedProfileMarkers(updated_target_s_m);
}

// 入力: なし。
// 出力: なし。局所横プロファイルを無効状態へ戻す。
// 処理概要: PASS文脈が切れた時に、古い対象車両の回避区間を次回へ持ち越さない。
void OvertakePlannerCore::clearLocalizedLateralProfile() {
  localized_lateral_profile_ = LocalizedLateralProfile{};
}

// 入力: ラッチ対象のunwrapped target_s。
// 出力: なし。局所プロファイルの開始/保持/マージmarkerを更新する。
// 処理概要: 対象車両の前から避け始め、通過後に一定距離保持してから中心へ戻す区間を作る。
void OvertakePlannerCore::setLocalizedProfileMarkers(double target_s_m) {
  localized_lateral_profile_.target_s_m = target_s_m;
  const double start_before =
      std::max(0.0, config_.localized_avoidance_start_before_target_m);
  const double full_before =
      std::max(0.0, config_.localized_avoidance_full_offset_before_target_m);
  const double hold_after =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  localized_lateral_profile_.avoid_start_s_m =
      target_s_m - std::max(start_before, full_before);
  localized_lateral_profile_.full_offset_start_s_m =
      target_s_m - std::min(start_before, full_before);
  localized_lateral_profile_.full_offset_end_s_m = target_s_m + hold_after;
  localized_lateral_profile_.merge_end_s_m =
      localized_lateral_profile_.full_offset_end_s_m + merge_distance;
}

// 入力: PASS_LEFT/PASS_RIGHT。
// 出力: そのPASS方向の目標横オフセットd。
// 処理概要: CandidateBuilder以外でもPASS方向から目標dを参照できるようにする。
double OvertakePlannerCore::targetOffsetForPass(CandidateType pass_type) const {
  if (pass_type == CandidateType::PASS_LEFT) {
    return config_.left_offset_m;
  }
  if (pass_type == CandidateType::PASS_RIGHT) {
    return config_.right_offset_m;
  }
  return 0.0;
}

// 入力: 自車状態とBlockedInfo。
// 出力: 横並び時に後方へ譲るべきならtrue。
// 処理概要: 未来譲り、相手が前寄り、コーナー壁余裕不足をまとめてYIELD_BEHINDへ誘導する。
bool OvertakePlannerCore::shouldYieldBehindSideBySide(
    const EgoState &ego, const BlockedInfo &blocked_info) const {
  if (blocked_info.future_yield_required) {
    return true;
  }
  if (!blocked_info.side_by_side) {
    return false;
  }
  if (blocked_info.side_delta_s > config_.side_yield_s_m) {
    return true;
  }
  if (!blocked_info.corner_side_by_side) {
    return false;
  }
  const bool opponent_not_clearly_behind =
      blocked_info.side_delta_s > -config_.side_yield_s_m;
  const bool close_to_wall = blocked_risk_.wallClearance(ego.frenet.d) <=
                             config_.corner_side_yield_wall_clearance_m;
  return opponent_not_clearly_behind || close_to_wall;
}

// 入力: 現在時刻、自車状態、最終PlannerOutput。
// 出力: なし。必要ならoutput.lateral_offsetsを保持済み値へ置き換える。
// 処理概要: 高速カーブ中の復帰/譲り/速度guardで横参照が毎周期揺れないようにholdする。
void OvertakePlannerCore::applyHighSpeedCurveLateralHold(
    double now_sec, const EgoState &ego, PlannerOutput &output) {
  output.lateral_target_hold_active = false;
  output.lateral_target_hold_reason.clear();

  if (output.reentry_gate.requested && !output.reentry_gate.permitted) {
    // 以前のカーブhold列が中心方向を向いていても、復帰ゲート閉鎖中の
    // 現在d保持候補を上書きしてはならない。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }

  if (!config_.high_speed_curve_lateral_hold_enabled) {
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const bool usable_output =
      output.active_override && !output.lateral_offsets.empty();
  const bool stabilized_mode = isLateralStabilizationMode(output.mode);
  const double curvature =
      std::max(output.blocked_info.corner_abs_curvature,
               output.blocked_info.future_abs_curvature);
  const double enter_curvature = highSpeedCurveHoldEnterCurvature(config_);
  const double release_curvature = highSpeedCurveHoldReleaseCurvature(config_);
  const double enter_speed = std::max(
      0.0, config_.high_speed_curve_lateral_hold_min_speed_mps);
  const double release_speed =
      config_.high_speed_curve_lateral_hold_release_speed_mps >= 0.0
          ? config_.high_speed_curve_lateral_hold_release_speed_mps
          : enter_speed;

  if (high_speed_curve_lateral_hold_active_) {
    // 処理ブロック: hold中は解除条件を満たすまで前回offset列を再利用する。
    // 設計意図: 横参照の再計算でMPC入力が急に変わることを避ける。
    const bool release = !usable_output || !stabilized_mode ||
                         ego.v <= release_speed ||
                         curvature <= release_curvature;
    if (release) {
      high_speed_curve_lateral_hold_active_ = false;
      high_speed_curve_lateral_hold_offsets_.clear();
      high_speed_curve_lateral_hold_sec_ =
          std::numeric_limits<double>::quiet_NaN();
    } else if (!high_speed_curve_lateral_hold_offsets_.empty()) {
      for (std::size_t i = 0; i < output.lateral_offsets.size(); ++i) {
        output.lateral_offsets[i] =
            heldOffsetAt(high_speed_curve_lateral_hold_offsets_, i);
      }
      output.target_lateral_offset_m = output.lateral_offsets.back();
      output.lateral_target_hold_active = true;
      output.lateral_target_hold_reason = "high_speed_curve_hold";
      return;
    }
  }

  const bool should_enter =
      usable_output && stabilized_mode &&
      hasLateralStabilizationRisk(output) && ego.v >= enter_speed &&
      curvature >= enter_curvature;
  if (!should_enter) {
    return;
  }

  // 処理ブロック: holdへ入る瞬間のoffset列を保存する。
  // 設計意図: 以後の周期ではこの列を基準にし、高速カーブが落ち着くまで横目標を固定する。
  high_speed_curve_lateral_hold_active_ = true;
  high_speed_curve_lateral_hold_offsets_ = output.lateral_offsets;
  high_speed_curve_lateral_hold_sec_ = now_sec;
  output.lateral_target_hold_active = true;
  output.lateral_target_hold_reason = "high_speed_curve_hold";
}

// 入力: 現在時刻とPlannerOutput。
// 出力: なし。横オフセット列を前回publish値からの最大変化量以内に制限する。
// 処理概要: publish周期ごとのtarget d急変を抑え、MPCへ滑らかな参照を渡す。
void OvertakePlannerCore::applyLateralTargetRateLimit(double now_sec,
                                                      PlannerOutput &output) {
  if (output.reentry_gate.requested && !output.reentry_gate.permitted) {
    // 前周期の通常ライン向けoffsetを混ぜると、hold中でも横方向に進む。
    // ゲート拒否時のRECOVERYはCandidateBuilderが生成した現在d保持列をそのまま使う。
    return;
  }
  if (!output.active_override || output.lateral_offsets.empty() ||
      config_.lateral_target_max_step_m <= 0.0 ||
      last_published_lateral_offsets_.empty() ||
      !std::isfinite(last_published_lateral_target_sec_) ||
      now_sec - last_published_lateral_target_sec_ >
          kPublishedLateralTargetMemorySec) {
    return;
  }

  const double max_step = config_.lateral_target_max_step_m;
  for (std::size_t i = 0; i < output.lateral_offsets.size(); ++i) {
    const std::size_t prev_i =
        std::min(i, last_published_lateral_offsets_.size() - 1);
    const double prev = last_published_lateral_offsets_[prev_i];
    double &current = output.lateral_offsets[i];
    if (!std::isfinite(prev) || !std::isfinite(current)) {
      continue;
    }
    current = prev + std::clamp(current - prev, -max_step, max_step);
  }
  output.target_lateral_offset_m = output.lateral_offsets.back();
}

// 入力: 現在時刻とPlannerOutput。
// 出力: なし。次周期のrate limit/hold用に最後の横オフセット列を記憶する。
// 処理概要: overrideが途切れて一定時間経ったら古い記憶を破棄し、無関係な制限を残さない。
void OvertakePlannerCore::rememberPublishedLateralTarget(
    double now_sec, const PlannerOutput &output) {
  if (output.active_override && !output.lateral_offsets.empty()) {
    last_published_lateral_offsets_ = output.lateral_offsets;
    last_published_lateral_target_sec_ = now_sec;
    return;
  }
  if (std::isfinite(last_published_lateral_target_sec_) &&
      now_sec - last_published_lateral_target_sec_ >
          kPublishedLateralTargetMemorySec) {
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
  }
}

// 入力: scoreとfeasibleが設定済みの候補集合。
// 出力: publish/状態機械へ渡す暫定選択候補。
// 処理概要: feasible候補を最優先し、全候補unsafeの場合だけ最小scoreのunsafe候補を診断用に返す。
CandidateTrajectory OvertakePlannerCore::selectCandidate(
    std::vector<CandidateTrajectory> &candidates) const {
  // feasible候補があるなら必ずそれを優先する。unsafe候補は全候補がunsafeの時だけ診断用に返す。
  const auto by_score = [](const CandidateTrajectory &a,
                           const CandidateTrajectory &b) {
    return a.score < b.score;
  };
  auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [&by_score](const CandidateTrajectory &a, const CandidateTrajectory &b) {
        if (a.feasible != b.feasible) {
          return a.feasible;
        }
        return by_score(a, b);
      });
  if (best == candidates.end()) {
    return {};
  }
  if (best->feasible) {
    return *best;
  }

  best = std::min_element(candidates.begin(), candidates.end(), by_score);
  if (best == candidates.end()) {
    return {};
  }
  return *best;
}

} // namespace overtake_planner
