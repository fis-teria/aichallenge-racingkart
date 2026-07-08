#include "overtake_planner/overtake_planner_core.hpp"

#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/planner_output_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace overtake_planner {

namespace {

bool isLeftPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_LEFT;
}

bool isRightPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

bool currentPassGapLost(BehaviorMode mode, const BlockedInfo &blocked_info) {
  return (isLeftPassMode(mode) && !blocked_info.can_pass_left) ||
         (isRightPassMode(mode) && !blocked_info.can_pass_right);
}

bool isPassCandidate(CandidateType type) {
  return type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
}

bool isFallbackCandidate(CandidateType type) {
  return type == CandidateType::FOLLOW || type == CandidateType::YIELD_BEHIND ||
         type == CandidateType::RECOVERY ||
         type == CandidateType::SIDE_BY_SIDE_KEEP;
}

bool isLateralStabilizationMode(BehaviorMode mode) {
  return mode == BehaviorMode::ABORT_RECOVERY ||
         mode == BehaviorMode::YIELD_BEHIND ||
         mode == BehaviorMode::SAFE_STOP || mode == BehaviorMode::SPEED_GUARD;
}

bool hasLateralStabilizationRisk(const PlannerOutput &output) {
  const auto &blocked = output.blocked_info;
  return blocked.corner_side_by_side || blocked.future_yield_required ||
         blocked.future_corner_side_by_side ||
         blocked.future_outer_wall_risk ||
         output.speed_only_fallback_active ||
         output.wall_risk_speed_guard_active ||
         output.mpc_health_speed_guard_active ||
         output.recovery_speed_guard_active;
}

double highSpeedCurveHoldEnterCurvature(const PlannerConfig &config) {
  if (config.corner_side_yield_curvature_m_inv > 0.0) {
    return config.corner_side_yield_curvature_m_inv;
  }
  return std::max(0.0, config.straight_overtake_max_curvature_m_inv);
}

double highSpeedCurveHoldReleaseCurvature(const PlannerConfig &config) {
  if (config.high_speed_curve_lateral_hold_release_curvature_m_inv >= 0.0) {
    return config.high_speed_curve_lateral_hold_release_curvature_m_inv;
  }
  return highSpeedCurveHoldEnterCurvature(config) * 0.5;
}

double heldOffsetAt(const std::vector<double> &offsets, std::size_t index) {
  if (offsets.empty()) {
    return 0.0;
  }
  return offsets[std::min(index, offsets.size() - 1)];
}

bool hasFeasibleCandidate(const std::vector<CandidateTrajectory> &candidates,
                          bool (*predicate)(CandidateType)) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [predicate](const CandidateTrajectory &candidate) {
                       return candidate.feasible && predicate(candidate.type);
                     });
}

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

} // namespace

OvertakePlannerCore::OvertakePlannerCore(FrenetFrame frame,
                                         PlannerConfig config)
    : frame_(std::move(frame)), config_(config), blocked_risk_(frame_, config_),
      future_side_risk_(frame_, config_, blocked_risk_), safety_(config),
      state_machine_(config) {}

PlannerOutput
OvertakePlannerCore::update(double now_sec, const EgoState &ego,
                            const std::vector<OpponentState> &opponents,
                            const MpcHealthStatus &mpc_health) {
  // デフォルトはMPCの元参照をそのまま使う。安全に判断できる時だけoverrideを有効化する。
  PlannerOutput output;
  output.lateral_offsets.assign(config_.horizon_points, 0.0);
  output.speed_caps.assign(config_.horizon_points, config_.v_passthrough_mps);

  if (!config_.enabled || !ego.valid || frame_.empty()) {
    // 自車状態や参照線が無いとFrenet判断ができないので、何も介入しない。
    mode_ = BehaviorMode::FREE_RUN;
    safe_stop_trigger_count_ = 0;
    slow_front_exception_count_ = 0;
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

  const ActiveSectionSafety active_section = activeSectionSafety(ego.frenet.s);
  const ActiveOvertakePermission active_overtake_permission =
      activeOvertakePermission(ego.frenet.s);
  const double wall_soft_margin = effectiveWallSoftMargin(active_section);
  BlockedInfo blocked = blocked_risk_.detectBlocked(ego, opponents, now_sec);
  const auto predictions = predictOpponents(opponents, now_sec);
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
  blocked.front_vehicle_low_speed =
      config_.slow_front_exception_enabled && blocked.nearest_index >= 0 &&
      std::isfinite(blocked.front_vehicle_speed_mps) &&
      blocked.front_vehicle_speed_mps <=
          std::max(0.0, config_.slow_front_exception_speed_mps) &&
      blocked.front_delta_s <=
          std::max(0.0, config_.slow_front_exception_distance_m);
  blocked.slow_front_exception_active = updateSlowFrontException(blocked);
  blocked.slow_front_exception_count = slow_front_exception_count_;
  if (!blocked.overtake_permission_allowed &&
      blocked.slow_front_exception_active) {
    blocked.overtake_permission_reason = "slow_front_exception";
  }
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
  const bool permission_start_allowed =
      blocked.overtake_permission_allowed ||
      blocked.slow_front_exception_active;
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
  const bool large_lateral_error =
      config_.large_lateral_error_threshold_m >= 0.0 &&
      std::abs(ego.frenet.d) > config_.large_lateral_error_threshold_m;
  const bool freeze_overtake_decisions =
      large_lateral_error &&
      (blocked.ego_wall_clearance_m < wall_soft_margin || blocked.blocked ||
       blocked.side_by_side || blocked.parallel_side_candidate ||
       blocked.future_side_by_side || blocked.future_yield_required ||
       currentPassGapLost(mode_, blocked));
  if (freeze_overtake_decisions) {
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
    if (blocked.pass_gap_reason.empty() || blocked.pass_gap_reason == "ok") {
      blocked.pass_gap_reason = "large_lateral_error";
    }
  }
  updateLocalizedLateralProfile(now_sec, ego, blocked, opponents);

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
      if (blocked.can_pass_left) {
        candidates.push_back(
            makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
      }
      if (blocked.can_pass_right) {
        candidates.push_back(
            makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
      }
      if (!blocked.can_pass_left && !blocked.can_pass_right) {
        candidates.push_back(makeCandidate(CandidateType::YIELD_BEHIND, ego,
                                           blocked, opponents));
      }
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
  const bool needs_safe_stop_fallback_check =
      config_.safe_stop_enabled &&
      (blocked.blocked || blocked.side_by_side ||
       blocked.future_yield_required || currentPassGapLost(mode_, blocked));
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

  for (auto &candidate : candidates) {
    // 壁/他車との安全余裕を見てから、目的に応じたスコアを付ける。
    safety_.evaluate(candidate, predictions);
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
       blocked.future_yield_required || currentPassGapLost(mode_, blocked)) &&
      !blocked.can_pass_left && !blocked.can_pass_right && no_feasible_pass &&
      no_feasible_fallback;
  const bool start_grace_active =
      safe_stop_base_condition &&
      shouldSuppressSafeStopForStartGrace(now_sec, ego, blocked);
  const bool effective_safe_stop_base_condition =
      safe_stop_base_condition && !start_grace_active;

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
  if (mode_ == BehaviorMode::SAFE_STOP) {
    selected = makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::ABORT_RECOVERY) {
    // 中止時は必ず中心線へ戻す候補を再生成し、最新予測で安全評価する。
    selected = makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::MERGE_BACK) {
    selected = makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP) {
    selected = makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego, blocked,
                             opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::YIELD_BEHIND) {
    selected =
        makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
             selected.type != CandidateType::FOLLOW) {
    // 追従モードでは速度上限だけを落とすFOLLOW候補を優先する。
    selected = makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
    if (!selected.feasible) {
      selected =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      safety_.evaluate(selected, predictions);
    }
  } else if (mode_ == BehaviorMode::FREE_RUN) {
    selected = makeCandidate(CandidateType::FASTEST, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  }

  // ROSノードがMPC overrideとdebug
  // JSONを作れるよう、選択結果を平坦な出力に詰める。
  auto output_built =
      PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
          mode_, ego, selected, blocked, safe_stop_context, safe_stop_candidate,
          safe_stop_candidate_infeasible, safe_stop_trigger_count_,
          state_machine_.safeStopHoldCount(),
          state_machine_.safeStopReleaseCount(), wall_soft_margin,
          active_section, mpc_health});
  output_built.start_grace_active = start_grace_active;
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
  }
  applyLateralTargetRateLimit(now_sec, output_built);
  applyHighSpeedCurveLateralHold(now_sec, ego, output_built);
  rememberPublishedLateralTarget(now_sec, output_built);
  return output_built;
}

std::vector<PredictedOpponent> OvertakePlannerCore::predictOpponents(
    const std::vector<OpponentState> &opponents, double now_sec) const {
  // V2X位置から推定した速度を使い、短いhorizonでは等速直線運動として予測する。
  std::vector<PredictedOpponent> out;
  for (const auto &opp : opponents) {
    if (!opp.valid ||
        now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
      continue;
    }
    PredictedOpponent pred;
    pred.id = opp.id;
    for (std::size_t i = 0; i < config_.horizon_points; ++i) {
      const double t = static_cast<double>(i) * config_.horizon_dt_sec;
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

CandidateTrajectory OvertakePlannerCore::makeCandidate(
    CandidateType type, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  return CandidateBuilder(frame_, config_)
      .makeCandidate(type, ego, blocked_info, opponents,
                     localized_lateral_profile_.active
                         ? &localized_lateral_profile_
                         : nullptr);
}

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

double OvertakePlannerCore::effectiveWallSoftMargin(
    const ActiveSectionSafety &section) const {
  const double base = std::max(0.0, config_.wall_soft_margin_m);
  return base * std::max(1.0, section.wall_margin_scale);
}

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

bool OvertakePlannerCore::shouldSuppressSafeStopForStartGrace(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked) const {
  if (!config_.start_grace_safe_stop_enabled ||
      config_.start_grace_duration_sec <= 0.0 ||
      !std::isfinite(first_valid_update_sec_) ||
      !std::isfinite(now_sec) || now_sec < first_valid_update_sec_) {
    return false;
  }
  if (now_sec - first_valid_update_sec_ >
      config_.start_grace_duration_sec) {
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

bool OvertakePlannerCore::localizedLateralProfileEnabled() const {
  return config_.overtake_lateral_profile_mode == "localized_latched";
}

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
  return CandidateType::FASTEST;
}

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

void OvertakePlannerCore::updateLocalizedLateralProfile(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) {
  if (!localizedLateralProfileEnabled()) {
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
  const double measured_target_s_m =
      localized_lateral_profile_.anchor_s_m +
      frame_.deltaS(localized_lateral_profile_.anchor_s_m, target.frenet.s);
  const double updated_target_s_m =
      localized_lateral_profile_.target_s_m * (1.0 - alpha) +
      measured_target_s_m * alpha;
  setLocalizedProfileMarkers(updated_target_s_m);
}

void OvertakePlannerCore::clearLocalizedLateralProfile() {
  localized_lateral_profile_ = LocalizedLateralProfile{};
}

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

double OvertakePlannerCore::targetOffsetForPass(CandidateType pass_type) const {
  if (pass_type == CandidateType::PASS_LEFT) {
    return config_.left_offset_m;
  }
  if (pass_type == CandidateType::PASS_RIGHT) {
    return config_.right_offset_m;
  }
  return 0.0;
}

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

void OvertakePlannerCore::applyHighSpeedCurveLateralHold(
    double now_sec, const EgoState &ego, PlannerOutput &output) {
  output.lateral_target_hold_active = false;
  output.lateral_target_hold_reason.clear();

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

  high_speed_curve_lateral_hold_active_ = true;
  high_speed_curve_lateral_hold_offsets_ = output.lateral_offsets;
  high_speed_curve_lateral_hold_sec_ = now_sec;
  output.lateral_target_hold_active = true;
  output.lateral_target_hold_reason = "high_speed_curve_hold";
}

void OvertakePlannerCore::applyLateralTargetRateLimit(double now_sec,
                                                      PlannerOutput &output) {
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
