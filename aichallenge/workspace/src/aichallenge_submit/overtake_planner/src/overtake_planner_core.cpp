#include "overtake_planner/overtake_planner_core.hpp"

#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/planner_output_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace overtake_planner {

namespace {

double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

constexpr double kSideDirectionEpsilon = 0.05;

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

double signedDeltaS(double ego_s, double other_s, double track_length) {
  double signed_delta_s = other_s - ego_s;
  if (track_length > 0.0) {
    signed_delta_s = std::fmod(signed_delta_s, track_length);
    if (signed_delta_s > track_length * 0.5) {
      signed_delta_s -= track_length;
    } else if (signed_delta_s < -track_length * 0.5) {
      signed_delta_s += track_length;
    }
  }
  return signed_delta_s;
}

int sideRiskIndex(const BlockedInfo &info) {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

int yieldTargetIndex(const BlockedInfo &info) {
  return info.nearest_index >= 0 ? info.nearest_index : sideRiskIndex(info);
}

} // namespace

OvertakePlannerCore::OvertakePlannerCore(FrenetFrame frame,
                                         PlannerConfig config)
    : frame_(std::move(frame)), config_(config), safety_(config),
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
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }

  const ActiveSectionSafety active_section = activeSectionSafety(ego.frenet.s);
  const double wall_soft_margin = effectiveWallSoftMargin(active_section);
  BlockedInfo blocked = detectBlocked(ego, opponents, now_sec);
  const auto predictions = predictOpponents(opponents, now_sec);
  blocked = evaluatePassGap(blocked, opponents, predictions);
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
        0.0, close_threshold -
                 std::max(0.0,
                          config_.straight_overtake_release_hysteresis_m_inv));
    if (straight_overtake_start_allowed_) {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= close_threshold;
    } else {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= open_threshold;
    }
    blocked.straight_overtake_start_allowed =
        straight_overtake_start_allowed_;
    if (!blocked.straight_overtake_start_allowed) {
      blocked.overtake_start_gate_reason = "curve";
    }
  } else {
    straight_overtake_start_allowed_ = true;
  }
  blocked.ego_wall_clearance_m = wallClearance(ego.frenet.d);
  blocked = evaluateFutureSideBySideRisk(ego, blocked, opponents);
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

  if (safe_stop_base_condition) {
    ++safe_stop_trigger_count_;
  } else {
    safe_stop_trigger_count_ = 0;
  }

  const int safe_stop_trigger_cycles_required =
      std::max(1, config_.safe_stop_trigger_cycles);
  SafeStopContext safe_stop_context;
  safe_stop_context.requested =
      safe_stop_base_condition &&
      safe_stop_trigger_count_ >= safe_stop_trigger_cycles_required;
  safe_stop_context.trigger_count = safe_stop_trigger_count_;
  safe_stop_context.ego_speed_mps = ego.v;
  const double safe_stop_target_d =
      std::clamp(ego.frenet.d, config_.d_min_m + config_.min_wall_margin_m,
                 config_.d_max_m - config_.min_wall_margin_m);
  safe_stop_context.lateral_error_m =
      std::abs(ego.frenet.d - safe_stop_target_d);

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

  // ROSノードがMPC overrideとdebug JSONを作れるよう、選択結果を平坦な出力に詰める。
  return PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
      mode_,
      ego,
      selected,
      blocked,
      safe_stop_context,
      safe_stop_candidate,
      safe_stop_candidate_infeasible,
      safe_stop_trigger_count_,
      state_machine_.safeStopHoldCount(),
      state_machine_.safeStopReleaseCount(),
      wall_soft_margin,
      active_section,
      mpc_health});
}

BlockedInfo
OvertakePlannerCore::detectBlocked(const EgoState &ego,
                                   const std::vector<OpponentState> &opponents,
                                   double now_sec) const {
  // 同一コリドー内の最も近い前方車両を探し、速度差/距離から閉塞を判断する。
  BlockedInfo info;
  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto &opp = opponents[i];
    if (!opp.valid ||
        now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, opp.frenet.s);
    const double signed_delta_s =
        signedDeltaS(ego.frenet.s, opp.frenet.s, frame_.length());
    const double s_dot = opponentSDot(opp);
    const bool direction_known = opp.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known || s_dot >= config_.same_direction_min_s_dot_mps;
    if (config_.same_direction_filter_enabled && direction_known &&
        !same_direction) {
      ++info.ignored_opposite_direction_count;
      continue;
    }
    const double delta_d = opp.frenet.d - ego.frenet.d;
    const bool front = delta_s > 0.0 && delta_s < config_.lookahead_s_m;
    const bool same_corridor =
        std::abs(delta_d) < config_.same_corridor_width_m;
    const bool side_by_side =
        std::abs(signed_delta_s) < config_.side_by_side_s_m &&
        std::abs(delta_d) < config_.side_margin_m;
    const bool parallel_side_candidate =
        config_.parallel_side_detection_enabled && !side_by_side &&
        config_.parallel_side_s_m > 0.0 &&
        config_.parallel_side_margin_m > 0.0 &&
        std::abs(signed_delta_s) < config_.parallel_side_s_m &&
        std::abs(delta_d) < config_.parallel_side_margin_m;
    if (side_by_side) {
      // 横並び中は不用意なmerge backを避けるため、状態機械へ明示的に伝える。
      info.side_by_side = true;
      if (info.side_index < 0 ||
          std::abs(signed_delta_s) < std::abs(info.side_delta_s)) {
        info.side_index = static_cast<int>(i);
        info.side_id = opp.id;
        info.side_delta_s = signed_delta_s;
        info.side_delta_d = delta_d;
        info.side_rel_v = ego.v - opp.v;
        info.side_s_dot_mps = s_dot;
        info.side_direction_known = direction_known;
        info.side_same_direction = same_direction;
      }
    }
    if (parallel_side_candidate) {
      // スタート直後の別ライン並走など、通常の横並び幅より広いがコーナーで収束し得る相手。
      info.parallel_side_candidate = true;
      if (info.parallel_side_index < 0 ||
          std::abs(signed_delta_s) < std::abs(info.parallel_side_delta_s)) {
        info.parallel_side_index = static_cast<int>(i);
        info.parallel_side_id = opp.id;
        info.parallel_side_delta_s = signed_delta_s;
        info.parallel_side_delta_d = delta_d;
        info.parallel_side_rel_v = ego.v - opp.v;
        info.parallel_side_s_dot_mps = s_dot;
        info.parallel_side_direction_known = direction_known;
        info.parallel_side_same_direction = same_direction;
      }
    }
    if (!front || !same_corridor) {
      continue;
    }
    if (delta_s < info.front_delta_s) {
      info.nearest_index = static_cast<int>(i);
      info.nearest_id = opp.id;
      info.front_delta_s = delta_s;
      info.front_delta_d = delta_d;
      info.front_rel_v = ego.v - opp.v;
      info.front_s_dot_mps = s_dot;
      info.front_direction_known = direction_known;
      info.front_same_direction = same_direction;
    }
  }

  if (info.nearest_index >= 0) {
    // 近い、または相対速度で詰まりつつある前走車を「blocked」と扱う。
    const bool closing = info.front_rel_v > config_.dv_block_threshold_mps;
    const bool slow_gap = info.front_delta_s < config_.follow_trigger_s_m;
    info.blocked = closing || slow_gap;
  }
  return info;
}

BlockedInfo OvertakePlannerCore::evaluatePassGap(
    const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const std::vector<PredictedOpponent> &predictions) const {
  BlockedInfo out = blocked_info;
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  const double ellipse_gap =
      config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h);
  out.pass_gap_required_m = std::max(config_.min_pass_gap_m, ellipse_gap);

  const int target_index = yieldTargetIndex(out);
  if (target_index < 0 ||
      static_cast<std::size_t>(target_index) >= opponents.size()) {
    out.left_pass_gap_m = 0.0;
    out.right_pass_gap_m = 0.0;
    out.can_pass_left = false;
    out.can_pass_right = false;
    out.pass_gap_reason = "no_target";
    return out;
  }

  const auto &target = opponents[static_cast<std::size_t>(target_index)];
  double min_left_gap = upper_d - target.frenet.d;
  double min_right_gap = target.frenet.d - lower_d;

  for (const auto &pred : predictions) {
    if (pred.id != target.id) {
      continue;
    }
    for (double d : pred.d) {
      min_left_gap = std::min(min_left_gap, upper_d - d);
      min_right_gap = std::min(min_right_gap, d - lower_d);
    }
    break;
  }

  out.left_pass_gap_m = min_left_gap;
  out.right_pass_gap_m = min_right_gap;
  const double left_threshold =
      out.pass_gap_required_m -
      (isLeftPassMode(mode_) ? config_.pass_gap_hysteresis_m : 0.0);
  const double right_threshold =
      out.pass_gap_required_m -
      (isRightPassMode(mode_) ? config_.pass_gap_hysteresis_m : 0.0);
  out.can_pass_left = min_left_gap >= left_threshold;
  out.can_pass_right = min_right_gap >= right_threshold;

  if (out.can_pass_left && out.can_pass_right) {
    out.pass_gap_reason = "ok";
  } else if (out.can_pass_left) {
    out.pass_gap_reason = "right_gap_narrow";
  } else if (out.can_pass_right) {
    out.pass_gap_reason = "left_gap_narrow";
  } else {
    out.pass_gap_reason = "both_gap_narrow";
  }
  return out;
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

BlockedInfo OvertakePlannerCore::evaluateFutureSideBySideRisk(
    const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  BlockedInfo out = blocked_info;
  const int target_index = sideRiskIndex(out);
  if (!config_.future_side_prediction_enabled || target_index < 0 ||
      static_cast<std::size_t>(target_index) >= opponents.size()) {
    return out;
  }

  const bool exact_side_target = out.side_index >= 0;
  const auto &opp = opponents[static_cast<std::size_t>(target_index)];
  const bool direction_known = exact_side_target
                                   ? out.side_direction_known
                                   : out.parallel_side_direction_known;
  const bool same_direction = exact_side_target
                                  ? out.side_same_direction
                                  : out.parallel_side_same_direction;
  if (direction_known && !same_direction) {
    return out;
  }
  const double target_delta_d =
      exact_side_target ? out.side_delta_d : out.parallel_side_delta_d;
  const double target_s_dot =
      exact_side_target ? out.side_s_dot_mps : out.parallel_side_s_dot_mps;
  const double future_side_s_m =
      exact_side_target
          ? config_.side_by_side_s_m
          : std::max(config_.side_by_side_s_m, config_.parallel_side_s_m);
  const double future_side_margin_m =
      exact_side_target
          ? config_.side_margin_m
          : std::max(config_.side_margin_m, config_.parallel_side_margin_m);

  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  const double left_space = upper_d - ego.frenet.d;
  const double right_space = ego.frenet.d - lower_d;
  double away_sign = 0.0;
  if (std::abs(target_delta_d) > kSideDirectionEpsilon) {
    away_sign = target_delta_d > 0.0 ? -1.0 : 1.0;
  } else {
    away_sign = left_space >= right_space ? 1.0 : -1.0;
  }
  const double gap_target =
      opp.frenet.d + away_sign * config_.side_by_side_target_gap_m;
  double side_keep_target_d = away_sign > 0.0
                                  ? std::max(ego.frenet.d, gap_target)
                                  : std::min(ego.frenet.d, gap_target);
  side_keep_target_d = std::clamp(side_keep_target_d, lower_d, upper_d);

  const double horizon_sec =
      std::max(0.0, config_.future_side_prediction_horizon_sec);
  const double dt_sec = std::max(1.0e-3, config_.future_side_prediction_dt_sec);
  const double s_dot = target_s_dot;
  const double ego_speed = std::max(0.0, ego.v);
  double best_clearance = std::numeric_limits<double>::infinity();
  double max_future_curvature = 0.0;
  bool found_future_side = false;
  bool found_future_corner = false;
  bool found_outer_wall_risk = false;

  for (double t = dt_sec; t <= horizon_sec + 1.0e-9; t += dt_sec) {
    const double ego_s = frame_.wrapS(ego.frenet.s + ego_speed * t);
    const double opp_s = frame_.wrapS(opp.frenet.s + s_dot * t);
    const double progress = ego_speed * t;
    const double ratio = smoothstep(
        progress / std::max(1.0, config_.side_by_side_shift_distance_m));
    const double ego_d =
        ego.frenet.d + (side_keep_target_d - ego.frenet.d) * ratio;
    const double opp_d = opp.frenet.d;
    const double delta_s = signedDeltaS(ego_s, opp_s, frame_.length());
    const double delta_d = opp_d - ego_d;
    const bool future_side = std::abs(delta_s) < future_side_s_m &&
                             std::abs(delta_d) < future_side_margin_m;
    const double future_curvature =
        maxAbsCurvatureAhead(ego_s, config_.corner_side_yield_lookahead_m);
    const bool future_corner =
        future_side && config_.corner_side_yield_curvature_m_inv > 0.0 &&
        future_curvature >= config_.corner_side_yield_curvature_m_inv;
    const double clearance = wallClearance(ego_d);
    const double target_clearance = wallClearance(side_keep_target_d);
    const double effective_clearance = std::min(clearance, target_clearance);
    const bool outer_wall_risk =
        future_side &&
        effective_clearance < config_.future_side_yield_wall_clearance_m;

    if (future_side) {
      found_future_side = true;
      if (future_corner) {
        found_future_corner = true;
      }
      if (outer_wall_risk) {
        found_outer_wall_risk = true;
      }
      max_future_curvature = std::max(max_future_curvature, future_curvature);
      if (effective_clearance < best_clearance) {
        best_clearance = effective_clearance;
        out.future_delta_s = delta_s;
        out.future_delta_d = delta_d;
        out.future_wall_clearance_m = effective_clearance;
        out.future_abs_curvature = future_curvature;
        out.predicted_opponent_s = opp_s;
        out.predicted_opponent_d = opp_d;
        out.future_prediction_time_sec = t;
      }
    }

    if (future_side && future_corner &&
        (effective_clearance < config_.future_side_yield_wall_clearance_m ||
         outer_wall_risk)) {
      out.future_side_by_side = true;
      out.future_corner_side_by_side = true;
      out.future_outer_wall_risk = outer_wall_risk;
      out.future_yield_required = true;
      out.future_delta_s = delta_s;
      out.future_delta_d = delta_d;
      out.future_wall_clearance_m = effective_clearance;
      out.future_abs_curvature = future_curvature;
      out.predicted_opponent_s = opp_s;
      out.predicted_opponent_d = opp_d;
      out.future_prediction_time_sec = t;
      out.yield_reason =
          outer_wall_risk ? "future_outer_wall_risk" : "future_wall_clearance";
      return out;
    }
  }

  out.future_side_by_side = found_future_side;
  out.future_corner_side_by_side = found_future_corner;
  out.future_outer_wall_risk = found_outer_wall_risk;
  if (found_future_side && found_outer_wall_risk) {
    out.future_yield_required = true;
    if (out.yield_reason.empty()) {
      out.yield_reason = "future_outer_wall_risk";
    }
  }
  out.future_abs_curvature =
      std::max(out.future_abs_curvature, max_future_curvature);
  if (!std::isfinite(out.future_wall_clearance_m) &&
      std::isfinite(best_clearance)) {
    out.future_wall_clearance_m = best_clearance;
  }
  return out;
}

CandidateTrajectory OvertakePlannerCore::makeCandidate(
    CandidateType type, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  return CandidateBuilder(frame_, config_)
      .makeCandidate(type, ego, blocked_info, opponents);
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

double OvertakePlannerCore::wallClearance(double d) const {
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

double OvertakePlannerCore::opponentSDot(const OpponentState &opponent) const {
  const auto ref = frame_.interpolate(opponent.frenet.s);
  return opponent.vx * std::cos(ref.yaw) + opponent.vy * std::sin(ref.yaw);
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

double OvertakePlannerCore::effectiveWallSoftMargin(
    const ActiveSectionSafety &section) const {
  const double base = std::max(0.0, config_.wall_soft_margin_m);
  return base * std::max(1.0, section.wall_margin_scale);
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
  const bool close_to_wall =
      wallClearance(ego.frenet.d) <= config_.corner_side_yield_wall_clearance_m;
  return opponent_not_clearly_behind || close_to_wall;
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
