#include "overtake_planner/overtake_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace overtake_planner
{

namespace
{

double smoothstep(double z)
{
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

constexpr double kSideDirectionEpsilon = 0.05;

bool isLeftPassMode(BehaviorMode mode)
{
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_LEFT;
}

bool isRightPassMode(BehaviorMode mode)
{
  return mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

bool currentPassGapLost(BehaviorMode mode, const BlockedInfo & blocked_info)
{
  return (isLeftPassMode(mode) && !blocked_info.can_pass_left) ||
         (isRightPassMode(mode) && !blocked_info.can_pass_right);
}

bool isPassCandidate(CandidateType type)
{
  return type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
}

bool isFallbackCandidate(CandidateType type)
{
  return type == CandidateType::FOLLOW ||
         type == CandidateType::YIELD_BEHIND ||
         type == CandidateType::RECOVERY ||
         type == CandidateType::SIDE_BY_SIDE_KEEP;
}

bool hasFeasibleCandidate(
  const std::vector<CandidateTrajectory> & candidates,
  bool (*predicate)(CandidateType))
{
  return std::any_of(
    candidates.begin(), candidates.end(),
    [predicate](const CandidateTrajectory & candidate) {
      return candidate.feasible && predicate(candidate.type);
    });
}

bool isFeasibleReleaseCandidate(
  const CandidateTrajectory & selected,
  const BlockedInfo & blocked_info)
{
  if (!selected.feasible || selected.type == CandidateType::SAFE_STOP) {
    return false;
  }
  if (!blocked_info.blocked && !blocked_info.side_by_side) {
    return selected.type == CandidateType::FASTEST || selected.type == CandidateType::RECOVERY;
  }
  return isPassCandidate(selected.type) ||
         selected.type == CandidateType::FOLLOW ||
         selected.type == CandidateType::YIELD_BEHIND ||
         selected.type == CandidateType::RECOVERY;
}

bool hasFeasibleReleaseCandidate(
  const std::vector<CandidateTrajectory> & candidates,
  const BlockedInfo & blocked_info)
{
  return std::any_of(
    candidates.begin(), candidates.end(),
    [&blocked_info](const CandidateTrajectory & candidate) {
      return isFeasibleReleaseCandidate(candidate, blocked_info);
    });
}

}  // namespace

OvertakePlannerCore::OvertakePlannerCore(FrenetFrame frame, PlannerConfig config)
: frame_(std::move(frame)),
  config_(config),
  safety_(config),
  state_machine_(config)
{
}

PlannerOutput OvertakePlannerCore::update(
  double now_sec,
  const EgoState & ego,
  const std::vector<OpponentState> & opponents)
{
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

  BlockedInfo blocked = detectBlocked(ego, opponents, now_sec);
  const auto predictions = predictOpponents(opponents, now_sec);
  blocked = evaluatePassGap(blocked, opponents, predictions);
  blocked.corner_abs_curvature =
    maxAbsCurvatureAhead(ego.frenet.s, config_.corner_side_yield_lookahead_m);
  blocked.corner_side_by_side =
    blocked.side_by_side &&
    config_.corner_side_yield_curvature_m_inv > 0.0 &&
    blocked.corner_abs_curvature >= config_.corner_side_yield_curvature_m_inv;
  blocked.ego_wall_clearance_m = wallClearance(ego.frenet.d);

  // まず全状況でFASTEST候補を作り、閉塞時だけ追従/左右追い越し候補を増やす。
  std::vector<CandidateTrajectory> candidates;
  candidates.push_back(makeCandidate(CandidateType::FASTEST, ego, blocked, opponents));
  if (blocked.side_by_side && !blocked.corner_side_by_side) {
    candidates.push_back(makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego, blocked, opponents));
  }
  if (shouldYieldBehindSideBySide(ego, blocked)) {
    candidates.push_back(makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
  }
  if (blocked.blocked) {
    candidates.push_back(makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents));
    if (blocked.can_pass_left) {
      candidates.push_back(makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
    }
    if (blocked.can_pass_right) {
      candidates.push_back(makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
    }
    if (!blocked.can_pass_left && !blocked.can_pass_right) {
      candidates.push_back(makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
    }
  }
  if (currentPassGapLost(mode_, blocked)) {
    candidates.push_back(makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
  }
  if (isPassMode(mode_) || mode_ == BehaviorMode::ABORT_RECOVERY) {
    // 追い越し中や中止中は、中心線へ戻るRECOVERY候補も常に評価する。
    candidates.push_back(makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }
  const bool needs_safe_stop_fallback_check =
    config_.safe_stop_enabled &&
    (blocked.blocked || blocked.side_by_side || currentPassGapLost(mode_, blocked));
  const bool has_recovery_candidate = std::any_of(
    candidates.begin(), candidates.end(),
    [](const CandidateTrajectory & candidate) {
      return candidate.type == CandidateType::RECOVERY;
    });
  if (needs_safe_stop_fallback_check && !has_recovery_candidate) {
    // SAFE_STOP判定前に、通常fallbackであるRECOVERYも必ず安全評価へ含める。
    candidates.push_back(makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }

  for (auto & candidate : candidates) {
    // 壁/他車との安全余裕を見てから、目的に応じたスコアを付ける。
    safety_.evaluate(candidate, predictions);
    candidate.score = candidateScore(candidate, blocked);
  }

  const bool no_feasible_pass = !hasFeasibleCandidate(candidates, isPassCandidate);
  const bool no_feasible_fallback = !hasFeasibleCandidate(candidates, isFallbackCandidate);
  const bool safe_stop_base_condition =
    config_.safe_stop_enabled &&
    (blocked.blocked || blocked.side_by_side || currentPassGapLost(mode_, blocked)) &&
    !blocked.can_pass_left &&
    !blocked.can_pass_right &&
    no_feasible_pass &&
    no_feasible_fallback;

  if (safe_stop_base_condition) {
    ++safe_stop_trigger_count_;
  } else {
    safe_stop_trigger_count_ = 0;
  }

  const int safe_stop_trigger_cycles_required = std::max(1, config_.safe_stop_trigger_cycles);
  SafeStopContext safe_stop_context;
  safe_stop_context.requested =
    safe_stop_base_condition &&
    safe_stop_trigger_count_ >= safe_stop_trigger_cycles_required;
  safe_stop_context.trigger_count = safe_stop_trigger_count_;
  safe_stop_context.ego_speed_mps = ego.v;
  const double safe_stop_target_d = std::clamp(
    ego.frenet.d,
    config_.d_min_m + config_.min_wall_margin_m,
    config_.d_max_m - config_.min_wall_margin_m);
  safe_stop_context.lateral_error_m = std::abs(ego.frenet.d - safe_stop_target_d);

  bool safe_stop_candidate_infeasible = false;
  bool safe_stop_request_infeasible = false;
  CandidateTrajectory safe_stop_candidate;
  if (safe_stop_context.requested || mode_ == BehaviorMode::SAFE_STOP) {
    // STOP要求時とSTOP保持中は、停止候補自体の安全性も毎周期確認する。
    safe_stop_candidate = makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    safety_.evaluate(safe_stop_candidate, predictions);
    safe_stop_candidate.score = candidateScore(safe_stop_candidate, blocked);
    safe_stop_context.candidate_feasible = safe_stop_candidate.feasible;
    safe_stop_context.reason = safe_stop_candidate.feasible ?
      "no_pass_and_no_safe_fallback" : "safe_stop_infeasible";
    if (safe_stop_context.requested && safe_stop_candidate.feasible) {
      candidates.push_back(safe_stop_candidate);
    } else if (!safe_stop_candidate.feasible) {
      safe_stop_candidate_infeasible = true;
      safe_stop_request_infeasible = safe_stop_context.requested;
    }
  }

  CandidateTrajectory selected = selectCandidate(candidates);
  const bool release_front_gap_ready =
    blocked.nearest_index < 0 ||
    blocked.front_delta_s >= config_.safe_stop_release_front_gap_m;
  safe_stop_context.release_ready =
    ego.valid &&
    blocked.ego_wall_clearance_m >= config_.safe_stop_release_wall_clearance_m &&
    !blocked.side_by_side &&
    release_front_gap_ready &&
    hasFeasibleReleaseCandidate(candidates, blocked) &&
    ego.v <= config_.safe_stop_release_speed_mps &&
    safe_stop_context.lateral_error_m <= config_.safe_stop_lateral_error_threshold_m;

  // 候補選択だけで急にモードを切り替えず、状態機械で保持時間や継続条件をかける。
  mode_ = state_machine_.update(
    now_sec, mode_, selected.type, blocked, selected.feasible, safe_stop_context);
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
    selected = makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::YIELD_BEHIND) {
    selected = makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::FOLLOW_BLOCKED && selected.type != CandidateType::FOLLOW) {
    // 追従モードでは速度上限だけを落とすFOLLOW候補を優先する。
    selected = makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
    if (!selected.feasible) {
      selected = makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      safety_.evaluate(selected, predictions);
    }
  } else if (mode_ == BehaviorMode::FREE_RUN) {
    selected = makeCandidate(CandidateType::FASTEST, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  }

  output.mode = mode_;
  // ROSノードがMPC overrideとdebug JSONを作れるよう、選択結果を平坦な出力に詰める。
  output.selected = selected.type;
  output.blocked_info = blocked;
  output.reason = selected.reject_reason;
  output.safe_stop_triggered =
    safe_stop_context.requested || mode_ == BehaviorMode::SAFE_STOP;
  output.safe_stop_release_ready = safe_stop_context.release_ready;
  output.safe_stop_v_mps = std::max(1.0e-3, config_.safe_stop_v_mps);
  output.safe_stop_trigger_count = safe_stop_trigger_count_;
  output.safe_stop_hold_count = state_machine_.safeStopHoldCount();
  output.safe_stop_release_count = state_machine_.safeStopReleaseCount();
  if (mode_ == BehaviorMode::SAFE_STOP) {
    if (!selected.feasible) {
      output.safe_stop_reason = "safe_stop_infeasible";
      output.safe_stop_reject_reason = selected.reject_reason;
    } else if (safe_stop_context.requested && output.safe_stop_hold_count <= 1) {
      output.safe_stop_reason = "no_pass_and_no_safe_fallback";
    } else if (safe_stop_context.release_ready) {
      output.safe_stop_reason = "safe_stop_release_pending";
    } else {
      output.safe_stop_reason = "safe_stop_holding";
    }
  } else if (safe_stop_candidate_infeasible) {
    output.safe_stop_triggered = true;
    output.safe_stop_reason = "safe_stop_infeasible";
    output.safe_stop_reject_reason = safe_stop_candidate.reject_reason;
  }
  if (!output.safe_stop_reason.empty()) {
    output.reason = output.safe_stop_reason == "no_pass_and_no_safe_fallback" ?
      "no_safe_avoidance" : output.safe_stop_reason;
  }
  const bool side_by_side_best_effort =
    selected.type == CandidateType::SIDE_BY_SIDE_KEEP &&
    selected.reject_reason == "opponent_collision";
  const bool yield_best_effort =
    selected.type == CandidateType::YIELD_BEHIND &&
    selected.reject_reason == "opponent_collision";
  const bool wall_margin_escape =
    selected.reject_reason == "wall_margin" &&
    selected.type == CandidateType::YIELD_BEHIND;
  const double selected_target_d = selected.d.empty() ? ego.frenet.d : selected.d.back();
  if (selected.type == CandidateType::SAFE_STOP) {
    output.active_override = selected.feasible;
  } else {
    output.active_override =
      selected.type != CandidateType::FASTEST &&
      !safe_stop_request_infeasible &&
      (selected.feasible || side_by_side_best_effort || yield_best_effort || wall_margin_escape);
  }
  output.target_lateral_offset_m = selected.d.empty() ? 0.0 : selected_target_d;
  output.min_cbf_h = selected.min_safety_margin;
  output.cbf_slack = selected.cbf_slack;
  output.active_cbf_constraint_count = selected.active_safety_constraint_count;
  output.lateral_offsets = selected.d;
  output.speed_caps = selected.v_ref;
  return output;
}

BlockedInfo OvertakePlannerCore::detectBlocked(
  const EgoState & ego,
  const std::vector<OpponentState> & opponents,
  double now_sec) const
{
  // 同一コリドー内の最も近い前方車両を探し、速度差/距離から閉塞を判断する。
  BlockedInfo info;
  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto & opp = opponents[i];
    if (!opp.valid || now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, opp.frenet.s);
    double signed_delta_s = opp.frenet.s - ego.frenet.s;
    if (frame_.length() > 0.0) {
      signed_delta_s = std::fmod(signed_delta_s, frame_.length());
      if (signed_delta_s > frame_.length() * 0.5) {
        signed_delta_s -= frame_.length();
      } else if (signed_delta_s < -frame_.length() * 0.5) {
        signed_delta_s += frame_.length();
      }
    }
    const double delta_d = opp.frenet.d - ego.frenet.d;
    const bool front = delta_s > 0.0 && delta_s < config_.lookahead_s_m;
    const bool same_corridor = std::abs(delta_d) < config_.same_corridor_width_m;
    const bool side_by_side = std::abs(signed_delta_s) < config_.side_by_side_s_m &&
      std::abs(delta_d) < config_.side_margin_m;
    if (side_by_side) {
      // 横並び中は不用意なmerge backを避けるため、状態機械へ明示的に伝える。
      info.side_by_side = true;
      if (info.side_index < 0 || std::abs(signed_delta_s) < std::abs(info.side_delta_s)) {
        info.side_index = static_cast<int>(i);
        info.side_id = opp.id;
        info.side_delta_s = signed_delta_s;
        info.side_delta_d = delta_d;
        info.side_rel_v = ego.v - opp.v;
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
  const BlockedInfo & blocked_info,
  const std::vector<OpponentState> & opponents,
  const std::vector<PredictedOpponent> & predictions) const
{
  BlockedInfo out = blocked_info;
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  const double ellipse_gap = config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h);
  out.pass_gap_required_m = std::max(config_.min_pass_gap_m, ellipse_gap);

  const int target_index = out.nearest_index >= 0 ? out.nearest_index : out.side_index;
  if (target_index < 0 || static_cast<std::size_t>(target_index) >= opponents.size()) {
    out.left_pass_gap_m = 0.0;
    out.right_pass_gap_m = 0.0;
    out.can_pass_left = false;
    out.can_pass_right = false;
    out.pass_gap_reason = "no_target";
    return out;
  }

  const auto & target = opponents[static_cast<std::size_t>(target_index)];
  double min_left_gap = upper_d - target.frenet.d;
  double min_right_gap = target.frenet.d - lower_d;

  for (const auto & pred : predictions) {
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
    out.pass_gap_required_m - (isLeftPassMode(mode_) ? config_.pass_gap_hysteresis_m : 0.0);
  const double right_threshold =
    out.pass_gap_required_m - (isRightPassMode(mode_) ? config_.pass_gap_hysteresis_m : 0.0);
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
  const std::vector<OpponentState> & opponents,
  double now_sec) const
{
  // V2X位置から推定した速度を使い、短いhorizonでは等速直線運動として予測する。
  std::vector<PredictedOpponent> out;
  for (const auto & opp : opponents) {
    if (!opp.valid || now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
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
  CandidateType type,
  const EgoState & ego,
  const BlockedInfo & blocked_info,
  const std::vector<OpponentState> & opponents) const
{
  // 候補ごとに目標横オフセットと速度上限を決め、Frenet上で滑らかに接続する。
  CandidateTrajectory candidate;
  candidate.type = type;
  candidate.t.reserve(config_.horizon_points);
  candidate.s.reserve(config_.horizon_points);
  candidate.d.reserve(config_.horizon_points);
  candidate.x.reserve(config_.horizon_points);
  candidate.y.reserve(config_.horizon_points);
  candidate.yaw.reserve(config_.horizon_points);
  candidate.v_ref.reserve(config_.horizon_points);

  double target_d = 0.0;
  double shift_distance = config_.merge_distance_m;
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  if (type == CandidateType::PASS_LEFT) {
    // 左右PASSは中心線から一定量オフセットした仮想参照をMPCへ渡す。
    target_d = config_.left_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::PASS_RIGHT) {
    target_d = config_.right_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::FOLLOW) {
    target_d = 0.0;
  } else if (type == CandidateType::RECOVERY) {
    target_d = 0.0;
  } else if (type == CandidateType::SIDE_BY_SIDE_KEEP) {
    target_d = ego.frenet.d;
    shift_distance = config_.side_by_side_shift_distance_m;
    if (blocked_info.side_index >= 0) {
      const auto & opp = opponents[static_cast<std::size_t>(blocked_info.side_index)];
      const double left_space = upper_d - ego.frenet.d;
      const double right_space = ego.frenet.d - lower_d;
      double away_sign = 0.0;
      if (std::abs(blocked_info.side_delta_d) > kSideDirectionEpsilon) {
        away_sign = blocked_info.side_delta_d > 0.0 ? -1.0 : 1.0;
      } else {
        away_sign = left_space >= right_space ? 1.0 : -1.0;
      }
      const double gap_target =
        opp.frenet.d + away_sign * config_.side_by_side_target_gap_m;
      target_d = away_sign > 0.0 ?
        std::max(ego.frenet.d, gap_target) :
        std::min(ego.frenet.d, gap_target);
      target_d = std::clamp(target_d, lower_d, upper_d);
    }
  } else if (type == CandidateType::YIELD_BEHIND) {
    const int target_index =
      blocked_info.nearest_index >= 0 ? blocked_info.nearest_index : blocked_info.side_index;
    target_d = std::clamp(config_.corner_yield_target_d_m, lower_d, upper_d);
    if (
      !blocked_info.corner_side_by_side &&
      wallClearance(ego.frenet.d) >= config_.yield_rejoin_wall_clearance_m &&
      target_index >= 0 && static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto & opp = opponents[static_cast<std::size_t>(target_index)];
      target_d = std::clamp(opp.frenet.d, lower_d, upper_d);
    }
  } else if (type == CandidateType::SAFE_STOP) {
    target_d = std::clamp(ego.frenet.d, lower_d, upper_d);
    shift_distance = std::max(config_.merge_distance_m, config_.prepare_distance_m);
  }

  double speed_cap = config_.v_passthrough_mps;
  const double ego_wall_clearance = wallClearance(ego.frenet.d);
  if (type == CandidateType::FOLLOW && blocked_info.nearest_index >= 0) {
    // FOLLOWは前走車より少し低い速度上限にして、MPC側の速度計画を抑える。
    const auto & opp = opponents[static_cast<std::size_t>(blocked_info.nearest_index)];
    speed_cap = std::max(0.5, opp.v - config_.follow_speed_margin_mps);
  } else if (type == CandidateType::RECOVERY) {
    speed_cap = ego_wall_clearance < 0.0 ?
      config_.wall_margin_recovery_v_max_mps : config_.recovery_v_max_mps;
  } else if (type == CandidateType::SIDE_BY_SIDE_KEEP) {
    speed_cap = config_.side_by_side_speed_cap_mps;
    if (blocked_info.side_index >= 0) {
      const auto & opp = opponents[static_cast<std::size_t>(blocked_info.side_index)];
      speed_cap = std::min(speed_cap, std::max(0.5, opp.v - config_.yield_speed_margin_mps));
    }
  } else if (type == CandidateType::YIELD_BEHIND) {
    speed_cap = 0.5;
    const int target_index =
      blocked_info.nearest_index >= 0 ? blocked_info.nearest_index : blocked_info.side_index;
    if (target_index >= 0 && static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto & opp = opponents[static_cast<std::size_t>(target_index)];
      const double margin = blocked_info.corner_side_by_side ?
        config_.corner_follow_speed_margin_mps : config_.yield_speed_margin_mps;
      speed_cap = std::max(0.5, opp.v - margin);
    }
    if (blocked_info.corner_side_by_side && config_.corner_yield_v_max_mps > 0.0) {
      speed_cap = std::min(speed_cap, config_.corner_yield_v_max_mps);
    }
  } else if (type == CandidateType::SAFE_STOP) {
    speed_cap = std::max(1.0e-3, config_.safe_stop_v_mps);
  }

  const bool recovery_like =
    type == CandidateType::RECOVERY ||
    type == CandidateType::YIELD_BEHIND ||
    type == CandidateType::SIDE_BY_SIDE_KEEP;
  if (recovery_like && ego_wall_clearance < 0.0) {
    speed_cap = std::min(speed_cap, config_.wall_margin_recovery_v_max_mps);
  }
  if (
    recovery_like &&
    config_.large_lateral_error_threshold_m >= 0.0 &&
    config_.large_lateral_error_v_max_mps > 0.0 &&
    std::abs(ego.frenet.d - target_d) > config_.large_lateral_error_threshold_m) {
    speed_cap = std::min(speed_cap, config_.large_lateral_error_v_max_mps);
  }

  for (std::size_t i = 0; i < config_.horizon_points; ++i) {
    // 候補ごとの速度想定でs列を作り、smoothstepで横方向を急変させない。
    const double t = static_cast<double>(i) * config_.horizon_dt_sec;
    const double longitudinal_speed = type == CandidateType::SAFE_STOP ?
      std::max(1.0e-3, config_.safe_stop_v_mps) : std::max(0.5, ego.v);
    const double ds = longitudinal_speed * t;
    const double s = frame_.wrapS(ego.frenet.s + ds);
    const double ratio = smoothstep(ds / std::max(1.0, shift_distance));
    double start_d = ego.frenet.d;
    if (
      type == CandidateType::RECOVERY ||
      type == CandidateType::YIELD_BEHIND ||
      type == CandidateType::SAFE_STOP) {
      start_d = std::clamp(start_d, lower_d, upper_d);
    }
    const double d = start_d + (target_d - start_d) * ratio;
    const auto p = frame_.frenetToCartesian(s, d);
    candidate.t.push_back(t);
    candidate.s.push_back(s);
    candidate.d.push_back(d);
    candidate.x.push_back(p.x);
    candidate.y.push_back(p.y);
    candidate.yaw.push_back(p.yaw);
    candidate.v_ref.push_back(speed_cap);
  }

  return candidate;
}

double OvertakePlannerCore::candidateScore(
  const CandidateTrajectory & candidate,
  const BlockedInfo & blocked_info) const
{
  // 候補の優先順位を単純なコストへ落とし、まず安全性、その後に追い越し意欲を見る。
  if (!candidate.feasible) {
    if (
      candidate.reject_reason == "wall_margin" &&
      blocked_info.side_by_side &&
      candidate.type == CandidateType::YIELD_BEHIND) {
      return blocked_info.corner_side_by_side ? -90.0 : -40.0;
    }
    if (
      candidate.reject_reason == "opponent_collision" &&
      blocked_info.side_by_side &&
      candidate.type == CandidateType::SIDE_BY_SIDE_KEEP &&
      !blocked_info.corner_side_by_side) {
      return -30.0;
    }
    if (
      candidate.reject_reason == "opponent_collision" &&
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
      score = blocked_info.side_by_side && !blocked_info.corner_side_by_side ? -30.0 : 80.0;
      break;
    case CandidateType::YIELD_BEHIND:
      if (blocked_info.corner_side_by_side) {
        score = -70.0;
      } else if (blocked_info.side_by_side && blocked_info.side_delta_s > config_.side_yield_s_m) {
        score = -60.0;
      } else if (currentPassGapLost(mode_, blocked_info)) {
        score = -50.0;
      } else {
        score = (!blocked_info.can_pass_left && !blocked_info.can_pass_right) ? 5.0 : 70.0;
      }
      break;
    case CandidateType::SAFE_STOP:
      score = -120.0;
      break;
  }
  if (
    (mode_ == BehaviorMode::OVERTAKE_LEFT && candidate.type == CandidateType::PASS_LEFT) ||
    (mode_ == BehaviorMode::OVERTAKE_RIGHT && candidate.type == CandidateType::PASS_RIGHT) ||
    (mode_ == BehaviorMode::FOLLOW_BLOCKED && candidate.type == CandidateType::FOLLOW) ||
    (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP && candidate.type == CandidateType::SIDE_BY_SIDE_KEEP) ||
    (mode_ == BehaviorMode::YIELD_BEHIND && candidate.type == CandidateType::YIELD_BEHIND) ||
    (mode_ == BehaviorMode::SAFE_STOP && candidate.type == CandidateType::SAFE_STOP)) {
    // いまのモードに沿う候補を少し優遇し、左右や追従/追い越しが細かく揺れないようにする。
    score -= config_.keep_mode_bonus;
  }
  return score;
}

double OvertakePlannerCore::maxAbsCurvatureAhead(double s, double lookahead_m) const
{
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

double OvertakePlannerCore::wallClearance(double d) const
{
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

bool OvertakePlannerCore::shouldYieldBehindSideBySide(
  const EgoState & ego,
  const BlockedInfo & blocked_info) const
{
  if (!blocked_info.side_by_side) {
    return false;
  }
  if (blocked_info.side_delta_s > config_.side_yield_s_m) {
    return true;
  }
  if (!blocked_info.corner_side_by_side) {
    return false;
  }
  const bool opponent_not_clearly_behind = blocked_info.side_delta_s > -config_.side_yield_s_m;
  const bool close_to_wall =
    wallClearance(ego.frenet.d) <= config_.corner_side_yield_wall_clearance_m;
  return opponent_not_clearly_behind || close_to_wall;
}

CandidateTrajectory OvertakePlannerCore::selectCandidate(
  std::vector<CandidateTrajectory> & candidates) const
{
  // スコア最小の候補を返す。空の場合はデフォルト候補を返して上位で安全側に倒す。
  auto best = std::min_element(
    candidates.begin(), candidates.end(),
    [](const CandidateTrajectory & a, const CandidateTrajectory & b) {
      return a.score < b.score;
    });
  if (best == candidates.end()) {
    return {};
  }
  return *best;
}

}  // namespace overtake_planner
