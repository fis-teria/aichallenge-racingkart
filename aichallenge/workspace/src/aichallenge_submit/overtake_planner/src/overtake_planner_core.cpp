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
  PlannerOutput output;
  output.lateral_offsets.assign(config_.horizon_points, 0.0);
  output.speed_caps.assign(config_.horizon_points, config_.v_passthrough_mps);

  if (!config_.enabled || !ego.valid || frame_.empty()) {
    mode_ = BehaviorMode::FREE_RUN;
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }

  const BlockedInfo blocked = detectBlocked(ego, opponents, now_sec);
  const auto predictions = predictOpponents(opponents, now_sec);

  std::vector<CandidateTrajectory> candidates;
  candidates.push_back(makeCandidate(CandidateType::FASTEST, ego, blocked, opponents));
  if (blocked.blocked) {
    candidates.push_back(makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents));
    candidates.push_back(makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
    candidates.push_back(makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
  }
  if (isPassMode(mode_) || mode_ == BehaviorMode::ABORT_RECOVERY) {
    candidates.push_back(makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }

  for (auto & candidate : candidates) {
    safety_.evaluate(candidate, predictions);
    candidate.score = candidateScore(candidate, blocked);
  }

  CandidateTrajectory selected = selectCandidate(candidates);
  mode_ = state_machine_.update(now_sec, mode_, selected.type, blocked, selected.feasible);
  if (mode_ == BehaviorMode::ABORT_RECOVERY) {
    selected = makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::MERGE_BACK) {
    selected = makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    safety_.evaluate(selected, predictions);
  } else if (mode_ == BehaviorMode::FOLLOW_BLOCKED && selected.type != CandidateType::FOLLOW) {
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
  output.selected = selected.type;
  output.blocked_info = blocked;
  output.reason = selected.reject_reason;
  output.active_override = selected.feasible && selected.type != CandidateType::FASTEST;
  output.target_lateral_offset_m = selected.d.empty() ? 0.0 : selected.d.back();
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
      info.side_by_side = true;
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
    const bool closing = info.front_rel_v > config_.dv_block_threshold_mps;
    const bool slow_gap = info.front_delta_s < config_.follow_trigger_s_m;
    info.blocked = closing || slow_gap;
  }
  return info;
}

std::vector<PredictedOpponent> OvertakePlannerCore::predictOpponents(
  const std::vector<OpponentState> & opponents,
  double now_sec) const
{
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
  if (type == CandidateType::PASS_LEFT) {
    target_d = config_.left_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::PASS_RIGHT) {
    target_d = config_.right_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::FOLLOW) {
    target_d = 0.0;
  } else if (type == CandidateType::RECOVERY) {
    target_d = 0.0;
  }

  double speed_cap = config_.v_passthrough_mps;
  if (type == CandidateType::FOLLOW && blocked_info.nearest_index >= 0) {
    const auto & opp = opponents[static_cast<std::size_t>(blocked_info.nearest_index)];
    speed_cap = std::max(0.5, opp.v - config_.follow_speed_margin_mps);
  } else if (type == CandidateType::RECOVERY) {
    speed_cap = config_.recovery_v_max_mps;
  }

  for (std::size_t i = 0; i < config_.horizon_points; ++i) {
    const double t = static_cast<double>(i) * config_.horizon_dt_sec;
    const double ds = std::max(0.5, ego.v) * t;
    const double s = frame_.wrapS(ego.frenet.s + ds);
    const double ratio = smoothstep(ds / std::max(1.0, shift_distance));
    const double d = ego.frenet.d + (target_d - ego.frenet.d) * ratio;
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
  if (!candidate.feasible) {
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
  }
  if (
    (mode_ == BehaviorMode::OVERTAKE_LEFT && candidate.type == CandidateType::PASS_LEFT) ||
    (mode_ == BehaviorMode::OVERTAKE_RIGHT && candidate.type == CandidateType::PASS_RIGHT) ||
    (mode_ == BehaviorMode::FOLLOW_BLOCKED && candidate.type == CandidateType::FOLLOW)) {
    score -= config_.keep_mode_bonus;
  }
  return score;
}

CandidateTrajectory OvertakePlannerCore::selectCandidate(
  std::vector<CandidateTrajectory> & candidates) const
{
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
