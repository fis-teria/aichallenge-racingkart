#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

constexpr double kSideDirectionEpsilon = 0.05;

double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
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

} // namespace

FutureSideBySideRiskAnalyzer::FutureSideBySideRiskAnalyzer(
    const FrenetFrame &frame, const PlannerConfig &config,
    const BlockedRiskAnalyzer &blocked_risk)
    : frame_(frame), config_(config), blocked_risk_(blocked_risk) {}

BlockedInfo FutureSideBySideRiskAnalyzer::evaluate(
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
    const double clearance = blocked_risk_.wallClearance(ego_d);
    const double target_clearance =
        blocked_risk_.wallClearance(side_keep_target_d);
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

int FutureSideBySideRiskAnalyzer::sideRiskIndex(
    const BlockedInfo &info) const {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

double FutureSideBySideRiskAnalyzer::maxAbsCurvatureAhead(
    double s, double lookahead_m) const {
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

} // namespace overtake_planner
