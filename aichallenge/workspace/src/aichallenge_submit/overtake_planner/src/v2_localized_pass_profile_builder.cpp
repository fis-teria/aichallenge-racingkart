#include "overtake_planner/v2_localized_pass_profile_builder.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace overtake_planner {

namespace {

double signedLocalDeltaS(double from_s, double to_s, double track_length_m) {
  double delta_s = to_s - from_s;
  if (std::isfinite(track_length_m) && track_length_m > 0.0) {
    delta_s = std::fmod(delta_s, track_length_m);
    if (delta_s > track_length_m * 0.5) {
      delta_s -= track_length_m;
    } else if (delta_s < -track_length_m * 0.5) {
      delta_s += track_length_m;
    }
  }
  return delta_s;
}

} // namespace

V2LocalizedPassProfileBuilder::V2LocalizedPassProfileBuilder(
    const FrenetFrame &frame, const PlannerConfig &config)
    : frame_(frame), config_(config), blocked_risk_(frame, config),
      candidate_builder_(frame, config) {}

std::optional<LocalizedLateralProfile> V2LocalizedPassProfileBuilder::build(
    double now_sec, const EgoState &ego, const OpponentState &target,
    const std::vector<OpponentState> &opponents, CandidateType pass_type,
    const BlockedInfo &blocked_info) const {
  if ((pass_type != CandidateType::PASS_LEFT &&
       pass_type != CandidateType::PASS_RIGHT) ||
      !std::isfinite(now_sec) || !ego.valid || !target.valid ||
      target.id.empty() || !std::isfinite(ego.frenet.s) ||
      !std::isfinite(ego.frenet.d) || !std::isfinite(target.frenet.s) ||
      !std::isfinite(target.frenet.d) || !std::isfinite(target.v) ||
      !inputTimestampFresh(now_sec, target.stamp_sec,
                           config_.opponent_stale_time_sec,
                           config_.input_future_stamp_tolerance_sec)) {
    return std::nullopt;
  }

  LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = pass_type;
  profile.target_id = target.id;
  profile.created_time_sec = now_sec;
  profile.anchor_s_m = ego.frenet.s;
  profile.ego_unwrapped_s_m = ego.frenet.s;
  profile.last_ego_wrapped_s_m = ego.frenet.s;
  profile.last_ego_stamp_sec = ego.stamp_sec;
  profile.start_d_m = ego.frenet.d;
  profile.target_d_m = targetOffset(pass_type, ego.frenet.d, target.frenet.d);
  const auto bounds =
      frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
  double lower_d_m = bounds.d_min + config_.min_wall_margin_m;
  double upper_d_m = bounds.d_max - config_.min_wall_margin_m;
  if (!std::isfinite(lower_d_m) || !std::isfinite(upper_d_m) ||
      lower_d_m > upper_d_m) {
    return std::nullopt;
  }
  if (blocked_info.gentle_curve_safe_pass_constraint_active &&
      config_.gentle_curve_safe_pass_enabled &&
      std::isfinite(
          config_.gentle_curve_safe_pass_max_lateral_displacement_m) &&
      config_.gentle_curve_safe_pass_max_lateral_displacement_m > 0.0) {
    const double anchor_d_m =
        std::isfinite(blocked_info.gentle_curve_safe_pass_anchor_d_m)
            ? blocked_info.gentle_curve_safe_pass_anchor_d_m
            : ego.frenet.d;
    lower_d_m = std::max(
        lower_d_m,
        anchor_d_m - config_.gentle_curve_safe_pass_max_lateral_displacement_m);
    upper_d_m = std::min(
        upper_d_m,
        anchor_d_m + config_.gentle_curve_safe_pass_max_lateral_displacement_m);
  }
  if (blocked_info.stationary_no_pass_safe_pass_constraint_active &&
      config_.stationary_no_pass_safe_pass_enabled &&
      std::isfinite(
          config_.stationary_no_pass_safe_pass_max_lateral_displacement_m) &&
      config_.stationary_no_pass_safe_pass_max_lateral_displacement_m > 0.0) {
    const double anchor_d_m =
        std::isfinite(blocked_info.stationary_no_pass_safe_pass_anchor_d_m)
            ? blocked_info.stationary_no_pass_safe_pass_anchor_d_m
            : ego.frenet.d;
    lower_d_m = std::max(
        lower_d_m,
        anchor_d_m -
            config_.stationary_no_pass_safe_pass_max_lateral_displacement_m);
    upper_d_m = std::min(
        upper_d_m,
        anchor_d_m +
            config_.stationary_no_pass_safe_pass_max_lateral_displacement_m);
  }
  if (lower_d_m > upper_d_m) {
    return std::nullopt;
  }
  profile.target_d_m = std::clamp(profile.target_d_m, lower_d_m, upper_d_m);
  const double target_s_m =
      profile.anchor_s_m + frame_.deltaS(profile.anchor_s_m, target.frenet.s);
  const double target_gap_m = target_s_m - profile.anchor_s_m;
  const bool early_low_speed_profile =
      blocked_info.early_low_speed_pass_target_active &&
      blocked_info.early_low_speed_pass_target_id == target.id;
  const double longitudinal_clearance_m =
      std::max(0.0, config_.safety_ellipse_a_m) *
      std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h));
  const double configured_start_before_m =
      early_low_speed_profile
          ? target_gap_m
          : std::max(
                0.0, config_.localized_avoidance_start_before_target_m);
  const double configured_full_before_m =
      early_low_speed_profile
          ? std::max(
                std::max(
                    0.0,
                    config_.localized_avoidance_full_offset_before_target_m),
                longitudinal_clearance_m)
          : std::max(
                0.0,
                config_.localized_avoidance_full_offset_before_target_m);
  const double requested_transition_m =
      std::abs(configured_start_before_m - configured_full_before_m);
  double tracking_speed_cap_mps = std::max(0.0, std::max(ego.v, target.v));
  if (blocked_info.pass_lateral_first_speed_gate_active &&
      std::isfinite(blocked_info.pass_lateral_first_speed_cap_mps) &&
      blocked_info.pass_lateral_first_speed_cap_mps > 0.0) {
    tracking_speed_cap_mps =
        std::min(tracking_speed_cap_mps,
                 blocked_info.pass_lateral_first_speed_cap_mps);
  }
  double required_transition_m =
      candidate_builder_.minimumTrackableLateralShiftDistance(
          ego, profile.target_d_m, tracking_speed_cap_mps, blocked_info,
          std::max(1.0, requested_transition_m));
  if (early_low_speed_profile &&
      std::isfinite(
          blocked_info.early_low_speed_pass_required_transition_m)) {
    required_transition_m =
        std::max(required_transition_m,
                 blocked_info.early_low_speed_pass_required_transition_m);
  }
  if (!std::isfinite(required_transition_m) ||
      !std::isfinite(target_gap_m) || target_gap_m < -1.0e-6) {
    return std::nullopt;
  }
  profile.full_offset_before_target_m =
      std::min(configured_full_before_m,
               target_gap_m - required_transition_m);
  profile.avoid_start_before_target_m =
      std::min(target_gap_m,
               std::max(configured_start_before_m,
                        profile.full_offset_before_target_m +
                            required_transition_m));
  profile.chain_tail_id = target.id;
  profile.chain_tail_s_m = target_s_m;
  profile.chain_tail_speed_mps = target.v;
  profile.chain_target_count = 1;
  profile.chain_waypoints = {LocalizedLateralWaypoint{
      target.id, target_s_m, profile.target_d_m, target_s_m, target.frenet.s,
      target.stamp_sec}};
  setMarkers(profile, target_s_m);
  extendSlowObstacleChain(now_sec, target, opponents, profile);
  profile.target_d_m = std::clamp(profile.target_d_m, lower_d_m, upper_d_m);
  for (auto &waypoint : profile.chain_waypoints) {
    waypoint.target_d_m = std::clamp(waypoint.target_d_m, lower_d_m, upper_d_m);
  }
  return profile;
}

bool V2LocalizedPassProfileBuilder::advanceLongitudinalProgress(
    double now_sec, const EgoState &ego,
    const std::vector<OpponentState> &opponents,
    LocalizedLateralProfile &profile) const {
  if (!profile.active || profile.target_id.empty() || !ego.valid ||
      !std::isfinite(now_sec) || !std::isfinite(ego.stamp_sec) ||
      !std::isfinite(ego.frenet.s) ||
      !std::isfinite(profile.ego_unwrapped_s_m) ||
      !std::isfinite(profile.last_ego_wrapped_s_m) ||
      !std::isfinite(profile.last_ego_stamp_sec) ||
      ego.stamp_sec + 1.0e-9 < profile.last_ego_stamp_sec ||
      profile.chain_waypoints.empty()) {
    return false;
  }

  LocalizedLateralProfile updated = profile;
  const double track_length_m = frame_.length();
  const auto plausible_delta = [&](double delta_s_m, double dt_sec,
                                   double observed_speed_mps) {
    if (!std::isfinite(delta_s_m) || !std::isfinite(dt_sec) || dt_sec < 0.0 ||
        !std::isfinite(observed_speed_mps) || observed_speed_mps < 0.0) {
      return false;
    }
    const double speed_bound_mps = std::max(
        {1.0, observed_speed_mps,
         std::max(0.0, config_.v_passthrough_mps)});
    const double distance_bound_m = speed_bound_mps * dt_sec + 0.50;
    return std::abs(delta_s_m) <= distance_bound_m + 1.0e-9;
  };

  if (ego.stamp_sec > updated.last_ego_stamp_sec + 1.0e-9) {
    const double dt_sec = ego.stamp_sec - updated.last_ego_stamp_sec;
    const double delta_s_m =
        signedLocalDeltaS(updated.last_ego_wrapped_s_m, ego.frenet.s,
                          track_length_m);
    if (!plausible_delta(delta_s_m, dt_sec, std::max(0.0, ego.v))) {
      return false;
    }
    updated.ego_unwrapped_s_m += delta_s_m;
    updated.last_ego_wrapped_s_m = ego.frenet.s;
    updated.last_ego_stamp_sec = ego.stamp_sec;
  }

  const double alpha =
      std::clamp(config_.maneuver_latch_target_update_alpha, 0.0, 1.0);
  if (!std::isfinite(alpha)) {
    return false;
  }
  double updated_target_s_m = updated.target_s_m;
  for (auto &waypoint : updated.chain_waypoints) {
    const auto observed = std::find_if(
        opponents.begin(), opponents.end(),
        [&](const OpponentState &opponent) {
          return opponent.id == waypoint.target_id && opponent.valid &&
                 std::isfinite(opponent.stamp_sec) &&
                 std::isfinite(opponent.frenet.s) &&
                 std::isfinite(opponent.v) && opponent.v >= 0.0 &&
                 inputTimestampFresh(
                     now_sec, opponent.stamp_sec,
                     config_.opponent_stale_time_sec,
                     config_.input_future_stamp_tolerance_sec);
        });
    if (observed == opponents.end() ||
        !std::isfinite(waypoint.observed_unwrapped_s_m) ||
        !std::isfinite(waypoint.last_observed_wrapped_s_m) ||
        !std::isfinite(waypoint.last_observed_stamp_sec) ||
        observed->stamp_sec + 1.0e-9 < waypoint.last_observed_stamp_sec) {
      return false;
    }
    if (observed->stamp_sec > waypoint.last_observed_stamp_sec + 1.0e-9) {
      const double dt_sec =
          observed->stamp_sec - waypoint.last_observed_stamp_sec;
      const double delta_s_m = signedLocalDeltaS(
          waypoint.last_observed_wrapped_s_m, observed->frenet.s,
          track_length_m);
      if (!plausible_delta(delta_s_m, dt_sec, observed->v) ||
          delta_s_m < -0.05) {
        return false;
      }
      waypoint.observed_unwrapped_s_m += std::max(0.0, delta_s_m);
      waypoint.last_observed_wrapped_s_m = observed->frenet.s;
      waypoint.last_observed_stamp_sec = observed->stamp_sec;
      const double filtered_s_m =
          waypoint.target_s_m * (1.0 - alpha) +
          waypoint.observed_unwrapped_s_m * alpha;
      waypoint.target_s_m = std::max(waypoint.target_s_m, filtered_s_m);
    }
    if (waypoint.target_id == updated.target_id) {
      updated_target_s_m = waypoint.target_s_m;
    }
    if (waypoint.target_id == updated.chain_tail_id) {
      updated.chain_tail_s_m = waypoint.target_s_m;
      updated.chain_tail_speed_mps = observed->v;
    }
  }

  double previous_s_m = updated_target_s_m;
  for (const auto &waypoint : updated.chain_waypoints) {
    if (!std::isfinite(waypoint.target_s_m) ||
        waypoint.target_s_m + 1.0e-6 < previous_s_m) {
      return false;
    }
    previous_s_m = waypoint.target_s_m;
  }
  if (!std::isfinite(updated_target_s_m) ||
      !std::isfinite(updated.chain_tail_s_m) ||
      updated.chain_tail_s_m + 1.0e-6 < updated_target_s_m) {
    return false;
  }

  const double previous_full_offset_end_s_m = updated.full_offset_end_s_m;
  const double previous_merge_end_s_m = updated.merge_end_s_m;
  setMarkers(updated, updated_target_s_m);
  const double hold_after_m =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance_m =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  updated.full_offset_end_s_m =
      std::max({updated.full_offset_end_s_m,
                updated.chain_tail_s_m + hold_after_m,
                previous_full_offset_end_s_m});
  updated.merge_end_s_m =
      std::max({updated.full_offset_end_s_m + merge_distance_m,
                previous_merge_end_s_m});
  if (updated.avoid_start_s_m > updated.full_offset_start_s_m ||
      updated.full_offset_start_s_m > updated.target_s_m ||
      updated.target_s_m > updated.full_offset_end_s_m ||
      updated.full_offset_end_s_m > updated.merge_end_s_m) {
    return false;
  }

  profile = std::move(updated);
  return true;
}

double V2LocalizedPassProfileBuilder::targetOffset(CandidateType pass_type,
                                                   double ego_d_m,
                                                   double opponent_d_m) const {
  const double required_gap_m =
      config_.safety_ellipse_b_m *
          std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h)) +
      std::max(0.0, config_.pass_target_lateral_margin_m);
  return passTargetOffset(config_, pass_type, ego_d_m, opponent_d_m,
                          required_gap_m);
}

void V2LocalizedPassProfileBuilder::setMarkers(LocalizedLateralProfile &profile,
                                               double target_s_m) const {
  profile.target_s_m = target_s_m;
  const double configured_start_before_m =
      std::max(0.0, config_.localized_avoidance_start_before_target_m);
  const double configured_full_before_m =
      std::max(0.0, config_.localized_avoidance_full_offset_before_target_m);
  const double nominal_avoid_before_m =
      std::max(configured_start_before_m, configured_full_before_m);
  const double nominal_full_before_m =
      std::min(configured_start_before_m, configured_full_before_m);
  const double transition_distance_m =
      nominal_avoid_before_m - nominal_full_before_m;
  // full_offset_before_target_mは、近距離で必要な横遷移距離を確保するため
  // 負値（target通過後のfull-offset）を取り得る。configured値でclampせず、
  // build()で決めたmarker provenanceをprogress更新でも維持する。
  const double full_before_m =
      std::isfinite(profile.full_offset_before_target_m)
          ? profile.full_offset_before_target_m
          : nominal_full_before_m;
  const double start_before_m =
      std::isfinite(profile.avoid_start_before_target_m)
          ? std::max(full_before_m, profile.avoid_start_before_target_m)
          : std::max(nominal_avoid_before_m,
                     full_before_m + transition_distance_m);
  const double hold_after_m =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance_m =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  profile.avoid_start_s_m = target_s_m - start_before_m;
  profile.full_offset_start_s_m = target_s_m - full_before_m;
  profile.full_offset_end_s_m = target_s_m + hold_after_m;
  profile.merge_end_s_m = profile.full_offset_end_s_m + merge_distance_m;
}

void V2LocalizedPassProfileBuilder::extendSlowObstacleChain(
    double now_sec, const OpponentState &target,
    const std::vector<OpponentState> &opponents,
    LocalizedLateralProfile &profile) const {
  if (!config_.slow_obstacle_chain_enabled ||
      !config_.slow_front_exception_enabled || !std::isfinite(target.v) ||
      target.v > std::max(0.0, config_.slow_front_exception_speed_mps)) {
    return;
  }

  struct ChainCandidate {
    std::string id;
    double unwrapped_s_m{0.0};
    double wrapped_s_m{0.0};
    double lateral_d_m{0.0};
    double speed_mps{0.0};
    double stamp_sec{0.0};
  };
  std::vector<ChainCandidate> candidates;
  const double lateral_limit_m = std::max(0.0, config_.same_corridor_width_m);
  const double cumulative_lateral_limit_m =
      std::max(lateral_limit_m, std::max(0.0, config_.parallel_side_margin_m));
  for (const auto &opponent : opponents) {
    if (opponent.id == target.id || !opponent.valid ||
        !inputTimestampFresh(now_sec, opponent.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec) ||
        !std::isfinite(opponent.v) || opponent.v < 0.0 ||
        opponent.v > std::max(0.0, config_.slow_front_exception_speed_mps) ||
        !std::isfinite(opponent.frenet.d) ||
        std::abs(opponent.frenet.d - target.frenet.d) >
            cumulative_lateral_limit_m) {
      continue;
    }
    const double s_dot_mps = blocked_risk_.opponentSDot(opponent);
    const bool direction_known =
        opponent.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known || (std::isfinite(s_dot_mps) &&
                             s_dot_mps >= config_.same_direction_min_s_dot_mps);
    if (config_.same_direction_filter_enabled && !same_direction) {
      continue;
    }
    const double unwrapped_s_m =
        profile.anchor_s_m +
        frame_.deltaS(profile.anchor_s_m, opponent.frenet.s);
    if (!std::isfinite(unwrapped_s_m) ||
        unwrapped_s_m <= profile.target_s_m + 1.0e-6) {
      continue;
    }
    candidates.push_back(ChainCandidate{opponent.id, unwrapped_s_m,
                                        opponent.frenet.s, opponent.frenet.d,
                                        opponent.v, opponent.stamp_sec});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const ChainCandidate &lhs, const ChainCandidate &rhs) {
              return lhs.unwrapped_s_m < rhs.unwrapped_s_m;
            });

  const double max_link_distance_m =
      std::max(0.0, config_.slow_obstacle_chain_distance_m);
  double tail_s_m = profile.target_s_m;
  double tail_d_m = target.frenet.d;
  double staged_target_d_m = profile.target_d_m;
  for (const auto &candidate : candidates) {
    if (candidate.unwrapped_s_m - tail_s_m > max_link_distance_m + 1.0e-6) {
      break;
    }
    if (std::abs(candidate.lateral_d_m - tail_d_m) > lateral_limit_m + 1.0e-6) {
      continue;
    }
    tail_s_m = candidate.unwrapped_s_m;
    tail_d_m = candidate.lateral_d_m;
    const double candidate_target_d_m = targetOffset(
        profile.pass_type, profile.start_d_m, candidate.lateral_d_m);
    staged_target_d_m = profile.pass_type == CandidateType::PASS_LEFT
                            ? std::max(staged_target_d_m, candidate_target_d_m)
                            : std::min(staged_target_d_m, candidate_target_d_m);
    profile.chain_waypoints.push_back(LocalizedLateralWaypoint{
        candidate.id, candidate.unwrapped_s_m, staged_target_d_m,
        candidate.unwrapped_s_m, candidate.wrapped_s_m,
        candidate.stamp_sec});
    profile.chain_tail_id = candidate.id;
    profile.chain_tail_s_m = candidate.unwrapped_s_m;
    profile.chain_tail_speed_mps = candidate.speed_mps;
    ++profile.chain_target_count;
  }
  const double hold_after_m =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance_m =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  profile.full_offset_end_s_m = std::max(profile.full_offset_end_s_m,
                                         profile.chain_tail_s_m + hold_after_m);
  profile.merge_end_s_m = profile.full_offset_end_s_m + merge_distance_m;
}

} // namespace overtake_planner
