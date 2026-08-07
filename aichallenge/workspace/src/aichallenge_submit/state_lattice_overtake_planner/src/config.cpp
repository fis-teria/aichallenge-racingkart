#include "state_lattice_overtake_planner/config.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>

namespace state_lattice_overtake_planner {
namespace {
constexpr double kVehicleMaximumSteerRad = 0.64;
constexpr double kVehicleMaximumSteerRateRadps = 128.0;

template <typename T> bool strictlyIncreasing(const std::vector<T> &values) {
  return std::adjacent_find(values.begin(), values.end(),
                            std::greater_equal<T>()) == values.end();
}

bool allFinite(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool containsCenterTarget(const std::vector<double> &values) {
  return std::find(values.begin(), values.end(), 0.0) != values.end();
}

bool containsDuplicate(const std::vector<double> &values) {
  for (auto current = values.begin(); current != values.end(); ++current) {
    if (std::find(std::next(current), values.end(), *current) != values.end()) {
      return true;
    }
  }
  return false;
}
} // namespace

ControllerTrackabilityProfile
parseControllerTrackabilityProfile(const std::string &value) {
  if (value == "shadow_only") {
    return ControllerTrackabilityProfile::SHADOW_ONLY;
  }
  if (value == "pure_pursuit") {
    return ControllerTrackabilityProfile::PURE_PURSUIT;
  }
  if (value == "instant") {
    return ControllerTrackabilityProfile::INSTANT;
  }
  return ControllerTrackabilityProfile::UNKNOWN;
}

const char *toString(ControllerTrackabilityProfile profile) {
  switch (profile) {
  case ControllerTrackabilityProfile::SHADOW_ONLY:
    return "shadow_only";
  case ControllerTrackabilityProfile::PURE_PURSUIT:
    return "pure_pursuit";
  case ControllerTrackabilityProfile::INSTANT:
    return "instant";
  case ControllerTrackabilityProfile::UNKNOWN:
    break;
  }
  return "unknown";
}

std::string validateControllerTrackabilityProfileContract(
    ControllerTrackabilityProfile profile, bool live_control_output_enabled,
    bool instant_control_enabled) {
  if (profile == ControllerTrackabilityProfile::UNKNOWN) {
    return "unknown controller trackability profile";
  }
  if (!live_control_output_enabled) {
    return profile == ControllerTrackabilityProfile::SHADOW_ONLY
               ? std::string{}
               : "non-live planner requires shadow_only trackability profile";
  }
  if (instant_control_enabled) {
    return profile == ControllerTrackabilityProfile::INSTANT
               ? std::string{}
               : "instant control requires instant trackability profile";
  }
  return profile == ControllerTrackabilityProfile::PURE_PURSUIT
             ? std::string{}
             : "Pure Pursuit control requires pure_pursuit trackability "
               "profile";
}

void applyResolvedControllerTrackabilityEnvelope(
    PlannerConfig *config, double instant_maximum_steering_angle_rad,
    double instant_maximum_steering_rate_radps) {
  if (config == nullptr || config->controller_trackability_profile !=
                               ControllerTrackabilityProfile::INSTANT) {
    return;
  }
  config->planner_max_steer_rad = std::min(config->planner_max_steer_rad,
                                           instant_maximum_steering_angle_rad);
  config->max_steer_rate_radps = std::min(config->max_steer_rate_radps,
                                          instant_maximum_steering_rate_radps);
}

std::string validateConfig(const PlannerConfig &c) {
  if (c.controller_trackability_profile ==
      ControllerTrackabilityProfile::UNKNOWN) {
    return "invalid controller trackability profile";
  }
  if (c.cost_levels.size() != 10U || !strictlyIncreasing(c.cost_levels)) {
    return "cost_levels must contain 10 strictly increasing values";
  }
  if (c.wall_distance_thresholds_m.size() != 3U ||
      !allFinite(c.wall_distance_thresholds_m) ||
      !strictlyIncreasing(c.wall_distance_thresholds_m) ||
      c.object_distance_thresholds_m.size() != 5U ||
      !allFinite(c.object_distance_thresholds_m) ||
      !strictlyIncreasing(c.object_distance_thresholds_m) ||
      c.reference_distance_thresholds_m.size() != 4U ||
      !allFinite(c.reference_distance_thresholds_m) ||
      !strictlyIncreasing(c.reference_distance_thresholds_m)) {
    return "distance threshold arrays have invalid size or ordering";
  }
  const bool supported_lateral_count =
      c.lateral_targets_m.size() == 5U || c.lateral_targets_m.size() == 7U;
  if (!supported_lateral_count || !allFinite(c.lateral_targets_m) ||
      c.tangent_scales.size() != 3U || !allFinite(c.tangent_scales) ||
      c.sampling_points != 5 || c.rear_sampling_points != 5) {
    return "lattice contract requires 5 or 7 lateral targets, 3 tangent "
           "scales, and five samples";
  }
  if (!containsCenterTarget(c.lateral_targets_m) ||
      containsDuplicate(c.lateral_targets_m)) {
    return "lateral_targets_m must contain one center target and no duplicates";
  }
  if (!(c.base_distance_m > 0.0 && c.max_distance_m >= c.base_distance_m &&
        c.max_distance_speed_mps > 0.0 &&
        c.minimum_lateral_transition_distance_m > 0.0 &&
        c.minimum_lateral_transition_distance_m <= c.max_distance_m &&
        c.lateral_transition_distance_gain >= 0.0 &&
        c.pass_lateral_extra_margin_m >= 0.0 &&
        c.max_adaptive_lateral_offset_m >= 1.2 &&
        c.minimum_obstacle_transition_distance_m > 0.0 &&
        c.minimum_obstacle_transition_distance_m <= c.max_distance_m &&
        c.front_detection_radius_m > 0.0 && c.frontmost_s_tolerance_m >= 0.0 &&
        c.front_enter_cycles >= 1 && c.front_release_cycles >= 1 &&
        std::isfinite(c.early_aware_base_distance_m) &&
        c.early_aware_base_distance_m >= 0.0 &&
        std::isfinite(c.early_aware_speed_horizon_sec) &&
        c.early_aware_speed_horizon_sec >= 0.0 &&
        std::isfinite(c.early_aware_max_deceleration_mps2) &&
        c.early_aware_max_deceleration_mps2 > 0.0 &&
        std::isfinite(c.early_aware_min_distance_m) &&
        std::isfinite(c.early_aware_max_distance_m) &&
        c.early_aware_min_distance_m > 0.0 &&
        c.early_aware_max_distance_m >= c.early_aware_min_distance_m &&
        c.early_aware_max_distance_m <= 30.0 &&
        std::isfinite(c.early_aware_min_tangent_speed_mps) &&
        c.early_aware_min_tangent_speed_mps > 0.0 &&
        std::isfinite(c.early_aware_reverse_tangent_tolerance_mps) &&
        c.early_aware_reverse_tangent_tolerance_mps >= 0.0 &&
        std::isfinite(c.early_aware_max_track_heading_error_rad) &&
        c.early_aware_max_track_heading_error_rad >= 0.0 &&
        c.early_aware_max_track_heading_error_rad < 1.5707963267948966 &&
        std::isfinite(c.early_aware_min_forward_delta_s_m) &&
        c.early_aware_min_forward_delta_s_m > 0.0 &&
        c.early_aware_required_fresh_stamps >= 1 &&
        c.rear_safety_distance_m > 0.0 && c.rear_terminal_distance_m > 0.0 &&
        c.rear_sampling_angle_rad >= 0.0 &&
        c.rear_sampling_angle_rad < 1.5707963267948966 &&
        c.rear_prediction_horizon_sec > 0.0 && c.return_required_cycles >= 1 &&
        c.passed_target_gap_m > 0.0 && c.return_prediction_sec >= 0.0 &&
        c.return_predicted_gap_m >= 0.0 && c.return_lateral_error_m >= 0.0 &&
        c.return_heading_error_rad >= 0.0)) {
    return "invalid forward detection geometry";
  }
  if (!std::isfinite(c.overtake_permission_lookahead_m) ||
      c.overtake_permission_lookahead_m < 0.0) {
    return "invalid overtake permission lookahead";
  }
  for (const auto &rule : c.overtake_permission_rules) {
    if (!std::isfinite(rule.s_start_m) || !std::isfinite(rule.s_end_m)) {
      return "overtake permission rule contains non-finite s";
    }
  }
  if (!(c.target_missing_prediction_grace_sec >= 0.0 &&
        c.target_missing_forget_sec > c.target_missing_prediction_grace_sec &&
        c.target_missing_uncertainty_growth_mps >= 0.0 &&
        c.target_missing_recovery_speed_mps >= c.safe_stop_speed_mps &&
        c.target_missing_recovery_speed_mps <= c.normal_speed_mps)) {
    return "invalid target-missing recovery configuration";
  }
  if (!(c.follow_desired_extra_gap_m >= 0.0 && c.follow_gap_gain_per_s > 0.0 &&
        c.follow_closing_speed_gain >= 0.0)) {
    return "invalid blocked-follow gap control configuration";
  }
  if (!(std::isfinite(c.preventive_side_role_trigger_clearance_m) &&
        c.preventive_side_role_trigger_clearance_m >
            c.opponent_hard_clearance_m &&
        std::isfinite(c.preventive_side_role_tie_band_m) &&
        c.preventive_side_role_tie_band_m >= 0.0 &&
        std::isfinite(c.preventive_side_role_max_abs_delta_s_m) &&
        c.preventive_side_role_max_abs_delta_s_m >
            c.preventive_side_role_tie_band_m &&
        std::isfinite(c.preventive_side_role_min_lateral_separation_m) &&
        c.preventive_side_role_min_lateral_separation_m > 0.0 &&
        std::isfinite(c.preventive_side_role_min_tangent_progress_mps) &&
        c.preventive_side_role_min_tangent_progress_mps > 0.0 &&
        std::isfinite(c.preventive_side_role_max_track_heading_error_rad) &&
        c.preventive_side_role_max_track_heading_error_rad > 0.0 &&
        c.preventive_side_role_max_track_heading_error_rad <
            1.5707963267948966 &&
        std::isfinite(c.preventive_side_role_max_relative_heading_error_rad) &&
        c.preventive_side_role_max_relative_heading_error_rad > 0.0 &&
        c.preventive_side_role_max_relative_heading_error_rad <
            1.5707963267948966 &&
        std::isfinite(c.preventive_side_role_separation_epsilon_m) &&
        c.preventive_side_role_separation_epsilon_m > 0.0 &&
        c.preventive_side_role_separation_epsilon_m <=
            c.opponent_hard_clearance_m &&
        std::isfinite(c.preventive_side_role_follower_speed_reduction_mps) &&
        c.preventive_side_role_follower_speed_reduction_mps > 0.0 &&
        std::isfinite(c.preventive_side_role_follower_gap_gain_per_s) &&
        c.preventive_side_role_follower_gap_gain_per_s > 0.0 &&
        std::isfinite(c.preventive_side_role_follower_min_gap_m) &&
        c.preventive_side_role_follower_min_gap_m >
            c.opponent_hard_clearance_m &&
        std::isfinite(c.preventive_side_role_controller_response_sec) &&
        c.preventive_side_role_controller_response_sec >= 0.0 &&
        std::isfinite(c.preventive_side_role_deadline_response_margin_sec) &&
        c.preventive_side_role_deadline_response_margin_sec > 0.0 &&
        std::isfinite(c.preventive_side_role_neutral_speed_reduction_mps) &&
        c.preventive_side_role_neutral_speed_reduction_mps > 0.0 &&
        c.preventive_side_role_role_confirm_samples >= 2 &&
        std::isfinite(c.preventive_side_role_prediction_horizon_sec) &&
        c.preventive_side_role_prediction_horizon_sec > 0.0 &&
        std::isfinite(c.preventive_side_role_prediction_step_sec) &&
        c.preventive_side_role_prediction_step_sec > 0.0 &&
        c.preventive_side_role_prediction_step_sec <=
            c.preventive_side_role_prediction_horizon_sec &&
        std::isfinite(c.preventive_side_role_yield_peer_speed_mps) &&
        c.preventive_side_role_yield_peer_speed_mps >= 0.0 &&
        c.preventive_side_role_yield_min_samples >= 3 &&
        std::isfinite(c.preventive_side_role_yield_min_span_sec) &&
        c.preventive_side_role_yield_min_span_sec > 0.0 &&
        std::isfinite(c.preventive_side_role_release_margin_m) &&
        c.preventive_side_role_release_margin_m > 0.0 &&
        std::isfinite(c.preventive_side_role_release_prediction_sec) &&
        c.preventive_side_role_release_prediction_sec > 0.0 &&
        std::isfinite(c.preventive_side_role_escape_crawl_speed_mps) &&
        c.preventive_side_role_escape_crawl_speed_mps >=
            c.safe_stop_speed_mps &&
        c.preventive_side_role_escape_crawl_speed_mps <= c.normal_speed_mps &&
        std::isfinite(c.preventive_side_role_exit_clearance_m) &&
        c.preventive_side_role_exit_clearance_m >
            c.preventive_side_role_trigger_clearance_m &&
        c.preventive_side_role_exit_samples >= 1)) {
    return "invalid preventive side-role configuration";
  }
  if (!(c.wheel_base_m > 0.0 && c.front_overhang_m >= 0.0 &&
        c.rear_overhang_m >= 0.0 && c.left_extent_m > 0.0 &&
        c.right_extent_m > 0.0 && c.wall_hard_margin_m >= 0.0 &&
        c.opponent_hard_clearance_m >= 0.0)) {
    return "invalid vehicle footprint";
  }
  if (!(c.sigma_multiplier > 0.0 && c.sigma_min_margin_m >= 0.0 &&
        c.sigma_max_margin_m >= c.sigma_min_margin_m)) {
    return "invalid opponent uncertainty limits";
  }
  if (!(c.hard_max_steer_rad >= c.planner_max_steer_rad &&
        c.hard_max_steer_rad <= kVehicleMaximumSteerRad &&
        c.planner_max_steer_rad > 0.0 &&
        c.planner_max_steer_rad <= kVehicleMaximumSteerRad &&
        c.max_steer_rate_radps > 0.0 &&
        c.max_steer_rate_radps <= kVehicleMaximumSteerRateRadps &&
        c.min_acceleration_mps2 < 0.0 && c.max_acceleration_mps2 > 0.0 &&
        c.max_acceleration_jerk_mps3 > 0.0 &&
        c.max_deceleration_jerk_mps3 > 0.0 &&
        c.lateral_acceleration_limit_mps2 > 0.0 &&
        c.collision_max_step_m > 0.0 && c.collision_max_yaw_step_rad > 0.0 &&
        c.lateral_tracking_margin_m >= 0.0 &&
        c.longitudinal_tracking_margin_m >= 0.0 &&
        c.opponent_lateral_tracking_margin_m >= 0.0 &&
        c.opponent_longitudinal_tracking_margin_m >= 0.0 &&
        c.candidate_entry_speed_tolerance_mps >= 0.0 &&
        c.reference_curvature_sanity_limit_radpm > 0.0)) {
    return "invalid motion limits";
  }
  if (c.horizon_points < 2 || c.horizon_points > 1000 ||
      c.mpc_wp_id_offset < 0 || c.nearest_index_uncertainty < 0 ||
      c.projection_initial_half_width_m <= 0.0 ||
      c.projection_follow_half_width_m <= 0.0 ||
      c.projection_max_backward_m < 0.0 || c.tie_break_epsilon < 0.0) {
    return "invalid downstream horizon configuration";
  }
  if (!(c.normal_speed_mps > 0.0 && c.stop_cost > c.free_run_return_cost &&
        c.rear_return_cost_threshold >= 0 &&
        c.safe_stop_release_cost < c.stop_cost && c.safe_stop_speed_mps > 0.0 &&
        c.candidate_cost_hysteresis >= 0 && c.safe_stop_release_cycles >= 1)) {
    return "invalid cost-to-speed configuration";
  }
  if (!(c.planner_rate_hz > 0.0 && c.planning_warn_ms > 0.0 &&
        c.planning_deadline_ms > c.planning_warn_ms && c.ego_stale_sec > 0.0 &&
        c.opponent_stale_sec > 0.0 && c.opponent_max_position_jump_m > 0.0 &&
        c.debug_costmap_rate_hz > 0.0 && c.normal_mode_min_hold_sec >= 0.0 &&
        c.overrun_previous_max_age_sec > 0.0 && c.overrun_stop_cycles >= 2)) {
    return "invalid timing configuration";
  }
  if (!(c.mpc_health_infeasible_count_threshold >= 1 &&
        c.mpc_health_solve_time_warn_ms > 0.0 && c.mpc_health_v_max_mps > 0.0 &&
        c.mpc_health_v_max_mps <= c.normal_speed_mps &&
        c.mpc_health_stale_time_sec > 0.0 &&
        c.mpc_health_release_samples >= 1)) {
    return "invalid MPC health guard configuration";
  }
  return {};
}

} // namespace state_lattice_overtake_planner
