#pragma once

#include <array>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner {

enum class ControllerTrackabilityProfile {
  UNKNOWN = 0,
  SHADOW_ONLY = 1,
  PURE_PURSUIT = 2,
  INSTANT = 3,
};

ControllerTrackabilityProfile
parseControllerTrackabilityProfile(const std::string &value);
const char *toString(ControllerTrackabilityProfile profile);
std::string validateControllerTrackabilityProfileContract(
    ControllerTrackabilityProfile profile, bool live_control_output_enabled,
    bool instant_control_enabled);

struct OvertakePermissionRule {
  std::string name;
  double s_start_m{0.0};
  double s_end_m{0.0};
  bool allow_overtake{true};
};

struct ActiveOvertakePermission {
  bool active{false};
  std::string name;
  bool allow_overtake{true};
  std::string reason{"permission_profile_disabled"};
};

struct PlannerConfig {
  // Launch-owned controller boundary. This does not alter common candidate
  // safety; it prevents PP-specific and instant-specific trackability
  // authority from being confused at the Node output boundary.
  ControllerTrackabilityProfile controller_trackability_profile{
      ControllerTrackabilityProfile::SHADOW_ONLY};
  // Debug-only master switch. Disabling safety evaluation keeps numerical and
  // vehicle-dynamics validity checks, but bypasses environmental fail-safes.
  bool safety_evaluation_enabled{true};
  // Exact-spatial object generation is not publication authority. This keeps
  // the existing safety-evaluated resampling path enabled; the node-owned live
  // publication gate remains independently default OFF.
  bool experimental_exact_spatial_follow_shadow_enabled{true};
  std::vector<int> cost_levels{6, 9, 12, 15, 18, 60, 70, 80, 90, 100};
  std::vector<double> wall_distance_thresholds_m{0.1, 0.3, 0.5};
  std::vector<double> object_distance_thresholds_m{0.8, 1.0, 1.3, 1.5, 1.7};
  std::vector<double> reference_distance_thresholds_m{0.5, 0.8, 1.2, 1.5};
  double reference_extra_step_m{0.2};
  double sigma_multiplier{3.0};
  double sigma_min_margin_m{0.15};
  double sigma_max_margin_m{1.0};

  std::string reference_package{"multi_purpose_mpc_ros"};
  std::string reference_csv{"env/final_ver3/traj_mincurv_manual.csv"};
  // Planning geometry. The output reference remains separately configurable
  // because the downstream controller consumes its own waypoint sequence.
  std::string output_reference_package{"multi_purpose_mpc_ros"};
  std::string output_reference_csv{"env/final_ver3/traj_mincurv_manual.csv"};
  std::string wall_map_package{"multi_purpose_mpc_ros"};
  std::string wall_map_yaml{"env/final_ver3/occupancy_grid_map.yaml"};
  std::string map_frame{"map"};
  double expected_map_resolution_m{0.1};

  // Runtime YAML enables this profile. It is disabled in the C++ default so
  // unit tests and embedders without a course-specific CSV remain portable.
  bool overtake_permission_profile_enabled{false};
  std::string overtake_permission_package{"state_lattice_overtake_planner"};
  std::string overtake_permission_csv{"config/overtake_permission.csv"};
  bool default_overtake_allowed{true};
  double overtake_permission_lookahead_m{8.0};
  std::vector<OvertakePermissionRule> overtake_permission_rules;

  double wheel_base_m{1.087};
  double front_overhang_m{0.467};
  double rear_overhang_m{0.510};
  double left_extent_m{0.650};
  double right_extent_m{0.650};
  double wall_hard_margin_m{0.25};
  double opponent_hard_clearance_m{0.25};

  std::vector<double> lateral_targets_m{0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2};
  std::vector<double> tangent_scales{0.8, 1.0, 1.2};
  int sampling_points{5};
  double base_distance_m{2.0};
  double max_distance_m{10.0};
  double max_distance_speed_mps{8.333333};
  double minimum_lateral_transition_distance_m{4.0};
  double lateral_transition_distance_gain{3.0};
  bool adaptive_pass_offset_enabled{true};
  double pass_lateral_extra_margin_m{0.05};
  double max_adaptive_lateral_offset_m{2.2};
  double minimum_obstacle_transition_distance_m{1.0};
  double front_detection_radius_m{2.0};
  double frontmost_s_tolerance_m{0.1};
  int front_enter_cycles{1};
  int front_release_cycles{5};
  bool initial_detection_sweep_enabled{true};
  // Read-only early awareness. This selector never enters FrontDetector,
  // candidate generation, safety admission, or lateral/speed authority.
  bool early_aware_enabled{true};
  double early_aware_base_distance_m{4.0};
  double early_aware_speed_horizon_sec{0.25};
  double early_aware_max_deceleration_mps2{0.9};
  double early_aware_min_distance_m{8.0};
  double early_aware_max_distance_m{30.0};
  double early_aware_min_tangent_speed_mps{0.05};
  double early_aware_reverse_tangent_tolerance_mps{0.05};
  double early_aware_max_track_heading_error_rad{0.66};
  double early_aware_min_forward_delta_s_m{0.10};
  int early_aware_required_fresh_stamps{2};
  int return_required_cycles{5};
  double passed_target_gap_m{6.0};
  double return_prediction_sec{1.0};
  double return_predicted_gap_m{4.0};
  double return_lateral_error_m{0.20};
  double return_heading_error_rad{0.0872665};
  double rear_safety_distance_m{6.0};
  double rear_terminal_distance_m{1.5};
  double rear_sampling_angle_rad{0.5};
  int rear_sampling_points{5};
  int rear_return_cost_threshold{70};
  double rear_prediction_horizon_sec{1.5};

  double target_missing_prediction_grace_sec{0.5};
  double target_missing_forget_sec{2.0};
  double target_missing_uncertainty_growth_mps{0.25};
  double target_missing_recovery_speed_mps{2.0};

  double follow_desired_extra_gap_m{1.0};
  double follow_gap_gain_per_s{0.8};
  double follow_closing_speed_gain{0.5};

  // Preventive D1/D2 side-by-side responsibility. This is deliberately above
  // the immutable hard-clearance threshold: the role may slow/hold before a
  // violation, but it never exempts the current or future collision checks.
  double preventive_side_role_trigger_clearance_m{2.0};
  double preventive_side_role_tie_band_m{0.20};
  double preventive_side_role_max_abs_delta_s_m{3.0};
  double preventive_side_role_min_lateral_separation_m{0.50};
  // Defaults are bounded just above the observed v2 corner evidence
  // (0.6432 rad track-heading, 0.5599 rad relative-heading). Eligibility also
  // requires strictly positive tangent progress for both vehicles, so these
  // bounds never authorize stationary or counterflow geometry.
  double preventive_side_role_min_tangent_progress_mps{0.05};
  double preventive_side_role_max_track_heading_error_rad{0.66};
  double preventive_side_role_max_relative_heading_error_rad{0.57};
  double preventive_side_role_separation_epsilon_m{1.0e-3};
  double preventive_side_role_follower_speed_reduction_mps{0.50};
  double preventive_side_role_follower_gap_gain_per_s{0.80};
  double preventive_side_role_follower_min_gap_m{2.00};
  double preventive_side_role_controller_response_sec{0.30};
  double preventive_side_role_deadline_response_margin_sec{0.30};
  double preventive_side_role_neutral_speed_reduction_mps{0.35};
  int preventive_side_role_role_confirm_samples{3};
  double preventive_side_role_prediction_horizon_sec{3.0};
  double preventive_side_role_prediction_step_sec{0.05};
  double preventive_side_role_yield_peer_speed_mps{0.05};
  int preventive_side_role_yield_min_samples{3};
  double preventive_side_role_yield_min_span_sec{0.20};
  double preventive_side_role_release_margin_m{0.25};
  double preventive_side_role_release_prediction_sec{0.50};
  double preventive_side_role_escape_crawl_speed_mps{0.80};
  double preventive_side_role_exit_clearance_m{2.50};
  int preventive_side_role_exit_samples{3};

  // Pure Pursuit route trackability proxy. Keep these aligned with the
  // downstream PP/Mux command limits; the instant-control route has separate
  // parameters and intentionally remains independent.
  double hard_max_steer_rad{0.64};
  double planner_max_steer_rad{0.64};
  double max_steer_rate_radps{128.0};
  double min_acceleration_mps2{-0.9};
  double max_acceleration_mps2{3.0};
  double max_acceleration_jerk_mps3{3.0};
  double max_deceleration_jerk_mps3{3.0};
  double lateral_acceleration_limit_mps2{30.0};
  double collision_max_step_m{0.05};
  double collision_max_yaw_step_rad{0.0174533};
  double lateral_tracking_margin_m{0.25};
  double longitudinal_tracking_margin_m{0.10};
  // Tracking margins used against dynamic opponents are intentionally
  // independent from the wall/output-corridor margins above. Reusing the wall
  // margin here double-counted controller uncertainty together with V2X
  // covariance and opponent_hard_clearance_m, making a physically open pass
  // lane appear blocked on most of the supplied course.
  double opponent_lateral_tracking_margin_m{0.0};
  double opponent_longitudinal_tracking_margin_m{0.10};
  double candidate_entry_speed_tolerance_mps{0.15};
  bool allow_reverse{false};
  bool reference_speed_limit_enabled{true};
  double reference_curvature_sanity_limit_radpm{1.0};

  int horizon_points{20};
  int mpc_wp_id_offset{2};
  int nearest_index_uncertainty{1};
  double projection_initial_half_width_m{3.0};
  double projection_follow_half_width_m{2.0};
  double projection_max_backward_m{0.05};
  double tie_break_epsilon{1.0e-6};

  double normal_speed_mps{9.722222};
  int free_run_return_cost{70};
  int stop_cost{380};
  double safe_stop_speed_mps{0.2};
  double planner_rate_hz{20.0};
  double planning_warn_ms{40.0};
  double planning_deadline_ms{50.0};
  double ego_stale_sec{0.5};
  double opponent_stale_sec{0.5};
  double opponent_max_position_jump_m{10.0};
  double debug_costmap_rate_hz{5.0};
  double normal_mode_min_hold_sec{0.5};
  int candidate_cost_hysteresis{5};
  int safe_stop_release_cost{375};
  int safe_stop_release_cycles{5};
  double overrun_previous_max_age_sec{0.1};
  int overrun_stop_cycles{2};

  bool mpc_health_speed_guard_enabled{true};
  int mpc_health_infeasible_count_threshold{1};
  double mpc_health_solve_time_warn_ms{80.0};
  double mpc_health_v_max_mps{3.0};
  double mpc_health_stale_time_sec{0.60};
  int mpc_health_release_samples{3};

  std::string own_vehicle_id{"auto"};
  std::string ego_topic{"/localization/kinematic_state"};
  std::string opponent_topic{"/v2x/vehicle_positions"};
  std::string mpc_health_topic{"/mpc/speed_profile_debug"};
  std::string override_topic{"/overtake/reference_override"};
};

void applyResolvedControllerTrackabilityEnvelope(
    PlannerConfig *config, double instant_maximum_steering_angle_rad,
    double instant_maximum_steering_rate_radps);
std::string validateConfig(const PlannerConfig &config);

} // namespace state_lattice_overtake_planner
