#include "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <type_traits>

namespace state_lattice_overtake_planner {
namespace {

using Authorized =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using CandidatePoint = multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint;
using Digest = overtake_transport_contract::c002ay0::Digest;

#ifndef STATE_LATTICE_AY0_IMPLEMENTATION_FINGERPRINT
#error "STATE_LATTICE_AY0_IMPLEMENTATION_FINGERPRINT must be provided by CMake"
#endif

template <typename Integer>
void appendInteger(std::vector<std::uint8_t> &bytes, Integer value) {
  static_assert(std::is_integral_v<Integer>);
  using Unsigned = std::make_unsigned_t<Integer>;
  Unsigned bits = static_cast<Unsigned>(value);
  for (std::size_t index = 0U; index < sizeof(Integer); ++index) {
    bytes.push_back(static_cast<std::uint8_t>(
        (bits >> static_cast<unsigned>(8U * index)) & 0xffU));
  }
}

void appendDouble(std::vector<std::uint8_t> &bytes, double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendInteger(bytes, bits);
}

void appendString(std::vector<std::uint8_t> &bytes, std::string_view value) {
  appendInteger(bytes, static_cast<std::uint32_t>(value.size()));
  bytes.insert(bytes.end(), value.begin(), value.end());
}

void appendDigest(std::vector<std::uint8_t> &bytes, const Digest &digest) {
  bytes.insert(bytes.end(), digest.begin(), digest.end());
}

void appendBool(std::vector<std::uint8_t> &bytes, bool value) {
  appendInteger(bytes, static_cast<std::uint8_t>(value ? 1U : 0U));
}

template <typename Value>
void appendVector(std::vector<std::uint8_t> &bytes,
                  const std::vector<Value> &values) {
  appendInteger(bytes, static_cast<std::uint32_t>(values.size()));
  for (const auto &value : values) {
    if constexpr (std::is_floating_point_v<Value>) {
      appendDouble(bytes, static_cast<double>(value));
    } else {
      appendInteger(bytes, value);
    }
  }
}

std::int64_t timeNs(const builtin_interfaces::msg::Time &stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return -1;
  }
  constexpr std::int64_t kNsecPerSec = 1000000000LL;
  if (static_cast<std::int64_t>(stamp.sec) >
      std::numeric_limits<std::int64_t>::max() / kNsecPerSec) {
    return -1;
  }
  return static_cast<std::int64_t>(stamp.sec) * kNsecPerSec +
         static_cast<std::int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time timeFromNs(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  if (nanoseconds < 0) {
    return stamp;
  }
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

builtin_interfaces::msg::Duration durationFromSeconds(double seconds) {
  builtin_interfaces::msg::Duration duration;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return duration;
  }
  const auto nanoseconds =
      static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
  duration.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

Digest hashDomain(std::string_view domain,
                  const std::vector<std::uint8_t> &payload) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(domain.size() + payload.size() + 4U);
  appendString(bytes, domain);
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  return overtake_transport_contract::c002ay0::sha256(bytes);
}

Digest worldSafetySnapshotDigest(const Digest &static_safety_digest,
                                 const EgoState &ego,
                                 const std::vector<OpponentState> &opponents,
                                 const builtin_interfaces::msg::Time &stamp) {
  std::vector<std::uint8_t> bytes;
  appendDigest(bytes, static_safety_digest);
  appendInteger(bytes, stamp.sec);
  appendInteger(bytes, stamp.nanosec);
  for (double value : {ego.x, ego.y, ego.yaw, ego.speed_mps, ego.yaw_rate_radps,
                       ego.curvature, ego.frenet.s, ego.frenet.d}) {
    appendDouble(bytes, value);
  }
  std::vector<const OpponentState *> ordered;
  ordered.reserve(opponents.size());
  for (const auto &opponent : opponents) {
    ordered.push_back(&opponent);
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const OpponentState *lhs, const OpponentState *rhs) {
              return lhs->id < rhs->id;
            });
  appendInteger(bytes, static_cast<std::uint32_t>(ordered.size()));
  for (const auto *opponent : ordered) {
    appendString(bytes, opponent->id);
    for (double value :
         {opponent->x, opponent->y, opponent->yaw, opponent->speed_mps,
          opponent->vx_mps, opponent->vy_mps, opponent->uncertainty_x_m,
          opponent->uncertainty_y_m, opponent->frenet.s, opponent->frenet.d,
          opponent->stamp_sec}) {
      appendDouble(bytes, value);
    }
  }
  return hashDomain("STATE_LATTICE_WORLD_SAFETY_SNAPSHOT_V1", bytes);
}

bool sameStart(const CandidateTrajectory &candidate, const EgoState &ego) {
  if (candidate.dense.empty()) {
    return false;
  }
  const auto &start = candidate.dense.front();
  return std::hypot(start.x - ego.x, start.y - ego.y) <= 1.0e-6 &&
         std::abs(normalizeAngle(start.yaw - ego.yaw)) <= 1.0e-6;
}

bool digestMissing(const Digest &digest) {
  return std::all_of(digest.begin(), digest.end(),
                     [](std::uint8_t value) { return value == 0U; });
}

bool convertPoints(const CandidateTrajectory &candidate,
                   const PlannerConfig &config,
                   Authorized::_points_type *points, double *total_arc_m) {
  if (points == nullptr || total_arc_m == nullptr ||
      candidate.dense.size() < 2U ||
      candidate.dense.size() >
          overtake_transport_contract::c002ay0::kMaxCartesianPoints) {
    return false;
  }
  points->clear();
  points->reserve(candidate.dense.size());
  *total_arc_m = 0.0;
  double previous_time = -1.0;
  for (std::size_t index = 0U; index < candidate.dense.size(); ++index) {
    const auto &source = candidate.dense[index];
    if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
        !std::isfinite(source.yaw) || !std::isfinite(source.kappa) ||
        !std::isfinite(source.speed_mps) || !std::isfinite(source.time_sec) ||
        source.speed_mps < 0.0 || source.time_sec < 0.0 ||
        (index > 0U && source.time_sec <= previous_time)) {
      return false;
    }
    CandidatePoint point;
    point.time_from_start = durationFromSeconds(source.time_sec);
    point.position_x_m = source.x;
    point.position_y_m = source.y;
    point.position_z_m = 0.0;
    point.orientation_z = std::sin(0.5 * source.yaw);
    point.orientation_w = std::cos(0.5 * source.yaw);
    point.longitudinal_velocity_mps = static_cast<float>(source.speed_mps);
    point.lateral_velocity_mps = 0.0F;
    point.acceleration_mps2 = 0.0F;
    point.heading_rate_rps =
        static_cast<float>(source.speed_mps * source.kappa);
    point.front_wheel_angle_rad =
        static_cast<float>(std::atan(config.wheel_base_m * source.kappa));
    point.rear_wheel_angle_rad = 0.0F;
    if (index > 0U) {
      *total_arc_m += std::hypot(source.x - candidate.dense[index - 1U].x,
                                 source.y - candidate.dense[index - 1U].y);
    }
    points->push_back(point);
    previous_time = source.time_sec;
  }
  return std::isfinite(*total_arc_m) && *total_arc_m > 0.0;
}

Ay0ShadowProposalResult fail(Ay0ShadowProposalFailure failure,
                             std::string reason) {
  Ay0ShadowProposalResult result;
  result.failure = failure;
  result.reason = std::move(reason);
  return result;
}

} // namespace

bool baseSnapshotCurrentForProposal(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &base,
    const builtin_interfaces::msg::Time &plan_stamp,
    double maximum_snapshot_age_sec) {
  if (!std::isfinite(maximum_snapshot_age_sec) ||
      maximum_snapshot_age_sec <= 0.0) {
    return false;
  }
  const auto plan_ns = timeNs(plan_stamp);
  const auto record_ns = timeNs(base.record_stamp);
  const auto source_ns = timeNs(base.base_source_stamp);
  const auto lease_ns = timeNs(base.lease_valid_until);
  const auto maximum_snapshot_age_ns =
      static_cast<std::int64_t>(std::llround(maximum_snapshot_age_sec * 1.0e9));
  return plan_ns >= 0 && record_ns >= 0 && source_ns >= 0 && lease_ns >= 0 &&
         maximum_snapshot_age_ns > 0 && plan_ns >= record_ns &&
         plan_ns >= source_ns &&
         plan_ns - record_ns <= maximum_snapshot_age_ns &&
         plan_ns < lease_ns;
}

const char *toString(Ay0ShadowProposalFailure failure) {
  switch (failure) {
  case Ay0ShadowProposalFailure::NONE:
    return "none";
  case Ay0ShadowProposalFailure::INVALID_INPUT:
    return "invalid_input";
  case Ay0ShadowProposalFailure::BASE_INVALID:
    return "base_invalid";
  case Ay0ShadowProposalFailure::BASE_NOT_CURRENT:
    return "base_not_current";
  case Ay0ShadowProposalFailure::CANDIDATE_NOT_FEASIBLE:
    return "candidate_not_feasible";
  case Ay0ShadowProposalFailure::CANDIDATE_START_MISMATCH:
    return "candidate_start_mismatch";
  case Ay0ShadowProposalFailure::GEOMETRY_INVALID:
    return "geometry_invalid";
  case Ay0ShadowProposalFailure::PROVENANCE_INVALID:
    return "provenance_invalid";
  }
  return "unknown";
}

Digest stateLatticeSafetyEvaluatorImplementationDigest() {
  std::vector<std::uint8_t> bytes;
  appendString(bytes, STATE_LATTICE_AY0_IMPLEMENTATION_FINGERPRINT);
  return hashDomain("STATE_LATTICE_SAFETY_EVALUATOR_IMPLEMENTATION_V1", bytes);
}

Digest stateLatticeSafetyEvaluatorConfigDigest(const PlannerConfig &config,
                                               const GridMap &map,
                                               const FrenetFrame &frame) {
  std::vector<std::uint8_t> bytes;
  appendString(bytes, config.map_frame);
  appendString(bytes, config.reference_package);
  appendString(bytes, config.reference_csv);
  appendString(bytes, config.wall_map_package);
  appendString(bytes, config.wall_map_yaml);
  appendString(bytes, config.overtake_permission_package);
  appendString(bytes, config.overtake_permission_csv);
  appendString(bytes, config.own_vehicle_id);
  appendBool(bytes, config.safety_evaluation_enabled);
  appendBool(bytes, config.overtake_permission_profile_enabled);
  appendBool(bytes, config.default_overtake_allowed);
  appendBool(bytes, config.adaptive_pass_offset_enabled);
  appendBool(bytes, config.initial_detection_sweep_enabled);
  appendBool(bytes, config.allow_reverse);
  appendBool(bytes, config.reference_speed_limit_enabled);
  appendBool(bytes, config.mpc_health_speed_guard_enabled);
  appendString(bytes, "controller_trackability_profile");
  appendString(bytes, toString(config.controller_trackability_profile));
  appendVector(bytes, config.cost_levels);
  appendVector(bytes, config.wall_distance_thresholds_m);
  appendVector(bytes, config.object_distance_thresholds_m);
  appendVector(bytes, config.reference_distance_thresholds_m);
  appendVector(bytes, config.lateral_targets_m);
  appendVector(bytes, config.tangent_scales);
  for (double value : {
           config.reference_extra_step_m,
           config.sigma_multiplier,
           config.sigma_min_margin_m,
           config.sigma_max_margin_m,
           config.expected_map_resolution_m,
           config.overtake_permission_lookahead_m,
           config.wheel_base_m,
           config.front_overhang_m,
           config.rear_overhang_m,
           config.left_extent_m,
           config.right_extent_m,
           config.wall_hard_margin_m,
           config.opponent_hard_clearance_m,
           config.base_distance_m,
           config.max_distance_m,
           config.max_distance_speed_mps,
           config.minimum_lateral_transition_distance_m,
           config.lateral_transition_distance_gain,
           config.pass_lateral_extra_margin_m,
           config.max_adaptive_lateral_offset_m,
           config.minimum_obstacle_transition_distance_m,
           config.front_detection_radius_m,
           config.frontmost_s_tolerance_m,
           config.passed_target_gap_m,
           config.return_prediction_sec,
           config.return_predicted_gap_m,
           config.return_lateral_error_m,
           config.return_heading_error_rad,
           config.rear_safety_distance_m,
           config.rear_terminal_distance_m,
           config.rear_sampling_angle_rad,
           config.rear_prediction_horizon_sec,
           config.target_missing_prediction_grace_sec,
           config.target_missing_forget_sec,
           config.target_missing_uncertainty_growth_mps,
           config.target_missing_recovery_speed_mps,
           config.follow_desired_extra_gap_m,
           config.follow_gap_gain_per_s,
           config.follow_closing_speed_gain,
           config.preventive_side_role_trigger_clearance_m,
           config.preventive_side_role_tie_band_m,
           config.preventive_side_role_max_abs_delta_s_m,
           config.preventive_side_role_min_lateral_separation_m,
           config.preventive_side_role_min_tangent_progress_mps,
           config.preventive_side_role_max_track_heading_error_rad,
           config.preventive_side_role_max_relative_heading_error_rad,
           config.preventive_side_role_separation_epsilon_m,
           config.preventive_side_role_follower_speed_reduction_mps,
           config.preventive_side_role_follower_gap_gain_per_s,
           config.preventive_side_role_follower_min_gap_m,
           config.preventive_side_role_controller_response_sec,
           config.preventive_side_role_deadline_response_margin_sec,
           config.preventive_side_role_neutral_speed_reduction_mps,
           config.preventive_side_role_prediction_horizon_sec,
           config.preventive_side_role_prediction_step_sec,
           config.preventive_side_role_yield_peer_speed_mps,
           config.preventive_side_role_yield_min_span_sec,
           config.preventive_side_role_release_margin_m,
           config.preventive_side_role_release_prediction_sec,
           config.preventive_side_role_escape_crawl_speed_mps,
           config.preventive_side_role_exit_clearance_m,
           config.hard_max_steer_rad,
           config.planner_max_steer_rad,
           config.max_steer_rate_radps,
           config.min_acceleration_mps2,
           config.max_acceleration_mps2,
           config.max_acceleration_jerk_mps3,
           config.max_deceleration_jerk_mps3,
           config.lateral_acceleration_limit_mps2,
           config.collision_max_step_m,
           config.collision_max_yaw_step_rad,
           config.lateral_tracking_margin_m,
           config.longitudinal_tracking_margin_m,
           config.opponent_lateral_tracking_margin_m,
           config.opponent_longitudinal_tracking_margin_m,
           config.candidate_entry_speed_tolerance_mps,
           config.reference_curvature_sanity_limit_radpm,
           config.projection_initial_half_width_m,
           config.projection_follow_half_width_m,
           config.projection_max_backward_m,
           config.tie_break_epsilon,
           config.normal_speed_mps,
           config.safe_stop_speed_mps,
           config.planner_rate_hz,
           config.planning_warn_ms,
           config.planning_deadline_ms,
           config.ego_stale_sec,
           config.opponent_stale_sec,
           config.opponent_max_position_jump_m,
           config.debug_costmap_rate_hz,
           config.normal_mode_min_hold_sec,
           config.overrun_previous_max_age_sec,
           config.mpc_health_solve_time_warn_ms,
           config.mpc_health_v_max_mps,
           config.mpc_health_stale_time_sec,
       }) {
    appendDouble(bytes, value);
  }
  for (int value : {
           config.sampling_points,
           config.front_enter_cycles,
           config.front_release_cycles,
           config.return_required_cycles,
           config.rear_sampling_points,
           config.rear_return_cost_threshold,
           config.horizon_points,
           config.mpc_wp_id_offset,
           config.nearest_index_uncertainty,
           config.free_run_return_cost,
           config.stop_cost,
           config.candidate_cost_hysteresis,
           config.safe_stop_release_cost,
           config.safe_stop_release_cycles,
           config.overrun_stop_cycles,
           config.mpc_health_infeasible_count_threshold,
           config.mpc_health_release_samples,
           config.preventive_side_role_yield_min_samples,
           config.preventive_side_role_exit_samples,
           config.preventive_side_role_role_confirm_samples,
       }) {
    appendInteger(bytes, value);
  }
  appendInteger(bytes, static_cast<std::uint32_t>(
                           config.overtake_permission_rules.size()));
  for (const auto &rule : config.overtake_permission_rules) {
    appendString(bytes, rule.name);
    appendDouble(bytes, rule.s_start_m);
    appendDouble(bytes, rule.s_end_m);
    appendBool(bytes, rule.allow_overtake);
  }
  appendInteger(bytes, map.staticGeneration());
  appendInteger(bytes, static_cast<std::uint64_t>(map.width()));
  appendInteger(bytes, static_cast<std::uint64_t>(map.height()));
  appendDouble(bytes, map.resolution());
  appendDouble(bytes, map.originX());
  appendDouble(bytes, map.originY());
  appendInteger(bytes, static_cast<std::uint32_t>(map.occupiedCells().size()));
  bytes.insert(bytes.end(), map.occupiedCells().begin(),
               map.occupiedCells().end());
  appendInteger(bytes, static_cast<std::uint32_t>(frame.points().size()));
  for (const auto &point : frame.points()) {
    for (double value :
         {point.x, point.y, point.yaw, point.s, point.kappa, point.speed_mps}) {
      appendDouble(bytes, value);
    }
  }
  return hashDomain("STATE_LATTICE_SAFETY_EVALUATOR_CONFIG_V1", bytes);
}

Ay0ShadowProposalResult buildAy0ShadowProposal(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &base,
    const CandidateTrajectory &candidate, const EgoState &ego,
    const std::vector<OpponentState> &opponents, const PlannerConfig &config,
    const Ay0ShadowSafetyEvidence &safety_evidence,
    const builtin_interfaces::msg::Time &plan_stamp,
    const Ay0ShadowProposalIdentity &identity) {
  using namespace overtake_transport_contract::c002ay0;
  if (!config.safety_evaluation_enabled || !ego.valid ||
      config.map_frame != "map" || identity.planner_instance_id == 0U ||
      identity.attempt_id == 0U || identity.connector_transaction_id == 0U ||
      identity.authority_token == 0U || identity.safety_snapshot_id == 0U ||
      identity.plan_generation == 0U || identity.candidate_revision == 0U ||
      identity.target_id.empty() || identity.target_id.size() > 64U ||
      !std::isfinite(config.planner_rate_hz) ||
      config.planner_rate_hz <= 0.0 ||
      !std::isfinite(config.ego_stale_sec) || config.ego_stale_sec <= 0.0 ||
      !std::isfinite(config.opponent_stale_sec) ||
      config.opponent_stale_sec <= 0.0 ||
      digestMissing(safety_evidence.evaluator_implementation_sha256) ||
      digestMissing(safety_evidence.evaluator_config_sha256)) {
    return fail(Ay0ShadowProposalFailure::INVALID_INPUT, "invalid_input");
  }
  if (validateBaseSnapshotV1(base) != ValidationError::NONE ||
      base.authority_eligible || base.frame_id != config.map_frame) {
    return fail(Ay0ShadowProposalFailure::BASE_INVALID, "base_invalid");
  }
  const auto plan_ns = timeNs(plan_stamp);
  const auto lease_ns = timeNs(base.lease_valid_until);
  if (!baseSnapshotCurrentForProposal(base, plan_stamp,
                                      config.ego_stale_sec)) {
    return fail(Ay0ShadowProposalFailure::BASE_NOT_CURRENT, "base_not_current");
  }
  if (!candidate.feasible) {
    return fail(Ay0ShadowProposalFailure::CANDIDATE_NOT_FEASIBLE,
                "candidate_not_feasible");
  }
  if (!sameStart(candidate, ego)) {
    return fail(Ay0ShadowProposalFailure::CANDIDATE_START_MISMATCH,
                "candidate_start_mismatch");
  }

  Ay0ShadowProposalResult result;
  auto &trajectory = result.trajectory;
  trajectory.schema_version = Authorized::SCHEMA_V1_SHADOW;
  trajectory.authority_eligible = false;
  trajectory.plan_stamp = plan_stamp;
  trajectory.frame_id = config.map_frame;
  trajectory.plan_sample_key.race_arm_epoch = base.race_arm_epoch;
  trajectory.plan_sample_key.planner_instance_id = identity.planner_instance_id;
  trajectory.plan_sample_key.attempt_id = identity.attempt_id;
  trajectory.plan_sample_key.target_vehicle_id = identity.target_id;
  trajectory.plan_sample_key.pass_direction =
      candidate.goal_d_m >= ego.frenet.d ? 1 : -1;
  trajectory.plan_sample_key.connector_transaction_id =
      identity.connector_transaction_id;
  trajectory.plan_sample_key.plan_stamp = plan_stamp;
  trajectory.plan_sample_key.plan_generation = identity.plan_generation;
  trajectory.candidate_revision = identity.candidate_revision;
  trajectory.authority_token = identity.authority_token;
  trajectory.candidate_type = candidate.goal_d_m >= ego.frenet.d
                                  ? Authorized::CANDIDATE_PASS_LEFT
                                  : Authorized::CANDIDATE_PASS_RIGHT;
  trajectory.phase = Authorized::PHASE_PASSING;
  trajectory.authorization_state = Authorized::AUTHORIZATION_AUTHORIZED;
  trajectory.source_controller_instance_id = base.controller_instance_id;
  trajectory.source_controller_sequence = base.controller_sequence;
  trajectory.base_lease_id = base.base_lease_id;
  trajectory.base_lease_valid_until = base.lease_valid_until;
  trajectory.base_source_kind = base.base_source_kind;
  trajectory.base_source_stamp = base.base_source_stamp;
  trajectory.base_source_generation = base.base_source_generation;
  trajectory.base_original_point_count = base.base_original_point_count;
  trajectory.base_first_source_index = base.first_source_index;
  trajectory.base_last_source_index = base.last_source_index;
  trajectory.base_nearest_source_index = base.nearest_source_index;
  trajectory.base_source_digest_state = base.base_source_digest_state;
  trajectory.canonical_algorithm_version = base.canonical_algorithm_version;
  trajectory.base_geometry_sha256 = base.base_geometry_sha256;
  trajectory.base_source_sha256 = base.base_source_sha256;
  trajectory.base_snapshot_sha256 = base.snapshot_sha256;

  double total_arc_m = 0.0;
  if (!convertPoints(candidate, config, &trajectory.points, &total_arc_m)) {
    return fail(Ay0ShadowProposalFailure::GEOMETRY_INVALID, "geometry_invalid");
  }
  trajectory.original_candidate_point_count =
      static_cast<std::uint32_t>(trajectory.points.size());
  trajectory.total_arc_length_m = total_arc_m;
  trajectory.required_spatial_horizon_m = total_arc_m;
  trajectory.join_end_arc_length_m = total_arc_m;
  trajectory.post_join_arc_length_m = 0.0;

  trajectory.safety_snapshot_id = identity.safety_snapshot_id;
  trajectory.safety_evaluation_result = Authorized::SAFETY_PASSED;
  trajectory.safety_evaluation_stamp = plan_stamp;
  // Availability is checked by the faster PP cycle against this absolute
  // lease. A lease equal to exactly one planner period creates deterministic
  // gaps whenever the 20 Hz publisher has normal scheduling jitter, clearing
  // the accepted Cartesian cache and restarting steering acquisition. Keep
  // two nominal publications of continuity, while remaining below the
  // downstream 120 ms exact-evidence bound and both owning input stale limits.
  constexpr double kMaximumExactEvidenceLifetimeSec = 0.12;
  const double safety_lifetime_sec =
      std::min({2.0 / config.planner_rate_hz,
                config.ego_stale_sec, config.opponent_stale_sec,
                kMaximumExactEvidenceLifetimeSec});
  const auto safety_lifetime_ns = static_cast<std::int64_t>(
      std::llround(safety_lifetime_sec * 1.0e9));
  if (!std::isfinite(safety_lifetime_sec) || safety_lifetime_ns <= 0) {
    return fail(Ay0ShadowProposalFailure::INVALID_INPUT, "invalid_input");
  }
  const auto safety_until_ns =
      std::min(lease_ns, plan_ns + safety_lifetime_ns);
  if (safety_until_ns <= plan_ns) {
    return fail(Ay0ShadowProposalFailure::BASE_NOT_CURRENT, "base_not_current");
  }
  trajectory.safety_valid_until = timeFromNs(safety_until_ns);
  trajectory.world_safety_snapshot_sha256 = worldSafetySnapshotDigest(
      safety_evidence.evaluator_config_sha256, ego, opponents, plan_stamp);
  trajectory.safety_evaluator_implementation_sha256 =
      safety_evidence.evaluator_implementation_sha256;
  trajectory.safety_evaluator_config_sha256 =
      safety_evidence.evaluator_config_sha256;
  trajectory.controller_implementation_sha256 =
      base.controller_implementation_sha256;
  trajectory.controller_config_sha256 = base.controller_config_sha256;
  trajectory.candidate_start_control_pose.position.x = ego.x;
  trajectory.candidate_start_control_pose.position.y = ego.y;
  trajectory.candidate_start_control_pose.orientation.z =
      std::sin(0.5 * ego.yaw);
  trajectory.candidate_start_control_pose.orientation.w =
      std::cos(0.5 * ego.yaw);
  trajectory.candidate_start_control_pose_stamp = plan_stamp;

  auto canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  if (!canonical.valid()) {
    return fail(Ay0ShadowProposalFailure::GEOMETRY_INVALID,
                "canonical_geometry_invalid");
  }
  trajectory.geometry_sha256 = canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  if (!canonical.valid()) {
    return fail(Ay0ShadowProposalFailure::PROVENANCE_INVALID,
                "canonical_payload_invalid");
  }
  trajectory.payload_sha256 = canonical.sha256;
  if (validateTrajectoryAgainstBaseSnapshotV1(base, trajectory) !=
      ValidationError::NONE) {
    return fail(Ay0ShadowProposalFailure::PROVENANCE_INVALID,
                "provenance_invalid");
  }
  result.failure = Ay0ShadowProposalFailure::NONE;
  result.reason = "ok";
  return result;
}

} // namespace state_lattice_overtake_planner
