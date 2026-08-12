#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner {

enum class BehaviorMode : int {
  FREE_RUN = 0,
  FOLLOW_BLOCKED = 1,
  PREPARE_OVERTAKE_LEFT = 2,
  PREPARE_OVERTAKE_RIGHT = 3,
  OVERTAKE_LEFT = 4,
  OVERTAKE_RIGHT = 5,
  MERGE_BACK = 6,
  ABORT_RECOVERY = 7,
  SIDE_BY_SIDE_KEEP = 8,
  YIELD_BEHIND = 9,
  SAFE_STOP = 10,
  SPEED_GUARD = 11,
};

enum class SolverHorizonIntent : int {
  NONE = 0,
  MANEUVER_AUTHORIZED = 1,
  MANDATORY_AVOIDANCE = 2,
};

enum class CollisionKind : int {
  NONE = 0,
  WALL = 1,
  OPPONENT = 2,
};

// Classification is deliberately limited to the current-pose guard. Future
// trajectory and merge-back checks continue to treat every opponent equally.
enum class CurrentPoseOpponentRelation : int {
  UNKNOWN = 0,
  FRONT_OR_OVERLAP = 1,
  SIDE_OVERLAP = 2,
  REAR_ONLY = 3,
};

enum class OutputHorizonFailure : int {
  NONE = 0,
  INVALID_INPUT = 1,
  INVALID_CONTRACT = 2,
  NON_POSITIVE_DT = 3,
  STEERING_RATE = 4,
  INVALID_POINT = 5,
  CURVATURE = 6,
  WAYPOINT_WALL_COLLISION = 7,
  WAYPOINT_OPPONENT_COLLISION = 8,
  INTERPOLATED_WALL_COLLISION = 9,
  INTERPOLATED_OPPONENT_COLLISION = 10,
  WAYPOINT_SIDE_ROLE_SEPARATION = 11,
  INTERPOLATED_SIDE_ROLE_SEPARATION = 12,
};

enum class FrontTargetClass : int {
  NONE = 0,
  FORWARD = 1,
  PARALLEL = 2,
};

enum class FrontDetectionFailure : int {
  NONE = 0,
  INVALID_OPPONENT = 1,
  LONGITUDINAL_TARGET = 2,
  TRANSITION_ENVELOPE = 3,
  SELECTION_ARBITRATION = 4,
  ENTER_HYSTERESIS = 5,
};

enum class PreventiveSideRoleEligibilityFailure : int {
  NONE = 0,
  INVALID_EGO_OBSERVATION = 1,
  INVALID_PEER_OBSERVATION = 2,
  DUPLICATE_PEER_ID = 3,
  INVALID_VEHICLE_ID = 4,
  DELTA_S = 5,
  EGO_TANGENT_PROGRESS = 6,
  PEER_TANGENT_PROGRESS = 7,
  EGO_TRACK_HEADING = 8,
  PEER_TRACK_HEADING = 9,
  RELATIVE_HEADING = 10,
  MUTUAL_SIDE_GEOMETRY = 11,
  LATERAL_SEPARATION = 12,
  HARD_CLEARANCE = 13,
  MULTIPLE_PEERS = 14,
  PEER_DROPOUT = 15,
  PEER_CHANGED = 16,
  ROLE_CHANGED = 17,
  RELEASE_UNVERIFIED = 18,
  ENTRY_ENVELOPE = 19,
  PREDICTED_HARD_CLEARANCE = 20,
  SYNCHRONIZATION = 21,
  DECISION_DEADLINE = 22,
  POSITION_JUMP = 23,
};

enum class PreventiveSideRolePhase : int {
  NONE = 0,
  FOLLOWER_YIELD = 1,
  LEADER_WAIT = 2,
  LEADER_ESCAPE = 3,
  NEUTRAL_HOLD = 4,
  LEADER_PROCEED = 5,
};

enum class PreventiveSideRoleCandidateFailure : int {
  NONE = 0,
  NO_GENERATED_CANDIDATE = 1,
  NO_BASE_FEASIBLE_CANDIDATE = 2,
  DENSE_PEER_SEPARATION = 3,
  OUTPUT_HORIZON = 4,
  RESAMPLED_PEER_SEPARATION = 5,
  DIRECTION = 6,
  DEADLINE = 7,
  PROFILE_LIMIT = 8,
};

enum class PreventiveSideRoleAction : int {
  NONE = 0,
  NEUTRAL_STRAIGHT_HOLD = 1,
  STRAIGHT_CURRENT_CORRIDOR = 2,
  LEFT_ESCAPE = 3,
  RIGHT_ESCAPE = 4,
  STRAIGHT_YIELD = 5,
  FOLLOW_BEHIND = 6,
  SAFE_STOP = 7,
};

struct PreventiveSideRoleEligibilityDiagnostic {
  PreventiveSideRoleEligibilityFailure failure{
      PreventiveSideRoleEligibilityFailure::NONE};
  std::string peer_id;
  int matching_peer_ids{0};
  double clearance_m{std::numeric_limits<double>::infinity()};
  double delta_s_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_separation_m{std::numeric_limits<double>::quiet_NaN()};
  double ego_tangent_progress_mps{std::numeric_limits<double>::quiet_NaN()};
  double peer_tangent_progress_mps{std::numeric_limits<double>::quiet_NaN()};
  double ego_track_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double peer_track_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double relative_heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double predicted_time_to_hard_sec{std::numeric_limits<double>::infinity()};
  double predicted_min_clearance_m{std::numeric_limits<double>::infinity()};
  double synchronized_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double synchronized_ego_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double synchronized_peer_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double cartesian_longitudinal_m{std::numeric_limits<double>::quiet_NaN()};
  double cartesian_lateral_m{std::numeric_limits<double>::quiet_NaN()};
  double decision_deadline_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string entry_reason;
  CurrentPoseOpponentRelation ego_relation{
      CurrentPoseOpponentRelation::UNKNOWN};
  CurrentPoseOpponentRelation peer_relation{
      CurrentPoseOpponentRelation::UNKNOWN};
};

struct PreventiveSideRoleCandidateDiagnostic {
  PreventiveSideRoleCandidateFailure failure{
      PreventiveSideRoleCandidateFailure::NONE};
  std::string peer_id;
  int generated_candidates{0};
  int base_feasible_candidates{0};
  int separation_feasible_candidates{0};
  int violating_sample_index{-1};
  bool violating_interpolated_sample{false};
  double initial_lateral_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double observed_lateral_gap_m{std::numeric_limits<double>::quiet_NaN()};
  double initial_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double observed_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  // Dense-generator clearance summaries are observational precheck metrics.
  // Only the finalized output/resampled horizon may authorize or reject.
  double minimum_opponent_clearance_m{std::numeric_limits<double>::infinity()};
  double minimum_wall_clearance_m{std::numeric_limits<double>::infinity()};
  bool precheck_clearance_observational_only{true};
  std::uint64_t evaluation_generation{0U};
  std::string first_reject_reason;
};

inline constexpr std::size_t kMaxFrontDetectionOpponentDiagnostics = 8U;
inline constexpr std::size_t kFrontDetectionDiagnosticIdCapacity = 32U;

struct FrontDetectionOpponentDiagnostic {
  std::array<char, kFrontDetectionDiagnosticIdCapacity> opponent_id{};
  FrontTargetClass target_class{FrontTargetClass::NONE};
  FrontDetectionFailure first_false{FrontDetectionFailure::INVALID_OPPONENT};
  double forward_gap_m{std::numeric_limits<double>::infinity()};
  double rear_gap_m{std::numeric_limits<double>::infinity()};
  double detection_radius_m{0.0};
  double minimum_sweep_distance_m{std::numeric_limits<double>::infinity()};
};

// Read-only bounded trace of the detector predicates. It must never feed
// target selection, safety admission, or motion authority.
struct FrontDetectionDiagnostic {
  std::array<FrontDetectionOpponentDiagnostic,
             kMaxFrontDetectionOpponentDiagnostics>
      opponents{};
  std::size_t opponent_count{0U};
  std::size_t dropped_opponent_count{0U};
  std::array<char, kFrontDetectionDiagnosticIdCapacity> selected_target_id{};
  int enter_cycles{0};
  int clear_cycles{0};
  bool evaluated{false};
  bool detected_latched{false};
};

enum class EarlyAwareState : int {
  INACTIVE = 0,
  CANDIDATE = 1,
  ACTIVE = 2,
};

// Read-only early awareness evidence. This is intentionally distinct from
// FrontDetectionDiagnostic: it cannot select a planner target or grant any
// motion authority.
struct EarlyAwareDiagnostic {
  bool evaluated{false};
  bool active{false};
  EarlyAwareState state{EarlyAwareState::INACTIVE};
  std::array<char, kFrontDetectionDiagnosticIdCapacity> target_id{};
  int distinct_fresh_stamp_count{0};
  double target_observation_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double forward_delta_s_m{std::numeric_limits<double>::quiet_NaN()};
  double dynamic_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double ego_tangent_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double target_tangent_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double projected_closing_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  std::string reason{"inactive"};
};

struct OutputHorizonDiagnostic {
  OutputHorizonFailure failure{OutputHorizonFailure::NONE};
  int nearest_shift{0};
  int layout_offset{0};
  int waypoint_index{-1};
  int interpolation_piece{-1};
  std::int64_t reference_index{-1};
  double candidate_goal_d_m{std::numeric_limits<double>::quiet_NaN()};
  double candidate_tangent_scale{std::numeric_limits<double>::quiet_NaN()};
  double s_m{std::numeric_limits<double>::quiet_NaN()};
  double d_m{std::numeric_limits<double>::quiet_NaN()};
  double x_m{std::numeric_limits<double>::quiet_NaN()};
  double y_m{std::numeric_limits<double>::quiet_NaN()};
  double curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double observed_value{std::numeric_limits<double>::quiet_NaN()};
  double limit_value{std::numeric_limits<double>::quiet_NaN()};
  std::string opponent_id;
};

struct CollisionDiagnostic {
  CollisionKind kind{CollisionKind::NONE};
  CurrentPoseOpponentRelation relation{CurrentPoseOpponentRelation::UNKNOWN};
  std::string opponent_id;
  double clearance_m{std::numeric_limits<double>::infinity()};
  double required_clearance_m{0.0};

  bool collision() const { return kind != CollisionKind::NONE; }
};

struct Pose2d {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ReferencePoint : Pose2d {
  double s{0.0};
  double kappa{0.0};
  double speed_mps{0.0};
};

struct FrenetPoint {
  double s{0.0};
  double d{0.0};
  double yaw_error{0.0};
  std::size_t segment_index{0U};
  bool valid{false};
};

struct EgoState : Pose2d {
  double stamp_sec{0.0};
  double speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double curvature{0.0};
  FrenetPoint frenet{};
  bool valid{false};
};

struct OpponentState : Pose2d {
  std::string id;
  double stamp_sec{0.0};
  double speed_mps{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double sigma_x_m{0.0};
  double sigma_y_m{0.0};
  double uncertainty_x_m{0.15};
  double uncertainty_y_m{0.15};
  FrenetPoint frenet{};
  bool valid{false};
};

struct TrajectoryPoint : Pose2d {
  double time_sec{0.0};
  double u{0.0};
  double s{0.0};
  double d{0.0};
  double kappa{0.0};
  double speed_mps{0.0};
};

constexpr std::size_t kRepresentativeObjectDiagnosticCount = 5U;
constexpr std::size_t kRepresentativeObjectOpponentIdCapacity = 64U;

// Observational-only attribution for the maximum object-cost level at one
// representative point. This is not a unique owner of the candidate's summed
// object cost and must never be consumed by ranking or safety admission.
struct RepresentativeObjectDiagnostic {
  bool winner_valid{false};
  bool prediction_valid{false};
  int object_level{-1};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double prediction_horizon_sec{std::numeric_limits<double>::quiet_NaN()};
  std::array<char, kRepresentativeObjectOpponentIdCapacity> opponent_id{};
  bool opponent_id_truncated{false};
};

struct CandidateTrajectory {
  std::size_t lateral_index{0U};
  std::size_t tangent_index{0U};
  double goal_d_m{0.0};
  double tangent_scale{1.0};
  // Longitudinal reference arc allocated to complete the lateral transition.
  double required_arc_m{0.0};
  std::vector<TrajectoryPoint> representative;
  std::vector<TrajectoryPoint> dense;
  int total_cost{0};
  // Wall/object risk only. Reference deviation ranks otherwise-safe geometry
  // but must not stop or slow the vehicle merely for executing an overtake.
  int safety_cost{0};
  // Read-only totals of the individual pose-cost inputs over representative
  // points. Wall/object values are above their clear-space baseline. They are
  // not additive because production ranking merges levels before converting
  // each pose to its total cost.
  int reference_cost{0};
  int wall_cost{0};
  int object_cost{0};
  // Representative-point, nominal-footprint observations only. The clearance
  // is a conservative center-distance proxy, not authoritative hard-wall
  // admission evidence.
  bool representative_wall_diagnostic_valid{false};
  double representative_minimum_nominal_wall_clearance_proxy_m{
      std::numeric_limits<double>::quiet_NaN()};
  int representative_minimum_nominal_wall_clearance_point_index{-1};
  int representative_maximum_nominal_wall_level{-1};
  int representative_maximum_nominal_wall_level_point_index{-1};
  std::array<RepresentativeObjectDiagnostic,
             kRepresentativeObjectDiagnosticCount>
      representative_object_diagnostics{};
  std::size_t representative_object_diagnostic_count{0U};
  // Soft ranking signal only. Positive values mean that the terminal
  // Cartesian footprint is farther from the closest opponent than the
  // candidate start. Hard wall/opponent/trackability admission remains owned
  // by evaluateTrajectory().
  double opponent_clearance_recovery_m{0.0};
  double total_abs_steer_change{0.0};
  double dynamic_speed_limit_mps{0.0};
  double entry_speed_limit_mps{0.0};
  bool requires_entry_deceleration{false};
  bool feasible{false};
  std::string rejection_reason;
};

constexpr std::size_t kMaximumCandidateDiagnosticCount = 21U;
constexpr std::size_t kCandidateRejectReasonCapacity = 48U;
inline constexpr std::size_t kPassClearanceDiagnosticTargetIdCapacity = 32U;

// Read-only snapshot of the post-feasibility pass-clearance predicate during
// update(). It retains the first passing candidate when one exists; otherwise
// it retains the most recently evaluated failing candidate. It is bounded and
// must never influence candidate selection, safety admission, or motion
// authority.
struct PassClearanceDiagnostic {
  std::array<char, kPassClearanceDiagnosticTargetIdCapacity> target_id{};
  double target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double target_uncertainty_y_m{std::numeric_limits<double>::quiet_NaN()};
  double target_observation_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double candidate_goal_d_m{std::numeric_limits<double>::quiet_NaN()};
  // -1 is right, +1 is left, and 0 means no lateral side was observed.
  int side{0};
  double actual_separation_m{std::numeric_limits<double>::quiet_NaN()};
  double required_separation_m{std::numeric_limits<double>::quiet_NaN()};
  double margin_m{std::numeric_limits<double>::quiet_NaN()};
  bool observed_predicate{false};
  bool evaluated{false};
  bool inputs_valid{false};
};

// Fixed-capacity, read-only evidence copied from the existing candidate
// evaluation. Planner selection and safety admission must never consume it.
struct CandidateRejectionDiagnostic {
  std::size_t lateral_index{0U};
  std::size_t tangent_index{0U};
  double goal_d_m{0.0};
  double tangent_scale{1.0};
  double required_arc_m{0.0};
  int total_cost{0};
  // Diagnostic-only non-additive inputs captured from the same production
  // pose-cost evaluation as total_cost.
  int reference_cost{0};
  int wall_cost{0};
  int object_cost{0};
  bool representative_wall_diagnostic_valid{false};
  double representative_minimum_nominal_wall_clearance_proxy_m{
      std::numeric_limits<double>::quiet_NaN()};
  int representative_minimum_nominal_wall_clearance_point_index{-1};
  int representative_maximum_nominal_wall_level{-1};
  int representative_maximum_nominal_wall_level_point_index{-1};
  std::array<RepresentativeObjectDiagnostic,
             kRepresentativeObjectDiagnosticCount>
      representative_object_diagnostics{};
  std::size_t representative_object_diagnostic_count{0U};
  double opponent_clearance_recovery_m{0.0};
  double start_x_m{0.0};
  double start_y_m{0.0};
  double start_yaw_rad{0.0};
  double start_s_m{0.0};
  double start_d_m{0.0};
  std::array<char, kCandidateRejectReasonCapacity> first_reject_reason{};
};

struct MpcHealthStatus {
  bool valid{false};
  int infeasible_count{0};
  double solve_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double age_sec{std::numeric_limits<double>::infinity()};
  std::uint64_t sample_sequence{0U};
};

// Read-only timing captured from the most recent update(). These values are
// diagnostics only: planner selection, safety admission, and deadline policy
// must never consume them.
struct PlanningCycleMetrics {
  double candidate_generation_ms{0.0};
  double output_horizon_ms{0.0};
  std::size_t candidate_dense_point_count{0U};
  std::uint32_t output_horizon_call_count{0U};
  PassClearanceDiagnostic pass_clearance_diagnostic;
  std::size_t candidate_diagnostic_count{0U};
  std::array<CandidateRejectionDiagnostic, kMaximumCandidateDiagnosticCount>
      candidate_diagnostics{};
};

// Deadline-rejected trial state has no wire generation. Clear only the
// diagnostic that would otherwise look generation-bound; retained timing and
// bounded candidate counters remain useful for deadline diagnosis.
inline void
discardUncommittedPassClearanceDiagnostic(PlanningCycleMetrics *metrics) {
  if (metrics != nullptr) {
    metrics->pass_clearance_diagnostic = PassClearanceDiagnostic{};
  }
}

enum class ExecutionGeometryKind : std::uint8_t {
  LEGACY_OFFSETS = 0,
  EXACT_CARTESIAN = 1,
};

struct PlannerOutput {
  BehaviorMode mode{BehaviorMode::FREE_RUN};
  SolverHorizonIntent intent{SolverHorizonIntent::NONE};
  std::vector<double> lateral_offsets_m;
  std::vector<double> speed_caps_mps;
  // Monotonic arc from the actual ego pose. When populated, the receiver
  // applies the lateral/speed profile by distance rather than by an ambiguous
  // reference-array index.
  std::vector<double> longitudinal_offsets_m;
  // The spatial profile was produced without an authority-eligible binding to
  // PP's exact base trajectory tuple. It may be recorded in shadow evidence,
  // but the wire producer must fail closed to speed-only.
  bool spatial_profile_shadow_only{false};
  ExecutionGeometryKind execution_geometry_kind{
      ExecutionGeometryKind::LEGACY_OFFSETS};
  double speed_cap_mps{0.0};
  double candidate_speed_limit_mps{0.0};
  int minimum_cost{-1};
  int rear_cost{-1};
  int generated_candidates{0};
  int feasible_candidates{0};
  int rejected_wall_candidates{0};
  int rejected_opponent_candidates{0};
  int rejected_curvature_candidates{0};
  int rejected_trackability_candidates{0};
  int rejected_other_candidates{0};
  int return_ready_cycles{0};
  double front_detection_radius_m{0.0};
  double front_detection_transition_distance_m{0.0};
  FrontDetectionDiagnostic front_detection_diagnostic;
  EarlyAwareDiagnostic early_aware_diagnostic;
  CollisionKind current_collision_kind{CollisionKind::NONE};
  CurrentPoseOpponentRelation current_opponent_relation{
      CurrentPoseOpponentRelation::UNKNOWN};
  bool current_pose_rear_only_exempt{false};
  std::string current_pose_rear_only_exempt_opponent_id;
  double current_pose_rear_only_exempt_clearance_m{
      std::numeric_limits<double>::infinity()};
  std::string blocking_opponent_id;
  double blocking_clearance_m{std::numeric_limits<double>::infinity()};
  double blocking_required_clearance_m{0.0};
  OutputHorizonDiagnostic output_horizon_diagnostic;
  std::string target_id;
  std::string reason;
  bool active{false};
  bool safe_lateral{false};
  bool rear_hard_safe{false};
  bool emergency_stop{false};
  bool target_missing{false};
  bool mpc_health_guard_active{false};
  // Read-only marker for the preventive side-by-side responsibility output.
  // The existing compatible SIDE_BY_SIDE_KEEP mode remains the wire behavior;
  // deadline policy uses this marker to refuse stale role-output reuse.
  bool preventive_side_role_active{false};
  bool preventive_side_role_leader{false};
  bool preventive_side_role_neutral{false};
  PreventiveSideRolePhase preventive_side_role_phase{
      PreventiveSideRolePhase::NONE};
  PreventiveSideRoleAction preventive_side_role_action{
      PreventiveSideRoleAction::NONE};
  std::uint64_t preventive_side_role_generation{0U};
  std::string preventive_side_role_peer_id;
  double preventive_side_role_clearance_m{
      std::numeric_limits<double>::infinity()};
  PreventiveSideRoleEligibilityDiagnostic
      preventive_side_role_eligibility_diagnostic;
  PreventiveSideRoleCandidateDiagnostic
      preventive_side_role_candidate_diagnostic;
  int preventive_side_role_yield_sample_count{0};
  double preventive_side_role_yield_sample_span_sec{0.0};
  double preventive_side_role_peer_tangent_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double preventive_side_role_required_gap_m{
      std::numeric_limits<double>::quiet_NaN()};
  double preventive_side_role_current_gap_m{
      std::numeric_limits<double>::quiet_NaN()};
  double preventive_side_role_requested_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double preventive_side_role_applied_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  std::string preventive_side_role_included_opponent_ids;
  std::string preventive_side_role_release_reason;
  // Read-only continuity evidence. These fields never grant authority or
  // participate in candidate selection.
  bool pass_continuation_latched{false};
  bool pass_continuation_active{false};
  bool pass_continuation_suspended{false};
  std::string pass_continuation_target_id;
  int pass_continuation_side{0};
  int pass_continuation_lateral_index{-1};
  int pass_continuation_tangent_index{-1};
  double pass_continuation_goal_d_m{std::numeric_limits<double>::quiet_NaN()};
  double pass_continuation_required_arc_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_continuation_target_observation_stamp_sec{
      std::numeric_limits<double>::quiet_NaN()};
  bool overtake_permission_allowed{true};
  std::string overtake_permission_section_name;
  std::string overtake_permission_reason{"permission_profile_disabled"};
};

const char *toString(BehaviorMode mode);
const char *toString(CollisionKind kind);
const char *toString(CurrentPoseOpponentRelation relation);
const char *toString(OutputHorizonFailure failure);
const char *toString(FrontTargetClass target_class);
const char *toString(FrontDetectionFailure failure);
const char *toString(EarlyAwareState state);
const char *toString(PreventiveSideRoleEligibilityFailure failure);
const char *toString(PreventiveSideRoleCandidateFailure failure);
const char *toString(PreventiveSideRolePhase phase);
const char *toString(PreventiveSideRoleAction action);
double normalizeAngle(double angle);

} // namespace state_lattice_overtake_planner
