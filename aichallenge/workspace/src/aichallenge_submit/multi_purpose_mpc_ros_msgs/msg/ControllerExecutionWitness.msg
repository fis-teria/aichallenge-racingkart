# v1 carries the legacy bounded geometry witness. v2 appends an exact,
# diagnostic-only Pure Pursuit command snapshot. 0 denotes uninitialized data.
uint8 schema_version

# Controller role/domain. AW1 publishes both roles for observation, but only a
# future, separately reviewed PRIMARY consumer may ever become authority.
uint8 ROLE_UNKNOWN=0
uint8 ROLE_PRIMARY=1
uint8 ROLE_RECOVERY=2
uint8 controller_role

# Controller-neutral source selected for this control sample.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_REFERENCE_TRAJECTORY=1
uint8 SOURCE_REFERENCE_OVERRIDE=2
uint8 SOURCE_PREDICTED_HORIZON=3
uint8 trajectory_source

# Identity is valid only when copied from the typed OvertakePlan topic.
uint8 IDENTITY_NONE=0
uint8 IDENTITY_TYPED_OVERTAKE_PLAN=1
uint8 identity_source
builtin_interfaces/Time plan_stamp
uint32 attempt_id
string target_vehicle_id
int8 pass_direction

# Typed identity and typed plan geometry are deliberately independent.
# Identity may be exact while geometry remains unproven.
bool typed_plan_geometry_matches_applied

# Source/reference provenance and exact controller geometry.
builtin_interfaces/Time source_stamp
builtin_interfaces/Time reference_stamp
autoware_auto_planning_msgs/Trajectory base_trajectory
autoware_auto_planning_msgs/Trajectory applied_trajectory
uint32 nearest_trajectory_index
geometry_msgs/Pose control_pose
float32 trajectory_progress_m

# Exact source generation and canonical whole-wire diagnostic binding.
uint32 source_generation
bool source_binding_complete
# This fingerprint covers the complete canonical source wire only when
# source_binding_complete=true. It is not cryptographic or authentication.
string source_payload_fingerprint
uint64 source_payload_original_size_bytes

# Horizon and steering evidence. Limits are explicit.
float32 available_spatial_horizon_m
float32 required_spatial_horizon_m
float32 raw_steering_tire_angle_rad
float32 bounded_steering_tire_angle_rad
float32 raw_steering_tire_rotation_rate_radps
float32 bounded_steering_tire_rotation_rate_radps
float32 steering_tire_angle_limit_rad
float32 steering_tire_rotation_rate_limit_radps

# The rollout array is IDL-bounded. Imported Trajectory messages are separately
# checked against the adapter's deterministic point limit.
multi_purpose_mpc_ros_msgs/ExecutionSweepSample[<=100] rollout_samples

# Completeness is shadow diagnostic state, never a motion-safety proof.
bool shadow_geometry_complete
string incompleteness_reason

# AW1 is observation-only. This field is always false and must not be used by
# Mux, Planner, release, tracking-usable, or any motion authority consumer.
bool authority_eligible

# Schema 2 exact Pure Pursuit diagnostic snapshot. Schema 1 leaves every field
# below at its zero/default value. This is observation-only and cannot grant
# motion authority.
bool diagnostic_only
bool exact_snapshot_complete
string exact_snapshot_reason

# Atomic command/candidate/source binding.
multi_purpose_mpc_ros_msgs/PlanSampleKey plan_sample_key
uint32 candidate_revision
uint8[32] candidate_content_sha256
builtin_interfaces/Time base_source_stamp
uint32 base_source_generation
uint32 base_source_point_count
uint8[32] base_source_sha256
uint8[32] controller_implementation_sha256
uint8[32] controller_config_sha256
builtin_interfaces/Time control_pose_stamp
float32 diagnostic_lease_duration_sec
# In schema 2 applied_trajectory carries this exact bounded sliding interval,
# rather than the schema-1 first-N prefix. Planner canonicalizes the received
# window off the controller timer; PP performs no trajectory hashing here.
uint32 exact_geometry_original_point_count
uint32 exact_geometry_first_source_index
uint32 exact_geometry_last_source_index

# Exact controller decisions and scalar inputs from the published command
# cycle. Indices refer to applied_trajectory before the v1 diagnostic cap.
uint32 selected_lookahead_trajectory_index
bool lookahead_endpoint_fallback
float64 control_speed_mps
float64 resolved_lookahead_distance_m
float64 geometric_steering_tire_angle_rad
float64 curvature_feedforward_steering_rad
float64 raw_exact_steering_tire_angle_rad
float64 requested_output_steering_tire_angle_rad
float64 bounded_exact_steering_tire_angle_rad
float64 requested_steering_tire_rotation_rate_radps
float64 bounded_exact_steering_tire_rotation_rate_radps
bool limiter_reference_valid
float64 limiter_reference_steering_rad
float64 command_dt_sec
float64 wheelbase_m
float64 steering_output_gain
float64 hard_steering_angle_limit_rad
float64 hard_steering_rate_limit_radps
bool steering_angle_limited
bool steering_rate_limited
