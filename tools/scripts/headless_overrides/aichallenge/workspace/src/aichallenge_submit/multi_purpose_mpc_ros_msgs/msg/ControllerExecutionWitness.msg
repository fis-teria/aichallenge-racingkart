# v1 is the only defined schema. 0 denotes uninitialized data.
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
