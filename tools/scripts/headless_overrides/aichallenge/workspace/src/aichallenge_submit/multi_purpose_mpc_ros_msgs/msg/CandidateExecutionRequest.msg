# AW2-1 shadow-only atomic delivery record. This record is evidence, never
# motion authority. Schema 0 means that one or more required bindings could not
# be proven; the record is still published for diagnosis.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V1=1

uint8 AUTHORIZATION_NOT_AUTHORIZED=0
uint8 AUTHORIZATION_AUTHORIZED=1
uint8 AUTHORIZATION_UNKNOWN=2

uint8 GEOMETRY_LONGITUDINAL_ONLY=0
uint8 GEOMETRY_LATERAL_REQUIRED=1
uint8 GEOMETRY_UNKNOWN=2

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_LEGACY_REFERENCE_OVERRIDE=1
uint8 SOURCE_V2_TRAJECTORY=2

uint8 SAFETY_NOT_EVALUATED=0
uint8 SAFETY_REJECTED=1
uint8 SAFETY_PASSED=2

uint8 DELIVERY_ACCEPTED=0
uint8 DELIVERY_CONSISTENT_DUPLICATE=1
uint8 DELIVERY_INVALID=2
uint8 DELIVERY_CONFLICT=3

uint8 schema_version
bool authority_eligible

builtin_interfaces/Time plan_stamp
string<=128 plan_frame_id
uint64 planner_instance_id
uint64 race_arm_epoch
uint64 attempt_id
string<=64 target_vehicle_id
int8 transaction_pass_direction
int8 published_pass_direction
uint64 connector_transaction_id
uint32 plan_generation
uint32 candidate_revision
uint8[32] candidate_content_sha256

uint8 phase
uint8 authorization_state
uint8 geometry_requirement
uint8 candidate_type
bool trajectory_authorized_legacy
bool lateral_maneuver_required_legacy

bool typed_trajectory_present
multi_purpose_mpc_ros_msgs/CandidateExecutionPoint[<=100] geometry_points
uint8[32] delivered_geometry_sha256

uint8 source_kind
uint32 source_generation
# Byte count of canonical_source_wire before copying it into this bounded ROS
# sequence. This is not the raw C++ object size or DDS serialized size.
uint32 source_original_size_bytes
uint8[<=4096] canonical_source_wire
uint8[32] canonical_source_sha256

builtin_interfaces/Time constraint_stamp
string<=128 constraint_frame_id
uint32 constraint_generation
uint32 constraint_plan_generation
bool constraint_valid
bool constraint_stop_requested
bool constraint_release_authorized
float32 constraint_speed_limit_mps
float32 constraint_required_brake_decel_mps2
string<=256 constraint_reason

float64 required_controller_spatial_horizon_m
float64 planned_target_d_m
float64 committed_target_d_m
uint64 safety_snapshot_id
uint8 safety_evaluation_result
string<=256 safety_evaluation_reason

uint8 delivery_observation
uint8[32] plan_sample_record_sha256
