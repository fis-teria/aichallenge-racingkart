# C-002AY-0 Phase 0 shadow-only exact controller base snapshot.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V1_SHADOW=1

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MPC_HORIZON=1
uint8 SOURCE_REFERENCE_TRAJECTORY=2

uint8 BASE_SOURCE_DIGEST_INDETERMINATE=0
uint8 BASE_SOURCE_DIGEST_COMPLETE=1

uint8 schema_version
bool authority_eligible

builtin_interfaces/Time record_stamp
string<=128 frame_id
uint64 race_arm_epoch
uint64 controller_instance_id
uint64 controller_sequence
uint64 base_lease_id
builtin_interfaces/Time lease_valid_until

uint8 base_source_kind
builtin_interfaces/Time base_source_stamp
uint32 base_source_generation
uint32 base_original_point_count
uint32 first_source_index
uint32 last_source_index
uint32 nearest_source_index
multi_purpose_mpc_ros_msgs/CandidateExecutionPoint[<=256] base_points

uint8 base_source_digest_state
uint8 canonical_algorithm_version
uint8[32] base_geometry_sha256
uint8[32] base_source_sha256
uint8[32] controller_implementation_sha256
uint8[32] controller_config_sha256
uint8[32] snapshot_sha256
