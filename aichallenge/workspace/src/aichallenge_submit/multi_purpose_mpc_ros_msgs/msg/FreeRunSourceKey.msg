# Identity of the immutable baseline reference used by Pure Pursuit.
uint8 CANONICAL_ALGORITHM_V1=1

uint8 canonical_algorithm_version
uint64 baseline_instance_id
uint64 controller_instance_id
uint32 source_generation
builtin_interfaces/Time source_stamp
uint32 original_point_count
uint8[32] baseline_reference_sha256
uint8[32] controller_implementation_sha256
uint8[32] controller_config_sha256
