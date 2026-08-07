# Stamp-independent semantic identity for one normal FREE_RUN plan delivery.
uint8 CANONICAL_ALGORITHM_V1=1

uint8 canonical_algorithm_version
uint64 race_arm_epoch
uint64 planner_instance_id
uint32 plan_generation
builtin_interfaces/Time plan_stamp
uint8[32] canonical_plan_payload_sha256
