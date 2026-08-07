# Immutable planner sample identity. AW2 uses it for shadow provenance and
# MotionAuthorityGrant schema 1 uses the same exact tuple for live binding.
uint64 race_arm_epoch
uint64 planner_instance_id
uint64 attempt_id
string<=64 target_vehicle_id
int8 pass_direction
uint64 connector_transaction_id
builtin_interfaces/Time plan_stamp
uint32 plan_generation
