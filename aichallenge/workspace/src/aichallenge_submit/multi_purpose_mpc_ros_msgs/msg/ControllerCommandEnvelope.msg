std_msgs/Header header

# v1 carries the atomic controller sample. v2 additionally binds the complete
# immutable planner candidate identity required for PASS motion authority.
# 0 denotes uninitialized/legacy data.
uint8 schema_version

# Fixed, non-zero identity for one Pure Pursuit node process.
uint64 producer_instance_id

# Starts at 1 and increases monotonically within producer_instance_id.
uint64 command_sequence

# Planner reference contract generation applied to this controller sample.
uint32 plan_generation

# The complete command and its tracking proof are one atomic sample.
autoware_auto_control_msgs/AckermannControlCommand command

bool mpc_horizon_usable
bool pp_command_fresh
bool trajectory_tracking_usable
# 実際に適用したtyped lateral STOP authority。planから推測せずPPが搬送する。
uint8 lateral_stop_authority_kind
int8 lateral_stop_transaction_pass_direction
uint64 lateral_stop_authority_token
float32 command_age_sec
string reason

# Schema 2 fields. Schema 1 must leave these zero/default and cannot obtain a
# MotionAuthorityGrant.
multi_purpose_mpc_ros_msgs/PlanSampleKey plan_sample_key
uint32 candidate_revision
uint8[32] candidate_content_sha256
