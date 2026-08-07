std_msgs/Header header

# Schema 1 is a Mux-issued, short-lived observation of an internally committed
# motion authority. It is never accepted back into Mux as an authority input.
uint8 schema_version
bool valid
string<=96 reason

uint64 grant_issuer_instance_id
uint64 grant_sequence
# Exact STOP PASS_WARMUP proof consumed by this commit.
multi_purpose_mpc_ros_msgs/PlanSampleKey warmup_plan_sample_key
uint32 warmup_candidate_revision
uint8[32] warmup_candidate_content_sha256
uint64 warmup_lateral_stop_authority_token
uint64 warmup_pp_producer_instance_id
uint64 warmup_pp_command_sequence
builtin_interfaces/Time warmup_pp_command_stamp

# Exact non-STOP motion proposal and command.
multi_purpose_mpc_ros_msgs/PlanSampleKey plan_sample_key
uint8 phase
uint32 candidate_revision
uint8[32] candidate_content_sha256

uint32 constraint_generation
builtin_interfaces/Time constraint_stamp

uint64 pp_producer_instance_id
uint64 pp_command_sequence
builtin_interfaces/Time pp_command_stamp
float32 speed_mps
float32 acceleration_mps2
float32 steering_tire_angle_rad
float32 steering_tire_rotation_rate_radps

# Lease age is measured from Mux steady time, never reconstructed from ROS time.
float32 lease_duration_sec
