# Bounded full geometry owned by one AW2 controller-applied record.
string<=128 frame_id
builtin_interfaces/Time source_stamp
uint32 original_point_count
# Exact contiguous controller-used interval in the original source trajectory.
uint32 first_source_index
uint32 last_source_index
uint32 nearest_source_index
uint32 speed_cap_source_index
uint32 curvature_last_read_source_index
uint32 lookahead_selected_source_index
uint32 required_horizon_end_source_index
bool lookahead_endpoint_fallback
multi_purpose_mpc_ros_msgs/CandidateExecutionPoint[<=256] points
uint8[32] geometry_sha256
float64 total_arc_length_m
float64 required_spatial_horizon_m
uint8[32] start_point_sha256
uint8[32] end_point_sha256
# This is deliberately distinct from geometry_sha256, which hashes only the
# interval above. The controller hot path reports INDETERMINATE unless it
# obtained a full source digest before capture.
uint8 FULL_SOURCE_DIGEST_INDETERMINATE=0
uint8 FULL_SOURCE_DIGEST_COMPLETE=1
uint8 full_source_digest_state
uint8[32] full_source_sha256
