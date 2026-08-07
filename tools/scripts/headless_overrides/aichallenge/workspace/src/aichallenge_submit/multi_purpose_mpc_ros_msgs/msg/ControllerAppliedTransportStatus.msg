# AW2-2B shadow transport loss/fault record. It is never motion authority.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V1=1

uint8 REASON_NONE=0
uint8 REASON_RESOURCE_LIMIT_EXCEEDED=1
uint8 REASON_QUEUE_DROP=2
uint8 REASON_GLOBAL_LOSS_MEMBERSHIP_UNKNOWN=3
uint8 REASON_TORN_SLOT=4
uint8 REASON_ABI_MISMATCH=5
uint8 REASON_DDS_ERROR=6
uint8 REASON_PUBLISHER_DISABLED=7

builtin_interfaces/Time stamp
uint8 schema_version
bool authority_eligible
uint64 ring_generation
uint64 controller_instance_id
uint64 consumer_instance_id
uint64 race_arm_epoch
uint64 loss_generation
uint64 cumulative_drop_count
uint32 queue_capacity
uint32 queue_depth
uint32 queue_high_water

uint64[<=16] first_dropped_sequence
uint64[<=16] last_dropped_sequence
multi_purpose_mpc_ros_msgs/ControllerSampleKey[<=16] first_dropped_key
multi_purpose_mpc_ros_msgs/ControllerSampleKey[<=16] last_dropped_key

bool global_loss_membership_unknown
uint64 first_uncertain_sequence
uint64 torn_slot_count
uint64 abi_error_count
uint64 dds_error_count
uint8 reason
string<=128 detail
