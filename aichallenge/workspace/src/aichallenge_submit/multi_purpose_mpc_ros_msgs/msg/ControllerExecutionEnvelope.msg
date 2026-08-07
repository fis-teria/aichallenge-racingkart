std_msgs/Header header

# v1 carries the legacy geometry witness. v2 appends the exact diagnostic-only
# Pure Pursuit snapshot in witness. 0 denotes uninitialized data.
uint8 schema_version

# These fields duplicate and must exactly equal command_envelope identity.
uint64 producer_instance_id
uint64 command_sequence
uint32 plan_generation

# Command and execution witness are one atomic shadow sample.
multi_purpose_mpc_ros_msgs/ControllerCommandEnvelope command_envelope
multi_purpose_mpc_ros_msgs/ControllerExecutionWitness witness
