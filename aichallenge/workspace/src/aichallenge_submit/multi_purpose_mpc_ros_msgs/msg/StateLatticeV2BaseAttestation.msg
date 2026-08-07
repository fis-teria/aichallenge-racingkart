# Phase 2 direct, non-authoritative PP base attestation.  This is observation
# evidence only and never grants controller, Mux, or actuator authority.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V1_NON_AUTHORITATIVE=1

uint8 schema_version
std_msgs/Header header
string<=128 producer_instance_id
string<=128 session_id
uint64 attestation_sequence
multi_purpose_mpc_ros_msgs/ControllerBaseTrajectorySnapshot base
