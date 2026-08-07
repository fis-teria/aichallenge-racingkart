# Phase 1 typed live proposal. This message never grants command authority.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V2_NON_AUTHORITATIVE=2

uint8 schema_version
std_msgs/Header header
multi_purpose_mpc_ros_msgs/StateLatticeV2Identity identity

# Reuse the already canonicalized V1 payload while keeping its shadow topic
# and schema isolated from this V2 transport.
multi_purpose_mpc_ros_msgs/AuthorizedCartesianTrajectory proposal
