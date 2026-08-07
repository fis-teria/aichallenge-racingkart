std_msgs/Header header

# Atomic State Lattice instantaneous-control sample.  A sample is authority
# eligible only when every safety/provenance flag below is true and
# stop_required is false.
uint8 SCHEMA_INVALID=0
uint8 SCHEMA_V1=1
uint8 schema_version

# Fixed for one producer process; command_sequence starts at 1 and increases
# monotonically.  Together with header.stamp and plan_generation this forms
# the exact sample identity checked by Hybrid Mux.
uint64 producer_instance_id
uint64 command_sequence
uint32 plan_generation

# active=false is an explicit handoff to the normal Mux primary source.
bool active
bool safety_evaluation_enabled
bool inputs_fresh
bool costmap_valid
bool trajectory_valid
bool trajectory_safe
bool stop_required

autoware_auto_control_msgs/AckermannControlCommand command
string<=128 reason
