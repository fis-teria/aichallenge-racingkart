std_msgs/Header header

uint8 INACTIVE=0
uint8 STUCK_CONFIRMING=1
uint8 STOP_HOLD=2
uint8 ACTIVE=3
uint8 HANDOFF_VERIFY=4
uint8 COMPLETE=5
uint8 ABORT=6
uint8 LOCKOUT=7

uint8 state
uint32 attempt_id
uint32 trajectory_generation
std_msgs/Header trajectory_header
bool input_complete
bool trajectory_valid
bool trajectory_safe
bool stop_required
bool handoff_ready
float32 min_wall_clearance_m
float32 min_vehicle_margin_m
string reason
