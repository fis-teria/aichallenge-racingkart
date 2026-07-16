std_msgs/Header header

uint8 FREE_RUN=0
uint8 ATTACK_FOLLOW=1
uint8 PASSING=2
uint8 ABORT_HOLD=3

uint8 phase
uint32 plan_generation
uint32 attempt_id
string target_vehicle_id
int8 pass_direction
bool trajectory_authorized
bool lateral_maneuver_required
autoware_auto_planning_msgs/Trajectory trajectory
