std_msgs/Header header

uint8 FREE_RUN=0
uint8 ATTACK_FOLLOW=1
uint8 PASSING=2
uint8 ABORT_HOLD=3

uint8 LATERAL_STOP_NONE=0
uint8 LATERAL_STOP_CURRENT_D_HOLD=1
uint8 LATERAL_STOP_PASS_WARMUP=2

uint32 AUTH_CANDIDATE_MISSING=1
uint32 AUTH_CANDIDATE_NOT_SAFETY_EVALUATED=2
uint32 AUTH_CANDIDATE_REJECTED=4
uint32 AUTH_EGO_STALE=8
uint32 AUTH_V2X_STALE=16
uint32 AUTH_OPPONENT_STALE=32
uint32 AUTH_OPPONENT_EXCLUDED=64
uint32 AUTH_REFERENCE_INVALID=128
uint32 AUTH_PREDICTION_INCOMPLETE=256
uint32 AUTH_TRACKING_UNUSABLE=512
uint32 AUTH_MPC_HEALTH_STALE=1024
uint32 AUTH_MPC_HARD_FAILURE=2048
uint32 AUTH_TRAJECTORY_NOT_PUBLISHABLE=4096
uint32 AUTH_CONSTRAINT_STOP_REQUESTED=8192
uint32 AUTH_SAFETY_INPUTS_INCOMPLETE=16384

uint8 phase
uint32 plan_generation
uint32 attempt_id
string target_vehicle_id
int8 pass_direction
bool trajectory_authorized
bool lateral_maneuver_required
string decision_reason
uint32 authorization_failure_mask
string[] authorization_failure_reasons
string candidate_reject_reason
bool safety_inputs_complete
bool tracking_usable
bool trajectory_publishable
string constraint_reason
# STOP中にPP横操舵を残せる用途を型で限定する。NONEは常にfail-closed。
uint8 lateral_stop_authority_kind
# current-d HOLDでも追越transactionの左右を失わないための内部side。
int8 lateral_stop_transaction_pass_direction
# Plannerがgenerationごとに発行し、PPが実適用proofへそのまま束縛する非zero token。
uint64 lateral_stop_authority_token
autoware_auto_planning_msgs/Trajectory trajectory

# AW2-0 semantic candidate identity. Schema 1 proves only the bounded source
# identity/digest contract; it does not prove delivered geometry, execution, or
# motion authority. Schema 0 means that even this AW2-0 evidence is incomplete.
# These fields never grant motion authority.
uint8 aw2_identity_schema_version
uint64 planner_instance_id
uint64 race_arm_epoch
uint64 connector_transaction_id
uint32 candidate_revision
# Digest of the canonical semantic source wire. Volatile delivery stamps and
# current-ego reconstructed map geometry are intentionally outside this digest;
# AW2-1 binds those in an atomic plan-sample record.
uint8[32] candidate_content_sha256

# Stamp-independent semantic identity of a normal FREE_RUN proposal. The
# Planner publishes this identity; consumers must recompute it from this
# OvertakePlan payload before using it as evidence. Zero means unavailable and
# never grants motion authority.
uint8 FREE_RUN_CANONICAL_ALGORITHM_V1=1
uint8 free_run_canonical_algorithm_version
uint8[32] free_run_canonical_payload_sha256
