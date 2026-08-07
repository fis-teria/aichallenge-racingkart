# Phase 1 non-authoritative planner-to-PP proposal identity.
string<=128 producer_instance_id
string<=128 session_id
uint64 proposal_sequence
uint32 plan_generation
uint32 source_generation
builtin_interfaces/Time source_stamp
string<=128 frame_id
uint8[32] canonical_sha256

# Diagnostic only. Never use across processes as an authority or lease clock.
uint64 publish_monotonic_ns
