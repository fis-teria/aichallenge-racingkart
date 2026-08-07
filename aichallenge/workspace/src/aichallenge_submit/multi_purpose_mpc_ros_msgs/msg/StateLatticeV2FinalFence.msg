# Test-only, non-authoritative immutable closure of committed Planner proposals.
uint8 SCHEMA_VERSION=1

uint8 schema_version
string<=128 execution_nonce
string<=128 producer_instance_id
string<=128 session_id
uint64 sealed_epoch_id
uint64 fence_id
uint64 request_id
uint8[32] request_canonical_sha256
uint64 final_committed_ordinal
uint64 successful_emission_count
bool final_identity_present
StateLatticeV2Identity final_identity
string<=256 proposal_qos_fingerprint

# Diagnostic-only CLOCK_MONOTONIC timestamps. Never deadline authority.
uint64 quiesced_monotonic_ns
uint64 fence_publish_monotonic_ns
uint8[32] canonical_sha256
