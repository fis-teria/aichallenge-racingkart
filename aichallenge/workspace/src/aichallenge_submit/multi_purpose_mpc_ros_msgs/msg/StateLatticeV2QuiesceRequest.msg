# Test-only, non-authoritative request to close one Planner V2 diagnostic epoch.
uint8 SCHEMA_VERSION=1

uint8 schema_version
string<=128 execution_nonce
string<=128 expected_producer_instance_id
string<=128 expected_session_id
uint64 sealed_epoch_id
uint64 request_id
uint8[32] canonical_sha256
