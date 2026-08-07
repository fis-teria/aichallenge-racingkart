#include "overtake_transport_contract/state_lattice_v2_binding.hpp"

#include <gtest/gtest.h>

#include <limits>

#include "overtake_transport_contract/c002ay0_canonical.hpp"
#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

namespace overtake_transport_contract::state_lattice_v2 {
struct BindingStoreTestPeer {
  static void setHoldCycleIndex(BindingStore &store, std::uint32_t value) {
    ASSERT_TRUE(store.active_availability_hold_.has_value());
    store.active_availability_hold_->hold_cycle_index = value;
  }
};
} // namespace overtake_transport_contract::state_lattice_v2

namespace {
namespace v2 = overtake_transport_contract::state_lattice_v2;
namespace canonical = overtake_transport_contract::c002ay0;

canonical::Digest digest(std::uint8_t value) {
  canonical::Digest result{};
  result.fill(value);
  return result;
}

canonical::CandidateExecutionPoint point(std::uint32_t index, double x_m) {
  canonical::CandidateExecutionPoint result;
  result.time_from_start.nanosec = index * 25000000U;
  result.position_x_m = x_m;
  result.orientation_w = 1.0;
  result.longitudinal_velocity_mps = 1.0F;
  return result;
}

v2::Proposal validProposal(std::uint64_t sequence = 1U) {
  v2::Proposal result;
  result.schema_version = v2::Proposal::SCHEMA_V2_NON_AUTHORITATIVE;
  result.header.frame_id = "map";
  result.header.stamp.sec = 23;
  result.header.stamp.nanosec = 30000000U;
  result.identity.producer_instance_id = "5";
  result.identity.session_id = "3";
  result.identity.proposal_sequence = sequence;
  result.identity.plan_generation = 18U + sequence;
  result.identity.source_generation = 43U;
  result.identity.source_stamp.sec = 23;
  result.identity.source_stamp.nanosec = 25000000U;
  result.identity.frame_id = "map";
  result.identity.publish_monotonic_ns = 100U;

  auto &trajectory = result.proposal;
  trajectory.schema_version = trajectory.SCHEMA_V1_SHADOW;
  trajectory.authority_eligible = false;
  trajectory.plan_stamp = result.header.stamp;
  trajectory.frame_id = "map";
  trajectory.plan_sample_key.race_arm_epoch = 3U;
  trajectory.plan_sample_key.planner_instance_id = 5U;
  trajectory.plan_sample_key.attempt_id = 7U;
  trajectory.plan_sample_key.target_vehicle_id = "D2";
  trajectory.plan_sample_key.pass_direction = -1;
  trajectory.plan_sample_key.connector_transaction_id = 11U;
  trajectory.plan_sample_key.plan_stamp = trajectory.plan_stamp;
  trajectory.plan_sample_key.plan_generation = 18U + sequence;
  trajectory.candidate_revision = 23U;
  trajectory.authority_token = 29U;
  trajectory.candidate_type = trajectory.CANDIDATE_PASS_LEFT;
  trajectory.phase = trajectory.PHASE_PASSING;
  trajectory.authorization_state = trajectory.AUTHORIZATION_AUTHORIZED;
  trajectory.source_controller_instance_id = 31U;
  trajectory.source_controller_sequence = 37U;
  trajectory.base_lease_id = 41U;
  trajectory.base_lease_valid_until.sec = 24;
  trajectory.base_source_kind = trajectory.SOURCE_MPC_HORIZON;
  trajectory.base_source_stamp = result.identity.source_stamp;
  trajectory.base_source_generation = 43U;
  trajectory.base_original_point_count = 2U;
  trajectory.base_first_source_index = 0U;
  trajectory.base_last_source_index = 1U;
  trajectory.base_nearest_source_index = 0U;
  trajectory.base_source_digest_state = trajectory.BASE_SOURCE_DIGEST_COMPLETE;
  trajectory.canonical_algorithm_version = 1U;
  trajectory.base_geometry_sha256 = digest(0x11U);
  trajectory.base_source_sha256 = digest(0x21U);
  trajectory.base_snapshot_sha256 = digest(0x31U);
  trajectory.points = {point(0U, 0.0), point(1U, 0.25)};
  trajectory.original_candidate_point_count = 2U;
  trajectory.total_arc_length_m = 0.25;
  trajectory.required_spatial_horizon_m = 0.20;
  trajectory.join_end_arc_length_m = 0.10;
  trajectory.post_join_arc_length_m = 0.15;
  trajectory.safety_snapshot_id = 47U;
  trajectory.safety_evaluation_result = trajectory.SAFETY_PASSED;
  trajectory.safety_evaluation_stamp.sec = 23;
  trajectory.safety_evaluation_stamp.nanosec = 29500000U;
  trajectory.safety_valid_until.sec = 24;
  trajectory.world_safety_snapshot_sha256 = digest(0x51U);
  trajectory.safety_evaluator_implementation_sha256 = digest(0x61U);
  trajectory.safety_evaluator_config_sha256 = digest(0x71U);
  trajectory.controller_implementation_sha256 = digest(0x31U);
  trajectory.controller_config_sha256 = digest(0x41U);
  trajectory.candidate_start_control_pose.orientation.w = 1.0;
  trajectory.candidate_start_control_pose_stamp = trajectory.plan_stamp;
  trajectory.geometry_sha256 =
      canonical::canonicalizeGeometryV1(trajectory.points,
                                        "C002AY0_AUTHORIZED_GEOMETRY_V1")
          .sha256;
  auto encoded = canonical::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.candidate_start_control_pose_sha256 = encoded.control_pose_sha256;
  trajectory.safety_proof_sha256 = encoded.safety_proof_sha256;
  encoded = canonical::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.payload_sha256 = encoded.sha256;
  result.identity.canonical_sha256 = trajectory.payload_sha256;
  return result;
}

void recanonicalize(v2::Proposal &proposal) {
  auto encoded = canonical::canonicalizeAuthorizedTrajectoryV1(proposal.proposal);
  proposal.proposal.safety_proof_sha256 = encoded.safety_proof_sha256;
  encoded = canonical::canonicalizeAuthorizedTrajectoryV1(proposal.proposal);
  proposal.proposal.payload_sha256 = encoded.sha256;
  proposal.identity.canonical_sha256 = encoded.sha256;
}

void setSafetyDeadline(v2::Proposal &proposal, std::int32_t sec,
                       std::uint32_t nanosec) {
  proposal.proposal.safety_valid_until.sec = sec;
  proposal.proposal.safety_valid_until.nanosec = nanosec;
  recanonicalize(proposal);
}

v2::BaseAttestation validAttestation() {
  canonical::FixedBaseRecord record{};
  record.session_generation = 1U;
  record.session_nonce = 1U;
  record.record_stamp = {23, 30000000U};
  record.frame_size = 3U;
  record.frame[0] = 'm';
  record.frame[1] = 'a';
  record.frame[2] = 'p';
  record.race_arm_epoch = 3U;
  record.controller_instance_id = 31U;
  record.controller_sequence = 37U;
  record.base_lease_id = 41U;
  record.lease_valid_until = {24, 0U};
  record.base_source_kind = canonical::kFixedBaseSourceMpcHorizon;
  record.base_source_stamp = {23, 25000000U};
  record.base_source_generation = 43U;
  record.base_original_point_count = 2U;
  record.nearest_source_index = 0U;
  record.point_count = 2U;
  record.points[0].orientation_w = 1.0;
  record.points[0].longitudinal_velocity_mps = 1.0F;
  record.points[1].time_from_start.nanosec = 25000000U;
  record.points[1].position_x_m = 0.25;
  record.points[1].orientation_w = 1.0;
  record.points[1].longitudinal_velocity_mps = 1.0F;
  record.controller_implementation_sha256 = digest(0x31U);
  record.controller_config_sha256 = digest(0x41U);

  v2::BaseAttestation attestation;
  EXPECT_EQ(canonical::buildBaseSnapshotFromFixedRecord(record,
                                                        attestation.base),
            canonical::ValidationError::NONE);
  attestation.schema_version =
      v2::BaseAttestation::SCHEMA_V1_NON_AUTHORITATIVE;
  attestation.header.stamp = attestation.base.record_stamp;
  attestation.header.frame_id = "map";
  attestation.producer_instance_id = "pp-primary";
  attestation.session_id = "3";
  attestation.attestation_sequence = 37U;
  return attestation;
}

void bindProposalToAttestation(v2::Proposal &proposal,
                               const v2::BaseAttestation &attestation) {
  const auto &base = attestation.base;
  auto &payload = proposal.proposal;
  payload.source_controller_instance_id = base.controller_instance_id;
  payload.source_controller_sequence = base.controller_sequence;
  payload.base_lease_id = base.base_lease_id;
  payload.base_lease_valid_until = base.lease_valid_until;
  payload.base_source_kind = base.base_source_kind;
  payload.base_source_stamp = base.base_source_stamp;
  payload.base_source_generation = base.base_source_generation;
  payload.base_original_point_count = base.base_original_point_count;
  payload.base_first_source_index = base.first_source_index;
  payload.base_last_source_index = base.last_source_index;
  payload.base_nearest_source_index = base.nearest_source_index;
  payload.base_source_digest_state = base.base_source_digest_state;
  payload.canonical_algorithm_version = base.canonical_algorithm_version;
  payload.base_geometry_sha256 = base.base_geometry_sha256;
  payload.base_source_sha256 = base.base_source_sha256;
  payload.base_snapshot_sha256 = base.snapshot_sha256;
  payload.controller_implementation_sha256 =
      base.controller_implementation_sha256;
  payload.controller_config_sha256 = base.controller_config_sha256;
  proposal.identity.source_generation = base.base_source_generation;
  proposal.identity.source_stamp = base.base_source_stamp;
  auto encoded = canonical::canonicalizeAuthorizedTrajectoryV1(payload);
  payload.safety_proof_sha256 = encoded.safety_proof_sha256;
  encoded = canonical::canonicalizeAuthorizedTrajectoryV1(payload);
  payload.payload_sha256 = encoded.sha256;
  proposal.identity.canonical_sha256 = encoded.sha256;
}

TEST(StateLatticeV2Binding, ExactProposalAvailabilityOutlivesTenCycles) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  ASSERT_TRUE(store.enqueue(validProposal(), 900U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  for (std::uint32_t cycle = 1U; cycle <= 15U; ++cycle) {
    const auto result = store.beginCycle(now, 1000U + cycle);
    EXPECT_EQ(result.accepted.has_value(), cycle == 1U);
    EXPECT_EQ(result.hold_cycle_index, cycle);
    EXPECT_EQ(result.first_uptake, cycle == 1U);
    ASSERT_EQ(result.event_count, 1U);
    EXPECT_EQ(result.events[0].hold_cycle_index, cycle);
    EXPECT_EQ(result.events[0].first_uptake, cycle == 1U);
    ASSERT_TRUE(result.events[0].identity.has_value());
    EXPECT_EQ(result.events[0].identity->producer_instance_id, "5");
    EXPECT_EQ(result.events[0].identity->session_id, "3");
    EXPECT_EQ(result.events[0].identity->proposal_sequence, 1U);
  }
  const auto held_after_ten = store.beginCycle(now, 2000U);
  EXPECT_TRUE(held_after_ten.availability_present);
  ASSERT_TRUE(held_after_ten.availability_identity.has_value());
  EXPECT_EQ(held_after_ten.availability_identity->proposal_sequence, 1U);
}

TEST(StateLatticeV2Binding,
     AvailabilitySummaryExpiresBeforeSameIdentityCanReachTenCycles) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  auto proposal = validProposal();
  setSafetyDeadline(proposal, 23, 90000000U);
  ASSERT_TRUE(store.enqueue(proposal, 1U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  const auto first = store.beginCycle(now, 10U);
  EXPECT_EQ(first.pp_cycle_sequence, 1U);
  EXPECT_TRUE(first.availability_present);
  EXPECT_EQ(first.availability_transition, v2::AvailabilityTransition::kFirst);
  for (std::uint32_t nanosec = 50000000U; nanosec < 90000000U;
       nanosec += 10000000U) {
    now.nanosec = nanosec;
    const auto held = store.beginCycle(now, 10U + nanosec);
    EXPECT_TRUE(held.availability_present);
    EXPECT_EQ(held.availability_transition, v2::AvailabilityTransition::kHeld);
  }
  now.nanosec = 90000000U;
  const auto expired = store.beginCycle(now, 100U);
  EXPECT_FALSE(expired.availability_present);
  EXPECT_EQ(expired.availability_transition, v2::AvailabilityTransition::kNone);
  EXPECT_LT(expired.pp_cycle_sequence, v2::BindingStore::kHoldCycles);
}

TEST(StateLatticeV2Binding, NewerProposalLegallyReplacesLiveAvailability) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  auto first = validProposal(1U);
  setSafetyDeadline(first, 23, 90000000U);
  ASSERT_TRUE(store.enqueue(first, 1U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.beginCycle(now, 10U).availability_present);
  auto replacement = validProposal(2U);
  replacement.proposal.safety_valid_until.sec = 25;
  replacement.proposal.base_lease_valid_until.sec = 25;
  recanonicalize(replacement);
  ASSERT_TRUE(store.enqueue(replacement, 2U, &reason));
  now.nanosec = 50000000U;
  const auto replaced = store.beginCycle(now, 20U);
  EXPECT_TRUE(replaced.availability_present);
  ASSERT_TRUE(replaced.availability_identity.has_value());
  EXPECT_EQ(replaced.availability_identity->proposal_sequence, 2U);
  EXPECT_EQ(replaced.availability_transition,
            v2::AvailabilityTransition::kReplaced);
}

TEST(StateLatticeV2Binding, AvailabilityOneCycleLateReplacementCreatesGap) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  auto first = validProposal(1U);
  setSafetyDeadline(first, 23, 50000000U);
  ASSERT_TRUE(store.enqueue(first, 1U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.beginCycle(now, 10U).availability_present);
  now.nanosec = 50000000U;
  EXPECT_FALSE(store.beginCycle(now, 20U).availability_present);
  ASSERT_TRUE(store.enqueue(validProposal(2U), 2U, &reason));
  now.nanosec = 60000000U;
  const auto next = store.beginCycle(now, 30U);
  EXPECT_TRUE(next.availability_present);
  EXPECT_EQ(next.availability_transition, v2::AvailabilityTransition::kFirst);
}

TEST(StateLatticeV2Binding, EmptyCycleHasObservableAvailabilitySummary) {
  v2::BindingStore store("5");
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  const auto result = store.beginCycle(now, 99U);
  EXPECT_EQ(result.pp_cycle_sequence, 1U);
  EXPECT_EQ(result.timer_entry_monotonic_ns, 99U);
  EXPECT_FALSE(result.availability_present);
  EXPECT_FALSE(result.availability_identity.has_value());
  EXPECT_EQ(result.availability_transition, v2::AvailabilityTransition::kNone);
  EXPECT_EQ(result.event_count, 0U);
}

TEST(StateLatticeV2Binding, RejectsIdentityMutationAndStaleProposal) {
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  auto mismatch = validProposal();
  ++mismatch.identity.source_generation;
  EXPECT_EQ(v2::validate(mismatch, now), v2::RejectReason::kIdentityMismatch);
  now.sec = 24;
  EXPECT_EQ(v2::validate(validProposal(), now),
            v2::RejectReason::kDeadlineMiss);
}

TEST(StateLatticeV2Binding, RejectsDuplicateOutOfOrderAndOverflow) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  ASSERT_TRUE(store.enqueue(validProposal(2U), 1U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.beginCycle(now, 1U).first_uptake);
  ASSERT_TRUE(store.enqueue(validProposal(2U), 2U, &reason));
  EXPECT_EQ(store.beginCycle(now, 2U).reject_reason,
            v2::RejectReason::kDuplicateOrReplay);
  ASSERT_TRUE(store.enqueue(validProposal(1U), 3U, &reason));
  EXPECT_EQ(store.beginCycle(now, 3U).reject_reason,
            v2::RejectReason::kOutOfOrder);

  for (std::size_t index = 0U; index < v2::BindingStore::kCapacity; ++index) {
    EXPECT_TRUE(store.enqueue(validProposal(3U + index), 4U + index, &reason));
  }
  EXPECT_FALSE(store.enqueue(validProposal(99U), 99U, &reason));
  EXPECT_EQ(reason, v2::RejectReason::kOverflow);
}

TEST(StateLatticeV2Binding, TerminalizesAtMostOneCandidatePerCycle) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  auto malformed = validProposal(1U);
  malformed.schema_version = malformed.SCHEMA_INVALID;
  ASSERT_TRUE(store.enqueue(malformed, 10U, &reason));
  ASSERT_TRUE(store.enqueue(validProposal(2U), 20U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;

  const auto first = store.beginCycle(now, 100U);
  EXPECT_EQ(first.reject_reason, v2::RejectReason::kMalformed);
  EXPECT_FALSE(first.first_uptake);
  EXPECT_EQ(store.pendingSize(), 1U);
  const auto second = store.beginCycle(now, 200U);
  EXPECT_TRUE(second.first_uptake);
  ASSERT_TRUE(second.accepted.has_value());
  EXPECT_EQ(second.accepted->identity.proposal_sequence, 2U);
}

TEST(StateLatticeV2Binding,
     RejectsProducerOrSessionSwitchAndClearsExpiredHold) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  ASSERT_TRUE(store.enqueue(validProposal(1U), 10U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.beginCycle(now, 100U).first_uptake);

  auto switched = validProposal(2U);
  switched.identity.producer_instance_id = "6";
  ASSERT_TRUE(store.enqueue(switched, 20U, &reason));
  EXPECT_EQ(store.beginCycle(now, 200U).reject_reason,
            v2::RejectReason::kIdentityMismatch);

  now.sec = 24;
  const auto expired = store.beginCycle(now, 300U);
  EXPECT_EQ(expired.reject_reason, v2::RejectReason::kStale);
  EXPECT_FALSE(expired.accepted.has_value());
  EXPECT_TRUE(expired.observed_identity.has_value());
}

TEST(StateLatticeV2Binding, WrongFirstProducerCannotCaptureCohort) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  auto wrong = validProposal(1U);
  wrong.identity.producer_instance_id = "6";
  wrong.proposal.plan_sample_key.planner_instance_id = 6U;
  auto encoded = canonical::canonicalizeAuthorizedTrajectoryV1(wrong.proposal);
  wrong.proposal.payload_sha256 = encoded.sha256;
  wrong.identity.canonical_sha256 = encoded.sha256;
  ASSERT_TRUE(store.enqueue(wrong, 10U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  EXPECT_EQ(store.beginCycle(now, 100U).reject_reason,
            v2::RejectReason::kIdentityMismatch);

  ASSERT_TRUE(store.enqueue(validProposal(2U), 20U, &reason));
  const auto expected = store.beginCycle(now, 200U);
  EXPECT_TRUE(expected.first_uptake);
  ASSERT_TRUE(expected.accepted.has_value());
  EXPECT_EQ(expected.accepted->identity.producer_instance_id, "5");
}

TEST(StateLatticeV2Binding, ClockInvalidFutureAndRegressionFailClosed) {
  builtin_interfaces::msg::Time invalid_now;
  invalid_now.sec = 0;
  EXPECT_EQ(v2::validate(validProposal(), invalid_now),
            v2::RejectReason::kClockFault);

  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 20000000U;
  EXPECT_EQ(v2::validate(validProposal(), now), v2::RejectReason::kClockFault);

  v2::BindingStore store("5");
  v2::RejectReason reason{};
  ASSERT_TRUE(store.enqueue(validProposal(), 10U, &reason));
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.beginCycle(now, 100U).first_uptake);
  now.nanosec = 35000000U;
  const auto regressed = store.beginCycle(now, 200U);
  EXPECT_EQ(regressed.reject_reason, v2::RejectReason::kClockFault);
  EXPECT_FALSE(regressed.accepted.has_value());
}

TEST(StateLatticeV2Binding, DirectBaseAttestationIsMandatoryAndExact) {
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  auto attestation = validAttestation();
  auto proposal = validProposal();
  bindProposalToAttestation(proposal, attestation);

  v2::BindingStore missing("5", "pp-primary", "3");
  v2::RejectReason reason{};
  ASSERT_TRUE(missing.enqueue(proposal, 10U, &reason));
  EXPECT_EQ(missing.beginCycle(now, 20U).reject_reason,
            v2::RejectReason::kBaseAttestationMissing);

  v2::BindingStore exact("5", "pp-primary", "3");
  ASSERT_TRUE(exact.recordBaseAttestation(attestation, now, &reason));
  ASSERT_TRUE(exact.enqueue(proposal, 10U, &reason));
  EXPECT_TRUE(exact.beginCycle(now, 20U).first_uptake);

  v2::BindingStore mutated("5", "pp-primary", "3");
  ASSERT_TRUE(mutated.recordBaseAttestation(attestation, now, &reason));
  ++proposal.proposal.base_lease_id;
  auto encoded = canonical::canonicalizeAuthorizedTrajectoryV1(proposal.proposal);
  proposal.proposal.payload_sha256 = encoded.sha256;
  proposal.identity.canonical_sha256 = encoded.sha256;
  ASSERT_TRUE(mutated.enqueue(proposal, 10U, &reason));
  EXPECT_EQ(mutated.beginCycle(now, 20U).reject_reason,
            v2::RejectReason::kBaseAttestationMismatch);
}

TEST(StateLatticeV2Binding, AttestationExpiryClockResetAndOverflowAreFailClosed) {
  auto attestation = validAttestation();
  builtin_interfaces::msg::Time now;
  now.sec = 24;
  EXPECT_EQ(v2::validateBaseAttestation(attestation, "pp-primary", "3", now),
            v2::RejectReason::kBaseAttestationExpired);

  v2::BindingStore store("5", "pp-primary", "3");
  v2::RejectReason reason{};
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.recordBaseAttestation(attestation, now, &reason));
  auto proposal = validProposal();
  bindProposalToAttestation(proposal, attestation);
  ASSERT_TRUE(store.enqueue(proposal, 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 2U).first_uptake);
  now.nanosec = 35000000U;
  EXPECT_EQ(store.beginCycle(now, 3U).reject_reason,
            v2::RejectReason::kClockFault);
  now.nanosec = 36000000U;
  EXPECT_EQ(store.beginCycle(now, 4U).reject_reason,
            v2::RejectReason::kClockFault);
  now.nanosec = 37000000U;
  EXPECT_FALSE(store.recordBaseAttestation(attestation, now, &reason));
  (void)store.beginCycle(now, 5U);
  EXPECT_TRUE(store.recordBaseAttestation(attestation, now, &reason));

  v2::BindingStore overflow("5");
  for (std::size_t index = 0U; index < v2::BindingStore::kCapacity; ++index) {
    ASSERT_TRUE(overflow.enqueue(validProposal(index + 1U), index, &reason));
  }
  EXPECT_FALSE(overflow.enqueue(validProposal(99U), 99U, &reason));
  const auto first = overflow.beginCycle(now, 100U);
  EXPECT_TRUE(first.run_invalid);
  EXPECT_EQ(first.reject_reason, v2::RejectReason::kOverflow);
  EXPECT_EQ(first.overflow_count, 1U);
  EXPECT_EQ(first.overflow_first_sequence, 99U);
  EXPECT_EQ(overflow.beginCycle(now, 101U).reject_reason,
            v2::RejectReason::kOverflow);
}

TEST(StateLatticeV2Binding,
     OverflowRemainsInvalidAcrossClockRecoveryAndCannotReceiveCredit) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  for (std::size_t index = 0U; index < v2::BindingStore::kCapacity; ++index) {
    ASSERT_TRUE(store.enqueue(validProposal(index + 1U), index, &reason));
  }
  ASSERT_FALSE(store.enqueue(validProposal(99U), 99U, &reason));
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  const auto overflow = store.beginCycle(now, 100U);
  EXPECT_TRUE(overflow.run_invalid);
  EXPECT_FALSE(overflow.availability_present);

  now.nanosec = 30000000U;
  EXPECT_TRUE(store.beginCycle(now, 101U).run_invalid);
  EXPECT_TRUE(store.beginCycle(now, 102U).run_invalid);
  now.nanosec = 40000000U;
  const auto recovered = store.beginCycle(now, 103U);
  EXPECT_TRUE(recovered.run_invalid);
  EXPECT_FALSE(recovered.availability_present);
  EXPECT_FALSE(store.enqueue(validProposal(100U), 104U, &reason));
  EXPECT_EQ(reason, v2::RejectReason::kOverflow);
  const auto no_credit = store.beginCycle(now, 105U);
  EXPECT_TRUE(no_credit.run_invalid);
  EXPECT_FALSE(no_credit.availability_present);
}

TEST(StateLatticeV2Binding,
     OverflowTerminalizesRejectedAndAdmittedIdentitiesExactlyOnce) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  for (std::size_t index = 0U; index < v2::BindingStore::kCapacity; ++index) {
    ASSERT_TRUE(store.enqueue(validProposal(index + 1U), 100U + index, &reason));
  }
  ASSERT_FALSE(store.enqueue(
      validProposal(v2::BindingStore::kCapacity + 1U), 200U, &reason));
  ASSERT_EQ(reason, v2::RejectReason::kOverflow);

  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  std::array<std::uint32_t, v2::BindingStore::kCapacity + 2U> terminal_counts{};
  const auto first = store.beginCycle(now, 1000U);
  EXPECT_TRUE(first.run_invalid);
  EXPECT_FALSE(first.accepted.has_value());
  ASSERT_EQ(first.event_count, 1U);
  ASSERT_TRUE(first.events[0].identity.has_value());
  EXPECT_EQ(first.events[0].identity->proposal_sequence,
            v2::BindingStore::kCapacity + 1U);
  EXPECT_EQ(first.events[0].reject_reason, v2::RejectReason::kOverflow);
  ++terminal_counts[first.events[0].identity->proposal_sequence];

  EXPECT_FALSE(store.enqueue(
      validProposal(v2::BindingStore::kCapacity + 2U), 300U, &reason));
  EXPECT_EQ(reason, v2::RejectReason::kOverflow);
  for (std::size_t cycle = 1U; cycle <= v2::BindingStore::kCapacity; ++cycle) {
    const auto result = store.beginCycle(now, 1000U + cycle);
    EXPECT_TRUE(result.run_invalid);
    EXPECT_FALSE(result.accepted.has_value());
    ASSERT_EQ(result.event_count, 1U);
    const auto &event = result.events[0];
    ASSERT_TRUE(event.identity.has_value());
    EXPECT_EQ(event.reject_reason, v2::RejectReason::kOverflow);
    EXPECT_FALSE(event.first_uptake);
    const auto sequence = event.identity->proposal_sequence;
    ASSERT_GE(sequence, 1U);
    ASSERT_LE(sequence, v2::BindingStore::kCapacity + 1U);
    ++terminal_counts[sequence];
  }
  for (std::size_t sequence = 1U;
       sequence <= v2::BindingStore::kCapacity + 1U; ++sequence) {
    EXPECT_EQ(terminal_counts[sequence], 1U) << "sequence=" << sequence;
  }
  EXPECT_EQ(store.pendingSize(), 0U);

  const auto drained = store.beginCycle(now, 2000U);
  EXPECT_TRUE(drained.run_invalid);
  EXPECT_FALSE(drained.accepted.has_value());
  EXPECT_EQ(drained.event_count, 0U);
  EXPECT_FALSE(drained.observed_identity.has_value());
}

TEST(StateLatticeV2Binding, ReplacementsFormOneContinuousAvailabilityChain) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  for (std::uint32_t cycle = 1U; cycle <= 20U; ++cycle) {
    if (cycle == 1U || cycle == 6U || cycle == 11U) {
      const std::uint64_t sequence = (cycle - 1U) / 5U + 1U;
      ASSERT_TRUE(store.enqueue(validProposal(sequence), cycle, &reason));
    }
    const auto result = store.beginCycle(now, 1000U + cycle);
    ASSERT_TRUE(result.availability_present);
    ASSERT_TRUE(result.availability_identity.has_value());
    const std::uint64_t expected_sequence =
        cycle < 6U ? 1U : (cycle < 11U ? 2U : 3U);
    EXPECT_EQ(result.availability_identity->proposal_sequence,
              expected_sequence);
    if (cycle == 1U) {
      EXPECT_EQ(result.availability_transition, v2::AvailabilityTransition::kFirst);
    } else if (cycle == 6U || cycle == 11U) {
      EXPECT_EQ(result.availability_transition,
                v2::AvailabilityTransition::kReplaced);
    }
  }
}

TEST(StateLatticeV2Binding, ReplacementRequiresStrictlyNewerPlanGeneration) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 10U).first_uptake);

  auto same_generation = validProposal(2U);
  same_generation.identity.plan_generation = 19U;
  same_generation.proposal.plan_sample_key.plan_generation = 19U;
  recanonicalize(same_generation);
  ASSERT_TRUE(store.enqueue(same_generation, 2U, &reason));
  const auto result = store.beginCycle(now, 20U);
  ASSERT_TRUE(result.availability_present);
  ASSERT_TRUE(result.availability_identity.has_value());
  EXPECT_EQ(result.availability_identity->proposal_sequence, 1U);
  EXPECT_EQ(result.availability_transition, v2::AvailabilityTransition::kHeld);
  ASSERT_EQ(result.event_count, 2U);
  EXPECT_EQ(result.events[1].reject_reason, v2::RejectReason::kOutOfOrder);
}

TEST(StateLatticeV2Binding, ExactDeadlineCanReplaceWithoutOldGenerationCredit) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time before_deadline;
  before_deadline.sec = 23;
  before_deadline.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(before_deadline, 10U).first_uptake);

  auto replacement = validProposal(2U);
  replacement.proposal.safety_valid_until.sec = 25;
  replacement.proposal.base_lease_valid_until.sec = 25;
  recanonicalize(replacement);
  ASSERT_TRUE(store.enqueue(replacement, 2U, &reason));
  builtin_interfaces::msg::Time at_deadline;
  at_deadline.sec = 24;
  const auto result = store.beginCycle(at_deadline, 20U);
  ASSERT_TRUE(result.availability_present);
  ASSERT_TRUE(result.availability_identity.has_value());
  EXPECT_EQ(result.availability_identity->proposal_sequence, 2U);
  EXPECT_EQ(result.availability_transition,
            v2::AvailabilityTransition::kReplaced);
  ASSERT_EQ(result.event_count, 2U);
  EXPECT_EQ(result.events[0].identity->proposal_sequence, 1U);
  EXPECT_EQ(result.events[0].reject_reason, v2::RejectReason::kStale);
  EXPECT_EQ(result.events[1].identity->proposal_sequence, 2U);
  EXPECT_TRUE(result.events[1].first_uptake);
}

TEST(StateLatticeV2Binding, EqualSequenceCannotReplaceWithHigherGeneration) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 10U).first_uptake);

  auto replay = validProposal(1U);
  replay.identity.plan_generation = 20U;
  replay.proposal.plan_sample_key.plan_generation = 20U;
  recanonicalize(replay);
  ASSERT_TRUE(store.enqueue(replay, 2U, &reason));
  const auto result = store.beginCycle(now, 20U);
  ASSERT_TRUE(result.availability_present);
  ASSERT_TRUE(result.availability_identity.has_value());
  EXPECT_EQ(result.availability_identity->proposal_sequence, 1U);
  EXPECT_EQ(result.availability_transition, v2::AvailabilityTransition::kHeld);
  ASSERT_EQ(result.event_count, 2U);
  EXPECT_EQ(result.events[1].reject_reason,
            v2::RejectReason::kDuplicateOrReplay);
}

TEST(StateLatticeV2Binding, HoldCounterExhaustionLatchesRunInvalidWithoutWrap) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 10U).first_uptake);
  v2::BindingStoreTestPeer::setHoldCycleIndex(
      store, std::numeric_limits<std::uint32_t>::max());

  now.nanosec = 50000000U;
  const auto overflow = store.beginCycle(now, 20U);
  EXPECT_TRUE(overflow.run_invalid);
  EXPECT_FALSE(overflow.availability_present);
  EXPECT_EQ(overflow.overflow_count, 1U);
  ASSERT_EQ(overflow.event_count, 1U);
  EXPECT_EQ(overflow.events[0].reject_reason, v2::RejectReason::kOverflow);
  EXPECT_EQ(overflow.events[0].hold_cycle_index,
            std::numeric_limits<std::uint32_t>::max());

  now.nanosec = 60000000U;
  const auto sticky = store.beginCycle(now, 30U);
  EXPECT_TRUE(sticky.run_invalid);
  EXPECT_FALSE(sticky.availability_present);
  EXPECT_EQ(sticky.overflow_count, 1U);
}

TEST(StateLatticeV2Binding, RejectedCandidateDoesNotSuppressExistingHold) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 10U).first_uptake);

  auto malformed = validProposal(2U);
  malformed.schema_version = malformed.SCHEMA_INVALID;
  ASSERT_TRUE(store.enqueue(malformed, 2U, &reason));
  const auto result = store.beginCycle(now, 20U);
  ASSERT_EQ(result.event_count, 2U);
  EXPECT_EQ(result.events[0].identity->proposal_sequence, 1U);
  EXPECT_EQ(result.events[0].hold_cycle_index, 2U);
  EXPECT_EQ(result.events[0].reject_reason, v2::RejectReason::kNone);
  EXPECT_EQ(result.events[1].identity->proposal_sequence, 2U);
  EXPECT_EQ(result.events[1].reject_reason, v2::RejectReason::kMalformed);
  EXPECT_EQ(result.reject_reason, v2::RejectReason::kMalformed);
}

TEST(StateLatticeV2Binding, RejectedCandidateDoesNotCreateAvailabilityGap) {
  v2::BindingStore store("5");
  v2::RejectReason reason{};
  builtin_interfaces::msg::Time now;
  now.sec = 23;
  now.nanosec = 40000000U;
  ASSERT_TRUE(store.enqueue(validProposal(1U), 1U, &reason));
  ASSERT_TRUE(store.beginCycle(now, 10U).availability_present);
  auto malformed = validProposal(2U);
  malformed.schema_version = malformed.SCHEMA_INVALID;
  ASSERT_TRUE(store.enqueue(malformed, 2U, &reason));
  const auto rejected = store.beginCycle(now, 20U);
  EXPECT_EQ(rejected.reject_reason, v2::RejectReason::kMalformed);
  ASSERT_TRUE(rejected.availability_identity.has_value());
  EXPECT_EQ(rejected.availability_identity->proposal_sequence, 1U);
}

} // namespace
