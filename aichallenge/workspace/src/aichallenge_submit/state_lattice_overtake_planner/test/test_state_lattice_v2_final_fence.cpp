#include "state_lattice_overtake_planner/state_lattice_v2_final_fence.hpp"

#include <gtest/gtest.h>

#include <thread>

namespace sl = state_lattice_overtake_planner;

namespace {

// Cross-language canonical known-answer contract, mirrored verbatim by
// tools/aic_test/tests/test_v2_final_fence.py:
// v1, execution-1, producer 7101, session 9, epoch 9, request 1.
constexpr sl::FinalFenceDigest kQuiesceRequestKnownAnswerSha256{
    0x75U, 0x4bU, 0x85U, 0x36U, 0x38U, 0xc4U, 0x04U, 0xfaU,
    0x4fU, 0xdbU, 0x10U, 0x67U, 0x7eU, 0x56U, 0xfbU, 0x44U,
    0x5eU, 0x46U, 0x32U, 0x87U, 0x30U, 0x88U, 0xa0U, 0x54U,
    0x32U, 0x76U, 0x0dU, 0x74U, 0x41U, 0xa5U, 0x2aU, 0x06U};

// Cross-language nonzero-final-identity fence known-answer contract, mirrored
// verbatim by tools/aic_test/tests/test_v2_final_fence.py. Fields are:
// execution-1, producer 7101, session/epoch 9, F/count 1, identity sequence 1,
// generation 1/1, source stamp 1s+0ns, map, digest 01+31x00, publish mono 101,
// quiesced/fence mono 200/201, and the config() QoS fingerprint.
constexpr sl::FinalFenceDigest kFinalFenceKnownAnswerSha256{
    0x6fU, 0xb4U, 0x3aU, 0x62U, 0x10U, 0x5cU, 0x9eU, 0x33U,
    0x8bU, 0xfbU, 0x50U, 0x6cU, 0x04U, 0xc9U, 0x32U, 0x6dU,
    0xcdU, 0x69U, 0xe4U, 0x02U, 0x5aU, 0x96U, 0xd7U, 0xdcU,
    0xf3U, 0x0cU, 0xabU, 0xceU, 0x84U, 0x95U, 0xabU, 0xe4U};

sl::StateLatticeV2FinalFenceState::Config config(bool enabled = true) {
  return {enabled, "execution-1", "7101", "9", 9U,
          "topic=/planning/overtake/state_lattice/v2_proposal;"
          "type=multi_purpose_mpc_ros_msgs/msg/"
          "AuthorizedCartesianTrajectoryV2;reliability=reliable;"
          "durability=volatile;history=keep_last;depth=8"};
}

sl::FinalFenceRequest request() {
  sl::FinalFenceRequest value{1U, "execution-1", "7101", "9", 9U, 1U, {}};
  value.canonical_sha256 = sl::final_fence_canonical::requestDigest(value);
  return value;
}

sl::FinalFenceIdentity identity(std::uint64_t ordinal) {
  sl::FinalFenceIdentity value;
  value.producer_instance_id = "7101";
  value.session_id = "9";
  value.proposal_sequence = ordinal;
  value.plan_generation = static_cast<std::uint32_t>(ordinal);
  value.source_generation = 1U;
  value.source_stamp_sec = 1;
  value.frame_id = "map";
  value.canonical_sha256.front() = static_cast<std::uint8_t>(ordinal);
  value.publish_monotonic_ns = 100U + ordinal;
  return value;
}

void commit(sl::StateLatticeV2FinalFenceState *state, std::uint64_t ordinal) {
  auto admission = state->tryAdmit("7101", "9", 9U);
  ASSERT_TRUE(admission.has_value());
  ASSERT_TRUE(state->finishCommitted(std::move(admission.value()), ordinal,
                                     identity(ordinal)));
}

TEST(StateLatticeV2FinalFence, DefaultOffHasNoAdmissionOrFence) {
  sl::StateLatticeV2FinalFenceState state(config(false));
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kDisabled);
  EXPECT_FALSE(state.tryAdmit("7101", "9", 9U).has_value());
  EXPECT_EQ(state.requestQuiesce(request(), 10U),
            sl::FinalFenceRequestResult::kDisabled);
  EXPECT_FALSE(state.prepareFence(11U).has_value());
}

TEST(StateLatticeV2FinalFence, ZeroOneAndMultipleCommittedClosures) {
  for (const std::uint64_t count : {0U, 1U, 3U}) {
    sl::StateLatticeV2FinalFenceState state(config());
    for (std::uint64_t ordinal = 1U; ordinal <= count; ++ordinal) {
      commit(&state, ordinal);
    }
    EXPECT_EQ(state.requestQuiesce(request(), 200U),
              sl::FinalFenceRequestResult::kAccepted);
    const auto fence = state.prepareFence(201U);
    ASSERT_TRUE(fence.has_value());
    EXPECT_EQ(fence->final_committed_ordinal, count);
    EXPECT_EQ(fence->successful_emission_count, count);
    EXPECT_EQ(fence->final_identity_present, count != 0U);
    if (count != 0U) {
      EXPECT_EQ(fence->final_identity.proposal_sequence, count);
    }
    EXPECT_EQ(fence->canonical_sha256,
              sl::final_fence_canonical::fenceDigest(fence.value()));
    EXPECT_FALSE(state.prepareFence(202U).has_value());
    EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kFenced);
    EXPECT_FALSE(state.tryAdmit("7101", "9", 9U).has_value());
  }
}

TEST(StateLatticeV2FinalFence, UncommittedAndAmbiguousDoNotAdvanceOrdinal) {
  sl::StateLatticeV2FinalFenceState uncommitted(config());
  auto rejected = uncommitted.tryAdmit("7101", "9", 9U);
  ASSERT_TRUE(rejected.has_value());
  uncommitted.finishUncommitted(std::move(rejected.value()));
  EXPECT_EQ(uncommitted.successfulEmissionCount(), 0U);

  sl::StateLatticeV2FinalFenceState ambiguous(config());
  auto admitted = ambiguous.tryAdmit("7101", "9", 9U);
  ASSERT_TRUE(admitted.has_value());
  ambiguous.finishAmbiguous(std::move(admitted.value()));
  EXPECT_EQ(ambiguous.successfulEmissionCount(), 0U);
  EXPECT_FALSE(ambiguous.evidenceFault().empty());
  EXPECT_EQ(ambiguous.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  EXPECT_FALSE(ambiguous.prepareFence(21U).has_value());
}

TEST(StateLatticeV2FinalFence, RequestDuringAdmissionDrainsWithoutBlocking) {
  sl::StateLatticeV2FinalFenceState state(config());
  auto admission = state.tryAdmit("7101", "9", 9U);
  ASSERT_TRUE(admission.has_value());
  EXPECT_EQ(state.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kQuiescing);
  EXPECT_FALSE(state.tryAdmit("7101", "9", 9U).has_value());
  EXPECT_FALSE(state.prepareFence(21U).has_value());
  EXPECT_TRUE(state.finishCommitted(std::move(admission.value()), 1U,
                                    identity(1U)));
  EXPECT_EQ(state.inFlight(), 0U);
  EXPECT_TRUE(state.prepareFence(22U).has_value());
}

TEST(StateLatticeV2FinalFence, MalformedTrafficOnlyInvalidatesEvidence) {
  sl::StateLatticeV2FinalFenceState state(config());
  auto malformed = request();
  malformed.execution_nonce = "wrong";
  EXPECT_EQ(state.requestQuiesce(malformed, 20U),
            sl::FinalFenceRequestResult::kMalformed);
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kOpen);
  commit(&state, 1U);
  EXPECT_EQ(state.finalCommittedOrdinal(), 1U);
  EXPECT_FALSE(state.evidenceFault().empty());
  EXPECT_EQ(state.requestQuiesce(request(), 21U),
            sl::FinalFenceRequestResult::kDuplicateOrConflicting);
  EXPECT_FALSE(state.prepareFence(22U).has_value());
}

TEST(StateLatticeV2FinalFence, WrongEpochAndOrdinalMismatchFailClosed) {
  sl::StateLatticeV2FinalFenceState state(config());
  EXPECT_FALSE(state.tryAdmit("7101", "9", 10U).has_value());
  auto admitted = state.tryAdmit("7101", "9", 9U);
  ASSERT_TRUE(admitted.has_value());
  EXPECT_FALSE(state.finishCommitted(std::move(admitted.value()), 2U,
                                     identity(2U)));
  EXPECT_EQ(state.finalCommittedOrdinal(), 0U);
  EXPECT_FALSE(state.evidenceFault().empty());
}

TEST(StateLatticeV2FinalFence, CanonicalRequestBindingIsDeterministic) {
  const auto first = request();
  const auto second = request();
  EXPECT_EQ(first.canonical_sha256, second.canonical_sha256);
  auto changed = second;
  changed.sealed_epoch_id = 10U;
  EXPECT_NE(first.canonical_sha256,
            sl::final_fence_canonical::requestDigest(changed));
}

TEST(StateLatticeV2FinalFence, CrossLanguageQuiesceRequestKnownAnswer) {
  EXPECT_EQ(request().canonical_sha256, kQuiesceRequestKnownAnswerSha256);
}

TEST(StateLatticeV2FinalFence,
     CrossLanguageNonzeroIdentityFinalFenceKnownAnswer) {
  sl::FinalFenceSnapshot snapshot;
  snapshot.execution_nonce = "execution-1";
  snapshot.producer_instance_id = "7101";
  snapshot.session_id = "9";
  snapshot.sealed_epoch_id = 9U;
  snapshot.fence_id = 1U;
  snapshot.request_id = 1U;
  snapshot.request_canonical_sha256 = kQuiesceRequestKnownAnswerSha256;
  snapshot.final_committed_ordinal = 1U;
  snapshot.successful_emission_count = 1U;
  snapshot.final_identity_present = true;
  snapshot.final_identity = identity(1U);
  snapshot.proposal_qos_fingerprint = config().proposal_qos_fingerprint;
  snapshot.quiesced_monotonic_ns = 200U;
  snapshot.fence_publish_monotonic_ns = 201U;
  EXPECT_EQ(sl::final_fence_canonical::fenceDigest(snapshot),
            kFinalFenceKnownAnswerSha256);
}

TEST(StateLatticeV2FinalFence, RequestAfterFenceIsTerminalAndCannotReplaceFence) {
  sl::StateLatticeV2FinalFenceState state(config());
  commit(&state, 1U);
  EXPECT_EQ(state.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  const auto first = state.prepareFence(21U);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(state.requestQuiesce(request(), 22U),
            sl::FinalFenceRequestResult::kAlreadyTerminal);
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kFenced);
  EXPECT_EQ(state.finalCommittedOrdinal(), 1U);
  EXPECT_FALSE(state.prepareFence(23U).has_value());
  EXPECT_EQ(first->canonical_sha256,
            sl::final_fence_canonical::fenceDigest(first.value()));
}

TEST(StateLatticeV2FinalFence, PostFenceAdmissionIsRejectedAndFaultsEvidence) {
  sl::StateLatticeV2FinalFenceState state(config());
  EXPECT_EQ(state.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  ASSERT_TRUE(state.prepareFence(21U).has_value());
  EXPECT_FALSE(state.tryAdmit("7101", "9", 9U).has_value());
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kFenced);
  EXPECT_EQ(state.finalCommittedOrdinal(), 0U);
  EXPECT_EQ(state.evidenceFault(), "post_fence_proposal_admission");
  EXPECT_FALSE(state.prepareFence(22U).has_value());
}

TEST(StateLatticeV2FinalFence, DuplicateOrConflictingRequestCannotQuiesceTwice) {
  sl::StateLatticeV2FinalFenceState duplicate(config());
  EXPECT_EQ(duplicate.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  EXPECT_EQ(duplicate.requestQuiesce(request(), 21U),
            sl::FinalFenceRequestResult::kDuplicateOrConflicting);
  EXPECT_FALSE(duplicate.prepareFence(22U).has_value());

  sl::StateLatticeV2FinalFenceState conflicting(config());
  EXPECT_EQ(conflicting.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  auto changed = request();
  changed.request_id = 2U;
  changed.canonical_sha256 =
      sl::final_fence_canonical::requestDigest(changed);
  EXPECT_EQ(conflicting.requestQuiesce(changed, 21U),
            sl::FinalFenceRequestResult::kDuplicateOrConflicting);
  EXPECT_FALSE(conflicting.prepareFence(22U).has_value());
}

TEST(StateLatticeV2FinalFence,
     AmbiguousFencePublicationRemainsFencedWithoutReplacement) {
  sl::StateLatticeV2FinalFenceState state(config());
  commit(&state, 1U);
  EXPECT_EQ(state.requestQuiesce(request(), 20U),
            sl::FinalFenceRequestResult::kAccepted);
  const auto frozen = state.prepareFence(21U);
  ASSERT_TRUE(frozen.has_value());
  state.markFencePublicationAmbiguous();
  EXPECT_EQ(state.phase(), sl::FinalFencePublicationPhase::kFenced);
  EXPECT_EQ(state.finalCommittedOrdinal(), 1U);
  EXPECT_EQ(state.successfulEmissionCount(), 1U);
  EXPECT_EQ(state.evidenceFault(), "fence_publication_ambiguous");
  EXPECT_FALSE(state.prepareFence(22U).has_value());
  EXPECT_FALSE(state.tryAdmit("7101", "9", 9U).has_value());
}

} // namespace
