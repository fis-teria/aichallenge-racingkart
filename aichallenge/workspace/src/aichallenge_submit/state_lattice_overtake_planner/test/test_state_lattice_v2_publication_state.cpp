#include "state_lattice_overtake_planner/state_lattice_v2_publication_state.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace sl = state_lattice_overtake_planner;

namespace {

sl::StateLatticeV2SemanticKey key(std::uint8_t geometry_byte,
                                  std::uint8_t base_byte = 0U) {
  sl::StateLatticeV2SemanticKey result;
  result.geometry_sha256.front() = geometry_byte;
  result.base_snapshot_sha256.front() = base_byte;
  result.target_id = "d2";
  result.safety_valid_until_sec = 10;
  result.base_source_generation = 1U;
  return result;
}

TEST(StateLatticeV2PublicationState, InitialAndChangedKeysIncrementGeneration) {
  sl::StateLatticeV2PublicationState state;
  const auto initial = state.prepare(key(1U));
  ASSERT_EQ(initial.result, sl::StateLatticeV2PublicationPrepareResult::kReady);
  ASSERT_TRUE(initial.intent.has_value());
  EXPECT_EQ(initial.intent->proposal_sequence, 1U);
  EXPECT_EQ(initial.intent->plan_generation, 1U);
  EXPECT_EQ(state.proposalSequence(), 0U);
  EXPECT_EQ(state.planGeneration(), 0U);
  EXPECT_TRUE(state.commit(initial.intent.value()));

  const auto changed = state.prepare(key(2U));
  ASSERT_EQ(changed.result, sl::StateLatticeV2PublicationPrepareResult::kReady);
  ASSERT_TRUE(changed.intent.has_value());
  EXPECT_EQ(changed.intent->proposal_sequence, 2U);
  EXPECT_EQ(changed.intent->plan_generation, 2U);
  EXPECT_TRUE(state.commit(changed.intent.value()));
  EXPECT_EQ(state.proposalSequence(), 2U);
  EXPECT_EQ(state.planGeneration(), 2U);
}

TEST(StateLatticeV2PublicationState, DuplicateDoesNotMutateOrRequestPublish) {
  sl::StateLatticeV2PublicationState state;
  const auto initial = state.prepare(key(1U));
  ASSERT_TRUE(initial.intent.has_value());
  ASSERT_TRUE(state.commit(initial.intent.value()));

  EXPECT_EQ(state.prepare(key(1U)).result,
            sl::StateLatticeV2PublicationPrepareResult::kDuplicate);
  EXPECT_EQ(state.proposalSequence(), 1U);
  EXPECT_EQ(state.planGeneration(), 1U);
}

TEST(StateLatticeV2PublicationState, SemanticKeyDoesNotIncludeWireGeneration) {
  sl::StateLatticeV2PublicationState state;
  const auto initial = state.prepare(key(1U));
  ASSERT_TRUE(initial.intent.has_value());
  ASSERT_TRUE(state.commit(initial.intent.value()));

  // The V2 key has no global wire-generation field; unchanged V2 semantics
  // remain a duplicate regardless of unrelated publisher generations.
  EXPECT_EQ(state.prepare(key(1U)).result,
            sl::StateLatticeV2PublicationPrepareResult::kDuplicate);
}

TEST(StateLatticeV2PublicationState,
     SafetyAndBaseProvenanceMutationsRequestNewPublication) {
  sl::StateLatticeV2PublicationState state;
  const auto initial = state.prepare(key(1U));
  ASSERT_TRUE(initial.intent.has_value());
  ASSERT_TRUE(state.commit(initial.intent.value()));

  auto safety_until_changed = key(1U);
  ++safety_until_changed.safety_valid_until_sec;
  EXPECT_EQ(state.prepare(safety_until_changed).result,
            sl::StateLatticeV2PublicationPrepareResult::kReady);

  auto source_generation_changed = key(1U);
  ++source_generation_changed.base_source_generation;
  EXPECT_EQ(state.prepare(source_generation_changed).result,
            sl::StateLatticeV2PublicationPrepareResult::kReady);

  auto source_digest_changed = key(1U);
  source_digest_changed.base_source_sha256.front() = 1U;
  EXPECT_EQ(state.prepare(source_digest_changed).result,
            sl::StateLatticeV2PublicationPrepareResult::kReady);

  auto world_safety_changed = key(1U);
  world_safety_changed.world_safety_snapshot_sha256.front() = 1U;
  EXPECT_EQ(state.prepare(world_safety_changed).result,
            sl::StateLatticeV2PublicationPrepareResult::kReady);
}

TEST(StateLatticeV2PublicationState,
     PlanGenerationExhaustionDoesNotMutateAndIsDistinct) {
  sl::StateLatticeV2PublicationState state(
      7U, std::numeric_limits<std::uint32_t>::max());
  EXPECT_EQ(
      state.prepare(key(1U)).result,
      sl::StateLatticeV2PublicationPrepareResult::kPlanGenerationExhausted);
  EXPECT_EQ(state.proposalSequence(), 7U);
  EXPECT_EQ(state.planGeneration(), std::numeric_limits<std::uint32_t>::max());
}

TEST(StateLatticeV2PublicationState,
     SequenceExhaustionDoesNotMutateAndIsDistinct) {
  sl::StateLatticeV2PublicationState state(
      std::numeric_limits<std::uint64_t>::max(), 7U);
  EXPECT_EQ(state.prepare(key(1U)).result,
            sl::StateLatticeV2PublicationPrepareResult::kSequenceExhausted);
  EXPECT_EQ(state.proposalSequence(),
            std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(state.planGeneration(), 7U);
}

TEST(StateLatticeV2PublicationState,
     PublishFailureHaltsWithoutCommittingOrReusingIdentity) {
  sl::StateLatticeV2PublicationState state;
  const auto pending = state.prepare(key(1U));
  ASSERT_TRUE(pending.intent.has_value());

  // A publisher exception leaves delivery ambiguous: the caller halts rather
  // than committing or preparing the same identity again.
  state.halt();
  EXPECT_TRUE(state.halted());
  EXPECT_EQ(state.proposalSequence(), 0U);
  EXPECT_EQ(state.planGeneration(), 0U);
  EXPECT_EQ(state.prepare(key(1U)).result,
            sl::StateLatticeV2PublicationPrepareResult::kHalted);
}

TEST(StateLatticeV2PublicationState,
     PostSendCommitMismatchHaltsWithoutIdentityReuse) {
  sl::StateLatticeV2PublicationState state;
  auto pending = state.prepare(key(1U));
  ASSERT_TRUE(pending.intent.has_value());
  ++pending.intent->plan_generation;

  EXPECT_FALSE(state.commit(pending.intent.value()));
  state.halt();
  EXPECT_TRUE(state.halted());
  EXPECT_EQ(state.proposalSequence(), 0U);
  EXPECT_EQ(state.planGeneration(), 0U);
  EXPECT_EQ(state.prepare(key(2U)).result,
            sl::StateLatticeV2PublicationPrepareResult::kHalted);
}

} // namespace
