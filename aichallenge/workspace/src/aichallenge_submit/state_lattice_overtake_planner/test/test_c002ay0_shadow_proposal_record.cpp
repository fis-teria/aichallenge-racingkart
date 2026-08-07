#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp"

#include <gtest/gtest.h>

#include <type_traits>

namespace shadow = state_lattice_overtake_planner::c002ay0_shadow;

namespace {

shadow::FixedProposalStaticConfig validConfig() {
  shadow::FixedProposalStaticConfig config;
  config.safety_evaluation_enabled = 1U;
  config.frame_size = 3U;
  config.frame[0] = 'm';
  config.frame[1] = 'a';
  config.frame[2] = 'p';
  config.wheel_base_m = 1.087;
  config.ego_stale_sec = 0.5;
  config.evaluator_implementation_sha256.fill(0x11U);
  config.evaluator_config_sha256.fill(0x22U);
  return config;
}

shadow::FixedProposalRecord record(std::uint64_t session_generation,
                                   std::uint64_t session_nonce,
                                   std::uint64_t attempt_id) {
  shadow::FixedProposalRecord result;
  result.session_generation = session_generation;
  result.capture_monotonic_ns = 1U;
  result.session_nonce = session_nonce;
  result.planner_instance_id = 9U;
  result.attempt_id = attempt_id;
  result.connector_transaction_id = attempt_id;
  result.authority_token = attempt_id;
  result.safety_snapshot_id = attempt_id;
  result.plan_generation = 3U;
  result.candidate_revision = static_cast<std::uint32_t>(attempt_id);
  return result;
}

} // namespace

TEST(C002Ay0ShadowProposalRecord, FixedAbiAndFifoDropNew) {
  static_assert(std::is_trivially_copyable_v<shadow::FixedProposalRecord>);
  static_assert(std::is_standard_layout_v<shadow::FixedProposalRecord>);
  EXPECT_TRUE(shadow::fixedProposalAtomicsAreLockFree());

  shadow::FixedProposalQueue queue;
  ASSERT_TRUE(
      shadow::initializeFixedProposalQueue(queue, 2U, 7U, 11U, validConfig()));
  queue.worker_ready = 1U;
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 11U, 1U)),
            shadow::FixedProposalQueueResult::kAccepted);
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 11U, 2U)),
            shadow::FixedProposalQueueResult::kAccepted);
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 11U, 3U)),
            shadow::FixedProposalQueueResult::kFull);
  EXPECT_EQ(queue.dropped_full_count, 1U);

  shadow::FixedProposalRecord output;
  ASSERT_EQ(shadow::tryPopFixedProposal(queue, output),
            shadow::FixedProposalQueueResult::kAccepted);
  EXPECT_EQ(output.attempt_id, 1U);
  ASSERT_EQ(shadow::tryPopFixedProposal(queue, output),
            shadow::FixedProposalQueueResult::kAccepted);
  EXPECT_EQ(output.attempt_id, 2U);
  EXPECT_EQ(shadow::tryPopFixedProposal(queue, output),
            shadow::FixedProposalQueueResult::kEmpty);
}

TEST(C002Ay0ShadowProposalRecord, DisabledUnreadyAndSessionMismatchFailClosed) {
  shadow::FixedProposalQueue queue;
  ASSERT_TRUE(
      shadow::initializeFixedProposalQueue(queue, 1U, 7U, 11U, validConfig()));
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 11U, 1U)),
            shadow::FixedProposalQueueResult::kDisabled);
  queue.worker_ready = 1U;
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 12U, 1U)),
            shadow::FixedProposalQueueResult::kInvalid);
  shadow::disableFixedProposalQueue(queue);
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, record(7U, 11U, 1U)),
            shadow::FixedProposalQueueResult::kDisabled);
}

TEST(C002Ay0ShadowProposalRecord, CaptureTimestampIsRequired) {
  shadow::FixedProposalQueue queue;
  ASSERT_TRUE(
      shadow::initializeFixedProposalQueue(queue, 1U, 7U, 11U, validConfig()));
  queue.worker_ready = 1U;
  auto invalid = record(7U, 11U, 1U);
  invalid.capture_monotonic_ns = 0U;
  EXPECT_EQ(shadow::tryPushFixedProposal(queue, invalid),
            shadow::FixedProposalQueueResult::kInvalid);
  EXPECT_EQ(queue.invalid_record_count, 1U);
}

TEST(C002Ay0ShadowProposalRecord, ExplicitWorkerStartsAndBoundedlyStops) {
  shadow::FixedProposalWorkerConfig config;
  config.enabled = true;
  config.executable_path = TEST_C002AY0_SHADOW_WORKER_PATH;
  config.session_generation = 7U;
  config.session_nonce = 11U;
  config.static_config = validConfig();

  auto session = shadow::FixedProposalWorkerSession::start(config);
  ASSERT_NE(session, nullptr);
  EXPECT_TRUE(session->ready());
  EXPECT_EQ(session->publishedCount(), 0U);
  EXPECT_EQ(session->validationRejectCount(), 0U);
  const auto diagnostics = session->diagnostics();
  EXPECT_TRUE(diagnostics.accepting);
  EXPECT_TRUE(diagnostics.worker_ready);
  EXPECT_GT(diagnostics.worker_process_age_ns, 0U);
  EXPECT_EQ(diagnostics.serialization_reject_count, 0U);
  const auto outcome = session->shutdown();
  EXPECT_NE(
      outcome,
      overtake_transport_contract::c002ay0::WorkerShutdownOutcome::kError);
  EXPECT_NE(outcome, overtake_transport_contract::c002ay0::
                         WorkerShutdownOutcome::kReapUnconfirmed);
}
