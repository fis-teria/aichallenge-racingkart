#include "state_lattice_overtake_planner/speed_transition_evidence.hpp"

#include <gtest/gtest.h>

#include <type_traits>

namespace state_lattice_overtake_planner {
namespace {

TrialSpeedEvidence completeTrial(double pre_speed_mps,
                                 double post_speed_mps) {
  TrialSpeedEvidence trial{};
  trial.capture_state = SpeedEvidenceCaptureState::TRIAL_COMPLETE_UNSEALED;
  trial.present_fields = kSpeedEvidenceRequiredFields;
  trial.branch = SpeedTransitionBranch::CANDIDATE_JERK_LIMITED;
  trial.effective_dt_sec = 0.05;
  trial.requested_speed_mps = post_speed_mps;
  trial.pre_state.initialized = 1U;
  trial.pre_state.command_speed_mps = pre_speed_mps;
  trial.post_state.initialized = 1U;
  trial.post_state.command_speed_mps = post_speed_mps;
  return trial;
}

SpeedEvidenceIdentity identity(std::uint64_t attempt,
                               std::uint64_t commit) {
  SpeedEvidenceIdentity value{};
  value.planner_session_id = 17U;
  value.state_epoch = 4U;
  value.attempt_ordinal = attempt;
  value.commit_ordinal = commit;
  value.predecessor_commit_ordinal = commit > 1U ? commit - 1U : 0U;
  value.previous_committed_attempt_ordinal = attempt > 1U ? attempt - 1U : 0U;
  return value;
}

TEST(SpeedTransitionEvidence, RecordsRemainFixedSizeAndTriviallyCopyable) {
  static_assert(std::is_trivially_copyable_v<TrialSpeedEvidence>);
  static_assert(std::is_trivially_copyable_v<CommittedSpeedEvidence>);
  const TrialSpeedEvidence zero{};
  EXPECT_EQ(zero.capture_state, SpeedEvidenceCaptureState::EMPTY);
  EXPECT_EQ(zero.present_fields, 0U);
  EXPECT_EQ(zero.branch, SpeedTransitionBranch::NONE);
}

TEST(SpeedTransitionEvidence, SealRejectsMissingExactPayloadBinding) {
  auto trial = completeTrial(1.0, 0.9);
  trial.present_fields &= ~SPEED_EVIDENCE_PAYLOAD_BINDING;

  const auto committed =
      sealCommittedSpeedEvidenceAfterCommit(trial, identity(10U, 3U));

  EXPECT_EQ(committed.sealed_after_commit, 1U);
  EXPECT_EQ(committed.trial.capture_state,
            SpeedEvidenceCaptureState::INVALID_CAPTURE);
  EXPECT_EQ(committed.trial.failure,
            SpeedEvidenceFailure::PAYLOAD_BINDING_MISSING);
}

TEST(SpeedTransitionEvidence, SealAcceptsOnlyCompleteTrial) {
  const auto committed = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), identity(10U, 3U));

  EXPECT_EQ(committed.sealed_after_commit, 1U);
  EXPECT_EQ(committed.trial.capture_state,
            SpeedEvidenceCaptureState::COMMITTED_SEALED);
  EXPECT_EQ(committed.trial.failure, SpeedEvidenceFailure::NONE);
}

TEST(SpeedTransitionEvidence, SealRejectsUnqualifiedProductionBranch) {
  auto trial = completeTrial(1.0, 0.9);
  trial.branch = SpeedTransitionBranch::MOVING_FOLLOW_DIRECT;
  const auto committed =
      sealCommittedSpeedEvidenceAfterCommit(trial, identity(10U, 3U));

  EXPECT_EQ(committed.trial.capture_state,
            SpeedEvidenceCaptureState::INVALID_CAPTURE);
  EXPECT_EQ(committed.trial.failure,
            SpeedEvidenceFailure::UNQUALIFIED_BRANCH);
}

TEST(SpeedTransitionEvidence, MailboxAcceptsOneExactAdjacentPair) {
  OneShotSpeedEvidencePairMailbox mailbox;
  auto predecessor = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), identity(10U, 3U));
  auto target_trial = completeTrial(0.9, 0.8);
  auto target_identity = identity(11U, 4U);
  target_identity.predecessor_commit_ordinal = 3U;
  target_identity.previous_committed_attempt_ordinal = 10U;
  auto target =
      sealCommittedSpeedEvidenceAfterCommit(target_trial, target_identity);

  EXPECT_TRUE(mailbox.offer(predecessor));
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::WRITING);
  EXPECT_TRUE(mailbox.offer(target));
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::READY);
  EXPECT_EQ(mailbox.dropCount(), 0U);
}

TEST(SpeedTransitionEvidence, MailboxFailsClosedOnContinuityMismatch) {
  OneShotSpeedEvidencePairMailbox mailbox;
  auto predecessor = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), identity(10U, 3U));
  auto target = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(0.8, 0.7), identity(11U, 4U));

  ASSERT_TRUE(mailbox.offer(predecessor));
  EXPECT_FALSE(mailbox.offer(target));
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::CONSUMED);
  EXPECT_EQ(mailbox.dropCount(), 1U);
}

TEST(SpeedTransitionEvidence, EpochResetDiscardsAnUnconsumedReadyPair) {
  OneShotSpeedEvidencePairMailbox mailbox;
  auto predecessor = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), identity(10U, 3U));
  auto target_identity = identity(11U, 4U);
  target_identity.predecessor_commit_ordinal = 3U;
  target_identity.previous_committed_attempt_ordinal = 10U;
  auto target = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(0.9, 0.8), target_identity);
  ASSERT_TRUE(mailbox.offer(predecessor));
  ASSERT_TRUE(mailbox.offer(target));
  ASSERT_EQ(mailbox.state(), SpeedEvidenceMailboxState::READY);

  mailbox.beginEpoch();

  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::EMPTY);
  EXPECT_EQ(mailbox.dropCount(), 1U);
  EXPECT_EQ(mailbox.predecessor().sealed_after_commit, 0U);
  EXPECT_EQ(mailbox.target().sealed_after_commit, 0U);
}

TEST(SpeedTransitionEvidence, IncompleteCycleInvalidatesOnlyAnOpenPair) {
  OneShotSpeedEvidencePairMailbox mailbox;
  mailbox.invalidate();
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::EMPTY);
  EXPECT_EQ(mailbox.dropCount(), 0U);

  auto predecessor = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), identity(10U, 3U));
  ASSERT_TRUE(mailbox.offer(predecessor));
  mailbox.invalidate();
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::CONSUMED);
  EXPECT_EQ(mailbox.dropCount(), 1U);

  mailbox.beginEpoch();
  EXPECT_EQ(mailbox.state(), SpeedEvidenceMailboxState::EMPTY);
}

TEST(SpeedTransitionEvidence,
     CommitAuthorityAdvancesOnlyAfterExplicitPlannerMoveBoundary) {
  SpeedEvidenceCommitState state;
  const auto before = state;

  discardSpeedEvidenceTrial(state); // deterministic deadline reuse
  EXPECT_EQ(state.commit_ordinal, before.commit_ordinal);
  EXPECT_EQ(state.record_sequence, before.record_sequence);
  EXPECT_EQ(state.mailbox.state(), SpeedEvidenceMailboxState::EMPTY);
  discardSpeedEvidenceTrial(state); // deterministic deadline stop
  EXPECT_EQ(state.commit_ordinal, before.commit_ordinal);
  EXPECT_EQ(state.record_sequence, before.record_sequence);

  auto first_identity = prepareSpeedEvidenceCommitAfterPlannerMove(
      state, 17U, 4U, 10U);
  auto first = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), first_identity);
  finishSpeedEvidenceCommit(state, first);
  EXPECT_EQ(state.commit_ordinal, 1U);
  EXPECT_EQ(state.record_sequence, 1U);
  EXPECT_EQ(state.mailbox.state(), SpeedEvidenceMailboxState::WRITING);

  discardSpeedEvidenceTrial(state); // an attempt may exist between commits
  auto second_identity = prepareSpeedEvidenceCommitAfterPlannerMove(
      state, 17U, 4U, 13U);
  EXPECT_EQ(second_identity.commit_ordinal, 2U);
  EXPECT_EQ(second_identity.predecessor_commit_ordinal, 1U);
  EXPECT_EQ(second_identity.previous_committed_attempt_ordinal, 10U);
  auto second = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(0.9, 0.8), second_identity);
  finishSpeedEvidenceCommit(state, second);
  EXPECT_EQ(state.mailbox.state(), SpeedEvidenceMailboxState::READY);
}

TEST(SpeedTransitionEvidence,
     InvalidCommittedCaptureBreaksPairAndCannotReuseStaleBinding) {
  SpeedEvidenceCommitState state;
  auto first_identity = prepareSpeedEvidenceCommitAfterPlannerMove(
      state, 17U, 4U, 10U);
  auto first = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(1.0, 0.9), first_identity);
  finishSpeedEvidenceCommit(state, first);
  ASSERT_EQ(state.mailbox.state(), SpeedEvidenceMailboxState::WRITING);

  auto invalid_trial = completeTrial(0.9, 0.8);
  invalid_trial.present_fields &= ~SPEED_EVIDENCE_PAYLOAD_BINDING;
  auto invalid_identity = prepareSpeedEvidenceCommitAfterPlannerMove(
      state, 17U, 4U, 11U);
  auto invalid =
      sealCommittedSpeedEvidenceAfterCommit(invalid_trial, invalid_identity);
  finishSpeedEvidenceCommit(state, invalid);
  EXPECT_EQ(state.mailbox.state(), SpeedEvidenceMailboxState::CONSUMED);

  auto later_identity = prepareSpeedEvidenceCommitAfterPlannerMove(
      state, 17U, 4U, 12U);
  EXPECT_EQ(later_identity.predecessor_commit_ordinal,
            invalid.identity.commit_ordinal);
  auto later = sealCommittedSpeedEvidenceAfterCommit(
      completeTrial(0.8, 0.7), later_identity);
  finishSpeedEvidenceCommit(state, later);
  EXPECT_NE(state.mailbox.state(), SpeedEvidenceMailboxState::READY);
}

} // namespace
} // namespace state_lattice_overtake_planner
