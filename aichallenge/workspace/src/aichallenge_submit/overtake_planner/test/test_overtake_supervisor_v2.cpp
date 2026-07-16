#include "overtake_planner/overtake_supervisor_v2.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

overtake_planner::CandidateTrajectory candidate(
    overtake_planner::CandidateType type, bool feasible, double score = 0.0) {
  overtake_planner::CandidateTrajectory value;
  value.type = type;
  value.feasible = feasible;
  value.score = score;
  value.x = {0.0, 1.0};
  value.y = {0.0, 0.0};
  value.yaw = {0.0, 0.0};
  value.d = {0.0, 0.0};
  value.v_ref = {2.0, 2.0};
  return value;
}

overtake_planner::SupervisorV2Input input(
    const std::vector<overtake_planner::CandidateTrajectory> &candidates) {
  overtake_planner::SupervisorV2Input value;
  value.target_present = true;
  value.target_vehicle_id = "d2";
  value.safety_inputs_complete = true;
  value.tracking_usable = true;
  value.pass_start_allowed = true;
  value.abort_release_allowed = true;
  value.candidates = &candidates;
  return value;
}

TEST(OvertakeSupervisorV2, Gate2PassBeatsFollowImmediately) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true, 0.0),
      candidate(overtake_planner::CandidateType::PASS_LEFT, true, -1.0)};

  const auto decision = supervisor.update(input(candidates));

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(decision.pass_direction, 1);
  EXPECT_TRUE(decision.trajectory_authorized);
}

TEST(OvertakeSupervisorV2, RejectedPassSelectsAttackFollow) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true),
      candidate(overtake_planner::CandidateType::PASS_LEFT, false)};

  const auto decision = supervisor.update(input(candidates));

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(decision.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakeSupervisorV2, PassingDoesNotFlipSideAndLossEntersAbortHold) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> initial = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  const auto passing = supervisor.update(input(initial));
  ASSERT_EQ(passing.phase, overtake_planner::TacticalPhase::PASSING);

  const std::vector<overtake_planner::CandidateTrajectory> changed = {
      candidate(overtake_planner::CandidateType::PASS_RIGHT, true),
      candidate(overtake_planner::CandidateType::RECOVERY, true)};
  const auto aborted = supervisor.update(input(changed));

  EXPECT_EQ(aborted.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(aborted.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_NE(aborted.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(aborted.pass_direction, 1);
  EXPECT_EQ(aborted.attempt_id, passing.attempt_id);
}

TEST(OvertakeSupervisorV2, PassingDoesNotReapplyStartOnlyGate) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  auto first_input = input(candidates);
  const auto started = supervisor.update(first_input);
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);

  auto continuation = input(candidates);
  continuation.pass_start_allowed = false;
  const auto continued = supervisor.update(continuation);

  EXPECT_EQ(continued.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(continued.attempt_id, started.attempt_id);
}

TEST(OvertakeSupervisorV2, UnusableTrackingDoesNotAuthorizeTrajectory) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto unusable = input(candidates);
  unusable.tracking_usable = false;

  const auto decision = supervisor.update(unusable);

  EXPECT_EQ(decision.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_FALSE(decision.trajectory_authorized);
}

TEST(OvertakeSupervisorV2, AbortHoldRequiresConsecutiveClearCycles) {
  overtake_planner::OvertakeSupervisorV2 supervisor(2);
  const std::vector<overtake_planner::CandidateTrajectory> pass = {
      candidate(overtake_planner::CandidateType::PASS_LEFT, true)};
  supervisor.update(input(pass));

  const std::vector<overtake_planner::CandidateTrajectory> hold = {
      candidate(overtake_planner::CandidateType::RECOVERY, true),
      candidate(overtake_planner::CandidateType::FOLLOW, true)};
  auto hold_input = input(hold);
  const auto entered = supervisor.update(hold_input);
  const auto still_held = supervisor.update(hold_input);
  const auto released = supervisor.update(hold_input);

  EXPECT_EQ(entered.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(still_held.phase, overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(released.phase, overtake_planner::TacticalPhase::ATTACK_FOLLOW);
}

TEST(OvertakeSupervisorV2, IdenticalDecisionKeepsGenerationStable) {
  overtake_planner::OvertakeSupervisorV2 supervisor;
  const std::vector<overtake_planner::CandidateTrajectory> candidates = {
      candidate(overtake_planner::CandidateType::FOLLOW, true)};

  const auto first = supervisor.update(input(candidates));
  const auto second = supervisor.update(input(candidates));

  EXPECT_EQ(first.plan_generation, second.plan_generation);
}

} // namespace
