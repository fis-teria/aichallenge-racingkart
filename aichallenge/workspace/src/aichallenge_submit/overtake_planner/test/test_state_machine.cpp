#include "overtake_planner/behavior_state_machine.hpp"

#include <gtest/gtest.h>

TEST(BehaviorStateMachine, BlockedWithoutEnoughSafeCyclesFallsBackToFollow)
{
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 5.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;

  const auto next = sm.update(
    1.0, overtake_planner::BehaviorMode::FREE_RUN,
    overtake_planner::CandidateType::PASS_LEFT, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, RepeatedSafePassTransitionsToPrepare)
{
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_RIGHT, blocked, true);
  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_RIGHT, blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(BehaviorStateMachine, SideBySideFromFreeRunUsesDistanceKeepMode)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.side_by_side = true;

  const auto next = sm.update(
    1.0, overtake_planner::BehaviorMode::FREE_RUN,
    overtake_planner::CandidateType::SIDE_BY_SIDE_KEEP, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
}

TEST(BehaviorStateMachine, SideBySideReturnsToFreeRunWhenClear)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;

  const auto next = sm.update(
    1.0, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP,
    overtake_planner::CandidateType::FASTEST, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine, SideBySideKeepsOvertakeAndDoesNotMerge)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.side_by_side = true;
  info.blocked = false;
  info.front_delta_s = 10.0;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
    overtake_planner::CandidateType::PASS_LEFT, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine, OvertakeTransitionsToYieldWhenGapIsLost)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.can_pass_left = false;
  info.can_pass_right = false;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
    overtake_planner::CandidateType::YIELD_BEHIND, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, YieldReturnsToFollowAfterRejoinGap)
{
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_gap_m = 3.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.nearest_index = 0;
  info.front_delta_s = 3.5;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
    overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, CornerYieldWaitsForWiderRejoinGap)
{
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_gap_m = 3.0;
  config.corner_yield_rejoin_gap_m = 5.5;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.side_by_side = true;
  info.corner_side_by_side = true;
  info.nearest_index = 0;
  info.front_delta_s = 4.0;
  info.ego_wall_clearance_m = 0.5;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
    overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, YieldWaitsUntilEgoHasWallClearance)
{
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_wall_clearance_m = 0.15;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.nearest_index = 0;
  info.front_delta_s = 8.0;
  info.ego_wall_clearance_m = 0.05;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
    overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, AbortRecoveryWaitsUntilEgoHasWallClearance)
{
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_wall_clearance_m = 0.15;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.05;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
    overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine, SafeStopRequestTransitionsToSafeStop)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;
  safe_stop.candidate_feasible = true;

  const auto next = sm.update(
    2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
    overtake_planner::CandidateType::SAFE_STOP, info, true, safe_stop);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sm.safeStopHoldCount(), 1);
}

TEST(BehaviorStateMachine, SafeStopHoldsUntilReleaseCyclesSatisfied)
{
  overtake_planner::PlannerConfig config;
  config.safe_stop_release_cycles = 2;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;

  auto mode = sm.update(
    2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
    overtake_planner::CandidateType::SAFE_STOP, info, true, safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  safe_stop.requested = false;
  safe_stop.candidate_feasible = true;
  safe_stop.release_ready = false;
  mode = sm.update(
    2.1, mode, overtake_planner::CandidateType::FASTEST, info, true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sm.safeStopReleaseCount(), 0);

  safe_stop.release_ready = true;
  mode = sm.update(
    2.2, mode, overtake_planner::CandidateType::FASTEST, info, true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sm.safeStopReleaseCount(), 1);

  mode = sm.update(
    2.3, mode, overtake_planner::CandidateType::FASTEST, info, true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine, SafeStopLeavesWhenStopCandidateBecomesInfeasible)
{
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;
  safe_stop.candidate_feasible = true;

  auto mode = sm.update(
    2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
    overtake_planner::CandidateType::SAFE_STOP, info, true, safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  safe_stop.requested = false;
  safe_stop.candidate_feasible = false;
  mode = sm.update(
    2.1, mode, overtake_planner::CandidateType::FOLLOW, info, false, safe_stop);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}
