#include "overtake_planner/behavior_state_machine.hpp"

#include <gtest/gtest.h>

TEST(BehaviorStateMachine, BlockedWithoutEnoughSafeCyclesFallsBackToFollow) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 5.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::PASS_LEFT, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, RepeatedSafePassTransitionsToPrepare) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(BehaviorStateMachine,
     ClassifierDropoutContinuityCanCompleteSameTargetPassDebounce) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.pass_start_target_continuity_active = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_target_id = "d2";
  blocked.straight_overtake_start_allowed = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passRightSafeCycles(), 1);

  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(sm.passRightSafeCycles(), 2);
}

TEST(BehaviorStateMachine,
     PassDebounceDoesNotTransferAcrossTargetSideOrExpiredContinuity) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.pass_start_target_continuity_active = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_target_id = "d2";
  blocked.straight_overtake_start_allowed = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(sm.passRightSafeCycles(), 1);

  blocked.maneuver_target_id = "d3";
  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passRightSafeCycles(), 1);
  EXPECT_EQ(sm.passSafeCycleResetReason(), "target_changed");

  mode = sm.update(1.2, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passLeftSafeCycles(), 1);
  EXPECT_EQ(sm.passRightSafeCycles(), 0);
  EXPECT_EQ(sm.passSafeCycleResetReason(), "side_changed");

  blocked.pass_start_target_continuity_expired = true;
  mode = sm.update(1.3, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passLeftSafeCycles(), 1);
  EXPECT_EQ(sm.passSafeCycleResetReason(), "continuity_expired");
}

TEST(BehaviorStateMachine,
     PreparedStartGridPassWaitsForTrackingProofThenExecutes) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 5.0;
  config.min_mode_hold_time_sec = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_target_confirmed_stationary = true;
  blocked.maneuver_transaction_prepared = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_transaction_tracking_release_pending = true;
  blocked.maneuver_transaction_tracking_stop_active = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;
  blocked.straight_overtake_start_allowed = true;
  blocked.maneuver_target_pass_safety_approved = true;
  blocked.overtake_permission_allowed = true;

  auto mode = sm.update(
      0.10, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
      overtake_planner::CandidateType::PASS_RIGHT, blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  mode = sm.update(0.15, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  // 新規生成PASSの診断が一周期だけ不成立でも、停止下で現在周期へ再評価した
  // 保存PASSがselectedとして成立している間は、一般PREPARE分岐へ抜けて
  // tracking proof前に内部FSMだけOVERTAKEへ進めない。
  blocked.maneuver_target_pass_safety_approved = false;
  mode = sm.update(0.18, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  blocked.maneuver_transaction_tracking_release_confirmed = true;
  blocked.maneuver_transaction_tracking_stop_active = false;
  mode = sm.update(0.20, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(BehaviorStateMachine,
     IncompleteTransactionRetriesSamePassAfterConsecutiveSafeCycles) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;
  blocked.straight_overtake_start_allowed = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(BehaviorStateMachine,
     PendingStartGridReleaseWithoutUsablePassCannotEnterAbort) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.abort_timeout_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.maneuver_transaction_prepared = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_transaction_tracking_release_pending = true;
  blocked.maneuver_transaction_tracking_stop_active = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;

  const auto from_prepare = sm.update(
      1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT,
      overtake_planner::CandidateType::SAFE_STOP, blocked, false);
  EXPECT_EQ(from_prepare, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  blocked.maneuver_transaction_prepared = false;
  blocked.maneuver_transaction_incomplete = true;
  const auto from_overtake = sm.update(
      2.0, overtake_planner::BehaviorMode::OVERTAKE_RIGHT,
      overtake_planner::CandidateType::RECOVERY, blocked, false);
  EXPECT_EQ(from_overtake, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     PreviouslyAuthorizedTransactionResumesSameSafePassWithoutNewStartDelay) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 5.0;
  config.min_mode_hold_time_sec = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_retry_active = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_chain_tail_observed = true;
  // This is continuation of an already approved profile.  A curve may reject
  // a brand-new pass start, but must not impose a new-start delay when the same
  // fresh target/side/profile passes SafetyEvaluator again.
  blocked.straight_overtake_start_allowed = false;

  const auto mode =
      sm.update(0.10, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::PASS_RIGHT, blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(BehaviorStateMachine, PrepareRequiresSameDirectionPassToCommit) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT,
                overtake_planner::CandidateType::FOLLOW, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, PrepareCannotCommitInfeasibleSameSidePass) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.straight_overtake_start_allowed = true;

  overtake_planner::BehaviorStateMachine left_sm(config);
  const auto left = left_sm.update(
      1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT,
      overtake_planner::CandidateType::PASS_LEFT, blocked, false);
  EXPECT_EQ(left, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  overtake_planner::BehaviorStateMachine right_sm(config);
  const auto right = right_sm.update(
      1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT,
      overtake_planner::CandidateType::PASS_RIGHT, blocked, false);
  EXPECT_EQ(right, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, PrepareRechecksCurrentPassStartGateBeforeCommit) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.straight_overtake_start_allowed = false;
  blocked.overtake_permission_allowed = true;

  overtake_planner::BehaviorStateMachine left_sm(config);
  const auto left = left_sm.update(
      1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT,
      overtake_planner::CandidateType::PASS_LEFT, blocked, true);
  EXPECT_EQ(left, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  overtake_planner::BehaviorStateMachine right_sm(config);
  const auto right = right_sm.update(
      1.0, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT,
      overtake_planner::CandidateType::PASS_RIGHT, blocked, true);
  EXPECT_EQ(right, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     IncompleteFollowTransactionDoesNotReleaseToFreeRunOnClassifierDropout) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::FASTEST, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, RecoveryWithObservedFollowTargetKeepsFollowBlocked) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_index = 0;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::RECOVERY, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     BrakingFollowWithoutAuthoritativeTargetDoesNotOverrideFreeRun) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.braking_follow_active = true;
  blocked.braking_follow_feasible = true;
  blocked.braking_follow_index = -1;
  blocked.braking_follow_id.clear();

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::FASTEST, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine,
     FeasibleBrakingFollowWithAuthoritativeTargetOverridesFreeRun) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.braking_follow_active = true;
  blocked.braking_follow_feasible = true;
  blocked.braking_follow_index = 0;
  blocked.braking_follow_id = "d1";

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::FASTEST, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     RecoveryWithoutTargetOrTransactionReleasesFollowBlocked) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  const overtake_planner::BlockedInfo blocked;
  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::RECOVERY, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine,
     RecoveryWithoutFollowTargetAbortsIncompleteTransaction) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_transaction_incomplete = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::RECOVERY, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine, RecoveryWithoutTargetKeepsReentryInAbortRecovery) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.reentry_hold_active = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::RECOVERY, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine,
     IncompleteFollowTransactionAbortsBeforeUsingAnotherTargetFollow) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = false;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::FOLLOW, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine,
     SafeCurrentLateralYieldKeepsIncompleteFollowTransaction) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_transaction_safe_lateral_hold_active = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::YIELD_BEHIND, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     RejectedCurrentLateralYieldDoesNotKeepIncompleteFollowTransaction) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_transaction_safe_lateral_hold_active = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::YIELD_BEHIND, blocked, false);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine,
     UncommittedStartGridCurrentLateralYieldDoesNotStartReentry) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_uncommitted_hold_active = true;
  blocked.start_grid_target_index = 0;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                overtake_planner::CandidateType::YIELD_BEHIND, blocked, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  overtake_planner::BehaviorStateMachine initial_sm(config);
  const auto initial_next =
      initial_sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                        overtake_planner::CandidateType::YIELD_BEHIND, blocked,
                        true);
  EXPECT_EQ(initial_next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, CurvedStartGateKeepsBlockedFollow) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.straight_overtake_start_allowed = false;
  blocked.overtake_start_gate_reason = "curve";

  auto mode =
      sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::PASS_LEFT, blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, GentleCurveSafetyApprovalBypassesOnlyModeHold) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.60;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.straight_overtake_start_allowed = true;
  blocked.gentle_curve_safe_pass_start_approved = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(0.10, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  mode = sm.update(0.15, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine,
     CleanSpeedGuardPreservesGentleCurvePassDebounceBeforeFollowHandoff) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_id = "d2";
  blocked.straight_overtake_start_allowed = true;
  blocked.gentle_curve_safe_pass_start_approved = true;

  auto mode = sm.update(
      0.10, overtake_planner::BehaviorMode::SPEED_GUARD,
      overtake_planner::CandidateType::PASS_RIGHT, blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passRightSafeCycles(), 1);
  EXPECT_NE(sm.passSafeCycleResetReason(), "mode_not_pass_start");

  mode = sm.update(0.15, mode, overtake_planner::CandidateType::PASS_RIGHT,
                   blocked, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(sm.passRightSafeCycles(), 2);
}

TEST(BehaviorStateMachine,
     AbortRecoveryNeverAccumulatesUncommittedGentleCurvePassDebounce) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_id = "d2";
  blocked.straight_overtake_start_allowed = true;
  blocked.gentle_curve_safe_pass_start_approved = true;

  const auto mode = sm.update(
      0.10, overtake_planner::BehaviorMode::ABORT_RECOVERY,
      overtake_planner::CandidateType::PASS_RIGHT, blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.passRightSafeCycles(), 0);
  EXPECT_EQ(sm.passSafeCycleResetReason(), "mode_not_pass_start");
}

TEST(BehaviorStateMachine, OrdinaryPassCannotBypassModeHold) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.60;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.straight_overtake_start_allowed = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(0.10, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  mode = sm.update(0.15, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, InfeasibleCycleResetsPassSafetyContinuity) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;

  auto mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  mode = sm.update(1.0, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);
  mode = sm.update(1.1, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, false);
  mode = sm.update(1.2, mode, overtake_planner::CandidateType::PASS_LEFT,
                   blocked, true);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, SideBySideFromFreeRunUsesDistanceKeepMode) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.side_by_side = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::SIDE_BY_SIDE_KEEP, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
}

TEST(BehaviorStateMachine, SideBySideReturnsToFreeRunWhenClear) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP,
                overtake_planner::CandidateType::FASTEST, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine,
     SideBySideKeepReturnsToFollowWhenSideClearsButFrontStillBlocked) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.side_by_side = false;
  info.blocked = true;

  const auto next =
      sm.update(1.0, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP,
                overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, SideBySideKeepsOvertakeAndDoesNotMerge) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.side_by_side = true;
  info.blocked = false;
  info.front_delta_s = 10.0;
  info.straight_overtake_start_allowed = false;
  info.overtake_start_gate_reason = "curve";

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::PASS_LEFT, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine, CurvedStartGateDoesNotAbortActiveOvertake) {
  overtake_planner::PlannerConfig config;
  config.abort_timeout_sec = 5.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.front_delta_s = 3.0;
  info.straight_overtake_start_allowed = false;
  info.overtake_start_gate_reason = "curve";

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::PASS_LEFT, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine, FeasibleSameSidePassIgnoresAbortTimeout) {
  overtake_planner::PlannerConfig config;
  config.abort_timeout_sec = 1.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.front_delta_s = 3.0;

  const auto next =
      sm.update(5.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::PASS_LEFT, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine,
     VerifiedCurrentLateralRecoveryHandsIncompletePassToAttackFollow) {
  overtake_planner::PlannerConfig config;
  config.abort_timeout_sec = 1.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.front_delta_s = 3.0;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;
  info.maneuver_transaction_safe_lateral_hold_active = true;

  const auto next =
      sm.update(5.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, UnverifiedRecoveryCannotBypassIncompletePassAbort) {
  overtake_planner::PlannerConfig config;
  config.abort_timeout_sec = 1.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.front_delta_s = 3.0;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;
  info.maneuver_transaction_safe_lateral_hold_active = false;

  const auto next =
      sm.update(5.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine, FeasibleRecoveryFromFreeRunUsesSpeedGuard) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::SPEED_GUARD);
}

TEST(BehaviorStateMachine, OvertakeTransitionsToYieldWhenGapIsLost) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.can_pass_left = false;
  info.can_pass_right = false;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::OVERTAKE_LEFT,
                overtake_planner::CandidateType::YIELD_BEHIND, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, YieldReturnsToFollowAfterRejoinGap) {
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_gap_m = 3.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.nearest_index = 0;
  info.front_delta_s = 3.5;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
                overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, CornerYieldWaitsForWiderRejoinGap) {
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

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
                overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, FutureYieldHoldStaysUntilCornerClears) {
  overtake_planner::PlannerConfig config;
  config.corner_side_yield_curvature_m_inv = 0.06;
  config.yield_rejoin_wall_clearance_m = 0.15;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.future_yield_required = true;
  info.future_corner_side_by_side = true;
  info.corner_abs_curvature = 0.08;
  info.ego_wall_clearance_m = 0.5;

  auto mode =
      sm.update(2.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::YIELD_BEHIND, info, true);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  ASSERT_TRUE(sm.futureYieldHoldActive());

  overtake_planner::BlockedInfo near_corner;
  near_corner.corner_abs_curvature = 0.08;
  near_corner.ego_wall_clearance_m = 0.5;
  mode = sm.update(2.1, mode, overtake_planner::CandidateType::FASTEST,
                   near_corner, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_TRUE(sm.futureYieldHoldActive());

  overtake_planner::BlockedInfo after_corner;
  after_corner.corner_abs_curvature = 0.0;
  after_corner.ego_wall_clearance_m = 0.5;
  mode = sm.update(2.2, mode, overtake_planner::CandidateType::FASTEST,
                   after_corner, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(sm.futureYieldHoldActive());
}

TEST(BehaviorStateMachine, FutureYieldHoldRespectsMinimumModeHoldTime) {
  overtake_planner::PlannerConfig config;
  config.corner_side_yield_curvature_m_inv = 0.06;
  config.yield_rejoin_wall_clearance_m = 0.15;
  config.min_mode_hold_time_sec = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.future_yield_required = true;
  info.future_corner_side_by_side = true;
  info.corner_abs_curvature = 0.08;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.0;

  auto mode =
      sm.update(2.0, overtake_planner::BehaviorMode::FREE_RUN,
                overtake_planner::CandidateType::YIELD_BEHIND, info, true);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  ASSERT_TRUE(sm.futureYieldHoldActive());

  overtake_planner::BlockedInfo clear;
  clear.ego_wall_clearance_m = 0.5;
  clear.ego_lateral_offset_m = 0.0;
  mode = sm.update(2.2, mode, overtake_planner::CandidateType::FASTEST, clear,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_TRUE(sm.futureYieldHoldActive());

  mode = sm.update(2.7, mode, overtake_planner::CandidateType::FASTEST, clear,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(sm.futureYieldHoldActive());
}

TEST(BehaviorStateMachine, YieldWaitsUntilEgoHasWallClearance) {
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_wall_clearance_m = 0.15;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.nearest_index = 0;
  info.front_delta_s = 8.0;
  info.ego_wall_clearance_m = 0.05;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
                overtake_planner::CandidateType::FOLLOW, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(BehaviorStateMachine, AbortRecoveryReleasesAtCenterDuringHighSpeedCurve) {
  overtake_planner::PlannerConfig config;
  config.high_speed_curve_lateral_hold_enabled = true;
  config.high_speed_curve_lateral_hold_release_speed_mps = 2.5;
  config.high_speed_curve_lateral_hold_release_curvature_m_inv = 0.025;
  config.yield_rejoin_wall_clearance_m = 0.15;
  config.recovery_release_lateral_error_m = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.0;
  info.ego_speed_mps = 5.0;
  info.corner_abs_curvature = 0.08;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                overtake_planner::CandidateType::FASTEST, info, true);

  // 高速カーブのholdはCoreだけがreentry gate通過を確認してSPEED_GUARDへ
  // 分離する。状態機械単体はABORTを保持しない。
  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine, AbortRecoveryResetsPassSafetyCyclesBeforeRetry) {
  overtake_planner::PlannerConfig config;
  config.pass_safe_required_cycles = 2.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.0;

  // ABORT中にPASS候補が安全でも、安全周期としては蓄積しない。
  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                        overtake_planner::CandidateType::PASS_LEFT, info, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  mode = sm.update(2.1, mode, overtake_planner::CandidateType::PASS_LEFT, info,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  mode = sm.update(2.2, mode, overtake_planner::CandidateType::PASS_LEFT, info,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(BehaviorStateMachine, AbortRecoveryReleasesAfterLowSpeedInCurve) {
  overtake_planner::PlannerConfig config;
  config.high_speed_curve_lateral_hold_enabled = true;
  config.high_speed_curve_lateral_hold_release_speed_mps = 2.5;
  config.high_speed_curve_lateral_hold_release_curvature_m_inv = 0.025;
  config.yield_rejoin_wall_clearance_m = 0.15;
  config.recovery_release_lateral_error_m = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.0;
  info.ego_speed_mps = 2.0;
  info.corner_abs_curvature = 0.08;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                overtake_planner::CandidateType::FASTEST, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine, YieldWaitsUntilLateralErrorRecovers) {
  overtake_planner::PlannerConfig config;
  config.yield_release_lateral_error_m = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 1.2;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::YIELD_BEHIND,
                        overtake_planner::CandidateType::FASTEST, info, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::YIELD_BEHIND);

  info.ego_lateral_offset_m = 0.2;
  mode = sm.update(3.0, mode, overtake_planner::CandidateType::FASTEST, info,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine, AbortRecoveryWaitsUntilEgoHasWallClearance) {
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_wall_clearance_m = 0.15;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.05;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine, AbortRecoveryWaitsUntilLateralErrorRecovers) {
  overtake_planner::PlannerConfig config;
  config.yield_rejoin_wall_clearance_m = 0.15;
  config.recovery_release_lateral_error_m = 0.60;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = -1.2;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                        overtake_planner::CandidateType::FASTEST, info, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);

  info.ego_lateral_offset_m = -0.2;
  mode = sm.update(3.0, mode, overtake_planner::CandidateType::FASTEST, info,
                   true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine,
     AbortRecoveryHandsVerifiedIncompletePassHoldToAttackFollow) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.2;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;
  info.maneuver_transaction_safe_lateral_hold_active = true;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, AbortRecoveryDoesNotReleaseIncompletePassToFreeRun) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.ego_wall_clearance_m = 0.5;
  info.ego_lateral_offset_m = 0.2;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                overtake_planner::CandidateType::FASTEST, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(BehaviorStateMachine,
     SpeedGuardHandsVerifiedIncompletePassHoldToAttackFollow) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;
  info.maneuver_transaction_safe_lateral_hold_active = true;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::SPEED_GUARD,
                overtake_planner::CandidateType::RECOVERY, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine, SpeedGuardRetainsIncompletePassWithoutSafeHandoff) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_target_latched = true;
  info.maneuver_target_observed = true;
  info.maneuver_target_fresh = true;

  const auto next =
      sm.update(2.0, overtake_planner::BehaviorMode::SPEED_GUARD,
                overtake_planner::CandidateType::FASTEST, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::SPEED_GUARD);
}

TEST(BehaviorStateMachine, SafeStopRequestTransitionsToSafeStop) {
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

TEST(BehaviorStateMachine, SafeStopHoldsUntilReleaseCyclesSatisfied) {
  overtake_planner::PlannerConfig config;
  config.safe_stop_release_cycles = 2;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                        overtake_planner::CandidateType::SAFE_STOP, info, true,
                        safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  safe_stop.requested = false;
  safe_stop.candidate_feasible = true;
  safe_stop.release_ready = false;
  mode = sm.update(2.1, mode, overtake_planner::CandidateType::FASTEST, info,
                   true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sm.safeStopReleaseCount(), 0);

  safe_stop.release_ready = true;
  mode = sm.update(2.2, mode, overtake_planner::CandidateType::FASTEST, info,
                   true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(sm.safeStopReleaseCount(), 1);

  mode = sm.update(2.3, mode, overtake_planner::CandidateType::FASTEST, info,
                   true, safe_stop);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(BehaviorStateMachine,
     SafeStopHandsOffToSpeedGuardWhenReleasedButNotCentered) {
  overtake_planner::PlannerConfig config;
  config.min_mode_hold_time_sec = 0.0;
  config.safe_stop_lateral_error_threshold_m = 0.40;
  config.safe_stop_release_wall_clearance_m = 0.20;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;
  safe_stop.candidate_feasible = true;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                        overtake_planner::CandidateType::SAFE_STOP, info, true,
                        safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  info.ego_lateral_offset_m = 0.75;
  info.ego_wall_clearance_m = 0.10;
  safe_stop.requested = false;
  safe_stop.release_ready = false;
  mode = sm.update(2.1, mode, overtake_planner::CandidateType::FASTEST, info,
                   true, safe_stop);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(sm.safeStopHoldCount(), 0);
  EXPECT_EQ(sm.safeStopReleaseCount(), 0);
}

TEST(BehaviorStateMachine, SafeStopReleaseWithBlockedFrontReturnsToFollow) {
  overtake_planner::PlannerConfig config;
  config.safe_stop_release_cycles = 1;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;
  safe_stop.candidate_feasible = true;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                        overtake_planner::CandidateType::SAFE_STOP, info, true,
                        safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  info.blocked = true;
  safe_stop.requested = false;
  safe_stop.release_ready = true;
  mode = sm.update(2.1, mode, overtake_planner::CandidateType::FOLLOW, info,
                   true, safe_stop);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(sm.safeStopHoldCount(), 0);
  EXPECT_EQ(sm.safeStopReleaseCount(), 0);
}

TEST(BehaviorStateMachine, SafeStopLeavesWhenStopCandidateBecomesInfeasible) {
  overtake_planner::PlannerConfig config;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  overtake_planner::SafeStopContext safe_stop;
  safe_stop.requested = true;
  safe_stop.candidate_feasible = true;

  auto mode = sm.update(2.0, overtake_planner::BehaviorMode::FOLLOW_BLOCKED,
                        overtake_planner::CandidateType::SAFE_STOP, info, true,
                        safe_stop);
  ASSERT_EQ(mode, overtake_planner::BehaviorMode::SAFE_STOP);

  safe_stop.requested = false;
  safe_stop.candidate_feasible = false;
  mode = sm.update(2.1, mode, overtake_planner::CandidateType::FOLLOW, info,
                   false, safe_stop);

  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     UnverifiedStartGridTrackingStopKeepsTransactionOutOfAbort) {
  overtake_planner::PlannerConfig config;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.start_grid_target_active = true;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_transaction_retry_active = true;
  info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  info.maneuver_transaction_tracking_stop_active = true;
  info.maneuver_transaction_safe_lateral_hold_active = false;

  // current-d停止軌道も不成立で、通常なら実行中PASSからABORTへ落ちる入力。
  // 横軌道を認可せずauthorityが縦停止するため、FSMはtarget/sideを保持できる
  // FOLLOW_BLOCKEDへ留まり、中心復帰を開始しない。
  const auto next = sm.update(
      2.0, overtake_planner::BehaviorMode::OVERTAKE_RIGHT,
      overtake_planner::CandidateType::YIELD_BEHIND, info, true);

  EXPECT_EQ(next, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(BehaviorStateMachine,
     IncompletePassTrackingStopDoesNotHandOwnershipToFallbackModes) {
  overtake_planner::PlannerConfig config;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::BehaviorStateMachine sm(config);

  overtake_planner::BlockedInfo info;
  info.blocked = true;
  info.maneuver_transaction_incomplete = true;
  info.maneuver_transaction_retry_active = true;
  info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  info.maneuver_transaction_tracking_stop_active = true;
  info.maneuver_transaction_safe_lateral_hold_active = false;

  auto mode = overtake_planner::BehaviorMode::OVERTAKE_RIGHT;
  for (const auto fallback : {overtake_planner::CandidateType::YIELD_BEHIND,
                              overtake_planner::CandidateType::RECOVERY,
                              overtake_planner::CandidateType::SAFE_STOP}) {
    mode = sm.update(2.0, mode, fallback, info, true);
    EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  }

  // exact current-d STOPがSafetyEvaluatorを通った場合も、SAFE_STOP modeへ
  // transaction ownerを移さずFOLLOW_BLOCKEDで同じPASS再開を待つ。
  info.maneuver_transaction_safe_lateral_hold_active = true;
  mode = sm.update(2.1, overtake_planner::BehaviorMode::ABORT_RECOVERY,
                   overtake_planner::CandidateType::SAFE_STOP, info, true);
  EXPECT_EQ(mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}
