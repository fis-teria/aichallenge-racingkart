#include "overtake_planner/overtake_planner_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

overtake_planner::FrenetFrame makeStraightFrame()
{
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
      static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.0, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::FrenetFrame makeCurvedFrame()
{
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
      static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.12, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::EgoState makeEgo(
  const overtake_planner::FrenetFrame & frame, double x, double d)
{
  overtake_planner::EgoState ego;
  ego.stamp_sec = 0.0;
  ego.x = x;
  ego.y = d;
  ego.yaw = 0.0;
  ego.v = 4.0;
  ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
  ego.valid = true;
  return ego;
}

overtake_planner::OpponentState makeOpponent(
  const overtake_planner::FrenetFrame & frame, double x, double d)
{
  overtake_planner::OpponentState opp;
  opp.id = "npc";
  opp.stamp_sec = 0.0;
  opp.x = x;
  opp.y = d;
  opp.vx = 4.0;
  opp.vy = 0.0;
  opp.v = 4.0;
  opp.frenet = frame.cartesianToFrenet(opp.x, opp.y, 0.0);
  opp.valid = true;
  return opp;
}

overtake_planner::PlannerConfig makeConfig()
{
  overtake_planner::PlannerConfig config;
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.side_by_side_s_m = 4.0;
  config.side_margin_m = 1.2;
  config.side_yield_s_m = 0.3;
  config.side_by_side_target_gap_m = 1.1;
  config.side_by_side_shift_distance_m = 2.0;
  config.side_by_side_speed_cap_mps = 4.5;
  config.min_pass_gap_m = 1.45;
  config.pass_gap_hysteresis_m = 0.15;
  config.yield_speed_margin_mps = 0.6;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  return config;
}

}  // namespace

TEST(OvertakePlannerCore, SideBySideOpponentOnLeftMovesRightAndOverrides)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.2, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::SIDE_BY_SIDE_KEEP);
  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_EQ(output.blocked_info.side_id, "npc");
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(output.lateral_offsets.back(), -0.35);
  EXPECT_LE(output.speed_caps.back(), config.side_by_side_speed_cap_mps);
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}

TEST(OvertakePlannerCore, SideBySideOpponentAheadYieldsBehind)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.6, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_GT(output.blocked_info.side_delta_s, config.side_yield_s_m);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}

TEST(OvertakePlannerCore, CornerSideBySideYieldsBehindWithCloseSpeedCap)
{
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.1, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.corner_side_by_side);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
  EXPECT_NEAR(output.target_lateral_offset_m, config.corner_yield_target_d_m, 1.0e-9);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, CornerSideBySideLeadCarDoesNotPushLaterally)
{
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.6, 0.0);
  const auto opponent = makeOpponent(frame, 5.0, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.corner_side_by_side);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_FALSE(output.active_override);
}

TEST(OvertakePlannerCore, CornerSideBySideNearWallUsesSafeYieldReference)
{
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.6, 1.2);
  const auto opponent = makeOpponent(frame, 5.0, 0.4);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.corner_side_by_side);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.reason.empty());
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto minmax_offset =
    std::minmax_element(output.lateral_offsets.begin(), output.lateral_offsets.end());
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  EXPECT_GE(*minmax_offset.first, lower_d - 1.0e-9);
  EXPECT_LE(*minmax_offset.second, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.large_lateral_error_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, WallMarginRecoveryStartsInsideCorridorAndSlows)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.8);

  const auto output = core.update(0.1, ego, {});
  const double lower_d = config.d_min_m + config.min_wall_margin_m;

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.reason.empty());
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto min_offset =
    *std::min_element(output.lateral_offsets.begin(), output.lateral_offsets.end());
  EXPECT_GE(min_offset, lower_d - 1.0e-9);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.wall_margin_recovery_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, LargeLateralErrorRecoveryUsesSlowCap)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.wall_margin_recovery_v_max_mps = 2.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.large_lateral_error_v_max_mps = 1.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 1.8);

  const auto output = core.update(0.1, ego, {});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.large_lateral_error_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, WallMarginRecoveryDoesNotReleaseToFastestTooEarly)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.8);
  const auto first = core.update(0.1, ego, {});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);

  const auto second = core.update(0.2, ego, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(second.active_override);
  EXPECT_TRUE(second.reason.empty());
}

TEST(OvertakePlannerCore, SideBySideDoesNotPushPastWallMargin)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.05);
  const auto opponent = makeOpponent(frame, 5.1, -0.9);

  const auto output = core.update(0.1, ego, {opponent});
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto min_offset = *std::min_element(output.lateral_offsets.begin(), output.lateral_offsets.end());

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  EXPECT_TRUE(output.active_override);
  EXPECT_GE(min_offset, lower_d - 1.0e-9);
  EXPECT_NEAR(output.target_lateral_offset_m, lower_d, 1.0e-9);
}

TEST(OvertakePlannerCore, CenterOpponentDoesNotCreatePassAndYieldsBehind)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_NEAR(output.blocked_info.left_pass_gap_m, 1.10, 1.0e-9);
  EXPECT_NEAR(output.blocked_info.right_pass_gap_m, 1.10, 1.0e-9);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "both_gap_narrow");
  EXPECT_FALSE(output.safe_stop_triggered);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}

TEST(OvertakePlannerCore, NoPassAndFallbacksUnsafeSelectsSafeStop)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.safe_stop_v_mps = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_reason, "no_pass_and_no_safe_fallback");
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_TRUE(std::all_of(
    output.speed_caps.begin(), output.speed_caps.end(),
    [&config](double speed_cap) {
      return std::abs(speed_cap - config.safe_stop_v_mps) < 1.0e-9;
    }));
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto minmax_offset =
    std::minmax_element(output.lateral_offsets.begin(), output.lateral_offsets.end());
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  EXPECT_GE(*minmax_offset.first, lower_d - 1.0e-9);
  EXPECT_LE(*minmax_offset.second, upper_d + 1.0e-9);
}

TEST(OvertakePlannerCore, SafeStopTriggerRequiresConsecutiveUnsafeCycles)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {opponent});
  EXPECT_NE(first.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_FALSE(first.safe_stop_triggered);
  EXPECT_EQ(first.safe_stop_trigger_count, 1);

  const auto second = core.update(0.2, ego, {opponent});
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_TRUE(second.safe_stop_triggered);
  EXPECT_EQ(second.safe_stop_trigger_count, 2);
}

TEST(OvertakePlannerCore, SafeStopHoldingKeepsPublishingFeasibleOverride)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  ASSERT_TRUE(first.active_override);

  const auto second = core.update(0.2, ego, {opponent});
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(second.active_override);
  EXPECT_EQ(second.safe_stop_reason, "safe_stop_holding");
}

TEST(OvertakePlannerCore, SafeStopForwardProgressUsesSafeStopSpeed)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.safe_stop_v_mps = 0.2;
  config.side_by_side_s_m = 0.1;
  config.safety_ellipse_a_m = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.6, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(output.active_override);
  EXPECT_NE(output.safe_stop_reason, "safe_stop_infeasible");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.safe_stop_v_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, InfeasibleSafeStopDoesNotActivateOverride)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.side_by_side_s_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.2, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(output.safe_stop_reject_reason, "opponent_collision");
  EXPECT_FALSE(output.active_override);
}

TEST(OvertakePlannerCore, SafeStopDoesNotLatchWithoutActiveOverrideWhenStopBecomesUnsafe)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.side_by_side_s_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto first_opponent = makeOpponent(frame, 10.0, 0.0);
  first_opponent.vx = 0.0;
  first_opponent.v = 0.0;
  const auto first = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  ASSERT_TRUE(first.active_override);

  auto close_opponent = makeOpponent(frame, 5.2, 0.0);
  close_opponent.vx = 0.0;
  close_opponent.v = 0.0;
  const auto second = core.update(0.2, ego, {close_opponent});

  EXPECT_NE(second.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_TRUE(second.safe_stop_triggered);
  EXPECT_EQ(second.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(second.safe_stop_reject_reason, "opponent_collision");
  EXPECT_FALSE(second.active_override);
}

TEST(OvertakePlannerCore, OpponentOnRightOnlyAllowsLeftPass)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_GT(output.target_lateral_offset_m, 0.0);
}

TEST(OvertakePlannerCore, OpponentOnLeftOnlyAllowsRightPass)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_LT(output.target_lateral_offset_m, 0.0);
}

TEST(OvertakePlannerCore, FutureNarrowGapPreventsTransientLeftPass)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vy = 2.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_LT(output.blocked_info.left_pass_gap_m, config.min_pass_gap_m);
}

TEST(OvertakePlannerCore, ActiveLeftPassGapLossYieldsInsteadOfSwitchingSides)
{
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto prepare = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(prepare.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto shifted_opponent = makeOpponent(frame, 13.0, 0.6);
  const auto output = core.update(0.2, ego, {shifted_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}
