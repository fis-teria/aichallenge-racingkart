#include "overtake_planner/overtake_planner_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

overtake_planner::FrenetFrame makeStraightFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
        static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.0, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::FrenetFrame makeCurvedFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
        static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.12, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::FrenetFrame makeFutureCornerFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    const double s = static_cast<double>(i);
    const double kappa = s >= 7.0 && s <= 8.5 ? 0.12 : 0.0;
    ref.push_back(overtake_planner::ReferencePoint{s, s, 0.0, 0.0, kappa, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::FrenetFrame makeStraightGateHysteresisFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 60; ++i) {
    const double s = static_cast<double>(i);
    double kappa = 0.0;
    if (s < 10.0) {
      kappa = 0.12;
    } else if (s < 20.0) {
      kappa = 0.023;
    }
    ref.push_back(overtake_planner::ReferencePoint{s, s, 0.0, 0.0, kappa, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::EgoState makeEgo(const overtake_planner::FrenetFrame &frame,
                                   double x, double d) {
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

overtake_planner::OpponentState
makeOpponent(const overtake_planner::FrenetFrame &frame, double x, double d) {
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

overtake_planner::PlannerConfig makeConfig() {
  overtake_planner::PlannerConfig config;
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.side_by_side_s_m = 4.0;
  config.side_margin_m = 1.2;
  config.parallel_side_detection_enabled = true;
  config.parallel_side_s_m = 12.0;
  config.parallel_side_margin_m = 4.0;
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
  config.lateral_target_max_step_m = 100.0;
  return config;
}

} // namespace

TEST(OvertakePlannerCore, SideBySideOpponentOnLeftMovesRightAndOverrides) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.2, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  EXPECT_EQ(output.selected,
            overtake_planner::CandidateType::SIDE_BY_SIDE_KEEP);
  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_EQ(output.blocked_info.side_id, "npc");
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(output.lateral_offsets.back(), -0.35);
  EXPECT_LE(output.speed_caps.back(), config.side_by_side_speed_cap_mps);
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}

TEST(OvertakePlannerCore, SideBySideKeepUsesConfiguredMinimumSpeedCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.yield_min_speed_cap_mps = 2.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.2, 0.6);
  opponent.v = 0.2;
  opponent.vx = 0.2;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  EXPECT_EQ(output.selected,
            overtake_planner::CandidateType::SIDE_BY_SIDE_KEEP);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.yield_min_speed_cap_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, SideBySideOpponentAheadYieldsBehind) {
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

TEST(OvertakePlannerCore, YieldBehindUsesConfiguredMinimumSpeedCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.yield_min_speed_cap_mps = 2.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.6, 0.6);
  opponent.v = 0.2;
  opponent.vx = 0.2;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.yield_min_speed_cap_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, CornerSideBySideYieldsBehindWithCloseSpeedCap) {
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
  EXPECT_NEAR(output.target_lateral_offset_m, config.corner_yield_target_d_m,
              1.0e-9);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, CornerSideBySideLeadCarDoesNotPushLaterally) {
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

TEST(OvertakePlannerCore, CornerSideBySideNearWallUsesSafeYieldReference) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 2.0;
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
  const auto minmax_offset = std::minmax_element(output.lateral_offsets.begin(),
                                                 output.lateral_offsets.end());
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  EXPECT_GE(*minmax_offset.first, lower_d - 1.0e-9);
  EXPECT_LE(*minmax_offset.second, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     FutureCornerSideBySideNearOuterWallYieldsBeforeCurrentCorner) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.corner_side_yield_lookahead_m = 0.2;
  config.corner_yield_v_max_mps = 2.5;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.9);
  const auto opponent = makeOpponent(frame, 5.1, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.corner_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_corner_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_outer_wall_risk);
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_EQ(output.blocked_info.yield_reason, "future_outer_wall_risk");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(output.target_lateral_offset_m, ego.frenet.d);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     FutureYieldHoldSurvivesBriefSideBySideClearBeforeCorner) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.corner_side_yield_lookahead_m = 0.2;
  config.corner_yield_v_max_mps = 2.5;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto first_ego = makeEgo(frame, 5.0, 0.9);
  const auto opponent = makeOpponent(frame, 5.1, 0.0);
  const auto first = core.update(0.1, first_ego, {opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  ASSERT_TRUE(first.blocked_info.future_yield_required);

  const auto second_ego = makeEgo(frame, 6.9, 0.7);
  const auto second = core.update(0.2, second_ego, {});

  EXPECT_FALSE(second.blocked_info.side_by_side);
  EXPECT_TRUE(second.blocked_info.future_yield_required);
  EXPECT_TRUE(second.blocked_info.future_corner_side_by_side);
  EXPECT_EQ(second.blocked_info.yield_reason, "future_yield_hold");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(second.active_override);

  const auto third_ego = makeEgo(frame, 9.0, 0.2);
  const auto third = core.update(0.3, third_ego, {});

  EXPECT_FALSE(third.blocked_info.future_yield_required);
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(third.active_override);
}

TEST(OvertakePlannerCore, ParallelSideCandidateCanTriggerFutureCornerYield) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.corner_side_yield_lookahead_m = 0.2;
  config.corner_yield_v_max_mps = 2.5;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.9);
  const auto opponent = makeOpponent(frame, 5.2, -1.4);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_EQ(output.blocked_info.parallel_side_id, "npc");
  EXPECT_TRUE(output.blocked_info.future_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_corner_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_EQ(output.blocked_info.yield_reason, "future_outer_wall_risk");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
}

TEST(OvertakePlannerCore,
     ParallelSideCandidateCanUseRearAdjacentVehicleForCornerYield) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.corner_side_yield_lookahead_m = 0.2;
  config.corner_yield_v_max_mps = 2.5;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.8, 0.9);
  const auto opponent = makeOpponent(frame, 0.2, -1.4);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_LT(output.blocked_info.parallel_side_delta_s, 0.0);
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
}

TEST(OvertakePlannerCore, FutureOuterWallRiskYieldsBeforeCornerThreshold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.corner_side_yield_curvature_m_inv = 0.5;
  config.corner_yield_v_max_mps = 2.5;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.55);
  const auto opponent = makeOpponent(frame, 5.2, -1.4);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_TRUE(output.blocked_info.future_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_corner_side_by_side);
  EXPECT_TRUE(output.blocked_info.future_outer_wall_risk);
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_EQ(output.blocked_info.yield_reason, "future_outer_wall_risk");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(output.target_lateral_offset_m, ego.frenet.d);
  EXPECT_GE(output.target_lateral_offset_m, config.corner_yield_target_d_m);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, OppositeDirectionVehicleIsNotSideOrFrontTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.1, 0.6);
  opponent.vx = -1.0;
  opponent.v = 1.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_LT(output.blocked_info.side_index, 0);
  EXPECT_LT(output.blocked_info.nearest_index, 0);
  EXPECT_EQ(output.blocked_info.ignored_opposite_direction_count, 1);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(output.active_override);
}

TEST(OvertakePlannerCore, StoppedVehicleRemainsDirectionUnknownObstacle) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.vy = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_EQ(output.blocked_info.nearest_id, "npc");
  EXPECT_FALSE(output.blocked_info.front_direction_known);
  EXPECT_TRUE(output.blocked_info.front_same_direction);
  EXPECT_EQ(output.blocked_info.ignored_opposite_direction_count, 0);
}

TEST(OvertakePlannerCore, SideBySideAndFrontBlockedCoexistForSameOpponent) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 7.0, 0.4);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_EQ(output.blocked_info.nearest_id, output.blocked_info.side_id);
  EXPECT_EQ(output.blocked_info.nearest_id, "npc");
  EXPECT_NEAR(output.blocked_info.front_delta_s, 2.0, 1.0e-9);
  EXPECT_NEAR(output.blocked_info.side_delta_s, 2.0, 1.0e-9);
}

TEST(OvertakePlannerCore, SideBySideAndSeparateFrontBlockedTargetsCoexist) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto side = makeOpponent(frame, 5.2, 0.6);
  side.id = "side";
  auto front = makeOpponent(frame, 10.0, 0.0);
  front.id = "front";

  const auto output = core.update(0.1, ego, {side, front});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_EQ(output.blocked_info.nearest_id, "front");
  EXPECT_EQ(output.blocked_info.side_id, "side");
  EXPECT_NE(output.blocked_info.nearest_id, output.blocked_info.side_id);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "both_gap_narrow");
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
}

TEST(OvertakePlannerCore, ClosingFrontVehicleOutsideFollowTriggerStillBlocked) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.follow_trigger_s_m = 10.0;
  config.lookahead_s_m = 25.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 20.0, 0.0);
  opponent.vx = 1.0;
  opponent.v = 1.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_GT(output.blocked_info.front_delta_s, config.follow_trigger_s_m);
  EXPECT_GT(output.blocked_info.front_rel_v, config.dv_block_threshold_mps);
  EXPECT_EQ(output.blocked_info.nearest_id, "npc");
}

TEST(OvertakePlannerCore, RejectedSideBySideKeepPublishesSpeedOnlyFallback) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.0, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_EQ(output.reason, "opponent_collision");
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason, "speed_only_fallback_opponent_collision");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(output.lateral_offsets.begin(),
                          output.lateral_offsets.end(),
                          [](double d) { return std::abs(d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.speed_only_fallback_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, RejectedYieldBehindPublishesSpeedOnlyFallback) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 5.6, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_GT(output.blocked_info.side_delta_s, config.side_yield_s_m);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_EQ(output.reason, "opponent_collision");
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason, "speed_only_fallback_opponent_collision");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(output.lateral_offsets.begin(),
                          output.lateral_offsets.end(),
                          [](double d) { return std::abs(d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.speed_only_fallback_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, WallMarginRecoveryStartsInsideCorridorAndSlows) {
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
  const auto min_offset = *std::min_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_GE(min_offset, lower_d - 1.0e-9);
  EXPECT_GT(output.target_lateral_offset_m, lower_d + 0.5);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(),
              std::min(config.wall_margin_recovery_v_max_mps,
                       config.large_lateral_error_v_max_mps),
              1.0e-9);
}

TEST(OvertakePlannerCore, StoppedOutsideCorridorRecoveryPullsTowardCenter) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 20;
  config.horizon_dt_sec = 0.025;
  config.outside_corridor_recovery_centering_time_sec = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.8);
  ego.v = 0.0;

  const auto output = core.update(0.1, ego, {});
  const double upper_d = config.d_max_m - config.min_wall_margin_m;

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto max_offset = *std::max_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_LE(max_offset, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d - 0.25);
}

TEST(OvertakePlannerCore, StoppedRecoveryKeepsCenterPullAfterReEnteringCorridor) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 20;
  config.horizon_dt_sec = 0.025;
  config.lateral_target_max_step_m = 0.25;
  config.outside_corridor_recovery_centering_time_sec = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto outside = makeEgo(frame, 5.0, 1.8);
  outside.v = 0.0;
  const auto first = core.update(0.1, outside, {});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);

  auto near_wall_inside = makeEgo(frame, 5.0, 0.8);
  near_wall_inside.v = 0.0;
  const auto second = core.update(0.2, near_wall_inside, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(second.active_override);
  EXPECT_LE(second.target_lateral_offset_m,
            first.target_lateral_offset_m + 1.0e-9);
  EXPECT_LT(std::abs(second.target_lateral_offset_m),
            config.recovery_release_lateral_error_m);
}

TEST(OvertakePlannerCore, LargeLateralErrorRecoveryUsesSlowCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.wall_margin_recovery_v_max_mps = 2.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.large_lateral_error_v_max_mps = 1.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 1.8);

  const auto output = core.update(0.1, ego, {});
  const double upper_d = config.d_max_m - config.min_wall_margin_m;

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto max_offset = *std::max_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_LE(max_offset, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d - 0.5);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.large_lateral_error_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, LargeLateralErrorFreezesPassDecisionToRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 0.6;
  config.large_lateral_error_v_max_mps = 1.8;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  const auto opponent = makeOpponent(frame, 8.0, 0.2);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "large_lateral_error");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(std::abs(output.target_lateral_offset_m), std::abs(ego.frenet.d));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.large_lateral_error_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore,
     LargeLateralErrorWithParallelSideCandidateFreezesToRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.future_side_prediction_enabled = false;
  config.large_lateral_error_threshold_m = 0.6;
  config.large_lateral_error_v_max_mps = 1.8;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  const auto opponent = makeOpponent(frame, 11.0, -0.25);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "large_lateral_error");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
}

TEST(OvertakePlannerCore, LargeLateralErrorPreservesExistingPassGapReason) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  const auto opponent = makeOpponent(frame, 8.0, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "both_gap_narrow");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
}

TEST(OvertakePlannerCore, WallMarginRecoveryDoesNotReleaseToFastestTooEarly) {
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

TEST(OvertakePlannerCore, WallRiskInsideCorridorPublishesSpeedGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.wall_soft_margin_m = 0.25;
  config.wall_risk_v_max_mps = 4.2;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  const auto ego = makeEgo(frame, 5.0, upper_d - 0.05);

  const auto output = core.update(0.1, ego, {});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_TRUE(output.active_override);
  EXPECT_FALSE(output.speed_only_fallback_active);
  EXPECT_TRUE(output.wall_risk_speed_guard_active);
  EXPECT_EQ(output.reason, "wall_risk_speed_guard");
  EXPECT_EQ(output.speed_cap_reason, "wall_risk_speed_guard");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(
      output.lateral_offsets.begin(), output.lateral_offsets.end(),
      [ego](double d) { return std::abs(d - ego.frenet.d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.wall_risk_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, LateralTargetRateLimitClampsPublishedOverrideStep) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lateral_target_max_step_m = 0.20;
  config.large_lateral_error_threshold_m = 2.0;
  config.wall_soft_margin_m = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent_on_left = makeOpponent(frame, 5.0, 0.6);
  const auto first = core.update(0.1, ego, {opponent_on_left});
  ASSERT_TRUE(first.active_override);
  ASSERT_FALSE(first.lateral_offsets.empty());
  ASSERT_LT(first.target_lateral_offset_m, -0.3);

  const auto opponent_on_right = makeOpponent(frame, 5.0, -0.6);
  const auto second = core.update(0.2, ego, {opponent_on_right});

  ASSERT_TRUE(second.active_override);
  ASSERT_FALSE(second.lateral_offsets.empty());
  EXPECT_LE(second.target_lateral_offset_m - first.target_lateral_offset_m,
            config.lateral_target_max_step_m + 1.0e-9);
  EXPECT_GT(second.target_lateral_offset_m, first.target_lateral_offset_m);
}

TEST(OvertakePlannerCore, SafeStopLateralTargetIsRateLimited) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lateral_target_max_step_m = 0.20;
  config.safe_stop_trigger_cycles = 1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent_on_left = makeOpponent(frame, 5.0, 0.6);
  const auto first = core.update(0.1, ego, {opponent_on_left});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  ASSERT_TRUE(first.active_override);
  ASSERT_LT(first.target_lateral_offset_m, -0.3);

  auto stopped_front = makeOpponent(frame, 10.0, 0.0);
  stopped_front.vx = 0.0;
  stopped_front.v = 0.0;
  const auto second = core.update(0.2, ego, {stopped_front});

  ASSERT_EQ(second.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  ASSERT_TRUE(second.active_override);
  EXPECT_LE(second.target_lateral_offset_m - first.target_lateral_offset_m,
            config.lateral_target_max_step_m + 1.0e-9);
  EXPECT_LT(second.target_lateral_offset_m, -0.05);
}

TEST(OvertakePlannerCore, LateralTargetRateLimitExpiresAfterPublishGap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lateral_target_max_step_m = 0.20;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent_on_left = makeOpponent(frame, 5.0, 0.6);
  const auto first = core.update(0.1, ego, {opponent_on_left});
  ASSERT_TRUE(first.active_override);
  ASSERT_LT(first.target_lateral_offset_m, -0.3);

  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  const auto near_right_wall = makeEgo(frame, 5.0, upper_d - 0.05);
  const auto second = core.update(0.8, near_right_wall, {});

  ASSERT_TRUE(second.active_override);
  EXPECT_GT(second.target_lateral_offset_m - first.target_lateral_offset_m,
            config.lateral_target_max_step_m);
}

TEST(OvertakePlannerCore, HighSpeedCurveLateralHoldKeepsGuardTarget) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.high_speed_curve_lateral_hold_enabled = true;
  config.high_speed_curve_lateral_hold_min_speed_mps = 4.0;
  config.high_speed_curve_lateral_hold_release_speed_mps = 2.5;
  config.large_lateral_error_threshold_m = 2.0;
  config.wall_soft_margin_m = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto right_wall = makeEgo(frame, 5.0, 0.80);
  right_wall.v = 5.0;
  const auto first = core.update(0.1, right_wall, {});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  ASSERT_TRUE(first.active_override);
  ASSERT_TRUE(first.lateral_target_hold_active);

  auto left_wall = makeEgo(frame, 5.0, -0.80);
  left_wall.v = 5.0;
  const auto second = core.update(0.2, left_wall, {});

  ASSERT_EQ(second.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  ASSERT_TRUE(second.lateral_target_hold_active);
  EXPECT_EQ(second.lateral_target_hold_reason, "high_speed_curve_hold");
  EXPECT_NEAR(second.target_lateral_offset_m, first.target_lateral_offset_m,
              1.0e-9);

  left_wall.v = 2.0;
  const auto third = core.update(0.3, left_wall, {});
  EXPECT_FALSE(third.lateral_target_hold_active);
}

TEST(OvertakePlannerCore, MultipleSpeedGuardsUseLowestCapReason) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.wall_soft_margin_m = 0.25;
  config.wall_risk_v_max_mps = 4.2;
  config.mpc_health_v_max_mps = 2.7;
  config.mpc_health_infeasible_count_threshold = 1;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  const auto ego = makeEgo(frame, 5.0, upper_d - 0.05);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 1;
  health.solve_time_ms = 5.0;
  health.age_sec = 0.1;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.wall_risk_speed_guard_active);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.reason, "mpc_health_infeasible_guard");
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_infeasible_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, MpcHealthInfeasiblePublishesSpeedGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_v_max_mps = 2.7;
  config.mpc_health_infeasible_count_threshold = 1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 1;
  health.solve_time_ms = 5.0;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_infeasible_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
  EXPECT_TRUE(output.mpc_health.valid);
  EXPECT_EQ(output.mpc_health.infeasible_count, 1);
}

TEST(OvertakePlannerCore, MpcHealthSolveTimePublishesSpeedGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_v_max_mps = 2.6;
  config.mpc_health_solve_time_warn_ms = 50.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 0;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.reason, "mpc_health_solve_time_guard");
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_solve_time_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
  EXPECT_TRUE(output.mpc_health.valid);
  EXPECT_NEAR(output.mpc_health.solve_time_ms, health.solve_time_ms, 1.0e-9);
}

TEST(OvertakePlannerCore, MpcHealthSolveTimePreservesFeasibleLateralOverride) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_v_max_mps = 2.6;
  config.mpc_health_solve_time_warn_ms = 50.0;
  config.wall_risk_speed_guard_enabled = false;
  config.large_lateral_error_v_max_mps = 8.5;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 1.8);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 0;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.reason.empty());
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_solve_time_guard");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_FALSE(std::all_of(output.lateral_offsets.begin(),
                           output.lateral_offsets.end(),
                           [](double d) { return std::abs(d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, RecoverySpeedGuardCanCapMpcHealthRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.wall_risk_speed_guard_enabled = false;
  config.large_lateral_error_v_max_mps = 8.5;
  config.mpc_health_v_max_mps = 8.0;
  config.mpc_health_solve_time_warn_ms = 50.0;
  config.recovery_speed_guard_v_max_mps = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 1.8);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 0;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_TRUE(output.recovery_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "recovery_mpc_health_speed_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.recovery_speed_guard_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, MpcHealthReasonPrefersInfeasibleOverOtherFaults) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_v_max_mps = 2.7;
  config.mpc_health_infeasible_count_threshold = 1;
  config.mpc_health_solve_time_warn_ms = 50.0;
  config.mpc_health_stale_time_sec = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 1;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.8;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.reason, "mpc_health_infeasible_guard");
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_infeasible_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, MpcHealthStalePublishesSpeedGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_v_max_mps = 2.4;
  config.mpc_health_stale_time_sec = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 0;
  health.solve_time_ms = 5.0;
  health.age_sec = 0.8;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_stale_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.mpc_health_v_max_mps, 1.0e-9);
  EXPECT_TRUE(output.mpc_health.valid);
  EXPECT_NEAR(output.mpc_health.age_sec, health.age_sec, 1.0e-9);
}

TEST(OvertakePlannerCore, StrictSectionOuterYieldScalesSpeedGuard) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.corner_side_yield_lookahead_m = 0.2;
  config.corner_yield_v_max_mps = 2.5;
  config.wall_soft_margin_m = 0.25;
  config.future_side_prediction_enabled = false;
  config.large_lateral_error_threshold_m = 2.0;
  config.section_safety_rules.push_back(overtake_planner::SectionSafetyRule{
      "first_corner", 4.0, 8.0, "side_by_side_corner_strict", "outer_yields"});
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.65);
  const auto opponent = makeOpponent(frame, 5.1, 0.0);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.active_section.active);
  EXPECT_EQ(output.active_section.name, "first_corner");
  EXPECT_EQ(output.active_section.profile, "side_by_side_corner_strict");
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_EQ(output.blocked_info.yield_reason, "section_outer_yield");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_TRUE(output.active_override);
  EXPECT_EQ(output.speed_cap_reason, "section_profile_speed_guard");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_FALSE(std::all_of(output.lateral_offsets.begin(),
                           output.lateral_offsets.end(),
                           [](double d) { return std::abs(d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.corner_yield_v_max_mps * 0.65,
              1.0e-9);
}

TEST(OvertakePlannerCore, UnsafeSideBySideKeepFallsBackToFeasibleRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.05);
  const auto opponent = makeOpponent(frame, 5.1, -0.3);

  const auto output = core.update(0.1, ego, {opponent});
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto min_offset = *std::min_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.reason.empty());
  EXPECT_TRUE(output.active_override);
  EXPECT_GE(min_offset, lower_d - 1.0e-9);
  EXPECT_GT(output.target_lateral_offset_m, lower_d);
}

TEST(OvertakePlannerCore, CenterOpponentDoesNotCreatePassAndYieldsBehind) {
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
  EXPECT_NEAR(output.blocked_info.left_pass_gap_m,
              config.d_max_m - config.min_wall_margin_m, 1.0e-9);
  EXPECT_NEAR(output.blocked_info.right_pass_gap_m,
              -config.d_min_m - config.min_wall_margin_m, 1.0e-9);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "both_gap_narrow");
  EXPECT_FALSE(output.safe_stop_triggered);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.speed_only_fallback_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, StartGraceSuppressesSafeStopForParallelStartRisk) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 8.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.speed_only_fallback_v_max_mps = 8.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 0.3;
  const auto opponent = makeOpponent(frame, 5.2, -0.60);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.start_grace_active);
  EXPECT_FALSE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_trigger_count, 0);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.reason, "start_grace_safe_stop_suppressed");
  EXPECT_TRUE(output.speed_only_fallback_active);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), config.safe_stop_v_mps);
}

TEST(OvertakePlannerCore, StartGraceSuppressesSafeStopForSideBySideStartRisk) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 8.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.speed_only_fallback_v_max_mps = 8.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 0.3;
  const auto opponent = makeOpponent(frame, 5.2, 0.15);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.start_grace_active);
  EXPECT_FALSE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_trigger_count, 0);
  EXPECT_EQ(output.reason, "start_grace_safe_stop_suppressed");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), config.safe_stop_v_mps);
}

TEST(OvertakePlannerCore, StartGraceDoesNotSuppressSafeStopAboveSpeedLimit) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 8.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 2.0;
  const auto opponent = makeOpponent(frame, 5.2, -0.60);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.start_grace_active);
  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(output.safe_stop_reject_reason, "opponent_collision");
}

TEST(OvertakePlannerCore, StartGraceExpiresAndAllowsSafeStopDiagnostics) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 1.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 0.3;
  const auto opponent = makeOpponent(frame, 5.2, -0.60);

  const auto first = core.update(0.1, ego, {opponent});
  ASSERT_TRUE(first.start_grace_active);
  ASSERT_FALSE(first.safe_stop_triggered);
  ASSERT_EQ(first.reason, "start_grace_safe_stop_suppressed");

  const auto second = core.update(1.2, ego, {opponent});

  EXPECT_FALSE(second.start_grace_active);
  EXPECT_TRUE(second.safe_stop_triggered);
  EXPECT_EQ(second.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(second.safe_stop_reject_reason, "opponent_collision");
}

TEST(OvertakePlannerCore, StartGraceDoesNotRearmAfterInvalidEgo) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 1.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 0.3;
  auto invalid_ego = ego;
  invalid_ego.valid = false;
  const auto opponent = makeOpponent(frame, 5.2, -0.60);

  const auto first = core.update(0.1, ego, {opponent});
  ASSERT_TRUE(first.start_grace_active);
  ASSERT_FALSE(first.safe_stop_triggered);

  const auto invalid = core.update(1.2, invalid_ego, {opponent});
  ASSERT_EQ(invalid.reason, "disabled_or_invalid");

  const auto second = core.update(1.3, ego, {opponent});

  EXPECT_FALSE(second.start_grace_active);
  EXPECT_TRUE(second.safe_stop_triggered);
  EXPECT_EQ(second.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(second.safe_stop_reject_reason, "opponent_collision");
}

TEST(OvertakePlannerCore, NoPassAndFallbacksUnsafeSelectsSafeStop) {
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
  EXPECT_TRUE(std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                          [&config](double speed_cap) {
                            return std::abs(speed_cap -
                                            config.safe_stop_v_mps) < 1.0e-9;
                          }));
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto minmax_offset = std::minmax_element(output.lateral_offsets.begin(),
                                                 output.lateral_offsets.end());
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  EXPECT_GE(*minmax_offset.first, lower_d - 1.0e-9);
  EXPECT_LE(*minmax_offset.second, upper_d + 1.0e-9);
}

TEST(OvertakePlannerCore, SafeStopTriggerRequiresConsecutiveUnsafeCycles) {
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

TEST(OvertakePlannerCore, SafeStopHoldingKeepsPublishingFeasibleOverride) {
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

TEST(OvertakePlannerCore, SafeStopReleaseHandsOffToRecoveryUntilCentered) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.min_mode_hold_time_sec = 0.0;
  config.large_lateral_error_v_max_mps = 8.5;
  config.wall_risk_v_max_mps = 8.0;
  config.recovery_speed_guard_v_max_mps = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;
  const auto first = core.update(0.1, ego, {opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SAFE_STOP);

  auto near_wall = makeEgo(frame, 5.0, 0.75);
  near_wall.v = 0.2;
  const auto second = core.update(0.2, near_wall, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(second.active_override);
  EXPECT_TRUE(second.wall_risk_speed_guard_active);
  EXPECT_TRUE(second.recovery_speed_guard_active);
  EXPECT_EQ(second.speed_cap_reason, "recovery_wall_risk_speed_guard");
  EXPECT_LT(std::abs(second.target_lateral_offset_m),
            std::abs(near_wall.frenet.d));
  ASSERT_FALSE(second.speed_caps.empty());
  EXPECT_NEAR(second.speed_caps.back(), config.recovery_speed_guard_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore, SafeStopReleaseRequiresCenteredEgoBeforeFreeRun) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.safe_stop_release_cycles = 2;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;
  const auto first = core.update(0.1, ego, {opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SAFE_STOP);

  auto centered = makeEgo(frame, 5.0, 0.2);
  centered.v = 0.1;
  const auto second = core.update(0.2, centered, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_TRUE(second.safe_stop_release_ready);
  EXPECT_EQ(second.safe_stop_release_count, 1);
  EXPECT_EQ(second.safe_stop_reason, "safe_stop_release_pending");

  const auto third = core.update(0.3, centered, {});

  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(third.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_TRUE(third.safe_stop_release_ready);
  EXPECT_FALSE(third.active_override);
}

TEST(OvertakePlannerCore, SafeStopForwardProgressUsesSafeStopSpeed) {
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

TEST(OvertakePlannerCore, InfeasibleSafeStopPublishesSpeedOnlyFallback) {
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
  EXPECT_EQ(output.reason, "safe_stop_infeasible");
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason,
            "speed_only_fallback_safe_stop_infeasible");
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(output.lateral_offsets.begin(),
                          output.lateral_offsets.end(),
                          [](double d) { return std::abs(d) < 1.0e-9; }));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), 0.0);
  EXPECT_LE(output.speed_caps.back(), config.speed_only_fallback_v_max_mps);
}

TEST(OvertakePlannerCore,
     SafeStopDoesNotLatchButPublishesSpeedGuardWhenStopBecomesUnsafe) {
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
  EXPECT_TRUE(second.active_override);
  EXPECT_TRUE(second.speed_only_fallback_active);
  EXPECT_EQ(second.speed_cap_reason,
            "speed_only_fallback_safe_stop_infeasible");
}

TEST(OvertakePlannerCore, OpponentOnRightOnlyAllowsLeftPass) {
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
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "right_gap_narrow");
  EXPECT_TRUE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_TRUE(output.blocked_info.overtake_start_gate_reason.empty());
  EXPECT_GT(output.target_lateral_offset_m, 0.0);
}

TEST(OvertakePlannerCore, OvertakeOnlyPublishModeHoldsFollowDuringPrepare) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.pass_safe_required_cycles = 2.0;
  config.pass_horizon_publish_mode = "overtake_only";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto first = core.update(0.1, ego, {opponent});
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(first.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(first.blocked_info.can_pass_left);

  const auto prepare = core.update(0.2, ego, {opponent});
  EXPECT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(prepare.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_NEAR(prepare.target_lateral_offset_m, 0.0, 1.0e-9);
  EXPECT_TRUE(prepare.active_override);

  const auto overtake = core.update(0.3, ego, {opponent});
  EXPECT_EQ(overtake.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(overtake.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_GT(overtake.target_lateral_offset_m, 0.0);
}

TEST(OvertakePlannerCore, CurvedRoadStraightOnlyGatePreventsPassStart) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_GT(output.blocked_info.overtake_start_abs_curvature,
            config.straight_overtake_max_curvature_m_inv);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(output.active_override);
}

TEST(OvertakePlannerCore, OvertakePermissionDisallowedSectionKeepsFollow) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = false;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0, false});
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.overtake_permission_allowed);
  EXPECT_EQ(output.blocked_info.overtake_permission_section_name,
            "slow_corner");
  EXPECT_EQ(output.blocked_info.overtake_permission_reason,
            "section_disallowed");
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason,
            "section_disallowed");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakePlannerCore, SlowFrontExceptionAllowsPassInDisallowedSection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0, false});
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {stopped_opponent});
  EXPECT_FALSE(first.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto second = core.update(0.2, ego, {stopped_opponent});
  EXPECT_TRUE(second.blocked_info.front_vehicle_low_speed);
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_EQ(second.blocked_info.slow_front_exception_count, 2);
  EXPECT_FALSE(second.blocked_info.overtake_permission_allowed);
  EXPECT_EQ(second.blocked_info.overtake_permission_reason,
            "slow_front_exception");
  EXPECT_TRUE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_TRUE(second.blocked_info.overtake_start_gate_reason.empty());
  EXPECT_EQ(second.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore, OvertakePermissionLookaheadPreventsPassStart) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 8.0;
  config.slow_front_exception_enabled = false;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"next_slow_corner", 12.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.overtake_permission_allowed);
  EXPECT_EQ(output.blocked_info.overtake_permission_section_name,
            "next_slow_corner");
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason,
            "section_disallowed");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(OvertakePlannerCore, StraightOnlyGateReopensAfterHysteresisClears) {
  const auto frame = makeStraightGateHysteresisFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_release_hysteresis_m_inv = 0.005;
  config.straight_overtake_lookahead_m = 0.1;
  config.keep_mode_bonus = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto first = core.update(0.1, makeEgo(frame, 5.0, 0.0),
                                 {makeOpponent(frame, 13.0, -0.6)});
  ASSERT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto still_closed = core.update(0.2, makeEgo(frame, 12.0, 0.0),
                                        {makeOpponent(frame, 20.0, -0.6)});
  EXPECT_FALSE(still_closed.blocked_info.straight_overtake_start_allowed);
  EXPECT_GT(still_closed.blocked_info.overtake_start_abs_curvature,
            config.straight_overtake_max_curvature_m_inv -
                config.straight_overtake_release_hysteresis_m_inv);
  EXPECT_EQ(still_closed.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto reopened = core.update(0.3, makeEgo(frame, 22.0, 0.0),
                                    {makeOpponent(frame, 30.0, -0.6)});
  EXPECT_TRUE(reopened.blocked_info.straight_overtake_start_allowed);
  EXPECT_TRUE(reopened.blocked_info.overtake_start_gate_reason.empty());
  EXPECT_EQ(reopened.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(reopened.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore, OpponentOnLeftOnlyAllowsRightPass) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, 0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "left_gap_narrow");
  EXPECT_LT(output.target_lateral_offset_m, 0.0);
}

TEST(OvertakePlannerCore, WidePassGapsReasonIsOk) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "ok");
}

TEST(OvertakePlannerCore, LocalizedLatchedPassKeepsNearEgoOffsetsStable) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.maneuver_latch_min_hold_sec = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_EQ(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(output.maneuver_latch_active);
  EXPECT_EQ(output.maneuver_latch_target_id, "npc");
  EXPECT_EQ(output.lateral_profile_mode, "localized_latched");
  EXPECT_NEAR(output.maneuver_latch_target_s_m, 13.0, 1.0e-9);
  EXPECT_NEAR(output.maneuver_latch_avoid_start_s_m, 7.0, 1.0e-9);
  EXPECT_NEAR(output.lateral_offsets.front(), ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(output.lateral_offsets[1], ego.frenet.d, 1.0e-9);
  EXPECT_GT(output.lateral_offsets.back(), 0.0);
  EXPECT_LT(output.lateral_offsets.back(), config.left_offset_m);
}

TEST(OvertakePlannerCore, LegacyPassStillStartsShiftingFromEgo) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "legacy";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto output = core.update(0.1, ego, {opponent});

  ASSERT_GT(output.lateral_offsets.size(), 1U);
  EXPECT_EQ(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_FALSE(output.maneuver_latch_active);
  EXPECT_EQ(output.lateral_profile_mode, "legacy");
  EXPECT_GT(output.lateral_offsets[1], ego.frenet.d);
}

TEST(OvertakePlannerCore, FutureNarrowGapPreventsTransientLeftPass) {
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

TEST(OvertakePlannerCore, NoTargetPassGapReasonIsNoTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);

  const auto output = core.update(0.1, ego, {});

  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.parallel_side_candidate);
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "no_target");
}

TEST(OvertakePlannerCore, ActiveLeftPassGapLossYieldsInsteadOfSwitchingSides) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto prepare = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto shifted_opponent = makeOpponent(frame, 13.0, 0.6);
  const auto output = core.update(0.2, ego, {shifted_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}

TEST(OvertakePlannerCore, ActiveRightPassGapLossYieldsInsteadOfSwitchingSides) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, 0.6);
  const auto prepare = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  const auto shifted_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto output = core.update(0.2, ego, {shifted_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_NEAR(output.speed_caps.back(), 3.4, 1.0e-9);
}
