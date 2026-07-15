#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/overtake_planner_core.hpp"
#include "overtake_planner/reference_override_contract.hpp"

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

overtake_planner::FrenetFrame makeGentleCurvedFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
        static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.035, 5.0});
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
  config.pass_gap_hysteresis_m = 0.25;
  // 既存fixtureは静的gapの回帰確認用。動的候補評価は専用fixtureで明示的に有効化する。
  config.dynamic_pass_candidate_enabled = false;
  // 既存fixtureは個別の追い越し/停止判定を対象にする。復帰ゲート専用fixtureだけを
  // 有効化して、入力鮮度と複数車両の条件を明示する。
  config.reentry_gate_enabled = false;
  config.max_brake_decel_mps2 = 1.5;
  config.longitudinal_response_delay_sec = 0.0;
  config.yield_speed_margin_mps = 0.6;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  config.lateral_target_max_step_m = 100.0;
  return config;
}

overtake_planner::ReentryInputStatus readyReentryInput() {
  overtake_planner::ReentryInputStatus status;
  status.ego_fresh = true;
  status.v2x_snapshot_fresh = true;
  status.all_observed_opponents_fresh = true;
  status.all_observed_opponents_included = true;
  status.reference_valid = true;
  status.mpc_healthy = true;
  return status;
}

void expectSpeedOnlyV2Contract(const overtake_planner::PlannerOutput &output,
                               double max_speed_cap_mps) {
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_TRUE(output.lateral_offsets.empty());
  ASSERT_FALSE(output.speed_caps.empty());
  for (const double speed_cap_mps : output.speed_caps) {
    EXPECT_TRUE(std::isfinite(speed_cap_mps));
    EXPECT_GT(speed_cap_mps, 0.0);
    EXPECT_LE(speed_cap_mps, max_speed_cap_mps + 1.0e-9);
  }
  ASSERT_TRUE(std::isfinite(output.applied_speed_cap_mps));
  EXPECT_GT(output.applied_speed_cap_mps, 0.0);
  EXPECT_LE(output.applied_speed_cap_mps, max_speed_cap_mps + 1.0e-9);

  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(output, 17U);
  EXPECT_EQ(wire.kind,
            overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2);
  ASSERT_EQ(wire.data.size(), 6U);
  EXPECT_FLOAT_EQ(wire.data[0], 1.0F);
  EXPECT_FLOAT_EQ(wire.data[2], 0.0F);
  EXPECT_FLOAT_EQ(wire.data[3], 2.0F);
  EXPECT_FLOAT_EQ(wire.data[4], 17.0F);
  EXPECT_FLOAT_EQ(wire.data[5],
                  static_cast<float>(output.applied_speed_cap_mps));
}

} // namespace

TEST(ReferenceOverrideContract, BuildsV3LateralPayloadAndV2SpeedOnly) {
  overtake_planner::PlannerOutput lateral;
  lateral.mode = overtake_planner::BehaviorMode::SPEED_GUARD;
  lateral.active_override = true;
  lateral.longitudinal_speed_cap_active = true;
  lateral.lateral_offsets = {0.4, 0.5};
  lateral.speed_caps = {0.8, 0.7};
  const auto v3 =
      overtake_planner::makeReferenceOverrideWirePayload(lateral, 42U);

  EXPECT_EQ(v3.kind,
            overtake_planner::ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3);
  EXPECT_EQ(v3.data, (std::vector<float>{1.0F, 11.0F, 2.0F, 0.4F, 0.5F, 0.8F,
                                         0.7F, 3.0F, 42.0F, 0.0F}));

  overtake_planner::PlannerOutput speed_only;
  speed_only.mode = overtake_planner::BehaviorMode::SPEED_GUARD;
  speed_only.longitudinal_speed_cap_active = true;
  speed_only.speed_caps = {0.5, 0.5};
  speed_only.applied_speed_cap_mps = 0.5;
  const auto v2 =
      overtake_planner::makeReferenceOverrideWirePayload(speed_only, 43U);

  EXPECT_EQ(v2.kind,
            overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2);
  EXPECT_EQ(v2.data,
            (std::vector<float>{1.0F, 11.0F, 0.0F, 2.0F, 43.0F, 0.5F}));
}

TEST(ReferenceOverrideContract, V3IntentChangesSemanticGeneration) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::ABORT_RECOVERY;
  output.active_override = true;
  output.lateral_offsets = {0.4, 0.5};
  output.speed_caps = {10.0, 10.0};

  const auto normal =
      overtake_planner::makeReferenceOverrideWirePayload(output, 10U);
  output.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  const auto mandatory =
      overtake_planner::makeReferenceOverrideWirePayload(output, 11U);

  EXPECT_FALSE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      normal, mandatory));
  EXPECT_EQ(mandatory.data.back(), 2.0F);
}

TEST(ReferenceOverrideContract, SemanticComparisonIgnoresGeneration) {
  overtake_planner::PlannerOutput speed_only;
  speed_only.mode = overtake_planner::BehaviorMode::SPEED_GUARD;
  speed_only.longitudinal_speed_cap_active = true;
  speed_only.speed_caps = {0.5, 0.5};
  speed_only.applied_speed_cap_mps = 0.5;

  const auto generation_one =
      overtake_planner::makeReferenceOverrideWirePayload(speed_only, 1U);
  const auto generation_two =
      overtake_planner::makeReferenceOverrideWirePayload(speed_only, 2U);
  EXPECT_TRUE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      generation_one, generation_two));

  speed_only.applied_speed_cap_mps = 0.4;
  const auto changed =
      overtake_planner::makeReferenceOverrideWirePayload(speed_only, 2U);
  EXPECT_FALSE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      generation_two, changed));
}

TEST(ReferenceOverrideContract, InvalidActiveOutputFailsClosedToExplicitClear) {
  overtake_planner::PlannerOutput invalid;
  invalid.mode = overtake_planner::BehaviorMode::SPEED_GUARD;
  invalid.active_override = true;
  invalid.lateral_offsets = {0.1};
  const auto clear =
      overtake_planner::makeReferenceOverrideWirePayload(invalid, 7U);

  EXPECT_EQ(clear.kind, overtake_planner::ReferenceOverrideWireKind::INACTIVE);
  EXPECT_EQ(clear.data, (std::vector<float>{1.0F, 0.0F, 0.0F, 1.0F, 7.0F}));
}

TEST(OvertakePlannerCore, PublicBehaviorModeIdsStayStable) {
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::FREE_RUN), 0);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::FOLLOW_BLOCKED),
            1);
  EXPECT_EQ(
      static_cast<int>(overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT),
      2);
  EXPECT_EQ(
      static_cast<int>(overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT),
      3);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::OVERTAKE_LEFT), 4);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::OVERTAKE_RIGHT),
            5);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::MERGE_BACK), 6);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::ABORT_RECOVERY),
            7);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP),
            8);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::YIELD_BEHIND), 9);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::SAFE_STOP), 10);
  EXPECT_EQ(static_cast<int>(overtake_planner::BehaviorMode::SPEED_GUARD), 11);
}

TEST(OvertakePlannerCore,
     BrakingProfileKeepsDelayedSafetyPredictionAndIssuesImmediateSpeedCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 4;
  config.horizon_dt_sec = 0.1;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.2;
  config.safe_stop_v_mps = 0.2;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto candidate =
      builder.makeCandidate(overtake_planner::CandidateType::SAFE_STOP, ego,
                            overtake_planner::BlockedInfo{}, {});

  ASSERT_EQ(candidate.s.size(), 4U);
  ASSERT_EQ(candidate.predicted_speed_mps.size(), 4U);
  ASSERT_EQ(candidate.v_ref.size(), 4U);
  EXPECT_TRUE(candidate.longitudinal_profile_valid);
  EXPECT_NEAR(candidate.s[0], ego.frenet.s, 1.0e-9);
  // 0.2 secの応答遅れ中は、速度cap 0.2 m/sではなく現在速度4 m/sで進む。
  EXPECT_NEAR(candidate.s[1] - ego.frenet.s, 0.4, 1.0e-9);
  EXPECT_NEAR(candidate.s[2] - ego.frenet.s, 0.8, 1.0e-9);
  EXPECT_NEAR(candidate.s[3] - ego.frenet.s, 1.195, 1.0e-9);
  // 予測速度はs(t)と同じ遅れ・制動モデルを保持する。
  EXPECT_NEAR(candidate.predicted_speed_mps[0], 4.0, 1.0e-9);
  EXPECT_NEAR(candidate.predicted_speed_mps[1], 4.0, 1.0e-9);
  EXPECT_NEAR(candidate.predicted_speed_mps[2], 4.0, 1.0e-9);
  EXPECT_NEAR(candidate.predicted_speed_mps[3], 3.9, 1.0e-9);
  // 下流は各周期v_ref[0]を即時に読む。応答遅れをここへ入れると、毎周期の
  // publishで減速要求が再び先送りされてしまうため、目標capを直ちに出す。
  for (const double speed_cap : candidate.v_ref) {
    EXPECT_NEAR(speed_cap, 0.2, 1.0e-9);
  }
}

TEST(OvertakePlannerCore,
     ActualSafetyCorridorRejectsBothPassesForCenteredStationaryObstacle) {
  const auto frame = makeStraightFrame();
  overtake_planner::PlannerConfig config;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 11.0, 0.0);
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped});

  EXPECT_TRUE(output.blocked_info.stationary_front_obstacle);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     DynamicPassCandidateCanStartOnlyWhenPhysicalCorridorIsSafe) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.min_wall_margin_m = 0.5;
  config.left_offset_m = 1.4;
  config.right_offset_m = -1.4;
  config.prepare_distance_m = 1.0;
  config.min_pass_gap_m = 1.8;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.9;
  config.min_ellipse_h = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 9.0, 0.0);
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped});

  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_TRUE(output.active_override);
}

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

TEST(OvertakePlannerCore,
     SideBySideKeepIssuesConfiguredMinimumSpeedCapImmediately) {
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
  EXPECT_NEAR(output.speed_caps.front(), config.yield_min_speed_cap_mps,
              1.0e-9);
  EXPECT_NEAR(output.speed_caps.back(), config.yield_min_speed_cap_mps, 1.0e-9);
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

TEST(OvertakePlannerCore,
     YieldBehindIssuesConfiguredMinimumSpeedCapImmediately) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.yield_min_speed_cap_mps = 2.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 5.6, 0.6);
  opponent.v = 2.0;
  opponent.vx = 2.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.front(), config.yield_min_speed_cap_mps,
              1.0e-9);
  EXPECT_NEAR(output.speed_caps.back(), config.yield_min_speed_cap_mps, 1.0e-9);
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

TEST(OvertakePlannerCore,
     ParallelObservationAloneDoesNotTriggerFutureCornerYield) {
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
  EXPECT_FALSE(output.blocked_info.future_parallel_interaction);
  EXPECT_FALSE(output.blocked_info.future_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_corner_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_yield_required);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(OvertakePlannerCore, StrictParallelApproachYieldsWithoutLateralCrossing) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.yield_min_speed_cap_mps = 0.5;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.8);
  auto opponent = makeOpponent(frame, 6.0, -0.8);
  opponent.vx = 2.0;
  opponent.v = 2.0;

  const auto output = core.update(0.1, ego, {opponent});

  ASSERT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_TRUE(output.blocked_info.parallel_yield_hold_lateral);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  ASSERT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-9);
  for (const double lateral_offset_m : output.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(),
              opponent.v - config.yield_speed_margin_mps, 1.0e-9);
  EXPECT_GT(output.min_cbf_h, config.min_ellipse_h);
}

TEST(OvertakePlannerCore,
     RearOrNonClosingParallelObservationDoesNotTriggerEarlyYield) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.8);
  auto opponent = makeOpponent(frame, 4.0, -0.8);
  opponent.vx = 2.0;
  opponent.v = 2.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.parallel_yield_hold_lateral);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(OvertakePlannerCore,
     RearParallelObservationDoesNotTriggerFutureCornerYield) {
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
  EXPECT_FALSE(output.blocked_info.future_parallel_interaction);
  EXPECT_FALSE(output.blocked_info.future_yield_required);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
}

TEST(OvertakePlannerCore,
     ParallelObservationDoesNotYieldForOuterWallWithoutCornerInteraction) {
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
  EXPECT_FALSE(output.blocked_info.future_parallel_interaction);
  EXPECT_FALSE(output.blocked_info.future_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_corner_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_outer_wall_risk);
  EXPECT_FALSE(output.blocked_info.future_yield_required);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
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
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason, "speed_only_fallback_opponent_collision");
  expectSpeedOnlyV2Contract(output,
                            config.opponent_collision_fallback_v_max_mps);
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
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason, "speed_only_fallback_opponent_collision");
  expectSpeedOnlyV2Contract(output,
                            config.opponent_collision_fallback_v_max_mps);
}

TEST(OvertakePlannerCore, WallMarginRecoveryStartsInsideCorridorAndSlows) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.8);

  const auto output = core.update(0.1, ego, {});
  const double lower_d = config.d_min_m + config.min_wall_margin_m;

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto max_offset = *std::max_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_LE(max_offset, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d - 0.25);
}

TEST(OvertakePlannerCore,
     StoppedRecoveryKeepsCenterPullAfterReEnteringCorridor) {
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
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SPEED_GUARD);

  auto near_wall_inside = makeEgo(frame, 5.0, 0.8);
  near_wall_inside.v = 0.0;
  const auto second = core.update(0.2, near_wall_inside, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto max_offset = *std::max_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_LE(max_offset, upper_d + 1.0e-9);
  EXPECT_LT(output.target_lateral_offset_m, upper_d - 0.5);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.front(), config.large_lateral_error_v_max_mps,
              1.0e-9);
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
  EXPECT_TRUE(output.blocked_info.pass_decision_frozen);
  EXPECT_EQ(output.blocked_info.pass_decision_freeze_reason,
            "large_lateral_error");
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "large_lateral_error");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_LT(std::abs(output.target_lateral_offset_m), std::abs(ego.frenet.d));
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.front(), config.large_lateral_error_v_max_mps,
              1.0e-9);
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
  EXPECT_TRUE(output.blocked_info.pass_decision_frozen);
  EXPECT_EQ(output.blocked_info.pass_decision_freeze_reason,
            "large_lateral_error");
  EXPECT_EQ(output.blocked_info.pass_gap_reason, "large_lateral_error");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
}

TEST(OvertakePlannerCore, ActivePassTargetDoesNotFreezeAsLargeLateralError) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 0.60;
  config.left_offset_m = 0.70;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto prepare = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto shifted_ego = makeEgo(frame, 5.0, config.left_offset_m);
  const auto output = core.update(0.2, shifted_ego, {first_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  EXPECT_NE(output.blocked_info.pass_decision_freeze_reason,
            "large_lateral_error");
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore, WallMarginRecoveryDoesNotReleaseToFastestTooEarly) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, -1.8);
  const auto first = core.update(0.1, ego, {});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SPEED_GUARD);

  const auto second = core.update(0.2, ego, {});

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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
  EXPECT_FALSE(output.speed_only_fallback_active);
  EXPECT_TRUE(output.wall_risk_speed_guard_active);
  EXPECT_EQ(output.reason, "wall_risk_speed_guard");
  EXPECT_EQ(output.speed_cap_reason, "wall_risk_speed_guard");
  expectSpeedOnlyV2Contract(output, config.wall_risk_v_max_mps);
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

TEST(OvertakePlannerCore,
     HighSpeedCurveSpeedOnlyGuardDoesNotRetainLateralHold) {
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
  expectSpeedOnlyV2Contract(first, config.wall_risk_v_max_mps);
  EXPECT_FALSE(first.lateral_target_hold_active);
  EXPECT_TRUE(first.lateral_target_hold_reason.empty());

  auto left_wall = makeEgo(frame, 5.0, -0.80);
  left_wall.v = 5.0;
  const auto second = core.update(0.2, left_wall, {});

  ASSERT_EQ(second.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  expectSpeedOnlyV2Contract(second, config.wall_risk_v_max_mps);
  EXPECT_FALSE(second.lateral_target_hold_active);
  EXPECT_TRUE(second.lateral_target_hold_reason.empty());

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
  EXPECT_TRUE(output.wall_risk_speed_guard_active);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.reason, "mpc_health_infeasible_guard");
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_infeasible_guard");
  expectSpeedOnlyV2Contract(output, config.mpc_health_v_max_mps);
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
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_infeasible_guard");
  expectSpeedOnlyV2Contract(output, config.mpc_health_v_max_mps);
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
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.reason, "mpc_health_solve_time_guard");
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_solve_time_guard");
  expectSpeedOnlyV2Contract(output, config.mpc_health_v_max_mps);
  EXPECT_TRUE(output.mpc_health.valid);
  EXPECT_NEAR(output.mpc_health.solve_time_ms, health.solve_time_ms, 1.0e-9);
}

TEST(OvertakePlannerCore,
     RaisedMpcHealthSolveTimeThresholdSuppressesSpeedGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_solve_time_warn_ms = 200.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 0;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;

  const auto output = core.update(0.1, ego, {}, health);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(output.active_override);
  EXPECT_FALSE(output.mpc_health_speed_guard_active);
  EXPECT_TRUE(output.speed_cap_reason.empty());
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_stale_guard");
  expectSpeedOnlyV2Contract(output, config.mpc_health_v_max_mps);
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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
  EXPECT_NEAR(output.speed_caps.back(),
              opponent.v - config.yield_speed_margin_mps, 1.0e-9);
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

TEST(OvertakePlannerCore, StartGraceUsesFirstMotionInsteadOfFirstValidUpdate) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.start_grace_safe_stop_enabled = true;
  config.start_grace_duration_sec = 1.0;
  config.start_grace_max_speed_mps = 1.5;
  config.opponent_stale_time_sec = 5.0;
  config.speed_only_fallback_v_max_mps = 8.0;
  config.safety_ellipse_a_m = 2.0;
  config.safety_ellipse_b_m = 2.0;
  config.large_lateral_error_threshold_m = 0.60;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.20);
  ego.v = 0.0;
  const auto idle = core.update(0.1, ego, {});
  ASSERT_FALSE(idle.safe_stop_triggered);

  ego.v = 0.3;
  const auto opponent = makeOpponent(frame, 5.2, -0.60);
  const auto output = core.update(2.5, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_TRUE(output.start_grace_active);
  EXPECT_FALSE(output.safe_stop_triggered);
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
  EXPECT_NEAR(output.speed_caps.front(), config.safe_stop_v_mps, 1.0e-9);
  EXPECT_NEAR(output.speed_caps.back(), config.safe_stop_v_mps, 1.0e-9);
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

  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
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

TEST(OvertakePlannerCore,
     SafeStopRejectsTooCloseObstacleWithPhysicalBrakingProfile) {
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

  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_EQ(output.safe_stop_reason, "safe_stop_infeasible");
  expectSpeedOnlyV2Contract(output,
                            config.opponent_collision_fallback_v_max_mps);
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
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason,
            "speed_only_fallback_safe_stop_infeasible");
  EXPECT_TRUE(output.lateral_offsets.empty());
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), 0.0);
  EXPECT_LE(output.speed_caps.back(), config.speed_only_fallback_v_max_mps);
  EXPECT_LE(output.speed_caps.back(),
            config.opponent_collision_fallback_v_max_mps);
}

TEST(OvertakePlannerCore,
     LeaderSideBySideInfeasibleSafeStopRelaxesCollisionFallbackCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.side_by_side_s_m = 4.0;
  config.side_by_side_leader_priority_enabled = true;
  config.side_by_side_leader_priority_enter_s_m = 1.0;
  config.side_by_side_leader_priority_release_s_m = 0.3;
  config.side_by_side_leader_priority_v_max_mps = 2.8;
  config.opponent_collision_fallback_v_max_mps = 0.5;
  config.safety_ellipse_a_m = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 3.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_LT(output.blocked_info.side_delta_s,
            -config.side_by_side_leader_priority_enter_s_m);
  EXPECT_TRUE(output.blocked_info.leader_priority_active);
  EXPECT_FALSE(output.blocked_info.leader_priority_latched);
  EXPECT_EQ(output.blocked_info.leader_priority_id, "npc");
  EXPECT_FALSE(output.safe_stop_triggered);
  EXPECT_TRUE(output.safe_stop_reason.empty());
  EXPECT_TRUE(output.safe_stop_reject_reason.empty());
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason, "speed_only_fallback_leader_priority");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(),
              config.side_by_side_leader_priority_v_max_mps, 1.0e-9);
  EXPECT_GT(output.speed_caps.back(),
            config.opponent_collision_fallback_v_max_mps);
}

TEST(OvertakePlannerCore,
     RearSideBySideInfeasibleSafeStopKeepsCollisionFallbackCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.side_by_side_s_m = 4.0;
  config.side_by_side_leader_priority_enabled = true;
  config.side_by_side_leader_priority_enter_s_m = 1.0;
  config.side_by_side_leader_priority_v_max_mps = 2.8;
  config.opponent_collision_fallback_v_max_mps = 0.5;
  config.safety_ellipse_a_m = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 7.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_GT(output.blocked_info.side_delta_s,
            config.side_by_side_leader_priority_enter_s_m);
  EXPECT_FALSE(output.blocked_info.leader_priority_active);
  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_EQ(output.safe_stop_reason, "safe_stop_infeasible");
  EXPECT_EQ(output.safe_stop_reject_reason, "opponent_collision");
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_EQ(output.speed_cap_reason,
            "speed_only_fallback_safe_stop_infeasible");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(),
              config.opponent_collision_fallback_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore, SeparateFrontVehicleBlocksLeaderPriority) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_enabled = false;
  config.side_by_side_s_m = 4.0;
  config.side_by_side_leader_priority_enabled = true;
  config.side_by_side_leader_priority_enter_s_m = 2.5;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto side = makeOpponent(frame, 3.8, 0.6);
  side.id = "side";
  auto front = makeOpponent(frame, 8.0, 0.0);
  front.id = "front";

  const auto output = core.update(0.1, ego, {side, front});

  EXPECT_TRUE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_EQ(output.blocked_info.side_id, "side");
  EXPECT_EQ(output.blocked_info.nearest_id, "front");
  EXPECT_FALSE(output.blocked_info.leader_priority_active);
}

TEST(OvertakePlannerCore, LeaderPriorityHoldsAcrossSmallDeltaSChange) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.side_by_side_s_m = 4.0;
  config.side_by_side_leader_priority_enabled = true;
  config.side_by_side_leader_priority_enter_s_m = 1.0;
  config.side_by_side_leader_priority_release_s_m = 0.3;
  config.side_by_side_leader_priority_hold_sec = 1.0;
  config.side_by_side_leader_priority_v_max_mps = 2.8;
  config.opponent_collision_fallback_v_max_mps = 0.5;
  config.safety_ellipse_a_m = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto clear_lead_opponent = makeOpponent(frame, 2.0, 0.0);
  clear_lead_opponent.vx = 0.0;
  clear_lead_opponent.v = 0.0;
  const auto first = core.update(0.1, ego, {clear_lead_opponent});
  ASSERT_TRUE(first.blocked_info.leader_priority_active);
  ASSERT_FALSE(first.blocked_info.leader_priority_latched);

  auto still_behind_opponent = makeOpponent(frame, 4.0, 0.0);
  still_behind_opponent.vx = 0.0;
  still_behind_opponent.v = 0.0;
  const auto second = core.update(0.2, ego, {still_behind_opponent});

  EXPECT_TRUE(second.blocked_info.leader_priority_active);
  EXPECT_TRUE(second.blocked_info.leader_priority_latched);
  EXPECT_EQ(second.blocked_info.leader_priority_reason, "leader_priority_hold");
  EXPECT_EQ(second.speed_cap_reason, "speed_only_fallback_leader_priority");
  ASSERT_FALSE(second.speed_caps.empty());
  EXPECT_NEAR(second.speed_caps.back(),
              config.side_by_side_leader_priority_v_max_mps, 1.0e-9);
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
  EXPECT_FALSE(second.active_override);
  EXPECT_TRUE(second.longitudinal_speed_cap_active);
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

TEST(OvertakePlannerCore,
     SlowFrontExceptionDoesNotBypassHighCurvatureCurveGate) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.slow_front_exception_max_start_curvature_m_inv = 0.04;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {stopped_opponent});
  EXPECT_FALSE(first.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto second = core.update(0.2, ego, {stopped_opponent});
  EXPECT_TRUE(second.blocked_info.front_vehicle_low_speed);
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_TRUE(second.blocked_info.can_pass_left);
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakePlannerCore,
     SlowFrontExceptionOpensOnlyConfiguredGentleCurveGate) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.slow_front_exception_max_start_curvature_m_inv = 0.04;
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
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "slow_front_exception_curve");
  EXPECT_TRUE(second.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     WideParallelObservationDoesNotPromoteToBlockedPassTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_required_cycles = 1;
  config.slow_obstacle_chain_enabled = true;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel_front = makeOpponent(frame, 11.0, -1.0);
  stopped_parallel_front.vx = 0.0;
  stopped_parallel_front.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped_parallel_front});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.slow_obstacle_chain_active);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.nearest_id.empty());
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore, WideParallelObservationDoesNotBypassCurveGate) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_required_cycles = 1;
  config.slow_obstacle_chain_enabled = true;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel_front = makeOpponent(frame, 11.0, -1.0);
  stopped_parallel_front.vx = 0.0;
  stopped_parallel_front.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped_parallel_front});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.slow_obstacle_chain_active);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore, WideParallelObservationDoesNotLatchPassCorridor) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.min_mode_hold_time_sec = 0.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 5.0;
  config.localized_avoidance_merge_distance_m = 8.0;
  config.maneuver_latch_min_hold_sec = 1.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_required_cycles = 1;
  config.slow_obstacle_chain_enabled = true;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto first_ego = makeEgo(frame, 5.0, 0.0);
  auto first_obstacle = makeOpponent(frame, 11.0, -1.0);
  first_obstacle.id = "d2";
  first_obstacle.vx = 0.0;
  first_obstacle.v = 0.0;
  const auto first = core.update(0.1, first_ego, {first_obstacle});
  EXPECT_TRUE(first.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(first.blocked_info.slow_obstacle_chain_active);
  EXPECT_FALSE(first.maneuver_latch_active);
  EXPECT_NE(first.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore, OvertakePermissionDisallowedSectionKeepsFollow) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = false;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
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

TEST(OvertakePlannerCore, SlowFrontExceptionDoesNotBypassDisallowedSection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
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
            "section_disallowed");
  EXPECT_FALSE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "section_disallowed");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::FOLLOW);
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
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
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
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
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

TEST(OvertakePlannerCore,
     ActiveLeftPassKeepsSameSideWhenStaticGapIsLostButCandidateSafe) {
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_TRUE(output.blocked_info.can_pass_right);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
}

TEST(OvertakePlannerCore,
     ActivePassPromotesOffsetSlowObstacleChainAndKeepsPassing) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.large_lateral_error_threshold_m = 0.60;
  config.left_offset_m = 0.70;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto prepare = core.update(0.1, ego, {first_opponent});
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  auto second_static = makeOpponent(frame, 13.0, -0.5);
  second_static.id = "d3";
  second_static.vx = 0.0;
  second_static.v = 0.0;
  const auto shifted_ego = makeEgo(frame, 5.0, config.left_offset_m);
  const auto output = core.update(0.2, shifted_ego, {second_static});

  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_TRUE(output.blocked_info.slow_obstacle_chain_active);
  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_EQ(output.blocked_info.nearest_id, "d3");
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     ActiveRightPassKeepsSameSideWhenStaticGapIsLostButCandidateSafe) {
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

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_feasible);
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

  const auto blocking_opponent = makeOpponent(frame, 5.6, 0.6);
  const auto output = core.update(0.2, ego, {blocking_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_generated);
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

  const auto blocking_opponent = makeOpponent(frame, 5.6, -0.6);
  const auto output = core.update(0.2, ego, {blocking_opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_generated);
}

TEST(OvertakePlannerCore,
     PublishedPassRateLimitRejectsAndPublishesRecoveryInSameCycle) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lateral_target_max_step_m = 0.05;
  config.large_lateral_error_threshold_m = 2.0;
  // このfixtureはpublish直前のrate-limit再評価だけを検証する。parallel接近から
  // 早期YIELDへ入る新しい経路は別fixtureで検証する。
  config.parallel_side_detection_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  // 1周期目で左PASSの横参照を記憶する。実車は次周期までに左側へ移動済みとする。
  const auto first_ego = makeEgo(frame, 5.0, -0.80);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.65);
  const auto prepare = core.update(0.1, first_ego, {first_opponent});
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepare.active_override);

  // 新しく作るPASSはd=+0.8で安全だが、前周期のd=-側プロファイルをrate limitで
  // 引きずると相手(d=-0.7)へ近づく。publish直前の再評価で必ず止める。
  const auto second_ego = makeEgo(frame, 5.0, 0.80);
  auto second_opponent = makeOpponent(frame, 7.0, -0.70);
  second_opponent.vx = 0.0;
  second_opponent.v = 0.0;
  const auto output = core.update(0.2, second_ego, {second_opponent});

  EXPECT_TRUE(output.published_lateral_safety_rejected);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_EQ(output.reason, "published_lateral_safety_reject_recovery");
  EXPECT_LT(output.target_lateral_offset_m, second_ego.frenet.d);
  ASSERT_FALSE(output.speed_caps.empty());
}

TEST(OvertakePlannerCore,
     CenteredFreeRunDoesNotEnterReentryGateForStaleInputs) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  overtake_planner::ReentryInputStatus stale_input;
  stale_input.ego_fresh = true;
  stale_input.v2x_snapshot_fresh = false;
  stale_input.all_observed_opponents_fresh = false;
  stale_input.all_observed_opponents_included = false;
  stale_input.reference_valid = true;
  stale_input.mpc_healthy = false;

  const auto output =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {},
                  overtake_planner::MpcHealthStatus{}, stale_input);

  EXPECT_FALSE(output.reentry_gate.requested);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(output.active_override);
}

TEST(OvertakePlannerCore,
     GenericCenteringRecoveryHoldsForSecondVehicleWithoutAbort) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto second_vehicle = makeOpponent(frame, 13.0, 0.0);
  second_vehicle.id = "d3";
  second_vehicle.vx = 0.0;
  second_vehicle.v = 0.0;
  const auto ego = makeEgo(frame, 5.0, 0.8);
  const auto output =
      core.update(0.1, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  // 短いRECOVERY horizonは中心へ動こうとするが、4秒評価ではd3へ接近する。
  // generic補正はABORTへ昇格せず、現在dを保持するSPEED_GUARDに留まる。
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_FALSE(output.reentry_gate.permitted);
  EXPECT_EQ(output.reentry_gate.blocking_vehicle_id, "d3");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.blocked_info.reentry_hold_active);
  EXPECT_TRUE(output.active_override);
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-6);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     GenericRecoveryStaleInputsRetainVerifiedHoldAndSpeedCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.reentry_require_mpc_health = true;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto second_vehicle = makeOpponent(frame, 13.0, 0.0);
  second_vehicle.id = "d3";
  second_vehicle.vx = 0.0;
  second_vehicle.v = 0.0;
  const auto ego = makeEgo(frame, 5.0, 1.8);
  const auto guarded =
      core.update(0.1, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(guarded.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  ASSERT_TRUE(guarded.reentry_gate.input_complete);
  ASSERT_TRUE(guarded.blocked_info.reentry_hold_active);
  ASSERT_TRUE(guarded.active_override);
  ASSERT_FALSE(guarded.lateral_offsets.empty());
  ASSERT_FALSE(guarded.speed_caps.empty());

  auto stale_v2x_input = readyReentryInput();
  stale_v2x_input.v2x_snapshot_fresh = false;
  const auto v2x_held =
      core.update(0.2, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, stale_v2x_input);
  EXPECT_EQ(v2x_held.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(v2x_held.reentry_gate.reason, "stale_v2x_snapshot");
  EXPECT_EQ(v2x_held.reason, "generic_recovery_stale_input_hold");
  EXPECT_EQ(v2x_held.lateral_offsets, guarded.lateral_offsets);
  EXPECT_EQ(v2x_held.speed_caps, guarded.speed_caps);

  auto stale_mpc_input = readyReentryInput();
  stale_mpc_input.mpc_healthy = false;
  const auto mpc_held =
      core.update(0.3, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, stale_mpc_input);
  EXPECT_EQ(mpc_held.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(mpc_held.reentry_gate.reason, "unhealthy_mpc");
  EXPECT_EQ(mpc_held.reason, "generic_recovery_stale_input_hold");
  EXPECT_EQ(mpc_held.lateral_offsets, guarded.lateral_offsets);
  EXPECT_EQ(mpc_held.speed_caps, guarded.speed_caps);

  auto stale_ego = makeEgo(frame, 5.0, 1.8);
  stale_ego.valid = false;
  const auto held =
      core.update(0.4, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});

  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(held.reentry_gate.requested);
  EXPECT_EQ(held.reentry_gate.reason, "generic_recovery_stale_ego_hold");
  EXPECT_TRUE(held.active_override);
  EXPECT_EQ(held.lateral_offsets, guarded.lateral_offsets);
  EXPECT_EQ(held.speed_caps, guarded.speed_caps);
}

TEST(OvertakePlannerCore,
     ReentryMpcLatencyUsesNewSamplesAndKeepsCurrentLateralPosition) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_mpc_degraded_hold_v_max_mps = 3.0;
  config.reentry_mpc_unhealthy_enter_samples = 2;
  config.reentry_mpc_healthy_release_samples = 3;
  config.reentry_require_mpc_health = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  // まずYIELDを経由して、通常ラインへの復帰gateが必要な文脈を作る。
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), {side_vehicle},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  auto latency_input = readyReentryInput();
  latency_input.mpc_healthy = false;
  latency_input.mpc_health_fresh = true;
  latency_input.mpc_latency_warning = true;
  latency_input.mpc_health_sample_sequence = 1U;
  overtake_planner::MpcHealthStatus latency_health;
  latency_health.valid = true;
  latency_health.solve_time_ms = 81.0;
  latency_health.age_sec = 0.0;
  latency_health.sample_sequence = 1U;

  const auto ego = makeEgo(frame, 5.0, 0.8);
  const auto first_latency =
      core.update(0.2, ego, {}, latency_health, latency_input);
  EXPECT_TRUE(first_latency.reentry_gate.requested);
  EXPECT_FALSE(first_latency.reentry_gate.permitted);
  EXPECT_EQ(first_latency.reentry_gate.reason, "mpc_latency_degraded");
  EXPECT_TRUE(first_latency.blocked_info.reentry_hold_active);
  ASSERT_TRUE(first_latency.active_override);
  EXPECT_NEAR(first_latency.target_lateral_offset_m, ego.frenet.d, 1.0e-9);
  ASSERT_FALSE(first_latency.speed_caps.empty());
  EXPECT_NEAR(first_latency.speed_caps.back(),
              config.reentry_mpc_degraded_hold_v_max_mps, 1.0e-9);

  // planner timerだけが進んでも同じMPC sampleを複数回数えない。
  const auto repeated_latency =
      core.update(0.25, ego, {}, latency_health, latency_input);
  EXPECT_EQ(repeated_latency.reentry_gate.reason, "mpc_latency_degraded");
  ASSERT_FALSE(repeated_latency.speed_caps.empty());
  EXPECT_NEAR(repeated_latency.speed_caps.back(),
              config.reentry_mpc_degraded_hold_v_max_mps, 1.0e-9);

  // 2つ目の新規latency sampleからはhard holdへ落ちる。
  latency_input.mpc_health_sample_sequence = 2U;
  latency_health.sample_sequence = 2U;
  const auto sustained_latency =
      core.update(0.3, ego, {}, latency_health, latency_input);
  EXPECT_EQ(sustained_latency.reentry_gate.reason, "unhealthy_mpc");
  ASSERT_FALSE(sustained_latency.speed_caps.empty());
  EXPECT_NEAR(sustained_latency.speed_caps.back(),
              config.reentry_hold_v_max_mps, 1.0e-9);

  // healthy sampleは3つ連続するまで中心復帰を再開できない。
  auto healthy_input = readyReentryInput();
  healthy_input.mpc_health_fresh = true;
  overtake_planner::MpcHealthStatus healthy_health;
  healthy_health.valid = true;
  healthy_health.solve_time_ms = 10.0;
  healthy_health.age_sec = 0.0;
  for (std::uint64_t sequence = 3U; sequence <= 4U; ++sequence) {
    healthy_input.mpc_health_sample_sequence = sequence;
    healthy_health.sample_sequence = sequence;
    const auto pending = core.update(0.3 + 0.1 * static_cast<double>(sequence),
                                     ego, {}, healthy_health, healthy_input);
    EXPECT_EQ(pending.reentry_gate.reason, "mpc_latency_degraded");
    ASSERT_FALSE(pending.speed_caps.empty());
    EXPECT_NEAR(pending.speed_caps.back(),
                config.reentry_mpc_degraded_hold_v_max_mps, 1.0e-9);
  }
  healthy_input.mpc_health_sample_sequence = 5U;
  healthy_health.sample_sequence = 5U;
  const auto released =
      core.update(0.9, ego, {}, healthy_health, healthy_input);
  EXPECT_TRUE(released.reentry_gate.permitted);
  EXPECT_EQ(released.reentry_gate.reason, "reentry_clear");

  // 保存済みの3 m/s holdはMPC healthがstaleになった瞬間に再利用しない。
  auto stale_input = readyReentryInput();
  stale_input.mpc_healthy = false;
  stale_input.mpc_health_fresh = false;
  stale_input.mpc_health_sample_sequence = 5U;
  const auto stale = core.update(
      1.0, ego, {}, overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_EQ(stale.reentry_gate.reason, "stale_mpc_health");
  ASSERT_FALSE(stale.speed_caps.empty());
  EXPECT_NEAR(stale.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     WallRecoveryWithoutOpponentDoesNotEnterGenericStaleHold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  // 相手車両リスクが無い壁寄り補正は通常のRECOVERY速度guardに留める。
  // ここでgeneric gateへ入ると、何も無い場所でもstale時に0.5m/s保持へ落ちる。
  const auto ego = makeEgo(frame, 5.0, 1.2);
  const auto output = core.update(
      0.1, ego, {}, overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_FALSE(output.reentry_gate.requested);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);

  auto stale_ego = ego;
  stale_ego.valid = false;
  const auto invalid =
      core.update(0.2, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});

  EXPECT_EQ(invalid.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(invalid.active_override);
  EXPECT_FALSE(invalid.reentry_gate.requested);
  EXPECT_EQ(invalid.reason, "disabled_or_invalid");
}

TEST(OvertakePlannerCore,
     GenericPublishedLateralRejectHoldsCurrentDWithoutAbort) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.lateral_target_max_step_m = 0.05;
  config.safety_ellipse_b_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  // 1周期目は低速前方parallelがあるためgeneric RECOVERYをrate-limit記憶へ残す。
  // 相手なしの壁寄り補正はgeneric gateへ入れない。
  auto slow_parallel = makeOpponent(frame, 13.0, -1.8);
  slow_parallel.id = "d2";
  slow_parallel.vx = 0.0;
  slow_parallel.v = 0.0;
  const auto first =
      core.update(0.1, makeEgo(frame, 5.0, -1.2), {slow_parallel},
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(first.reentry_gate.requested);
  ASSERT_TRUE(first.reentry_gate.permitted);
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SPEED_GUARD);

  // 2周期目の正しいRECOVERYはd=+側で安全だが、前周期のd=-側をrate limitで
  // 混ぜると停止車へ近づく。これはgate通過後のpublish再検証経路であり、
  // generic文脈ではABORTではなく現d holdに閉じる。
  auto stopped = makeOpponent(frame, 7.0, -0.45);
  stopped.id = "d3";
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto output = core.update(0.2, makeEgo(frame, 5.0, 0.8), {stopped},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.published_lateral_safety_rejected);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.blocked_info.reentry_hold_active);
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_FALSE(output.reentry_gate.permitted);
  EXPECT_EQ(output.reentry_gate.reason,
            "generic_published_lateral_safety_reject");
  EXPECT_EQ(output.reason, "generic_published_lateral_safety_reject_hold");
  EXPECT_NEAR(output.target_lateral_offset_m, 0.8, 1.0e-9);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     GenericInfeasibleHoldDelegatesEgoV2xAndMpcStaleToWatchdog) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.1;
  config.reentry_require_mpc_health = true;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.max_brake_decel_mps2 = 0.5;
  config.safety_ellipse_b_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);

  // まず低速前方parallelでgeneric中心復帰を開始する。次周期に現dの直近へ停止車を
  // 出すと、gateは拒否され、現d holdとSAFE_STOPはともに不成立になる。
  // それでも横列を捏造せず、speed-only watchdogへ渡すことを確認する。
  const auto ego = makeEgo(frame, 5.0, 1.2);
  auto slow_parallel = makeOpponent(frame, 13.0, 1.8);
  slow_parallel.id = "d2";
  slow_parallel.vx = 0.0;
  slow_parallel.v = 0.0;
  const auto primed =
      core.update(0.1, ego, {slow_parallel},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_TRUE(primed.reentry_gate.requested);
  ASSERT_TRUE(primed.reentry_gate.permitted);

  auto stopped = makeOpponent(frame, 5.2, 1.2);
  stopped.id = "d3";
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto no_hold =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(no_hold.reentry_gate.requested);
  ASSERT_FALSE(no_hold.reentry_gate.permitted);
  ASSERT_EQ(no_hold.reentry_gate.reason, "generic_recovery_hold_infeasible");
  ASSERT_EQ(no_hold.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  ASSERT_FALSE(no_hold.active_override);
  ASSERT_TRUE(no_hold.longitudinal_speed_cap_active);
  ASSERT_TRUE(no_hold.lateral_offsets.empty());
  ASSERT_FALSE(no_hold.speed_caps.empty());
  EXPECT_EQ(no_hold.reason, "generic_recovery_no_safe_hold_watchdog");
  EXPECT_NEAR(no_hold.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);

  auto stale_ego = ego;
  stale_ego.valid = false;
  const auto ego_stale =
      core.update(0.3, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});
  EXPECT_EQ(ego_stale.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(ego_stale.active_override);
  EXPECT_TRUE(ego_stale.longitudinal_speed_cap_active);
  EXPECT_TRUE(ego_stale.lateral_offsets.empty());
  EXPECT_EQ(ego_stale.reentry_gate.reason, "generic_recovery_stale_ego_hold");
  EXPECT_EQ(ego_stale.reason,
            "generic_recovery_stale_ego_no_safe_hold_watchdog");
  ASSERT_FALSE(ego_stale.speed_caps.empty());
  EXPECT_NEAR(ego_stale.speed_caps.back(), config.reentry_hold_v_max_mps,
              1.0e-9);

  auto stale_v2x_input = readyReentryInput();
  stale_v2x_input.v2x_snapshot_fresh = false;
  const auto v2x_stale =
      core.update(0.4, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  stale_v2x_input);
  EXPECT_EQ(v2x_stale.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(v2x_stale.active_override);
  EXPECT_TRUE(v2x_stale.longitudinal_speed_cap_active);
  EXPECT_TRUE(v2x_stale.lateral_offsets.empty());
  EXPECT_EQ(v2x_stale.reentry_gate.reason, "stale_v2x_snapshot");
  EXPECT_EQ(v2x_stale.reason,
            "generic_recovery_stale_input_no_safe_hold_watchdog");
  ASSERT_FALSE(v2x_stale.speed_caps.empty());
  EXPECT_NEAR(v2x_stale.speed_caps.back(), config.reentry_hold_v_max_mps,
              1.0e-9);

  auto stale_mpc_input = readyReentryInput();
  stale_mpc_input.mpc_healthy = false;
  const auto mpc_stale =
      core.update(0.5, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  stale_mpc_input);
  EXPECT_EQ(mpc_stale.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(mpc_stale.active_override);
  EXPECT_TRUE(mpc_stale.longitudinal_speed_cap_active);
  EXPECT_TRUE(mpc_stale.lateral_offsets.empty());
  EXPECT_EQ(mpc_stale.reentry_gate.reason, "unhealthy_mpc");
  EXPECT_EQ(mpc_stale.reason,
            "generic_recovery_stale_input_no_safe_hold_watchdog");
  ASSERT_FALSE(mpc_stale.speed_caps.empty());
  EXPECT_NEAR(mpc_stale.speed_caps.back(), config.reentry_hold_v_max_mps,
              1.0e-9);
}

TEST(OvertakePlannerCore,
     GenericInfeasibleHoldFallsBackToSafeStopWithoutAbort) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.1;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.lateral_target_max_step_m = 0.05;
  config.safety_ellipse_b_m = 0.6;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto slow_parallel = makeOpponent(frame, 13.0, -1.8);
  slow_parallel.id = "d2";
  slow_parallel.vx = 0.0;
  slow_parallel.v = 0.0;
  const auto first =
      core.update(0.1, makeEgo(frame, 5.0, -1.2), {slow_parallel},
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(first.reentry_gate.requested);
  ASSERT_TRUE(first.reentry_gate.permitted);

  // `GenericPublishedLateralRejectHoldsCurrentDWithoutAbort` と異なり、d4で
  // 現d hold自体がgate時点で不成立になる。unsafe holdをpublish再検証へ渡さず、
  // まずSAFE_STOPへ閉じるのが正しいfail-closed順序である。
  auto rate_limited_blocker = makeOpponent(frame, 7.0, -0.45);
  rate_limited_blocker.id = "d3";
  rate_limited_blocker.vx = 0.0;
  rate_limited_blocker.v = 0.0;
  auto hold_blocker = makeOpponent(frame, 8.3, 0.8);
  hold_blocker.id = "d4";
  hold_blocker.vx = 0.0;
  hold_blocker.v = 0.0;
  const auto output = core.update(0.2, makeEgo(frame, 5.0, 0.8),
                                  {rate_limited_blocker, hold_blocker},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_FALSE(output.published_lateral_safety_rejected);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.blocked_info.reentry_hold_active);
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_FALSE(output.reentry_gate.permitted);
  EXPECT_EQ(output.reentry_gate.reason, "generic_recovery_hold_infeasible");
  EXPECT_EQ(output.reason, "no_safe_avoidance");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_LE(output.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
}

TEST(OvertakePlannerCore,
     ReentryGateHoldsLateralOffsetUntilSecondVehicleIsClear) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.reentry_hold_v_max_mps = 0.5;
  config.dynamic_pass_candidate_enabled = true;
  config.lateral_target_max_step_m = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  // d2へ譲るため左側へ出た後、自車が通常ラインへ戻る文脈を再現する。
  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_vehicle = makeOpponent(frame, 5.6, 0.6);
  const auto yield = core.update(0.1, centered_ego, {first_vehicle},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(yield.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);

  // d2を抜いた直後、通常ライン復帰の終端にd3がいる。d3はfront_idだけでなく
  // 復帰候補全体の予測として検出され、中心方向へのRECOVERYをpublishしてはならない。
  const auto passed_ego = makeEgo(frame, 5.0, 0.80);
  auto second_vehicle = makeOpponent(frame, 13.0, 0.0);
  second_vehicle.id = "d3";
  second_vehicle.vx = 0.0;
  second_vehicle.v = 0.0;
  const auto denied = core.update(0.2, passed_ego, {second_vehicle},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(denied.reentry_gate.requested);
  EXPECT_FALSE(denied.reentry_gate.permitted);
  EXPECT_EQ(denied.reentry_gate.reason, "opponent_collision");
  EXPECT_EQ(denied.reentry_gate.blocking_vehicle_id, "d3");
  EXPECT_EQ(denied.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(denied.blocked_info.reentry_hold_active);
  EXPECT_TRUE(denied.active_override);
  EXPECT_NEAR(denied.target_lateral_offset_m, passed_ego.frenet.d, 1.0e-6);

  // d3が消えても1周期だけでは許可しない。連続clear後にだけ中心復帰候補を出す。
  const auto pending = core.update(0.3, passed_ego, {},
                                   overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(pending.reentry_gate.requested);
  EXPECT_FALSE(pending.reentry_gate.permitted);
  EXPECT_EQ(pending.reentry_gate.reason, "reentry_clear_pending");
  EXPECT_TRUE(pending.blocked_info.reentry_hold_active);

  const auto released = core.update(0.4, passed_ego, {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(released.reentry_gate.permitted);
  EXPECT_EQ(released.reentry_gate.reason, "reentry_clear");
  EXPECT_FALSE(released.blocked_info.reentry_hold_active);
  EXPECT_LT(released.target_lateral_offset_m, passed_ego.frenet.d);
}

TEST(OvertakePlannerCore,
     ReentryGateKeepsRecoveryUntilPermittedTrajectoryPhysicallyConverges) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  // YIELD後の通常ライン復帰は許可済みでも、まだd=0.49 mなのでFREE_RUNへ
  // 抜けず、同じRECOVERYを出し続ける。
  const auto residual = core.update(0.2, makeEgo(frame, 5.0, 0.49), {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(residual.reentry_gate.requested);
  EXPECT_TRUE(residual.reentry_gate.permitted);
  EXPECT_EQ(residual.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(residual.selected, overtake_planner::CandidateType::RECOVERY);

  // 安全許可済みかつd=0.05 m以内へ収束して初めてphaseを解放する。
  const auto complete = core.update(0.3, makeEgo(frame, 5.0, 0.04), {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(complete.reentry_gate.requested);
  EXPECT_EQ(complete.mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(OvertakePlannerCore, ReentryGateFailsClosedForStaleOpponent) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_vehicle},
                        overtake_planner::MpcHealthStatus{}, ready)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  auto stale_input = ready;
  stale_input.all_observed_opponents_fresh = false;
  auto stale_second_vehicle = makeOpponent(frame, 15.0, 0.0);
  stale_second_vehicle.id = "d3";
  stale_second_vehicle.stamp_sec = -1.0;
  const auto denied =
      core.update(0.2, makeEgo(frame, 5.0, 0.80), {stale_second_vehicle},
                  overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_TRUE(denied.reentry_gate.requested);
  EXPECT_FALSE(denied.reentry_gate.permitted);
  EXPECT_EQ(denied.reentry_gate.reason, "stale_reentry_opponent");
  EXPECT_EQ(denied.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(denied.blocked_info.reentry_hold_active);
}

TEST(OvertakePlannerCore, ReentryGateFailsClosedForUntrackedNearOpponent) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_vehicle},
                        overtake_planner::MpcHealthStatus{}, ready)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  // Nodeがself重複対策で近接他車を通常候補から外しても、復帰だけは
  // 「相手なし」と解釈せず保持する。
  auto untracked_input = ready;
  untracked_input.all_observed_opponents_included = false;
  const auto denied =
      core.update(0.2, makeEgo(frame, 5.0, 0.8), {},
                  overtake_planner::MpcHealthStatus{}, untracked_input);
  EXPECT_TRUE(denied.reentry_gate.requested);
  EXPECT_FALSE(denied.reentry_gate.permitted);
  EXPECT_EQ(denied.reentry_gate.reason, "untracked_reentry_opponent");
  EXPECT_EQ(denied.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(denied.blocked_info.reentry_hold_active);
}

TEST(OvertakePlannerCore, ReentryLockoutRemainsActiveAtCenterUntilClearCycles) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  auto blocker = makeOpponent(frame, 13.0, 0.0);
  blocker.id = "d3";
  blocker.vx = 0.0;
  blocker.v = 0.0;
  const auto denied = core.update(0.2, makeEgo(frame, 5.0, 0.8), {blocker},
                                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_FALSE(denied.reentry_gate.permitted);

  // 物理的に中心へ寄った後でも、lockoutをclear判定なしで捨てない。
  const auto pending = core.update(0.3, centered_ego, {},
                                   overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(pending.reentry_gate.requested);
  EXPECT_FALSE(pending.reentry_gate.permitted);
  EXPECT_EQ(pending.reentry_gate.reason, "reentry_clear_pending");
  EXPECT_EQ(pending.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);

  const auto released = core.update(0.4, centered_ego, {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(released.reentry_gate.permitted);
  EXPECT_EQ(released.reentry_gate.reason, "reentry_clear");
}

TEST(OvertakePlannerCore, ReentryGateCoversYieldToFreeRunPath) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  // YIELDから相手が消えても、通常ライン外ならFREE_RUNへ直帰しない。
  const auto denied = core.update(0.2, makeEgo(frame, 5.0, 0.8), {},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(denied.reentry_gate.requested);
  EXPECT_FALSE(denied.reentry_gate.permitted);
  EXPECT_EQ(denied.reentry_gate.reason, "reentry_clear_pending");
  EXPECT_EQ(denied.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(denied.blocked_info.reentry_hold_active);
}

TEST(OvertakePlannerCore, PassDepartureDoesNotEnterReentryGate) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto front_vehicle = makeOpponent(frame, 13.0, -0.6);
  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), {front_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  // PASS準備から横へ出る途中は通常ラインへの復帰ではない。
  const auto departure =
      core.update(0.2, makeEgo(frame, 5.0, 0.8), {front_vehicle},
                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(departure.reentry_gate.requested);
  EXPECT_EQ(departure.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore, ReentryLockoutKeepsLastOverrideWhenEgoIsStale) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.reentry_hold_v_max_mps = 0.5;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  auto blocker = makeOpponent(frame, 13.0, 0.0);
  blocker.id = "d3";
  blocker.vx = 0.0;
  blocker.v = 0.0;
  const auto held = core.update(0.2, makeEgo(frame, 5.0, 0.8), {blocker},
                                overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(held.active_override);
  ASSERT_FALSE(held.lateral_offsets.empty());

  auto stale_ego = makeEgo(frame, 5.0, 0.8);
  stale_ego.valid = false;
  const auto stale =
      core.update(0.3, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(stale.active_override);
  EXPECT_EQ(stale.reentry_gate.reason, "stale_ego");
  EXPECT_EQ(stale.reason, "reentry_stale_ego_hold");
  EXPECT_NEAR(stale.target_lateral_offset_m, held.target_lateral_offset_m,
              1.0e-9);
  ASSERT_FALSE(stale.speed_caps.empty());
  EXPECT_NEAR(stale.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);
}
