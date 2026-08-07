#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/overtake_planner_core.hpp"
#include "overtake_planner/planner_output_builder.hpp"
#include "overtake_planner/pp_core_exact_snapshot.hpp"
#include "overtake_planner/reference_override_contract.hpp"
#include "overtake_planner/safety_constraint_authority.hpp"
#include "overtake_planner/safety_evaluator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

TEST(PlannerOutputBuilder, AuthoritativeTargetKeepsLatchedPassOpponent) {
  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_id = "d2";
  blocked.start_grid_target_id = "d3";
  blocked.nearest_id = "d4";

  EXPECT_EQ(overtake_planner::authoritativeTargetVehicleId(blocked), "d2");

  blocked.maneuver_target_latched = false;
  EXPECT_EQ(overtake_planner::authoritativeTargetVehicleId(blocked), "d3");

  blocked.start_grid_target_id.clear();
  EXPECT_EQ(overtake_planner::authoritativeTargetVehicleId(blocked), "d4");
}

TEST(PlannerOutputBuilder,
     InactiveAuthoritativePlanDoesNotCarryMutableTrajectoryPayload) {
  EXPECT_FALSE(overtake_planner::authoritativePlanTrajectoryPayloadRequired(
      false, false));
  EXPECT_TRUE(overtake_planner::authoritativePlanTrajectoryPayloadRequired(
      true, false));
  EXPECT_FALSE(overtake_planner::authoritativePlanTrajectoryPayloadRequired(
      false, true));
  EXPECT_TRUE(
      overtake_planner::authoritativePlanTrajectoryPayloadRequired(true, true));
}

TEST(StopLateralTrackingContract,
     AllowsOnlyFreshSafetyEvaluatedCurrentDHoldWithMatchingBlocker) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::ABORT_RECOVERY;
  output.selected = overtake_planner::CandidateType::RECOVERY;
  output.active_override = true;
  output.lateral_offsets = {1.2, 1.2, 1.2};
  output.blocked_info.ego_lateral_offset_m = 1.2;
  output.blocked_info.reentry_hold_active = true;
  output.blocked_info.parallel_side_candidate = true;
  output.blocked_info.parallel_side_id = "d2";
  output.reentry_gate.requested = true;
  output.reentry_gate.input_complete = true;
  output.reentry_gate.permitted = false;
  output.reentry_gate.reason = "opponent_collision";
  output.reentry_gate.blocking_vehicle_id = "d2";

  EXPECT_TRUE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          output));

  auto centering = output;
  centering.lateral_offsets = {1.2, 1.0, 0.8};
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          centering));

  auto stale = output;
  stale.reentry_gate.input_complete = false;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          stale));

  auto rejected = output;
  rejected.published_lateral_safety_rejected = true;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          rejected));

  auto blocker_mismatch = output;
  blocker_mismatch.reentry_gate.blocking_vehicle_id = "d3";
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          blocker_mismatch));

  auto moving_anchor = output;
  moving_anchor.blocked_info.ego_lateral_offset_m = 1.3;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
          moving_anchor));
}

TEST(StopLateralTrackingContract,
     SafeStopAllowsOnlyCompleteVerifiedConstantCurrentDHold) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::SAFE_STOP;
  output.selected = overtake_planner::CandidateType::SAFE_STOP;
  output.safe_stop_triggered = true;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.lateral_stop_inputs_complete = true;
  output.lateral_offsets = {0.13, 0.13, 0.13};
  output.blocked_info.ego_lateral_offset_m = 0.13;

  EXPECT_TRUE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          output));

  auto centering = output;
  centering.lateral_offsets = {0.13, 0.10, 0.0};
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          centering));

  auto stale = output;
  stale.lateral_stop_inputs_complete = false;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          stale));

  auto unevaluated = output;
  unevaluated.selected_lateral_profile_safety_verified = false;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          unevaluated));

  auto rejected = output;
  rejected.published_lateral_safety_rejected = true;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          rejected));

  auto moving_anchor = output;
  moving_anchor.blocked_info.ego_lateral_offset_m = 0.14;
  EXPECT_FALSE(
      overtake_planner::detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
          moving_anchor));
}

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

overtake_planner::FrenetFrame makeStationaryCurvePreflightFrame() {
  constexpr double kCurvatureMInv = 0.246944;
  std::vector<overtake_planner::ReferencePoint> ref;
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (int i = 0; i <= 100; ++i) {
    const double s_m = static_cast<double>(i);
    ref.push_back(overtake_planner::ReferencePoint{s_m, s_m, 0.0, 0.0,
                                                   kCurvatureMInv, 9.0});
    corridor.push_back(overtake_planner::FrenetCorridorPoint{s_m, -4.0, 4.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  frame.setCorridor(corridor);
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

overtake_planner::FrenetFrame makeModerateCurvedFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 40; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
        static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.080, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

overtake_planner::FrenetFrame makeD1RepresentativeCurveFrame() {
  std::vector<overtake_planner::ReferencePoint> ref;
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (int i = 0; i <= 80; ++i) {
    const double s_m = static_cast<double>(i);
    ref.push_back(
        overtake_planner::ReferencePoint{s_m, s_m, 0.0, 0.0, 0.054, 5.0});
    // D1実ログと同じ右PASSを一意にする。右側は必要楕円分離まで使えるが、
    // 左側はtarget dへ到達できずcorridor preflightで閉じる。
    corridor.push_back(overtake_planner::FrenetCorridorPoint{s_m, -3.50, 1.50});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  frame.setCorridor(corridor);
  return frame;
}

overtake_planner::FrenetFrame makeSteeringRateTransitionFrame() {
  constexpr double kStepM = 0.25;
  constexpr double kTransitionStartM = 11.7;
  constexpr double kTransitionLengthM = 1.0;
  constexpr double kFinalCurvatureMInv = 0.049;
  std::vector<overtake_planner::ReferencePoint> ref;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  for (int i = 0; i <= 240; ++i) {
    const double s_m = kStepM * static_cast<double>(i);
    const double transition_ratio =
        std::clamp((s_m - kTransitionStartM) / kTransitionLengthM, 0.0, 1.0);
    const double curvature_m_inv = kFinalCurvatureMInv * transition_ratio;
    ref.push_back(overtake_planner::ReferencePoint{s_m, x_m, y_m, yaw_rad,
                                                   curvature_m_inv, 5.0});
    yaw_rad += curvature_m_inv * kStepM;
    x_m += std::cos(yaw_rad) * kStepM;
    y_m += std::sin(yaw_rad) * kStepM;
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
  // 共通fixtureは各既存guardを単独で検証する。moving PASSの相対完遂性は
  // 専用fixtureでONにし、実運用YAMLもONのまま固定する。
  config.moving_pass_reachability_enabled = false;
  // 既存Core fixtureは各planner分岐の単体確認用。車体外形そのものは
  // SafetyEvaluator専用testで有効化し、狭い合成corridorへ暗黙に重ねない。
  config.wall_footprint_check_enabled = false;
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  // 既存の単一分岐fixtureは従来どおり1周期でwarm-upへ入れる。fresh観測を
  // またぐruntime debounceは専用fixtureで2周期にして検証する。
  config.start_grid_tracking_probe_required_cycles = 1;
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
  // Core fixtureは各guardを個別に検証するため、未指定guardがruntimeの
  // fail-safe profile (0.5 m/s)で結果を上書きしない旧単体値を使う。
  // 実運用YAMLの低速値はlaunch contract testで別途固定する。
  config.recovery_v_max_mps = 8.5;
  config.wall_margin_recovery_v_max_mps = 8.5;
  config.post_abort_curve_hold_v_max_mps = 4.0;
  config.speed_only_fallback_v_max_mps = 1.0;
  config.wall_risk_v_max_mps = 5.0;
  config.mpc_health_v_max_mps = 3.0;
  config.recovery_speed_guard_v_max_mps = 3.0;
  // 既存のCore fixtureは実runtime Nodeを通さない仮想controller契約を個別に
  // 検証する。AWSIM実走のphysical hard limitはNode/YAMLとC-002L専用fixtureで
  // 固定し、ここへ暗黙に混在させない。
  config.attack_follow_max_steering_angle_rad = 0.5585053606381855;
  return config;
}

overtake_planner::PlannerConfig makePassStartContinuityConfig() {
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_horizon_publish_mode = "overtake_only";
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.90;
  config.parallel_side_detection_enabled = false;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.pass_safe_required_cycles = 5.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.1;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.min_wall_margin_m = 0.4;
  return config;
}

overtake_planner::PlannerConfig makeD1RepresentativeCurveConfig() {
  auto config = makeConfig();
  config.control_rate_hz = 20.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 5.0;
  config.min_mode_hold_time_sec = 0.60;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.090;
  config.gentle_curve_safe_pass_v_max_mps = 10.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.20;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  config.future_side_prediction_enabled = false;
  config.safety_ellipse_b_m = 1.80;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  config.min_pass_gap_m = 0.20;
  config.min_wall_margin_m = 0.10;
  config.pass_speed_cap_mps = 10.0;
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
  status.mpc_health_fresh = true;
  status.pure_pursuit_primary_and_fresh = true;
  status.start_grid_pass_probe_transport_evidence =
      overtake_planner::StartGridProbeTransportEvidence::EXACT_CURRENT;
  return status;
}

std::vector<overtake_planner::PredictedOpponent> makeStraightPredictions(
    const overtake_planner::FrenetFrame &frame,
    const std::vector<overtake_planner::OpponentState> &opponents,
    const overtake_planner::PlannerConfig &config) {
  std::vector<overtake_planner::PredictedOpponent> predictions;
  for (const auto &opponent : opponents) {
    overtake_planner::PredictedOpponent prediction;
    prediction.id = opponent.id;
    for (std::size_t i = 0; i < config.horizon_points; ++i) {
      const double t = static_cast<double>(i) * config.horizon_dt_sec;
      const double x = opponent.x + opponent.vx * t;
      const double y = opponent.y + opponent.vy * t;
      const auto frenet = frame.cartesianToFrenet(x, y, 0.0);
      prediction.t.push_back(t);
      prediction.x.push_back(x);
      prediction.y.push_back(y);
      prediction.s.push_back(frenet.s);
      prediction.d.push_back(frenet.d);
    }
    predictions.push_back(std::move(prediction));
  }
  return predictions;
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     CurrentFrontWinsOverCurrentParallelAndFutureTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 20;
  config.horizon_dt_sec = 0.1;
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  ego.v = 8.0;
  auto front = makeOpponent(frame, 18.0, 0.0);
  front.id = "d3";
  front.vx = 2.0;
  front.v = 2.0;
  auto parallel = makeOpponent(frame, 11.0, 1.0);
  parallel.id = "d2";
  auto future = makeOpponent(frame, 28.0, 0.0);
  future.id = "d4";
  future.vx = 2.0;
  future.v = 2.0;
  const std::vector<overtake_planner::OpponentState> opponents{front, parallel,
                                                               future};
  auto blocked = analyzer.detectBlocked(ego, opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;

  const auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents,
      makeStraightPredictions(frame, opponents, config), 0.0);

  EXPECT_TRUE(output.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.predictive_pass_target_shadow.target_id, "d3");
  EXPECT_EQ(output.predictive_pass_target_shadow.source, "current_front");
  EXPECT_DOUBLE_EQ(
      output.predictive_pass_target_shadow.earliest_blocking_time_sec, 0.0);
  EXPECT_EQ(output.nearest_id, blocked.nearest_id);
  EXPECT_EQ(output.side_id, blocked.side_id);
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     SelectsFutureFrontWithinExistingPredictionAxis) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 21;
  config.horizon_dt_sec = 0.1;
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 8.0;
  config.parallel_side_detection_enabled = false;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  ego.v = 10.0;
  auto opponent = makeOpponent(frame, 30.0, 0.0);
  opponent.id = "d2";
  opponent.vx = 2.0;
  opponent.v = 2.0;
  const std::vector<overtake_planner::OpponentState> opponents{opponent};
  auto blocked = analyzer.detectBlocked(ego, opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  ASSERT_LT(blocked.nearest_index, 0);

  const auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents,
      makeStraightPredictions(frame, opponents, config), 0.0);

  EXPECT_TRUE(output.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.predictive_pass_target_shadow.target_id, "d2");
  EXPECT_EQ(output.predictive_pass_target_shadow.source, "future_front");
  EXPECT_NEAR(output.predictive_pass_target_shadow.earliest_blocking_time_sec,
              1.5, 1.0e-9);
  EXPECT_LT(output.nearest_index, 0);
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     SelectsCurrentParallelWhenNoCurrentFrontExists) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  auto parallel = makeOpponent(frame, 11.0, 1.0);
  parallel.id = "d2";
  const std::vector<overtake_planner::OpponentState> opponents{parallel};
  auto blocked = analyzer.detectBlocked(ego, opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  ASSERT_LT(blocked.nearest_index, 0);
  ASSERT_EQ(blocked.side_id, "d2");

  const auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents,
      makeStraightPredictions(frame, opponents, config), 0.0);

  EXPECT_TRUE(output.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.predictive_pass_target_shadow.target_id, "d2");
  EXPECT_EQ(output.predictive_pass_target_shadow.source, "current_parallel");
  EXPECT_EQ(output.nearest_id, blocked.nearest_id);
  EXPECT_EQ(output.side_id, blocked.side_id);
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     ResolvesEqualFutureTargetsByStableVehicleId) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 21;
  config.horizon_dt_sec = 0.1;
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 8.0;
  config.parallel_side_detection_enabled = false;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  ego.v = 10.0;
  auto target_b = makeOpponent(frame, 30.0, 0.0);
  target_b.id = "d3";
  target_b.vx = 2.0;
  target_b.v = 2.0;
  auto target_a = target_b;
  target_a.id = "d2";
  const std::vector<overtake_planner::OpponentState> opponents{target_b,
                                                               target_a};
  auto blocked = analyzer.detectBlocked(ego, opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;

  const auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents,
      makeStraightPredictions(frame, opponents, config), 0.0);

  EXPECT_TRUE(output.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.predictive_pass_target_shadow.target_id, "d2");
  EXPECT_EQ(output.predictive_pass_target_shadow.source, "future_front");
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     RejectsRearDivergingAndIncompleteInputsWithoutChangingAuthorityFields) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 20.0, 0.0);
  ego.v = 4.0;
  auto rear = makeOpponent(frame, 15.0, 0.0);
  rear.id = "rear";
  rear.vx = 2.0;
  rear.v = 2.0;
  auto diverging = makeOpponent(frame, 35.0, 0.0);
  diverging.id = "diverging";
  diverging.vx = 8.0;
  diverging.v = 8.0;
  const std::vector<overtake_planner::OpponentState> opponents{rear, diverging};
  auto blocked = analyzer.detectBlocked(ego, opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  blocked.maneuver_target_id = "latched";
  blocked.maneuver_target_latched = true;
  auto predictions = makeStraightPredictions(frame, opponents, config);

  auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "no_predictive_target");
  EXPECT_EQ(output.maneuver_target_id, "latched");
  EXPECT_TRUE(output.maneuver_target_latched);

  predictions.front().t.pop_back();
  output = analyzer.evaluatePredictivePassTargetShadow(ego, blocked, opponents,
                                                       predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "prediction_axis_incomplete");
  EXPECT_EQ(output.maneuver_target_id, "latched");
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     RejectsFutureStampAndNonFinitePredictionFailClosed) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  auto opponent = makeOpponent(frame, 18.0, 0.0);
  opponent.id = "d2";
  opponent.stamp_sec = 0.10;
  const std::vector<overtake_planner::OpponentState> future_opponents{opponent};
  auto blocked = analyzer.detectBlocked(ego, future_opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  auto predictions = makeStraightPredictions(frame, future_opponents, config);

  auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, future_opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "invalid_or_stale_opponent");

  opponent.stamp_sec = 0.0;
  const std::vector<overtake_planner::OpponentState> fresh_opponents{opponent};
  blocked = analyzer.detectBlocked(ego, fresh_opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  predictions = makeStraightPredictions(frame, fresh_opponents, config);
  predictions.front().d[3] = std::numeric_limits<double>::quiet_NaN();
  output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, fresh_opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "prediction_axis_invalid");
}

TEST(BlockedRiskAnalyzerPredictivePassTargetShadow,
     RejectsDuplicateIdentityAndInvalidCartesianPredictionFailClosed) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  overtake_planner::BlockedRiskAnalyzer analyzer(frame, config);
  auto ego = makeEgo(frame, 10.0, 0.0);
  auto opponent_a = makeOpponent(frame, 18.0, 0.0);
  opponent_a.id = "d2";
  auto opponent_b = makeOpponent(frame, 20.0, 0.2);
  opponent_b.id = "d2";
  const std::vector<overtake_planner::OpponentState> duplicate_opponents{
      opponent_a, opponent_b};
  auto blocked = analyzer.detectBlocked(ego, duplicate_opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  auto predictions =
      makeStraightPredictions(frame, duplicate_opponents, config);
  predictions.back().id = "d3";

  auto output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, duplicate_opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "duplicate_opponent_id");

  opponent_b.id = "d3";
  const std::vector<overtake_planner::OpponentState> unique_opponents{
      opponent_a, opponent_b};
  blocked = analyzer.detectBlocked(ego, unique_opponents, 0.0);
  blocked.opponent_prediction_inputs_complete = true;
  predictions = makeStraightPredictions(frame, unique_opponents, config);
  predictions.back().id = "d2";
  output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, unique_opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "duplicate_prediction_id");

  predictions = makeStraightPredictions(frame, unique_opponents, config);
  predictions.front().x[2] = std::numeric_limits<double>::infinity();
  output = analyzer.evaluatePredictivePassTargetShadow(
      ego, blocked, unique_opponents, predictions, 0.0);
  EXPECT_FALSE(output.predictive_pass_target_shadow.valid);
  EXPECT_FALSE(output.predictive_pass_target_shadow.inputs_complete);
  EXPECT_EQ(output.predictive_pass_target_shadow.reason,
            "prediction_axis_invalid");
}

TEST(CandidateBuilder,
     PassTransitionDeadlineRejectsRun122317ShortGapsWithoutCompression) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  config.attack_follow_max_steering_rate_radps = 0.4;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::BlockedInfo blocked;
  blocked.corner_abs_curvature = 0.039;
  const double lateral_clearance_m =
      config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
      config.pass_target_lateral_margin_m;
  const double longitudinal_clearance_m =
      config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);

  auto d1_ego = makeEgo(frame, 32.80, 0.45);
  constexpr double d1_to_d2_opponent_d_m = -0.43;
  const double d1_to_d2_available_m = 3.01 - longitudinal_clearance_m;
  const auto d1_to_d2_left = builder.passTransitionFeasibility(
      d1_ego, d1_to_d2_opponent_d_m + lateral_clearance_m, 4.0, blocked,
      d1_to_d2_available_m);
  const auto d1_to_d2_right = builder.passTransitionFeasibility(
      d1_ego, d1_to_d2_opponent_d_m - lateral_clearance_m, 4.0, blocked,
      d1_to_d2_available_m);
  EXPECT_FALSE(d1_to_d2_left.reachable);
  EXPECT_GT(d1_to_d2_left.required_transition_m,
            d1_to_d2_left.available_deadline_m);
  EXPECT_FALSE(d1_to_d2_right.reachable);
  EXPECT_GT(d1_to_d2_right.required_transition_m,
            d1_to_d2_right.available_deadline_m);

  auto d2_ego = makeEgo(frame, 34.17, -0.83);
  blocked.corner_abs_curvature = 0.062;
  constexpr double d2_to_d3_opponent_d_m = 0.65;
  const auto d2_to_d3_right = builder.passTransitionFeasibility(
      d2_ego, d2_to_d3_opponent_d_m - lateral_clearance_m, 4.0, blocked,
      1.88 - longitudinal_clearance_m);
  EXPECT_FALSE(d2_to_d3_right.reachable);
  EXPECT_GT(d2_to_d3_right.required_transition_m,
            d2_to_d3_right.available_deadline_m);

  const auto sufficient_distance = builder.passTransitionFeasibility(
      d1_ego, d1_to_d2_opponent_d_m - lateral_clearance_m, 4.0, blocked,
      40.0 - longitudinal_clearance_m);
  EXPECT_TRUE(sufficient_distance.reachable);
  EXPECT_LE(sufficient_distance.required_transition_m,
            sufficient_distance.available_deadline_m);
}

void expectLegacyOutputsEqual(
    const overtake_planner::PlannerOutput &shadow_off,
    const overtake_planner::PlannerOutput &shadow_on) {
  const auto expect_same_double = [](double lhs, double rhs) {
    if (std::isnan(lhs) || std::isnan(rhs)) {
      EXPECT_TRUE(std::isnan(lhs));
      EXPECT_TRUE(std::isnan(rhs));
    } else {
      EXPECT_EQ(lhs, rhs);
    }
  };
  EXPECT_EQ(shadow_off.mode, shadow_on.mode);
  EXPECT_EQ(shadow_off.selected, shadow_on.selected);
  EXPECT_EQ(shadow_off.reason, shadow_on.reason);
  EXPECT_EQ(shadow_off.active_override, shadow_on.active_override);
  EXPECT_EQ(shadow_off.longitudinal_speed_cap_active,
            shadow_on.longitudinal_speed_cap_active);
  EXPECT_EQ(shadow_off.lateral_offsets, shadow_on.lateral_offsets);
  EXPECT_EQ(shadow_off.longitudinal_offsets_m,
            shadow_on.longitudinal_offsets_m);
  EXPECT_EQ(shadow_off.speed_caps, shadow_on.speed_caps);
  EXPECT_EQ(shadow_off.solver_horizon_intent, shadow_on.solver_horizon_intent);
  EXPECT_EQ(shadow_off.target_lateral_offset_m,
            shadow_on.target_lateral_offset_m);
  expect_same_double(shadow_off.applied_speed_cap_mps,
                     shadow_on.applied_speed_cap_mps);
  EXPECT_EQ(shadow_off.speed_cap_reason, shadow_on.speed_cap_reason);
  EXPECT_EQ(shadow_off.safe_stop_triggered, shadow_on.safe_stop_triggered);
  EXPECT_EQ(shadow_off.published_lateral_safety_rejected,
            shadow_on.published_lateral_safety_rejected);
  EXPECT_EQ(shadow_off.wall_risk_speed_guard_active,
            shadow_on.wall_risk_speed_guard_active);
  EXPECT_EQ(shadow_off.mpc_health_speed_guard_active,
            shadow_on.mpc_health_speed_guard_active);
  EXPECT_EQ(shadow_off.recovery_speed_guard_active,
            shadow_on.recovery_speed_guard_active);
  EXPECT_EQ(shadow_off.blocked_info.reentry_hold_active,
            shadow_on.blocked_info.reentry_hold_active);
  EXPECT_EQ(shadow_off.blocked_info.reentry_centering_authorized,
            shadow_on.blocked_info.reentry_centering_authorized);
  EXPECT_EQ(shadow_off.reentry_gate.requested,
            shadow_on.reentry_gate.requested);
  EXPECT_EQ(shadow_off.reentry_gate.permitted,
            shadow_on.reentry_gate.permitted);
  EXPECT_EQ(shadow_off.reentry_gate.input_complete,
            shadow_on.reentry_gate.input_complete);
  EXPECT_EQ(shadow_off.reentry_gate.clear_cycles,
            shadow_on.reentry_gate.clear_cycles);
  EXPECT_EQ(shadow_off.reentry_gate.evaluated_opponent_count,
            shadow_on.reentry_gate.evaluated_opponent_count);
  EXPECT_EQ(shadow_off.reentry_gate.reason, shadow_on.reentry_gate.reason);
  EXPECT_EQ(shadow_off.reentry_gate.blocking_vehicle_id,
            shadow_on.reentry_gate.blocking_vehicle_id);
  expect_same_double(shadow_off.reentry_gate.min_safety_margin,
                     shadow_on.reentry_gate.min_safety_margin);
  expect_same_double(shadow_off.reentry_gate.cbf_slack,
                     shadow_on.reentry_gate.cbf_slack);
  expect_same_double(shadow_off.reentry_gate.blocking_time_sec,
                     shadow_on.reentry_gate.blocking_time_sec);

  const auto off_payload =
      overtake_planner::makeReferenceOverrideWirePayload(shadow_off, 77U);
  const auto on_payload =
      overtake_planner::makeReferenceOverrideWirePayload(shadow_on, 77U);
  EXPECT_EQ(off_payload.kind, on_payload.kind);
  EXPECT_EQ(off_payload.mode_id, on_payload.mode_id);
  EXPECT_EQ(off_payload.data, on_payload.data);
}

overtake_planner::PlannerOutput runStateLatticeStablePassFixture(
    const std::string &backend,
    const overtake_planner::ReentryInputStatus &input, double now_sec = 0.1,
    const overtake_planner::PurePursuitExactSnapshot *pp_exact_snapshot =
        nullptr) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.trajectory_backend = backend;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.pass_speed_cap_mps = 5.0;
  config.pass_assumed_accel_mps2 = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.0;
  ego.stamp_sec = now_sec;
  auto stopped = makeOpponent(frame, 10.0, -0.6);
  stopped.id = "d2";
  stopped.stamp_sec = now_sec;
  stopped.vx = 0.0;
  stopped.v = 0.0;
  return core.update(now_sec, ego, {stopped},
                     overtake_planner::MpcHealthStatus{}, input,
                     pp_exact_snapshot);
}

TEST(OvertakePlannerCore,
     StateLatticeShadowReparameterizesTimeAxisAndKeepsPpUntrackable) {
  const auto output = runStateLatticeStablePassFixture("state_lattice_shadow",
                                                       readyReentryInput());
  const auto &comparison = output.state_lattice_shadow_comparison;

  ASSERT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(comparison.requested);
  EXPECT_TRUE(comparison.snapshot_complete);
  EXPECT_TRUE(comparison.geometry_generated) << comparison.status_reason;
  EXPECT_TRUE(comparison.adapter_valid) << comparison.status_reason;
  // Coreにはactive PPのexact nearest/lookahead/feedforward/current steering
  // snapshotが無い。壁/CBF shadow比較は記録してもPP trackableへ昇格しない。
  EXPECT_TRUE(comparison.cartesian_trackability_valid);
  EXPECT_FALSE(comparison.cartesian_trackable);
  EXPECT_TRUE(comparison.evaluated);
  EXPECT_EQ(comparison.status_reason, "comparison_recorded");
  EXPECT_EQ(comparison.target_id, "d2");
  EXPECT_TRUE(
      comparison.pass_type == overtake_planner::CandidateType::PASS_LEFT ||
      comparison.pass_type == overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(comparison.pass_side, 0);
  EXPECT_EQ(comparison.opponent_count, 1U);
  EXPECT_NE(comparison.snapshot_hash, 0U);
  EXPECT_TRUE(comparison.current.safety_evaluated);
  EXPECT_TRUE(comparison.current.feasible);
  EXPECT_TRUE(comparison.lattice.safety_evaluated);
  EXPECT_EQ(comparison.lattice.first_reject_reason,
            "steering_angle_limit_exceeded");
  EXPECT_TRUE(std::isfinite(comparison.current.pure_pursuit_required_arc_m));
  EXPECT_TRUE(std::isfinite(comparison.current.pure_pursuit_available_arc_m));
}

TEST(OvertakePlannerCore,
     StateLatticePpExactLeaseRejectsClockRollbackOutsideSharedTolerance) {
  const auto input = readyReentryInput();
  const auto baseline =
      runStateLatticeStablePassFixture("state_lattice_shadow", input);
  ASSERT_NE(baseline.state_lattice_shadow_comparison.pass_side, 0);

  overtake_planner::PurePursuitExactSnapshot snapshot;
  snapshot.valid = true;
  snapshot.reason = "complete";
  snapshot.command_sequence = 7U;
  snapshot.binding.target_vehicle_id = "d2";
  snapshot.binding.pass_direction =
      baseline.state_lattice_shadow_comparison.pass_side;
  snapshot.evaluator_input.active_lookahead_distance_m = 0.4;
  snapshot.requested_output_steering_tire_angle_rad = 0.1;
  snapshot.bounded_steering_tire_angle_rad = 0.05;

  snapshot.command_stamp_sec = 0.15;
  snapshot.valid_until_sec = 0.20;
  auto output = runStateLatticeStablePassFixture("state_lattice_shadow", input,
                                                 0.10, &snapshot);
  EXPECT_TRUE(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_valid);
  EXPECT_EQ(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_reason,
      "complete");

  snapshot.command_stamp_sec = 0.150001;
  snapshot.valid_until_sec = 0.200001;
  output = runStateLatticeStablePassFixture("state_lattice_shadow", input, 0.10,
                                            &snapshot);
  EXPECT_FALSE(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_valid);
  EXPECT_EQ(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_reason,
      "command_stamp_future_or_invalid");

  snapshot.command_stamp_sec = 0.05;
  snapshot.valid_until_sec = 0.10;
  output = runStateLatticeStablePassFixture("state_lattice_shadow", input, 0.10,
                                            &snapshot);
  EXPECT_TRUE(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_valid);

  snapshot.valid_until_sec = 0.10 - 1.0e-6;
  output = runStateLatticeStablePassFixture("state_lattice_shadow", input, 0.10,
                                            &snapshot);
  EXPECT_FALSE(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_valid);
  EXPECT_EQ(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_reason,
      "lease_expired");

  snapshot.command_stamp_sec = 0.10;
  snapshot.valid_until_sec = 0.10;
  output = runStateLatticeStablePassFixture("state_lattice_shadow", input, 0.10,
                                            &snapshot);
  EXPECT_FALSE(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_valid);
  EXPECT_EQ(
      output.state_lattice_shadow_comparison.current_pp_exact_snapshot_reason,
      "command_stamp_future_or_invalid");
}

TEST(OvertakePlannerCore,
     StateLatticeShadowDoesNotSafetyEvaluateIncompleteSnapshot) {
  auto incomplete = readyReentryInput();
  incomplete.all_observed_opponents_included = false;
  const auto output =
      runStateLatticeStablePassFixture("state_lattice_shadow", incomplete);
  const auto &comparison = output.state_lattice_shadow_comparison;

  EXPECT_TRUE(comparison.requested);
  EXPECT_FALSE(comparison.snapshot_complete);
  EXPECT_FALSE(comparison.evaluated);
  EXPECT_FALSE(comparison.lattice.safety_evaluated);
  EXPECT_EQ(comparison.status_reason, "snapshot_incomplete");
}

TEST(OvertakePlannerCore,
     StateLatticeShadowPreservesLegacyOutputAndWireGeneration) {
  const auto frame = makeStraightFrame();
  auto constraint_ego = makeEgo(frame, 5.0, 0.0);
  constraint_ego.v = 0.0;
  constraint_ego.stamp_sec = 0.1;
  const auto inputs = readyReentryInput();
  const auto shadow_off = runStateLatticeStablePassFixture("current", inputs);
  const auto shadow_on =
      runStateLatticeStablePassFixture("state_lattice_shadow", inputs);

  expectLegacyOutputsEqual(shadow_off, shadow_on);
  const auto constraint_off = overtake_planner::makeSafetyConstraint(
      shadow_off, constraint_ego, inputs, 15.0, 6.0);
  const auto constraint_on = overtake_planner::makeSafetyConstraint(
      shadow_on, constraint_ego, inputs, 15.0, 6.0);
  EXPECT_TRUE(overtake_planner::safetyConstraintSemanticallyEqual(
      constraint_off, constraint_on));
  EXPECT_FALSE(shadow_off.state_lattice_shadow_comparison.requested);
  EXPECT_TRUE(shadow_on.state_lattice_shadow_comparison.requested);
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

TEST(InputTimestampFresh, AcceptsSmallFutureSkewButPreservesStaleBoundary) {
  EXPECT_TRUE(overtake_planner::inputTimestampFresh(10.0, 10.049, 0.5, 0.05));
  EXPECT_FALSE(overtake_planner::inputTimestampFresh(10.0, 10.051, 0.5, 0.05));
  EXPECT_TRUE(overtake_planner::inputTimestampFresh(10.0, 9.5, 0.5, 0.05));
  EXPECT_FALSE(overtake_planner::inputTimestampFresh(10.0, 9.499, 0.5, 0.05));
  EXPECT_FALSE(overtake_planner::inputTimestampFresh(
      10.0, std::numeric_limits<double>::quiet_NaN(), 0.5, 0.05));
}

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

TEST(CurrentTransactionStopHoldContract,
     AuthorizesOnlyVerifiedCompleteCurrentDNonPassHold) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::ABORT_RECOVERY;
  output.selected = overtake_planner::CandidateType::RECOVERY;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.lateral_stop_inputs_complete = true;
  output.lateral_offsets = {1.25, 1.25, 1.25};
  output.blocked_info.ego_lateral_offset_m = 1.25;
  output.blocked_info.maneuver_transaction_incomplete = true;
  output.blocked_info.maneuver_transaction_safe_lateral_hold_active = true;
  output.blocked_info.maneuver_target_latched = true;
  output.blocked_info.maneuver_target_observed = true;
  output.blocked_info.maneuver_target_fresh = true;
  output.blocked_info.maneuver_target_id = "d3";
  output.blocked_info.maneuver_chain_tail_observed = true;
  output.blocked_info.maneuver_chain_tail_id = "d3";

  EXPECT_TRUE(overtake_planner::detail::
                  isSafetyEvaluatedCurrentTransactionHoldDuringStop(output));

  output.lateral_offsets.back() = 1.20;
  EXPECT_FALSE(overtake_planner::detail::
                   isSafetyEvaluatedCurrentTransactionHoldDuringStop(output));
  output.lateral_offsets.back() = 1.25;
  output.lateral_stop_inputs_complete = false;
  EXPECT_FALSE(overtake_planner::detail::
                   isSafetyEvaluatedCurrentTransactionHoldDuringStop(output));
  output.lateral_stop_inputs_complete = true;
  output.selected = overtake_planner::CandidateType::PASS_RIGHT;
  EXPECT_FALSE(overtake_planner::detail::
                   isSafetyEvaluatedCurrentTransactionHoldDuringStop(output));
}

TEST(OvertakePlannerCore,
     AuthoritativeStopOutsideCommittedPassEnvelopeLatchesRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_lateral_profile_mode = "localized_latched";
  config.dynamic_pass_candidate_enabled = true;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.large_lateral_error_threshold_m = 0.6;
  config.d_min_m = -2.5;
  config.d_max_m = 2.5;
  config.min_wall_margin_m = 0.2;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.reentry_safe_cycles = 2;
  // 実走相当の小さい周期rate limitでも、直前PASS列をlockoutのcurrent-d
  // STOP holdへ混ぜず、評価済み空間profileをそのまま維持する。
  config.lateral_target_max_step_m = 0.01;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, 0.6);
  target.id = "d3";
  target.stamp_sec = 0.1;
  const auto committed =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  const auto committed_pass_type =
      committed.blocked_info.maneuver_transaction_pass_type;
  ASSERT_TRUE(
      committed_pass_type == overtake_planner::CandidateType::PASS_LEFT ||
      committed_pass_type == overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_TRUE(committed.blocked_info.maneuver_transaction_incomplete);
  const double opposite_d_m =
      committed_pass_type == overtake_planner::CandidateType::PASS_LEFT ? -1.0
                                                                        : 1.0;

  auto stopped_feedback = readyReentryInput();
  stopped_feedback.previous_authoritative_plan_feedback_valid = true;
  stopped_feedback.previous_authoritative_plan_stop_requested = true;
  stopped_feedback.previous_authoritative_plan_trajectory_authorized = false;
  stopped_feedback.previous_authoritative_plan_target_id = "d3";
  stopped_feedback.previous_authoritative_plan_pass_type = committed_pass_type;
  target.stamp_sec = 0.2;
  const auto held =
      core.update(0.2, makeEgo(frame, 5.5, opposite_d_m), {target},
                  overtake_planner::MpcHealthStatus{}, stopped_feedback);

  EXPECT_TRUE(held.blocked_info.pass_reauthorization_lockout_active);
  EXPECT_EQ(held.blocked_info.pass_decision_freeze_reason,
            "pass_profile_recovery_latched");
  EXPECT_TRUE(held.lateral_target_hold_active);
  EXPECT_EQ(held.lateral_target_hold_reason, "pass_profile_recovery_latched");
  EXPECT_NE(held.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(held.selected, overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_EQ(held.selected, overtake_planner::CandidateType::RECOVERY);
  ASSERT_FALSE(held.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(held.lateral_offsets.begin(),
                          held.lateral_offsets.end(),
                          [opposite_d_m](double d_m) {
                            return std::abs(d_m - opposite_d_m) <= 1.0e-4;
                          }));
  EXPECT_TRUE(held.lateral_tracking_authorized_during_stop);
  ASSERT_TRUE(held.controller_spatial_horizon_proof_valid)
      << "reason=" << held.reason
      << " required_arc=" << held.required_controller_spatial_horizon_m
      << " lateral_count=" << held.lateral_offsets.size()
      << " speed_count=" << held.speed_caps.size()
      << " ds_count=" << held.longitudinal_offsets_m.size();
  EXPECT_GT(held.required_controller_spatial_horizon_m, 0.0);
  ASSERT_EQ(held.lateral_offsets.size(), held.speed_caps.size());
  ASSERT_EQ(held.lateral_offsets.size(), held.longitudinal_offsets_m.size());
  EXPECT_GE(held.longitudinal_offsets_m.back(),
            held.required_controller_spatial_horizon_m);
  EXPECT_EQ(held.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);

  auto released_feedback = readyReentryInput();
  released_feedback.previous_authoritative_plan_feedback_valid = true;
  released_feedback.previous_authoritative_plan_stop_requested = false;
  released_feedback.previous_authoritative_plan_trajectory_authorized = true;
  released_feedback.previous_authoritative_plan_target_id = "d3";
  released_feedback.previous_authoritative_plan_pass_type = committed_pass_type;
  target.stamp_sec = 0.3;
  const auto recovering =
      core.update(0.3, makeEgo(frame, 6.0, opposite_d_m), {target},
                  overtake_planner::MpcHealthStatus{}, released_feedback);
  EXPECT_TRUE(recovering.blocked_info.pass_reauthorization_lockout_active);
  EXPECT_TRUE(recovering.lateral_target_hold_active);
  EXPECT_EQ(recovering.lateral_target_hold_reason,
            "pass_profile_recovery_latched");
  ASSERT_EQ(recovering.selected, overtake_planner::CandidateType::RECOVERY);
  ASSERT_FALSE(recovering.lateral_offsets.empty())
      << "reason=" << recovering.reason
      << " raw_reject=" << recovering.raw_selected_reject_reason
      << " published_reject=" << recovering.published_lateral_safety_rejected
      << " speed_only=" << recovering.speed_only_fallback_active
      << " recovery_target="
      << recovering.blocked_info.pass_reauthorization_recovery_target_d_m;
  if (opposite_d_m > 0.0) {
    EXPECT_LT(recovering.lateral_offsets.back(), opposite_d_m);
  } else {
    EXPECT_GT(recovering.lateral_offsets.back(), opposite_d_m);
  }
  EXPECT_EQ(recovering.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::
                MANEUVER_AUTHORIZED);
  EXPECT_FALSE(recovering.lateral_tracking_authorized_during_stop);

  target.stamp_sec = 0.4;
  const auto centered_once =
      core.update(0.4, makeEgo(frame, 6.5, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, released_feedback);
  EXPECT_TRUE(centered_once.blocked_info.pass_reauthorization_lockout_active);
  EXPECT_EQ(centered_once.blocked_info.pass_reauthorization_clear_cycles, 1);

  target.stamp_sec = 0.5;
  const auto centered_twice =
      core.update(0.5, makeEgo(frame, 7.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, released_feedback);
  EXPECT_FALSE(centered_twice.blocked_info.pass_reauthorization_lockout_active);
}

TEST(ReferenceOverrideContract, BuildsV4SpatialLateralPayload) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::ABORT_RECOVERY;
  output.active_override = true;
  output.lateral_offsets = {0.8, 0.6, 0.2};
  output.speed_caps = {1.0, 1.0, 1.0};
  output.longitudinal_offsets_m = {0.0, 0.4, 1.2};
  output.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE;

  const auto v4 =
      overtake_planner::makeReferenceOverrideWirePayload(output, 44U);

  EXPECT_EQ(v4.kind, overtake_planner::ReferenceOverrideWireKind::
                         SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(v4.data,
            (std::vector<float>{1.0F, 7.0F, 3.0F, 0.8F, 0.6F, 0.2F, 1.0F, 1.0F,
                                1.0F, 0.0F, 0.4F, 1.2F, 4.0F, 44.0F, 2.0F}));
}

TEST(ReferenceOverrideContract,
     PreparedPassAndExecutingPassKeepSameWirePayload) {
  overtake_planner::PlannerOutput prepared;
  prepared.mode = overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT;
  prepared.selected = overtake_planner::CandidateType::PASS_RIGHT;
  prepared.active_override = true;
  prepared.lateral_offsets = {1.2, 0.8, 0.2};
  prepared.speed_caps = {0.2, 0.2, 0.2};
  prepared.longitudinal_offsets_m = {0.0, 0.4, 1.2};
  prepared.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;

  auto executing = prepared;
  executing.mode = overtake_planner::BehaviorMode::OVERTAKE_RIGHT;

  const auto prepared_payload =
      overtake_planner::makeReferenceOverrideWirePayload(prepared, 51U);
  const auto executing_payload =
      overtake_planner::makeReferenceOverrideWirePayload(executing, 51U);

  EXPECT_EQ(prepared_payload.kind, overtake_planner::ReferenceOverrideWireKind::
                                       SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(prepared_payload.mode_id,
            static_cast<int>(overtake_planner::BehaviorMode::OVERTAKE_RIGHT));
  EXPECT_EQ(prepared_payload.data, executing_payload.data);
  EXPECT_TRUE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      prepared_payload, executing_payload));
}

TEST(ReferenceOverrideContract,
     InvalidSuppliedSpatialAxisFallsBackToSpeedOnly) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::ABORT_RECOVERY;
  output.active_override = true;
  output.longitudinal_speed_cap_active = true;
  output.applied_speed_cap_mps = 0.5;
  output.lateral_offsets = {0.8, 0.6, 0.2};
  output.speed_caps = {0.5, 0.5, 0.5};
  output.longitudinal_offsets_m = {0.0, 0.0, 0.0};

  const auto speed_only =
      overtake_planner::makeReferenceOverrideWirePayload(output, 45U);

  EXPECT_EQ(speed_only.kind,
            overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2);
  EXPECT_EQ(speed_only.data,
            (std::vector<float>{1.0F, 7.0F, 0.0F, 2.0F, 45.0F, 0.5F}));

  output.longitudinal_offsets_m = {0.0, 0.4};
  const auto wrong_size =
      overtake_planner::makeReferenceOverrideWirePayload(output, 46U);
  EXPECT_EQ(wrong_size.kind,
            overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2);
}

TEST(PlannerOutputBuilder,
     InfeasibleFastestPublishesSpeedOnlyFallbackInSameCycle) {
  auto config = makeConfig();
  config.speed_only_fallback_enabled = true;
  config.speed_only_fallback_v_max_mps = 0.7;
  overtake_planner::PlannerOutputBuilder builder(config);
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.v = 4.0;
  ego.frenet.d = 0.0;
  overtake_planner::CandidateTrajectory fastest;
  fastest.type = overtake_planner::CandidateType::FASTEST;
  fastest.feasible = false;
  fastest.reject_reason = "braking_follow_rejected";
  fastest.v_ref = {5.0, 5.0};
  overtake_planner::BlockedInfo blocked;
  overtake_planner::SafeStopContext safe_stop_context;
  overtake_planner::CandidateTrajectory safe_stop;
  overtake_planner::ActiveSectionSafety section;
  overtake_planner::MpcHealthStatus mpc_health;
  const overtake_planner::PlannerOutputBuildInput input{
      overtake_planner::BehaviorMode::FREE_RUN,
      ego,
      fastest,
      blocked,
      safe_stop_context,
      safe_stop,
      false,
      0,
      0,
      0,
      0.0,
      section,
      mpc_health};

  const auto output = builder.build(input);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_EQ(output.speed_cap_reason,
            "speed_only_fallback_braking_follow_rejected");
  EXPECT_LE(output.applied_speed_cap_mps, 0.7);
  EXPECT_TRUE(output.lateral_offsets.empty());
  EXPECT_TRUE(output.longitudinal_offsets_m.empty());
}

TEST(PlannerOutputBuilder,
     InfeasibleSafeStopPreservesOnlyEvaluatedRecoveryLateralProfile) {
  auto config = makeConfig();
  config.speed_only_fallback_enabled = true;
  config.speed_only_fallback_v_max_mps = 0.5;
  overtake_planner::PlannerOutputBuilder builder(config);

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.v = 4.0;
  ego.frenet.d = -0.4;
  overtake_planner::CandidateTrajectory recovery;
  recovery.type = overtake_planner::CandidateType::RECOVERY;
  recovery.safety_evaluated = true;
  recovery.feasible = true;
  recovery.pass_target_corridor_valid = true;
  recovery.controller_tracking_profile_valid = true;
  recovery.longitudinal_profile_valid = true;
  recovery.d = {-0.4, -0.2, 0.0};
  recovery.v_ref = {2.0, 2.0, 2.0};
  recovery.longitudinal_offsets_m = {0.0, 1.0, 2.0};

  overtake_planner::BlockedInfo blocked;
  overtake_planner::SafeStopContext safe_stop_context;
  overtake_planner::CandidateTrajectory safe_stop;
  safe_stop.feasible = false;
  safe_stop.reject_reason = "opponent_collision";
  overtake_planner::ActiveSectionSafety section;
  overtake_planner::MpcHealthStatus mpc_health;
  const auto build =
      [&](const overtake_planner::CandidateTrajectory &selected) {
        return builder.build(overtake_planner::PlannerOutputBuildInput{
            overtake_planner::BehaviorMode::ABORT_RECOVERY, ego, selected,
            blocked, safe_stop_context, safe_stop, true, 1, 0, 0, 0.25, section,
            mpc_health});
      };

  const auto authorized = build(recovery);
  EXPECT_TRUE(authorized.active_override);
  EXPECT_TRUE(authorized.speed_only_fallback_active);
  EXPECT_EQ(authorized.lateral_offsets, recovery.d);
  EXPECT_EQ(authorized.longitudinal_offsets_m, recovery.longitudinal_offsets_m);
  ASSERT_EQ(authorized.speed_caps.size(), recovery.v_ref.size());
  EXPECT_LE(authorized.speed_caps.back(), config.speed_only_fallback_v_max_mps);

  auto unevaluated = recovery;
  unevaluated.safety_evaluated = false;
  const auto rejected = build(unevaluated);
  EXPECT_FALSE(rejected.active_override);
  EXPECT_TRUE(rejected.speed_only_fallback_active);
  EXPECT_TRUE(rejected.lateral_offsets.empty());
  EXPECT_TRUE(rejected.longitudinal_offsets_m.empty());
}

TEST(PlannerOutputBuilder,
     RecoveryLateralGuardUsesSelectedRecoveryTargetAndFailsClosed) {
  auto config = makeConfig();
  config.wall_risk_speed_guard_enabled = false;
  config.mpc_health_speed_guard_enabled = false;
  config.recovery_speed_guard_enabled = true;
  config.recovery_speed_guard_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  overtake_planner::PlannerOutputBuilder builder(config);

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.frenet.d = 0.8;
  overtake_planner::BlockedInfo blocked;
  blocked.ego_wall_clearance_m = 10.0;
  overtake_planner::SafeStopContext safe_stop_context;
  overtake_planner::CandidateTrajectory safe_stop;
  overtake_planner::ActiveSectionSafety section;
  overtake_planner::MpcHealthStatus mpc_health;

  overtake_planner::CandidateTrajectory hold;
  hold.type = overtake_planner::CandidateType::RECOVERY;
  hold.feasible = true;
  hold.d = {0.8, 0.8};
  hold.v_ref = {2.0, 2.0};
  const auto build =
      [&](const overtake_planner::CandidateTrajectory &candidate) {
        return builder.build(overtake_planner::PlannerOutputBuildInput{
            overtake_planner::BehaviorMode::SPEED_GUARD, ego, candidate,
            blocked, safe_stop_context, safe_stop, false, 0, 0, 0, 0.25,
            section, mpc_health});
      };

  const auto held = build(hold);
  EXPECT_NEAR(held.recovery_tracking_target_d_m, ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(held.recovery_tracking_error_m, 0.0, 1.0e-9);
  EXPECT_FALSE(held.recovery_speed_guard_active);

  auto centering = hold;
  centering.d.back() = 0.0;
  const auto centered = build(centering);
  EXPECT_NEAR(centered.recovery_tracking_target_d_m, 0.0, 1.0e-9);
  EXPECT_NEAR(centered.recovery_tracking_error_m, 0.8, 1.0e-9);
  EXPECT_TRUE(centered.recovery_speed_guard_active);
  EXPECT_EQ(centered.speed_cap_reason, "recovery_lateral_error_speed_guard");

  auto invalid = hold;
  invalid.d.back() = std::numeric_limits<double>::quiet_NaN();
  const auto failed_closed = build(invalid);
  EXPECT_NEAR(failed_closed.recovery_tracking_target_d_m, 0.0, 1.0e-9);
  EXPECT_TRUE(failed_closed.recovery_speed_guard_active);
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

TEST(ReferenceOverrideContract,
     TrackingReleaseTokenForcesExactlyOneGenerationAdvance) {
  EXPECT_FALSE(overtake_planner::referenceOverrideGenerationMustAdvance(
      false, false, 0U, false, 0U));
  EXPECT_TRUE(overtake_planner::referenceOverrideGenerationMustAdvance(
      false, true, 7U, false, 0U));
  EXPECT_FALSE(overtake_planner::referenceOverrideGenerationMustAdvance(
      false, true, 7U, true, 7U));
  EXPECT_TRUE(overtake_planner::referenceOverrideGenerationMustAdvance(
      false, true, 8U, true, 7U));
  EXPECT_TRUE(overtake_planner::referenceOverrideGenerationMustAdvance(
      true, true, 8U, true, 8U));
}

TEST(ReferenceOverrideContract,
     TrackingReleaseProofRequiresExactGenerationTargetSideAndToken) {
  using overtake_planner::CandidateType;
  EXPECT_TRUE(overtake_planner::trackingReleaseProofReady(
      true, false, 0U, 0U, 10U, false, "", "", CandidateType::FASTEST,
      CandidateType::FASTEST));
  EXPECT_FALSE(overtake_planner::trackingReleaseProofReady(
      true, true, 0U, 10U, 10U, true, "d2", "d2", CandidateType::PASS_RIGHT,
      CandidateType::PASS_RIGHT));
  EXPECT_FALSE(overtake_planner::trackingReleaseProofReady(
      true, true, 7U, 9U, 10U, true, "d2", "d2", CandidateType::PASS_RIGHT,
      CandidateType::PASS_RIGHT));
  EXPECT_FALSE(overtake_planner::trackingReleaseProofReady(
      true, true, 7U, 10U, 10U, true, "d3", "d2", CandidateType::PASS_RIGHT,
      CandidateType::PASS_RIGHT));
  EXPECT_FALSE(overtake_planner::trackingReleaseProofReady(
      true, true, 7U, 10U, 10U, true, "d2", "d2", CandidateType::PASS_LEFT,
      CandidateType::PASS_RIGHT));
  EXPECT_TRUE(overtake_planner::trackingReleaseProofReady(
      true, true, 7U, 10U, 10U, true, "d2", "d2", CandidateType::PASS_RIGHT,
      CandidateType::PASS_RIGHT));
  EXPECT_FALSE(overtake_planner::trackingReleaseProofReady(
      false, true, 7U, 10U, 10U, true, "d2", "d2", CandidateType::PASS_RIGHT,
      CandidateType::PASS_RIGHT));
}

TEST(ReferenceOverrideContract,
     AttackFollowStopTransportProofIsTypedPreviousGenerationOnly) {
  EXPECT_TRUE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, true, true, 10U, 11U));
  // 診断reasonは引数に存在せず、typed provenanceだけを契約に使う。
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, true, false, 10U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, false, true, 10U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, false, true, true, true, 10U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, false, true, true, 10U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      true, true, true, true, true, 10U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, true, true, 11U, 11U));
  EXPECT_FALSE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, true, true, 9U, 11U));
  EXPECT_TRUE(overtake_planner::attackFollowStopTransportReleaseProofReady(
      false, true, true, true, true, 16777215U, 1U));
}

TEST(ReferenceOverrideContract,
     AttackFollowTrackingIdentityRequiresPublishedSafeTransaction) {
  using overtake_planner::BehaviorMode;
  using overtake_planner::CandidateType;
  using overtake_planner::OverrideTrackingIdentityKind;

  overtake_planner::PlannerOutput follow;
  follow.mode = BehaviorMode::FOLLOW_BLOCKED;
  follow.selected = CandidateType::FOLLOW;
  follow.raw_selected = CandidateType::FOLLOW;
  follow.active_override = true;
  follow.selected_lateral_profile_safety_verified = true;
  follow.published_lateral_safety_rejected = false;
  follow.maneuver_latch_active = true;
  follow.maneuver_latch_target_id = "d2";
  follow.lateral_offsets = {3.36, 3.10, 2.80};
  follow.speed_caps = {0.20, 0.50, 1.30};
  follow.longitudinal_offsets_m = {0.0, 0.25, 0.75};
  follow.blocked_info.maneuver_transaction_retry_active = true;
  follow.blocked_info.maneuver_transaction_incomplete = true;
  follow.blocked_info.maneuver_transaction_tracking_continuity_armed = true;
  follow.blocked_info.maneuver_transaction_tracking_stop_active = false;
  follow.blocked_info.maneuver_target_latched = true;
  follow.blocked_info.maneuver_target_id = "d2";
  follow.blocked_info.maneuver_transaction_pass_type =
      CandidateType::PASS_RIGHT;
  follow.blocked_info.attack_follow_hold_pass_side = true;
  follow.blocked_info.attack_follow_candidate_generated = true;
  follow.blocked_info.attack_follow_candidate_feasible = true;
  follow.blocked_info.attack_follow_candidate_tracking_profile_valid = true;
  follow.blocked_info.attack_follow_candidate_safe_lateral_hold = false;

  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(follow, 33U);
  ASSERT_EQ(wire.kind, overtake_planner::ReferenceOverrideWireKind::
                           SPATIAL_LATERAL_AND_SPEED_V4);
  const auto identity =
      overtake_planner::makeOverrideTrackingIdentity(follow, wire, 33U, 7U);
  EXPECT_EQ(identity.kind, OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  EXPECT_EQ(identity.generation, 33U);
  EXPECT_EQ(identity.attempt_id, 7U);
  EXPECT_EQ(identity.target_id, "d2");
  EXPECT_EQ(identity.pass_type, CandidateType::PASS_RIGHT);

  auto rejected = follow;
  rejected.selected_lateral_profile_safety_verified = false;
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(rejected, wire, 33U, 7U)
          .kind,
      OverrideTrackingIdentityKind::NONE);
  rejected = follow;
  rejected.blocked_info.attack_follow_candidate_tracking_profile_valid = false;
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(rejected, wire, 33U, 7U)
          .kind,
      OverrideTrackingIdentityKind::NONE);
  rejected = follow;
  rejected.blocked_info.maneuver_transaction_tracking_stop_active = true;
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(rejected, wire, 33U, 7U)
          .kind,
      OverrideTrackingIdentityKind::NONE);
  rejected = follow;
  rejected.maneuver_latch_target_id = "d3";
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(rejected, wire, 33U, 7U)
          .kind,
      OverrideTrackingIdentityKind::NONE);

  auto generic_safe_hold = follow;
  generic_safe_hold.blocked_info.attack_follow_candidate_safe_lateral_hold =
      true;
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(generic_safe_hold,
                                                           wire, 33U, 7U)
                .kind,
            OverrideTrackingIdentityKind::NONE);
  auto collision_current_d_hold = generic_safe_hold;
  collision_current_d_hold.blocked_info
      .attack_follow_candidate_opponent_collision_current_d_hold = true;
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(
                collision_current_d_hold, wire, 33U, 7U)
                .kind,
            OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  auto collision_inward_connector = generic_safe_hold;
  collision_inward_connector.blocked_info
      .attack_follow_candidate_opponent_collision_inward_connector = true;
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(
                collision_inward_connector, wire, 33U, 7U)
                .kind,
            OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  auto ambiguous_special_hold = collision_inward_connector;
  ambiguous_special_hold.blocked_info
      .attack_follow_candidate_opponent_collision_current_d_hold = true;
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(
                ambiguous_special_hold, wire, 33U, 7U)
                .kind,
            OverrideTrackingIdentityKind::NONE);

  auto speed_only_wire = wire;
  speed_only_wire.kind =
      overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2;
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(
                follow, speed_only_wire, 33U, 7U)
                .kind,
            OverrideTrackingIdentityKind::NONE);
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(follow, wire, 33U, 0U)
          .kind,
      OverrideTrackingIdentityKind::NONE);

  auto pass = follow;
  pass.mode = BehaviorMode::OVERTAKE_RIGHT;
  pass.selected = CandidateType::PASS_RIGHT;
  pass.raw_selected = CandidateType::PASS_RIGHT;
  pass.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  const auto pass_wire =
      overtake_planner::makeReferenceOverrideWirePayload(pass, 34U);
  const auto pass_identity =
      overtake_planner::makeOverrideTrackingIdentity(pass, pass_wire, 34U, 7U);
  EXPECT_EQ(pass_identity.kind, OverrideTrackingIdentityKind::COMMITTED_PASS);
  EXPECT_EQ(pass_identity.target_id, "d2");
  EXPECT_EQ(pass_identity.pass_type, CandidateType::PASS_RIGHT);
}

TEST(ReferenceOverrideContract,
     AttackFollowTrackingIdentityIsSameAttemptAndAtMostPreviousGeneration) {
  using overtake_planner::CandidateType;
  using overtake_planner::OverrideTrackingIdentity;
  using overtake_planner::OverrideTrackingIdentityKind;

  OverrideTrackingIdentity previous;
  previous.kind = OverrideTrackingIdentityKind::ATTACK_FOLLOW;
  previous.generation = 33U;
  previous.attempt_id = 7U;
  previous.target_id = "d2";
  previous.pass_type = CandidateType::PASS_RIGHT;
  auto current = previous;
  current.generation = 34U;

  const auto *n_minus_one = overtake_planner::overrideTrackingIdentityForProof(
      current, previous, 33U, 34U, 7U, "d2", CandidateType::PASS_RIGHT);
  ASSERT_NE(n_minus_one, nullptr);
  EXPECT_EQ(n_minus_one->kind, OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  EXPECT_EQ(n_minus_one->target_id, "d2");
  EXPECT_EQ(n_minus_one->pass_type, CandidateType::PASS_RIGHT);

  EXPECT_EQ(
      overtake_planner::overrideTrackingIdentityForProof(
          current, previous, 32U, 34U, 7U, "d2", CandidateType::PASS_RIGHT),
      nullptr);
  EXPECT_EQ(
      overtake_planner::overrideTrackingIdentityForProof(
          current, previous, 33U, 34U, 8U, "d2", CandidateType::PASS_RIGHT),
      nullptr);
  previous.target_id = "d3";
  EXPECT_EQ(
      overtake_planner::overrideTrackingIdentityForProof(
          current, previous, 33U, 34U, 7U, "d2", CandidateType::PASS_RIGHT),
      nullptr);
  previous.target_id = "d2";
  previous.pass_type = CandidateType::PASS_LEFT;
  EXPECT_EQ(
      overtake_planner::overrideTrackingIdentityForProof(
          current, previous, 33U, 34U, 7U, "d2", CandidateType::PASS_RIGHT),
      nullptr);

  previous = current;
  previous.generation = 16777215U;
  current.generation = 1U;
  EXPECT_NE(overtake_planner::overrideTrackingIdentityForProof(
                current, previous, 16777215U, 1U, 7U, "d2",
                CandidateType::PASS_RIGHT),
            nullptr);
}

TEST(ReferenceOverrideContract,
     PublishedInwardConnectorRequiresExactCurrentLateralAuthority) {
  using overtake_planner::BehaviorMode;
  using overtake_planner::CandidateType;
  using overtake_planner::OverrideTrackingIdentityKind;

  overtake_planner::PlannerOutput connector;
  connector.mode = BehaviorMode::FOLLOW_BLOCKED;
  connector.selected = CandidateType::FOLLOW;
  connector.raw_selected = CandidateType::FOLLOW;
  connector.active_override = true;
  connector.selected_lateral_profile_safety_verified = true;
  connector.published_lateral_safety_rejected = false;
  connector.maneuver_latch_active = true;
  connector.maneuver_latch_target_id = "d2";
  connector.lateral_offsets = {3.36, 3.20, 3.06};
  connector.speed_caps = {0.20, 0.50, 1.30};
  connector.longitudinal_offsets_m = {0.0, 0.25, 0.75};
  connector.blocked_info.maneuver_target_latched = true;
  connector.blocked_info.maneuver_target_id = "d2";
  connector.blocked_info.maneuver_target_observed = true;
  connector.blocked_info.maneuver_target_fresh = true;
  connector.blocked_info.opponent_prediction_inputs_complete = true;
  connector.blocked_info.maneuver_transaction_pass_type =
      CandidateType::PASS_RIGHT;
  connector.blocked_info.maneuver_transaction_retry_active = true;
  connector.blocked_info.maneuver_transaction_incomplete = true;
  connector.blocked_info.maneuver_transaction_tracking_continuity_armed = true;
  connector.blocked_info.maneuver_transaction_tracking_stop_active = false;
  connector.blocked_info.attack_follow_hold_pass_side = true;
  connector.blocked_info.attack_follow_candidate_generated = true;
  connector.blocked_info.attack_follow_candidate_feasible = true;
  connector.blocked_info.attack_follow_candidate_tracking_profile_valid = true;
  connector.blocked_info.attack_follow_candidate_safe_lateral_hold = true;
  connector.blocked_info
      .attack_follow_candidate_opponent_collision_inward_connector = true;
  // 実wireではcurrent-d holdを先に生成してwall rejectし、その同周期に
  // inward connectorを生成・採用する。current-d variantの生成自体は拒否根拠に
  // せず、usedとopponent-collision identityだけを相互排他で確認する。
  connector.blocked_info.attack_follow_current_d_hold_variant_generated = true;
  connector.blocked_info.attack_follow_current_d_hold_variant_feasible = false;
  connector.blocked_info.attack_follow_current_d_hold_variant_used = false;
  connector.blocked_info.attack_follow_inward_connector_variant_generated =
      true;
  connector.blocked_info.attack_follow_inward_connector_variant_feasible = true;
  connector.blocked_info.attack_follow_inward_connector_variant_used = true;

  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(connector, 33U);
  ASSERT_EQ(wire.kind, overtake_planner::ReferenceOverrideWireKind::
                           SPATIAL_LATERAL_AND_SPEED_V4);
  const auto identity =
      overtake_planner::makeOverrideTrackingIdentity(connector, wire, 33U, 7U);
  ASSERT_EQ(identity.kind, OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.frenet.d = 3.36;

  const auto lateral_required =
      [&connector, &ego, &identity, &wire](
          const auto &output, const auto &current_identity,
          const auto &current_wire, std::uint32_t generation = 33U,
          std::uint64_t attempt_id = 7U, const std::string &target_id = "d2",
          std::int8_t pass_direction = -1, bool trajectory_authorized = true) {
        return overtake_planner::
            isPublishedAttackFollowInwardConnectorLateralRequired(
                output, ego, current_identity, current_wire, generation,
                attempt_id, target_id, pass_direction, trajectory_authorized);
      };

  ASSERT_TRUE(overtake_planner::isAttackFollowInwardConnectorOutput(connector));
  EXPECT_TRUE(lateral_required(connector, identity, wire));
  EXPECT_TRUE(overtake_planner::composeLateralManeuverRequired(
      true, true, lateral_required(connector, identity, wire)));

  auto current_d_hold = connector;
  current_d_hold.blocked_info
      .attack_follow_candidate_opponent_collision_current_d_hold = true;
  EXPECT_FALSE(lateral_required(current_d_hold, identity, wire));
  auto current_d_hold_used = connector;
  current_d_hold_used.blocked_info.attack_follow_current_d_hold_variant_used =
      true;
  EXPECT_FALSE(lateral_required(current_d_hold_used, identity, wire));
  EXPECT_FALSE(
      overtake_planner::isAttackFollowInwardConnectorOutput(current_d_hold));
  EXPECT_TRUE(
      overtake_planner::composeLateralManeuverRequired(true, false, false));
  EXPECT_FALSE(
      overtake_planner::composeLateralManeuverRequired(false, false, true));

  auto speed_only = wire;
  speed_only.kind = overtake_planner::ReferenceOverrideWireKind::SPEED_ONLY_V2;
  EXPECT_FALSE(lateral_required(connector, identity, speed_only));
  auto v3 = wire;
  v3.kind = overtake_planner::ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3;
  EXPECT_FALSE(lateral_required(connector, identity, v3));

  auto wire_mode_struct_mismatch = wire;
  ++wire_mode_struct_mismatch.mode_id;
  EXPECT_FALSE(
      lateral_required(connector, identity, wire_mode_struct_mismatch));
  auto wire_mode_data_mismatch = wire;
  wire_mode_data_mismatch.data[1] += 1.0F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_mode_data_mismatch));
  auto wire_count_mismatch = wire;
  wire_count_mismatch.data[2] += 1.0F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_count_mismatch));
  const std::size_t count = connector.lateral_offsets.size();
  auto wire_d_mismatch = wire;
  wire_d_mismatch.data[3] += 0.01F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_d_mismatch));
  auto wire_speed_mismatch = wire;
  wire_speed_mismatch.data[3U + count] += 0.01F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_speed_mismatch));
  auto wire_distance_mismatch = wire;
  wire_distance_mismatch.data[3U + 2U * count] += 0.01F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_distance_mismatch));
  auto wire_intent_mismatch = wire;
  wire_intent_mismatch.data.back() += 1.0F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_intent_mismatch));
  EXPECT_FALSE(lateral_required(connector, identity, wire, 34U));
  EXPECT_FALSE(lateral_required(connector, identity, wire, 33U, 8U));
  EXPECT_FALSE(lateral_required(connector, identity, wire, 33U, 7U, "d3"));
  EXPECT_FALSE(lateral_required(connector, identity, wire, 33U, 7U, "d2", 1));

  auto wrong_identity = identity;
  wrong_identity.kind = OverrideTrackingIdentityKind::COMMITTED_PASS;
  EXPECT_FALSE(lateral_required(connector, wrong_identity, wire));
  EXPECT_FALSE(overtake_planner::composeLateralManeuverRequired(
      true, overtake_planner::isAttackFollowInwardConnectorOutput(connector),
      lateral_required(connector, wrong_identity, wire)));
  wrong_identity = identity;
  wrong_identity.generation = 32U;
  EXPECT_FALSE(lateral_required(connector, wrong_identity, wire));

  auto wire_generation_mismatch = wire;
  wire_generation_mismatch.data[wire_generation_mismatch.data.size() - 2U] =
      34.0F;
  EXPECT_FALSE(lateral_required(connector, identity, wire_generation_mismatch));

  auto stale = connector;
  stale.blocked_info.maneuver_target_fresh = false;
  EXPECT_FALSE(lateral_required(stale, identity, wire));
  auto incomplete = connector;
  incomplete.blocked_info.maneuver_transaction_incomplete = false;
  EXPECT_FALSE(lateral_required(incomplete, identity, wire));
  auto safety_rejected = connector;
  safety_rejected.published_lateral_safety_rejected = true;
  EXPECT_FALSE(lateral_required(safety_rejected, identity, wire));
  auto tracking_stop = connector;
  tracking_stop.blocked_info.maneuver_transaction_tracking_stop_active = true;
  EXPECT_FALSE(lateral_required(tracking_stop, identity, wire));
  auto constant_profile = connector;
  constant_profile.lateral_offsets = {3.36, 3.36, 3.36};
  EXPECT_FALSE(lateral_required(constant_profile, identity, wire));
  auto invalid_profile = connector;
  invalid_profile.lateral_offsets[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(lateral_required(invalid_profile, identity, wire));
  EXPECT_FALSE(
      lateral_required(connector, identity, wire, 33U, 7U, "d2", -1, false));
}

TEST(ReferenceOverrideContract,
     CurrentDTrackingStopClearsTypedPassDirectionOnlyForExactStopContract) {
  using overtake_planner::BehaviorMode;
  using overtake_planner::CandidateType;

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.frenet.d = 1.25;

  overtake_planner::PlannerOutput stop;
  stop.mode = BehaviorMode::FOLLOW_BLOCKED;
  stop.selected = CandidateType::SAFE_STOP;
  stop.raw_selected = CandidateType::SAFE_STOP;
  // tracking専用SAFE_STOPは一般safe-stop latchとは独立で、実Core出力ではfalse。
  stop.safe_stop_triggered = false;
  stop.active_override = true;
  stop.selected_lateral_profile_safety_verified = true;
  stop.lateral_stop_inputs_complete = true;
  stop.lateral_tracking_authorized_during_stop = true;
  stop.safe_stop_v_mps = 0.20;
  stop.lateral_offsets = {1.25, 1.25};
  stop.speed_caps = {0.20, 0.20};
  stop.longitudinal_offsets_m = {0.0, 0.8};
  stop.blocked_info.start_grid_target_active = true;
  stop.blocked_info.maneuver_transaction_prepared = true;
  stop.blocked_info.maneuver_transaction_tracking_release_pending = true;
  stop.blocked_info.maneuver_transaction_tracking_stop_active = true;
  stop.blocked_info.maneuver_target_latched = true;
  stop.blocked_info.maneuver_target_id = "d2";
  stop.blocked_info.maneuver_transaction_pass_type = CandidateType::PASS_RIGHT;

  const bool exact_stop =
      overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
          stop, ego, true, true, false, 0.20,
          "maneuver_transaction_tracking_stop");
  ASSERT_TRUE(exact_stop);
  EXPECT_EQ(overtake_planner::authoritativePlanPassDirection(stop, exact_stop),
            0);

  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, true, 0.20, "maneuver_transaction_tracking_stop"));
  EXPECT_TRUE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, false, 0.10, "release_pending_safe_cycles"));
  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, false, 0.21,
      "maneuver_transaction_tracking_stop"));
  stop.raw_selected = CandidateType::FOLLOW;
  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, false, 0.20,
      "maneuver_transaction_tracking_stop"));
  stop.raw_selected = CandidateType::SAFE_STOP;
  stop.lateral_offsets.back() += 0.01;
  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, false, 0.20,
      "maneuver_transaction_tracking_stop"));

  stop.lateral_offsets.back() = ego.frenet.d;
  stop.selected = CandidateType::FOLLOW;
  stop.raw_selected = CandidateType::SAFE_STOP;
  stop.safe_stop_triggered = false;
  stop.lateral_tracking_authorized_during_stop = false;
  stop.blocked_info.maneuver_transaction_tracking_release_pending = false;
  const bool stopped_follow =
      overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
          stop, ego, true, true, false, 0.01, "release_pending_safe_cycles");
  EXPECT_TRUE(stopped_follow);
  EXPECT_EQ(
      overtake_planner::authoritativePlanPassDirection(stop, stopped_follow),
      0);
  stop.speed_caps.back() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      stop, ego, true, true, false, 0.01, "release_pending_safe_cycles"));

  stop.speed_caps.back() = 0.20;
  stop.mode = BehaviorMode::OVERTAKE_RIGHT;
  stop.selected = CandidateType::PASS_RIGHT;
  stop.tracking_release_pass_warmup = true;
  stop.tracking_release_token = 9U;
  const bool pass_is_stop =
      overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
          stop, ego, true, true, false, 0.20,
          "maneuver_transaction_tracking_stop");
  EXPECT_FALSE(pass_is_stop);
  EXPECT_EQ(
      overtake_planner::authoritativePlanPassDirection(stop, pass_is_stop), -1);
}

TEST(ReferenceOverrideContract,
     TrackingStopBootstrapCanonicalizesSafeStopAndFollowToOneGeneration) {
  using overtake_planner::BehaviorMode;
  using overtake_planner::CandidateType;
  using overtake_planner::ReferenceOverrideWireKind;

  overtake_planner::PlannerOutput safe_stop;
  safe_stop.mode = BehaviorMode::FOLLOW_BLOCKED;
  safe_stop.selected = CandidateType::SAFE_STOP;
  safe_stop.raw_selected = CandidateType::SAFE_STOP;
  safe_stop.active_override = true;
  safe_stop.selected_lateral_profile_safety_verified = true;
  safe_stop.lateral_stop_inputs_complete = true;
  safe_stop.lateral_tracking_authorized_during_stop = true;
  safe_stop.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE;
  safe_stop.safe_stop_v_mps = 0.20;
  safe_stop.lateral_offsets = {3.354, 3.354};
  safe_stop.speed_caps = {0.20, 0.20};
  safe_stop.longitudinal_offsets_m = {0.0, 0.8};
  safe_stop.blocked_info.start_grid_target_active = true;
  safe_stop.blocked_info.maneuver_transaction_prepared = true;
  safe_stop.blocked_info.maneuver_transaction_tracking_release_pending = true;
  safe_stop.blocked_info.maneuver_transaction_tracking_stop_active = true;
  safe_stop.blocked_info.maneuver_target_latched = true;
  safe_stop.blocked_info.maneuver_target_id = "d2";
  safe_stop.blocked_info.maneuver_transaction_pass_type =
      CandidateType::PASS_RIGHT;

  ASSERT_TRUE(overtake_planner::isTrackingStopBootstrapOutput(safe_stop));
  const auto stop_wire =
      overtake_planner::makeReferenceOverrideWirePayload(safe_stop, 940U);
  ASSERT_EQ(stop_wire.kind,
            ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(stop_wire.mode_id, static_cast<int>(BehaviorMode::FOLLOW_BLOCKED));
  EXPECT_EQ(stop_wire.data,
            (std::vector<float>{1.0F, 1.0F, 2.0F, 3.354F, 3.354F, 0.20F, 0.20F,
                                0.0F, 0.8F, 4.0F, 940.0F, 2.0F}));
  EXPECT_TRUE(overtake_planner::trackingStopLateralWireMatchesAuthorization(
      safe_stop, stop_wire, 940U));
  EXPECT_FALSE(overtake_planner::trackingStopLateralWireMatchesAuthorization(
      safe_stop, stop_wire, 939U));

  auto stopped_follow = safe_stop;
  stopped_follow.selected = CandidateType::FOLLOW;
  stopped_follow.raw_selected = CandidateType::SAFE_STOP;
  stopped_follow.safe_stop_triggered = false;
  stopped_follow.lateral_tracking_authorized_during_stop = false;
  stopped_follow.blocked_info.maneuver_transaction_tracking_release_pending =
      false;
  // 横認可のない一段目bootstrapだけはspeed-onlyを維持する。typed planが
  // current-d横軌道を認可していないため、PPのbaseline追従を横証明にしない。
  stopped_follow.lateral_offsets = {3.361, 3.361, 3.361};
  stopped_follow.speed_caps = {0.31, 0.46, 0.60};
  stopped_follow.longitudinal_offsets_m = {0.0, 0.2, 0.6};
  ASSERT_TRUE(overtake_planner::isTrackingStopBootstrapOutput(stopped_follow));
  const auto follow_wire =
      overtake_planner::makeReferenceOverrideWirePayload(stopped_follow, 941U);
  ASSERT_EQ(follow_wire.kind, ReferenceOverrideWireKind::SPEED_ONLY_V2);
  EXPECT_FALSE(overtake_planner::trackingStopLateralWireMatchesAuthorization(
      stopped_follow, follow_wire, 941U));
  EXPECT_FALSE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      stop_wire, follow_wire));

  // 横認可フラグだけがtrueでも、PPへ渡せる正の空間horizonが無ければ
  // 横payloadを出さず、typed plan側も横trajectoryを認可してはならない。
  auto untrackable_stop = safe_stop;
  untrackable_stop.longitudinal_offsets_m = {0.0, 0.0};
  const auto untrackable_wire =
      overtake_planner::makeReferenceOverrideWirePayload(untrackable_stop,
                                                         942U);
  EXPECT_NE(untrackable_wire.kind,
            ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_NE(untrackable_wire.kind,
            ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3);
  EXPECT_FALSE(overtake_planner::trackingStopLateralWireMatchesAuthorization(
      untrackable_stop, untrackable_wire, 942U));

  for (const auto &unverified_stop : {
           [&safe_stop]() {
             auto value = safe_stop;
             value.selected_lateral_profile_safety_verified = false;
             return value;
           }(),
           [&safe_stop]() {
             auto value = safe_stop;
             value.lateral_stop_inputs_complete = false;
             return value;
           }(),
           [&safe_stop]() {
             auto value = safe_stop;
             value.lateral_tracking_authorized_during_stop = false;
             return value;
           }(),
           [&safe_stop]() {
             auto value = safe_stop;
             value.selected = CandidateType::FOLLOW;
             EXPECT_TRUE(
                 overtake_planner::isTrackingStopStructuralIntent(value));
             return value;
           }(),
           [&safe_stop]() {
             auto value = safe_stop;
             value.lateral_offsets.back() += 0.05;
             return value;
           }(),
           [&safe_stop]() {
             auto value = safe_stop;
             value.mode = BehaviorMode::ABORT_RECOVERY;
             value.selected_lateral_profile_safety_verified = false;
             return value;
           }(),
       }) {
    const auto unverified_wire =
        overtake_planner::makeReferenceOverrideWirePayload(unverified_stop,
                                                           942U);
    EXPECT_NE(unverified_wire.kind,
              ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4);
    EXPECT_NE(unverified_wire.kind,
              ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3);
    EXPECT_FALSE(overtake_planner::trackingStopLateralWireMatchesAuthorization(
        unverified_stop, unverified_wire, 942U));
  }

  // STOP解除後のFOLLOWは評価済み横列を持つ通常v4へ戻り、別generationの
  // ControllerTrackingStatusを要求する。
  stopped_follow.blocked_info.maneuver_transaction_tracking_stop_active = false;
  stopped_follow.raw_selected = CandidateType::FOLLOW;
  EXPECT_FALSE(overtake_planner::isTrackingStopBootstrapOutput(stopped_follow));
  const auto released_wire =
      overtake_planner::makeReferenceOverrideWirePayload(stopped_follow, 943U);
  EXPECT_EQ(released_wire.kind,
            ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_FALSE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      follow_wire, released_wire));
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
     SafetyEvaluatedStationaryPassCanAccelerateFromRestWithFreshInputs) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.pass_speed_cap_mps = 5.0;
  config.pass_assumed_accel_mps2 = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.0;
  auto stopped = makeOpponent(frame, 10.0, -0.6);
  stopped.id = "d2";
  stopped.stamp_sec = 0.1;
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  // 相対速度0の厳密停止からはTTC分類を作らないが、観測対象そのものは
  // freshな低速前走車としてPASSの全車両評価へ含まれている。
  EXPECT_TRUE(output.blocked_info.front_vehicle_low_speed);
  EXPECT_TRUE(output.blocked_info.pass_acceleration_allowed);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(output.selected == overtake_planner::CandidateType::PASS_LEFT ||
              output.selected == overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_FALSE(output.speed_caps.empty());
  for (const double speed_cap_mps : output.speed_caps) {
    EXPECT_NEAR(speed_cap_mps, config.pass_speed_cap_mps, 1.0e-9);
  }

  // 同じ停止対象でもfreshness/MPC契約が無ければ、加速後s(t)を保証できない。
  // 別Coreで未認可経路を確認し、単なる「停止車なら加速」にはしない。
  overtake_planner::OvertakePlannerCore stale_core(frame, config);
  const auto stale = stale_core.update(0.1, ego, {stopped});
  EXPECT_FALSE(stale.blocked_info.pass_acceleration_allowed);
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
  // このテストは静的gapではなく実回廊評価を検証する。1 mで1.4 m横移動する
  // 旧fixtureはactive controller契約に反するため、同じ目標dへC2で到達可能な
  // 距離を与える。
  config.prepare_distance_m = 6.0;
  config.min_pass_gap_m = 1.8;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.9;
  config.min_ellipse_h = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 12.0, 0.0);
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped});

  EXPECT_FALSE(output.blocked_info.can_pass_left);
  EXPECT_FALSE(output.blocked_info.can_pass_right);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible)
      << output.blocked_info.pass_left_candidate_reject_reason;
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.blocked_info.nearest_id, stopped.id);
  EXPECT_GT(output.blocked_info.pass_left_candidate_target_d_m, ego.frenet.d);
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_TRUE(output.active_override);
}

TEST(CandidateBuilder, RejectsPassWhoseTargetCannotReachFutureCorridor) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    const bool narrows_before_target = point.s >= 8.0;
    corridor.push_back({point.s, narrows_before_target ? -1.0 : -3.0,
                        narrows_before_target ? 1.0 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 2;
  config.horizon_dt_sec = 0.1;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.5;
  config.left_offset_m = 1.0;
  config.prepare_distance_m = 4.0;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 10.0, 0.0);
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;

  auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {opponent});

  // controller実行契約に合わせてhorizon自体が伸びても、回廊preflightは
  // 目標dまでの全区間を独立に拒否する。
  ASSERT_GT(candidate.s.back(), 6.0);
  EXPECT_FALSE(candidate.pass_target_corridor_valid);
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "pass_target_unreachable");
}

TEST(CandidateBuilder,
     RejectsLocalizedPassWhoseFullOffsetCannotReachFutureCorridor) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    const bool narrows_before_target = point.s >= 8.0;
    corridor.push_back({point.s, narrows_before_target ? -1.0 : -3.0,
                        narrows_before_target ? 1.0 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 2;
  config.horizon_dt_sec = 0.1;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.5;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 10.0, 0.0);
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;
  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  profile.anchor_s_m = ego.frenet.s;
  profile.avoid_start_s_m = ego.frenet.s;
  profile.full_offset_start_s_m = ego.frenet.s + 4.0;
  profile.full_offset_end_s_m = profile.full_offset_start_s_m + 1.0;
  profile.merge_end_s_m = profile.full_offset_end_s_m + 2.0;
  profile.start_d_m = ego.frenet.d;
  profile.target_d_m = 1.0;

  auto candidate =
      builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT, ego,
                            blocked, {opponent}, &profile);

  ASSERT_GT(candidate.s.back(), 6.0);
  EXPECT_FALSE(candidate.pass_target_corridor_valid);
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "pass_target_unreachable");
}

TEST(CandidateBuilder,
     MinimumClearancePassAvoidsLegacyOffsetBeyondDrivableCorridor) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -2.0, 4.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.pass_target_policy = "minimum_clearance";
  config.d_min_m = -2.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.10;
  config.left_offset_m = 2.10;
  config.right_offset_m = -2.10;
  config.prepare_distance_m = 8.0;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  const auto ego = makeEgo(frame, 5.0, 3.2968);
  const auto opponent = makeOpponent(frame, 12.0, 1.9779);
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;

  auto right = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {opponent});
  const double required_gap = 1.8 * std::sqrt(1.2) + 0.10;
  EXPECT_NEAR(right.planned_target_d_m, opponent.frenet.d - required_gap,
              1.0e-6);
  EXPECT_GT(right.planned_target_d_m, -1.90);
  EXPECT_TRUE(right.pass_target_corridor_valid);
  EXPECT_TRUE(evaluator.evaluate(right, {}));

  const auto left = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {opponent});
  EXPECT_FALSE(left.pass_target_corridor_valid);
  EXPECT_NEAR(left.planned_target_d_m, opponent.frenet.d + required_gap,
              1.0e-6);

  auto legacy_config = config;
  legacy_config.pass_target_policy = "legacy_fixed_offset";
  overtake_planner::CandidateBuilder legacy_builder(frame, legacy_config);
  const auto legacy_right = legacy_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {opponent});
  EXPECT_NEAR(legacy_right.planned_target_d_m, -2.10, 1.0e-9);
  EXPECT_FALSE(legacy_right.pass_target_corridor_valid);
}

TEST(CandidateBuilder,
     EarlyStationaryParallelFallbackHoldsLateralWithoutAcceleration) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  config.recovery_v_max_mps = 4.0;
  config.reentry_hold_v_max_mps = 2.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto ego = makeEgo(frame, 5.0, 1.20);
  overtake_planner::BlockedInfo blocked;
  blocked.early_stationary_parallel_pass_hold_lateral = true;

  const auto recovery = builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, blocked, {});

  ASSERT_FALSE(recovery.d.empty());
  for (const double d_m : recovery.d) {
    EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(recovery.v_ref.empty());
  for (const double speed_cap_mps : recovery.v_ref) {
    EXPECT_LE(speed_cap_mps, config.reentry_hold_v_max_mps);
    EXPECT_LT(speed_cap_mps, ego.v);
  }
}

TEST(CandidateBuilder,
     PreparedStationaryHoldFreezesOnlySubMillimeterLocalizationNoise) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);

  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  profile.start_d_m = 1.20;
  profile.pass_safety_approved_once = false;
  overtake_planner::BlockedInfo blocked;
  blocked.early_stationary_parallel_pass_hold_lateral = true;

  auto within_noise = makeEgo(frame, 5.0, 1.20025);
  within_noise.v = 0.005;
  const auto held =
      builder.makeCandidate(overtake_planner::CandidateType::RECOVERY,
                            within_noise, blocked, {}, &profile);
  ASSERT_FALSE(held.d.empty());
  EXPECT_TRUE(std::all_of(held.d.begin(), held.d.end(), [&profile](double d) {
    return std::abs(d - profile.start_d_m) <= 1.0e-9;
  }));

  // sub-mmでも現在の相手楕円余裕がmin_h+0.10以下なら固定しない。
  auto near_opponent = makeOpponent(frame, 5.0, 1.47);
  near_opponent.id = "d2";
  const auto opponent_guard_replanned =
      builder.makeCandidate(overtake_planner::CandidateType::RECOVERY,
                            within_noise, blocked, {near_opponent}, &profile);
  ASSERT_FALSE(opponent_guard_replanned.d.empty());
  EXPECT_TRUE(
      std::all_of(opponent_guard_replanned.d.begin(),
                  opponent_guard_replanned.d.end(), [&within_noise](double d) {
                    return std::abs(d - within_noise.frenet.d) <= 1.0e-9;
                  }));

  auto wall_profile = profile;
  wall_profile.start_d_m = -2.89975;
  auto near_wall_ego = makeEgo(frame, 5.0, -2.8995);
  near_wall_ego.v = 0.005;
  const auto wall_guard_replanned =
      builder.makeCandidate(overtake_planner::CandidateType::RECOVERY,
                            near_wall_ego, blocked, {}, &wall_profile);
  ASSERT_FALSE(wall_guard_replanned.d.empty());
  EXPECT_TRUE(
      std::all_of(wall_guard_replanned.d.begin(), wall_guard_replanned.d.end(),
                  [&near_wall_ego](double d) {
                    return std::abs(d - near_wall_ego.frenet.d) <= 1.0e-9;
                  }));

  auto outside_noise = makeEgo(frame, 5.0, 1.20051);
  outside_noise.v = 0.005;
  const auto replanned =
      builder.makeCandidate(overtake_planner::CandidateType::RECOVERY,
                            outside_noise, blocked, {}, &profile);
  ASSERT_FALSE(replanned.d.empty());
  EXPECT_TRUE(std::all_of(
      replanned.d.begin(), replanned.d.end(), [&outside_noise](double d) {
        return std::abs(d - outside_noise.frenet.d) <= 1.0e-9;
      }));
}

TEST(CandidateBuilder,
     StartGridAnchorCorrectionAlwaysStartsAtMeasuredLateralPosition) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -2.5, 2.5});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.2;
  config.start_grid_hold_correction_distance_m = 3.0;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  auto opponent = makeOpponent(frame, 15.0, -1.0);
  opponent.id = "d2";
  opponent.v = 3.0;
  opponent.vx = 3.0;
  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_target_index = 0;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_hold_target_d_m = 1.20;
  blocked.corner_abs_curvature = 0.0;
  blocked.future_abs_curvature = 0.0;

  for (const double error_m : {0.00005, 0.00025, 0.0015, 0.0021}) {
    auto ego =
        makeEgo(frame, 5.0, blocked.start_grid_hold_target_d_m + error_m);
    ego.v = 3.0;
    auto candidate = builder.makeCandidate(
        overtake_planner::CandidateType::FOLLOW, ego, blocked, {opponent});

    ASSERT_FALSE(candidate.d.empty());
    EXPECT_NEAR(candidate.d.front(), ego.frenet.d, 1.0e-12);
    EXPECT_LT(candidate.d.back(), ego.frenet.d);
    EXPECT_GE(candidate.d.back(), blocked.start_grid_hold_target_d_m - 1.0e-9);
    EXPECT_TRUE(candidate.pass_target_corridor_valid);
    EXPECT_TRUE(candidate.controller_tracking_profile_valid);
    overtake_planner::PredictedOpponent prediction;
    prediction.id = opponent.id;
    for (const double t_sec : candidate.t) {
      const double opponent_s =
          frame.wrapS(opponent.frenet.s + opponent.v * t_sec);
      const auto point = frame.frenetToCartesian(opponent_s, opponent.frenet.d);
      prediction.t.push_back(t_sec);
      prediction.x.push_back(point.x);
      prediction.y.push_back(point.y);
      prediction.s.push_back(opponent_s);
      prediction.d.push_back(opponent.frenet.d);
    }
    EXPECT_TRUE(evaluator.evaluate(candidate, {prediction}));
  }
}

TEST(CandidateBuilder,
     StartGridExactAnchorRecoveryDoesNotRequireLateralControllerHorizon) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  // 低速時に旧6秒上限では必要なPP空間horizonへ届かない境界を検証する。
  // runtimeの20秒契約とは分け、短い上限をこのfixtureだけに固定する。
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_hold_target_d_m = 1.20;
  blocked.corner_abs_curvature = 0.0;
  blocked.future_abs_curvature = 0.0;

  for (const double speed_mps : {0.0, 0.5, 3.0}) {
    auto ego = makeEgo(frame, 5.0, blocked.start_grid_hold_target_d_m);
    ego.v = speed_mps;
    const auto exact = builder.makeCandidate(
        overtake_planner::CandidateType::RECOVERY, ego, blocked, {});
    ASSERT_FALSE(exact.d.empty());
    EXPECT_TRUE(std::all_of(exact.d.begin(), exact.d.end(), [&](double d_m) {
      return std::abs(d_m - ego.frenet.d) <= 1.0e-12;
    }));
    EXPECT_NEAR(exact.required_controller_spatial_horizon_m, 0.0, 1.0e-12);
    EXPECT_TRUE(exact.controller_tracking_profile_valid);
  }

  for (const double speed_mps : {0.0, 0.5}) {
    auto drifted_ego = makeEgo(frame, 5.0, 1.19);
    drifted_ego.v = speed_mps;
    const auto drifted = builder.makeCandidate(
        overtake_planner::CandidateType::RECOVERY, drifted_ego, blocked, {});
    ASSERT_FALSE(drifted.d.empty());
    EXPECT_NEAR(drifted.d.front(), drifted_ego.frenet.d, 1.0e-12);
    EXPECT_FALSE(drifted.controller_tracking_profile_valid);
  }

  auto moving_ego = makeEgo(frame, 5.0, 1.19);
  moving_ego.v = 3.0;
  const auto moving = builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, moving_ego, blocked, {});
  EXPECT_TRUE(moving.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     StartGridStationarySubMillimeterNoiseHoldsMeasuredLateralPosition) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_hold_target_d_m = 1.20;
  blocked.corner_abs_curvature = 0.0;
  blocked.future_abs_curvature = 0.0;

  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  profile.start_d_m = blocked.start_grid_hold_target_d_m;
  profile.target_d_m = 1.8;

  // Gate 2 bagの失敗周期を再現する。0.02 m/sをわずかに超えただけで
  // current-d holdを解除せず、PASS未commit中のFOLLOW/RECOVERYを同じ実測dで
  // SafetyEvaluatorへ渡せることを確認する。
  for (const auto type : {overtake_planner::CandidateType::FOLLOW,
                          overtake_planner::CandidateType::RECOVERY}) {
    auto ego = makeEgo(frame, 5.0, 1.196060);
    ego.v = 0.02163;
    const auto hold = builder.makeCandidate(type, ego, blocked, {}, &profile);

    ASSERT_FALSE(hold.d.empty());
    EXPECT_TRUE(std::all_of(hold.d.begin(), hold.d.end(), [&ego](double d_m) {
      return std::isfinite(d_m) && std::abs(d_m - ego.frenet.d) <= 1.0e-12;
    }));
    EXPECT_NEAR(hold.required_controller_spatial_horizon_m, 0.0, 1.0e-12);
    EXPECT_TRUE(hold.pass_target_corridor_valid);
    EXPECT_TRUE(hold.controller_tracking_profile_valid);
  }

  // current-d保持はsafe-stop crawl以下だけ。通常走行速度では既存どおり
  // anchorへ滑らかに補正し、初期横位置を無期限に追従しない。
  auto moving_ego = makeEgo(frame, 5.0, 1.196060);
  moving_ego.v = config.safe_stop_v_mps + 0.01;
  const auto correcting =
      builder.makeCandidate(overtake_planner::CandidateType::FOLLOW, moving_ego,
                            blocked, {}, &profile);
  ASSERT_FALSE(correcting.d.empty());
  EXPECT_NEAR(correcting.d.front(), moving_ego.frenet.d, 1.0e-12);
  EXPECT_GT(correcting.d.back(), moving_ego.frenet.d);
  EXPECT_LE(correcting.d.back(), blocked.start_grid_hold_target_d_m + 1.0e-9);
}

TEST(CandidateBuilder,
     StartGridUncommittedCurrentDHoldRequiresExecutableControllerArc) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_uncommitted_hold_active = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.start_grid_hold_target_d_m = 1.2;
  blocked.corner_abs_curvature = 0.0;
  blocked.future_abs_curvature = 0.0;

  auto stopped_ego = makeEgo(frame, 5.0, 1.2);
  stopped_ego.v = 0.0;
  const auto stopped_hold = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, stopped_ego, blocked, {});
  ASSERT_FALSE(stopped_hold.d.empty());
  EXPECT_TRUE(std::all_of(
      stopped_hold.d.begin(), stopped_hold.d.end(), [&stopped_ego](double d_m) {
        return std::isfinite(d_m) &&
               std::abs(d_m - stopped_ego.frenet.d) <= 1.0e-12;
      }));
  EXPECT_GE(stopped_hold.required_controller_spatial_horizon_m,
            config.lateral_override_lookahead_min_distance_m);
  EXPECT_LT(stopped_hold.longitudinal_offsets_m.back(),
            stopped_hold.required_controller_spatial_horizon_m);
  EXPECT_FALSE(stopped_hold.controller_tracking_profile_valid);

  auto moving_ego = makeEgo(frame, 5.0, 1.2);
  moving_ego.v = 3.0;
  const auto moving_hold = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, moving_ego, blocked, {});
  ASSERT_FALSE(moving_hold.d.empty());
  EXPECT_TRUE(std::all_of(
      moving_hold.d.begin(), moving_hold.d.end(), [&moving_ego](double d_m) {
        return std::isfinite(d_m) &&
               std::abs(d_m - moving_ego.frenet.d) <= 1.0e-12;
      }));
  EXPECT_GE(moving_hold.required_controller_spatial_horizon_m,
            config.lateral_override_lookahead_min_distance_m);
  EXPECT_GE(moving_hold.longitudinal_offsets_m.back() + 1.0e-6,
            moving_hold.required_controller_spatial_horizon_m);
  EXPECT_TRUE(moving_hold.controller_tracking_profile_valid);
}

TEST(CandidateTrajectory, ControllerSpatialHorizonProofUsesExactBoundary) {
  overtake_planner::CandidateTrajectory candidate;
  candidate.safety_evaluated = true;
  candidate.feasible = true;
  candidate.longitudinal_profile_valid = true;
  candidate.controller_tracking_profile_valid = true;
  candidate.desired_path_trackable = true;
  candidate.pure_pursuit_command_trackable = true;
  candidate.required_controller_spatial_horizon_m = 0.50;
  candidate.d = {0.0, 0.0};
  candidate.v_ref = {0.20, 0.20};
  candidate.longitudinal_offsets_m = {0.0, 0.50};

  EXPECT_TRUE(overtake_planner::hasControllerSpatialHorizonProof(candidate));

  for (const double captured_endpoint_m : {0.326648742, 0.315290004}) {
    candidate.longitudinal_offsets_m = {0.0, captured_endpoint_m};
    EXPECT_FALSE(overtake_planner::hasControllerSpatialHorizonProof(candidate));
  }

  candidate.longitudinal_offsets_m = {0.0, 0.50};
  candidate.longitudinal_offsets_m.back() = std::nextafter(0.50, 0.0);
  EXPECT_FALSE(overtake_planner::hasControllerSpatialHorizonProof(candidate));

  candidate.longitudinal_offsets_m = {0.0, 0.50};
  candidate.v_ref.back() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(overtake_planner::hasControllerSpatialHorizonProof(candidate));
}

TEST(CandidateBuilder,
     StopHoldAuthorityContextsRequireControllerSpatialHorizon) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);
  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.2;
  auto stationary = makeOpponent(frame, 100.0, 0.0);
  stationary.v = 0.0;
  stationary.vx = 0.0;
  stationary.vy = 0.0;

  for (const auto type : {overtake_planner::CandidateType::FOLLOW,
                          overtake_planner::CandidateType::YIELD_BEHIND,
                          overtake_planner::CandidateType::RECOVERY}) {
    const auto ordinary = builder.makeCandidate(type, ego, {}, {});
    EXPECT_DOUBLE_EQ(ordinary.required_controller_spatial_horizon_m, 0.0);

    overtake_planner::BlockedInfo stop_hold;
    stop_hold.reentry_hold_active = true;
    stop_hold.opponent_prediction_inputs_complete = true;
    const auto held = builder.makeCandidate(type, ego, stop_hold, {stationary});
    EXPECT_GE(held.required_controller_spatial_horizon_m, 0.50);
    ASSERT_FALSE(held.longitudinal_offsets_m.empty());
    EXPECT_GE(held.longitudinal_offsets_m.back(),
              held.required_controller_spatial_horizon_m);
    EXPECT_TRUE(held.controller_tracking_profile_valid);

    auto moving = stationary;
    moving.v = 0.5;
    moving.vx = 0.5;
    const auto moving_hold =
        builder.makeCandidate(type, ego, stop_hold, {moving});
    ASSERT_FALSE(moving_hold.longitudinal_offsets_m.empty());
    EXPECT_LT(moving_hold.longitudinal_offsets_m.back(),
              moving_hold.required_controller_spatial_horizon_m);
    EXPECT_FALSE(moving_hold.controller_tracking_profile_valid);

    stop_hold.opponent_prediction_inputs_complete = false;
    const auto incomplete_hold =
        builder.makeCandidate(type, ego, stop_hold, {stationary});
    ASSERT_FALSE(incomplete_hold.longitudinal_offsets_m.empty());
    EXPECT_LT(incomplete_hold.longitudinal_offsets_m.back(),
              incomplete_hold.required_controller_spatial_horizon_m);
    EXPECT_FALSE(incomplete_hold.controller_tracking_profile_valid);
  }
}

TEST(CandidateBuilder,
     StartGridInvalidAnchorFailsClosedWithoutCurrentDFallback) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.2;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.0);
  ego.v = 2.0;
  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_hold_target_d_m = std::numeric_limits<double>::quiet_NaN();

  const auto nan_anchor = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {});
  EXPECT_FALSE(nan_anchor.pass_target_corridor_valid);
  EXPECT_FALSE(nan_anchor.controller_tracking_profile_valid);

  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = 1.2;
  const auto nan_anchor_with_attack_follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {});
  EXPECT_FALSE(nan_anchor_with_attack_follow.pass_target_corridor_valid);
  EXPECT_FALSE(nan_anchor_with_attack_follow.controller_tracking_profile_valid);
  blocked.attack_follow_hold_pass_side = false;

  blocked.start_grid_hold_target_d_m = 2.9;
  const auto outside_anchor = builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, blocked, {});
  EXPECT_FALSE(outside_anchor.pass_target_corridor_valid);
  EXPECT_FALSE(outside_anchor.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     StartGridAnchorRejectsFutureCorridorAndCurvatureBeyondPublishedArc) {
  auto narrowing_frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : narrowing_frame.reference()) {
    const bool narrow = point.s >= 7.0;
    corridor.push_back({point.s, -3.0, narrow ? 1.1 : 3.0});
  }
  narrowing_frame.setCorridor(corridor);

  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.start_grid_hold_correction_distance_m = 3.0;
  overtake_planner::CandidateBuilder narrowing_builder(narrowing_frame, config);
  auto ego = makeEgo(narrowing_frame, 5.0, 0.9);
  ego.v = 2.0;
  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_follow_hold_lateral = true;
  blocked.start_grid_hold_target_d_m = 1.2;
  blocked.corner_abs_curvature = 0.0;
  blocked.future_abs_curvature = 0.0;
  const auto corridor_rejected = narrowing_builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {});
  EXPECT_FALSE(corridor_rejected.pass_target_corridor_valid);
  EXPECT_FALSE(corridor_rejected.controller_tracking_profile_valid);

  // 未commit current-d例外は将来corridorへclampして横移動を捏造しない。
  // 4種類のhold候補すべてでd列を実測値に固定し、入らない回廊なら不成立にする。
  auto uncommitted_ego = makeEgo(narrowing_frame, 5.0, 1.2);
  uncommitted_ego.v = 2.0;
  blocked.start_grid_hold_target_d_m = uncommitted_ego.frenet.d;
  blocked.start_grid_uncommitted_hold_active = true;
  for (const auto type : {overtake_planner::CandidateType::FOLLOW,
                          overtake_planner::CandidateType::RECOVERY,
                          overtake_planner::CandidateType::YIELD_BEHIND,
                          overtake_planner::CandidateType::SAFE_STOP}) {
    const auto exact_current_d =
        narrowing_builder.makeCandidate(type, uncommitted_ego, blocked, {});
    ASSERT_FALSE(exact_current_d.d.empty());
    EXPECT_TRUE(std::all_of(exact_current_d.d.begin(), exact_current_d.d.end(),
                            [&uncommitted_ego](double d_m) {
                              return std::isfinite(d_m) &&
                                     std::abs(d_m - uncommitted_ego.frenet.d) <=
                                         1.0e-9;
                            }));
    EXPECT_FALSE(exact_current_d.pass_target_corridor_valid);
    EXPECT_FALSE(exact_current_d.controller_tracking_profile_valid);
  }
  blocked.start_grid_uncommitted_hold_active = false;

  const auto corner_frame = makeFutureCornerFrame();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  config.v_passthrough_mps = 0.2;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.attack_follow_max_steering_angle_rad = 0.10;
  overtake_planner::CandidateBuilder corner_builder(corner_frame, config);
  auto slow_ego = makeEgo(corner_frame, 5.0, 0.0);
  slow_ego.v = 0.2;
  blocked.start_grid_hold_target_d_m = 0.15;
  const auto curvature_rejected = corner_builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, slow_ego, blocked, {});
  ASSERT_FALSE(curvature_rejected.longitudinal_offsets_m.empty());
  EXPECT_LT(curvature_rejected.longitudinal_offsets_m.back(), 2.0);
  EXPECT_TRUE(curvature_rejected.pass_target_corridor_valid);
  EXPECT_FALSE(curvature_rejected.controller_tracking_profile_valid);
}

TEST(CandidateBuilder, SupervisorV2AbortSafeStopKeepsCurrentLateralPosition) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.8);
  overtake_planner::BlockedInfo blocked;
  blocked.abort_safe_stop_hold_lateral = true;
  const auto stop = builder.makeCandidate(
      overtake_planner::CandidateType::SAFE_STOP, ego, blocked, {});

  ASSERT_FALSE(stop.d.empty());
  EXPECT_TRUE(std::all_of(stop.d.begin(), stop.d.end(), [&ego](double d) {
    return std::isfinite(d) && std::abs(d - ego.frenet.d) <= 1.0e-9;
  }));
  ASSERT_FALSE(stop.v_ref.empty());
  EXPECT_TRUE(std::all_of(stop.v_ref.begin(), stop.v_ref.end(),
                          [&config](double speed_mps) {
                            return std::isfinite(speed_mps) &&
                                   speed_mps <= config.safe_stop_v_mps + 1.0e-9;
                          }));
}

TEST(CandidateBuilder,
     SupervisorV2CenteringProfileRejectsOnlyCorridorClampTowardCenter) {
  const auto straight_frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  overtake_planner::CandidateBuilder straight_builder(straight_frame, config);
  for (const double ego_d_m : {-0.8, 0.8}) {
    const auto ego = makeEgo(straight_frame, 5.0, ego_d_m);
    const auto recovery = straight_builder.makeCandidate(
        overtake_planner::CandidateType::RECOVERY, ego, {}, {});
    EXPECT_TRUE(
        straight_builder.centeringProfileMatchesNominal(ego, {}, recovery));
  }

  auto narrowing_frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : narrowing_frame.reference()) {
    corridor.push_back({point.s, -3.0, point.s <= 6.0 ? 3.0 : 0.5});
  }
  narrowing_frame.setCorridor(corridor);
  overtake_planner::CandidateBuilder narrowing_builder(narrowing_frame, config);
  const auto ego = makeEgo(narrowing_frame, 5.0, 0.8);
  const auto clamped_recovery = narrowing_builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, {}, {});
  overtake_planner::SafetyEvaluator evaluator(narrowing_frame, config);
  auto evaluated_recovery = clamped_recovery;
  EXPECT_TRUE(evaluator.evaluate(evaluated_recovery, {}));
  EXPECT_FALSE(narrowing_builder.centeringProfileMatchesNominal(
      ego, {}, clamped_recovery));

  auto already_narrow_frame = makeStraightFrame();
  corridor.clear();
  for (const auto &point : already_narrow_frame.reference()) {
    corridor.push_back({point.s, -3.0, 0.5});
  }
  already_narrow_frame.setCorridor(corridor);
  overtake_planner::CandidateBuilder already_narrow_builder(
      already_narrow_frame, config);
  const auto outside_ego = makeEgo(already_narrow_frame, 5.0, 0.8);
  const auto discontinuous_recovery = already_narrow_builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, outside_ego, {}, {});
  EXPECT_FALSE(already_narrow_builder.centeringProfileMatchesNominal(
      outside_ego, {}, discontinuous_recovery));
}

TEST(CandidateBuilder, RecoveryAccelerationIsIncludedInSafetyPrediction) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.recovery_v_max_mps = 4.0;
  config.recovery_assumed_accel_mps2 = 3.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.4);
  ego.v = 0.6;
  const auto recovery = builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, {}, {});

  ASSERT_FALSE(recovery.v_ref.empty());
  ASSERT_EQ(recovery.v_ref.size(), recovery.predicted_speed_mps.size());
  for (std::size_t i = 0; i < recovery.v_ref.size(); ++i) {
    const double t = static_cast<double>(i) * config.horizon_dt_sec;
    const double expected_speed =
        std::min(config.recovery_v_max_mps,
                 ego.v + config.recovery_assumed_accel_mps2 * t);
    EXPECT_DOUBLE_EQ(recovery.v_ref[i], config.recovery_v_max_mps);
    EXPECT_NEAR(recovery.predicted_speed_mps[i], expected_speed, 1.0e-9);
  }
}

TEST(CandidateBuilder, RejectsLocalizedPassWhoseMergeLeavesFutureCorridor) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -3.0, point.s <= 10.0 ? 1.5 : 0.5});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 2;
  config.horizon_dt_sec = 0.1;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.5;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 10.0, 0.0);
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;
  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  profile.anchor_s_m = ego.frenet.s;
  profile.avoid_start_s_m = ego.frenet.s;
  profile.full_offset_start_s_m = 7.0;
  profile.full_offset_end_s_m = 10.0;
  profile.merge_end_s_m = 12.0;
  profile.start_d_m = ego.frenet.d;
  profile.target_d_m = 1.0;

  auto candidate =
      builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT, ego,
                            blocked, {opponent}, &profile);

  ASSERT_GT(candidate.s.back(), 6.0);
  EXPECT_FALSE(candidate.pass_target_corridor_valid);
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "pass_target_unreachable");
}

TEST(OvertakePlannerCore,
     VerifiedCorridorAndSafetyPassedPassPreemptFollowModeHold) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -3.0, 3.0});
  }
  frame.setCorridor(corridor);
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.50;
  config.left_offset_m = 2.10;
  config.right_offset_m = -2.10;
  config.pass_target_lateral_margin_m = 0.10;
  config.prepare_distance_m = 1.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.safety_ellipse_a_m = 1.0;
  // mode-preemptionのfixtureなので、4 m/sで実行不能な2 m超の横断ではなく、
  // 同じSafetyEvaluatorを通るcontroller追従可能な離隔を使う。
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.20;
  config.pass_speed_cap_mps = 5.0;
  config.pass_assumed_accel_mps2 = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  // このtestはmode-holdよりGate 2認可済みPASSが優先されることを検証する。
  // 同速低速のfixtureへ、active controllerが横profileを実行できる前方余裕を
  // 与え、別の物理的不成立を混ぜない。
  auto opponent = makeOpponent(frame, 12.0, 0.0);
  opponent.v = 1.0;
  opponent.vx = 1.0;
  auto complete_stop_input = readyReentryInput();
  complete_stop_input.pure_pursuit_primary_and_fresh = false;
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  complete_stop_input);

  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible)
      << output.blocked_info.pass_left_candidate_reject_reason;
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     StartGridStationaryTargetUsesOnlySafetyEvaluatedPass) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.20;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_EQ(output.blocked_info.start_grid_target_id, "grid_d2");
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(output.selected == overtake_planner::CandidateType::PASS_LEFT ||
              output.selected == overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StartGridAdjacentTargetPrefersExistingSideWhenBothSidesAreEvaluated) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;

  const auto evaluate = [&](double ego_d_m, double target_d_m) {
    overtake_planner::OvertakePlannerCore core(frame, config);
    auto ego = makeEgo(frame, 5.0, ego_d_m);
    ego.v = 1.0;
    auto stopped = makeOpponent(frame, 9.5, target_d_m);
    stopped.id = "grid_d2";
    stopped.v = 0.0;
    stopped.vx = 0.0;
    return core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  // targetが自車の右にいる実D1配置では、左側の現在dを維持する。
  const auto left = evaluate(3.0, 1.6);
  ASSERT_TRUE(left.blocked_info.start_grid_target_active);
  EXPECT_LT(left.blocked_info.start_grid_target_delta_d,
            -config.same_corridor_width_m);
  EXPECT_EQ(left.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(left.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(left.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(left.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(left.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NEAR(left.blocked_info.pass_left_candidate_target_d_m, 3.0, 1.0e-9);

  // 左右反転時も、targetを横切らず右側の現在dを維持する。
  const auto right = evaluate(-3.0, -1.6);
  ASSERT_TRUE(right.blocked_info.start_grid_target_active);
  EXPECT_GT(right.blocked_info.start_grid_target_delta_d,
            config.same_corridor_width_m);
  EXPECT_EQ(right.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(right.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(right.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(right.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(right.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(right.blocked_info.pass_right_candidate_target_d_m, -3.0, 1.0e-9);
}

TEST(OvertakePlannerCore,
     StartGridUnapprovedOuterSideDoesNotCompressUntrackableInnerPass) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    // D1/D2のGate 2配置を単純化したモデル。自車がいる左外側は対象手前から
    // 狭くなるが、右内側には車幅楕円を満たす回廊が残る。
    corridor.push_back({point.s, -5.0, point.s >= 8.0 ? 3.0 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.6);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_EQ(output.blocked_info.maneuver_target_id, "grid_d2");
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(output.blocked_info.pass_left_candidate_reject_reason,
            "pass_target_unreachable");
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(output.blocked_info.pass_right_candidate_reject_reason,
            "pass_transition_deadline_unreachable");
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
}

TEST(OvertakePlannerCore,
     StartGridZeroSpeedStillEvaluatesAcceleratingInnerPassHorizon) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    // Gate 2のD1/D2配置と同じく、現在いる左側は対象位置で狭いが、
    // 右側には車幅楕円を満たす物理回廊が残る。
    corridor.push_back({point.s, -5.0, point.s >= 12.0 ? 3.0 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.3;
  config.min_ellipse_h = 0.2;
  config.pass_target_lateral_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.36);
  ego.v = 0.0;
  auto stopped = makeOpponent(frame, 12.2, 1.98);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.stamp_sec = 0.1;
  auto tracking_not_ready = readyReentryInput();
  tracking_not_ready.pure_pursuit_primary_and_fresh = false;

  const auto pending =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  tracking_not_ready);
  ASSERT_TRUE(pending.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_EQ(pending.selected, overtake_planner::CandidateType::FOLLOW);

  // 1サンプル停止では大横断PASSへ入らず、同じfresh IDを連続停止で
  // 確認できてから加速PASSの長horizon評価を行う。
  stopped.stamp_sec = 1.2;
  const auto output =
      core.update(1.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  tracking_not_ready);

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.blocked_info.pass_proposal_acceleration_allowed)
      << "blocked=" << output.blocked_info.blocked
      << " attack_hold=" << output.blocked_info.attack_follow_hold_pass_side
      << " transaction=" << output.blocked_info.maneuver_transaction_incomplete
      << " latched=" << output.blocked_info.maneuver_target_latched
      << " observed=" << output.blocked_info.maneuver_target_observed
      << " fresh=" << output.blocked_info.maneuver_target_fresh
      << " chain_observed=" << output.blocked_info.maneuver_chain_tail_observed
      << " chain_index=" << output.blocked_info.maneuver_chain_tail_index
      << " chain_ds=" << output.blocked_info.maneuver_chain_tail_relative_s_m
      << " side=" << output.blocked_info.side_by_side
      << " future_side=" << output.blocked_info.future_side_by_side
      << " future_corner=" << output.blocked_info.future_corner_side_by_side
      << " future_yield=" << output.blocked_info.future_yield_required
      << " reentry_hold=" << output.blocked_info.reentry_hold_active
      << " attack_accel="
      << output.blocked_info.attack_follow_acceleration_allowed;
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_feasible)
      << output.blocked_info.pass_right_candidate_reject_reason;
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_tracking_profile_valid);
  EXPECT_GE(output.blocked_info.pass_right_candidate_endpoint_arc_m + 1.0e-6,
            output.blocked_info.pass_right_candidate_required_arc_m);
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
  EXPECT_EQ(output.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_TRUE(output.tracking_release_pass_warmup);
  EXPECT_NE(output.tracking_release_token, 0U);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(
      output.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(output.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(
      output.blocked_info.maneuver_transaction_tracking_release_confirmed);

  // C-002AAでは、commit済みPASSが現周期のwall評価でATTACK_FOLLOWへ
  // 戻った直後、PP proofがN-2まで遅れた周期だけ候補評価用の加速予測を
  // 失い、current-d STOPへの大きな軌道ジャンプを作った。固定済みの
  // target/side/profileと全fresh入力、正常MPCが揃う場合に限り、古いproofを
  // authorityへ使わずATTACK_FOLLOW候補の縦予測だけを維持する。
  auto exact_pass_ack = tracking_not_ready;
  exact_pass_ack.pass_probe_exact_current_usable = true;
  exact_pass_ack.pass_probe_lateral_stop_authority_token =
      output.tracking_release_token;
  exact_pass_ack.pass_probe_exact_plan_generation = 1U;
  exact_pass_ack.pass_probe_exact_sample_stamp_sec = 1.25;
  stopped.stamp_sec = 1.25;
  const auto exact_one =
      core.update(1.25, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  exact_pass_ack);
  EXPECT_FALSE(
      exact_one.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(exact_one.blocked_info.maneuver_transaction_tracking_stop_active);

  exact_pass_ack.pass_probe_exact_sample_stamp_sec = 1.26;
  stopped.stamp_sec = 1.26;
  const auto exact_two =
      core.update(1.26, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  exact_pass_ack);
  EXPECT_FALSE(
      exact_two.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(exact_two.blocked_info.maneuver_transaction_tracking_stop_active);

  stopped.stamp_sec = 1.27;
  const auto committed =
      core.update(1.27, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  exact_pass_ack);
  ASSERT_TRUE(committed.blocked_info.maneuver_transaction_incomplete);
  ASSERT_TRUE(committed.blocked_info.start_grid_lateral_release_pending);
  EXPECT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(committed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(
      committed.blocked_info.maneuver_transaction_tracking_release_confirmed);
  ASSERT_EQ(committed.maneuver_latch_target_id, stopped.id);
  ASSERT_EQ(committed.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);

  stopped.stamp_sec = 1.28;
  const auto executing =
      core.update(1.28, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(executing.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_EQ(executing.selected, overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_FALSE(
      executing.blocked_info.maneuver_transaction_tracking_release_pending);
  ASSERT_TRUE(
      executing.blocked_info.maneuver_transaction_tracking_continuity_armed);

  stopped.stamp_sec = 1.3;
  auto drifted_ego = makeEgo(frame, 5.05, 3.0);
  drifted_ego.v = 1.0;
  auto n2_transport_gap = tracking_not_ready;
  n2_transport_gap.pure_pursuit_tracking_continuity_usable = false;
  n2_transport_gap.pure_pursuit_release_ready = false;
  const auto n2_held =
      core.update(1.3, drifted_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, n2_transport_gap);
  EXPECT_EQ(n2_held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(n2_held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(n2_held.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(n2_held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(n2_held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(n2_held.blocked_info.attack_follow_hold_pass_side);
  EXPECT_TRUE(n2_held.blocked_info.attack_follow_acceleration_allowed);
  EXPECT_TRUE(n2_held.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(n2_held.blocked_info.attack_follow_candidate_feasible);
  EXPECT_TRUE(
      n2_held.blocked_info.attack_follow_candidate_tracking_profile_valid);
  EXPECT_TRUE(n2_held.selected_lateral_profile_safety_verified);
  ASSERT_FALSE(n2_held.longitudinal_offsets_m.empty());
  EXPECT_GE(n2_held.longitudinal_offsets_m.back(),
            config.lateral_override_lookahead_min_distance_m);
  EXPECT_FALSE(
      n2_held.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      n2_held.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_EQ(n2_held.tracking_release_token, 0U);

  const auto expect_committed_attack_follow_bootstrap_rejected =
      [&](const std::string &case_name,
          const std::function<void(overtake_planner::ReentryInputStatus &)>
              &mutate_input,
          bool replace_target_id = false) {
        SCOPED_TRACE(case_name);
        overtake_planner::OvertakePlannerCore rejected_core(frame, config);
        auto rejected_target = stopped;
        rejected_target.stamp_sec = 0.1;
        rejected_core.update(0.1, ego, {rejected_target},
                             overtake_planner::MpcHealthStatus{},
                             tracking_not_ready);
        rejected_target.stamp_sec = 1.2;
        const auto rejected_warmup = rejected_core.update(
            1.2, ego, {rejected_target}, overtake_planner::MpcHealthStatus{},
            tracking_not_ready);
        ASSERT_TRUE(rejected_warmup.tracking_release_pass_warmup);
        auto rejected_exact_ack = tracking_not_ready;
        rejected_exact_ack.pass_probe_exact_current_usable = true;
        rejected_exact_ack.pass_probe_lateral_stop_authority_token =
            rejected_warmup.tracking_release_token;
        rejected_exact_ack.pass_probe_exact_plan_generation = 1U;
        rejected_exact_ack.pass_probe_exact_sample_stamp_sec = 1.25;
        rejected_target.stamp_sec = 1.25;
        rejected_core.update(1.25, ego, {rejected_target},
                             overtake_planner::MpcHealthStatus{},
                             rejected_exact_ack);
        rejected_exact_ack.pass_probe_exact_sample_stamp_sec = 1.26;
        rejected_target.stamp_sec = 1.26;
        rejected_core.update(1.26, ego, {rejected_target},
                             overtake_planner::MpcHealthStatus{},
                             rejected_exact_ack);
        rejected_target.stamp_sec = 1.27;
        rejected_core.update(1.27, ego, {rejected_target},
                             overtake_planner::MpcHealthStatus{},
                             rejected_exact_ack);
        rejected_target.stamp_sec = 1.28;
        const auto rejected_executing = rejected_core.update(
            1.28, ego, {rejected_target}, overtake_planner::MpcHealthStatus{},
            readyReentryInput());
        ASSERT_TRUE(rejected_executing.blocked_info
                        .maneuver_transaction_tracking_continuity_armed);

        rejected_target.stamp_sec = 1.3;
        if (replace_target_id) {
          rejected_target.id = "different_committed_target";
        }
        auto rejected_gap = n2_transport_gap;
        mutate_input(rejected_gap);
        const auto rejected = rejected_core.update(
            1.3, drifted_ego, {rejected_target},
            overtake_planner::MpcHealthStatus{}, rejected_gap);
        EXPECT_FALSE(rejected.blocked_info.attack_follow_acceleration_allowed);
      };

  expect_committed_attack_follow_bootstrap_rejected(
      "committed_stale_ego", [](auto &input) { input.ego_fresh = false; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_stale_opponent_snapshot",
      [](auto &input) { input.all_observed_opponents_fresh = false; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_invalid_reference",
      [](auto &input) { input.reference_valid = false; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_mpc_not_fresh",
      [](auto &input) { input.mpc_health_fresh = false; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_mpc_unhealthy",
      [](auto &input) { input.mpc_healthy = false; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_mpc_hard_failure",
      [](auto &input) { input.mpc_hard_failure = true; });
  expect_committed_attack_follow_bootstrap_rejected(
      "committed_target_identity_changed", [](auto &) {}, true);

  const auto expect_bootstrap_rejected =
      [&](const std::string &case_name,
          const std::function<void(overtake_planner::ReentryInputStatus &)>
              &mutate_input,
          bool moving_target = false, bool replace_target_id = false) {
        SCOPED_TRACE(case_name);
        overtake_planner::OvertakePlannerCore rejected_core(frame, config);
        auto rejected_target = stopped;
        rejected_target.stamp_sec = 0.1;
        if (moving_target) {
          rejected_target.v = 1.0;
          rejected_target.vx = 1.0;
        }
        auto rejected_input = tracking_not_ready;
        rejected_core.update(0.1, ego, {rejected_target},
                             overtake_planner::MpcHealthStatus{},
                             rejected_input);
        rejected_target.stamp_sec = 1.2;
        if (replace_target_id) {
          rejected_target.id = "different_grid_target";
        }
        mutate_input(rejected_input);
        const auto rejected = rejected_core.update(
            1.2, ego, {rejected_target}, overtake_planner::MpcHealthStatus{},
            rejected_input);
        EXPECT_FALSE(rejected.blocked_info.pass_acceleration_allowed);
        EXPECT_FALSE(rejected.tracking_release_pass_warmup);
        EXPECT_EQ(rejected.tracking_release_token, 0U);
        EXPECT_FALSE(rejected.blocked_info
                         .maneuver_transaction_tracking_release_pending);
        EXPECT_FALSE(rejected.lateral_tracking_authorized_during_stop);
      };

  expect_bootstrap_rejected("stale_ego",
                            [](auto &input) { input.ego_fresh = false; });
  expect_bootstrap_rejected(
      "stale_v2x", [](auto &input) { input.v2x_snapshot_fresh = false; });
  expect_bootstrap_rejected("stale_opponent_snapshot", [](auto &input) {
    input.all_observed_opponents_fresh = false;
  });
  expect_bootstrap_rejected("opponent_not_included", [](auto &input) {
    input.all_observed_opponents_included = false;
  });
  expect_bootstrap_rejected("invalid_reference",
                            [](auto &input) { input.reference_valid = false; });
  expect_bootstrap_rejected(
      "mpc_not_fresh", [](auto &input) { input.mpc_health_fresh = false; });
  expect_bootstrap_rejected("mpc_unhealthy",
                            [](auto &input) { input.mpc_healthy = false; });
  expect_bootstrap_rejected("mpc_hard_failure",
                            [](auto &input) { input.mpc_hard_failure = true; });
  expect_bootstrap_rejected(
      "moving_target", [](auto &) {}, true);
  expect_bootstrap_rejected(
      "target_identity_changed", [](auto &) {}, false, true);

  auto denied_config = config;
  denied_config.overtake_permission_profile_enabled = true;
  denied_config.default_overtake_allowed = false;
  denied_config.overtake_permission_lookahead_m = 0.0;
  denied_config.overtake_permission_rules.clear();
  denied_config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"start_denied", 0.0, 40.0,
                                               false});
  denied_config.slow_front_permission_exception_enabled = false;
  denied_config.early_stationary_parallel_permission_exception_enabled = false;
  denied_config.stationary_no_pass_safe_pass_enabled = false;
  overtake_planner::OvertakePlannerCore denied_core(frame, denied_config);
  auto denied_target = stopped;
  denied_target.stamp_sec = 0.1;
  denied_core.update(0.1, ego, {denied_target},
                     overtake_planner::MpcHealthStatus{}, tracking_not_ready);
  denied_target.stamp_sec = 1.2;
  const auto denied = denied_core.update(1.2, ego, {denied_target},
                                         overtake_planner::MpcHealthStatus{},
                                         tracking_not_ready);
  EXPECT_FALSE(denied.blocked_info.overtake_permission_allowed);
  EXPECT_FALSE(denied.blocked_info.straight_overtake_start_allowed);
  EXPECT_FALSE(denied.blocked_info.pass_acceleration_allowed);
  EXPECT_FALSE(denied.tracking_release_pass_warmup);
  EXPECT_EQ(denied.tracking_release_token, 0U);
}

TEST(OvertakePlannerCore,
     RuntimeGateTwoStartDoesNotEnterRecoveryForIntentionalOuterGridLane) {
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_min_delta_s_m = -1.0;
  config.start_grid_target_max_delta_s_m = 8.0;
  config.start_grid_target_lateral_width_m = 1.50;
  config.start_grid_stationary_confirmation_sec = 1.0;
  config.start_grid_stationary_confirmation_distance_m = 3.0;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_uncommitted_hold_max_lateral_drift_m = 0.15;
  config.start_grid_hold_correction_distance_m = 3.0;
  config.start_grid_moving_pass_max_lateral_displacement_m = 2.20;
  config.start_grid_tracking_probe_required_cycles = 2;
  config.start_grid_tracking_release_timeout_cycles = 4;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 5.0;
  config.min_mode_hold_time_sec = 0.60;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.50;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.80;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.prepare_distance_m = 8.0;
  config.merge_distance_m = 12.0;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.0;
  config.slow_front_exception_required_cycles = 3;
  config.slow_front_exception_max_start_curvature_m_inv = 0.040;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.50;
  config.gentle_curve_safe_pass_enabled = false;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 3.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 2.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto make_runtime_ego = [&frame](double s_m, double d_m) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::EgoState ego;
    ego.stamp_sec = 0.0;
    ego.x = point.x;
    ego.y = point.y;
    ego.yaw = point.yaw;
    ego.v = 0.0;
    ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_runtime_opponent = [&frame](double s_m, double d_m) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::OpponentState opponent;
    opponent.stamp_sec = 0.0;
    opponent.x = point.x;
    opponent.y = point.y;
    opponent.vx = 0.0;
    opponent.vy = 0.0;
    opponent.v = 0.0;
    opponent.frenet =
        frame.cartesianToFrenet(opponent.x, opponent.y, point.yaw);
    opponent.valid = true;
    return opponent;
  };

  auto ego = make_runtime_ego(24.995486, 3.361522);
  auto stopped_d2 = make_runtime_opponent(32.231900, 1.978219);
  stopped_d2.id = "d2";
  auto stopped_d3 = make_runtime_opponent(40.171700, 1.978219);
  stopped_d3.id = "d3";
  auto stopped_d4 = make_runtime_opponent(48.111541, 1.978219);
  stopped_d4.id = "d4";

  // 最新D1/D2開始poseをそのまま使い、probe transportだけを単独fixture化する。
  // productionの6秒horizonではFOLLOW不成立時に初回warm-up STOPへ直行するため、
  // ここでは既取得probeを作れる空間horizonだけを延長し、wall/全相手/PP評価、
  // target/side/Float32 wire、required cycle数はproduction値のままにする。
  auto probe_config = config;
  probe_config.lateral_override_max_evaluation_horizon_sec = 20.0;
  overtake_planner::OvertakePlannerCore probe_core(frame, probe_config);
  overtake_planner::PlannerOutput probe_warmup;
  double probe_now_sec = 0.0;
  overtake_planner::EgoState probe_ego;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    probe_now_sec = 0.1 * static_cast<double>(cycle);
    probe_ego = make_runtime_ego(24.995486, cycle == 1 ? 3.361522 : 3.361230);
    probe_ego.v = 0.0;
    stopped_d2.stamp_sec = probe_now_sec;
    stopped_d3.stamp_sec = probe_now_sec;
    stopped_d4.stamp_sec = probe_now_sec;
    probe_warmup = probe_core.update(
        probe_now_sec, probe_ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (probe_warmup.tracking_release_pass_warmup) {
      break;
    }
  }
  ASSERT_TRUE(probe_warmup.tracking_release_pass_warmup);
  ASSERT_NE(probe_warmup.tracking_release_token, 0U);
  EXPECT_EQ(
      probe_warmup.blocked_info.maneuver_transaction_tracking_probe_cycles, 0);

  auto gap_input = readyReentryInput();
  gap_input.pure_pursuit_primary_and_fresh = false;
  gap_input.start_grid_pass_probe_transport_evidence =
      overtake_planner::StartGridProbeTransportEvidence::NORMAL_DELIVERY_GAP;
  stopped_d2.stamp_sec = probe_now_sec + 0.001;
  stopped_d3.stamp_sec = probe_now_sec + 0.001;
  stopped_d4.stamp_sec = probe_now_sec + 0.001;
  const auto gap_output = probe_core.update(
      probe_now_sec + 0.001, probe_ego, {stopped_d2, stopped_d3, stopped_d4},
      overtake_planner::MpcHealthStatus{}, gap_input);
  EXPECT_TRUE(
      gap_output.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(
      gap_output.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(gap_output.blocked_info.maneuver_transaction_tracking_probe_cycles,
            0);
  EXPECT_FALSE(
      gap_output.blocked_info.maneuver_transaction_tracking_release_confirmed);

  overtake_planner::PlannerOutput prepared;
  bool gate_two_prepared = false;
  bool saw_tracking_probe_follow = false;
  int prepared_cycle = 0;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    ego = make_runtime_ego(24.995486, cycle == 1 ? 3.361522 : 3.361230);
    ego.v = 0.0;
    stopped_d2.stamp_sec = now_sec;
    stopped_d3.stamp_sec = now_sec;
    stopped_d4.stamp_sec = now_sec;
    prepared =
        core.update(now_sec, ego, {stopped_d2, stopped_d3, stopped_d4},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (prepared.blocked_info.maneuver_transaction_tracking_probe_cycles > 0 &&
        prepared.blocked_info.maneuver_transaction_tracking_probe_cycles <
            config.start_grid_tracking_probe_required_cycles) {
      saw_tracking_probe_follow = true;
      EXPECT_EQ(prepared.selected, overtake_planner::CandidateType::FOLLOW);
      EXPECT_FALSE(
          prepared.blocked_info.maneuver_transaction_tracking_release_pending);
      EXPECT_FALSE(
          prepared.blocked_info.maneuver_transaction_tracking_stop_active);
      EXPECT_EQ(prepared.tracking_release_token, 0U);
      EXPECT_EQ(prepared.maneuver_latch_target_id, stopped_d2.id);
      EXPECT_EQ(prepared.blocked_info.maneuver_transaction_pass_type,
                overtake_planner::CandidateType::PASS_RIGHT);
    }
    EXPECT_NE(prepared.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY)
        << "cycle=" << cycle << " reason=" << prepared.reason
        << " selected_reject=" << prepared.raw_selected_reject_reason
        << " reentry=" << prepared.reentry_gate.reason;
    EXPECT_FALSE(prepared.reentry_gate.requested) << "cycle=" << cycle;
    EXPECT_NE(prepared.blocked_info.pass_decision_freeze_reason,
              "large_lateral_error")
        << "cycle=" << cycle << " mode=" << static_cast<int>(prepared.mode)
        << " selected=" << static_cast<int>(prepared.selected)
        << " target_active=" << prepared.blocked_info.start_grid_target_active
        << " target_id=" << prepared.blocked_info.start_grid_target_id
        << " latch=" << prepared.maneuver_latch_active << " decision_error="
        << prepared.blocked_info.decision_freeze_lateral_error_m
        << " frozen=" << prepared.blocked_info.pass_decision_frozen
        << " pass_left=" << prepared.blocked_info.pass_left_candidate_feasible
        << " pass_right="
        << prepared.blocked_info.pass_right_candidate_feasible;
    gate_two_prepared =
        prepared.blocked_info.maneuver_transaction_prepared &&
        prepared.blocked_info.maneuver_transaction_tracking_release_pending;
    if (gate_two_prepared) {
      prepared_cycle = cycle;
      break;
    }
  }

  ASSERT_TRUE(gate_two_prepared)
      << "selected=" << static_cast<int>(prepared.selected)
      << " mode=" << static_cast<int>(prepared.mode)
      << " pass_right=" << prepared.blocked_info.pass_right_candidate_feasible
      << " reject=" << prepared.blocked_info.pass_right_candidate_reject_reason
      << " prepared=" << prepared.blocked_info.maneuver_transaction_prepared
      << " safety="
      << prepared.blocked_info.maneuver_target_pass_safety_approved << " probe="
      << prepared.blocked_info.maneuver_transaction_tracking_probe_cycles
      << " probe_reason="
      << prepared.blocked_info.maneuver_transaction_tracking_probe_reset_reason;
  // current-d FOLLOWにPPが実行できる空間arcがある時だけprobe中の走行を許す。
  // このruntime fixtureは停止・未commitかつ6秒上限なので、C-002X以降は
  // FOLLOWを横認可せず、下の停止付きPASS warm-upへ直接fail-closeしてよい。
  if (saw_tracking_probe_follow) {
    EXPECT_GT(prepared_cycle, 1);
  }
  ASSERT_GT(prepared_cycle, 0);
  EXPECT_EQ(prepared.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(prepared.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(prepared.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(prepared.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(prepared.lateral_tracking_authorized_during_stop);
  EXPECT_TRUE(prepared.tracking_release_pass_warmup);
  EXPECT_NE(prepared.tracking_release_token, 0U);
  EXPECT_FALSE(prepared.blocked_info.start_grid_uncommitted_hold_active);
  ASSERT_FALSE(prepared.lateral_offsets.empty());

  // 実bagの失敗周期と同じ約4 mmのd変化と0.02163 m/s微動を入れる。
  // 保存PASSは0 m/s開始の縦予測なので、絶対sだけを現在egoへ張り直さない。
  // 現速度で安全なPASSを再生成できなくても、未commitの同じtarget/sideを保つ
  // current-d FOLLOWがSafetyEvaluatorとPP追従性を満たすならSTOPへ閉じない。
  const double mismatch_time = 0.1 * static_cast<double>(prepared_cycle + 1);
  ego = make_runtime_ego(24.999000, 3.360617);
  ego.v = 0.02163;
  stopped_d2.stamp_sec = mismatch_time;
  stopped_d3.stamp_sec = mismatch_time;
  stopped_d4.stamp_sec = mismatch_time;
  auto mismatch_input = readyReentryInput();
  mismatch_input.pure_pursuit_primary_and_fresh = false;
  mismatch_input.pure_pursuit_attack_follow_transport_usable = true;
  mismatch_input.pure_pursuit_release_ready = false;
  const auto waiting =
      core.update(mismatch_time, ego, {stopped_d2, stopped_d3, stopped_d4},
                  overtake_planner::MpcHealthStatus{}, mismatch_input);
  EXPECT_EQ(waiting.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED)
      << "reason=" << waiting.reason
      << " raw=" << static_cast<int>(waiting.raw_selected)
      << " raw_reject=" << waiting.raw_selected_reject_reason
      << " prepared=" << waiting.blocked_info.maneuver_transaction_prepared
      << " pending="
      << waiting.blocked_info.maneuver_transaction_tracking_release_pending
      << " confirmed="
      << waiting.blocked_info.maneuver_transaction_tracking_release_confirmed
      << " stop="
      << waiting.blocked_info.maneuver_transaction_tracking_stop_active
      << " pass_safe="
      << waiting.blocked_info.maneuver_target_pass_safety_approved
      << " pass_reject="
      << waiting.blocked_info.maneuver_target_pass_reject_reason;
  EXPECT_EQ(waiting.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_NE(waiting.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_TRUE(waiting.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(waiting.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(waiting.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(waiting.tracking_release_pass_warmup);
  EXPECT_EQ(waiting.tracking_release_token, 0U);
  EXPECT_FALSE(waiting.blocked_info.start_grid_uncommitted_hold_active);
  EXPECT_TRUE(waiting.selected_lateral_profile_safety_verified);
  ASSERT_FALSE(waiting.lateral_offsets.empty());
  for (const double d_m : waiting.lateral_offsets) {
    EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(waiting.speed_caps.empty());
  for (const double speed_mps : waiting.speed_caps) {
    EXPECT_LE(speed_mps, config.start_grid_attack_follow_v_max_mps + 1.0e-9);
  }

  // 現在速度から同じidentityのPASSを再生成しても、この実測poseではPP追従性が
  // 成立しない。旧ACKで無理にcommitせず、target/sideを保持したcurrent-d
  // FOLLOWで次のfresh観測を待つ。
  const double proof_time = mismatch_time + 0.1;
  ego.v = 0.0;
  stopped_d2.stamp_sec = proof_time;
  stopped_d3.stamp_sec = proof_time;
  stopped_d4.stamp_sec = proof_time;
  auto proof_input = mismatch_input;
  proof_input.pure_pursuit_release_ready = true;
  const auto still_held =
      core.update(proof_time, ego, {stopped_d2, stopped_d3, stopped_d4},
                  overtake_planner::MpcHealthStatus{}, proof_input);
  EXPECT_EQ(still_held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(still_held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(still_held.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(
      still_held.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      still_held.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_FALSE(
      still_held.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(still_held.maneuver_latch_target_id, stopped_d2.id);
  EXPECT_EQ(still_held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(still_held.tracking_release_pass_warmup);
  EXPECT_EQ(still_held.tracking_release_token, 0U);
  EXPECT_TRUE(still_held.selected_lateral_profile_safety_verified);
  ASSERT_FALSE(still_held.lateral_offsets.empty());
  for (const double d_m : still_held.lateral_offsets) {
    EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
  }

  // 実bagではPASS warm-upのpendingがtimeoutした後、current-d FOLLOW STOPの
  // exact PP proofが来ても候補gateがpending=falseだけを見てPASSを永久reject
  // していた。追従可能poseを保つ独立Coreでpendingを明示的にtimeoutさせる。
  overtake_planner::OvertakePlannerCore timeout_core(frame, config);
  overtake_planner::PlannerOutput timeout_prepared;
  auto timeout_ego = make_runtime_ego(24.995486, 3.361230);
  timeout_ego.v = 0.0;
  double timeout_time = 0.0;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    timeout_time = 0.1 * static_cast<double>(cycle);
    stopped_d2.stamp_sec = timeout_time;
    stopped_d3.stamp_sec = timeout_time;
    stopped_d4.stamp_sec = timeout_time;
    timeout_prepared = timeout_core.update(
        timeout_time, timeout_ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (timeout_prepared.blocked_info.maneuver_transaction_prepared &&
        timeout_prepared.blocked_info
            .maneuver_transaction_tracking_release_pending) {
      break;
    }
  }
  ASSERT_TRUE(timeout_prepared.blocked_info.maneuver_transaction_prepared);
  ASSERT_TRUE(timeout_prepared.blocked_info
                  .maneuver_transaction_tracking_release_pending);

  auto no_tracking_proof = readyReentryInput();
  no_tracking_proof.pure_pursuit_primary_and_fresh = false;
  no_tracking_proof.pure_pursuit_release_ready = false;
  overtake_planner::PlannerOutput timed_out_hold;
  for (int cycle = 0; cycle < 6; ++cycle) {
    timeout_time += 0.1;
    stopped_d2.stamp_sec = timeout_time;
    stopped_d3.stamp_sec = timeout_time;
    stopped_d4.stamp_sec = timeout_time;
    timed_out_hold = timeout_core.update(
        timeout_time, timeout_ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, no_tracking_proof);
    if (!timed_out_hold.blocked_info
             .maneuver_transaction_tracking_release_pending) {
      break;
    }
  }
  ASSERT_TRUE(timed_out_hold.blocked_info.maneuver_transaction_prepared);
  ASSERT_FALSE(timed_out_hold.blocked_info.maneuver_transaction_incomplete);
  ASSERT_FALSE(timed_out_hold.blocked_info
                   .maneuver_transaction_tracking_release_pending);
  ASSERT_EQ(timed_out_hold.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");

  // timeout後のprepared ATTACK_FOLLOWで、Muxが同一target/attempt/directionと
  // 連続trajectoryのN-1 transportを検証した周期は、PASS候補をtracking gate
  // だけで落とさず、現在poseに対するSafetyEvaluator/追従性まで再評価する。
  // このposeのPASS自体は追従不能なのでFOLLOWを維持し、PASS解放はしない。
  auto transport_proof = no_tracking_proof;
  transport_proof.pure_pursuit_attack_follow_transport_usable = true;
  timeout_time += 0.1;
  stopped_d2.stamp_sec = timeout_time;
  stopped_d3.stamp_sec = timeout_time;
  stopped_d4.stamp_sec = timeout_time;
  const auto transport_rechecked = timeout_core.update(
      timeout_time, timeout_ego, {stopped_d2, stopped_d3, stopped_d4},
      overtake_planner::MpcHealthStatus{}, transport_proof);
  EXPECT_EQ(transport_rechecked.selected,
            overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(transport_rechecked.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_FALSE(transport_rechecked.blocked_info
                   .maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(transport_rechecked.blocked_info
                   .maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(transport_rechecked.maneuver_latch_target_id, stopped_d2.id);

  // 停止中の同generation plan/constraint/PP bundleが揃った周期はtracking gateを
  // 解除する。このposeでは再生成PASS自体が操舵制約外なので、無理にwarm-upせず
  // 同じtarget/sideを保持したATTACK_FOLLOWへ戻り、次周期以降もPASSを再評価する。
  auto restart_proof = no_tracking_proof;
  restart_proof.pure_pursuit_release_ready = true;
  restart_proof.mpc_healthy = false;
  restart_proof.verified_non_mpc_pure_pursuit = true;
  timeout_time += 0.1;
  stopped_d2.stamp_sec = timeout_time;
  stopped_d3.stamp_sec = timeout_time;
  stopped_d4.stamp_sec = timeout_time;
  const auto released_follow = timeout_core.update(
      timeout_time, timeout_ego, {stopped_d2, stopped_d3, stopped_d4},
      overtake_planner::MpcHealthStatus{}, restart_proof);
  EXPECT_EQ(released_follow.mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(released_follow.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(released_follow.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_NE(released_follow.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_FALSE(released_follow.blocked_info
                   .maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      released_follow.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(released_follow.tracking_release_pass_warmup);
  EXPECT_EQ(released_follow.tracking_release_token, 0U);
  EXPECT_EQ(released_follow.maneuver_latch_target_id, stopped_d2.id);
  EXPECT_TRUE(released_follow.selected_lateral_profile_safety_verified);
  EXPECT_LT(
      released_follow.blocked_info.maneuver_transaction_tracking_probe_cycles,
      config.start_grid_tracking_probe_required_cycles);

  // 同じ実測poseではPASS snapshotが次のfresh観測へ張り直した時に追従不能の
  // ままなので、単発のraw成立が混ざってもSTOP
  // warm-upを再開しない。target/sideを
  // 保ったFOLLOWで次の成立機会を監視し続ける。
  for (int cycle = 0; cycle < 6; ++cycle) {
    timeout_time += 0.1;
    stopped_d2.stamp_sec = timeout_time;
    stopped_d3.stamp_sec = timeout_time;
    stopped_d4.stamp_sec = timeout_time;
    const auto continued_follow = timeout_core.update(
        timeout_time, timeout_ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, restart_proof);
    EXPECT_EQ(continued_follow.selected,
              overtake_planner::CandidateType::FOLLOW)
        << "cycle=" << cycle << " probe="
        << continued_follow.blocked_info
               .maneuver_transaction_tracking_probe_cycles
        << " probe_reason="
        << continued_follow.blocked_info
               .maneuver_transaction_tracking_probe_reset_reason;
    EXPECT_FALSE(continued_follow.blocked_info
                     .maneuver_transaction_tracking_release_pending);
    EXPECT_FALSE(continued_follow.blocked_info
                     .maneuver_transaction_tracking_stop_active);
    EXPECT_EQ(continued_follow.tracking_release_token, 0U);
    EXPECT_EQ(continued_follow.maneuver_latch_target_id, stopped_d2.id);
    EXPECT_EQ(continued_follow.blocked_info.maneuver_transaction_pass_type,
              overtake_planner::CandidateType::PASS_RIGHT);
  }

  // PREPARE後にfreshness/包含契約が失効した場合は、保存PASSが幾何的に
  // feasibleでもproof待ちを継続しない。古い例外認可を実行commitへ運ばず、
  // pendingを破棄してfail-closedに戻す。
  overtake_planner::OvertakePlannerCore revoked_core(frame, config);
  overtake_planner::PlannerOutput revoked_prepared;
  int revoked_prepared_cycle = 0;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    ego = make_runtime_ego(24.995486, cycle == 1 ? 3.361522 : 3.361230);
    stopped_d2.stamp_sec = now_sec;
    stopped_d3.stamp_sec = now_sec;
    stopped_d4.stamp_sec = now_sec;
    revoked_prepared = revoked_core.update(
        now_sec, ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (revoked_prepared.blocked_info.maneuver_transaction_prepared &&
        revoked_prepared.blocked_info
            .maneuver_transaction_tracking_release_pending) {
      revoked_prepared_cycle = cycle;
      break;
    }
  }
  ASSERT_GT(revoked_prepared_cycle, 0);

  const double revoked_time =
      0.1 * static_cast<double>(revoked_prepared_cycle + 1);
  stopped_d2.stamp_sec = revoked_time;
  stopped_d3.stamp_sec = revoked_time;
  stopped_d4.stamp_sec = revoked_time;
  auto incomplete_inputs = readyReentryInput();
  incomplete_inputs.v2x_snapshot_fresh = false;
  incomplete_inputs.pure_pursuit_release_ready = true;
  const auto revoked = revoked_core.update(
      revoked_time, ego, {stopped_d2, stopped_d3, stopped_d4},
      overtake_planner::MpcHealthStatus{}, incomplete_inputs);
  EXPECT_FALSE(
      revoked.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      revoked.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_FALSE(revoked.blocked_info.maneuver_transaction_incomplete);
  EXPECT_NE(revoked.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);

  // PREPAREのACK待ち中に区間permissionが失効した場合も、入力欠損時と同様に
  // 旧PASS payload/tokenを実行開始へ使わない。開始前だけのauthorityであり、
  // 既にcommit済みのPASS継続契約とは分離して検証する。
  auto permission_config = config;
  permission_config.overtake_permission_profile_enabled = true;
  permission_config.default_overtake_allowed = false;
  permission_config.overtake_permission_lookahead_m = 0.0;
  permission_config.overtake_permission_rules.clear();
  permission_config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"start_allowed", 0.0, 25.05,
                                               true});
  permission_config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"start_denied", 25.06, 39.90,
                                               false});
  permission_config.slow_front_permission_exception_enabled = false;
  permission_config.early_stationary_parallel_permission_exception_enabled =
      false;
  permission_config.stationary_no_pass_safe_pass_enabled = false;
  overtake_planner::OvertakePlannerCore permission_core(frame,
                                                        permission_config);
  overtake_planner::PlannerOutput permission_prepared;
  int permission_prepared_cycle = 0;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    ego = make_runtime_ego(24.995486, cycle == 1 ? 3.361522 : 3.361230);
    ego.v = 0.0;
    stopped_d2.stamp_sec = now_sec;
    stopped_d3.stamp_sec = now_sec;
    stopped_d4.stamp_sec = now_sec;
    permission_prepared = permission_core.update(
        now_sec, ego, {stopped_d2, stopped_d3, stopped_d4},
        overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (permission_prepared.blocked_info.maneuver_transaction_prepared &&
        permission_prepared.blocked_info
            .maneuver_transaction_tracking_release_pending) {
      permission_prepared_cycle = cycle;
      break;
    }
  }
  ASSERT_GT(permission_prepared_cycle, 0)
      << "gate=" << permission_prepared.blocked_info.overtake_start_gate_reason
      << " permission="
      << permission_prepared.blocked_info.overtake_permission_reason
      << " pass_reject="
      << permission_prepared.blocked_info.pass_right_candidate_reject_reason;
  ASSERT_TRUE(permission_prepared.tracking_release_pass_warmup);
  ASSERT_NE(permission_prepared.tracking_release_token, 0U);

  const double permission_revoked_time =
      0.1 * static_cast<double>(permission_prepared_cycle + 1);
  ego = make_runtime_ego(25.10, 3.361230);
  ego.v = 0.0;
  stopped_d2.stamp_sec = permission_revoked_time;
  stopped_d3.stamp_sec = permission_revoked_time;
  stopped_d4.stamp_sec = permission_revoked_time;
  auto old_ack = readyReentryInput();
  old_ack.pure_pursuit_release_ready = true;
  const auto permission_revoked = permission_core.update(
      permission_revoked_time, ego, {stopped_d2, stopped_d3, stopped_d4},
      overtake_planner::MpcHealthStatus{}, old_ack);
  EXPECT_FALSE(permission_revoked.blocked_info.overtake_permission_allowed);
  EXPECT_FALSE(
      permission_revoked.blocked_info.permission_start_exception_active);
  EXPECT_FALSE(permission_revoked.blocked_info
                   .maneuver_transaction_tracking_release_confirmed);
  EXPECT_FALSE(permission_revoked.tracking_release_pass_warmup);
  EXPECT_EQ(permission_revoked.tracking_release_token, 0U);
  EXPECT_FALSE(permission_revoked.lateral_tracking_authorized_during_stop);
  EXPECT_NE(permission_revoked.selected,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(permission_revoked.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(permission_revoked.mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(permission_revoked.mode,
            overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     SupervisorV2AttackFollowHoldsCurrentLateralUntilPassStartAllowed) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.3;
  config.min_ellipse_h = 0.2;
  config.pass_target_lateral_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  // このfixtureでは曲率例外を使わず、V2がPASS前のATTACK_FOLLOWを
  // 独立した安全候補として生成する契約だけを確認する。
  config.slow_front_exception_enabled = false;
  config.gentle_curve_safe_pass_enabled = false;
  config.stationary_no_pass_safe_pass_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.36);
  ego.v = 0.0;
  auto stopped = makeOpponent(frame, 12.2, 1.98);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_EQ(output.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ATTACK_FOLLOW);
  EXPECT_EQ(output.supervisor_v2.target_vehicle_id, "grid_d2");
  EXPECT_EQ(output.supervisor_v2.selected,
            overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.supervisor_v2.reason, "pass_unavailable_attack_follow");
  EXPECT_TRUE(output.supervisor_v2.trajectory.safety_evaluated);
  EXPECT_TRUE(output.supervisor_v2.trajectory.feasible)
      << output.supervisor_v2.trajectory.reject_reason;
  EXPECT_TRUE(
      output.supervisor_v2.trajectory.controller_tracking_profile_valid);
  EXPECT_TRUE(output.supervisor_v2.trajectory_authorized);
  EXPECT_TRUE(output.supervisor_v2.follow_candidate.generated);
  EXPECT_TRUE(output.supervisor_v2.follow_candidate.safety_evaluated);
  EXPECT_TRUE(output.supervisor_v2.follow_candidate.feasible)
      << output.supervisor_v2.follow_candidate.reject_reason;
  EXPECT_TRUE(
      output.supervisor_v2.follow_candidate.controller_tracking_profile_valid);
  EXPECT_GE(output.supervisor_v2.follow_candidate.endpoint_arc_m + 1.0e-6,
            output.supervisor_v2.follow_candidate.required_arc_m);
  EXPECT_NEAR(output.supervisor_v2.follow_candidate.planned_target_d_m,
              ego.frenet.d, 1.0e-6);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.generated);
  ASSERT_FALSE(output.supervisor_v2.trajectory.d.empty());
  EXPECT_TRUE(std::all_of(
      output.supervisor_v2.trajectory.d.begin(),
      output.supervisor_v2.trajectory.d.end(), [&ego](double candidate_d) {
        return std::isfinite(candidate_d) &&
               std::abs(candidate_d - ego.frenet.d) <= 1.0e-6;
      }));
}

TEST(OvertakePlannerCore,
     SupervisorV2GateTwoStartSelectsEvaluatedPassWhenCurveExceptionOpens) {
  auto frame = makeGentleCurvedFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    // Gate2実配置と同様、現在いる左外側は対象手前から狭くなり、右内側の
    // localized profileだけが全区間の回廊preflightを通る。
    corridor.push_back({point.s, -5.0, point.s >= 12.0 ? 3.0 : 5.0});
  }
  frame.setCorridor(corridor);
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_stationary_confirmation_sec = 1.0;
  config.start_grid_stationary_confirmation_distance_m = 3.0;
  config.same_corridor_width_m = 0.90;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.3;
  config.min_ellipse_h = 0.2;
  config.pass_target_lateral_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.0;
  config.slow_front_exception_required_cycles = 3;
  config.slow_front_exception_max_start_curvature_m_inv = 0.040;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  // Gate2 start gridではearly-stationary curve exceptionを使う。gentle-curve
  // 制限PASSとは別経路にし、localized Frenet profile再評価を直接検証する。
  config.gentle_curve_safe_pass_enabled = false;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.090;
  config.gentle_curve_safe_pass_v_max_mps = 10.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.2;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.36);
  ego.v = 0.0;
  auto stopped = makeOpponent(frame, 12.2, 1.98);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  overtake_planner::PlannerOutput output;
  bool start_gate_opened = false;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    stopped.stamp_sec = now_sec;
    output =
        core.update(now_sec, ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    if (output.blocked_info.straight_overtake_start_allowed) {
      start_gate_opened = true;
      break;
    }
  }

  ASSERT_TRUE(start_gate_opened);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.generated);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.safety_evaluated);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.feasible)
      << output.supervisor_v2.pass_right_candidate.reject_reason
      << " endpoint_arc_m="
      << output.supervisor_v2.pass_right_candidate.endpoint_arc_m
      << " required_arc_m="
      << output.supervisor_v2.pass_right_candidate.required_arc_m
      << " target_d_m="
      << output.supervisor_v2.pass_right_candidate.planned_target_d_m
      << " blocking_id="
      << output.supervisor_v2.pass_right_candidate.blocking_opponent_id
      << " blocking_time_sec="
      << output.supervisor_v2.pass_right_candidate.blocking_time_sec
      << " legacy_right_feasible="
      << output.blocked_info.pass_right_candidate_feasible
      << " legacy_right_reject="
      << output.blocked_info.pass_right_candidate_reject_reason
      << " desired_path="
      << output.supervisor_v2.pass_right_candidate.desired_path_trackable
      << " pp_command="
      << output.supervisor_v2.pass_right_candidate
             .pure_pursuit_command_trackable
      << " transaction_type="
      << static_cast<int>(output.blocked_info.maneuver_transaction_pass_type);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate
                  .controller_tracking_profile_valid);
  EXPECT_EQ(output.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(output.supervisor_v2.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(output.supervisor_v2.target_vehicle_id, "grid_d2");
  EXPECT_TRUE(output.supervisor_v2.trajectory_authorized);
}

TEST(OvertakePlannerCore,
     SupervisorV2UsesDedicatedProfileWhenLegacyUsesSmoothstep) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.overtake_lateral_profile_mode = "legacy_smoothstep";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.id = "d2";
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_EQ(output.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_TRUE(output.supervisor_v2.trajectory_authorized);
  EXPECT_TRUE(output.supervisor_v2.pass_left_candidate.generated);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.generated);
  EXPECT_TRUE(output.supervisor_v2.pass_left_candidate.safety_evaluated);
  EXPECT_TRUE(output.supervisor_v2.pass_right_candidate.safety_evaluated);
}

TEST(OvertakePlannerCore,
     GateTwoStagedRightChainKeepsFirstTargetAndAvoidsIntermediateMerge) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -5.0, point.s >= 12.0 ? 3.0 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.supervisor_v2_target_missing_hold_cycles = 2;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.parallel_side_margin_m = 4.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.3;
  config.min_ellipse_h = 0.2;
  config.pass_target_lateral_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.36);
  ego.v = 0.0;
  auto make_stopped = [&](const char *id, double s_m, double d_m) {
    auto stopped = makeOpponent(frame, s_m, d_m);
    stopped.id = id;
    stopped.v = 0.0;
    stopped.vx = 0.0;
    return stopped;
  };
  auto d2 = make_stopped("d2", 12.2, 1.98);
  auto d3 = make_stopped("d3", 20.2, 1.08);
  auto d4 = make_stopped("d4", 28.2, 0.18);
  d2.stamp_sec = 0.1;
  d3.stamp_sec = 0.1;
  d4.stamp_sec = 0.1;

  const auto pending =
      core.update(0.1, ego, {d2, d3, d4}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(pending.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_EQ(pending.selected, overtake_planner::CandidateType::FOLLOW);

  d2.stamp_sec = 1.2;
  d3.stamp_sec = 1.2;
  d4.stamp_sec = 1.2;
  const auto output =
      core.update(1.2, ego, {d2, d3, d4}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_EQ(output.maneuver_latch_target_id, "d2");
  EXPECT_EQ(output.blocked_info.maneuver_chain_tail_id, "d4");
  EXPECT_EQ(output.maneuver_latch_waypoint_count, 3);
  EXPECT_EQ(output.maneuver_latch_last_waypoint_id, "d4");
  EXPECT_LT(output.maneuver_latch_last_waypoint_d_m,
            output.blocked_info.pass_right_candidate_target_d_m);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_feasible)
      << output.blocked_info.pass_right_candidate_reject_reason;
  EXPECT_EQ(output.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);

  ASSERT_EQ(output.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  ASSERT_EQ(output.supervisor_v2.target_vehicle_id, "d2");
  ASSERT_EQ(output.supervisor_v2.pass_direction, -1);
  ASSERT_TRUE(output.supervisor_v2.trajectory_authorized)
      << output.supervisor_v2.reason << " / "
      << output.supervisor_v2.candidate_reject_reason;
  const auto attempt_id = output.supervisor_v2.attempt_id;

  // 先頭d2だけがfreshでも、Gate 2でcommitした車列中のd3/d4が欠けた周期は
  // PASSを再認可しない。短い欠測中はtarget/side/profileを保持した未認可HOLDへ
  // 閉じ、観測が戻れば同じattemptを再開する。
  auto fresh_d2 = d2;
  fresh_d2.stamp_sec = 1.3;
  const auto first_missing =
      core.update(1.3, ego, {fresh_d2}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(first_missing.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(first_missing.supervisor_v2.reason,
            "passing_target_temporarily_unavailable_hold");
  EXPECT_EQ(first_missing.supervisor_v2.target_vehicle_id, "d2");
  EXPECT_EQ(first_missing.supervisor_v2.pass_direction, -1);
  EXPECT_EQ(first_missing.supervisor_v2.attempt_id, attempt_id);
  EXPECT_FALSE(first_missing.supervisor_v2.trajectory_authorized);
  EXPECT_NE(first_missing.supervisor_v2.authorization_failure_mask &
                overtake_planner::SUPERVISOR_V2_AUTH_OPPONENT_STALE,
            0U);

  auto recovered_d2 = d2;
  auto recovered_d3 = d3;
  auto recovered_d4 = d4;
  recovered_d2.stamp_sec = 1.4;
  recovered_d3.stamp_sec = 1.4;
  recovered_d4.stamp_sec = 1.4;
  const auto recovered =
      core.update(1.4, ego, {recovered_d2, recovered_d3, recovered_d4},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_EQ(recovered.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(recovered.supervisor_v2.target_vehicle_id, "d2");
  EXPECT_EQ(recovered.supervisor_v2.pass_direction, -1);
  EXPECT_EQ(recovered.supervisor_v2.attempt_id, attempt_id);
  EXPECT_TRUE(recovered.supervisor_v2.trajectory_authorized)
      << recovered.supervisor_v2.reason << " / "
      << recovered.supervisor_v2.candidate_reject_reason;

  fresh_d2.stamp_sec = 1.5;
  const auto missing_again_first =
      core.update(1.5, ego, {fresh_d2}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  fresh_d2.stamp_sec = 1.6;
  const auto missing_again_second =
      core.update(1.6, ego, {fresh_d2}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  fresh_d2.stamp_sec = 1.7;
  const auto missing_timeout =
      core.update(1.7, ego, {fresh_d2}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(missing_again_first.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(missing_again_second.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(missing_timeout.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(missing_timeout.supervisor_v2.reason,
            "passing_target_unavailable_timeout");
  EXPECT_EQ(missing_timeout.supervisor_v2.target_vehicle_id, "d2");
  EXPECT_EQ(missing_timeout.supervisor_v2.pass_direction, -1);
  EXPECT_EQ(missing_timeout.supervisor_v2.attempt_id, attempt_id);
}

TEST(OvertakePlannerCore,
     GateTwoApprovedSideStaysLatchedWhenStartGridClassifierActivates) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0; // start-grid速度窓だけを閉じる。
  auto stopped = makeOpponent(frame, 12.0, 1.6);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  const auto pre_arm =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_FALSE(pre_arm.blocked_info.start_grid_target_active);
  ASSERT_TRUE(pre_arm.blocked_info.early_stationary_parallel_pass_target);
  ASSERT_EQ(pre_arm.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);

  ego.v = 1.0;
  stopped.stamp_sec = 0.2;
  const auto armed =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(armed.blocked_info.start_grid_target_active);
  EXPECT_LT(armed.blocked_info.start_grid_target_delta_d,
            -config.same_corridor_width_m);
  EXPECT_EQ(armed.blocked_info.maneuver_target_id, "grid_d2");
  EXPECT_EQ(armed.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(armed.blocked_info.pass_decision_frozen)
      << armed.blocked_info.pass_decision_freeze_reason;
  EXPECT_FALSE(armed.blocked_info.side_by_side);
  EXPECT_FALSE(armed.blocked_info.future_yield_required)
      << armed.blocked_info.yield_reason;
  EXPECT_FALSE(armed.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(armed.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(armed.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(armed.raw_selected, overtake_planner::CandidateType::PASS_RIGHT);
  // 分類が切り替わっても同じGate 2認可済みside/IDなので、中断や反対側への
  // 遷移を挟まず実行中PASSを継続する。
  EXPECT_EQ(armed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(armed.selected, overtake_planner::CandidateType::PASS_RIGHT);

  stopped.stamp_sec = 0.3;
  const auto right_prepare =
      core.update(0.3, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(right_prepare.blocked_info.maneuver_target_id, "grid_d2");
  EXPECT_EQ(right_prepare.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(right_prepare.raw_selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(right_prepare.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);

  stopped.stamp_sec = 0.4;
  const auto right_overtake =
      core.update(0.4, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(right_overtake.blocked_info.maneuver_target_id, "grid_d2");
  EXPECT_EQ(right_overtake.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(right_overtake.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(right_overtake.mode,
            overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(right_overtake.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
}

TEST(OvertakePlannerCore,
     GateTwoCommittedStagedChainKeepsGeometryAcrossTargetHandoff) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -5.0, point.s >= 12.0 ? 3.0 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.start_grid_target_enabled = false;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.parallel_side_margin_m = 4.0;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.3;
  config.min_ellipse_h = 0.2;
  config.pass_target_lateral_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.merge_front_gap_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.36);
  // 停止chainを通常のGate 2で低速publish/commitし、初回有限wireが
  // d2 stageへ到達した状態を作る。
  ego.v = 1.0;
  auto make_stopped = [&](const char *id, double s_m, double d_m,
                          double stamp_sec) {
    auto stopped = makeOpponent(frame, s_m, d_m);
    stopped.id = id;
    stopped.v = 0.0;
    stopped.vx = 0.0;
    stopped.stamp_sec = stamp_sec;
    return stopped;
  };
  auto targets_at = [&](double stamp_sec) {
    return std::vector<overtake_planner::OpponentState>{
        make_stopped("d2", 12.2, 1.98, stamp_sec),
        make_stopped("d3", 20.2, 1.08, stamp_sec),
        make_stopped("d4", 28.2, 0.18, stamp_sec)};
  };

  const auto direct_commit =
      core.update(0.1, ego, targets_at(0.1),
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(direct_commit.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_TRUE(direct_commit.blocked_info.maneuver_transaction_incomplete);
  double now_sec = 0.1;
  ASSERT_EQ(direct_commit.maneuver_latch_waypoint_count, 3);
  const double initial_avoid_start_s_m =
      direct_commit.maneuver_latch_avoid_start_s_m;
  const double d3_stage_d_m =
      1.08 -
      (config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
       config.pass_target_lateral_margin_m);

  // d2のscalar targetより外側だが、d3用waypointの正当な横位置にいる。
  // これをd2 overshootとしてABORTせず、latched chain geometryをfreshに
  // 再評価して継続する。
  auto between_targets_ego = makeEgo(frame, 17.0, d3_stage_d_m);
  between_targets_ego.v = 4.0;
  now_sec = 4.2;
  const auto between_targets =
      core.update(now_sec, between_targets_ego, targets_at(now_sec),
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_NE(between_targets.blocked_info.pass_right_candidate_reject_reason,
            "committed_pass_geometry_tracking_error");
  EXPECT_EQ(between_targets.maneuver_latch_target_id, "d2");
  EXPECT_EQ(between_targets.maneuver_latch_waypoint_count, 3);
  EXPECT_LT(between_targets.maneuver_latch_last_waypoint_d_m, d3_stage_d_m);

  // merge_front_gapを確保した周期まではd2を保持し、次周期にだけd3へ進む。
  // handoff後も初回のavoid markerとd4 waypointを保持し、現在位置から
  // 1--2 mで新規profileを作り直してuntrackableにしない。
  auto completed_d2_ego = makeEgo(frame, 18.3, d3_stage_d_m);
  completed_d2_ego.v = 2.0;
  now_sec += 0.2;
  const auto completed_d2 =
      core.update(now_sec, completed_d2_ego, targets_at(now_sec),
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(completed_d2.maneuver_latch_target_id, "d2");
  ASSERT_TRUE(completed_d2.blocked_info.maneuver_target_pass_complete);

  auto handed_off_ego = makeEgo(frame, 18.4, d3_stage_d_m);
  handed_off_ego.v = 2.0;
  now_sec += 0.1;
  const auto handed_off =
      core.update(now_sec, handed_off_ego, targets_at(now_sec),
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_EQ(handed_off.maneuver_latch_target_id, "d3");
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_previous_id, "d2");
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_change_reason,
            "previous_chain_target_passed");
  EXPECT_TRUE(handed_off.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(handed_off.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(handed_off.maneuver_latch_waypoint_count, 3);
  EXPECT_EQ(handed_off.maneuver_latch_last_waypoint_id, "d4");
  EXPECT_NEAR(handed_off.maneuver_latch_avoid_start_s_m,
              initial_avoid_start_s_m, 1.0e-9);
  EXPECT_NE(handed_off.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_NE(handed_off.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(OvertakePlannerCore,
     StartGridCommittedPassAttackFollowsBeforePassTrackingProof) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0;
  auto stopped = makeOpponent(frame, 12.0, 1.6);
  stopped.id = "grid_unproven_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {stopped},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_incomplete);

  // 直前のstart-grid PASS tracking proofが無い状態で、1世代遅れという
  // 形だけのcontinuityを入れる。target/side文字列まで一致していても、
  // 初回PASS解放の代わりにはならない。同一transactionを保持した
  // SafetyEvaluator済みATTACK_FOLLOWへ閉じ、PASSは追従proofまで開始しない。
  ego.v = 1.0;
  stopped.stamp_sec = 0.2;
  auto unproven_continuity = readyReentryInput();
  unproven_continuity.pure_pursuit_primary_and_fresh = false;
  unproven_continuity.pure_pursuit_tracking_continuity_usable = true;
  unproven_continuity.pure_pursuit_tracking_target_id = stopped.id;
  unproven_continuity.pure_pursuit_tracking_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  unproven_continuity.mpc_healthy = false;
  unproven_continuity.mpc_latency_warning = true;
  overtake_planner::MpcHealthStatus bounded_latency_mpc;
  bounded_latency_mpc.valid = true;
  bounded_latency_mpc.infeasible_count = 0;
  bounded_latency_mpc.solve_time_ms = 115.0;
  bounded_latency_mpc.age_sec = 0.0;
  const auto held = core.update(0.2, ego, {stopped}, bounded_latency_mpc,
                                unproven_continuity);

  ASSERT_TRUE(held.blocked_info.start_grid_target_active);
  EXPECT_FALSE(
      held.blocked_info.maneuver_transaction_tracking_continuity_armed);
  EXPECT_EQ(held.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(held.blocked_info.attack_follow_hold_pass_side);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_feasible);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_tracking_profile_valid);
  EXPECT_TRUE(held.selected_lateral_profile_safety_verified);
  EXPECT_EQ(held.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(held.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(OvertakePlannerCore,
     StartGridCommittedPassAttackFollowsUntilTrackingContractRecovers) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  // このケースは最終PP wireの追従性まで検証するため、12点の軽量Core fixture
  // ではなく実運用と同じ25 ms空間samplingを使う。粗い線形補間だけを理由に
  // 有効なlocalized profileをrejectするテストにはしない。
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0; // 最初の周期だけstart-grid速度窓を閉じ、PASSを先に認可する。
  auto stopped = makeOpponent(frame, 12.0, 1.6);
  stopped.id = "grid_latency_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {stopped},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_incomplete);

  ego.v = 1.0;
  stopped.stamp_sec = 0.2;
  const auto armed =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(armed.blocked_info.start_grid_target_active);
  ASSERT_TRUE(armed.blocked_info.start_grid_lateral_release_pending);
  ASSERT_EQ(armed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_EQ(armed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_EQ(armed.maneuver_latch_target_id, stopped.id);
  const double committed_target_d_m =
      armed.blocked_info.pass_right_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(committed_target_d_m));

  // 実行済みPASSのexact current-generation proofを一度確認し、同じ
  // transactionのATTACK_FOLLOW typed continuityを再試行根拠へarmする。
  stopped.stamp_sec = 0.25;
  const auto tracked_pass =
      core.update(0.25, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      tracked_pass.blocked_info.maneuver_transaction_tracking_continuity_armed);

  auto drifted_ego = makeEgo(frame, 5.05, 3.0);
  drifted_ego.v = 1.0;

  // 実機ログ相当の単発solve-time警告とPP契約失効を入れる。PASSを無理に
  // 継続せず、同じtarget/sideへ張り付くSafetyEvaluator済みATTACK_FOLLOWを
  // Planner出力として保持する。非稼働MPCの異常だけでは後段constraintも
  // 全停止させず速度capに留め、Planner transactionをSAFE_STOPやABORTへ
  // 崩さない。
  stopped.stamp_sec = 0.3;
  auto transient_tracking = readyReentryInput();
  transient_tracking.mpc_healthy = false;
  transient_tracking.mpc_health_fresh = true;
  transient_tracking.mpc_hard_failure = false;
  transient_tracking.mpc_latency_warning = true;
  transient_tracking.pure_pursuit_primary_and_fresh = false;
  overtake_planner::MpcHealthStatus transient_mpc;
  transient_mpc.valid = true;
  transient_mpc.infeasible_count = 1;
  transient_mpc.solve_time_ms = 466.95;
  transient_mpc.age_sec = 0.0;
  transient_mpc.sample_sequence = 1U;
  const auto held = core.update(0.3, drifted_ego, {stopped}, transient_mpc,
                                transient_tracking);

  EXPECT_EQ(held.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_safe_lateral_hold_active);
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(held.raw_selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(held.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(held.blocked_info.attack_follow_hold_pass_side);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_feasible);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_tracking_profile_valid);
  EXPECT_TRUE(held.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(held.blocked_info.attack_follow_acceleration_allowed);
  ASSERT_FALSE(held.speed_caps.empty());
  EXPECT_TRUE(std::all_of(
      held.speed_caps.begin(), held.speed_caps.end(),
      [&](double speed_mps) { return speed_mps <= drifted_ego.v + 1.0e-9; }));
  const auto expect_attack_follow_profile =
      [committed_target_d_m](const overtake_planner::PlannerOutput &output,
                             double ego_d_m) {
        ASSERT_FALSE(output.lateral_offsets.empty());
        const double pass_direction =
            committed_target_d_m >= ego_d_m ? 1.0 : -1.0;
        EXPECT_NEAR(output.lateral_offsets.front(), ego_d_m, 1.0e-3);
        double previous_d_m = output.lateral_offsets.front();
        for (const double d_m : output.lateral_offsets) {
          EXPECT_TRUE(std::isfinite(d_m));
          EXPECT_GE(pass_direction * (d_m - previous_d_m), -1.0e-6);
          EXPECT_LE(pass_direction * (d_m - committed_target_d_m), 1.0e-6);
          previous_d_m = d_m;
        }
        EXPECT_GT(pass_direction * (output.lateral_offsets.back() - ego_d_m),
                  1.0e-3);
      };
  ASSERT_FALSE(held.lateral_offsets.empty());
  expect_attack_follow_profile(held, drifted_ego.frenet.d);
  const auto stop_constraint = overtake_planner::makeSafetyConstraint(
      held, drifted_ego, transient_tracking, 8.0, 1.0);
  EXPECT_TRUE(stop_constraint.valid);
  EXPECT_FALSE(stop_constraint.stop_requested);
  EXPECT_EQ(stop_constraint.reason, "mpc_health_infeasible_guard");
  EXPECT_FALSE(overtake_planner::isSafetyEvaluatedCurrentDTrackingStop(
      held, drifted_ego, stop_constraint.valid, stop_constraint.stop_requested,
      stop_constraint.release_authorized, stop_constraint.speed_limit_mps,
      stop_constraint.reason));

  // health capが解消し、同じID/sideのexact ATTACK_FOLLOW proofが揃っても、
  // cross-phase PASSを直接走行させない。現在の公開PASS wireがPP追従不能なら
  // ACK対象のpayload/tokenが無いpending STOPを残さず、同じtarget/sideの
  // SafetyEvaluator済みATTACK_FOLLOWへ戻し、次周期のPASS再評価を継続する。
  stopped.stamp_sec = 0.35;
  auto moving_attack_follow_proof = readyReentryInput();
  moving_attack_follow_proof.pure_pursuit_release_ready = false;
  moving_attack_follow_proof.pure_pursuit_tracking_continuity_usable = true;
  moving_attack_follow_proof.pure_pursuit_tracking_target_id = stopped.id;
  moving_attack_follow_proof.pure_pursuit_tracking_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  const auto rejected_pass_follow = core.update(
      0.35, drifted_ego, {stopped}, overtake_planner::MpcHealthStatus{},
      moving_attack_follow_proof);
  EXPECT_EQ(rejected_pass_follow.mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(rejected_pass_follow.selected,
            overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(rejected_pass_follow.maneuver_latch_target_id, stopped.id);
  EXPECT_FALSE(rejected_pass_follow.blocked_info
                   .maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(rejected_pass_follow.blocked_info
                   .maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(rejected_pass_follow.tracking_release_pass_warmup);
  EXPECT_EQ(rejected_pass_follow.tracking_release_token, 0U);
  EXPECT_EQ(rejected_pass_follow.raw_selected,
            overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(
      rejected_pass_follow.blocked_info.maneuver_transaction_retry_active);
  EXPECT_TRUE(
      rejected_pass_follow.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(rejected_pass_follow.blocked_info
                  .maneuver_transaction_tracking_continuity_armed);
  EXPECT_TRUE(rejected_pass_follow.selected_lateral_profile_safety_verified);
  EXPECT_TRUE(rejected_pass_follow.active_override);
  EXPECT_FALSE(rejected_pass_follow.published_lateral_safety_rejected);
  EXPECT_EQ(rejected_pass_follow.reason,
            "tracking_release_published_pass_rejected_attack_follow");
  EXPECT_TRUE(rejected_pass_follow.blocked_info.attack_follow_hold_pass_side);
  EXPECT_TRUE(
      rejected_pass_follow.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(
      rejected_pass_follow.blocked_info.attack_follow_candidate_feasible);
  EXPECT_TRUE(rejected_pass_follow.blocked_info
                  .attack_follow_candidate_tracking_profile_valid);
  EXPECT_EQ(rejected_pass_follow.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  const auto attack_follow_wire =
      overtake_planner::makeReferenceOverrideWirePayload(rejected_pass_follow,
                                                         77U);
  EXPECT_EQ(attack_follow_wire.kind,
            overtake_planner::ReferenceOverrideWireKind::
                SPATIAL_LATERAL_AND_SPEED_V4);
  const auto attack_follow_identity =
      overtake_planner::makeOverrideTrackingIdentity(
          rejected_pass_follow, attack_follow_wire, 77U, 1U);
  EXPECT_EQ(attack_follow_identity.kind,
            overtake_planner::OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  EXPECT_EQ(attack_follow_identity.target_id, stopped.id);
  EXPECT_EQ(attack_follow_identity.pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  const auto attack_follow_constraint = overtake_planner::makeSafetyConstraint(
      rejected_pass_follow, drifted_ego, moving_attack_follow_proof, 8.0, 1.0);
  EXPECT_TRUE(attack_follow_constraint.valid);
  EXPECT_FALSE(attack_follow_constraint.stop_requested);

  // 0.35秒周期で実際のpayload/tokenを持つATTACK_FOLLOWがpublishされた。
  // 次周期はそのexact proofを再使用する。同じpose/yawで不成立に
  // なったPASS wireは再armしないため、実走で車体が前進しPASS側へ
  // 追従した相当のpose変化後にfresh評価し、release-ready=falseのまま
  // 新PASS warm-up/tokenを
  // 生成する。token 0 STOPのproofは注入しない。
  stopped.stamp_sec = 0.4;
  auto retry_ego = drifted_ego;
  retry_ego.x = 5.10;
  retry_ego.y = 2.95;
  retry_ego.frenet = frame.cartesianToFrenet(5.10, 2.95, 0.0);
  const auto pass_warmup = core.update(0.4, retry_ego, {stopped},
                                       overtake_planner::MpcHealthStatus{},
                                       moving_attack_follow_proof);
  if (!pass_warmup.tracking_release_pass_warmup) {
    // poseが変化しても同じPASSがPP追従不能なら、「テストのための
    // release-ready」で突破しない。実wireでpublishできる同一target/sideの
    // ATTACK_FOLLOWを維持し、ACK対象の無いpending STOPに入らないことを
    // このfixtureの受入条件とする。PASSが実際にtrackableな場合は下の
    // nonzero token warm-up/exact ACK契約も続けて検証する。
    EXPECT_EQ(pass_warmup.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_EQ(pass_warmup.selected, overtake_planner::CandidateType::FOLLOW);
    EXPECT_EQ(pass_warmup.raw_selected,
              overtake_planner::CandidateType::FOLLOW);
    EXPECT_EQ(pass_warmup.maneuver_latch_target_id, stopped.id);
    EXPECT_EQ(pass_warmup.blocked_info.maneuver_transaction_pass_type,
              overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_TRUE(pass_warmup.blocked_info.attack_follow_hold_pass_side);
    EXPECT_TRUE(pass_warmup.blocked_info.attack_follow_candidate_feasible);
    EXPECT_TRUE(pass_warmup.selected_lateral_profile_safety_verified);
    EXPECT_TRUE(pass_warmup.active_override);
    EXPECT_FALSE(pass_warmup.published_lateral_safety_rejected);
    EXPECT_FALSE(
        pass_warmup.blocked_info.maneuver_transaction_tracking_release_pending);
    EXPECT_FALSE(
        pass_warmup.blocked_info.maneuver_transaction_tracking_stop_active);
    EXPECT_EQ(pass_warmup.tracking_release_token, 0U);
    return;
  }
  EXPECT_EQ(pass_warmup.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(pass_warmup.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(pass_warmup.maneuver_latch_target_id, stopped.id);
  EXPECT_TRUE(
      pass_warmup.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      pass_warmup.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(
      pass_warmup.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(pass_warmup.lateral_tracking_authorized_during_stop);
  EXPECT_NE(pass_warmup.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_EQ(pass_warmup.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(pass_warmup.blocked_info.pass_right_candidate_target_d_m,
              committed_target_d_m, 1.0e-9);
  EXPECT_TRUE(pass_warmup.active_override);
  EXPECT_TRUE(pass_warmup.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(pass_warmup.published_lateral_safety_rejected);
  EXPECT_TRUE(pass_warmup.lateral_stop_inputs_complete);
  EXPECT_EQ(pass_warmup.lateral_offsets.size(), pass_warmup.speed_caps.size());
  EXPECT_EQ(pass_warmup.lateral_offsets.size(),
            pass_warmup.longitudinal_offsets_m.size());
  ASSERT_TRUE(pass_warmup.tracking_release_pass_warmup) << pass_warmup.reason;
  const auto first_pass_release_token = pass_warmup.tracking_release_token;
  ASSERT_NE(first_pass_release_token, 0U);

  // warm-up PASSをpublishした直後は、PPのexact ACKがまだ届かない正常な配送窓が
  // ある。この間も縦STOPは維持しつつ、解除後の加速s(t)で同じPASSを再評価し、
  // untrackable/FOLLOWへ自己崩壊させない。
  stopped.stamp_sec = 0.45;
  auto waiting_for_exact_ack = readyReentryInput();
  waiting_for_exact_ack.pure_pursuit_primary_and_fresh = false;
  waiting_for_exact_ack.pure_pursuit_release_ready = false;
  const auto warmup_waiting =
      core.update(0.45, retry_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, waiting_for_exact_ack);
  EXPECT_EQ(warmup_waiting.mode,
            overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(warmup_waiting.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(warmup_waiting.blocked_info.pass_acceleration_allowed);
  EXPECT_TRUE(warmup_waiting.blocked_info.pass_right_candidate_feasible);
  EXPECT_NE(warmup_waiting.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_TRUE(warmup_waiting.blocked_info
                  .maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(
      warmup_waiting.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(warmup_waiting.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(warmup_waiting.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);

  // ACK到着と同じ周期に現在yawが追従契約外へ変わった場合、古いsnapshotを
  // confirmせず、target/sideを保持したcurrent-d STOPへ閉じる。
  stopped.stamp_sec = 0.5;
  auto first_release_proof = readyReentryInput();
  first_release_proof.pure_pursuit_primary_and_fresh = false;
  first_release_proof.pure_pursuit_release_ready = true;
  auto yaw_mismatch_ego = retry_ego;
  yaw_mismatch_ego.yaw += 1.0;
  const auto yaw_rejected =
      core.update(0.5, yaw_mismatch_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, first_release_proof);
  EXPECT_NE(yaw_rejected.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(yaw_rejected.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_EQ(yaw_rejected.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(yaw_rejected.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(
      yaw_rejected.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(yaw_rejected.blocked_info
                   .maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(
      yaw_rejected.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(yaw_rejected.tracking_release_pass_warmup);
  EXPECT_EQ(yaw_rejected.tracking_release_token, 0U);
  ASSERT_FALSE(yaw_rejected.lateral_offsets.empty());
  for (const double d_m : yaw_rejected.lateral_offsets) {
    EXPECT_NEAR(d_m, yaw_mismatch_ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(yaw_rejected.speed_caps.empty());
  for (const double speed_mps : yaw_rejected.speed_caps) {
    EXPECT_LE(speed_mps, config.safe_stop_v_mps + 1.0e-9);
  }

  // yaw復旧後は同じID/sideでPASSを再生成するが、この周期に見えている旧ACKでは
  // 解除しない。新tokenをpublishし、Nodeが新generationのproofを待つ。
  stopped.stamp_sec = 0.6;
  const auto regenerated_warmup =
      core.update(0.6, retry_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, first_release_proof);
  EXPECT_EQ(regenerated_warmup.selected,
            overtake_planner::CandidateType::PASS_RIGHT)
      << "reject="
      << regenerated_warmup.blocked_info.pass_right_candidate_reject_reason
      << " probe_reason="
      << regenerated_warmup.blocked_info
             .maneuver_transaction_tracking_probe_reset_reason
      << " pending="
      << regenerated_warmup.blocked_info
             .maneuver_transaction_tracking_release_pending
      << " generated="
      << regenerated_warmup.blocked_info.pass_right_candidate_generated
      << " tracking="
      << regenerated_warmup.blocked_info
             .pass_right_candidate_tracking_profile_valid
      << " desired="
      << regenerated_warmup.blocked_info
             .pass_right_candidate_desired_path_trackable
      << " pp="
      << regenerated_warmup.blocked_info
             .pass_right_candidate_pure_pursuit_command_trackable
      << " continuity="
      << regenerated_warmup.blocked_info
             .committed_pass_spatial_profile_continuity_used
      << " error="
      << regenerated_warmup.blocked_info
             .committed_pass_spatial_profile_tracking_error_m
      << " source_age="
      << regenerated_warmup.blocked_info
             .committed_pass_spatial_profile_source_age_sec
      << " reason=" << regenerated_warmup.reason << " pass_lockout="
      << regenerated_warmup.blocked_info.pass_reauthorization_lockout_active
      << " reentry_center="
      << regenerated_warmup.blocked_info.reentry_centering_authorized
      << " start_grid="
      << regenerated_warmup.blocked_info.start_grid_target_active;
  EXPECT_TRUE(regenerated_warmup.blocked_info.maneuver_transaction_retry_active)
      << " reason=" << regenerated_warmup.reason
      << " raw=" << static_cast<int>(regenerated_warmup.raw_selected)
      << " raw_reject=" << regenerated_warmup.raw_selected_reject_reason
      << " incomplete="
      << regenerated_warmup.blocked_info.maneuver_transaction_incomplete
      << " prepared="
      << regenerated_warmup.blocked_info.maneuver_transaction_prepared;
  EXPECT_TRUE(regenerated_warmup.tracking_release_pass_warmup);
  EXPECT_NE(regenerated_warmup.tracking_release_token,
            first_pass_release_token);
  EXPECT_FALSE(regenerated_warmup.blocked_info
                   .maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(regenerated_warmup.blocked_info
                  .maneuver_transaction_tracking_stop_active);

  // 実publish wireが変わる周期はtokenを更新し、その周期に見えていたproofを
  // 取り消す。次周期のexact proofを繰り返し、token/payloadが連続した時だけ
  // confirmedへ進める。
  auto previous_warmup = regenerated_warmup;
  overtake_planner::PlannerOutput release_confirmed;
  bool release_ready = false;
  int token_change_count = 0;
  double release_time = 0.6;
  for (int cycle = 0; cycle < 100; ++cycle) {
    release_time += 0.1;
    stopped.stamp_sec = release_time;
    const auto next =
        core.update(release_time, retry_ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, first_release_proof);
    ASSERT_NE(next.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
    ASSERT_TRUE(next.tracking_release_pass_warmup);
    if (next.blocked_info.maneuver_transaction_tracking_release_confirmed) {
      EXPECT_EQ(next.tracking_release_token,
                previous_warmup.tracking_release_token);
      EXPECT_EQ(next.lateral_offsets, previous_warmup.lateral_offsets);
      EXPECT_EQ(next.speed_caps, previous_warmup.speed_caps);
      EXPECT_EQ(next.longitudinal_offsets_m,
                previous_warmup.longitudinal_offsets_m);
      release_confirmed = next;
      release_ready = true;
      break;
    }
    if (next.tracking_release_token != previous_warmup.tracking_release_token) {
      ++token_change_count;
    }
    previous_warmup = next;
  }
  ASSERT_TRUE(release_ready)
      << "token=" << previous_warmup.tracking_release_token
      << " changes=" << token_change_count
      << " mode=" << static_cast<int>(previous_warmup.mode)
      << " selected=" << static_cast<int>(previous_warmup.selected)
      << " reason=" << previous_warmup.reason
      << " raw_reject=" << previous_warmup.raw_selected_reject_reason;

  // 2段目: stop下で安定したPASS generation自身のPP proofだけが、同じ
  // target/side/target dのtransactionを解除候補にできる。
  EXPECT_EQ(release_confirmed.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(release_confirmed.blocked_info
                  .maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(release_confirmed.blocked_info
                  .maneuver_transaction_tracking_release_confirmed);
  EXPECT_FALSE(
      release_confirmed.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(release_confirmed.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(release_confirmed.lateral_offsets, previous_warmup.lateral_offsets);
  EXPECT_EQ(release_confirmed.speed_caps, previous_warmup.speed_caps);
  EXPECT_EQ(release_confirmed.longitudinal_offsets_m,
            previous_warmup.longitudinal_offsets_m);
  EXPECT_EQ(release_confirmed.tracking_release_token,
            previous_warmup.tracking_release_token);

  // final muxがPASSをprimaryとして出せた後にだけhandshake状態を解放する。
  release_time += 0.1;
  stopped.stamp_sec = release_time;
  const auto resumed =
      core.update(release_time, retry_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_EQ(resumed.selected, overtake_planner::CandidateType::PASS_RIGHT)
      << " reason=" << resumed.reason
      << " reject=" << resumed.blocked_info.pass_right_candidate_reject_reason
      << " mode=" << static_cast<int>(resumed.mode)
      << " raw=" << static_cast<int>(resumed.raw_selected)
      << " start_grid=" << resumed.blocked_info.start_grid_target_active
      << " continuity="
      << resumed.blocked_info.committed_pass_spatial_profile_continuity_used;
  EXPECT_FALSE(
      resumed.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      resumed.blocked_info.maneuver_transaction_tracking_release_confirmed);

  // Plannerが次generationをpublishする直前は、final PPのtyped proofが正常に
  // 1世代だけ遅れる。MPC horizonを実際に使っていてもhealthがfresh/feasibleな
  // 通常周期なら、latency warningの有無を逆条件にせず同じPASSを継続する。
  auto healthy_one_generation_lag = readyReentryInput();
  healthy_one_generation_lag.pure_pursuit_primary_and_fresh = false;
  healthy_one_generation_lag.pure_pursuit_tracking_continuity_usable = true;
  healthy_one_generation_lag.pure_pursuit_tracking_target_id = stopped.id;
  healthy_one_generation_lag.pure_pursuit_tracking_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  healthy_one_generation_lag.verified_non_mpc_pure_pursuit = false;
  overtake_planner::MpcHealthStatus healthy_mpc;
  healthy_mpc.valid = true;
  healthy_mpc.infeasible_count = 0;
  healthy_mpc.solve_time_ms = 60.0;
  healthy_mpc.age_sec = 0.0;
  release_time += 0.1;
  stopped.stamp_sec = release_time;
  auto healthy_moving_ego = retry_ego;
  healthy_moving_ego.frenet = frame.cartesianToFrenet(5.10, 2.95, 0.0);
  healthy_moving_ego.x = 5.10;
  healthy_moving_ego.y = 2.95;
  // Gate2実測相当の低速域でも、同一PASSの1世代transport continuityを
  // 「primary tracking未準備」と誤解して加速予測を切らない。ここがfalseだと
  // s(t)がPPの必要空間horizonへ届かず、物理的には同じ安全なPASSが
  // untrackable_lateral_profileへ交互に落ちる。
  healthy_moving_ego.v = 0.17;
  const auto healthy_continuous =
      core.update(release_time, healthy_moving_ego, {stopped}, healthy_mpc,
                  healthy_one_generation_lag);
  EXPECT_EQ(healthy_continuous.mode,
            overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(healthy_continuous.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(healthy_continuous.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(healthy_continuous.blocked_info.pass_acceleration_allowed);
  EXPECT_TRUE(healthy_continuous.blocked_info.pass_right_candidate_feasible);
  EXPECT_NE(healthy_continuous.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_NE(healthy_continuous.blocked_info.pass_right_candidate_reject_reason,
            "untrackable_lateral_profile");

  // 走行開始後はplanner/controllerの非同期で1 generation遅れるのが正常。
  // 初回認可済みtransactionかつfinal sourceが実PPの時だけ継続を許可する。
  auto one_generation_lag = readyReentryInput();
  one_generation_lag.pure_pursuit_primary_and_fresh = false;
  one_generation_lag.pure_pursuit_tracking_continuity_usable = true;
  one_generation_lag.pure_pursuit_tracking_target_id = stopped.id;
  one_generation_lag.pure_pursuit_tracking_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  one_generation_lag.mpc_healthy = false;
  one_generation_lag.mpc_latency_warning = true;
  overtake_planner::MpcHealthStatus bounded_latency_mpc;
  bounded_latency_mpc.valid = true;
  bounded_latency_mpc.infeasible_count = 0;
  bounded_latency_mpc.solve_time_ms = 115.0;
  bounded_latency_mpc.age_sec = 0.0;
  stopped.stamp_sec = 0.9;
  auto moving_ego = drifted_ego;
  moving_ego.frenet = frame.cartesianToFrenet(5.20, 2.90, 0.0);
  moving_ego.x = 5.20;
  moving_ego.y = 2.90;
  const auto continuous = core.update(0.9, moving_ego, {stopped},
                                      bounded_latency_mpc, one_generation_lag);
  EXPECT_EQ(continuous.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(continuous.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_FALSE(
      continuous.blocked_info.maneuver_transaction_tracking_stop_active);

  // final controllerがfreshな同一target/sideのPPで、MPC horizonを実際には
  // 使用していない時は、非稼働MPCのstale/solve-timeだけを理由にPASSを
  // 停止しない。SafetyEvaluator、typed tracking identity、1 generation上限は
  // 上のcontinuity契約で満たしたままにする。
  auto inactive_mpc_continuity = one_generation_lag;
  inactive_mpc_continuity.mpc_health_fresh = false;
  inactive_mpc_continuity.mpc_latency_warning = false;
  inactive_mpc_continuity.verified_non_mpc_pure_pursuit = true;
  auto inactive_mpc = bounded_latency_mpc;
  inactive_mpc.solve_time_ms = 905.0;
  inactive_mpc.age_sec = 0.8;
  stopped.stamp_sec = 0.95;
  const auto inactive_mpc_pass = core.update(
      0.95, moving_ego, {stopped}, inactive_mpc, inactive_mpc_continuity);
  EXPECT_EQ(inactive_mpc_pass.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(inactive_mpc_pass.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_FALSE(
      inactive_mpc_pass.blocked_info.maneuver_transaction_tracking_stop_active);

  // 同じcontinuity proofでも上限超過したPASSは許可しない。一方で、別途
  // SafetyEvaluatorとcontroller trackabilityを満たしたATTACK_FOLLOWは継続し、
  // target/sideを失わず次周期もPASS成立を再評価する。
  auto excessive_latency_mpc = bounded_latency_mpc;
  excessive_latency_mpc.solve_time_ms = 121.0;
  stopped.stamp_sec = 1.0;
  const auto excessive_latency = core.update(
      1.0, moving_ego, {stopped}, excessive_latency_mpc, one_generation_lag);
  EXPECT_EQ(excessive_latency.selected,
            overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(excessive_latency.mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_FALSE(
      excessive_latency.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(excessive_latency.blocked_info.attack_follow_hold_pass_side);
  EXPECT_TRUE(excessive_latency.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(excessive_latency.blocked_info.attack_follow_candidate_feasible);
  EXPECT_TRUE(excessive_latency.blocked_info
                  .attack_follow_candidate_tracking_profile_valid);
  EXPECT_TRUE(excessive_latency.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(
      excessive_latency.blocked_info.attack_follow_acceleration_allowed)
      << " pass_accel="
      << excessive_latency.blocked_info.pass_acceleration_allowed
      << " continuity_armed="
      << excessive_latency.blocked_info
             .maneuver_transaction_tracking_continuity_armed
      << " reason=" << excessive_latency.reason;
  EXPECT_EQ(excessive_latency.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(excessive_latency.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  expect_attack_follow_profile(excessive_latency, moving_ego.frenet.d);
}

TEST(OvertakePlannerCore,
     CommittedAttackFollowOpponentCollisionUsesVerifiedCurrentDVariant) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "grid_collision_d2";
  target.v = 0.0;
  target.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {target},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_incomplete);

  ego.v = 1.0;
  target.stamp_sec = 0.2;
  const auto committed =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_EQ(committed.maneuver_latch_target_id, target.id);
  const double committed_target_d_m =
      committed.blocked_info.pass_right_candidate_target_d_m;

  target.stamp_sec = 0.25;
  const auto tracked =
      core.update(0.25, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      tracked.blocked_info.maneuver_transaction_tracking_continuity_armed);

  auto close_target = makeOpponent(frame, 8.0, 1.6);
  close_target.id = target.id;
  close_target.v = 0.0;
  close_target.vx = 0.0;
  close_target.stamp_sec = 0.3;
  auto held_ego = makeEgo(frame, 5.05, 3.0);
  held_ego.v = 1.0;
  const auto held =
      core.update(0.3, held_ego, {close_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_TRUE(held.blocked_info.attack_follow_current_d_hold_variant_generated)
      << " reject=" << held.blocked_info.attack_follow_candidate_reject_reason
      << " tx=" << held.blocked_info.maneuver_transaction_incomplete
      << " latched=" << held.blocked_info.maneuver_target_latched
      << " observed=" << held.blocked_info.maneuver_target_observed
      << " fresh=" << held.blocked_info.maneuver_target_fresh
      << " tail=" << held.blocked_info.maneuver_chain_tail_observed
      << " reauth=" << held.blocked_info.pass_reauthorization_lockout_active
      << " side=" << held.blocked_info.side_by_side
      << " future_yield=" << held.blocked_info.future_yield_required
      << " future_corner=" << held.blocked_info.future_corner_side_by_side
      << " future_wall=" << held.blocked_info.future_outer_wall_risk
      << " reentry_before=" << held.reentry_lockout_before_update
      << " reentry_after=" << held.reentry_lockout_after_update;
  EXPECT_TRUE(held.blocked_info.attack_follow_current_d_hold_variant_feasible)
      << held.blocked_info.attack_follow_current_d_hold_variant_reject_reason;
  EXPECT_TRUE(held.blocked_info.attack_follow_current_d_hold_variant_used);
  EXPECT_TRUE(held.blocked_info
                  .attack_follow_candidate_opponent_collision_current_d_hold);
  EXPECT_EQ(held.blocked_info
                .attack_follow_current_d_hold_source_blocking_opponent_id,
            target.id);
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(held.blocked_info.attack_follow_candidate_committed_target_d_m,
              committed_target_d_m, 1.0e-9);
  EXPECT_NEAR(held.blocked_info.attack_follow_candidate_planned_target_d_m,
              held_ego.frenet.d, 1.0e-9);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
  ASSERT_FALSE(held.lateral_offsets.empty());
  for (const double d_m : held.lateral_offsets) {
    EXPECT_NEAR(d_m, held_ego.frenet.d, 1.0e-9);
  }
}

TEST(
    OvertakePlannerCore,
    FreshMinimumFeasibleInwardConnectorBecomesIndependentAttackFollowCandidate) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    // 0.15 mではfootprint+uncertaintyが上側境界へ残り、0.30 mなら
    // 同じ縦profileのまま初めて成立するC-002AJ実走相当の狭窄を作る。
    corridor.push_back({point.s, -5.0, point.s >= 10.0 ? 3.70 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.wall_footprint_check_enabled = true;
  config.wall_localization_uncertainty_m = 0.25;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "grid_wall_d2";
  target.v = 0.0;
  target.vx = 0.0;
  const auto prepared =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(prepared.blocked_info.maneuver_transaction_incomplete)
      << prepared.blocked_info.pass_right_candidate_reject_reason;

  ego.v = 1.0;
  target.stamp_sec = 0.2;
  const auto committed =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT)
      << committed.reason;
  const double committed_target_d_m =
      committed.blocked_info.pass_right_candidate_target_d_m;

  target.stamp_sec = 0.25;
  const auto tracked =
      core.update(0.25, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      tracked.blocked_info.maneuver_transaction_tracking_continuity_armed);

  auto close_target = makeOpponent(frame, 8.0, 1.6);
  close_target.id = target.id;
  close_target.v = 0.0;
  close_target.vx = 0.0;
  close_target.stamp_sec = 0.3;
  auto held_ego = makeEgo(frame, 5.05, 3.0);
  held_ego.v = 1.0;
  const auto held =
      core.update(0.3, held_ego, {close_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_TRUE(held.blocked_info.attack_follow_current_d_hold_variant_generated);
  ASSERT_EQ(
      held.blocked_info.attack_follow_current_d_hold_variant_reject_reason,
      "wall_footprint_margin");
  EXPECT_FALSE(held.blocked_info.attack_follow_current_d_hold_variant_used);
  const auto &diagnostic = held.attack_follow_inner_band_diagnostic;
  EXPECT_TRUE(diagnostic.evaluated);
  EXPECT_TRUE(diagnostic.complete) << diagnostic.status_reason;
  EXPECT_LE(diagnostic.evaluated_probe_count, 4);
  EXPECT_GT(diagnostic.evaluated_probe_count, 0);
  EXPECT_EQ(diagnostic.target_id, target.id);
  EXPECT_EQ(diagnostic.pass_type, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(diagnostic.committed_target_d_m, committed_target_d_m, 1.0e-9);
  for (int i = 0; i < diagnostic.evaluated_probe_count; ++i) {
    const auto &probe = diagnostic.probes[static_cast<std::size_t>(i)];
    EXPECT_TRUE(probe.longitudinal_contract_unchanged);
    EXPECT_FALSE(probe.terminal_d_m == held_ego.frenet.d);
  }
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(
      held.blocked_info.attack_follow_inward_connector_variant_generated);
  EXPECT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_feasible)
      << held.blocked_info.attack_follow_inward_connector_variant_reject_reason;
  EXPECT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_used);
  EXPECT_TRUE(held.blocked_info
                  .attack_follow_candidate_opponent_collision_inward_connector);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(held.active_override);
  EXPECT_TRUE(held.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(held.published_lateral_safety_rejected);
  EXPECT_FALSE(held.tracking_release_pass_warmup);
  EXPECT_EQ(held.tracking_release_token, 0U);
  ASSERT_FALSE(held.lateral_offsets.empty());
  EXPECT_NEAR(held.lateral_offsets.front(), held_ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(held.lateral_offsets.back(), held_ego.frenet.d - 0.30, 1.0e-9);
  EXPECT_NEAR(held.blocked_info.attack_follow_inward_connector_terminal_d_m,
              held_ego.frenet.d - 0.30, 1.0e-9);
  EXPECT_NEAR(held.blocked_info.attack_follow_candidate_committed_target_d_m,
              committed_target_d_m, 1.0e-9);
  const auto connector_wire =
      overtake_planner::makeReferenceOverrideWirePayload(held, 91U);
  EXPECT_EQ(connector_wire.kind, overtake_planner::ReferenceOverrideWireKind::
                                     SPATIAL_LATERAL_AND_SPEED_V4);
  const auto connector_identity =
      overtake_planner::makeOverrideTrackingIdentity(held, connector_wire, 91U,
                                                     1U);
  EXPECT_EQ(connector_identity.kind,
            overtake_planner::OverrideTrackingIdentityKind::ATTACK_FOLLOW);
  EXPECT_EQ(connector_identity.target_id, target.id);
  EXPECT_EQ(connector_identity.pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  const auto connector_constraint = overtake_planner::makeSafetyConstraint(
      held, held_ego, readyReentryInput(), 8.0, 1.0);
  EXPECT_TRUE(connector_constraint.valid);
  EXPECT_FALSE(connector_constraint.stop_requested);

  close_target.stamp_sec = 0.35;
  auto next_ego = makeEgo(frame, 5.05, 2.99);
  next_ego.v = held_ego.v;
  const auto next_cycle =
      core.update(0.35, next_ego, {close_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(
      next_cycle.blocked_info.attack_follow_inward_connector_variant_generated);
  EXPECT_TRUE(
      next_cycle.blocked_info.attack_follow_inward_connector_variant_used);
  ASSERT_FALSE(next_cycle.lateral_offsets.empty());
  EXPECT_NEAR(next_cycle.lateral_offsets.front(), next_ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(next_cycle.lateral_offsets.back(), next_ego.frenet.d - 0.30,
              1.0e-9);

  ASSERT_FALSE(held.longitudinal_offsets_m.empty());
  const double connector_blocker_s_m =
      next_ego.frenet.s + held.longitudinal_offsets_m.back();
  auto connector_blocker =
      makeOpponent(frame, connector_blocker_s_m, next_ego.frenet.d - 0.55);
  connector_blocker.id = "connector_blocker";
  connector_blocker.v = 0.0;
  connector_blocker.vx = 0.0;
  connector_blocker.stamp_sec = 0.4;
  close_target.stamp_sec = 0.4;
  const auto blocked_connector =
      core.update(0.4, next_ego, {close_target, connector_blocker},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(blocked_connector.blocked_info
                  .attack_follow_inward_connector_variant_generated);
  EXPECT_FALSE(blocked_connector.blocked_info
                   .attack_follow_inward_connector_variant_feasible);
  EXPECT_FALSE(blocked_connector.blocked_info
                   .attack_follow_inward_connector_variant_used);
  EXPECT_EQ(blocked_connector.blocked_info
                .attack_follow_inward_connector_variant_reject_reason,
            "opponent_collision");
  EXPECT_FALSE(
      blocked_connector.blocked_info
          .attack_follow_candidate_opponent_collision_inward_connector);
  const auto blocked_connector_wire =
      overtake_planner::makeReferenceOverrideWirePayload(blocked_connector,
                                                         93U);
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(
                blocked_connector, blocked_connector_wire, 93U, 1U)
                .kind,
            overtake_planner::OverrideTrackingIdentityKind::NONE);
  EXPECT_EQ(blocked_connector.maneuver_latch_target_id, target.id);
  EXPECT_EQ(blocked_connector.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);

  const auto stale_cycle =
      core.update(1.0, next_ego, {close_target, connector_blocker},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_FALSE(stale_cycle.blocked_info
                   .attack_follow_inward_connector_variant_generated);
  EXPECT_FALSE(
      stale_cycle.blocked_info.attack_follow_inward_connector_variant_used);
  EXPECT_FALSE(stale_cycle.blocked_info
                   .attack_follow_candidate_opponent_collision_inward_connector)
      << "cached C-002AI shadow result must never authorize a stale cycle";
  const auto stale_wire =
      overtake_planner::makeReferenceOverrideWirePayload(stale_cycle, 92U);
  EXPECT_EQ(overtake_planner::makeOverrideTrackingIdentity(stale_cycle,
                                                           stale_wire, 92U, 1U)
                .kind,
            overtake_planner::OverrideTrackingIdentityKind::NONE);
}

TEST(OvertakePlannerCore,
     DISABLED_FixedConnectorsRejectedButBaselineFollowBecomesAuthorized) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -5.0, point.s >= 18.0 ? 1.40 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 200;
  config.horizon_dt_sec = 0.025;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.wall_footprint_check_enabled = true;
  config.wall_localization_uncertainty_m = 0.25;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.0);
  ego.v = 2.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "direct_wall_d2";
  target.v = 0.0;
  target.vx = 0.0;
  const auto prepared =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(prepared.blocked_info.maneuver_transaction_incomplete);

  ego.v = 1.0;
  target.stamp_sec = 0.2;
  const auto committed =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT)
      << committed.reason;

  target.stamp_sec = 0.25;
  const auto tracked =
      core.update(0.25, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      tracked.blocked_info.maneuver_transaction_tracking_continuity_armed);

  // 相手はfreshなcommit対象として残すが、評価horizonの衝突楕円より前方へ
  // 置く。これによりATTACK_FOLLOW sourceの最初の棄却をwall
  // footprintへ固定する。
  auto wall_approach_ego = makeEgo(frame, 12.0, 1.0);
  wall_approach_ego.v = 4.0;
  target.frenet.s = 22.0;
  target.v = 3.0;
  target.vx = 3.0;
  const auto target_xy =
      frame.frenetToCartesian(target.frenet.s, target.frenet.d);
  target.x = target_xy.x;
  target.y = target_xy.y;
  target.stamp_sec = 0.3;
  const auto held =
      core.update(0.3, wall_approach_ego, {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_EQ(held.blocked_info.attack_follow_candidate_reject_reason,
            "wall_footprint_margin");
  EXPECT_TRUE(held.blocked_info.attack_follow_current_d_hold_variant_generated);
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_FALSE(held.blocked_info.attack_follow_current_d_hold_variant_feasible);
  EXPECT_FALSE(held.blocked_info.attack_follow_current_d_hold_variant_used);
  EXPECT_TRUE(
      held.blocked_info.attack_follow_inward_connector_variant_generated);
  EXPECT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_feasible)
      << held.blocked_info.attack_follow_inward_connector_variant_reject_reason;
  EXPECT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_used);
  EXPECT_NEAR(held.blocked_info.attack_follow_inward_connector_terminal_d_m,
              0.0, 1.0e-9);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(held.active_override);
  EXPECT_TRUE(held.selected_lateral_profile_safety_verified);
  ASSERT_FALSE(held.lateral_offsets.empty());
  EXPECT_NEAR(held.lateral_offsets.front(), wall_approach_ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(held.lateral_offsets.back(), 0.0, 1.0e-9);
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  const auto constraint = overtake_planner::makeSafetyConstraint(
      held, wall_approach_ego, readyReentryInput(), 8.0, 1.0);
  EXPECT_TRUE(constraint.valid);
  EXPECT_FALSE(constraint.stop_requested);
}

TEST(OvertakePlannerCore,
     FirstFeasibleInwardConnectorNeverSkipsValidMinimumShift) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    // 0.15 m connectorが最初からwall reserveを満たす幅を与える。
    corridor.push_back({point.s, -5.0, point.s >= 10.0 ? 3.85 : 5.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.wall_footprint_check_enabled = true;
  config.wall_localization_uncertainty_m = 0.25;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "minimum_connector_d2";
  target.v = 0.0;
  target.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {target},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_incomplete);

  ego.v = 1.0;
  target.stamp_sec = 0.2;
  const auto committed =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  const double committed_target_d_m =
      committed.blocked_info.pass_right_candidate_target_d_m;

  target.stamp_sec = 0.25;
  ASSERT_TRUE(core.update(0.25, ego, {target},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_tracking_continuity_armed);

  auto close_target = makeOpponent(frame, 8.0, 1.6);
  close_target.id = target.id;
  close_target.v = 0.0;
  close_target.vx = 0.0;
  close_target.stamp_sec = 0.3;
  auto held_ego = makeEgo(frame, 5.05, 3.0);
  held_ego.v = 1.0;
  const auto held =
      core.update(0.3, held_ego, {close_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_TRUE(
      held.blocked_info.attack_follow_inward_connector_variant_generated);
  ASSERT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_feasible)
      << held.blocked_info.attack_follow_inward_connector_variant_reject_reason;
  ASSERT_TRUE(held.blocked_info.attack_follow_inward_connector_variant_used);
  ASSERT_FALSE(held.lateral_offsets.empty());
  EXPECT_NEAR(held.blocked_info.attack_follow_inward_connector_terminal_d_m,
              held_ego.frenet.d - 0.15, 1.0e-9);
  EXPECT_NEAR(held.lateral_offsets.back(), held_ego.frenet.d - 0.15, 1.0e-9);
  EXPECT_GT(std::abs(held.lateral_offsets.back() - (held_ego.frenet.d - 0.30)),
            0.10);
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NEAR(held.blocked_info.attack_follow_candidate_committed_target_d_m,
              committed_target_d_m, 1.0e-9);
  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(held, 94U);
  EXPECT_EQ(wire.kind, overtake_planner::ReferenceOverrideWireKind::
                           SPATIAL_LATERAL_AND_SPEED_V4);
  EXPECT_EQ(
      overtake_planner::makeOverrideTrackingIdentity(held, wire, 94U, 1U).kind,
      overtake_planner::OverrideTrackingIdentityKind::ATTACK_FOLLOW);
}

TEST(AttackFollowInwardConnectorContract,
     RejectsCenterReserveCrossingAndNonFiniteInputs) {
  constexpr double kReserveM = 0.05;
  EXPECT_TRUE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      3.0, 2.85, kReserveM));
  EXPECT_TRUE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      -3.0, -2.85, kReserveM));
  EXPECT_FALSE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      0.05, -0.10, kReserveM));
  EXPECT_FALSE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      0.04, -0.11, kReserveM));
  EXPECT_FALSE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      -0.05, 0.10, kReserveM));
  EXPECT_FALSE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      -0.04, 0.11, kReserveM));
  EXPECT_FALSE(overtake_planner::inwardConnectorStaysOutsideCenterReserve(
      std::numeric_limits<double>::quiet_NaN(), 0.20, kReserveM));
}

TEST(OvertakePlannerCore,
     CommittedPassAdvancesFreshlyReevaluatedWireAcrossCorrectionDeadline) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.pass_speed_cap_mps = 0.75;
  config.pass_lateral_tracking_lag_threshold_m = 0.01;
  config.pass_lateral_tracking_lag_speed_cap_mps = 0.30;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.same_corridor_width_m = 0.90;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 3.0;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.5;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 4.0;
  auto stopped = makeOpponent(frame, 12.0, 2.8);
  stopped.id = "grid_wire_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {stopped},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.maneuver_transaction_incomplete);

  ego.v = 1.0;
  stopped.stamp_sec = 0.2;
  const auto armed =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(armed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_EQ(armed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_EQ(armed.lateral_offsets.size(), armed.longitudinal_offsets_m.size());
  ASSERT_FALSE(armed.lateral_offsets.empty());

  const auto sample_armed_d = [&](double offset_m) {
    const auto upper =
        std::lower_bound(armed.longitudinal_offsets_m.begin(),
                         armed.longitudinal_offsets_m.end(), offset_m);
    if (upper == armed.longitudinal_offsets_m.begin()) {
      return armed.lateral_offsets.front();
    }
    if (upper == armed.longitudinal_offsets_m.end()) {
      return armed.lateral_offsets.back();
    }
    const std::size_t upper_index = static_cast<std::size_t>(
        std::distance(armed.longitudinal_offsets_m.begin(), upper));
    const std::size_t lower_index = upper_index - 1U;
    const double span_m = armed.longitudinal_offsets_m[upper_index] -
                          armed.longitudinal_offsets_m[lower_index];
    const double ratio =
        (offset_m - armed.longitudinal_offsets_m[lower_index]) / span_m;
    return armed.lateral_offsets[lower_index] +
           ratio * (armed.lateral_offsets[upper_index] -
                    armed.lateral_offsets[lower_index]);
  };

  bool used_fresh_snapshot_shape = false;
  double progress_at_freeze_m = 0.0;
  double now_sec = 0.2;
  // Gate2認可時に最初にpublishしたwireを固定する。target dへ届くまでrolling
  // current-d候補へ差し替えると、後の短い残距離へ横移動が圧縮される。
  for (int cycle = 1; cycle <= 80; ++cycle) {
    const double progress_m = 0.05 * static_cast<double>(cycle);
    const double nominal_d_m = sample_armed_d(progress_m);
    const double tracking_lag_m = 0.44 * std::clamp(progress_m / 1.5, 0.0, 1.0);
    ego = makeEgo(frame, 5.0 + progress_m, nominal_d_m + tracking_lag_m);
    ego.v = 0.57;
    now_sec += 0.05;
    stopped.stamp_sec = now_sec;
    const auto output =
        core.update(now_sec, ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    ASSERT_NE(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY)
        << "cycle=" << cycle << " pass_reject="
        << output.blocked_info.pass_right_candidate_reject_reason;
    ASSERT_EQ(output.maneuver_latch_target_id, stopped.id);
    ASSERT_EQ(output.blocked_info.maneuver_transaction_pass_type,
              overtake_planner::CandidateType::PASS_RIGHT);
    if (output.blocked_info.committed_pass_spatial_profile_continuity_used) {
      EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
      EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
      EXPECT_TRUE(output.blocked_info.pass_right_candidate_feasible);
      EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
      ASSERT_FALSE(output.lateral_offsets.empty());
      EXPECT_NEAR(output.lateral_offsets.front(), ego.frenet.d, 1.0e-9);
      EXPECT_GT(
          output.blocked_info.committed_pass_spatial_profile_tracking_error_m,
          config.pass_lateral_tracking_lag_threshold_m);
      EXPECT_FALSE(output.blocked_info.pass_lateral_clearance_ready)
          << "actual=" << output.blocked_info.pass_lateral_separation_actual_m
          << " required="
          << output.blocked_info.pass_lateral_separation_required_m
          << " target=" << output.blocked_info.pass_lateral_first_target_id;
      EXPECT_TRUE(output.blocked_info.pass_lateral_first_speed_gate_active);
      EXPECT_NEAR(output.blocked_info.pass_lateral_first_speed_cap_mps,
                  config.pass_lateral_tracking_lag_speed_cap_mps, 1.0e-9);
      EXPECT_TRUE(std::all_of(
          output.speed_caps.begin(), output.speed_caps.end(),
          [&](double speed_mps) {
            return speed_mps <=
                   config.pass_lateral_tracking_lag_speed_cap_mps + 1.0e-9;
          }));
      used_fresh_snapshot_shape = true;
      progress_at_freeze_m = progress_m;
      break;
    }
  }
  EXPECT_TRUE(used_fresh_snapshot_shape);
  EXPECT_LE(progress_at_freeze_m, 0.05 + 1.0e-9);

  // 0.25秒を越えた後も旧Safety proofは使わず、geometryだけを固定したまま、
  // 時刻列・全相手予測・SafetyEvaluatorを毎周期更新してPASS候補を継続する。
  // 前周期のControllerTrackingStatusを候補生成条件へ循環させない。
  bool saw_spatial_profile_after_snapshot_expiry = false;
  for (int cycle = 0; cycle < 8; ++cycle) {
    now_sec += 0.05;
    stopped.stamp_sec = now_sec;
    const auto output =
        core.update(now_sec, ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    ASSERT_TRUE(
        output.blocked_info.committed_pass_spatial_profile_continuity_used)
        << "cycle=" << cycle
        << " reject=" << output.blocked_info.pass_right_candidate_reject_reason;
    ASSERT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
    ASSERT_EQ(output.maneuver_latch_target_id, stopped.id);
    ASSERT_TRUE(output.selected_lateral_profile_safety_verified);
    if (!output.blocked_info.committed_pass_snapshot_continuity_used) {
      EXPECT_GT(
          output.blocked_info.committed_pass_spatial_profile_source_age_sec,
          0.25);
      saw_spatial_profile_after_snapshot_expiry = true;
    }
  }
  EXPECT_TRUE(saw_spatial_profile_after_snapshot_expiry);

  // PASS_RIGHT方向へ固定wireより先行した実測dは、差の絶対値だけで拒否しない。
  // 実測d connectorを含む最終形状がfresh SafetyEvaluatorと
  // controller trackabilityを通った時だけPASSを継続する。
  for (int cycle = 0; cycle < 6; ++cycle) {
    auto ahead_ego = makeEgo(frame, ego.x + 0.01, ego.frenet.d - 0.035);
    ahead_ego.v = 0.20;
    now_sec += 0.05;
    stopped.stamp_sec = now_sec;
    const auto preserved_ahead_progress =
        core.update(now_sec, ahead_ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    ASSERT_TRUE(preserved_ahead_progress.blocked_info
                    .committed_pass_spatial_profile_continuity_used)
        << "cycle=" << cycle << " reject="
        << preserved_ahead_progress.blocked_info
               .pass_right_candidate_reject_reason;
    ASSERT_EQ(preserved_ahead_progress.selected,
              overtake_planner::CandidateType::PASS_RIGHT);
    ASSERT_TRUE(
        preserved_ahead_progress.selected_lateral_profile_safety_verified);
    EXPECT_TRUE(std::all_of(
        preserved_ahead_progress.lateral_offsets.begin(),
        preserved_ahead_progress.lateral_offsets.end(),
        [&](double d_m) { return d_m <= ahead_ego.frenet.d + 1.0e-9; }));
    ego = ahead_ego;
  }

  // 実Gate2では一度STOPした後、速度ほぼ0のraw PASSがendpoint=0.12 m、
  // required=3.65 mとなり、前周期のPP proofが無いこと自体で加速s(t)を
  // 作れず自己ロックした。固定geometryでは加速候補をfresh SafetyEvaluatorへ
  // 載せられ、現在周期の全入力で認可された時だけ再始動できることを確認する。
  auto stopped_ego = ego;
  stopped_ego.v = 0.0;
  now_sec += 0.05;
  stopped.stamp_sec = now_sec;
  const auto restarted_from_stop =
      core.update(now_sec, stopped_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_TRUE(restarted_from_stop.blocked_info
                  .committed_pass_spatial_profile_continuity_used)
      << "reject="
      << restarted_from_stop.blocked_info.pass_right_candidate_reject_reason
      << " endpoint="
      << restarted_from_stop.blocked_info.pass_right_candidate_endpoint_arc_m
      << " required="
      << restarted_from_stop.blocked_info.pass_right_candidate_required_arc_m;
  ASSERT_EQ(restarted_from_stop.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
  ASSERT_TRUE(restarted_from_stop.selected_lateral_profile_safety_verified);
  EXPECT_GE(
      restarted_from_stop.blocked_info.pass_right_candidate_endpoint_arc_m +
          1.0e-6,
      restarted_from_stop.blocked_info.pass_right_candidate_required_arc_m);

  // target単体がfreshでも全観測相手の包含証明を失った周期は、固定geometryを
  // SafetyEvaluatorへ載せない。欠落相手を「衝突なし」と誤認しない。
  auto incomplete_inputs = readyReentryInput();
  incomplete_inputs.all_observed_opponents_included = false;
  now_sec += 0.01;
  stopped.stamp_sec = now_sec;
  const auto incomplete =
      core.update(now_sec, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  incomplete_inputs);
  EXPECT_FALSE(
      incomplete.blocked_info.committed_pass_spatial_profile_continuity_used);

  // fresh入力が揃っていても、実車dが固定geometryから設定上限を超えて外れた時は
  // connectorで救済せず、raw localized PASSへも戻らない。
  auto tracking_error_ego = makeEgo(frame, ego.x, ego.frenet.d + 1.00);
  tracking_error_ego.v = ego.v;
  now_sec += 0.05;
  stopped.stamp_sec = now_sec;
  const auto excessive_tracking_error =
      core.update(now_sec, tracking_error_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_FALSE(excessive_tracking_error.blocked_info
                   .committed_pass_spatial_profile_continuity_used);
  EXPECT_NE(excessive_tracking_error.selected,
            overtake_planner::CandidateType::PASS_RIGHT);

  // d列が似ていても別IDへは継続権限を移さない。旧snapshotの安全結果も
  // target missingを埋める認可には使わず、通常の保持/停止判断へ閉じる。
  stopped.id = "grid_wire_d3";
  now_sec += 0.05;
  stopped.stamp_sec = now_sec;
  const auto changed_target =
      core.update(now_sec, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_FALSE(
      changed_target.blocked_info.committed_pass_snapshot_continuity_used);
  EXPECT_FALSE(changed_target.blocked_info
                   .committed_pass_spatial_profile_continuity_used);
  EXPECT_NE(changed_target.selected,
            overtake_planner::CandidateType::PASS_RIGHT);
}

TEST(OvertakePlannerCore,
     CommittedStartGridReleaseKeepsStopButRevokesLateralOnIncompleteInputs) {
  enum class IncompleteReleaseInput {
    OPPONENT_EXCLUDED,
    V2X_STALE,
    OPPONENT_STALE,
    PREDICTION_INCOMPLETE,
  };

  for (const auto incomplete_case :
       {IncompleteReleaseInput::OPPONENT_EXCLUDED,
        IncompleteReleaseInput::V2X_STALE,
        IncompleteReleaseInput::OPPONENT_STALE,
        IncompleteReleaseInput::PREDICTION_INCOMPLETE}) {
    SCOPED_TRACE(static_cast<int>(incomplete_case));
    const auto frame = makeStraightFrame();
    auto config = makeConfig();
    // 最終PP wireをACK対象にするrelease契約なので、実運用と同じsamplingで
    // trackabilityを成立させ、各subcaseは入力欠損だけを検証する。
    config.horizon_points = 50;
    config.horizon_dt_sec = 0.025;
    // このfixtureは、天候・実車条件で操舵包絡を保守設定へ下げた場合の
    // release取消を検証する。production default 0.64/128とは独立に固定する。
    config.attack_follow_max_steering_angle_rad = 0.3665191429188092;
    config.attack_follow_max_steering_rate_radps = 8.0;
    config.start_grid_target_enabled = true;
    config.start_grid_target_window_sec = 5.0;
    config.start_grid_target_window_distance_m = 8.0;
    config.start_grid_target_max_ego_speed_mps = 3.0;
    config.start_grid_target_lateral_width_m = 2.0;
    config.same_corridor_width_m = 0.90;
    config.early_stationary_parallel_pass_enabled = true;
    config.early_stationary_parallel_pass_distance_m = 8.0;
    config.early_stationary_parallel_pass_lateral_width_m = 2.0;
    config.dynamic_pass_candidate_enabled = true;
    config.overtake_lateral_profile_mode = "localized_latched";
    config.pass_target_policy = "minimum_clearance";
    config.maneuver_latch_min_hold_sec = 0.0;
    config.pass_safe_required_cycles = 1.0;
    config.min_mode_hold_time_sec = 0.0;
    config.min_pass_gap_m = 3.0;
    config.d_min_m = -5.0;
    config.d_max_m = 5.0;
    config.min_wall_margin_m = 0.5;
    config.safety_ellipse_a_m = 0.8;
    config.safety_ellipse_b_m = 0.5;
    config.min_ellipse_h = 0.1;
    overtake_planner::OvertakePlannerCore core(frame, config);

    auto ego = makeEgo(frame, 5.0, 3.0);
    ego.v = 4.0;
    auto stopped = makeOpponent(frame, 12.0, 1.6);
    stopped.id = "grid_release_input_d2";
    stopped.v = 0.0;
    stopped.vx = 0.0;
    ASSERT_TRUE(core.update(0.1, ego, {stopped},
                            overtake_planner::MpcHealthStatus{},
                            readyReentryInput())
                    .blocked_info.maneuver_transaction_incomplete);

    ego.v = 1.0;
    stopped.stamp_sec = 0.2;
    const auto armed =
        core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    ASSERT_TRUE(armed.blocked_info.start_grid_target_active);
    ASSERT_EQ(armed.selected, overtake_planner::CandidateType::PASS_RIGHT);
    ASSERT_EQ(armed.maneuver_latch_target_id, stopped.id);

    auto drifted_ego = makeEgo(frame, 5.05, 3.0);
    drifted_ego.v = 1.0;
    stopped.stamp_sec = 0.3;
    auto transient_tracking = readyReentryInput();
    transient_tracking.mpc_healthy = false;
    transient_tracking.mpc_health_fresh = true;
    transient_tracking.mpc_hard_failure = false;
    transient_tracking.mpc_latency_warning = true;
    transient_tracking.pure_pursuit_primary_and_fresh = false;
    overtake_planner::MpcHealthStatus transient_mpc;
    transient_mpc.valid = true;
    transient_mpc.infeasible_count = 1;
    transient_mpc.solve_time_ms = 466.95;
    transient_mpc.age_sec = 0.0;
    transient_mpc.sample_sequence = 1U;
    const auto held = core.update(0.3, drifted_ego, {stopped}, transient_mpc,
                                  transient_tracking);
    ASSERT_TRUE(held.blocked_info.maneuver_transaction_incomplete);
    ASSERT_EQ(held.selected, overtake_planner::CandidateType::FOLLOW);
    ASSERT_FALSE(held.blocked_info.maneuver_transaction_tracking_stop_active);
    ASSERT_TRUE(held.blocked_info.attack_follow_hold_pass_side);
    ASSERT_TRUE(held.blocked_info.attack_follow_candidate_feasible);
    ASSERT_TRUE(held.selected_lateral_profile_safety_verified);

    stopped.stamp_sec = 0.4;
    auto release_proof = readyReentryInput();
    release_proof.pure_pursuit_primary_and_fresh = false;
    release_proof.pure_pursuit_release_ready = true;
    const auto warmup =
        core.update(0.4, drifted_ego, {stopped},
                    overtake_planner::MpcHealthStatus{}, release_proof);
    ASSERT_TRUE(
        warmup.blocked_info.maneuver_transaction_tracking_release_pending);
    ASSERT_TRUE(warmup.lateral_tracking_authorized_during_stop);

    stopped.stamp_sec = 0.5;
    auto incomplete = release_proof;
    std::vector<overtake_planner::OpponentState> opponents{stopped};
    if (incomplete_case == IncompleteReleaseInput::OPPONENT_EXCLUDED) {
      incomplete.all_observed_opponents_included = false;
    } else if (incomplete_case == IncompleteReleaseInput::V2X_STALE) {
      incomplete.v2x_snapshot_fresh = false;
    } else if (incomplete_case == IncompleteReleaseInput::OPPONENT_STALE) {
      incomplete.all_observed_opponents_fresh = false;
    } else {
      auto missing_prediction = makeOpponent(frame, 30.0, -3.0);
      missing_prediction.id = "invalid_prediction_source";
      missing_prediction.stamp_sec = 0.5;
      missing_prediction.valid = false;
      opponents.push_back(missing_prediction);
    }
    const auto revoked =
        core.update(0.5, drifted_ego, opponents,
                    overtake_planner::MpcHealthStatus{}, incomplete);

    EXPECT_EQ(revoked.maneuver_latch_target_id, stopped.id);
    EXPECT_EQ(revoked.blocked_info.maneuver_transaction_pass_type,
              overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_TRUE(revoked.blocked_info.maneuver_transaction_incomplete);
    EXPECT_FALSE(
        revoked.blocked_info.maneuver_transaction_tracking_release_confirmed);
    EXPECT_FALSE(revoked.lateral_stop_inputs_complete);
    EXPECT_FALSE(revoked.lateral_tracking_authorized_during_stop);
    const auto stop_constraint = overtake_planner::makeSafetyConstraint(
        revoked, drifted_ego, incomplete, 8.0, 1.0);
    EXPECT_TRUE(stop_constraint.valid);
    EXPECT_TRUE(stop_constraint.stop_requested);
  }
}

TEST(OvertakePlannerCore,
     AuthorizedPassEnvelopeFailsClosedWhenNearTargetProfileIsUntrackable) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 0.2;
  config.side_by_side_s_m = 0.01;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  // PASS自体のGate 2には影響せず、ATTACK_FOLLOWの再生成だけを必要arc不足に
  // するfixture。物理操舵上限を0にするとPASSまで失格になるため使わない。
  config.attack_follow_min_spatial_horizon_m = 20.0;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  // 保守操舵包絡でnear-target PASSが不成立になるfail-closed経路を固定する。
  config.attack_follow_max_steering_angle_rad = 0.3665191429188092;
  config.attack_follow_max_steering_rate_radps = 8.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 4.0;
  auto stopped = makeOpponent(frame, 13.0, 0.0);
  stopped.id = "near_target_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  const auto prepare = core.update(0.1, ego, {stopped},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_EQ(prepare.selected, overtake_planner::CandidateType::PASS_LEFT);
  ASSERT_TRUE(prepare.blocked_info.maneuver_transaction_incomplete);

  ego.v = 4.0;
  stopped.stamp_sec = 0.2;
  const auto passing = core.update(0.2, ego, {stopped},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_EQ(passing.selected, overtake_planner::CandidateType::PASS_LEFT);
  // Gate 2認可直後でも、実車がまだstart dにいる間は制動距離不足を免除する
  // current-d holdへ昇格してはいけない。
  EXPECT_FALSE(passing.blocked_info.authorized_pass_current_d_hold_active);
  const double authorized_d_m =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(authorized_d_m));
  ASSERT_GT(std::abs(authorized_d_m - ego.frenet.d), 1.0e-3);

  // Gate 2認可済みの左側dへ到達した直後、停止対象まで0.39 mになる実ログ相当。
  // 低速PASSはPP必要arcを評価上限内に作れず失格になるが、対象とは横に分離済み。
  // PASSを例外合格にせず、同じtarget/sideを保持する。現在d RECOVERYも
  // PP必要arcを証明できない場合は、未証明の横holdをpublishせず縦STOPへ閉じる。
  auto near_ego = makeEgo(frame, stopped.frenet.s - 0.39, authorized_d_m);
  near_ego.v = 0.64;
  stopped.stamp_sec = 0.3;
  const auto held = core.update(0.3, near_ego, {stopped},
                                overtake_planner::MpcHealthStatus{}, input);

  EXPECT_EQ(held.blocked_info.pass_left_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_FALSE(held.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(held.blocked_info.authorized_pass_current_d_hold_active)
      << "progress_m=" << held.blocked_info.maneuver_pass_lateral_progress_m
      << " ego_d=" << held.blocked_info.ego_lateral_offset_m
      << " authorized_d=" << authorized_d_m
      << " candidate_d=" << held.blocked_info.pass_left_candidate_target_d_m
      << " chain_count=" << held.blocked_info.maneuver_chain_target_count;
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_safe_lateral_hold_active)
      << "attack_hold=" << held.blocked_info.attack_follow_hold_pass_side
      << " attack_generated="
      << held.blocked_info.attack_follow_candidate_generated
      << " attack_feasible="
      << held.blocked_info.attack_follow_candidate_feasible
      << " attack_tracking="
      << held.blocked_info.attack_follow_candidate_tracking_profile_valid
      << " attack_reason="
      << held.blocked_info.attack_follow_candidate_reject_reason
      << " start_grid=" << held.blocked_info.start_grid_target_active;
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(held.safe_stop_triggered);
  EXPECT_FALSE(held.reentry_gate.requested);
  EXPECT_EQ(held.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(held.active_override);
  EXPECT_FALSE(held.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(held.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_TRUE(held.lateral_offsets.empty());
  EXPECT_TRUE(held.longitudinal_offsets_m.empty());
  EXPECT_TRUE(held.longitudinal_speed_cap_active);
  EXPECT_TRUE(held.speed_only_fallback_active);
  ASSERT_FALSE(held.speed_caps.empty());
  EXPECT_TRUE(std::all_of(held.speed_caps.begin(), held.speed_caps.end(),
                          [&config](double speed_mps) {
                            return speed_mps <= config.safe_stop_v_mps;
                          }));

  // front分類から外れる真横直後でもtransactionのtarget/sideは保持するが、
  // current-d profileのPP必要arcを証明できない限り横holdは再認可しない。
  auto just_behind_ego = near_ego;
  just_behind_ego.x = stopped.x + 0.20;
  just_behind_ego.frenet =
      frame.cartesianToFrenet(just_behind_ego.x, just_behind_ego.y, 0.0);
  stopped.stamp_sec = 0.4;
  const auto still_held =
      core.update(0.4, just_behind_ego, {stopped},
                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(still_held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(still_held.blocked_info.authorized_pass_current_d_hold_active);
  EXPECT_TRUE(
      still_held.blocked_info.maneuver_transaction_safe_lateral_hold_active);
  EXPECT_EQ(still_held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NE(still_held.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(still_held.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_EQ(still_held.maneuver_latch_target_id, stopped.id);
  EXPECT_FALSE(still_held.active_override);
  EXPECT_FALSE(still_held.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(still_held.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_TRUE(still_held.lateral_offsets.empty());
  EXPECT_TRUE(still_held.longitudinal_offsets_m.empty());
  EXPECT_TRUE(still_held.longitudinal_speed_cap_active);
  ASSERT_FALSE(still_held.speed_caps.empty());
  EXPECT_TRUE(std::all_of(still_held.speed_caps.begin(),
                          still_held.speed_caps.end(),
                          [&config](double speed_mps) {
                            return speed_mps <= config.safe_stop_v_mps;
                          }));
}

TEST(OvertakePlannerCore,
     IncompletePassStopsAtCurrentDInsteadOfYieldingMidLateralShift) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 0.2;
  config.side_by_side_s_m = 0.01;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  config.attack_follow_min_spatial_horizon_m = 20.0;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  // 保守操舵包絡でmid-shift PASSが不成立になるfail-closed経路を固定する。
  config.attack_follow_max_steering_angle_rad = 0.3665191429188092;
  config.attack_follow_max_steering_rate_radps = 8.0;
  // 直前PASS列との周期間補間が有効だとcurrent-d STOPを変形する小さい値。
  // transaction HOLDではこのrate limitを適用せず、評価済み定数dを保つ。
  config.lateral_target_max_step_m = 0.01;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 4.0;
  auto stopped = makeOpponent(frame, 13.0, 0.0);
  stopped.id = "mid_shift_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  const auto prepare = core.update(0.1, ego, {stopped},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.selected, overtake_planner::CandidateType::PASS_LEFT);

  stopped.stamp_sec = 0.2;
  const auto passing = core.update(0.2, ego, {stopped},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_EQ(passing.selected, overtake_planner::CandidateType::PASS_LEFT);
  const double authorized_d_m =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(authorized_d_m));

  // 横目標へまだ半分しか移動していないまま対象へ接近し、同側PASSが
  // untrackableになった周期をGate2-07相当として再現する。
  auto mid_shift_ego =
      makeEgo(frame, stopped.frenet.s - 0.39, authorized_d_m * 0.5);
  mid_shift_ego.v = 0.64;
  stopped.stamp_sec = 0.3;
  const auto held = core.update(0.3, mid_shift_ego, {stopped},
                                overtake_planner::MpcHealthStatus{}, input);

  EXPECT_EQ(held.blocked_info.pass_left_candidate_reject_reason,
            "untrackable_lateral_profile");
  EXPECT_FALSE(held.blocked_info.authorized_pass_current_d_hold_active);
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(held.blocked_info.attack_follow_candidate_generated);
  EXPECT_TRUE(
      !held.blocked_info.attack_follow_candidate_feasible ||
      !held.blocked_info.attack_follow_candidate_tracking_profile_valid);
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(held.tracking_release_pass_warmup);
  EXPECT_EQ(held.tracking_release_token, 0U);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_FALSE(held.reentry_gate.requested);
  EXPECT_EQ(held.maneuver_latch_target_id, stopped.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_FALSE(held.active_override);
  EXPECT_FALSE(held.lateral_tracking_authorized_during_stop);
  EXPECT_TRUE(held.lateral_offsets.empty());
  EXPECT_TRUE(held.longitudinal_offsets_m.empty());
  EXPECT_TRUE(held.longitudinal_speed_cap_active);
  ASSERT_FALSE(held.speed_caps.empty());
  EXPECT_LE(held.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
  const auto constraint = overtake_planner::makeSafetyConstraint(
      held, mid_shift_ego, input, 8.0, 1.0);
  EXPECT_TRUE(constraint.valid);
  EXPECT_TRUE(constraint.stop_requested);

  // 実障害は一周期のrejectではなく、YIELD/RECOVERY/SAFE_STOPの所有権が
  // 周期ごとに振動したこと。fresh stampを更新しても同じtarget/sideと
  // FOLLOW_BLOCKED current-d STOPを維持し、旧PASS列をrate limitで混ぜない。
  overtake_planner::PlannerOutput repeated_hold = held;
  for (int cycle = 0; cycle < 4; ++cycle) {
    const double now_sec = 0.4 + 0.1 * static_cast<double>(cycle);
    stopped.stamp_sec = now_sec;
    repeated_hold = core.update(now_sec, mid_shift_ego, {stopped},
                                overtake_planner::MpcHealthStatus{}, input);
    SCOPED_TRACE(::testing::Message() << "hold_cycle=" << cycle);
    EXPECT_EQ(repeated_hold.mode,
              overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_EQ(repeated_hold.selected,
              overtake_planner::CandidateType::SAFE_STOP);
    EXPECT_NE(repeated_hold.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
    EXPECT_NE(repeated_hold.mode,
              overtake_planner::BehaviorMode::ABORT_RECOVERY);
    EXPECT_NE(repeated_hold.mode, overtake_planner::BehaviorMode::SAFE_STOP);
    EXPECT_TRUE(repeated_hold.blocked_info.maneuver_transaction_incomplete);
    EXPECT_TRUE(
        repeated_hold.blocked_info.maneuver_transaction_tracking_stop_active);
    EXPECT_TRUE(repeated_hold.blocked_info.attack_follow_candidate_generated);
    EXPECT_TRUE(!repeated_hold.blocked_info.attack_follow_candidate_feasible ||
                !repeated_hold.blocked_info
                     .attack_follow_candidate_tracking_profile_valid);
    EXPECT_FALSE(repeated_hold.blocked_info
                     .maneuver_transaction_tracking_release_pending);
    EXPECT_FALSE(repeated_hold.tracking_release_pass_warmup);
    EXPECT_EQ(repeated_hold.tracking_release_token, 0U);
    EXPECT_FALSE(repeated_hold.reentry_gate.requested);
    EXPECT_EQ(repeated_hold.maneuver_latch_target_id, stopped.id);
    EXPECT_EQ(repeated_hold.blocked_info.maneuver_transaction_pass_type,
              overtake_planner::CandidateType::PASS_LEFT);
    EXPECT_FALSE(repeated_hold.active_override);
    EXPECT_FALSE(repeated_hold.lateral_tracking_authorized_during_stop);
    EXPECT_TRUE(repeated_hold.lateral_offsets.empty());
    EXPECT_TRUE(repeated_hold.longitudinal_offsets_m.empty());
    EXPECT_TRUE(repeated_hold.longitudinal_speed_cap_active);
    ASSERT_FALSE(repeated_hold.speed_caps.empty());
    EXPECT_LE(repeated_hold.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
    const auto repeated_constraint = overtake_planner::makeSafetyConstraint(
        repeated_hold, mid_shift_ego, input, 8.0, 1.0);
    EXPECT_TRUE(repeated_constraint.valid);
    EXPECT_TRUE(repeated_constraint.stop_requested);
    EXPECT_FALSE(repeated_constraint.release_authorized);
  }
}

TEST(OvertakePlannerCore,
     IncompleteDynamicPassRewarmsControllerAfterInputCompletenessHold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.maneuver_latch_min_hold_sec = 0.0;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.min_pass_gap_m = 0.2;
  config.side_by_side_s_m = 0.01;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.25;
  config.min_ellipse_h = 0.1;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.attack_follow_min_spatial_horizon_m = 20.0;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  auto target = makeOpponent(frame, 13.0, 0.0);
  target.id = "dynamic_hold_d2";
  target.v = 0.0;
  target.vx = 0.0;
  target.stamp_sec = 0.1;
  const auto prepared =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(prepared.selected, overtake_planner::CandidateType::PASS_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_TRUE(passing.blocked_info.maneuver_transaction_incomplete);

  // 全観測相手の包含が一周期欠けたら、同じPASSが幾何的に生成できても横実行を
  // 続けず、target/sideを保持したspeed-only STOPへ閉じる。
  auto incomplete_input = readyReentryInput();
  incomplete_input.all_observed_opponents_included = false;
  target.stamp_sec = 0.3;
  const auto held =
      core.update(0.3, ego, {target}, overtake_planner::MpcHealthStatus{},
                  incomplete_input);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(held.maneuver_latch_target_id, target.id);
  EXPECT_EQ(held.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_FALSE(held.active_override);
  EXPECT_TRUE(held.lateral_offsets.empty());
  const auto held_constraint = overtake_planner::makeSafetyConstraint(
      held, ego, incomplete_input, 8.0, 1.0);
  EXPECT_TRUE(held_constraint.valid);
  EXPECT_TRUE(held_constraint.stop_requested);
  EXPECT_FALSE(held_constraint.release_authorized);

  // 入力復旧だけでは直接PASSを走らせない。current STOPのexact proofを受けた後、
  // 新しいPASS generation/tokenをSTOP下でpublishする。
  auto release_proof = readyReentryInput();
  release_proof.pure_pursuit_primary_and_fresh = false;
  release_proof.pure_pursuit_release_ready = true;
  target.stamp_sec = 0.4;
  const auto warmup = core.update(
      0.4, ego, {target}, overtake_planner::MpcHealthStatus{}, release_proof);
  ASSERT_EQ(warmup.selected, overtake_planner::CandidateType::PASS_LEFT)
      << warmup.blocked_info.pass_left_candidate_reject_reason;
  EXPECT_TRUE(
      warmup.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(
      warmup.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(warmup.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(warmup.tracking_release_pass_warmup) << warmup.reason;
  ASSERT_NE(warmup.tracking_release_token, 0U);

  // 同じtokenの異なるexact command sampleを2件受けても、その2件目の周期は
  // STOPを維持する。次のPlanner周期だけがmotion grant候補へ進める。
  release_proof.pass_probe_exact_current_usable = true;
  release_proof.pass_probe_lateral_stop_authority_token =
      warmup.tracking_release_token;
  release_proof.pass_probe_exact_plan_generation = 7U;
  release_proof.pass_probe_exact_sample_stamp_sec = 0.5;
  target.stamp_sec = 0.5;
  const auto exact_one = core.update(
      0.5, ego, {target}, overtake_planner::MpcHealthStatus{}, release_proof);
  EXPECT_FALSE(
      exact_one.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(exact_one.blocked_info.maneuver_transaction_tracking_stop_active);
  target.stamp_sec = 0.55;
  const auto duplicate_exact = core.update(
      0.55, ego, {target}, overtake_planner::MpcHealthStatus{}, release_proof);
  EXPECT_EQ(
      duplicate_exact.blocked_info.maneuver_transaction_tracking_probe_cycles,
      1);
  EXPECT_FALSE(duplicate_exact.blocked_info
                   .maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(
      duplicate_exact.blocked_info.maneuver_transaction_tracking_stop_active);

  release_proof.pass_probe_exact_sample_stamp_sec = 0.6;
  target.stamp_sec = 0.6;
  const auto exact_two = core.update(
      0.6, ego, {target}, overtake_planner::MpcHealthStatus{}, release_proof);
  EXPECT_FALSE(
      exact_two.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_TRUE(exact_two.blocked_info.maneuver_transaction_tracking_stop_active);

  target.stamp_sec = 0.7;
  const auto confirmed = core.update(
      0.7, ego, {target}, overtake_planner::MpcHealthStatus{}, release_proof);
  EXPECT_TRUE(
      confirmed.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(
      confirmed.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_FALSE(
      confirmed.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_TRUE(confirmed.tracking_release_pass_warmup);
  EXPECT_EQ(confirmed.tracking_release_token, warmup.tracking_release_token);

  target.stamp_sec = 0.8;
  const auto resumed =
      core.update(0.8, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(resumed.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(resumed.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(resumed.maneuver_latch_target_id, target.id);
  EXPECT_FALSE(
      resumed.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_FALSE(resumed.blocked_info.maneuver_transaction_tracking_stop_active);
}

TEST(OvertakePlannerCore, StartGridOvertakeOnlyKeepsEvaluatedPassAfterPrepare) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.pass_horizon_publish_mode = "overtake_only";
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.20;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto prepare =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      prepare.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
      prepare.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(prepare.selected, overtake_planner::CandidateType::FOLLOW);

  stopped.stamp_sec = 0.2;
  const auto overtake =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  const auto expected_pass =
      prepare.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT
          ? overtake_planner::CandidateType::PASS_LEFT
          : overtake_planner::CandidateType::PASS_RIGHT;
  const auto expected_mode =
      prepare.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT
          ? overtake_planner::BehaviorMode::OVERTAKE_LEFT
          : overtake_planner::BehaviorMode::OVERTAKE_RIGHT;
  EXPECT_EQ(overtake.mode, expected_mode);
  EXPECT_EQ(overtake.selected, expected_pass);

  stopped.stamp_sec = 0.3;
  const auto held =
      core.update(0.3, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(held.mode, expected_mode);
  EXPECT_EQ(held.selected, expected_pass);
  EXPECT_FALSE(held.published_lateral_safety_rejected);
}

TEST(OvertakePlannerCore,
     StartGridPassWaitsForFinalPurePursuitGenerationProof) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.20;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_wait_tracking";
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.stamp_sec = 0.1;
  auto tracking_not_ready = readyReentryInput();
  tracking_not_ready.pure_pursuit_primary_and_fresh = false;

  const auto waiting =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  tracking_not_ready);
  ASSERT_TRUE(waiting.blocked_info.start_grid_target_active);
  EXPECT_EQ(waiting.blocked_info.pass_left_candidate_reject_reason,
            "pass_target_unreachable");
  EXPECT_EQ(waiting.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_tracking_unhealthy");
  EXPECT_NE(waiting.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(waiting.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(waiting.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  ASSERT_FALSE(waiting.lateral_offsets.empty());
  for (const double d_m : waiting.lateral_offsets) {
    EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
  }

  // final muxのPP command・generation一致が確認できた周期は、同じ対象を
  // Gate 2へ戻し、待機中に別ID/sideへ張り替えない。
  stopped.stamp_sec = 0.2;
  const auto ready =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(ready.blocked_info.pass_left_candidate_feasible ||
              ready.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(ready.blocked_info.start_grid_target_id, stopped.id);
  EXPECT_NE(ready.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(OvertakePlannerCore,
     StartGridTargetFallsBackToCurrentLateralFollowWhenPassIsUnsafe) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -0.50;
  config.d_max_m = 0.50;
  config.min_wall_margin_m = 0.40;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  for (const double d_m : output.lateral_offsets) {
    EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_LE(output.speed_caps.back(),
            config.start_grid_attack_follow_v_max_mps + 1.0e-9);
}

TEST(
    OvertakePlannerCore,
    StartGridMovingAdjacentTargetUsesBoundedAttackFollowWithoutStationaryException) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -0.50;
  config.d_max_m = 0.50;
  config.min_wall_margin_m = 0.40;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto moving = makeOpponent(frame, 7.0, 1.2);
  moving.id = "moving_grid_d2";
  moving.v = 1.5;
  moving.vx = 1.5;
  moving.stamp_sec = 0.1;

  const auto output =
      core.update(0.1, ego, {moving}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_EQ(output.blocked_info.start_grid_target_id, moving.id);
  EXPECT_TRUE(output.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(output.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_FALSE(output.blocked_info.permission_start_exception_active);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), moving.v);
  EXPECT_LE(output.speed_caps.back(),
            config.start_grid_attack_follow_v_max_mps + 1.0e-9);
}

TEST(OvertakePlannerCore,
     StartGridMovingTargetRejectsLargeLateralPassAndKeepsAttackFollow) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.25;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.4;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto moving = makeOpponent(frame, 7.0, 1.2);
  moving.id = "moving_grid_large_shift";
  moving.v = 1.5;
  moving.vx = 1.5;
  moving.stamp_sec = 0.1;

  const auto output =
      core.update(0.1, ego, {moving}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(output.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.blocked_info.pass_left_candidate_reject_reason,
            "start_grid_moving_pass_lateral_displacement");
  EXPECT_EQ(output.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_moving_pass_lateral_displacement");
  EXPECT_FALSE(output.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     StartGridMovingTargetNeedsFreshContinuousStationaryConfirmation) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_stationary_confirmation_sec = 1.0;
  config.start_grid_stationary_confirmation_distance_m = 100.0;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.25;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.4;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 7.0, 1.2);
  target.id = "moving_then_transiently_slow_grid";
  target.v = 1.5;
  target.vx = 1.5;
  target.stamp_sec = 0.1;
  ASSERT_TRUE(core.update(0.1, ego, {target},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.start_grid_target_active);

  // 初回観測からは1秒以上経過していても、直前まで動いていた同じIDの
  // 一周期の低速観測を停止確定にしない。大横断PASSはまだ認可しない。
  target.v = 0.0;
  target.vx = 0.0;
  target.stamp_sec = 1.2;
  const auto transient_stop =
      core.update(1.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(transient_stop.blocked_info.start_grid_target_active);
  EXPECT_TRUE(
      transient_stop.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(
      transient_stop.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_FALSE(transient_stop.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(transient_stop.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(transient_stop.blocked_info.pass_left_candidate_reject_reason,
            "start_grid_unconfirmed_pass_lateral_displacement");
  EXPECT_EQ(transient_stop.blocked_info.pass_right_candidate_reject_reason,
            "start_grid_unconfirmed_pass_lateral_displacement");
  EXPECT_FALSE(transient_stop.blocked_info.maneuver_transaction_incomplete);

  // 同じfresh IDを停止状態で連続観測できた後だけ、左右候補を通常の
  // SafetyEvaluatorへ戻す。ここでも候補安全性そのものは別途必須。
  target.stamp_sec = 2.3;
  const auto confirmed =
      core.update(2.3, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(confirmed.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_FALSE(confirmed.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_TRUE(confirmed.blocked_info.pass_left_candidate_feasible ||
              confirmed.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(
      confirmed.selected == overtake_planner::CandidateType::PASS_LEFT ||
      confirmed.selected == overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(confirmed.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(confirmed.blocked_info.start_grid_target_id, target.id);
}

TEST(OvertakePlannerCore,
     StartGridMovingTargetKeepsInitialLateralAnchorAndTargetAfterWindow) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 2.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 9.0, 1.0);
  target.id = "latched_grid_target";
  target.v = 1.5;
  target.vx = 1.5;
  target.stamp_sec = 0.1;

  const auto armed =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(armed.blocked_info.start_grid_target_active);
  ASSERT_FALSE(armed.blocked_info.start_grid_lateral_release_pending);
  EXPECT_FALSE(armed.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(armed.blocked_info.start_grid_target_id, target.id);
  EXPECT_NEAR(armed.blocked_info.start_grid_hold_target_d_m, 2.0, 1.0e-9);
  EXPECT_EQ(armed.selected, overtake_planner::CandidateType::FOLLOW);

  // Gate 2未認可の2 cm誤差は、開始直後の曲率/PP generation遅れを新しい
  // 横操舵要求へ変換せず、freshな現在d保持として全相手へ再評価する。
  auto slight_drift_ego = makeEgo(frame, 5.1, 1.98);
  slight_drift_ego.v = 1.2;
  target.x = 9.1;
  target.frenet = frame.cartesianToFrenet(target.x, target.y, 0.0);
  target.stamp_sec = 0.2;
  const auto correcting =
      core.update(0.2, slight_drift_ego, {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(correcting.selected, overtake_planner::CandidateType::FOLLOW);
  ASSERT_FALSE(correcting.lateral_offsets.empty());
  EXPECT_TRUE(correcting.blocked_info.start_grid_uncommitted_hold_active);
  for (const double d_m : correcting.lateral_offsets) {
    EXPECT_NEAR(d_m, slight_drift_ego.frenet.d, 1.0e-9);
  }
  EXPECT_EQ(correcting.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_FALSE(correcting.reentry_gate.requested);
  ASSERT_FALSE(correcting.speed_caps.empty());
  EXPECT_GT(correcting.speed_caps.back(), 0.5);
  EXPECT_LE(correcting.speed_caps.back(),
            config.start_grid_attack_follow_v_max_mps);

  // 実追従が基準線側へ30 cm移っても、安全回廊内の同側driftならcurrent-d
  // holdを継続する。window満了や別IDの出現でもtarget/anchorは張り替えない。
  auto drifted_ego = makeEgo(frame, 5.5, 1.7);
  drifted_ego.v = 1.2;
  target.x = 9.8;
  target.frenet = frame.cartesianToFrenet(target.x, target.y, 0.0);
  target.stamp_sec = 5.2;
  auto nearer = makeOpponent(frame, 7.0, -2.0);
  nearer.id = "closer_but_not_latched";
  nearer.v = 1.0;
  nearer.vx = 1.0;
  nearer.stamp_sec = 5.2;

  const auto retained =
      core.update(5.2, drifted_ego, {nearer, target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_TRUE(retained.blocked_info.start_grid_target_active);
  EXPECT_FALSE(retained.blocked_info.start_grid_lateral_release_pending);
  EXPECT_FALSE(retained.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(retained.blocked_info.start_grid_target_id, target.id);
  EXPECT_NEAR(retained.blocked_info.start_grid_hold_target_d_m, 2.0, 1.0e-9);
  EXPECT_TRUE(retained.blocked_info.start_grid_uncommitted_hold_active);
  EXPECT_EQ(retained.selected, overtake_planner::CandidateType::FOLLOW);
  ASSERT_FALSE(retained.lateral_offsets.empty());
  for (const double d_m : retained.lateral_offsets) {
    EXPECT_NEAR(d_m, drifted_ego.frenet.d, 1.0e-9);
  }
  EXPECT_EQ(retained.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_FALSE(retained.reentry_gate.requested);
}

TEST(OvertakePlannerCore,
     StartGridUncommittedCurrentDHoldUsesDirectionAwareDriftEnvelope) {
  const auto evaluate_drift = [](double anchor_d_m, double current_d_m,
                                 bool reference_inside_safe_corridor) {
    auto frame = makeStraightFrame();
    if (!reference_inside_safe_corridor) {
      std::vector<overtake_planner::FrenetCorridorPoint> corridor;
      corridor.reserve(frame.reference().size());
      for (const auto &point : frame.reference()) {
        corridor.push_back({point.s, 1.0, 4.0});
      }
      frame.setCorridor(corridor);
    }

    auto config = makeConfig();
    config.start_grid_target_enabled = true;
    config.start_grid_target_window_sec = 5.0;
    config.start_grid_target_window_distance_m = 8.0;
    config.start_grid_target_max_ego_speed_mps = 3.0;
    // drift predicateだけを比較するため、大横断caseでも同じfresh targetが
    // retention範囲から外れない幅を使う。
    config.start_grid_target_lateral_width_m = 4.0;
    config.start_grid_attack_follow_v_max_mps = 3.0;
    config.start_grid_uncommitted_hold_max_lateral_drift_m = 0.15;
    config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
    config.dynamic_pass_candidate_enabled = true;
    config.d_min_m = -4.0;
    config.d_max_m = 4.0;
    config.min_wall_margin_m = 0.2;
    config.safety_ellipse_a_m = 1.0;
    config.safety_ellipse_b_m = 0.5;
    config.min_ellipse_h = 0.05;
    overtake_planner::OvertakePlannerCore core(frame, config);

    auto ego = makeEgo(frame, 5.0, anchor_d_m);
    ego.v = 1.0;
    auto target = makeOpponent(frame, 9.0, anchor_d_m + 0.5);
    target.id = "directional_drift_grid_target";
    target.v = 1.5;
    target.vx = 1.5;
    target.stamp_sec = 0.1;
    const auto armed =
        core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    EXPECT_TRUE(armed.blocked_info.start_grid_target_active);
    EXPECT_EQ(armed.blocked_info.start_grid_target_id, target.id);
    EXPECT_NEAR(armed.blocked_info.start_grid_hold_target_d_m, anchor_d_m,
                1.0e-9);

    auto drifted_ego = makeEgo(frame, 5.1, current_d_m);
    drifted_ego.v = 1.2;
    target.x = 9.1;
    target.frenet = frame.cartesianToFrenet(target.x, target.y, 0.0);
    target.stamp_sec = 0.2;
    return core.update(0.2, drifted_ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  const auto expect_current_d_hold = [](const auto &output,
                                        double expected_d_m) {
    ASSERT_TRUE(output.blocked_info.start_grid_target_active);
    EXPECT_TRUE(output.blocked_info.start_grid_uncommitted_hold_active);
    EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_FALSE(output.reentry_gate.requested);
    ASSERT_FALSE(output.lateral_offsets.empty());
    for (const double d_m : output.lateral_offsets) {
      EXPECT_NEAR(d_m, expected_d_m, 1.0e-9);
    }
  };

  expect_current_d_hold(evaluate_drift(2.0, 1.7, true), 1.7);
  expect_current_d_hold(evaluate_drift(-2.0, -1.7, true), -1.7);
  expect_current_d_hold(evaluate_drift(2.0, 2.14, true), 2.14);
  // 基準線付近の10 cm符号揺れは、従来の15 cm envelope内として維持する。
  expect_current_d_hold(evaluate_drift(0.05, -0.05, true), -0.05);

  const auto outward_over_limit = evaluate_drift(2.0, 2.16, true);
  ASSERT_TRUE(outward_over_limit.blocked_info.start_grid_target_active);
  EXPECT_FALSE(
      outward_over_limit.blocked_info.start_grid_uncommitted_hold_active);

  const auto crossed_reference = evaluate_drift(2.0, -0.10, true);
  ASSERT_TRUE(crossed_reference.blocked_info.start_grid_target_active);
  EXPECT_FALSE(
      crossed_reference.blocked_info.start_grid_uncommitted_hold_active);

  // 基準線d=0が安全回廊外なら、大きな「内向き」driftを特例にしない。
  const auto reference_outside_corridor = evaluate_drift(2.0, 1.7, false);
  ASSERT_TRUE(reference_outside_corridor.blocked_info.start_grid_target_active);
  EXPECT_FALSE(reference_outside_corridor.blocked_info
                   .start_grid_uncommitted_hold_active);
}

TEST(OvertakePlannerCore,
     StartGridUncommittedCurrentDHoldRequiresFreshV2xSnapshot) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 2.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 9.0, 1.0);
  target.id = "stale_v2x_grid_target";
  target.v = 1.5;
  target.vx = 1.5;
  target.stamp_sec = 0.1;
  ASSERT_TRUE(core.update(0.1, ego, {target},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.start_grid_target_active);

  auto drifted_ego = makeEgo(frame, 5.1, 1.98);
  drifted_ego.v = 1.2;
  target.x = 9.1;
  target.frenet = frame.cartesianToFrenet(target.x, target.y, 0.0);
  target.stamp_sec = 0.2;
  auto stale_input = readyReentryInput();
  stale_input.v2x_snapshot_fresh = false;
  const auto output =
      core.update(0.2, drifted_ego, {target},
                  overtake_planner::MpcHealthStatus{}, stale_input);

  EXPECT_FALSE(output.blocked_info.start_grid_uncommitted_hold_active);
}

TEST(OvertakePlannerCore,
     StartGridUncommittedHoldRejectsFutureCorridorClampWithoutReentry) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -3.0, point.s >= 7.0 ? 1.1 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.same_corridor_width_m = 0.90;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.4);
  ego.v = 2.5;
  auto target = makeOpponent(frame, 9.0, 0.0);
  target.id = "future_narrow_grid_target";
  target.v = 2.5;
  target.vx = 2.5;
  target.stamp_sec = 0.1;

  auto no_pass_tracking = readyReentryInput();
  no_pass_tracking.pure_pursuit_primary_and_fresh = false;
  const auto output =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  no_pass_tracking);

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.blocked_info.start_grid_uncommitted_hold_active);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_FALSE(output.reentry_gate.requested);
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.lateral_offsets.empty());
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_LE(output.applied_speed_cap_mps,
            config.speed_only_fallback_v_max_mps + 1.0e-9);
}

TEST(OvertakePlannerCore,
     StartGridUnapprovedTargetLossDoesNotStartAbortOrReentry) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 0.2;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.10;
  config.dynamic_pass_candidate_enabled = true;
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 2.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 9.0, 1.0);
  target.id = "grid_release_target";
  target.v = 1.5;
  target.vx = 1.5;
  target.stamp_sec = 0.1;
  const auto prepared =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(prepared.blocked_info.start_grid_target_active);
  ASSERT_FALSE(prepared.blocked_info.start_grid_lateral_release_pending);
  ASSERT_FALSE(prepared.blocked_info.maneuver_transaction_incomplete);

  auto drifted_ego = makeEgo(frame, 5.5, 1.7);
  drifted_ego.v = 1.2;
  // 大横断PASSをGate 2が一度も認可していない状態で元targetが観測範囲を
  // 外れても、初期配置のdを「追越後」と誤認してABORT/reentryを開始しない。
  const auto released =
      core.update(0.5, drifted_ego, {}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(released.blocked_info.start_grid_target_active);
  EXPECT_FALSE(released.blocked_info.start_grid_lateral_release_pending);
  EXPECT_FALSE(released.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(released.reentry_gate.requested);
  EXPECT_NE(released.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(released.selected, overtake_planner::CandidateType::FASTEST);
}

TEST(OvertakePlannerCore,
     UnsafeUncommittedStartGridAttackFollowUsesEvaluatedHoldWithoutAbort) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.same_corridor_width_m = 0.90;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.min_wall_margin_m = 0.20;
  config.safety_ellipse_a_m = 10.0;
  config.safety_ellipse_b_m = 1.2;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.6;
  auto target = makeOpponent(frame, 9.5, 1.0);
  target.id = "unsafe_current_d_grid_target";
  target.v = 0.0;
  target.vx = 0.0;
  target.stamp_sec = 0.1;

  const auto output =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.blocked_info.start_grid_uncommitted_hold_active);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_FALSE(output.reentry_gate.requested);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_LE(output.applied_speed_cap_mps,
            config.reentry_hold_v_max_mps + 1.0e-9);
  if (!output.lateral_offsets.empty()) {
    for (const double d_m : output.lateral_offsets) {
      EXPECT_NEAR(d_m, ego.frenet.d, 1.0e-9);
    }
  }
}

TEST(OvertakePlannerCore,
     StartGridUnapprovedTargetHandsOffToAuthoritativeBlockedFront) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 1.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 2.0);
  ego.v = 1.0;
  auto grid_target = makeOpponent(frame, 9.0, 1.0);
  grid_target.id = "unapproved_grid_d3";
  grid_target.v = 1.5;
  grid_target.vx = 1.5;
  grid_target.stamp_sec = 0.1;
  const auto prepared =
      core.update(0.1, ego, {grid_target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(prepared.blocked_info.start_grid_target_active);
  ASSERT_EQ(prepared.blocked_info.start_grid_target_id, grid_target.id);
  ASSERT_FALSE(prepared.blocked_info.maneuver_transaction_incomplete);

  // start window内でも、未認可のgrid対象より近い同方向低速車を
  // BlockedRiskAnalyzerが前方閉塞車として確定する。ここで古いIDを保持すると、
  // 実車では新しい車の安全楕に入るまで速度上限が更新されない。
  auto drifted_ego = makeEgo(frame, 5.5, 1.7);
  drifted_ego.v = 2.0;
  grid_target.x = 10.0;
  grid_target.frenet =
      frame.cartesianToFrenet(grid_target.x, grid_target.y, 0.0);
  grid_target.stamp_sec = 0.5;
  // side-by-sideは別契約なので、ここではBlockedRiskAnalyzerが通常frontとして
  // 明示的に選ぶ4 m超かつ旧targetより近い位置に置く。
  auto blocked_front = makeOpponent(frame, 9.6, 1.6);
  blocked_front.id = "authoritative_front_d2";
  blocked_front.v = 0.2;
  blocked_front.vx = 0.2;
  blocked_front.stamp_sec = 0.5;

  const auto handed_off =
      core.update(0.5, drifted_ego, {grid_target, blocked_front},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_TRUE(
      handed_off.blocked_info.start_grid_target_superseded_by_blocked_front);
  EXPECT_TRUE(handed_off.blocked_info.start_grid_target_reselection_suppressed);
  EXPECT_EQ(handed_off.blocked_info.start_grid_superseded_target_id,
            grid_target.id);
  EXPECT_EQ(handed_off.blocked_info.start_grid_replacement_target_id,
            blocked_front.id);
  EXPECT_FALSE(handed_off.blocked_info.start_grid_target_active);
  EXPECT_FALSE(handed_off.blocked_info.start_grid_uncommitted_hold_active);
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_id, blocked_front.id);
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_previous_id,
            grid_target.id);
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_change_reason,
            "unapproved_start_grid_target_superseded_by_blocked_front");
  EXPECT_FALSE(handed_off.blocked_info.start_grid_lateral_release_pending);
  EXPECT_NE(handed_off.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);

  // start windowがまだ有効でも、次周期に旧grid候補を再ラッチしてはならない。
  grid_target.stamp_sec = 0.6;
  blocked_front.stamp_sec = 0.6;
  const auto retained_handoff =
      core.update(0.6, drifted_ego, {grid_target, blocked_front},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(
      retained_handoff.blocked_info.start_grid_target_reselection_suppressed);
  EXPECT_FALSE(retained_handoff.blocked_info.start_grid_target_active);
  EXPECT_EQ(retained_handoff.blocked_info.maneuver_target_id, blocked_front.id);
}

TEST(OvertakePlannerCore,
     StartGridD3ToD2HandoffKeepsSafeCurrentDFollowInsteadOfAbort) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  config.start_grid_stationary_confirmation_sec = 100.0;
  config.start_grid_stationary_confirmation_distance_m = 100.0;
  config.start_grid_attack_follow_v_max_mps = 3.0;
  config.start_grid_uncommitted_hold_max_lateral_drift_m = 0.15;
  config.start_grid_moving_pass_max_lateral_displacement_m = 0.01;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.same_corridor_width_m = 0.90;
  config.side_by_side_s_m = 3.0;
  config.parallel_side_detection_enabled = true;
  config.parallel_side_s_m = 12.0;
  config.parallel_side_margin_m = 4.0;
  config.future_side_prediction_enabled = false;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_max_target_speed_mps = 1.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.20;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.reentry_gate_enabled = true;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_safe_cycles = 2;
  config.safe_stop_trigger_cycles = 1;
  config.large_lateral_error_threshold_m = 0.60;
  config.d_min_m = -3.5;
  config.d_max_m = 3.5;
  config.min_wall_margin_m = 0.50;
  config.lateral_target_max_step_m = 100.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto expect_current_d_follow = [](const auto &output,
                                          double expected_d_m) {
    EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED)
        << "raw_reject=" << output.raw_selected_reject_reason
        << " side=" << output.blocked_info.side_by_side
        << " parallel=" << output.blocked_info.parallel_side_candidate
        << " future_yield=" << output.blocked_info.future_yield_required
        << " braking=" << output.blocked_info.braking_follow_active
        << " braking_id=" << output.blocked_info.braking_follow_id
        << " braking_reject="
        << output.blocked_info.braking_follow_candidate_reject_reason;
    EXPECT_EQ(output.raw_selected, overtake_planner::CandidateType::FOLLOW);
    EXPECT_TRUE(output.raw_selected_feasible)
        << output.raw_selected_reject_reason;
    EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
    EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
    EXPECT_TRUE(output.active_override);
    EXPECT_TRUE(output.longitudinal_speed_cap_active);
    EXPECT_FALSE(output.reentry_gate.requested) << output.reentry_gate.reason;
    EXPECT_FALSE(output.safe_stop_triggered);
    ASSERT_FALSE(output.lateral_offsets.empty());
    for (const double d_m : output.lateral_offsets) {
      EXPECT_NEAR(d_m, expected_d_m, 1.0e-9);
    }
  };

  auto ego = makeEgo(frame, 28.30, 2.60);
  ego.v = 1.00;
  auto d3 = makeOpponent(frame, 32.60, 1.77);
  d3.id = "d3";
  d3.v = 1.00;
  d3.vx = 1.00;
  d3.stamp_sec = 0.10;
  auto d2 = makeOpponent(frame, 29.41, -0.99);
  d2.id = "d2";
  d2.v = 0.80;
  d2.vx = 0.80;
  d2.stamp_sec = 0.10;

  const auto armed =
      core.update(0.10, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(armed.blocked_info.start_grid_target_active);
  ASSERT_EQ(armed.blocked_info.start_grid_target_id, d3.id);
  EXPECT_FALSE(armed.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(armed.blocked_info.maneuver_transaction_incomplete);
  expect_current_d_follow(armed, ego.frenet.d);

  ego = makeEgo(frame, 30.53, 2.15);
  ego.v = 1.19;
  // 実bagと同じく、旧targetのD3は通常same-corridor判定のすぐ外に残し、
  // より近い低速chain targetのD2をparallel昇格で権威的frontへ引き継ぐ。
  // D2をhandoff周期だけ同一レーンへ瞬間移動させるテストにはしない。
  d2 = makeOpponent(frame, 33.86, -0.81);
  d2.id = "d2";
  d2.v = 0.80;
  d2.vx = 0.80;
  d2.stamp_sec = 4.85;
  d3 = makeOpponent(frame, 34.03, 1.19);
  d3.id = "d3";
  d3.v = 0.80;
  d3.vx = 0.80;
  d3.stamp_sec = 4.85;

  const auto handed_off =
      core.update(4.85, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(
      handed_off.blocked_info.start_grid_target_superseded_by_blocked_front);
  EXPECT_EQ(handed_off.blocked_info.start_grid_superseded_target_id, d3.id);
  EXPECT_EQ(handed_off.blocked_info.start_grid_replacement_target_id, d2.id);
  EXPECT_EQ(handed_off.blocked_info.maneuver_target_id, d2.id);
  EXPECT_FALSE(handed_off.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(handed_off.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(handed_off.blocked_info.slow_obstacle_chain_active);
  EXPECT_EQ(handed_off.blocked_info.slow_obstacle_chain_id, d2.id);
  ASSERT_TRUE(handed_off.blocked_info.braking_follow_active);
  EXPECT_EQ(handed_off.blocked_info.braking_follow_id, d2.id);
  EXPECT_GT(handed_off.blocked_info.braking_follow_required_distance_m,
            handed_off.blocked_info.braking_follow_available_distance_m);
  EXPECT_TRUE(handed_off.blocked_info.braking_follow_feasible)
      << handed_off.blocked_info.braking_follow_candidate_reject_reason;
  EXPECT_FALSE(handed_off.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(handed_off.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(handed_off.blocked_info.pass_left_candidate_reject_reason,
            "pass_transition_deadline_unreachable");
  EXPECT_EQ(handed_off.blocked_info.pass_right_candidate_reject_reason,
            "pass_transition_deadline_unreachable");
  expect_current_d_follow(handed_off, ego.frenet.d);
  ASSERT_FALSE(handed_off.speed_caps.empty());
  EXPECT_LE(handed_off.speed_caps.back(),
            config.start_grid_attack_follow_v_max_mps + 1.0e-9);

  ego = makeEgo(frame, 30.62, 2.13);
  ego.v = 1.19;
  d2 = makeOpponent(frame, 33.85, -0.81);
  d2.id = "d2";
  d2.v = 0.80;
  d2.vx = 0.80;
  d2.stamp_sec = 4.90;
  d3 = makeOpponent(frame, 34.10, 1.20);
  d3.id = "d3";
  d3.v = 0.80;
  d3.vx = 0.80;
  d3.stamp_sec = 4.90;
  const auto retained =
      core.update(4.90, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(retained.blocked_info.start_grid_target_reselection_suppressed);
  EXPECT_FALSE(
      retained.blocked_info.start_grid_target_superseded_by_blocked_front);
  EXPECT_EQ(retained.blocked_info.maneuver_target_id, d2.id);
  EXPECT_EQ(retained.blocked_info.maneuver_chain_tail_id, d2.id);
  EXPECT_EQ(retained.transition_previous_mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_FALSE(retained.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(retained.blocked_info.maneuver_transaction_incomplete);
  ASSERT_TRUE(retained.blocked_info.braking_follow_active);
  EXPECT_GT(retained.blocked_info.braking_follow_required_distance_m,
            retained.blocked_info.braking_follow_available_distance_m);
  EXPECT_TRUE(retained.blocked_info.braking_follow_feasible)
      << retained.blocked_info.braking_follow_candidate_reject_reason;
  EXPECT_GE(retained.blocked_info.braking_follow_candidate_min_safety_margin,
            config.min_ellipse_h + 0.10);
  EXPECT_FALSE(retained.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(retained.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(retained.blocked_info.pass_left_candidate_reject_reason,
            "pass_transition_deadline_unreachable");
  EXPECT_EQ(retained.blocked_info.pass_right_candidate_reject_reason,
            "pass_transition_deadline_unreachable");
  expect_current_d_follow(retained, ego.frenet.d);

  // 同じhandoff世代でもD2が同一レーンへ入った場合、短い評価horizonの
  // SafetyEvaluator marginだけで制動不足を免除してはいけない。
  ego = makeEgo(frame, 30.70, 2.13);
  ego.v = 1.19;
  d2 = makeOpponent(frame, 34.20, 2.13);
  d2.id = "d2";
  d2.v = 0.80;
  d2.vx = 0.80;
  d2.stamp_sec = 4.95;
  d3 = makeOpponent(frame, 34.18, 1.20);
  d3.id = "d3";
  d3.v = 0.80;
  d3.vx = 0.80;
  d3.stamp_sec = 4.95;
  const auto same_lane_shortfall =
      core.update(4.95, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(same_lane_shortfall.blocked_info.braking_follow_active);
  EXPECT_EQ(same_lane_shortfall.blocked_info.braking_follow_id, d2.id);
  EXPECT_GT(
      same_lane_shortfall.blocked_info.braking_follow_required_distance_m,
      same_lane_shortfall.blocked_info.braking_follow_available_distance_m);
  EXPECT_FALSE(same_lane_shortfall.blocked_info.braking_follow_feasible);
  EXPECT_EQ(
      same_lane_shortfall.blocked_info.braking_follow_candidate_reject_reason,
      "insufficient_braking_distance");

  // handoff対象が欠測した時点で世代IDを破棄する。その後に別の前方車が
  // 現れても、古いD3→D2例外をD4へ引き継いではならない。
  ego = makeEgo(frame, 30.78, 2.13);
  ego.v = 1.19;
  const auto target_lost = core.update(
      5.60, ego, {}, overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(
      target_lost.blocked_info.start_grid_target_reselection_suppressed);
  EXPECT_TRUE(
      target_lost.blocked_info.start_grid_replacement_target_id.empty());

  auto d4 = makeOpponent(frame, 34.28, 2.13);
  d4.id = "d4";
  d4.v = 0.80;
  d4.vx = 0.80;
  d4.stamp_sec = 5.65;
  const auto unrelated_target =
      core.update(5.65, ego, {d4}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(
      unrelated_target.blocked_info.start_grid_target_reselection_suppressed);
  EXPECT_TRUE(
      unrelated_target.blocked_info.start_grid_replacement_target_id.empty());
  ASSERT_TRUE(unrelated_target.blocked_info.braking_follow_active);
  EXPECT_EQ(unrelated_target.blocked_info.braking_follow_id, d4.id);
  EXPECT_FALSE(unrelated_target.blocked_info.braking_follow_feasible);
  EXPECT_EQ(
      unrelated_target.blocked_info.braking_follow_candidate_reject_reason,
      "insufficient_braking_distance");
}

TEST(OvertakePlannerCore,
     StartGridTargetPromotesToConfirmedStationaryAfterConfiguredTime) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_stationary_confirmation_sec = 1.0;
  config.start_grid_stationary_confirmation_distance_m = 100.0;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -0.50;
  config.d_max_m = 0.50;
  config.min_wall_margin_m = 0.40;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_d2";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto pending =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(pending.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(pending.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_FALSE(pending.blocked_info.stationary_front_obstacle);

  stopped.stamp_sec = 1.2;
  const auto confirmed =
      core.update(1.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_FALSE(confirmed.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_TRUE(confirmed.blocked_info.start_grid_target_confirmed_stationary);
}

TEST(OvertakePlannerCore, StartGridTargetExcludesClearlyTrailingVehicle) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_min_delta_s_m = -2.0;
  config.side_by_side_leader_priority_enter_s_m = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto trailing = makeOpponent(frame, 3.8, 1.0);
  trailing.v = 0.0;
  trailing.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {trailing}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_FALSE(output.blocked_info.start_grid_target_active);
}

TEST(OvertakePlannerCore,
     StartGridTargetAcceptsSlightlyTrailingAdjacentVehicleAndPrefersSafePass) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_target_min_delta_s_m = -1.0;
  config.side_by_side_leader_priority_enter_s_m = 1.0;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.20;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto adjacent = makeOpponent(frame, 4.5, 1.2);
  adjacent.id = "adjacent_grid";
  adjacent.v = 0.0;
  adjacent.vx = 0.0;

  const auto output =
      core.update(0.1, ego, {adjacent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_NEAR(output.blocked_info.start_grid_target_delta_s, -0.5, 1.0e-6);
  EXPECT_TRUE(output.selected == overtake_planner::CandidateType::PASS_LEFT ||
              output.selected == overtake_planner::CandidateType::PASS_RIGHT);
}

TEST(OvertakePlannerCore,
     StartGridSmallBackwardMotionDoesNotCountAsOneLapOfProgress) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_stationary_confirmation_sec = 100.0;
  config.start_grid_stationary_confirmation_distance_m = 0.5;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -0.50;
  config.d_max_m = 0.50;
  config.min_wall_margin_m = 0.40;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 6.0, 1.2);
  stopped.v = 0.0;
  stopped.vx = 0.0;
  ASSERT_TRUE(core.update(0.1, ego, {stopped},
                          overtake_planner::MpcHealthStatus{},
                          readyReentryInput())
                  .blocked_info.start_grid_target_confirmation_pending);

  ego = makeEgo(frame, 4.95, 0.0);
  ego.v = 1.0;
  stopped.stamp_sec = 0.2;
  const auto retreated =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(retreated.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(retreated.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_NEAR(retreated.blocked_info.start_grid_ego_progress_m, 0.0, 1.0e-9);
}

TEST(OvertakePlannerCore,
     StartGridNoPassExceptionWaitsForStationaryConfirmation) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.start_grid_target_enabled = true;
  config.start_grid_stationary_confirmation_sec = 1.0;
  config.start_grid_stationary_confirmation_distance_m = 100.0;
  config.dynamic_pass_candidate_enabled = true;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.slow_front_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"grid_no_pass", 0.0, 40.0,
                                               false});
  config.safety_ellipse_a_m = 0.8;
  config.safety_ellipse_b_m = 0.20;
  config.min_ellipse_h = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto stopped = makeOpponent(frame, 9.5, 1.2);
  stopped.id = "grid_no_pass_target";
  stopped.v = 0.0;
  stopped.vx = 0.0;

  const auto pending =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(pending.blocked_info.start_grid_target_confirmation_pending);
  EXPECT_FALSE(pending.blocked_info.permission_start_exception_active);
  EXPECT_EQ(pending.selected, overtake_planner::CandidateType::FOLLOW);

  stopped.stamp_sec = 1.2;
  const auto confirmed =
      core.update(1.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(confirmed.blocked_info.start_grid_target_confirmed_stationary);
  EXPECT_TRUE(confirmed.blocked_info.permission_start_exception_active);
  EXPECT_TRUE(
      confirmed.selected == overtake_planner::CandidateType::PASS_LEFT ||
      confirmed.selected == overtake_planner::CandidateType::PASS_RIGHT);
}

TEST(CandidateBuilder, PassAccelerationPredictionMatchesPublishedSpeedCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.pass_speed_cap_mps = 5.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.left_offset_m = 0.70;
  config.safety_ellipse_b_m = 0.25;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 3.0;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;
  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {});

  ASSERT_FALSE(candidate.v_ref.empty());
  ASSERT_GT(candidate.predicted_speed_mps.size(), 1U);
  EXPECT_NEAR(candidate.v_ref.front(), config.pass_speed_cap_mps, 1.0e-9);
  EXPECT_GT(candidate.predicted_speed_mps[1], ego.v);
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
  EXPECT_EQ(output.raw_selected, overtake_planner::CandidateType::YIELD_BEHIND);
  EXPECT_TRUE(output.raw_selected_feasible);
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
  EXPECT_GE(output.blocked_info.ego_wall_clearance_m, 0.0);
  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.corner_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_yield_required);
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

TEST(OvertakePlannerCore,
     BrakingDistanceFollowStartsBeforeFixedFrontLookaheadAndHoldsCurrentD) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.max_brake_decel_mps2 = 1.5;
  config.longitudinal_response_delay_sec = 0.25;
  // PASS側へ出た後はd3が通常front幅から外れ、近いparallel代表は後方d2に
  // なる状況を再現する。明示braking targetを旧分類で見失わないことを検証する。
  config.same_corridor_width_m = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.8);
  ego.v = 6.37;
  auto stopped = makeOpponent(frame, 22.0, 0.8);
  stopped.id = "d2";
  stopped.vx = 0.42;
  stopped.v = 0.42;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "d2");
  EXPECT_GT(output.blocked_info.braking_follow_required_distance_m,
            output.blocked_info.braking_follow_delta_s);
  EXPECT_GT(output.blocked_info.braking_follow_trigger_distance_m,
            output.blocked_info.braking_follow_required_distance_m);
  EXPECT_NEAR(output.blocked_info.braking_follow_target_speed_mps, 0.42,
              1.0e-9);
  EXPECT_GT(output.blocked_info.braking_follow_relative_speed_mps, 0.0);
  EXPECT_LT(output.blocked_info.braking_follow_ttc_sec,
            config.braking_follow_ttc_threshold_sec);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(output.active_override);
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-9);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_LE(output.speed_caps.front(),
            output.blocked_info.braking_follow_speed_cap_mps + 1.0e-9);
}

TEST(OvertakePlannerCore,
     MovingBrakingTargetCanEnterGateTwoPassBeforeFixedLookahead) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.min_wall_margin_m = 0.4;
  config.safety_ellipse_b_m = 0.25;
  config.min_pass_gap_m = 0.2;
  config.prepare_distance_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 6.37;
  auto moving_slow = makeOpponent(frame, 22.0, 0.0);
  moving_slow.id = "d2";
  moving_slow.vx = 0.42;
  moving_slow.v = 0.42;

  const auto output =
      core.update(0.1, ego, {moving_slow}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(output.selected == overtake_planner::CandidateType::PASS_LEFT ||
              output.selected == overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     FreshLatchedRightPassSurvivesFrontClassifierDropoutUntilFiveSafeCycles) {
  const auto frame = makeStraightFrame();
  const auto config = makePassStartContinuityConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto sample = [&](double now_sec, double relative_s_m,
                          double target_speed_mps) {
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 5.0 + relative_s_m, 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = target_speed_mps;
    target.vx = target_speed_mps;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const auto output = sample(0.1 * static_cast<double>(cycle), 14.901, 3.70);
    ASSERT_TRUE(output.blocked_info.blocked) << "cycle=" << cycle;
    EXPECT_EQ(output.blocked_info.maneuver_target_id, "d2");
    EXPECT_TRUE(output.blocked_info.maneuver_target_observed);
    EXPECT_TRUE(output.blocked_info.maneuver_target_fresh);
    EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
    EXPECT_TRUE(output.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << output.blocked_info.pass_right_candidate_reject_reason;
    EXPECT_EQ(output.raw_selected, overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, cycle);
  }

  // 実bagの14.901 -> 15.508 mと同じく、同一fresh IDは残るが固定15 m
  // front分類だけが落ちる。5周期目も候補を新規SafetyEvaluatorへ通せた時だけ
  // PREPAREへ進み、過去周期の安全結果は流用しない。
  const auto dropout = sample(0.5, 15.508, 3.85);
  EXPECT_FALSE(dropout.blocked_info.blocked);
  EXPECT_EQ(dropout.blocked_info.nearest_index, -1);
  EXPECT_FALSE(dropout.blocked_info.braking_follow_active);
  EXPECT_TRUE(dropout.blocked_info.maneuver_target_latched);
  EXPECT_EQ(dropout.blocked_info.maneuver_target_id, "d2");
  EXPECT_TRUE(dropout.blocked_info.maneuver_target_observed);
  EXPECT_TRUE(dropout.blocked_info.maneuver_target_fresh);
  EXPECT_TRUE(dropout.blocked_info.pass_start_target_continuity_active)
      << dropout.blocked_info.pass_start_target_continuity_reason;
  EXPECT_EQ(dropout.blocked_info.pass_start_target_continuity_cycles, 1);
  EXPECT_TRUE(dropout.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(dropout.blocked_info.pass_right_candidate_feasible)
      << dropout.blocked_info.pass_right_candidate_reject_reason;
  EXPECT_EQ(dropout.raw_selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(dropout.blocked_info.pass_right_safe_cycles, 5);
  EXPECT_TRUE(dropout.supervisor_v2.pass_right_candidate.generated);
  EXPECT_TRUE(dropout.supervisor_v2.pass_right_candidate.safety_evaluated);
  ASSERT_EQ(dropout.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto &committed_candidate =
      dropout.supervisor_v2.selected ==
              overtake_planner::CandidateType::PASS_LEFT
          ? dropout.supervisor_v2.pass_left_candidate
          : dropout.supervisor_v2.pass_right_candidate;
  EXPECT_TRUE(committed_candidate.feasible)
      << committed_candidate.reject_reason
      << " desired=" << committed_candidate.desired_path_trackable
      << " pp=" << committed_candidate.pure_pursuit_command_trackable
      << " phase=" << static_cast<int>(dropout.supervisor_v2.phase)
      << " selected=" << static_cast<int>(dropout.supervisor_v2.selected)
      << " target=" << dropout.supervisor_v2.target_vehicle_id
      << " auth=" << dropout.supervisor_v2.authorization_failure_mask
      << " reason=" << dropout.supervisor_v2.reason;
  EXPECT_EQ(dropout.supervisor_v2.target_vehicle_id, "d2");
  EXPECT_EQ(dropout.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  const auto resumed = sample(0.6, 15.508, 3.85);
  EXPECT_TRUE(resumed.blocked_info.pass_start_target_continuity_active)
      << resumed.blocked_info.pass_start_target_continuity_reason;
  EXPECT_EQ(resumed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(resumed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_EQ(resumed.blocked_info.maneuver_target_id, "d2");
  EXPECT_TRUE(resumed.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     FreshLatchedLeftPassSurvivesFrontClassifierDropoutUntilFiveSafeCycles) {
  const auto frame = makeStraightFrame();
  const auto config = makePassStartContinuityConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto sample = [&](double now_sec, double relative_s_m,
                          double target_speed_mps) {
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 5.0 + relative_s_m, -0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = target_speed_mps;
    target.vx = target_speed_mps;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const auto output = sample(0.1 * static_cast<double>(cycle), 14.901, 3.70);
    ASSERT_TRUE(output.blocked_info.blocked) << "cycle=" << cycle;
    EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
    EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << output.blocked_info.pass_left_candidate_reject_reason;
    EXPECT_EQ(output.raw_selected, overtake_planner::CandidateType::PASS_LEFT);
    EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, cycle);
  }

  const auto dropout = sample(0.5, 15.508, 3.85);
  EXPECT_FALSE(dropout.blocked_info.blocked);
  EXPECT_EQ(dropout.blocked_info.nearest_index, -1);
  EXPECT_TRUE(dropout.blocked_info.pass_start_target_continuity_active)
      << dropout.blocked_info.pass_start_target_continuity_reason;
  EXPECT_TRUE(dropout.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(dropout.blocked_info.pass_left_candidate_feasible)
      << dropout.blocked_info.pass_left_candidate_reject_reason;
  EXPECT_EQ(dropout.raw_selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(dropout.blocked_info.pass_left_safe_cycles, 5);
  EXPECT_TRUE(dropout.supervisor_v2.pass_left_candidate.generated);
  EXPECT_TRUE(dropout.supervisor_v2.pass_left_candidate.safety_evaluated);
  EXPECT_TRUE(dropout.supervisor_v2.pass_left_candidate.feasible);
  EXPECT_EQ(dropout.supervisor_v2.target_vehicle_id, "d2");
  EXPECT_EQ(dropout.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto resumed = sample(0.6, 15.508, 3.85);
  EXPECT_TRUE(resumed.blocked_info.pass_start_target_continuity_active)
      << resumed.blocked_info.pass_start_target_continuity_reason;
  EXPECT_EQ(resumed.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(resumed.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(resumed.blocked_info.maneuver_target_id, "d2");
  EXPECT_TRUE(resumed.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     ProductionModeHoldAllowsPersistentBagClassifierDropoutToCommit) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.control_rate_hz = 20.0;
  config.horizon_dt_sec = 0.025;
  config.min_mode_hold_time_sec = 0.60;
  config.parallel_side_detection_enabled = true;
  config.min_wall_margin_m = 0.50;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto sample = [&](double now_sec, double relative_s_m) {
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 5.0 + relative_s_m, 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = relative_s_m < config.lookahead_s_m ? 3.70 : 3.85;
    target.vx = target.v;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (int cycle = 1; cycle <= 5; ++cycle) {
    const auto direct = sample(0.05 * static_cast<double>(cycle), 14.901);
    ASSERT_TRUE(direct.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << direct.blocked_info.pass_right_candidate_reject_reason;
    EXPECT_EQ(direct.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_EQ(direct.blocked_info.maneuver_target_id, "d2");
    EXPECT_EQ(direct.raw_selected, overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_EQ(direct.blocked_info.pass_right_safe_cycles, cycle);
  }

  int prepare_cycle = -1;
  for (int cycle = 6; cycle <= 16; ++cycle) {
    const auto dropout = sample(0.05 * static_cast<double>(cycle), 15.508);
    EXPECT_TRUE(dropout.blocked_info.pass_start_target_continuity_active)
        << "cycle=" << cycle << " reason="
        << dropout.blocked_info.pass_start_target_continuity_reason;
    EXPECT_FALSE(dropout.blocked_info.pass_start_target_continuity_expired);
    EXPECT_EQ(dropout.blocked_info.pass_start_target_continuity_budget_cycles,
              17);
    EXPECT_EQ(dropout.blocked_info.maneuver_target_id, "d2");
    EXPECT_EQ(dropout.raw_selected,
              overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_TRUE(dropout.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle << " reject="
        << dropout.blocked_info.pass_right_candidate_reject_reason;
    EXPECT_GE(dropout.blocked_info.pass_right_safe_cycles, 5);
    if (dropout.mode ==
        overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT) {
      prepare_cycle = cycle;
      break;
    }
    EXPECT_EQ(dropout.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  }
  ASSERT_GT(prepare_cycle, 0);

  const auto committed =
      sample(0.05 * static_cast<double>(prepare_cycle + 1), 15.508);
  EXPECT_TRUE(committed.blocked_info.pass_start_target_continuity_active)
      << committed.blocked_info.pass_start_target_continuity_reason;
  EXPECT_EQ(committed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(committed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(committed.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     DirectDynamicPassCannotCommitWithIncompleteOpponentInputs) {
  enum class IncompleteInput {
    OPPONENT_STALE,
    OPPONENT_EXCLUDED,
    V2X_STALE,
    REFERENCE_INVALID
  };
  const std::vector<IncompleteInput> cases = {
      IncompleteInput::OPPONENT_STALE, IncompleteInput::OPPONENT_EXCLUDED,
      IncompleteInput::V2X_STALE, IncompleteInput::REFERENCE_INVALID};

  for (const auto incomplete_input : cases) {
    const auto frame = makeStraightFrame();
    auto config = makePassStartContinuityConfig();
    config.pass_safe_required_cycles = 1.0;
    overtake_planner::OvertakePlannerCore core(frame, config);

    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = 0.1;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 19.901, 0.60);
    target.id = "d2";
    target.stamp_sec = 0.1;
    target.v = 3.70;
    target.vx = 3.70;
    const auto prepared =
        core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    ASSERT_EQ(prepared.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

    ego.stamp_sec = 0.2;
    target.stamp_sec = 0.2;
    auto incomplete = readyReentryInput();
    if (incomplete_input == IncompleteInput::OPPONENT_STALE) {
      incomplete.all_observed_opponents_fresh = false;
    } else if (incomplete_input == IncompleteInput::OPPONENT_EXCLUDED) {
      incomplete.all_observed_opponents_included = false;
    } else if (incomplete_input == IncompleteInput::V2X_STALE) {
      incomplete.v2x_snapshot_fresh = false;
    } else {
      incomplete.reference_valid = false;
    }
    const auto rejected = core.update(
        0.2, ego, {target}, overtake_planner::MpcHealthStatus{}, incomplete);

    EXPECT_EQ(rejected.blocked_info.pass_start_target_continuity_reason,
              "direct_target_classified");
    EXPECT_TRUE(rejected.blocked_info.pass_right_candidate_generated);
    EXPECT_FALSE(rejected.blocked_info.pass_right_candidate_feasible);
    EXPECT_EQ(rejected.blocked_info.pass_right_candidate_reject_reason,
              "pass_start_inputs_incomplete");
    EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
    EXPECT_FALSE(rejected.blocked_info.maneuver_transaction_incomplete);
  }
}

TEST(OvertakePlannerCore,
     IncompleteFirstCycleCannotLockGateTwoPassSideOrProfile) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.pass_safe_required_cycles = 1.0;
  // このテストは不完全入力後のside再選択を検証する。右車両を避ける左PASSの
  // 横移動を十分手前から始め、controller追従性を独立した失敗要因にしない。
  config.localized_avoidance_start_before_target_m = 10.0;
  config.safety_ellipse_b_m = 0.15;
  config.pass_target_lateral_margin_m = 0.0;
  config.attack_follow_max_steering_angle_rad = 0.5585053606381855;
  config.attack_follow_max_steering_rate_radps = 8.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.stamp_sec = 0.1;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 19.901, 0.20);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 2.0;
  target.vx = 2.0;
  auto incomplete = readyReentryInput();
  incomplete.all_observed_opponents_included = false;
  const auto first = core.update(
      0.1, ego, {target}, overtake_planner::MpcHealthStatus{}, incomplete);

  ASSERT_TRUE(
      std::isfinite(first.blocked_info.pass_right_candidate_target_d_m));
  EXPECT_FALSE(first.blocked_info.maneuver_target_pass_safety_approved);
  EXPECT_FALSE(first.blocked_info.maneuver_transaction_prepared);
  EXPECT_FALSE(first.blocked_info.maneuver_transaction_incomplete);
  EXPECT_NE(first.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  // 入力が揃った次周期で、未認可の右側候補だけを塞ぐ。
  // 初回の不完全入力がside/profileを固定していなければ、同周期の
  // SafetyEvaluatorが合格した左側を独立Gate 2で選び直せる。
  ego.stamp_sec = 0.2;
  target.stamp_sec = 0.2;
  auto right_blocker = makeOpponent(
      frame, 19.901, first.blocked_info.pass_right_candidate_target_d_m);
  right_blocker.id = "d3";
  right_blocker.stamp_sec = 0.2;
  right_blocker.v = 2.0;
  right_blocker.vx = 2.0;
  const auto approved =
      core.update(0.2, ego, {target, right_blocker},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_FALSE(approved.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(approved.blocked_info.pass_left_candidate_feasible)
      << approved.blocked_info.pass_left_candidate_reject_reason
      << " left_target=" << approved.blocked_info.pass_left_candidate_target_d_m
      << " right_target="
      << approved.blocked_info.pass_right_candidate_target_d_m;
  EXPECT_TRUE(approved.blocked_info.maneuver_target_pass_safety_approved);
  EXPECT_EQ(approved.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(approved.raw_selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(approved.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     StaticLocalizedPassStillRequiresLiveTrackingBeforeCommit) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.dynamic_pass_candidate_enabled = false;
  config.pass_safe_required_cycles = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.stamp_sec = 0.1;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 19.901, 0.60);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 3.70;
  target.vx = 3.70;
  const auto prepared =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(prepared.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  ego.stamp_sec = 0.2;
  target.stamp_sec = 0.2;
  auto tracking_unusable = readyReentryInput();
  tracking_unusable.pure_pursuit_primary_and_fresh = false;
  tracking_unusable.controller_tracking_status_received = true;
  tracking_unusable.controller_tracking_plan_generation = 68U;
  tracking_unusable.controller_tracking_expected_generation = 70U;
  tracking_unusable.controller_tracking_status_reason = "generation_mismatch";
  tracking_unusable.controller_tracking_mpc_horizon_usable = false;
  tracking_unusable.controller_tracking_continuity_usable = false;
  const auto rejected =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  tracking_unusable);

  EXPECT_TRUE(rejected.blocked_info.pass_right_candidate_generated);
  EXPECT_FALSE(rejected.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(rejected.blocked_info.pass_right_candidate_reject_reason,
            "pass_start_tracking_unusable");
  const auto &tracking_diagnostic =
      rejected.blocked_info.pass_start_tracking_diagnostic;
  EXPECT_TRUE(tracking_diagnostic.evaluated);
  EXPECT_TRUE(tracking_diagnostic.candidate_present);
  EXPECT_TRUE(tracking_diagnostic.actual_pose_start_evaluated);
  EXPECT_TRUE(tracking_diagnostic.actual_pose_start);
  EXPECT_TRUE(std::isfinite(tracking_diagnostic.endpoint_arc_m));
  EXPECT_TRUE(std::isfinite(tracking_diagnostic.required_arc_m));
  EXPECT_EQ(tracking_diagnostic.controller_plan_generation, 68U);
  EXPECT_EQ(tracking_diagnostic.controller_expected_generation, 70U);
  EXPECT_EQ(tracking_diagnostic.controller_status_reason,
            "generation_mismatch");
  EXPECT_FALSE(tracking_diagnostic.controller_mpc_horizon_usable);
  EXPECT_FALSE(tracking_diagnostic.controller_continuity_usable);
  EXPECT_EQ(tracking_diagnostic.first_false, "generation_mismatch");
  EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(rejected.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(rejected.tracking_release_token, 0U);
}

TEST(OvertakePlannerCore,
     PassStartTrackingDiagnosticDoesNotChangeAuthorityOrProbeState) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.dynamic_pass_candidate_enabled = false;
  config.pass_safe_required_cycles = 1.0;
  const auto run = [&](bool include_diagnostic) {
    overtake_planner::OvertakePlannerCore core(frame, config);
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = 0.1;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 19.901, 0.60);
    target.id = "d2";
    target.stamp_sec = 0.1;
    target.v = 3.70;
    target.vx = 3.70;
    const auto prepared =
        core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    EXPECT_EQ(prepared.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

    ego.stamp_sec = 0.2;
    target.stamp_sec = 0.2;
    auto input = readyReentryInput();
    input.pure_pursuit_primary_and_fresh = false;
    if (include_diagnostic) {
      input.controller_tracking_status_received = true;
      input.controller_tracking_plan_generation = 68U;
      input.controller_tracking_expected_generation = 70U;
      input.controller_tracking_status_reason = "generation_mismatch";
      input.controller_tracking_mpc_horizon_usable = false;
      input.controller_tracking_continuity_usable = false;
    }
    return core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                       input);
  };

  const auto plain = run(false);
  const auto diagnosed = run(true);
  EXPECT_EQ(diagnosed.selected, plain.selected);
  EXPECT_EQ(diagnosed.mode, plain.mode);
  EXPECT_EQ(diagnosed.active_override, plain.active_override);
  EXPECT_EQ(diagnosed.tracking_release_token, plain.tracking_release_token);
  EXPECT_EQ(diagnosed.tracking_release_token, 0U);
  EXPECT_EQ(diagnosed.blocked_info.maneuver_transaction_tracking_probe_cycles,
            plain.blocked_info.maneuver_transaction_tracking_probe_cycles);
  EXPECT_EQ(diagnosed.blocked_info.pass_start_tracking_diagnostic.first_false,
            "generation_mismatch");
}

TEST(OvertakePlannerCore,
     PassStartContinuityRejectsUnusableLiveTrackingInputs) {
  enum class TrackingFailure { PP_NOT_PRIMARY, MPC_STALE, MPC_HARD_FAILURE };
  const std::vector<TrackingFailure> failures = {
      TrackingFailure::PP_NOT_PRIMARY, TrackingFailure::MPC_STALE,
      TrackingFailure::MPC_HARD_FAILURE};

  for (const auto failure : failures) {
    const auto frame = makeStraightFrame();
    const auto config = makePassStartContinuityConfig();
    overtake_planner::OvertakePlannerCore core(frame, config);

    for (int cycle = 1; cycle <= 4; ++cycle) {
      const double now_sec = 0.1 * static_cast<double>(cycle);
      auto ego = makeEgo(frame, 5.0, 0.0);
      ego.stamp_sec = now_sec;
      ego.v = 4.0;
      auto target = makeOpponent(frame, 19.901, 0.60);
      target.id = "d2";
      target.stamp_sec = now_sec;
      target.v = 3.70;
      target.vx = 3.70;
      const auto primed =
          core.update(now_sec, ego, {target},
                      overtake_planner::MpcHealthStatus{}, readyReentryInput());
      ASSERT_TRUE(primed.blocked_info.pass_right_candidate_feasible);
    }

    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = 0.5;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 20.508, 0.60);
    target.id = "d2";
    target.stamp_sec = 0.5;
    target.v = 3.85;
    target.vx = 3.85;
    auto tracking = readyReentryInput();
    if (failure == TrackingFailure::PP_NOT_PRIMARY) {
      tracking.pure_pursuit_primary_and_fresh = false;
    } else if (failure == TrackingFailure::MPC_STALE) {
      tracking.mpc_health_fresh = false;
    } else {
      tracking.mpc_hard_failure = true;
    }

    const auto rejected = core.update(
        0.5, ego, {target}, overtake_planner::MpcHealthStatus{}, tracking);
    EXPECT_FALSE(rejected.blocked_info.pass_start_target_continuity_active);
    EXPECT_EQ(rejected.blocked_info.pass_start_target_continuity_reason,
              "pass_start_tracking_unusable");
    EXPECT_EQ(rejected.blocked_info.pass_right_safe_cycles, 0);
    EXPECT_NE(rejected.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
    EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  }
}

TEST(OvertakePlannerCore,
     UnsafePersistentDropoutDuringPrepareCannotCommitPass) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.braking_follow_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto safe_sample = [&](double now_sec, double relative_s_m) {
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 5.0 + relative_s_m, 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = relative_s_m < config.lookahead_s_m ? 3.70 : 3.85;
    target.vx = target.v;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const auto primed = safe_sample(0.1 * static_cast<double>(cycle), 14.901);
    ASSERT_TRUE(primed.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << primed.blocked_info.pass_right_candidate_reject_reason;
  }
  const auto prepared = safe_sample(0.5, 15.508);
  ASSERT_EQ(prepared.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.stamp_sec = 0.6;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 20.508, 0.60);
  target.id = "d2";
  target.stamp_sec = 0.6;
  target.v = 3.85;
  target.vx = 3.85;
  auto blocker = makeOpponent(frame, 20.508, 0.0);
  blocker.id = "d4";
  blocker.stamp_sec = 0.6;
  blocker.v = 0.0;
  blocker.vx = 0.0;
  const auto rejected =
      core.update(0.6, ego, {target, blocker},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(rejected.blocked_info.pass_start_target_continuity_active)
      << rejected.blocked_info.pass_start_target_continuity_reason;
  EXPECT_TRUE(rejected.blocked_info.maneuver_target_safety_evaluated);
  EXPECT_FALSE(rejected.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(rejected.blocked_info.pass_right_candidate_reject_reason,
            "opponent_collision");
  EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_FALSE(rejected.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     PassStartContinuityRejectsStaleReverseOutsideCorridorAndDifferentId) {
  enum class RejectionCase { STALE, REVERSE, OUTSIDE_CORRIDOR, DIFFERENT_ID };
  const std::vector<RejectionCase> rejection_cases = {
      RejectionCase::STALE, RejectionCase::REVERSE,
      RejectionCase::OUTSIDE_CORRIDOR, RejectionCase::DIFFERENT_ID};

  for (const auto rejection_case : rejection_cases) {
    const auto frame = makeStraightFrame();
    const auto config = makePassStartContinuityConfig();
    overtake_planner::OvertakePlannerCore core(frame, config);

    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = 0.1;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 19.90, 0.60);
    target.id = "d2";
    target.stamp_sec = 0.1;
    target.v = 3.70;
    target.vx = 3.70;
    const auto primed =
        core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    ASSERT_TRUE(primed.blocked_info.pass_right_candidate_feasible);
    ASSERT_EQ(primed.blocked_info.pass_right_safe_cycles, 1);

    ego.stamp_sec = 0.2;
    target = makeOpponent(frame, 20.02, 0.60);
    target.id = "d2";
    target.stamp_sec = 0.2;
    target.v = 3.85;
    target.vx = 3.85;
    switch (rejection_case) {
    case RejectionCase::STALE:
      target.stamp_sec = -1.0;
      break;
    case RejectionCase::REVERSE:
      target.v = 1.0;
      target.vx = -1.0;
      break;
    case RejectionCase::OUTSIDE_CORRIDOR:
      target = makeOpponent(frame, 20.02, 1.10);
      target.id = "d2";
      target.stamp_sec = 0.2;
      target.v = 3.85;
      target.vx = 3.85;
      break;
    case RejectionCase::DIFFERENT_ID:
      target.id = "d3";
      break;
    }

    const auto rejected =
        core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    EXPECT_FALSE(rejected.blocked_info.pass_start_target_continuity_active)
        << "case=" << static_cast<int>(rejection_case) << " reason="
        << rejected.blocked_info.pass_start_target_continuity_reason;
    EXPECT_NE(rejected.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
    EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
    EXPECT_EQ(rejected.blocked_info.pass_right_safe_cycles, 0)
        << "case=" << static_cast<int>(rejection_case);
  }
}

TEST(OvertakePlannerCore,
     PassStartContinuityStillRequiresCurrentCycleSafetyEvaluation) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.braking_follow_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = 4.0;
    auto target = makeOpponent(frame, 19.90, 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = 3.70;
    target.vx = 3.70;
    const auto output =
        core.update(now_sec, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    ASSERT_TRUE(output.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << output.blocked_info.pass_right_candidate_reject_reason;
  }

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.stamp_sec = 0.5;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 20.508, 0.60);
  target.id = "d2";
  target.stamp_sec = 0.5;
  target.v = 3.85;
  target.vx = 3.85;
  auto blocker = makeOpponent(frame, 20.508, 0.0);
  blocker.id = "d4";
  blocker.stamp_sec = 0.5;
  blocker.v = 0.0;
  blocker.vx = 0.0;

  const auto output =
      core.update(0.5, ego, {target, blocker},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(output.blocked_info.pass_start_target_continuity_active)
      << output.blocked_info.pass_start_target_continuity_reason;
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.maneuver_target_safety_evaluated);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible)
      << "reject=" << output.blocked_info.pass_right_candidate_reject_reason
      << " target_d=" << output.blocked_info.pass_right_candidate_target_d_m
      << " corridor_margin="
      << output.blocked_info.pass_right_candidate_corridor_min_margin_m;
  EXPECT_EQ(output.blocked_info.pass_right_candidate_reject_reason,
            "opponent_collision");
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
}

TEST(OvertakePlannerCore, PassStartContinuityExpiresAfterBoundedPrestartHold) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.braking_follow_enabled = false;
  config.min_mode_hold_time_sec = 10.0;
  config.control_rate_hz = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto sample = [&](double now_sec, double relative_s_m,
                          bool obstruct_pass) {
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.stamp_sec = now_sec;
    ego.v = obstruct_pass ? 10.0 : 4.0;
    auto target =
        makeOpponent(frame, 5.0 + relative_s_m, obstruct_pass ? 0.20 : 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = obstruct_pass
                   ? 1.0
                   : (relative_s_m < config.lookahead_s_m ? 3.70 : 3.85);
    target.vx = target.v;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const auto primed = sample(0.1 * static_cast<double>(cycle), 14.901, false);
    ASSERT_TRUE(primed.blocked_info.pass_right_candidate_feasible);
  }
  for (int cycle = 5; cycle <= 19; ++cycle) {
    const auto held = sample(0.1 * static_cast<double>(cycle), 15.508, false);
    EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    EXPECT_TRUE(held.blocked_info.pass_start_target_continuity_active)
        << "cycle=" << cycle
        << " reason=" << held.blocked_info.pass_start_target_continuity_reason;
    EXPECT_TRUE(held.blocked_info.pass_right_candidate_feasible)
        << held.blocked_info.pass_right_candidate_reject_reason;
    EXPECT_EQ(held.blocked_info.pass_start_target_continuity_budget_cycles, 15);
  }

  const auto expired = sample(2.0, 15.508, false);
  EXPECT_TRUE(expired.blocked_info.pass_start_target_continuity_expired);
  EXPECT_FALSE(expired.blocked_info.pass_start_target_continuity_active);
  EXPECT_EQ(expired.blocked_info.pass_start_target_continuity_reason,
            "bounded_lifetime_expired");
  EXPECT_FALSE(expired.blocked_info.pass_right_candidate_generated);
  EXPECT_EQ(expired.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_NE(expired.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     PassStartProfileLifetimeDoesNotResetOnAlternatingTrackingFailure) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.braking_follow_enabled = false;
  config.min_mode_hold_time_sec = 10.0;
  config.control_rate_hz = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.stamp_sec = 0.1;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 19.901, 0.60);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 3.70;
  target.vx = 3.70;
  const auto primed =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(primed.blocked_info.pass_right_candidate_feasible);

  for (int cycle = 1; cycle <= 15; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle + 1);
    ego.stamp_sec = now_sec;
    target = makeOpponent(frame, 20.508, 0.60);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = 3.85;
    target.vx = 3.85;
    auto input = readyReentryInput();
    if (cycle % 2 == 0) {
      input.pure_pursuit_primary_and_fresh = false;
    }
    const auto held = core.update(now_sec, ego, {target},
                                  overtake_planner::MpcHealthStatus{}, input);
    EXPECT_FALSE(held.blocked_info.pass_start_target_continuity_expired)
        << "cycle=" << cycle;
    EXPECT_EQ(held.blocked_info.pass_start_target_continuity_cycles, cycle);
    EXPECT_EQ(held.blocked_info.pass_start_target_continuity_budget_cycles, 15);
    if (cycle % 2 == 0) {
      EXPECT_FALSE(held.blocked_info.pass_start_target_continuity_active);
      EXPECT_EQ(held.blocked_info.pass_start_target_continuity_reason,
                "pass_start_tracking_unusable");
      EXPECT_EQ(held.blocked_info.pass_right_safe_cycles, 0);
    } else {
      EXPECT_TRUE(held.blocked_info.pass_start_target_continuity_active)
          << held.blocked_info.pass_start_target_continuity_reason;
    }
  }

  ego.stamp_sec = 1.7;
  target.stamp_sec = 1.7;
  const auto expired =
      core.update(1.7, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_TRUE(expired.blocked_info.pass_start_target_continuity_expired);
  EXPECT_FALSE(expired.blocked_info.pass_start_target_continuity_active);
  EXPECT_FALSE(expired.blocked_info.pass_right_candidate_generated);
  EXPECT_EQ(expired.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_NE(expired.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(expired.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     BrakingFollowRejectsStaleReverseAndOutsideCorridorTargets) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  // このテストは速度上限gateそのものを個別に固定する。runtime既定値は
  // 動く遅い前走車を含める10 m/sへ拡張している。
  config.braking_follow_max_target_speed_mps = 1.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.same_corridor_width_m = 0.8;

  const auto run = [&](overtake_planner::OpponentState target) {
    overtake_planner::OvertakePlannerCore core(frame, config);
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.v = 6.0;
    return core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  auto stale = makeOpponent(frame, 22.0, 0.0);
  stale.stamp_sec = -1.0;
  stale.vx = 0.4;
  stale.v = 0.4;
  const auto stale_output = run(stale);
  EXPECT_FALSE(stale_output.blocked_info.braking_follow_active);
  EXPECT_FALSE(stale_output.blocked_info.early_low_speed_pass_target_active);

  auto future = makeOpponent(frame, 22.0, 0.0);
  future.stamp_sec = 1.0;
  future.vx = 0.4;
  future.v = 0.4;
  const auto future_output = run(future);
  EXPECT_FALSE(future_output.blocked_info.braking_follow_active);
  EXPECT_FALSE(future_output.blocked_info.early_low_speed_pass_target_active);

  auto reverse = makeOpponent(frame, 22.0, 0.0);
  reverse.vx = -1.0;
  reverse.v = 1.0;
  const auto reverse_output = run(reverse);
  EXPECT_FALSE(reverse_output.blocked_info.braking_follow_active);
  EXPECT_FALSE(reverse_output.blocked_info.early_low_speed_pass_target_active);

  auto low_speed_reverse = makeOpponent(frame, 22.0, 0.0);
  low_speed_reverse.vx = -0.2;
  low_speed_reverse.v = 0.2;
  const auto low_speed_reverse_output = run(low_speed_reverse);
  EXPECT_FALSE(low_speed_reverse_output.blocked_info.braking_follow_active);
  EXPECT_FALSE(
      low_speed_reverse_output.blocked_info.early_low_speed_pass_target_active);

  auto outside = makeOpponent(frame, 22.0, 1.0);
  outside.vx = 0.4;
  outside.v = 0.4;
  const auto outside_output = run(outside);
  EXPECT_FALSE(outside_output.blocked_info.braking_follow_active);
  EXPECT_FALSE(outside_output.blocked_info.early_low_speed_pass_target_active);

  auto normal_speed = makeOpponent(frame, 22.0, 0.0);
  normal_speed.vx = 2.0;
  normal_speed.v = 2.0;
  EXPECT_FALSE(run(normal_speed).blocked_info.braking_follow_active);

  auto high_speed_low_projection = makeOpponent(frame, 22.0, 0.0);
  high_speed_low_projection.vx = 0.4;
  high_speed_low_projection.v = 1.2;
  EXPECT_FALSE(
      run(high_speed_low_projection).blocked_info.braking_follow_active);
}

TEST(OvertakePlannerCore,
     BrakingFollowIncludesMovingRaceTargetWhenClosingEnvelopeRequiresIt) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.max_brake_decel_mps2 = 1.5;
  config.longitudinal_response_delay_sec = 0.25;
  config.same_corridor_width_m = 0.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 6.4;
  auto moving_slow = makeOpponent(frame, 17.0, 0.0);
  moving_slow.id = "d2";
  moving_slow.vx = 2.5;
  moving_slow.v = 2.5;

  const auto output =
      core.update(0.1, ego, {moving_slow}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "d2");
  EXPECT_NEAR(output.blocked_info.braking_follow_target_speed_mps, 2.5, 1.0e-9);
  EXPECT_GT(output.blocked_info.braking_follow_relative_speed_mps, 3.8);
  EXPECT_LT(output.blocked_info.braking_follow_ttc_sec,
            config.braking_follow_ttc_threshold_sec);
  EXPECT_LE(output.blocked_info.braking_follow_delta_s,
            output.blocked_info.braking_follow_trigger_distance_m);
}

TEST(OvertakePlannerCore,
     BrakingFollowThreeSecondTtcDoesNotOverridePhysicalDistanceBoundary) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  // Latest D3 braking-follow vector.  Keep the production braking model
  // explicit so this fixture exercises the TTC envelope, not a test default.
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.same_corridor_width_m = 0.8;

  const auto run = [&](double delta_s_m) {
    overtake_planner::OvertakePlannerCore core(frame, config);
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.v = 6.791105;
    auto stopped = makeOpponent(frame, 5.0 + delta_s_m, 0.0);
    stopped.id = "d3";
    stopped.vx = 0.0;
    stopped.v = 0.0;
    return core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  // With 5 s this vector was inside the TTC envelope.  At 3 s it is beyond
  // both envelopes, while the physical braking distance remains unchanged.
  const auto distant = run(38.707149);
  EXPECT_FALSE(distant.blocked_info.braking_follow_active);

  const double expected_required_distance_m =
      6.791105 * config.longitudinal_response_delay_sec +
      (6.791105 * 6.791105 - config.safe_stop_v_mps * config.safe_stop_v_mps) /
          (2.0 * config.max_brake_decel_mps2) +
      config.safety_ellipse_a_m + config.braking_follow_trigger_margin_m;
  // The bag diagnostic is printed as 29.737331 m; keep that observed vector
  // while retaining the full-precision value for its +/- boundary.
  EXPECT_NEAR(expected_required_distance_m, 29.737331, 2.0e-6);

  // Just outside the physical bound must remain inactive; this complements
  // the inside-bound fail-safe assertion below without expanding TTC again.
  const auto outside_physical_bound =
      run(expected_required_distance_m + 1.0e-3);
  EXPECT_FALSE(outside_physical_bound.blocked_info.braking_follow_active);

  // Just inside the physical bound must still activate FOLLOW even though the
  // 3 s TTC envelope is shorter.  This is the fail-safe boundary.
  const auto inside_physical_bound = run(expected_required_distance_m - 1.0e-3);
  ASSERT_TRUE(inside_physical_bound.blocked_info.braking_follow_active);
  EXPECT_NEAR(
      inside_physical_bound.blocked_info.braking_follow_required_distance_m,
      expected_required_distance_m, 1.0e-9);
  EXPECT_NEAR(
      inside_physical_bound.blocked_info.braking_follow_trigger_distance_m,
      expected_required_distance_m, 1.0e-9);
}

TEST(OvertakePlannerCore,
     EarlyLowSpeedPassAdmissionPrecedesBrakingWithoutChangingFollowAuthority) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.same_corridor_width_m = 0.8;
  config.slow_front_exception_speed_mps = 1.0;
  config.stationary_obstacle_speed_threshold_mps = 0.3;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_safe_required_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 6.791105;
  // 物理braking境界(約29.737 m)の外、かつrequired arcとdelivery reserveを
  // 合わせた早期PASS admission境界の内側。
  auto stopped = makeOpponent(frame, 35.0, 0.0);
  stopped.id = "d3";
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_FALSE(output.blocked_info.braking_follow_active);
  ASSERT_TRUE(output.blocked_info.early_low_speed_pass_target_active);
  EXPECT_EQ(output.blocked_info.early_low_speed_pass_target_id, "d3");
  EXPECT_EQ(output.blocked_info.early_low_speed_pass_target_index, 0);
  EXPECT_GT(output.blocked_info.early_low_speed_pass_required_distance_m,
            output.blocked_info.early_low_speed_pass_target_delta_s_m);
  EXPECT_GT(output.blocked_info.early_low_speed_pass_required_transition_m,
            0.0);
  EXPECT_GT(output.blocked_info.early_low_speed_pass_required_controller_arc_m,
            0.0);
  EXPECT_EQ(output.blocked_info.maneuver_target_id, "d3");
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated ||
              output.blocked_info.pass_right_candidate_generated);
  // candidate admissionだけではFOLLOW制動やPASS motion authorityを得ない。
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     BrakingDistanceFollowRejectsFastestWhenCurrentDHoldIsUnsafe) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.max_brake_decel_mps2 = 1.5;
  config.longitudinal_response_delay_sec = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.8);
  ego.v = 6.37;
  auto stopped = makeOpponent(frame, 12.0, 0.8);
  stopped.id = "d2";
  stopped.vx = 0.20;
  stopped.v = 0.20;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_FALSE(output.blocked_info.braking_follow_feasible);
  EXPECT_FALSE(
      output.blocked_info.braking_follow_candidate_reject_reason.empty());
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_TRUE(output.speed_only_fallback_active || output.active_override ||
              output.safe_stop_triggered);
}

TEST(OvertakePlannerCore,
     BrakingDistanceFollowRequiresCompleteFreshInputsForLateralHold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 10.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.max_brake_decel_mps2 = 1.5;
  config.longitudinal_response_delay_sec = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.8);
  ego.v = 6.37;
  auto stopped = makeOpponent(frame, 22.0, 0.8);
  stopped.id = "d2";
  stopped.vx = 0.20;
  stopped.v = 0.20;
  auto stale_input = readyReentryInput();
  stale_input.all_observed_opponents_fresh = false;

  const auto output = core.update(
      0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{}, stale_input);

  EXPECT_FALSE(output.blocked_info.braking_follow_active);
  EXPECT_FALSE(output.blocked_info.braking_follow_hold_lateral);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FOLLOW);
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

TEST(OvertakePlannerCore,
     StoppedOutsideCorridorRecoveryDoesNotCompressLateralShift) {
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
  ASSERT_EQ(output.longitudinal_offsets_m.size(),
            output.lateral_offsets.size());
  EXPECT_NEAR(output.longitudinal_offsets_m.front(), 0.0, 1.0e-9);
  EXPECT_GT(output.longitudinal_offsets_m.back(), 0.0);
  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(output, 21U);
  EXPECT_EQ(wire.kind, overtake_planner::ReferenceOverrideWireKind::
                           SPATIAL_LATERAL_AND_SPEED_V4);
  ASSERT_FALSE(output.lateral_offsets.empty());
  const auto max_offset = *std::max_element(output.lateral_offsets.begin(),
                                            output.lateral_offsets.end());
  EXPECT_LE(max_offset, upper_d + 1.0e-9);
  EXPECT_GE(output.target_lateral_offset_m, upper_d - 0.1);
  EXPECT_LT(output.longitudinal_offsets_m.back(), 1.0);
}

TEST(OvertakePlannerCore, StoppedRecoveryWaitsForForwardDistanceToCenter) {
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
  EXPECT_LE(second.target_lateral_offset_m, near_wall_inside.frenet.d + 1.0e-9);
  EXPECT_GE(second.target_lateral_offset_m, near_wall_inside.frenet.d - 0.1);
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
  EXPECT_TRUE(output.recovery_speed_guard_active);
  EXPECT_GT(output.recovery_tracking_error_m,
            config.large_lateral_error_threshold_m);
  EXPECT_NEAR(output.recovery_tracking_target_d_m,
              output.target_lateral_offset_m, 1.0e-9);
}

TEST(OvertakePlannerCore,
     LargeLateralErrorBlockedContextEvaluatesPassWithoutAcceleration) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 0.6;
  config.large_lateral_error_v_max_mps = 1.8;
  config.future_side_prediction_enabled = false;
  config.dynamic_pass_candidate_enabled = true;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  const auto opponent = makeOpponent(frame, 13.0, 0.2);

  const auto stale_output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(stale_output.blocked_info.blocked);
  EXPECT_FALSE(stale_output.blocked_info.pass_decision_frozen);
  EXPECT_TRUE(stale_output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(stale_output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(stale_output.blocked_info.pass_left_candidate_feasible ||
              stale_output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(stale_output.blocked_info.pass_acceleration_allowed);
  EXPECT_NE(stale_output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(stale_output.selected, overtake_planner::CandidateType::PASS_RIGHT);

  const auto output =
      core.update(0.2, ego, {opponent}, {}, readyReentryInput());

  EXPECT_TRUE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_acceleration_allowed);
  EXPECT_TRUE(output.selected == overtake_planner::CandidateType::PASS_LEFT ||
              output.selected == overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(output.active_override);
  EXPECT_LE(output.cbf_slack, 1.0e-9);
}

TEST(OvertakePlannerCore,
     LargeLateralErrorWithParallelSideCandidateUsesSoftWallSpeedGuardOnly) {
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
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  EXPECT_NE(output.blocked_info.pass_decision_freeze_reason,
            "large_lateral_error");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_TRUE(output.wall_risk_speed_guard_active);
  expectSpeedOnlyV2Contract(output, config.wall_risk_v_max_mps);
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

TEST(OvertakePlannerCore,
     ActivePassTransitionEnvelopeDoesNotFreezeMidLateralShift) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.large_lateral_error_threshold_m = 0.60;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.min_wall_margin_m = 0.20;
  config.left_offset_m = 1.40;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);
  const auto prepare =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepare.maneuver_latch_active);

  // 開始d=0.0 mから認可済みPASS目標d=1.4 mへ遷移中の0.7 m地点。
  // 両端との距離ではなく、遷移包絡[0.0, 1.4]から外れた量だけを
  // large_lateral_errorに使うため、この周期はfreezeしてはならない。
  const auto transitioning_ego = makeEgo(frame, 5.5, 0.70);
  const auto output =
      core.update(0.2, transitioning_ego, {opponent},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  EXPECT_NE(output.blocked_info.pass_decision_freeze_reason,
            "large_lateral_error");
  EXPECT_NEAR(output.blocked_info.decision_freeze_lateral_error_m, 0.0, 1.0e-9);
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

TEST(OvertakePlannerCore,
     SoftWallAfterClearedSideBySidePublishesSpeedOnlyInsteadOfRecovery) {
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

  EXPECT_TRUE(second.wall_risk_speed_guard_active);
  expectSpeedOnlyV2Contract(second, config.wall_risk_v_max_mps);
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
     FreshPrimaryPurePursuitBypassesOnlyCleanFreeRunMpcSoftGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_clean_free_run_soft_guard_bypass_enabled = true;
  config.mpc_health_infeasible_count_threshold = 3;
  config.mpc_health_solve_time_warn_ms = 50.0;
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 1;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;
  const auto ego = makeEgo(frame, 5.0, 0.0);

  overtake_planner::OvertakePlannerCore fresh_pp_core(frame, config);
  const auto free_run =
      fresh_pp_core.update(0.1, ego, {}, health, readyReentryInput());
  EXPECT_EQ(free_run.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(free_run.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_FALSE(free_run.mpc_health_speed_guard_active);
  EXPECT_FALSE(free_run.longitudinal_speed_cap_active);

  auto no_primary_pp = readyReentryInput();
  no_primary_pp.pure_pursuit_primary_and_fresh = false;
  overtake_planner::OvertakePlannerCore unconfirmed_pp_core(frame, config);
  const auto guarded =
      unconfirmed_pp_core.update(0.1, ego, {}, health, no_primary_pp);
  EXPECT_EQ(guarded.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(guarded.mpc_health_speed_guard_active);
  EXPECT_EQ(guarded.speed_cap_reason, "mpc_health_infeasible_soft_guard");
}

TEST(OvertakePlannerCore,
     FreshPrimaryPurePursuitNeverBypassesHardOrStaleMpcGuard) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_clean_free_run_soft_guard_bypass_enabled = true;
  config.mpc_health_infeasible_count_threshold = 3;
  config.mpc_health_stale_time_sec = 0.6;
  const auto ego = makeEgo(frame, 5.0, 0.0);

  overtake_planner::MpcHealthStatus hard_health;
  hard_health.valid = true;
  hard_health.infeasible_count = 3;
  hard_health.solve_time_ms = 5.0;
  hard_health.age_sec = 0.1;
  overtake_planner::OvertakePlannerCore hard_core(frame, config);
  const auto hard =
      hard_core.update(0.1, ego, {}, hard_health, readyReentryInput());
  EXPECT_EQ(hard.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(hard.speed_cap_reason, "mpc_health_infeasible_guard");

  overtake_planner::MpcHealthStatus stale_health;
  stale_health.valid = true;
  stale_health.infeasible_count = 0;
  stale_health.solve_time_ms = 5.0;
  stale_health.age_sec = 0.8;
  overtake_planner::OvertakePlannerCore stale_core(frame, config);
  const auto stale =
      stale_core.update(0.1, ego, {}, stale_health, readyReentryInput());
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(stale.speed_cap_reason, "mpc_health_stale_guard");
}

TEST(OvertakePlannerCore,
     VerifiedInactiveMpcPurePursuitBypassesHardAndStaleInNonPassMotion) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.mpc_health_inactive_mpc_free_run_bypass_enabled = true;
  config.mpc_health_infeasible_count_threshold = 3;
  config.mpc_health_stale_time_sec = 0.6;
  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto verified_non_mpc = readyReentryInput();
  verified_non_mpc.verified_non_mpc_pure_pursuit = true;

  overtake_planner::MpcHealthStatus hard_health;
  hard_health.valid = true;
  hard_health.infeasible_count = 3;
  hard_health.solve_time_ms = 5.0;
  hard_health.age_sec = 0.1;
  overtake_planner::OvertakePlannerCore hard_core(frame, config);
  const auto hard =
      hard_core.update(0.1, ego, {}, hard_health, verified_non_mpc);
  EXPECT_EQ(hard.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(hard.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_FALSE(hard.mpc_health_speed_guard_active);
  EXPECT_FALSE(hard.longitudinal_speed_cap_active);

  overtake_planner::MpcHealthStatus stale_health;
  stale_health.valid = true;
  stale_health.infeasible_count = 0;
  stale_health.solve_time_ms = 5.0;
  stale_health.age_sec = 0.8;
  overtake_planner::OvertakePlannerCore stale_core(frame, config);
  const auto stale =
      stale_core.update(0.1, ego, {}, stale_health, verified_non_mpc);
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_FALSE(stale.mpc_health_speed_guard_active);
  EXPECT_FALSE(stale.longitudinal_speed_cap_active);

  auto front = makeOpponent(frame, 13.0, 0.0);
  config.dynamic_pass_candidate_enabled = false;
  overtake_planner::OvertakePlannerCore blocked_core(frame, config);
  const auto blocked =
      blocked_core.update(0.1, ego, {front}, hard_health, verified_non_mpc);
  EXPECT_TRUE(blocked.blocked_info.blocked);
  EXPECT_FALSE(blocked.mpc_health_speed_guard_active);
  EXPECT_NE(blocked.speed_cap_reason, "mpc_health_infeasible_guard");

  auto unverified = verified_non_mpc;
  unverified.verified_non_mpc_pure_pursuit = false;
  overtake_planner::OvertakePlannerCore unverified_core(frame, config);
  const auto guarded =
      unverified_core.update(0.1, ego, {}, hard_health, unverified);
  EXPECT_EQ(guarded.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(guarded.mpc_health_speed_guard_active);
  EXPECT_EQ(guarded.speed_cap_reason, "mpc_health_infeasible_guard");
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

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

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
  // Default ReentryInputStatus has no freshness/inclusion proof. The stop
  // trajectory remains publishable for longitudinal fail-safe, but must not
  // authorize lateral tracking under incomplete inputs.
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(output.lateral_stop_inputs_complete);
  EXPECT_FALSE(output.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(output.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
}

TEST(OvertakePlannerCore,
     CompleteCurrentDSafeStopFailsClosedToSpeedOnlyWhenHorizonIsUnproven) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.safe_stop_trigger_cycles = 1;
  config.safe_stop_v_mps = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 10.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 0.0;
  opponent.stamp_sec = 0.1;

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_EQ(output.mode, overtake_planner::BehaviorMode::SAFE_STOP);
  ASSERT_EQ(output.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_FALSE(output.active_override);
  EXPECT_FALSE(output.selected_lateral_profile_safety_verified);
  EXPECT_FALSE(output.lateral_tracking_authorized_during_stop);
  EXPECT_FALSE(output.controller_spatial_horizon_proof_valid);
  EXPECT_TRUE(std::isfinite(output.required_controller_spatial_horizon_m));
  EXPECT_GT(output.required_controller_spatial_horizon_m, 0.0);
  EXPECT_TRUE(output.lateral_offsets.empty());
  EXPECT_TRUE(output.longitudinal_offsets_m.empty());
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_TRUE(output.speed_only_fallback_active);
  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_EQ(output.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_EQ(output.reason,
            "controller_spatial_horizon_unproven_speed_only_stop");
  EXPECT_EQ(output.speed_cap_reason,
            "controller_spatial_horizon_unproven_speed_only_stop");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_TRUE(std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                          [&config](double speed_mps) {
                            return speed_mps <= config.safe_stop_v_mps;
                          }));
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
  // RECOVERYは下流最大加速を含むs(t)でSafetyEvaluatorを通すため、停止後も
  // guard上限までは再加速でき、ABORT/復帰を永久化しない。
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
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {opponent},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(first.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(first.blocked_info.can_pass_left);
  EXPECT_FALSE(first.blocked_info.maneuver_transaction_incomplete);

  const auto prepare = core.update(0.2, ego, {opponent},
                                   overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(prepare.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_NEAR(prepare.target_lateral_offset_m, 0.0, 1.0e-9);
  EXPECT_TRUE(prepare.active_override);
  // Gate 2でPASS候補が成立していても、overtake_onlyがFOLLOWをpublishする間は
  // 実行transactionを開始せず、ABORT保持の根拠にしない。
  EXPECT_TRUE(prepare.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(prepare.blocked_info.maneuver_transaction_incomplete);

  const auto overtake = core.update(0.3, ego, {opponent},
                                    overtake_planner::MpcHealthStatus{}, input);
  SCOPED_TRACE(::testing::Message()
               << "blocked=" << overtake.blocked_info.blocked
               << " nearest=" << overtake.blocked_info.nearest_index
               << " prev=" << overtake.blocked_info.maneuver_target_previous_id
               << " new=" << overtake.blocked_info.maneuver_target_new_id
               << " change="
               << overtake.blocked_info.maneuver_target_change_reason
               << " pass_gap=" << overtake.blocked_info.pass_gap_reason);
  EXPECT_EQ(overtake.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(overtake.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_GT(overtake.target_lateral_offset_m, 0.0);
  EXPECT_TRUE(overtake.blocked_info.maneuver_target_latched);
  EXPECT_TRUE(overtake.blocked_info.maneuver_target_observed);
  EXPECT_TRUE(overtake.blocked_info.maneuver_target_fresh);
  EXPECT_TRUE(overtake.maneuver_latch_active);
  EXPECT_FALSE(overtake.maneuver_latch_target_id.empty());
  EXPECT_TRUE(overtake.active_override);
  EXPECT_TRUE(overtake.selected_lateral_profile_safety_verified);
  EXPECT_NE(overtake.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_TRUE(overtake.blocked_info.maneuver_transaction_incomplete);
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
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_max_start_curvature_m_inv = 0.04;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {stopped_opponent},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(first.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto second = core.update(0.2, ego, {stopped_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(second.blocked_info.front_vehicle_low_speed);
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(second.blocked_info.overtake_permission_allowed);
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
     StationaryCurvePreflightConstrainsCandidateBeforePassAuthority) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.9;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 3.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 2.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.pass_safe_required_cycles = 5.0;

  overtake_planner::OvertakePlannerCore core(frame, config);
  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 7.796087;
  auto stopped = makeOpponent(frame, 42.318508, 0.0);
  stopped.id = "stop-1";
  stopped.stamp_sec = 0.1;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  const double curve_speed_cap_mps = std::min(
      config.stationary_no_pass_safe_pass_v_max_mps,
      std::sqrt(config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 /
                0.246944));
  const double speed_reachable_distance_m =
      ego.v * config.longitudinal_response_delay_sec +
      (ego.v * ego.v - curve_speed_cap_mps * curve_speed_cap_mps) /
          (2.0 * config.max_brake_decel_mps2);
  const double minimum_without_lateral_m =
      speed_reachable_distance_m + config.safety_ellipse_a_m +
      config.braking_follow_trigger_margin_m;

  EXPECT_NEAR(curve_speed_cap_mps, 2.845875, 1.0e-6);
  EXPECT_NEAR(speed_reachable_distance_m, 28.289007, 1.0e-6);
  EXPECT_NEAR(minimum_without_lateral_m, 33.289007, 1.0e-6);
  EXPECT_FALSE(output.blocked_info.stationary_front_obstacle);
  ASSERT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "stop-1");
  EXPECT_TRUE(
      output.blocked_info.stationary_no_pass_safe_pass_constraint_active);
  EXPECT_NEAR(output.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps,
              curve_speed_cap_mps, 1.0e-6);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated ||
              output.blocked_info.pass_right_candidate_generated);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, 0);
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_FALSE(output.blocked_info.maneuver_transaction_prepared);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);

  // preflightはcallback-localであり、次周期に対象が消えたらcapや
  // motion-authority結果を再利用しない。
  ego.stamp_sec = 0.2;
  const auto cleared = core.update(
      0.2, ego, {}, overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_FALSE(
      cleared.blocked_info.stationary_no_pass_safe_pass_constraint_active);
  EXPECT_FALSE(std::isfinite(
      cleared.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps));
  EXPECT_FALSE(
      cleared.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_FALSE(cleared.blocked_info.maneuver_transaction_prepared);
}

TEST(OvertakePlannerCore,
     StationaryCurvePreflightNeverTurnsDistanceBoundaryIntoPassAuthority) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.9;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 3.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 2.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.pass_safe_required_cycles = 5.0;

  const auto run = [&](double target_delta_s_m) {
    overtake_planner::OvertakePlannerCore core(frame, config);
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.v = 7.796087;
    auto stopped = makeOpponent(frame, 5.0 + target_delta_s_m, 0.0);
    stopped.id = "stop-1";
    stopped.stamp_sec = 0.1;
    stopped.v = 0.0;
    stopped.vx = 0.0;
    stopped.vy = 0.0;
    return core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                       readyReentryInput());
  };

  for (const double target_delta_s_m : {14.533410, 33.289007}) {
    const auto output = run(target_delta_s_m);
    EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_eligible)
        << "delta_s=" << target_delta_s_m;
    EXPECT_FALSE(
        output.blocked_info.stationary_no_pass_safe_pass_start_approved)
        << "delta_s=" << target_delta_s_m;
    EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, 0)
        << "delta_s=" << target_delta_s_m;
    EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0)
        << "delta_s=" << target_delta_s_m;
    EXPECT_FALSE(output.blocked_info.maneuver_transaction_prepared)
        << "delta_s=" << target_delta_s_m;
    EXPECT_NE(output.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT)
        << "delta_s=" << target_delta_s_m;
    EXPECT_NE(output.mode,
              overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT)
        << "delta_s=" << target_delta_s_m;
  }
}

TEST(OvertakePlannerCore,
     StationaryCurveCurrentSpeedPassRequiresConfiguredDynamicsAndFullSafety) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.9;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 10.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 18.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.overtake_lateral_profile_mode = "localized_latched";

  overtake_planner::OvertakePlannerCore core(frame, config);
  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 7.796087;
  ego.stamp_sec = 0.1;
  auto stopped = makeOpponent(frame, 42.318508, 0.0);
  stopped.id = "stop-1";
  stopped.stamp_sec = 0.1;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.stationary_front_obstacle);
  ASSERT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "stop-1");
  EXPECT_TRUE(
      output.blocked_info.stationary_no_pass_safe_pass_constraint_active);
  EXPECT_GE(output.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps,
            ego.v);
  EXPECT_TRUE(output.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible ||
              output.blocked_info.pass_right_candidate_feasible)
      << "left=" << output.blocked_info.pass_left_candidate_reject_reason
      << " right=" << output.blocked_info.pass_right_candidate_reject_reason;
  EXPECT_TRUE(output.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_tracking_profile_valid ||
              output.blocked_info.pass_right_candidate_tracking_profile_valid);
  EXPECT_TRUE((output.blocked_info.pass_left_candidate_endpoint_arc_m >=
               output.blocked_info.pass_left_candidate_required_arc_m) ||
              (output.blocked_info.pass_right_candidate_endpoint_arc_m >=
               output.blocked_info.pass_right_candidate_required_arc_m));
  EXPECT_TRUE(
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
      output.mode == overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GE(output.speed_caps.front(), ego.v);

  ego.stamp_sec = 0.2;
  stopped.stamp_sec = 0.2;
  const auto continued =
      core.update(0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(continued.blocked_info.maneuver_target_id, "stop-1");
  EXPECT_TRUE(continued.mode ==
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
              continued.mode ==
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
              continued.mode == overtake_planner::BehaviorMode::OVERTAKE_LEFT ||
              continued.mode == overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  ASSERT_FALSE(continued.speed_caps.empty());
  EXPECT_GE(continued.speed_caps.front(), ego.v);

  // 横移動開始後に通常のsame-corridor分類から外れても、freshかつ停止中の
  // 既存maneuver targetを同じPASS transactionのauthorityとして保持する。
  ego = makeEgo(frame, 5.0, 0.95);
  ego.v = 7.796087;
  ego.stamp_sec = 0.3;
  stopped.stamp_sec = 0.3;
  const auto laterally_continued =
      core.update(0.3, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(laterally_continued.blocked_info.maneuver_target_id, "stop-1");
  EXPECT_TRUE(laterally_continued.mode ==
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
              laterally_continued.mode ==
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
              laterally_continued.mode ==
                  overtake_planner::BehaviorMode::OVERTAKE_LEFT ||
              laterally_continued.mode ==
                  overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StationaryCurveCurrentSpeedPassFallsBackWhenActualPathDynamicsFail) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 15.0;
  config.follow_trigger_s_m = 12.0;
  config.same_corridor_width_m = 0.9;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 10.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  // 基準線だけなら7.796087 m/sを許すが、同速度の横connectorまで含めると
  // 余裕が不足する境界。reference curvatureだけでPASS認可しないことを固定する。
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 15.1;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;

  overtake_planner::OvertakePlannerCore core(frame, config);
  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 7.796087;
  ego.stamp_sec = 0.1;
  auto stopped = makeOpponent(frame, 42.318508, 0.0);
  stopped.id = "stop-1";
  stopped.stamp_sec = 0.1;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "stop-1");
  EXPECT_TRUE(output.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, 0);
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_FALSE(output.blocked_info.maneuver_transaction_prepared);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_TRUE(std::all_of(
      output.lateral_offsets.begin(), output.lateral_offsets.end(),
      [&](double d_m) { return std::abs(d_m - ego.frenet.d) <= 1.0e-6; }));
}

TEST(OvertakePlannerCore,
     StationaryCurvePreflightRejectsNonCurrentOrIncompleteTargetInputs) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 15.0;
  config.same_corridor_width_m = 0.9;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 10.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safety_ellipse_a_m = 3.0;
  config.safe_stop_v_mps = 0.20;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 3.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 2.0;

  const auto run_with_ego_speed =
      [&](overtake_planner::OpponentState target,
          overtake_planner::ReentryInputStatus input, double ego_speed_mps) {
        overtake_planner::OvertakePlannerCore core(frame, config);
        auto ego = makeEgo(frame, 5.0, 0.0);
        ego.v = ego_speed_mps;
        return core.update(0.1, ego, {target},
                           overtake_planner::MpcHealthStatus{}, input);
      };
  const auto run = [&](overtake_planner::OpponentState target,
                       overtake_planner::ReentryInputStatus input) {
    return run_with_ego_speed(target, input, 7.796087);
  };
  const auto expect_rejected = [](const auto &output) {
    EXPECT_FALSE(
        output.blocked_info.stationary_no_pass_safe_pass_constraint_active);
    EXPECT_FALSE(std::isfinite(
        output.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps));
    EXPECT_FALSE(
        output.blocked_info.stationary_no_pass_safe_pass_start_approved);
    EXPECT_FALSE(output.blocked_info.maneuver_transaction_prepared);
  };

  auto target = makeOpponent(frame, 42.318508, 0.0);
  target.id = "stop-1";
  target.stamp_sec = 0.1;
  target.v = 0.0;
  target.vx = 0.0;
  target.vy = 0.0;

  auto stale = target;
  stale.stamp_sec = -1.0;
  expect_rejected(run(stale, readyReentryInput()));

  auto future = target;
  future.stamp_sec = 1.0;
  expect_rejected(run(future, readyReentryInput()));

  auto invalid = target;
  invalid.valid = false;
  expect_rejected(run(invalid, readyReentryInput()));

  auto moving = target;
  moving.v = 0.300001;
  moving.vx = 0.300001;
  expect_rejected(run(moving, readyReentryInput()));

  auto reverse = target;
  reverse.v = 0.20;
  reverse.vx = -0.20;
  expect_rejected(run(reverse, readyReentryInput()));

  auto incomplete = readyReentryInput();
  incomplete.all_observed_opponents_included = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.reference_valid = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.mpc_healthy = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.ego_fresh = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.v2x_snapshot_fresh = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.all_observed_opponents_fresh = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.mpc_health_fresh = false;
  expect_rejected(run(target, incomplete));

  incomplete = readyReentryInput();
  incomplete.mpc_hard_failure = true;
  expect_rejected(run(target, incomplete));

  auto rear = makeOpponent(frame, 3.0, 0.0);
  rear.id = "stop-rear";
  rear.stamp_sec = 0.1;
  rear.v = 0.0;
  rear.vx = 0.0;
  rear.vy = 0.0;
  expect_rejected(run(rear, readyReentryInput()));

  auto no_closing = target;
  no_closing.v = 0.20;
  no_closing.vx = 0.20;
  expect_rejected(run_with_ego_speed(no_closing, readyReentryInput(), 0.20));
}

TEST(OvertakePlannerCore,
     PredictiveShadowAloneNeverActivatesStationaryCurvePreflight) {
  const auto frame = makeStationaryCurvePreflightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.follow_trigger_s_m = 12.0;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.350;
  config.stationary_no_pass_safe_pass_v_max_mps = 3.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 2.20;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 2.0;
  config.dynamic_pass_candidate_enabled = true;

  overtake_planner::OvertakePlannerCore core(frame, config);
  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 2.0;
  auto stopped = makeOpponent(frame, 18.0, 0.0);
  stopped.id = "shadow-stop";
  stopped.stamp_sec = 0.1;
  stopped.v = 0.0;
  stopped.vx = 0.0;
  stopped.vy = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.blocked_info.predictive_pass_target_shadow.valid);
  EXPECT_EQ(output.blocked_info.predictive_pass_target_shadow.target_id,
            "shadow-stop");
  EXPECT_EQ(output.blocked_info.predictive_pass_target_shadow.source,
            "future_front");
  EXPECT_FALSE(output.blocked_info.stationary_front_obstacle);
  EXPECT_FALSE(output.blocked_info.braking_follow_active);
  EXPECT_FALSE(
      output.blocked_info.stationary_no_pass_safe_pass_constraint_active);
  EXPECT_FALSE(std::isfinite(
      output.blocked_info.stationary_no_pass_safe_pass_speed_cap_mps));
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, 0);
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_FALSE(output.blocked_info.maneuver_transaction_prepared);
}

TEST(OvertakePlannerCore,
     StationaryNoPassSafePassUsesConstrainedGateTwoInCurvedSection) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"no_pass", 0.0, 20.0, false});
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.15;
  config.stationary_no_pass_safe_pass_v_max_mps = 5.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 0.70;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  // future-yield中はこの例外を使わないことを別ケースで守る。このケースは
  // Gate 2を通った停止障害物PASSだけを検証するため、future近似を無効化する。
  config.future_side_prediction_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 13.0, -0.5);
  stopped.id = "d2";
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {stopped},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(first.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_FALSE(first.blocked_info.permission_start_exception_active);
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto second = core.update(0.2, ego, {stopped},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(second.blocked_info.stationary_front_obstacle);
  EXPECT_FALSE(second.blocked_info.overtake_permission_allowed);
  EXPECT_FALSE(second.blocked_info.side_by_side);
  EXPECT_FALSE(second.blocked_info.corner_side_by_side);
  EXPECT_FALSE(second.blocked_info.future_side_by_side);
  EXPECT_FALSE(second.blocked_info.future_yield_required);
  EXPECT_FALSE(second.blocked_info.reentry_hold_active);
  EXPECT_LE(second.blocked_info.overtake_start_abs_curvature,
            config.stationary_no_pass_safe_pass_max_curvature_m_inv);
  EXPECT_TRUE(second.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_TRUE(second.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_TRUE(second.blocked_info.permission_start_exception_active);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "stationary_no_pass_safe_pass");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);

  const auto third = core.update(0.3, ego, {stopped},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(third.blocked_info.permission_start_exception_active);
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(third.selected, overtake_planner::CandidateType::PASS_LEFT);

  auto stale_input = input;
  stale_input.v2x_snapshot_fresh = false;
  const auto stale = core.update(
      0.4, ego, {stopped}, overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_FALSE(stale.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_NE(stale.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(stale.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StationaryAllowedSectionUsesSameConstrainedGateTwoInHighCurvature) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.15;
  config.stationary_no_pass_safe_pass_v_max_mps = 5.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 0.70;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 13.0, -0.5);
  stopped.id = "d2";
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {stopped},
                                 overtake_planner::MpcHealthStatus{}, input);
  const auto second = core.update(0.2, ego, {stopped},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(first.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_TRUE(second.blocked_info.overtake_permission_allowed);
  EXPECT_TRUE(second.blocked_info.stationary_front_obstacle);
  EXPECT_TRUE(second.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_TRUE(second.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_FALSE(second.blocked_info.permission_start_exception_active);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "stationary_high_curvature_safe_pass");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto third = core.update(0.3, ego, {stopped},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(third.selected, overtake_planner::CandidateType::PASS_LEFT);

  auto stale_input = input;
  stale_input.v2x_snapshot_fresh = false;
  const auto stale = core.update(
      0.4, ego, {stopped}, overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_FALSE(stale.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_NE(stale.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_NE(stale.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(stale.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StationaryNoPassSafePassReapprovalRejectsIdChangeAndUnhealthyMpc) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"no_pass", 0.0, 20.0, false});
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.15;
  config.stationary_no_pass_safe_pass_v_max_mps = 5.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 0.70;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.stationary_no_pass_safe_pass_max_cbf_slack = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 13.0, -0.5);
  stopped.id = "d2";
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto input = readyReentryInput();
  const auto enter_special_pass =
      [&](overtake_planner::OvertakePlannerCore &core) {
        const auto first = core.update(
            0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{}, input);
        const auto second = core.update(
            0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{}, input);
        const auto third = core.update(
            0.3, ego, {stopped}, overtake_planner::MpcHealthStatus{}, input);
        EXPECT_FALSE(
            first.blocked_info.stationary_no_pass_safe_pass_start_approved);
        EXPECT_TRUE(
            second.blocked_info.stationary_no_pass_safe_pass_start_approved);
        EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
      };
  const auto expect_reapproval_loss = [](const auto &output) {
    EXPECT_FALSE(
        output.blocked_info.stationary_no_pass_safe_pass_start_approved);
    EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_LEFT);
    EXPECT_NE(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_NE(output.selected, overtake_planner::CandidateType::FASTEST);
    EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
    EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  };

  auto immediate_id_config = config;
  immediate_id_config.slow_front_exception_required_cycles = 1;
  overtake_planner::OvertakePlannerCore id_change_core(frame,
                                                       immediate_id_config);
  const auto id_first = id_change_core.update(
      0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{}, input);
  const auto id_second = id_change_core.update(
      0.2, ego, {stopped}, overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(
      id_first.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_EQ(id_first.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(id_second.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  auto changed_id = stopped;
  changed_id.id = "d3";
  const auto id_changed = id_change_core.update(
      0.3, ego, {changed_id}, overtake_planner::MpcHealthStatus{}, input);
  expect_reapproval_loss(id_changed);

  overtake_planner::OvertakePlannerCore unhealthy_mpc_core(frame, config);
  enter_special_pass(unhealthy_mpc_core);
  auto unhealthy_input = input;
  unhealthy_input.mpc_healthy = false;
  const auto unhealthy = unhealthy_mpc_core.update(
      0.4, ego, {stopped}, overtake_planner::MpcHealthStatus{},
      unhealthy_input);
  expect_reapproval_loss(unhealthy);
}

TEST(OvertakePlannerCore,
     StationaryNoPassSafePassRejectsFutureYieldBeforeGateTwo) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"no_pass", 0.0, 20.0, false});
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 1;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.15;
  config.stationary_no_pass_safe_pass_v_max_mps = 5.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 0.70;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 13.0, -0.5);
  stopped.id = "d2";
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.stationary_front_obstacle);
  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_start_approved);
  EXPECT_FALSE(output.blocked_info.permission_start_exception_active);
}

TEST(OvertakePlannerCore,
     StationaryNoPassSafePassRejectsCurvatureAboveDedicatedLimit) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"no_pass", 0.0, 20.0, false});
  config.slow_front_exception_required_cycles = 1;
  config.stationary_no_pass_safe_pass_enabled = true;
  config.stationary_no_pass_safe_pass_max_curvature_m_inv = 0.10;
  config.stationary_no_pass_safe_pass_v_max_mps = 5.0;
  config.stationary_no_pass_safe_pass_max_lateral_displacement_m = 0.70;
  config.stationary_no_pass_safe_pass_max_lateral_accel_mps2 = 4.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 13.0, -0.7);
  stopped.vx = 0.0;
  stopped.v = 0.0;
  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(output.blocked_info.stationary_no_pass_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.permission_start_exception_active);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(OvertakePlannerCore,
     GentleCurveSafePassStartsOnlyAfterConstrainedCandidateSafetyPasses) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  // 通常の0.60 sec mode holdより短いgateでも、同周期のSafetyEvaluator承認を
  // 継続している制限PASSだけは必要safe cycle達成時に開始できる。
  config.min_mode_hold_time_sec = 0.60;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vx = 2.0;
  opponent.v = 2.0;
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {opponent},
                                 overtake_planner::MpcHealthStatus{}, input);

  ASSERT_TRUE(first.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_TRUE(first.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_TRUE(first.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.blocked_info.overtake_start_gate_reason,
            "gentle_curve_safe_pass");
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(first.selected, overtake_planner::CandidateType::PASS_LEFT);
  ASSERT_FALSE(first.lateral_offsets.empty());
  ASSERT_FALSE(first.speed_caps.empty());
  for (const double d : first.lateral_offsets) {
    EXPECT_LE(std::abs(d - ego.frenet.d),
              config.gentle_curve_safe_pass_max_lateral_displacement_m +
                  1.0e-9);
  }
  for (const double v_ref : first.speed_caps) {
    EXPECT_LE(v_ref, config.gentle_curve_safe_pass_v_max_mps + 1.0e-9);
  }

  const auto second = core.update(0.2, ego, {opponent},
                                  overtake_planner::MpcHealthStatus{}, input);
  SCOPED_TRACE(::testing::Message()
               << "selected_reason=" << second.reason << " pass_left_feasible="
               << second.blocked_info.pass_left_candidate_feasible
               << " pass_left_reject="
               << second.blocked_info.pass_left_candidate_reject_reason
               << " pass_right_feasible="
               << second.blocked_info.pass_right_candidate_feasible
               << " pass_right_reject="
               << second.blocked_info.pass_right_candidate_reject_reason
               << " start_allowed="
               << second.blocked_info.straight_overtake_start_allowed
               << " start_reason="
               << second.blocked_info.overtake_start_gate_reason
               << " gentle_eligible="
               << second.blocked_info.gentle_curve_safe_pass_eligible
               << " gentle_approved="
               << second.blocked_info.gentle_curve_safe_pass_start_approved
               << " continuity="
               << second.blocked_info.pass_start_target_continuity_active
               << " continuity_reason="
               << second.blocked_info.pass_start_target_continuity_reason);
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);
  ASSERT_FALSE(second.lateral_offsets.empty());
  ASSERT_FALSE(second.speed_caps.empty());
  for (const double d : second.lateral_offsets) {
    EXPECT_LE(std::abs(d - ego.frenet.d),
              config.gentle_curve_safe_pass_max_lateral_displacement_m +
                  1.0e-9);
  }
  for (const double v_ref : second.speed_caps) {
    EXPECT_LE(v_ref, config.gentle_curve_safe_pass_v_max_mps + 1.0e-9);
  }
}

TEST(OvertakePlannerCore,
     GentleCurvePrepareCannotCommitAfterCurrentSafetyEvaluationFails) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  config.min_mode_hold_time_sec = 0.60;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.vx = 2.0;
  target.v = 2.0;
  const auto input = readyReentryInput();
  const auto prepared = core.update(0.1, ego, {target},
                                    overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepared.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepared.blocked_info.gentle_curve_safe_pass_start_approved);

  auto blocker = makeOpponent(
      frame, 13.0, prepared.blocked_info.pass_left_candidate_target_d_m);
  blocker.id = "d3";
  blocker.vx = 0.0;
  blocker.v = 0.0;
  const auto rejected = core.update(0.2, ego, {target, blocker},
                                    overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(rejected.blocked_info.pass_left_candidate_generated);
  EXPECT_FALSE(rejected.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(rejected.blocked_info.pass_left_candidate_reject_reason,
            "opponent_collision");
  EXPECT_FALSE(rejected.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_FALSE(rejected.blocked_info.straight_overtake_start_allowed);
  EXPECT_NE(rejected.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  // PREPAREでpublish済みのtransaction identityは保持するが、安全不成立の
  // 同周期にOVERTAKEを継続せず、次の再認証まで横移動を進めない。
  EXPECT_TRUE(rejected.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(rejected.blocked_info.maneuver_target_id, "d2");
}

TEST(OvertakePlannerCore, GentleCurveSafePassKeepsInitialLateralAnchor) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto initial_ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vx = 2.0;
  opponent.v = 2.0;
  const auto first = core.update(0.1, initial_ego, {opponent},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  const auto moved_ego = makeEgo(frame, 5.2, 0.55);
  const auto second = core.update(0.2, moved_ego, {opponent},
                                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_FALSE(second.lateral_offsets.empty());
  for (const double d : second.lateral_offsets) {
    EXPECT_LE(std::abs(d - initial_ego.frenet.d),
              config.gentle_curve_safe_pass_max_lateral_displacement_m +
                  1.0e-9);
  }
}

TEST(CandidateBuilder, GentleCurveConstraintClampsEntireTrajectoryToAnchor) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  overtake_planner::BlockedInfo blocked;
  blocked.gentle_curve_safe_pass_constraint_active = true;
  blocked.gentle_curve_safe_pass_anchor_d_m = 0.0;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {});
  ASSERT_FALSE(candidate.d.empty());
  for (const double d : candidate.d) {
    EXPECT_LE(d, 0.70 + 1.0e-9);
    EXPECT_GE(d, -0.70 - 1.0e-9);
  }
}

TEST(OvertakePlannerCore,
     ProductionScaleModerateCurvePassUsesLateralAccelerationSpeedCap) {
  const auto frame = makeModerateCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.090;
  config.gentle_curve_safe_pass_v_max_mps = 10.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.20;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.gentle_curve_safe_pass_max_cbf_slack = 0.0;
  config.gentle_curve_safe_pass_bypass_mode_hold_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.future_side_prediction_enabled = false;
  config.large_lateral_error_threshold_m = 0.60;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  config.same_corridor_width_m = 1.20;
  config.d_min_m = -2.50;
  config.d_max_m = 2.50;
  config.min_wall_margin_m = 0.10;
  config.min_pass_gap_m = 0.2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.75);
  auto opponent = makeOpponent(frame, 13.0, 0.0);
  opponent.vx = 2.0;
  opponent.v = 2.0;

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  const double expected_speed_cap_mps = std::sqrt(4.0 / 0.080);
  EXPECT_GE(output.blocked_info.ego_wall_clearance_m, 0.0);
  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_FALSE(output.blocked_info.corner_side_by_side);
  EXPECT_FALSE(output.blocked_info.future_yield_required);
  EXPECT_TRUE(output.blocked_info.gentle_curve_safe_pass_eligible)
      << "reason=" << output.blocked_info.gentle_curve_safe_pass_block_reason
      << " identity=" << output.blocked_info.gentle_curve_target_identity_valid
      << " direct=" << output.blocked_info.gentle_curve_direct_normal_target
      << " dynamic="
      << output.blocked_info.gentle_curve_fresh_dynamic_gap_target
      << " reachable="
      << output.blocked_info.gentle_curve_dynamic_target_speed_reachable
      << " clean=" << output.blocked_info.gentle_curve_context_clean
      << " lateral="
      << output.blocked_info.gentle_curve_lateral_capacity_sufficient
      << " speed=" << output.blocked_info.gentle_curve_dynamic_speed_cap_valid
      << " curvature="
      << output.blocked_info.gentle_curve_curvature_within_limit;
  EXPECT_TRUE(output.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_FALSE(output.blocked_info.pass_decision_frozen);
  // localized_latchedでは中心線との差ではなく認可対象profileとの差で横誤差を
  // 判定するため、加速予測もGate 2へ含められる。最終速度は下の曲率由来capで
  // 拘束され、SafetyEvaluatorを通った同じs(t)だけがpublishされる。
  EXPECT_TRUE(output.blocked_info.pass_acceleration_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason,
            "gentle_curve_safe_pass");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  ASSERT_FALSE(output.speed_caps.empty());
  for (const double v_ref : output.speed_caps) {
    EXPECT_LE(v_ref, expected_speed_cap_mps + 1.0e-9);
  }
  EXPECT_LE(output.cbf_slack, 1.0e-9);
}

TEST(OvertakePlannerCore,
     D1RepresentativeDynamicTargetExposesInsufficientCurveLateralCapacity) {
  const auto frame = makeD1RepresentativeCurveFrame();
  const auto config = makeD1RepresentativeCurveConfig();
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 32.86, 0.68);
  ego.stamp_sec = 0.1;
  ego.v = 0.19;
  auto target = makeOpponent(frame, 38.95, 0.18);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 2.25;
  target.vx = 2.25;

  const auto output =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_NEAR(output.blocked_info.front_delta_s, 6.09, 1.0e-6);
  EXPECT_NEAR(output.blocked_info.front_delta_d, -0.50, 1.0e-6);
  EXPECT_NEAR(output.blocked_info.front_rel_v, -2.06, 1.0e-6);
  EXPECT_TRUE(output.blocked_info.gentle_curve_target_identity_valid);
  EXPECT_FALSE(output.blocked_info.gentle_curve_direct_normal_target);
  EXPECT_TRUE(output.blocked_info.gentle_curve_fresh_dynamic_gap_target);
  EXPECT_TRUE(output.blocked_info.gentle_curve_dynamic_target_speed_reachable);
  EXPECT_TRUE(output.blocked_info.gentle_curve_curvature_within_limit);
  EXPECT_FALSE(output.blocked_info.gentle_curve_lateral_capacity_sufficient);
  EXPECT_FALSE(output.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.gentle_curve_safe_pass_block_reason,
            "lateral_capacity_insufficient");
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
}

TEST(OvertakePlannerCore,
     D1RepresentativeDynamicTargetNeedsFiveFreshSafeCurveCycles) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  // D1実測幾何では0.68 -> -1.892 mが必要。2.20 mの短い見かけ上の
  // PASSではなく、必要楕円分離まで届く容量をこの正例だけに与える。
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  overtake_planner::OvertakePlannerCore core(frame, config);

  overtake_planner::PlannerOutput output;
  for (int cycle = 1; cycle <= 5; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    auto ego = makeEgo(frame, 32.86, 0.68);
    ego.stamp_sec = now_sec;
    ego.v = 0.19;
    auto target = makeOpponent(frame, 38.95, 0.18);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = 2.25;
    target.vx = 2.25;
    output =
        core.update(now_sec, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());

    ASSERT_TRUE(output.blocked_info.gentle_curve_fresh_dynamic_gap_target)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_dynamic_target_speed_reachable)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_lateral_capacity_sufficient)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_safe_pass_eligible)
        << "cycle=" << cycle << " reason="
        << output.blocked_info.gentle_curve_safe_pass_block_reason;
    ASSERT_TRUE(output.blocked_info.pass_right_candidate_feasible)
        << "cycle=" << cycle
        << " reject=" << output.blocked_info.pass_right_candidate_reject_reason;
    EXPECT_TRUE(output.blocked_info.gentle_curve_safe_pass_start_approved);
    EXPECT_EQ(output.blocked_info.gentle_curve_safe_pass_side,
              overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_EQ(output.raw_selected, overtake_planner::CandidateType::PASS_RIGHT);
    EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, cycle);
    if (cycle < 5) {
      EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
      EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
      EXPECT_TRUE(output.blocked_info.follow_gap_closing_allowed);
      EXPECT_TRUE(output.blocked_info.prestart_attack_follow_hold_lateral);
      EXPECT_TRUE(output.blocked_info.attack_follow_candidate_generated);
      EXPECT_TRUE(output.blocked_info.attack_follow_candidate_feasible)
          << output.blocked_info.attack_follow_candidate_reject_reason;
      EXPECT_TRUE(
          output.blocked_info.attack_follow_candidate_safe_lateral_hold);
      EXPECT_TRUE(
          output.blocked_info.attack_follow_candidate_tracking_profile_valid);
      ASSERT_FALSE(output.lateral_offsets.empty());
      for (const double lateral_offset_m : output.lateral_offsets) {
        EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-9);
      }
      EXPECT_GT(output.blocked_info.follow_candidate_speed_cap_mps, target.v);
      EXPECT_GT(output.blocked_info.follow_candidate_terminal_speed_mps,
                target.v);
    }
  }

  EXPECT_EQ(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_RIGHT);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_LT(output.lateral_offsets.back(), 0.68);
  ASSERT_FALSE(output.speed_caps.empty());
  const double curve_speed_cap_mps = std::sqrt(4.0 / 0.054);
  for (const double speed_cap_mps : output.speed_caps) {
    EXPECT_LE(speed_cap_mps, curve_speed_cap_mps + 1.0e-9);
  }
  EXPECT_LE(output.cbf_slack, 1.0e-9);
}

TEST(OvertakePlannerCore,
     PrestartAttackFollowDoesNotBlendPreviousLateralOverride) {
  const auto frame = makeStraightFrame();
  auto config = makePassStartContinuityConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.parallel_side_detection_enabled = true;
  config.same_corridor_width_m = 0.60;
  config.side_by_side_s_m = 0.50;
  config.side_margin_m = 0.80;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  config.min_mode_hold_time_sec = 0.0;
  config.lateral_target_max_step_m = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto first_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 5.0, 0.6);
  const auto first = core.update(0.1, first_ego, {first_opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  ASSERT_TRUE(first.active_override);

  const auto parallel_ego = makeEgo(frame, 6.0, 0.4);
  const auto parallel_front = makeOpponent(frame, 10.0, -0.55);
  const auto parallel = core.update(0.2, parallel_ego, {parallel_front});
  ASSERT_EQ(parallel.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  ASSERT_TRUE(parallel.blocked_info.parallel_follow_candidate);
  ASSERT_TRUE(parallel.active_override);
  ASSERT_FALSE(parallel.lateral_offsets.empty());
  for (const double lateral_offset_m : parallel.lateral_offsets) {
    ASSERT_NEAR(lateral_offset_m, parallel_ego.frenet.d, 1.0e-9);
  }

  auto ego = makeEgo(frame, 7.0, 0.80);
  ego.stamp_sec = 0.3;
  ego.v = 1.0;
  auto target = makeOpponent(frame, 14.0, 0.30);
  target.id = "d2";
  target.stamp_sec = 0.3;
  target.v = 2.0;
  target.vx = 2.0;
  const auto output =
      core.update(0.3, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(output.blocked_info.prestart_attack_follow_hold_lateral);
  EXPECT_TRUE(output.blocked_info.attack_follow_candidate_feasible)
      << output.blocked_info.attack_follow_candidate_reject_reason;
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
  ASSERT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  for (const double lateral_offset_m : output.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_GT(output.speed_caps.back(), target.v);
  EXPECT_FALSE(output.published_lateral_safety_rejected);
}

TEST(OvertakePlannerCore,
     RuntimeD1CurveAndFootprintDistinguishLeftAndRequiredRightEnvelopes) {
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  auto config = makeD1RepresentativeCurveConfig();
  // 実測幾何の右PASSで必要な約2.57 mと、実回廊で選べる短い左PASSを分けて
  // 評価する。runtime既定2.20 mは維持し、右側だけ2.60 mの候補を別に作る。
  config.d_min_m = -1.35;
  config.d_max_m = 1.35;
  config.min_wall_margin_m = 0.50;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.parallel_side_detection_enabled = false;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;

  const auto make_runtime_ego = [&frame](double stamp_sec, double s_m,
                                         double d_m, double speed_mps) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::EgoState ego;
    ego.stamp_sec = stamp_sec;
    ego.x = point.x;
    ego.y = point.y;
    ego.yaw = point.yaw;
    ego.v = speed_mps;
    ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_runtime_target = [&frame](const std::string &id,
                                            double stamp_sec, double s_m,
                                            double d_m, double speed_mps) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::OpponentState target;
    target.id = id;
    target.stamp_sec = stamp_sec;
    target.x = point.x;
    target.y = point.y;
    target.vx = speed_mps * std::cos(point.yaw);
    target.vy = speed_mps * std::sin(point.yaw);
    target.v = speed_mps;
    target.frenet = frame.cartesianToFrenet(target.x, target.y, point.yaw);
    target.valid = true;
    return target;
  };

  const auto make_scene = [&](double now_sec, double elapsed_sec) {
    const auto target = make_runtime_target(
        "d2", now_sec, 38.95 + 2.25 * elapsed_sec, 0.18, 2.25);
    return std::vector<overtake_planner::OpponentState>{target};
  };

  const auto initial_ego = make_runtime_ego(0.05, 32.86, 0.68, 0.19);
  const auto initial_target = make_scene(0.05, 0.0).front();
  overtake_planner::BlockedInfo right_blocked;
  right_blocked.nearest_index = 0;
  right_blocked.gentle_curve_safe_pass_constraint_active = true;
  right_blocked.gentle_curve_safe_pass_anchor_d_m = initial_ego.frenet.d;
  right_blocked.pass_acceleration_allowed = true;

  overtake_planner::CandidateBuilder limited_builder(frame, config);
  const auto limited_right = limited_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, initial_ego, right_blocked,
      {initial_target});
  ASSERT_TRUE(std::isfinite(limited_right.planned_target_d_m));
  EXPECT_FALSE(limited_right.pass_target_corridor_valid);
  EXPECT_LE(std::abs(limited_right.planned_target_d_m - initial_ego.frenet.d),
            2.20 + 1.0e-9);

  auto wide_config = config;
  wide_config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  overtake_planner::CandidateBuilder wide_builder(frame, wide_config);
  overtake_planner::SafetyEvaluator wide_evaluator(frame, wide_config);
  auto wide_right =
      wide_builder.makeCandidate(overtake_planner::CandidateType::PASS_RIGHT,
                                 initial_ego, right_blocked, {initial_target});
  ASSERT_TRUE(std::isfinite(wide_right.planned_target_d_m));
  const double required_right_displacement_m =
      std::abs(wide_right.planned_target_d_m - initial_ego.frenet.d);
  EXPECT_GT(required_right_displacement_m, 2.20);
  EXPECT_LE(required_right_displacement_m, 2.60 + 1.0e-9);
  EXPECT_TRUE(wide_right.pass_target_corridor_valid);
  EXPECT_TRUE(wide_right.desired_path_trackable)
      << "v_ref_max="
      << *std::max_element(wide_right.v_ref.begin(), wide_right.v_ref.end())
      << " predicted_max="
      << *std::max_element(wide_right.predicted_speed_mps.begin(),
                           wide_right.predicted_speed_mps.end())
      << " endpoint_arc=" << wide_right.longitudinal_offsets_m.back()
      << " required_arc=" << wide_right.required_controller_spatial_horizon_m;
  EXPECT_TRUE(wide_right.pure_pursuit_command_trackable);
  EXPECT_TRUE(wide_right.controller_tracking_profile_valid);
  overtake_planner::PredictedOpponent target_prediction;
  target_prediction.id = initial_target.id;
  for (const double t_sec : wide_right.t) {
    const double target_s_m =
        frame.wrapS(initial_target.frenet.s + initial_target.v * t_sec);
    const auto point =
        frame.frenetToCartesian(target_s_m, initial_target.frenet.d);
    target_prediction.t.push_back(t_sec);
    target_prediction.x.push_back(point.x);
    target_prediction.y.push_back(point.y);
    target_prediction.s.push_back(target_s_m);
    target_prediction.d.push_back(initial_target.frenet.d);
  }
  EXPECT_TRUE(wide_evaluator.evaluate(wide_right, {target_prediction}))
      << wide_right.reject_reason;

  // 同じ実CSV・同じ回廊・同じPASSを、意図的に実行不能な操舵速度契約へ
  // 落とした場合はSafetyEvaluatorより手前で許可しない。回廊不成立との
  // 取り違えを防ぎ、controller契約だけがnegative要因であることも固定する。
  auto low_rate_config = wide_config;
  low_rate_config.attack_follow_max_steering_rate_radps = 1.0e-3;
  overtake_planner::CandidateBuilder low_rate_builder(frame, low_rate_config);
  overtake_planner::SafetyEvaluator low_rate_evaluator(frame, low_rate_config);
  overtake_planner::LocalizedLateralProfile low_rate_profile;
  low_rate_profile.active = true;
  low_rate_profile.pass_type = overtake_planner::CandidateType::PASS_RIGHT;
  low_rate_profile.target_id = initial_target.id;
  low_rate_profile.anchor_s_m = initial_ego.frenet.s;
  low_rate_profile.target_s_m = initial_target.frenet.s;
  low_rate_profile.avoid_start_s_m = initial_ego.frenet.s;
  low_rate_profile.full_offset_start_s_m = initial_target.frenet.s - 2.0;
  low_rate_profile.full_offset_end_s_m = initial_target.frenet.s + 2.0;
  low_rate_profile.merge_end_s_m =
      low_rate_profile.full_offset_end_s_m +
      low_rate_config.localized_avoidance_merge_distance_m;
  low_rate_profile.start_d_m = initial_ego.frenet.d;
  low_rate_profile.target_d_m = wide_right.planned_target_d_m;
  auto low_rate_right = low_rate_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, initial_ego, right_blocked,
      {initial_target}, &low_rate_profile, true);
  EXPECT_TRUE(low_rate_right.pass_target_corridor_valid);
  EXPECT_FALSE(low_rate_right.controller_tracking_profile_valid);
  EXPECT_FALSE(low_rate_evaluator.evaluate(low_rate_right, {}));
  EXPECT_EQ(low_rate_right.reject_reason, "untrackable_lateral_profile");

  // Gate 2認可待ちの攻めFOLLOWは、実CSV上でも現在dを変えず、active PPの
  // 操舵契約とSafetyEvaluatorの両方を通ることを確認する。
  overtake_planner::BlockedInfo current_d_blocked;
  current_d_blocked.blocked = true;
  current_d_blocked.nearest_index = 0;
  current_d_blocked.nearest_id = initial_target.id;
  current_d_blocked.front_delta_s =
      initial_target.frenet.s - initial_ego.frenet.s;
  current_d_blocked.follow_gap_closing_allowed = true;
  current_d_blocked.prestart_attack_follow_hold_lateral = true;
  auto current_d_follow = wide_builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, initial_ego, current_d_blocked,
      {initial_target});
  ASSERT_FALSE(current_d_follow.d.empty());
  EXPECT_TRUE(std::all_of(
      current_d_follow.d.begin(), current_d_follow.d.end(), [&](double d_m) {
        return std::abs(d_m - initial_ego.frenet.d) <= 1.0e-6;
      }));
  EXPECT_TRUE(current_d_follow.controller_tracking_profile_valid);
  EXPECT_TRUE(wide_evaluator.evaluate(current_d_follow, {}))
      << current_d_follow.reject_reason;

  overtake_planner::OvertakePlannerCore core(frame, config);

  overtake_planner::PlannerOutput output;
  for (int cycle = 1; cycle <= 5; ++cycle) {
    constexpr double cycle_dt_sec = 0.05;
    const double elapsed_sec = cycle_dt_sec * static_cast<double>(cycle - 1);
    const double now_sec = cycle_dt_sec * static_cast<double>(cycle);
    const auto ego =
        make_runtime_ego(now_sec, 32.86 + 0.19 * elapsed_sec, 0.68, 0.19);
    output =
        core.update(now_sec, ego, make_scene(now_sec, elapsed_sec),
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());

    ASSERT_TRUE(output.blocked_info.gentle_curve_target_identity_valid)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_lateral_capacity_sufficient)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_safe_pass_eligible)
        << "cycle=" << cycle << " reason="
        << output.blocked_info.gentle_curve_safe_pass_block_reason;
    ASSERT_TRUE(output.blocked_info.gentle_curve_safe_pass_found)
        << "cycle=" << cycle;
    ASSERT_TRUE(output.blocked_info.gentle_curve_safe_pass_start_approved)
        << "cycle=" << cycle;
    EXPECT_EQ(output.blocked_info.gentle_curve_safe_pass_side,
              overtake_planner::CandidateType::PASS_LEFT)
        << "cycle=" << cycle << " ego_d=" << ego.frenet.d
        << " left_target=" << output.blocked_info.pass_left_candidate_target_d_m
        << " left_reject="
        << output.blocked_info.pass_left_candidate_reject_reason
        << " right_target="
        << output.blocked_info.pass_right_candidate_target_d_m
        << " right_reject="
        << output.blocked_info.pass_right_candidate_reject_reason;
    ASSERT_TRUE(
        std::isfinite(output.blocked_info.pass_left_candidate_target_d_m));
    const double planned_displacement_m = std::abs(
        output.blocked_info.pass_left_candidate_target_d_m - ego.frenet.d);
    EXPECT_LE(planned_displacement_m, 2.20 + 1.0e-9) << "cycle=" << cycle;
    EXPECT_EQ(output.blocked_info.pass_left_safe_cycles, cycle);
    if (cycle < 5) {
      EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
      EXPECT_NE(output.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
      EXPECT_FALSE(output.blocked_info.maneuver_transaction_incomplete);
    }
  }

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(output.active_override);
  EXPECT_LE(output.cbf_slack, 1.0e-9);
}

TEST(OvertakePlannerCore,
     LatestDev3D1WallOnsetReportsFirstFalseAndEvaluatesRecovery) {
  // 20260727-151855 D1の最初のwall_footprint_margin周期を固定する。
  // 実運用閾値のままでは速度と曲率が同時に未達だが、既存predicate順では
  // speedが最初のfalseである。診断追加だけでauthorityが変わらないことと、
  // 同じ実配置でtrigger成立を仮定したRECOVERYが全安全評価を通るかを分離する。
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.25;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.moving_lateral_override_max_evaluation_horizon_sec = 6.0;
  config.attack_follow_max_steering_angle_rad = 0.64;
  config.attack_follow_max_steering_rate_radps = 128.0;
  config.attack_follow_steering_tire_angle_gain = 1.54;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.recovery_v_max_mps = 3.0;
  config.merge_distance_m = 12.0;
  config.prepare_distance_m = 8.0;
  config.high_speed_curve_lateral_hold_enabled = true;
  config.high_speed_curve_lateral_hold_min_speed_mps = 4.0;
  config.preemptive_wall_recovery_min_curvature_m_inv = 0.50;

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 75.114998321;
  ego.x = 89623.67087211026;
  ego.y = 43165.25268290918;
  ego.yaw = -0.126675;
  ego.v = 3.82561142265;
  // Nodeの同周期診断値をそのまま固定する。kinematic_stateの実poseを
  // offline再投影するとs=77.85,d=1.19になるが、Plannerへ渡ったFrenet値は
  // s=78.40,d=1.44だった。この差もactual-pose始端の契約対象に含める。
  ego.frenet.s = 78.40;
  ego.frenet.d = 1.44;

  const auto make_bag_opponent = [&](const char *id, double s_m, double d_m,
                                     double speed_mps) {
    overtake_planner::OpponentState opponent;
    const auto pose = frame.frenetToCartesian(s_m, d_m);
    opponent.id = id;
    opponent.stamp_sec = ego.stamp_sec;
    opponent.x = pose.x;
    opponent.y = pose.y;
    opponent.vx = speed_mps * std::cos(pose.yaw);
    opponent.vy = speed_mps * std::sin(pose.yaw);
    opponent.v = speed_mps;
    opponent.frenet = frame.cartesianToFrenet(opponent.x, opponent.y, pose.yaw);
    opponent.valid = true;
    return opponent;
  };
  // 同stamp近傍の各planner実測。D2/D3はD1から十分前方だが、RECOVERYの
  // 全相手評価から省略しない。
  const std::vector<overtake_planner::OpponentState> opponents{
      make_bag_opponent("d2", 122.14, -0.99, 5.0),
      make_bag_opponent("d3", 149.31, -0.10, 5.0)};

  overtake_planner::OvertakePlannerCore diagnostic_core(frame, config);
  const auto diagnostic_output = diagnostic_core.update(
      ego.stamp_sec, ego, opponents, overtake_planner::MpcHealthStatus{},
      readyReentryInput());
  EXPECT_NEAR(ego.frenet.s, 78.40, 0.20);
  EXPECT_NEAR(ego.frenet.d, 1.44, 0.20);
  EXPECT_NEAR(diagnostic_output.blocked_info.ego_wall_clearance_m, 1.10, 0.30);
  EXPECT_EQ(
      diagnostic_output.blocked_info.early_wall_recovery_probe_first_false,
      "ego_speed_below_min");
  EXPECT_FALSE(
      diagnostic_output.blocked_info.early_wall_recovery_probe_requested);

  auto curvature_only_ego = ego;
  curvature_only_ego.v = 4.1;
  overtake_planner::OvertakePlannerCore curvature_core(frame, config);
  const auto curvature_output = curvature_core.update(
      ego.stamp_sec, curvature_only_ego, opponents,
      overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_EQ(curvature_output.blocked_info.early_wall_recovery_probe_first_false,
            "corner_curvature_below_min");
  EXPECT_FALSE(
      curvature_output.blocked_info.early_wall_recovery_probe_requested);

  // runtime閾値を変更せず、fixture内だけ実測値を包含させて同じsnapshotの
  // RECOVERY safety/trackabilityを証明する。
  auto proof_config = config;
  proof_config.high_speed_curve_lateral_hold_min_speed_mps = 3.8;
  proof_config.preemptive_wall_recovery_min_curvature_m_inv = 0.19;
  overtake_planner::OvertakePlannerCore proof_core(frame, proof_config);
  const auto proof_output = proof_core.update(
      ego.stamp_sec, ego, opponents, overtake_planner::MpcHealthStatus{},
      readyReentryInput());
  EXPECT_EQ(proof_output.blocked_info.early_wall_recovery_probe_first_false,
            "none");
  ASSERT_TRUE(proof_output.blocked_info.early_wall_recovery_probe_requested);
  ASSERT_TRUE(proof_output.blocked_info.early_wall_recovery_probe_generated);
  EXPECT_FALSE(proof_output.blocked_info.early_wall_recovery_probe_feasible);
  EXPECT_EQ(proof_output.blocked_info.early_wall_recovery_reject_reason,
            "wall_footprint_margin");
  EXPECT_NE(proof_output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_FALSE(proof_output.active_override);

  // first-falseをtrigger層とcandidate層で混同しない。実pose始端を含む
  // RECOVERYを全相手・壁へ通すと、最初の権威的失敗はwall footprint。
  // PP required arc分の点列自体は生成済みなので、未実行の後段proofでwall
  // rejectを上書きせず、authorityなしに閉じる。
  overtake_planner::BlockedInfo recovery_blocked = proof_output.blocked_info;
  recovery_blocked.early_wall_recovery_probe_requested = true;
  recovery_blocked.opponent_prediction_inputs_complete = true;
  overtake_planner::CandidateBuilder recovery_builder(frame, proof_config);
  auto recovery =
      recovery_builder.makeCandidate(overtake_planner::CandidateType::RECOVERY,
                                     ego, recovery_blocked, opponents);
  std::vector<overtake_planner::PredictedOpponent> recovery_predictions;
  for (const auto &opponent : opponents) {
    overtake_planner::PredictedOpponent prediction;
    prediction.id = opponent.id;
    for (const double time_sec : recovery.t) {
      const double x_m = opponent.x + opponent.vx * time_sec;
      const double y_m = opponent.y + opponent.vy * time_sec;
      const auto frenet = frame.cartesianToFrenet(x_m, y_m, 0.0);
      prediction.t.push_back(time_sec);
      prediction.x.push_back(x_m);
      prediction.y.push_back(y_m);
      prediction.s.push_back(frenet.s);
      prediction.d.push_back(frenet.d);
    }
    recovery_predictions.push_back(std::move(prediction));
  }
  overtake_planner::SafetyEvaluator recovery_evaluator(frame, proof_config);
  EXPECT_FALSE(recovery_evaluator.evaluate(recovery, recovery_predictions));
  EXPECT_EQ(recovery.reject_reason, "wall_footprint_margin");
  EXPECT_TRUE(recovery.blocking_wall_footprint_valid);
  EXPECT_GT(recovery.blocking_wall_time_sec, 0.0);
  EXPECT_LT(recovery.blocking_wall_time_sec, 0.10);
  ASSERT_FALSE(recovery.longitudinal_offsets_m.empty());
  EXPECT_FALSE(recovery.controller_spatial_horizon_proof_valid);
  EXPECT_GE(recovery.longitudinal_offsets_m.back() + 1.0e-9,
            recovery.required_controller_spatial_horizon_m);
}

TEST(CandidateBuilder,
     LatestDev3D3StoppedD1ReplaysLeftRejectAndActualStartRightFeasibility) {
  // 20260727-151855 D3終盤: 停止D1までds=7.60 mの周期を、D1 wall
  // fixtureとは別Core/別candidateとして固定する。左右の最初の不成立を
  // 混ぜず、CandidateBuilder所有のcorridor/deadline/trackabilityへ閉じる。
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.25;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.left_offset_m = 2.10;
  config.right_offset_m = -2.10;
  config.pass_target_policy = "minimum_clearance";
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.prepare_distance_m = 8.0;
  config.pass_speed_cap_mps = 10.0;
  config.pass_target_lateral_margin_m = 0.10;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 1.80;
  config.min_ellipse_h = 0.20;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.attack_follow_max_steering_angle_rad = 0.64;
  config.attack_follow_max_steering_rate_radps = 128.0;
  config.attack_follow_steering_tire_angle_gain = 1.54;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.moving_lateral_override_max_evaluation_horizon_sec = 6.0;

  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 114.794997434;
  ego.x = 89616.68869792321;
  ego.y = 43165.478376463616;
  ego.yaw = 0.464849;
  ego.v = 7.32848653914;
  ego.frenet.s = 72.61;
  ego.frenet.d = 0.73;

  overtake_planner::OpponentState stopped_d1;
  stopped_d1.id = "d1";
  stopped_d1.valid = true;
  stopped_d1.stamp_sec = ego.stamp_sec;
  stopped_d1.x = 89626.51795122148;
  stopped_d1.y = 43163.89283765542;
  stopped_d1.vx = 0.0;
  stopped_d1.vy = 0.0;
  stopped_d1.v = 0.0;
  stopped_d1.frenet.s = ego.frenet.s + 7.60;
  stopped_d1.frenet.d = ego.frenet.d + 1.12;

  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;
  blocked.nearest_id = stopped_d1.id;
  blocked.blocked = true;
  blocked.front_delta_s = 7.60;
  blocked.front_delta_d = 1.12;
  blocked.front_vehicle_speed_mps = 0.0;
  blocked.front_vehicle_low_speed = true;
  blocked.stationary_front_obstacle = true;
  blocked.pass_acceleration_allowed = true;
  blocked.corner_abs_curvature = 0.178;
  blocked.opponent_prediction_inputs_complete = true;

  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);
  const double lateral_clearance_m =
      config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
      config.pass_target_lateral_margin_m;
  const double longitudinal_clearance_m =
      config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);
  const auto make_profile = [&](overtake_planner::CandidateType pass_type,
                                double target_d_m) {
    overtake_planner::LocalizedLateralProfile profile;
    profile.active = true;
    profile.pass_type = pass_type;
    profile.target_id = stopped_d1.id;
    profile.anchor_s_m = ego.frenet.s;
    profile.target_s_m = stopped_d1.frenet.s;
    profile.avoid_start_s_m = ego.frenet.s;
    profile.full_offset_start_s_m =
        stopped_d1.frenet.s - longitudinal_clearance_m;
    profile.full_offset_end_s_m = stopped_d1.frenet.s + 2.0;
    profile.merge_end_s_m = stopped_d1.frenet.s + 8.0;
    profile.start_d_m = ego.frenet.d;
    profile.target_d_m = target_d_m;
    return profile;
  };
  auto left_profile = make_profile(overtake_planner::CandidateType::PASS_LEFT,
                                   stopped_d1.frenet.d + lateral_clearance_m);
  auto right_profile = make_profile(overtake_planner::CandidateType::PASS_RIGHT,
                                    stopped_d1.frenet.d - lateral_clearance_m);
  auto left = builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT,
                                    ego, blocked, {stopped_d1}, &left_profile);
  auto right =
      builder.makeCandidate(overtake_planner::CandidateType::PASS_RIGHT, ego,
                            blocked, {stopped_d1}, &right_profile);

  const auto make_static_prediction =
      [&](const overtake_planner::CandidateTrajectory &candidate) {
        overtake_planner::PredictedOpponent prediction;
        prediction.id = stopped_d1.id;
        for (const double time_sec : candidate.t) {
          prediction.t.push_back(time_sec);
          prediction.x.push_back(stopped_d1.x);
          prediction.y.push_back(stopped_d1.y);
          prediction.s.push_back(stopped_d1.frenet.s);
          prediction.d.push_back(stopped_d1.frenet.d);
        }
        return prediction;
      };
  EXPECT_FALSE(evaluator.evaluate(left, {make_static_prediction(left)}));
  EXPECT_EQ(left.reject_reason, "pass_target_unreachable");
  // 同じ観測snapshotから実測始端で新規生成した右候補は成立する。runtimeの
  // untrackable_lateral_profileは配置固有の不可避な幾何ではなく、保持中profile
  // のmarker/provenanceに原因が残る。値を推測してfixtureを捏造しない。
  EXPECT_TRUE(evaluator.evaluate(right, {make_static_prediction(right)}))
      << right.reject_reason;
}

TEST(OvertakePlannerCore,
     LatestDev3D3WallRejectEvaluatesActualPoseRecoveryBeforeClearanceIsLost) {
  // 20260727-135724 D3: large_lateral_error直前の最初の
  // wall_footprint_margin周期を固定する。ここではまだFrenet上のwall
  // clearanceが正なので、FASTESTの横列を失ってspeed-onlyで外へ進む前に、
  // actual pose始端のRECOVERYを同じSafetyEvaluatorへ通す。
  overtake_planner::FrenetFrame frame;
  const std::string source_dir = OVERTAKE_PLANNER_SOURCE_DIR;
  std::string error;
  ASSERT_TRUE(frame.loadCsv(
      source_dir +
          "/../multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv",
      &error))
      << error;
  ASSERT_TRUE(frame.loadCorridorCsv(
      source_dir + "/config/final_ver3_drivable_corridor.csv", &error))
      << error;

  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.25;
  config.wall_footprint_check_enabled = true;
  config.ego_front_extent_m = 1.554;
  config.ego_rear_extent_m = 0.510;
  config.ego_half_width_m = 0.650;
  config.wall_localization_uncertainty_m = 0.250;
  config.wall_footprint_max_sample_distance_m = 0.250;
  config.wall_footprint_max_sample_yaw_rad = 0.050;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  config.moving_lateral_override_max_evaluation_horizon_sec = 8.0;
  config.attack_follow_max_steering_angle_rad = 0.64;
  config.attack_follow_max_steering_rate_radps = 128.0;
  config.attack_follow_steering_tire_angle_gain = 1.54;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.recovery_v_max_mps = 3.0;
  config.merge_distance_m = 12.0;
  config.prepare_distance_m = 8.0;
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.stamp_sec = 0.1;
  ego.v = 7.8;
  ego.frenet.s = 222.99;
  ego.frenet.d = 3.24;
  const auto pose = frame.frenetToCartesian(ego.frenet.s, ego.frenet.d);
  // referenceから再生成した始端でも通るfixtureにしない。bagの実姿勢と
  // Frenet再投影の微小差を模擬し、CandidateBuilderがego実測始端を保持する
  // ことを直接証明する。
  ego.x = pose.x + 0.005;
  ego.y = pose.y - 0.004;
  ego.yaw = pose.yaw + 0.001;

  overtake_planner::BlockedInfo recovery_blocked;
  recovery_blocked.early_wall_recovery_probe_requested = true;
  recovery_blocked.ego_wall_clearance_m =
      frame.corridorBounds(ego.frenet.s, config.d_min_m, config.d_max_m).d_max -
      ego.frenet.d;
  overtake_planner::CandidateBuilder recovery_builder(frame, config);
  auto recovery = recovery_builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, recovery_blocked, {});
  overtake_planner::SafetyEvaluator recovery_evaluator(frame, config);
  ASSERT_TRUE(recovery_evaluator.evaluate(recovery, {}))
      << recovery.reject_reason << " endpoint="
      << (recovery.longitudinal_offsets_m.empty()
              ? -1.0
              : recovery.longitudinal_offsets_m.back())
      << " required=" << recovery.required_controller_spatial_horizon_m;
  ASSERT_TRUE(recovery.controller_tracking_profile_valid);
  ASSERT_FALSE(recovery.x.empty());
  ASSERT_FALSE(recovery.y.empty());
  ASSERT_FALSE(recovery.yaw.empty());
  EXPECT_NEAR(recovery.x.front(), ego.x, 1.0e-9);
  EXPECT_NEAR(recovery.y.front(), ego.y, 1.0e-9);
  EXPECT_NEAR(recovery.yaw.front(), ego.yaw, 1.0e-9);
  ASSERT_FALSE(recovery.longitudinal_offsets_m.empty());
  EXPECT_GE(recovery.longitudinal_offsets_m.back(),
            recovery.required_controller_spatial_horizon_m);

  auto runtime_config = config;
  runtime_config.moving_lateral_override_max_evaluation_horizon_sec = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, runtime_config);
  const auto output = core.update(
      0.1, ego, {}, overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_GE(output.blocked_info.ego_wall_clearance_m, 0.0);
  ASSERT_LE(output.blocked_info.ego_wall_clearance_m, 2.44);
  ASSERT_TRUE(output.blocked_info.early_wall_recovery_probe_requested);
  ASSERT_TRUE(output.blocked_info.early_wall_recovery_probe_generated);
  ASSERT_TRUE(output.blocked_info.early_wall_recovery_probe_feasible)
      << output.blocked_info.early_wall_recovery_reject_reason;
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY)
      << output.reason;
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
  EXPECT_TRUE(output.controller_spatial_horizon_proof_valid);
  EXPECT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  ASSERT_FALSE(output.longitudinal_offsets_m.empty());
  EXPECT_NEAR(output.lateral_offsets.front(), ego.frenet.d, 1.0e-6);
  EXPECT_LT(output.lateral_offsets.back(), ego.frenet.d);
  EXPECT_GE(output.longitudinal_offsets_m.back(),
            config.lateral_override_lookahead_min_distance_m);
  EXPECT_FALSE(output.published_lateral_safety_rejected);

  // moving相手または速度成分が不完全な周期は未校正の予測を8秒へ延ばさない。
  // 既存6秒内でrequired arcを証明できないRECOVERYは横authorityを得ない。
  const auto expect_moving_or_invalid_opponent_fails_closed =
      [&](double v_mps, double vx_mps, double vy_mps) {
        auto opponent = makeOpponent(frame, ego.frenet.s + 100.0, 0.0);
        opponent.id = "d1";
        opponent.stamp_sec = ego.stamp_sec;
        opponent.v = v_mps;
        opponent.vx = vx_mps;
        opponent.vy = vy_mps;
        overtake_planner::OvertakePlannerCore moving_opponent_core(
            frame, runtime_config);
        const auto moving_output = moving_opponent_core.update(
            0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
            readyReentryInput());
        EXPECT_TRUE(
            moving_output.blocked_info.early_wall_recovery_probe_requested);
        EXPECT_TRUE(
            moving_output.blocked_info.early_wall_recovery_probe_generated);
        EXPECT_FALSE(
            moving_output.blocked_info.early_wall_recovery_probe_feasible);
        EXPECT_NE(moving_output.selected,
                  overtake_planner::CandidateType::RECOVERY);
        EXPECT_FALSE(moving_output.active_override);
        EXPECT_FALSE(moving_output.lateral_tracking_authorized_during_stop);
      };
  const double moving_speed_mps =
      runtime_config.stationary_obstacle_speed_threshold_mps + 1.0;
  expect_moving_or_invalid_opponent_fails_closed(0.0, moving_speed_mps, 0.0);
  expect_moving_or_invalid_opponent_fails_closed(0.0, 0.0, moving_speed_mps);
  expect_moving_or_invalid_opponent_fails_closed(
      0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
}

TEST(OvertakePlannerCore,
     D1RepresentativeCurvePassFailsClosedWhenTrackingIsUnusable) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  config.follow_gap_closing_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 32.86, 0.68);
  ego.stamp_sec = 0.1;
  ego.v = 0.19;
  auto target = makeOpponent(frame, 38.95, 0.18);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 2.25;
  target.vx = 2.25;
  auto input = readyReentryInput();
  input.pure_pursuit_primary_and_fresh = false;

  const auto output = core.update(0.1, ego, {target},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.gentle_curve_tracking_usable);
  EXPECT_FALSE(output.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_FALSE(output.blocked_info.follow_gap_closing_allowed);
  EXPECT_FALSE(output.blocked_info.prestart_attack_follow_hold_lateral);
  EXPECT_FALSE(output.blocked_info.attack_follow_candidate_generated);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.gentle_curve_safe_pass_block_reason,
            "tracking_unusable");
  EXPECT_EQ(output.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     D1RepresentativeCurveHardMpcFailureResetsPrestartPassDebounce) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto update_cycle = [&](int cycle,
                                overtake_planner::ReentryInputStatus input) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    auto ego = makeEgo(frame, 32.86, 0.68);
    ego.stamp_sec = now_sec;
    ego.v = 0.19;
    auto target = makeOpponent(frame, 38.95, 0.18);
    target.id = "d2";
    target.stamp_sec = now_sec;
    target.v = 2.25;
    target.vx = 2.25;
    return core.update(now_sec, ego, {target},
                       overtake_planner::MpcHealthStatus{}, input);
  };

  for (int cycle = 1; cycle <= 4; ++cycle) {
    const auto output = update_cycle(cycle, readyReentryInput());
    ASSERT_EQ(output.blocked_info.pass_right_safe_cycles, cycle);
    ASSERT_TRUE(output.blocked_info.prestart_attack_follow_hold_lateral);
    ASSERT_TRUE(output.blocked_info.attack_follow_candidate_feasible);
  }

  auto hard_failure_input = readyReentryInput();
  hard_failure_input.mpc_healthy = false;
  hard_failure_input.mpc_hard_failure = true;
  const auto hard_failure = update_cycle(5, hard_failure_input);
  EXPECT_EQ(hard_failure.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_FALSE(hard_failure.blocked_info.follow_gap_closing_allowed);
  EXPECT_FALSE(hard_failure.blocked_info.prestart_attack_follow_hold_lateral);
  EXPECT_FALSE(hard_failure.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_NE(hard_failure.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(hard_failure.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_TRUE(hard_failure.active_override ||
              hard_failure.speed_only_fallback_active ||
              hard_failure.safe_stop_triggered);
  if (hard_failure.active_override) {
    ASSERT_FALSE(hard_failure.lateral_offsets.empty());
    for (const double lateral_offset_m : hard_failure.lateral_offsets) {
      EXPECT_NEAR(lateral_offset_m,
                  hard_failure.blocked_info.ego_lateral_offset_m, 1.0e-3);
    }
    EXPECT_TRUE(hard_failure.selected_lateral_profile_safety_verified);
  } else {
    EXPECT_TRUE(hard_failure.longitudinal_speed_cap_active);
  }
  ASSERT_FALSE(hard_failure.speed_caps.empty());
  for (const double speed_cap_mps : hard_failure.speed_caps) {
    EXPECT_LE(speed_cap_mps, 0.19 + 1.0e-9);
  }

  overtake_planner::PlannerOutput recovered;
  for (int cycle = 6; cycle <= 10; ++cycle) {
    recovered = update_cycle(cycle, readyReentryInput());
    const int fresh_safe_cycle = cycle - 5;
    EXPECT_EQ(recovered.blocked_info.pass_right_safe_cycles, fresh_safe_cycle);
    if (cycle < 10) {
      EXPECT_EQ(recovered.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
    }
  }
  EXPECT_EQ(recovered.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     D1PrestartAttackFollowClearsOnlyGenericSpeedGuardWithoutAbort) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.min_mode_hold_time_sec = 0.0;
  config.slow_front_exception_speed_mps = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 32.86, 0.68);
  ego.stamp_sec = 0.1;
  ego.v = 4.0;
  auto target = makeOpponent(frame, 38.95, 0.18);
  target.id = "d2";
  target.stamp_sec = 0.1;
  target.v = 0.0;
  target.vx = 0.0;
  auto hard_failure_input = readyReentryInput();
  hard_failure_input.mpc_healthy = false;
  hard_failure_input.mpc_hard_failure = true;
  const auto guarded =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  hard_failure_input);

  ASSERT_TRUE(guarded.generic_recovery_after_update)
      << "mode=" << static_cast<int>(guarded.mode)
      << " selected=" << static_cast<int>(guarded.selected)
      << " raw=" << static_cast<int>(guarded.raw_selected)
      << " raw_feasible=" << guarded.raw_selected_feasible
      << " raw_reject=" << guarded.raw_selected_reject_reason;
  EXPECT_EQ(guarded.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_NE(guarded.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(guarded.blocked_info.reentry_hold_active);

  ego.stamp_sec = 0.2;
  ego.v = 0.19;
  target.stamp_sec = 0.2;
  // 停止判定は解除するがslow-front確認はまだ2/3周期に留める。これにより
  // generic recovery要求自体は残しつつ、通常moving frontのcurrent-d FOLLOWを
  // 同周期に独立評価し、generic phaseだけをpreemptする分岐を通す。
  target.v = 2.25;
  target.vx = 2.25;
  const auto follow =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(follow.generic_recovery_before_update);
  EXPECT_EQ(follow.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NE(follow.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(follow.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_FALSE(follow.blocked_info.reentry_hold_active);
  EXPECT_TRUE(follow.blocked_info.prestart_attack_follow_hold_lateral);
  EXPECT_TRUE(follow.blocked_info.attack_follow_candidate_feasible)
      << follow.blocked_info.attack_follow_candidate_reject_reason;
  // generic phaseを解除した同周期は、以前のreentry gate診断をsafe-cycleへ
  // 流用しない。次のfresh周期から改めてGate 2 debounceを開始する。
  EXPECT_EQ(follow.blocked_info.pass_right_safe_cycles, 0);
  ASSERT_TRUE(follow.active_override);
  ASSERT_FALSE(follow.lateral_offsets.empty());
  for (const double lateral_offset_m : follow.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-6);
  }
  ASSERT_FALSE(follow.speed_caps.empty());
  EXPECT_GT(follow.speed_caps.back(), target.v);

  ego.stamp_sec = 0.3;
  target.stamp_sec = 0.3;
  target.v = 3.20;
  target.vx = 3.20;
  const auto fresh =
      core.update(0.3, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_FALSE(fresh.generic_recovery_before_update);
  EXPECT_EQ(fresh.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_TRUE(fresh.blocked_info.prestart_attack_follow_hold_lateral);
  EXPECT_TRUE(fresh.blocked_info.attack_follow_candidate_feasible)
      << fresh.blocked_info.attack_follow_candidate_reject_reason;
  EXPECT_EQ(fresh.blocked_info.pass_right_safe_cycles, 1);

  overtake_planner::PlannerOutput recovered = fresh;
  for (int cycle = 4; cycle <= 7; ++cycle) {
    const double now_sec = 0.1 * static_cast<double>(cycle);
    ego.stamp_sec = now_sec;
    target.stamp_sec = now_sec;
    recovered =
        core.update(now_sec, ego, {target}, overtake_planner::MpcHealthStatus{},
                    readyReentryInput());
    const int safe_cycle = cycle - 2;
    EXPECT_EQ(recovered.blocked_info.pass_right_safe_cycles, safe_cycle);
    EXPECT_EQ(recovered.blocked_info.maneuver_target_id, target.id);
    if (cycle < 7) {
      EXPECT_EQ(recovered.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
      EXPECT_TRUE(recovered.blocked_info.prestart_attack_follow_hold_lateral);
      EXPECT_TRUE(recovered.blocked_info.attack_follow_candidate_feasible)
          << recovered.blocked_info.attack_follow_candidate_reject_reason;
    }
  }
  EXPECT_EQ(recovered.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_EQ(recovered.selected, overtake_planner::CandidateType::PASS_RIGHT);
}

TEST(OvertakePlannerCore,
     AttackFollowClosesGapFromTargetDistanceInsteadOfWaitingForTenMeters) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  // この曲率では新規PASS開始gateを閉じ、PASS開始不能時に選ばれるFOLLOW
  // そのものを検証する。単なるdebug値の計算だけでは、実際の攻め追従契約を
  // 満たしたことにならない。
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 12.0;
  config.gentle_curve_safe_pass_enabled = false;
  config.future_side_prediction_enabled = false;
  config.parallel_side_detection_enabled = false;
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_engage_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.80;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.19;
  // D1実測相当の約0.5 m横差を持たせ、左右PASS候補は存在するが曲率開始gate
  // だけが閉じている状態にする。
  auto target = makeOpponent(frame, 11.09, -0.6);
  target.v = 2.25;
  target.vx = 2.25;

  const auto output =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_TRUE(output.blocked_info.follow_gap_closing_allowed);
  EXPECT_GT(output.blocked_info.follow_candidate_speed_cap_mps, target.v);
  EXPECT_GT(output.blocked_info.follow_candidate_terminal_speed_mps, target.v);
  EXPECT_LE(output.blocked_info.follow_candidate_speed_cap_mps,
            target.v + config.follow_gap_closing_max_speed_bonus_mps + 1.0e-9);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED)
      << "blocked=" << output.blocked_info.blocked
      << " side=" << output.blocked_info.side_by_side
      << " corner_side=" << output.blocked_info.corner_side_by_side
      << " future_side=" << output.blocked_info.future_side_by_side
      << " future_corner=" << output.blocked_info.future_corner_side_by_side
      << " future_yield=" << output.blocked_info.future_yield_required
      << " parallel=" << output.blocked_info.parallel_side_candidate
      << " reason=" << output.reason;
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.selected_lateral_profile_safety_verified);
}

TEST(OvertakePlannerCore, FollowGapClosingUsesAccelerationAwarePrediction) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.follow_gap_closing_enabled = true;
  config.follow_gap_closing_target_gap_m = 5.0;
  config.follow_gap_closing_engage_gap_m = 6.0;
  config.follow_gap_closing_speed_gain_per_m = 0.10;
  config.follow_gap_closing_max_speed_bonus_mps = 0.30;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 3.0;
  auto opponent = makeOpponent(frame, 13.0, 0.0);
  opponent.vx = 4.0;
  opponent.v = 4.0;
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.follow_gap_closing_allowed);

  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::BlockedInfo blocked = output.blocked_info;
  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {opponent});
  ASSERT_FALSE(candidate.v_ref.empty());
  ASSERT_GT(candidate.predicted_speed_mps.size(), 1U);
  EXPECT_NEAR(candidate.v_ref.front(),
              opponent.v + config.follow_gap_closing_max_speed_bonus_mps,
              1.0e-9);
  EXPECT_GT(candidate.predicted_speed_mps[1], ego.v);
}

TEST(OvertakePlannerCore, FollowGapClosingFailsClosedForStaleInput) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.follow_gap_closing_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 3.0;
  auto opponent = makeOpponent(frame, 13.0, 0.0);
  opponent.vx = 4.0;
  opponent.v = 4.0;
  auto stale_input = readyReentryInput();
  stale_input.all_observed_opponents_fresh = false;
  const auto output = core.update(
      0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{}, stale_input);

  EXPECT_FALSE(output.blocked_info.follow_gap_closing_allowed);
}

TEST(OvertakePlannerCore, FollowGapClosingFailsClosedForUnknownDirection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.follow_gap_closing_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 3.0;
  auto opponent = makeOpponent(frame, 13.0, 0.0);
  opponent.vx = 0.0;
  opponent.v = 4.0;
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.front_direction_known);
  EXPECT_FALSE(output.blocked_info.follow_gap_closing_allowed);
}

TEST(OvertakePlannerCore,
     GentleCurveSafePassKeepsSingleFreshMpcLatencyAsSpeedCapOnly) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.mpc_health_solve_time_warn_ms = 50.0;
  config.mpc_health_v_max_mps = 3.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vx = 2.0;
  opponent.v = 2.0;
  auto input = readyReentryInput();
  input.mpc_healthy = false;
  input.mpc_health_fresh = true;
  input.mpc_latency_warning = true;
  input.mpc_health_sample_sequence = 1U;
  overtake_planner::MpcHealthStatus health;
  health.valid = true;
  health.solve_time_ms = 80.0;
  health.age_sec = 0.1;
  health.sample_sequence = 1U;

  const auto output = core.update(0.1, ego, {opponent}, health, input);

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(output.active_override);
  EXPECT_TRUE(output.mpc_health_speed_guard_active);
  EXPECT_EQ(output.speed_cap_reason, "mpc_health_solve_time_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  for (const double v_ref : output.speed_caps) {
    EXPECT_LE(v_ref, config.mpc_health_v_max_mps + 1.0e-9);
  }
}

TEST(OvertakePlannerCore, GentleCurveSafePassDoesNotBypassHighCurvatureGate) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.090;
  config.gentle_curve_safe_pass_v_max_mps = 10.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.20;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vx = 2.0;
  opponent.v = 2.0;

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(OvertakePlannerCore,
     GentleCurveSafePassDoesNotOpenWhenBothConstrainedPassesAreUnsafe) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.dynamic_pass_candidate_enabled = true;
  config.d_min_m = -0.80;
  config.d_max_m = 0.80;
  config.min_wall_margin_m = 0.50;
  config.safety_ellipse_b_m = 0.90;
  config.min_pass_gap_m = 0.10;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 8.0, -0.60);
  opponent.vx = 2.0;
  opponent.v = 2.0;

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
}

TEST(OvertakePlannerCore, GentleCurveSafePassDoesNotOpenForFutureYield) {
  const auto frame = makeFutureCornerFrame();
  auto config = makeConfig();
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.gentle_curve_safe_pass_enabled = true;
  config.gentle_curve_safe_pass_max_curvature_m_inv = 0.040;
  config.gentle_curve_safe_pass_v_max_mps = 5.0;
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 0.70;
  config.gentle_curve_safe_pass_max_lateral_accel_mps2 = 4.0;
  config.corner_side_yield_lookahead_m = 0.2;
  config.future_side_prediction_horizon_sec = 1.2;
  config.future_side_prediction_dt_sec = 0.3;
  config.future_side_yield_wall_clearance_m = 0.35;
  config.large_lateral_error_threshold_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.9);
  const auto opponent = makeOpponent(frame, 5.1, 0.0);
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(output.blocked_info.future_yield_required);
  EXPECT_FALSE(output.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason, "curve");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);
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
  EXPECT_FALSE(output.blocked_info.parallel_follow_candidate);
  EXPECT_FALSE(output.blocked_info.slow_obstacle_chain_active);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.nearest_id.empty());
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     EnabledParallelFollowKeepsCurrentLateralOffsetOutsideSameCorridor) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.4);
  const auto opponent = makeOpponent(frame, 9.0, -0.55);

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_FALSE(output.blocked_info.side_by_side);
  EXPECT_TRUE(output.blocked_info.parallel_follow_candidate);
  EXPECT_TRUE(output.blocked_info.parallel_follow_feasible);
  EXPECT_EQ(output.blocked_info.parallel_follow_id, "npc");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_generated);
  ASSERT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-9);
  for (const double lateral_offset_m : output.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-9);
  }
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(),
              opponent.v - config.follow_speed_margin_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     Dev3D1ArbitrationUsesNearestOpponentInsideParallelFollowBand) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 12.0;
  config.same_corridor_width_m = 0.90;
  config.parallel_follow_lateral_width_m = 1.20;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto d2_outside_follow_band = makeOpponent(frame, 6.11, -3.58);
  d2_outside_follow_band.id = "d2";
  auto d3_inside_follow_band = makeOpponent(frame, 9.68, -0.91);
  d3_inside_follow_band.id = "d3";

  overtake_planner::OvertakePlannerCore initial_core(frame, config);
  const auto initial = initial_core.update(
      0.1, ego, {d2_outside_follow_band, d3_inside_follow_band});

  EXPECT_FALSE(initial.blocked_info.blocked);
  ASSERT_TRUE(initial.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(initial.blocked_info.parallel_follow_id, "d3");
  EXPECT_NEAR(initial.blocked_info.parallel_follow_delta_s, 4.68, 1.0e-6);
  EXPECT_NEAR(initial.blocked_info.parallel_follow_delta_d, -0.91, 1.0e-6);

  // Later in the recorded run D2 moved into the configured lateral band.
  // Arbitration may then select D2 because it is the nearest fresh candidate;
  // no vehicle-ID-specific priority or widened corridor is required.
  auto d2_inside_follow_band = makeOpponent(frame, 7.79, -1.01);
  d2_inside_follow_band.id = "d2";
  overtake_planner::OvertakePlannerCore updated_core(frame, config);
  const auto updated = updated_core.update(
      0.2, ego, {d2_inside_follow_band, d3_inside_follow_band});

  EXPECT_FALSE(updated.blocked_info.blocked);
  ASSERT_TRUE(updated.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(updated.blocked_info.parallel_follow_id, "d2");
  EXPECT_NEAR(updated.blocked_info.parallel_follow_delta_s, 2.79, 1.0e-6);
  EXPECT_NEAR(updated.blocked_info.parallel_follow_delta_d, -1.01, 1.0e-6);
}

TEST(OvertakePlannerCore, DisabledParallelFollowPreservesFreeRun) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = false;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.4);
  const auto opponent = makeOpponent(frame, 9.0, -0.55);
  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(output.blocked_info.parallel_follow_index, -1);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_FALSE(output.active_override);
}

TEST(OvertakePlannerCore, StaleParallelFollowObservationIsIgnored) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.4);
  auto opponent = makeOpponent(frame, 9.0, -0.55);
  opponent.stamp_sec = 0.0;
  const auto output = core.update(1.0, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(output.blocked_info.parallel_follow_index, -1);
  EXPECT_TRUE(output.blocked_info.parallel_follow_id.empty());
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
}

TEST(OvertakePlannerCore, OppositeDirectionParallelFollowObservationIsIgnored) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.4);
  auto opponent = makeOpponent(frame, 9.0, -0.55);
  opponent.vx = -4.0;
  opponent.v = 4.0;
  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_FALSE(output.blocked_info.parallel_follow_candidate);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FASTEST);
}

TEST(OvertakePlannerCore, ParallelFollowRejectsOutOfConfiguredRange) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  // Keep the longitudinal threshold below the 4 m separation used below.
  // This avoids the closed-reference wrap-around boundary while exercising the
  // configured-range contract directly.
  config.parallel_follow_s_m = 3.9;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  const auto ego = makeEgo(frame, 5.0, 0.4);

  overtake_planner::OvertakePlannerCore longitudinal_core(frame, config);
  const auto beyond_longitudinal_range = makeOpponent(frame, 9.0, -0.55);
  const auto longitudinal_output =
      longitudinal_core.update(0.1, ego, {beyond_longitudinal_range});
  EXPECT_FALSE(longitudinal_output.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(longitudinal_output.blocked_info.parallel_follow_index, -1);
  EXPECT_NE(longitudinal_output.mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NE(longitudinal_output.selected,
            overtake_planner::CandidateType::FOLLOW);

  auto lateral_config = config;
  lateral_config.parallel_follow_s_m = 8.0;
  overtake_planner::OvertakePlannerCore lateral_core(frame, lateral_config);
  const auto beyond_lateral_range = makeOpponent(frame, 9.0, -1.01);
  const auto lateral_output =
      lateral_core.update(0.1, ego, {beyond_lateral_range});
  EXPECT_FALSE(lateral_output.blocked_info.parallel_follow_candidate);
  EXPECT_EQ(lateral_output.blocked_info.parallel_follow_index, -1);
  EXPECT_NE(lateral_output.mode,
            overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_NE(lateral_output.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakePlannerCore, InfeasibleParallelFollowUsesStrictSpeedOnlyFailSafe) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.parallel_side_detection_enabled = false;
  config.same_corridor_width_m = 0.4;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.9;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.8;
  config.min_ellipse_h = 0.1;
  config.speed_only_fallback_v_max_mps = 10.0;
  config.opponent_collision_fallback_v_max_mps = 0.5;
  config.start_grace_safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.4);
  const auto opponent = makeOpponent(frame, 6.0, -0.10);
  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_TRUE(output.blocked_info.parallel_follow_candidate);
  EXPECT_FALSE(output.blocked_info.parallel_follow_feasible);
  EXPECT_TRUE(output.safe_stop_triggered);
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.speed_only_fallback_active);
  expectSpeedOnlyV2Contract(output,
                            config.opponent_collision_fallback_v_max_mps);
}

TEST(OvertakePlannerCore,
     ParallelFollowDoesNotRateLimitCurrentLateralHoldAgainstPreviousOverride) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.6;
  config.side_by_side_s_m = 0.5;
  config.side_margin_m = 0.8;
  config.lateral_target_max_step_m = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto first_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 5.0, 0.6);
  const auto first = core.update(0.1, first_ego, {first_opponent});
  ASSERT_EQ(first.mode, overtake_planner::BehaviorMode::SIDE_BY_SIDE_KEEP);
  ASSERT_TRUE(first.active_override);

  const auto second_ego = makeEgo(frame, 6.0, 0.4);
  const auto parallel_front = makeOpponent(frame, 10.0, -0.55);
  const auto second = core.update(0.2, second_ego, {parallel_front});

  EXPECT_TRUE(second.blocked_info.parallel_follow_candidate);
  EXPECT_TRUE(second.blocked_info.parallel_follow_feasible);
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  ASSERT_TRUE(second.active_override);
  ASSERT_FALSE(second.lateral_offsets.empty());
  EXPECT_NEAR(second.target_lateral_offset_m, second_ego.frenet.d, 1.0e-9);
  for (const double lateral_offset_m : second.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, second_ego.frenet.d, 1.0e-9);
  }
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

TEST(OvertakePlannerCore,
     EarlyStationaryParallelPassStartsOnlyAfterGateTwoApproval) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.min_ellipse_h = 0.1;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.id = "d2";
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;
  const auto input = readyReentryInput();

  const auto output = core.update(0.1, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_EQ(output.blocked_info.early_stationary_parallel_pass_id, "d2");
  EXPECT_EQ(output.blocked_info.early_stationary_parallel_pass_count, 1);
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     EarlyStationaryParallelPassDoesNotApplyToMovingOrDistantTargets) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto moving_parallel = makeOpponent(frame, 12.0, -1.0);
  moving_parallel.vx = 0.5;
  moving_parallel.v = 0.5;
  overtake_planner::OvertakePlannerCore moving_core(frame, config);
  const auto moving = moving_core.update(
      0.1, ego, {moving_parallel}, overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(moving.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_FALSE(moving.blocked_info.pass_left_candidate_generated);

  auto stopped_distant_parallel = makeOpponent(frame, 13.1, -1.0);
  stopped_distant_parallel.vx = 0.0;
  stopped_distant_parallel.v = 0.0;
  overtake_planner::OvertakePlannerCore distant_core(frame, config);
  const auto distant =
      distant_core.update(0.1, ego, {stopped_distant_parallel},
                          overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(distant.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_FALSE(distant.blocked_info.pass_left_candidate_generated);

  auto stale_stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stale_stopped_parallel.vx = 0.0;
  stale_stopped_parallel.v = 0.0;
  stale_stopped_parallel.stamp_sec = -1.0;
  overtake_planner::OvertakePlannerCore stale_core(frame, config);
  const auto stale =
      stale_core.update(0.1, ego, {stale_stopped_parallel},
                        overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(stale.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_FALSE(stale.blocked_info.pass_left_candidate_generated);

  auto low_speed_reverse_parallel = makeOpponent(frame, 12.0, -1.0);
  low_speed_reverse_parallel.vx = -0.1;
  low_speed_reverse_parallel.v = 0.1;
  overtake_planner::OvertakePlannerCore reverse_core(frame, config);
  const auto reverse =
      reverse_core.update(0.1, ego, {low_speed_reverse_parallel},
                          overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(reverse.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_FALSE(reverse.blocked_info.pass_left_candidate_generated);
}

TEST(OvertakePlannerCore,
     EarlyStationaryParallelPassRequiresCompleteSafetyInputs) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;
  const auto expect_no_early_probe =
      [&](const overtake_planner::ReentryInputStatus &input) {
        overtake_planner::OvertakePlannerCore core(frame, config);
        const auto output =
            core.update(0.1, ego, {stopped_parallel},
                        overtake_planner::MpcHealthStatus{}, input);
        EXPECT_FALSE(output.blocked_info.early_stationary_parallel_pass_target);
        EXPECT_FALSE(output.blocked_info.pass_left_candidate_generated);
        EXPECT_NE(output.mode,
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
        EXPECT_NE(output.mode,
                  overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
      };

  auto missing_inclusion = readyReentryInput();
  missing_inclusion.all_observed_opponents_included = false;
  expect_no_early_probe(missing_inclusion);
  auto stale_snapshot = readyReentryInput();
  stale_snapshot.v2x_snapshot_fresh = false;
  expect_no_early_probe(stale_snapshot);
  auto unhealthy_mpc = readyReentryInput();
  unhealthy_mpc.mpc_healthy = false;
  expect_no_early_probe(unhealthy_mpc);
}

TEST(OvertakePlannerCore,
     EarlyStationaryParallelPassKeepsSafeFallbackWhenGateTwoRejects) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 1.8;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.2;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;
  const auto input = readyReentryInput();

  const auto output = core.update(0.1, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_TRUE(output.blocked_info.early_stationary_parallel_pass_hold_lateral);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FASTEST);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
}

TEST(OvertakePlannerCore,
     EarlyStationaryParallelCurveExceptionRequiresThreeConfirmations) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.min_ellipse_h = 0.1;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.slow_front_exception_required_cycles = 3;
  config.slow_front_exception_max_start_curvature_m_inv = 0.040;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {stopped_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  const auto second = core.update(0.2, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);
  const auto third = core.update(0.3, ego, {stopped_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);

  EXPECT_EQ(first.blocked_info.early_stationary_parallel_pass_count, 1);
  EXPECT_EQ(second.blocked_info.early_stationary_parallel_pass_count, 2);
  EXPECT_FALSE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::FREE_RUN);
  EXPECT_EQ(third.blocked_info.early_stationary_parallel_pass_count, 3);
  EXPECT_TRUE(third.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(third.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(third.blocked_info.overtake_start_gate_reason,
            "early_stationary_parallel_curve");
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     EarlyStationaryParallelConfirmationSurvivesSameIdFrontPromotion) {
  const auto frame = makeGentleCurvedFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.min_ellipse_h = 0.1;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.slow_front_exception_required_cycles = 3;
  config.slow_front_exception_max_start_curvature_m_inv = 0.040;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto parallel = makeOpponent(frame, 12.0, -1.0);
  parallel.id = "d2";
  parallel.vx = 0.0;
  parallel.v = 0.0;
  auto same_front = parallel;
  same_front.y = -0.4;
  same_front.frenet = frame.cartesianToFrenet(same_front.x, same_front.y, 0.0);
  const auto input = readyReentryInput();

  const auto first = core.update(0.1, ego, {parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  const auto second = core.update(0.2, ego, {same_front},
                                  overtake_planner::MpcHealthStatus{}, input);
  const auto third = core.update(0.3, ego, {same_front},
                                 overtake_planner::MpcHealthStatus{}, input);

  EXPECT_EQ(first.blocked_info.early_stationary_parallel_pass_count, 1);
  EXPECT_TRUE(second.blocked_info.blocked);
  EXPECT_TRUE(second.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_EQ(second.blocked_info.early_stationary_parallel_pass_id, "d2");
  EXPECT_EQ(second.blocked_info.early_stationary_parallel_pass_count, 2);
  EXPECT_TRUE(third.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_EQ(third.blocked_info.early_stationary_parallel_pass_count, 3);
  EXPECT_TRUE(third.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(third.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(third.blocked_info.overtake_start_gate_reason,
            "early_stationary_parallel_curve");
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
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

TEST(OvertakePlannerCore,
     SafeSlowFrontPassExceptionBypassesCurrentDisallowedSection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 2;
  config.slow_front_permission_exception_enabled = true;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;

  const auto first = core.update(0.1, ego, {stopped_opponent},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(first.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(first.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);

  const auto second = core.update(0.2, ego, {stopped_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(second.blocked_info.front_vehicle_low_speed);
  EXPECT_TRUE(second.blocked_info.slow_front_exception_active);
  EXPECT_EQ(second.blocked_info.slow_front_exception_count, 2);
  EXPECT_FALSE(second.blocked_info.overtake_permission_allowed);
  EXPECT_EQ(second.blocked_info.overtake_permission_reason,
            "section_disallowed");
  EXPECT_TRUE(second.blocked_info.pass_left_candidate_feasible);
  // 停止車へ接近するPASS開始は、将来の近接を予測する補助診断には載る。
  // ただし実際の譲り/コーナーriskではなく、PASS候補はSafetyEvaluatorを通過済み。
  EXPECT_TRUE(second.blocked_info.future_side_by_side);
  EXPECT_FALSE(second.blocked_info.future_corner_side_by_side);
  EXPECT_FALSE(second.blocked_info.future_yield_required);
  EXPECT_TRUE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "slow_front_exception_permission");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);

  const auto third = core.update(0.3, ego, {stopped_opponent},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(third.blocked_info.overtake_permission_allowed);
  EXPECT_TRUE(third.blocked_info.permission_start_exception_active);
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(third.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     ConfirmedStationaryParallelPassExceptionBypassesCurrentDisallowedSection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.slow_front_exception_required_cycles = 2;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.id = "d2";
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;

  const auto first = core.update(0.1, ego, {stopped_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(first.blocked_info.early_stationary_parallel_pass_count, 1);
  EXPECT_FALSE(
      first.blocked_info.confirmed_stationary_parallel_permission_exception);
  EXPECT_FALSE(first.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(first.mode, overtake_planner::BehaviorMode::FREE_RUN);

  const auto second = core.update(0.2, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(second.blocked_info.overtake_permission_allowed);
  EXPECT_EQ(second.blocked_info.overtake_permission_reason,
            "section_disallowed");
  EXPECT_EQ(second.blocked_info.early_stationary_parallel_pass_id, "d2");
  EXPECT_EQ(second.blocked_info.early_stationary_parallel_pass_count, 2);
  EXPECT_TRUE(second.blocked_info.pass_left_candidate_feasible);
  EXPECT_TRUE(
      second.blocked_info.confirmed_stationary_parallel_permission_exception);
  EXPECT_TRUE(second.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(second.blocked_info.overtake_start_gate_reason,
            "early_stationary_parallel_permission_exception");
  EXPECT_EQ(second.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(second.selected, overtake_planner::CandidateType::PASS_LEFT);

  const auto third = core.update(0.3, ego, {stopped_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(third.blocked_info.overtake_permission_allowed);
  EXPECT_TRUE(
      third.blocked_info.confirmed_stationary_parallel_permission_exception);
  EXPECT_TRUE(third.blocked_info.permission_start_exception_active);
  EXPECT_EQ(third.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(third.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     StationaryParallelPermissionExceptionRejectsSafetyEvaluatorFailure) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.slow_front_exception_required_cycles = 1;
  config.min_pass_gap_m = 1.8;
  config.safety_ellipse_b_m = 1.8;
  config.min_ellipse_h = 0.2;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.id = "d2";
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.early_stationary_parallel_pass_target);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(
      output.blocked_info.confirmed_stationary_parallel_permission_exception);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StationaryParallelPermissionExceptionRequiresGlobalPermissionOptIn) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_permission_exception_enabled = false;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.slow_front_exception_required_cycles = 1;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.id = "d2";
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;

  const auto output = core.update(0.1, ego, {stopped_parallel},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_FALSE(output.blocked_info.overtake_permission_allowed);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(
      output.blocked_info.confirmed_stationary_parallel_permission_exception);
  EXPECT_FALSE(output.blocked_info.permission_start_exception_active);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     StationaryParallelPermissionExceptionCancelsPrepareWhenTargetDisappears) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_enabled = true;
  config.early_stationary_parallel_permission_exception_enabled = true;
  config.early_stationary_parallel_pass_distance_m = 8.0;
  config.early_stationary_parallel_pass_lateral_width_m = 1.5;
  config.same_corridor_width_m = 0.6;
  config.slow_front_exception_required_cycles = 1;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_parallel = makeOpponent(frame, 12.0, -1.0);
  stopped_parallel.id = "d2";
  stopped_parallel.vx = 0.0;
  stopped_parallel.v = 0.0;

  const auto prepare = core.update(0.1, ego, {stopped_parallel},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(
      prepare.blocked_info.confirmed_stationary_parallel_permission_exception);

  const auto cancelled =
      core.update(0.2, ego, {}, overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(cancelled.blocked_info
                   .confirmed_stationary_parallel_permission_exception);
  EXPECT_NE(cancelled.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(cancelled.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);
  EXPECT_EQ(cancelled.mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(OvertakePlannerCore,
     SlowFrontPermissionExceptionRejectsSafetyEvaluatorFailure) {
  const auto frame = makeStraightFrame();
  overtake_planner::PlannerConfig config;
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.pass_safe_required_cycles = 1.0;
  config.min_mode_hold_time_sec = 0.0;
  config.dynamic_pass_candidate_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 11.0, 0.0);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;
  const auto output = core.update(0.1, ego, {stopped_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_NE(output.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     SafeLowSpeedFrontPassExceptionBypassesCurrentDisallowedSection) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto slow_opponent = makeOpponent(frame, 13.0, -0.7);
  slow_opponent.vx = 0.5;
  slow_opponent.v = 0.5;
  const auto output = core.update(0.1, ego, {slow_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(output.blocked_info.front_direction_known);
  EXPECT_TRUE(output.blocked_info.front_same_direction);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.overtake_permission_allowed);
  EXPECT_TRUE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.blocked_info.overtake_start_gate_reason,
            "slow_front_exception_permission");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     SlowFrontPermissionExceptionRejectsVehicleAboveSpeedThreshold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.7);
  opponent.vx = 1.01;
  opponent.v = 1.01;
  const auto output = core.update(0.1, ego, {opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_FALSE(output.blocked_info.front_vehicle_low_speed);
  EXPECT_FALSE(output.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(OvertakePlannerCore,
     SlowFrontPermissionExceptionDoesNotBypassLookaheadOnlyRestriction) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"next_slow_corner", 12.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;
  const auto output = core.update(0.1, ego, {stopped_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.overtake_permission_allowed);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
}

TEST(OvertakePlannerCore,
     SlowFrontPermissionExceptionRequiresFreshCompleteInputs) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_permission_lookahead_m = 0.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_permission_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 8.5;
  config.slow_front_exception_required_cycles = 1;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"slow_corner", 0.0, 20.0,
                                               false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  auto input = readyReentryInput();
  input.v2x_snapshot_fresh = false;

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped_opponent = makeOpponent(frame, 13.0, -0.7);
  stopped_opponent.vx = 0.0;
  stopped_opponent.v = 0.0;
  const auto output = core.update(0.1, ego, {stopped_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(output.blocked_info.slow_front_exception_active);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(output.blocked_info.straight_overtake_start_allowed);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
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

  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

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
  EXPECT_NEAR(output.lateral_offsets.back(), config.left_offset_m, 1.0e-9);
}

TEST(OvertakePlannerCore,
     LocalizedPassUsesTrackableMarkersBeforeEvaluatedEllipseAdmission) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.1;
  config.min_ellipse_h = 0.20;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);
  const auto output =
      core.update(0.1, ego, {opponent}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_TRUE(output.maneuver_latch_active);
  const double configured_transition_distance_m =
      config.localized_avoidance_start_before_target_m -
      config.localized_avoidance_full_offset_before_target_m;
  EXPECT_GE(output.maneuver_latch_full_offset_start_s_m -
                output.maneuver_latch_avoid_start_s_m,
            configured_transition_distance_m - 1.0e-9);
  EXPECT_LE(output.maneuver_latch_full_offset_start_s_m,
            output.maneuver_latch_target_s_m + 1.0e-9);
  EXPECT_TRUE(output.blocked_info.pass_left_candidate_feasible)
      << output.blocked_info.pass_left_candidate_reject_reason;
}

TEST(CandidateBuilder, LocalizedTrackingCorrectionEndsBySafetyEllipseEntry) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 12;
  config.horizon_dt_sec = 0.1;
  config.safety_ellipse_a_m = 3.0;
  config.min_ellipse_h = 0.20;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 7.0, 0.0);
  ego.v = 4.0;
  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  profile.anchor_s_m = 5.0;
  profile.target_s_m = 13.0;
  const double safety_entry_s_m =
      profile.target_s_m -
      config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h);
  profile.avoid_start_s_m = safety_entry_s_m - 4.0;
  profile.full_offset_start_s_m = safety_entry_s_m;
  profile.full_offset_end_s_m = 17.0;
  profile.merge_end_s_m = 23.0;
  profile.start_d_m = 0.0;
  profile.target_d_m = 2.0;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, {}, {}, &profile);

  ASSERT_EQ(candidate.s.size(), candidate.d.size());
  const auto entry_it = std::find_if(
      candidate.s.begin(), candidate.s.end(),
      [safety_entry_s_m](double s) { return s >= safety_entry_s_m; });
  ASSERT_NE(entry_it, candidate.s.end());
  const auto index =
      static_cast<std::size_t>(std::distance(candidate.s.begin(), entry_it));
  EXPECT_NEAR(candidate.d[index], profile.target_d_m, 1.0e-9);
}

TEST(CandidateBuilder, IncompleteLocalizedPassNeverPublishesFutureCenterMerge) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 12.0, -1.20);
  ego.v = 4.0;
  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_RIGHT;
  profile.anchor_s_m = 5.0;
  profile.target_s_m = 13.0;
  profile.avoid_start_s_m = 7.0;
  profile.full_offset_start_s_m = 10.0;
  profile.full_offset_end_s_m = 15.0;
  profile.merge_end_s_m = 23.0;
  profile.start_d_m = 0.0;
  profile.target_d_m = ego.frenet.d;
  profile.pass_complete_confirmed = false;

  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = false;
  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {}, &profile);

  ASSERT_EQ(candidate.d.size(), config.horizon_points);
  ASSERT_GT(candidate.longitudinal_offsets_m.back(),
            profile.full_offset_end_s_m - ego.frenet.s);
  EXPECT_TRUE(
      std::all_of(candidate.d.begin(), candidate.d.end(), [&](double d_m) {
        return std::abs(d_m - profile.target_d_m) <= 1.0e-9;
      }));
}

TEST(CandidateBuilder, AttackFollowUsesLatchedPassSideInsteadOfCurrentD) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.merge_distance_m = 1.0;
  config.localized_avoidance_start_before_target_m = 1.0;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 7.0, 0.35);
  auto target = makeOpponent(frame, 12.0, 0.0);
  target.v = 2.0;
  target.vx = 2.0;
  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_index = 0;
  blocked.nearest_id = target.id;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = 1.10;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});

  ASSERT_FALSE(candidate.d.empty());
  EXPECT_NEAR(candidate.d.front(), ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(candidate.planned_target_d_m, blocked.attack_follow_target_d_m,
              1.0e-9);
  EXPECT_GT(candidate.d.back(), ego.frenet.d);
  EXPECT_LE(candidate.d.back(), blocked.attack_follow_target_d_m + 1.0e-9);
  EXPECT_LT(std::abs(blocked.attack_follow_target_d_m - candidate.d.back()),
            std::abs(blocked.attack_follow_target_d_m - ego.frenet.d));
}

TEST(
    CandidateBuilder,
    AttackFollowKeepsCommittedTargetButExecutesSafeCurrentDWhenCorridorNarrows) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    const bool narrow = point.s >= 9.0;
    corridor.push_back({point.s, narrow ? -1.0 : -3.0, narrow ? 1.0 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 40;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.5;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  auto ego = makeEgo(frame, 5.0, -0.30);
  ego.v = 2.0;
  auto target = makeOpponent(frame, 16.0, 0.0);
  target.id = "d2";
  target.v = 2.0;
  target.vx = 2.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -1.50;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 11.0;

  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_RIGHT;
  profile.target_id = target.id;
  profile.anchor_s_m = ego.frenet.s;
  profile.target_s_m = target.frenet.s;
  profile.avoid_start_s_m = ego.frenet.s;
  profile.full_offset_start_s_m = 10.0;
  profile.full_offset_end_s_m = 17.0;
  profile.merge_end_s_m = 20.0;
  profile.start_d_m = 0.0;
  profile.target_d_m = blocked.attack_follow_target_d_m;

  auto candidate =
      builder.makeCandidate(overtake_planner::CandidateType::FOLLOW, ego,
                            blocked, {target}, &profile);

  EXPECT_TRUE(candidate.attack_follow_safe_lateral_hold);
  EXPECT_NEAR(candidate.committed_attack_follow_target_d_m,
              blocked.attack_follow_target_d_m, 1.0e-9);
  EXPECT_NEAR(candidate.planned_target_d_m, ego.frenet.d, 1.0e-9);
  EXPECT_TRUE(candidate.pass_target_corridor_valid);
  EXPECT_TRUE(candidate.controller_tracking_profile_valid);
  ASSERT_FALSE(candidate.d.empty());
  EXPECT_TRUE(
      std::all_of(candidate.d.begin(), candidate.d.end(), [&](double d_m) {
        return std::abs(d_m - ego.frenet.d) <= 1.0e-9;
      }));
  EXPECT_TRUE(evaluator.evaluate(candidate, {}));
}

TEST(CandidateBuilder,
     OpponentCollisionCurrentDHoldPreservesExactLongitudinalProfile) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -4.0;
  config.d_max_m = 4.0;
  config.min_wall_margin_m = 0.1;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.25);
  ego.v = 0.25;
  auto target = makeOpponent(frame, 12.0, 0.0);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.40;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;

  overtake_planner::LocalizedLateralProfile profile;
  profile.active = true;
  profile.pass_type = overtake_planner::CandidateType::PASS_RIGHT;
  profile.target_id = target.id;
  profile.anchor_s_m = ego.frenet.s;
  profile.target_s_m = target.frenet.s;
  profile.avoid_start_s_m = ego.frenet.s;
  profile.full_offset_start_s_m = 10.0;
  profile.full_offset_end_s_m = 15.0;
  profile.merge_end_s_m = 20.0;
  profile.start_d_m = ego.frenet.d;
  profile.target_d_m = blocked.attack_follow_target_d_m;

  const auto source =
      builder.makeCandidate(overtake_planner::CandidateType::FOLLOW, ego,
                            blocked, {target}, &profile);
  ASSERT_FALSE(source.d.empty());
  ASSERT_FALSE(source.attack_follow_safe_lateral_hold);
  ASSERT_NEAR(source.committed_attack_follow_target_d_m, profile.target_d_m,
              1.0e-9);

  auto hold = builder.makeAttackFollowCurrentDHoldVariant(source, ego);
  EXPECT_TRUE(hold.attack_follow_safe_lateral_hold);
  EXPECT_TRUE(hold.attack_follow_opponent_collision_current_d_hold);
  EXPECT_EQ(hold.t, source.t);
  EXPECT_EQ(hold.longitudinal_offsets_m, source.longitudinal_offsets_m);
  EXPECT_EQ(hold.s, source.s);
  EXPECT_EQ(hold.predicted_speed_mps, source.predicted_speed_mps);
  EXPECT_EQ(hold.v_ref, source.v_ref);
  EXPECT_NEAR(hold.committed_attack_follow_target_d_m, profile.target_d_m,
              1.0e-9);
  EXPECT_NEAR(hold.planned_target_d_m, ego.frenet.d, 1.0e-9);
  EXPECT_TRUE(hold.pass_target_corridor_valid);
  EXPECT_TRUE(hold.controller_tracking_profile_valid);
  EXPECT_TRUE(std::all_of(hold.d.begin(), hold.d.end(), [&](double d_m) {
    return std::abs(d_m - ego.frenet.d) <= 1.0e-9;
  }));
  EXPECT_TRUE(evaluator.evaluate(hold, {}));
}

TEST(CandidateBuilder,
     AttackFollowInnerBandDiagnosticPreservesLongitudinalContract) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.40;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;

  const auto source = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});
  ASSERT_FALSE(source.d.empty());
  const double terminal_d_m = ego.frenet.d - 0.45;
  const auto probe = builder.makeAttackFollowInnerBandDiagnosticVariant(
      source, ego, terminal_d_m);

  EXPECT_FALSE(probe.attack_follow_safe_lateral_hold);
  EXPECT_FALSE(probe.attack_follow_opponent_collision_current_d_hold);
  EXPECT_EQ(probe.t, source.t);
  EXPECT_EQ(probe.longitudinal_offsets_m, source.longitudinal_offsets_m);
  EXPECT_EQ(probe.s, source.s);
  EXPECT_EQ(probe.predicted_speed_mps, source.predicted_speed_mps);
  EXPECT_EQ(probe.v_ref, source.v_ref);
  EXPECT_EQ(probe.committed_attack_follow_target_d_m,
            source.committed_attack_follow_target_d_m);
  EXPECT_EQ(probe.planned_target_d_m, source.planned_target_d_m);
  ASSERT_EQ(probe.d.size(), source.d.size());
  EXPECT_NEAR(probe.d.front(), ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(probe.d.back(), terminal_d_m, 1.0e-9);
  for (std::size_t i = 0U; i < probe.d.size(); ++i) {
    const auto projected =
        frame.cartesianToFrenet(probe.x[i], probe.y[i], probe.yaw[i]);
    EXPECT_NEAR(projected.s, probe.s[i], 1.0e-6);
    EXPECT_NEAR(projected.d, probe.d[i], 1.0e-6);
  }
}

TEST(CandidateBuilder,
     AttackFollowMinimumInwardConnectorPreservesLongitudinalContract) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.40;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;

  const auto source = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});
  ASSERT_FALSE(source.d.empty());
  constexpr double kConnectorShiftM = 0.15;
  const double terminal_d_m = ego.frenet.d - kConnectorShiftM;
  const auto connector =
      builder.makeAttackFollowInwardConnectorVariant(source, ego, terminal_d_m);

  EXPECT_TRUE(connector.attack_follow_safe_lateral_hold);
  EXPECT_FALSE(connector.attack_follow_opponent_collision_current_d_hold);
  EXPECT_FALSE(connector.attack_follow_opponent_collision_inward_connector);
  EXPECT_EQ(connector.t, source.t);
  EXPECT_EQ(connector.longitudinal_offsets_m, source.longitudinal_offsets_m);
  EXPECT_EQ(connector.s, source.s);
  EXPECT_EQ(connector.predicted_speed_mps, source.predicted_speed_mps);
  EXPECT_EQ(connector.v_ref, source.v_ref);
  EXPECT_EQ(connector.committed_attack_follow_target_d_m,
            source.committed_attack_follow_target_d_m);
  EXPECT_NEAR(connector.planned_target_d_m, terminal_d_m, 1.0e-9);
  ASSERT_EQ(connector.d.size(), source.d.size());
  EXPECT_NEAR(connector.d.front(), ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(connector.d.back(), terminal_d_m, 1.0e-9);
  for (std::size_t i = 0U; i < connector.d.size(); ++i) {
    EXPECT_TRUE(std::isfinite(connector.d[i]));
    if (i > 0U) {
      EXPECT_LE(connector.d[i], connector.d[i - 1U] + 1.0e-12);
    }
    const auto projected = frame.cartesianToFrenet(
        connector.x[i], connector.y[i], connector.yaw[i]);
    EXPECT_NEAR(projected.s, connector.s[i], 1.0e-6);
    EXPECT_NEAR(projected.d, connector.d[i], 1.0e-6);
  }

  overtake_planner::SafetyEvaluator opponent_evaluator(frame, config);
  auto opponent_rejected = connector;
  const std::size_t blocking_index = connector.d.size() / 2U;
  auto connector_opponent = makeOpponent(frame, connector.s[blocking_index],
                                         connector.d[blocking_index]);
  connector_opponent.id = "connector_only_blocker";
  connector_opponent.v = 0.0;
  connector_opponent.vx = 0.0;
  overtake_planner::PredictedOpponent connector_prediction;
  connector_prediction.id = connector_opponent.id;
  const auto connector_opponent_point = frame.frenetToCartesian(
      connector_opponent.frenet.s, connector_opponent.frenet.d);
  for (const double t_sec : connector.t) {
    connector_prediction.t.push_back(t_sec);
    connector_prediction.x.push_back(connector_opponent_point.x);
    connector_prediction.y.push_back(connector_opponent_point.y);
    connector_prediction.s.push_back(connector_opponent.frenet.s);
    connector_prediction.d.push_back(connector_opponent.frenet.d);
  }
  EXPECT_FALSE(
      opponent_evaluator.evaluate(opponent_rejected, {connector_prediction}));
  EXPECT_EQ(opponent_rejected.reject_reason, "opponent_collision");
  EXPECT_FALSE(
      opponent_rejected.attack_follow_opponent_collision_inward_connector);

  auto wall_frame = frame;
  std::vector<overtake_planner::FrenetCorridorPoint> wall_corridor;
  for (const auto &point : wall_frame.reference()) {
    wall_corridor.push_back({point.s, -5.0, 3.80});
  }
  wall_frame.setCorridor(wall_corridor);
  auto wall_config = config;
  wall_config.wall_footprint_check_enabled = true;
  wall_config.wall_localization_uncertainty_m = 0.25;
  overtake_planner::SafetyEvaluator wall_evaluator(wall_frame, wall_config);
  auto wall_rejected = connector;
  EXPECT_FALSE(wall_evaluator.evaluate(wall_rejected, {}));
  EXPECT_EQ(wall_rejected.reject_reason, "wall_footprint_margin");
  EXPECT_FALSE(wall_rejected.attack_follow_opponent_collision_inward_connector);

  auto untrackable_config = config;
  untrackable_config.attack_follow_max_steering_angle_rad = 1.0e-4;
  overtake_planner::CandidateBuilder untrackable_builder(frame,
                                                         untrackable_config);
  const auto untrackable =
      untrackable_builder.makeAttackFollowInwardConnectorVariant(source, ego,
                                                                 terminal_d_m);
  EXPECT_FALSE(untrackable.controller_tracking_profile_valid);
  EXPECT_TRUE(!untrackable.desired_path_trackable ||
              !untrackable.pure_pursuit_command_trackable);
  EXPECT_FALSE(untrackable.attack_follow_opponent_collision_inward_connector);
}

TEST(CandidateBuilder,
     AttackFollowBaselineConnectorPreservesFollowProfileAndIsTrackable) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    corridor.push_back({point.s, -3.0, 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 101;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.25;
  config.wall_footprint_check_enabled = true;
  config.wall_localization_uncertainty_m = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  auto ego = makeEgo(frame, 5.0, 1.0);
  ego.v = 2.0;
  overtake_planner::CandidateTrajectory source;
  source.type = overtake_planner::CandidateType::FOLLOW;
  source.committed_attack_follow_target_d_m = 1.8;
  source.planned_target_d_m = 1.8;
  source.longitudinal_initial_measured_speed_mps = ego.v;
  constexpr std::size_t kPointCount = 101U;
  source.t.reserve(kPointCount);
  source.longitudinal_offsets_m.reserve(kPointCount);
  source.s.reserve(kPointCount);
  source.d.reserve(kPointCount);
  source.x.reserve(kPointCount);
  source.y.reserve(kPointCount);
  source.yaw.reserve(kPointCount);
  source.predicted_speed_mps.reserve(kPointCount);
  source.v_ref.reserve(kPointCount);
  for (std::size_t i = 0U; i < kPointCount; ++i) {
    const double offset_m = 0.10 * static_cast<double>(i);
    const double s_m = ego.frenet.s + offset_m;
    const auto point = frame.frenetToCartesian(s_m, ego.frenet.d);
    source.t.push_back(0.05 * static_cast<double>(i));
    source.longitudinal_offsets_m.push_back(offset_m);
    source.s.push_back(s_m);
    source.d.push_back(ego.frenet.d);
    source.x.push_back(point.x);
    source.y.push_back(point.y);
    source.yaw.push_back(point.yaw);
    source.predicted_speed_mps.push_back(ego.v);
    source.v_ref.push_back(ego.v);
  }

  overtake_planner::BlockedInfo blocked;
  const double join_distance_m = builder.minimumTrackableLateralShiftDistance(
      ego, 0.0, ego.v, blocked, 1.0);
  ASSERT_TRUE(std::isfinite(join_distance_m));
  ASSERT_GT(join_distance_m, 1.0);
  ASSERT_LT(join_distance_m, source.longitudinal_offsets_m.back());
  const auto baseline = builder.makeAttackFollowInwardConnectorVariant(
      source, ego, 0.0, join_distance_m);

  EXPECT_EQ(baseline.type, overtake_planner::CandidateType::FOLLOW);
  EXPECT_TRUE(baseline.attack_follow_safe_lateral_hold);
  EXPECT_FALSE(baseline.attack_follow_opponent_collision_current_d_hold);
  EXPECT_FALSE(baseline.attack_follow_opponent_collision_inward_connector);
  EXPECT_EQ(baseline.t, source.t);
  EXPECT_EQ(baseline.longitudinal_offsets_m, source.longitudinal_offsets_m);
  EXPECT_EQ(baseline.s, source.s);
  EXPECT_EQ(baseline.predicted_speed_mps, source.predicted_speed_mps);
  EXPECT_EQ(baseline.v_ref, source.v_ref);
  EXPECT_EQ(baseline.committed_attack_follow_target_d_m,
            source.committed_attack_follow_target_d_m);
  EXPECT_TRUE(baseline.pass_target_corridor_valid);
  EXPECT_TRUE(baseline.controller_tracking_profile_valid);
  EXPECT_TRUE(baseline.desired_path_trackable);
  EXPECT_TRUE(baseline.pure_pursuit_command_trackable);
  ASSERT_EQ(baseline.d.size(), kPointCount);
  EXPECT_NEAR(baseline.d.front(), ego.frenet.d, 1.0e-9);
  EXPECT_NEAR(baseline.d.back(), 0.0, 1.0e-9);
  const auto joined_it =
      std::lower_bound(source.longitudinal_offsets_m.begin(),
                       source.longitudinal_offsets_m.end(), join_distance_m);
  ASSERT_NE(joined_it, source.longitudinal_offsets_m.end());
  const auto joined_index = static_cast<std::size_t>(
      std::distance(source.longitudinal_offsets_m.begin(), joined_it));
  EXPECT_NEAR(baseline.d[joined_index], 0.0, 1.0e-3);
  for (std::size_t i = 1U; i < baseline.d.size(); ++i) {
    EXPECT_LE(baseline.d[i], baseline.d[i - 1U] + 1.0e-12);
  }
  auto evaluated = baseline;
  EXPECT_TRUE(evaluator.evaluate(evaluated, {}));
  EXPECT_TRUE(evaluated.feasible);
}

TEST(CandidateBuilder,
     AttackFollowSafeHoldStillRejectsCurrentDOutsideFutureCorridor) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    const bool narrow = point.s >= 9.0;
    corridor.push_back({point.s, narrow ? -1.0 : -3.0, narrow ? 1.0 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.horizon_points = 40;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.5;
  config.localized_avoidance_start_before_target_m = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);
  overtake_planner::SafetyEvaluator evaluator(frame, config);

  auto ego = makeEgo(frame, 5.0, -0.80);
  ego.v = 2.0;
  auto target = makeOpponent(frame, 16.0, 0.0);
  target.id = "d2";
  target.v = 2.0;
  target.vx = 2.0;
  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -1.50;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 11.0;

  auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});

  EXPECT_TRUE(candidate.attack_follow_safe_lateral_hold);
  EXPECT_FALSE(candidate.pass_target_corridor_valid);
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "pass_target_unreachable");
}

TEST(CandidateBuilder,
     AttackFollowRecoveryFallbackKeepsCurrentLateralInsteadOfCentering) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.merge_distance_m = 4.0;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 7.0, 1.15);
  ego.v = 0.5;
  overtake_planner::BlockedInfo blocked;
  blocked.maneuver_transaction_incomplete = true;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = 1.30;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::RECOVERY, ego, blocked, {});

  ASSERT_FALSE(candidate.d.empty());
  for (const double d : candidate.d) {
    EXPECT_NEAR(d, ego.frenet.d, 1.0e-9);
  }
}

TEST(CandidateBuilder,
     AttackFollowUsesFreshChainTailForExecutableLowSpeedSpatialProfile) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.2;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 34.5, -1.20);
  ego.v = 0.17;
  auto chain_tail = makeOpponent(frame, 39.8, 0.0);
  chain_tail.id = "d3";
  chain_tail.v = 0.0;
  chain_tail.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.89;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 5.30;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_EQ(candidate.d.size(), 50U);
  ASSERT_EQ(candidate.d.size(), candidate.longitudinal_offsets_m.size());
  EXPECT_NEAR(candidate.planned_target_d_m, blocked.attack_follow_target_d_m,
              1.0e-9);
  // 実ログで0.06--0.25 mだった旧horizonを、PPが検証済み横profileとして
  // 採用できる最低0.50 mより長くする。同じ非加速profileと延長時刻列を
  // SafetyEvaluatorへ渡すため、下流だけが先へ進む契約にはならない。
  EXPECT_GT(candidate.longitudinal_offsets_m.back(), 0.50);
  EXPECT_TRUE(candidate.longitudinal_profile_valid);
  EXPECT_LE(candidate.d.back(), blocked.attack_follow_target_d_m + 1.0e-9);
  // 操舵角・操舵角速度を満たす範囲で、少量でもlatched側へ単調に進む。
  // 変位量そのものを固定すると、追従速度に対する物理制約と競合する。
  EXPECT_GT(candidate.d.back(), ego.frenet.d + 0.005);
  EXPECT_LT(std::abs(blocked.attack_follow_target_d_m - candidate.d.back()),
            std::abs(blocked.attack_follow_target_d_m - ego.frenet.d));
}

TEST(CandidateBuilder,
     AuthorizedAttackFollowAddsBoundedBonusAndBrakingCapWins) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.0;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.8;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  config.pass_speed_cap_mps = 3.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto chain_tail = makeOpponent(frame, 14.0, 0.0);
  chain_tail.id = "d2";
  chain_tail.v = 2.5;
  chain_tail.vx = 2.5;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = 0.0;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 9.0;

  const auto attack = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_FALSE(attack.v_ref.empty());
  const double expected_bonus =
      std::min((9.0 - config.follow_gap_closing_target_gap_m) *
                   config.follow_gap_closing_speed_gain_per_m,
               config.follow_gap_closing_max_speed_bonus_mps);
  EXPECT_NEAR(
      attack.v_ref.front(),
      std::min(config.pass_speed_cap_mps, chain_tail.v + expected_bonus),
      1.0e-9);
  EXPECT_GT(attack.v_ref.front(), chain_tail.v);
  EXPECT_TRUE(attack.longitudinal_profile_valid);

  blocked.braking_follow_active = true;
  blocked.braking_follow_speed_cap_mps = 2.0;
  const auto braking = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});
  ASSERT_FALSE(braking.v_ref.empty());
  EXPECT_NEAR(braking.v_ref.front(), 2.0, 1.0e-9);
}

TEST(CandidateBuilder,
     StationaryLateralFirstCapWinsAfterAttackFollowBonusAndReachesConnector) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.0;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.8;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  config.pass_speed_cap_mps = 3.0;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 0.20;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.40;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_id = target.id;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;
  blocked.braking_follow_active = true;
  blocked.braking_follow_speed_cap_mps = 0.90;
  blocked.pass_lateral_first_speed_gate_active = true;
  blocked.pass_lateral_clearance_ready = false;
  blocked.pass_lateral_first_target_id = target.id;
  blocked.pass_lateral_first_target_speed_mps = 0.0;
  blocked.pass_lateral_first_speed_cap_mps = 0.75;

  const auto source = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});
  ASSERT_FALSE(source.v_ref.empty());
  ASSERT_FALSE(source.predicted_speed_mps.empty());
  EXPECT_TRUE(std::all_of(
      source.v_ref.begin(), source.v_ref.end(),
      [](double speed_mps) { return std::abs(speed_mps - 0.75) <= 1.0e-9; }));
  EXPECT_NEAR(source.predicted_speed_mps.back(), 0.75, 1.0e-6);

  const auto connector = builder.makeAttackFollowInwardConnectorVariant(
      source, ego, ego.frenet.d - 0.15);
  EXPECT_EQ(connector.t, source.t);
  EXPECT_EQ(connector.longitudinal_offsets_m, source.longitudinal_offsets_m);
  EXPECT_EQ(connector.s, source.s);
  EXPECT_EQ(connector.predicted_speed_mps, source.predicted_speed_mps);
  EXPECT_EQ(connector.v_ref, source.v_ref);
}

TEST(CandidateBuilder,
     StationaryLateralFirstAttackFollowCapUsesDelayedBrakingAboveCap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 121;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.0;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.8;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 1.30;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.40;
  blocked.attack_follow_acceleration_allowed = true;
  blocked.maneuver_transaction_incomplete = true;
  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_id = target.id;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;
  blocked.pass_lateral_first_speed_gate_active = true;
  blocked.pass_lateral_clearance_ready = false;
  blocked.pass_lateral_first_target_id = target.id;
  blocked.pass_lateral_first_target_speed_mps = 0.0;
  blocked.pass_lateral_first_speed_cap_mps = 0.75;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});

  ASSERT_FALSE(candidate.v_ref.empty());
  ASSERT_GT(candidate.predicted_speed_mps.size(), 11U);
  EXPECT_TRUE(std::all_of(
      candidate.v_ref.begin(), candidate.v_ref.end(),
      [](double speed_mps) { return std::abs(speed_mps - 0.75) <= 1.0e-9; }));
  EXPECT_GT(candidate.predicted_speed_mps.front(), ego.v);
  EXPECT_NEAR(candidate.response_delay_sec,
              config.longitudinal_response_delay_sec, 1.0e-9);
  EXPECT_NEAR(candidate.assumed_brake_decel_mps2, config.max_brake_decel_mps2,
              1.0e-9);
  EXPECT_NEAR(candidate.predicted_speed_mps.back(), 0.75, 1.0e-6);
  EXPECT_GT(candidate.required_brake_distance_m, 0.0);
  EXPECT_TRUE(candidate.longitudinal_profile_valid);
}

TEST(CandidateBuilder,
     StationaryLateralFirstAttackFollowCapFailsClosedOnlyForMalformedCommit) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.d_min_m = -5.0;
  config.d_max_m = 5.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.0;
  config.follow_gap_closing_target_gap_m = 4.5;
  config.follow_gap_closing_speed_gain_per_m = 0.15;
  config.follow_gap_closing_max_speed_bonus_mps = 0.8;
  config.follow_gap_closing_assumed_accel_mps2 = 3.0;
  config.pass_speed_cap_mps = 3.0;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 3.0);
  ego.v = 0.20;
  auto target = makeOpponent(frame, 12.0, 1.6);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;

  overtake_planner::BlockedInfo committed;
  committed.attack_follow_hold_pass_side = true;
  committed.attack_follow_target_d_m = -0.40;
  committed.attack_follow_acceleration_allowed = true;
  committed.maneuver_transaction_incomplete = true;
  committed.maneuver_target_latched = true;
  committed.maneuver_target_id = target.id;
  committed.maneuver_chain_tail_id = target.id;
  committed.maneuver_chain_tail_index = 0;
  committed.maneuver_chain_tail_observed = true;
  committed.maneuver_chain_tail_relative_s_m = 7.0;
  committed.pass_lateral_first_speed_gate_active = true;
  committed.pass_lateral_clearance_ready = false;
  committed.pass_lateral_first_target_id = target.id;
  committed.pass_lateral_first_target_speed_mps = 0.0;
  committed.pass_lateral_first_speed_cap_mps = 0.75;

  const auto speed_cap = [&](const overtake_planner::BlockedInfo &blocked,
                             const overtake_planner::OpponentState &opponent) {
    const auto candidate = builder.makeCandidate(
        overtake_planner::CandidateType::FOLLOW, ego, blocked, {opponent});
    EXPECT_FALSE(candidate.v_ref.empty());
    return candidate.v_ref.empty() ? std::numeric_limits<double>::quiet_NaN()
                                   : candidate.v_ref.front();
  };

  auto mismatched = committed;
  mismatched.pass_lateral_first_target_id = "d3";
  EXPECT_NEAR(speed_cap(mismatched, target), 0.0, 1.0e-9);

  auto invalid_cap = committed;
  invalid_cap.pass_lateral_first_speed_cap_mps =
      std::numeric_limits<double>::infinity();
  EXPECT_NEAR(speed_cap(invalid_cap, target), 0.0, 1.0e-9);

  auto uncommitted = committed;
  uncommitted.maneuver_transaction_incomplete = false;
  uncommitted.maneuver_target_latched = false;
  const double generic_speed_cap_mps = speed_cap(uncommitted, target);
  EXPECT_GT(generic_speed_cap_mps, 0.75);

  auto moving = committed;
  auto moving_target = target;
  moving_target.v = 2.0;
  moving_target.vx = 2.0;
  moving.pass_lateral_first_target_speed_mps = 2.0;
  EXPECT_GT(speed_cap(moving, moving_target), 0.75);

  auto inactive = committed;
  inactive.pass_lateral_first_speed_gate_active = false;
  EXPECT_GT(speed_cap(inactive, target), 0.75);

  auto clearance_ready = committed;
  clearance_ready.pass_lateral_clearance_ready = true;
  EXPECT_GT(speed_cap(clearance_ready, target), 0.75);
}

TEST(CandidateBuilder,
     AttackFollowLateralCorrectionRespectsActiveMpcSteeringLimits) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 81;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.pass_speed_cap_mps = 10.0;
  config.attack_follow_tracking_wheelbase_m = 1.087;
  config.attack_follow_max_steering_angle_rad = 0.5585053606381855;
  config.attack_follow_max_steering_rate_radps = 0.35;
  config.attack_follow_steering_tire_angle_gain = 1.639;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 20.0, -1.20);
  ego.v = 10.0;
  auto chain_tail = makeOpponent(frame, 24.0, 0.0);
  chain_tail.id = "d3";
  chain_tail.v = 10.0;
  chain_tail.vx = 10.0;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.89;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 4.0;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_EQ(candidate.d.size(), candidate.longitudinal_offsets_m.size());
  ASSERT_GT(candidate.d.size(), 4U);
  const double ds =
      candidate.longitudinal_offsets_m[1] - candidate.longitudinal_offsets_m[0];
  ASSERT_GT(ds, 0.0);
  double max_abs_second_derivative = 0.0;
  std::vector<double> steering_angles_rad;
  for (std::size_t i = 1; i + 1 < candidate.d.size(); ++i) {
    const double second_derivative =
        (candidate.d[i + 1] - 2.0 * candidate.d[i] + candidate.d[i - 1]) /
        (ds * ds);
    max_abs_second_derivative =
        std::max(max_abs_second_derivative, std::abs(second_derivative));
    steering_angles_rad.push_back(std::atan(
        config.attack_follow_tracking_wheelbase_m * second_derivative));
  }
  const double active_mpc_curvature_limit =
      std::tan(config.attack_follow_max_steering_angle_rad) /
      config.attack_follow_tracking_wheelbase_m;
  EXPECT_LE(max_abs_second_derivative, active_mpc_curvature_limit + 1.0e-6);
  double max_abs_steering_rate_radps = 0.0;
  for (std::size_t i = 1; i < steering_angles_rad.size(); ++i) {
    const double interval_sec = candidate.t[i + 1U] - candidate.t[i];
    ASSERT_GT(interval_sec, 0.0);
    max_abs_steering_rate_radps =
        std::max(max_abs_steering_rate_radps,
                 std::abs(steering_angles_rad[i] - steering_angles_rad[i - 1]) /
                     interval_sec);
  }
  EXPECT_LE(max_abs_steering_rate_radps,
            config.attack_follow_max_steering_rate_radps /
                    config.attack_follow_steering_tire_angle_gain *
                    config.attack_follow_steering_rate_reserve_ratio +
                1.0e-6);
  EXPECT_TRUE(candidate.controller_tracking_profile_valid);
  // 旧1 m補正ならds=1 mでほぼ目標dへ到達する。操舵速度制約を反映した
  // profileでは同地点の横移動を抑え、実MPCが追従できる形のままにする。
  const auto one_meter_it =
      std::lower_bound(candidate.longitudinal_offsets_m.begin(),
                       candidate.longitudinal_offsets_m.end(), 1.0);
  ASSERT_NE(one_meter_it, candidate.longitudinal_offsets_m.end());
  const auto one_meter_index = static_cast<std::size_t>(
      std::distance(candidate.longitudinal_offsets_m.begin(), one_meter_it));
  EXPECT_LT(candidate.d[one_meter_index] - ego.frenet.d, 0.08);
}

TEST(CandidateBuilder,
     AttackFollowTrackabilityUsesReferenceCurvatureAcrossSparseCsvKnots) {
  constexpr double kRadiusM = 25.0;
  constexpr double kSampleSpacingM = 0.9;
  overtake_planner::FrenetFrame frame;
  std::vector<overtake_planner::ReferencePoint> reference;
  for (int i = 0; i <= 70; ++i) {
    const double s_m = kSampleSpacingM * static_cast<double>(i);
    const double theta_rad = s_m / kRadiusM;
    reference.push_back(
        overtake_planner::ReferencePoint{s_m, kRadiusM * std::sin(theta_rad),
                                         kRadiusM * (1.0 - std::cos(theta_rad)),
                                         theta_rad, 1.0 / kRadiusM, 5.0});
  }
  frame.setReference(reference);

  auto config = makeConfig();
  config.horizon_points = 81;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.attack_follow_tracking_wheelbase_m = 1.087;
  config.attack_follow_max_steering_angle_rad = 0.5585053606381855;
  config.attack_follow_max_steering_rate_radps = 0.35;
  config.attack_follow_steering_tire_angle_gain = 1.639;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto ego_point = frame.frenetToCartesian(5.2, -1.20);
  overtake_planner::EgoState ego;
  ego.x = ego_point.x;
  ego.y = ego_point.y;
  ego.yaw = ego_point.yaw;
  ego.v = 4.0;
  ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
  ego.valid = true;

  const auto opponent_point = frame.frenetToCartesian(13.2, 0.0);
  overtake_planner::OpponentState chain_tail;
  chain_tail.id = "d3";
  chain_tail.x = opponent_point.x;
  chain_tail.y = opponent_point.y;
  chain_tail.v = 4.0;
  chain_tail.vx = 4.0 * std::cos(opponent_point.yaw);
  chain_tail.vy = 4.0 * std::sin(opponent_point.yaw);
  chain_tail.frenet =
      frame.cartesianToFrenet(chain_tail.x, chain_tail.y, opponent_point.yaw);
  chain_tail.valid = true;

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.89;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 8.0;
  blocked.corner_abs_curvature = 1.0 / kRadiusM;

  const auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_GT(candidate.longitudinal_offsets_m.back(), 0.9);
  EXPECT_TRUE(candidate.pass_target_corridor_valid);
  EXPECT_TRUE(candidate.desired_path_trackable);
  EXPECT_TRUE(candidate.pure_pursuit_command_trackable);
  EXPECT_TRUE(candidate.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     AttackFollowRejectsUntrackableTotalCurvatureOnCurvedReference) {
  constexpr double kRadiusM = 10.0;
  overtake_planner::FrenetFrame frame;
  std::vector<overtake_planner::ReferencePoint> reference;
  for (int i = 0; i <= 160; ++i) {
    const double s_m = 0.25 * static_cast<double>(i);
    const double theta_rad = s_m / kRadiusM;
    reference.push_back(
        overtake_planner::ReferencePoint{s_m, kRadiusM * std::sin(theta_rad),
                                         kRadiusM * (1.0 - std::cos(theta_rad)),
                                         theta_rad, 1.0 / kRadiusM, 5.0});
  }
  frame.setReference(reference);

  auto config = makeConfig();
  config.horizon_points = 81;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.attack_follow_tracking_wheelbase_m = 1.087;
  // 基準線だけでも約0.108 rad必要な曲率に対し、意図的に低い上限を置く。
  config.attack_follow_max_steering_angle_rad = 0.06;
  config.attack_follow_max_steering_rate_radps = 0.35;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 5.0;
  auto chain_tail = makeOpponent(frame, 13.0, 0.0);
  chain_tail.id = "d3";
  chain_tail.v = 5.0;
  chain_tail.vx = 5.0 * std::cos(chain_tail.frenet.s / kRadiusM);
  chain_tail.vy = 5.0 * std::sin(chain_tail.frenet.s / kRadiusM);

  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = 0.10;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 8.0;

  auto candidate = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});
  EXPECT_TRUE(candidate.pass_target_corridor_valid);
  EXPECT_FALSE(candidate.controller_tracking_profile_valid);

  overtake_planner::SafetyEvaluator evaluator(frame, config);
  EXPECT_FALSE(evaluator.evaluate(candidate, {}));
  EXPECT_EQ(candidate.reject_reason, "untrackable_lateral_profile");

  // PASS開始前のcurrent-d FOLLOWも曲線上では実オフセット軌道である。
  // 横補正量が0でも基準曲率を無視してtracking-validへ倒してはならない。
  overtake_planner::BlockedInfo prestart_blocked;
  prestart_blocked.blocked = true;
  prestart_blocked.nearest_index = 0;
  prestart_blocked.nearest_id = chain_tail.id;
  prestart_blocked.front_delta_s = 8.0;
  prestart_blocked.follow_gap_closing_allowed = true;
  prestart_blocked.prestart_attack_follow_hold_lateral = true;
  auto prestart_follow =
      builder.makeCandidate(overtake_planner::CandidateType::FOLLOW, ego,
                            prestart_blocked, {chain_tail});
  EXPECT_TRUE(prestart_follow.pass_target_corridor_valid);
  EXPECT_FALSE(prestart_follow.controller_tracking_profile_valid);
  EXPECT_FALSE(evaluator.evaluate(prestart_follow, {}));
  EXPECT_EQ(prestart_follow.reject_reason, "untrackable_lateral_profile");
}

TEST(CandidateBuilder,
     TrackabilityRequiresDesiredPathAndCurrentPurePursuitCommandIndependently) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 80;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.attack_follow_max_steering_angle_rad = 0.40;
  config.attack_follow_max_steering_rate_radps = 8.0;
  overtake_planner::CandidateBuilder command_builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 4.0;
  auto yaw_mismatch = ego;
  yaw_mismatch.yaw += 1.0;
  const auto command_rejected = command_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, yaw_mismatch, {}, {});
  EXPECT_TRUE(command_rejected.desired_path_trackable);
  EXPECT_FALSE(command_rejected.pure_pursuit_command_trackable);
  EXPECT_FALSE(command_rejected.controller_tracking_profile_valid);

  overtake_planner::LocalizedLateralProfile late_sharp_profile;
  late_sharp_profile.active = true;
  late_sharp_profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  late_sharp_profile.anchor_s_m = 5.0;
  late_sharp_profile.target_s_m = 11.0;
  late_sharp_profile.avoid_start_s_m = 10.0;
  late_sharp_profile.full_offset_start_s_m = 10.3;
  late_sharp_profile.full_offset_end_s_m = 14.0;
  late_sharp_profile.merge_end_s_m = 20.0;
  late_sharp_profile.start_d_m = 0.0;
  late_sharp_profile.target_d_m = 1.0;
  auto path_config = config;
  path_config.attack_follow_max_steering_rate_radps = 0.35;
  overtake_planner::CandidateBuilder path_builder(frame, path_config);
  const auto path_rejected =
      path_builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT,
                                 ego, {}, {}, &late_sharp_profile);
  EXPECT_FALSE(path_rejected.desired_path_trackable);
  EXPECT_TRUE(path_rejected.pure_pursuit_command_trackable);
  EXPECT_FALSE(path_rejected.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     UnapprovedFollowAndParallelYieldDoNotPublishUnevaluatedAcceleration) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 20;
  config.horizon_dt_sec = 0.025;
  config.follow_speed_margin_mps = 0.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 15.0, 0.0);
  target.v = 5.0;
  target.vx = 5.0;
  overtake_planner::BlockedInfo blocked;
  blocked.blocked = true;
  blocked.nearest_index = 0;
  blocked.nearest_id = target.id;

  const auto follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});
  ASSERT_FALSE(follow.v_ref.empty());
  EXPECT_TRUE(std::all_of(
      follow.v_ref.begin(), follow.v_ref.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));
  EXPECT_TRUE(std::all_of(
      follow.predicted_speed_mps.begin(), follow.predicted_speed_mps.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));

  blocked.parallel_yield_hold_lateral = true;
  const auto yield = builder.makeCandidate(
      overtake_planner::CandidateType::YIELD_BEHIND, ego, blocked, {target});
  ASSERT_FALSE(yield.v_ref.empty());
  EXPECT_TRUE(std::all_of(
      yield.v_ref.begin(), yield.v_ref.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));
}

TEST(CandidateBuilder,
     UnapprovedAttackFollowExtendsEvaluatedTimeWithoutAllowingAcceleration) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.0;
  config.attack_follow_min_spatial_horizon_m = 0.55;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 35.0, -0.85);
  ego.v = 0.20;
  auto chain_tail = makeOpponent(frame, 39.0, 0.0);
  chain_tail.id = "d3";
  chain_tail.v = 0.5;
  chain_tail.vx = 0.5;
  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.89;
  blocked.attack_follow_acceleration_allowed = false;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 4.0;

  const auto follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_EQ(follow.t.size(), 50U);
  ASSERT_EQ(follow.t.size(), follow.longitudinal_offsets_m.size());
  ASSERT_EQ(follow.t.size(), follow.v_ref.size());
  EXPECT_GT(follow.t.back(), static_cast<double>(config.horizon_points - 1U) *
                                 config.horizon_dt_sec);
  EXPECT_GE(follow.longitudinal_offsets_m.back(),
            config.attack_follow_min_spatial_horizon_m - 1.0e-6);
  EXPECT_TRUE(std::all_of(
      follow.v_ref.begin(), follow.v_ref.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));
  EXPECT_TRUE(std::all_of(
      follow.predicted_speed_mps.begin(), follow.predicted_speed_mps.end(),
      [&](double speed_mps) {
        return speed_mps <=
               ego.v +
                   config.pass_assumed_accel_mps2 *
                       config.lateral_override_execution_speed_reserve_sec +
                   1.0e-9;
      }));
  ASSERT_FALSE(follow.predicted_speed_mps.empty());
  EXPECT_GT(follow.predicted_speed_mps.front(), ego.v);
  // 6秒先でもactive PPのlookaheadへ届かないため、短い横profileを
  // Plannerだけで実行可能とは扱わない。
  EXPECT_FALSE(follow.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     RuntimeLowSpeedAttackFollowCoversPurePursuitArcWithoutAcceleration) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.follow_speed_margin_mps = 0.20;
  config.attack_follow_min_spatial_horizon_m = 0.55;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.lateral_override_max_evaluation_horizon_sec = 20.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 35.0, -0.85);
  ego.v = 0.20;
  auto chain_tail = makeOpponent(frame, 39.0, 0.0);
  chain_tail.id = "d3";
  chain_tail.v = 0.0;
  chain_tail.vx = 0.0;
  overtake_planner::BlockedInfo blocked;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = -0.89;
  blocked.attack_follow_acceleration_allowed = false;
  blocked.maneuver_chain_tail_id = chain_tail.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 4.0;
  blocked.opponent_prediction_inputs_complete = true;

  const auto follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});

  ASSERT_FALSE(follow.longitudinal_offsets_m.empty());
  EXPECT_GE(follow.longitudinal_offsets_m.back() + 1.0e-6,
            follow.required_controller_spatial_horizon_m);
  EXPECT_TRUE(follow.desired_path_trackable);
  EXPECT_TRUE(follow.pure_pursuit_command_trackable);
  EXPECT_TRUE(follow.controller_tracking_profile_valid);
  EXPECT_TRUE(std::all_of(
      follow.v_ref.begin(), follow.v_ref.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));

  // moving相手をCartesian直線で20秒外挿しない。1台でも静止閾値を超えたら
  // 6秒上限へ閉じ、必要arc未達の横profileは不認可にする。
  auto moving_tail = chain_tail;
  moving_tail.v = 0.5;
  moving_tail.vx = 0.5;
  const auto moving_follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {moving_tail});
  ASSERT_FALSE(moving_follow.longitudinal_offsets_m.empty());
  EXPECT_LT(moving_follow.longitudinal_offsets_m.back() + 1.0e-6,
            moving_follow.required_controller_spatial_horizon_m);
  EXPECT_FALSE(moving_follow.controller_tracking_profile_valid);

  // 相手配列に静止車しか残っていても、近接観測の包含契約がfalseなら20秒を
  // 許可しない。除外されたmoving車を「存在しない」と扱わず6秒へ閉じる。
  blocked.opponent_prediction_inputs_complete = false;
  const auto incomplete_follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {chain_tail});
  ASSERT_FALSE(incomplete_follow.longitudinal_offsets_m.empty());
  EXPECT_LT(incomplete_follow.longitudinal_offsets_m.back() + 1.0e-6,
            incomplete_follow.required_controller_spatial_horizon_m);
  EXPECT_FALSE(incomplete_follow.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     CommittedStartGridFollowRequiresAttackFollowAccelerationAuthority) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.follow_speed_margin_mps = 0.0;
  config.lateral_override_execution_speed_reserve_sec = 0.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto target = makeOpponent(frame, 12.0, 0.0);
  target.id = "committed_grid_tail";
  target.v = 5.0;
  target.vx = 5.0;
  overtake_planner::BlockedInfo blocked;
  blocked.start_grid_target_active = true;
  blocked.start_grid_target_index = 0;
  blocked.maneuver_transaction_incomplete = true;
  blocked.attack_follow_hold_pass_side = true;
  blocked.attack_follow_target_d_m = ego.frenet.d;
  blocked.attack_follow_acceleration_allowed = false;
  blocked.maneuver_chain_tail_id = target.id;
  blocked.maneuver_chain_tail_index = 0;
  blocked.maneuver_chain_tail_observed = true;
  blocked.maneuver_chain_tail_relative_s_m = 7.0;

  const auto follow = builder.makeCandidate(
      overtake_planner::CandidateType::FOLLOW, ego, blocked, {target});

  ASSERT_FALSE(follow.v_ref.empty());
  ASSERT_FALSE(follow.predicted_speed_mps.empty());
  EXPECT_TRUE(std::all_of(
      follow.v_ref.begin(), follow.v_ref.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));
  EXPECT_TRUE(std::all_of(
      follow.predicted_speed_mps.begin(), follow.predicted_speed_mps.end(),
      [&](double speed_mps) { return speed_mps <= ego.v + 1.0e-9; }));
}

TEST(CandidateBuilder,
     PassExtendsSafetyEvaluatedArcToPurePursuitExecutionContract) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safe_stop_v_mps = 0.20;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 35.0, -1.20);
  ego.v = 3.30;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = false;

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {});

  const double default_horizon_sec =
      static_cast<double>(config.horizon_points - 1U) * config.horizon_dt_sec;
  const double reserved_speed_mps =
      ego.v + config.pass_assumed_accel_mps2 *
                  config.lateral_override_execution_speed_reserve_sec;
  const double required_brake_arc_m =
      reserved_speed_mps * 0.25 +
      (reserved_speed_mps * reserved_speed_mps -
       config.safe_stop_v_mps * config.safe_stop_v_mps) /
          (2.0 * config.max_brake_decel_mps2);
  ASSERT_EQ(pass.t.size(), config.horizon_points);
  ASSERT_EQ(pass.t.size(), pass.longitudinal_offsets_m.size());
  EXPECT_GT(pass.t.back(), default_horizon_sec);
  EXPECT_GE(pass.longitudinal_offsets_m.back(), required_brake_arc_m - 1.0e-6);
  EXPECT_TRUE(pass.desired_path_trackable);
  EXPECT_TRUE(pass.pure_pursuit_command_trackable);
  EXPECT_TRUE(pass.controller_tracking_profile_valid);
  EXPECT_TRUE(
      std::all_of(pass.v_ref.begin(), pass.v_ref.end(), [&](double speed_mps) {
        return speed_mps <= ego.v + 1.0e-9;
      }));
}

TEST(CandidateBuilder,
     PublishedPassTrackabilityChecksTheFinalMutatedSpatialProfile) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.prepare_distance_m = 8.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 4.0;
  auto pass = builder.makeCandidate(overtake_planner::CandidateType::PASS_LEFT,
                                    ego, {}, {});
  ASSERT_TRUE(pass.controller_tracking_profile_valid);
  ASSERT_TRUE(builder.publishedLateralProfileTrackable(pass, ego));

  auto yaw_mismatch = ego;
  yaw_mismatch.yaw += 1.0;
  EXPECT_FALSE(builder.publishedLateralProfileTrackable(pass, yaw_mismatch));

  // 候補生成後の周期間rate limit/holdが鋸歯状のd列を作った想定。元候補の
  // tracking証跡を流用せず、publishする最終列そのものを拒否する。
  for (std::size_t i = 1U; i + 1U < pass.d.size(); ++i) {
    pass.d[i] += (i % 2U == 0U) ? 1.0 : -1.0;
  }
  EXPECT_FALSE(builder.publishedLateralProfileTrackable(pass, ego));
}

TEST(CandidateBuilder,
     PhysicalHardSteeringLimitCapsConfiguredTrackabilityLimit) {
  constexpr double kWheelbaseM = 1.087;
  EXPECT_DOUBLE_EQ(
      overtake_planner::PlannerConfig{}.attack_follow_max_steering_angle_rad,
      overtake_planner::kVehicleHardSteeringTireAngleRad);
  EXPECT_DOUBLE_EQ(
      overtake_planner::PlannerConfig{}.attack_follow_max_steering_rate_radps,
      overtake_planner::kVehicleHardSteeringRateRadps);
  const auto make_constant_curvature_frame = [](double steering_angle_rad) {
    overtake_planner::FrenetFrame frame;
    std::vector<overtake_planner::ReferencePoint> reference;
    const double curvature_m_inv = std::tan(steering_angle_rad) / kWheelbaseM;
    for (int i = 0; i <= 200; ++i) {
      const double s_m = 0.25 * static_cast<double>(i);
      reference.push_back(overtake_planner::ReferencePoint{
          s_m, s_m, 0.0, 0.0, curvature_m_inv, 4.0});
    }
    frame.setReference(reference);
    return frame;
  };
  const auto make_hard_angle_config = [] {
    auto config = makeConfig();
    config.d_min_m = -3.0;
    config.d_max_m = 3.0;
    config.min_wall_margin_m = 0.1;
    config.left_offset_m = 0.0;
    config.right_offset_m = 0.0;
    config.attack_follow_tracking_wheelbase_m = kWheelbaseM;
    config.attack_follow_max_steering_angle_rad =
        overtake_planner::kVehicleHardSteeringTireAngleRad;
    config.attack_follow_max_steering_rate_radps = 100.0;
    config.attack_follow_steering_tire_angle_gain = 1.0;
    return config;
  };

  const auto boundary_frame = make_constant_curvature_frame(
      overtake_planner::kVehicleHardSteeringTireAngleRad);
  const auto config = make_hard_angle_config();
  overtake_planner::CandidateBuilder boundary_builder(boundary_frame, config);
  auto boundary_ego = makeEgo(boundary_frame, 5.0, 0.0);
  boundary_ego.v = 1.0;
  const auto boundary_candidate = boundary_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, boundary_ego, {}, {});
  EXPECT_TRUE(boundary_candidate.desired_path_trackable);
  EXPECT_TRUE(boundary_candidate.pure_pursuit_command_trackable);
  EXPECT_TRUE(boundary_candidate.controller_tracking_profile_valid);

  const auto over_hard_frame = make_constant_curvature_frame(
      std::nextafter(overtake_planner::kVehicleHardSteeringTireAngleRad,
                     std::numeric_limits<double>::infinity()));
  overtake_planner::CandidateBuilder over_hard_builder(over_hard_frame, config);
  auto over_hard_ego = makeEgo(over_hard_frame, 5.0, 0.0);
  over_hard_ego.v = 1.0;
  auto over_hard_candidate = over_hard_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, over_hard_ego, {}, {});
  EXPECT_FALSE(over_hard_candidate.desired_path_trackable);
  EXPECT_FALSE(over_hard_candidate.controller_tracking_profile_valid);

  overtake_planner::SafetyEvaluator evaluator(over_hard_frame, config);
  EXPECT_FALSE(evaluator.evaluate(over_hard_candidate, {}));
  EXPECT_EQ(over_hard_candidate.reject_reason, "untrackable_lateral_profile");
}

TEST(CandidateBuilder,
     PhysicalHardSteeringLimitRejectsGainAppliedPurePursuitBoundary) {
  constexpr double kWheelbaseM = 1.087;
  constexpr double kSteeringGain = 1.639;
  constexpr double kLookaheadM = 3.5;
  constexpr double kProfileLengthM = 8.0;
  constexpr double kLookaheadPointDistanceM = 4.0;
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.attack_follow_tracking_wheelbase_m = kWheelbaseM;
  config.attack_follow_max_steering_angle_rad =
      overtake_planner::kVehicleHardSteeringTireAngleRad;
  config.attack_follow_max_steering_rate_radps = 100.0;
  config.attack_follow_steering_tire_angle_gain = kSteeringGain;
  config.lateral_override_lookahead_min_distance_m = kLookaheadM;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto make_constant_d_profile = [] {
    overtake_planner::CandidateTrajectory candidate;
    candidate.type = overtake_planner::CandidateType::PASS_LEFT;
    candidate.required_controller_spatial_horizon_m = kProfileLengthM;
    for (int i = 0; i <= 160; ++i) {
      const double distance_m = 0.05 * static_cast<double>(i);
      candidate.longitudinal_offsets_m.push_back(distance_m);
      candidate.d.push_back(0.0);
      candidate.v_ref.push_back(0.0);
      candidate.predicted_speed_mps.push_back(0.0);
    }
    return candidate;
  };
  const auto make_ego_for_output_angle = [&](double output_angle_rad) {
    const double raw_angle_rad = output_angle_rad / kSteeringGain;
    const double alpha_rad =
        std::asin(std::tan(raw_angle_rad) * kLookaheadM / (2.0 * kWheelbaseM));
    auto ego = makeEgo(frame, 5.0, 0.0);
    ego.yaw = 0.0;
    const double lookahead_x_m = ego.frenet.s + kLookaheadPointDistanceM;
    // s=4.0 mのsampleが初めてchord=lookaheadとなるようrear axleを置く。
    // これでCandidateBuilderが評価するgain込みPP outputを境界値へ直接固定する。
    ego.x =
        lookahead_x_m - kLookaheadM * std::cos(alpha_rad) + 0.5 * kWheelbaseM;
    ego.y = -kLookaheadM * std::sin(alpha_rad);
    return ego;
  };

  for (const double sign : {1.0, -1.0}) {
    const auto trackable_at_abs_output_angle = [&](double abs_angle_rad) {
      return builder.publishedLateralProfileTrackable(
          make_constant_d_profile(),
          make_ego_for_output_angle(sign * abs_angle_rad));
    };
    // 同一のgain込みPP式で生成する入力角をhardの両側から二分探索する。
    // 実装側の比較はepsilonなしのまま維持する。fixture側の逆PP幾何は、
    // 0.64 rad境界ではsample chordの丸めを最大1e-6 rad含む。
    double accepted_angle_rad =
        overtake_planner::kVehicleHardSteeringTireAngleRad - 1.0e-6;
    double rejected_angle_rad =
        overtake_planner::kVehicleHardSteeringTireAngleRad + 1.0e-6;
    ASSERT_TRUE(trackable_at_abs_output_angle(accepted_angle_rad));
    ASSERT_FALSE(trackable_at_abs_output_angle(rejected_angle_rad));
    for (int iteration = 0; iteration < 48; ++iteration) {
      const double middle_angle_rad =
          0.5 * (accepted_angle_rad + rejected_angle_rad);
      if (trackable_at_abs_output_angle(middle_angle_rad)) {
        accepted_angle_rad = middle_angle_rad;
      } else {
        rejected_angle_rad = middle_angle_rad;
      }
    }
    EXPECT_NEAR(accepted_angle_rad,
                overtake_planner::kVehicleHardSteeringTireAngleRad, 1.0e-6);
    EXPECT_NEAR(rejected_angle_rad,
                overtake_planner::kVehicleHardSteeringTireAngleRad, 1.0e-6);
  }
}

TEST(CandidateBuilder,
     MovingPassRequiresBoundedRelativeCompletionAfterLateralSeparation) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.05;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.pass_target_policy = "minimum_clearance";
  config.safety_ellipse_b_m = 0.20;
  config.pass_target_lateral_margin_m = 0.05;
  config.prepare_distance_m = 6.0;
  config.merge_front_gap_m = 2.0;
  config.moving_pass_max_completion_time_sec = 10.0;
  config.moving_pass_min_closing_speed_mps = 0.05;
  config.moving_pass_reachability_enabled = true;

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 4.0;
  auto moving = makeOpponent(frame, 10.0, 0.0);
  moving.id = "moving_target";
  moving.v = 4.0;
  moving.vx = 4.0;
  moving.vy = 0.0;
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;
  blocked.pass_acceleration_allowed = true;

  auto unreachable_config = config;
  unreachable_config.pass_speed_cap_mps = 4.0;
  overtake_planner::CandidateBuilder unreachable_builder(frame,
                                                         unreachable_config);
  overtake_planner::SafetyEvaluator unreachable_evaluator(frame,
                                                          unreachable_config);
  auto unreachable = unreachable_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {moving});
  EXPECT_TRUE(unreachable.controller_tracking_profile_valid);
  EXPECT_FALSE(unreachable.moving_target_relatively_reachable);
  EXPECT_FALSE(unreachable_evaluator.evaluate(unreachable, {}));
  EXPECT_EQ(unreachable.reject_reason,
            "moving_target_not_relatively_reachable");

  auto reachable_config = config;
  reachable_config.pass_speed_cap_mps = 6.0;
  overtake_planner::CandidateBuilder reachable_builder(frame, reachable_config);
  overtake_planner::SafetyEvaluator reachable_evaluator(frame,
                                                        reachable_config);
  auto reachable = reachable_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {moving});
  EXPECT_TRUE(reachable.controller_tracking_profile_valid);
  EXPECT_TRUE(reachable.moving_target_relatively_reachable);
  EXPECT_TRUE(reachable_evaluator.evaluate(reachable, {}))
      << reachable.reject_reason;

  auto passed_ego = makeEgo(frame, 18.0, 1.0);
  passed_ego.v = 6.0;
  auto passed_target = moving;
  passed_target.frenet = frame.cartesianToFrenet(10.0, 0.0, 0.0);
  passed_target.x = 10.0;
  passed_target.y = 0.0;
  overtake_planner::LocalizedLateralProfile committed_profile;
  committed_profile.active = true;
  committed_profile.pass_type = overtake_planner::CandidateType::PASS_LEFT;
  committed_profile.target_id = passed_target.id;
  committed_profile.anchor_s_m = 5.0;
  committed_profile.ego_unwrapped_s_m = 18.0;
  committed_profile.last_ego_wrapped_s_m = 18.0;
  committed_profile.target_s_m = 10.0;
  committed_profile.avoid_start_s_m = 4.0;
  committed_profile.full_offset_start_s_m = 7.0;
  committed_profile.full_offset_end_s_m = 14.0;
  committed_profile.merge_end_s_m = 20.0;
  committed_profile.start_d_m = 0.0;
  committed_profile.target_d_m = 1.0;
  auto localized_config = reachable_config;
  localized_config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::CandidateBuilder localized_builder(frame, localized_config);
  auto completed_side = localized_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, passed_ego, blocked,
      {passed_target}, &committed_profile);
  EXPECT_TRUE(completed_side.moving_target_relatively_reachable)
      << "a committed target behind ego must not wrap to one lap ahead";

  auto low_speed_ego = makeEgo(frame, 5.0, 0.0);
  low_speed_ego.v = 0.1;
  auto low_speed_target = moving;
  low_speed_target.v = 0.2;
  low_speed_target.vx = 0.2;
  auto low_speed_config = config;
  low_speed_config.pass_speed_cap_mps = 0.1;
  overtake_planner::CandidateBuilder low_speed_builder(frame, low_speed_config);
  auto low_speed_unreachable = low_speed_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, low_speed_ego, blocked,
      {low_speed_target});
  EXPECT_FALSE(low_speed_unreachable.moving_target_relatively_reachable)
      << "positive low-speed targets still require bounded completion";

  auto stationary = moving;
  stationary.v = 0.0;
  stationary.vx = 0.0;
  auto stationary_pass = unreachable_builder.makeCandidate(
      overtake_planner::CandidateType::PASS_LEFT, ego, blocked, {stationary});
  EXPECT_TRUE(stationary_pass.moving_target_relatively_reachable);
}

TEST(CandidateBuilder,
     AcceleratingPassReservesExecutionSpeedBeforePurePursuitAdmission) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safe_stop_v_mps = 0.20;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.lateral_override_execution_speed_reserve_sec = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, -1.20);
  ego.v = 3.86;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {});

  const double execution_speed_mps =
      ego.v + config.pass_assumed_accel_mps2 *
                  config.lateral_override_execution_speed_reserve_sec;
  const double required_brake_arc_m =
      execution_speed_mps * 0.25 +
      (execution_speed_mps * execution_speed_mps -
       config.safe_stop_v_mps * config.safe_stop_v_mps) /
          (2.0 * config.max_brake_decel_mps2);
  ASSERT_FALSE(pass.longitudinal_offsets_m.empty());
  EXPECT_GT(required_brake_arc_m, 8.5);
  EXPECT_GE(pass.longitudinal_offsets_m.back(), required_brake_arc_m - 1.0e-6);
  EXPECT_TRUE(pass.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     ExecutionSpeedReserveKeepsResidualAccelerationAboveDeceleratingTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.safe_stop_v_mps = 0.20;
  config.pass_speed_cap_mps = 2.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.lateral_override_lookahead_gain = 0.0;
  config.lateral_override_lookahead_min_distance_m = 0.5;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.lateral_override_execution_speed_reserve_sec = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, -1.20);
  ego.v = 4.0;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;
  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {});

  const double reserved_speed_mps =
      ego.v + config.pass_assumed_accel_mps2 *
                  config.lateral_override_execution_speed_reserve_sec;
  const double required_reserved_speed_brake_arc_m =
      reserved_speed_mps * 0.25 +
      (reserved_speed_mps * reserved_speed_mps -
       config.safe_stop_v_mps * config.safe_stop_v_mps) /
          (2.0 * config.max_brake_decel_mps2);
  ASSERT_FALSE(pass.longitudinal_offsets_m.empty());
  EXPECT_GE(pass.longitudinal_offsets_m.back(),
            required_reserved_speed_brake_arc_m - 1.0e-6);
  ASSERT_FALSE(pass.predicted_speed_mps.empty());
  EXPECT_NEAR(pass.predicted_speed_mps.front(), reserved_speed_mps, 1.0e-9);
  EXPECT_TRUE(pass.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     LateralFirstPassCapUsesSameDelayedBrakingProfileAsPublishedSpeed) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  config.lateral_override_lookahead_gain = 0.5;
  config.lateral_override_lookahead_min_distance_m = 3.5;
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  config.lateral_override_execution_speed_reserve_sec = 0.10;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, -1.20);
  ego.v = 4.0;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;
  blocked.pass_lateral_first_speed_gate_active = true;
  blocked.pass_lateral_first_speed_cap_mps = 2.0;

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {});

  ASSERT_FALSE(pass.v_ref.empty());
  ASSERT_FALSE(pass.predicted_speed_mps.empty());
  EXPECT_TRUE(
      std::all_of(pass.v_ref.begin(), pass.v_ref.end(), [](double speed_mps) {
        return std::abs(speed_mps - 2.0) <= 1.0e-9;
      }));
  const double reserved_speed_mps =
      ego.v + config.pass_assumed_accel_mps2 *
                  config.lateral_override_execution_speed_reserve_sec;
  EXPECT_NEAR(pass.predicted_speed_mps.front(), reserved_speed_mps, 1.0e-9);
  EXPECT_NEAR(pass.predicted_speed_mps.back(), 2.0, 1.0e-6);
  EXPECT_NEAR(pass.assumed_brake_decel_mps2, config.max_brake_decel_mps2,
              1.0e-9);
  EXPECT_NEAR(pass.response_delay_sec, config.longitudinal_response_delay_sec,
              1.0e-9);
  EXPECT_GT(pass.required_brake_distance_m, 0.0);
  EXPECT_TRUE(pass.longitudinal_profile_valid);
  EXPECT_TRUE(pass.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     LateralFirstPassCapDoesNotRemainAfterActualClearanceIsReady) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.pass_speed_cap_mps = 6.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 5.0, -1.20);
  ego.v = 2.0;
  overtake_planner::BlockedInfo blocked;
  blocked.pass_acceleration_allowed = true;
  blocked.pass_lateral_first_speed_gate_active = false;
  blocked.pass_lateral_clearance_ready = true;
  blocked.pass_lateral_first_speed_cap_mps = 2.0;

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {});

  ASSERT_FALSE(pass.v_ref.empty());
  EXPECT_NEAR(pass.v_ref.front(), config.pass_speed_cap_mps, 1.0e-9);
  EXPECT_GT(pass.predicted_speed_mps.back(), ego.v);
}

TEST(OvertakePlannerCore,
     PassLateralFirstGateTracksFreshTargetUntilActualSeparationIsReady) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.safety_ellipse_b_m = 0.50;
  config.min_ellipse_h = 0.20;
  config.pass_target_lateral_margin_m = 0.10;
  config.pass_lateral_first_stationary_creep_v_max_mps = 0.75;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 3.0;
  auto target = makeOpponent(frame, 13.0, 0.0);
  target.id = "d2";
  target.v = 2.0;
  target.vx = 2.0;
  target.stamp_sec = 0.1;

  const auto approaching =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  const double required_separation_m =
      config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
      config.pass_target_lateral_margin_m;
  EXPECT_EQ(approaching.blocked_info.pass_lateral_first_target_id, "d2");
  EXPECT_TRUE(approaching.blocked_info.pass_lateral_first_speed_gate_active);
  EXPECT_FALSE(approaching.blocked_info.pass_lateral_clearance_ready);
  EXPECT_NEAR(approaching.blocked_info.pass_lateral_separation_actual_m, 0.0,
              1.0e-9);
  EXPECT_NEAR(approaching.blocked_info.pass_lateral_separation_required_m,
              required_separation_m, 1.0e-9);
  EXPECT_NEAR(approaching.blocked_info.pass_lateral_first_target_speed_mps,
              target.v, 1.0e-9);
  EXPECT_NEAR(approaching.blocked_info.pass_lateral_first_speed_cap_mps,
              target.v, 1.0e-9);

  ego.x = 5.2;
  ego.y = required_separation_m + 0.05;
  ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
  target.stamp_sec = 0.2;
  const auto separated =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_EQ(separated.blocked_info.pass_lateral_first_target_id, "d2");
  EXPECT_TRUE(separated.blocked_info.pass_lateral_clearance_ready);
  EXPECT_FALSE(separated.blocked_info.pass_lateral_first_speed_gate_active);
  EXPECT_GT(separated.blocked_info.pass_lateral_separation_actual_m,
            separated.blocked_info.pass_lateral_separation_required_m);
}

TEST(OvertakePlannerCore,
     StationaryPassLateralFirstGateUsesOnlyConfiguredBoundedCreep) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_target_policy = "minimum_clearance";
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.safety_ellipse_b_m = 0.10;
  config.pass_lateral_first_stationary_creep_v_max_mps = 0.75;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 0.0;
  auto target = makeOpponent(frame, 12.5, 0.0);
  target.id = "d2";
  target.v = 0.0;
  target.vx = 0.0;
  target.stamp_sec = 0.1;

  const auto output =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_EQ(output.blocked_info.pass_lateral_first_target_id, "d2");
  EXPECT_TRUE(output.blocked_info.pass_lateral_first_speed_gate_active);
  EXPECT_NEAR(output.blocked_info.pass_lateral_first_target_speed_mps, 0.0,
              1.0e-9);
  EXPECT_NEAR(output.blocked_info.pass_lateral_first_speed_cap_mps, 0.75,
              1.0e-9);
  EXPECT_TRUE(output.blocked_info.pass_acceleration_allowed);
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_TRUE(
      std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                  [](double speed_mps) { return speed_mps <= 0.75 + 1.0e-9; }));
}

TEST(CandidateBuilder, InvalidExecutionSpeedReserveFailsClosed) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lateral_override_execution_speed_reserve_sec = 0.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, makeEgo(frame, 5.0, -1.20),
      overtake_planner::BlockedInfo{}, {});

  EXPECT_FALSE(pass.longitudinal_profile_valid);
  EXPECT_FALSE(pass.controller_tracking_profile_valid);
}

TEST(CandidateBuilder,
     VerifiedLowSpeedPassPublishesSameAcceleratingProfileEvaluatedForSafety) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.horizon_points = 50;
  config.horizon_dt_sec = 0.025;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.1;
  config.pass_speed_cap_mps = 10.0;
  config.pass_assumed_accel_mps2 = 3.0;
  overtake_planner::CandidateBuilder builder(frame, config);

  auto ego = makeEgo(frame, 34.5, -0.89);
  ego.v = 0.17;
  auto target = makeOpponent(frame, 40.5, 0.0);
  target.id = "d3";
  overtake_planner::BlockedInfo blocked;
  blocked.nearest_index = 0;
  blocked.nearest_id = target.id;
  blocked.pass_acceleration_allowed = true;

  const auto pass = builder.makeCandidate(
      overtake_planner::CandidateType::PASS_RIGHT, ego, blocked, {target});
  ASSERT_FALSE(pass.longitudinal_offsets_m.empty());
  ASSERT_EQ(pass.longitudinal_offsets_m.size(), pass.v_ref.size());
  ASSERT_EQ(pass.longitudinal_offsets_m.size(),
            pass.predicted_speed_mps.size());
  EXPECT_TRUE(pass.longitudinal_profile_valid);
  EXPECT_GT(pass.longitudinal_offsets_m.back(), 0.50);
  EXPECT_GT(pass.v_ref.front(), ego.v);
  EXPECT_GT(pass.predicted_speed_mps.back(), ego.v);
}

TEST(OvertakePlannerCore,
     LocalizedLatchedPassPublishesEvaluatedSpatialProfileWithoutIndexMixing) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_horizon_publish_mode = "overtake_only";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.lateral_target_max_step_m = 0.05;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto target = makeOpponent(frame, 13.0, -0.6);
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_FALSE(prepare.lateral_offsets.empty());

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_EQ(passing.selected, overtake_planner::CandidateType::PASS_LEFT);
  ASSERT_TRUE(passing.maneuver_latch_active);
  ASSERT_FALSE(passing.lateral_offsets.empty());
  EXPECT_FALSE(passing.published_lateral_safety_rejected);
  // localized profileはsに対して滑らかで、先頭点はego dに連続している。
  // 旧index rate limitの0.05 mへ全horizonを潰さず、SafetyEvaluatorが通した
  // 空間profileの将来オフセットを保持する。
  EXPECT_NEAR(passing.lateral_offsets.front(), ego.frenet.d, 1.0e-9);
  EXPECT_GT(passing.lateral_offsets.back(),
            prepare.lateral_offsets.back() + config.lateral_target_max_step_m +
                1.0e-6);
}

TEST(OvertakePlannerCore,
     LatchedPassTracksFreshTargetLongitudinallyWithoutChangingIdSideOrD) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_target_update_alpha = 1.0;
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_TRUE(passing.blocked_info.maneuver_transaction_incomplete);
  const double approved_target_d_m =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(approved_target_d_m));

  // 相手が前進してもtarget ID・PASS側・認可済みdは固定し、s markerと
  // 同じIDのchain tailだけをfresh観測へ追従させる。旧実装ではtarget dsは
  // 9.2 mなのにchain-tail dsだけ6.7 mへ残り、実走では差が約14 mまで拡大した。
  target = makeOpponent(frame, 15.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.3;
  const auto moved =
      core.update(0.3, makeEgo(frame, 5.8, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_EQ(moved.maneuver_latch_target_id, "d2");
  EXPECT_EQ(moved.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NEAR(moved.blocked_info.pass_left_candidate_target_d_m,
              approved_target_d_m, 1.0e-9);
  EXPECT_NEAR(moved.maneuver_latch_target_s_m, 15.0, 1.0e-9);
  EXPECT_EQ(moved.maneuver_latch_last_waypoint_id, "d2");
  EXPECT_NEAR(moved.maneuver_latch_last_waypoint_s_m, 15.0, 1.0e-9);
  EXPECT_EQ(moved.blocked_info.maneuver_chain_tail_id, "d2");
  EXPECT_NEAR(moved.blocked_info.maneuver_target_relative_s_m, 9.2, 1.0e-9);
  EXPECT_NEAR(moved.blocked_info.maneuver_chain_tail_relative_s_m,
              moved.blocked_info.maneuver_target_relative_s_m, 1.0e-9);
  EXPECT_TRUE(moved.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     LatchedPassKeepsContinuousRelativeDistanceAcrossMoreThanOneLap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_target_update_alpha = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_TRUE(passing.blocked_info.maneuver_transaction_incomplete);

  const std::vector<std::pair<double, double>> wrapped_positions{
      {25.0, 33.0}, {45.0, 53.0}, {65.0, 73.0}, {5.0, 13.0},
      {25.0, 33.0}, {45.0, 53.0}, {65.0, 73.0}, {5.0, 13.0}};
  overtake_planner::PlannerOutput continued;
  double stamp_sec = 0.3;
  const auto make_wrapped_ego = [&frame](double s_m) {
    auto ego = makeEgo(frame, 0.0, 0.0);
    const auto pose = frame.frenetToCartesian(s_m, 0.0);
    ego.x = pose.x;
    ego.y = pose.y;
    ego.yaw = pose.yaw;
    ego.frenet = overtake_planner::FrenetPose{s_m, 0.0};
    return ego;
  };
  const auto make_wrapped_opponent = [&frame](double s_m) {
    auto opponent = makeOpponent(frame, 0.0, -0.6);
    const auto pose = frame.frenetToCartesian(s_m, -0.6);
    opponent.x = pose.x;
    opponent.y = pose.y;
    opponent.frenet = overtake_planner::FrenetPose{s_m, -0.6};
    opponent.vx = opponent.v * std::cos(pose.yaw);
    opponent.vy = opponent.v * std::sin(pose.yaw);
    return opponent;
  };
  std::size_t update_index = 0U;
  for (const auto &[ego_s_m, target_s_m] : wrapped_positions) {
    SCOPED_TRACE(::testing::Message()
                 << "update_index=" << update_index << " ego_s_m=" << ego_s_m
                 << " target_s_m=" << target_s_m);
    auto ego = make_wrapped_ego(ego_s_m);
    auto observed = make_wrapped_opponent(target_s_m);
    observed.id = "d2";
    observed.stamp_sec = stamp_sec;
    continued =
        core.update(stamp_sec, ego, {observed},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    SCOPED_TRACE(
        ::testing::Message()
        << "relative_s=" << continued.blocked_info.maneuver_target_relative_s_m
        << " geometric_complete="
        << continued.blocked_info.maneuver_target_pass_geometric_complete
        << " pass_complete="
        << continued.blocked_info.maneuver_target_pass_complete
        << " change_reason="
        << continued.blocked_info.maneuver_target_change_reason
        << " mode=" << static_cast<int>(continued.mode));
    ASSERT_TRUE(continued.maneuver_latch_active);
    ASSERT_TRUE(continued.blocked_info.maneuver_transaction_incomplete);
    stamp_sec += 0.1;
    ++update_index;
  }

  EXPECT_EQ(continued.maneuver_latch_target_id, "d2");
  EXPECT_NEAR(continued.maneuver_latch_target_s_m, 173.0, 1.0e-6);
  EXPECT_NEAR(continued.maneuver_latch_last_waypoint_s_m, 173.0, 1.0e-6);
  EXPECT_NEAR(continued.blocked_info.maneuver_target_relative_s_m, 8.0, 1.0e-6);
  EXPECT_NEAR(continued.blocked_info.maneuver_chain_tail_relative_s_m, 8.0,
              1.0e-6);
  EXPECT_FALSE(continued.blocked_info.maneuver_target_pass_geometric_complete);
}

TEST(OvertakePlannerCore,
     LatchedPassTracksTargetThatPullsMoreThanHalfLapAhead) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_target_update_alpha = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_TRUE(passing.blocked_info.maneuver_transaction_incomplete);

  const auto make_wrapped_ego = [&frame](double s_m) {
    auto ego = makeEgo(frame, 0.0, 0.0);
    const auto pose = frame.frenetToCartesian(s_m, 0.0);
    ego.x = pose.x;
    ego.y = pose.y;
    ego.yaw = pose.yaw;
    ego.frenet = overtake_planner::FrenetPose{s_m, 0.0};
    return ego;
  };
  const auto make_wrapped_opponent = [&frame](double s_m) {
    auto opponent = makeOpponent(frame, 0.0, -0.6);
    const auto pose = frame.frenetToCartesian(s_m, -0.6);
    opponent.x = pose.x;
    opponent.y = pose.y;
    opponent.frenet = overtake_planner::FrenetPose{s_m, -0.6};
    opponent.vx = opponent.v * std::cos(pose.yaw);
    opponent.vy = opponent.v * std::sin(pose.yaw);
    return opponent;
  };
  const std::vector<double> ego_positions{7.0,  9.0,  11.0, 13.0,
                                          15.0, 17.0, 19.0, 21.0};
  const std::vector<double> target_positions{33.0, 53.0, 73.0, 13.0,
                                             33.0, 53.0, 73.0, 13.0};
  overtake_planner::PlannerOutput continued;
  double stamp_sec = 0.3;
  for (std::size_t i = 0; i < ego_positions.size(); ++i) {
    auto ego = make_wrapped_ego(ego_positions[i]);
    auto observed = make_wrapped_opponent(target_positions[i]);
    observed.id = "d2";
    observed.stamp_sec = stamp_sec;
    continued =
        core.update(stamp_sec, ego, {observed},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    ASSERT_TRUE(continued.maneuver_latch_active);
    ASSERT_TRUE(continued.blocked_info.maneuver_transaction_incomplete);
    stamp_sec += 0.1;
  }

  EXPECT_EQ(continued.maneuver_latch_target_id, "d2");
  EXPECT_NEAR(continued.maneuver_latch_target_s_m, 173.0, 1.0e-6);
  EXPECT_NEAR(continued.maneuver_latch_last_waypoint_s_m, 173.0, 1.0e-6);
  EXPECT_NEAR(continued.blocked_info.maneuver_target_relative_s_m, 152.0,
              1.0e-6);
  EXPECT_FALSE(continued.blocked_info.maneuver_target_pass_geometric_complete);
}

TEST(OvertakePlannerCore,
     UnstartedCommittedPassReleasesFreshTargetThatKeepsPullingAway) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_target_update_alpha = 1.0;
  config.unstarted_pass_target_release_enabled = true;
  config.unstarted_pass_target_release_min_gap_m = 8.0;
  config.unstarted_pass_target_release_min_opening_speed_mps = 1.0;
  config.unstarted_pass_target_release_max_lateral_progress_m = 0.10;
  config.unstarted_pass_target_release_required_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_TRUE(passing.blocked_info.maneuver_transaction_incomplete);
  ASSERT_TRUE(passing.maneuver_latch_active);

  auto pulling_target = makeOpponent(frame, 15.0, -0.6);
  pulling_target.id = "d2";
  pulling_target.v = 6.0;
  pulling_target.vx = 6.0;
  pulling_target.stamp_sec = 0.3;
  auto unstarted_ego = makeEgo(frame, 5.7, 0.0);
  const auto first =
      core.update(0.3, unstarted_ego, {pulling_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_TRUE(first.maneuver_latch_active);
  EXPECT_TRUE(first.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(first.blocked_info.maneuver_unstarted_target_released);
  EXPECT_EQ(first.blocked_info.maneuver_unstarted_target_pulling_away_cycles,
            1);

  pulling_target.x = 15.6;
  pulling_target.frenet =
      frame.cartesianToFrenet(pulling_target.x, pulling_target.y, 0.0);
  pulling_target.stamp_sec = 0.4;
  unstarted_ego.x = 5.9;
  unstarted_ego.frenet = frame.cartesianToFrenet(
      unstarted_ego.x, unstarted_ego.y, unstarted_ego.yaw);
  const auto released =
      core.update(0.4, unstarted_ego, {pulling_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_FALSE(released.maneuver_latch_active);
  EXPECT_FALSE(released.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(released.blocked_info.maneuver_unstarted_target_released);
  EXPECT_EQ(released.blocked_info.maneuver_target_previous_id, "d2");
  EXPECT_TRUE(released.blocked_info.maneuver_target_id.empty());
  EXPECT_EQ(released.blocked_info.maneuver_target_change_reason,
            "unstarted_pass_target_pulled_away_handoff");
  EXPECT_TRUE(released.blocked_info.pass_decision_frozen);
  EXPECT_NE(released.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(released.selected, overtake_planner::CandidateType::PASS_RIGHT);

  // 同じ対象がinteraction外へ離れ続ける間は、次周期に新しいPASSとして
  // 再ラッチしない。release/re-latchのgenerationチャタリングを禁止する。
  for (int cycle = 0; cycle < 3; ++cycle) {
    const double stamp_sec = 0.5 + 0.1 * static_cast<double>(cycle);
    pulling_target.x = 15.6 + 0.2 * static_cast<double>(cycle);
    pulling_target.frenet =
        frame.cartesianToFrenet(pulling_target.x, pulling_target.y, 0.0);
    pulling_target.stamp_sec = stamp_sec;
    unstarted_ego.x = 6.1 + 0.2 * static_cast<double>(cycle);
    unstarted_ego.frenet = frame.cartesianToFrenet(
        unstarted_ego.x, unstarted_ego.y, unstarted_ego.yaw);
    const auto suppressed =
        core.update(stamp_sec, unstarted_ego, {pulling_target},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
    EXPECT_FALSE(suppressed.maneuver_latch_active);
    EXPECT_FALSE(suppressed.blocked_info.maneuver_transaction_incomplete);
    EXPECT_TRUE(
        suppressed.blocked_info.maneuver_unstarted_target_reacquire_suppressed);
    EXPECT_EQ(suppressed.blocked_info.maneuver_target_previous_id, "d2");
    EXPECT_TRUE(suppressed.blocked_info.maneuver_target_id.empty());
    EXPECT_EQ(suppressed.blocked_info.maneuver_target_change_reason,
              "unstarted_pass_released_target_reacquire_suppressed");
    EXPECT_NE(suppressed.selected, overtake_planner::CandidateType::PASS_LEFT);
    EXPECT_NE(suppressed.selected, overtake_planner::CandidateType::PASS_RIGHT);
  }

  // 永久blacklistにはしない。同じIDでも相対速度がcatch可能へ戻れば、
  // 通常のGate 2から新規transactionとして再評価できる。
  pulling_target.v = 0.0;
  pulling_target.vx = 0.0;
  pulling_target.x = 15.0;
  pulling_target.frenet =
      frame.cartesianToFrenet(pulling_target.x, pulling_target.y, 0.0);
  pulling_target.stamp_sec = 0.8;
  unstarted_ego.x = 6.7;
  unstarted_ego.frenet = frame.cartesianToFrenet(
      unstarted_ego.x, unstarted_ego.y, unstarted_ego.yaw);
  const auto reacquired =
      core.update(0.8, unstarted_ego, {pulling_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_FALSE(
      reacquired.blocked_info.maneuver_unstarted_target_reacquire_suppressed);
  EXPECT_TRUE(reacquired.maneuver_latch_active);
  EXPECT_EQ(reacquired.maneuver_latch_target_id, "d2");
}

TEST(OvertakePlannerCore,
     PullingAwayTargetRemainsLatchedAfterPhysicalPassSideProgress) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_target_update_alpha = 1.0;
  config.unstarted_pass_target_release_enabled = true;
  config.unstarted_pass_target_release_min_gap_m = 8.0;
  config.unstarted_pass_target_release_min_opening_speed_mps = 1.0;
  config.unstarted_pass_target_release_max_lateral_progress_m = 0.10;
  config.unstarted_pass_target_release_required_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d2";
  target.stamp_sec = 0.1;
  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  target.stamp_sec = 0.2;
  ASSERT_EQ(core.update(0.2, makeEgo(frame, 5.5, 0.0), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);

  auto pulling_target = makeOpponent(frame, 15.0, -0.6);
  pulling_target.id = "d2";
  pulling_target.v = 6.0;
  pulling_target.vx = 6.0;
  auto committed_ego = makeEgo(frame, 5.7, 0.25);
  overtake_planner::PlannerOutput continued;
  for (int cycle = 0; cycle < 4; ++cycle) {
    const double stamp_sec = 0.3 + 0.1 * static_cast<double>(cycle);
    pulling_target.x = 15.0 + 0.6 * static_cast<double>(cycle);
    pulling_target.frenet =
        frame.cartesianToFrenet(pulling_target.x, pulling_target.y, 0.0);
    pulling_target.stamp_sec = stamp_sec;
    committed_ego.x = 5.7 + 0.2 * static_cast<double>(cycle);
    committed_ego.frenet = frame.cartesianToFrenet(
        committed_ego.x, committed_ego.y, committed_ego.yaw);
    continued =
        core.update(stamp_sec, committed_ego, {pulling_target},
                    overtake_planner::MpcHealthStatus{}, readyReentryInput());
  }

  EXPECT_TRUE(continued.maneuver_latch_active);
  EXPECT_TRUE(continued.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(continued.blocked_info.maneuver_unstarted_target_released);
  EXPECT_EQ(
      continued.blocked_info.maneuver_unstarted_target_pulling_away_cycles, 0);
  EXPECT_GT(continued.blocked_info.maneuver_pass_lateral_progress_m, 0.10);
  EXPECT_EQ(continued.maneuver_latch_target_id, "d2");
  EXPECT_EQ(continued.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
}

TEST(OvertakePlannerCore,
     LocalizedPassHandsOffTargetSequentiallyWithoutMergingInsideSlowChain) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.merge_front_gap_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 13.0, -0.6);
  d2.id = "d2";
  d2.vx = 0.0;
  d2.v = 0.0;
  auto d3 = makeOpponent(frame, 20.0, -0.6);
  d3.id = "d3";
  d3.vx = 0.0;
  d3.v = 0.0;
  auto d4 = makeOpponent(frame, 27.0, -0.6);
  d4.id = "d4";
  d4.vx = 0.0;
  d4.v = 0.0;
  auto stamp_targets = [&](double stamp_sec) {
    d2.stamp_sec = stamp_sec;
    d3.stamp_sec = stamp_sec;
    d4.stamp_sec = stamp_sec;
    return std::vector<overtake_planner::OpponentState>{d2, d3, d4};
  };
  const auto input = readyReentryInput();

  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), stamp_targets(0.1),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepare.maneuver_latch_active);
  EXPECT_EQ(prepare.maneuver_latch_target_id, "d2");
  EXPECT_EQ(prepare.blocked_info.maneuver_chain_tail_id, "d4");
  EXPECT_EQ(prepare.blocked_info.maneuver_chain_target_count, 3);
  EXPECT_TRUE(prepare.blocked_info.maneuver_chain_tail_observed);
  EXPECT_NEAR(prepare.maneuver_latch_full_offset_end_s_m, 31.0, 1.0e-9);
  EXPECT_NEAR(prepare.maneuver_latch_merge_end_s_m, 37.0, 1.0e-9);

  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), stamp_targets(0.2),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);

  // d2を6 m以上抜いた時点で、この周期まではtarget ID=d2を保持する。
  // d3/d4用waypointは既に同じ側へ延長済みなので、MERGE_BACKへ入らない。
  const auto completed_d2 =
      core.update(0.3, makeEgo(frame, 19.1, 0.0), stamp_targets(0.3),
                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(completed_d2.maneuver_latch_target_id, "d2");
  EXPECT_EQ(completed_d2.blocked_info.maneuver_chain_tail_id, "d4");
  EXPECT_NEAR(completed_d2.blocked_info.maneuver_target_relative_s_m, -6.1,
              1.0e-9);
  EXPECT_NEAR(completed_d2.blocked_info.maneuver_chain_tail_relative_s_m, 7.9,
              1.0e-9);
  EXPECT_TRUE(
      completed_d2.blocked_info.maneuver_target_pass_geometric_complete);
  EXPECT_TRUE(completed_d2.blocked_info.maneuver_target_pass_complete);
  // この人工入力は1周期で13.6 mテレポートするため、tracking envelope側が
  // YIELDを選ぶことは許す。ただし列の途中で中心へmergeしてはならない。
  EXPECT_NE(completed_d2.mode, overtake_planner::BehaviorMode::MERGE_BACK);

  // 次周期にだけd3へ張り替え、同じPASS_LEFTとstaged profileを継続する。
  const auto handed_off =
      core.update(0.4, makeEgo(frame, 19.2, 0.0), stamp_targets(0.4),
                  overtake_planner::MpcHealthStatus{}, input);
  EXPECT_EQ(handed_off.maneuver_latch_target_id, "d3");
  EXPECT_EQ(handed_off.blocked_info.maneuver_chain_tail_id, "d4");
  EXPECT_EQ(handed_off.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_NE(handed_off.mode, overtake_planner::BehaviorMode::MERGE_BACK);
}

TEST(OvertakePlannerCore,
     LocalizedPassDynamicallyExtendsLaterallyStaggeredSlowChain) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.same_corridor_width_m = 0.90;
  config.parallel_side_margin_m = 4.0;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 6.0;
  config.localized_avoidance_full_offset_before_target_m = 2.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 13.0, 0.8);
  d2.id = "d2";
  d2.vx = 0.0;
  d2.v = 0.0;
  auto d3 = makeOpponent(frame, 20.0, 0.0);
  d3.id = "d3";
  d3.vx = 0.0;
  d3.v = 0.0;
  auto d4 = makeOpponent(frame, 27.0, -0.8);
  d4.id = "d4";
  d4.vx = 0.0;
  d4.v = 0.0;
  const auto input = readyReentryInput();

  d2.stamp_sec = 0.1;
  d3.stamp_sec = 0.1;
  const auto initial = core.update(0.1, makeEgo(frame, 5.0, 0.8), {d2, d3},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(initial.maneuver_latch_active);
  ASSERT_EQ(initial.maneuver_latch_target_id, "d2");
  ASSERT_EQ(initial.blocked_info.maneuver_chain_tail_id, "d3");
  ASSERT_EQ(initial.blocked_info.maneuver_chain_target_count, 2);

  // d4はd2から1.6 m離れてsame_corridor_widthを超えるが、d3との隣接差は
  // 0.8 mである。後からfresh観測へ入ってもtarget/sideを変えず列末尾を延長する。
  d2.stamp_sec = 0.2;
  d3.stamp_sec = 0.2;
  d4.stamp_sec = 0.2;
  const auto extended = core.update(0.2, makeEgo(frame, 5.2, 0.8), {d2, d3, d4},
                                    overtake_planner::MpcHealthStatus{}, input);

  EXPECT_EQ(extended.maneuver_latch_target_id, "d2");
  EXPECT_EQ(extended.blocked_info.maneuver_chain_tail_id, "d4");
  EXPECT_EQ(extended.blocked_info.maneuver_chain_target_count, 3);
  EXPECT_TRUE(extended.blocked_info.maneuver_chain_tail_observed);
  EXPECT_NEAR(extended.maneuver_latch_full_offset_end_s_m, 31.0, 1.0e-9);
  EXPECT_NEAR(extended.maneuver_latch_merge_end_s_m, 37.0, 1.0e-9);
}

TEST(OvertakePlannerCore,
     GentleCurveDynamicStartResetsWhenRetainedChainTailDisappears) {
  const auto frame = makeD1RepresentativeCurveFrame();
  auto config = makeD1RepresentativeCurveConfig();
  config.gentle_curve_safe_pass_max_lateral_displacement_m = 2.60;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_required_cycles = 1;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 32.86, 0.68);
  ego.stamp_sec = 0.1;
  ego.v = 0.19;
  auto d2 = makeOpponent(frame, 38.95, 0.18);
  d2.id = "d2";
  d2.stamp_sec = 0.1;
  d2.v = 0.5;
  d2.vx = 0.5;
  auto d3 = makeOpponent(frame, 46.0, 0.18);
  d3.id = "d3";
  d3.stamp_sec = 0.1;
  d3.v = 0.5;
  d3.vx = 0.5;

  const auto latched =
      core.update(0.1, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_TRUE(latched.maneuver_latch_active);
  ASSERT_EQ(latched.maneuver_latch_target_id, "d2");
  ASSERT_EQ(latched.blocked_info.maneuver_chain_tail_id, "d3");
  ASSERT_TRUE(latched.blocked_info.maneuver_chain_tail_observed);

  // profileはd3まで保持されたまま、次周期はheadのd2だけが動的targetとして
  // 観測される状況を再現する。chain tailを見失った状態でhead IDだけを照合し、
  // 新規PASSのsafe-cycleを進めてはならない。
  ego.stamp_sec = 0.2;
  d2.stamp_sec = 0.2;
  d2.v = 2.25;
  d2.vx = 2.25;
  const auto tail_missing = core.update(
      0.2, ego, {d2}, overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_TRUE(tail_missing.maneuver_latch_active);
  EXPECT_EQ(tail_missing.maneuver_latch_target_id, "d2");
  EXPECT_EQ(tail_missing.blocked_info.maneuver_chain_tail_id, "d3");
  EXPECT_FALSE(tail_missing.blocked_info.maneuver_chain_tail_observed);
  EXPECT_FALSE(tail_missing.blocked_info.gentle_curve_target_identity_valid);
  EXPECT_FALSE(tail_missing.blocked_info.gentle_curve_fresh_dynamic_gap_target);
  EXPECT_FALSE(tail_missing.blocked_info.gentle_curve_safe_pass_eligible);
  EXPECT_FALSE(tail_missing.blocked_info.gentle_curve_safe_pass_start_approved);
  EXPECT_EQ(tail_missing.blocked_info.pass_right_safe_cycles, 0);
  EXPECT_NE(tail_missing.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);
}

TEST(OvertakePlannerCore,
     SafetyApprovedPassKeepsPassSideAndTargetAcrossClassifierHandoff) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 4.0;
  config.localized_avoidance_full_offset_before_target_m = 1.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.merge_front_gap_m = 6.0;
  config.lateral_target_max_step_m = 0.01;
  config.reentry_gate_enabled = true;
  // このfixtureはPASS取引の継続契約を検証する。既定の狭い固定回廊で
  // 意図的に外側へ置いたegoがpreflight失格にならないよう、回廊条件を分離する。
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 10.0, 0.0);
  d2.id = "d2";
  d2.v = 0.0;
  d2.vx = 0.0;
  auto d3 = makeOpponent(frame, 19.0, 0.0);
  d3.id = "d3";
  d3.v = 0.0;
  d3.vx = 0.0;
  auto stamped_targets = [&](double stamp_sec) {
    d2.stamp_sec = stamp_sec;
    d3.stamp_sec = stamp_sec;
    return std::vector<overtake_planner::OpponentState>{d2, d3};
  };
  const auto input = readyReentryInput();

  const auto prepare =
      core.update(0.1, makeEgo(frame, 5.0, 0.0), stamped_targets(0.1),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepare.maneuver_latch_active);
  ASSERT_EQ(prepare.maneuver_latch_target_id, "d2");

  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), stamped_targets(0.2),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  ASSERT_FALSE(passing.lateral_offsets.empty());
  const double latched_pass_d =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(latched_pass_d));
  ASSERT_GT(latched_pass_d, 0.0);

  // d2は横安全間隔を保って抜いたが、同じ車列のd3はまだ前方にいる。
  // chain tailが残るだけでSafetyEvaluator余裕のあるPASSをFOLLOWへ落とさず、
  // d2のIDとPASS側dを保持して列全体を抜き続ける。
  auto low_speed_handoff_ego = makeEgo(frame, 14.5, latched_pass_d + 0.40);
  low_speed_handoff_ego.v = 0.20;
  const auto continued =
      core.update(0.3, low_speed_handoff_ego, stamped_targets(0.3),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(continued.blocked_info.maneuver_chain_tail_observed);
  ASSERT_EQ(continued.blocked_info.maneuver_chain_tail_id, "d3");
  ASSERT_LT(continued.blocked_info.maneuver_target_relative_s_m, 0.0);
  ASSERT_GT(std::abs(continued.blocked_info.maneuver_target_relative_d_m),
            config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h));
  ASSERT_EQ(continued.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(continued.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(continued.maneuver_latch_target_id, "d2");
  EXPECT_TRUE(continued.blocked_info.maneuver_transaction_incomplete);
  EXPECT_EQ(continued.blocked_info.maneuver_transaction_pass_type,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(continued.blocked_info.pass_left_candidate_feasible);
  EXPECT_GT(continued.blocked_info.maneuver_target_pass_min_safety_margin,
            config.min_ellipse_h + 0.10);
  EXPECT_TRUE(continued.raw_selected_feasible);
  EXPECT_FALSE(continued.reentry_gate.requested);
  ASSERT_FALSE(continued.lateral_offsets.empty());
  EXPECT_GT(continued.lateral_offsets.back(), 0.0);
}

TEST(OvertakePlannerCore,
     PassChainBrakingDistanceKeepsSafetyApprovedPassWithMarginReserve) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  config.safety_ellipse_b_m = 0.5;
  config.min_ellipse_h = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 4.0;
  config.localized_avoidance_full_offset_before_target_m = 1.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_max_target_speed_mps = 1.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  // active PPが検証済みとみなす最大減速度と同じ契約にする。
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  // このfixtureは短い予測で制動不足になるFOLLOWと、安全なPASSの分離を
  // 確認する。実運用20秒horizonでFOLLOW自体が成立するケースとは分ける。
  config.lateral_override_max_evaluation_horizon_sec = 6.0;
  // 制動距離handoffだけを検証し、固定回廊preflightの失格と混ぜない。
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.reentry_gate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 10.0, 0.0);
  d2.id = "d2";
  d2.v = 0.0;
  d2.vx = 0.0;
  auto d3 = makeOpponent(frame, 17.0, 0.0);
  d3.id = "d3";
  d3.v = 0.0;
  d3.vx = 0.0;
  auto stamped_targets = [&](double stamp_sec) {
    d2.stamp_sec = stamp_sec;
    d3.stamp_sec = stamp_sec;
    return std::vector<overtake_planner::OpponentState>{d2, d3};
  };
  const auto input = readyReentryInput();

  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), stamped_targets(0.1),
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.0), stamped_targets(0.2),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  const double pass_target_d =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(pass_target_d));

  // まだ抜いている本人d2が必要制動距離内でも、それを理由にPASSを解除して
  // FOLLOWへ戻らない。安全な同側PASSを継続し、対象IDを保持する。
  auto before_target = makeEgo(frame, 6.0, pass_target_d + 0.10);
  before_target.v = 1.5;
  const auto same_target_braking =
      core.update(0.3, before_target, stamped_targets(0.3),
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(same_target_braking.blocked_info.braking_follow_active);
  EXPECT_EQ(same_target_braking.blocked_info.braking_follow_id, "d2");
  // まだ抜いている本人へ同じdでFOLLOWする候補は必要制動距離不足なので、
  // 安全な候補として扱わない。ここで必要なのは、その不成立を同側PASSの
  // 不成立へ誤伝播させず、SafetyEvaluator済みPASSを維持すること。
  EXPECT_FALSE(same_target_braking.blocked_info.braking_follow_feasible);
  EXPECT_FALSE(same_target_braking.blocked_info
                   .braking_follow_candidate_reject_reason.empty());
  EXPECT_EQ(same_target_braking.mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(same_target_braking.selected,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(same_target_braking.maneuver_latch_target_id, "d2");

  const auto near_target = makeEgo(frame, 8.3, pass_target_d + 0.80);
  const auto safe_chain_pass =
      core.update(0.4, near_target, stamped_targets(0.4),
                  overtake_planner::MpcHealthStatus{}, input);

  ASSERT_TRUE(safe_chain_pass.blocked_info.braking_follow_active);
  EXPECT_EQ(safe_chain_pass.blocked_info.braking_follow_id, "d3");
  EXPECT_TRUE(safe_chain_pass.blocked_info.blocked);
  ASSERT_TRUE(safe_chain_pass.blocked_info.parallel_side_candidate);
  EXPECT_EQ(safe_chain_pass.blocked_info.parallel_side_id, "d2");
  EXPECT_GT(safe_chain_pass.blocked_info.parallel_side_delta_s, 0.0);
  EXPECT_LE(safe_chain_pass.blocked_info.braking_follow_delta_s,
            safe_chain_pass.blocked_info.braking_follow_required_distance_m);
  EXPECT_GT(safe_chain_pass.blocked_info.maneuver_target_relative_s_m,
            -config.safety_ellipse_a_m * std::sqrt(1.0 + config.min_ellipse_h));
  EXPECT_FALSE(
      safe_chain_pass.blocked_info.maneuver_target_pass_geometric_complete);
  EXPECT_EQ(safe_chain_pass.maneuver_latch_target_id, "d2");
  EXPECT_TRUE(safe_chain_pass.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(safe_chain_pass.blocked_info.attack_follow_hold_pass_side);
  EXPECT_FALSE(safe_chain_pass.blocked_info.attack_follow_acceleration_allowed);
  EXPECT_EQ(safe_chain_pass.blocked_info.maneuver_chain_tail_index, 1);
  // 同じdの制動距離内でも、SafetyEvaluator済みPASSに予備余裕がある間は
  // 縦距離だけでFOLLOWへ落とさない。実Gate 2でmargin 0.73の右側軌道を
  // 停止させた回帰を表し、min_ellipse_h未満の候補を許可するものではない。
  EXPECT_TRUE(safe_chain_pass.blocked_info.pass_left_candidate_feasible);
  EXPECT_GT(safe_chain_pass.blocked_info.maneuver_target_pass_min_safety_margin,
            config.min_ellipse_h + 0.10);
  EXPECT_EQ(safe_chain_pass.mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(safe_chain_pass.selected,
            overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(safe_chain_pass.maneuver_latch_target_id, "d2");
  EXPECT_FALSE(safe_chain_pass.reentry_gate.requested);
}

TEST(OvertakePlannerCore,
     SafePassStaysActiveThenHandsOffToVerifiedAttackFollowWithSameTarget) {
  const auto frame = makeSteeringRateTransitionFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_a_m = 1.0;
  // 最小clearance PASSのhを安全floor 0.1以上、handoff予備帯0.2未満へ
  // 置き、SafetyEvaluator合格を維持したままbraking-distance handoffを作る。
  config.safety_ellipse_b_m = 0.65;
  config.min_ellipse_h = 0.1;
  config.pass_target_policy = "minimum_clearance";
  config.pass_target_lateral_margin_m = 0.02;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 4.0;
  config.localized_avoidance_full_offset_before_target_m = 1.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 30.0;
  config.braking_follow_max_target_speed_mps = 1.0;
  config.braking_follow_trigger_margin_m = 0.3;
  config.braking_follow_ttc_threshold_sec = 5.0;
  // active PPが検証済みとみなす最大減速度と同じ契約にする。
  config.max_brake_decel_mps2 = 1.0;
  config.longitudinal_response_delay_sec = 0.25;
  config.d_min_m = -2.0;
  config.d_max_m = 2.0;
  config.reentry_gate_enabled = true;
  config.straight_overtake_lookahead_m = 3.5;
  config.attack_follow_tracking_wheelbase_m = 1.087;
  config.attack_follow_max_steering_angle_rad = 0.5585053606381855;
  // PASSは実controller上限で成立させ、handoff後のATTACK_FOLLOWだけを
  // 空間horizon不足にする。共通物理上限を下げてPASSまで失格にしない。
  config.attack_follow_max_steering_rate_radps = 8.0;
  config.attack_follow_min_spatial_horizon_m = 20.0;
  config.attack_follow_max_evaluation_horizon_sec = 4.0;
  config.attack_follow_steering_tire_angle_gain = 1.639;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto make_curved_ego = [&](double s_m, double d_m) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::EgoState ego;
    ego.stamp_sec = 0.0;
    ego.x = point.x;
    ego.y = point.y;
    ego.yaw = point.yaw;
    ego.v = 4.0;
    ego.frenet = frame.cartesianToFrenet(ego.x, ego.y, ego.yaw);
    ego.valid = true;
    return ego;
  };
  const auto make_stationary_target = [&](const char *id, double s_m,
                                          double d_m = 0.0) {
    const auto point = frame.frenetToCartesian(s_m, d_m);
    overtake_planner::OpponentState target;
    target.id = id;
    target.x = point.x;
    target.y = point.y;
    target.v = 0.0;
    target.vx = 0.0;
    target.vy = 0.0;
    target.frenet = frame.cartesianToFrenet(target.x, target.y, point.yaw);
    target.valid = true;
    return target;
  };
  auto d2 = make_stationary_target("d2", 10.2);
  auto d3 = make_stationary_target("d3", 16.5);
  const auto stamped_targets = [&](double stamp_sec) {
    d2.stamp_sec = stamp_sec;
    d3.stamp_sec = stamp_sec;
    return std::vector<overtake_planner::OpponentState>{d2, d3};
  };
  const auto input = readyReentryInput();

  auto prepare_ego = make_curved_ego(5.0, 0.0);
  prepare_ego.v = 2.5;
  const auto prepare = core.update(0.1, prepare_ego, stamped_targets(0.1),
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode, overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT)
      << "nearest=" << prepare.blocked_info.nearest_id
      << " left=" << prepare.blocked_info.pass_left_candidate_reject_reason
      << " right=" << prepare.blocked_info.pass_right_candidate_reject_reason;
  // 状態遷移そのものを1周期進める。prepare周期の軌道をまだ実車が消費して
  // いないのに0.5 m前進・横移動0を注入すると、別のtracking不整合になる。
  auto passing_ego = make_curved_ego(5.0, 0.0);
  passing_ego.v = 2.5;
  const auto passing = core.update(0.2, passing_ego, stamped_targets(0.2),
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  const double pass_target_d =
      passing.blocked_info.pass_left_candidate_target_d_m;
  ASSERT_TRUE(std::isfinite(pass_target_d));

  auto handoff_ego = make_curved_ego(8.0, pass_target_d + 0.50);
  handoff_ego.v = 4.0;
  const auto handoff = core.update(0.3, handoff_ego, stamped_targets(0.3),
                                   overtake_planner::MpcHealthStatus{}, input);
  // 攻めFOLLOWの横補正が操舵制約で不成立でも、同側PASS自身が
  // SafetyEvaluatorを通っている間はPASSを捨てない。追越不能時だけ
  // current-d YIELDへ移る契約を次周期で検証する。
  ASSERT_EQ(handoff.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT)
      << "braking=" << handoff.blocked_info.braking_follow_active
      << " braking_id=" << handoff.blocked_info.braking_follow_id
      << " chain=" << handoff.blocked_info.maneuver_chain_tail_id
      << " raw=" << static_cast<int>(handoff.raw_selected) << " pass_reject="
      << handoff.blocked_info.pass_left_candidate_reject_reason
      << " pass_margin="
      << handoff.blocked_info.maneuver_target_pass_min_safety_margin;
  ASSERT_EQ(handoff.maneuver_latch_target_id, "d2");
  ASSERT_TRUE(handoff.blocked_info.braking_follow_active);
  ASSERT_FALSE(handoff.blocked_info.braking_follow_feasible);
  ASSERT_FALSE(
      handoff.blocked_info.braking_follow_candidate_reject_reason.empty());

  // chain tailが元PASS目標dへ入っても、実車がさらに外側へ到達済みなら
  // そのcurrent-d holdを全相手へ再評価し、安全な間は同じtarget/sideのPASSを
  // 維持する。短距離で内側へ戻してABORTする回帰を許さない。
  d3 = make_stationary_target("d3", 16.5, pass_target_d);
  auto hold_ego = make_curved_ego(9.8, pass_target_d + 0.70);
  hold_ego.v = 3.5;
  const auto held = core.update(0.4, hold_ego, stamped_targets(0.4),
                                overtake_planner::MpcHealthStatus{}, input);

  ASSERT_TRUE(held.blocked_info.braking_follow_active);
  // 制動FOLLOW単体は不成立でも、外側current-dのPASS軌道は成立する。
  EXPECT_FALSE(held.blocked_info.braking_follow_feasible);
  EXPECT_FALSE(
      held.blocked_info.braking_follow_candidate_reject_reason.empty());
  EXPECT_TRUE(held.blocked_info.pass_left_candidate_feasible)
      << held.blocked_info.pass_left_candidate_reject_reason;
  EXPECT_TRUE(held.blocked_info.attack_follow_hold_pass_side);
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_safe_lateral_hold_active);
  EXPECT_EQ(held.raw_selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(held.raw_selected_feasible);
  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(held.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_EQ(held.maneuver_latch_target_id, "d2");
  EXPECT_EQ(held.blocked_info.maneuver_chain_tail_id, "d3");
  EXPECT_TRUE(held.blocked_info.maneuver_transaction_incomplete);
  EXPECT_FALSE(held.reentry_gate.requested);
  ASSERT_FALSE(held.lateral_offsets.empty());
  EXPECT_GT(held.lateral_offsets.back(), 0.0);
  ASSERT_TRUE(std::isfinite(held.blocked_info.attack_follow_target_d_m));
  // freshなchain tailが元PASS線へ入った場合は、同じ左側のまま必要な
  // クリアランス分だけ外向きへ広げてよい。内向き/反対側へ戻すことは禁止する。
  EXPECT_GE(held.blocked_info.attack_follow_target_d_m, pass_target_d - 1.0e-9);
  EXPECT_NEAR(held.target_lateral_offset_m, hold_ego.frenet.d, 1.0e-9);
  ASSERT_FALSE(held.speed_caps.empty());
  EXPECT_TRUE(std::all_of(
      held.speed_caps.begin(), held.speed_caps.end(),
      [&](double speed_mps) { return speed_mps <= hold_ego.v + 1.0e-9; }));
}

TEST(OvertakePlannerCore,
     LocalizedPassUsesOutermostClearanceForOffsetSlowChain) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.pass_target_policy = "minimum_clearance";
  config.pass_target_lateral_margin_m = 0.10;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.50;
  config.min_ellipse_h = 0.20;
  config.large_lateral_error_threshold_m = 10.0;
  config.d_min_m = -2.0;
  config.d_max_m = 2.5;
  config.horizon_points = 50;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.localized_avoidance_start_before_target_m = 8.0;
  config.localized_avoidance_full_offset_before_target_m = 4.0;
  config.localized_avoidance_hold_after_target_m = 4.0;
  config.localized_avoidance_merge_distance_m = 6.0;
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 13.0, 0.4);
  d2.id = "d2";
  d2.vx = 0.0;
  d2.v = 0.0;
  d2.stamp_sec = 0.1;
  auto d3 = makeOpponent(frame, 20.0, 1.2);
  d3.id = "d3";
  d3.vx = 0.0;
  d3.v = 0.0;
  d3.stamp_sec = 0.1;
  const auto ego = makeEgo(frame, 5.0, 0.8);

  const auto prepare =
      core.update(0.1, ego, {d2, d3}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  const double required_gap_m =
      config.safety_ellipse_b_m * std::sqrt(1.0 + config.min_ellipse_h) +
      config.pass_target_lateral_margin_m;
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_EQ(prepare.blocked_info.maneuver_chain_tail_id, "d3");
  EXPECT_EQ(prepare.blocked_info.maneuver_chain_target_count, 2);
  EXPECT_GE(prepare.blocked_info.pass_left_candidate_target_d_m,
            d2.frenet.d + required_gap_m - 1.0e-9);
  EXPECT_EQ(prepare.maneuver_latch_waypoint_count, 2);
  EXPECT_EQ(prepare.maneuver_latch_last_waypoint_id, "d3");
  EXPECT_GE(prepare.maneuver_latch_last_waypoint_d_m,
            d3.frenet.d + required_gap_m - 1.0e-9);
  EXPECT_TRUE(prepare.blocked_info.pass_left_candidate_feasible);

  d2.stamp_sec = 0.2;
  d3.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.2, 0.75), {d2, d3},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  EXPECT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_EQ(passing.selected, overtake_planner::CandidateType::PASS_LEFT);
  EXPECT_TRUE(passing.blocked_info.pass_left_candidate_feasible);

  d2.stamp_sec = 0.3;
  d3.stamp_sec = 0.3;
  auto between_targets_ego = makeEgo(frame, 14.0, 1.80);
  between_targets_ego.v = 0.60;
  const auto staged =
      core.update(0.3, between_targets_ego, {d2, d3},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  // d2はまだmerge_front_gap分を抜き切っていないのでauthoritative IDは固定する。
  // 一方、速度gateと攻めFOLLOWの横目標は、次に実際に接近するd3のstaged
  // waypointを使い、先頭d2用の内側targetへ戻さない。
  EXPECT_EQ(staged.maneuver_latch_target_id, "d2");
  EXPECT_TRUE(staged.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(staged.blocked_info.maneuver_transaction_retry_active);
  EXPECT_EQ(staged.blocked_info.pass_lateral_first_target_id, "d3");
  EXPECT_TRUE(staged.blocked_info.attack_follow_hold_pass_side);
  EXPECT_FALSE(staged.blocked_info.authorized_pass_current_d_hold_active);
  EXPECT_GE(staged.blocked_info.attack_follow_target_d_m,
            d3.frenet.d + required_gap_m - 1.0e-9);
  EXPECT_GT(staged.blocked_info.attack_follow_target_d_m,
            passing.blocked_info.pass_left_candidate_target_d_m + 0.50);
}

TEST(OvertakePlannerCore,
     LocalizedChainDoesNotTreatSubMillimeterEgoRetreatAsCompletedLap) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.future_side_prediction_enabled = false;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.slow_obstacle_chain_enabled = true;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_obstacle_chain_distance_m = 12.0;
  config.merge_front_gap_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto d2 = makeOpponent(frame, 13.0, -0.6);
  d2.id = "d2";
  d2.vx = 0.0;
  d2.v = 0.0;
  auto d3 = makeOpponent(frame, 20.0, -0.6);
  d3.id = "d3";
  d3.vx = 0.0;
  d3.v = 0.0;
  const auto input = readyReentryInput();

  d2.stamp_sec = 0.1;
  d3.stamp_sec = 0.1;
  const auto prepare = core.update(0.1, makeEgo(frame, 5.0, 0.0), {d2, d3},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_EQ(prepare.blocked_info.maneuver_chain_tail_id, "d3");

  d2.stamp_sec = 0.2;
  d3.stamp_sec = 0.2;
  const auto tiny_retreat =
      core.update(0.2, makeEgo(frame, 5.0 - 1.0e-6, 0.0), {d2, d3},
                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_NEAR(tiny_retreat.blocked_info.maneuver_chain_tail_relative_s_m,
              15.0 + 1.0e-6, 1.0e-7);
  EXPECT_FALSE(
      tiny_retreat.blocked_info.maneuver_target_pass_geometric_complete);
  EXPECT_FALSE(tiny_retreat.blocked_info.maneuver_target_pass_complete);
  EXPECT_EQ(tiny_retreat.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
}

TEST(OvertakePlannerCore,
     ActivePassKeepsLatchedTargetWhenFrontClassifierSelectsAnotherVehicle) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_min_hold_sec = 0.1;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto initial_target = makeOpponent(frame, 13.0, -0.6);
  initial_target.id = "d3";
  const auto prepare =
      core.update(0.1, ego, {initial_target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);
  ASSERT_TRUE(prepare.maneuver_latch_active);
  ASSERT_EQ(prepare.maneuver_latch_target_id, "d3");

  // d3はfreshな観測一覧には残るがfront分類の外へ移り、d2がnearestになる。
  // PASS中は分類器の代表indexではなくラッチIDを解決し、d3を抜くまで
  // 同じ横プロファイルを保持する。
  auto still_ahead_target = makeOpponent(frame, 13.0, -2.0);
  still_ahead_target.id = "d3";
  still_ahead_target.stamp_sec = 2.0;
  auto newly_classified_front = makeOpponent(frame, 12.5, -0.6);
  newly_classified_front.id = "d2";
  newly_classified_front.stamp_sec = 2.0;
  const auto output =
      core.update(2.0, makeEgo(frame, 5.5, 0.2),
                  {still_ahead_target, newly_classified_front},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_TRUE(output.maneuver_latch_active);
  EXPECT_EQ(output.maneuver_latch_target_id, "d3");
  EXPECT_EQ(output.blocked_info.nearest_id, "d2");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
}

TEST(OvertakePlannerCore, RejectedPreparedProfileRetargetsBeforeGateTwoCommit) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.overtake_lateral_profile_mode = "localized_latched";
  config.maneuver_latch_min_hold_sec = 1.0;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.5;
  config.d_min_m = -0.5;
  config.d_max_m = 0.5;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 1.5;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto first_target = makeOpponent(frame, 13.0, 0.0);
  first_target.id = "d3";
  first_target.v = 0.0;
  first_target.vx = 0.0;
  first_target.stamp_sec = 0.1;
  auto initial_ego = makeEgo(frame, 5.0, 0.0);
  initial_ego.v = 0.005;
  const auto rejected = core.update(0.1, initial_ego, {first_target});

  ASSERT_TRUE(rejected.maneuver_latch_active);
  ASSERT_EQ(rejected.maneuver_latch_target_id, "d3");
  EXPECT_FALSE(rejected.blocked_info.pass_left_candidate_feasible);
  EXPECT_FALSE(rejected.blocked_info.pass_right_candidate_feasible);
  EXPECT_FALSE(rejected.blocked_info.maneuver_transaction_incomplete);

  first_target.stamp_sec = 0.15;
  // 実bagの停止中dノイズ全幅0.249 mmを覆う0.25 mm以内なら、同じ
  // SafetyEvaluator対象profileを再利用してwireを完全一致に保つ。
  auto tiny_drift_ego = makeEgo(frame, 5.0, 0.00025);
  tiny_drift_ego.v = 0.005;
  const auto held = core.update(0.15, tiny_drift_ego, {first_target});
  const auto rejected_wire =
      overtake_planner::makeReferenceOverrideWirePayload(rejected, 1U);
  const auto held_wire =
      overtake_planner::makeReferenceOverrideWirePayload(held, 2U);
  EXPECT_TRUE(overtake_planner::referenceOverrideWirePayloadSemanticallyEqual(
      rejected_wire, held_wire));
  EXPECT_FALSE(held.blocked_info.maneuver_transaction_incomplete);

  // Gate 2未認可の準備profileは固定契約ではない。分類対象がd2へ変わったら、
  // 旧d3を保持せず現在の実障害物へ候補を作り直す。
  auto old_target_outside_front = makeOpponent(frame, 13.0, -2.0);
  old_target_outside_front.id = "d3";
  old_target_outside_front.stamp_sec = 0.2;
  auto new_front = makeOpponent(frame, 12.0, 0.0);
  new_front.id = "d2";
  new_front.stamp_sec = 0.2;
  // 未認可候補に限り、target IDの変更は1.0 sのチャタリングhold中でも
  // 即時に反映する。旧d3 profileへ新d2観測を混在させて認可しない。
  const auto retargeted = core.update(0.2, makeEgo(frame, 5.2, 0.0),
                                      {old_target_outside_front, new_front});

  ASSERT_TRUE(retargeted.maneuver_latch_active);
  EXPECT_EQ(retargeted.blocked_info.nearest_id, "d2");
  EXPECT_EQ(retargeted.maneuver_latch_target_id, "d2");
  EXPECT_EQ(retargeted.blocked_info.maneuver_target_index, 1);
  EXPECT_EQ(retargeted.blocked_info.maneuver_target_previous_id, "d3");
  EXPECT_EQ(retargeted.blocked_info.maneuver_target_new_id, "d2");
  EXPECT_FALSE(retargeted.blocked_info.maneuver_transaction_incomplete);
}

TEST(OvertakePlannerCore,
     ActivePassMergesOnlyAfterLatchedTargetPassCompletionContract) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.merge_front_gap_m = 6.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d3";
  target.stamp_sec = 0.1;
  const auto prepare =
      core.update(0.1, ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  const auto passing =
      core.update(0.2, makeEgo(frame, 5.5, 0.2), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(passing.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_FALSE(passing.blocked_info.maneuver_target_pass_complete);

  // 横方向の離隔ではなく、ラッチd3が自車後方6 m以上、かつ自車が
  // targetより遅くなく、同側PASS候補がSafetyEvaluatorを通った時に完了。
  target.stamp_sec = 0.3;
  const auto completed =
      core.update(0.3, makeEgo(frame, 20.0, config.left_offset_m), {target},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_EQ(completed.maneuver_latch_target_id, "d3");
  EXPECT_NEAR(completed.blocked_info.maneuver_target_relative_s_m, -7.0,
              1.0e-9);
  EXPECT_TRUE(completed.blocked_info.maneuver_target_pass_geometric_complete);
  EXPECT_TRUE(completed.blocked_info.maneuver_target_pass_safety_approved);
  EXPECT_TRUE(completed.blocked_info.maneuver_target_pass_complete);
  EXPECT_EQ(completed.mode, overtake_planner::BehaviorMode::MERGE_BACK);
  EXPECT_EQ(completed.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(completed.active_override);
  EXPECT_FALSE(completed.lateral_offsets.empty());
  EXPECT_FALSE(completed.published_lateral_safety_rejected);
}

TEST(OvertakePlannerCore,
     GateTwoApprovedTargetCompletesWhenNewObstacleRejectsCurrentPass) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d3";
  target.stamp_sec = 0.1;
  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  ASSERT_EQ(core.update(0.2, makeEgo(frame, 5.5, 0.2), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);

  // d3は認可済みの左側で十分後方まで抜いた。一方、現在位置には別のd4を
  // 置いて同周期のPASS候補を不成立にする。d4の安全処理は必要だが、既に
  // 完了したd3のtarget IDまで未完了へ巻き戻してはならない。
  target.stamp_sec = 0.3;
  auto next_obstacle = makeOpponent(frame, 20.2, config.left_offset_m);
  next_obstacle.id = "d4";
  next_obstacle.stamp_sec = 0.3;
  const auto completed = core.update(
      0.3, makeEgo(frame, 20.0, config.left_offset_m), {target, next_obstacle},
      overtake_planner::MpcHealthStatus{}, readyReentryInput());

  EXPECT_EQ(completed.maneuver_latch_target_id, "d3");
  EXPECT_TRUE(completed.blocked_info.maneuver_target_pass_geometric_complete);
  EXPECT_FALSE(completed.blocked_info.maneuver_target_pass_safety_approved);
  EXPECT_FALSE(completed.blocked_info.maneuver_target_pass_candidate_feasible);
  EXPECT_TRUE(completed.blocked_info.maneuver_target_pass_complete);
}

TEST(OvertakePlannerCore, StaleLatchedPassTargetStopsWithoutStartingCentering) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.1;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.opponent_stale_time_sec = 0.5;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto target = makeOpponent(frame, 13.0, -0.6);
  target.id = "d3";
  target.stamp_sec = 0.1;
  ASSERT_EQ(core.update(0.1, makeEgo(frame, 5.0, 0.0), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  target.stamp_sec = 0.2;
  ASSERT_EQ(core.update(0.2, makeEgo(frame, 5.5, 0.2), {target},
                        overtake_planner::MpcHealthStatus{},
                        readyReentryInput())
                .mode,
            overtake_planner::BehaviorMode::OVERTAKE_LEFT);

  // 観測配列にIDが残っていても規定timeoutを超えたstampは継続PASSに使わない。
  const auto stale_ego = makeEgo(frame, 6.0, 0.4);
  const auto stale =
      core.update(1.0, stale_ego, {target}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  EXPECT_TRUE(stale.maneuver_latch_active);
  EXPECT_EQ(stale.maneuver_latch_target_id, "d3");
  EXPECT_TRUE(stale.blocked_info.maneuver_target_latched);
  EXPECT_FALSE(stale.blocked_info.maneuver_target_observed);
  EXPECT_EQ(stale.blocked_info.maneuver_target_change_reason,
            "latched_target_missing_or_stale");
  EXPECT_EQ(stale.blocked_info.pass_decision_freeze_reason,
            "maneuver_target_missing_or_stale");
  EXPECT_TRUE(stale.blocked_info.maneuver_transaction_incomplete);
  EXPECT_TRUE(stale.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(stale.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_FALSE(stale.active_override);
  EXPECT_TRUE(stale.lateral_offsets.empty());
  EXPECT_FALSE(stale.reentry_gate.requested);
  EXPECT_TRUE(stale.safe_stop_triggered);
  EXPECT_EQ(stale.reason, "incomplete_pass_execution_speed_only_stop");
  const auto stale_constraint = overtake_planner::makeSafetyConstraint(
      stale, stale_ego, readyReentryInput(), 8.0, 1.0);
  EXPECT_TRUE(stale_constraint.valid);
  EXPECT_TRUE(stale_constraint.stop_requested);
  EXPECT_FALSE(stale_constraint.release_authorized);

  // 対象IDが配列から完全に消えた場合も、別targetや中心復帰へ所有権を渡さず
  // 同じidentityを保持したspeed-only STOPへ閉じる。
  const auto missing =
      core.update(1.1, stale_ego, {}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());
  EXPECT_EQ(missing.maneuver_latch_target_id, "d3");
  EXPECT_EQ(missing.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(missing.selected, overtake_planner::CandidateType::SAFE_STOP);
  EXPECT_TRUE(missing.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(missing.active_override);
  EXPECT_TRUE(missing.lateral_offsets.empty());
  EXPECT_FALSE(missing.reentry_gate.requested);
  const auto missing_constraint = overtake_planner::makeSafetyConstraint(
      missing, stale_ego, readyReentryInput(), 8.0, 1.0);
  EXPECT_TRUE(missing_constraint.valid);
  EXPECT_TRUE(missing_constraint.stop_requested);
  EXPECT_FALSE(missing_constraint.release_authorized);
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
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto opponent = makeOpponent(frame, 13.0, -0.6);
  opponent.vy = 2.0;

  const auto output = core.update(0.1, ego, {opponent});

  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::YIELD_BEHIND)
      << "raw_selected=" << static_cast<int>(output.raw_selected)
      << " raw_feasible=" << output.raw_selected_feasible
      << " state_machine=" << static_cast<int>(output.state_machine_mode)
      << " post_reentry="
      << static_cast<int>(output.post_reentry_arbitration_mode)
      << " final_selected=" << static_cast<int>(output.selected)
      << " reason=" << output.reason;
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::YIELD_BEHIND)
      << "left_generated=" << output.blocked_info.pass_left_candidate_generated
      << " left_feasible=" << output.blocked_info.pass_left_candidate_feasible
      << " left_reject="
      << output.blocked_info.pass_left_candidate_reject_reason
      << " right_generated="
      << output.blocked_info.pass_right_candidate_generated
      << " right_feasible=" << output.blocked_info.pass_right_candidate_feasible
      << " right_reject="
      << output.blocked_info.pass_right_candidate_reject_reason
      << " transaction_side="
      << static_cast<int>(output.blocked_info.maneuver_transaction_pass_type)
      << " reason=" << output.reason;
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
     BrakingFollowOutsideNormalLookaheadBindsLocalizedPassTarget) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.lookahead_s_m = 10.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.braking_follow_enabled = true;
  config.braking_follow_max_distance_m = 60.0;
  config.braking_follow_max_target_speed_mps = 1.0;
  config.braking_follow_trigger_margin_m = 2.0;
  config.braking_follow_ttc_threshold_sec = 5.0;
  config.longitudinal_response_delay_sec = 0.25;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  auto stopped = makeOpponent(frame, 20.0, 0.0);
  stopped.id = "d2";
  stopped.stamp_sec = 0.1;
  stopped.vx = 0.0;
  stopped.v = 0.0;

  const auto output =
      core.update(0.1, ego, {stopped}, overtake_planner::MpcHealthStatus{},
                  readyReentryInput());

  // 通常の10 m lookaheadには入らないが、停止距離/TTCの15 m対象としては
  // FOLLOW/PASS評価が必要。汎用offsetへfallbackせず、実車IDをprofileへ固定する。
  EXPECT_FALSE(output.blocked_info.blocked);
  EXPECT_EQ(output.blocked_info.nearest_index, -1);
  ASSERT_TRUE(output.blocked_info.braking_follow_active);
  EXPECT_EQ(output.blocked_info.braking_follow_id, "d2");
  EXPECT_TRUE(output.maneuver_latch_active);
  EXPECT_EQ(output.maneuver_latch_target_id, "d2");
  EXPECT_TRUE(output.blocked_info.maneuver_target_latched);
  EXPECT_EQ(output.blocked_info.maneuver_target_id, "d2");
  if (output.blocked_info.pass_left_candidate_generated) {
    EXPECT_NE(output.blocked_info.pass_left_candidate_reject_reason,
              "pass_target_profile_missing");
  }
  if (output.blocked_info.pass_right_candidate_generated) {
    EXPECT_NE(output.blocked_info.pass_right_candidate_reject_reason,
              "pass_target_profile_missing");
  }
}

TEST(OvertakePlannerCore,
     ActiveLeftPassKeepsSameSideWhenStaticGapIsLostButCandidateSafe) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto input = readyReentryInput();
  const auto prepare = core.update(0.1, ego, {first_opponent},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_LEFT);

  // 静的gap閾値だけは失うが、固定した左PASS dとは楕円余裕を残す。
  const auto shifted_opponent = makeOpponent(frame, 13.0, 1.1);
  const auto output = core.update(0.2, ego, {shifted_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

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
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, 0.6);
  const auto input = readyReentryInput();
  const auto prepare = core.update(0.1, ego, {first_opponent},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(prepare.mode,
            overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT);

  const auto shifted_opponent = makeOpponent(frame, 13.0, -1.1);
  const auto output = core.update(0.2, ego, {shifted_opponent},
                                  overtake_planner::MpcHealthStatus{}, input);

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
     SupervisorV2ShadowTogglePreservesLegacyOutputsAcrossPassAbortAndReentry) {
  const auto frame = makeStraightFrame();
  auto common_config = makeConfig();
  common_config.dynamic_pass_candidate_enabled = true;
  common_config.reentry_gate_enabled = true;
  common_config.reentry_safe_cycles = 1;
  common_config.reentry_min_safety_margin_h = 0.15;
  common_config.reentry_evaluation_horizon_sec = 4.0;
  common_config.supervisor_v2_abort_release_cycles = 10;
  common_config.lateral_target_max_step_m = 100.0;
  auto shadow_off_config = common_config;
  shadow_off_config.supervisor_v2_shadow_enabled = false;
  auto shadow_on_config = common_config;
  shadow_on_config.supervisor_v2_shadow_enabled = true;
  overtake_planner::OvertakePlannerCore shadow_off(frame, shadow_off_config);
  overtake_planner::OvertakePlannerCore shadow_on(frame, shadow_on_config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  const auto off_pass =
      shadow_off.update(0.1, centered_ego, {first_opponent},
                        overtake_planner::MpcHealthStatus{}, ready);
  const auto on_pass =
      shadow_on.update(0.1, centered_ego, {first_opponent},
                       overtake_planner::MpcHealthStatus{}, ready);
  expectLegacyOutputsEqual(off_pass, on_pass);
  ASSERT_EQ(on_pass.supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);

  const auto close_opponent = makeOpponent(frame, 5.6, 0.6);
  const auto off_abort =
      shadow_off.update(0.2, centered_ego, {close_opponent},
                        overtake_planner::MpcHealthStatus{}, ready);
  const auto on_abort =
      shadow_on.update(0.2, centered_ego, {close_opponent},
                       overtake_planner::MpcHealthStatus{}, ready);
  expectLegacyOutputsEqual(off_abort, on_abort);
  ASSERT_EQ(on_abort.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);

  const auto offset_ego = makeEgo(frame, 5.0, 0.8);
  const auto off_reentry = shadow_off.update(
      0.3, offset_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  const auto on_reentry = shadow_on.update(
      0.3, offset_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  expectLegacyOutputsEqual(off_reentry, on_reentry);
  ASSERT_TRUE(on_reentry.reentry_gate.requested);
  ASSERT_TRUE(on_reentry.reentry_gate.permitted);
  ASSERT_EQ(on_reentry.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);

  const auto off_centered = shadow_off.update(
      0.4, centered_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  const auto on_centered = shadow_on.update(
      0.4, centered_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  expectLegacyOutputsEqual(off_centered, on_centered);
  EXPECT_FALSE(on_centered.reentry_gate.requested);
  EXPECT_EQ(on_centered.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
}

TEST(OvertakePlannerCore,
     SupervisorV2TransientTrackingMismatchRetainsTargetSideAndProfile) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.supervisor_v2_tracking_unusable_hold_cycles = 2;
  config.dynamic_pass_candidate_enabled = true;
  config.lateral_target_max_step_m = 100.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();
  const auto ego = makeEgo(frame, 5.0, 0.0);
  const auto opponent = makeOpponent(frame, 13.0, -0.6);

  const auto started = core.update(0.1, ego, {opponent},
                                   overtake_planner::MpcHealthStatus{}, ready)
                           .supervisor_v2;
  ASSERT_EQ(started.phase, overtake_planner::TacticalPhase::PASSING);
  const auto committed_target_d_m = started.trajectory.planned_target_d_m;

  auto transient = ready;
  transient.pure_pursuit_primary_and_fresh = false;
  const auto held = core.update(0.2, ego, {opponent},
                                overtake_planner::MpcHealthStatus{}, transient)
                        .supervisor_v2;
  EXPECT_EQ(held.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(held.reason, "passing_tracking_temporarily_unavailable_hold");
  EXPECT_EQ(held.target_vehicle_id, started.target_vehicle_id);
  EXPECT_EQ(held.pass_direction, started.pass_direction);
  EXPECT_EQ(held.attempt_id, started.attempt_id);
  EXPECT_FALSE(held.trajectory_authorized);

  const auto resumed = core.update(0.3, ego, {opponent},
                                   overtake_planner::MpcHealthStatus{}, ready)
                           .supervisor_v2;
  EXPECT_EQ(resumed.phase, overtake_planner::TacticalPhase::PASSING);
  EXPECT_EQ(resumed.reason, "passing_same_generation_side");
  EXPECT_EQ(resumed.target_vehicle_id, started.target_vehicle_id);
  EXPECT_EQ(resumed.pass_direction, started.pass_direction);
  EXPECT_EQ(resumed.attempt_id, started.attempt_id);
  EXPECT_EQ(resumed.selected, started.selected);
  EXPECT_NEAR(resumed.trajectory.planned_target_d_m, committed_target_d_m,
              1.0e-9);
}

TEST(OvertakePlannerCore, SupervisorV2AbortHoldsCentersFallsBackAndReleases) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.supervisor_v2_abort_release_cycles = 2;
  config.dynamic_pass_candidate_enabled = true;
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.lateral_target_max_step_m = 100.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_opponent},
                        overtake_planner::MpcHealthStatus{}, ready)
                .supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto close_opponent = makeOpponent(frame, 5.6, 0.6);
  const auto entered = core.update(0.2, centered_ego, {close_opponent},
                                   overtake_planner::MpcHealthStatus{}, ready);
  ASSERT_EQ(entered.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);

  const auto offset_ego = makeEgo(frame, 5.0, 0.8);
  auto centering_blocker = makeOpponent(frame, 13.0, 0.0);
  centering_blocker.id = "d3";
  centering_blocker.vx = 0.0;
  centering_blocker.v = 0.0;
  const auto blocked_centering =
      core.update(0.3, offset_ego, {centering_blocker},
                  overtake_planner::MpcHealthStatus{}, ready);
  ASSERT_EQ(blocked_centering.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(blocked_centering.supervisor_v2.reason,
            "abort_hold_current_lateral");
  ASSERT_FALSE(blocked_centering.supervisor_v2.trajectory.d.empty());
  EXPECT_TRUE(std::all_of(blocked_centering.supervisor_v2.trajectory.d.begin(),
                          blocked_centering.supervisor_v2.trajectory.d.end(),
                          [&offset_ego](double d) {
                            return std::abs(d - offset_ego.frenet.d) <= 1.0e-6;
                          }));

  const auto pending = core.update(0.4, offset_ego, {},
                                   overtake_planner::MpcHealthStatus{}, ready);
  ASSERT_EQ(pending.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  ASSERT_FALSE(pending.supervisor_v2.trajectory.d.empty());
  EXPECT_TRUE(std::all_of(pending.supervisor_v2.trajectory.d.begin(),
                          pending.supervisor_v2.trajectory.d.end(),
                          [&offset_ego](double d) {
                            return std::abs(d - offset_ego.frenet.d) <= 1.0e-6;
                          }));
  EXPECT_EQ(pending.supervisor_v2.reason, "abort_hold_current_lateral");

  const auto centering = core.update(
      0.5, offset_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  ASSERT_EQ(centering.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(centering.supervisor_v2.reason, "abort_hold_centering_recovery");
  EXPECT_TRUE(std::any_of(
      centering.supervisor_v2.trajectory.d.begin(),
      centering.supervisor_v2.trajectory.d.end(), [&offset_ego](double d) {
        return std::abs(d) + 1.0e-3 < std::abs(offset_ego.frenet.d);
      }));

  auto stale = ready;
  stale.v2x_snapshot_fresh = false;
  const auto held_on_stale =
      core.update(0.6, makeEgo(frame, 5.0, 0.5), {},
                  overtake_planner::MpcHealthStatus{}, stale);
  EXPECT_EQ(held_on_stale.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(held_on_stale.supervisor_v2.reason, "abort_hold_current_lateral");
  EXPECT_FALSE(held_on_stale.supervisor_v2.trajectory_authorized);
  ASSERT_FALSE(held_on_stale.supervisor_v2.trajectory.d.empty());
  EXPECT_TRUE(
      std::all_of(held_on_stale.supervisor_v2.trajectory.d.begin(),
                  held_on_stale.supervisor_v2.trajectory.d.end(),
                  [](double d) { return std::abs(d - 0.5) <= 1.0e-6; }));

  const auto offset_after_stale = makeEgo(frame, 5.0, 0.5);
  const auto fresh_pending = core.update(
      0.7, offset_after_stale, {}, overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(fresh_pending.supervisor_v2.reason, "abort_hold_current_lateral");
  const auto fresh_centering = core.update(
      0.8, offset_after_stale, {}, overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(fresh_centering.supervisor_v2.reason,
            "abort_hold_centering_recovery");

  const auto release_pending = core.update(
      0.9, centered_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(release_pending.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(release_pending.supervisor_v2.reason,
            "abort_center_release_confirm");
  auto invalid_centered_ego = centered_ego;
  invalid_centered_ego.valid = false;
  core.update(1.0, invalid_centered_ego, {},
              overtake_planner::MpcHealthStatus{}, ready);
  const auto release_centering_restarted = core.update(
      1.1, centered_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(release_centering_restarted.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(release_centering_restarted.supervisor_v2.reason,
            "abort_hold_current_lateral");
  const auto release_confirmation_restarted = core.update(
      1.2, centered_ego, {}, overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(release_confirmation_restarted.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(release_confirmation_restarted.supervisor_v2.reason,
            "abort_center_release_confirm");
  const auto released = core.update(1.3, centered_ego, {},
                                    overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(released.supervisor_v2.phase,
            overtake_planner::TacticalPhase::FREE_RUN);
  EXPECT_EQ(released.supervisor_v2.reason, "abort_hold_released");
}

TEST(OvertakePlannerCore,
     SupervisorV2CenteringUsesFreshPurePursuitDuringMpcLatencyOnly) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.supervisor_v2_abort_release_cycles = 10;
  config.dynamic_pass_candidate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_opponent},
                        overtake_planner::MpcHealthStatus{}, ready)
                .supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto close_opponent = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.2, centered_ego, {close_opponent},
                        overtake_planner::MpcHealthStatus{}, ready)
                .supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);

  auto latency_only = ready;
  latency_only.mpc_healthy = false;
  latency_only.mpc_latency_warning = true;
  latency_only.mpc_health_sample_sequence = 1U;
  const auto centering =
      core.update(0.3, makeEgo(frame, 5.0, 0.8), {},
                  overtake_planner::MpcHealthStatus{}, latency_only);
  EXPECT_EQ(centering.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(centering.supervisor_v2.reason, "abort_hold_centering_recovery");

  auto hard_failure = latency_only;
  hard_failure.mpc_latency_warning = false;
  hard_failure.mpc_hard_failure = true;
  hard_failure.mpc_health_sample_sequence = 2U;
  const auto held =
      core.update(0.4, makeEgo(frame, 5.0, 0.7), {},
                  overtake_planner::MpcHealthStatus{}, hard_failure);
  EXPECT_EQ(held.supervisor_v2.reason, "abort_hold_current_lateral");
}

TEST(OvertakePlannerCore, SupervisorV2CenteringHoldsBeforeFutureCorridorClamp) {
  auto frame = makeStraightFrame();
  std::vector<overtake_planner::FrenetCorridorPoint> corridor;
  for (const auto &point : frame.reference()) {
    const bool narrow = point.s == 11.0 || point.s == 12.0;
    corridor.push_back({point.s, -3.0, narrow ? 0.5 : 3.0});
  }
  frame.setCorridor(corridor);

  auto config = makeConfig();
  config.supervisor_v2_shadow_enabled = true;
  config.supervisor_v2_abort_release_cycles = 10;
  config.dynamic_pass_candidate_enabled = true;
  config.prepare_distance_m = 4.0;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.d_min_m = -3.0;
  config.d_max_m = 3.0;
  config.min_wall_margin_m = 0.10;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto ready = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto first_opponent = makeOpponent(frame, 13.0, -0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {first_opponent},
                        overtake_planner::MpcHealthStatus{}, ready)
                .supervisor_v2.phase,
            overtake_planner::TacticalPhase::PASSING);
  const auto close_opponent = makeOpponent(frame, 5.1, 0.0);
  ASSERT_EQ(core.update(0.2, centered_ego, {close_opponent},
                        overtake_planner::MpcHealthStatus{}, ready)
                .supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);

  const auto held = core.update(0.3, makeEgo(frame, 5.0, 0.8), {},
                                overtake_planner::MpcHealthStatus{}, ready);
  EXPECT_EQ(held.supervisor_v2.phase,
            overtake_planner::TacticalPhase::ABORT_HOLD);
  EXPECT_EQ(held.supervisor_v2.reason, "abort_hold_current_lateral");
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
     UnsafeCenteringRecoveryEntersAbortHoldForSecondVehicle) {
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
  // 通常blockedの大きなdを候補評価へ渡した後も、reentry gate拒否は緩めず
  // 現在d保持のABORTへfail-closedにする。
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_FALSE(output.reentry_gate.permitted);
  EXPECT_EQ(output.reentry_gate.blocking_vehicle_id, "d3");
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(output.blocked_info.reentry_hold_active);
  EXPECT_TRUE(output.active_override);
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-6);
  EXPECT_NEAR(output.recovery_tracking_target_d_m, ego.frenet.d, 1.0e-6);
  EXPECT_NEAR(output.recovery_tracking_error_m, 0.0, 1.0e-6);
  // wall/MPC由来の復帰guardは残り得るが、現在d holdを中心線誤差として
  // 二重制限してはいけない。
  EXPECT_NE(output.speed_cap_reason, "recovery_lateral_error_speed_guard");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_NEAR(output.speed_caps.back(), config.reentry_hold_v_max_mps, 1.0e-9);
}

TEST(OvertakePlannerCore,
     GenericRecoveryStaleInputsDoNotAuthorizePastLateralProof) {
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
  const auto ego = makeEgo(frame, 5.0, 0.8);
  const auto guarded =
      core.update(0.1, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, readyReentryInput());
  ASSERT_EQ(guarded.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  ASSERT_EQ(guarded.selected, overtake_planner::CandidateType::RECOVERY);
  ASSERT_TRUE(guarded.reentry_gate.input_complete);
  ASSERT_TRUE(guarded.blocked_info.reentry_hold_active);
  ASSERT_TRUE(guarded.active_override);
  ASSERT_TRUE(guarded.lateral_tracking_authorized_during_stop);
  ASSERT_TRUE(guarded.controller_spatial_horizon_proof_valid);
  ASSERT_GT(guarded.required_controller_spatial_horizon_m, 0.0);
  ASSERT_FALSE(guarded.lateral_offsets.empty());
  ASSERT_FALSE(guarded.speed_caps.empty());
  ASSERT_EQ(guarded.lateral_offsets.size(), guarded.speed_caps.size());
  ASSERT_EQ(guarded.lateral_offsets.size(),
            guarded.longitudinal_offsets_m.size());
  EXPECT_GE(guarded.longitudinal_offsets_m.back(),
            guarded.required_controller_spatial_horizon_m);

  const auto expect_stale_recovery_hold = [](const auto &output) {
    EXPECT_TRUE(output.active_override);
    EXPECT_FALSE(output.lateral_tracking_authorized_during_stop);
    EXPECT_EQ(output.solver_horizon_intent,
              overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
    EXPECT_FALSE(output.lateral_offsets.empty());
    EXPECT_FALSE(output.speed_caps.empty());
  };

  auto stale_v2x_input = readyReentryInput();
  stale_v2x_input.v2x_snapshot_fresh = false;
  const auto v2x_held =
      core.update(0.2, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, stale_v2x_input);
  EXPECT_EQ(v2x_held.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(v2x_held.reentry_gate.reason, "stale_v2x_snapshot");
  EXPECT_TRUE(v2x_held.reason.empty());
  EXPECT_EQ(v2x_held.speed_cap_reason, "recovery_wall_risk_speed_guard");
  expect_stale_recovery_hold(v2x_held);

  auto stale_mpc_input = readyReentryInput();
  stale_mpc_input.mpc_healthy = false;
  const auto mpc_held =
      core.update(0.3, ego, {second_vehicle},
                  overtake_planner::MpcHealthStatus{}, stale_mpc_input);
  EXPECT_EQ(mpc_held.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(mpc_held.reentry_gate.reason, "unhealthy_mpc");
  EXPECT_TRUE(mpc_held.reason.empty());
  EXPECT_EQ(mpc_held.speed_cap_reason, "recovery_wall_risk_speed_guard");
  expect_stale_recovery_hold(mpc_held);

  auto stale_ego = makeEgo(frame, 5.0, 0.8);
  stale_ego.valid = false;
  const auto held =
      core.update(0.4, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});

  EXPECT_EQ(held.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_TRUE(held.reentry_gate.requested);
  EXPECT_EQ(held.reentry_gate.reason, "stale_ego");
  EXPECT_EQ(held.reason, "reentry_stale_ego_hold");
  EXPECT_EQ(held.speed_cap_reason, "reentry_stale_ego_hold");
  expect_stale_recovery_hold(held);
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
     PublishedRejectWithoutSafeHoldOrStopUsesSafeStopSpeedOnly) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.lateral_target_max_step_m = 0.05;
  config.safety_ellipse_b_m = 0.6;
  config.merge_distance_m = 2.0;
  config.prepare_distance_m = 2.0;
  config.safe_stop_lateral_error_threshold_m = 1.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto slow_parallel = makeOpponent(frame, 13.0, -1.8);
  slow_parallel.id = "d2";
  slow_parallel.vx = 0.0;
  slow_parallel.v = 0.0;
  auto first_ego = makeEgo(frame, 5.0, -1.2);
  first_ego.v = 0.5;
  const auto first = core.update(0.1, first_ego, {slow_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(first.reentry_gate.requested);
  ASSERT_TRUE(first.reentry_gate.permitted);

  // 中心復帰候補自体は通るが、前周期dとのrate-limit後はd3へ接近する。
  // さらにd4により現在d holdとSAFE_STOPも不成立にし、最終fallbackを通す。
  auto rate_limited_blocker = makeOpponent(frame, 7.0, -0.7);
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

  EXPECT_TRUE(output.published_lateral_safety_rejected);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_TRUE(output.lateral_offsets.empty());
  EXPECT_TRUE(output.longitudinal_offsets_m.empty());
  EXPECT_EQ(output.reason,
            "generic_published_lateral_safety_reject_no_safe_hold_watchdog");
  ASSERT_FALSE(output.speed_caps.empty());
  EXPECT_LE(output.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
}

TEST(OvertakePlannerCore,
     ChangingSafeStopProfileNeverGainsAuthorityAndIsNotReusedWhenStale) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_hold_v_max_mps = 0.5;
  config.large_lateral_error_threshold_m = 0.6;
  config.lateral_target_max_step_m = 0.05;
  config.safety_ellipse_b_m = 0.6;
  config.merge_distance_m = 2.0;
  config.prepare_distance_m = 2.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto slow_parallel = makeOpponent(frame, 13.0, -1.8);
  slow_parallel.id = "d2";
  slow_parallel.vx = 0.0;
  slow_parallel.v = 0.0;
  auto first_ego = makeEgo(frame, 5.0, -1.2);
  first_ego.v = 0.5;
  const auto first = core.update(0.1, first_ego, {slow_parallel},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(first.reentry_gate.permitted);

  auto rate_limited_blocker = makeOpponent(frame, 7.0, -0.7);
  rate_limited_blocker.id = "d3";
  rate_limited_blocker.vx = 0.0;
  rate_limited_blocker.v = 0.0;
  auto hold_blocker = makeOpponent(frame, 8.3, 0.8);
  hold_blocker.id = "d4";
  hold_blocker.vx = 0.0;
  hold_blocker.v = 0.0;
  const auto ego = makeEgo(frame, 5.0, 0.8);
  const auto safe_stop =
      core.update(0.2, ego, {rate_limited_blocker, hold_blocker},
                  overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(safe_stop.published_lateral_safety_rejected);
  ASSERT_EQ(safe_stop.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(safe_stop.active_override);
  EXPECT_FALSE(safe_stop.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(safe_stop.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_TRUE(safe_stop.longitudinal_speed_cap_active);
  EXPECT_TRUE(safe_stop.lateral_offsets.empty());
  EXPECT_TRUE(safe_stop.longitudinal_offsets_m.empty());
  ASSERT_FALSE(safe_stop.speed_caps.empty());
  EXPECT_LE(safe_stop.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);

  auto stale_input = readyReentryInput();
  stale_input.mpc_healthy = false;
  const auto stale =
      core.update(0.3, ego, {rate_limited_blocker, hold_blocker},
                  overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_FALSE(stale.active_override);
  EXPECT_FALSE(stale.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(stale.solver_horizon_intent,
            overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
  EXPECT_TRUE(stale.longitudinal_speed_cap_active);
  EXPECT_TRUE(stale.lateral_offsets.empty());
  EXPECT_TRUE(stale.longitudinal_offsets_m.empty());
  ASSERT_FALSE(stale.speed_caps.empty());
  EXPECT_LE(stale.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
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
  ASSERT_TRUE(no_hold.longitudinal_offsets_m.empty());
  ASSERT_FALSE(no_hold.speed_caps.empty());
  EXPECT_EQ(no_hold.reason, "generic_recovery_no_safe_hold_watchdog");
  EXPECT_LE(no_hold.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);

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
  EXPECT_LE(ego_stale.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);

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
  EXPECT_LE(v2x_stale.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);

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
  EXPECT_LE(mpc_stale.speed_caps.back(), config.safe_stop_v_mps + 1.0e-9);
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
  // 現d hold自体がgate時点で不成立になる。unsafeなSAFE_STOP横列もpublish
  // せず、safe-stop速度以下のspeed-onlyへ閉じるのが正しいfail-closed順序。
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
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_FALSE(output.active_override);
  EXPECT_TRUE(output.longitudinal_speed_cap_active);
  EXPECT_TRUE(output.lateral_offsets.empty());
  EXPECT_TRUE(output.longitudinal_offsets_m.empty());
  EXPECT_TRUE(output.blocked_info.reentry_hold_active);
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_FALSE(output.reentry_gate.permitted);
  EXPECT_EQ(output.reentry_gate.reason, "generic_recovery_hold_infeasible");
  EXPECT_EQ(output.reason, "generic_recovery_no_safe_hold_watchdog");
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

TEST(OvertakePlannerCore, FeasibleParallelFollowPreemptsReentryAbortRecovery) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 8.0;
  config.parallel_follow_lateral_width_m = 1.4;
  config.same_corridor_width_m = 0.4;
  config.lateral_target_max_step_m = 100.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  const auto yield = core.update(0.1, centered_ego, {side_vehicle},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_EQ(yield.mode, overtake_planner::BehaviorMode::YIELD_BEHIND);

  const auto offset_ego = makeEgo(frame, 5.0, 0.4);
  const auto parallel_front = makeOpponent(frame, 6.0, -0.55);
  const auto follow = core.update(0.2, offset_ego, {parallel_front},
                                  overtake_planner::MpcHealthStatus{}, input);

  EXPECT_TRUE(follow.reentry_gate.requested);
  EXPECT_FALSE(follow.reentry_gate.permitted);
  // The reentry gate is still closed while its two-cycle clear confirmation is
  // pending.  A feasible parallel FOLLOW must take precedence over the
  // otherwise-selected abort recovery during this closed interval.
  EXPECT_EQ(follow.reentry_gate.reason, "reentry_clear_pending");
  EXPECT_TRUE(follow.blocked_info.parallel_follow_candidate);
  EXPECT_TRUE(follow.blocked_info.parallel_follow_feasible);
  EXPECT_EQ(follow.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(follow.selected, overtake_planner::CandidateType::FOLLOW);
  EXPECT_FALSE(follow.blocked_info.reentry_hold_active);
  ASSERT_TRUE(follow.active_override);
  ASSERT_FALSE(follow.lateral_offsets.empty());
  EXPECT_NEAR(follow.target_lateral_offset_m, offset_ego.frenet.d, 1.0e-9);
}

TEST(OvertakePlannerCore,
     Dev3D1D2WideParallelSideUsesVerifiedCurrentDFollowWhenPassUnavailable) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 12.0;
  config.parallel_follow_lateral_width_m = 1.20;
  config.parallel_side_detection_enabled = true;
  config.parallel_side_s_m = 12.0;
  config.parallel_side_margin_m = 4.0;
  // Keep the exact D1/D2 point in the wider parallel-side band while
  // excluding the separate near-zero longitudinal side-by-side arbitration.
  config.side_by_side_s_m = 0.25;
  config.same_corridor_width_m = 0.90;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.8;
  config.min_ellipse_h = 0.1;
  config.start_grace_safe_stop_enabled = false;
  // The exact start geometry remains in the parallel-side diagnostic band,
  // but no PASS candidate is enabled in this fixture.  The fallback must be
  // a SafetyEvaluator-verified current-d FOLLOW rather than FASTEST/RECOVERY.
  config.dynamic_pass_candidate_enabled = false;
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 6.0;
  auto d2 = makeOpponent(frame, 5.52, 1.60);
  d2.id = "d2";
  d2.vx = 2.0;
  d2.v = 2.0;
  const auto output = core.update(
      0.1, ego, {d2}, overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_NEAR(output.blocked_info.parallel_side_delta_s, 0.52, 1.0e-9);
  EXPECT_NEAR(output.blocked_info.parallel_side_delta_d, 1.60, 1.0e-9);
  EXPECT_TRUE(output.blocked_info.parallel_side_direction_known);
  EXPECT_TRUE(output.blocked_info.parallel_side_same_direction);
  EXPECT_EQ(output.blocked_info.parallel_side_id, "d2");
  EXPECT_TRUE(output.blocked_info.parallel_follow_recheck_attempted);
  EXPECT_EQ(output.blocked_info.parallel_follow_recheck_reason,
            "safety_evaluated_feasible");
  EXPECT_FALSE(output.blocked_info.pass_left_candidate_generated);
  EXPECT_FALSE(output.blocked_info.pass_right_candidate_generated);
  EXPECT_EQ(output.mode, overtake_planner::BehaviorMode::FOLLOW_BLOCKED);
  EXPECT_EQ(output.selected, overtake_planner::CandidateType::FOLLOW);
  ASSERT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_NEAR(output.target_lateral_offset_m, ego.frenet.d, 1.0e-9);
  for (const double lateral_offset_m : output.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, ego.frenet.d, 1.0e-9);
  }
}

TEST(OvertakePlannerCore,
     WideParallelSideRecheckKeepsSafetyEvaluatorCollisionAsFallback) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 12.0;
  config.parallel_follow_lateral_width_m = 1.20;
  config.parallel_side_detection_enabled = true;
  config.parallel_side_s_m = 12.0;
  config.parallel_side_margin_m = 4.0;
  config.same_corridor_width_m = 0.90;
  config.safety_ellipse_a_m = 3.0;
  config.safety_ellipse_b_m = 0.8;
  config.min_ellipse_h = 0.1;
  config.start_grace_safe_stop_enabled = false;
  overtake_planner::OvertakePlannerCore core(frame, config);

  const auto ego = makeEgo(frame, 5.0, 0.0);
  auto d2 = makeOpponent(frame, 5.52, 1.60);
  d2.id = "d2";
  // Initial geometry is in the wide parallel-side band, but the predicted
  // lateral closing enters the safety ellipse.  Arbitration must not turn
  // this into motion by itself.
  d2.vy = -4.0;
  const auto output = core.update(0.1, ego, {d2});

  ASSERT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_TRUE(output.blocked_info.parallel_follow_recheck_attempted);
  EXPECT_EQ(output.blocked_info.parallel_follow_recheck_reason,
            "opponent_collision");
  EXPECT_NE(output.selected, overtake_planner::CandidateType::FOLLOW);
}

TEST(OvertakePlannerCore,
     Dev3D2StartGridParallelGeometryDoesNotUseParallelRecheck) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.parallel_follow_enabled = true;
  config.parallel_follow_s_m = 12.0;
  config.parallel_follow_lateral_width_m = 1.20;
  config.parallel_side_detection_enabled = true;
  config.parallel_side_s_m = 12.0;
  config.parallel_side_margin_m = 4.0;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 3.0;
  config.start_grid_target_lateral_width_m = 2.0;
  config.start_grid_stationary_confirmation_sec = 0.0;
  config.start_grid_stationary_confirmation_distance_m = 0.0;
  overtake_planner::OvertakePlannerCore core(frame, config);

  auto ego = makeEgo(frame, 5.0, 0.0);
  ego.v = 1.0;
  auto d3 = makeOpponent(frame, 6.85, 1.48);
  d3.id = "d3";
  d3.vx = 0.0;
  d3.v = 0.0;
  const auto output = core.update(
      0.1, ego, {d3}, overtake_planner::MpcHealthStatus{}, readyReentryInput());

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.blocked_info.parallel_side_candidate);
  EXPECT_FALSE(output.blocked_info.parallel_follow_recheck_attempted);
  EXPECT_NE(output.blocked_info.parallel_follow_recheck_reason,
            "safety_evaluated_feasible");
}

TEST(OvertakePlannerCore,
     ReentryClearAllowsOnlyNonMotionWarmupForSafeStoppedTargetPass) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 2;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.dynamic_pass_candidate_enabled = true;
  config.overtake_lateral_profile_mode = "localized_latched";
  config.pass_horizon_publish_mode = "overtake_only";
  config.straight_only_overtake_enabled = true;
  config.straight_overtake_max_curvature_m_inv = 0.025;
  config.straight_overtake_lookahead_m = 8.0;
  config.slow_front_exception_enabled = true;
  config.slow_front_exception_speed_mps = 1.0;
  config.slow_front_exception_distance_m = 10.0;
  config.slow_front_exception_required_cycles = 2;
  config.stationary_obstacle_speed_threshold_mps = 0.30;
  config.future_side_prediction_enabled = false;
  config.same_corridor_width_m = 1.2;
  config.min_pass_gap_m = 0.2;
  config.safety_ellipse_b_m = 0.10;
  config.start_grid_tracking_probe_required_cycles = 2;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  side_vehicle.id = "d2";
  ASSERT_EQ(core.update(0.1, centered_ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  const auto offset_ego = makeEgo(frame, 5.0, 0.4);
  auto stopped_target = makeOpponent(frame, 13.0, -0.5);
  stopped_target.id = "d3";
  stopped_target.vx = 0.0;
  stopped_target.v = 0.0;
  stopped_target.stamp_sec = 0.2;
  const auto pending = core.update(0.2, offset_ego, {stopped_target},
                                   overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(pending.reentry_gate.requested);
  ASSERT_FALSE(pending.reentry_gate.permitted);
  EXPECT_EQ(pending.reentry_gate.reason, "reentry_clear_pending");
  EXPECT_TRUE(pending.mode == overtake_planner::BehaviorMode::ABORT_RECOVERY ||
              pending.mode == overtake_planner::BehaviorMode::SPEED_GUARD)
      << overtake_planner::toString(pending.mode);
  EXPECT_FALSE(pending.tracking_release_pass_warmup);
  EXPECT_FALSE(
      pending.blocked_info.maneuver_transaction_tracking_release_pending);

  stopped_target.stamp_sec = 0.3;
  const auto warmed = core.update(0.3, offset_ego, {stopped_target},
                                  overtake_planner::MpcHealthStatus{}, input);
  SCOPED_TRACE(
      ::testing::Message()
      << " mode=" << overtake_planner::toString(warmed.mode)
      << " selected=" << overtake_planner::toString(warmed.selected)
      << " slow_exception=" << warmed.blocked_info.slow_front_exception_active
      << " target=" << warmed.blocked_info.maneuver_target_id
      << " target_latched=" << warmed.blocked_info.maneuver_target_latched
      << " target_observed=" << warmed.blocked_info.maneuver_target_observed
      << " target_fresh=" << warmed.blocked_info.maneuver_target_fresh
      << " chain=" << warmed.blocked_info.maneuver_chain_tail_id
      << " chain_observed=" << warmed.blocked_info.maneuver_chain_tail_observed
      << " nearest=" << warmed.blocked_info.nearest_id << " pass_type="
      << overtake_planner::toString(
             warmed.blocked_info.maneuver_transaction_pass_type)
      << " left_ok=" << warmed.blocked_info.pass_left_candidate_feasible
      << " left_profile="
      << warmed.blocked_info.pass_left_candidate_tracking_profile_valid
      << " left_desired="
      << warmed.blocked_info.pass_left_candidate_desired_path_trackable
      << " left_pp="
      << warmed.blocked_info.pass_left_candidate_pure_pursuit_command_trackable
      << " left_arc=" << warmed.blocked_info.pass_left_candidate_endpoint_arc_m
      << "/" << warmed.blocked_info.pass_left_candidate_required_arc_m
      << " left_reason="
      << warmed.blocked_info.pass_left_candidate_reject_reason << " right_ok="
      << warmed.blocked_info.pass_right_candidate_feasible << " right_reason="
      << warmed.blocked_info.pass_right_candidate_reject_reason
      << " gate=" << warmed.blocked_info.overtake_start_gate_reason
      << " reentry_reason=" << warmed.reentry_gate.reason
      << " permission=" << warmed.blocked_info.overtake_permission_allowed
      << " frozen=" << warmed.blocked_info.pass_decision_frozen
      << " reauth=" << warmed.blocked_info.pass_reauthorization_lockout_active
      << " post_abort=" << warmed.blocked_info.post_abort_curve_hold_active
      << " side=" << warmed.blocked_info.side_by_side
      << " corner_side=" << warmed.blocked_info.corner_side_by_side
      << " future_corner=" << warmed.blocked_info.future_corner_side_by_side
      << " future_yield=" << warmed.blocked_info.future_yield_required);
  ASSERT_TRUE(warmed.reentry_gate.requested);
  ASSERT_TRUE(warmed.reentry_gate.permitted) << warmed.reentry_gate.reason;
  ASSERT_TRUE(warmed.blocked_info.pass_left_candidate_feasible ||
              warmed.blocked_info.pass_right_candidate_feasible);
  EXPECT_TRUE(warmed.tracking_release_pass_warmup) << warmed.reason;
  EXPECT_TRUE(
      warmed.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(warmed.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(
      warmed.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_NE(warmed.tracking_release_token, 0U);
  EXPECT_NE(warmed.mode, overtake_planner::BehaviorMode::OVERTAKE_LEFT);
  EXPECT_NE(warmed.mode, overtake_planner::BehaviorMode::OVERTAKE_RIGHT);

  const auto constraint = overtake_planner::makeSafetyConstraint(
      warmed, offset_ego, input, 8.0, 1.0);
  EXPECT_TRUE(constraint.valid);
  EXPECT_TRUE(constraint.stop_requested);
  EXPECT_FALSE(constraint.release_authorized);

  // D1実bagで観測したN/N-1配送順差はfaultへ昇格させず、既に発行した
  // 非走行probeを停止状態で一時停止する。旧tokenやprobe countをNの証明へ
  // 読み替えず、exact current tupleが来るまで進行させない。
  const auto warmup_token = warmed.tracking_release_token;
  auto gap_input = readyReentryInput();
  gap_input.pure_pursuit_primary_and_fresh = false;
  gap_input.start_grid_pass_probe_transport_evidence =
      overtake_planner::StartGridProbeTransportEvidence::NORMAL_DELIVERY_GAP;
  stopped_target.stamp_sec = 0.31;
  const auto paused =
      core.update(0.31, offset_ego, {stopped_target},
                  overtake_planner::MpcHealthStatus{}, gap_input);
  EXPECT_TRUE(
      paused.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(paused.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(
      paused.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_EQ(paused.blocked_info.maneuver_transaction_tracking_probe_cycles, 0);
  // gap周期は保存tokenを再publishせず、走行権限を持たない。
  EXPECT_EQ(paused.tracking_release_token, 0U);

  // 配送gap解消後も、exact sampleが無いだけではprobe countを増やさない。
  // 同じtokenのexact Nを受けた周期だけ1件目のproofとして進める。
  auto exact_input = readyReentryInput();
  exact_input.pass_probe_exact_current_usable = true;
  exact_input.pass_probe_lateral_stop_authority_token = warmup_token;
  exact_input.pass_probe_exact_plan_generation = 19U;
  exact_input.pass_probe_exact_sample_stamp_sec = 0.32;
  stopped_target.stamp_sec = 0.32;
  const auto exact_one =
      core.update(0.32, offset_ego, {stopped_target},
                  overtake_planner::MpcHealthStatus{}, exact_input);
  EXPECT_TRUE(
      exact_one.blocked_info.maneuver_transaction_tracking_release_pending);
  EXPECT_TRUE(exact_one.blocked_info.maneuver_transaction_tracking_stop_active);
  EXPECT_FALSE(
      exact_one.blocked_info.maneuver_transaction_tracking_release_confirmed);
  EXPECT_EQ(exact_one.blocked_info.maneuver_transaction_tracking_probe_cycles,
            1);
  // exact proofは内部の同一probeだけを進める。新しいwarm-up transactionを
  // 発行した周期ではないため、出力tokenは引き続き0である。
  EXPECT_EQ(exact_one.tracking_release_token, 0U);
}

TEST(OvertakePlannerCore,
     ReentryGateKeepsRecoveryUntilPermittedTrajectoryPhysicallyConverges) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.dynamic_pass_candidate_enabled = true;
  config.reentry_completion_lateral_error_m = 0.20;
  config.reentry_completion_rearm_lateral_error_m = 0.30;
  config.overtake_permission_profile_enabled = true;
  config.overtake_permission_lookahead_m = 0.0;
  config.overtake_permission_rules.push_back(
      overtake_planner::OvertakePermissionRule{"no_pass", 0.0, 20.0, false});
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  // d=0.30 mを超えた時点で復帰phaseを開始する。
  const auto armed = core.update(0.2, makeEgo(frame, 5.0, 0.31), {},
                                 overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(armed.reentry_gate.requested);
  ASSERT_TRUE(armed.reentry_gate.permitted);

  // 許可済みならABORT holdを解除する。ただし、まだd=0.21 mなので
  // FREE_RUNへは抜けず、SafetyEvaluator済みRECOVERYだけを出し続ける。
  const auto residual = core.update(0.3, makeEgo(frame, 5.0, 0.21), {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_TRUE(residual.reentry_gate.requested);
  EXPECT_TRUE(residual.reentry_gate.permitted);
  EXPECT_EQ(residual.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(residual.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_FALSE(residual.blocked_info.overtake_permission_allowed);
  EXPECT_FALSE(residual.blocked_info.reentry_hold_active);
  EXPECT_TRUE(residual.blocked_info.reentry_centering_authorized);
  EXPECT_TRUE(residual.active_override);
  EXPECT_FALSE(residual.lateral_offsets.empty());
  EXPECT_LT(residual.target_lateral_offset_m, 0.21);

  // 一度許可されてもfreshnessを失った周期は即座にABORT holdへ戻す。
  auto stale_input = input;
  stale_input.v2x_snapshot_fresh = false;
  const auto stale =
      core.update(0.35, makeEgo(frame, 5.0, 0.21), {},
                  overtake_planner::MpcHealthStatus{}, stale_input);
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_FALSE(stale.reentry_gate.permitted);
  EXPECT_TRUE(stale.blocked_info.reentry_hold_active);
  EXPECT_FALSE(stale.blocked_info.reentry_centering_authorized);

  // 安全許可済みかつ設定したd=0.20 m以内へ収束して初めてphaseを解放する。
  const auto complete = core.update(0.4, makeEgo(frame, 5.0, 0.19), {},
                                    overtake_planner::MpcHealthStatus{}, input);
  EXPECT_FALSE(complete.reentry_gate.requested);
  EXPECT_EQ(complete.mode, overtake_planner::BehaviorMode::FREE_RUN);
}

TEST(OvertakePlannerCore,
     PermittedCenteredReentryUsesSafetyEvaluatedPostAbortCurveGuard) {
  const auto frame = makeCurvedFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.dynamic_pass_candidate_enabled = true;
  config.high_speed_curve_lateral_hold_enabled = true;
  config.high_speed_curve_lateral_hold_release_speed_mps = 2.5;
  config.high_speed_curve_lateral_hold_release_curvature_m_inv = 0.025;
  config.post_abort_curve_hold_v_max_mps = 3.5;
  config.corner_yield_v_max_mps = 3.5;
  config.lateral_target_max_step_m = 100.0;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  auto centered_ego = makeEgo(frame, 5.0, 0.0);
  centered_ego.v = 5.0;
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  auto residual_ego = makeEgo(frame, 5.0, 0.49);
  residual_ego.v = 5.0;
  const auto residual = core.update(0.2, residual_ego, {},
                                    overtake_planner::MpcHealthStatus{}, input);
  ASSERT_TRUE(residual.reentry_gate.requested);
  ASSERT_TRUE(residual.reentry_gate.permitted);
  ASSERT_EQ(residual.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_TRUE(residual.blocked_info.reentry_centering_authorized);

  auto complete_ego = makeEgo(frame, 5.0, 0.04);
  complete_ego.v = 5.0;
  const auto complete = core.update(0.3, complete_ego, {},
                                    overtake_planner::MpcHealthStatus{}, input);

  EXPECT_FALSE(complete.reentry_gate.requested);
  EXPECT_EQ(complete.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  EXPECT_EQ(complete.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(complete.blocked_info.post_abort_curve_hold_active);
  EXPECT_TRUE(complete.blocked_info.pass_decision_frozen);
  EXPECT_EQ(complete.blocked_info.pass_decision_freeze_reason,
            "post_abort_curve_hold");
  EXPECT_TRUE(complete.active_override);
  EXPECT_EQ(complete.speed_cap_reason, "post_abort_curve_hold");
  EXPECT_LE(complete.applied_speed_cap_mps,
            config.post_abort_curve_hold_v_max_mps + 1.0e-9);
  ASSERT_FALSE(complete.lateral_offsets.empty());
  for (const double lateral_offset_m : complete.lateral_offsets) {
    EXPECT_NEAR(lateral_offset_m, complete_ego.frenet.d, 1.0e-9);
  }

  auto stale_ego = complete_ego;
  stale_ego.valid = false;
  const auto stale =
      core.update(0.4, stale_ego, {}, overtake_planner::MpcHealthStatus{}, {});
  EXPECT_EQ(stale.mode, overtake_planner::BehaviorMode::ABORT_RECOVERY);
  EXPECT_EQ(stale.selected, overtake_planner::CandidateType::RECOVERY);
  EXPECT_TRUE(stale.reentry_gate.requested);
  EXPECT_FALSE(stale.reentry_gate.permitted);
  EXPECT_FALSE(stale.active_override);
  EXPECT_TRUE(stale.longitudinal_speed_cap_active);
  EXPECT_TRUE(stale.lateral_offsets.empty());
  EXPECT_EQ(stale.reason, "post_abort_curve_hold_stale_ego_abort");
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

TEST(OvertakePlannerCore,
     ReentryGateEvaluatesCenteringInsteadOfStartGridAnchorHold) {
  const auto frame = makeStraightFrame();
  auto config = makeConfig();
  config.reentry_gate_enabled = true;
  config.reentry_safe_cycles = 1;
  config.reentry_min_safety_margin_h = 0.15;
  config.reentry_evaluation_horizon_sec = 4.0;
  config.start_grid_target_enabled = true;
  config.start_grid_target_window_sec = 5.0;
  config.start_grid_target_window_distance_m = 8.0;
  config.start_grid_target_max_ego_speed_mps = 5.0;
  config.start_grid_target_min_delta_s_m = 2.0;
  config.start_grid_target_max_delta_s_m = 12.0;
  config.start_grid_target_lateral_width_m = 0.25;
  config.dynamic_pass_candidate_enabled = true;
  overtake_planner::OvertakePlannerCore core(frame, config);
  const auto input = readyReentryInput();

  // start-grid分類に入らない近接side車で、まずYIELD由来のreentry phaseを作る。
  const auto centered_ego = makeEgo(frame, 5.0, 0.0);
  const auto side_vehicle = makeOpponent(frame, 5.6, 0.6);
  ASSERT_EQ(core.update(0.1, centered_ego, {side_vehicle},
                        overtake_planner::MpcHealthStatus{}, input)
                .mode,
            overtake_planner::BehaviorMode::YIELD_BEHIND);

  // 次周期はcurrent d付近のmoving start-grid対象を分類する。publish用FOLLOWは
  // current-d/anchorを保持してよいが、reentry gateはそのHOLD列ではなくd=0までの
  // 全復帰を評価しなければならない。
  auto offset_ego = makeEgo(frame, 5.0, 0.8);
  offset_ego.v = 4.0;
  auto moving_target = makeOpponent(frame, 13.0, 0.9);
  moving_target.id = "moving_grid_d3";
  moving_target.stamp_sec = 0.2;
  moving_target.v = 3.0;
  moving_target.vx = 3.0;
  const auto output = core.update(0.2, offset_ego, {moving_target},
                                  overtake_planner::MpcHealthStatus{}, input);

  ASSERT_TRUE(output.blocked_info.start_grid_target_active);
  EXPECT_TRUE(output.reentry_gate.requested);
  EXPECT_TRUE(output.reentry_gate.input_complete);
  EXPECT_TRUE(output.reentry_gate.permitted) << output.reentry_gate.reason;
  EXPECT_EQ(output.reentry_gate.reason, "reentry_clear");
  EXPECT_NE(output.reentry_gate.reason, "incomplete_reentry_centering_horizon");
  EXPECT_NE(output.reentry_gate.reason, "untrackable_lateral_profile");
  ASSERT_EQ(output.mode, overtake_planner::BehaviorMode::SPEED_GUARD);
  ASSERT_EQ(output.selected, overtake_planner::CandidateType::RECOVERY);
  ASSERT_TRUE(output.active_override);
  ASSERT_FALSE(output.lateral_offsets.empty());
  EXPECT_LT(std::abs(output.target_lateral_offset_m),
            std::abs(offset_ego.frenet.d));
  EXPECT_LT(std::abs(output.lateral_offsets.back()),
            std::abs(offset_ego.frenet.d));
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
  EXPECT_FALSE(released.reentry_gate.requested);
  EXPECT_EQ(released.mode, overtake_planner::BehaviorMode::FREE_RUN);
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
