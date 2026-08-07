#include "overtake_planner/cartesian_trackability_evaluator.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace {

overtake_planner::FrenetFrame makeStraightFrame() {
  overtake_planner::FrenetFrame frame;
  frame.setReference({
      {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
      {100.0, 100.0, 0.0, 0.0, 0.0, 5.0},
      {200.0, 100.0, 100.0, 1.5707963267948966, 0.0, 5.0},
      {300.0, 0.0, 100.0, 3.1415926535897932, 0.0, 5.0},
  });
  return frame;
}

overtake_planner::CandidateTrajectory
makeCandidate(const std::vector<double> &x, const std::vector<double> &y,
              double duration_sec) {
  overtake_planner::CandidateTrajectory candidate;
  candidate.x = x;
  candidate.y = y;
  candidate.yaw.resize(x.size(), 0.0);
  double total_arc_m = 0.0;
  for (std::size_t index = 1U; index < x.size(); ++index) {
    total_arc_m +=
        std::hypot(x[index] - x[index - 1U], y[index] - y[index - 1U]);
  }
  const double speed_mps = total_arc_m / duration_sec;
  candidate.predicted_speed_mps.resize(x.size(), speed_mps);
  candidate.v_ref.resize(x.size(), speed_mps);
  candidate.longitudinal_offsets_m.resize(x.size(), 0.0);
  double accumulated_arc_m = 0.0;
  for (std::size_t index = 0U; index < x.size(); ++index) {
    if (index > 0U) {
      accumulated_arc_m +=
          std::hypot(x[index] - x[index - 1U], y[index] - y[index - 1U]);
    }
    candidate.longitudinal_offsets_m[index] = accumulated_arc_m;
    candidate.t.push_back(duration_sec * static_cast<double>(index) /
                          static_cast<double>(x.size() - 1U));
  }
  return candidate;
}

overtake_planner::CartesianTrackabilityConfig baseConfig() {
  overtake_planner::CartesianTrackabilityConfig config;
  config.wheelbase_m = 1.0;
  config.steering_gain = 1.0;
  config.max_steering_angle_rad = 0.8;
  config.max_steering_rate_radps = 100.0;
  config.max_yaw_tangent_error_rad = 1.5;
  config.speed_consistency_abs_tolerance_mps = 2.0;
  config.pure_pursuit_required_arc_m = 1.0;
  config.target_d_m = 0.0;
  config.target_d_deadline_arc_m = 1.0;
  config.target_d_tolerance_m = 0.05;
  return config;
}

overtake_planner::CartesianTrackabilityInput baseInput() {
  overtake_planner::CartesianTrackabilityInput input;
  input.ego.valid = true;
  input.ego.x = 1.0;
  input.ego.y = 0.0;
  input.ego.yaw = 0.0;
  input.ego.v = 1.0;
  input.active_lookahead_valid = true;
  input.active_lookahead_distance_m = 1.0;
  input.nearest_source_index_valid = true;
  input.nearest_source_index = 0U;
  input.curvature_feedforward_valid = true;
  input.curvature_feedforward_steering_rad = 0.0;
  input.steering_reference_valid = true;
  input.steering_reference_angle_rad = 0.0;
  input.steering_command_dt_sec = 1.0;
  return input;
}

TEST(CartesianTrackabilityEvaluator, AcceptsValidStraightCartesianCandidate) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0),
                         baseInput(), baseConfig());

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_TRUE(result.pure_pursuit_command_evaluated);
  EXPECT_TRUE(result.pure_pursuit_command_trackable);
  EXPECT_TRUE(result.trackable);
  EXPECT_EQ(result.reason, "ok");
  EXPECT_LE(result.resampled_point_count, 2000U);
  EXPECT_NEAR(result.total_arc_m, 2.0, 1.0e-12);
  EXPECT_NEAR(result.max_abs_curvature_m_inv, 0.0, 1.0e-12);
}

TEST(CartesianTrackabilityEvaluator, RejectsNanAndInfinity) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto nan_candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  nan_candidate.x[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      evaluator.evaluate(nan_candidate, baseInput(), baseConfig()).valid);

  auto inf_candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  inf_candidate.yaw[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
      evaluator.evaluate(inf_candidate, baseInput(), baseConfig()).valid);
}

TEST(CartesianTrackabilityEvaluator, RejectsZeroSegment) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 1.0, 2.0}, {0.0, 0.0, 0.0}, 2.0),
                         baseInput(), baseConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "zero_or_invalid_cartesian_segment");
}

TEST(CartesianTrackabilityEvaluator, RejectsYawTangentMismatch) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  candidate.yaw[1] = 0.4;
  auto config = baseConfig();
  config.max_yaw_tangent_error_rad = 0.1;
  const auto result = evaluator.evaluate(candidate, baseInput(), config);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "cartesian_yaw_tangent_mismatch");
}

TEST(CartesianTrackabilityEvaluator, RejectsNonmonotonicCandidateTime) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  candidate.t[2] = candidate.t[1];
  const auto result = evaluator.evaluate(candidate, baseInput(), baseConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "nonfinite_or_nonmonotonic_cartesian_candidate");
}

TEST(CartesianTrackabilityEvaluator, RejectsMoreThanTwoThousandInputPoints) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  std::vector<double> x(2001U);
  std::vector<double> y(2001U, 0.0);
  for (std::size_t index = 0U; index < x.size(); ++index) {
    x[index] = 1.0 + 0.001 * static_cast<double>(index);
  }
  const auto result =
      evaluator.evaluate(makeCandidate(x, y, 2.0), baseInput(), baseConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "invalid_cartesian_candidate_size");
}

TEST(CartesianTrackabilityEvaluator,
     RejectsMoreThanTwoThousandResampledPoints) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 51.0, 101.0}, {0.0, 0.0, 0.0}, 10.0), baseInput(),
      baseConfig());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "resampled_point_limit_exceeded");
}

TEST(CartesianTrackabilityEvaluator, RejectsSteeringAngleLimit) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_angle_rad = 0.1;
  config.target_d_deadline_arc_m = 10.0;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 1.5, 2.0}, {-0.5, 0.5, 0.0}, 2.0),
                         baseInput(), config);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "steering_angle_limit_exceeded");
}

TEST(CartesianTrackabilityEvaluator, RejectsSteeringRateFromCandidateTime) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_angle_rad = 1.5;
  config.max_steering_rate_radps = 0.1;
  config.target_d_m = 0.8;
  config.target_d_deadline_arc_m = 10.0;
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 1.5, 2.0, 2.5}, {0.0, 0.0, 0.2, 0.8}, 0.2),
      baseInput(), config);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "steering_rate_limit_exceeded");
}

TEST(CartesianTrackabilityEvaluator, RejectsInsufficientPurePursuitArc) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.pure_pursuit_required_arc_m = 2.1;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0),
                         baseInput(), config);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "insufficient_pure_pursuit_arc");
}

TEST(CartesianTrackabilityEvaluator, RejectsTargetDReachedAfterFrenetDeadline) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 1.0;
  config.max_steering_angle_rad = 1.5;
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 2.0, 3.0, 4.0}, {0.0, 0.1, 0.5, 1.0}, 3.0),
      baseInput(), config);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "target_d_deadline_exceeded");
  EXPECT_GT(result.target_d_reach_arc_m, config.target_d_deadline_arc_m);
}

TEST(CartesianTrackabilityEvaluator,
     InterpolatesFirstEntryIntoTargetToleranceForDeadline) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto candidate = makeCandidate({1.0, 2.0, 3.0}, {0.20, 0.12, 0.04}, 2.0);
  auto input = baseInput();
  input.ego.valid = false;
  auto config = baseConfig();
  config.target_d_deadline_arc_m = 1.90;

  const auto result = evaluator.evaluate(candidate, input, config);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_NEAR(result.target_d_reach_arc_m,
              std::hypot(1.0, 0.08) * (1.0 + 0.07 / 0.08), 1.0e-9);
  EXPECT_EQ(result.reason, "pure_pursuit_command_snapshot_unavailable");
}

TEST(CartesianTrackabilityEvaluator,
     KeepsPurePursuitProofIndeterminateWithoutExactSteeringSnapshot) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto input = baseInput();
  input.steering_reference_valid = false;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0),
                         input, baseConfig());

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_FALSE(result.pure_pursuit_command_evaluated);
  EXPECT_FALSE(result.pure_pursuit_command_trackable);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "pure_pursuit_command_snapshot_unavailable");
}

TEST(CartesianTrackabilityEvaluator,
     SelectsActivePurePursuitTargetFromRearAxleAndAppliesRawFormula) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0),
                         baseInput(), baseConfig());

  ASSERT_TRUE(result.pure_pursuit_command_evaluated);
  EXPECT_EQ(result.pure_pursuit_target_source_index, 1U);
  EXPECT_NEAR(result.pure_pursuit_lookahead_distance_m, 1.0, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_geometric_steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_raw_steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_requested_steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_bounded_steering_angle_rad, 0.0, 1.0e-12);
}

TEST(CartesianTrackabilityEvaluator,
     UsesExactActiveNearestIndexInsteadOfPlannerDistanceResearch) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto input = baseInput();
  input.active_lookahead_distance_m = 0.4;
  input.nearest_source_index = 2U;
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 0.0, 0.0}, 3.0), input,
      baseConfig());

  ASSERT_TRUE(result.trackable) << result.reason;
  EXPECT_EQ(result.pure_pursuit_target_source_index, 2U);
}

TEST(CartesianTrackabilityEvaluator,
     RejectsFeedforwardThatMakesExactRawPurePursuitCommandAngleLimited) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto input = baseInput();
  input.curvature_feedforward_steering_rad = 0.9;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0),
                         input, baseConfig());

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_TRUE(result.pure_pursuit_command_evaluated);
  EXPECT_FALSE(result.pure_pursuit_command_trackable);
  EXPECT_NEAR(result.pure_pursuit_geometric_steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_raw_steering_angle_rad, 0.9, 1.0e-12);
  EXPECT_EQ(result.reason, "pure_pursuit_command_angle_limited");
}

TEST(CartesianTrackabilityEvaluator,
     RejectsPurePursuitCommandWhenActiveRateBoundWouldClipIt) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_rate_radps = 1.0;
  auto input = baseInput();
  input.steering_reference_angle_rad = 0.5;
  input.steering_command_dt_sec = 0.01;
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0), input, config);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_TRUE(result.pure_pursuit_command_evaluated);
  EXPECT_FALSE(result.pure_pursuit_command_trackable);
  EXPECT_EQ(result.reason, "pure_pursuit_command_rate_limited");
  EXPECT_NEAR(result.pure_pursuit_bounded_steering_angle_rad, 0.49, 1.0e-12);
}

TEST(CartesianTrackabilityEvaluator, RejectsNonfinitePurePursuitRateAllowance) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto input = baseInput();
  input.steering_command_dt_sec = std::numeric_limits<double>::max();
  auto config = baseConfig();
  config.max_steering_rate_radps = std::numeric_limits<double>::max();

  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0), input, config);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable);
  EXPECT_FALSE(result.pure_pursuit_command_evaluated);
  EXPECT_FALSE(result.pure_pursuit_command_trackable);
  EXPECT_EQ(result.reason, "invalid_pure_pursuit_rate_allowance");
}

TEST(CartesianTrackabilityEvaluator,
     RejectsMissingOrInvalidCandidateSpeedColumns) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto missing = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  missing.predicted_speed_mps.pop_back();
  EXPECT_EQ(evaluator.evaluate(missing, baseInput(), baseConfig()).reason,
            "invalid_cartesian_candidate_size");

  auto negative = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 2.0);
  negative.v_ref[1] = -0.1;
  EXPECT_EQ(evaluator.evaluate(negative, baseInput(), baseConfig()).reason,
            "nonfinite_or_nonmonotonic_cartesian_candidate");
}

TEST(CartesianTrackabilityEvaluator,
     RejectsLongitudinalAxisTimeThatDisagreesWithPredictedSpeed) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 0.2);
  candidate.predicted_speed_mps.assign(candidate.x.size(), 1.0);
  auto config = baseConfig();
  config.speed_consistency_abs_tolerance_mps = 0.05;
  config.speed_consistency_relative_tolerance = 0.01;
  const auto result = evaluator.evaluate(candidate, baseInput(), config);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "execution_speed_inconsistent_with_prediction");
}

TEST(CartesianTrackabilityEvaluator,
     ValidatesTimeAgainstLongitudinalAxisNotLateralCartesianArc) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto candidate = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.5, 1.0}, 2.0);
  candidate.longitudinal_offsets_m = {0.0, 1.0, 2.0};
  candidate.predicted_speed_mps = {1.0, 1.0, 1.0};
  candidate.v_ref = {1.0, 1.0, 1.0};
  auto config = baseConfig();
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 3.0;
  config.max_steering_angle_rad = 1.5;
  auto input = baseInput();
  input.steering_reference_valid = false;

  const auto result = evaluator.evaluate(candidate, input, config);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.desired_path_trackable) << result.reason;
  EXPECT_EQ(result.reason, "pure_pursuit_command_snapshot_unavailable");
  EXPECT_GT(result.total_arc_m, candidate.longitudinal_offsets_m.back());
}

TEST(CartesianTrackabilityEvaluator,
     AllowsDelayedBrakingAboveVRefButRejectsPredictionOutsideEnvelope) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto delayed_braking = makeCandidate({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0}, 0.5);
  delayed_braking.predicted_speed_mps = {5.0, 5.0, 4.0};
  delayed_braking.v_ref = {2.0, 2.0, 2.0};
  auto config = baseConfig();
  config.speed_consistency_abs_tolerance_mps = 4.0;
  const auto allowed = evaluator.evaluate(delayed_braking, baseInput(), config);
  EXPECT_TRUE(allowed.desired_path_trackable) << allowed.reason;

  delayed_braking.predicted_speed_mps[1] = 5.2;
  const auto rejected =
      evaluator.evaluate(delayed_braking, baseInput(), config);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(rejected.reason, "predicted_speed_outside_v_ref_envelope");
}

TEST(CartesianTrackabilityEvaluator,
     AcceptsSameSideOvershootWhenTerminalReturnsInsideTolerance) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_angle_rad = 1.5;
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 4.0;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0, 4.0, 5.0},
                                       {0.0, 0.5, 1.15, 1.10, 1.02}, 4.0),
                         baseInput(), config);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.desired_path_trackable) << result.reason;
  EXPECT_TRUE(result.trackable) << result.reason;
  EXPECT_NEAR(result.terminal_projected_d_m, 1.02, 1.0e-6);
}

TEST(CartesianTrackabilityEvaluator, RejectsTargetCrossBackAfterFirstReach) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_angle_rad = 1.5;
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 4.0;
  const auto result =
      evaluator.evaluate(makeCandidate({1.0, 2.0, 3.0, 4.0, 5.0},
                                       {0.0, 0.6, 1.10, 0.80, 1.0}, 4.0),
                         baseInput(), config);

  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "target_d_cross_back");
}

TEST(CartesianTrackabilityEvaluator,
     RejectsTargetCrossingWithWrongTerminalOffset) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  auto config = baseConfig();
  config.max_steering_angle_rad = 1.5;
  config.target_d_m = 1.0;
  config.target_d_deadline_arc_m = 4.0;
  const auto result = evaluator.evaluate(
      makeCandidate({1.0, 2.0, 3.0, 4.0}, {0.0, 0.6, 1.10, 1.20}, 3.0),
      baseInput(), config);

  EXPECT_FALSE(result.trackable);
  EXPECT_EQ(result.reason, "target_d_wrong_terminal");
}

TEST(CartesianTrackabilityEvaluator,
     ReportsObservedDurationForBoundedTwoThousandPointFixture) {
  const auto frame = makeStraightFrame();
  const overtake_planner::CartesianTrackabilityEvaluator evaluator(frame);
  std::vector<double> x(2000U);
  std::vector<double> y(2000U, 0.0);
  constexpr double spacing_m = 0.049;
  for (std::size_t index = 0U; index < x.size(); ++index) {
    x[index] = 1.0 + spacing_m * static_cast<double>(index);
  }
  auto config = baseConfig();
  config.target_d_deadline_arc_m = x.back() - x.front();
  const auto candidate = makeCandidate(x, y, x.back() - x.front());

  const auto started = std::chrono::steady_clock::now();
  const auto result = evaluator.evaluate(candidate, baseInput(), config);
  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - started)
                              .count();

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.resampled_point_count, 2000U);
  RecordProperty("fixture_point_count", 2000);
  RecordProperty("observed_elapsed_us", elapsed_us);
  RecordProperty("realtime_assertion", "none_observational_fixture");
}

} // namespace
