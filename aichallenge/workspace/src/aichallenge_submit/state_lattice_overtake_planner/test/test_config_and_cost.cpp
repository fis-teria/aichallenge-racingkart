#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/cost_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace sl = state_lattice_overtake_planner;

TEST(Config, DefaultsAreValidAndUnsafeOrderingIsRejected) {
  sl::PlannerConfig config;
  const auto default_error = sl::validateConfig(config);
  EXPECT_TRUE(default_error.empty()) << default_error;
  EXPECT_DOUBLE_EQ(config.hard_max_steer_rad, 0.64);
  EXPECT_DOUBLE_EQ(config.planner_max_steer_rad, 0.64);
  EXPECT_DOUBLE_EQ(config.max_steer_rate_radps, 128.0);
  config.stop_cost = config.free_run_return_cost;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.collision_max_step_m = 0.0;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.tangent_scales[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.follow_gap_gain_per_s = 0.0;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_trigger_clearance_m =
      config.opponent_hard_clearance_m;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_tie_band_m =
      config.preventive_side_role_max_abs_delta_s_m;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_min_tangent_progress_mps = 0.0;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_max_track_heading_error_rad = M_PI_2;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_max_relative_heading_error_rad = M_PI_2;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_separation_epsilon_m = 0.0;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_prediction_step_sec =
      config.preventive_side_role_prediction_horizon_sec + 0.1;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_yield_min_samples = 2;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_exit_clearance_m =
      config.preventive_side_role_trigger_clearance_m;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.preventive_side_role_escape_crawl_speed_mps =
      config.normal_speed_mps + 0.1;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.early_aware_max_distance_m = config.early_aware_min_distance_m - 0.1;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.early_aware_max_deceleration_mps2 = 0.0;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.early_aware_max_track_heading_error_rad = M_PI_2;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.early_aware_max_distance_m = 30.0001;
  EXPECT_FALSE(sl::validateConfig(config).empty());
  config = sl::PlannerConfig{};
  config.early_aware_max_distance_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(sl::validateConfig(config).empty());
}

TEST(Config, ControllerTrackabilityProfileIsLaunchBoundAndFailClosed) {
  using Profile = sl::ControllerTrackabilityProfile;
  EXPECT_EQ(sl::parseControllerTrackabilityProfile("shadow_only"),
            Profile::SHADOW_ONLY);
  EXPECT_EQ(sl::parseControllerTrackabilityProfile("pure_pursuit"),
            Profile::PURE_PURSUIT);
  EXPECT_EQ(sl::parseControllerTrackabilityProfile("instant"),
            Profile::INSTANT);
  EXPECT_EQ(sl::parseControllerTrackabilityProfile("other"), Profile::UNKNOWN);

  EXPECT_TRUE(sl::validateControllerTrackabilityProfileContract(
                  Profile::SHADOW_ONLY, false, false)
                  .empty());
  EXPECT_TRUE(sl::validateControllerTrackabilityProfileContract(
                  Profile::PURE_PURSUIT, true, false)
                  .empty());
  EXPECT_TRUE(sl::validateControllerTrackabilityProfileContract(
                  Profile::INSTANT, true, true)
                  .empty());

  EXPECT_FALSE(sl::validateControllerTrackabilityProfileContract(
                   Profile::UNKNOWN, false, false)
                   .empty());
  EXPECT_FALSE(sl::validateControllerTrackabilityProfileContract(
                   Profile::PURE_PURSUIT, false, false)
                   .empty());
  EXPECT_FALSE(sl::validateControllerTrackabilityProfileContract(
                   Profile::INSTANT, true, false)
                   .empty());
  EXPECT_FALSE(sl::validateControllerTrackabilityProfileContract(
                   Profile::PURE_PURSUIT, true, true)
                   .empty());
}

TEST(Config, ControllerTrackabilityEnvelopeSeparatesInstantFromPurePursuit) {
  sl::PlannerConfig pure_pursuit;
  pure_pursuit.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  sl::applyResolvedControllerTrackabilityEnvelope(&pure_pursuit, 0.5236, 0.35);
  EXPECT_DOUBLE_EQ(pure_pursuit.planner_max_steer_rad, 0.64);
  EXPECT_DOUBLE_EQ(pure_pursuit.max_steer_rate_radps, 128.0);

  sl::PlannerConfig instant;
  instant.controller_trackability_profile =
      sl::ControllerTrackabilityProfile::INSTANT;
  sl::applyResolvedControllerTrackabilityEnvelope(&instant, 0.5236, 0.35);
  EXPECT_DOUBLE_EQ(instant.planner_max_steer_rad, 0.5236);
  EXPECT_DOUBLE_EQ(instant.max_steer_rate_radps, 0.35);
}

TEST(Config, VehicleSteeringEnvelopeIsFailClosed) {
  sl::PlannerConfig config;
  config.hard_max_steer_rad = 0.640001;
  EXPECT_FALSE(sl::validateConfig(config).empty());

  config = sl::PlannerConfig{};
  config.planner_max_steer_rad = 0.640001;
  config.hard_max_steer_rad = 0.640001;
  EXPECT_FALSE(sl::validateConfig(config).empty());

  config = sl::PlannerConfig{};
  config.max_steer_rate_radps = 128.000001;
  EXPECT_FALSE(sl::validateConfig(config).empty());
}

TEST(Config, LateralTargetsAreDefinedByRuntimeParameters) {
  sl::PlannerConfig config;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};

  EXPECT_TRUE(sl::validateConfig(config).empty());

  config.lateral_targets_m = {0.2, 1.0, -1.0, 2.4, -2.4};
  EXPECT_FALSE(sl::validateConfig(config).empty());

  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, 2.4};
  EXPECT_FALSE(sl::validateConfig(config).empty());

  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4};
  EXPECT_FALSE(sl::validateConfig(config).empty());
}

TEST(CostModel, DistanceBoundariesAndMergeRules) {
  const sl::PlannerConfig config;
  EXPECT_EQ(sl::wallCostLevel(0.0999, config), 9);
  EXPECT_EQ(sl::wallCostLevel(0.1, config), 7);
  EXPECT_EQ(sl::wallCostLevel(0.3, config), 6);
  EXPECT_EQ(sl::wallCostLevel(0.5, config), 0);
  EXPECT_EQ(sl::objectCostLevel(0.8, config), 8);
  EXPECT_EQ(sl::objectCostLevel(1.7, config), 0);
  EXPECT_EQ(sl::referenceCostLevel(1.499, config), 3);
  EXPECT_EQ(sl::referenceCostLevel(1.5, config), 4);
  EXPECT_EQ(sl::referenceCostLevel(1.7, config), 5);
  EXPECT_EQ(sl::referenceCostLevel(1.9, config), 6);
  EXPECT_EQ(sl::mergeCostLevels(7, 8), 9);
  EXPECT_EQ(sl::mergeCostLevels(5, 6), 7);
  EXPECT_EQ(sl::mergeCostLevels(8, 6), 9);
  EXPECT_EQ(sl::trajectoryCost({6, 9, 12}), 27);
  EXPECT_EQ(sl::trajectoryCost({6, 6, 6, 6, 6}), 30);
  EXPECT_EQ(sl::trajectoryCost({100, 100, 100, 100, 100}), 500);
}

TEST(CostModel, SpeedAndTerminalDistanceAreLinear) {
  const sl::PlannerConfig config;
  EXPECT_DOUBLE_EQ(sl::forwardTerminalDistance(0.0, config), 2.0);
  EXPECT_NEAR(sl::forwardTerminalDistance(30.0 / 3.6, config), 10.0, 1.0e-5);
  EXPECT_NEAR(sl::forwardTerminalDistance(15.0 / 3.6, config), 6.0, 1.0e-5);
  EXPECT_NEAR(sl::targetSpeedForCost(70, config), 35.0 / 3.6, 1.0e-5);
  EXPECT_DOUBLE_EQ(sl::targetSpeedForCost(380, config), 0.0);
  EXPECT_NEAR(sl::targetSpeedForCost(225, config),
              config.normal_speed_mps * 0.5, 1.0e-6);
  EXPECT_TRUE(std::isnan(sl::forwardTerminalDistance(
      std::numeric_limits<double>::quiet_NaN(), config)));
}

TEST(CostModel, UncertaintyUsesThreeSigmaAndFailsClosed) {
  const sl::PlannerConfig config;
  auto margin = sl::uncertaintyMargin(0.0, 0.2, config);
  ASSERT_TRUE(margin.valid);
  EXPECT_DOUBLE_EQ(margin.x_m, 0.15);
  EXPECT_NEAR(margin.y_m, 0.6, 1.0e-12);
  EXPECT_FALSE(sl::uncertaintyMargin(-0.1, 0.1, config).valid);
  EXPECT_FALSE(sl::uncertaintyMargin(std::numeric_limits<double>::infinity(),
                                     0.1, config)
                   .valid);
  EXPECT_FALSE(sl::uncertaintyMargin(0.34, 0.1, config).valid);
}

TEST(CostModel, FootprintUsesRearAxleOriginAndClearance) {
  const sl::PlannerConfig config;
  const auto nominal = sl::nominalFootprint(config);
  EXPECT_NEAR(nominal.front_m, 1.554, 1.0e-12);
  EXPECT_NEAR(nominal.rear_m, 0.510, 1.0e-12);
  const auto expanded = sl::wallFootprint(config);
  EXPECT_NEAR(expanded.front_m, 1.804, 1.0e-12);
  EXPECT_NEAR(expanded.rear_m, 0.760, 1.0e-12);
  EXPECT_NEAR(expanded.left_m, 0.900, 1.0e-12);
  const auto corners = sl::footprintCorners({1.0, 2.0, M_PI_2}, nominal);
  EXPECT_NEAR(corners[0].x, 1.0 - 0.650, 1.0e-9);
  EXPECT_NEAR(corners[0].y, 2.0 + 1.554, 1.0e-9);
  const sl::Pose2d ego{0.0, 0.0, 0.0};
  const sl::Pose2d too_close{2.064 + 0.249, 0.0, 0.0};
  const sl::Pose2d safe{2.064 + 0.251, 0.0, 0.0};
  EXPECT_TRUE(
      sl::rectanglesWithinClearance(ego, nominal, too_close, nominal, 0.25));
  EXPECT_FALSE(
      sl::rectanglesWithinClearance(ego, nominal, safe, nominal, 0.25));
}

TEST(MotionLimits, SteeringCurvatureCurveSpeedAndJerk) {
  const sl::PlannerConfig config;
  const double curvature =
      sl::curvatureForSteering(30.0 * M_PI / 180.0, config.wheel_base_m);
  EXPECT_NEAR(curvature, 0.5311, 1.0e-4);
  EXPECT_NEAR(1.0 / curvature, 1.8828, 1.0e-3);
  EXPECT_LE(std::pow(sl::curveSpeedLimit(0.5, config), 2.0) * 0.5,
            config.lateral_acceleration_limit_mps2 + 1.0e-9);
  EXPECT_NEAR(sl::jerkLimitedAcceleration(10.0, 0.0, 0.0, 0.05, false, config),
              0.15, 1.0e-12);
  bool jerk_exceeded = false;
  const double emergency = sl::jerkLimitedAcceleration(
      0.0, 10.0, 0.0, 0.05, true, config, &jerk_exceeded);
  EXPECT_TRUE(jerk_exceeded);
  EXPECT_GE(emergency, config.min_acceleration_mps2);
}
