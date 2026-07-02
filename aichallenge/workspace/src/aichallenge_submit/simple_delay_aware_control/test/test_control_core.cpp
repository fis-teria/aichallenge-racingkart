#include "simple_delay_aware_control/control_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(ControlCore, NormalizeAngleWrapsToPiRange)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_NEAR(simple_delay_aware_control::normalizeAngle(3.0 * pi), pi, 1.0e-9);
  EXPECT_NEAR(simple_delay_aware_control::normalizeAngle(-3.0 * pi), -pi, 1.0e-9);
}

TEST(ControlCore, DelayPredictionMovesStraightAhead)
{
  simple_delay_aware_control::VehicleState state;
  state.velocity = 10.0;

  simple_delay_aware_control::DelayControlParams params;
  params.delay_sec = 0.20;
  params.prediction_dt = 0.02;
  params.use_steering_lag = false;

  const auto prediction = simple_delay_aware_control::predictDelay(state, 0.0, 0.0, params);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_EQ(prediction.prediction_steps, 10);
  EXPECT_NEAR(prediction.predicted.x, 2.0, 1.0e-9);
  EXPECT_NEAR(prediction.predicted.y, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.predicted.yaw, 0.0, 1.0e-9);
}

TEST(ControlCore, DelayPredictionTurnsWithPositiveSteering)
{
  simple_delay_aware_control::VehicleState state;
  state.velocity = 5.0;

  simple_delay_aware_control::DelayControlParams params;
  params.delay_sec = 0.20;
  params.prediction_dt = 0.02;
  params.wheel_base = 1.0;
  params.use_steering_lag = false;

  const auto prediction = simple_delay_aware_control::predictDelay(state, 0.0, 0.2, params);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_GT(prediction.predicted.yaw, 0.0);
  EXPECT_GT(prediction.predicted.x, 0.0);
}

TEST(ControlCore, CurvatureFromThreePointsMatchesUnitCircle)
{
  const double curvature = simple_delay_aware_control::curvatureFromThreePoints(
    simple_delay_aware_control::Point2D{1.0, 0.0},
    simple_delay_aware_control::Point2D{0.0, 1.0},
    simple_delay_aware_control::Point2D{-1.0, 0.0});

  EXPECT_NEAR(curvature, 1.0, 1.0e-9);
}

TEST(ControlCore, CurvatureSpeedUsesLateralAccelerationLimit)
{
  simple_delay_aware_control::CurvatureSpeedParams params;
  params.max_speed_mps = 20.0;
  params.min_speed_mps = 1.0;
  params.lateral_accel_limit_mps2 = 4.0;

  EXPECT_NEAR(simple_delay_aware_control::speedLimitForCurvature(0.25, params), 4.0, 1.0e-9);
}

TEST(ControlCore, PlanCurvatureSpeedUsesTrajectoryAndCurveLimits)
{
  std::vector<simple_delay_aware_control::TrajectorySample> trajectory;
  trajectory.push_back({1.0, 0.0, 0.0, 10.0});
  trajectory.push_back({0.0, 1.0, 0.0, 10.0});
  trajectory.push_back({-1.0, 0.0, 0.0, 10.0});
  trajectory.push_back({0.0, -1.0, 0.0, 10.0});

  simple_delay_aware_control::VehicleState state;
  state.x = 1.0;
  state.y = 0.0;

  simple_delay_aware_control::CurvatureSpeedParams params;
  params.max_speed_mps = 10.0;
  params.min_speed_mps = 1.0;
  params.lateral_accel_limit_mps2 = 4.0;
  params.curvature_lookahead_points = 2;

  const auto plan = simple_delay_aware_control::planCurvatureSpeed(trajectory, state, params, true);

  EXPECT_EQ(plan.nearest_index, 0U);
  EXPECT_NEAR(std::abs(plan.curvature), 1.0, 1.0e-9);
  EXPECT_NEAR(plan.curvature_speed_limit_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.target_speed_mps, 2.0, 1.0e-9);
}
