#include "simple_pure_pursuit/delay_compensation.hpp"
#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/safety.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <optional>

namespace {

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

TrajectoryPoint makePoint(double x, double y, double yaw_rad) {
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = yawToQuaternion(yaw_rad);
  return point;
}

TrajectoryPoint makePointWithSpeed(double x, double y, double yaw_rad,
                                   double speed_mps) {
  auto point = makePoint(x, y, yaw_rad);
  point.longitudinal_velocity_mps = speed_mps;
  return point;
}

Trajectory makeStraightThenArcTrajectory() {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;
  Trajectory trajectory;
  for (int i = 0; i <= 5; ++i) {
    trajectory.points.push_back(makePoint(static_cast<double>(i), 0.0, 0.0));
  }
  for (int i = 1; i <= 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(5.0 + radius_m * std::sin(theta),
                                          radius_m * (1.0 - std::cos(theta)),
                                          theta));
  }
  return trajectory;
}

Trajectory makeArcTrajectory(double direction) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;
  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta),
        direction * radius_m * (1.0 - std::cos(theta)), direction * theta));
  }
  return trajectory;
}

} // namespace

TEST(Lookahead, SpeedBasedDistanceMatchesExistingFormula) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;

  EXPECT_NEAR(
      simple_pure_pursuit::speedBasedLookaheadDistance(9.5, 0.0, params), 8.25,
      1.0e-9);
}

TEST(Lookahead, CurrentSpeedPreventsSuddenLookaheadShrink) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;

  EXPECT_NEAR(
      simple_pure_pursuit::speedBasedLookaheadDistance(2.5, 9.5, params), 8.25,
      1.0e-9);
}

TEST(Lookahead, CurvatureDisabledKeepsSpeedBasedDistance) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = false;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 0.20, params),
      8.25, 1.0e-9);
}

TEST(Lookahead, CurvatureShortensDistanceButRespectsMinimum) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  const double moderate_curve =
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 0.10, params);
  EXPECT_LT(moderate_curve, 8.25);
  EXPECT_GT(moderate_curve, 3.5);

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(9.5, 0.0, 1.00, params),
      3.5, 1.0e-9);
}

TEST(Lookahead, ZeroSpeedStillUsesFiniteMinimumDistance) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 2.0;
  params.curvature_sensitivity = 8.0;

  EXPECT_NEAR(
      simple_pure_pursuit::adaptiveLookaheadDistance(0.0, 0.0, 0.10, params),
      3.5, 1.0e-9);
}

TEST(Lookahead, SmoothingBlendsPreviousAndDesiredDistance) {
  EXPECT_NEAR(
      simple_pure_pursuit::smoothLookaheadDistance(4.0, 8.0, true, 0.25), 7.0,
      1.0e-9);
  EXPECT_NEAR(
      simple_pure_pursuit::smoothLookaheadDistance(4.0, 8.0, false, 0.25), 4.0,
      1.0e-9);
}

TEST(Lookahead, ForwardTrajectoryIndexSkipsCurrentAnchor) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 0.0));
  trajectory.points.push_back(makePointWithSpeed(0.1, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx =
      simple_pure_pursuit::selectForwardTrajectoryIndex(trajectory, 0, 0.25);

  EXPECT_EQ(idx, 2U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 9.5, 1.0e-9);
}

TEST(Lookahead, ForwardTrajectoryIndexFallsBackToNearestAtEnd) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 8.0));

  const auto idx =
      simple_pure_pursuit::selectForwardTrajectoryIndex(trajectory, 0, 0.25);

  EXPECT_EQ(idx, 0U);
}

TEST(Lookahead, MpcHorizonVelocityCapSkipsZeroIndexAnchor) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 0.0));
  trajectory.points.push_back(makePointWithSpeed(0.1, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 0, 0.25, true);

  EXPECT_EQ(idx, 2U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 9.5, 1.0e-9);
}

TEST(Lookahead, MpcHorizonVelocityCapKeepsForwardNearestIndex) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 9.5));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 4.0));
  trajectory.points.push_back(makePointWithSpeed(1.2, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 1, 0.25, true);

  EXPECT_EQ(idx, 1U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 4.0, 1.0e-9);
}

TEST(Lookahead, MpcHorizonVelocityCapKeepsSolverZeroIndex) {
  Trajectory trajectory;
  trajectory.points.push_back(makePointWithSpeed(0.0, 0.0, 0.0, 2.0));
  trajectory.points.push_back(makePointWithSpeed(0.6, 0.0, 0.0, 9.5));

  const auto idx = simple_pure_pursuit::selectMpcHorizonVelocityCapIndex(
      trajectory, 0, 0.25, false);

  EXPECT_EQ(idx, 0U);
  EXPECT_NEAR(trajectory.points.at(idx).longitudinal_velocity_mps, 2.0, 1.0e-9);
}

TEST(Lookahead, StraightTrajectoryCurvatureIsZero) {
  Trajectory trajectory;
  for (int i = 0; i < 6; ++i) {
    trajectory.points.push_back(makePoint(static_cast<double>(i), 0.0, 0.0));
  }

  EXPECT_NEAR(
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 5, 0.5),
      0.0, 1.0e-9);
}

TEST(Lookahead, CurvedTrajectoryCurvatureIsPositive) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;

  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta), radius_m * (1.0 - std::cos(theta)), theta));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, CurvatureUsesGeometryEvenWhenYawIsStraight) {
  constexpr double radius_m = 10.0;
  constexpr double step_rad = 0.10;

  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double theta = step_rad * static_cast<double>(i);
    trajectory.points.push_back(makePoint(
        radius_m * std::sin(theta), radius_m * (1.0 - std::cos(theta)), 0.0));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, CurvatureUsesHeadingChangeEvenWhenGeometryIsStraight) {
  Trajectory trajectory;
  for (int i = 0; i < 8; ++i) {
    const double x = static_cast<double>(i);
    const double yaw = 0.10 * static_cast<double>(i);
    trajectory.points.push_back(makePoint(x, 0.0, yaw));
  }

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.09);
  EXPECT_LT(curvature, 0.11);
}

TEST(Lookahead, CurvatureDoesNotRecognizeCornerOutsideWindow) {
  const auto trajectory = makeStraightThenArcTrajectory();

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 4.0, 0.5);

  EXPECT_NEAR(curvature, 0.0, 1.0e-9);
}

TEST(Lookahead, CurvatureRecognizesCornerAheadWithinWindow) {
  const auto trajectory = makeStraightThenArcTrajectory();

  const double curvature =
      simple_pure_pursuit::estimateTrajectoryCurvature(trajectory, 0, 6.0, 0.5);

  EXPECT_GT(curvature, 0.05);
  EXPECT_LT(curvature, 0.15);
}

TEST(Lookahead, SignedCurvatureKeepsLeftRightDirection) {
  const double left_curvature =
      simple_pure_pursuit::estimateSignedTrajectoryCurvature(
          makeArcTrajectory(1.0), 0, 6.0, 0.5);
  const double right_curvature =
      simple_pure_pursuit::estimateSignedTrajectoryCurvature(
          makeArcTrajectory(-1.0), 0, 6.0, 0.5);

  EXPECT_GT(left_curvature, 0.05);
  EXPECT_LT(left_curvature, 0.15);
  EXPECT_LT(right_curvature, -0.05);
  EXPECT_GT(right_curvature, -0.15);
}

TEST(Lookahead, AdaptiveLookaheadReturnsBaseForNaNAndInfCurvature) {
  simple_pure_pursuit::LookaheadParams params;
  params.lookahead_gain = 0.5;
  params.lookahead_min_distance = 3.5;
  params.curvature_adaptive_enabled = true;
  params.curvature_min_distance = 3.5;
  params.curvature_sensitivity = 8.0;

  const double base =
      simple_pure_pursuit::speedBasedLookaheadDistance(9.5, 0.0, params);
  EXPECT_NEAR(simple_pure_pursuit::adaptiveLookaheadDistance(
                  9.5, 0.0, std::numeric_limits<double>::quiet_NaN(), params),
              base, 1.0e-9);
  EXPECT_NEAR(simple_pure_pursuit::adaptiveLookaheadDistance(
                  9.5, 0.0, std::numeric_limits<double>::infinity(), params),
              base, 1.0e-9);
}

TEST(Lookahead, InvalidAndShortTrajectoryFallsBackToZeroCurvature) {
  Trajectory short_trajectory;
  short_trajectory.points.push_back(makePoint(0.0, 0.0, 0.0));
  short_trajectory.points.push_back(makePoint(1.0, 0.0, 0.0));

  EXPECT_NEAR(simple_pure_pursuit::estimateTrajectoryCurvature(short_trajectory,
                                                               0, 5, 0.5),
              0.0, 1.0e-9);

  Trajectory repeated_trajectory;
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 0.0));
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 0.5));
  repeated_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0));

  EXPECT_NEAR(simple_pure_pursuit::estimateTrajectoryCurvature(
                  repeated_trajectory, 0, 5, 0.5),
              0.0, 1.0e-9);
}

TEST(Safety, MissingInputsAreInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      std::nullopt, std::nullopt, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "missing_odom");
}

TEST(Safety, StaleOdometryIsInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.0, 9.9, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "stale_odom");
  EXPECT_NEAR(result.ages.odom_age_sec, 1.0, 1.0e-9);
}

TEST(Safety, StaleTrajectoryIsInvalid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.9, 9.0, 10.0, 0.2, 0.5);

  EXPECT_FALSE(result.fresh);
  EXPECT_EQ(result.reason, "stale_trajectory");
  EXPECT_NEAR(result.ages.trajectory_age_sec, 1.0, 1.0e-9);
}

TEST(Safety, FreshInputsAreValid) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      9.9, 9.7, 10.0, 0.2, 0.5);

  EXPECT_TRUE(result.fresh);
  EXPECT_EQ(result.reason, "fresh");
  EXPECT_NEAR(result.ages.odom_age_sec, 0.1, 1.0e-9);
  EXPECT_NEAR(result.ages.trajectory_age_sec, 0.3, 1.0e-9);
}

TEST(Safety, NegativeMaxAgeDisablesThatFreshnessCheck) {
  const auto result = simple_pure_pursuit::evaluateRequiredInputFreshness(
      1.0, 1.0, 10.0, -1.0, -1.0);

  EXPECT_TRUE(result.fresh);
}

TEST(Safety, DisabledMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      false, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "disabled");
}

TEST(Safety, StaleMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.0, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "stale");
}

TEST(Safety, ShortMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 2, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "short");
}

TEST(Safety, FarMpcHorizonStartIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 3.0, 2.0, 6.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "start_distance");
}

TEST(Safety, ShortArcMpcHorizonIsRejected) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 1.0, 2.0, true, true);

  EXPECT_FALSE(result.usable);
  EXPECT_EQ(result.reason, "short_arc");
}

TEST(Safety, FreshMpcHorizonIsUsable) {
  const auto result = simple_pure_pursuit::evaluateMpcHorizonFreshness(
      true, 9.99, 10.0, 0.15, 20, 5, 0.1, 2.0, 6.0, 2.0, true, true);

  EXPECT_TRUE(result.usable);
  EXPECT_EQ(result.reason, "fresh");
  EXPECT_NEAR(result.age_sec, 0.01, 1.0e-9);
}

TEST(DelayCompensation, ZeroDelayKeepsInputPose) {
  const simple_pure_pursuit::EgoControlState state{1.0, 2.0, 0.3, 5.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.1, 0.2, 0.0, 0.02, 0.30, 1.087);

  EXPECT_FALSE(prediction.shifted);
  EXPECT_EQ(prediction.prediction_steps, 0);
  EXPECT_NEAR(prediction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 2.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 0.3, 1.0e-9);
  EXPECT_NEAR(prediction.applied_steering_rad, 0.1, 1.0e-9);
}

TEST(DelayCompensation, StraightMotionAdvancesByDelayDistance) {
  const simple_pure_pursuit::EgoControlState state{0.0, 0.0, 0.0, 5.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.0, 0.0, 0.20, 0.02, 0.30, 1.087);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_EQ(prediction.prediction_steps, 10);
  EXPECT_NEAR(prediction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 0.0, 1.0e-9);
}

TEST(DelayCompensation, SteeringLagMovesTowardPreviousCommand) {
  const double next_steer =
      simple_pure_pursuit::predictLaggedSteering(0.0, 0.4, 0.10, 0.30);

  EXPECT_GT(next_steer, 0.0);
  EXPECT_LT(next_steer, 0.4);
}

TEST(DelayCompensation, ZeroVelocityDoesNotDriftYaw) {
  const simple_pure_pursuit::EgoControlState state{0.0, 0.0, 1.0, 0.0};
  const auto prediction = simple_pure_pursuit::predictDelayedPose(
      state, 0.5, 0.5, 0.20, 0.02, 0.30, 1.087);

  EXPECT_TRUE(prediction.shifted);
  EXPECT_NEAR(prediction.x, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.yaw, 1.0, 1.0e-9);
}
