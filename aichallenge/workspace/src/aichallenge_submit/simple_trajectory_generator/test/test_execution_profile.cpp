#include "simple_trajectory_generator/execution_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using simple_trajectory_generator::buildExecutionProfile;
using simple_trajectory_generator::ExecutionProfileConfig;

TrajectoryPoint makePoint(const double x_m, const double y_m,
                          const double yaw_rad, const float speed_mps) {
  TrajectoryPoint point;
  point.pose.position.x = x_m;
  point.pose.position.y = y_m;
  point.pose.orientation.z = std::sin(0.5 * yaw_rad);
  point.pose.orientation.w = std::cos(0.5 * yaw_rad);
  point.longitudinal_velocity_mps = speed_mps;
  return point;
}

std::int64_t
durationNanoseconds(const builtin_interfaces::msg::Duration &duration) {
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL +
         duration.nanosec;
}

double yaw(const TrajectoryPoint &point) {
  const auto &q = point.pose.orientation;
  return std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
}

TEST(ExecutionProfile, DensifiesAndBuildsOneCoherentBoundedProfile) {
  Trajectory source;
  source.header.frame_id = "map";
  source.points.push_back(makePoint(0.0, 0.0, 0.0, 15.6505993F));
  source.points.push_back(makePoint(1.0, 0.0, 0.10, 8.0F));
  source.points.push_back(makePoint(2.0, 0.0, 0.20, 18.4260406F));

  const auto result = buildExecutionProfile(source, ExecutionProfileConfig{});

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.trajectory.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.execution_speed_mps, 8.0);
  ASSERT_GT(result.trajectory.points.size(), source.points.size());
  std::int64_t previous_time_ns = -1;
  for (std::size_t index = 0; index < result.trajectory.points.size();
       ++index) {
    const auto &point = result.trajectory.points[index];
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 8.0F);
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 0.0F);
    EXPECT_LE(point.longitudinal_velocity_mps, 10.0F);
    const auto time_ns = durationNanoseconds(point.time_from_start);
    EXPECT_GT(time_ns, previous_time_ns);
    previous_time_ns = time_ns;
    if (index == 0U) {
      continue;
    }
    const auto &previous = result.trajectory.points[index - 1U];
    EXPECT_LE(std::hypot(point.pose.position.x - previous.pose.position.x,
                         point.pose.position.y - previous.pose.position.y),
              0.25 + 1.0e-9);
    EXPECT_LE(std::abs(std::remainder(yaw(point) - yaw(previous), kTwoPi)),
              0.05 + 1.0e-9);
  }
}

TEST(ExecutionProfile, UsesTenMpsWhenEverySourcePointIsAboveTheCeiling) {
  Trajectory source;
  for (std::size_t index = 0U; index < 41U; ++index) {
    const float speed_mps = index == 40U ? 15.6505993F : 12.0F;
    source.points.push_back(
        makePoint(static_cast<double>(index), 0.0, 0.0, speed_mps));
  }
  const auto result = buildExecutionProfile(source, ExecutionProfileConfig{});
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.execution_speed_mps, 10.0);
  for (const auto &point : result.trajectory.points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 10.0F);
  }
}

TEST(ExecutionProfile, RejectsInvalidConfigurationAndSourceAtomically) {
  Trajectory source;
  source.points.push_back(makePoint(0.0, 0.0, 0.0, 5.0F));
  source.points.push_back(makePoint(1.0, 0.0, 0.0, 5.0F));

  for (const double invalid_cap :
       {0.0, -1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(), 10.000001}) {
    ExecutionProfileConfig config;
    config.max_speed_mps = invalid_cap;
    const auto result = buildExecutionProfile(source, config);
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.trajectory.points.empty());
  }

  source.points[1].longitudinal_velocity_mps =
      std::numeric_limits<float>::quiet_NaN();
  const auto invalid_source =
      buildExecutionProfile(source, ExecutionProfileConfig{});
  EXPECT_FALSE(invalid_source.valid);
  EXPECT_TRUE(invalid_source.trajectory.points.empty());
}

TEST(ExecutionProfile, RejectsDegenerateAndZeroSpeedSegments) {
  Trajectory source;
  source.points.push_back(makePoint(0.0, 0.0, 0.0, 5.0F));
  source.points.push_back(makePoint(0.0, 0.0, 0.0, 5.0F));
  EXPECT_FALSE(buildExecutionProfile(source, ExecutionProfileConfig{}).valid);

  source.points[1] = makePoint(1.0, 0.0, 0.0, 0.0F);
  EXPECT_FALSE(buildExecutionProfile(source, ExecutionProfileConfig{}).valid);
}

} // namespace
