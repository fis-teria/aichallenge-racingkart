#include "wall_recovery_planner/wall_recovery_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace wall_recovery_planner {

TEST(WallRecoveryCore, SelectsForwardWaypointInsideArcWindow) {
  const std::vector<Waypoint2d> trajectory{
      {0.0, 0.0, 0.0, 0.5},
      {1.0, 0.0, 0.0, 0.5},
      {2.0, 0.0, 0.0, 0.5},
      {3.0, 0.0, 0.0, 0.5},
  };
  ForwardWaypointConfig config;
  config.min_arc_distance_m = 1.5;
  config.max_arc_distance_m = 4.0;
  config.max_heading_error_rad = 0.5;

  const auto result = selectForwardWaypoint(trajectory, 0.0, 0.0, 0.0, config);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.index, 2U);
  EXPECT_NEAR(result.arc_distance_m, 2.0, 1.0e-6);
}

TEST(WallRecoveryCore, RejectsWaypointBehindEgo) {
  const std::vector<Waypoint2d> trajectory{
      {-1.0, 0.0, 0.0, 0.5},
      {-2.0, 0.0, 0.0, 0.5},
  };
  ForwardWaypointConfig config;
  config.min_arc_distance_m = 0.5;
  config.max_arc_distance_m = 3.0;

  const auto result = selectForwardWaypoint(trajectory, 0.0, 0.0, 0.0, config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "candidate_behind_ego");
}

TEST(WallRecoveryCore, CollisionTrackerUsesFirstValueAsBaseline) {
  CollisionEventTracker tracker(30);

  const auto first = tracker.observe(100);
  const auto small = tracker.observe(110);
  const auto edge = tracker.observe(145);

  EXPECT_FALSE(first.event);
  EXPECT_FALSE(small.event);
  EXPECT_TRUE(edge.event);
  EXPECT_EQ(edge.sequence, 1U);
}

TEST(WallRecoveryCore, BuildsLowSpeedForwardTrajectory) {
  const Waypoint2d target{2.0, 1.0, 0.5, 0.5};

  const auto trajectory =
      buildRecoveryTrajectory(0.0, 0.0, 0.0, target, 0.7, 5);

  ASSERT_EQ(trajectory.size(), 5U);
  EXPECT_NEAR(trajectory.front().x, 0.0, 1.0e-6);
  EXPECT_NEAR(trajectory.back().x, 2.0, 1.0e-6);
  EXPECT_NEAR(trajectory.back().velocity_mps, 0.7, 1.0e-6);
}

}  // namespace wall_recovery_planner
