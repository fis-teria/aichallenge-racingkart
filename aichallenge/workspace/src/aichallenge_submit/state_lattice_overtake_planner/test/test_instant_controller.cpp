#include "state_lattice_overtake_planner/instant_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace state_lattice_overtake_planner {
namespace {

std::vector<TrajectoryPoint> straightTrajectory() {
  std::vector<TrajectoryPoint> points;
  for (int index = 0; index <= 20; ++index) {
    TrajectoryPoint point;
    point.x = 0.25 * static_cast<double>(index);
    point.y = 0.0;
    point.yaw = 0.0;
    point.speed_mps = 4.0;
    points.push_back(point);
  }
  return points;
}

TEST(InstantController, ProducesBoundedCommandForTrackableTrajectory) {
  InstantControllerConfig config;
  config.lookahead_min_m = 1.0;
  config.maximum_steering_rate_radps = 0.35;
  InstantController controller(config);
  EgoState ego;
  ego.valid = true;
  ego.speed_mps = 2.0;

  const auto result = controller.update(ego, straightTrajectory(), 1.0);

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.speed_mps, 4.0);
  EXPECT_NEAR(result.steering_angle_rad, 0.0, 1.0e-12);
  EXPECT_LE(std::abs(result.steering_rate_radps),
            config.maximum_steering_rate_radps + 1.0e-12);
}

TEST(InstantController, RejectsTrajectoryWithoutRequiredSpatialHorizon) {
  InstantControllerConfig config;
  config.lookahead_min_m = 3.0;
  InstantController controller(config);
  EgoState ego;
  ego.valid = true;
  ego.speed_mps = 2.0;
  auto trajectory = straightTrajectory();
  trajectory.resize(3U);

  const auto result = controller.update(ego, trajectory, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "required_arc_unavailable");
}

TEST(InstantController, RejectsSteeringBeyondPlannerLimit) {
  InstantControllerConfig config;
  config.lookahead_min_m = 0.5;
  config.maximum_steering_angle_rad = 0.1;
  InstantController controller(config);
  EgoState ego;
  ego.valid = true;
  ego.speed_mps = 1.0;
  auto trajectory = straightTrajectory();
  for (auto &point : trajectory) {
    point.y = point.x;
  }

  const auto result = controller.update(ego, trajectory, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "steering_angle_untrackable");
}

} // namespace
} // namespace state_lattice_overtake_planner
