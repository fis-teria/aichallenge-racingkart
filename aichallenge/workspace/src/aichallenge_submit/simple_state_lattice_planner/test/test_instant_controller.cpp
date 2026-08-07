#include "simple_state_lattice_planner/instant_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace simple_state_lattice_planner {
namespace {

EgoState ego() {
  EgoState value;
  value.snapshot_id = 1U;
  value.speed_mps = 1.0;
  return value;
}

Candidate candidate() {
  Candidate value;
  value.snapshot_id = 1U;
  for (int index = 0; index <= 100; ++index) {
    TrajectoryPoint point;
    point.x_m = 0.05 * static_cast<double>(index);
    point.s_m = point.x_m;
    point.speed_mps = 2.0;
    value.points.push_back(point);
  }
  return value;
}

InstantControlInput input(const EgoState *ego_state,
                          const Candidate *selected,
                          std::int64_t monotonic_now_ns) {
  InstantControlInput value;
  value.ego = ego_state;
  value.selected_candidate = selected;
  value.planner_snapshot_id = 1U;
  value.inputs_fresh = true;
  value.planner_output_fresh = true;
  value.monotonic_now_ns = monotonic_now_ns;
  return value;
}

void expectStop(const InstantControlResult &result) {
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.stop_required);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_LE(result.acceleration_mps2, 0.0);
  EXPECT_DOUBLE_EQ(result.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.steering_rate_radps, 0.0);
}

TEST(InstantController, ProducesBoundedCommand) {
  InstantController controller;
  const auto ego_state = ego();
  const auto selected = candidate();
  const auto result = controller.update(input(&ego_state, &selected, 1'000'000'000));
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_FALSE(result.stop_required);
  EXPECT_DOUBLE_EQ(result.speed_mps, 2.0);
  EXPECT_LE(std::abs(result.steering_angle_rad), 0.5236);
  EXPECT_LE(std::abs(result.steering_rate_radps), 0.35 + 1.0e-12);
}

TEST(InstantController, InvalidTimeStopsAndResets) {
  InstantController controller;
  const auto ego_state = ego();
  auto selected = candidate();
  ASSERT_TRUE(controller.update(input(&ego_state, &selected, 1'000'000'000)).valid);
  expectStop(controller.update(input(&ego_state, &selected, 1'250'000'001)));

  for (auto &point : selected.points) {
    point.y_m = 0.2 * point.x_m;
  }
  const auto after_reset =
      controller.update(input(&ego_state, &selected, 2'000'000'000));
  ASSERT_TRUE(after_reset.valid) << after_reset.reason;
  EXPECT_LE(std::abs(after_reset.steering_rate_radps), 0.35 + 1.0e-12);
}

TEST(InstantController, StaleSnapshotAndEmptyTrajectoryStop) {
  InstantController controller;
  auto ego_state = ego();
  auto selected = candidate();
  auto value = input(&ego_state, &selected, 1'000'000'000);
  value.inputs_fresh = false;
  expectStop(controller.update(value));

  selected.snapshot_id = 2U;
  expectStop(controller.update(input(&ego_state, &selected, 1'100'000'000)));

  selected = candidate();
  selected.points.clear();
  expectStop(controller.update(input(&ego_state, &selected, 1'200'000'000)));
}

TEST(InstantController, NonFiniteNegativeAndShortArcStop) {
  InstantController controller;
  auto ego_state = ego();
  auto selected = candidate();
  ego_state.x_m = std::numeric_limits<double>::quiet_NaN();
  expectStop(controller.update(input(&ego_state, &selected, 1'000'000'000)));

  ego_state = ego();
  selected.points[1].speed_mps = -0.1;
  expectStop(controller.update(input(&ego_state, &selected, 1'100'000'000)));

  selected = candidate();
  selected.points.resize(2U);
  expectStop(controller.update(input(&ego_state, &selected, 1'200'000'000)));
}

TEST(InstantController, RawSteeringBeyondLimitStops) {
  InstantControllerConfig config;
  config.maximum_steering_angle_rad = 0.1;
  config.lookahead_min_m = 0.5;
  InstantController controller(config);
  const auto ego_state = ego();
  auto selected = candidate();
  for (auto &point : selected.points) {
    point.y_m = point.x_m;
  }
  expectStop(controller.update(input(&ego_state, &selected, 1'000'000'000)));
}

}  // namespace
}  // namespace simple_state_lattice_planner
