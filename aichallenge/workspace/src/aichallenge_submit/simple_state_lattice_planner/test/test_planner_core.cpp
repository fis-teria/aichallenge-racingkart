#include "simple_state_lattice_planner/planner_core.hpp"

#include <gtest/gtest.h>

namespace simple_state_lattice_planner {
namespace {

ReferenceWindow reference(std::uint64_t snapshot_id) {
  ReferenceWindow value;
  value.snapshot_id = snapshot_id;
  value.stamp_sec = 10.0;
  for (int index = 0; index <= 60; ++index) {
    ReferencePoint point;
    point.x_m = static_cast<double>(index);
    point.s_m = static_cast<double>(index);
    value.points.push_back(point);
  }
  return value;
}

Costmap2D map(std::uint64_t snapshot_id) {
  Costmap2D value;
  value.snapshot_id = snapshot_id;
  value.stamp_sec = 10.0;
  value.origin_x_m = -5.0;
  value.origin_y_m = -10.0;
  value.width = 700U;
  value.height = 200U;
  value.cells.assign(value.width * value.height, 0U);
  return value;
}

void occupy(Costmap2D *value, double x_m, double y_m) {
  std::size_t x = 0U;
  std::size_t y = 0U;
  ASSERT_TRUE(value->worldToCell(x_m, y_m, &x, &y));
  value->cells[y * value->width + x] = kOccupiedCost;
}

PlannerInput input(std::uint64_t snapshot_id, double ego_x_m,
                   bool with_opponent = false) {
  PlannerInput value;
  value.snapshot_id = snapshot_id;
  value.reference = reference(snapshot_id);
  value.costmap = map(snapshot_id);
  value.ego.snapshot_id = snapshot_id;
  value.ego.stamp_sec = 10.0;
  value.ego.x_m = ego_x_m;
  value.ego.speed_mps = 2.0;
  if (with_opponent) {
    OpponentState opponent;
    opponent.snapshot_id = snapshot_id;
    opponent.stamp_sec = 10.0;
    opponent.x_m = 8.0;
    value.opponent = opponent;
  }
  return value;
}

TEST(PlannerCore, FreeRunSelectsCenterDeterministically) {
  PlannerCore planner;
  const auto output = planner.plan(input(1U, 0.0), 10.0);
  ASSERT_EQ(output.state, PlannerState::FREE_RUN);
  ASSERT_EQ(output.selected_id, 0U);
  EXPECT_EQ(output.reason, OutputReason::CENTER_CLEAR);
  EXPECT_GT(output.speed_limit_mps, 0.0);
  EXPECT_FALSE(output.trajectory.empty());
}

TEST(PlannerCore, BlockedCenterSelectsLeftAndReturnsAfterPassing) {
  PlannerCore planner;
  auto entering = input(1U, 0.0, true);
  occupy(&entering.costmap, 8.0, 0.0);
  occupy(&entering.costmap, 8.0, -1.2);
  const auto enter_output = planner.plan(entering, 10.0);
  ASSERT_EQ(enter_output.state, PlannerState::OVERTAKE);
  ASSERT_TRUE(enter_output.selected_id.has_value());
  EXPECT_GE(*enter_output.selected_id, 1U);
  EXPECT_LE(*enter_output.selected_id, 3U);
  EXPECT_EQ(enter_output.reason, OutputReason::CENTER_BLOCKED_SIDE_SELECTED);
  ASSERT_FALSE(enter_output.trajectory.empty());
  EXPECT_NEAR(enter_output.trajectory.back().d_m, 0.0, 1.0e-12);

  auto continuing = input(2U, 4.0, true);
  continuing.reference.stamp_sec = 10.05;
  continuing.ego.stamp_sec = 10.05;
  continuing.costmap.stamp_sec = 10.05;
  continuing.opponent->snapshot_id = 2U;
  continuing.opponent->stamp_sec = 10.05;
  occupy(&continuing.costmap, 8.0, 0.0);
  occupy(&continuing.costmap, 8.0, -1.2);
  const auto continue_output = planner.plan(continuing, 10.05);
  EXPECT_EQ(continue_output.state, PlannerState::OVERTAKE);
  ASSERT_TRUE(continue_output.selected_id.has_value());
  EXPECT_GE(*continue_output.selected_id, 1U);
  EXPECT_LE(*continue_output.selected_id, 3U);

  auto passed = input(3U, 12.0, true);
  passed.reference.stamp_sec = 10.10;
  passed.ego.stamp_sec = 10.10;
  passed.costmap.stamp_sec = 10.10;
  passed.opponent->snapshot_id = 3U;
  passed.opponent->stamp_sec = 10.10;
  const auto passed_output = planner.plan(passed, 10.10);
  EXPECT_EQ(passed_output.state, PlannerState::FREE_RUN);
  EXPECT_EQ(passed_output.selected_id, 0U);
  EXPECT_EQ(passed_output.reason, OutputReason::OPPONENT_PASSED_CENTER_CLEAR);
}

TEST(PlannerCore, RealOpponentInflationLeavesAValidatedPassCandidate) {
  PlannerCore planner;
  auto value = input(1U, 0.0, true);
  const auto built = buildCostmap(value.costmap, value.opponent);
  ASSERT_TRUE(built.valid) << built.reason;
  value.costmap = built.costmap;
  const auto output = planner.plan(value, 10.0);
  ASSERT_EQ(output.state, PlannerState::OVERTAKE);
  ASSERT_TRUE(output.selected_id.has_value());
  const auto selected = std::find_if(
      output.candidates.begin(), output.candidates.end(),
      [&output](const Candidate &candidate) {
        return candidate.id == *output.selected_id;
      });
  ASSERT_NE(selected, output.candidates.end());
  EXPECT_TRUE(selected->valid);
  ASSERT_EQ(output.trajectory.size(), selected->points.size());
  EXPECT_DOUBLE_EQ(output.trajectory.front().x_m, selected->points.front().x_m);
  EXPECT_DOUBLE_EQ(output.trajectory.back().d_m, selected->points.back().d_m);
}

TEST(PlannerCore, DoesNotSwitchSidesOrAssumeMissingOpponentPassed) {
  PlannerCore planner;
  auto entering = input(1U, 0.0, true);
  occupy(&entering.costmap, 8.0, 0.0);
  occupy(&entering.costmap, 8.0, -1.2);
  ASSERT_EQ(planner.plan(entering, 10.0).state, PlannerState::OVERTAKE);

  auto missing = input(2U, 4.0, false);
  missing.reference.stamp_sec = 10.05;
  missing.ego.stamp_sec = 10.05;
  missing.costmap.stamp_sec = 10.05;
  const auto output = planner.plan(missing, 10.05);
  EXPECT_EQ(output.state, PlannerState::OVERTAKE);
  EXPECT_FALSE(output.selected_id.has_value());
  EXPECT_TRUE(output.trajectory.empty());
  EXPECT_DOUBLE_EQ(output.speed_limit_mps, 0.0);
  EXPECT_EQ(output.reason, OutputReason::OPPONENT_UNAVAILABLE);
}

TEST(PlannerCore, SnapshotMismatchStopsAndResets) {
  PlannerCore planner;
  auto value = input(1U, 0.0);
  value.costmap.snapshot_id = 2U;
  const auto output = planner.plan(value, 10.0);
  EXPECT_FALSE(output.selected_id.has_value());
  EXPECT_TRUE(output.trajectory.empty());
  EXPECT_DOUBLE_EQ(output.speed_limit_mps, 0.0);
  EXPECT_EQ(output.reason, OutputReason::INPUT_INVALID);
}

}  // namespace
}  // namespace simple_state_lattice_planner
