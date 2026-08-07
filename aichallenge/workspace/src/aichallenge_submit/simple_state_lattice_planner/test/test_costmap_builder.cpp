#include "simple_state_lattice_planner/costmap_builder.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace simple_state_lattice_planner {
namespace {

Costmap2D openMap() {
  Costmap2D map;
  map.snapshot_id = 1U;
  map.stamp_sec = 10.0;
  map.origin_x_m = -5.0;
  map.origin_y_m = -5.0;
  map.width = 100U;
  map.height = 100U;
  map.cells.assign(map.width * map.height, 0U);
  return map;
}

TEST(CostmapBuilder, FreeOccupiedUnknownAndOutOfBoundsAreDeterministic) {
  auto map = openMap();
  map.cells[50U * map.width + 50U] = kOccupiedCost;
  map.cells[51U * map.width + 50U] = kUnknownCost;
  EXPECT_FALSE(map.occupiedAt(-4.0, -4.0));
  EXPECT_TRUE(map.occupiedAt(0.05, 0.05));
  EXPECT_TRUE(map.occupiedAt(0.05, 0.15));
  EXPECT_TRUE(map.occupiedAt(-5.01, 0.0));
  EXPECT_TRUE(map.occupiedAt(std::numeric_limits<double>::quiet_NaN(), 0.0));
}

TEST(CostmapBuilder, InflatesStaticWallWithExistingHardMargin) {
  auto map = openMap();
  map.cells[50U * map.width + 50U] = kOccupiedCost;
  const auto result = buildCostmap(map, std::nullopt);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.costmap.occupiedAt(0.95, 0.95));
  EXPECT_FALSE(result.costmap.occupiedAt(1.25, 1.25));
}

TEST(CostmapBuilder, InflatesAtMostOneOpponent) {
  OpponentState opponent;
  opponent.x_m = 0.0;
  opponent.y_m = 0.0;
  const auto result = buildCostmap(openMap(), opponent);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.costmap.occupiedAt(1.45, 0.0));
  EXPECT_FALSE(result.costmap.occupiedAt(1.85, 0.0));
  EXPECT_FALSE(result.costmap.occupiedAt(2.10, 0.0));
}

TEST(CostmapBuilder, RejectsInvalidMapConfigAndOpponent) {
  auto map = openMap();
  map.frame_id = "odom";
  EXPECT_FALSE(buildCostmap(map, std::nullopt).valid);

  map = openMap();
  auto config = CostmapBuilderConfig{};
  config.wall_margin_m = 0.24;
  EXPECT_FALSE(buildCostmap(map, std::nullopt, config).valid);

  config = CostmapBuilderConfig{};
  config.maximum_cell_count = std::numeric_limits<std::size_t>::max();
  EXPECT_FALSE(buildCostmap(map, std::nullopt, config).valid);

  OpponentState opponent;
  opponent.x_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(buildCostmap(map, opponent).valid);

  opponent.x_m = std::numeric_limits<double>::max();
  EXPECT_FALSE(buildCostmap(map, opponent).valid);

  config = CostmapBuilderConfig{};
  config.wall_margin_m = std::numeric_limits<double>::max();
  EXPECT_FALSE(buildCostmap(map, std::nullopt, config).valid);
}

TEST(CostmapBuilder, AllUnknownWorstAllowedMapIsBoundedAndFailClosed) {
  Costmap2D map;
  map.snapshot_id = 1U;
  map.stamp_sec = 10.0;
  map.resolution_m = 0.01;
  map.width = 1000U;
  map.height = 1000U;
  map.cells.assign(map.width * map.height, kUnknownCost);
  const auto result = buildCostmap(map, std::nullopt);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.costmap.occupiedAt(5.0, 5.0));
}

}  // namespace
}  // namespace simple_state_lattice_planner
