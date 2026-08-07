#include "simple_state_lattice_planner/costmap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace simple_state_lattice_planner {
namespace {

constexpr std::size_t kHardMaximumCellCount = 1'000'000U;
constexpr double kMinimumResolutionM = 0.01;
constexpr double kMaximumResolutionM = 1.0;
constexpr double kMaximumPhysicalExtentM = 10.0;
constexpr double kMaximumWorldCoordinateM = 1.0e6;
constexpr std::size_t kMaximumInflationCells = 2048U;

bool finite(double value) { return std::isfinite(value); }

bool validConfig(const CostmapBuilderConfig &config) {
  return finite(config.vehicle_front_m) && finite(config.vehicle_rear_m) &&
         finite(config.vehicle_left_m) && finite(config.vehicle_right_m) &&
         finite(config.wall_margin_m) && finite(config.opponent_margin_m) &&
         config.vehicle_front_m > 0.0 && config.vehicle_rear_m > 0.0 &&
         config.vehicle_left_m > 0.0 && config.vehicle_right_m > 0.0 &&
         config.wall_margin_m >= 0.25 && config.opponent_margin_m >= 0.25 &&
         config.vehicle_front_m <= kMaximumPhysicalExtentM &&
         config.vehicle_rear_m <= kMaximumPhysicalExtentM &&
         config.vehicle_left_m <= kMaximumPhysicalExtentM &&
         config.vehicle_right_m <= kMaximumPhysicalExtentM &&
         config.wall_margin_m <= kMaximumPhysicalExtentM &&
         config.opponent_margin_m <= kMaximumPhysicalExtentM &&
         config.maximum_cell_count > 0U &&
         config.maximum_cell_count <= kHardMaximumCellCount;
}

bool markDisk(Costmap2D *map, double center_x_m, double center_y_m,
              double radius_m) {
  if (map == nullptr || !finite(center_x_m) || !finite(center_y_m) ||
      !finite(radius_m) || radius_m < 0.0 ||
      std::abs(center_x_m) > kMaximumWorldCoordinateM ||
      std::abs(center_y_m) > kMaximumWorldCoordinateM) {
    return false;
  }
  const double radius_cells_double = std::ceil(radius_m / map->resolution_m);
  const double center_x_double =
      std::floor((center_x_m - map->origin_x_m) / map->resolution_m);
  const double center_y_double =
      std::floor((center_y_m - map->origin_y_m) / map->resolution_m);
  if (!finite(radius_cells_double) || !finite(center_x_double) ||
      !finite(center_y_double) || radius_cells_double < 0.0 ||
      radius_cells_double > static_cast<double>(kMaximumInflationCells) ||
      center_x_double < static_cast<double>(std::numeric_limits<long>::min()) ||
      center_x_double > static_cast<double>(std::numeric_limits<long>::max()) ||
      center_y_double < static_cast<double>(std::numeric_limits<long>::min()) ||
      center_y_double > static_cast<double>(std::numeric_limits<long>::max())) {
    return false;
  }
  const long center_x = static_cast<long>(center_x_double);
  const long center_y = static_cast<long>(center_y_double);
  const long cells = static_cast<long>(radius_cells_double);
  for (long dy = -cells; dy <= cells; ++dy) {
    for (long dx = -cells; dx <= cells; ++dx) {
      const double sample_x_m = static_cast<double>(dx) * map->resolution_m;
      const double sample_y_m = static_cast<double>(dy) * map->resolution_m;
      if (std::hypot(sample_x_m, sample_y_m) > radius_m + 1.0e-9) {
        continue;
      }
      const long x = center_x + dx;
      const long y = center_y + dy;
      if (x < 0 || y < 0 || x >= static_cast<long>(map->width) ||
          y >= static_cast<long>(map->height)) {
        continue;
      }
      map->cells[static_cast<std::size_t>(y) * map->width +
                 static_cast<std::size_t>(x)] = kOccupiedCost;
    }
  }
  return true;
}

bool inflateStaticLayer(Costmap2D *map, double radius_m) {
  if (map == nullptr || !map->valid() || !finite(radius_m) || radius_m < 0.0) {
    return false;
  }
  const double radius_cells_double = std::ceil(radius_m / map->resolution_m);
  if (!finite(radius_cells_double) || radius_cells_double < 0.0 ||
      radius_cells_double > static_cast<double>(kMaximumInflationCells)) {
    return false;
  }
  const int radius_cells = static_cast<int>(radius_cells_double);
  std::vector<int> distance(map->cells.size(), -1);
  std::queue<std::size_t> frontier;
  for (std::size_t index = 0U; index < map->cells.size(); ++index) {
    if (map->cells[index] >= kOccupiedCost) {
      distance[index] = 0;
      frontier.push(index);
    }
  }
  while (!frontier.empty()) {
    const std::size_t index = frontier.front();
    frontier.pop();
    const int current_distance = distance[index];
    if (current_distance >= radius_cells) {
      continue;
    }
    const long x = static_cast<long>(index % map->width);
    const long y = static_cast<long>(index / map->width);
    for (long dy = -1; dy <= 1; ++dy) {
      for (long dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const long next_x = x + dx;
        const long next_y = y + dy;
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<long>(map->width) ||
            next_y >= static_cast<long>(map->height)) {
          continue;
        }
        const std::size_t next = static_cast<std::size_t>(next_y) * map->width +
                                 static_cast<std::size_t>(next_x);
        if (distance[next] >= 0) {
          continue;
        }
        distance[next] = current_distance + 1;
        map->cells[next] = kOccupiedCost;
        frontier.push(next);
      }
    }
  }
  return true;
}

}  // namespace

bool Costmap2D::valid() const {
  if (snapshot_id == 0U || !finite(stamp_sec) || frame_id != "map" ||
      resolution_m < kMinimumResolutionM ||
      resolution_m > kMaximumResolutionM || !finite(origin_x_m) ||
      !finite(origin_y_m) || width == 0U || height == 0U ||
      width > kHardMaximumCellCount || height > kHardMaximumCellCount ||
      width > kHardMaximumCellCount / height) {
    return false;
  }
  return cells.size() == width * height;
}

bool Costmap2D::worldToCell(double x_m, double y_m, std::size_t *x,
                            std::size_t *y) const {
  if (!valid() || x == nullptr || y == nullptr || !finite(x_m) ||
      !finite(y_m)) {
    return false;
  }
  const double cell_x = std::floor((x_m - origin_x_m) / resolution_m);
  const double cell_y = std::floor((y_m - origin_y_m) / resolution_m);
  if (!finite(cell_x) || !finite(cell_y) || cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= static_cast<double>(width) ||
      cell_y >= static_cast<double>(height)) {
    return false;
  }
  *x = static_cast<std::size_t>(cell_x);
  *y = static_cast<std::size_t>(cell_y);
  return true;
}

std::uint8_t Costmap2D::costAt(double x_m, double y_m) const {
  std::size_t x = 0U;
  std::size_t y = 0U;
  if (!worldToCell(x_m, y_m, &x, &y)) {
    return kOccupiedCost;
  }
  const std::uint8_t value = cells[y * width + x];
  return value == kUnknownCost ? kOccupiedCost : value;
}

bool Costmap2D::occupiedAt(double x_m, double y_m) const {
  return costAt(x_m, y_m) >= kOccupiedCost;
}

CostmapBuildResult buildCostmap(
    const Costmap2D &static_layer,
    const std::optional<OpponentState> &opponent,
    const CostmapBuilderConfig &config) {
  CostmapBuildResult result;
  if (!static_layer.valid() || !validConfig(config) ||
      static_layer.cells.size() > config.maximum_cell_count) {
    result.reason = "invalid_costmap_or_config";
    return result;
  }
  if (opponent.has_value() &&
      (!finite(opponent->x_m) || !finite(opponent->y_m) ||
       !finite(opponent->yaw_rad) ||
       std::abs(opponent->x_m) > kMaximumWorldCoordinateM ||
       std::abs(opponent->y_m) > kMaximumWorldCoordinateM)) {
    result.reason = "opponent_non_finite";
    return result;
  }

  result.costmap = static_layer;
  const double maximum_longitudinal_m =
      std::max(config.vehicle_front_m, config.vehicle_rear_m);
  const double maximum_lateral_m =
      std::max(config.vehicle_left_m, config.vehicle_right_m);
  const double vehicle_corner_radius_m =
      std::hypot(maximum_longitudinal_m, maximum_lateral_m);
  const double wall_inflation_m = vehicle_corner_radius_m + config.wall_margin_m;
  if (!inflateStaticLayer(&result.costmap, wall_inflation_m)) {
    result.reason = "wall_inflation_invalid";
    return result;
  }

  if (opponent.has_value()) {
    const double opponent_inflation_m =
        2.0 * maximum_lateral_m + config.opponent_margin_m;
    if (!markDisk(&result.costmap, opponent->x_m, opponent->y_m,
                  opponent_inflation_m)) {
      result.reason = "opponent_inflation_invalid";
      return result;
    }
  }
  result.valid = true;
  result.reason = "valid";
  return result;
}

CostmapBuildResult addOpponentToInflatedCostmap(
    const Costmap2D &inflated_static_costmap,
    const std::optional<OpponentState> &opponent,
    const CostmapBuilderConfig &config) {
  CostmapBuildResult result;
  if (!inflated_static_costmap.valid() || !validConfig(config) ||
      inflated_static_costmap.cells.size() > config.maximum_cell_count) {
    result.reason = "invalid_costmap_or_config";
    return result;
  }
  result.costmap = inflated_static_costmap;
  if (opponent.has_value()) {
    if (!finite(opponent->x_m) || !finite(opponent->y_m) ||
        !finite(opponent->yaw_rad)) {
      result.reason = "opponent_non_finite";
      return result;
    }
    const double lateral =
        std::max(config.vehicle_left_m, config.vehicle_right_m);
    // The design represents the opponent layer by lateral half-widths plus
    // the unchanged hard margin. Longitudinal clearance is handled by the
    // trajectory's hold/merge extent and full segment sampling.
    const double radius_m = 2.0 * lateral + config.opponent_margin_m;
    if (!markDisk(&result.costmap, opponent->x_m, opponent->y_m, radius_m)) {
      result.reason = "opponent_inflation_invalid";
      return result;
    }
  }
  result.valid = true;
  result.reason = "valid";
  return result;
}

}  // namespace simple_state_lattice_planner
