#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace simple_state_lattice_planner {

inline constexpr std::uint8_t kOccupiedCost = 100U;
inline constexpr std::uint8_t kUnknownCost = 255U;

struct Costmap2D {
  std::uint64_t snapshot_id{0U};
  double stamp_sec{0.0};
  std::string frame_id{"map"};
  double resolution_m{0.10};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  std::size_t width{0U};
  std::size_t height{0U};
  std::vector<std::uint8_t> cells;

  bool valid() const;
  bool worldToCell(double x_m, double y_m, std::size_t *x,
                   std::size_t *y) const;
  std::uint8_t costAt(double x_m, double y_m) const;
  bool occupiedAt(double x_m, double y_m) const;
};

struct OpponentState {
  std::uint64_t snapshot_id{0U};
  double stamp_sec{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
};

struct CostmapBuilderConfig {
  double vehicle_front_m{0.467};
  double vehicle_rear_m{0.510};
  double vehicle_left_m{0.650};
  double vehicle_right_m{0.650};
  double wall_margin_m{0.25};
  double opponent_margin_m{0.25};
  std::size_t maximum_cell_count{1'000'000U};
};

struct CostmapBuildResult {
  bool valid{false};
  std::string reason{"uninitialized"};
  Costmap2D costmap;
};

CostmapBuildResult buildCostmap(
    const Costmap2D &static_layer,
    const std::optional<OpponentState> &opponent,
    const CostmapBuilderConfig &config = CostmapBuilderConfig{});

CostmapBuildResult addOpponentToInflatedCostmap(
    const Costmap2D &inflated_static_costmap,
    const std::optional<OpponentState> &opponent,
    const CostmapBuilderConfig &config = CostmapBuilderConfig{});

}  // namespace simple_state_lattice_planner
