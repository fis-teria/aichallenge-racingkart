#pragma once

#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/cost_model.hpp"
#include "state_lattice_overtake_planner/types.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace state_lattice_overtake_planner
{

class FrenetFrame;

class GridMap
{
public:
  bool load(const std::string & yaml_path, const PlannerConfig & config, std::string * error = nullptr);
  bool buildReferenceLayer(const FrenetFrame & frame, const PlannerConfig & config);
  bool initialized() const {return width_ > 0U && height_ > 0U && !occupied_.empty();}
  std::size_t width() const {return width_;}
  std::size_t height() const {return height_;}
  double resolution() const {return resolution_m_;}
  double originX() const {return origin_x_m_;}
  double originY() const {return origin_y_m_;}
  std::uint64_t staticGeneration() const {return static_generation_;}

  bool worldToCell(double x, double y, int * cell_x, int * cell_y) const;
  Pose2d cellCenter(int cell_x, int cell_y) const;
  bool occupied(int cell_x, int cell_y) const;
  double wallDistance(int cell_x, int cell_y) const;
  int wallLevel(int cell_x, int cell_y) const;
  int maxWallLevelInFootprint(const Pose2d & pose, const Footprint & footprint) const;
  bool footprintHitsWall(const Pose2d & pose, const Footprint & footprint) const;
  const std::vector<std::uint8_t> & occupiedCells() const {return occupied_;}
  const std::vector<std::uint8_t> & wallLevels() const {return wall_levels_;}
  const std::vector<std::uint8_t> & referenceLevels() const {return reference_levels_;}

private:
  bool readPgm(
    const std::string & path, double occupied_threshold, bool negate,
    int minimum_component_cells, std::string * error);
  void buildWallDistanceAndCost(const PlannerConfig & config);
  std::size_t index(int x, int y) const;
  bool pointInsideFootprint(double x, double y, const Pose2d & pose, const Footprint & fp) const;

  std::size_t width_{0U};
  std::size_t height_{0U};
  double resolution_m_{0.0};
  double origin_x_m_{0.0};
  double origin_y_m_{0.0};
  std::vector<std::uint8_t> occupied_;
  std::vector<float> wall_distance_m_;
  std::vector<std::uint8_t> wall_levels_;
  std::vector<std::uint8_t> reference_levels_;
  double wall_level_9_distance_m_{0.0};
  double wall_cost_max_distance_m_{0.0};
  std::uint64_t static_generation_{0U};
};

}  // namespace state_lattice_overtake_planner
