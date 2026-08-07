#pragma once

#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/types.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner
{

struct Footprint
{
  double front_m{1.554};
  double rear_m{0.510};
  double left_m{0.650};
  double right_m{0.650};
};

struct UncertaintyMargin
{
  double x_m{0.0};
  double y_m{0.0};
  bool valid{false};
  std::string reason;
};

int wallCostLevel(double distance_m, const PlannerConfig & config);
int objectCostLevel(double effective_distance_m, const PlannerConfig & config);
int referenceCostLevel(double distance_m, const PlannerConfig & config);
int mergeCostLevels(int lhs, int rhs);
int trajectoryCost(const std::vector<int> & pose_costs);
double forwardTerminalDistance(double speed_mps, const PlannerConfig & config);
double targetSpeedForCost(int cost, const PlannerConfig & config);
double curvatureForSteering(double steering_rad, double wheel_base_m);
double curveSpeedLimit(double curvature, const PlannerConfig & config);
double jerkLimitedAcceleration(
  double target_speed_mps, double previous_speed_mps, double previous_acceleration_mps2,
  double dt_sec, bool emergency, const PlannerConfig & config, bool * jerk_exceeded = nullptr);
UncertaintyMargin uncertaintyMargin(double sigma_x_m, double sigma_y_m, const PlannerConfig & config);

Footprint nominalFootprint(const PlannerConfig & config);
Footprint wallFootprint(const PlannerConfig & config, bool include_tracking_margin = false);
std::array<Pose2d, 4> footprintCorners(const Pose2d & pose, const Footprint & footprint);
bool rectanglesWithinClearance(
  const Pose2d & a, const Footprint & a_footprint, const Pose2d & b,
  const Footprint & b_footprint, double clearance_m);
double rectangleClearance(
  const Pose2d & a, const Footprint & a_footprint, const Pose2d & b,
  const Footprint & b_footprint);
double pointToSegmentDistance(
  double px, double py, double ax, double ay, double bx, double by);

}  // namespace state_lattice_overtake_planner
