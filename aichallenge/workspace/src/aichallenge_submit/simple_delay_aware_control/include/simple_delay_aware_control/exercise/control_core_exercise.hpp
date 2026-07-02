#ifndef SIMPLE_DELAY_AWARE_CONTROL__EXERCISE__CONTROL_CORE_EXERCISE_HPP_
#define SIMPLE_DELAY_AWARE_CONTROL__EXERCISE__CONTROL_CORE_EXERCISE_HPP_

#include "simple_delay_aware_control/control_core.hpp"

#include <cstddef>
#include <vector>

namespace simple_delay_aware_control::exercise
{

double normalizeAngle(double angle);

std::size_t nearestTrajectoryIndex(
  const std::vector<TrajectorySample> & trajectory, double x, double y);

double curvatureFromThreePoints(const Point2D & p0, const Point2D & p1, const Point2D & p2);

double estimateTrajectoryCurvature(
  const std::vector<TrajectorySample> & trajectory, std::size_t nearest_index,
  std::size_t lookahead_points, bool trajectory_is_closed);

double speedLimitForCurvature(double curvature, const CurvatureSpeedParams & params);

DelayPrediction predictDelay(
  const VehicleState & state, double current_steering_rad, double target_steering_rad,
  const DelayControlParams & params);

SpeedPlan planCurvatureSpeed(
  const std::vector<TrajectorySample> & trajectory, const VehicleState & control_state,
  const CurvatureSpeedParams & params, bool trajectory_is_closed);

}  // namespace simple_delay_aware_control::exercise

#endif  // SIMPLE_DELAY_AWARE_CONTROL__EXERCISE__CONTROL_CORE_EXERCISE_HPP_
