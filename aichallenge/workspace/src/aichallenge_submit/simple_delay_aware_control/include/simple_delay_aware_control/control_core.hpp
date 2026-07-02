#ifndef SIMPLE_DELAY_AWARE_CONTROL__CONTROL_CORE_HPP_
#define SIMPLE_DELAY_AWARE_CONTROL__CONTROL_CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace simple_delay_aware_control
{

struct Point2D
{
  double x{};
  double y{};
};

struct VehicleState
{
  double x{};
  double y{};
  double yaw{};
  double velocity{};
  double yaw_rate{};
};

struct TrajectorySample
{
  double x{};
  double y{};
  double yaw{};
  double speed_mps{};
};

struct DelayControlParams
{
  bool enabled{true};
  bool use_steering_lag{true};
  double delay_sec{0.20};
  double prediction_dt{0.02};
  double steering_time_constant_sec{0.30};
  double wheel_base{1.087};
  double max_steering_rad{0.70};
};

struct CurvatureSpeedParams
{
  bool enabled{true};
  bool use_trajectory_velocity{true};
  double max_speed_mps{8.333333333333334};
  double min_speed_mps{1.50};
  double lateral_accel_limit_mps2{3.0};
  double curvature_epsilon{1.0e-4};
  std::size_t curvature_lookahead_points{8};
};

struct DelayPrediction
{
  VehicleState input;
  VehicleState predicted;
  double current_steering_rad{};
  double applied_steering_rad{};
  int prediction_steps{};
  bool shifted{};
};

struct SpeedPlan
{
  std::size_t nearest_index{};
  double curvature{};
  double curvature_speed_limit_mps{};
  double trajectory_speed_mps{};
  double target_speed_mps{};
};

double clamp(double value, double lower, double upper);
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

}  // namespace simple_delay_aware_control

#endif  // SIMPLE_DELAY_AWARE_CONTROL__CONTROL_CORE_HPP_
