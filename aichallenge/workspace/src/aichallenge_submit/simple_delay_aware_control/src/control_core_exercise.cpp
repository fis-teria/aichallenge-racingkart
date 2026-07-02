#include "simple_delay_aware_control/exercise/control_core_exercise.hpp"

#include <stdexcept>

namespace simple_delay_aware_control::exercise
{

double normalizeAngle(double /*angle*/)
{
  // TODO-01: sin/cosとatan2を使って、角度を[-pi, pi]へ丸めよう。
  throw std::logic_error("TODO-01 normalizeAngle");
}

std::size_t nearestTrajectoryIndex(
  const std::vector<TrajectorySample> & /*trajectory*/, double /*x*/, double /*y*/)
{
  // TODO-02: 全点との距離を比べて、一番近いtrajectory indexを返そう。
  throw std::logic_error("TODO-02 nearestTrajectoryIndex");
}

double curvatureFromThreePoints(
  const Point2D & /*p0*/, const Point2D & /*p1*/, const Point2D & /*p2*/)
{
  // TODO-03: 3点を通る円の曲率を、外積と辺の長さから計算しよう。
  throw std::logic_error("TODO-03 curvatureFromThreePoints");
}

double estimateTrajectoryCurvature(
  const std::vector<TrajectorySample> & /*trajectory*/, std::size_t /*nearest_index*/,
  std::size_t /*lookahead_points*/, bool /*trajectory_is_closed*/)
{
  // TODO-04: nearest/middle/lookaheadの3点を選び、曲率を推定しよう。
  throw std::logic_error("TODO-04 estimateTrajectoryCurvature");
}

double speedLimitForCurvature(double /*curvature*/, const CurvatureSpeedParams & /*params*/)
{
  // TODO-05: v = sqrt(a_lat / |kappa|) を使い、min/max速度へ丸めよう。
  throw std::logic_error("TODO-05 speedLimitForCurvature");
}

DelayPrediction predictDelay(
  const VehicleState & /*state*/, double /*current_steering_rad*/,
  double /*target_steering_rad*/, const DelayControlParams & /*params*/)
{
  // TODO-06: delay秒ぶん、kinematic bicycle modelで姿勢を前に進めよう。
  throw std::logic_error("TODO-06 predictDelay");
}

SpeedPlan planCurvatureSpeed(
  const std::vector<TrajectorySample> & /*trajectory*/, const VehicleState & /*control_state*/,
  const CurvatureSpeedParams & /*params*/, bool /*trajectory_is_closed*/)
{
  // TODO-07: 最近傍点、曲率、曲率速度上限、最終target speedをまとめよう。
  throw std::logic_error("TODO-07 planCurvatureSpeed");
}

}  // namespace simple_delay_aware_control::exercise
