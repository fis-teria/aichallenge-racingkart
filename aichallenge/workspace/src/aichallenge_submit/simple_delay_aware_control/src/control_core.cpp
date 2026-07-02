#include "simple_delay_aware_control/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace simple_delay_aware_control
{
namespace
{

double distance2(const double ax, const double ay, const double bx, const double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy;
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::size_t sampleIndex(
  const std::size_t index, const std::size_t size, const bool trajectory_is_closed)
{
  if (size == 0) {
    return 0;
  }
  if (trajectory_is_closed) {
    return index % size;
  }
  return std::min(index, size - 1);
}

}  // namespace

double clamp(const double value, const double lower, const double upper)
{
  return std::min(std::max(value, lower), upper);
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::size_t nearestTrajectoryIndex(
  const std::vector<TrajectorySample> & trajectory, const double x, const double y)
{
  if (trajectory.empty()) {
    return 0;
  }

  std::size_t best_index = 0;
  double best_distance2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const double d2 = distance2(x, y, trajectory[i].x, trajectory[i].y);
    if (d2 < best_distance2) {
      best_index = i;
      best_distance2 = d2;
    }
  }
  return best_index;
}

double curvatureFromThreePoints(const Point2D & p0, const Point2D & p1, const Point2D & p2)
{
  const double a = distance(p0, p1);
  const double b = distance(p1, p2);
  const double c = distance(p2, p0);
  const double denominator = a * b * c;
  if (denominator < 1.0e-9) {
    return 0.0;
  }

  const double cross =
    (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / denominator;
}

double estimateTrajectoryCurvature(
  const std::vector<TrajectorySample> & trajectory, const std::size_t nearest_index,
  const std::size_t lookahead_points, const bool trajectory_is_closed)
{
  if (trajectory.size() < 3) {
    return 0.0;
  }

  const std::size_t span = std::max<std::size_t>(2, lookahead_points);
  const std::size_t i0 = sampleIndex(nearest_index, trajectory.size(), trajectory_is_closed);
  const std::size_t i1 =
    sampleIndex(nearest_index + span / 2, trajectory.size(), trajectory_is_closed);
  const std::size_t i2 = sampleIndex(nearest_index + span, trajectory.size(), trajectory_is_closed);

  const Point2D p0{trajectory[i0].x, trajectory[i0].y};
  const Point2D p1{trajectory[i1].x, trajectory[i1].y};
  const Point2D p2{trajectory[i2].x, trajectory[i2].y};
  return curvatureFromThreePoints(p0, p1, p2);
}

double speedLimitForCurvature(const double curvature, const CurvatureSpeedParams & params)
{
  if (!params.enabled) {
    return params.max_speed_mps;
  }

  const double abs_curvature = std::abs(curvature);
  if (abs_curvature <= std::max(params.curvature_epsilon, 1.0e-9)) {
    return params.max_speed_mps;
  }

  const double raw_limit = std::sqrt(std::max(0.0, params.lateral_accel_limit_mps2) / abs_curvature);
  return clamp(raw_limit, params.min_speed_mps, params.max_speed_mps);
}

DelayPrediction predictDelay(
  const VehicleState & state, const double current_steering_rad,
  const double target_steering_rad, const DelayControlParams & params)
{
  const double delay_sec = std::max(0.0, params.delay_sec);
  const double prediction_dt = std::max(1.0e-3, params.prediction_dt);
  const double wheel_base = std::max(1.0e-3, params.wheel_base);
  const double time_constant = std::max(1.0e-3, params.steering_time_constant_sec);
  const double max_steering = std::max(1.0e-6, params.max_steering_rad);

  const double initial_steering = clamp(current_steering_rad, -max_steering, max_steering);
  const double target_steering = clamp(target_steering_rad, -max_steering, max_steering);

  DelayPrediction prediction;
  prediction.input = state;
  prediction.predicted = state;
  prediction.current_steering_rad = initial_steering;
  prediction.applied_steering_rad = initial_steering;

  if (!params.enabled || delay_sec <= 0.0) {
    prediction.shifted = false;
    return prediction;
  }

  double elapsed = 0.0;
  double steering = initial_steering;
  while (elapsed < delay_sec - 1.0e-12) {
    const double dt = std::min(prediction_dt, delay_sec - elapsed);
    if (params.use_steering_lag) {
      const double alpha = 1.0 - std::exp(-dt / time_constant);
      steering += (target_steering - steering) * alpha;
    } else {
      steering = target_steering;
    }

    prediction.predicted.x += prediction.predicted.velocity * std::cos(prediction.predicted.yaw) * dt;
    prediction.predicted.y += prediction.predicted.velocity * std::sin(prediction.predicted.yaw) * dt;
    prediction.predicted.yaw = normalizeAngle(
      prediction.predicted.yaw +
      prediction.predicted.velocity / wheel_base * std::tan(steering) * dt);

    elapsed += dt;
    ++prediction.prediction_steps;
  }

  prediction.applied_steering_rad = steering;
  prediction.shifted = true;
  return prediction;
}

SpeedPlan planCurvatureSpeed(
  const std::vector<TrajectorySample> & trajectory, const VehicleState & control_state,
  const CurvatureSpeedParams & params, const bool trajectory_is_closed)
{
  SpeedPlan plan;
  if (trajectory.empty()) {
    plan.curvature_speed_limit_mps = 0.0;
    plan.trajectory_speed_mps = 0.0;
    plan.target_speed_mps = 0.0;
    return plan;
  }

  plan.nearest_index = nearestTrajectoryIndex(trajectory, control_state.x, control_state.y);
  plan.curvature = estimateTrajectoryCurvature(
    trajectory, plan.nearest_index, params.curvature_lookahead_points, trajectory_is_closed);
  plan.curvature_speed_limit_mps = speedLimitForCurvature(plan.curvature, params);

  const double trajectory_speed = trajectory[plan.nearest_index].speed_mps;
  plan.trajectory_speed_mps =
    params.use_trajectory_velocity && trajectory_speed > 0.0 ? trajectory_speed : params.max_speed_mps;

  double target_speed =
    params.enabled ? std::min(plan.trajectory_speed_mps, plan.curvature_speed_limit_mps)
                   : plan.trajectory_speed_mps;
  if (target_speed > 0.05) {
    target_speed = clamp(target_speed, params.min_speed_mps, params.max_speed_mps);
  }
  plan.target_speed_mps = target_speed;
  return plan;
}

}  // namespace simple_delay_aware_control
