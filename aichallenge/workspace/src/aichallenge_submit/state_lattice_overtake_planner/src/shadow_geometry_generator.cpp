#include "state_lattice_overtake_planner/shadow_geometry_generator.hpp"

#include "state_lattice_overtake_planner/lattice_planner.hpp"

#include <cmath>
#include <limits>

namespace state_lattice_overtake_planner
{
namespace
{
bool finitePose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}
}  // namespace

ShadowGeometryResult ShadowGeometryGenerator::generate(const ShadowGeometryRequest & request) const
{
  ShadowGeometryResult result;
  if (!finitePose(request.start) || !finitePose(request.goal) ||
    !std::isfinite(request.start_curvature) || !std::isfinite(request.goal_curvature) ||
    !std::isfinite(request.tangent_scale) || request.tangent_scale <= 0.0 ||
    !std::isfinite(request.target_d_m) || request.pass_side == 0 ||
    request.sample_count < 2U || request.sample_count > 256U)
  {
    result.reason = "invalid_shadow_geometry_request";
    return result;
  }
  ParametricQuintic polynomial;
  if (!polynomial.configure(request.start, request.start_curvature, request.goal,
      request.goal_curvature, request.tangent_scale))
  {
    result.reason = "quintic_configuration_failed";
    return result;
  }
  result.dense.reserve(request.sample_count);
  double previous_x = std::numeric_limits<double>::quiet_NaN();
  double previous_y = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 0U; index < request.sample_count; ++index) {
    TrajectoryPoint point = polynomial.sample(
      static_cast<double>(index) / static_cast<double>(request.sample_count - 1U));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.yaw) ||
      !std::isfinite(point.kappa))
    {
      result.dense.clear();
      result.reason = "nonfinite_shadow_geometry";
      return result;
    }
    if (index > 0U) {
      const double step = std::hypot(point.x - previous_x, point.y - previous_y);
      if (!std::isfinite(step) || step <= 1.0e-6) {
        result.dense.clear();
        result.reason = "degenerate_shadow_geometry";
        return result;
      }
      result.raw_cost += step + 0.1 * std::abs(point.kappa);
    }
    previous_x = point.x;
    previous_y = point.y;
    result.dense.push_back(point);
  }
  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace state_lattice_overtake_planner
