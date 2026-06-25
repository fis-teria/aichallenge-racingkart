#include "overtake_planner/safety_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner
{

SafetyEvaluator::SafetyEvaluator(PlannerConfig config) : config_(config) {}

double SafetyEvaluator::ellipseMargin(
  double ego_x, double ego_y, double ego_yaw,
  double opp_x, double opp_y) const
{
  const double dx = opp_x - ego_x;
  const double dy = opp_y - ego_y;
  const double c = std::cos(ego_yaw);
  const double s = std::sin(ego_yaw);
  const double x_body = c * dx + s * dy;
  const double y_body = -s * dx + c * dy;
  const double h =
    (x_body / config_.safety_ellipse_a_m) * (x_body / config_.safety_ellipse_a_m) +
    (y_body / config_.safety_ellipse_b_m) * (y_body / config_.safety_ellipse_b_m) -
    1.0;
  return h;
}

bool SafetyEvaluator::evaluate(
  CandidateTrajectory & candidate,
  const std::vector<PredictedOpponent> & predictions) const
{
  candidate.feasible = true;
  candidate.reject_reason.clear();

  for (double d : candidate.d) {
    if (d < config_.d_min_m + config_.min_wall_margin_m ||
        d > config_.d_max_m - config_.min_wall_margin_m) {
      candidate.feasible = false;
      candidate.reject_reason = "wall_margin";
      return false;
    }
  }

  for (const auto & pred : predictions) {
    const std::size_t n = std::min(candidate.x.size(), pred.x.size());
    for (std::size_t i = 0; i < n; ++i) {
      const double margin = ellipseMargin(
        candidate.x[i], candidate.y[i], candidate.yaw[i], pred.x[i], pred.y[i]);
      if (margin <= config_.min_ellipse_h) {
        candidate.feasible = false;
        candidate.reject_reason = "opponent_collision";
        return false;
      }
    }
  }

  return true;
}

}  // namespace overtake_planner
