#include "overtake_planner/safety_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner
{

SafetyEvaluator::SafetyEvaluator(PlannerConfig config) : config_(config) {}

double SafetyEvaluator::ellipseMargin(
  double ego_x, double ego_y, double ego_yaw,
  double opp_x, double opp_y) const
{
  // 自車の向きに合わせた楕円座標へ変換し、前後方向を広めに取った接近余裕を見る。
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
  // 評価結果は候補に直接書き戻し、選択理由やレポート用の指標にも使えるようにする。
  candidate.feasible = true;
  candidate.min_safety_margin = std::numeric_limits<double>::infinity();
  candidate.cbf_slack = 0.0;
  candidate.active_safety_constraint_count = 0;
  candidate.reject_reason.clear();

  for (double d : candidate.d) {
    // 横オフセットが壁マージンを割る候補は、他車を見る前に即rejectする。
    if (d < config_.d_min_m + config_.min_wall_margin_m ||
        d > config_.d_max_m - config_.min_wall_margin_m) {
      candidate.feasible = false;
      candidate.reject_reason = "wall_margin";
      return false;
    }
  }

  for (const auto & pred : predictions) {
    // 他車予測と候補軌道を同じhorizon indexで突き合わせ、安全楕円の余裕を調べる。
    const std::size_t n = std::min(candidate.x.size(), pred.x.size());
    for (std::size_t i = 0; i < n; ++i) {
      const double margin = ellipseMargin(
        candidate.x[i], candidate.y[i], candidate.yaw[i], pred.x[i], pred.y[i]);
      candidate.min_safety_margin = std::min(candidate.min_safety_margin, margin);
      if (margin <= config_.min_ellipse_h + 0.10) {
        // 閾値近傍の制約数を数え、後段の解析で「危なかった候補」を可視化する。
        ++candidate.active_safety_constraint_count;
      }
      if (margin <= config_.min_ellipse_h) {
        // 実際に閾値を割ったら不可。cbf_slackは不足量としてdebugへ出す。
        candidate.feasible = false;
        candidate.cbf_slack = config_.min_ellipse_h - margin;
        candidate.reject_reason = "opponent_collision";
        return false;
      }
    }
  }

  return true;
}

}  // namespace overtake_planner
