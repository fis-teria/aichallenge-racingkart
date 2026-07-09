#include "overtake_planner/safety_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner
{

// 入力: PlannerConfig。壁マージン、安全楕円サイズ、許容安全余裕を含む。
// 出力: SafetyEvaluatorインスタンス。以後の候補評価で同じ設定を使う。
// 処理概要: 設定を値で保持し、評価中に外部パラメータが変わらないようにする。
SafetyEvaluator::SafetyEvaluator(PlannerConfig config) : config_(config) {}

// 入力: 候補軌道上の自車位置/姿勢と、同じ時刻の相手車位置。
// 出力: 安全楕円の余裕h。0より大きいほど楕円外側、負値は衝突領域内。
// 処理概要: 相対位置を自車body座標へ回し、前後/左右で別半径の楕円制約に変換する。
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

// 入力: 評価対象の候補軌道と、相手車の予測軌道リスト。
// 出力: 候補が安全ならtrue。不安全ならfalseを返し、candidate内に理由と余裕を記録する。
// 処理概要: まず壁マージンで早期rejectし、その後に各時刻の他車楕円制約を評価する。
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

  // 処理ブロック: 壁との安全余裕を先に確認する。
  // 設計意図: 壁違反は相手車有無に関係なく危険なので、計算量の大きい相手車評価より前に落とす。
  for (double d : candidate.d) {
    // 横オフセットが壁マージンを割る候補は、他車を見る前に即rejectする。
    if (d < config_.d_min_m + config_.min_wall_margin_m ||
        d > config_.d_max_m - config_.min_wall_margin_m) {
      candidate.feasible = false;
      candidate.reject_reason = "wall_margin";
      return false;
    }
  }

  // 処理ブロック: 相手車予測と候補軌道を同じhorizon indexで比較する。
  // 設計意図: MPCへ渡す各点が将来の相手車位置と干渉しないことを候補単位で保証する。
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
