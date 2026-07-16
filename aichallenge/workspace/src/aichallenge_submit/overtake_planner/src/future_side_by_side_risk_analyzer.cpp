#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

constexpr double kSideDirectionEpsilon = 0.05;

// 入力: 0から1へ進む正規化値z。
// 出力: 端点の傾きが0になる補間率。
// 処理概要: 横方向へ離れる予測dを急変させないために使う。
double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

// 入力: 自車s、相手s、コース長。
// 出力: 周回を考慮した符号付きs差分。正は相手が前方、負は後方。
// 処理概要: 将来の横並び判定で、周回境界をまたいでも前後関係が破綻しないようにする。
double signedDeltaS(double ego_s, double other_s, double track_length) {
  double signed_delta_s = other_s - ego_s;
  if (track_length > 0.0) {
    signed_delta_s = std::fmod(signed_delta_s, track_length);
    if (signed_delta_s > track_length * 0.5) {
      signed_delta_s -= track_length;
    } else if (signed_delta_s < -track_length * 0.5) {
      signed_delta_s += track_length;
    }
  }
  return signed_delta_s;
}

} // namespace

// 入力: Frenet変換器、planner設定、現在リスク解析器。
// 出力: 将来横並びリスク解析器のインスタンス。
// 処理概要: 現在の壁余裕計算を再利用しながら、相手車の未来位置を評価できるよう依存を保持する。
FutureSideBySideRiskAnalyzer::FutureSideBySideRiskAnalyzer(
    const FrenetFrame &frame, const PlannerConfig &config,
    const BlockedRiskAnalyzer &blocked_risk)
    : frame_(frame), config_(config), blocked_risk_(blocked_risk) {}

// 入力: 自車状態、現在のBlockedInfo、相手車一覧。
// 出力: 将来横並び/コーナー/外壁リスクを反映したBlockedInfo。
// 処理概要: 横並びまたは並走候補の未来位置を等速予測し、コーナーで譲るべき状況を先読みする。
BlockedInfo FutureSideBySideRiskAnalyzer::evaluate(
    const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  BlockedInfo out = blocked_info;
  // 処理ブロック: 未来予測の対象車両を選ぶ。
  // 設計意図: 横並び候補がない時は通常の前方閉塞判断に任せ、不要な譲りを作らない。
  const int target_index = sideRiskIndex(out);
  if (!config_.future_side_prediction_enabled || target_index < 0 ||
      static_cast<std::size_t>(target_index) >= opponents.size()) {
    return out;
  }

  const bool exact_side_target = out.side_index >= 0;
  const auto &opp = opponents[static_cast<std::size_t>(target_index)];
  const bool direction_known = exact_side_target
                                   ? out.side_direction_known
                                   : out.parallel_side_direction_known;
  const bool same_direction = exact_side_target
                                  ? out.side_same_direction
                                  : out.parallel_side_same_direction;
  if (direction_known && !same_direction) {
    return out;
  }
  // 処理ブロック: 自車が相手から離れる方向と横目標を仮定する。
  // 設計意図: SIDE_BY_SIDE_KEEP候補が選ばれた場合の将来dを近似し、壁余裕を先に評価する。
  const double target_delta_d =
      exact_side_target ? out.side_delta_d : out.parallel_side_delta_d;
  const double target_s_dot =
      exact_side_target ? out.side_s_dot_mps : out.parallel_side_s_dot_mps;
  // parallel_side_* は観測・debug用の広い窓であり、譲り判断には使わない。
  const double future_side_s_m = config_.side_by_side_s_m;
  const double future_side_margin_m = config_.side_margin_m;
  const double initial_abs_delta_s = std::abs(
      exact_side_target ? out.side_delta_s : out.parallel_side_delta_s);

  const auto ego_bounds = frame_.corridorBounds(
      ego.frenet.s, config_.d_min_m, config_.d_max_m);
  const double lower_d = ego_bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = ego_bounds.d_max - config_.min_wall_margin_m;
  const double left_space = upper_d - ego.frenet.d;
  const double right_space = ego.frenet.d - lower_d;
  double away_sign = 0.0;
  if (std::abs(target_delta_d) > kSideDirectionEpsilon) {
    away_sign = target_delta_d > 0.0 ? -1.0 : 1.0;
  } else {
    away_sign = left_space >= right_space ? 1.0 : -1.0;
  }
  const double gap_target =
      opp.frenet.d + away_sign * config_.side_by_side_target_gap_m;
  double side_keep_target_d = away_sign > 0.0
                                  ? std::max(ego.frenet.d, gap_target)
                                  : std::min(ego.frenet.d, gap_target);
  side_keep_target_d = std::clamp(side_keep_target_d, lower_d, upper_d);

  const double horizon_sec =
      std::max(0.0, config_.future_side_prediction_horizon_sec);
  const double dt_sec = std::max(1.0e-3, config_.future_side_prediction_dt_sec);
  const double s_dot = target_s_dot;
  const double ego_speed = std::max(0.0, ego.v);
  double best_clearance = std::numeric_limits<double>::infinity();
  double max_future_curvature = 0.0;
  bool found_future_side = false;
  bool found_future_corner = false;
  bool found_outer_wall_risk = false;

  // 処理ブロック: 未来時刻を離散サンプリングして、横並びと壁余裕を評価する。
  // 設計意図: MPC horizonより軽い近似で、コーナー進入前に譲り判断を立てる。
  for (double t = dt_sec; t <= horizon_sec + 1.0e-9; t += dt_sec) {
    const double ego_s = frame_.wrapS(ego.frenet.s + ego_speed * t);
    const double opp_s = frame_.wrapS(opp.frenet.s + s_dot * t);
    const double progress = ego_speed * t;
    const double ratio = smoothstep(
        progress / std::max(1.0, config_.side_by_side_shift_distance_m));
    const double ego_d =
        ego.frenet.d + (side_keep_target_d - ego.frenet.d) * ratio;
    const double opp_d = opp.frenet.d;
    const double delta_s = signedDeltaS(ego_s, opp_s, frame_.length());
    const double delta_d = opp_d - ego_d;
    const bool parallel_interaction =
        exact_side_target ||
        (std::isfinite(initial_abs_delta_s) &&
         std::abs(delta_s) + 1.0e-3 < initial_abs_delta_s);
    const bool future_side = parallel_interaction &&
                             std::abs(delta_s) < future_side_s_m &&
                             std::abs(delta_d) < future_side_margin_m;
    const double future_curvature =
        maxAbsCurvatureAhead(ego_s, config_.corner_side_yield_lookahead_m);
    const bool future_corner =
        future_side && config_.corner_side_yield_curvature_m_inv > 0.0 &&
        future_curvature >= config_.corner_side_yield_curvature_m_inv;
    const double clearance = blocked_risk_.wallClearance(ego_s, ego_d);
    const double target_clearance =
        blocked_risk_.wallClearance(ego_s, side_keep_target_d);
    const double effective_clearance = std::min(clearance, target_clearance);
    const bool outer_wall_risk =
        future_side &&
        effective_clearance < config_.future_side_yield_wall_clearance_m;

    if (future_side) {
      // 処理ブロック: 最も壁余裕が小さかった未来状態をdebug用に保持する。
      // 設計意図: なぜ譲りになったかをログ/レポートで後から追えるようにする。
      found_future_side = true;
      out.future_parallel_interaction =
          out.future_parallel_interaction || !exact_side_target;
      if (future_corner) {
        found_future_corner = true;
      }
      if (outer_wall_risk) {
        found_outer_wall_risk = true;
      }
      max_future_curvature = std::max(max_future_curvature, future_curvature);
      if (effective_clearance < best_clearance) {
        best_clearance = effective_clearance;
        out.future_delta_s = delta_s;
        out.future_delta_d = delta_d;
        out.future_wall_clearance_m = effective_clearance;
        out.future_abs_curvature = future_curvature;
        out.predicted_opponent_s = opp_s;
        out.predicted_opponent_d = opp_d;
        out.future_prediction_time_sec = t;
      }
    }

    if (future_side && future_corner &&
        (effective_clearance < config_.future_side_yield_wall_clearance_m ||
         outer_wall_risk)) {
      out.future_side_by_side = true;
      out.future_corner_side_by_side = true;
      out.future_outer_wall_risk = outer_wall_risk;
      out.future_yield_required = true;
      out.future_delta_s = delta_s;
      out.future_delta_d = delta_d;
      out.future_wall_clearance_m = effective_clearance;
      out.future_abs_curvature = future_curvature;
      out.predicted_opponent_s = opp_s;
      out.predicted_opponent_d = opp_d;
      out.future_prediction_time_sec = t;
      out.yield_reason =
          outer_wall_risk ? "future_outer_wall_risk" : "future_wall_clearance";
      return out;
    }
  }

  out.future_side_by_side = found_future_side;
  out.future_corner_side_by_side = found_future_corner;
  out.future_outer_wall_risk = found_outer_wall_risk;
  if (found_future_side && found_outer_wall_risk &&
      (exact_side_target || found_future_corner)) {
    out.future_yield_required = true;
    if (out.yield_reason.empty()) {
      out.yield_reason = "future_outer_wall_risk";
    }
  }
  out.future_abs_curvature =
      std::max(out.future_abs_curvature, max_future_curvature);
  if (!std::isfinite(out.future_wall_clearance_m) &&
      std::isfinite(best_clearance)) {
    out.future_wall_clearance_m = best_clearance;
  }
  return out;
}

// 入力: BlockedInfo。
// 出力: 未来横並び予測の対象index。無ければ-1。
// 処理概要: 現在横並び車両を優先し、なければ並走候補を使う。
int FutureSideBySideRiskAnalyzer::sideRiskIndex(
    const BlockedInfo &info) const {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

// 入力: 評価開始sとlookahead距離[m]。
// 出力: lookahead区間内の最大絶対曲率[1/m]。
// 処理概要: 将来横並びが直線ではなくコーナーで起きるかを軽量に判定する。
double FutureSideBySideRiskAnalyzer::maxAbsCurvatureAhead(
    double s, double lookahead_m) const {
  if (frame_.empty() || lookahead_m <= 0.0) {
    return 0.0;
  }
  const int sample_count = 8;
  const double ds = lookahead_m / static_cast<double>(sample_count);
  double max_abs_kappa = 0.0;
  for (int i = 0; i <= sample_count; ++i) {
    const auto ref = frame_.interpolate(s + ds * static_cast<double>(i));
    max_abs_kappa = std::max(max_abs_kappa, std::abs(ref.kappa));
  }
  return max_abs_kappa;
}

} // namespace overtake_planner
