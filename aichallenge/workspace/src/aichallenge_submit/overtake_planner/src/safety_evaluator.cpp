#include "overtake_planner/safety_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

bool finiteNondecreasing(const std::vector<double> &values,
                         bool require_nonnegative) {
  if (values.empty()) {
    return false;
  }
  double previous = -std::numeric_limits<double>::infinity();
  for (const double value : values) {
    if (!std::isfinite(value) || (require_nonnegative && value < 0.0) ||
        value < previous - 1.0e-9) {
      return false;
    }
    previous = value;
  }
  return true;
}

bool finiteVector(const std::vector<double> &values) {
  return !values.empty() &&
         std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

double interpolateYaw(double from_rad, double to_rad, double ratio) {
  constexpr double kTwoPi = 6.28318530717958647692;
  const double delta_rad = std::remainder(to_rad - from_rad, kTwoPi);
  return from_rad + ratio * delta_rad;
}

double interpolateLinear(double from, double to, double ratio) {
  return from + ratio * (to - from);
}

double candidatePathYaw(const CandidateTrajectory &candidate,
                        std::size_t index) {
  if (candidate.x.size() < 2U) {
    return candidate.yaw[index];
  }
  std::size_t from = index == 0U ? 0U : index - 1U;
  std::size_t to = index + 1U < candidate.x.size() ? index + 1U : index;
  const double dx = candidate.x[to] - candidate.x[from];
  const double dy = candidate.y[to] - candidate.y[from];
  if (std::hypot(dx, dy) <= 1.0e-6) {
    return candidate.yaw[index];
  }
  return std::atan2(dy, dx);
}

} // namespace

// 入力: PlannerConfig。壁マージン、安全楕円サイズ、許容安全余裕を含む。
// 出力: SafetyEvaluatorインスタンス。以後の候補評価で同じ設定を使う。
// 処理概要: 設定を値で保持し、評価中に外部パラメータが変わらないようにする。
SafetyEvaluator::SafetyEvaluator(PlannerConfig config) : config_(config) {}

SafetyEvaluator::SafetyEvaluator(const FrenetFrame &frame, PlannerConfig config)
    : frame_(&frame), config_(config) {}

// 入力: 候補軌道上の自車位置/姿勢と、同じ時刻の相手車位置。
// 出力: 安全楕円の余裕h。0より大きいほど楕円外側、負値は衝突領域内。
// 処理概要:
// 相対位置を自車body座標へ回し、前後/左右で別半径の楕円制約に変換する。
double SafetyEvaluator::ellipseMargin(double ego_x, double ego_y,
                                      double ego_yaw, double opp_x,
                                      double opp_y) const {
  // 自車の向きに合わせた楕円座標へ変換し、前後方向を広めに取った接近余裕を見る。
  const double dx = opp_x - ego_x;
  const double dy = opp_y - ego_y;
  const double c = std::cos(ego_yaw);
  const double s = std::sin(ego_yaw);
  const double x_body = c * dx + s * dy;
  const double y_body = -s * dx + c * dy;
  const double h = (x_body / config_.safety_ellipse_a_m) *
                       (x_body / config_.safety_ellipse_a_m) +
                   (y_body / config_.safety_ellipse_b_m) *
                       (y_body / config_.safety_ellipse_b_m) -
                   1.0;
  return h;
}

// 入力: 評価対象の候補軌道と、相手車の予測軌道リスト。
// 出力:
// 候補が安全ならtrue。不安全ならfalseを返し、candidate内に理由と余裕を記録する。
// 処理概要:
// まず壁マージンで早期rejectし、その後に各時刻の他車楕円制約を評価する。
bool SafetyEvaluator::evaluate(
    CandidateTrajectory &candidate,
    const std::vector<PredictedOpponent> &predictions) const {
  // 評価結果は候補に直接書き戻し、選択理由やレポート用の指標にも使えるようにする。
  candidate.safety_evaluated = true;
  candidate.feasible = true;
  candidate.min_safety_margin = std::numeric_limits<double>::infinity();
  candidate.cbf_slack = 0.0;
  candidate.active_safety_constraint_count = 0;
  candidate.reject_reason.clear();
  candidate.blocking_opponent_id.clear();
  candidate.blocking_time_sec = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_candidate_x_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_candidate_y_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_candidate_yaw_rad =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_candidate_s_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_candidate_d_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_opponent_x_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_opponent_y_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_opponent_s_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_opponent_d_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_footprint_valid = false;
  candidate.blocking_wall_segment_index = -1;
  candidate.blocking_wall_segment_ratio =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corner_index = -1;
  candidate.blocking_wall_time_sec = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_candidate_x_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_candidate_y_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_candidate_yaw_rad =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_candidate_s_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_candidate_d_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corner_x_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corner_y_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corner_s_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corner_d_m = std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corridor_d_min_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_corridor_d_max_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_physical_clearance_m =
      std::numeric_limits<double>::quiet_NaN();
  candidate.blocking_wall_effective_clearance_m =
      std::numeric_limits<double>::quiet_NaN();

  const std::size_t point_count = candidate.d.size();
  const bool candidate_shape_valid =
      point_count > 0U && candidate.x.size() == point_count &&
      candidate.y.size() == point_count &&
      candidate.yaw.size() == point_count &&
      candidate.t.size() == point_count &&
      candidate.longitudinal_offsets_m.size() == point_count &&
      candidate.s.size() == point_count &&
      candidate.predicted_speed_mps.size() == point_count &&
      candidate.v_ref.size() == point_count && finiteVector(candidate.d) &&
      finiteVector(candidate.x) && finiteVector(candidate.y) &&
      finiteVector(candidate.yaw) && finiteVector(candidate.t) &&
      finiteVector(candidate.longitudinal_offsets_m) &&
      finiteVector(candidate.s) &&
      finiteVector(candidate.predicted_speed_mps) &&
      finiteVector(candidate.v_ref) && finiteNondecreasing(candidate.t, true) &&
      finiteNondecreasing(candidate.longitudinal_offsets_m, true) &&
      std::all_of(candidate.predicted_speed_mps.begin(),
                  candidate.predicted_speed_mps.end(),
                  [](double value) { return value >= 0.0; }) &&
      std::all_of(candidate.v_ref.begin(), candidate.v_ref.end(),
                  [](double value) { return value >= 0.0; });
  if (!candidate_shape_valid) {
    candidate.feasible = false;
    candidate.reject_reason = "invalid_candidate_horizon";
    return false;
  }

  if (!candidate.pass_target_corridor_valid) {
    candidate.feasible = false;
    candidate.reject_reason = "pass_target_unreachable";
    return false;
  }

  if (!candidate.controller_tracking_profile_valid) {
    candidate.feasible = false;
    candidate.reject_reason = "untrackable_lateral_profile";
    return false;
  }

  if ((candidate.type == CandidateType::PASS_LEFT ||
       candidate.type == CandidateType::PASS_RIGHT) &&
      !candidate.moving_target_relatively_reachable) {
    candidate.feasible = false;
    candidate.reject_reason = "moving_target_not_relatively_reachable";
    return false;
  }

  const bool footprint_config_valid =
      std::isfinite(config_.ego_front_extent_m) &&
      config_.ego_front_extent_m >= 0.0 &&
      std::isfinite(config_.ego_rear_extent_m) &&
      config_.ego_rear_extent_m >= 0.0 &&
      std::isfinite(config_.ego_half_width_m) &&
      config_.ego_half_width_m >= 0.0 &&
      std::isfinite(config_.wall_localization_uncertainty_m) &&
      config_.wall_localization_uncertainty_m >= 0.0 &&
      std::isfinite(config_.wall_footprint_max_sample_distance_m) &&
      config_.wall_footprint_max_sample_distance_m > 0.0 &&
      std::isfinite(config_.wall_footprint_max_sample_yaw_rad) &&
      config_.wall_footprint_max_sample_yaw_rad > 0.0;
  if (config_.wall_footprint_check_enabled && !footprint_config_valid) {
    candidate.feasible = false;
    candidate.reject_reason = "invalid_wall_footprint_config";
    return false;
  }

  const auto evaluate_footprint = [&](double center_x, double center_y,
                                      double yaw, double sample_time_sec,
                                      int segment_index, double segment_ratio) {
    if (!config_.wall_footprint_check_enabled || frame_ == nullptr) {
      return true;
    }
    const double c = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const std::array<double, 4U> longitudinal_m{
        config_.ego_front_extent_m, config_.ego_front_extent_m,
        -config_.ego_rear_extent_m, -config_.ego_rear_extent_m};
    const std::array<double, 4U> lateral_m{
        config_.ego_half_width_m, -config_.ego_half_width_m,
        config_.ego_half_width_m, -config_.ego_half_width_m};
    for (std::size_t corner = 0U; corner < longitudinal_m.size(); ++corner) {
      const double corner_x =
          center_x + longitudinal_m[corner] * c - lateral_m[corner] * sin_yaw;
      const double corner_y =
          center_y + longitudinal_m[corner] * sin_yaw + lateral_m[corner] * c;
      const FrenetPose corner_pose =
          frame_->cartesianToFrenet(corner_x, corner_y, yaw);
      if (!std::isfinite(corner_pose.s) || !std::isfinite(corner_pose.d)) {
        candidate.feasible = false;
        candidate.reject_reason = "invalid_wall_footprint_projection";
        return false;
      }
      const auto corner_bounds = frame_->corridorBounds(
          corner_pose.s, config_.d_min_m, config_.d_max_m);
      const double physical_clearance_m =
          std::min(corner_pose.d - corner_bounds.d_min,
                   corner_bounds.d_max - corner_pose.d);
      const double effective_clearance_m =
          physical_clearance_m - config_.wall_localization_uncertainty_m;
      candidate.corridor_min_margin_m =
          std::isfinite(candidate.corridor_min_margin_m)
              ? std::min(candidate.corridor_min_margin_m, effective_clearance_m)
              : effective_clearance_m;
      if (effective_clearance_m < -1.0e-9) {
        candidate.feasible = false;
        candidate.reject_reason = "wall_footprint_margin";
        candidate.blocking_wall_footprint_valid = true;
        candidate.blocking_wall_segment_index = segment_index;
        candidate.blocking_wall_segment_ratio = segment_ratio;
        candidate.blocking_wall_corner_index = static_cast<int>(corner);
        candidate.blocking_wall_time_sec = sample_time_sec;
        candidate.blocking_wall_candidate_x_m = center_x;
        candidate.blocking_wall_candidate_y_m = center_y;
        candidate.blocking_wall_candidate_yaw_rad = yaw;
        // 診断map poseと同じsampleをFrenetへ再投影する。結果は認可判定へ
        // 戻さず、診断projectionが不成立でも既存wall rejectは変えない。
        const FrenetPose center_pose =
            frame_->cartesianToFrenet(center_x, center_y, yaw);
        candidate.blocking_wall_candidate_s_m =
            std::isfinite(center_pose.s)
                ? center_pose.s
                : std::numeric_limits<double>::quiet_NaN();
        candidate.blocking_wall_candidate_d_m =
            std::isfinite(center_pose.d)
                ? center_pose.d
                : std::numeric_limits<double>::quiet_NaN();
        candidate.blocking_wall_corner_x_m = corner_x;
        candidate.blocking_wall_corner_y_m = corner_y;
        candidate.blocking_wall_corner_s_m = corner_pose.s;
        candidate.blocking_wall_corner_d_m = corner_pose.d;
        candidate.blocking_wall_corridor_d_min_m = corner_bounds.d_min;
        candidate.blocking_wall_corridor_d_max_m = corner_bounds.d_max;
        candidate.blocking_wall_physical_clearance_m = physical_clearance_m;
        candidate.blocking_wall_effective_clearance_m = effective_clearance_m;
        return false;
      }
    }
    return true;
  };

  const auto evaluate_center_wall_margin = [&](double center_x, double center_y,
                                               double yaw) {
    if (frame_ == nullptr) {
      return true;
    }
    const FrenetPose center_pose =
        frame_->cartesianToFrenet(center_x, center_y, yaw);
    if (!std::isfinite(center_pose.s) || !std::isfinite(center_pose.d)) {
      candidate.feasible = false;
      candidate.reject_reason = "invalid_wall_footprint_projection";
      return false;
    }
    const auto center_bounds =
        frame_->corridorBounds(center_pose.s, config_.d_min_m, config_.d_max_m);
    if (center_pose.d < center_bounds.d_min + config_.min_wall_margin_m ||
        center_pose.d > center_bounds.d_max - config_.min_wall_margin_m) {
      candidate.feasible = false;
      candidate.reject_reason = "wall_margin";
      return false;
    }
    return true;
  };

  // 処理ブロック: 壁との安全余裕を先に確認する。
  // 設計意図:
  // 壁違反は相手車有無に関係なく危険なので、計算量の大きい相手車評価より前に落とす。
  for (std::size_t i = 0; i < candidate.d.size(); ++i) {
    // 横オフセットが壁マージンを割る候補は、他車を見る前に即rejectする。
    const double s = i < candidate.s.size() ? candidate.s[i] : 0.0;
    const auto bounds =
        frame_ == nullptr
            ? FrenetCorridorBounds{config_.d_min_m, config_.d_max_m}
            : frame_->corridorBounds(s, config_.d_min_m, config_.d_max_m);
    if (candidate.d[i] < bounds.d_min + config_.min_wall_margin_m ||
        candidate.d[i] > bounds.d_max - config_.min_wall_margin_m) {
      candidate.feasible = false;
      candidate.reject_reason = "wall_margin";
      return false;
    }

    if (config_.wall_footprint_check_enabled && frame_ != nullptr) {
      // corridorはlanelet路面端なので、中心点だけでなく実車体四隅を同じ
      // FrenetFrameへ戻す。横profile中の姿勢はCSV yawではなくCartesian列の
      // 接線から求め、旋回時の前端corner張り出しを見落とさない。
      const double yaw = candidatePathYaw(candidate, i);
      const int segment_index = i == 0U ? 0 : static_cast<int>(i - 1U);
      const double segment_ratio = i == 0U ? 0.0 : 1.0;
      if (!evaluate_footprint(candidate.x[i], candidate.y[i], yaw,
                              candidate.t[i], segment_index, segment_ratio)) {
        return false;
      }
      if (i > 0U) {
        const double previous_yaw = candidatePathYaw(candidate, i - 1U);
        const double distance_m =
            std::hypot(candidate.x[i] - candidate.x[i - 1U],
                       candidate.y[i] - candidate.y[i - 1U]);
        const double yaw_delta_rad = std::abs(
            std::remainder(yaw - previous_yaw, 6.28318530717958647692));
        const std::size_t distance_subdivisions =
            static_cast<std::size_t>(std::ceil(
                distance_m / config_.wall_footprint_max_sample_distance_m));
        const std::size_t yaw_subdivisions = static_cast<std::size_t>(std::ceil(
            yaw_delta_rad / config_.wall_footprint_max_sample_yaw_rad));
        const std::size_t subdivision_count = std::max<std::size_t>(
            1U, std::max(distance_subdivisions, yaw_subdivisions));
        for (std::size_t step = 1U; step < subdivision_count; ++step) {
          const double ratio = static_cast<double>(step) /
                               static_cast<double>(subdivision_count);
          const double center_x =
              interpolateLinear(candidate.x[i - 1U], candidate.x[i], ratio);
          const double center_y =
              interpolateLinear(candidate.y[i - 1U], candidate.y[i], ratio);
          const double sample_yaw = interpolateYaw(previous_yaw, yaw, ratio);
          const double sample_time_sec =
              interpolateLinear(candidate.t[i - 1U], candidate.t[i], ratio);
          if (!evaluate_center_wall_margin(center_x, center_y, sample_yaw) ||
              !evaluate_footprint(center_x, center_y, sample_yaw,
                                  sample_time_sec, static_cast<int>(i - 1U),
                                  ratio)) {
            return false;
          }
        }
      }
    }
  }

  // 処理ブロック: 相手車予測と候補軌道を同じhorizon indexで比較する。
  // 設計意図:
  // MPCへ渡す各点が将来の相手車位置と干渉しないことを候補単位で保証する。
  for (const auto &pred : predictions) {
    // 他車予測と候補軌道を同じhorizon
    // indexで突き合わせ、安全楕円の余裕を調べる。
    const bool prediction_shape_valid =
        pred.t.size() == point_count && pred.x.size() == point_count &&
        pred.y.size() == point_count && pred.s.size() == point_count &&
        pred.d.size() == point_count && finiteVector(pred.t) &&
        finiteVector(pred.x) && finiteVector(pred.y) && finiteVector(pred.s) &&
        finiteVector(pred.d) && finiteNondecreasing(pred.t, true);
    const bool prediction_time_axis_aligned =
        prediction_shape_valid &&
        std::equal(candidate.t.begin(), candidate.t.end(), pred.t.begin(),
                   [](double candidate_t, double prediction_t) {
                     return std::abs(candidate_t - prediction_t) <= 1.0e-3;
                   });
    if (!prediction_shape_valid || !prediction_time_axis_aligned) {
      candidate.feasible = false;
      candidate.reject_reason = prediction_shape_valid
                                    ? "opponent_prediction_time_mismatch"
                                    : "invalid_opponent_prediction_horizon";
      candidate.blocking_opponent_id = pred.id;
      return false;
    }
    const double max_evaluation_step_sec =
        std::isfinite(config_.horizon_dt_sec) && config_.horizon_dt_sec > 1.0e-3
            ? config_.horizon_dt_sec
            : 0.025;
    const auto evaluate_sample = [&](double ego_x, double ego_y, double ego_yaw,
                                     double ego_s, double ego_d, double opp_x,
                                     double opp_y, double opp_s, double opp_d,
                                     double sample_time_sec) {
      const double margin = ellipseMargin(ego_x, ego_y, ego_yaw, opp_x, opp_y);
      candidate.min_safety_margin =
          std::min(candidate.min_safety_margin, margin);
      if (margin <= config_.min_ellipse_h + 0.10) {
        // 閾値近傍の制約数を数え、後段の解析で「危なかった候補」を可視化する。
        ++candidate.active_safety_constraint_count;
      }
      if (margin <= config_.min_ellipse_h) {
        // 実際に閾値を割ったら不可。cbf_slackは不足量としてdebugへ出す。
        candidate.feasible = false;
        candidate.cbf_slack = config_.min_ellipse_h - margin;
        candidate.reject_reason = "opponent_collision";
        candidate.blocking_opponent_id = pred.id;
        candidate.blocking_time_sec = sample_time_sec;
        candidate.blocking_candidate_x_m = ego_x;
        candidate.blocking_candidate_y_m = ego_y;
        candidate.blocking_candidate_yaw_rad = ego_yaw;
        candidate.blocking_candidate_s_m = ego_s;
        candidate.blocking_candidate_d_m = ego_d;
        candidate.blocking_opponent_x_m = opp_x;
        candidate.blocking_opponent_y_m = opp_y;
        candidate.blocking_opponent_s_m = opp_s;
        candidate.blocking_opponent_d_m = opp_d;
        return false;
      }
      return true;
    };
    if (!evaluate_sample(candidate.x.front(), candidate.y.front(),
                         candidate.yaw.front(), candidate.s.front(),
                         candidate.d.front(), pred.x.front(), pred.y.front(),
                         pred.s.front(), pred.d.front(), candidate.t.front())) {
      return false;
    }
    for (std::size_t i = 1; i < point_count; ++i) {
      // ATTACK_FOLLOWはPPが使える空間長を確保するため時刻間隔が広がり得る。
      // 端点だけの比較では間を横切る車両を見落とすので、従来dt以下の間隔で
      // ego/opponentのswept segmentを補間評価する。SafetyEvaluatorの閾値や
      // 楕円自体は変えず、粗い時刻列によるfail-openだけを防ぐ。
      const double interval_sec = candidate.t[i] - candidate.t[i - 1U];
      const std::size_t subdivision_count = std::max<std::size_t>(
          1U, static_cast<std::size_t>(
                  std::ceil(interval_sec / max_evaluation_step_sec)));
      for (std::size_t step = 1; step <= subdivision_count; ++step) {
        const double ratio =
            static_cast<double>(step) / static_cast<double>(subdivision_count);
        const double ego_x =
            interpolateLinear(candidate.x[i - 1U], candidate.x[i], ratio);
        const double ego_y =
            interpolateLinear(candidate.y[i - 1U], candidate.y[i], ratio);
        const double ego_yaw =
            interpolateYaw(candidate.yaw[i - 1U], candidate.yaw[i], ratio);
        const double ego_s =
            interpolateLinear(candidate.s[i - 1U], candidate.s[i], ratio);
        const double ego_d =
            interpolateLinear(candidate.d[i - 1U], candidate.d[i], ratio);
        const double opp_x =
            interpolateLinear(pred.x[i - 1U], pred.x[i], ratio);
        const double opp_y =
            interpolateLinear(pred.y[i - 1U], pred.y[i], ratio);
        const double opp_s =
            interpolateLinear(pred.s[i - 1U], pred.s[i], ratio);
        const double opp_d =
            interpolateLinear(pred.d[i - 1U], pred.d[i], ratio);
        const double sample_time_sec =
            interpolateLinear(candidate.t[i - 1U], candidate.t[i], ratio);
        if (!evaluate_sample(ego_x, ego_y, ego_yaw, ego_s, ego_d, opp_x, opp_y,
                             opp_s, opp_d, sample_time_sec)) {
          return false;
        }
      }
    }
  }

  return true;
}

} // namespace overtake_planner
