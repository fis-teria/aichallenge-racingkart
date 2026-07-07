#include "overtake_planner/candidate_builder.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner {

namespace {

double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

double finitePositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

int sideRiskIndex(const BlockedInfo &info) {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

int yieldTargetIndex(const BlockedInfo &info) {
  return info.nearest_index >= 0 ? info.nearest_index : sideRiskIndex(info);
}

constexpr double kSideDirectionEpsilon = 0.05;
constexpr double kOutsideCorridorRecoveryShiftScale = 0.5;

} // namespace

CandidateBuilder::CandidateBuilder(const FrenetFrame &frame,
                                   const PlannerConfig &config)
    : frame_(frame), config_(config) {}

CandidateTrajectory CandidateBuilder::makeCandidate(
    CandidateType type, const EgoState &ego,
    const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  // 候補ごとに目標横オフセットと速度上限を決め、Frenet上で滑らかに接続する。
  CandidateTrajectory candidate;
  candidate.type = type;
  candidate.t.reserve(config_.horizon_points);
  candidate.s.reserve(config_.horizon_points);
  candidate.d.reserve(config_.horizon_points);
  candidate.x.reserve(config_.horizon_points);
  candidate.y.reserve(config_.horizon_points);
  candidate.yaw.reserve(config_.horizon_points);
  candidate.v_ref.reserve(config_.horizon_points);

  double target_d = 0.0;
  double shift_distance = config_.merge_distance_m;
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  const bool outside_safe_corridor =
      ego.frenet.d < lower_d || ego.frenet.d > upper_d;
  if (type == CandidateType::PASS_LEFT) {
    // 左右PASSは中心線から一定量オフセットした仮想参照をMPCへ渡す。
    target_d = config_.left_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::PASS_RIGHT) {
    target_d = config_.right_offset_m;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::FOLLOW) {
    target_d = 0.0;
  } else if (type == CandidateType::RECOVERY) {
    target_d = 0.0;
    if (outside_safe_corridor && !blocked_info.side_by_side) {
      const double base_distance =
          std::min(finitePositiveOr(config_.merge_distance_m, 12.0),
                   finitePositiveOr(config_.prepare_distance_m,
                                    config_.merge_distance_m));
      shift_distance =
          std::max(1.0, base_distance * kOutsideCorridorRecoveryShiftScale);
    }
  } else if (type == CandidateType::SIDE_BY_SIDE_KEEP) {
    target_d = ego.frenet.d;
    shift_distance = config_.side_by_side_shift_distance_m;
    if (blocked_info.side_index >= 0) {
      const auto &opp =
          opponents[static_cast<std::size_t>(blocked_info.side_index)];
      const double left_space = upper_d - ego.frenet.d;
      const double right_space = ego.frenet.d - lower_d;
      double away_sign = 0.0;
      if (std::abs(blocked_info.side_delta_d) > kSideDirectionEpsilon) {
        away_sign = blocked_info.side_delta_d > 0.0 ? -1.0 : 1.0;
      } else {
        away_sign = left_space >= right_space ? 1.0 : -1.0;
      }
      const double gap_target =
          opp.frenet.d + away_sign * config_.side_by_side_target_gap_m;
      target_d = away_sign > 0.0 ? std::max(ego.frenet.d, gap_target)
                                 : std::min(ego.frenet.d, gap_target);
      target_d = std::clamp(target_d, lower_d, upper_d);
    }
  } else if (type == CandidateType::YIELD_BEHIND) {
    const int target_index = yieldTargetIndex(blocked_info);
    const bool corner_yield = blocked_info.corner_side_by_side ||
                              blocked_info.future_corner_side_by_side ||
                              blocked_info.future_outer_wall_risk;
    target_d = std::clamp(config_.corner_yield_target_d_m, lower_d, upper_d);
    if (!corner_yield &&
        wallClearance(ego.frenet.d) >= config_.yield_rejoin_wall_clearance_m &&
        target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto &opp = opponents[static_cast<std::size_t>(target_index)];
      target_d = std::clamp(opp.frenet.d, lower_d, upper_d);
    }
  } else if (type == CandidateType::SAFE_STOP) {
    const bool release_threshold_active =
        std::isfinite(config_.safe_stop_lateral_error_threshold_m) &&
        config_.safe_stop_lateral_error_threshold_m >= 0.0;
    const bool safe_stop_lateral_error_remaining =
        release_threshold_active &&
        std::abs(ego.frenet.d) >
            config_.safe_stop_lateral_error_threshold_m;
    target_d = outside_safe_corridor || safe_stop_lateral_error_remaining
                   ? 0.0
                   : std::clamp(ego.frenet.d, lower_d, upper_d);
    target_d = std::clamp(target_d, lower_d, upper_d);
    shift_distance =
        std::max(config_.merge_distance_m, config_.prepare_distance_m);
  }

  double speed_cap = config_.v_passthrough_mps;
  const double yield_min_speed_cap =
      finitePositiveOr(config_.yield_min_speed_cap_mps, 0.5);
  const double ego_wall_clearance = wallClearance(ego.frenet.d);
  if (type == CandidateType::FOLLOW && blocked_info.nearest_index >= 0) {
    // FOLLOWは前走車より少し低い速度上限にして、MPC側の速度計画を抑える。
    const auto &opp =
        opponents[static_cast<std::size_t>(blocked_info.nearest_index)];
    speed_cap = std::max(0.5, opp.v - config_.follow_speed_margin_mps);
  } else if (type == CandidateType::RECOVERY) {
    speed_cap = ego_wall_clearance < 0.0
                    ? config_.wall_margin_recovery_v_max_mps
                    : config_.recovery_v_max_mps;
  } else if (type == CandidateType::SIDE_BY_SIDE_KEEP) {
    speed_cap = config_.side_by_side_speed_cap_mps;
    if (blocked_info.side_index >= 0) {
      const auto &opp =
          opponents[static_cast<std::size_t>(blocked_info.side_index)];
      speed_cap =
          std::min(speed_cap, std::max(yield_min_speed_cap,
                                       opp.v - config_.yield_speed_margin_mps));
    }
  } else if (type == CandidateType::YIELD_BEHIND) {
    speed_cap = yield_min_speed_cap;
    const int target_index = yieldTargetIndex(blocked_info);
    const bool corner_yield = blocked_info.corner_side_by_side ||
                              blocked_info.future_corner_side_by_side ||
                              blocked_info.future_outer_wall_risk;
    if (target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto &opp = opponents[static_cast<std::size_t>(target_index)];
      const double margin = corner_yield
                                ? config_.corner_follow_speed_margin_mps
                                : config_.yield_speed_margin_mps;
      speed_cap = std::max(yield_min_speed_cap, opp.v - margin);
    }
    if (corner_yield && config_.corner_yield_v_max_mps > 0.0) {
      speed_cap = std::min(speed_cap, config_.corner_yield_v_max_mps);
    }
  } else if (type == CandidateType::SAFE_STOP) {
    speed_cap = std::max(1.0e-3, config_.safe_stop_v_mps);
  }

  const bool recovery_like = type == CandidateType::RECOVERY ||
                             type == CandidateType::YIELD_BEHIND ||
                             type == CandidateType::SIDE_BY_SIDE_KEEP;
  if (recovery_like && ego_wall_clearance < 0.0) {
    speed_cap = std::min(speed_cap, config_.wall_margin_recovery_v_max_mps);
  }
  if (recovery_like && config_.large_lateral_error_threshold_m >= 0.0 &&
      config_.large_lateral_error_v_max_mps > 0.0 &&
      std::abs(ego.frenet.d - target_d) >
          config_.large_lateral_error_threshold_m) {
    speed_cap = std::min(speed_cap, config_.large_lateral_error_v_max_mps);
  }

  for (std::size_t i = 0; i < config_.horizon_points; ++i) {
    // 候補ごとの速度想定でs列を作り、smoothstepで横方向を急変させない。
    const double t = static_cast<double>(i) * config_.horizon_dt_sec;
    const double longitudinal_speed =
        type == CandidateType::SAFE_STOP
            ? std::max(1.0e-3, config_.safe_stop_v_mps)
            : std::max(0.5, ego.v);
    const double ds = longitudinal_speed * t;
    const double s = frame_.wrapS(ego.frenet.s + ds);
    double start_d = ego.frenet.d;
    if (type == CandidateType::RECOVERY ||
        type == CandidateType::YIELD_BEHIND ||
        type == CandidateType::SAFE_STOP) {
      start_d = std::clamp(start_d, lower_d, upper_d);
    }
    const bool release_threshold_active =
        std::isfinite(config_.recovery_release_lateral_error_m) &&
        config_.recovery_release_lateral_error_m >= 0.0;
    const bool recovery_lateral_error_remaining =
        release_threshold_active &&
        std::abs(start_d - target_d) >
            config_.recovery_release_lateral_error_m;
    const bool recovery_center_pull =
        type == CandidateType::RECOVERY && !blocked_info.side_by_side &&
        (outside_safe_corridor || recovery_lateral_error_remaining);
    const bool safe_stop_center_pull =
        type == CandidateType::SAFE_STOP &&
        (outside_safe_corridor ||
         std::abs(start_d - target_d) >
             std::max(0.0, config_.safe_stop_lateral_error_threshold_m));
    double ratio = smoothstep(ds / std::max(1.0, shift_distance));
    if ((recovery_center_pull || safe_stop_center_pull) &&
        config_.outside_corridor_recovery_centering_time_sec > 0.0) {
      ratio = std::max(
          ratio,
          smoothstep(t / config_.outside_corridor_recovery_centering_time_sec));
    }
    const double d = start_d + (target_d - start_d) * ratio;
    const auto p = frame_.frenetToCartesian(s, d);
    candidate.t.push_back(t);
    candidate.s.push_back(s);
    candidate.d.push_back(d);
    candidate.x.push_back(p.x);
    candidate.y.push_back(p.y);
    candidate.yaw.push_back(p.yaw);
    candidate.v_ref.push_back(speed_cap);
  }

  return candidate;
}

double CandidateBuilder::wallClearance(double d) const {
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

} // namespace overtake_planner
