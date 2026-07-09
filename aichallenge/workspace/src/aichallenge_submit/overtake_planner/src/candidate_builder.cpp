#include "overtake_planner/candidate_builder.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner {

namespace {

// 入力: 0から1へ進む正規化値z。
// 出力: 端点の傾きが0になる補間率。
// 処理概要: 横移動の開始/終了で急な速度変化を作らないための3次補間を返す。
double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

// 入力: 始点a、終点b、正規化された補間率ratio。
// 出力: smoothstepを通したaからbへの補間値。
// 処理概要: 局所回避プロファイルの横オフセットを滑らかにつなぐ。
double interpolate(double a, double b, double ratio) {
  return a + (b - a) * smoothstep(ratio);
}

// 入力: 候補値valueと、valueが使えない場合のfallback。
// 出力: valueが有限かつ正ならvalue、それ以外はfallback。
// 処理概要: 距離や速度の設定値が0/NaNでも安全側の既定値で処理を続ける。
double finitePositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

// 入力: BlockedInfo。横並び車両と広めの並走候補のindexを含む。
// 出力: 横方向リスクとして使う相手index。無ければ-1。
// 処理概要: 通常のside_indexを優先し、無い場合だけparallel_side_indexへフォールバックする。
int sideRiskIndex(const BlockedInfo &info) {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

// 入力: BlockedInfo。前方車両、横並び車両、並走候補のindexを含む。
// 出力: YIELD_BEHINDの追従対象index。無ければ-1。
// 処理概要: 前方閉塞車両を最優先し、横並び/並走リスクが残る場合はそれを譲り対象にする。
int yieldTargetIndex(const BlockedInfo &info) {
  return info.nearest_index >= 0 ? info.nearest_index : sideRiskIndex(info);
}

constexpr double kSideDirectionEpsilon = 0.05;
constexpr double kOutsideCorridorRecoveryShiftScale = 0.5;

} // namespace

// 入力: Frenet変換器とplanner設定。
// 出力: 候補軌道生成器のインスタンス。
// 処理概要: 参照線とパラメータを保持し、各CandidateTypeを同じ設定で軌道化できるようにする。
CandidateBuilder::CandidateBuilder(const FrenetFrame &frame,
                                   const PlannerConfig &config)
    : frame_(frame), config_(config) {}

// 入力: 候補種別、自車状態、閉塞判定、相手車一覧、任意の局所横プロファイル。
// 出力: Frenet/Cartesianのhorizon点と速度上限を持つCandidateTrajectory。
// 処理概要: modeそのものはここで決めず、候補種別ごとの「もしこの行動を取るなら」の軌道を作る。
CandidateTrajectory CandidateBuilder::makeCandidate(
    CandidateType type, const EgoState &ego,
    const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const LocalizedLateralProfile *localized_profile) const {
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
  // 処理ブロック: 候補種別から横方向の目標dと遷移距離を決める。
  // 設計意図: 状態機械は候補を比較するだけにし、軌道形状の責務をここへ閉じ込める。
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
    // 処理ブロック: 譲り時は相手の後ろに戻りやすい横位置を選ぶ。
    // 設計意図: コーナー譲りでは壁余裕を優先して中心へ、それ以外は相手のdへ寄せて後方追従へ入る。
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
  const bool use_localized_profile =
      localized_profile != nullptr && localized_profile->active &&
      (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      localized_profile->pass_type == type &&
      config_.overtake_lateral_profile_mode == "localized_latched";

  // 処理ブロック: 候補種別ごとの速度上限を決める。
  // 設計意図: 横参照だけでなく速度参照も同時に作り、MPCが危険な候補を速く走らないようにする。
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

  // 処理ブロック: 復帰系候補に追加の速度ガードを重ねる。
  // 設計意図: 横位置が危ない時は横軌道だけでなく速度も抑え、MPCが無理な回避を解かないようにする。
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

  // 処理ブロック: 時間horizonごとのs,d,x,y,yaw,v_refを生成する。
  // 設計意図: MPC入力は固定長配列なので、候補評価とpublishで同じhorizon列を再利用できる形にする。
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
    const double d =
        use_localized_profile
            ? localizedProfileD(*localized_profile, ego, s, ds)
            : start_d + (target_d - start_d) * ratio;
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

// 入力: ラッチ済み局所回避プロファイル、自車状態、評価対象s、現在からの距離ds。
// 出力: 壁マージン内にクランプした横オフセットd。
// 処理概要: 固定された局所プロファイルに、現在自車dへ連続接続する補正を短距離だけ重ねる。
double CandidateBuilder::localizedProfileD(
    const LocalizedLateralProfile &profile, const EgoState &ego, double s,
    double ds) const {
  const double nominal_current = nominalLocalizedProfileD(profile, ego.frenet.s);
  const double nominal = nominalLocalizedProfileD(profile, s);
  const double correction_distance =
      std::max(1.0, config_.localized_avoidance_start_before_target_m);
  const double correction_ratio = smoothstep(ds / correction_distance);
  const double correction = (ego.frenet.d - nominal_current) *
                            (1.0 - correction_ratio);
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  return std::clamp(nominal + correction, lower_d, upper_d);
}

// 入力: ラッチ済み局所回避プロファイルと評価対象s。
// 出力: 補正前の名目横オフセットd。
// 処理概要: 回避開始、最大オフセット保持、マージ終了の4区間でpiecewiseにdを返す。
double CandidateBuilder::nominalLocalizedProfileD(
    const LocalizedLateralProfile &profile, double s) const {
  if (!profile.active || !std::isfinite(profile.anchor_s_m)) {
    return 0.0;
  }

  const double unwrapped_s =
      profile.anchor_s_m + frame_.deltaS(profile.anchor_s_m, s);
  const double start_d = profile.start_d_m;
  const double target_d = profile.target_d_m;
  if (unwrapped_s <= profile.avoid_start_s_m) {
    return start_d;
  }
  if (unwrapped_s <= profile.full_offset_start_s_m) {
    const double distance = std::max(
        1.0e-3, profile.full_offset_start_s_m - profile.avoid_start_s_m);
    return interpolate(start_d, target_d,
                       (unwrapped_s - profile.avoid_start_s_m) / distance);
  }
  if (unwrapped_s <= profile.full_offset_end_s_m) {
    return target_d;
  }
  if (unwrapped_s <= profile.merge_end_s_m) {
    const double distance = std::max(
        1.0e-3, profile.merge_end_s_m - profile.full_offset_end_s_m);
    return interpolate(target_d, 0.0,
                       (unwrapped_s - profile.full_offset_end_s_m) / distance);
  }
  return 0.0;
}

// 入力: Frenet横位置d。
// 出力: 左右壁マージンのうち小さい方の余裕[m]。負なら安全コリドー外。
// 処理概要: 候補や現在位置が壁側へ寄りすぎていないかを1値で扱えるようにする。
double CandidateBuilder::wallClearance(double d) const {
  const double lower_d = config_.d_min_m + config_.min_wall_margin_m;
  const double upper_d = config_.d_max_m - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

} // namespace overtake_planner
