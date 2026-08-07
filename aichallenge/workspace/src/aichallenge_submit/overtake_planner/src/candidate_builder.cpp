#include "overtake_planner/candidate_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace overtake_planner {

namespace {

// 二分探索の成立境界をそのまま速度capへ使うと、最終horizonの浮動小数点丸めで
// 同じ操舵速度契約を数ulpだけ超える。設定済み20%操舵reserveとは別に、探索値へ
// 0.5%の数値/離散化余裕を残し、候補生成前後の判定を同じ安全側へそろえる。
constexpr double kTrackabilitySpeedSearchReserveRatio = 0.995;

// 位置差分由来のごく小さい速度ノイズだけを停止扱いにする。停止判定用の
// stationary_obstacle_speed_threshold_mpsは、正速度moving targetの
// 「抜き切れるか」という別契約には使わない。
constexpr double kMovingPassStationarySpeedEpsilonMps = 1.0e-3;

// 入力: 0から1へ進む正規化値z。
// 出力: 端点の傾きが0になる補間率。
// 処理概要: 横移動の開始/終了で急な速度変化を作らないための3次補間を返す。
double smoothstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * (3.0 - 2.0 * z);
}

// 入力: 0から1へ進む正規化値z。
// 出力: 位置・傾き・曲率が両端で連続する5次補間率。
// 処理概要: ATTACK_FOLLOWの再計画開始点で曲率をjumpさせず、MPCの操舵速度
// 制約を有限値として評価できる横profileを作る。
double smootherstep(double z) {
  z = std::clamp(z, 0.0, 1.0);
  return z * z * z * (z * (z * 6.0 - 15.0) + 10.0);
}

// 入力: 始点a、終点b、正規化された補間率ratio。
// 出力: smootherstepを通したaからbへの補間値。
// 処理概要: 局所回避プロファイルの横オフセットを滑らかにつなぐ。
double interpolate(double a, double b, double ratio) {
  return a + (b - a) * smootherstep(ratio);
}

// 入力: 候補値valueと、valueが使えない場合のfallback。
// 出力: valueが有限かつ正ならvalue、それ以外はfallback。
// 処理概要: 距離や速度の設定値が0/NaNでも安全側の既定値で処理を続ける。
double finitePositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

// 入力: ループ上のfrom/to位置と周長[m]。
// 出力: 半周以内に折り返した符号付き距離[m]。
// 処理概要: 連続s基準と現在/将来のwrapped sの局所差分へ使い、
// 停止時の微小後退を1周進捗と誤解釈しない。
double signedLocalDeltaS(double from_s, double to_s, double track_length_m) {
  double delta_s = to_s - from_s;
  if (std::isfinite(track_length_m) && track_length_m > 0.0) {
    delta_s = std::fmod(delta_s, track_length_m);
    if (delta_s > track_length_m * 0.5) {
      delta_s -= track_length_m;
    } else if (delta_s < -track_length_m * 0.5) {
      delta_s += track_length_m;
    }
  }
  return delta_s;
}

// 入力: ラッチ済みprofileと評価対象wrapped s。
// 出力: 周回を跨いでも連続するprofile座標上のs。
// 処理概要: Coreが積算した直近ego sを基準に局所差分だけを足す。旧profileや
// 単体fixtureで積算値が無い場合だけanchor基準へ後方互換fallbackする。
double profileUnwrappedS(const LocalizedLateralProfile &profile,
                         double wrapped_s, double track_length_m) {
  if (std::isfinite(profile.ego_unwrapped_s_m) &&
      std::isfinite(profile.last_ego_wrapped_s_m)) {
    return profile.ego_unwrapped_s_m +
           signedLocalDeltaS(profile.last_ego_wrapped_s_m, wrapped_s,
                             track_length_m);
  }
  return profile.anchor_s_m +
         signedLocalDeltaS(profile.anchor_s_m, wrapped_s, track_length_m);
}

// 入力: 安全楕円の前後半径aと許容最小余裕h。
// 出力: 横方向の分離が0でも楕円外にいられる前後距離[m]。
// 処理概要: h=(ds/a)^2+(dd/b)^2-1 のdd=0を解き、
// SafetyEvaluatorと同じ境界からプロファイルの収束期限を作る。
double longitudinalEllipseClearance(const PlannerConfig &config) {
  const double a_m = std::max(0.0, config.safety_ellipse_a_m);
  const double min_h = std::max(0.0, config.min_ellipse_h);
  return a_m * std::sqrt(1.0 + min_h);
}

// 入力: BlockedInfo。横並び車両と広めの並走候補のindexを含む。
// 出力: 横方向リスクとして使う相手index。無ければ-1。
// 処理概要:
// 通常のside_indexを優先し、無い場合だけparallel_side_indexへフォールバックする。
int sideRiskIndex(const BlockedInfo &info) {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

// 入力: BlockedInfo。前方車両、横並び車両、並走候補のindexを含む。
// 出力: YIELD_BEHINDの追従対象index。無ければ-1。
// 処理概要:
// 前方閉塞車両を最優先し、横並び/並走リスクが残る場合はそれを譲り対象にする。
int yieldTargetIndex(const BlockedInfo &info) {
  if (info.start_grid_target_active && info.start_grid_target_index >= 0) {
    return info.start_grid_target_index;
  }
  if (info.nearest_index >= 0) {
    return info.nearest_index;
  }
  if (info.braking_follow_active && info.braking_follow_index >= 0) {
    return info.braking_follow_index;
  }
  return sideRiskIndex(info);
}

// 入力: BlockedInfoと現周期の相手一覧。
// 出力: PASS横分離を計算するauthoritative target index。無ければ従来target。
// 処理概要: gentle-curve制約でlocalized profileを使わない周期でも、nearestの
// 一時変化で別車両のdを使わず、freshなmaneuver target IDを優先する。
int passTargetIndex(const BlockedInfo &info,
                    const std::vector<OpponentState> &opponents) {
  if (info.maneuver_target_latched && info.maneuver_target_observed &&
      info.maneuver_target_fresh && info.maneuver_target_index >= 0 &&
      static_cast<std::size_t>(info.maneuver_target_index) < opponents.size() &&
      !info.maneuver_target_id.empty() &&
      opponents[static_cast<std::size_t>(info.maneuver_target_index)].id ==
          info.maneuver_target_id) {
    return info.maneuver_target_index;
  }
  return yieldTargetIndex(info);
}

// 入力: BlockedInfo。通常前走車とparallel FOLLOW候補のindexを含む。
// 出力: FOLLOW速度capの対象index。無ければ-1。
// 処理概要:
// 通常の同一コリドー前走車を優先し、無い場合だけparallel由来FOLLOW対象を使う。
int followTargetIndex(const BlockedInfo &info) {
  if (info.attack_follow_hold_pass_side && info.maneuver_chain_tail_observed &&
      info.maneuver_chain_tail_index >= 0) {
    return info.maneuver_chain_tail_index;
  }
  if (info.start_grid_target_active && info.start_grid_target_index >= 0) {
    return info.start_grid_target_index;
  }
  if (info.braking_follow_active && info.braking_follow_index >= 0) {
    return info.braking_follow_index;
  }
  if (info.nearest_index >= 0) {
    return info.nearest_index;
  }
  return info.parallel_follow_candidate ? info.parallel_follow_index : -1;
}

constexpr double kSideDirectionEpsilon = 0.05;
constexpr double kOutsideCorridorRecoveryShiftScale = 0.5;
constexpr double kLateralCorrectionDistanceUpperBoundM = 100.0;
constexpr double kLateralCorrectionTrackabilitySpacingM = 0.05;
constexpr std::size_t kLateralCorrectionTrackabilityMaxSamples = 2000U;

// 入力: 候補種別。
// 出力: 速度capまでの減速到達可能性を安全評価へ反映すべきならtrue。
// 処理概要:
// FASTESTは下流の加速を仮定せず現速度で予測する。追従/譲り/停止に加え、
// lateral-first gateで相手速度へ閉じたPASSも同じ減速モデルを使う。
bool usesBrakingProfile(CandidateType type) {
  return type == CandidateType::FOLLOW || type == CandidateType::PASS_LEFT ||
         type == CandidateType::PASS_RIGHT || type == CandidateType::RECOVERY ||
         type == CandidateType::SIDE_BY_SIDE_KEEP ||
         type == CandidateType::YIELD_BEHIND ||
         type == CandidateType::SAFE_STOP;
}

} // namespace

// 入力: Frenet変換器とplanner設定。
// 出力: 候補軌道生成器のインスタンス。
// 処理概要:
// 参照線とパラメータを保持し、各CandidateTypeを同じ設定で軌道化できるようにする。
CandidateBuilder::CandidateBuilder(const FrenetFrame &frame,
                                   const PlannerConfig &config)
    : frame_(frame), config_(config) {}

double CandidateBuilder::requiredControllerSpatialHorizon(
    double execution_speed_mps, double controller_target_speed_mps,
    bool attack_follow_profile) const {
  constexpr double kControllerMinSpatialArcM = 0.50;
  constexpr double kControllerMinForwardTimeSec = 0.75;
  constexpr double kControllerMinResponseDelaySec = 0.25;
  constexpr double kControllerMaxVerifiedBrakeDecelMps2 = 1.0;
  constexpr double kControllerMaxFailSafeSpeedMps = 0.20;
  if (!std::isfinite(execution_speed_mps) || execution_speed_mps < 0.0 ||
      !std::isfinite(controller_target_speed_mps) ||
      controller_target_speed_mps < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double target_speed_mps =
      std::max(execution_speed_mps, controller_target_speed_mps);
  const double required_lookahead_m =
      config_.lateral_override_lookahead_gain * target_speed_mps +
      config_.lateral_override_lookahead_min_distance_m;
  const double verified_brake_decel_mps2 = std::min(
      config_.max_brake_decel_mps2, kControllerMaxVerifiedBrakeDecelMps2);
  const double response_delay_sec = std::max(
      config_.longitudinal_response_delay_sec, kControllerMinResponseDelaySec);
  const double fail_safe_speed_mps =
      std::clamp(config_.safe_stop_v_mps, 0.0, kControllerMaxFailSafeSpeedMps);
  double required_brake_distance_m = 0.0;
  if (std::isfinite(verified_brake_decel_mps2) &&
      verified_brake_decel_mps2 > 0.0 &&
      execution_speed_mps > fail_safe_speed_mps + 1.0e-9) {
    required_brake_distance_m = execution_speed_mps * response_delay_sec +
                                (execution_speed_mps * execution_speed_mps -
                                 fail_safe_speed_mps * fail_safe_speed_mps) /
                                    (2.0 * verified_brake_decel_mps2);
  }
  double required_arc_m =
      std::max({kControllerMinSpatialArcM,
                execution_speed_mps * kControllerMinForwardTimeSec,
                required_lookahead_m, required_brake_distance_m});
  if (attack_follow_profile) {
    required_arc_m =
        std::max(required_arc_m, config_.attack_follow_min_spatial_horizon_m);
  }
  return required_arc_m;
}

// 入力: 予測時刻[sec]。
// 出力: 下流制御の遅れと減速上限を反映した、保守的な前進距離[m]。
// 処理概要:
// 速度capそのものではなく、応答遅れ後に最大制動だけが掛かる最遠到達距離を積分する。
double CandidateBuilder::LongitudinalProfile::distanceAt(double t_sec) const {
  const double t = std::max(0.0, t_sec);
  const double initial_speed = std::max(0.0, initial_speed_mps);
  const double target_speed = std::max(0.0, target_speed_mps);
  if (!valid) {
    return initial_speed * t;
  }
  if (acceleration_requested && target_speed > initial_speed &&
      accel_mps2 > 0.0) {
    const double acceleration_time =
        (target_speed - initial_speed) / accel_mps2;
    if (t <= acceleration_time) {
      return initial_speed * t + 0.5 * accel_mps2 * t * t;
    }
    const double acceleration_distance =
        initial_speed * acceleration_time +
        0.5 * accel_mps2 * acceleration_time * acceleration_time;
    return acceleration_distance + target_speed * (t - acceleration_time);
  }
  if (!braking_requested || target_speed >= initial_speed ||
      brake_decel_mps2 <= 0.0) {
    return initial_speed * t;
  }

  const double delay = std::max(0.0, response_delay_sec);
  if (t <= delay) {
    return initial_speed * t;
  }
  const double braking_time = t - delay;
  const double required_braking_time =
      (initial_speed - target_speed) / brake_decel_mps2;
  if (braking_time <= required_braking_time) {
    return initial_speed * delay + initial_speed * braking_time -
           0.5 * brake_decel_mps2 * braking_time * braking_time;
  }
  const double braking_distance =
      initial_speed * required_braking_time -
      0.5 * brake_decel_mps2 * required_braking_time * required_braking_time;
  return initial_speed * delay + braking_distance +
         target_speed * (braking_time - required_braking_time);
}

// 入力: 予測時刻[sec]。
// 出力: 同じ遅れ・減速度モデルで、その時刻に到達可能と仮定する速度[m/s]。
// 処理概要: s(t) と整合する予測速度を返す。下流へ出すv_refは毎周期index 0を
// 直ちに消費するため、この予測値とは分離して即時capを渡す。
double CandidateBuilder::LongitudinalProfile::speedAt(double t_sec) const {
  const double initial_speed = std::max(0.0, initial_speed_mps);
  const double target_speed = std::max(0.0, target_speed_mps);
  if (!valid) {
    return initial_speed;
  }
  if (acceleration_requested && target_speed > initial_speed &&
      accel_mps2 > 0.0) {
    return std::min(target_speed,
                    initial_speed + accel_mps2 * std::max(0.0, t_sec));
  }
  if (!braking_requested || brake_decel_mps2 <= 0.0) {
    return initial_speed;
  }

  const double braking_time = std::max(0.0, t_sec - response_delay_sec);
  return std::max(target_speed,
                  initial_speed - brake_decel_mps2 * braking_time);
}

// 入力: 目標速度[m/s]。
// 出力: 応答遅れを含め、その目標速度まで減速するのに必要な距離[m]。
// 処理概要: 前方停止車との間隔をdebugへ出すため、候補と同じ制動モデルを使う。
double CandidateBuilder::LongitudinalProfile::requiredDistanceTo(
    double target_speed_mps) const {
  const double initial_speed = std::max(0.0, initial_speed_mps);
  const double target_speed = std::clamp(target_speed_mps, 0.0, initial_speed);
  if (target_speed >= initial_speed) {
    return 0.0;
  }
  if (!valid || brake_decel_mps2 <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return initial_speed * std::max(0.0, response_delay_sec) +
         (initial_speed * initial_speed - target_speed * target_speed) /
             (2.0 * brake_decel_mps2);
}

// 入力: 候補種別、自車状態、速度cap[m/s]。
// 出力: 候補安全評価で使う縦方向到達可能性プロファイル。
// 処理概要: 過大な制動能力を仮定しないため、plannerの1.5 m/s^2保守上限内の
// 設定値だけを採用する。不正値は減速候補を無効化する。
CandidateBuilder::LongitudinalProfile CandidateBuilder::makeLongitudinalProfile(
    CandidateType type, const EgoState &ego, double speed_cap_mps,
    bool acceleration_allowed, bool controller_spatial_profile) const {
  LongitudinalProfile profile;
  const double measured_speed_mps = std::max(0.0, ego.v);
  profile.initial_speed_mps = measured_speed_mps;
  const bool acceleration_commanded =
      ((type == CandidateType::RECOVERY) ||
       ((type == CandidateType::FOLLOW || type == CandidateType::PASS_LEFT ||
         type == CandidateType::PASS_RIGHT) &&
        acceleration_allowed)) &&
      std::isfinite(speed_cap_mps) &&
      speed_cap_mps > measured_speed_mps + 1.0e-6;
  // 下流へ出すtargetは実測速度を基準に決める。残留加速headroomは安全予測専用で、
  // 未認可FOLLOWや停止候補の速度指令を実測値より上へ持ち上げてはならない。
  profile.target_speed_mps =
      acceleration_commanded
          ? std::max(0.0, speed_cap_mps)
          : std::clamp(speed_cap_mps, 0.0, measured_speed_mps);
  if (controller_spatial_profile) {
    // 速度capを下げた周期でも、前周期の駆動指令と下流実行遅れの間は実車が
    // 加速を続け得る。target速度でreserveを頭打ちにせず、検証済み加速度上限を
    // 瞬時速度headroomとして最初から加える。実際のrampより前進量を大きくし、
    // SafetyEvaluatorとPPの空間arcを同じ保守側へ揃える。
    const double reserve_sec =
        config_.lateral_override_execution_speed_reserve_sec;
    const double accel_bound_mps2 = config_.pass_assumed_accel_mps2;
    if (!std::isfinite(reserve_sec) || reserve_sec < 0.05 ||
        reserve_sec > 0.50 || !std::isfinite(accel_bound_mps2) ||
        accel_bound_mps2 <= 0.0 || accel_bound_mps2 > 3.0) {
      profile.valid = false;
      return profile;
    }
    profile.initial_speed_mps += accel_bound_mps2 * reserve_sec;
  }
  const bool acceleration_requested =
      acceleration_commanded &&
      profile.target_speed_mps > profile.initial_speed_mps + 1.0e-6;
  if (acceleration_requested) {
    profile.acceleration_requested = true;
    const double assumed_accel_mps2 =
        type == CandidateType::RECOVERY
            ? config_.recovery_assumed_accel_mps2
            : (type == CandidateType::FOLLOW
                   ? config_.follow_gap_closing_assumed_accel_mps2
                   : config_.pass_assumed_accel_mps2);
    if (!std::isfinite(assumed_accel_mps2) || assumed_accel_mps2 <= 0.0 ||
        assumed_accel_mps2 > 3.0) {
      profile.valid = false;
      return profile;
    }
    profile.accel_mps2 = assumed_accel_mps2;
    return profile;
  }
  const bool braking_requested =
      usesBrakingProfile(type) &&
      profile.target_speed_mps + 1.0e-6 < profile.initial_speed_mps;
  profile.braking_requested = braking_requested;
  if (!braking_requested) {
    return profile;
  }
  if (!std::isfinite(config_.max_brake_decel_mps2) ||
      config_.max_brake_decel_mps2 <= 0.0 ||
      config_.max_brake_decel_mps2 > 1.5 ||
      !std::isfinite(config_.longitudinal_response_delay_sec) ||
      config_.longitudinal_response_delay_sec < 0.0) {
    profile.valid = false;
    return profile;
  }
  // PPが空間horizonの停止可能性に使う検証済み上限は1.0 m/s^2。
  // planner側だけ1.5 m/s^2で早く減速する前提にすると、安全予測s(t)が短くなり
  // PP実行時の必要arcに届かない。横override候補は弱い側へ統一する。
  constexpr double kControllerMaxVerifiedBrakeDecelMps2 = 1.0;
  constexpr double kControllerMinResponseDelaySec = 0.25;
  profile.brake_decel_mps2 =
      controller_spatial_profile
          ? std::min(config_.max_brake_decel_mps2,
                     kControllerMaxVerifiedBrakeDecelMps2)
          : config_.max_brake_decel_mps2;
  profile.response_delay_sec =
      controller_spatial_profile
          ? std::max(config_.longitudinal_response_delay_sec,
                     kControllerMinResponseDelaySec)
          : config_.longitudinal_response_delay_sec;
  return profile;
}

// 入力: 候補種別、自車状態、閉塞判定、相手車一覧、任意の局所横プロファイル。
// 出力: Frenet/Cartesianのhorizon点と速度上限を持つCandidateTrajectory。
// 処理概要:
// modeそのものはここで決めず、候補種別ごとの「もしこの行動を取るなら」の軌道を作る。
CandidateTrajectory CandidateBuilder::makeCandidate(
    CandidateType type, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const LocalizedLateralProfile *localized_profile,
    bool force_localized_pass_profile, CandidatePurpose purpose) const {
  // 候補ごとに目標横オフセットと速度上限を決め、Frenet上で滑らかに接続する。
  CandidateTrajectory candidate;
  candidate.type = type;
  candidate.longitudinal_initial_measured_speed_mps = std::max(0.0, ego.v);
  candidate.t.reserve(config_.horizon_points);
  candidate.longitudinal_offsets_m.reserve(config_.horizon_points);
  candidate.s.reserve(config_.horizon_points);
  candidate.d.reserve(config_.horizon_points);
  candidate.x.reserve(config_.horizon_points);
  candidate.y.reserve(config_.horizon_points);
  candidate.yaw.reserve(config_.horizon_points);
  candidate.predicted_speed_mps.reserve(config_.horizon_points);
  candidate.v_ref.reserve(config_.horizon_points);

  double target_d = 0.0;
  double unconstrained_pass_target_d = std::numeric_limits<double>::quiet_NaN();
  double shift_distance = config_.merge_distance_m;
  const auto ego_bounds =
      frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
  const double lower_d = ego_bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = ego_bounds.d_max - config_.min_wall_margin_m;
  const bool outside_safe_corridor =
      ego.frenet.d < lower_d || ego.frenet.d > upper_d;
  // 旧dev3 bagの開始20 sで観測した停止中dノイズ全幅0.249 mmを通常上限にし、
  // Gate 2で観測した未発進settling 1.52 mmだけは追加の壁/相手reserve検査付きで
  // 2 mmまで許可する。2 cm級の差を同じ軌道として扱わない。
  constexpr double kPreparedHoldMaxLateralErrorM = 5.0e-4;
  constexpr double kStartGridHoldMaxLateralErrorM = 2.0e-3;
  constexpr double kPreparedHoldStationarySpeedUpperBoundMps = 0.02;
  // Gate 2実走では発進直後0.02163 m/sで0.02 m/s境界を跨ぎ、PASSがまだ
  // publish/commitされていないのにcurrent-d holdからanchor補正へ切り替わった。
  // safe-stop crawl以下は同じ低速準備区間として扱い、FOLLOW/RECOVERYの双方で
  // 2 mm以内の実測dを保持する。実軌道は後段のSafetyEvaluatorへ毎周期通す。
  const double prepared_hold_low_speed_upper_bound_mps =
      // PP空間horizon契約も0.20 m/sを最大fail-safe速度として検証している。
      // 設定がそれより小さくても同じ検証済み低速域を途中で分断しない。
      std::max(0.20, std::max(0.0, config_.safe_stop_v_mps));
  const bool start_grid_pre_execution_low_speed_hold =
      (type == CandidateType::FOLLOW || type == CandidateType::RECOVERY) &&
      blocked_info.start_grid_target_active && std::isfinite(ego.v) &&
      std::abs(ego.v) <= prepared_hold_low_speed_upper_bound_mps;
  const double prepared_hold_max_lateral_error_m =
      start_grid_pre_execution_low_speed_hold ? kStartGridHoldMaxLateralErrorM
                                              : kPreparedHoldMaxLateralErrorM;
  const bool prepared_hold_wall_reserve =
      ego.frenet.d >= lower_d + prepared_hold_max_lateral_error_m &&
      ego.frenet.d <= upper_d - prepared_hold_max_lateral_error_m &&
      localized_profile != nullptr &&
      std::isfinite(localized_profile->start_d_m) &&
      localized_profile->start_d_m >=
          lower_d + prepared_hold_max_lateral_error_m &&
      localized_profile->start_d_m <=
          upper_d - prepared_hold_max_lateral_error_m;
  const bool prepared_hold_opponent_reserve = [&]() {
    if (localized_profile == nullptr ||
        !std::isfinite(localized_profile->start_d_m) ||
        !std::isfinite(ego.frenet.s) || config_.safety_ellipse_a_m <= 0.0 ||
        config_.safety_ellipse_b_m <= 0.0) {
      return false;
    }
    const auto start_point =
        frame_.frenetToCartesian(ego.frenet.s, localized_profile->start_d_m);
    const auto actual_point =
        frame_.frenetToCartesian(ego.frenet.s, ego.frenet.d);
    if (!std::isfinite(start_point.x) || !std::isfinite(start_point.y) ||
        !std::isfinite(start_point.yaw) || !std::isfinite(actual_point.x) ||
        !std::isfinite(actual_point.y)) {
      return false;
    }
    const double c = std::cos(start_point.yaw);
    const double s = std::sin(start_point.yaw);
    const auto min_abs_on_interval = [](double lhs, double rhs) {
      return (lhs <= 0.0 && rhs >= 0.0) || (rhs <= 0.0 && lhs >= 0.0)
                 ? 0.0
                 : std::min(std::abs(lhs), std::abs(rhs));
    };
    for (const auto &opponent : opponents) {
      if (!opponent.valid || !std::isfinite(opponent.x) ||
          !std::isfinite(opponent.y)) {
        return false;
      }
      const auto relative_body = [&](double ego_x, double ego_y) {
        const double dx = opponent.x - ego_x;
        const double dy = opponent.y - ego_y;
        return std::pair<double, double>{c * dx + s * dy, -s * dx + c * dy};
      };
      const auto start_relative = relative_body(start_point.x, start_point.y);
      const auto actual_relative =
          relative_body(actual_point.x, actual_point.y);
      // 各body軸の区間最小値を別々に使うため、実際の同一点最小値以下となる
      // 保守評価である。2 mm swept segmentが楕円境界へ近い時は固定しない。
      const double min_abs_x =
          min_abs_on_interval(start_relative.first, actual_relative.first);
      const double min_abs_y =
          min_abs_on_interval(start_relative.second, actual_relative.second);
      const double margin =
          std::pow(min_abs_x / config_.safety_ellipse_a_m, 2.0) +
          std::pow(min_abs_y / config_.safety_ellipse_b_m, 2.0) - 1.0;
      if (!std::isfinite(margin) || margin <= config_.min_ellipse_h + 0.10) {
        return false;
      }
    }
    return true;
  }();
  // 初回Gate 2認可の前後を問わず、まだstart-grid/parallel停止holdを選ぶ間は
  // sub-mm級の観測差に対して同じstart-dを使う。候補は毎周期
  // SafetyEvaluatorへ通し、許容超過、壁/相手reserve喪失、hold解除の
  // いずれかでprofile固定を止める。
  const bool prepared_profile_hold =
      localized_profile != nullptr && localized_profile->active &&
      (blocked_info.start_grid_follow_hold_lateral ||
       blocked_info.early_stationary_parallel_pass_hold_lateral ||
       start_grid_pre_execution_low_speed_hold) &&
      std::isfinite(localized_profile->start_d_m) &&
      std::abs(ego.frenet.d - localized_profile->start_d_m) <=
          prepared_hold_max_lateral_error_m &&
      prepared_hold_wall_reserve && prepared_hold_opponent_reserve;
  const double prepared_hold_d =
      prepared_profile_hold
          ? std::clamp(localized_profile->start_d_m, lower_d, upper_d)
          : std::clamp(ego.frenet.d, lower_d, upper_d);
  const bool start_grid_anchor_context =
      blocked_info.start_grid_target_active &&
      blocked_info.start_grid_follow_hold_lateral &&
      (type == CandidateType::FOLLOW || type == CandidateType::RECOVERY);
  const bool start_grid_anchor_valid =
      start_grid_anchor_context &&
      std::isfinite(blocked_info.start_grid_hold_target_d_m) &&
      blocked_info.start_grid_hold_target_d_m >= lower_d &&
      blocked_info.start_grid_hold_target_d_m <= upper_d;
  const bool start_grid_low_speed_measurement_hold =
      start_grid_anchor_valid && start_grid_pre_execution_low_speed_hold &&
      prepared_hold_wall_reserve && prepared_hold_opponent_reserve;
  const double start_grid_hold_d =
      blocked_info.start_grid_uncommitted_hold_active
          ? std::clamp(ego.frenet.d, lower_d, upper_d)
      : start_grid_low_speed_measurement_hold
          // Gate 2認可前の低速発進で生じた数mmの実車移動を、anchorへ戻す
          // 新しい横操舵要求へ変換しない。profileのID/side/start
          // d契約は保持した まま、FOLLOW/RECOVERY候補だけを実測current-d
          // holdにし、この列を 壁・全相手SafetyEvaluatorへ通す。wall/opponent
          // reserve喪失または safe-stop
          // crawl超過時は従来の追従可能なanchor補正へ戻る。
          ? ego.frenet.d
          : start_grid_anchor_valid
                ? std::clamp(blocked_info.start_grid_hold_target_d_m, lower_d,
                             upper_d)
                : prepared_hold_d;
  // anchorと実測dが同値、または低速準備中にwall/opponent reserveを保つなら
  // 横補正を要求せずcurrent-dを保持する。特に低速RECOVERYを、不要なPP空間
  // horizon不足だけで停止不能扱いにしない。補正が必要な場合も実測dを始点にする。
  constexpr double kNoLateralCorrectionEpsilonM = 1.0e-9;
  const bool start_grid_anchored_hold_profile =
      start_grid_anchor_valid &&
      std::abs(start_grid_hold_d - ego.frenet.d) > kNoLateralCorrectionEpsilonM;
  const bool start_grid_uncommitted_current_d_candidate =
      blocked_info.start_grid_uncommitted_hold_active &&
      (type == CandidateType::FOLLOW || type == CandidateType::RECOVERY ||
       type == CandidateType::YIELD_BEHIND || type == CandidateType::SAFE_STOP);
  bool start_grid_uncommitted_future_corridor_valid = true;
  if (start_grid_anchor_context && !start_grid_anchor_valid) {
    // activeなstart-grid契約のanchorがNaNまたは現在回廊外なら、現在dへ暗黙
    // fallbackして「anchor保持済み」とは扱わない。Coreで明示rejectする。
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.corridor_min_margin_m = -std::numeric_limits<double>::infinity();
  }
  const bool attack_follow_profile =
      type == CandidateType::FOLLOW &&
      blocked_info.attack_follow_hold_pass_side &&
      std::isfinite(blocked_info.attack_follow_target_d_m);
  const bool prestart_attack_follow_profile =
      type == CandidateType::FOLLOW &&
      blocked_info.prestart_attack_follow_hold_lateral;
  const bool pass_reauthorization_recovery_profile =
      type == CandidateType::RECOVERY &&
      blocked_info.pass_reauthorization_lockout_active &&
      std::isfinite(blocked_info.pass_reauthorization_recovery_target_d_m);
  const double committed_attack_follow_target_d_m =
      attack_follow_profile
          ? std::clamp(blocked_info.attack_follow_target_d_m, lower_d, upper_d)
          : std::numeric_limits<double>::quiet_NaN();
  if (attack_follow_profile) {
    candidate.committed_attack_follow_target_d_m =
        committed_attack_follow_target_d_m;
  }
  // 処理ブロック: 候補種別から横方向の目標dと遷移距離を決める。
  // 設計意図:
  // 状態機械は候補を比較するだけにし、軌道形状の責務をここへ閉じ込める。
  if (type == CandidateType::PASS_LEFT) {
    // 対象を特定できない候補だけは従来の固定offsetへフォールバックする。
    // 通常PASSは相手楕円間隔を満たす最小横移動を選び、不要な壁側への
    // 横断を作らない。
    target_d = config_.left_offset_m;
    const int target_index = passTargetIndex(blocked_info, opponents);
    if (target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const double required_gap =
          config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h) +
          std::max(0.0, config_.pass_target_lateral_margin_m);
      target_d = passTargetOffset(
          config_, type, ego.frenet.d,
          opponents[static_cast<std::size_t>(target_index)].frenet.d,
          required_gap);
    }
    unconstrained_pass_target_d = target_d;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::PASS_RIGHT) {
    target_d = config_.right_offset_m;
    const int target_index = passTargetIndex(blocked_info, opponents);
    if (target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const double required_gap =
          config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h) +
          std::max(0.0, config_.pass_target_lateral_margin_m);
      target_d = passTargetOffset(
          config_, type, ego.frenet.d,
          opponents[static_cast<std::size_t>(target_index)].frenet.d,
          required_gap);
    }
    unconstrained_pass_target_d = target_d;
    shift_distance = config_.prepare_distance_m;
  } else if (type == CandidateType::FOLLOW) {
    if (blocked_info.attack_follow_hold_pass_side &&
        std::isfinite(blocked_info.attack_follow_target_d_m)) {
      target_d =
          std::clamp(blocked_info.attack_follow_target_d_m, lower_d, upper_d);
      // 低速FOLLOWで毎周期merge_distanceから張り直すと、時間horizon内の
      // 横補正がほぼ0になり、PPがPASS側を保持できない。chain tailの安全楕円へ
      // 入るまでの実残距離を固定期限にして、再計画しても期限が逃げない形にする。
      shift_distance =
          std::max(1.0, config_.localized_avoidance_start_before_target_m);
    } else {
      target_d = blocked_info.start_grid_follow_hold_lateral ? start_grid_hold_d
                 : blocked_info.prestart_attack_follow_hold_lateral
                     ? prepared_hold_d
                 : (blocked_info.parallel_follow_hold_lateral ||
                    blocked_info.braking_follow_hold_lateral)
                     ? prepared_hold_d
                     : 0.0;
    }
  } else if (type == CandidateType::RECOVERY) {
    // 復帰ゲートが閉じている時は、通常ラインへ寄せず現在の安全コリドー内横位置を維持する。
    // 中心復帰を止めた直後にbase trajectoryへfall
    // throughしないための内部holdである。
    target_d =
        pass_reauthorization_recovery_profile
            ? std::clamp(blocked_info.pass_reauthorization_recovery_target_d_m,
                         lower_d, upper_d)
        : (blocked_info.reentry_hold_active ||
           blocked_info.start_grid_follow_hold_lateral ||
           blocked_info.early_stationary_parallel_pass_hold_lateral ||
           blocked_info.braking_follow_hold_lateral ||
           blocked_info.attack_follow_hold_pass_side)
            ? (blocked_info.start_grid_follow_hold_lateral ? start_grid_hold_d
                                                           : prepared_hold_d)
            : 0.0;
    shift_distance = recoveryShiftDistanceM(ego, blocked_info);
  } else if (type == CandidateType::SAFE_STOP &&
             (blocked_info.abort_safe_stop_hold_lateral ||
              blocked_info.start_grid_uncommitted_hold_active)) {
    // V2 ABORTの停止fallbackは、停止を理由に中心線へ横断しない。
    // 現在dが将来コリドー内で維持できない場合はSafetyEvaluator後の
    // current-d検証で候補不成立となり、停止constraintへ閉じる。
    target_d = std::clamp(ego.frenet.d, lower_d, upper_d);
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
    // 設計意図:
    // コーナー譲りでは壁余裕を優先して中心へ、それ以外は相手のdへ寄せて後方追従へ入る。
    const int target_index = yieldTargetIndex(blocked_info);
    const bool corner_yield = blocked_info.corner_side_by_side ||
                              blocked_info.future_corner_side_by_side ||
                              blocked_info.future_outer_wall_risk;
    target_d = std::clamp(config_.corner_yield_target_d_m, lower_d, upper_d);
    if (blocked_info.start_grid_uncommitted_hold_active) {
      // Gate 2未認可の開始追従を、YIELDという状態名だけで中心復帰へ昇格しない。
      // 現在d列はこの後も全相手・壁・CBFのSafetyEvaluatorへ通る。
      target_d = std::clamp(ego.frenet.d, lower_d, upper_d);
    } else if (blocked_info.parallel_yield_hold_lateral) {
      // CBFへ近づく前のstrict parallel YIELDは、相手dや中心線へ横切らない。
      // 全fresh相手に対するSafetyEvaluatorが通る現d保持のまま速度差を作る。
      target_d = std::clamp(ego.frenet.d, lower_d, upper_d);
    } else if (!corner_yield &&
               wallClearance(ego.frenet.s, ego.frenet.d) >=
                   config_.yield_rejoin_wall_clearance_m &&
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
        std::abs(ego.frenet.d) > config_.safe_stop_lateral_error_threshold_m;
    target_d = outside_safe_corridor || safe_stop_lateral_error_remaining
                   ? 0.0
                   : std::clamp(ego.frenet.d, lower_d, upper_d);
    target_d = std::clamp(target_d, lower_d, upper_d);
    shift_distance =
        std::max(config_.merge_distance_m, config_.prepare_distance_m);
  }
  const bool gentle_curve_safe_pass_constraint =
      (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      blocked_info.gentle_curve_safe_pass_constraint_active &&
      config_.gentle_curve_safe_pass_enabled &&
      std::isfinite(
          config_.gentle_curve_safe_pass_max_lateral_displacement_m) &&
      config_.gentle_curve_safe_pass_max_lateral_displacement_m > 0.0;
  const bool stationary_no_pass_safe_pass_constraint =
      (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      blocked_info.stationary_no_pass_safe_pass_constraint_active &&
      config_.stationary_no_pass_safe_pass_enabled &&
      std::isfinite(
          config_.stationary_no_pass_safe_pass_max_lateral_displacement_m) &&
      config_.stationary_no_pass_safe_pass_max_lateral_displacement_m > 0.0;
  double gentle_curve_min_d = -std::numeric_limits<double>::infinity();
  double gentle_curve_max_d = std::numeric_limits<double>::infinity();
  if (gentle_curve_safe_pass_constraint) {
    // 制限済みPASSは「中心線からの通常offset」ではなく、開始時dから許容した
    // 横移動量の中だけに収める。SafetyEvaluatorはこのd列そのものを評価する。
    const double max_displacement =
        config_.gentle_curve_safe_pass_max_lateral_displacement_m;
    const double anchor_d =
        std::isfinite(blocked_info.gentle_curve_safe_pass_anchor_d_m)
            ? blocked_info.gentle_curve_safe_pass_anchor_d_m
            : ego.frenet.d;
    gentle_curve_min_d = std::max(lower_d, anchor_d - max_displacement);
    gentle_curve_max_d = std::min(upper_d, anchor_d + max_displacement);
    const bool full_lateral_separation_reachable =
        std::isfinite(unconstrained_pass_target_d) &&
        unconstrained_pass_target_d >= gentle_curve_min_d - 1.0e-9 &&
        unconstrained_pass_target_d <= gentle_curve_max_d + 1.0e-9;
    if (!full_lateral_separation_reachable) {
      // horizon中にまだ相手へ追い付かないだけの短いclamp軌道を、完遂可能な
      // PASSとして認可しない。必要楕円分離へ届く目標dそのものが制約内にある
      // 場合だけ、後段のwall/全相手SafetyEvaluatorへ進める。
      candidate.pass_target_corridor_valid = false;
    }
    target_d = std::clamp(target_d, gentle_curve_min_d, gentle_curve_max_d);
    target_d = std::clamp(target_d, lower_d, upper_d);
  }
  double stationary_no_pass_min_d = -std::numeric_limits<double>::infinity();
  double stationary_no_pass_max_d = std::numeric_limits<double>::infinity();
  if (stationary_no_pass_safe_pass_constraint) {
    // 停止障害物の禁止区間PASSも、通常の外側offsetを無制限に使わず開始dからの
    // 横移動量を制約する。全d列をGate 2と同じ制約内に保つ。
    const double max_displacement =
        config_.stationary_no_pass_safe_pass_max_lateral_displacement_m;
    const double anchor_d =
        std::isfinite(blocked_info.stationary_no_pass_safe_pass_anchor_d_m)
            ? blocked_info.stationary_no_pass_safe_pass_anchor_d_m
            : ego.frenet.d;
    stationary_no_pass_min_d = std::max(lower_d, anchor_d - max_displacement);
    stationary_no_pass_max_d = std::min(upper_d, anchor_d + max_displacement);
    target_d = std::clamp(target_d, stationary_no_pass_min_d,
                          stationary_no_pass_max_d);
    target_d = std::clamp(target_d, lower_d, upper_d);
    // 停止車curve PASSは現在速度を優先するが、短い横遷移へ押し込まない。
    // active controllerの角度・rateと、設定済み横加速度上限を満たす最短距離へ
    // 既存bounded探索で延長する。不成立なら後段で候補をfail-closedに落とす。
    const double current_speed_request_mps = std::min(
        config_.stationary_no_pass_safe_pass_v_max_mps, std::max(0.0, ego.v));
    shift_distance = trackingLimitedLateralShiftDistanceM(
        ego, target_d, current_speed_request_mps, blocked_info, shift_distance);
  }
  const bool use_localized_profile =
      localized_profile != nullptr && localized_profile->active &&
      (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      localized_profile->pass_type == type &&
      (force_localized_pass_profile ||
       config_.overtake_lateral_profile_mode == "localized_latched") &&
      // gentle curve safe PASSでは、以前の通常PASS profileがより大きい目標dを
      // 保持している可能性がある。制限済みtarget_dだけでd列を作り、横移動上限を
      // SafetyEvaluatorとpublishで同一に保つ。
      (force_localized_pass_profile ||
       (!gentle_curve_safe_pass_constraint &&
        !stationary_no_pass_safe_pass_constraint));
  if ((type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      !use_localized_profile) {
    // moving targetは現在の空間位置より先へ進むため、固定8 mへ横移動を圧縮して
    // 操舵速度超過にする必要はない。相手速度と残留加速headroomの大きい方で
    // 追従可能な最短C2距離までだけ延長し、実際の時刻付き衝突可否は後段の
    // SafetyEvaluatorへ委ねる。停止対象は従来の空間回避期限を維持する。
    const int target_index = passTargetIndex(blocked_info, opponents);
    if (target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto &target = opponents[static_cast<std::size_t>(target_index)];
      const double moving_speed_threshold_mps =
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
      if (target.valid && std::isfinite(target.v) &&
          target.v > moving_speed_threshold_mps) {
        const double execution_speed_headroom_mps =
            std::max(0.0, ego.v) +
            config_.pass_assumed_accel_mps2 *
                config_.lateral_override_execution_speed_reserve_sec;
        const double required_tracking_speed_mps =
            std::max(target.v, execution_speed_headroom_mps);
        shift_distance = trackingLimitedLateralShiftDistanceM(
            ego, target_d, required_tracking_speed_mps, blocked_info,
            shift_distance);
      }
    }
  }
  if (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT ||
      attack_follow_profile) {
    candidate.planned_target_d_m =
        use_localized_profile ? localized_profile->target_d_m : target_d;
  }
  if ((type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      // 開始前は全PASSを検査し、局所profileはPASS継続中もraw形状の更新・
      // 現在d補正で回廊外にならないことを毎周期確認する。
      (blocked_info.pass_target_corridor_preflight_required ||
       use_localized_profile)) {
    // 評価horizonがprepare完了より短くても、目標dへ届くまでの全横移動が
    // s依存安全回廊内でなければPASSを始めない。短いhorizonだけ安全に見えて
    // その先で物理的に行き止まりになる開始をfail-closedで拒否する。
    const CorridorPreflightResult preflight =
        use_localized_profile
            ? localizedPassTargetCorridorReachable(*localized_profile, ego)
            : passTargetCorridorReachable(ego, target_d, shift_distance);
    candidate.pass_target_corridor_valid =
        candidate.pass_target_corridor_valid && preflight.valid;
    candidate.corridor_min_margin_m = preflight.min_margin_m;
  }

  // 処理ブロック: 候補種別ごとの速度上限を決める。
  // 設計意図:
  // 横参照だけでなく速度参照も同時に作り、MPCが危険な候補を速く走らないようにする。
  double speed_cap = config_.v_passthrough_mps;
  const double yield_min_speed_cap =
      finitePositiveOr(config_.yield_min_speed_cap_mps, 0.5);
  const double ego_wall_clearance = wallClearance(ego.frenet.s, ego.frenet.d);
  const int follow_target_index = followTargetIndex(blocked_info);
  const bool uncommitted_start_grid_follow_acceleration_allowed =
      blocked_info.start_grid_target_active &&
      !blocked_info.maneuver_transaction_incomplete;
  const bool stationary_prestart_current_d_stop_hold =
      prestart_attack_follow_profile && follow_target_index >= 0 &&
      static_cast<std::size_t>(follow_target_index) < opponents.size() &&
      opponents[static_cast<std::size_t>(follow_target_index)].valid &&
      std::isfinite(
          opponents[static_cast<std::size_t>(follow_target_index)].v) &&
      std::isfinite(
          opponents[static_cast<std::size_t>(follow_target_index)].vx) &&
      std::isfinite(
          opponents[static_cast<std::size_t>(follow_target_index)].vy) &&
      std::max(
          std::abs(opponents[static_cast<std::size_t>(follow_target_index)].v),
          std::hypot(
              opponents[static_cast<std::size_t>(follow_target_index)].vx,
              opponents[static_cast<std::size_t>(follow_target_index)].vy)) <=
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps) +
              1.0e-9;
  const bool pass_acceleration_allowed =
      purpose == CandidatePurpose::PROPOSAL_SAFETY_EVALUATION
          ? blocked_info.pass_proposal_acceleration_allowed
          : blocked_info.pass_acceleration_allowed;
  if (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) {
    speed_cap = finitePositiveOr(config_.pass_speed_cap_mps, ego.v);
    if (!pass_acceleration_allowed) {
      speed_cap = std::min(speed_cap, std::max(0.0, ego.v));
    }
    if (blocked_info.pass_lateral_first_speed_gate_active &&
        std::isfinite(blocked_info.pass_lateral_first_speed_cap_mps) &&
        blocked_info.pass_lateral_first_speed_cap_mps > 0.0) {
      // 横へ抜け切る前は、将来のPASS形状が安全でも実車の横追従遅れを理由に
      // 相手へ縦方向に詰めない。相手速度/停止creepからCoreが求めた上限を
      // ここで最終capし、下のLongitudinalProfileとpublish v_refへ同じ値を渡す。
      speed_cap =
          std::min(speed_cap, blocked_info.pass_lateral_first_speed_cap_mps);
    }
  } else if (type == CandidateType::FOLLOW && follow_target_index >= 0 &&
             static_cast<std::size_t>(follow_target_index) < opponents.size()) {
    // 通常FOLLOWは前走車基準、PASS取引中の攻めFOLLOWは同一chain tailへ
    // 追い付ける範囲だけbounded bonusを加える。加速を許す既存freshness/MPC/
    // target-ID gateがfalseなら従来capのままで、後段のLongitudinalProfileと
    // SafetyEvaluatorが同じ加速s(t)を評価する。
    const auto &opp = opponents[static_cast<std::size_t>(follow_target_index)];
    speed_cap = std::max(0.5, opp.v - config_.follow_speed_margin_mps);
    if (stationary_prestart_current_d_stop_hold) {
      // PASS未commitのcurrent-d FOLLOWで停止targetへ最低0.5 m/sを与えると、
      // PPに必要な空間horizonを作るため同じdのまま安全楕円へ進む。横authorityを
      // 証明できない短距離では、target IDを保持したSTOP_HOLDとして速度0へ閉じ、
      // Coreが同じtargetの左右PASSを次周期も再評価する。moving targetと
      // commit済みPASS-side ATTACK_FOLLOWのcreep契約はこの分岐へ入れない。
      speed_cap = 0.0;
    } else if (uncommitted_start_grid_follow_acceleration_allowed) {
      // 相手速度と同等を基準に、目標車間より離れている量だけbounded bonusを
      // 加える。固定3 m/sで4.5 m先の低速車へ詰めず、候補のs(t)を同じ
      // SafetyEvaluatorへ渡したうえで上限3 m/sまで攻め追従する。
      const double gap_error_m =
          std::isfinite(blocked_info.start_grid_target_delta_s)
              ? std::max(0.0, blocked_info.start_grid_target_delta_s -
                                  config_.follow_gap_closing_target_gap_m)
              : 0.0;
      // レース追従なので、対象が目標車間より近くても同速で離されないよう
      // 1 m分だけを最小攻めbonusとする。実行可否は固定距離判定ではなく、
      // この加速を含むs(t)を評価したSafetyEvaluatorが最終決定する。
      const double attack_distance_m = std::max(1.0, gap_error_m);
      const double speed_bonus_mps = std::clamp(
          attack_distance_m * config_.follow_gap_closing_speed_gain_per_m, 0.0,
          config_.follow_gap_closing_max_speed_bonus_mps);
      speed_cap =
          std::min(finitePositiveOr(config_.start_grid_attack_follow_v_max_mps,
                                    std::max(0.5, opp.v)),
                   std::max(0.5, opp.v) + speed_bonus_mps);
    }
    if (!blocked_info.start_grid_target_active &&
        blocked_info.follow_gap_closing_allowed &&
        std::isfinite(blocked_info.front_delta_s)) {
      const double gap_error_m =
          std::max(0.0, blocked_info.front_delta_s -
                            config_.follow_gap_closing_target_gap_m);
      const double speed_bonus_mps =
          std::clamp(gap_error_m * config_.follow_gap_closing_speed_gain_per_m,
                     0.0, config_.follow_gap_closing_max_speed_bonus_mps);
      // 攻めFOLLOW中は相手速度を基準にする。従来は通常FOLLOWのmarginを
      // 引いた値へbonusを足していたため、目標gap付近では相手より遅いまま
      // になり、PASS開始を待つほど離された。加速を含む同じs(t)は下の
      // LongitudinalProfileとSafetyEvaluatorへ渡すので、安全判定は迂回しない。
      speed_cap = std::max(0.5, opp.v) + speed_bonus_mps;
    }
    if (blocked_info.attack_follow_acceleration_allowed &&
        blocked_info.attack_follow_hold_pass_side &&
        std::isfinite(blocked_info.maneuver_chain_tail_relative_s_m)) {
      const double gap_error_m =
          std::max(0.0, blocked_info.maneuver_chain_tail_relative_s_m -
                            config_.follow_gap_closing_target_gap_m);
      const double speed_bonus_mps =
          std::clamp(gap_error_m * config_.follow_gap_closing_speed_gain_per_m,
                     0.0, config_.follow_gap_closing_max_speed_bonus_mps);
      speed_cap += speed_bonus_mps;
      speed_cap = std::min(
          speed_cap, finitePositiveOr(config_.pass_speed_cap_mps, speed_cap));
    }
    // 制動FOLLOWは攻めbonusより常に強い。bonus適用後に最後にcapしないと、
    // TTC/制動距離で求めた物理上限を再び持ち上げてしまう。
    if (blocked_info.braking_follow_active &&
        std::isfinite(blocked_info.braking_follow_speed_cap_mps) &&
        blocked_info.braking_follow_speed_cap_mps > 0.0) {
      speed_cap =
          std::min(speed_cap, blocked_info.braking_follow_speed_cap_mps);
    }
    const bool committed_attack_follow_speed_gate_context =
        attack_follow_profile && blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_target_latched;
    if (committed_attack_follow_speed_gate_context &&
        blocked_info.pass_lateral_first_speed_gate_active &&
        !blocked_info.pass_lateral_clearance_ready) {
      const bool stationary_threshold_valid =
          std::isfinite(config_.stationary_obstacle_speed_threshold_mps) &&
          config_.stationary_obstacle_speed_threshold_mps >= 0.0;
      const bool target_speed_finite =
          std::isfinite(blocked_info.pass_lateral_first_target_speed_mps);
      const bool stationary_target =
          stationary_threshold_valid && target_speed_finite &&
          blocked_info.pass_lateral_first_target_speed_mps <=
              config_.stationary_obstacle_speed_threshold_mps;
      const bool target_identity_matches =
          !blocked_info.maneuver_target_id.empty() &&
          blocked_info.pass_lateral_first_target_id ==
              blocked_info.maneuver_target_id;
      // Coreが停止対象creepへ使う絶対上限と同じ1 m/sに閉じる。activeな
      // committed contextでstationary入力または入力自体が壊れている場合、
      // ID/cap不整合を攻めFOLLOW速度へ暗黙fallbackさせず停止targetへ閉じる。
      constexpr double kMaxStationaryLateralFirstCapMps = 1.0;
      const bool cap_valid =
          std::isfinite(blocked_info.pass_lateral_first_speed_cap_mps) &&
          blocked_info.pass_lateral_first_speed_cap_mps > 0.0 &&
          blocked_info.pass_lateral_first_speed_cap_mps <=
              kMaxStationaryLateralFirstCapMps + 1.0e-9;
      if (stationary_target && target_identity_matches && cap_valid) {
        // bonusと制動capの双方より後で最終適用し、生成したs(t)/v_refをcopyする
        // inward connectorにも同じ縦契約をそのまま継承させる。
        speed_cap =
            std::min(speed_cap, blocked_info.pass_lateral_first_speed_cap_mps);
      } else if (!target_speed_finite || !stationary_threshold_valid ||
                 (stationary_target &&
                  (!target_identity_matches || !cap_valid))) {
        speed_cap = 0.0;
      }
    }
  } else if (type == CandidateType::RECOVERY) {
    speed_cap = ego_wall_clearance < 0.0
                    ? config_.wall_margin_recovery_v_max_mps
                    : config_.recovery_v_max_mps;
    if (blocked_info.reentry_hold_active ||
        blocked_info.start_grid_follow_hold_lateral ||
        blocked_info.early_stationary_parallel_pass_hold_lateral ||
        blocked_info.braking_follow_hold_lateral) {
      const double configured_hold_cap_mps =
          std::isfinite(blocked_info.reentry_hold_speed_cap_mps) &&
                  blocked_info.reentry_hold_speed_cap_mps > 0.0
              ? blocked_info.reentry_hold_speed_cap_mps
              : config_.reentry_hold_v_max_mps;
      speed_cap = std::min(
          speed_cap,
          finitePositiveOr(configured_hold_cap_mps,
                           config_.opponent_collision_fallback_v_max_mps));
    }
    if (blocked_info.braking_follow_hold_lateral &&
        std::isfinite(blocked_info.braking_follow_speed_cap_mps) &&
        blocked_info.braking_follow_speed_cap_mps > 0.0) {
      speed_cap =
          std::min(speed_cap, blocked_info.braking_follow_speed_cap_mps);
    }
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
    if (blocked_info.stationary_front_obstacle) {
      // 停止障害物へ譲る時は、通常の高速yield下限を使わず確実に減速要求を出す。
      speed_cap = std::min(std::max(0.0, ego.v),
                           std::max(std::max(1.0e-3, config_.safe_stop_v_mps),
                                    config_.yield_speed_margin_mps));
    }
    if (blocked_info.braking_follow_hold_lateral &&
        std::isfinite(blocked_info.braking_follow_speed_cap_mps) &&
        blocked_info.braking_follow_speed_cap_mps > 0.0) {
      // PASS側で停止chain tailへ接近した一時holdは、横位置を保つだけでなく
      // 同周期に算出した制動capも使う。現速度維持のまま次周期を待たない。
      speed_cap =
          std::min(speed_cap, blocked_info.braking_follow_speed_cap_mps);
    }
  } else if (type == CandidateType::SAFE_STOP) {
    speed_cap = std::max(1.0e-3, config_.safe_stop_v_mps);
  }
  if (gentle_curve_safe_pass_constraint &&
      std::isfinite(blocked_info.gentle_curve_safe_pass_speed_cap_mps) &&
      blocked_info.gentle_curve_safe_pass_speed_cap_mps > 0.0) {
    // Coreが曲率と横加速度上限から算出し、PASS中もラッチした上限を使う。
    // configの固定上限だけへ戻すと、曲率が上がった次周期に横加速度制限を
    // 破るため、NaNなら候補はCore側で開始不可として扱う。
    speed_cap =
        std::min(speed_cap, blocked_info.gentle_curve_safe_pass_speed_cap_mps);
  }
  if (stationary_no_pass_safe_pass_constraint &&
      std::isfinite(blocked_info.stationary_no_pass_safe_pass_speed_cap_mps) &&
      blocked_info.stationary_no_pass_safe_pass_speed_cap_mps > 0.0) {
    speed_cap = std::min(
        speed_cap, blocked_info.stationary_no_pass_safe_pass_speed_cap_mps);
  }

  if ((type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) &&
      use_localized_profile) {
    const int target_index = passTargetIndex(blocked_info, opponents);
    if (target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto &target = opponents[static_cast<std::size_t>(target_index)];
      const double target_relative_s_m =
          blocked_info.maneuver_target_latched &&
                  std::isfinite(blocked_info.maneuver_target_relative_s_m)
              ? blocked_info.maneuver_target_relative_s_m
              : frame_.deltaS(ego.frenet.s, target.frenet.s);
      const bool existing_profile_authorized =
          localized_profile != nullptr &&
          (localized_profile->pass_safety_approved_once ||
           localized_profile->pass_execution_committed ||
           localized_profile->pass_complete_confirmed);
      const bool requires_new_lateral_transition =
          !existing_profile_authorized &&
          !blocked_info.maneuver_transaction_incomplete &&
          target_relative_s_m > 0.0 &&
          std::abs(target_d - ego.frenet.d) > 1.0e-6;
      const bool has_profile_transition_provenance =
          localized_profile != nullptr &&
          std::isfinite(localized_profile->avoid_start_before_target_m) &&
          std::isfinite(localized_profile->full_offset_before_target_m);
      auto &deadline_diagnostic = candidate.pass_transition_deadline;
      deadline_diagnostic.requires_new_lateral_transition =
          requires_new_lateral_transition;
      deadline_diagnostic.lateral_shift_m = std::abs(target_d - ego.frenet.d);
      deadline_diagnostic.deadline_speed_cap_mps = speed_cap;
      const auto record_transition_diagnostic =
          [&](const PassTransitionFeasibility &transition,
              PassTransitionDeadlineSource source, double transition_start_s_m,
              double transition_end_s_m) {
            deadline_diagnostic.evaluated = true;
            deadline_diagnostic.input_valid = transition.input_valid;
            deadline_diagnostic.reachable = transition.reachable;
            deadline_diagnostic.source = source;
            deadline_diagnostic.lateral_shift_m = transition.lateral_shift_m;
            deadline_diagnostic.transition_start_s_m = transition_start_s_m;
            deadline_diagnostic.transition_end_s_m = transition_end_s_m;
            deadline_diagnostic.required_transition_m =
                transition.required_transition_m;
            deadline_diagnostic.available_deadline_m =
                transition.available_deadline_m;
            deadline_diagnostic.deadline_slack_m =
                transition.available_deadline_m -
                transition.required_transition_m;
            deadline_diagnostic.evaluated_tracking_speed_mps =
                transition.evaluated_tracking_speed_mps;
          };
      if (force_localized_pass_profile && has_profile_transition_provenance &&
          requires_new_lateral_transition) {
        // V2 profileはbuild時にactual ego poseからPPの最小横移動距離で作る。
        // target現在sではなく、そのprofileがfull offsetへ到達する実際の終端を
        // reachabilityのdeadlineにする。相手との時系列衝突はここで推測せず、
        // 生成後の全点をSafetyEvaluatorへ必ず渡す。
        const double current_unwrapped_s = profileUnwrappedS(
            *localized_profile, ego.frenet.s, frame_.length());
        const double available_profile_transition_m =
            localized_profile->full_offset_start_s_m - current_unwrapped_s;
        const auto transition =
            passTransitionFeasibility(ego, target_d, speed_cap, blocked_info,
                                      available_profile_transition_m);
        record_transition_diagnostic(
            transition, PassTransitionDeadlineSource::PROFILE_MARKER,
            current_unwrapped_s, localized_profile->full_offset_start_s_m);
        candidate.required_pass_transition_m = transition.required_transition_m;
        candidate.available_pass_transition_deadline_m =
            transition.available_deadline_m;
        candidate.pass_transition_deadline_reachable = transition.reachable;
        if (!transition.reachable) {
          // A malformed/provenance-mismatched V2 profile must not turn the
          // post-target marker rule into an authority bypass.
          candidate.pass_target_corridor_valid = false;
          candidate.reject_reason = "pass_profile_transition_untrackable";
        }
      } else {
        const double available_deadline_m =
            target_relative_s_m - longitudinalEllipseClearance(config_);
        const bool profile_would_be_compressed =
            target_relative_s_m <
            std::max(1.0, config_.localized_avoidance_start_before_target_m);
        if (requires_new_lateral_transition && profile_would_be_compressed) {
          const auto transition = passTransitionFeasibility(
              ego, target_d, speed_cap, blocked_info, available_deadline_m);
          record_transition_diagnostic(
              transition, PassTransitionDeadlineSource::TARGET_CLEARANCE,
              ego.frenet.s, ego.frenet.s + available_deadline_m);
          candidate.required_pass_transition_m =
              transition.required_transition_m;
          candidate.available_pass_transition_deadline_m =
              transition.available_deadline_m;
          candidate.pass_transition_deadline_reachable = transition.reachable;
          if (!transition.reachable) {
            // Do not shorten a new C2 transition to fit the remaining gap.
            // An already committed transaction is re-evaluated using its
            // retained profile, and a current-d PASS requires no lateral
            // transition.  The Core will otherwise retain the same target's
            // existing FOLLOW/current-d fallback.
            candidate.pass_target_corridor_valid = false;
            candidate.reject_reason = "pass_transition_deadline_unreachable";
          }
        }
      }
    }
  }

  // PASSはSafetyEvaluatorへ渡す前に、実際のC2横profileと基準CSVを合成した
  // 操舵速度から実行可能な速度上限を求める。単にtracking=falseへ落とすだけでは
  // 安全な低速PASSまで失うため、元の速度cap以下で成立する最大値を二分探索する。
  // ただしmoving
  // targetより遅い上限しか得られない候補は追越として完遂不能なので、
  // safe-cycleを進めず攻めFOLLOW/通常FOLLOWへ戻す。
  bool pass_controller_geometry_valid = true;
  if (type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT) {
    const double execution_speed_headroom_mps =
        std::max(0.0, ego.v) +
        config_.pass_assumed_accel_mps2 *
            config_.lateral_override_execution_speed_reserve_sec;
    const auto trackable_at_speed = [&](double requested_speed_mps) {
      if (!std::isfinite(requested_speed_mps) || requested_speed_mps < 0.0 ||
          !std::isfinite(execution_speed_headroom_mps) ||
          execution_speed_headroom_mps < 0.0) {
        return false;
      }
      const double evaluated_speed_mps =
          std::max(requested_speed_mps, execution_speed_headroom_mps);
      if (!use_localized_profile) {
        CandidateTrajectory probe;
        probe.v_ref = {evaluated_speed_mps};
        probe.predicted_speed_mps = {evaluated_speed_mps};
        const double lateral_delta_m = target_d - ego.frenet.d;
        // 最終候補はactive PPが要求するlookahead arcまでpublishして、その全点を
        // SafetyEvaluatorと追従性判定へ渡す。横移動の終了点だけで速度を決めると、
        // その先のoffset
        // reference曲率で最終判定だけが落ち、候補生成時の速度capと
        // 認可結果が食い違う。速度二分探索でも同じ前方arcを先に検査する。
        const double controller_check_distance_m = std::max(
            shift_distance,
            config_.lateral_override_lookahead_gain * evaluated_speed_mps +
                config_.lateral_override_lookahead_min_distance_m);
        const auto sampled = sampledLateralProfileTrackability(
            probe, ego, controller_check_distance_m, [&](double distance_m) {
              const double z =
                  std::clamp(distance_m / shift_distance, 0.0, 1.0);
              return ego.frenet.d + lateral_delta_m * smootherstep(z);
            });
        return sampled.valid() &&
               nominalLateralCorrectionPathTrackable(
                   ego, target_d, shift_distance, evaluated_speed_mps);
      }
      CandidateTrajectory probe;
      probe.d = {ego.frenet.d};
      probe.longitudinal_offsets_m = {0.0};
      probe.v_ref = {evaluated_speed_mps};
      probe.predicted_speed_mps = {evaluated_speed_mps};
      probe.required_controller_spatial_horizon_m = 0.0;
      return localizedPassProfileTrackability(*localized_profile, probe, ego)
          .valid();
    };

    if (!std::isfinite(speed_cap) || speed_cap < 0.0 ||
        !trackable_at_speed(0.0)) {
      pass_controller_geometry_valid = false;
    } else if (!trackable_at_speed(speed_cap)) {
      double lower_speed_mps = 0.0;
      double upper_speed_mps = speed_cap;
      for (int iteration = 0; iteration < 40; ++iteration) {
        const double middle_speed_mps =
            0.5 * (lower_speed_mps + upper_speed_mps);
        if (trackable_at_speed(middle_speed_mps)) {
          lower_speed_mps = middle_speed_mps;
        } else {
          upper_speed_mps = middle_speed_mps;
        }
      }
      speed_cap = lower_speed_mps * kTrackabilitySpeedSearchReserveRatio;
    }
  }

  // 処理ブロック: 復帰系候補に追加の速度ガードを重ねる。
  // 設計意図:
  // 横位置が危ない時は横軌道だけでなく速度も抑え、MPCが無理な回避を解かないようにする。
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
  if (start_grid_anchored_hold_profile) {
    // 通常merge距離で毎周期張り直すと、horizon先端がanchorへ届かず実dの
    // driftを追認してしまう。専用期限を使うが、操舵角/操舵速度制約から
    // 必要な距離より短くはしない。
    shift_distance = trackingLimitedLateralShiftDistanceM(
        ego, start_grid_hold_d, speed_cap, blocked_info,
        config_.start_grid_hold_correction_distance_m);
    const double corridor_check_distance_m =
        std::isfinite(shift_distance)
            ? shift_distance
            : std::max(1.0, config_.start_grid_hold_correction_distance_m);
    const CorridorPreflightResult hold_preflight =
        attackFollowTargetCorridorReachable(ego, start_grid_hold_d,
                                            corridor_check_distance_m);
    candidate.pass_target_corridor_valid = hold_preflight.valid;
    candidate.corridor_min_margin_m = hold_preflight.min_margin_m;
  }
  if (pass_reauthorization_recovery_profile) {
    // stop保持中はtarget==current-dとなり、解除後だけ認可済みPASS包絡の
    // nearest boundへ低速で戻す。どちらも通常PASSと同じcontroller幾何・
    // s依存corridorを先に検証し、clampされた見かけ上の復帰を許可しない。
    shift_distance = trackingLimitedLateralShiftDistanceM(
        ego, target_d, speed_cap, blocked_info, config_.merge_distance_m);
    const CorridorPreflightResult recovery_preflight =
        attackFollowTargetCorridorReachable(ego, target_d, shift_distance);
    candidate.pass_target_corridor_valid = recovery_preflight.valid;
    candidate.corridor_min_margin_m = recovery_preflight.min_margin_m;
  }
  if (attack_follow_profile) {
    // 操舵速度制約は、この周期に下流へ許可し得る速度capで評価する。固定の
    // PASS最高速を使うと、低速FOLLOWまで不必要に長い横補正へしてしまう。
    shift_distance =
        attackFollowShiftDistanceM(ego, target_d, speed_cap, blocked_info);
    // 元PASS profileの先まで含む回廊と、現在dから追従可能な距離で
    // つなぎ直す区間を両方検査する。
    CorridorPreflightResult correction_preflight =
        attackFollowTargetCorridorReachable(ego, target_d, shift_distance);
    bool committed_transaction_corridor_valid = true;
    double committed_transaction_corridor_min_margin_m =
        std::numeric_limits<double>::infinity();
    if (localized_profile != nullptr && localized_profile->active &&
        localized_profile->pass_type ==
            blocked_info.maneuver_transaction_pass_type &&
        config_.overtake_lateral_profile_mode == "localized_latched") {
      const CorridorPreflightResult transaction_preflight =
          localizedPassTargetCorridorReachable(*localized_profile, ego);
      committed_transaction_corridor_valid = transaction_preflight.valid;
      committed_transaction_corridor_min_margin_m =
          transaction_preflight.min_margin_m;
    }
    if (!correction_preflight.valid || !committed_transaction_corridor_valid) {
      // 元PASS目標へ直接戻すprofileが狭い回廊を通れない周期は、その危険な
      // d列を緩和して許可しない。transactionのtarget ID/side/target dは
      // ラッチしたまま、現在のPASS側dを保持する別軌道を作り直す。
      // このholdも下で操舵契約と全相手SafetyEvaluatorを通らなければ不成立。
      target_d = std::clamp(ego.frenet.d, lower_d, upper_d);
      shift_distance =
          attackFollowShiftDistanceM(ego, target_d, speed_cap, blocked_info);
      correction_preflight =
          attackFollowTargetCorridorReachable(ego, target_d, shift_distance);
      candidate.attack_follow_safe_lateral_hold = true;
      candidate.planned_target_d_m = target_d;
    }
    candidate.pass_target_corridor_valid = correction_preflight.valid;
    candidate.corridor_min_margin_m = correction_preflight.min_margin_m;
    if (!candidate.attack_follow_safe_lateral_hold) {
      candidate.pass_target_corridor_valid =
          candidate.pass_target_corridor_valid &&
          committed_transaction_corridor_valid;
      candidate.corridor_min_margin_m =
          std::min(candidate.corridor_min_margin_m,
                   committed_transaction_corridor_min_margin_m);
    }
  }
  const bool acceleration_allowed =
      type == CandidateType::FOLLOW
          ? (!stationary_prestart_current_d_stop_hold &&
             (blocked_info.follow_gap_closing_allowed ||
              uncommitted_start_grid_follow_acceleration_allowed ||
              blocked_info.attack_follow_acceleration_allowed))
          : ((type == CandidateType::PASS_LEFT ||
              type == CandidateType::PASS_RIGHT) &&
             pass_acceleration_allowed);
  // 実ログでPPの空間horizon不足を確認したPASSと、その横位置を維持する
  // 攻めFOLLOWへ実行速度reserveを適用する。RECOVERY/YIELD/SAFE_STOPは
  // 停止可能距離を含む別の物理契約移行が必要なため、未検証の一括変更をしない。
  const bool stop_hold_authority_context =
      blocked_info.maneuver_transaction_safe_lateral_hold_active ||
      blocked_info.maneuver_transaction_tracking_stop_active ||
      blocked_info.abort_safe_stop_hold_lateral ||
      blocked_info.reentry_hold_active ||
      blocked_info.post_abort_curve_hold_active;
  // 通常FOLLOW/YIELD/RECOVERYを20秒評価へ広げない。CoreがSTOP中の横authorityを
  // 得うる文脈だけをPP空間proof契約へ入れる。SAFE_STOPの通常候補選択は
  // 変えず、Coreが横authority直前に立てる専用flagでだけ再評価する。
  const bool stop_hold_lateral_profile =
      stop_hold_authority_context &&
      (type == CandidateType::SAFE_STOP || type == CandidateType::FOLLOW ||
       type == CandidateType::YIELD_BEHIND || type == CandidateType::RECOVERY);
  const bool controller_lateral_profile =
      type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT ||
      (type == CandidateType::RECOVERY &&
       blocked_info.early_wall_recovery_probe_requested) ||
      attack_follow_profile || prestart_attack_follow_profile ||
      start_grid_anchored_hold_profile ||
      start_grid_uncommitted_current_d_candidate ||
      pass_reauthorization_recovery_profile || stop_hold_lateral_profile;
  EgoState longitudinal_profile_ego = ego;
  if (prepared_profile_hold &&
      longitudinal_profile_ego.v < kPreparedHoldStationarySpeedUpperBoundMps) {
    // start-grid候補のcurrent-d holdは、初回認可の前後とも停止ノイズだけで
    // ds列とplan generationを毎周期変えない。0へ丸めて前進量を過小評価せず、
    // 2 cm/sの保守上限をSafetyEvaluatorにも同じまま渡す。
    longitudinal_profile_ego.v = kPreparedHoldStationarySpeedUpperBoundMps;
  }
  const LongitudinalProfile longitudinal_profile =
      makeLongitudinalProfile(type, longitudinal_profile_ego, speed_cap,
                              acceleration_allowed, controller_lateral_profile);
  // 下流へ渡す速度capとSafetyEvaluatorのs(t)を同じ契約にする。加速が未認可の
  // FOLLOW/YIELD/PASSで元のspeed_capを出すと、評価していない前進量をMPC/PPへ
  // 許すため、縦profileが実際に採用したtarget速度へ必ず閉じる。
  speed_cap = longitudinal_profile.target_speed_mps;
  double evaluation_dt_sec = config_.horizon_dt_sec;
  double required_controller_spatial_horizon_m = 0.0;
  if (controller_lateral_profile && config_.horizon_points > 1U) {
    // primary Pure Pursuitは、現在速度/速度capから得るlookaheadに加えて、
    // 0.75秒の前方時間と、0.25秒の応答遅れ後に最大1.0 m/s^2で
    // fail-safe速度まで落とせる距離を要求する。Plannerも同じかより保守的な
    // 契約で時刻軸を伸ばし、その全点をSafetyEvaluatorへ通す。
    const double current_speed_mps = std::max(0.0, ego.v);
    // makeLongitudinalProfileが実行遅れ中の残留加速を初期速度headroomへ含める。
    // target capを下げた周期もこの値を使い、PPだけが次callbackで必要arc不足へ
    // 落ちないようにする。長い空間列は同じSafetyEvaluatorへ渡す。
    const double execution_speed_mps =
        std::max(current_speed_mps, longitudinal_profile.speedAt(0.0));
    const double controller_target_speed_mps =
        std::max(execution_speed_mps, std::max(0.0, speed_cap));
    required_controller_spatial_horizon_m = requiredControllerSpatialHorizon(
        execution_speed_mps, controller_target_speed_mps,
        attack_follow_profile);

    const double default_horizon_sec =
        static_cast<double>(config_.horizon_points - 1U) * evaluation_dt_sec;
    const bool all_observed_opponents_stationary =
        blocked_info.opponent_prediction_inputs_complete &&
        !opponents.empty() &&
        std::all_of(
            opponents.begin(), opponents.end(), [this](const auto &opp) {
              return opp.valid && std::isfinite(opp.v) &&
                     std::isfinite(opp.vx) && std::isfinite(opp.vy) &&
                     std::max(std::abs(opp.v), std::hypot(opp.vx, opp.vy)) <=
                         config_.stationary_obstacle_speed_threshold_mps +
                             1.0e-9;
            });
    // 長いCartesian等速予測は静止相手なら位置不変である。moving/欠損相手を
    // 20秒まで直線外挿せず、従来の短い上限で必要arc未達をfail-closedにする。
    const double context_max_horizon_sec =
        all_observed_opponents_stationary
            ? config_.lateral_override_max_evaluation_horizon_sec
            : config_.moving_lateral_override_max_evaluation_horizon_sec;
    const double configured_max_horizon_sec =
        attack_follow_profile
            ? std::max(config_.attack_follow_max_evaluation_horizon_sec,
                       context_max_horizon_sec)
            : context_max_horizon_sec;
    const double max_horizon_sec =
        std::max(default_horizon_sec, configured_max_horizon_sec);
    if (longitudinal_profile.distanceAt(default_horizon_sec) + 1.0e-9 <
        required_controller_spatial_horizon_m) {
      // 同じ縦profileを長い時刻までSafetyEvaluatorへ渡す。PPだけへ架空の
      // 空間点を追加せず、相手予測との時刻整合を維持する。
      if (longitudinal_profile.distanceAt(max_horizon_sec) + 1.0e-9 >=
          required_controller_spatial_horizon_m) {
        double lower_sec = default_horizon_sec;
        double upper_sec = max_horizon_sec;
        for (int iteration = 0; iteration < 32; ++iteration) {
          const double middle_sec = 0.5 * (lower_sec + upper_sec);
          if (longitudinal_profile.distanceAt(middle_sec) >=
              required_controller_spatial_horizon_m) {
            upper_sec = middle_sec;
          } else {
            lower_sec = middle_sec;
          }
        }
        evaluation_dt_sec =
            upper_sec / static_cast<double>(config_.horizon_points - 1U);
      } else {
        // 上限時刻でも届かない場合は、評価範囲を上限まで伸ばしたうえで
        // controller_tracking_profile_valid=falseへ閉じる。
        evaluation_dt_sec =
            max_horizon_sec / static_cast<double>(config_.horizon_points - 1U);
      }
    }
  }
  candidate.longitudinal_profile_valid = longitudinal_profile.valid;
  candidate.required_controller_spatial_horizon_m =
      required_controller_spatial_horizon_m;
  candidate.assumed_brake_decel_mps2 = longitudinal_profile.brake_decel_mps2;
  candidate.response_delay_sec = longitudinal_profile.response_delay_sec;
  candidate.required_brake_distance_m =
      longitudinal_profile.requiredDistanceTo(speed_cap);
  if (blocked_info.braking_follow_active &&
      std::isfinite(blocked_info.braking_follow_delta_s)) {
    candidate.available_brake_distance_m = std::max(
        0.0, blocked_info.braking_follow_delta_s - config_.safety_ellipse_a_m);
  } else if (blocked_info.stationary_front_obstacle &&
             std::isfinite(blocked_info.front_delta_s)) {
    candidate.available_brake_distance_m =
        std::max(0.0, blocked_info.front_delta_s - config_.safety_ellipse_a_m);
  }

  // 処理ブロック: 時間horizonごとのs,d,x,y,yaw,v_refを生成する。
  // 設計意図:
  // MPC入力は固定長配列なので、候補評価とpublishで同じhorizon列を再利用できる形にする。
  for (std::size_t i = 0; i < config_.horizon_points; ++i) {
    // 候補ごとの速度想定でs列を作り、smoothstepで横方向を急変させない。
    const double t = static_cast<double>(i) * evaluation_dt_sec;
    const double ds = longitudinal_profile.distanceAt(t);
    const double s = frame_.wrapS(ego.frenet.s + ds);
    // SafetyEvaluatorのt=0と実車位置を一致させる。start-grid中は停止settlingが
    // 2 mm未満でもprepared profileへsnapせず、実測dから固定anchorへつなぐ。
    const bool start_grid_measured_start =
        blocked_info.start_grid_target_active &&
        (type == CandidateType::FOLLOW || type == CandidateType::RECOVERY);
    double start_d =
        start_grid_measured_start
            ? ego.frenet.d
            : (prepared_profile_hold ? prepared_hold_d : ego.frenet.d);
    if (type == CandidateType::RECOVERY ||
        type == CandidateType::YIELD_BEHIND ||
        type == CandidateType::SAFE_STOP) {
      start_d = std::clamp(start_d, lower_d, upper_d);
    }
    // 横profileは物理前進距離だけで生成する。停止近傍で時間比率を使うと、
    // 極小dsへ大きな横移動を圧縮して曲率を過大化するため、前進距離が
    // 足りない周期は現在d付近を保持し、速度制約の下で次周期へ継続する。
    const double normalized_shift = ds / std::max(1.0, shift_distance);
    const bool pass_profile =
        type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
    const double ratio = (pass_profile || attack_follow_profile ||
                          start_grid_anchored_hold_profile ||
                          pass_reauthorization_recovery_profile)
                             ? smootherstep(normalized_shift)
                             : smoothstep(normalized_shift);
    double d = use_localized_profile
                   ? localizedProfileD(*localized_profile, ego, s, ds)
                   : start_d + (target_d - start_d) * ratio;
    if (gentle_curve_safe_pass_constraint) {
      // 自車dが周期間に動いても、制限PASSのd列全体を開始anchorの範囲内へ
      // 留める。targetだけのclampでは先頭点が累積移動し得るためである。
      d = std::clamp(d, gentle_curve_min_d, gentle_curve_max_d);
    }
    if (stationary_no_pass_safe_pass_constraint) {
      d = std::clamp(d, stationary_no_pass_min_d, stationary_no_pass_max_d);
    }
    const auto point_bounds =
        frame_.corridorBounds(s, config_.d_min_m, config_.d_max_m);
    if (type != CandidateType::PASS_LEFT && type != CandidateType::PASS_RIGHT) {
      const double point_lower_d_m =
          point_bounds.d_min + config_.min_wall_margin_m;
      const double point_upper_d_m =
          point_bounds.d_max - config_.min_wall_margin_m;
      if (start_grid_uncommitted_current_d_candidate) {
        // 未commit例外は「現在dを保持する」契約であり、将来corridor clampを
        // 暗黙の横操舵へ変換しない。全点を実測dのままSafetyEvaluatorへ渡し、
        // 1点でも回廊外または非current-dなら候補不成立へ閉じる。
        start_grid_uncommitted_future_corridor_valid =
            start_grid_uncommitted_future_corridor_valid &&
            std::isfinite(point_lower_d_m) && std::isfinite(point_upper_d_m) &&
            point_lower_d_m <= point_upper_d_m && std::isfinite(ego.frenet.d) &&
            ego.frenet.d >= point_lower_d_m &&
            ego.frenet.d <= point_upper_d_m &&
            std::abs(d - ego.frenet.d) <= 1.0e-6;
        d = ego.frenet.d;
      } else {
        d = std::clamp(d, point_lower_d_m, point_upper_d_m);
      }
    }
    const auto p = frame_.frenetToCartesian(s, d);
    candidate.t.push_back(t);
    candidate.longitudinal_offsets_m.push_back(ds);
    candidate.s.push_back(s);
    candidate.d.push_back(d);
    candidate.x.push_back(p.x);
    candidate.y.push_back(p.y);
    candidate.yaw.push_back(p.yaw);
    // s(t) は応答遅れと制動上限を含む最遠到達可能性で安全評価する。一方で
    // 下流MPC/PPは各周期にv_ref[0]を目標速度へ直接使うため、予測速度をv_refへ
    // 出すと遅れが毎周期リセットされる。安全予測は保持したまま、速度capは即時に
    // 要求する。実制動が予測より早ければ保守側となる。
    candidate.predicted_speed_mps.push_back(longitudinal_profile.speedAt(t));
    candidate.v_ref.push_back(speed_cap);
  }

  if (type == CandidateType::RECOVERY &&
      blocked_info.early_wall_recovery_probe_requested &&
      !candidate.x.empty() && !candidate.y.empty() && !candidate.yaw.empty() &&
      std::isfinite(ego.x) && std::isfinite(ego.y) && std::isfinite(ego.yaw)) {
    // D3 wall接近時はFrenet再投影値ではなく、controllerが現在いる実測poseを
    // RECOVERYの厳密な始端にする。この差を後段で無視せず、変更後の全Cartesian
    // 列をtrackabilityとSafetyEvaluatorへそのまま通す。
    candidate.x.front() = ego.x;
    candidate.y.front() = ego.y;
    candidate.yaw.front() = ego.yaw;
  }

  if (candidate.pass_transition_deadline.evaluated) {
    auto &diagnostic = candidate.pass_transition_deadline;
    diagnostic.proposal_speed_cap_mps = speed_cap;
    diagnostic.pp_required_arc_m =
        candidate.required_controller_spatial_horizon_m;
    if (!candidate.t.empty()) {
      diagnostic.proposal_horizon_sec = candidate.t.back();
    }
    if (!candidate.longitudinal_offsets_m.empty()) {
      diagnostic.proposal_endpoint_arc_m =
          candidate.longitudinal_offsets_m.back();
    }
    if (!candidate.predicted_speed_mps.empty()) {
      diagnostic.proposal_end_speed_mps = candidate.predicted_speed_mps.back();
    }
    if (std::isfinite(diagnostic.required_transition_m)) {
      const auto required_sample =
          std::lower_bound(candidate.longitudinal_offsets_m.cbegin(),
                           candidate.longitudinal_offsets_m.cend(),
                           diagnostic.required_transition_m);
      if (required_sample != candidate.longitudinal_offsets_m.cend()) {
        const auto sample_index = static_cast<std::size_t>(std::distance(
            candidate.longitudinal_offsets_m.cbegin(), required_sample));
        if (sample_index < candidate.t.size()) {
          diagnostic.proposal_time_to_required_transition_sec =
              candidate.t[sample_index];
        }
      }
    }
  }

  const bool pass_profile =
      type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
  if (pass_profile) {
    // PASSも空間horizonだけでなく、実際にpublishするC2横profileと基準CSV曲率を
    // 合成したactive-controller操舵角/速度契約へ通す。localized profileは
    // piecewise/chain/current-d補正を含むため、単一区間の名目式と分離して評価する。
    const auto trackability =
        use_localized_profile
            ? localizedPassProfileTrackability(*localized_profile, candidate,
                                               ego)
            : lateralCorrectionProfileTrackability(candidate, ego, target_d,
                                                   shift_distance);
    candidate.desired_path_trackable = trackability.desired_path_trackable;
    candidate.pure_pursuit_command_trackable =
        trackability.pure_pursuit_command_trackable;
    candidate.controller_tracking_profile_valid =
        pass_controller_geometry_valid &&
        candidate.pass_target_corridor_valid && trackability.valid();

    // moving targetに対して「速度capが相手以上」という瞬間値だけでは、同速の
    // まま永遠に横へ並ぶ候補や、横移動後に追い付けない候補をPASS認可してしまう。
    // 最終LongitudinalProfileをbounded時間先まで進め、横分離が完了した後に
    // merge_front_gap_mだけ前へ出て、正のclosing speedが2 sample連続することを
    // 確認する。停止対象は従来の停止回避契約を使い、この完遂性gateから除外する。
    candidate.moving_target_relatively_reachable =
        !config_.moving_pass_reachability_enabled;
    int target_index = passTargetIndex(blocked_info, opponents);
    if (use_localized_profile && localized_profile != nullptr &&
        !localized_profile->target_id.empty()) {
      const auto localized_target =
          std::find_if(opponents.begin(), opponents.end(),
                       [&localized_profile](const OpponentState &opponent) {
                         return opponent.id == localized_profile->target_id;
                       });
      target_index = localized_target == opponents.end()
                         ? -1
                         : static_cast<int>(std::distance(opponents.begin(),
                                                          localized_target));
    }
    if (config_.moving_pass_reachability_enabled && target_index >= 0 &&
        static_cast<std::size_t>(target_index) < opponents.size()) {
      const auto &target = opponents[static_cast<std::size_t>(target_index)];
      if (target.valid && std::isfinite(target.v) && target.v >= 0.0 &&
          target.v <= kMovingPassStationarySpeedEpsilonMps) {
        candidate.moving_target_relatively_reachable = true;
      } else if (target.valid && std::isfinite(target.v) && target.v >= 0.0 &&
                 std::isfinite(target.frenet.s) &&
                 std::isfinite(ego.frenet.s) &&
                 std::isfinite(config_.moving_pass_max_completion_time_sec) &&
                 config_.moving_pass_max_completion_time_sec > 0.0 &&
                 std::isfinite(config_.moving_pass_min_closing_speed_mps) &&
                 config_.moving_pass_min_closing_speed_mps >= 0.0) {
        const auto target_reference = frame_.interpolate(target.frenet.s);
        const double projected_s_dot_mps =
            target.vx * std::cos(target_reference.yaw) +
            target.vy * std::sin(target_reference.yaw);
        const double vector_speed_mps = std::hypot(target.vx, target.vy);
        const double target_s_dot_mps =
            std::isfinite(projected_s_dot_mps) &&
                    std::isfinite(vector_speed_mps) && vector_speed_mps > 1.0e-3
                ? std::max(0.0, projected_s_dot_mps)
                : target.v;
        double lateral_ready_distance_m = shift_distance;
        if (use_localized_profile && localized_profile != nullptr) {
          const double current_unwrapped_s_m = profileUnwrappedS(
              *localized_profile, ego.frenet.s, frame_.length());
          lateral_ready_distance_m =
              localized_profile->full_offset_start_s_m - current_unwrapped_s_m;
        }
        double initial_relative_s_m =
            frame_.deltaS(ego.frenet.s, target.frenet.s);
        if (use_localized_profile && localized_profile != nullptr &&
            std::isfinite(localized_profile->target_s_m)) {
          const double current_unwrapped_s_m = profileUnwrappedS(
              *localized_profile, ego.frenet.s, frame_.length());
          if (std::isfinite(current_unwrapped_s_m)) {
            // commit済みprofileではtargetが後方へ回った後も、前方距離deltaSで
            // 約1周先へ折り返さず、transactionの連続座標で完了余裕を評価する。
            initial_relative_s_m =
                localized_profile->target_s_m - current_unwrapped_s_m;
          }
        }
        const double sample_dt_sec =
            std::clamp(config_.horizon_dt_sec, 0.05, 0.25);
        int consecutive_reachable_samples = 0;
        for (double t_sec = 0.0;
             t_sec <= config_.moving_pass_max_completion_time_sec + 1.0e-9;
             t_sec += sample_dt_sec) {
          const double ego_progress_m = longitudinal_profile.distanceAt(t_sec);
          const double ego_speed_mps = longitudinal_profile.speedAt(t_sec);
          const double relative_s_m =
              initial_relative_s_m + target_s_dot_mps * t_sec - ego_progress_m;
          const double closing_speed_mps = ego_speed_mps - target_s_dot_mps;
          const bool completion_sample =
              std::isfinite(ego_progress_m) && std::isfinite(ego_speed_mps) &&
              std::isfinite(relative_s_m) && std::isfinite(closing_speed_mps) &&
              std::isfinite(lateral_ready_distance_m) &&
              ego_progress_m + 1.0e-6 >=
                  std::max(0.0, lateral_ready_distance_m) &&
              relative_s_m <= -std::max(0.0, config_.merge_front_gap_m) &&
              closing_speed_mps + 1.0e-9 >=
                  config_.moving_pass_min_closing_speed_mps;
          consecutive_reachable_samples =
              completion_sample ? consecutive_reachable_samples + 1 : 0;
          if (consecutive_reachable_samples >= 2) {
            candidate.moving_target_relatively_reachable = true;
            break;
          }
        }
      }
    }
  } else if (attack_follow_profile || prestart_attack_follow_profile ||
             start_grid_anchored_hold_profile ||
             pass_reauthorization_recovery_profile ||
             stop_hold_lateral_profile) {
    // Frenet補正単体の解析解に加え、基準CSVの連続曲率と横profileを合成した
    // 実オフセット軌道でもactive controllerの操舵契約を満たすことを確認する。
    const auto trackability = lateralCorrectionProfileTrackability(
        candidate, ego, target_d, shift_distance);
    candidate.desired_path_trackable = trackability.desired_path_trackable;
    candidate.pure_pursuit_command_trackable =
        trackability.pure_pursuit_command_trackable;
    candidate.controller_tracking_profile_valid =
        candidate.pass_target_corridor_valid && trackability.valid();
  }
  if (start_grid_uncommitted_current_d_candidate &&
      !start_grid_uncommitted_future_corridor_valid) {
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.corridor_min_margin_m = -std::numeric_limits<double>::infinity();
  }
  if (!candidate.longitudinal_profile_valid) {
    candidate.controller_tracking_profile_valid = false;
  }
  if (controller_lateral_profile &&
      (!std::isfinite(required_controller_spatial_horizon_m) ||
       candidate.longitudinal_offsets_m.empty() ||
       candidate.longitudinal_offsets_m.back() <
           required_controller_spatial_horizon_m)) {
    // downstream controllerが必要とする距離まで安全評価できない横profileを、
    // PlannerではPASS可・PPではspeed-onlyと判定させない。両者をfail-closedで
    // 同じ不成立へ揃える。
    candidate.controller_tracking_profile_valid = false;
  }
  if (start_grid_anchor_context && !start_grid_anchor_valid) {
    // ATTACK_FOLLOWなど後段の別profile評価がtrueでも、activeなstart-grid
    // anchor欠損を上書きしてはならない。target/sideを保持したままCoreで
    // start_grid_hold_anchor_invalidとしてfail-closedにする最終不変条件。
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.corridor_min_margin_m = -std::numeric_limits<double>::infinity();
  }
  // SafetyEvaluator後の同一candidateだけをproof済みとする。
  candidate.controller_spatial_horizon_proof_valid = false;

  return candidate;
}

// 入力: 自車状態、RECOVERY生成時のblocked情報、生成済みcenter復帰候補。
// 出力: 横profileが名目smoothstepより中心側へ先行していなければtrue。
// 処理概要: s依存corridorのpoint clampが安全評価上は壁内でも、PP/MPCが
// 追従できない急なcenter方向jumpを作る場合はV2のCENTERING認可前に検出する。
bool CandidateBuilder::centeringProfileMatchesNominal(
    const EgoState &ego, const BlockedInfo &blocked_info,
    const CandidateTrajectory &candidate) const {
  if (!std::isfinite(ego.frenet.d) || candidate.d.empty() ||
      candidate.d.size() != candidate.longitudinal_offsets_m.size()) {
    return false;
  }
  if (std::abs(candidate.d.front() - ego.frenet.d) > 1.0e-6) {
    return false;
  }
  const double start_d = ego.frenet.d;
  const double shift_distance =
      std::max(1.0, recoveryShiftDistanceM(ego, blocked_info));
  double previous_ds = -1.0e-9;
  for (std::size_t i = 0; i < candidate.d.size(); ++i) {
    const double ds = candidate.longitudinal_offsets_m[i];
    const double candidate_d = candidate.d[i];
    if (!std::isfinite(ds) || !std::isfinite(candidate_d) ||
        ds + 1.0e-9 < previous_ds) {
      return false;
    }
    const double ratio = smoothstep(ds / shift_distance);
    const double nominal_d = start_d * (1.0 - ratio);
    if (std::abs(candidate_d) + 1.0e-6 < std::abs(nominal_d)) {
      return false;
    }
    previous_ds = ds;
  }
  return true;
}

// 入力: 自車状態とRECOVERY生成時のblocked情報。
// 出力: center復帰smoothstepに使う前進距離[m]。
// 処理概要: Candidate生成とV2追従可能性評価で同じ距離を共有し、回廊外だけは
// 既存の短縮profileを使う。
double CandidateBuilder::recoveryShiftDistanceM(
    const EgoState &ego, const BlockedInfo &blocked_info) const {
  const auto ego_bounds =
      frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
  const double lower_d = ego_bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = ego_bounds.d_max - config_.min_wall_margin_m;
  const bool outside_safe_corridor =
      ego.frenet.d < lower_d || ego.frenet.d > upper_d;
  const bool holds_lateral =
      blocked_info.reentry_hold_active ||
      blocked_info.start_grid_follow_hold_lateral ||
      blocked_info.early_stationary_parallel_pass_hold_lateral ||
      blocked_info.braking_follow_hold_lateral ||
      blocked_info.attack_follow_hold_pass_side;
  if (holds_lateral || !outside_safe_corridor || blocked_info.side_by_side) {
    return config_.merge_distance_m;
  }
  const double base_distance = std::min(
      finitePositiveOr(config_.merge_distance_m, 12.0),
      finitePositiveOr(config_.prepare_distance_m, config_.merge_distance_m));
  return std::max(1.0, base_distance * kOutsideCorridorRecoveryShiftScale);
}

// 入力: 未完了PASS transactionと観測済みchain tailの相対距離。
// 出力: 現在dからラッチ済みPASS側dへ収束させる物理距離[m]。
// 処理概要:
// chain tailの安全楕円境界を補正期限として使うが、active
// PurePursuit/Muxの操舵角・操舵速度
// 制約から求めた追従可能距離よりは短縮しない。期限内に必要dへ届かない形は
// SafetyEvaluatorが実際の緩やかなd列でrejectし、急な見かけ上の回避を許可しない。
double CandidateBuilder::attackFollowShiftDistanceM(
    const EgoState &ego, double target_d, double speed_cap_mps,
    const BlockedInfo &blocked_info) const {
  const double configured_distance_m =
      std::max(1.0, config_.localized_avoidance_start_before_target_m);
  double deadline_distance_m = configured_distance_m;
  if (blocked_info.maneuver_chain_tail_observed &&
      std::isfinite(blocked_info.maneuver_chain_tail_relative_s_m)) {
    const double remaining_to_safety_entry_m =
        blocked_info.maneuver_chain_tail_relative_s_m -
        longitudinalEllipseClearance(config_);
    deadline_distance_m =
        std::max(1.0, std::min(configured_distance_m,
                               std::max(0.0, remaining_to_safety_entry_m)));
  }

  return trackingLimitedLateralShiftDistanceM(
      ego, target_d, speed_cap_mps, blocked_info, deadline_distance_m);
}

double CandidateBuilder::minimumTrackableLateralShiftDistance(
    const EgoState &ego, double target_d, double speed_cap_mps,
    const BlockedInfo &blocked_info, double requested_distance_m) const {
  return trackingLimitedLateralShiftDistanceM(
      ego, target_d, speed_cap_mps, blocked_info, requested_distance_m);
}

PassTransitionFeasibility CandidateBuilder::passTransitionFeasibility(
    const EgoState &ego, double target_d, double speed_cap_mps,
    const BlockedInfo &blocked_info, double available_deadline_m) const {
  PassTransitionFeasibility result;
  result.lateral_shift_m = std::abs(target_d - ego.frenet.d);
  const double reserve_sec =
      config_.lateral_override_execution_speed_reserve_sec;
  const double accel_bound_mps2 = config_.pass_assumed_accel_mps2;
  const double execution_speed_headroom_mps =
      std::max(0.0, ego.v) + accel_bound_mps2 * reserve_sec;
  result.evaluated_tracking_speed_mps =
      std::max({0.0, execution_speed_headroom_mps,
                finitePositiveOr(speed_cap_mps, ego.v)});
  // Compare the deadline with the physical controller minimum, not with the
  // configured profile-design distance.  The latter is intentionally a lower
  // bound for normal shape generation and would reject a shorter but still
  // PP-trackable transition as if it were physically impossible.
  result.required_transition_m = trackingLimitedLateralShiftDistanceM(
      ego, target_d, speed_cap_mps, blocked_info, 1.0);
  result.available_deadline_m = available_deadline_m;
  result.input_valid = ego.valid && std::isfinite(ego.frenet.s) &&
                       std::isfinite(ego.frenet.d) && std::isfinite(ego.v) &&
                       std::isfinite(target_d) &&
                       std::isfinite(speed_cap_mps) && speed_cap_mps >= 0.0 &&
                       std::isfinite(result.lateral_shift_m) &&
                       std::isfinite(result.evaluated_tracking_speed_mps) &&
                       std::isfinite(result.required_transition_m) &&
                       std::isfinite(result.available_deadline_m);
  result.reachable =
      std::isfinite(result.required_transition_m) &&
      std::isfinite(result.available_deadline_m) &&
      result.required_transition_m <= result.available_deadline_m;
  return result;
}

// 入力: 自車、横目標d、候補速度、コース曲率、希望する補正距離[m]。
// 出力: 操舵角・操舵速度制約を満たす、希望距離以上の最短横補正距離[m]。
// 処理概要: PASS後の攻めFOLLOWとstart-grid anchor保持で同じ実controller契約を
// 使い、短い見かけ上の横軌道をSafetyEvaluatorへ渡さない。
double CandidateBuilder::trackingLimitedLateralShiftDistanceM(
    const EgoState &ego, double target_d, double speed_cap_mps,
    const BlockedInfo &blocked_info, double requested_distance_m) const {
  const double deadline_distance_m = std::max(1.0, requested_distance_m);
  const double track_length_m = frame_.length();
  const double distance_upper_bound_m =
      std::isfinite(track_length_m) && track_length_m > 1.0
          ? std::min(kLateralCorrectionDistanceUpperBoundM, track_length_m)
          : kLateralCorrectionDistanceUpperBoundM;
  if (!std::isfinite(deadline_distance_m) ||
      deadline_distance_m > distance_upper_bound_m) {
    return std::numeric_limits<double>::infinity();
  }

  const double lateral_delta_m = std::abs(target_d - ego.frenet.d);
  if (!std::isfinite(lateral_delta_m) || lateral_delta_m <= 1.0e-6) {
    return deadline_distance_m;
  }

  const double wheelbase_m = config_.attack_follow_tracking_wheelbase_m;
  const double max_steering_angle_rad =
      config_.attack_follow_max_steering_angle_rad;
  const double max_steering_rate_radps =
      config_.attack_follow_max_steering_rate_radps;
  const double steering_tire_angle_gain =
      config_.attack_follow_steering_tire_angle_gain;
  const auto finite_abs_or_zero = [](double value) {
    return std::isfinite(value) ? std::abs(value) : 0.0;
  };
  const double reference_curvature_reserve_m_inv =
      std::max(finite_abs_or_zero(blocked_info.corner_abs_curvature),
               finite_abs_or_zero(blocked_info.future_abs_curvature));
  if (!std::isfinite(max_steering_angle_rad) || max_steering_angle_rad <= 0.0 ||
      max_steering_angle_rad >= 1.5707963267948966 ||
      !std::isfinite(max_steering_rate_radps) ||
      max_steering_rate_radps <= 0.0 || !std::isfinite(wheelbase_m) ||
      wheelbase_m <= 0.0 || !std::isfinite(steering_tire_angle_gain) ||
      steering_tire_angle_gain <= 0.0 ||
      !std::isfinite(config_.attack_follow_steering_rate_reserve_ratio) ||
      config_.attack_follow_steering_rate_reserve_ratio <= 0.0 ||
      config_.attack_follow_steering_rate_reserve_ratio > 1.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double reserve_sec =
      config_.lateral_override_execution_speed_reserve_sec;
  const double accel_bound_mps2 = config_.pass_assumed_accel_mps2;
  if (!std::isfinite(reserve_sec) || reserve_sec < 0.05 || reserve_sec > 0.50 ||
      !std::isfinite(accel_bound_mps2) || accel_bound_mps2 <= 0.0 ||
      accel_bound_mps2 > 3.0) {
    return std::numeric_limits<double>::infinity();
  }
  // makeLongitudinalProfile()が横overrideの安全予測へ足す残留加速headroomを
  // 同じまま横補正距離の操舵速度評価にも使う。実測/速度capだけで距離を決めると、
  // 生成後のpredicted_speedだけが高くなって同じ候補をuntrackableにしてしまう。
  const double execution_speed_headroom_mps =
      std::max(0.0, ego.v) + accel_bound_mps2 * reserve_sec;
  const double evaluated_speed_mps =
      std::max({0.0, execution_speed_headroom_mps,
                finitePositiveOr(speed_cap_mps, ego.v)});
  const double usable_steering_rate_radps =
      (max_steering_rate_radps / steering_tire_angle_gain) *
      config_.attack_follow_steering_rate_reserve_ratio;
  const auto profile_trackable = [&](double distance_m) {
    if (!std::isfinite(distance_m) || distance_m <= 0.0) {
      return false;
    }
    if (!nominalLateralCorrectionPathTrackable(ego, target_d, distance_m,
                                               evaluated_speed_mps)) {
      return false;
    }
    constexpr int kSamples = 200;
    for (int sample = 0; sample <= kSamples; ++sample) {
      const double z =
          static_cast<double>(sample) / static_cast<double>(kSamples);
      // quintic smootherstep f(z)=6z^5-15z^4+10z^3 の導関数。
      const double f1 = 30.0 * z * z * (1.0 - z) * (1.0 - z);
      const double f2 = 60.0 * z * (2.0 * z * z - 3.0 * z + 1.0);
      const double f3 = 60.0 * (6.0 * z * z - 6.0 * z + 1.0);
      const double d1 = lateral_delta_m * f1 / distance_m;
      const double d2 = lateral_delta_m * f2 / (distance_m * distance_m);
      const double d3 =
          lateral_delta_m * f3 / (distance_m * distance_m * distance_m);
      const double slope_norm = 1.0 + d1 * d1;
      const double profile_curvature = d2 / std::pow(slope_norm, 1.5);
      const double curvature_rate_per_m =
          d3 / std::pow(slope_norm, 1.5) -
          3.0 * d1 * d2 * d2 / std::pow(slope_norm, 2.5);
      const double total_curvature_bound =
          reference_curvature_reserve_m_inv + std::abs(profile_curvature);
      const double steering_angle_bound_rad =
          std::atan(wheelbase_m * total_curvature_bound);
      // atan(wheelbase*kappa)の分母は1以上なので、省略した値を上限に使う。
      const double steering_rate_bound_radps =
          wheelbase_m * std::abs(curvature_rate_per_m) * evaluated_speed_mps;
      const bool stationary_lateral_accel_limit_active =
          blocked_info.stationary_no_pass_safe_pass_constraint_active &&
          std::isfinite(
              config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2) &&
          config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 > 0.0;
      const double lateral_accel_bound_mps2 =
          evaluated_speed_mps * evaluated_speed_mps * total_curvature_bound;
      if (steering_angle_bound_rad > max_steering_angle_rad ||
          steering_rate_bound_radps > usable_steering_rate_radps + 1.0e-9 ||
          (stationary_lateral_accel_limit_active &&
           (!std::isfinite(lateral_accel_bound_mps2) ||
            lateral_accel_bound_mps2 >
                config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 +
                    1.0e-9))) {
        return false;
      }
    }
    CandidateTrajectory probe;
    probe.v_ref = {evaluated_speed_mps};
    probe.predicted_speed_mps = {evaluated_speed_mps};
    const double signed_lateral_delta_m = target_d - ego.frenet.d;
    return sampledLateralProfileTrackability(
               probe, ego, distance_m,
               [&](double query_distance_m) {
                 const double z =
                     std::clamp(query_distance_m / distance_m, 0.0, 1.0);
                 return ego.frenet.d + signed_lateral_delta_m * smootherstep(z);
               })
        .valid();
  };

  double lower_distance_m = 0.0;
  double upper_distance_m = std::max(1.0, deadline_distance_m);
  for (int expansion = 0;
       expansion < 8 && !profile_trackable(upper_distance_m) &&
       upper_distance_m < distance_upper_bound_m;
       ++expansion) {
    lower_distance_m = upper_distance_m;
    upper_distance_m = std::min(distance_upper_bound_m, upper_distance_m * 2.0);
  }
  if (!profile_trackable(upper_distance_m)) {
    return std::numeric_limits<double>::infinity();
  }
  for (int iteration = 0; iteration < 40; ++iteration) {
    const double middle_distance_m =
        0.5 * (lower_distance_m + upper_distance_m);
    if (profile_trackable(middle_distance_m)) {
      upper_distance_m = middle_distance_m;
    } else {
      lower_distance_m = middle_distance_m;
    }
  }
  return std::max(deadline_distance_m, upper_distance_m);
}

// 入力: 実測自車状態、固定横目標、補正完了距離、候補の最大実行速度。
// 出力: 補正開始から完了まで、基準線とquintic横補正を合成した操舵角/速度が
// active controller契約内ならtrue。
// 処理概要: publish horizonの先も最大5 cm刻みで検査し、将来の曲率・曲率変化を
// 見ないまま補正へcommitしない。距離とsample数は明示上限でplanner時間を拘束する。
bool CandidateBuilder::nominalLateralCorrectionPathTrackable(
    const EgoState &ego, double target_d, double shift_distance_m,
    double evaluated_speed_mps) const {
  const double track_length_m = frame_.length();
  const double distance_upper_bound_m =
      std::isfinite(track_length_m) && track_length_m > 1.0
          ? std::min(kLateralCorrectionDistanceUpperBoundM, track_length_m)
          : kLateralCorrectionDistanceUpperBoundM;
  if (!std::isfinite(ego.frenet.s) || !std::isfinite(ego.frenet.d) ||
      !std::isfinite(target_d) || !std::isfinite(shift_distance_m) ||
      shift_distance_m <= 0.0 || shift_distance_m > distance_upper_bound_m ||
      !std::isfinite(evaluated_speed_mps) || evaluated_speed_mps < 0.0) {
    return false;
  }

  const double wheelbase_m = config_.attack_follow_tracking_wheelbase_m;
  const double max_angle_rad = config_.attack_follow_max_steering_angle_rad;
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
      !std::isfinite(max_angle_rad) || max_angle_rad <= 0.0 ||
      !std::isfinite(config_.attack_follow_steering_tire_angle_gain) ||
      config_.attack_follow_steering_tire_angle_gain <= 0.0 ||
      !std::isfinite(config_.attack_follow_max_steering_rate_radps) ||
      config_.attack_follow_max_steering_rate_radps <= 0.0 ||
      !std::isfinite(config_.attack_follow_steering_rate_reserve_ratio) ||
      config_.attack_follow_steering_rate_reserve_ratio <= 0.0 ||
      config_.attack_follow_steering_rate_reserve_ratio > 1.0) {
    return false;
  }
  const double max_rate_radps =
      (config_.attack_follow_max_steering_rate_radps /
       config_.attack_follow_steering_tire_angle_gain) *
      config_.attack_follow_steering_rate_reserve_ratio;

  const std::size_t requested_samples = static_cast<std::size_t>(
      std::ceil(shift_distance_m / kLateralCorrectionTrackabilitySpacingM));
  const std::size_t sample_count =
      std::clamp(requested_samples, std::size_t{2U},
                 kLateralCorrectionTrackabilityMaxSamples);
  constexpr double kCurvatureDerivativeWindowM = 0.05;
  constexpr double kMinPathMetric = 1.0e-3;
  constexpr double kMinSegmentM = 1.0e-5;
  const double lateral_delta_m = target_d - ego.frenet.d;
  double previous_steering_rad = 0.0;
  double previous_metric = 0.0;
  double previous_distance_m = 0.0;
  bool have_previous = false;

  for (std::size_t sample = 0U; sample <= sample_count; ++sample) {
    const double z =
        static_cast<double>(sample) / static_cast<double>(sample_count);
    const double distance_m = shift_distance_m * z;
    const double f = smootherstep(z);
    const double f1 = 30.0 * z * z * (1.0 - z) * (1.0 - z);
    const double f2 = 60.0 * z * (2.0 * z * z - 3.0 * z + 1.0);
    const double d = ego.frenet.d + lateral_delta_m * f;
    const double d_prime = lateral_delta_m * f1 / shift_distance_m;
    const double d_second =
        lateral_delta_m * f2 / (shift_distance_m * shift_distance_m);
    const double s = frame_.wrapS(ego.frenet.s + distance_m);
    const double reference_curvature = frame_.interpolate(s).kappa;
    const double curvature_before =
        frame_.interpolate(s - kCurvatureDerivativeWindowM).kappa;
    const double curvature_after =
        frame_.interpolate(s + kCurvatureDerivativeWindowM).kappa;
    if (!std::isfinite(reference_curvature) ||
        !std::isfinite(curvature_before) || !std::isfinite(curvature_after)) {
      return false;
    }
    const double reference_curvature_derivative =
        (curvature_after - curvature_before) /
        (2.0 * kCurvatureDerivativeWindowM);
    const double tangent_scale = 1.0 - reference_curvature * d;
    const double metric_sq = tangent_scale * tangent_scale + d_prime * d_prime;
    if (!std::isfinite(metric_sq) ||
        metric_sq < kMinPathMetric * kMinPathMetric) {
      return false;
    }
    const double metric = std::sqrt(metric_sq);
    const double curvature_numerator =
        reference_curvature * tangent_scale * tangent_scale +
        tangent_scale * d_second +
        reference_curvature_derivative * d * d_prime +
        2.0 * reference_curvature * d_prime * d_prime;
    const double offset_curvature = curvature_numerator / (metric_sq * metric);
    const double steering_rad = std::atan(wheelbase_m * offset_curvature);
    if (!std::isfinite(steering_rad) ||
        std::abs(steering_rad) > max_angle_rad) {
      return false;
    }
    if (have_previous) {
      const double reference_segment_m = distance_m - previous_distance_m;
      const double path_segment_m =
          reference_segment_m * 0.5 * (previous_metric + metric);
      if (!std::isfinite(path_segment_m)) {
        return false;
      }
      if (path_segment_m > kMinSegmentM) {
        const double steering_rate_radps =
            std::abs(steering_rad - previous_steering_rad) / path_segment_m *
            evaluated_speed_mps;
        if (!std::isfinite(steering_rate_radps) ||
            steering_rate_radps > max_rate_radps + 1.0e-9) {
          return false;
        }
      }
    }
    previous_steering_rad = steering_rad;
    previous_metric = metric;
    previous_distance_m = distance_m;
    have_previous = true;
  }
  return true;
}

// 入力: 生成済みATTACK_FOLLOW候補、現在姿勢、目標d、横補正距離[m]。
// 出力:
// 基準線と横補正を合成した総曲率から求める操舵角/操舵速度が制約内ならtrue。
// 処理概要: 約1 m間隔の基準CSVを数cm間隔の3点外接円で評価すると、線形補間の
// 節点を無限大に近い曲率変化と誤認する。CSVが持つ連続kappaとquintic d(s)から
// オフセット曲線の曲率を解析し、候補速度で操舵速度へ変換する。回廊clamp等で
// 名目profileと実d列がずれた候補は、このモデルで正当化せずfail-closedにする。
CandidateBuilder::LateralTrackabilityResult
CandidateBuilder::lateralCorrectionProfileTrackability(
    const CandidateTrajectory &candidate, const EgoState &ego, double target_d,
    double shift_distance_m) const {
  const std::size_t point_count = candidate.s.size();
  if (point_count < 3U || candidate.d.size() != point_count ||
      candidate.longitudinal_offsets_m.size() != point_count ||
      candidate.v_ref.size() != point_count ||
      candidate.predicted_speed_mps.size() != point_count ||
      !std::isfinite(ego.frenet.d) || !std::isfinite(target_d) ||
      !std::isfinite(shift_distance_m) || shift_distance_m <= 0.0) {
    return {};
  }
  const double lateral_delta_m = target_d - ego.frenet.d;
  double evaluated_speed_mps = 0.0;
  for (std::size_t i = 0U; i < point_count; ++i) {
    const double ds = candidate.longitudinal_offsets_m[i];
    if (!std::isfinite(ds) ||
        (i > 0U && ds + 1.0e-9 < candidate.longitudinal_offsets_m[i - 1U])) {
      return {};
    }
    const double z = std::clamp(ds / shift_distance_m, 0.0, 1.0);
    const double f = smootherstep(z);
    const double nominal_d = ego.frenet.d + lateral_delta_m * f;
    if (!std::isfinite(candidate.d[i]) ||
        std::abs(candidate.d[i] - nominal_d) > 1.0e-4) {
      return {};
    }
    if (!std::isfinite(candidate.v_ref[i]) ||
        !std::isfinite(candidate.predicted_speed_mps[i])) {
      return {};
    }
    evaluated_speed_mps = std::max({evaluated_speed_mps, candidate.v_ref[i],
                                    candidate.predicted_speed_mps[i]});
  }
  const double check_distance_m =
      std::max({shift_distance_m, candidate.longitudinal_offsets_m.back(),
                candidate.required_controller_spatial_horizon_m});
  auto result = sampledLateralProfileTrackability(
      candidate, ego, check_distance_m, [&](double distance_m) {
        const double z = std::clamp(distance_m / shift_distance_m, 0.0, 1.0);
        return ego.frenet.d + lateral_delta_m * smootherstep(z);
      });
  result.desired_path_trackable =
      result.desired_path_trackable &&
      nominalLateralCorrectionPathTrackable(ego, target_d, shift_distance_m,
                                            evaluated_speed_mps);
  return result;
}

// 入力: 実際にpublishするlocalized PASS profile、生成済み候補、自車状態。
// 出力: current-d補正、車列waypoint、保持/mergeを含む全profileが、active
// PurePursuitの操舵角・操舵速度契約内ならtrue。
// 処理概要: 候補horizonとraw profileの一致を先に確認し、merge終端までの
// piecewise d(s)をPP lookahead式へ通す。corridor clamp等で一致しない場合や、
// sample上限外・非有限値は、安全に見える短い区間だけを採用せずfalseへ閉じる。
CandidateBuilder::LateralTrackabilityResult
CandidateBuilder::localizedPassProfileTrackability(
    const LocalizedLateralProfile &profile,
    const CandidateTrajectory &candidate, const EgoState &ego) const {
  if (!profile.active || !std::isfinite(ego.frenet.s) ||
      !std::isfinite(ego.frenet.d) || candidate.d.empty() ||
      candidate.d.size() != candidate.longitudinal_offsets_m.size() ||
      candidate.v_ref.size() != candidate.d.size() ||
      candidate.predicted_speed_mps.size() != candidate.d.size()) {
    return {};
  }

  for (std::size_t i = 0U; i < candidate.d.size(); ++i) {
    const double distance_m = candidate.longitudinal_offsets_m[i];
    if (!std::isfinite(distance_m) ||
        (i > 0U &&
         distance_m + 1.0e-9 < candidate.longitudinal_offsets_m[i - 1U]) ||
        !std::isfinite(candidate.d[i]) || !std::isfinite(candidate.v_ref[i]) ||
        !std::isfinite(candidate.predicted_speed_mps[i])) {
      return {};
    }
    const double s = frame_.wrapS(ego.frenet.s + distance_m);
    const double expected_d = localizedProfileRawD(profile, ego, s, distance_m);
    if (!std::isfinite(expected_d) ||
        std::abs(candidate.d[i] - expected_d) > 1.0e-4) {
      return {};
    }
  }

  const double current_unwrapped_s =
      profileUnwrappedS(profile, ego.frenet.s, frame_.length());
  if (!std::isfinite(current_unwrapped_s) ||
      !std::isfinite(profile.merge_end_s_m)) {
    return {};
  }
  const double profile_remaining_m =
      std::max(0.0, profile.merge_end_s_m - current_unwrapped_s);
  const double check_distance_m =
      std::max({profile_remaining_m, candidate.longitudinal_offsets_m.back(),
                candidate.required_controller_spatial_horizon_m});
  const double track_length_m = frame_.length();
  const double distance_upper_bound_m =
      std::isfinite(track_length_m) && track_length_m > 1.0
          ? std::min(kLateralCorrectionDistanceUpperBoundM, track_length_m)
          : kLateralCorrectionDistanceUpperBoundM;
  if (!std::isfinite(check_distance_m) || check_distance_m <= 0.0 ||
      check_distance_m > distance_upper_bound_m) {
    return {};
  }

  return sampledLateralProfileTrackability(
      candidate, ego, check_distance_m, [&](double distance_m) {
        const double s = frame_.wrapS(ego.frenet.s + distance_m);
        return localizedProfileRawD(profile, ego, s, distance_m);
      });
}

// 入力: publish直前の横d列と、それを生成した自車状態。
// 出力: 実際の最終d列がactive PurePursuitの操舵契約内ならtrue。
// 処理概要: 周期間rate limit/holdで変形したlegacy PASSを線形補間して直接
// PP lookahead式へ通す。候補生成前の名目profileだけが安全でも許可しない。
bool CandidateBuilder::publishedLateralProfileTrackable(
    const CandidateTrajectory &candidate, const EgoState &ego) const {
  const std::size_t point_count = candidate.d.size();
  if (point_count < 4U || !std::isfinite(ego.frenet.s) ||
      !std::isfinite(ego.frenet.d) ||
      candidate.longitudinal_offsets_m.size() != point_count ||
      candidate.v_ref.size() != point_count ||
      candidate.predicted_speed_mps.size() != point_count) {
    return false;
  }
  for (std::size_t i = 0U; i < point_count; ++i) {
    const double distance_m = candidate.longitudinal_offsets_m[i];
    if (!std::isfinite(distance_m) || !std::isfinite(candidate.d[i]) ||
        (i > 0U &&
         distance_m + 1.0e-9 < candidate.longitudinal_offsets_m[i - 1U])) {
      return false;
    }
  }
  const double check_distance_m = candidate.longitudinal_offsets_m.back();
  if (!std::isfinite(check_distance_m) || check_distance_m <= 0.0 ||
      !std::isfinite(candidate.required_controller_spatial_horizon_m) ||
      check_distance_m + 1.0e-6 <
          candidate.required_controller_spatial_horizon_m) {
    return false;
  }

  return sampledLateralProfileTrackability(
             candidate, ego, check_distance_m,
             [&](double distance_m) {
               const auto upper = std::upper_bound(
                   candidate.longitudinal_offsets_m.begin(),
                   candidate.longitudinal_offsets_m.end(), distance_m);
               if (upper == candidate.longitudinal_offsets_m.begin()) {
                 return candidate.d.front();
               }
               if (upper == candidate.longitudinal_offsets_m.end()) {
                 return candidate.d.back();
               }
               const std::size_t upper_index =
                   static_cast<std::size_t>(std::distance(
                       candidate.longitudinal_offsets_m.begin(), upper));
               const std::size_t lower_index = upper_index - 1U;
               const double lower_distance_m =
                   candidate.longitudinal_offsets_m[lower_index];
               const double upper_distance_m =
                   candidate.longitudinal_offsets_m[upper_index];
               const double interval_m = upper_distance_m - lower_distance_m;
               if (!std::isfinite(interval_m) || interval_m <= 1.0e-9) {
                 return candidate.d[upper_index];
               }
               const double ratio = std::clamp(
                   (distance_m - lower_distance_m) / interval_m, 0.0, 1.0);
               return candidate.d[lower_index] +
                      ratio *
                          (candidate.d[upper_index] - candidate.d[lower_index]);
             })
      .valid();
}

CandidateTrajectory CandidateBuilder::makeAttackFollowCurrentDHoldVariant(
    const CandidateTrajectory &source, const EgoState &ego) const {
  CandidateTrajectory candidate = source;
  candidate.safety_evaluated = false;
  candidate.feasible = false;
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
  candidate.attack_follow_safe_lateral_hold = true;
  candidate.attack_follow_opponent_collision_current_d_hold = true;
  candidate.attack_follow_opponent_collision_inward_connector = false;

  const std::size_t point_count = source.t.size();
  const bool source_shape_valid =
      source.type == CandidateType::FOLLOW && point_count >= 4U &&
      source.longitudinal_offsets_m.size() == point_count &&
      source.s.size() == point_count && source.d.size() == point_count &&
      source.x.size() == point_count && source.y.size() == point_count &&
      source.yaw.size() == point_count &&
      source.predicted_speed_mps.size() == point_count &&
      source.v_ref.size() == point_count && std::isfinite(ego.frenet.d) &&
      std::isfinite(source.committed_attack_follow_target_d_m);
  if (!source_shape_valid) {
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.desired_path_trackable = false;
    candidate.pure_pursuit_command_trackable = false;
    candidate.reject_reason = "attack_follow_current_d_hold_invalid_source";
    return candidate;
  }

  candidate.planned_target_d_m = ego.frenet.d;
  for (std::size_t i = 0U; i < point_count; ++i) {
    if (!std::isfinite(source.s[i])) {
      candidate.pass_target_corridor_valid = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.desired_path_trackable = false;
      candidate.pure_pursuit_command_trackable = false;
      candidate.reject_reason = "attack_follow_current_d_hold_invalid_source";
      return candidate;
    }
    const auto point = frame_.frenetToCartesian(source.s[i], ego.frenet.d);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw)) {
      candidate.pass_target_corridor_valid = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.desired_path_trackable = false;
      candidate.pure_pursuit_command_trackable = false;
      candidate.reject_reason = "attack_follow_current_d_hold_invalid_frame";
      return candidate;
    }
    candidate.d[i] = ego.frenet.d;
    candidate.x[i] = point.x;
    candidate.y[i] = point.y;
    candidate.yaw[i] = point.yaw;
  }

  const double check_distance_m = source.longitudinal_offsets_m.back();
  const CorridorPreflightResult corridor =
      attackFollowTargetCorridorReachable(ego, ego.frenet.d, check_distance_m);
  candidate.pass_target_corridor_valid = corridor.valid;
  candidate.corridor_min_margin_m = corridor.min_margin_m;
  const LateralTrackabilityResult tracking = sampledLateralProfileTrackability(
      candidate, ego, check_distance_m,
      [&ego](double) { return ego.frenet.d; });
  candidate.desired_path_trackable = tracking.desired_path_trackable;
  candidate.pure_pursuit_command_trackable =
      tracking.pure_pursuit_command_trackable;
  candidate.controller_tracking_profile_valid =
      candidate.pass_target_corridor_valid && tracking.valid();
  if (!candidate.pass_target_corridor_valid) {
    candidate.reject_reason = "pass_target_unreachable";
  } else if (!candidate.controller_tracking_profile_valid) {
    candidate.reject_reason = "untrackable_lateral_profile";
  }
  return candidate;
}

CandidateTrajectory
CandidateBuilder::makeAttackFollowInnerBandDiagnosticVariant(
    const CandidateTrajectory &source, const EgoState &ego, double terminal_d_m,
    double requested_shift_distance_m) const {
  CandidateTrajectory candidate = source;
  candidate.safety_evaluated = false;
  candidate.feasible = false;
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
  // shadow候補に実行可能なATTACK_FOLLOW identityを与えない。
  candidate.attack_follow_safe_lateral_hold = false;
  candidate.attack_follow_opponent_collision_current_d_hold = false;
  candidate.attack_follow_opponent_collision_inward_connector = false;

  const std::size_t point_count = source.t.size();
  const bool source_shape_valid =
      source.type == CandidateType::FOLLOW && point_count >= 4U &&
      source.longitudinal_offsets_m.size() == point_count &&
      source.s.size() == point_count && source.d.size() == point_count &&
      source.x.size() == point_count && source.y.size() == point_count &&
      source.yaw.size() == point_count &&
      source.predicted_speed_mps.size() == point_count &&
      source.v_ref.size() == point_count && std::isfinite(ego.frenet.d) &&
      std::isfinite(terminal_d_m) &&
      std::isfinite(source.committed_attack_follow_target_d_m) &&
      !source.longitudinal_offsets_m.empty() &&
      std::isfinite(source.longitudinal_offsets_m.back()) &&
      source.longitudinal_offsets_m.back() > 1.0e-6;
  if (!source_shape_valid) {
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.desired_path_trackable = false;
    candidate.pure_pursuit_command_trackable = false;
    candidate.reject_reason = "attack_follow_inner_band_invalid_source";
    return candidate;
  }

  const double source_distance_m = source.longitudinal_offsets_m.back();
  const bool explicit_shift_requested =
      std::isfinite(requested_shift_distance_m);
  const double shift_distance_m =
      explicit_shift_requested
          ? std::clamp(requested_shift_distance_m, 1.0e-6, source_distance_m)
          : source_distance_m;
  if (explicit_shift_requested &&
      (requested_shift_distance_m <= 1.0e-6 ||
       requested_shift_distance_m > source_distance_m + 1.0e-9)) {
    candidate.pass_target_corridor_valid = false;
    candidate.controller_tracking_profile_valid = false;
    candidate.desired_path_trackable = false;
    candidate.pure_pursuit_command_trackable = false;
    candidate.reject_reason = "attack_follow_inner_band_invalid_shift";
    return candidate;
  }
  for (std::size_t i = 0U; i < point_count; ++i) {
    if (!std::isfinite(source.s[i]) ||
        !std::isfinite(source.longitudinal_offsets_m[i])) {
      candidate.pass_target_corridor_valid = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.desired_path_trackable = false;
      candidate.pure_pursuit_command_trackable = false;
      candidate.reject_reason = "attack_follow_inner_band_invalid_source";
      return candidate;
    }
    const double ratio = std::clamp(
        source.longitudinal_offsets_m[i] / shift_distance_m, 0.0, 1.0);
    const double probe_d_m =
        ego.frenet.d + (terminal_d_m - ego.frenet.d) * smootherstep(ratio);
    const auto point = frame_.frenetToCartesian(source.s[i], probe_d_m);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw)) {
      candidate.pass_target_corridor_valid = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.desired_path_trackable = false;
      candidate.pure_pursuit_command_trackable = false;
      candidate.reject_reason = "attack_follow_inner_band_invalid_frame";
      return candidate;
    }
    candidate.d[i] = probe_d_m;
    candidate.x[i] = point.x;
    candidate.y[i] = point.y;
    candidate.yaw[i] = point.yaw;
  }

  const CorridorPreflightResult corridor =
      attackFollowTargetCorridorReachable(ego, terminal_d_m, shift_distance_m);
  candidate.pass_target_corridor_valid = corridor.valid;
  candidate.corridor_min_margin_m = corridor.min_margin_m;
  const LateralTrackabilityResult tracking =
      lateralCorrectionProfileTrackability(candidate, ego, terminal_d_m,
                                           shift_distance_m);
  candidate.desired_path_trackable = tracking.desired_path_trackable;
  candidate.pure_pursuit_command_trackable =
      tracking.pure_pursuit_command_trackable;
  candidate.controller_tracking_profile_valid =
      candidate.pass_target_corridor_valid && tracking.valid();
  if (!candidate.pass_target_corridor_valid) {
    candidate.reject_reason = "pass_target_unreachable";
  } else if (!candidate.controller_tracking_profile_valid) {
    candidate.reject_reason = "untrackable_lateral_profile";
  }
  return candidate;
}

CandidateTrajectory CandidateBuilder::makeAttackFollowInwardConnectorVariant(
    const CandidateTrajectory &source, const EgoState &ego, double terminal_d_m,
    double shift_distance_m) const {
  CandidateTrajectory candidate = makeAttackFollowInnerBandDiagnosticVariant(
      source, ego, terminal_d_m, shift_distance_m);
  // generic FOLLOWとしてtransportされないようspecial hold側へ閉じ、
  // Coreの同周期SafetyEvaluator・wall・trackability全成立まで専用
  // execution flagはfalseのままにする。
  candidate.attack_follow_safe_lateral_hold = true;
  candidate.attack_follow_opponent_collision_current_d_hold = false;
  candidate.attack_follow_opponent_collision_inward_connector = false;
  candidate.planned_target_d_m = terminal_d_m;
  return candidate;
}

// 入力: 候補速度契約、評価距離、任意のd(ds) sampler。
// 出力: desired path自身とactive PP commandを分離した追従性結果。
// 処理概要: 最大5 cm刻みでoffset pathの総曲率と、実行系と同じ
// rear-axle/lookahead式のraw steering commandを独立に評価する。認可側は
// 両方のANDだけを使い、入力・設定・sample異常は両方falseへ閉じる。
CandidateBuilder::LateralTrackabilityResult
CandidateBuilder::sampledLateralProfileTrackability(
    const CandidateTrajectory &candidate, const EgoState &ego,
    double check_distance_m,
    const std::function<double(double)> &sample_d_at_distance) const {
  if (!std::isfinite(ego.frenet.s) || !std::isfinite(check_distance_m) ||
      check_distance_m <= 0.0 || !sample_d_at_distance ||
      candidate.v_ref.empty() ||
      candidate.v_ref.size() != candidate.predicted_speed_mps.size()) {
    return {};
  }
  const double wheelbase_m = config_.attack_follow_tracking_wheelbase_m;
  const double max_angle_rad = config_.attack_follow_max_steering_angle_rad;
  const double steering_gain = config_.attack_follow_steering_tire_angle_gain;
  const double max_output_rate_radps =
      config_.attack_follow_max_steering_rate_radps;
  const double rate_reserve_ratio =
      config_.attack_follow_steering_rate_reserve_ratio;
  const double lookahead_distance_m =
      config_.lateral_override_lookahead_min_distance_m;
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
      !std::isfinite(max_angle_rad) || max_angle_rad <= 0.0 ||
      max_angle_rad >= 1.5707963267948966 || !std::isfinite(steering_gain) ||
      steering_gain <= 0.0 || !std::isfinite(max_output_rate_radps) ||
      max_output_rate_radps <= 0.0 || !std::isfinite(rate_reserve_ratio) ||
      rate_reserve_ratio <= 0.0 || rate_reserve_ratio > 1.0 ||
      !std::isfinite(lookahead_distance_m) || lookahead_distance_m <= 1.0e-3) {
    return {};
  }
  const double max_raw_rate_radps =
      (max_output_rate_radps / steering_gain) * rate_reserve_ratio;
  double evaluated_speed_mps = 0.0;
  for (std::size_t i = 0U; i < candidate.v_ref.size(); ++i) {
    if (!std::isfinite(candidate.v_ref[i]) ||
        !std::isfinite(candidate.predicted_speed_mps[i])) {
      return {};
    }
    evaluated_speed_mps = std::max({evaluated_speed_mps, candidate.v_ref[i],
                                    candidate.predicted_speed_mps[i]});
  }

  const std::size_t requested_samples = static_cast<std::size_t>(
      std::ceil(check_distance_m / kLateralCorrectionTrackabilitySpacingM));
  if (requested_samples > kLateralCorrectionTrackabilityMaxSamples) {
    return {};
  }
  const std::size_t sample_count = std::max<std::size_t>(3U, requested_samples);
  const double sample_step_m =
      check_distance_m / static_cast<double>(sample_count);
  if (!std::isfinite(sample_step_m) || sample_step_m <= 1.0e-5) {
    return {};
  }

  std::vector<double> sampled_d;
  sampled_d.reserve(sample_count + 1U);
  for (std::size_t i = 0U; i <= sample_count; ++i) {
    const double distance_m = sample_step_m * static_cast<double>(i);
    const double d = sample_d_at_distance(distance_m);
    if (!std::isfinite(d)) {
      return {};
    }
    sampled_d.push_back(d);
  }

  constexpr double kMinPathMetric = 1.0e-3;
  constexpr double kMinSegmentM = 1.0e-5;
  std::vector<double> sampled_x;
  std::vector<double> sampled_y;
  sampled_x.reserve(sample_count + 1U);
  sampled_y.reserve(sample_count + 1U);
  LateralTrackabilityResult result{true, true};
  constexpr double kCurvatureDerivativeWindowM = 0.05;
  // v4距離軸のd列はPP側で線形補間される。5 cmの隣接差分だけで2階微分を
  // 取ると、滑らかな元profileでも各horizon knotを無限大に近い曲率jumpと
  // 誤認する。車体/制御が解像できる25 cm窓で局所傾きと曲率を評価しつつ、
  // 1 m級の鋸歯状変形は同じ窓内で確実にrejectする。
  const double lateral_derivative_window_m =
      std::min(0.25, check_distance_m / 3.0);
  if (!std::isfinite(lateral_derivative_window_m) ||
      lateral_derivative_window_m <= 1.0e-3) {
    return {};
  }
  double previous_desired_steering_rad = 0.0;
  double previous_desired_metric = 0.0;
  bool have_previous_desired = false;
  for (std::size_t i = 0U; i <= sample_count; ++i) {
    double d_prime = 0.0;
    double d_second = 0.0;
    const double distance_m = sample_step_m * static_cast<double>(i);
    const double h = lateral_derivative_window_m;
    if (distance_m < h) {
      const double local_h = std::min(h, (check_distance_m - distance_m) / 3.0);
      if (!std::isfinite(local_h) || local_h <= 1.0e-3) {
        return {};
      }
      const double f0 = sampled_d[i];
      const double f1 = sample_d_at_distance(distance_m + local_h);
      const double f2 = sample_d_at_distance(distance_m + 2.0 * local_h);
      const double f3 = sample_d_at_distance(distance_m + 3.0 * local_h);
      d_prime =
          (-11.0 * f0 + 18.0 * f1 - 9.0 * f2 + 2.0 * f3) / (6.0 * local_h);
      d_second = (2.0 * f0 - 5.0 * f1 + 4.0 * f2 - f3) / (local_h * local_h);
    } else if (distance_m > check_distance_m - h) {
      const double local_h = std::min(h, distance_m / 3.0);
      if (!std::isfinite(local_h) || local_h <= 1.0e-3) {
        return {};
      }
      const double f0 = sampled_d[i];
      const double f1 = sample_d_at_distance(distance_m - local_h);
      const double f2 = sample_d_at_distance(distance_m - 2.0 * local_h);
      const double f3 = sample_d_at_distance(distance_m - 3.0 * local_h);
      d_prime = (11.0 * f0 - 18.0 * f1 + 9.0 * f2 - 2.0 * f3) / (6.0 * local_h);
      d_second = (2.0 * f0 - 5.0 * f1 + 4.0 * f2 - f3) / (local_h * local_h);
    } else {
      const double before_d = sample_d_at_distance(distance_m - h);
      const double after_d = sample_d_at_distance(distance_m + h);
      d_prime = (after_d - before_d) / (2.0 * h);
      d_second = (after_d - 2.0 * sampled_d[i] + before_d) / (h * h);
    }
    const double s = frame_.wrapS(ego.frenet.s + distance_m);
    const auto reference = frame_.interpolate(s);
    const double reference_curvature = reference.kappa;
    const double curvature_before =
        frame_.interpolate(s - kCurvatureDerivativeWindowM).kappa;
    const double curvature_after =
        frame_.interpolate(s + kCurvatureDerivativeWindowM).kappa;
    if (!std::isfinite(d_prime) || !std::isfinite(d_second) ||
        !std::isfinite(reference_curvature) ||
        !std::isfinite(curvature_before) || !std::isfinite(curvature_after) ||
        !std::isfinite(reference.yaw)) {
      return {};
    }
    const double d = sampled_d[i];
    const double tangent_scale = 1.0 - reference_curvature * d;
    const double metric_sq = tangent_scale * tangent_scale + d_prime * d_prime;
    if (!std::isfinite(metric_sq) ||
        metric_sq < kMinPathMetric * kMinPathMetric) {
      return {};
    }
    const double metric = std::sqrt(metric_sq);
    const double reference_curvature_derivative =
        (curvature_after - curvature_before) /
        (2.0 * kCurvatureDerivativeWindowM);
    const double curvature_numerator =
        reference_curvature * tangent_scale * tangent_scale +
        tangent_scale * d_second +
        reference_curvature_derivative * d * d_prime +
        2.0 * reference_curvature * d_prime * d_prime;
    const double desired_curvature = curvature_numerator / (metric_sq * metric);
    const double desired_steering_rad =
        std::atan(wheelbase_m * desired_curvature);
    if (!std::isfinite(desired_steering_rad)) {
      return {};
    }
    if (std::abs(desired_steering_rad) > max_angle_rad) {
      result.desired_path_trackable = false;
    }
    if (have_previous_desired) {
      const double path_segment_m =
          sample_step_m * 0.5 * (previous_desired_metric + metric);
      if (!std::isfinite(path_segment_m) || path_segment_m <= kMinSegmentM) {
        return {};
      }
      const double desired_steering_rate_radps =
          std::abs(desired_steering_rad - previous_desired_steering_rad) /
          path_segment_m * evaluated_speed_mps;
      if (!std::isfinite(desired_steering_rate_radps)) {
        return {};
      }
      if (desired_steering_rate_radps > max_raw_rate_radps + 1.0e-9) {
        result.desired_path_trackable = false;
      }
    }
    previous_desired_steering_rad = desired_steering_rad;
    previous_desired_metric = metric;
    have_previous_desired = true;
    const auto point = frame_.frenetToCartesian(s, d);
    const double path_yaw = reference.yaw + std::atan2(d_prime, tangent_scale);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(path_yaw)) {
      return {};
    }
    sampled_x.push_back(point.x);
    sampled_y.push_back(point.y);
  }

  // desired pathの局所曲率をそのままcommandと見なさず、実PPと同じ
  // rear-axle/lookahead式で評価する。curvature adaptiveで短くなり得る
  // 最短lookaheadを使い、active launchで0のfeed-forwardは加えない。
  // PPがこの周期に消費するのは、現在control poseから見た1 commandである。
  // 有限horizonを未来位置から繰り返し消費すると、終端に近い仮想位置で
  // lookahead点が背後へ回る偽の大舵角を作る。将来全区間の舵角・舵角速度は
  // 上のdesired-path検査で担保し、ここではactive PPと同じ現在姿勢・後輪中心・
  // 終端fallbackから実際にpublish直後に出るcommandだけを独立検査する。
  if (!std::isfinite(ego.x) || !std::isfinite(ego.y) ||
      !std::isfinite(ego.yaw)) {
    return {};
  }
  const double rear_x = ego.x - 0.5 * wheelbase_m * std::cos(ego.yaw);
  const double rear_y = ego.y - 0.5 * wheelbase_m * std::sin(ego.yaw);
  std::size_t lookahead_index = 0U;
  double lookahead_chord_m = 0.0;
  for (std::size_t j = 1U; j <= sample_count; ++j) {
    const double chord_m =
        std::hypot(sampled_x[j] - rear_x, sampled_y[j] - rear_y);
    if (!std::isfinite(chord_m)) {
      return {};
    }
    lookahead_index = j;
    lookahead_chord_m = chord_m;
    if (chord_m >= lookahead_distance_m) {
      break;
    }
  }
  if (lookahead_index == 0U || !std::isfinite(lookahead_chord_m) ||
      lookahead_chord_m <= 1.0e-3) {
    result.pure_pursuit_command_trackable = false;
    return result;
  }
  const double applied_lookahead_m =
      std::min(lookahead_distance_m, lookahead_chord_m);
  const double alpha_unwrapped_rad =
      std::atan2(sampled_y[lookahead_index] - rear_y,
                 sampled_x[lookahead_index] - rear_x) -
      ego.yaw;
  const double alpha_rad =
      std::atan2(std::sin(alpha_unwrapped_rad), std::cos(alpha_unwrapped_rad));
  const double raw_steering_rad =
      std::atan2(2.0 * wheelbase_m * std::sin(alpha_rad), applied_lookahead_m);
  const double output_steering_rad = steering_gain * raw_steering_rad;
  if (!std::isfinite(output_steering_rad)) {
    return {};
  }
  if (std::abs(output_steering_rad) > max_angle_rad) {
    result.pure_pursuit_command_trackable = false;
  }
  return result;
}

// 入力: PASS開始時の自車状態、最終目標d、目標へ横移動する距離[m]。
// 出力: 開始から目標到達までの名目d列が全てs依存安全回廊内ならtrue。
// 処理概要: publish/SafetyEvaluatorのhorizon外で回廊が狭くなる場合も、
// PASS開始前に検出して短い予測区間だけの誤許可を防ぐ。
CandidateBuilder::CorridorPreflightResult
CandidateBuilder::passTargetCorridorReachable(const EgoState &ego,
                                              double target_d,
                                              double shift_distance_m) const {
  CorridorPreflightResult result;
  if (!std::isfinite(ego.frenet.s) || !std::isfinite(ego.frenet.d) ||
      !std::isfinite(target_d) || !std::isfinite(shift_distance_m) ||
      shift_distance_m <= 0.0) {
    return result;
  }
  result.min_margin_m = std::numeric_limits<double>::infinity();
  constexpr double kCheckSpacingM = 0.25;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(shift_distance_m / kCheckSpacingM));
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double ratio =
        static_cast<double>(i) /
        static_cast<double>(std::max<std::size_t>(1U, sample_count));
    const double distance_m = shift_distance_m * ratio;
    const double d =
        ego.frenet.d + (target_d - ego.frenet.d) * smootherstep(ratio);
    const auto bounds =
        frame_.corridorBounds(frame_.wrapS(ego.frenet.s + distance_m),
                              config_.d_min_m, config_.d_max_m);
    const double lower_d = bounds.d_min + config_.min_wall_margin_m;
    const double upper_d = bounds.d_max - config_.min_wall_margin_m;
    const double margin_m = std::min(d - lower_d, upper_d - d);
    result.min_margin_m = std::min(result.min_margin_m, margin_m);
    if (!std::isfinite(lower_d) || !std::isfinite(upper_d) ||
        lower_d > upper_d || d < lower_d || d > upper_d) {
      return result;
    }
  }
  result.valid = true;
  return result;
}

CandidateBuilder::CorridorPreflightResult
CandidateBuilder::attackFollowTargetCorridorReachable(
    const EgoState &ego, double target_d, double shift_distance_m) const {
  CorridorPreflightResult result;
  if (!std::isfinite(ego.frenet.s) || !std::isfinite(ego.frenet.d) ||
      !std::isfinite(target_d) || !std::isfinite(shift_distance_m) ||
      shift_distance_m <= 0.0) {
    return result;
  }
  result.min_margin_m = std::numeric_limits<double>::infinity();
  constexpr double kCheckSpacingM = 0.25;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(shift_distance_m / kCheckSpacingM));
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double ratio =
        static_cast<double>(i) /
        static_cast<double>(std::max<std::size_t>(1U, sample_count));
    const double distance_m = shift_distance_m * ratio;
    const double d =
        ego.frenet.d + (target_d - ego.frenet.d) * smootherstep(ratio);
    const auto bounds =
        frame_.corridorBounds(frame_.wrapS(ego.frenet.s + distance_m),
                              config_.d_min_m, config_.d_max_m);
    const double lower_d = bounds.d_min + config_.min_wall_margin_m;
    const double upper_d = bounds.d_max - config_.min_wall_margin_m;
    const double margin_m = std::min(d - lower_d, upper_d - d);
    result.min_margin_m = std::min(result.min_margin_m, margin_m);
    if (!std::isfinite(lower_d) || !std::isfinite(upper_d) ||
        lower_d > upper_d || d < lower_d || d > upper_d) {
      return result;
    }
  }
  result.valid = true;
  return result;
}

// 入力: 局所PASSプロファイルと開始時の自車状態。
// 出力: 現在位置からfull-offset到達点まで名目profileが安全回廊内ならtrue。
// 処理概要: localized_latchedは通常PASSと異なるmarkerを使うため、同じ0.25 m
// 刻みで実際にpublishする横移動を確認し、開始前のpreflightを省略しない。
CandidateBuilder::CorridorPreflightResult
CandidateBuilder::localizedPassTargetCorridorReachable(
    const LocalizedLateralProfile &profile, const EgoState &ego) const {
  CorridorPreflightResult result;
  if (!profile.active || !std::isfinite(profile.anchor_s_m) ||
      !std::isfinite(profile.full_offset_start_s_m) ||
      !std::isfinite(profile.merge_end_s_m) || !std::isfinite(ego.frenet.s) ||
      !std::isfinite(ego.frenet.d)) {
    return result;
  }
  const double current_unwrapped_s =
      profileUnwrappedS(profile, ego.frenet.s, frame_.length());
  // full-offsetだけでなく保持・mergeまでraw profileを検査する。publish時の
  // clampを根拠に、将来の壁寄りmergeを安全と誤認しない。
  const double target_distance_m =
      std::max(0.0, profile.merge_end_s_m - current_unwrapped_s);
  result.min_margin_m = std::numeric_limits<double>::infinity();
  constexpr double kCheckSpacingM = 0.25;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(target_distance_m / kCheckSpacingM));
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double ratio =
        static_cast<double>(i) /
        static_cast<double>(std::max<std::size_t>(1U, sample_count));
    const double distance_m = target_distance_m * ratio;
    const double s = frame_.wrapS(ego.frenet.s + distance_m);
    const double d = localizedProfileRawD(profile, ego, s, distance_m);
    const auto bounds =
        frame_.corridorBounds(s, config_.d_min_m, config_.d_max_m);
    const double lower_d = bounds.d_min + config_.min_wall_margin_m;
    const double upper_d = bounds.d_max - config_.min_wall_margin_m;
    const double margin_m = std::min(d - lower_d, upper_d - d);
    result.min_margin_m = std::min(result.min_margin_m, margin_m);
    if (!std::isfinite(lower_d) || !std::isfinite(upper_d) ||
        lower_d > upper_d || d < lower_d || d > upper_d) {
      return result;
    }
  }
  result.valid = true;
  return result;
}

// 入力: ラッチ済み局所回避プロファイル、自車状態、評価対象s、現在からの距離ds。
// 出力: 壁マージン内にクランプした横オフセットd。
// 処理概要:
// 固定された局所プロファイルに、現在自車dへ連続接続する補正を短距離だけ重ねる。
double
CandidateBuilder::localizedProfileD(const LocalizedLateralProfile &profile,
                                    const EgoState &ego, double s,
                                    double ds) const {
  const double raw_d = localizedProfileRawD(profile, ego, s, ds);
  const auto bounds =
      frame_.corridorBounds(s, config_.d_min_m, config_.d_max_m);
  const double lower_d = bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = bounds.d_max - config_.min_wall_margin_m;
  return std::clamp(raw_d, lower_d, upper_d);
}

// 入力: ラッチ済み局所回避プロファイル、自車状態、評価対象s、現在からの距離ds。
// 出力: 物理回廊clamp前の局所横オフセットd。
// 処理概要: PASS開始前の到達性検査では、clampで見かけ上回廊内にした値ではなく
// 本来publishしようとするprofileを検査するために使う。
double
CandidateBuilder::localizedProfileRawD(const LocalizedLateralProfile &profile,
                                       const EgoState &ego, double s,
                                       double ds) const {
  static_cast<void>(s);
  // 認可済みPASSで実車が計画目標より既に外側へ到達した場合、対象/chainを
  // 抜き切る前に短距離で内側へ引き戻さない。現在dを保持した別形状として
  // corridor preflightと全相手SafetyEvaluatorへ通し、壁余裕や相手余裕が
  // 無ければ通常どおり不成立へ閉じる。PASS完了後だけ名目mergeを再開する。
  double outermost_target_d_m = profile.target_d_m;
  for (const auto &waypoint : profile.chain_waypoints) {
    if (!std::isfinite(waypoint.target_d_m)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (profile.pass_type == CandidateType::PASS_LEFT) {
      outermost_target_d_m =
          std::max(outermost_target_d_m, waypoint.target_d_m);
    } else if (profile.pass_type == CandidateType::PASS_RIGHT) {
      outermost_target_d_m =
          std::min(outermost_target_d_m, waypoint.target_d_m);
    }
  }
  constexpr double kOutwardHoldToleranceM = 1.0e-6;
  const bool already_outside_authorized_target =
      !profile.pass_complete_confirmed && std::isfinite(ego.frenet.d) &&
      std::isfinite(outermost_target_d_m) &&
      ((profile.pass_type == CandidateType::PASS_LEFT &&
        ego.frenet.d > outermost_target_d_m + kOutwardHoldToleranceM) ||
       (profile.pass_type == CandidateType::PASS_RIGHT &&
        ego.frenet.d < outermost_target_d_m - kOutwardHoldToleranceM));
  if (already_outside_authorized_target) {
    return ego.frenet.d;
  }
  const double current_unwrapped_s =
      profileUnwrappedS(profile, ego.frenet.s, frame_.length());
  const double nominal_current =
      nominalLocalizedProfileDAtUnwrappedS(profile, current_unwrapped_s);
  // 予測点はwrapped sから再推定せず、候補が持つ現在からの物理前進距離を
  // 加える。半周・一周を越えるhorizonでもprofileを開始位置へ巻き戻さない。
  const double nominal =
      nominalLocalizedProfileDAtUnwrappedS(profile, current_unwrapped_s + ds);
  // 現在dへの連続補正は毎周期必要だが、固定6 m先まで残すと
  // 目標へ近づくたびに収束点も逃げる。名目full-offset点と
  // SafetyEvaluatorの楕円進入点の早い方を固定期限にする。
  double correction_deadline_s_m = profile.full_offset_start_s_m;
  if (std::isfinite(profile.target_s_m)) {
    correction_deadline_s_m =
        std::min(correction_deadline_s_m,
                 profile.target_s_m - longitudinalEllipseClearance(config_));
  }
  const double remaining_to_deadline_m =
      correction_deadline_s_m - current_unwrapped_s;
  const double configured_correction_distance_m =
      std::max(1.0, config_.localized_avoidance_start_before_target_m);
  // 期限直前でも先頭点は現在dに連続させる。その結果が
  // 楕円内へ入るなら、後段SafetyEvaluatorが実際のd列をrejectする。
  const double correction_distance =
      std::max(1.0, std::min(configured_correction_distance_m,
                             std::max(0.0, remaining_to_deadline_m)));
  const double correction_ratio = smootherstep(ds / correction_distance);
  const double correction =
      (ego.frenet.d - nominal_current) * (1.0 - correction_ratio);
  return nominal + correction;
}

// 入力: ラッチ済み局所回避プロファイルと評価対象s。
// 出力: 補正前の名目横オフセットd。
// 処理概要:
// 回避開始、最大オフセット保持、マージ終了の4区間でpiecewiseにdを返す。
double CandidateBuilder::nominalLocalizedProfileD(
    const LocalizedLateralProfile &profile, double s) const {
  if (!profile.active || !std::isfinite(profile.anchor_s_m)) {
    return 0.0;
  }
  const double unwrapped_s = profileUnwrappedS(profile, s, frame_.length());
  return nominalLocalizedProfileDAtUnwrappedS(profile, unwrapped_s);
}

// 入力: ラッチ済み局所profileと連続s。
// 出力: 周回境界で巻き戻らない補正前の横オフセットd。
// 処理概要: marker/waypointはすべて同じunwrapped軸なので、候補の物理dsから
// 得た連続sをそのままpiecewise profileへ適用する。
double CandidateBuilder::nominalLocalizedProfileDAtUnwrappedS(
    const LocalizedLateralProfile &profile, double unwrapped_s) const {
  if (!profile.active || !std::isfinite(profile.anchor_s_m) ||
      !std::isfinite(unwrapped_s)) {
    return 0.0;
  }
  const double start_d = profile.start_d_m;
  const bool staged_chain = !profile.chain_waypoints.empty();
  const double target_d =
      staged_chain && std::isfinite(profile.chain_waypoints.front().target_d_m)
          ? profile.chain_waypoints.front().target_d_m
          : profile.target_d_m;
  if (unwrapped_s <= profile.avoid_start_s_m) {
    return start_d;
  }
  if (unwrapped_s <= profile.full_offset_start_s_m) {
    const double distance = std::max(1.0e-3, profile.full_offset_start_s_m -
                                                 profile.avoid_start_s_m);
    return interpolate(start_d, target_d,
                       (unwrapped_s - profile.avoid_start_s_m) / distance);
  }
  if (staged_chain) {
    double previous_target_s_m = profile.chain_waypoints.front().target_s_m;
    double previous_target_d_m = target_d;
    if (unwrapped_s <= previous_target_s_m) {
      return previous_target_d_m;
    }
    const double ellipse_clearance_m = longitudinalEllipseClearance(config_);
    for (std::size_t i = 1U; i < profile.chain_waypoints.size(); ++i) {
      const auto &waypoint = profile.chain_waypoints[i];
      if (!std::isfinite(waypoint.target_s_m) ||
          !std::isfinite(waypoint.target_d_m)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      // 先行対象の中心を通過してから、次対象の安全楕円へ入る手前までに
      // 同じPASS側の次dへ滑らかに接続する。target IDは先行車のまま、
      // 外側への単調移動だけを許し、車間で中心へmergeしない。
      const double transition_start_s_m = previous_target_s_m;
      const double transition_end_s_m =
          waypoint.target_s_m - ellipse_clearance_m;
      if (unwrapped_s <= transition_end_s_m) {
        const double distance_m =
            std::max(1.0e-3, transition_end_s_m - transition_start_s_m);
        return interpolate(previous_target_d_m, waypoint.target_d_m,
                           (unwrapped_s - transition_start_s_m) / distance_m);
      }
      if (unwrapped_s <= waypoint.target_s_m) {
        return waypoint.target_d_m;
      }
      previous_target_s_m = waypoint.target_s_m;
      previous_target_d_m = waypoint.target_d_m;
    }
    // 固定targetを抜き切る前は、将来horizonにも中心方向mergeを一切載せない。
    // PP/MPCのlookaheadがfull_offset_endより先を参照しても、同じPASS側または
    // chainで必要な外側だけを保持する。完了後はCoreが現在egoからRECOVERYを
    // 再生成・再評価するため、過去markerを使った途中mergeには依存しない。
    if (!profile.pass_complete_confirmed) {
      return previous_target_d_m;
    }
    if (unwrapped_s <= profile.full_offset_end_s_m) {
      return previous_target_d_m;
    }
    if (unwrapped_s <= profile.merge_end_s_m) {
      const double distance =
          std::max(1.0e-3, profile.merge_end_s_m - profile.full_offset_end_s_m);
      return interpolate(previous_target_d_m, 0.0,
                         (unwrapped_s - profile.full_offset_end_s_m) /
                             distance);
    }
    return 0.0;
  }
  if (!profile.pass_complete_confirmed) {
    return target_d;
  }
  if (unwrapped_s <= profile.full_offset_end_s_m) {
    return target_d;
  }
  if (unwrapped_s <= profile.merge_end_s_m) {
    const double distance =
        std::max(1.0e-3, profile.merge_end_s_m - profile.full_offset_end_s_m);
    return interpolate(target_d, 0.0,
                       (unwrapped_s - profile.full_offset_end_s_m) / distance);
  }
  return 0.0;
}

// 入力: Frenet横位置d。
// 出力: 左右壁マージンのうち小さい方の余裕[m]。負なら安全コリドー外。
// 処理概要: 候補や現在位置が壁側へ寄りすぎていないかを1値で扱えるようにする。
double CandidateBuilder::wallClearance(double s, double d) const {
  const auto bounds =
      frame_.corridorBounds(s, config_.d_min_m, config_.d_max_m);
  const double lower_d = bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = bounds.d_max - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

} // namespace overtake_planner
