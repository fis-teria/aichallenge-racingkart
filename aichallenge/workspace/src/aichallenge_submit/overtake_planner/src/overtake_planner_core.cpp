#include "overtake_planner/overtake_planner_core.hpp"

#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/cartesian_trackability_evaluator.hpp"
#include "overtake_planner/overtake_supervisor_v2.hpp"
#include "overtake_planner/planner_output_builder.hpp"
#include "overtake_planner/pp_core_exact_snapshot.hpp"
#include "overtake_planner/state_lattice_shadow_adapter.hpp"
#include "overtake_planner/v2_localized_pass_profile_builder.hpp"
#include "state_lattice_overtake_planner/shadow_geometry_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>

namespace overtake_planner {

namespace {

bool isWallRejectReason(const std::string &reason) {
  return reason == "wall_margin" || reason == "wall_footprint_margin" ||
         reason == "invalid_wall_footprint_config" ||
         reason == "invalid_wall_footprint_projection";
}

StateLatticeShadowCandidateMetrics
stateLatticeShadowMetrics(const CandidateTrajectory &candidate) {
  StateLatticeShadowCandidateMetrics metrics;
  metrics.safety_evaluated = candidate.safety_evaluated;
  metrics.feasible = candidate.feasible;
  metrics.wall_rejected = isWallRejectReason(candidate.reject_reason);
  metrics.wall_clearance_m = candidate.corridor_min_margin_m;
  metrics.cbf_min_margin = candidate.min_safety_margin;
  metrics.cbf_slack = candidate.cbf_slack;
  metrics.cbf_blocking_opponent_id = candidate.blocking_opponent_id;
  metrics.cbf_blocking_time_sec = candidate.blocking_time_sec;
  metrics.pure_pursuit_required_arc_m =
      candidate.required_controller_spatial_horizon_m;
  metrics.pure_pursuit_available_arc_m =
      candidate.longitudinal_offsets_m.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : candidate.longitudinal_offsets_m.back();
  metrics.deadline_required_arc_m =
      candidate.pass_transition_deadline.evaluated
          ? candidate.pass_transition_deadline.required_transition_m
          : candidate.required_pass_transition_m;
  metrics.deadline_available_arc_m =
      candidate.pass_transition_deadline.evaluated
          ? candidate.pass_transition_deadline.available_deadline_m
          : candidate.available_pass_transition_deadline_m;
  metrics.deadline_slack_m =
      candidate.pass_transition_deadline.evaluated
          ? candidate.pass_transition_deadline.deadline_slack_m
          : metrics.deadline_available_arc_m - metrics.deadline_required_arc_m;
  metrics.first_reject_reason = candidate.reject_reason;
  return metrics;
}

class SnapshotFnv1a {
public:
  void addBytes(const void *data, std::size_t size) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (std::size_t index = 0U; index < size; ++index) {
      value_ ^= static_cast<std::uint64_t>(bytes[index]);
      value_ *= 1099511628211ULL;
    }
  }

  template <typename T> void addScalar(const T &value) {
    addBytes(&value, sizeof(value));
  }

  void addString(const std::string &value) {
    const std::uint64_t size = static_cast<std::uint64_t>(value.size());
    addScalar(size);
    addBytes(value.data(), value.size());
  }

  std::uint64_t value() const { return value_; }

private:
  std::uint64_t value_{1469598103934665603ULL};
};

std::uint64_t
stateLatticeSnapshotHash(std::uint64_t cycle, double now_sec,
                         const EgoState &ego,
                         const std::vector<OpponentState> &opponents,
                         const std::vector<PredictedOpponent> &predictions) {
  SnapshotFnv1a hash;
  hash.addScalar(cycle);
  hash.addScalar(now_sec);
  hash.addScalar(ego.stamp_sec);
  hash.addScalar(ego.x);
  hash.addScalar(ego.y);
  hash.addScalar(ego.yaw);
  hash.addScalar(ego.v);
  hash.addScalar(ego.frenet.s);
  hash.addScalar(ego.frenet.d);
  hash.addScalar(ego.valid);
  hash.addScalar(static_cast<std::uint64_t>(opponents.size()));
  for (const auto &opponent : opponents) {
    hash.addString(opponent.id);
    hash.addScalar(opponent.stamp_sec);
    hash.addScalar(opponent.x);
    hash.addScalar(opponent.y);
    hash.addScalar(opponent.v);
    hash.addScalar(opponent.vx);
    hash.addScalar(opponent.vy);
    hash.addScalar(opponent.frenet.s);
    hash.addScalar(opponent.frenet.d);
    hash.addScalar(opponent.valid);
  }
  hash.addScalar(static_cast<std::uint64_t>(predictions.size()));
  for (const auto &prediction : predictions) {
    hash.addString(prediction.id);
    const auto add_vector = [&hash](const std::vector<double> &values) {
      hash.addScalar(static_cast<std::uint64_t>(values.size()));
      for (const double value : values) {
        hash.addScalar(value);
      }
    };
    add_vector(prediction.t);
    add_vector(prediction.x);
    add_vector(prediction.y);
    add_vector(prediction.s);
    add_vector(prediction.d);
  }
  return hash.value();
}

// 入力: 現在mode。
// 出力: 左追い越し準備/実行中ならtrue。
// 処理概要: 左側pass gapの喪失判定や優先候補維持に使う。
bool isLeftPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_LEFT;
}

// 入力: 現在mode。
// 出力: 右追い越し準備/実行中ならtrue。
// 処理概要: 右側pass gapの喪失判定や優先候補維持に使う。
bool isRightPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

// 入力: 現在mode。
// 出力: 左右どちらかの追い越し準備/実行中ならtrue。
// 処理概要:
// 追い越し中だけ中心線基準ではなく、追い越し目標基準の横誤差も許容する。
bool isAnyPassMode(BehaviorMode mode) {
  return isLeftPassMode(mode) || isRightPassMode(mode);
}

// 入力: ループ上のfrom/to位置と周長[m]。
// 出力: 半周以内へ折り返した符号付き距離[m]。
// 処理概要: start gridでは僅かに後方の隣接車も扱うため、常に前向き距離を
// 返すFrenetFrame::deltaSとは分ける。微小後退を1周進捗と誤認するのも防ぐ。
double signedDeltaS(double from_s, double to_s, double track_length_m) {
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

// 入力: stale周期へ保存候補の横offset列。
// 出力: 全点が同一のcurrent-d holdとして再利用できる時だけtrue。
// 処理概要: 距離軸を失ったv3で中心移動列を再生せず、point-index差の影響を
// 受けない定数holdだけを許可する。
bool isConstantLateralHold(const std::vector<double> &offsets_m) {
  if (offsets_m.empty() || !std::isfinite(offsets_m.front())) {
    return false;
  }
  return std::all_of(offsets_m.begin(), offsets_m.end(),
                     [anchor_m = offsets_m.front()](double offset_m) {
                       return std::isfinite(offset_m) &&
                              std::abs(offset_m - anchor_m) <= 1.0e-6;
                     });
}

// 入力: 保存候補、現在ego、参照線。
// 出力: Nodeが実publishするv4と同じ現在ego.s + ds基準へ再構築できればtrue。
// 処理概要: ACK待ち中に保存した旧絶対s/x/yをSafetyEvaluatorへ再利用せず、
// 実際のtyped trajectoryと同じ空間位置へ全点を張り直す。
bool rebaseCandidateToCurrentEgo(CandidateTrajectory &candidate,
                                 const EgoState &ego,
                                 const FrenetFrame &frame) {
  const std::size_t point_count = candidate.d.size();
  if (!ego.valid || !std::isfinite(ego.frenet.s) || point_count == 0U ||
      candidate.longitudinal_offsets_m.size() != point_count ||
      candidate.t.size() != point_count ||
      candidate.v_ref.size() != point_count ||
      candidate.predicted_speed_mps.size() != point_count) {
    return false;
  }
  candidate.s.resize(point_count);
  candidate.x.resize(point_count);
  candidate.y.resize(point_count);
  candidate.yaw.resize(point_count);
  for (std::size_t i = 0U; i < point_count; ++i) {
    const double ds_m = candidate.longitudinal_offsets_m[i];
    const double d_m = candidate.d[i];
    if (!std::isfinite(ds_m) || ds_m < 0.0 || !std::isfinite(d_m) ||
        (i > 0U && ds_m + 1.0e-9 < candidate.longitudinal_offsets_m[i - 1U])) {
      return false;
    }
    const double s_m = frame.wrapS(ego.frenet.s + ds_m);
    const auto point = frame.frenetToCartesian(s_m, d_m);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw)) {
      return false;
    }
    candidate.s[i] = s_m;
    candidate.x[i] = point.x;
    candidate.y[i] = point.y;
    candidate.yaw[i] = point.yaw;
  }
  return true;
}

// 入力: 保存候補と現在ego。
// 出力: 保存候補の縦予測が現在速度から開始している時だけtrue。
// 処理概要: 絶対sだけを張り直しても、ACK待ち中にego速度が変われば古い
// s(t)/predicted_speed(t)を現在時刻の安全評価へ流用できない。停止constraintの
// 微小な速度揺れもFloat32 wireの世代更新で明示的に取り直す。
bool candidateStartsFromCurrentEgoSpeed(const CandidateTrajectory &candidate,
                                        const EgoState &ego) {
  if (!ego.valid || !std::isfinite(ego.v) ||
      !std::isfinite(candidate.longitudinal_initial_measured_speed_mps)) {
    return false;
  }
  constexpr double kStartSpeedToleranceMps = 1.0e-3;
  return std::abs(candidate.longitudinal_initial_measured_speed_mps -
                  std::max(0.0, ego.v)) <= kStartSpeedToleranceMps;
}

// 入力: 2つのPASS候補。
// 出力: Nodeがv4 wireへ載せるrelative d/v/dsがFloat32単位で同じ時だけtrue。
// 処理概要:
// publish直前整形で実payloadが変わったかをNodeと同じ量子化で判定する。
std::string releaseCandidateWireChangeReason(const CandidateTrajectory &lhs,
                                             const CandidateTrajectory &rhs) {
  const auto same_float_vector = [](const std::vector<double> &a,
                                    const std::vector<double> &b) {
    if (a.size() != b.size()) {
      return false;
    }
    for (std::size_t i = 0U; i < a.size(); ++i) {
      if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
          static_cast<float>(a[i]) != static_cast<float>(b[i])) {
        return false;
      }
    }
    return true;
  };
  if (lhs.type != rhs.type) {
    return "type";
  }
  if (!same_float_vector(lhs.d, rhs.d)) {
    return "lateral";
  }
  if (!same_float_vector(lhs.v_ref, rhs.v_ref)) {
    return "speed";
  }
  if (!same_float_vector(lhs.longitudinal_offsets_m,
                         rhs.longitudinal_offsets_m)) {
    return "spatial_axis";
  }
  return {};
}

bool releaseCandidateWireSemanticallyEqual(const CandidateTrajectory &lhs,
                                           const CandidateTrajectory &rhs) {
  return releaseCandidateWireChangeReason(lhs, rhs).empty();
}

// 入力: publish用HOLDやPASS recovery文脈を含むBlockedInfo。
// 出力: 通常ラインへの中心復帰候補を生成するための評価用コピー。
// 処理概要: 未認可中のcurrent-d/anchor HOLDだけを解除し、壁・他車・制動・
// pass reauthorization lockoutなどの物理的安全文脈はそのまま保持する。
BlockedInfo centeringEvaluationBlockedInfo(const BlockedInfo &blocked_info) {
  BlockedInfo centering_blocked = blocked_info;
  centering_blocked.reentry_hold_active = false;
  centering_blocked.start_grid_follow_hold_lateral = false;
  centering_blocked.start_grid_uncommitted_hold_active = false;
  centering_blocked.early_stationary_parallel_pass_hold_lateral = false;
  centering_blocked.parallel_follow_hold_lateral = false;
  centering_blocked.braking_follow_hold_lateral = false;
  centering_blocked.attack_follow_hold_pass_side = false;
  return centering_blocked;
}

// 入力: 現在modeと左右pass可否を含むBlockedInfo。
// 出力: 現在走っている側のpass gapが失われたならtrue。
// 処理概要:
// 追い越し中に走行中の側が塞がれた場合、YIELD/復帰候補を追加するトリガにする。
bool currentPassGapLost(BehaviorMode mode, const BlockedInfo &blocked_info) {
  if ((mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
       mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT) &&
      blocked_info.maneuver_transaction_incomplete) {
    // PREPAREはまだ横移動を実行していない。race arm前のearly-stationary
    // probeとarm後のstart-grid分類で側が訂正された場合は、古いmode名ではなく
    // SafetyEvaluatorへ渡す最新transaction側の成立性を見る。OVERTAKE中は下の
    // mode側を使い、実行中の左右固定を崩さない。
    if (blocked_info.maneuver_transaction_pass_type ==
        CandidateType::PASS_LEFT) {
      return blocked_info.pass_left_candidate_generated
                 ? !blocked_info.pass_left_candidate_feasible
                 : !blocked_info.can_pass_left;
    }
    if (blocked_info.maneuver_transaction_pass_type ==
        CandidateType::PASS_RIGHT) {
      return blocked_info.pass_right_candidate_generated
                 ? !blocked_info.pass_right_candidate_feasible
                 : !blocked_info.can_pass_right;
    }
  }
  if (isLeftPassMode(mode)) {
    return blocked_info.pass_left_candidate_generated
               ? !blocked_info.pass_left_candidate_feasible
               : !blocked_info.can_pass_left;
  }
  if (isRightPassMode(mode)) {
    return blocked_info.pass_right_candidate_generated
               ? !blocked_info.pass_right_candidate_feasible
               : !blocked_info.can_pass_right;
  }
  return false;
}

// 入力: planner設定とBlockedInfo。
// 出力: raw
// parallel観測ではなく、即時の横方向安全余裕として扱うべき近接並走ならtrue。
// 処理概要: parallel_side_candidateは広めに拾う診断値なので、SAFE_STOPやgeneric
// recovery gateでは横楕円または横並び幅に入る近接車両だけを強いリスクにする。
bool hasCloseParallelSideRisk(const PlannerConfig &config,
                              const BlockedInfo &blocked_info) {
  if (!blocked_info.parallel_side_candidate ||
      blocked_info.parallel_side_index < 0 ||
      !std::isfinite(blocked_info.parallel_side_delta_s) ||
      !std::isfinite(blocked_info.parallel_side_delta_d)) {
    return false;
  }
  const double s_limit = std::max(0.0, config.side_by_side_s_m);
  const double d_limit = std::max(std::max(0.0, config.side_margin_m),
                                  std::max(0.0, config.safety_ellipse_b_m));
  return std::abs(blocked_info.parallel_side_delta_s) <= s_limit &&
         std::abs(blocked_info.parallel_side_delta_d) <= d_limit;
}

// 入力: planner設定とparallel side観測。
// 出力: CBF楕円へ入る前に減速して後方へ譲るべき、狭い条件の接近並走ならtrue。
// 処理概要:
// parallel_side_candidateは診断用に広く拾うため、前方・同方向・接近中・
// side_by_side判定距離内に絞ってのみYIELDへ昇格させる。
bool hasStrictParallelYieldRisk(const PlannerConfig &config,
                                const BlockedInfo &blocked_info) {
  if (!blocked_info.parallel_side_candidate ||
      blocked_info.parallel_side_index < 0 ||
      !std::isfinite(blocked_info.parallel_side_delta_s) ||
      !std::isfinite(blocked_info.parallel_side_delta_d) ||
      !std::isfinite(blocked_info.parallel_side_rel_v)) {
    return false;
  }
  if (blocked_info.parallel_side_direction_known &&
      !blocked_info.parallel_side_same_direction) {
    return false;
  }
  return blocked_info.parallel_side_delta_s > config.side_yield_s_m &&
         blocked_info.parallel_side_delta_s <= config.side_by_side_s_m &&
         std::abs(blocked_info.parallel_side_delta_d) <=
             config.parallel_side_margin_m &&
         blocked_info.parallel_side_rel_v > config.dv_block_threshold_mps;
}

// 入力: planner設定とBlockedInfo。
// 出力: 横に少し離れた低速前方車へ中心復帰で近づき得るならtrue。
// 処理概要: 同一コリドーfrontには分類されないが、停止車列や2台目が前方に
// 残るケースはgeneric recovery gateで長期評価する。
bool hasSlowForwardParallelRecoveryRisk(const PlannerConfig &config,
                                        const BlockedInfo &blocked_info) {
  if (!blocked_info.parallel_side_candidate ||
      blocked_info.parallel_side_index < 0 ||
      !std::isfinite(blocked_info.parallel_side_delta_s) ||
      !std::isfinite(blocked_info.parallel_side_delta_d) ||
      !std::isfinite(blocked_info.parallel_side_rel_v)) {
    return false;
  }
  const double s_limit =
      std::max(std::max(0.0, config.parallel_side_s_m),
               std::max(0.0, config.slow_obstacle_chain_distance_m));
  return blocked_info.parallel_side_delta_s > 0.0 &&
         blocked_info.parallel_side_delta_s <= s_limit &&
         std::abs(blocked_info.parallel_side_delta_d) <=
             std::max(0.0, config.parallel_side_margin_m) &&
         blocked_info.parallel_side_rel_v >
             std::max(0.0, config.dv_block_threshold_mps);
}

// 入力: BlockedInfo。
// 出力: FOLLOW_BLOCKEDで車間形成すべき対象があるならtrue。
// 処理概要:
// 通常front blockedに加え、parallel
// FOLLOW候補をFOLLOW候補生成と状態優先に使う。
bool hasFollowBlockedTarget(const BlockedInfo &blocked_info) {
  return blocked_info.blocked || blocked_info.start_grid_target_active ||
         blocked_info.parallel_follow_candidate ||
         blocked_info.braking_follow_active;
}

// PASS側のgapが失われた後でも、front分類が一時的にblocked=falseへ揺れる
// ケースがある。SafetyEvaluatorへ通せる同方向の前方車が残っているなら、
// その周期に現d保持FOLLOWを作り、RECOVERY/SAFE_STOPへ直結させない。
// ここでは候補を許可せず、あくまでFOLLOW候補を評価する入口だけを作る。
bool hasForwardFollowFallbackTarget(const BlockedInfo &blocked_info) {
  const bool front_target =
      blocked_info.nearest_index >= 0 && !blocked_info.nearest_id.empty() &&
      std::isfinite(blocked_info.front_delta_s) &&
      blocked_info.front_delta_s > 0.0 && blocked_info.front_direction_known &&
      blocked_info.front_same_direction;
  const bool parallel_target =
      blocked_info.parallel_side_index >= 0 &&
      !blocked_info.parallel_side_id.empty() &&
      std::isfinite(blocked_info.parallel_side_delta_s) &&
      blocked_info.parallel_side_delta_s > 0.0 &&
      (!blocked_info.parallel_side_direction_known ||
       blocked_info.parallel_side_same_direction);
  const bool committed_transaction_target =
      blocked_info.maneuver_transaction_incomplete &&
      blocked_info.maneuver_target_latched &&
      blocked_info.maneuver_target_observed &&
      blocked_info.maneuver_target_fresh &&
      blocked_info.maneuver_chain_tail_observed &&
      blocked_info.maneuver_chain_tail_index >= 0 &&
      !blocked_info.maneuver_chain_tail_id.empty() &&
      std::isfinite(blocked_info.maneuver_chain_tail_relative_s_m) &&
      blocked_info.maneuver_chain_tail_relative_s_m > 0.0;
  return front_target || parallel_target || committed_transaction_target;
}

// 入力: 現周期のparallel-side観測と相手一覧。
// 出力: 通常のparallel FOLLOW帯の外でも、PASS不成立時だけcurrent-d FOLLOWを
//       再評価してよい、freshな同方向前方targetならtrue。
// 処理概要:
// parallel_sideは広い診断帯なので、そのままFOLLOWへ昇格させない。通常front、
// start-grid、未完了PASS transactionを除外し、実観測ID・stamp・方向が揃う
// 相手だけをCandidateBuilderへ渡す。候補の採否は下流の全相手/wall/CBF/PP
// spatial-horizon評価に委ね、PASSがfeasibleな周期ではscoreで選択不能にする。
bool canRecheckParallelSideAsCurrentDFollow(
    const PlannerConfig &config, double now_sec,
    const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) {
  // wide parallel-side診断を通常FOLLOWの代替として昇格するのは、既に
  // SafetyEvaluatorの縦安全楕円へ近接した前方相手だけに限定する。
  // 遠方のparallel-side観測までFOLLOWへ変えると、parallel_followの設定範囲外
  // に対する従来のFREE_RUN契約を破る。
  const double h = std::max(0.0, config.min_ellipse_h);
  const double close_parallel_s_m =
      std::max(0.0, config.safety_ellipse_a_m) * std::sqrt(1.0 + h);
  if (!config.parallel_follow_enabled || blocked_info.blocked ||
      blocked_info.nearest_index >= 0 ||
      blocked_info.start_grid_target_active ||
      blocked_info.maneuver_transaction_incomplete ||
      blocked_info.parallel_side_index < 0 ||
      static_cast<std::size_t>(blocked_info.parallel_side_index) >=
          opponents.size() ||
      blocked_info.parallel_side_id.empty() ||
      !std::isfinite(blocked_info.parallel_side_delta_s) ||
      blocked_info.parallel_side_delta_s <= 0.0 ||
      blocked_info.parallel_side_delta_s > close_parallel_s_m ||
      !blocked_info.parallel_side_direction_known ||
      !blocked_info.parallel_side_same_direction) {
    return false;
  }
  const auto &target =
      opponents[static_cast<std::size_t>(blocked_info.parallel_side_index)];
  return target.valid && target.id == blocked_info.parallel_side_id &&
         inputTimestampFresh(now_sec, target.stamp_sec,
                             config.opponent_stale_time_sec,
                             config.input_future_stamp_tolerance_sec);
}

// 入力: planner設定と現在ラッチしているPASS対象。
// 出力: ラッチ対象そのものを安全楕円の後方まで抜けたならtrue。
// 処理概要:
// chain tail（次の低速車）を含むPASSでは、target_idを保持したまま次車へ
// 近づくため、chain tailの完了判定だけでは先行対象の通過を判別できない。
// ここでは対象自身の相対s/dがSafetyEvaluatorの楕円外へ出たことだけを確認し、
// その後に限ってchain tail向けFOLLOWへhandoffできるようにする。
bool maneuverTargetPassed(const PlannerConfig &config,
                          const BlockedInfo &blocked_info) {
  if (!blocked_info.maneuver_target_latched ||
      !blocked_info.maneuver_target_observed ||
      !std::isfinite(blocked_info.maneuver_target_relative_s_m) ||
      !std::isfinite(blocked_info.maneuver_target_relative_d_m)) {
    return false;
  }
  const double h = std::max(0.0, config.min_ellipse_h);
  const double s_clearance =
      std::max(0.0, config.safety_ellipse_a_m) * std::sqrt(1.0 + h);
  const double d_clearance =
      std::max(0.0, config.safety_ellipse_b_m) * std::sqrt(1.0 + h);
  return blocked_info.maneuver_target_relative_s_m <= -s_clearance &&
         std::abs(blocked_info.maneuver_target_relative_d_m) >= d_clearance;
}

// parallel_sideは通常はPASS/side診断用であり、FOLLOW対象へ自動昇格させない。
// ただし同側PASSが物理的に成立せず、SafetyEvaluatorへ攻め追従を試す時だけ、
// CandidateBuilderが使うparallel_follow欄へ明示的に束ねる。
BlockedInfo followFallbackInfo(const BlockedInfo &blocked_info) {
  if (blocked_info.nearest_index >= 0) {
    BlockedInfo fallback = blocked_info;
    fallback.parallel_follow_hold_lateral = true;
    return fallback;
  }
  if (blocked_info.parallel_side_index < 0) {
    return blocked_info;
  }
  BlockedInfo fallback = blocked_info;
  fallback.parallel_follow_candidate = true;
  fallback.parallel_follow_hold_lateral = true;
  fallback.parallel_follow_index = blocked_info.parallel_side_index;
  fallback.parallel_follow_id = blocked_info.parallel_side_id;
  fallback.parallel_follow_delta_s = blocked_info.parallel_side_delta_s;
  fallback.parallel_follow_delta_d = blocked_info.parallel_side_delta_d;
  fallback.parallel_follow_rel_v = blocked_info.parallel_side_rel_v;
  fallback.parallel_follow_s_dot_mps = blocked_info.parallel_side_s_dot_mps;
  fallback.parallel_follow_direction_known =
      blocked_info.parallel_side_direction_known;
  fallback.parallel_follow_same_direction =
      blocked_info.parallel_side_same_direction;
  return fallback;
}

// 入力: planner設定、現在mode、BlockedInfo。
// 出力: 回避不能ならSAFE_STOP候補まで評価すべき危険文脈ならtrue。
// 処理概要:
// 広めのparallel観測だけで停止判定へ入ると通常走行まで低速化するため、
// 前方閉塞、近接横並び、未来譲り、追い越しgap喪失に絞ってfail-closedへ渡す。
bool needsSafeStopFallbackCheck(const PlannerConfig &config, BehaviorMode mode,
                                const BlockedInfo &blocked_info) {
  if (!config.safe_stop_enabled) {
    return false;
  }
  return blocked_info.blocked || blocked_info.start_grid_target_active ||
         blocked_info.early_stationary_parallel_pass_target ||
         blocked_info.parallel_follow_candidate || blocked_info.side_by_side ||
         blocked_info.future_yield_required ||
         blocked_info.stationary_front_obstacle ||
         hasCloseParallelSideRisk(config, blocked_info) ||
         currentPassGapLost(mode, blocked_info);
}

// 入力: planner設定、現在mode、BlockedInfo。
// 出力: FREE/FOLLOW系の中心復帰にも長期reentry gateを掛けるべきならtrue。
// 処理概要: 壁寄り補正や通常追従だけではgeneric gateを起動しない。低速前走車へ
// 横断し得る時、横並び、未来譲り、追い越し中のgap喪失だけを保持対象にする。
bool needsGenericRecoveryGate(const PlannerConfig &config, BehaviorMode mode,
                              const BlockedInfo &blocked_info) {
  return blocked_info.start_grid_lateral_release_pending ||
         blocked_info.side_by_side || blocked_info.corner_side_by_side ||
         blocked_info.future_side_by_side ||
         blocked_info.future_corner_side_by_side ||
         blocked_info.future_yield_required ||
         blocked_info.future_parallel_interaction ||
         blocked_info.stationary_front_obstacle ||
         blocked_info.slow_obstacle_chain_active ||
         blocked_info.front_vehicle_low_speed ||
         hasCloseParallelSideRisk(config, blocked_info) ||
         hasSlowForwardParallelRecoveryRisk(config, blocked_info) ||
         currentPassGapLost(mode, blocked_info);
}

// 入力: CandidateType。
// 出力: 左右PASS候補ならtrue。
// 処理概要: safe stop判定や候補集合の有無チェックでPASSだけを抽出する。
bool isPassCandidate(CandidateType type) {
  return type == CandidateType::PASS_LEFT || type == CandidateType::PASS_RIGHT;
}

// 入力: CandidateType。
// 出力: PASS以外の通常回避/追従候補ならtrue。
// 処理概要: PASSが使えない時に安全な代替候補があるかを調べる。
bool isFallbackCandidate(CandidateType type) {
  return type == CandidateType::FOLLOW || type == CandidateType::YIELD_BEHIND ||
         type == CandidateType::RECOVERY ||
         type == CandidateType::SIDE_BY_SIDE_KEEP;
}

// 入力: BehaviorMode。
// 出力: 横位置を安定化すべきmodeならtrue。
// 処理概要: YIELD/RECOVERY/SAFE_STOP/SPEED_GUARD中の横参照hold対象をまとめる。
bool isLateralStabilizationMode(BehaviorMode mode) {
  return mode == BehaviorMode::ABORT_RECOVERY ||
         mode == BehaviorMode::YIELD_BEHIND ||
         mode == BehaviorMode::SAFE_STOP || mode == BehaviorMode::SPEED_GUARD;
}

// 入力: 最終PlannerOutput。
// 出力: 横参照holdを発動するほどの危険文脈があるならtrue。
// 処理概要:
// コーナー横並び、未来譲り、速度ガードなど短周期で横目標を動かしたくない状態を集約する。
bool hasLateralStabilizationRisk(const PlannerOutput &output) {
  const auto &blocked = output.blocked_info;
  return blocked.corner_side_by_side || blocked.future_yield_required ||
         blocked.future_corner_side_by_side || blocked.future_outer_wall_risk ||
         output.speed_only_fallback_active ||
         output.wall_risk_speed_guard_active ||
         output.mpc_health_speed_guard_active ||
         output.recovery_speed_guard_active ||
         (output.reentry_gate.requested && !output.reentry_gate.permitted);
}

// 入力: publish直前まで安全再評価したPlannerOutput。
// 出力: ABORTの横列をsolver predictionとして許可してよい時だけtrue。
// 処理概要: ABORT_RECOVERYという状態名やreentry gateの閉鎖だけでは横solver
// 軌道を認可しない。freshな入力で全車両を評価し、中心復帰が相手との衝突余裕を
// 失うため現横位置の安全保持が必要な場合だけ、必須回避として明示する。
bool isMandatoryAbortLateralAvoidance(const PlannerOutput &output) {
  if (output.mode != BehaviorMode::ABORT_RECOVERY ||
      output.selected != CandidateType::RECOVERY || !output.active_override ||
      output.lateral_offsets.empty() ||
      output.published_lateral_safety_rejected ||
      !output.reentry_gate.requested || !output.reentry_gate.input_complete ||
      output.reentry_gate.permitted) {
    return false;
  }
  const std::string &reason = output.reentry_gate.reason;
  const bool blocked_reentry_is_collision_related =
      reason == "opponent_collision" ||
      reason == "reentry_margin_below_threshold" ||
      reason == "reentry_cbf_slack";
  const bool immediate_lateral_interaction =
      output.blocked_info.side_by_side ||
      output.blocked_info.corner_side_by_side ||
      output.blocked_info.future_side_by_side ||
      output.blocked_info.future_corner_side_by_side ||
      output.blocked_info.future_yield_required ||
      output.blocked_info.future_parallel_interaction ||
      output.blocked_info.stationary_front_obstacle ||
      output.blocked_info.slow_obstacle_chain_active;
  const bool matched_parallel_reentry_blocker =
      output.blocked_info.parallel_side_candidate &&
      !output.blocked_info.parallel_side_id.empty() &&
      output.blocked_info.parallel_side_id ==
          output.reentry_gate.blocking_vehicle_id;
  return blocked_reentry_is_collision_related &&
         (immediate_lateral_interaction || matched_parallel_reentry_blocker);
}

// 入力: publish直前まで安全再評価したPlannerOutputとplanner設定。
// 出力:
// なし。禁止区間でも安全許可済みの通常復帰横列を維持し、速度だけをcapする。
// 処理概要: permissionは新規PASSの開始条件であり、reentry gateを通過した
// RECOVERY横列を中心収束前に取り除く理由にはならない。横列を消すとABORTが
// 物理的に完了せず、再進入を繰り返すため、既存のSafetyEvaluator済み横列と
// より厳しい既存speed capを必ず保存する。
void capNoPassNormalRecoverySpeed(const PlannerConfig &config,
                                  PlannerOutput &output) {
  const bool no_pass_or_start_disallowed =
      !output.blocked_info.overtake_permission_allowed ||
      !output.blocked_info.straight_overtake_start_allowed;
  const bool normal_reentry =
      (output.mode == BehaviorMode::ABORT_RECOVERY ||
       (output.mode == BehaviorMode::SPEED_GUARD &&
        output.blocked_info.reentry_centering_authorized)) &&
      output.selected == CandidateType::RECOVERY && output.active_override &&
      output.reentry_gate.requested && output.reentry_gate.input_complete &&
      output.reentry_gate.permitted &&
      output.solver_horizon_intent == PlannerOutput::SolverHorizonIntent::NONE;
  if (!no_pass_or_start_disallowed || !normal_reentry) {
    return;
  }
  constexpr double kRaceSpeedCapMps = 10.0;
  const double speed_cap_mps =
      std::isfinite(config.normal_recovery_speed_only_v_max_mps) &&
              config.normal_recovery_speed_only_v_max_mps > 0.0
          ? std::min(kRaceSpeedCapMps,
                     config.normal_recovery_speed_only_v_max_mps)
          : kRaceSpeedCapMps;
  if (output.speed_caps.empty()) {
    output.speed_caps.assign(config.horizon_points, speed_cap_mps);
  } else {
    for (auto &speed_cap : output.speed_caps) {
      speed_cap = std::min(speed_cap_mps, speed_cap);
    }
  }
  const double applied_speed_cap_mps =
      *std::min_element(output.speed_caps.begin(), output.speed_caps.end());
  output.longitudinal_speed_cap_active = true;
  output.applied_speed_cap_mps = applied_speed_cap_mps;
  if (output.speed_cap_reason.empty()) {
    output.speed_cap_reason = "no_pass_normal_recovery_speed_cap";
  }
}

// 入力: PlannerConfig。
// 出力: 高速カーブ横参照holdへ入る曲率しきい値[1/m]。
// 処理概要: コーナー譲り設定を優先し、無ければ直線追い越しgateの曲率を使う。
double highSpeedCurveHoldEnterCurvature(const PlannerConfig &config) {
  if (config.corner_side_yield_curvature_m_inv > 0.0) {
    return config.corner_side_yield_curvature_m_inv;
  }
  return std::max(0.0, config.straight_overtake_max_curvature_m_inv);
}

// 入力: PlannerConfig。
// 出力: 高速カーブ横参照holdを解除する曲率しきい値[1/m]。
// 処理概要:
// 明示解除しきい値が無い場合、入場曲率の半分をヒステリシスとして使う。
double highSpeedCurveHoldReleaseCurvature(const PlannerConfig &config) {
  if (config.high_speed_curve_lateral_hold_release_curvature_m_inv >= 0.0) {
    return config.high_speed_curve_lateral_hold_release_curvature_m_inv;
  }
  return highSpeedCurveHoldEnterCurvature(config) * 0.5;
}

// 入力: 設定、自車、周辺リスク。
// 出力: ABORT解除後も高速カーブの現d保持を継続すべきならtrue。
// 処理概要:
// ABORTそのものを高速カーブ保持に使わない。既に安全に再合流を完了した後だけ、
// 既存の高速カーブ解除しきい値でSPEED_GUARDの現d保持を継続する。
bool shouldHoldPostAbortCurve(const PlannerConfig &config, const EgoState &ego,
                              const BlockedInfo &blocked_info) {
  if (!config.high_speed_curve_lateral_hold_enabled) {
    return false;
  }
  const double release_speed =
      config.high_speed_curve_lateral_hold_release_speed_mps >= 0.0
          ? config.high_speed_curve_lateral_hold_release_speed_mps
          : config.high_speed_curve_lateral_hold_min_speed_mps;
  const double curvature = std::max(blocked_info.corner_abs_curvature,
                                    blocked_info.future_abs_curvature);
  return ego.v > release_speed &&
         curvature > highSpeedCurveHoldReleaseCurvature(config);
}

// 入力: 設定。
// 出力: post-ABORTカーブholdに使う有限かつ保守的な速度上限[m/s]。
// 処理概要:
// 専用設定を基本にし、corner YIELDの上限が低い場合はそれを超えない。これにより
// 状態名だけSPEED_GUARDにして高速PASSを流す経路を作らない。
double postAbortCurveHoldSpeedCap(const PlannerConfig &config) {
  double speed_cap = config.post_abort_curve_hold_v_max_mps;
  if (std::isfinite(config.corner_yield_v_max_mps) &&
      config.corner_yield_v_max_mps > 0.0) {
    speed_cap = std::min(speed_cap, config.corner_yield_v_max_mps);
  }
  return speed_cap;
}

// 入力: 保持済みoffset配列と参照index。
// 出力: indexに対応する保持offset。範囲外は最後の値。
// 処理概要: 新しいhorizon長が変わっても保持中の横参照を安全に再利用する。
double heldOffsetAt(const std::vector<double> &offsets, std::size_t index) {
  if (offsets.empty()) {
    return 0.0;
  }
  return offsets[std::min(index, offsets.size() - 1)];
}

// 入力: 候補集合と候補種別predicate。
// 出力: predicateに合うfeasible候補が1つでもあればtrue。
// 処理概要: safe stopへ落とす前に、PASSやfallback候補の有無を確認する。
bool hasFeasibleCandidate(const std::vector<CandidateTrajectory> &candidates,
                          bool (*predicate)(CandidateType)) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [predicate](const CandidateTrajectory &candidate) {
                       return candidate.feasible && predicate(candidate.type);
                     });
}

// 入力: 選択候補とBlockedInfo。
// 出力: SAFE_STOP解除時に通常復帰へ使える候補ならtrue。
// 処理概要:
// 停止解除後に危険候補へ飛ばないよう、解放に使える候補種別を制限する。
bool isFeasibleReleaseCandidate(const CandidateTrajectory &selected,
                                const BlockedInfo &blocked_info) {
  if (!selected.feasible || selected.type == CandidateType::SAFE_STOP) {
    return false;
  }
  if (!blocked_info.blocked && !blocked_info.side_by_side &&
      !blocked_info.future_yield_required) {
    return selected.type == CandidateType::FASTEST ||
           selected.type == CandidateType::RECOVERY;
  }
  return isPassCandidate(selected.type) ||
         selected.type == CandidateType::FOLLOW ||
         selected.type == CandidateType::YIELD_BEHIND ||
         selected.type == CandidateType::RECOVERY;
}

// 入力: BehaviorMode。
// 出力: PREPARE_OVERTAKE_*ならtrue。
// 処理概要: PREPARE中だけPASS horizon publishを抑制する設定に使う。
bool isPrepareOvertakeMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT;
}

// 入力: PlannerConfig。
// 出力: PREPARE中にもPASS horizonをpublishする設定ならtrue。
// 処理概要: 内部PASS評価とMPCへ出すhorizonを分離するための切替を読む。
bool publishPassHorizonInPrepare(const PlannerConfig &config) {
  return config.pass_horizon_publish_mode != "overtake_only";
}

// 入力: planner設定、現在mode、ラッチ済み局所プロファイル。
// 出力: 現在の追い越し方向が目標にしているd。非PASS文脈ならNaN。
// 処理概要:
// 横ずれ判定で「中心から離れたこと」自体を異常扱いしないため、PASS目標dを参照する。
double passTargetForMode(const PlannerConfig &config, BehaviorMode mode,
                         const LocalizedLateralProfile &profile) {
  if (!isAnyPassMode(mode)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const bool prepare_mode = mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                            mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT;
  if (profile.active &&
      (prepare_mode ||
       (isLeftPassMode(mode) &&
        profile.pass_type == CandidateType::PASS_LEFT) ||
       (isRightPassMode(mode) &&
        profile.pass_type == CandidateType::PASS_RIGHT)) &&
      std::isfinite(profile.target_d_m)) {
    return profile.target_d_m;
  }
  if (isLeftPassMode(mode)) {
    return config.left_offset_m;
  }
  if (isRightPassMode(mode)) {
    return config.right_offset_m;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// 入力: planner設定、現在mode、自車、ラッチ済み局所プロファイル。
// 出力: large_lateral_error判定に使う横誤差[m]。
// 処理概要:
// 通常走行/復帰時は中心線基準。start-grid検出後、PASS、攻めFOLLOW中は
// 開始dから認可候補のPASS目標dまでを横遷移包絡として、その外へ出た量だけを
// 誤差にする。FSM遷移前の1周期だけ中心線誤差へ戻さない。
double lateralErrorForPassAwareFreeze(const PlannerConfig &config,
                                      BehaviorMode mode, const EgoState &ego,
                                      const LocalizedLateralProfile &profile) {
  if (!std::isfinite(ego.frenet.d)) {
    return std::numeric_limits<double>::infinity();
  }
  const bool prepared_transaction_context =
      profile.active &&
      (mode == BehaviorMode::FREE_RUN || mode == BehaviorMode::FOLLOW_BLOCKED ||
       // start-gridの停止holdがtracking horizon不足で一時SPEED_GUARDに
       // 落ちても、準備済みPASS包絡を中心線誤差へ読み替えない。ここで
       // large_lateral_errorを再armすると加速到達horizonを作れず、同じ
       // start dで永久停止する。実PASS未実行の候補承認は別の
       // pass_execution_committed契約で管理されるため、ABORT開始根拠にはしない。
       mode == BehaviorMode::SPEED_GUARD || isAnyPassMode(mode));
  if (!isAnyPassMode(mode) && !prepared_transaction_context) {
    return std::abs(ego.frenet.d);
  }

  double pass_start_d = 0.0;
  double pass_target_d = passTargetForMode(config, mode, profile);
  const bool prepare_mode = mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                            mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT;
  const bool matching_localized_profile =
      profile.active && (prepared_transaction_context || prepare_mode ||
                         (isLeftPassMode(mode) &&
                          profile.pass_type == CandidateType::PASS_LEFT) ||
                         (isRightPassMode(mode) &&
                          profile.pass_type == CandidateType::PASS_RIGHT));
  if (matching_localized_profile) {
    pass_start_d = profile.start_d_m;
    pass_target_d = profile.target_d_m;
  }
  if (!std::isfinite(pass_start_d) || !std::isfinite(pass_target_d)) {
    // 壊れたPASS契約を中心線誤差へ読み替えて継続しない。
    return std::numeric_limits<double>::infinity();
  }

  const double lower_d = std::min(pass_start_d, pass_target_d);
  const double upper_d = std::max(pass_start_d, pass_target_d);
  if (ego.frenet.d < lower_d) {
    return lower_d - ego.frenet.d;
  }
  if (ego.frenet.d > upper_d) {
    return ego.frenet.d - upper_d;
  }
  return 0.0;
}

// 入力: 候補集合とBlockedInfo。
// 出力: SAFE_STOP解除時に使える候補が1つでもあればtrue。
// 処理概要: SAFE_STOP release_readyの判定を候補集合全体に広げる。
bool hasFeasibleReleaseCandidate(
    const std::vector<CandidateTrajectory> &candidates,
    const BlockedInfo &blocked_info) {
  return std::any_of(candidates.begin(), candidates.end(),
                     [&blocked_info](const CandidateTrajectory &candidate) {
                       return isFeasibleReleaseCandidate(candidate,
                                                         blocked_info);
                     });
}

constexpr double kPublishedLateralTargetMemorySec = 0.5;
constexpr double kStartGraceMotionSpeedMps = 0.20;

double reentryCompletionLateralErrorM(const PlannerConfig &config) {
  return std::isfinite(config.reentry_completion_lateral_error_m) &&
                 config.reentry_completion_lateral_error_m > 0.0
             ? config.reentry_completion_lateral_error_m
             : 0.05;
}

double reentryCompletionRearmLateralErrorM(const PlannerConfig &config) {
  const double completion_m = reentryCompletionLateralErrorM(config);
  return std::isfinite(config.reentry_completion_rearm_lateral_error_m) &&
                 config.reentry_completion_rearm_lateral_error_m > completion_m
             ? config.reentry_completion_rearm_lateral_error_m
             : completion_m;
}

// 入力: 状態機械の現在/直前mode。
// 出力: 通常ラインへ戻る横断を開始し得るmodeならtrue。
// 処理概要: FREE_RUN/FOLLOW/SPEED_GUARDの横ずれを復帰gateに混ぜず、PASS後の
// MERGE/YIELD/SAFE_STOPとpublish再評価失敗だけを複数車両reentry評価の対象にする。
bool isReentrySourceMode(BehaviorMode mode) {
  return mode == BehaviorMode::MERGE_BACK ||
         mode == BehaviorMode::ABORT_RECOVERY ||
         mode == BehaviorMode::YIELD_BEHIND || mode == BehaviorMode::SAFE_STOP;
}

} // namespace

bool detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(
    const PlannerOutput &output) {
  if (!isMandatoryAbortLateralAvoidance(output) ||
      !output.blocked_info.reentry_hold_active ||
      !isConstantLateralHold(output.lateral_offsets) ||
      !std::isfinite(output.blocked_info.ego_lateral_offset_m) ||
      std::abs(output.lateral_offsets.front() -
               output.blocked_info.ego_lateral_offset_m) > 1.0e-4 ||
      !output.reentry_gate.input_complete || output.reentry_gate.permitted ||
      output.reentry_gate.blocking_vehicle_id.empty()) {
    return false;
  }
  const std::string &blocking_id = output.reentry_gate.blocking_vehicle_id;
  return blocking_id == output.blocked_info.parallel_side_id ||
         blocking_id == output.blocked_info.side_id ||
         blocking_id == output.blocked_info.stationary_front_id ||
         blocking_id == output.blocked_info.slow_obstacle_chain_id;
}

bool detail::isSafetyEvaluatedCurrentTransactionHoldDuringStop(
    const PlannerOutput &output) {
  const bool non_pass_hold = output.selected == CandidateType::FOLLOW ||
                             output.selected == CandidateType::YIELD_BEHIND ||
                             output.selected == CandidateType::RECOVERY ||
                             output.selected == CandidateType::SAFE_STOP;
  const auto &blocked = output.blocked_info;
  return non_pass_hold && output.active_override &&
         output.selected_lateral_profile_safety_verified &&
         output.lateral_stop_inputs_complete &&
         !output.published_lateral_safety_rejected &&
         blocked.maneuver_transaction_incomplete &&
         blocked.maneuver_transaction_safe_lateral_hold_active &&
         blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
         blocked.maneuver_target_fresh &&
         blocked.maneuver_chain_tail_observed &&
         !blocked.maneuver_target_id.empty() &&
         !blocked.maneuver_chain_tail_id.empty() &&
         isConstantLateralHold(output.lateral_offsets) &&
         std::isfinite(blocked.ego_lateral_offset_m) &&
         std::abs(output.lateral_offsets.front() -
                  blocked.ego_lateral_offset_m) <= 1.0e-4;
}

bool detail::isSafetyEvaluatedCurrentSafeStopLateralHold(
    const PlannerOutput &output) {
  return output.mode == BehaviorMode::SAFE_STOP &&
         output.selected == CandidateType::SAFE_STOP &&
         output.safe_stop_triggered && output.active_override &&
         output.selected_lateral_profile_safety_verified &&
         output.lateral_stop_inputs_complete &&
         !output.published_lateral_safety_rejected &&
         isConstantLateralHold(output.lateral_offsets) &&
         std::isfinite(output.blocked_info.ego_lateral_offset_m) &&
         std::abs(output.lateral_offsets.front() -
                  output.blocked_info.ego_lateral_offset_m) <= 1.0e-4;
}

// 入力: Frenet参照線とplanner設定。
// 出力: overtake planner coreのインスタンス。
// 処理概要:
// リスク解析、候補生成、安全評価、状態機械を同じ参照線/設定で動かすために初期化する。
OvertakePlannerCore::OvertakePlannerCore(FrenetFrame frame,
                                         PlannerConfig config)
    : frame_(std::move(frame)), config_(config), blocked_risk_(frame_, config_),
      future_side_risk_(frame_, config_, blocked_risk_),
      safety_(frame_, config), state_machine_(config),
      supervisor_v2_(config.supervisor_v2_abort_release_cycles,
                     config.reentry_safe_cycles,
                     config.supervisor_v2_pass_completion_cycles,
                     config.supervisor_v2_target_missing_hold_cycles,
                     config.supervisor_v2_tracking_unusable_hold_cycles) {}

void OvertakePlannerCore::rememberAuthorizedPassEnvelope(
    const CandidateTrajectory &candidate,
    const LocalizedLateralProfile &profile) {
  if (!profile.active || profile.target_id.empty() ||
      (candidate.type != CandidateType::PASS_LEFT &&
       candidate.type != CandidateType::PASS_RIGHT) ||
      candidate.type != profile.pass_type || candidate.d.empty() ||
      pass_reauthorization_lockout_active_) {
    return;
  }
  double min_d_m = std::numeric_limits<double>::infinity();
  double max_d_m = -std::numeric_limits<double>::infinity();
  const auto include = [&min_d_m, &max_d_m](double d_m) {
    if (std::isfinite(d_m)) {
      min_d_m = std::min(min_d_m, d_m);
      max_d_m = std::max(max_d_m, d_m);
    }
  };
  include(profile.start_d_m);
  include(profile.target_d_m);
  for (const double d_m : candidate.d) {
    include(d_m);
  }
  for (const auto &waypoint : profile.chain_waypoints) {
    include(waypoint.target_d_m);
  }
  if (!std::isfinite(min_d_m) || !std::isfinite(max_d_m) || min_d_m > max_d_m) {
    return;
  }
  const bool same_profile =
      authorized_pass_envelope_target_id_ == profile.target_id &&
      authorized_pass_envelope_pass_type_ == profile.pass_type &&
      std::isfinite(authorized_pass_envelope_min_d_m_) &&
      std::isfinite(authorized_pass_envelope_max_d_m_);
  authorized_pass_envelope_min_d_m_ =
      same_profile ? std::min(authorized_pass_envelope_min_d_m_, min_d_m)
                   : min_d_m;
  authorized_pass_envelope_max_d_m_ =
      same_profile ? std::max(authorized_pass_envelope_max_d_m_, max_d_m)
                   : max_d_m;
  authorized_pass_envelope_target_id_ = profile.target_id;
  authorized_pass_envelope_pass_type_ = profile.pass_type;
}

double OvertakePlannerCore::authorizedPassEnvelopeErrorM(double ego_d_m) const {
  if (!std::isfinite(ego_d_m) ||
      !std::isfinite(authorized_pass_envelope_min_d_m_) ||
      !std::isfinite(authorized_pass_envelope_max_d_m_) ||
      authorized_pass_envelope_min_d_m_ > authorized_pass_envelope_max_d_m_) {
    return std::numeric_limits<double>::infinity();
  }
  if (ego_d_m < authorized_pass_envelope_min_d_m_) {
    return authorized_pass_envelope_min_d_m_ - ego_d_m;
  }
  if (ego_d_m > authorized_pass_envelope_max_d_m_) {
    return ego_d_m - authorized_pass_envelope_max_d_m_;
  }
  return 0.0;
}

// 入力: 現在時刻、自車状態、相手車一覧、MPC health。
// 出力: MPC overrideとdebug情報を含むPlannerOutput。
// 処理概要:
// リスク判定、候補生成、安全評価、状態機械、publish用候補整形を1周期分実行する。
PlannerOutput
OvertakePlannerCore::update(double now_sec, const EgoState &ego,
                            const std::vector<OpponentState> &opponents,
                            const MpcHealthStatus &mpc_health,
                            const ReentryInputStatus &reentry_input,
                            const PurePursuitExactSnapshot *pp_exact_snapshot) {
  // デフォルトはMPCの元参照をそのまま使う。安全に判断できる時だけoverrideを有効化する。
  PlannerOutput output;
  StateLatticeShadowComparison state_lattice_shadow_comparison;
  state_lattice_shadow_comparison.requested =
      config_.trajectory_backend == "state_lattice_shadow";
  state_lattice_shadow_comparison.status_reason =
      state_lattice_shadow_comparison.requested
          ? "current_pass_candidate_not_generated"
          : "disabled";
  SupervisorV2Decision supervisor_v2_decision;
  ++supervisor_v2_cycle_sequence_;
  if (supervisor_v2_cycle_sequence_ == 0U) {
    ++supervisor_v2_cycle_sequence_;
  }
  output.lateral_offsets.assign(config_.horizon_points, 0.0);
  output.speed_caps.assign(config_.horizon_points, config_.v_passthrough_mps);

  if (!config_.enabled || frame_.empty()) {
    // planner無効化または参照線欠損時は、既存契約どおりoverrideを出さない。
    mode_ = BehaviorMode::FREE_RUN;
    safe_stop_trigger_count_ = 0;
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = false;
    reentry_phase_active_ = false;
    reentry_gate_permitted_ = false;
    generic_recovery_phase_active_ = false;
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    slow_front_exception_count_ = 0;
    slow_front_exception_id_.clear();
    unstarted_pass_released_target_id_.clear();
    early_stationary_parallel_pass_id_.clear();
    early_stationary_parallel_pass_count_ = 0;
    start_grid_target_id_.clear();
    start_grid_target_reselection_suppressed_ = false;
    start_grid_handoff_target_id_.clear();
    start_grid_target_first_seen_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_first_seen_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_lateral_release_pending_ = false;
    start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_release_pending_ = false;
    start_grid_tracking_release_confirmed_ = false;
    start_grid_tracking_release_cycles_ = 0;
    start_grid_tracking_release_target_id_.clear();
    start_grid_tracking_release_candidate_valid_ = false;
    start_grid_tracking_release_candidate_ = CandidateTrajectory{};
    start_grid_tracking_probe_cycles_ = 0;
    start_grid_tracking_probe_target_id_.clear();
    start_grid_tracking_probe_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_probe_target_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_last_target_stamp_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_candidate_valid_ = false;
    start_grid_tracking_probe_candidate_ = CandidateTrajectory{};
    start_grid_tracking_failed_candidate_valid_ = false;
    start_grid_tracking_failed_target_id_.clear();
    start_grid_tracking_failed_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_failed_candidate_ = CandidateTrajectory{};
    start_grid_tracking_failed_ego_yaw_rad_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_parallel_permission_prepare_id_.clear();
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    post_abort_curve_hold_active_ = false;
    gentle_curve_safe_pass_constraint_latched_ = false;
    gentle_curve_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    gentle_curve_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_no_pass_safe_pass_constraint_latched_ = false;
    stationary_no_pass_safe_pass_target_id_.clear();
    stationary_no_pass_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_no_pass_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    clearLocalizedLateralProfile();
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }
  if (!ego.valid) {
    // 制限PASSの開始dはfreshな自車位置にだけ紐づける。入力欠損中に以前の
    // anchorを保持して再開すると、安全評価対象と実状態がずれるため即clearする。
    gentle_curve_safe_pass_constraint_latched_ = false;
    gentle_curve_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    gentle_curve_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_no_pass_safe_pass_constraint_latched_ = false;
    stationary_no_pass_safe_pass_target_id_.clear();
    stationary_no_pass_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_no_pass_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    if (post_abort_curve_hold_active_) {
      // post-ABORT hold中にegoがstaleなら、以前の横列でPASS/通常走行へ
      // fall throughしない。復帰gateを再度閉じてABORTへ戻し、横列は出さず
      // watchdogが扱える低速capだけを残す。
      post_abort_curve_hold_active_ = false;
      reentry_clear_cycles_ = 0;
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      reentry_gate_permitted_ = false;
      mode_ = BehaviorMode::ABORT_RECOVERY;
      const double fallback_cap_mps = std::min(
          std::max(1.0e-3, config_.reentry_hold_v_max_mps),
          std::min(
              std::max(1.0e-3, config_.speed_only_fallback_v_max_mps),
              std::max(1.0e-3, config_.opponent_collision_fallback_v_max_mps)));
      output.mode = mode_;
      output.selected = CandidateType::RECOVERY;
      output.blocked_info.post_abort_curve_hold_active = false;
      output.blocked_info.reentry_hold_active = true;
      output.reentry_gate.requested = true;
      output.reentry_gate.permitted = false;
      output.reentry_gate.input_complete = false;
      output.reentry_gate.reason = "stale_ego";
      output.active_override = false;
      output.longitudinal_speed_cap_active = true;
      output.lateral_offsets.clear();
      output.speed_caps.assign(config_.horizon_points, fallback_cap_mps);
      output.applied_speed_cap_mps = fallback_cap_mps;
      output.speed_cap_reason = "post_abort_curve_hold_stale_ego_abort";
      output.reason = "post_abort_curve_hold_stale_ego_abort";
      return output;
    }
    // 復帰/lockout中に自己状態が欠けても、overrideを消して通常ラインへ
    // fall throughしない。最後に安全にpublishした横列を低速で維持する。
    const bool generic_recovery_context =
        config_.reentry_gate_enabled && generic_recovery_phase_active_ &&
        !reentry_lockout_active_ && !reentry_phase_active_;
    if (generic_recovery_context) {
      // generic横補正は、stale ego時にABORTやFREE_RUNへ切り替えない。直前に
      // 検証済みの現d保持列と低速capをそのまま出し、Node側watchdogと併走する。
      output.mode = BehaviorMode::SPEED_GUARD;
      output.selected = CandidateType::RECOVERY;
      output.blocked_info.reentry_hold_active = true;
      output.reentry_gate.requested = true;
      output.reentry_gate.permitted = false;
      output.reentry_gate.input_complete = false;
      output.reentry_gate.reason = "generic_recovery_stale_ego_hold";
      if (!generic_recovery_hold_offsets_.empty()) {
        output.active_override = true;
        output.lateral_offsets = generic_recovery_hold_offsets_;
        // 保存済み横列と現在周期の距離軸を混ぜない。stale holdは評価済み横列を
        // そのまま扱うlegacy v3へ明示的にfallbackする。
        output.longitudinal_offsets_m.clear();
        // stale egoでは、以前にlatency-onlyで許可した3 m/s holdを再利用しない。
        // 現在の相手配置を再評価できないため、常にhard fail-safe capへ落とす。
        const double hold_cap_mps =
            std::min(std::isfinite(generic_recovery_hold_speed_cap_mps_) &&
                             generic_recovery_hold_speed_cap_mps_ > 0.0
                         ? generic_recovery_hold_speed_cap_mps_
                         : std::max(1.0e-3, config_.reentry_hold_v_max_mps),
                     std::max(1.0e-3, config_.reentry_hold_v_max_mps));
        output.speed_caps.assign(output.lateral_offsets.size(), hold_cap_mps);
        output.target_lateral_offset_m = output.lateral_offsets.back();
        output.applied_speed_cap_mps = hold_cap_mps;
        output.speed_cap_reason = "generic_recovery_stale_ego_hold";
        output.reason = "generic_recovery_stale_ego_hold";
      } else {
        // 安全評価済み横列が無い時に現在d/中心dを捏造してpublishしない。
        // overrideは無効に保ち、既存Node/controllerのstale watchdogへ明示的に
        // 委譲する。ただしdebug/下流診断用の速度capは低速側に残す。
        const double fallback_cap_mps = std::min(
            std::max(1.0e-3, config_.reentry_hold_v_max_mps),
            std::min(
                std::max(1.0e-3, config_.safe_stop_v_mps),
                std::min(
                    std::max(1.0e-3, config_.speed_only_fallback_v_max_mps),
                    std::max(1.0e-3,
                             config_.opponent_collision_fallback_v_max_mps))));
        output.active_override = false;
        output.longitudinal_speed_cap_active = true;
        output.lateral_offsets.clear();
        output.longitudinal_offsets_m.clear();
        output.speed_caps.assign(config_.horizon_points, fallback_cap_mps);
        output.applied_speed_cap_mps = fallback_cap_mps;
        output.speed_cap_reason =
            "generic_recovery_stale_ego_no_safe_hold_watchdog";
        output.reason = "generic_recovery_stale_ego_no_safe_hold_watchdog";
      }
      return output;
    }
    const bool reentry_context =
        config_.reentry_gate_enabled &&
        (reentry_lockout_active_ || reentry_phase_active_);
    if (reentry_context) {
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      reentry_clear_cycles_ = 0;
      reentry_gate_permitted_ = false;
      mode_ = BehaviorMode::ABORT_RECOVERY;
      output.mode = mode_;
      output.selected = CandidateType::RECOVERY;
      output.blocked_info.reentry_hold_active = true;
      output.reentry_gate.requested = true;
      output.reentry_gate.permitted = false;
      output.reentry_gate.input_complete = false;
      output.reentry_gate.reason = "stale_ego";
      if (!last_published_lateral_offsets_.empty()) {
        output.active_override = true;
        output.lateral_offsets = last_published_lateral_offsets_;
        output.speed_caps.assign(
            output.lateral_offsets.size(),
            std::max(1.0e-3, config_.reentry_hold_v_max_mps));
        output.target_lateral_offset_m = output.lateral_offsets.back();
        output.applied_speed_cap_mps = config_.reentry_hold_v_max_mps;
        output.speed_cap_reason = "reentry_stale_ego_hold";
        output.reason = "reentry_stale_ego_hold";
      } else {
        // 初回の有効override前は保持すべき安全横列が無いため、値を捏造しない。
        // Node/controller側の既存stale watchdogへ明示的に委譲する。
        output.reason = "reentry_stale_ego_no_hold_reference";
      }
      return output;
    }

    // 復帰文脈でない自己状態欠損は従来どおりplanner overrideを無効化する。
    mode_ = BehaviorMode::FREE_RUN;
    safe_stop_trigger_count_ = 0;
    reentry_clear_cycles_ = 0;
    reentry_phase_active_ = false;
    reentry_gate_permitted_ = false;
    generic_recovery_phase_active_ = false;
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    slow_front_exception_count_ = 0;
    slow_front_exception_id_.clear();
    early_stationary_parallel_pass_id_.clear();
    early_stationary_parallel_pass_count_ = 0;
    start_grid_target_id_.clear();
    start_grid_target_reselection_suppressed_ = false;
    start_grid_handoff_target_id_.clear();
    start_grid_target_first_seen_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_first_seen_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_lateral_release_pending_ = false;
    start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_release_pending_ = false;
    start_grid_tracking_release_confirmed_ = false;
    start_grid_tracking_release_cycles_ = 0;
    start_grid_tracking_release_target_id_.clear();
    start_grid_tracking_release_candidate_valid_ = false;
    start_grid_tracking_release_candidate_ = CandidateTrajectory{};
    start_grid_tracking_probe_cycles_ = 0;
    start_grid_tracking_probe_target_id_.clear();
    start_grid_tracking_probe_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_probe_target_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_last_target_stamp_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_candidate_valid_ = false;
    start_grid_tracking_probe_candidate_ = CandidateTrajectory{};
    start_grid_tracking_failed_candidate_valid_ = false;
    start_grid_tracking_failed_target_id_.clear();
    start_grid_tracking_failed_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_failed_candidate_ = CandidateTrajectory{};
    start_grid_tracking_failed_ego_yaw_rad_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_parallel_permission_prepare_id_.clear();
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    post_abort_curve_hold_active_ = false;
    gentle_curve_safe_pass_constraint_latched_ = false;
    gentle_curve_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    clearLocalizedLateralProfile();
    output.mode = mode_;
    output.reason = "disabled_or_invalid";
    return output;
  }
  if (!std::isfinite(first_valid_update_sec_)) {
    first_valid_update_sec_ = now_sec;
    first_valid_update_s_m_ = ego.frenet.s;
  }
  if (!std::isfinite(first_motion_update_sec_) && std::isfinite(ego.v) &&
      ego.v >= kStartGraceMotionSpeedMps) {
    first_motion_update_sec_ = now_sec;
  }

  // 処理ブロック: コース区間設定、追い越し許可、現在相手車リスクを集約する。
  // 設計意図: 候補生成前に「今どの制約が有効か」をBlockedInfoへ一度まとめる。
  const ActiveSectionSafety active_section = activeSectionSafety(ego.frenet.s);
  // active_overtake_permission はlookahead先の禁止区間も返し得る。一方、低速
  // 障害物の例外は現在地点そのものが禁止の時だけに限定するため、両方を保持する。
  const ActiveOvertakePermission current_overtake_permission =
      overtakePermissionAtS(ego.frenet.s);
  const ActiveOvertakePermission active_overtake_permission =
      activeOvertakePermission(ego.frenet.s);
  const double wall_soft_margin = effectiveWallSoftMargin(active_section);
  BlockedInfo blocked = blocked_risk_.detectBlocked(ego, opponents, now_sec);
  blocked.maneuver_transaction_retry_active =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  const auto predictions = predictOpponents(opponents, now_sec);
  blocked.opponent_prediction_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && predictions.size() == opponents.size();
  blocked = blocked_risk_.evaluatePredictivePassTargetShadow(
      ego, blocked, opponents, predictions, now_sec);
  promoteSlowObstacleChain(blocked, opponents);
  classifyEarlyStationaryParallelPassTarget(now_sec, ego, reentry_input,
                                            blocked, opponents);
  classifyParallelFollowCandidate(now_sec, ego, blocked, opponents);
  classifyStationaryFrontObstacle(now_sec, ego, blocked, opponents);
  classifyBrakingFollowTarget(now_sec, ego, reentry_input, blocked, opponents);
  classifyEarlyLowSpeedPassTarget(now_sec, ego, reentry_input, blocked,
                                  opponents);
  if (blocked.start_grid_target_confirmation_pending) {
    // grid保持中の静止は通常の停止障害物例外へ即時昇格させない。制動距離
    // FOLLOWは残し、時間/進行距離確認後だけstationary診断へ移す。
    blocked.stationary_front_obstacle = false;
    blocked.stationary_front_id.clear();
    blocked.stationary_front_ttc_sec = std::numeric_limits<double>::infinity();
    blocked.stationary_front_brake_feasible = false;
  }
  // 固定15 mのfront classifierへ入る前でも、既存braking-followが同周期の
  // freshな停止対象を一意に束縛できているなら、制限PASS候補を先行評価する。
  // これはcallback-localな候補入力であり、後段の全安全評価が成立した場合だけ
  // 既存start gateへ渡す。safe-cycle、exact ACK、motion authorityは迂回しない。
  const bool stationary_curve_preflight_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && reentry_input.mpc_healthy &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  const int stationary_curve_preflight_target_index =
      blocked.braking_follow_active ? blocked.braking_follow_index
                                    : blocked.early_low_speed_pass_target_index;
  const std::string &stationary_curve_preflight_classified_id =
      blocked.braking_follow_active ? blocked.braking_follow_id
                                    : blocked.early_low_speed_pass_target_id;
  const double stationary_curve_preflight_delta_s_m =
      blocked.braking_follow_active
          ? blocked.braking_follow_delta_s
          : blocked.early_low_speed_pass_target_delta_s_m;
  const double stationary_curve_preflight_relative_speed_mps =
      blocked.braking_follow_active
          ? blocked.braking_follow_relative_speed_mps
          : ego.v - blocked.early_low_speed_pass_target_speed_mps;
  const bool stationary_curve_preflight_index_valid =
      stationary_curve_preflight_target_index >= 0 &&
      static_cast<std::size_t>(stationary_curve_preflight_target_index) <
          opponents.size();
  bool stationary_curve_preflight_target_current = false;
  std::string stationary_curve_preflight_target_id;
  if (stationary_curve_preflight_inputs_complete &&
      stationary_curve_preflight_index_valid &&
      !stationary_curve_preflight_classified_id.empty()) {
    const auto &target = opponents[static_cast<std::size_t>(
        stationary_curve_preflight_target_index)];
    const bool stationary_front_identity_consistent =
        !blocked.stationary_front_obstacle ||
        (!blocked.stationary_front_id.empty() &&
         blocked.stationary_front_id ==
             stationary_curve_preflight_classified_id);
    stationary_curve_preflight_target_current =
        target.valid && target.id == stationary_curve_preflight_classified_id &&
        inputTimestampFresh(now_sec, target.stamp_sec,
                            config_.opponent_stale_time_sec,
                            config_.input_future_stamp_tolerance_sec) &&
        std::isfinite(target.v) && target.v >= 0.0 &&
        target.v <=
            std::max(0.0, config_.stationary_obstacle_speed_threshold_mps) &&
        std::isfinite(stationary_curve_preflight_delta_s_m) &&
        stationary_curve_preflight_delta_s_m > 0.0 &&
        std::isfinite(stationary_curve_preflight_relative_speed_mps) &&
        stationary_curve_preflight_relative_speed_mps >
            std::max(0.0, config_.dv_block_threshold_mps) &&
        stationary_front_identity_consistent;
    if (stationary_curve_preflight_target_current) {
      stationary_curve_preflight_target_id = target.id;
    }
  }
  blocked =
      blocked_risk_.evaluatePassGap(blocked, opponents, predictions, mode_);
  blocked.overtake_permission_allowed =
      !config_.overtake_permission_profile_enabled ||
      active_overtake_permission.allow_overtake;
  blocked.overtake_permission_section_name = active_overtake_permission.name;
  if (!config_.overtake_permission_profile_enabled) {
    blocked.overtake_permission_reason = "permission_profile_disabled";
  } else if (active_overtake_permission.active) {
    blocked.overtake_permission_reason =
        active_overtake_permission.allow_overtake ? "section_allowed"
                                                  : "section_disallowed";
  } else {
    blocked.overtake_permission_reason = config_.default_overtake_allowed
                                             ? "default_allowed"
                                             : "default_disallowed";
  }
  const bool front_low_speed_by_distance =
      (!blocked.start_grid_target_active ||
       blocked.start_grid_target_confirmed_stationary) &&
      blocked.nearest_index >= 0 &&
      std::isfinite(blocked.front_vehicle_speed_mps) &&
      blocked.front_vehicle_speed_mps <=
          std::max(0.0, config_.slow_front_exception_speed_mps) &&
      (blocked.front_delta_s <=
           std::max(0.0, config_.slow_front_exception_distance_m) ||
       blocked.slow_obstacle_chain_active);
  blocked.front_vehicle_low_speed =
      config_.slow_front_exception_enabled && front_low_speed_by_distance;
  blocked.slow_front_exception_active = updateSlowFrontException(blocked);
  blocked.slow_front_exception_count = slow_front_exception_count_;
  if (generic_recovery_phase_active_ &&
      !needsGenericRecoveryGate(config_, mode_, blocked)) {
    // 過去周期の広めparallel/壁寄り補正でgeneric phaseへ入っていても、
    // 現周期で相手車両リスクが消えていれば低速holdを継続しない。
    generic_recovery_phase_active_ = false;
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
  }
  // 処理ブロック: 追い越し開始gateとコーナー横並びリスクを判定する。
  // 設計意図:
  // コーナー入口で新規追い越しを始めない一方、既に横並びなら譲りや維持へ誘導する。
  blocked.corner_abs_curvature =
      maxAbsCurvatureAhead(ego.frenet.s, config_.corner_side_yield_lookahead_m);
  blocked.corner_side_by_side =
      blocked.side_by_side && config_.corner_side_yield_curvature_m_inv > 0.0 &&
      blocked.corner_abs_curvature >= config_.corner_side_yield_curvature_m_inv;
  blocked.overtake_start_abs_curvature =
      maxAbsCurvatureAhead(ego.frenet.s, config_.straight_overtake_lookahead_m);
  blocked.straight_overtake_start_allowed = true;
  if (config_.straight_only_overtake_enabled &&
      config_.straight_overtake_max_curvature_m_inv > 0.0) {
    const double close_threshold =
        std::max(0.0, config_.straight_overtake_max_curvature_m_inv);
    const double open_threshold = std::max(
        0.0,
        close_threshold -
            std::max(0.0, config_.straight_overtake_release_hysteresis_m_inv));
    if (straight_overtake_start_allowed_) {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= close_threshold;
    } else {
      straight_overtake_start_allowed_ =
          blocked.overtake_start_abs_curvature <= open_threshold;
    }
    blocked.straight_overtake_start_allowed = straight_overtake_start_allowed_;
    if (!blocked.straight_overtake_start_allowed) {
      blocked.overtake_start_gate_reason = "curve";
    }
  } else {
    straight_overtake_start_allowed_ = true;
  }
  // 停止/低速前走車の例外は、曲率による「新規開始だけ」の抑止を外す。
  // permission profile、PASS候補のSafetyEvaluator、壁/CBF/制動到達可能性は
  // そのまま残すため、カーブ中の無条件PASSにはならない。
  const bool slow_front_curve_exception =
      blocked.slow_front_exception_active &&
      blocked.overtake_permission_allowed &&
      config_.slow_front_exception_max_start_curvature_m_inv > 0.0 &&
      blocked.overtake_start_abs_curvature <=
          config_.slow_front_exception_max_start_curvature_m_inv &&
      !blocked.straight_overtake_start_allowed &&
      blocked.overtake_start_gate_reason == "curve";
  const bool early_stationary_parallel_curve_exception =
      blocked.early_stationary_parallel_pass_target &&
      (!blocked.start_grid_target_active ||
       blocked.start_grid_target_confirmed_stationary) &&
      blocked.early_stationary_parallel_pass_count >=
          std::max(1, config_.slow_front_exception_required_cycles) &&
      blocked.overtake_permission_allowed &&
      config_.slow_front_exception_max_start_curvature_m_inv > 0.0 &&
      blocked.overtake_start_abs_curvature <=
          config_.slow_front_exception_max_start_curvature_m_inv &&
      !blocked.straight_overtake_start_allowed &&
      blocked.overtake_start_gate_reason == "curve";
  if (slow_front_curve_exception || early_stationary_parallel_curve_exception) {
    straight_overtake_start_allowed_ = true;
    blocked.straight_overtake_start_allowed = true;
    blocked.overtake_start_gate_reason =
        slow_front_curve_exception ? "slow_front_exception_curve"
                                   : "early_stationary_parallel_curve";
  }
  // permissionを重ねる前の曲率gate結果を保存する。停止/低速例外でpermission
  // だけを開く際にも、高曲率で閉じた開始gateは絶対に開かない。
  const bool curvature_start_allowed = blocked.straight_overtake_start_allowed;
  const bool permission_start_allowed = blocked.overtake_permission_allowed;
  if (!permission_start_allowed) {
    straight_overtake_start_allowed_ = false;
    blocked.straight_overtake_start_allowed = false;
    if (blocked.overtake_start_gate_reason.empty()) {
      blocked.overtake_start_gate_reason =
          blocked.overtake_permission_reason.empty()
              ? "section_disallowed"
              : blocked.overtake_permission_reason;
    }
  }
  blocked.ego_lateral_offset_m = ego.frenet.d;
  blocked.ego_speed_mps = ego.v;
  blocked.ego_wall_clearance_m =
      blocked_risk_.wallClearance(ego.frenet.s, ego.frenet.d);
  // 処理ブロック: 未来横並びとsection
  // safetyを重ねて、事前譲りが必要かを決める。 設計意図:
  // 現在は接触していなくても、コーナーで外側車両が壁へ寄る場面を先に抑える。
  blocked = future_side_risk_.evaluate(ego, blocked, opponents);
  bool post_abort_curve_hold_reentry_required = false;
  if (post_abort_curve_hold_active_) {
    const bool centered_for_post_abort_hold =
        std::isfinite(ego.frenet.d) &&
        std::abs(ego.frenet.d) <= reentryCompletionRearmLateralErrorM(config_);
    if (!centered_for_post_abort_hold) {
      // 専用hold中に中心許容帯から外れたら、通常走行へ戻さず再びgate付き
      // ABORTを要求する。中心d以外を「カーブ保持」として温存しない。
      post_abort_curve_hold_active_ = false;
      post_abort_curve_hold_reentry_required = true;
      reentry_clear_cycles_ = 0;
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      reentry_gate_permitted_ = false;
      blocked.reentry_hold_active = true;
    } else if (!shouldHoldPostAbortCurve(config_, ego, blocked)) {
      post_abort_curve_hold_active_ = false;
    }
  }
  blocked.post_abort_curve_hold_active = post_abort_curve_hold_active_;
  if (blocked.post_abort_curve_hold_active) {
    // hold中は候補PASSを評価・蓄積しない。解除後はFREE/FOLLOWから同じ
    // SafetyEvaluatorを通してpass_safe_required_cyclesを最初から数える。
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason = "post_abort_curve_hold";
  }
  // 診断用の広いparallel観測をそのままYIELDにしない。CBFへ近づく前に
  // 減速して後方へ譲る必要がある近接・同方向・接近中だけを昇格する。
  blocked.parallel_yield_hold_lateral =
      hasStrictParallelYieldRisk(config_, blocked);
  if (active_section.force_outer_yield &&
      (blocked.side_by_side || blocked.parallel_side_candidate ||
       blocked.future_side_by_side) &&
      blocked.ego_wall_clearance_m <= wall_soft_margin) {
    blocked.future_yield_required = true;
    blocked.future_corner_side_by_side = true;
    blocked.future_outer_wall_risk = true;
    if (blocked.yield_reason.empty()) {
      blocked.yield_reason = "section_outer_yield";
    }
  }
  updateLeaderPriority(now_sec, blocked);
  const int pass_start_debounce_cycles = std::max(
      1, static_cast<int>(std::ceil(config_.pass_safe_required_cycles)));
  const int pass_start_mode_hold_cycles =
      static_cast<int>(std::ceil(std::max(0.0, config_.min_mode_hold_time_sec) *
                                 std::max(1.0, config_.control_rate_hz)));
  const int pass_start_target_continuity_budget_cycles =
      pass_start_debounce_cycles + pass_start_mode_hold_cycles;
  const bool direct_pass_probe_context =
      blocked.blocked || blocked.start_grid_target_active ||
      blocked.early_stationary_parallel_pass_target ||
      blocked.braking_follow_active ||
      blocked.early_low_speed_pass_target_active;
  const auto has_direct_same_target_pass_probe_context = [&]() {
    return localized_lateral_profile_.active &&
           ((!blocked.nearest_id.empty() &&
             blocked.nearest_id == localized_lateral_profile_.target_id) ||
            (blocked.start_grid_target_active &&
             blocked.start_grid_target_id ==
                 localized_lateral_profile_.target_id) ||
            (blocked.early_stationary_parallel_pass_target &&
             blocked.early_stationary_parallel_pass_id ==
                 localized_lateral_profile_.target_id) ||
            (blocked.braking_follow_active &&
             blocked.braking_follow_id ==
                 localized_lateral_profile_.target_id) ||
            (blocked.early_low_speed_pass_target_active &&
             blocked.early_low_speed_pass_target_id ==
                 localized_lateral_profile_.target_id));
  };
  const bool direct_same_target_pass_probe_context =
      has_direct_same_target_pass_probe_context();
  const bool pass_start_target_continuity_expired =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_safety_approved_once &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !direct_same_target_pass_probe_context &&
      localized_lateral_profile_.pass_start_target_continuity_cycles >=
          pass_start_target_continuity_budget_cycles;
  if (pass_start_target_continuity_expired) {
    // Gate 2認可だけを過去の安全証明として永久保持しない。分類外の同一targetを
    // 現周期に再評価できる期間は、新規PASS debounceとmode holdを
    // 完了できる上限周期数までとする。
    clearLocalizedLateralProfile();
  }
  // PASS対象契約は、large_lateral_errorや候補生成より先に同じIDで解決する。
  // front分類の代表車が変わっても、freshなラッチ対象を抜き切るまでは
  // 横遷移包絡とSafetyEvaluatorの対象を別IDへすり替えない。
  updateLocalizedLateralProfile(now_sec, ego, blocked, opponents);
  blocked.pass_start_target_continuity_expired =
      pass_start_target_continuity_expired;
  blocked.pass_start_target_continuity_budget_cycles =
      pass_start_target_continuity_budget_cycles;
  // start-grid handoffの限定例外は、作り直された未認可profileと同じ
  // fresh targetを保持している間だけ有効にする。認可/commit、欠測、別IDへの
  // profile更新、完了/clearのいずれかで世代IDを破棄し、後続車へ継承しない。
  const bool start_grid_handoff_lineage_valid =
      !start_grid_handoff_target_id_.empty() &&
      localized_lateral_profile_.active &&
      !localized_lateral_profile_.pass_safety_approved_once &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      localized_lateral_profile_.target_id == start_grid_handoff_target_id_ &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh &&
      blocked.maneuver_target_id == start_grid_handoff_target_id_;
  if (!start_grid_handoff_lineage_valid) {
    start_grid_handoff_target_id_.clear();
  }
  blocked.start_grid_replacement_target_id = start_grid_handoff_target_id_;
  const auto predicted_target_d_for_pass = [&](CandidateType pass_type,
                                               const OpponentState &target) {
    // 現在dだけで目標を作ると、相手が評価horizon内にPASS側へ移動中でも
    // 縦距離が残っている間は楕円衝突が発生せず、将来塞がる側を先に
    // Gate 2認可できてしまう。BlockedRiskAnalyzerと同じ予測列の横包絡を
    // 使い、左PASSは最大d、右PASSは最小dを安全側の対象位置にする。
    double predicted_d_m = target.frenet.d;
    const auto prediction =
        std::find_if(predictions.begin(), predictions.end(),
                     [&target](const PredictedOpponent &value) {
                       return value.id == target.id;
                     });
    if (prediction == predictions.end()) {
      return predicted_d_m;
    }
    for (const double d_m : prediction->d) {
      if (!std::isfinite(d_m)) {
        continue;
      }
      if (pass_type == CandidateType::PASS_LEFT) {
        predicted_d_m = std::max(predicted_d_m, d_m);
      } else if (pass_type == CandidateType::PASS_RIGHT) {
        predicted_d_m = std::min(predicted_d_m, d_m);
      }
    }
    return predicted_d_m;
  };
  if (localized_lateral_profile_.active &&
      !localized_lateral_profile_.pass_safety_approved_once &&
      blocked.maneuver_target_observed && blocked.maneuver_target_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_target_index) <
          opponents.size()) {
    // Gate 2の初回認可前だけ、fresh予測で相手がPASS側へ寄る場合の必要dを
    // 外側へ拡張する。認可後はユーザー契約どおり同じ横位置を固定し、相手の
    // 変化に対しては目標dを追従させず、固定profile自身のSafetyEvaluator結果で
    // 継続またはFOLLOW/YIELDへのhandoffを決める。
    const auto &target =
        opponents[static_cast<std::size_t>(blocked.maneuver_target_index)];
    const CandidateType pass_type = localized_lateral_profile_.pass_type;
    const double predicted_target_d_m =
        predicted_target_d_for_pass(pass_type, target);
    const double required_target_d_m = targetOffsetForPass(
        pass_type, localized_lateral_profile_.start_d_m, predicted_target_d_m);
    if (pass_type == CandidateType::PASS_LEFT) {
      localized_lateral_profile_.target_d_m =
          std::max(localized_lateral_profile_.target_d_m, required_target_d_m);
    } else if (pass_type == CandidateType::PASS_RIGHT) {
      localized_lateral_profile_.target_d_m =
          std::min(localized_lateral_profile_.target_d_m, required_target_d_m);
    }
    if (!localized_lateral_profile_.chain_waypoints.empty()) {
      double staged_target_d_m = localized_lateral_profile_.target_d_m;
      localized_lateral_profile_.chain_waypoints.front().target_d_m =
          staged_target_d_m;
      for (std::size_t i = 1U;
           i < localized_lateral_profile_.chain_waypoints.size(); ++i) {
        auto &waypoint = localized_lateral_profile_.chain_waypoints[i];
        staged_target_d_m =
            pass_type == CandidateType::PASS_LEFT
                ? std::max(staged_target_d_m, waypoint.target_d_m)
                : std::min(staged_target_d_m, waypoint.target_d_m);
        waypoint.target_d_m = staged_target_d_m;
      }
    }
  }
  blocked.maneuver_transaction_incomplete =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  const auto is_confirmed_start_grid_profile = [&]() {
    return blocked.start_grid_target_active &&
           blocked.start_grid_target_confirmed_stationary &&
           !blocked.start_grid_target_id.empty() &&
           localized_lateral_profile_.active &&
           localized_lateral_profile_.target_id == blocked.start_grid_target_id;
  };
  blocked.maneuver_transaction_prepared =
      is_confirmed_start_grid_profile() &&
      localized_lateral_profile_.pass_safety_approved_once &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  blocked.maneuver_transaction_pass_type =
      localized_lateral_profile_.active ? localized_lateral_profile_.pass_type
                                        : CandidateType::FASTEST;
  const auto maneuver_execution_hold_identity_matches = [&]() {
    return maneuver_execution_hold_active_ &&
           blocked.maneuver_transaction_incomplete &&
           !maneuver_execution_hold_target_id_.empty() &&
           maneuver_execution_hold_target_id_ == blocked.maneuver_target_id &&
           maneuver_execution_hold_target_id_ ==
               localized_lateral_profile_.target_id &&
           maneuver_execution_hold_pass_type_ ==
               blocked.maneuver_transaction_pass_type &&
           maneuver_execution_hold_pass_type_ ==
               localized_lateral_profile_.pass_type &&
           (maneuver_execution_hold_pass_type_ == CandidateType::PASS_LEFT ||
            maneuver_execution_hold_pass_type_ == CandidateType::PASS_RIGHT);
  };
  if (maneuver_execution_hold_active_ &&
      !maneuver_execution_hold_identity_matches()) {
    // profile完了・破棄、target/side変更、明示centeringのいずれでも旧STOPの
    // 再開権限を次transactionへ持ち越さない。
    maneuver_execution_hold_active_ = false;
    maneuver_execution_hold_target_id_.clear();
    maneuver_execution_hold_pass_type_ = CandidateType::FASTEST;
  }
  const auto reset_start_grid_tracking_release = [this]() {
    start_grid_tracking_release_pending_ = false;
    start_grid_tracking_release_confirmed_ = false;
    start_grid_tracking_release_cycles_ = 0;
    pass_probe_exact_ack_cycles_ = 0;
    pass_probe_last_exact_sample_stamp_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    pass_probe_last_exact_plan_generation_ = 0U;
    start_grid_tracking_release_target_id_.clear();
    start_grid_tracking_release_candidate_valid_ = false;
    start_grid_tracking_release_candidate_ = CandidateTrajectory{};
    start_grid_tracking_release_token_ = 0U;
  };
  const auto reset_start_grid_tracking_probe = [this]() {
    start_grid_tracking_probe_cycles_ = 0;
    start_grid_tracking_probe_target_id_.clear();
    start_grid_tracking_probe_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_probe_target_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_last_target_stamp_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_tracking_probe_candidate_valid_ = false;
    start_grid_tracking_probe_candidate_ = CandidateTrajectory{};
  };
  const auto reset_start_grid_tracking_failed_candidate = [this]() {
    start_grid_tracking_failed_candidate_valid_ = false;
    start_grid_tracking_failed_target_id_.clear();
    start_grid_tracking_failed_pass_type_ = CandidateType::FASTEST;
    start_grid_tracking_failed_candidate_ = CandidateTrajectory{};
    start_grid_tracking_failed_ego_yaw_rad_ =
        std::numeric_limits<double>::quiet_NaN();
  };
  const auto advance_start_grid_tracking_release_token = [this]() {
    start_grid_tracking_release_token_counter_ =
        start_grid_tracking_release_token_counter_ ==
                std::numeric_limits<std::uint64_t>::max()
            ? 1U
            : start_grid_tracking_release_token_counter_ + 1U;
    start_grid_tracking_release_token_ =
        start_grid_tracking_release_token_counter_;
  };
  const bool committed_execution_hold_tracking_release_context =
      maneuver_execution_hold_identity_matches() &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty();
  const bool committed_start_grid_tracking_release_context =
      blocked.start_grid_target_active &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty();
  const bool committed_pending_tracking_release_context =
      start_grid_tracking_release_pending_ &&
      !start_grid_tracking_release_target_id_.empty() &&
      start_grid_tracking_release_target_id_ == blocked.maneuver_target_id &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty();
  const bool prepared_pass_tracking_release_context =
      blocked.start_grid_target_active &&
      blocked.start_grid_target_confirmed_stationary &&
      blocked.maneuver_transaction_prepared &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty();
  const bool start_grid_tracking_release_context =
      committed_execution_hold_tracking_release_context ||
      committed_start_grid_tracking_release_context ||
      committed_pending_tracking_release_context ||
      prepared_pass_tracking_release_context;
  const bool committed_tracking_release_context =
      committed_execution_hold_tracking_release_context ||
      committed_start_grid_tracking_release_context ||
      committed_pending_tracking_release_context;
  if (committed_tracking_release_context &&
      localized_lateral_profile_.pass_tracking_proof_published_last_cycle &&
      reentry_input.pure_pursuit_primary_and_fresh) {
    // readyは現在override generationとの一致までNodeで検証済み。直前に
    // publishした同じtarget/sideのPASS自身をfinal PPが追従できた時だけ、
    // 以後の高々1 generation transport skewを継続根拠としてarmする。
    localized_lateral_profile_.pass_tracking_continuity_armed = true;
  }
  const bool execution_hold_release_completed =
      committed_execution_hold_tracking_release_context &&
      start_grid_tracking_release_pending_ &&
      start_grid_tracking_release_confirmed_ &&
      localized_lateral_profile_.pass_tracking_proof_published_last_cycle &&
      reentry_input.pure_pursuit_primary_and_fresh;
  if (!start_grid_tracking_release_context ||
      (committed_start_grid_tracking_release_context &&
       !committed_execution_hold_tracking_release_context &&
       reentry_input.pure_pursuit_primary_and_fresh) ||
      execution_hold_release_completed) {
    reset_start_grid_tracking_release();
    reset_start_grid_tracking_probe();
    reset_start_grid_tracking_failed_candidate();
    if (execution_hold_release_completed) {
      maneuver_execution_hold_active_ = false;
      maneuver_execution_hold_target_id_.clear();
      maneuver_execution_hold_pass_type_ = CandidateType::FASTEST;
    }
  } else {
    const bool destructive_probe_transport_evidence =
        reentry_input.start_grid_pass_probe_transport_evidence ==
            StartGridProbeTransportEvidence::STALE ||
        reentry_input.start_grid_pass_probe_transport_evidence ==
            StartGridProbeTransportEvidence::PAYLOAD_MUTATION ||
        reentry_input.start_grid_pass_probe_transport_evidence ==
            StartGridProbeTransportEvidence::REGRESSION_OR_GAP;
    const bool target_matches_pending =
        start_grid_tracking_release_pending_ &&
        start_grid_tracking_release_target_id_ == blocked.maneuver_target_id &&
        start_grid_tracking_release_candidate_valid_ &&
        start_grid_tracking_release_candidate_.type ==
            blocked.maneuver_transaction_pass_type;
    if (destructive_probe_transport_evidence) {
      reset_start_grid_tracking_release();
      reset_start_grid_tracking_probe();
    } else if (!target_matches_pending) {
      reset_start_grid_tracking_release();
      // current-d STOPの一段目proofだけではPASS pendingを開始しない。後段で
      // 同周期のSafetyEvaluator/trackabilityを通った同target/side
      // PASSが実在する
      // 場合だけwarm-upを作る。PASS不能時は安全なATTACK_FOLLOWを選択できる。
    } else if (start_grid_tracking_release_candidate_valid_ &&
               start_grid_tracking_release_token_ != 0U &&
               reentry_input.pass_probe_exact_current_usable &&
               reentry_input.pass_probe_lateral_stop_authority_token ==
                   start_grid_tracking_release_token_ &&
               reentry_input.pass_probe_exact_plan_generation != 0U &&
               std::isfinite(reentry_input.pass_probe_exact_sample_stamp_sec)) {
      const int required_exact_probe_samples =
          std::max(2, config_.start_grid_tracking_probe_required_cycles);
      const bool exact_count_was_complete =
          pass_probe_exact_ack_cycles_ >= required_exact_probe_samples;
      const bool distinct_exact_sample =
          reentry_input.pass_probe_exact_plan_generation !=
              pass_probe_last_exact_plan_generation_ ||
          !std::isfinite(pass_probe_last_exact_sample_stamp_sec_) ||
          reentry_input.pass_probe_exact_sample_stamp_sec >
              pass_probe_last_exact_sample_stamp_sec_ + 1.0e-9;
      if (distinct_exact_sample) {
        pass_probe_last_exact_plan_generation_ =
            reentry_input.pass_probe_exact_plan_generation;
        pass_probe_last_exact_sample_stamp_sec_ =
            reentry_input.pass_probe_exact_sample_stamp_sec;
        pass_probe_exact_ack_cycles_ = std::min(
            required_exact_probe_samples, pass_probe_exact_ack_cycles_ + 1);
      }
      // 2件目のexact sampleを受けた周期もSTOPを維持する。confirmedは
      // 次のPlanner出力で非STOP motion grantを作るための内部状態だけであり、
      // Muxは後続generationのexact grantが揃うまで速度を出さない。
      start_grid_tracking_release_confirmed_ = exact_count_was_complete;
    }
    if (start_grid_tracking_release_pending_) {
      ++start_grid_tracking_release_cycles_;
      if (start_grid_tracking_release_cycles_ >
          config_.start_grid_tracking_release_timeout_cycles) {
        reset_start_grid_tracking_release();
      }
    }
  }
  blocked.maneuver_transaction_tracking_release_pending =
      start_grid_tracking_release_pending_;
  blocked.maneuver_transaction_tracking_release_confirmed =
      start_grid_tracking_release_confirmed_;
  blocked.maneuver_transaction_tracking_release_cycles =
      start_grid_tracking_release_cycles_;
  blocked.maneuver_transaction_tracking_probe_cycles =
      pass_probe_exact_ack_cycles_;
  blocked.maneuver_transaction_tracking_continuity_armed =
      localized_lateral_profile_.pass_tracking_continuity_armed;
  const auto update_start_grid_uncommitted_hold = [&]() {
    constexpr double kDirectionEpsilonM = 1.0e-9;
    const auto hold_bounds =
        frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
    const double hold_lower_d_m = hold_bounds.d_min + config_.min_wall_margin_m;
    const double hold_upper_d_m = hold_bounds.d_max - config_.min_wall_margin_m;
    const bool target_index_valid =
        blocked.start_grid_target_index >= 0 &&
        static_cast<std::size_t>(blocked.start_grid_target_index) <
            opponents.size();
    const bool target_observed =
        target_index_valid && !blocked.start_grid_target_id.empty() &&
        !start_grid_target_id_.empty() &&
        opponents[static_cast<std::size_t>(blocked.start_grid_target_index)]
            .valid &&
        opponents[static_cast<std::size_t>(blocked.start_grid_target_index)]
                .id == blocked.start_grid_target_id &&
        blocked.start_grid_target_id == start_grid_target_id_;
    const bool inputs_complete =
        reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
        reentry_input.all_observed_opponents_fresh &&
        reentry_input.all_observed_opponents_included &&
        reentry_input.reference_valid;
    const bool lateral_values_finite =
        std::isfinite(ego.frenet.d) &&
        std::isfinite(blocked.start_grid_hold_target_d_m) &&
        std::isfinite(hold_lower_d_m) && std::isfinite(hold_upper_d_m);
    const bool bounded_anchor_drift =
        lateral_values_finite &&
        std::abs(ego.frenet.d - blocked.start_grid_hold_target_d_m) <=
            config_.start_grid_uncommitted_hold_max_lateral_drift_m +
                kDirectionEpsilonM;
    const bool same_reference_side =
        lateral_values_finite &&
        ((blocked.start_grid_hold_target_d_m >= 0.0 && ego.frenet.d >= 0.0) ||
         (blocked.start_grid_hold_target_d_m <= 0.0 && ego.frenet.d <= 0.0));
    const bool reference_inside_safe_corridor =
        lateral_values_finite && hold_lower_d_m <= 0.0 && 0.0 <= hold_upper_d_m;
    const bool same_side_reference_inward =
        reference_inside_safe_corridor && same_reference_side &&
        std::abs(ego.frenet.d) <=
            std::abs(blocked.start_grid_hold_target_d_m) + kDirectionEpsilonM;
    // 15 cm以内の従来許容は測位/制御ノイズ用に維持する。上限を超える場合は、
    // 基準線が安全回廊内かつ同じFrenet側への内向き移動だけを許可する。
    // abs(d)だけで比べると+2.6 -> -2.5の大横断も内向きになるため、符号を
    // 必ず固定する。anchor自体は更新せず、外向き上限のratchetも防ぐ。
    const bool lateral_drift_allowed =
        bounded_anchor_drift || same_side_reference_inward;
    const bool lateral_state_valid =
        lateral_values_finite && hold_lower_d_m <= hold_upper_d_m &&
        ego.frenet.d >= hold_lower_d_m && ego.frenet.d <= hold_upper_d_m &&
        blocked.start_grid_hold_target_d_m >= hold_lower_d_m &&
        blocked.start_grid_hold_target_d_m <= hold_upper_d_m &&
        lateral_drift_allowed;
    blocked.start_grid_uncommitted_hold_active =
        blocked.start_grid_target_active &&
        blocked.start_grid_follow_hold_lateral && target_observed &&
        inputs_complete && lateral_state_valid &&
        !blocked.start_grid_target_superseded_by_blocked_front &&
        !blocked.start_grid_lateral_release_pending &&
        !blocked.maneuver_transaction_prepared &&
        !blocked.maneuver_transaction_incomplete &&
        !blocked.maneuver_transaction_tracking_release_pending &&
        !blocked.pass_decision_frozen && !blocked.future_yield_required &&
        !blocked.parallel_yield_hold_lateral && !reentry_phase_active_ &&
        !reentry_lockout_active_ && !generic_recovery_phase_active_;
  };
  update_start_grid_uncommitted_hold();
  // PASS候補の将来dが安全でも、実車がまだ横へ到達していない間に相手より
  // 速く詰めると、横追従遅れだけで制動余裕を使い切る。固定target/chain tailの
  // fresh実測位置から、候補生成前にlateral-first速度上限を確定する。
  updatePassLateralFirstSpeedGate(now_sec, ego, blocked, opponents);
  const bool attack_follow_transaction_active =
      blocked.maneuver_transaction_incomplete &&
      (isAnyPassMode(mode_) || mode_ == BehaviorMode::FOLLOW_BLOCKED ||
       mode_ == BehaviorMode::ABORT_RECOVERY ||
       mode_ == BehaviorMode::SPEED_GUARD) &&
      std::isfinite(localized_lateral_profile_.target_d_m);
  if (attack_follow_transaction_active) {
    // PASS候補の安全余裕が低下してFOLLOWへ移っても、現在dを新しい正解として
    // 毎周期張り直さない。元のtarget IDと同じPASS側dを保持したFOLLOW候補を
    // 作り、壁・相手との成立性は後段のSafetyEvaluatorへ必ず通す。
    blocked.attack_follow_hold_pass_side = true;
    double staged_target_d_m = localized_lateral_profile_.target_d_m;
    if (!localized_lateral_profile_.chain_waypoints.empty() &&
        std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) &&
        std::isfinite(ego.frenet.s)) {
      const double ego_unwrapped_s_m =
          localized_lateral_profile_.ego_unwrapped_s_m;
      // scalar target_d_mは先頭target用であり、staged
      // chainの途中では古い内側dに
      // なる。egoより前に残る最初のwaypoint（全車通過後は末尾）の累積外側dを
      // FOLLOW目標に使い、authoritative target ID/side自体は変更しない。
      auto staged_waypoint = std::find_if(
          localized_lateral_profile_.chain_waypoints.begin(),
          localized_lateral_profile_.chain_waypoints.end(),
          [ego_unwrapped_s_m](const LocalizedLateralWaypoint &waypoint) {
            return std::isfinite(waypoint.target_s_m) &&
                   waypoint.target_s_m + 1.0e-6 >= ego_unwrapped_s_m;
          });
      if (staged_waypoint == localized_lateral_profile_.chain_waypoints.end()) {
        staged_waypoint =
            std::prev(localized_lateral_profile_.chain_waypoints.end());
      }
      if (std::isfinite(staged_waypoint->target_d_m)) {
        staged_target_d_m = staged_waypoint->target_d_m;
      }
    }
    blocked.attack_follow_target_d_m = staged_target_d_m;
  }
  const bool authorized_envelope_matches_transaction =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      localized_lateral_profile_.target_id ==
          authorized_pass_envelope_target_id_ &&
      localized_lateral_profile_.pass_type ==
          authorized_pass_envelope_pass_type_ &&
      std::isfinite(authorized_pass_envelope_min_d_m_) &&
      std::isfinite(authorized_pass_envelope_max_d_m_);
  const double authorized_envelope_error_m =
      authorized_envelope_matches_transaction
          ? authorizedPassEnvelopeErrorM(ego.frenet.d)
          : std::numeric_limits<double>::infinity();
  // 保存包絡には横移動前のstart dも含むため、包絡内というだけでは停止車の
  // 制動距離不足を免除しない。単独対象の固定target dへ90%以上進み、残差も
  // 10%（小変位では数値余裕1 cm）以内になった後だけ「PASS側到達済み」とする。
  const double authorized_pass_lateral_delta_m =
      localized_lateral_profile_.target_d_m -
      localized_lateral_profile_.start_d_m;
  const double authorized_pass_direction =
      authorized_pass_lateral_delta_m >= 0.0 ? 1.0 : -1.0;
  const double authorized_pass_lateral_progress_m =
      authorized_pass_direction *
      (ego.frenet.d - localized_lateral_profile_.start_d_m);
  const double authorized_pass_target_error_m =
      std::abs(ego.frenet.d - localized_lateral_profile_.target_d_m);
  const double authorized_pass_target_tolerance_m =
      std::max(0.01, 0.10 * std::abs(authorized_pass_lateral_delta_m));
  const bool authorized_pass_side_reached =
      std::isfinite(authorized_pass_lateral_delta_m) &&
      std::abs(authorized_pass_lateral_delta_m) > 1.0e-6 &&
      std::isfinite(authorized_pass_lateral_progress_m) &&
      authorized_pass_lateral_progress_m + 1.0e-6 >=
          0.90 * std::abs(authorized_pass_lateral_delta_m) &&
      std::isfinite(authorized_pass_target_error_m) &&
      authorized_pass_target_error_m <=
          authorized_pass_target_tolerance_m + 1.0e-6;
  blocked.authorized_pass_current_d_hold_active =
      authorized_envelope_matches_transaction &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == localized_lateral_profile_.target_id &&
      !blocked.maneuver_chain_tail_id.empty() &&
      blocked.maneuver_chain_tail_id == blocked.maneuver_target_id &&
      blocked.maneuver_chain_target_count == 1 &&
      authorized_pass_side_reached && std::isfinite(ego.frenet.d) &&
      authorized_envelope_error_m <= 1.0e-6;
  if (blocked.authorized_pass_current_d_hold_active &&
      blocked.attack_follow_hold_pass_side) {
    // PASS側へ既に到達した後は、局所profileの数mmの終端補正を低速時に
    // 毎周期作り直さない。元target dはprofileへ保持したまま、fallback候補だけを
    // 現在dへ固定し、後段のSafetyEvaluatorで全相手・壁を改めて検査する。
    blocked.attack_follow_target_d_m = ego.frenet.d;
  }
  double pass_aware_lateral_error =
      authorized_envelope_matches_transaction
          ? authorized_envelope_error_m
          : lateralErrorForPassAwareFreeze(config_, mode_, ego,
                                           localized_lateral_profile_);
  const bool authoritative_feedback_matches_transaction =
      reentry_input.previous_authoritative_plan_feedback_valid &&
      authorized_envelope_matches_transaction &&
      reentry_input.previous_authoritative_plan_target_id ==
          localized_lateral_profile_.target_id &&
      reentry_input.previous_authoritative_plan_pass_type ==
          localized_lateral_profile_.pass_type;
  const double lockout_trigger_error_m =
      std::max(0.0, config_.large_lateral_error_threshold_m);
  if (!pass_reauthorization_lockout_active_ &&
      authorized_envelope_matches_transaction &&
      authoritative_feedback_matches_transaction &&
      authorized_envelope_error_m > lockout_trigger_error_m &&
      (reentry_input.previous_authoritative_plan_stop_requested ||
       !reentry_input.previous_authoritative_plan_trajectory_authorized)) {
    pass_reauthorization_lockout_active_ = true;
    pass_reauthorization_lockout_target_id_ =
        localized_lateral_profile_.target_id;
    pass_reauthorization_lockout_pass_type_ =
        localized_lateral_profile_.pass_type;
    pass_reauthorization_clear_cycles_ = 0;
  }
  const bool lockout_matches_transaction =
      pass_reauthorization_lockout_active_ &&
      authorized_envelope_matches_transaction &&
      pass_reauthorization_lockout_target_id_ ==
          localized_lateral_profile_.target_id &&
      pass_reauthorization_lockout_pass_type_ ==
          localized_lateral_profile_.pass_type;
  if (lockout_matches_transaction) {
    const double release_error_m = reentryCompletionLateralErrorM(config_);
    const bool release_sample =
        authoritative_feedback_matches_transaction &&
        !reentry_input.previous_authoritative_plan_stop_requested &&
        reentry_input.previous_authoritative_plan_trajectory_authorized &&
        reentry_input.pure_pursuit_primary_and_fresh &&
        authorized_envelope_error_m <= release_error_m;
    pass_reauthorization_clear_cycles_ =
        release_sample ? pass_reauthorization_clear_cycles_ + 1 : 0;
    if (pass_reauthorization_clear_cycles_ >=
        std::max(1, config_.reentry_safe_cycles)) {
      pass_reauthorization_lockout_active_ = false;
      pass_reauthorization_lockout_target_id_.clear();
      pass_reauthorization_lockout_pass_type_ = CandidateType::FASTEST;
      pass_reauthorization_clear_cycles_ = 0;
    }
  } else if (pass_reauthorization_lockout_active_) {
    // profileのclear/rebuildだけでlockoutを迂回させない。対象/sideの一致が
    // 戻るまではcurrent-d保持側へ閉じ、別PASSを開始しない。
    pass_reauthorization_clear_cycles_ = 0;
  }
  blocked.pass_reauthorization_lockout_active =
      pass_reauthorization_lockout_active_;
  blocked.pass_authorized_envelope_min_d_m = authorized_pass_envelope_min_d_m_;
  blocked.pass_authorized_envelope_max_d_m = authorized_pass_envelope_max_d_m_;
  blocked.pass_authorized_envelope_error_m = authorized_envelope_error_m;
  blocked.pass_reauthorization_clear_cycles =
      pass_reauthorization_clear_cycles_;
  if (pass_reauthorization_lockout_active_) {
    constexpr double kRecoveryInsideMarginM = 0.05;
    double recovery_target_d_m = ego.frenet.d;
    if (lockout_matches_transaction &&
        !reentry_input.previous_authoritative_plan_stop_requested) {
      if (ego.frenet.d > authorized_pass_envelope_max_d_m_) {
        recovery_target_d_m = std::max(authorized_pass_envelope_min_d_m_,
                                       authorized_pass_envelope_max_d_m_ -
                                           kRecoveryInsideMarginM);
      } else if (ego.frenet.d < authorized_pass_envelope_min_d_m_) {
        recovery_target_d_m = std::min(authorized_pass_envelope_max_d_m_,
                                       authorized_pass_envelope_min_d_m_ +
                                           kRecoveryInsideMarginM);
      }
    }
    blocked.pass_reauthorization_recovery_target_d_m = recovery_target_d_m;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason = "pass_profile_recovery_latched";
    blocked.pass_gap_reason = "pass_profile_recovery_latched";
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
  }
  blocked.decision_freeze_lateral_error_m = pass_aware_lateral_error;
  const bool large_lateral_error =
      config_.large_lateral_error_threshold_m >= 0.0 &&
      pass_aware_lateral_error > config_.large_lateral_error_threshold_m;
  // safe corridor内で現在dを保持して減速するstrict parallel YIELDは、中心へ
  // 戻すRECOVERYより先に評価する。コリドー外では従来どおりfreezeして壁側の
  // 横位置をそのまま走り続けない。
  const bool strict_parallel_yield_preempts_freeze =
      blocked.parallel_yield_hold_lateral &&
      blocked.ego_wall_clearance_m >= 0.0;
  if (blocked.ego_wall_clearance_m < 0.0) {
    wall_recovery_latched_ = true;
  } else if (std::isfinite(ego.frenet.d) &&
             std::abs(ego.frenet.d) <=
                 config_.recovery_release_lateral_error_m) {
    wall_recovery_latched_ = false;
  }
  const bool established_lateral_recovery =
      wall_recovery_latched_ || mode_ == BehaviorMode::SAFE_STOP ||
      mode_ == BehaviorMode::ABORT_RECOVERY ||
      mode_ == BehaviorMode::MERGE_BACK;
  const auto direct_target_identity_matches =
      [&](int target_index, const std::string &target_id) {
        return target_index >= 0 &&
               static_cast<std::size_t>(target_index) < opponents.size() &&
               !target_id.empty() && localized_lateral_profile_.active &&
               blocked.maneuver_target_latched &&
               blocked.maneuver_target_observed &&
               blocked.maneuver_target_fresh &&
               blocked.maneuver_target_index == target_index &&
               blocked.maneuver_target_id == target_id &&
               localized_lateral_profile_.target_id == target_id &&
               blocked.maneuver_chain_tail_observed &&
               blocked.maneuver_chain_tail_index >= 0 &&
               static_cast<std::size_t>(blocked.maneuver_chain_tail_index) <
                   opponents.size() &&
               !blocked.maneuver_chain_tail_id.empty() &&
               blocked.maneuver_chain_tail_id ==
                   localized_lateral_profile_.chain_tail_id &&
               opponents[static_cast<std::size_t>(
                             blocked.maneuver_chain_tail_index)]
                       .id == blocked.maneuver_chain_tail_id &&
               opponents[static_cast<std::size_t>(target_index)].id ==
                   target_id;
      };
  const bool direct_normal_front = [&]() {
    if (!blocked.blocked || blocked.nearest_index < 0 ||
        static_cast<std::size_t>(blocked.nearest_index) >= opponents.size() ||
        !std::isfinite(blocked.front_delta_s) || blocked.front_delta_s <= 0.0 ||
        !std::isfinite(blocked.front_rel_v) ||
        blocked.front_rel_v <= config_.dv_block_threshold_mps ||
        (!std::isfinite(blocked.front_vehicle_speed_mps) ||
         blocked.front_vehicle_speed_mps <=
             config_.slow_front_exception_speed_mps) ||
        blocked.slow_front_exception_active ||
        blocked.stationary_front_obstacle ||
        blocked.slow_obstacle_chain_active) {
      return false;
    }
    const auto &front =
        opponents[static_cast<std::size_t>(blocked.nearest_index)];
    return front.valid && blocked.front_direction_known &&
           blocked.front_same_direction &&
           inputTimestampFresh(now_sec, front.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec) &&
           direct_target_identity_matches(blocked.nearest_index,
                                          blocked.nearest_id);
  }();
  const bool direct_dynamic_braking_front = [&]() {
    if (!blocked.braking_follow_active || blocked.braking_follow_index < 0 ||
        static_cast<std::size_t>(blocked.braking_follow_index) >=
            opponents.size() ||
        !std::isfinite(blocked.braking_follow_delta_s) ||
        blocked.braking_follow_delta_s <= 0.0 ||
        !std::isfinite(blocked.braking_follow_relative_speed_mps) ||
        blocked.braking_follow_relative_speed_mps <=
            std::max(0.0, config_.dv_block_threshold_mps)) {
      return false;
    }
    const auto &front =
        opponents[static_cast<std::size_t>(blocked.braking_follow_index)];
    return front.valid &&
           inputTimestampFresh(now_sec, front.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec) &&
           direct_target_identity_matches(blocked.braking_follow_index,
                                          blocked.braking_follow_id);
  }();
  const bool pass_start_target_continuity = [&]() {
    blocked.pass_start_target_continuity_reason =
        "no_gate2_approved_prestart_profile";
    if (pass_start_target_continuity_expired) {
      blocked.pass_start_target_continuity_reason = "bounded_lifetime_expired";
      return false;
    }
    const bool precommit_mode_matches_profile =
        mode_ == BehaviorMode::FREE_RUN ||
        mode_ == BehaviorMode::FOLLOW_BLOCKED ||
        (mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT &&
         localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT) ||
        (mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT &&
         localized_lateral_profile_.pass_type == CandidateType::PASS_RIGHT);
    if (!localized_lateral_profile_.active ||
        !localized_lateral_profile_.pass_safety_approved_once ||
        localized_lateral_profile_.pass_execution_committed ||
        localized_lateral_profile_.pass_complete_confirmed ||
        !precommit_mode_matches_profile) {
      return false;
    }
    if (direct_pass_probe_context) {
      blocked.pass_start_target_continuity_reason = "direct_target_classified";
      return false;
    }
    if (!blocked.maneuver_target_latched || !blocked.maneuver_target_observed ||
        !blocked.maneuver_target_fresh || blocked.maneuver_target_index < 0 ||
        static_cast<std::size_t>(blocked.maneuver_target_index) >=
            opponents.size() ||
        blocked.maneuver_target_id != localized_lateral_profile_.target_id ||
        !blocked.maneuver_chain_tail_observed) {
      blocked.pass_start_target_continuity_reason =
          "target_missing_stale_or_lineage_mismatch";
      return false;
    }
    if (blocked.nearest_index >= 0 && !blocked.nearest_id.empty() &&
        blocked.nearest_id != localized_lateral_profile_.target_id) {
      blocked.pass_start_target_continuity_reason =
          "different_authoritative_front";
      return false;
    }
    const auto &target =
        opponents[static_cast<std::size_t>(blocked.maneuver_target_index)];
    const double target_s_dot_mps = blocked_risk_.opponentSDot(target);
    const bool direction_known =
        target.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        direction_known &&
        target_s_dot_mps >= config_.same_direction_min_s_dot_mps;
    if (!target.valid || !same_direction) {
      blocked.pass_start_target_continuity_reason = "target_direction_invalid";
      return false;
    }
    const double interaction_extension_m =
        std::max(0.0, config_.safety_ellipse_a_m) *
        std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h));
    const bool target_in_longitudinal_envelope =
        std::isfinite(blocked.maneuver_target_relative_s_m) &&
        blocked.maneuver_target_relative_s_m > 0.0 &&
        blocked.maneuver_target_relative_s_m <=
            std::max(0.0, config_.lookahead_s_m) + interaction_extension_m;
    const bool target_in_front_corridor =
        std::isfinite(blocked.maneuver_target_relative_d_m) &&
        std::abs(blocked.maneuver_target_relative_d_m) <
            std::max(0.0, config_.same_corridor_width_m);
    const bool target_not_pulling_away =
        std::isfinite(blocked.maneuver_target_relative_speed_mps) &&
        blocked.maneuver_target_relative_speed_mps >= 0.0;
    if (!target_in_longitudinal_envelope || !target_in_front_corridor ||
        !target_not_pulling_away) {
      blocked.pass_start_target_continuity_reason =
          "target_outside_interaction_envelope";
      return false;
    }
    const bool inputs_complete =
        reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
        reentry_input.all_observed_opponents_fresh &&
        reentry_input.all_observed_opponents_included &&
        reentry_input.reference_valid;
    const bool pass_start_tracking_usable =
        reentry_input.pure_pursuit_primary_and_fresh &&
        reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
    if (!pass_start_tracking_usable) {
      blocked.pass_start_target_continuity_reason =
          "pass_start_tracking_unusable";
      return false;
    }
    const bool tactical_context_clean =
        !blocked.side_by_side && !blocked.corner_side_by_side &&
        !blocked.parallel_yield_hold_lateral &&
        !blocked.future_corner_side_by_side && !blocked.future_yield_required &&
        !blocked.reentry_hold_active && !reentry_phase_active_ &&
        !reentry_lockout_active_ && !generic_recovery_phase_active_ &&
        !blocked.pass_decision_frozen && blocked.overtake_permission_allowed;
    if (!inputs_complete || !tactical_context_clean) {
      blocked.pass_start_target_continuity_reason =
          "inputs_or_tactical_context_incomplete";
      return false;
    }
    blocked.pass_start_target_continuity_reason =
        "fresh_same_target_within_interaction_envelope";
    return true;
  }();
  blocked.pass_start_target_continuity_active = pass_start_target_continuity;
  blocked.pass_start_target_continuity_cycles =
      localized_lateral_profile_.active
          ? localized_lateral_profile_.pass_start_target_continuity_cycles
          : 0;
  // PREPAREは実行前の最終再承認周期であるため、開始元と同じ入力完全性・
  // conflict条件のままgentle-curve制約候補を再評価する。ここで除外すると、
  // 前周期に承認した制約PASSが曲率gateだけで必ずFOLLOWへ戻ってしまう。
  const bool gentle_curve_context_clean =
      !blocked.side_by_side && !blocked.corner_side_by_side &&
      !blocked.parallel_yield_hold_lateral &&
      !blocked.future_corner_side_by_side && !blocked.future_yield_required &&
      !blocked.reentry_hold_active && !reentry_phase_active_ &&
      !reentry_lockout_active_ && !generic_recovery_phase_active_ &&
      !blocked.pass_decision_frozen && !blocked.post_abort_curve_hold_active &&
      (mode_ == BehaviorMode::FREE_RUN ||
       mode_ == BehaviorMode::FOLLOW_BLOCKED ||
       mode_ == BehaviorMode::SPEED_GUARD || isPrepareOvertakeMode(mode_));
  // 曲線例外で横移動を制限する場合も、通常PASSと同じSafetyEvaluator
  // 楕円間隔へ到達できなければ開始させない。短い予測区間だけ通っても
  // その後に必要な車間を作れないPASSを防ぐための開始gateである。
  const double required_gentle_curve_lateral_gap_m =
      config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h) +
      std::max(0.0, config_.pass_target_lateral_margin_m);
  const double gentle_curve_required_lateral_displacement_m =
      localized_lateral_profile_.active &&
              std::isfinite(localized_lateral_profile_.target_d_m) &&
              std::isfinite(ego.frenet.d)
          ? std::abs(localized_lateral_profile_.target_d_m - ego.frenet.d)
          : std::numeric_limits<double>::infinity();
  const bool gentle_curve_lateral_capacity_sufficient =
      config_.gentle_curve_safe_pass_max_lateral_displacement_m >=
          required_gentle_curve_lateral_gap_m &&
      gentle_curve_required_lateral_displacement_m <=
          config_.gentle_curve_safe_pass_max_lateral_displacement_m + 1.0e-9;
  const bool gentle_curve_lateral_accel_limit_valid =
      std::isfinite(config_.gentle_curve_safe_pass_max_lateral_accel_mps2) &&
      config_.gentle_curve_safe_pass_max_lateral_accel_mps2 > 0.0 &&
      std::isfinite(blocked.overtake_start_abs_curvature) &&
      blocked.overtake_start_abs_curvature > 0.0;
  const double gentle_curve_dynamic_speed_cap_mps =
      gentle_curve_lateral_accel_limit_valid
          ? std::sqrt(config_.gentle_curve_safe_pass_max_lateral_accel_mps2 /
                      blocked.overtake_start_abs_curvature)
          : std::numeric_limits<double>::quiet_NaN();
  const bool gentle_curve_dynamic_speed_cap_valid =
      std::isfinite(gentle_curve_dynamic_speed_cap_mps) &&
      gentle_curve_dynamic_speed_cap_mps > 0.0;
  const double gentle_curve_speed_cap_mps =
      gentle_curve_dynamic_speed_cap_valid
          ? std::min(config_.gentle_curve_safe_pass_v_max_mps,
                     gentle_curve_dynamic_speed_cap_mps)
          : std::numeric_limits<double>::quiet_NaN();
  const bool gentle_curve_target_identity_valid =
      direct_target_identity_matches(blocked.nearest_index, blocked.nearest_id);
  const bool direct_fresh_dynamic_gap_front = [&]() {
    if (!blocked.blocked || !gentle_curve_target_identity_valid ||
        blocked.nearest_index < 0 ||
        static_cast<std::size_t>(blocked.nearest_index) >= opponents.size() ||
        !std::isfinite(blocked.front_delta_s) || blocked.front_delta_s <= 0.0 ||
        blocked.front_delta_s > std::max(0.0, config_.follow_trigger_s_m) ||
        !std::isfinite(blocked.front_delta_d) ||
        std::abs(blocked.front_delta_d) >=
            std::max(0.0, config_.same_corridor_width_m) ||
        !std::isfinite(blocked.front_rel_v) ||
        !std::isfinite(blocked.front_vehicle_speed_mps) ||
        blocked.front_vehicle_speed_mps <=
            std::max(config_.stationary_obstacle_speed_threshold_mps,
                     config_.slow_front_exception_speed_mps) ||
        !blocked.front_direction_known || !blocked.front_same_direction ||
        blocked.start_grid_target_active ||
        blocked.early_stationary_parallel_pass_target ||
        blocked.braking_follow_active || blocked.stationary_front_obstacle ||
        blocked.slow_front_exception_active ||
        blocked.slow_obstacle_chain_active ||
        blocked.maneuver_unstarted_target_released) {
      return false;
    }
    const auto &front =
        opponents[static_cast<std::size_t>(blocked.nearest_index)];
    return front.valid && !front.id.empty() && std::isfinite(front.frenet.s) &&
           std::isfinite(front.frenet.d) && std::isfinite(front.v) &&
           inputTimestampFresh(now_sec, front.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec);
  }();
  const double direct_dynamic_target_s_dot_mps =
      direct_fresh_dynamic_gap_front
          ? blocked_risk_.opponentSDot(
                opponents[static_cast<std::size_t>(blocked.nearest_index)])
          : std::numeric_limits<double>::quiet_NaN();
  const double dynamic_pass_capability_mps =
      gentle_curve_dynamic_speed_cap_valid
          ? std::min(config_.pass_speed_cap_mps, gentle_curve_speed_cap_mps)
          : std::numeric_limits<double>::quiet_NaN();
  const bool gentle_curve_dynamic_target_speed_reachable =
      direct_fresh_dynamic_gap_front &&
      std::isfinite(direct_dynamic_target_s_dot_mps) &&
      std::isfinite(dynamic_pass_capability_mps) &&
      direct_dynamic_target_s_dot_mps +
              std::max(0.0, config_.dv_block_threshold_mps) <=
          dynamic_pass_capability_mps;
  const bool gentle_curve_curvature_within_limit =
      std::isfinite(blocked.overtake_start_abs_curvature) &&
      blocked.overtake_start_abs_curvature <=
          config_.gentle_curve_safe_pass_max_curvature_m_inv;
  const bool gentle_curve_target_context =
      direct_normal_front || direct_dynamic_braking_front ||
      (direct_fresh_dynamic_gap_front &&
       gentle_curve_dynamic_target_speed_reachable) ||
      pass_start_target_continuity;
  blocked.gentle_curve_target_identity_valid =
      gentle_curve_target_identity_valid;
  blocked.gentle_curve_direct_normal_target = direct_normal_front;
  blocked.gentle_curve_direct_braking_target = direct_dynamic_braking_front;
  blocked.gentle_curve_fresh_dynamic_gap_target =
      direct_fresh_dynamic_gap_front;
  blocked.gentle_curve_dynamic_target_speed_reachable =
      gentle_curve_dynamic_target_speed_reachable;
  blocked.gentle_curve_target_context_reason =
      direct_normal_front            ? "closing_dynamic_front"
      : direct_dynamic_braking_front ? "dynamic_braking_front"
      : direct_fresh_dynamic_gap_front &&
              gentle_curve_dynamic_target_speed_reachable
          ? "fresh_dynamic_front_with_speed_reserve"
      : pass_start_target_continuity        ? "gate2_target_continuity"
      : !gentle_curve_target_identity_valid ? "target_identity_invalid"
      : direct_fresh_dynamic_gap_front      ? "dynamic_target_speed_unreachable"
                                            : "no_direct_dynamic_target";
  blocked.gentle_curve_context_clean = gentle_curve_context_clean;
  blocked.gentle_curve_lateral_capacity_sufficient =
      gentle_curve_lateral_capacity_sufficient;
  blocked.gentle_curve_dynamic_speed_cap_valid =
      gentle_curve_dynamic_speed_cap_valid;
  blocked.gentle_curve_curvature_within_limit =
      gentle_curve_curvature_within_limit;
  const bool gentle_curve_safe_pass_eligible =
      config_.gentle_curve_safe_pass_enabled &&
      config_.gentle_curve_safe_pass_max_curvature_m_inv > 0.0 &&
      config_.gentle_curve_safe_pass_v_max_mps > 0.0 &&
      config_.gentle_curve_safe_pass_max_lateral_displacement_m > 0.0 &&
      !curvature_start_allowed &&
      blocked.overtake_start_gate_reason == "curve" &&
      blocked.overtake_permission_allowed && gentle_curve_target_context &&
      gentle_curve_context_clean && gentle_curve_lateral_capacity_sufficient &&
      gentle_curve_dynamic_speed_cap_valid &&
      gentle_curve_curvature_within_limit;
  blocked.gentle_curve_safe_pass_eligible = gentle_curve_safe_pass_eligible;
  blocked.gentle_curve_safe_pass_constraint_active =
      gentle_curve_safe_pass_constraint_latched_ ||
      gentle_curve_safe_pass_eligible;
  blocked.gentle_curve_safe_pass_anchor_d_m =
      gentle_curve_safe_pass_constraint_latched_ &&
              std::isfinite(gentle_curve_safe_pass_anchor_d_m_)
          ? gentle_curve_safe_pass_anchor_d_m_
          : ego.frenet.d;
  blocked.gentle_curve_safe_pass_speed_cap_mps =
      gentle_curve_safe_pass_constraint_latched_ &&
              std::isfinite(gentle_curve_safe_pass_speed_cap_mps_)
          ? (gentle_curve_dynamic_speed_cap_valid
                 ? std::min(gentle_curve_safe_pass_speed_cap_mps_,
                            gentle_curve_speed_cap_mps)
                 : gentle_curve_safe_pass_speed_cap_mps_)
          : (gentle_curve_safe_pass_eligible
                 ? gentle_curve_speed_cap_mps
                 : std::numeric_limits<double>::quiet_NaN());
  // 停止障害物の高曲率PASSは、通常のgentle curve PASSとは分離する。許可区間
  // でも通常の曲率gateだけでは停止車の後方で永久停止するため、禁止区間と同じ
  // 専用d/v制約とGate 2を必須にする。permissionの例外化は後段で禁止区間だけに
  // 適用し、reentry・future yield・横並びではこの経路を閉じる。
  const bool stationary_no_pass_context_clean =
      !blocked.side_by_side && !blocked.corner_side_by_side &&
      !blocked.parallel_yield_hold_lateral && !blocked.future_side_by_side &&
      !blocked.future_corner_side_by_side && !blocked.future_yield_required &&
      !blocked.reentry_hold_active && !reentry_phase_active_ &&
      !reentry_lockout_active_ && !generic_recovery_phase_active_ &&
      (mode_ == BehaviorMode::FREE_RUN ||
       mode_ == BehaviorMode::FOLLOW_BLOCKED ||
       mode_ == BehaviorMode::YIELD_BEHIND || isAnyPassMode(mode_));
  const bool stationary_no_pass_lateral_capacity_sufficient =
      config_.stationary_no_pass_safe_pass_max_lateral_displacement_m >=
      required_gentle_curve_lateral_gap_m;
  const bool stationary_no_pass_lateral_accel_limit_valid =
      std::isfinite(
          config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2) &&
      config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 > 0.0 &&
      std::isfinite(blocked.overtake_start_abs_curvature) &&
      blocked.overtake_start_abs_curvature > 0.0;
  const double stationary_no_pass_dynamic_speed_cap_mps =
      stationary_no_pass_lateral_accel_limit_valid
          ? std::sqrt(
                config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 /
                blocked.overtake_start_abs_curvature)
          : std::numeric_limits<double>::quiet_NaN();
  const bool stationary_no_pass_dynamic_speed_cap_valid =
      std::isfinite(stationary_no_pass_dynamic_speed_cap_mps) &&
      stationary_no_pass_dynamic_speed_cap_mps > 0.0;
  const double stationary_no_pass_speed_cap_mps =
      stationary_no_pass_dynamic_speed_cap_valid
          ? std::min(config_.stationary_no_pass_safe_pass_v_max_mps,
                     stationary_no_pass_dynamic_speed_cap_mps)
          : std::numeric_limits<double>::quiet_NaN();
  // 専用例外を開始した後は、同じ停止対象だけを再認証する。slow-frontの
  // 必要連続周期を1に設定しても別IDへすり替わらないよう、通常の周期countとは
  // 独立して開始対象IDをラッチする。
  const bool latched_stationary_pass_target_current = [&]() {
    if (!isAnyPassMode(mode_) || !blocked.maneuver_target_latched ||
        !blocked.maneuver_target_observed || !blocked.maneuver_target_fresh ||
        blocked.maneuver_target_id.empty() ||
        blocked.maneuver_target_index < 0 ||
        static_cast<std::size_t>(blocked.maneuver_target_index) >=
            opponents.size()) {
      return false;
    }
    const auto &target =
        opponents[static_cast<std::size_t>(blocked.maneuver_target_index)];
    return target.valid && target.id == blocked.maneuver_target_id &&
           inputTimestampFresh(now_sec, target.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec) &&
           std::isfinite(target.v) && target.v >= 0.0 &&
           target.v <=
               std::max(0.0, config_.stationary_obstacle_speed_threshold_mps) &&
           (stationary_no_pass_safe_pass_target_id_.empty() ||
            stationary_no_pass_safe_pass_target_id_ == target.id);
  }();
  const std::string &stationary_no_pass_authoritative_target_id =
      blocked.stationary_front_obstacle ? blocked.stationary_front_id
      : stationary_curve_preflight_target_current
          ? stationary_curve_preflight_target_id
          : blocked.maneuver_target_id;
  const bool stationary_no_pass_target_current =
      (blocked.stationary_front_obstacle &&
       blocked.slow_front_exception_active &&
       !blocked.stationary_front_id.empty()) ||
      stationary_curve_preflight_target_current ||
      latched_stationary_pass_target_current;
  const bool stationary_no_pass_target_id_matches =
      !isAnyPassMode(mode_) ||
      (!stationary_no_pass_safe_pass_target_id_.empty() &&
       stationary_no_pass_safe_pass_target_id_ ==
           stationary_no_pass_authoritative_target_id);
  const bool stationary_no_pass_safe_pass_eligible =
      config_.stationary_no_pass_safe_pass_enabled &&
      config_.stationary_no_pass_safe_pass_max_curvature_m_inv > 0.0 &&
      config_.stationary_no_pass_safe_pass_v_max_mps > 0.0 &&
      config_.stationary_no_pass_safe_pass_max_lateral_displacement_m > 0.0 &&
      stationary_no_pass_target_current &&
      !blocked.slow_obstacle_chain_active && !curvature_start_allowed &&
      blocked.overtake_start_gate_reason == "curve" &&
      stationary_no_pass_context_clean &&
      stationary_no_pass_lateral_capacity_sufficient &&
      stationary_no_pass_dynamic_speed_cap_valid &&
      stationary_no_pass_target_id_matches && !large_lateral_error &&
      std::isfinite(ego.v) &&
      ego.v <= stationary_no_pass_speed_cap_mps + 1.0e-6 &&
      blocked.overtake_start_abs_curvature <=
          config_.stationary_no_pass_safe_pass_max_curvature_m_inv;
  const bool stationary_curve_preflight_constraint =
      config_.stationary_no_pass_safe_pass_enabled &&
      stationary_curve_preflight_target_current && !curvature_start_allowed &&
      blocked.overtake_start_gate_reason == "curve" &&
      stationary_no_pass_context_clean &&
      stationary_no_pass_lateral_capacity_sufficient &&
      stationary_no_pass_dynamic_speed_cap_valid &&
      blocked.overtake_start_abs_curvature <=
          config_.stationary_no_pass_safe_pass_max_curvature_m_inv;
  blocked.stationary_no_pass_safe_pass_eligible =
      stationary_no_pass_safe_pass_eligible;
  blocked.stationary_no_pass_safe_pass_constraint_active =
      stationary_no_pass_safe_pass_constraint_latched_ ||
      stationary_no_pass_safe_pass_eligible ||
      stationary_curve_preflight_constraint;
  blocked.stationary_no_pass_safe_pass_anchor_d_m =
      stationary_no_pass_safe_pass_constraint_latched_ &&
              std::isfinite(stationary_no_pass_safe_pass_anchor_d_m_)
          ? stationary_no_pass_safe_pass_anchor_d_m_
          : ego.frenet.d;
  blocked.stationary_no_pass_safe_pass_speed_cap_mps =
      stationary_no_pass_safe_pass_constraint_latched_ &&
              std::isfinite(stationary_no_pass_safe_pass_speed_cap_mps_)
          ? (stationary_no_pass_dynamic_speed_cap_valid
                 ? std::min(stationary_no_pass_safe_pass_speed_cap_mps_,
                            stationary_no_pass_speed_cap_mps)
                 : stationary_no_pass_safe_pass_speed_cap_mps_)
          : (stationary_no_pass_safe_pass_eligible ||
                     stationary_curve_preflight_constraint
                 ? stationary_no_pass_speed_cap_mps
                 : std::numeric_limits<double>::quiet_NaN());
  // soft wall（安全コリドー内）と広めのparallel診断だけでは、速度capに加えて
  // RECOVERY横override/PASS freezeまで連鎖させない。壁マージン外、実干渉、
  // future yield、PASS中gap喪失は従来どおりfail-closedでfreezeする。
  const bool freeze_overtake_decisions =
      blocked.pass_reauthorization_lockout_active ||
      blocked.post_abort_curve_hold_active ||
      (blocked.maneuver_target_latched && !blocked.maneuver_target_observed) ||
      (large_lateral_error &&
       (blocked.ego_wall_clearance_m < 0.0 || established_lateral_recovery ||
        // 通常の前方閉塞かつ大きなdだけでは候補評価を凍結しない。壁外・既存復帰・
        // 横並び等の実リスクは従来どおり優先し、左右PASSはGate 2へ委ねる。
        blocked.side_by_side || blocked.corner_side_by_side ||
        blocked.future_yield_required || currentPassGapLost(mode_, blocked)) &&
       !strict_parallel_yield_preempts_freeze);
  const bool follow_fallback_context = currentPassGapLost(mode_, blocked) ||
                                       mode_ == BehaviorMode::ABORT_RECOVERY ||
                                       attack_follow_transaction_active;
  // D1/D2のように通常FOLLOW帯の外側へ並走した相手は、PASS候補を同周期に
  // 先に評価した上でのみcurrent-d FOLLOWへ再評価する。ここでは候補集合へ
  // 載せるだけで、下のscoreでfeasible PASSを常に優先する。
  const bool parallel_side_current_d_follow_recheck =
      canRecheckParallelSideAsCurrentDFollow(config_, now_sec, blocked,
                                             opponents);
  if (parallel_side_current_d_follow_recheck) {
    blocked.parallel_follow_recheck_attempted = true;
    blocked.parallel_follow_recheck_reason = "candidate_pending";
  }
  if (freeze_overtake_decisions) {
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason =
        blocked.pass_reauthorization_lockout_active
            ? "pass_profile_recovery_latched"
        : blocked.post_abort_curve_hold_active ? "post_abort_curve_hold"
        : blocked.maneuver_target_latched && !blocked.maneuver_target_observed
            ? "maneuver_target_missing_or_stale"
            : "large_lateral_error";
    if (blocked.pass_gap_reason.empty() || blocked.pass_gap_reason == "ok") {
      blocked.pass_gap_reason = blocked.pass_decision_freeze_reason;
    }
  }
  // gapを詰めるFOLLOWは、同一コリドーの通常前走車だけに限定する。SafetyEvaluator
  // に渡すs(t)は下流上限以上の3.0 m/s^2を仮定するため、ここでfalseなら従来の
  // 「前走車より少し遅い」FOLLOWへ直ちに戻る。
  const bool follow_gap_closing_mpc_ready =
      reentry_input.mpc_health_sample_sequence == 0U
          ? reentry_input.mpc_healthy
          : reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  // 初回PASS releaseにはexact current-generation proofを要求する一方、
  // 一度認可した同一target/sideのtransactionは、planner publish直前に見える
  // final PP proofが高々1世代遅れる通常の非同期を許容する。実際に使ったMPC
  // horizonがfresh/valid/feasibleでsolve-time上限内なら、latency warningが
  // 立っていない正常周期もcontinuityとして扱う。stale、hard failure、
  // infeasible、target/side不一致は引き続きfail closedとする。
  const bool bounded_mpc_tracking_continuity_usable =
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure &&
      (reentry_input.mpc_healthy || reentry_input.mpc_latency_warning) &&
      mpc_health.valid && mpc_health.infeasible_count == 0 &&
      std::isfinite(mpc_health.solve_time_ms) &&
      mpc_health.solve_time_ms <=
          config_.start_grid_tracking_continuity_max_mpc_solve_time_ms;
  const bool committed_tracking_continuity_authorized =
      (blocked.start_grid_target_active ||
       maneuver_execution_hold_identity_matches()) &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_transaction_tracking_continuity_armed &&
      reentry_input.pure_pursuit_tracking_continuity_usable &&
      reentry_input.pure_pursuit_tracking_target_id ==
          blocked.maneuver_target_id &&
      reentry_input.pure_pursuit_tracking_pass_type ==
          blocked.maneuver_transaction_pass_type &&
      ((reentry_input.verified_non_mpc_pure_pursuit &&
        !reentry_input.mpc_hard_failure) ||
       bounded_mpc_tracking_continuity_usable);
  const bool committed_retry_follow_blocked =
      mode_ == BehaviorMode::FOLLOW_BLOCKED &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete;
  // Moving ATTACK_FOLLOWからPASSを再試行する入口は、final muxが現在generationを
  // PPで追従中と確認し、typed target/sideも同一transactionへ一致する周期だけ
  // 許可する。N-1 continuityだけではcross-phase PASSを動かさず、後段で既存の
  // STOP warm-upとexact token/generation ACKを必ず作る。
  const bool moving_attack_follow_retry_ready =
      committed_retry_follow_blocked &&
      reentry_input.pure_pursuit_primary_and_fresh &&
      blocked.maneuver_transaction_tracking_continuity_armed &&
      reentry_input.pure_pursuit_tracking_continuity_usable &&
      reentry_input.pure_pursuit_tracking_target_id ==
          blocked.maneuver_target_id &&
      reentry_input.pure_pursuit_tracking_pass_type ==
          blocked.maneuver_transaction_pass_type &&
      follow_gap_closing_mpc_ready && !reentry_input.mpc_hard_failure;
  // PASS warm-up中は縦方向をSTOP constraintで拘束したまま、解除後に実行する
  // 加速s(t)を使って同じ候補を毎周期SafetyEvaluatorへ再投入する。現在のPPが
  // STOP中なので加速予測まで無効化すると、空間horizonが不足して候補が
  // untrackableとなり、exact ACKを受け取る前にwarm-up自身を破棄してしまう。
  // ここで認めるのは候補評価用の予測だけであり、実際のSTOP解除は従来どおり
  // generation/token/target/sideが一致するpure_pursuit_release_readyへ委ねる。
  const bool pass_release_warmup_prediction_authorized =
      (blocked.start_grid_target_active ||
       maneuver_execution_hold_identity_matches()) &&
      blocked.maneuver_transaction_tracking_release_pending &&
      (blocked.maneuver_transaction_prepared ||
       (blocked.maneuver_transaction_retry_active &&
        blocked.maneuver_transaction_incomplete)) &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && !blocked.maneuver_target_id.empty() &&
      start_grid_tracking_release_candidate_valid_ &&
      start_grid_tracking_release_target_id_ == blocked.maneuver_target_id &&
      start_grid_tracking_release_candidate_.type ==
          blocked.maneuver_transaction_pass_type &&
      (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT ||
       blocked.maneuver_transaction_pass_type == CandidateType::PASS_RIGHT) &&
      // 同一prepared transactionのATTACK_FOLLOW transportが既に成立する
      // 場合は、既存どおり安全評価済みcurrent-d FOLLOWで走行を継続する。
      // その経路までSTOP warm-upへ巻き戻さない。
      !reentry_input.pure_pursuit_attack_follow_transport_usable &&
      follow_gap_closing_mpc_ready;
  const bool longitudinal_prediction_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid;
  const bool initial_start_grid_prediction_complete =
      predictions.size() == opponents.size() &&
      std::all_of(predictions.begin(), predictions.end(),
                  [this](const auto &prediction) {
                    return prediction.t.size() == config_.horizon_points &&
                           prediction.x.size() == config_.horizon_points &&
                           prediction.y.size() == config_.horizon_points &&
                           prediction.s.size() == config_.horizon_points &&
                           prediction.d.size() == config_.horizon_points;
                  });
  const bool initial_start_grid_remaining_chain_fresh = [&]() {
    if (!localized_lateral_profile_.active ||
        localized_lateral_profile_.chain_waypoints.empty()) {
      return false;
    }
    const auto current_waypoint =
        std::find_if(localized_lateral_profile_.chain_waypoints.cbegin(),
                     localized_lateral_profile_.chain_waypoints.cend(),
                     [&blocked](const auto &waypoint) {
                       return waypoint.target_id == blocked.maneuver_target_id;
                     });
    if (current_waypoint == localized_lateral_profile_.chain_waypoints.cend()) {
      return false;
    }
    return std::all_of(
        current_waypoint, localized_lateral_profile_.chain_waypoints.cend(),
        [&opponents, now_sec, this](const auto &waypoint) {
          return std::any_of(
              opponents.cbegin(), opponents.cend(),
              [&waypoint, now_sec, this](const auto &opponent) {
                return opponent.valid && opponent.id == waypoint.target_id &&
                       inputTimestampFresh(
                           now_sec, opponent.stamp_sec,
                           config_.opponent_stale_time_sec,
                           config_.input_future_stamp_tolerance_sec);
              });
        });
  }();
  const bool initial_start_grid_target_identity_consistent =
      localized_lateral_profile_.active &&
      (localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT ||
       localized_lateral_profile_.pass_type == CandidateType::PASS_RIGHT) &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == blocked.start_grid_target_id &&
      blocked.maneuver_target_id == localized_lateral_profile_.target_id &&
      blocked.maneuver_target_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_target_index) <
          opponents.size() &&
      opponents[static_cast<std::size_t>(blocked.maneuver_target_index)]
          .valid &&
      opponents[static_cast<std::size_t>(blocked.maneuver_target_index)].id ==
          blocked.maneuver_target_id &&
      blocked.maneuver_chain_tail_observed &&
      blocked.maneuver_chain_tail_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_chain_tail_index) <
          opponents.size() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      blocked.maneuver_chain_tail_id ==
          localized_lateral_profile_.chain_tail_id &&
      opponents[static_cast<std::size_t>(blocked.maneuver_chain_tail_index)]
          .valid &&
      opponents[static_cast<std::size_t>(blocked.maneuver_chain_tail_index)]
              .id == blocked.maneuver_chain_tail_id;
  // 初回start-grid PASSでは、追従させるべきPASS wireがまだ無いため、現在の
  // normal trajectoryに対するPP proofを候補生成の前提にすると自己循環する。
  // 全入力・対象identity・静止確認・走行許可・周辺riskが揃う時だけ、解除後の
  // bounded加速s(t)を候補評価へ使う。実行は既存のSTOP constraint下warm-upへ
  // 留め、exact generation/token ACK前の正速度・横actuationは許可しない。
  const bool initial_start_grid_pass_prediction_bootstrap =
      longitudinal_prediction_inputs_complete &&
      initial_start_grid_prediction_complete && reentry_input.mpc_healthy &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure &&
      blocked.start_grid_target_active &&
      blocked.start_grid_target_confirmed_stationary &&
      !blocked.maneuver_transaction_prepared &&
      !blocked.maneuver_transaction_incomplete &&
      initial_start_grid_target_identity_consistent &&
      initial_start_grid_remaining_chain_fresh &&
      blocked.straight_overtake_start_allowed &&
      (blocked.overtake_permission_allowed ||
       blocked.permission_start_exception_active) &&
      !large_lateral_error && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_side_by_side &&
      !blocked.future_corner_side_by_side && !blocked.future_outer_wall_risk &&
      !blocked.future_yield_required && !blocked.parallel_yield_hold_lateral &&
      !blocked.reentry_hold_active && !reentry_phase_active_ &&
      !reentry_lockout_active_ && !blocked.pass_reauthorization_lockout_active;
  const bool follow_gap_closing_inputs_complete =
      longitudinal_prediction_inputs_complete &&
      reentry_input.pure_pursuit_primary_and_fresh &&
      follow_gap_closing_mpc_ready;
  // PASS中のN-1 transport proofをMuxが同一attempt/target/side/trajectoryとして
  // 検証済みなら、その1周期だけexact-readyの代わりに使う。また、固定済みの
  // spatial transactionは前周期の実行proofが一時的に無くても、freshな全入力で
  // 加速s(t)を候補として作る。ここで許すのは候補生成だけで、固定dとの合成後に
  // 現周期のSafetyEvaluatorを再実行し、最終実行権限はMuxのgeneration契約へ残す。
  // これにより「tracking proof遅れ -> 加速予測off -> 空間horizon不足 ->
  // PASS失格 -> STOPのため次proofも生成不能」という自己ロックだけを断つ。
  const bool committed_spatial_prediction_bootstrap_authorized =
      longitudinal_prediction_inputs_complete &&
      committed_pass_snapshot_valid_ &&
      committed_pass_spatial_profile_frozen_ &&
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      blocked.maneuver_target_id == committed_pass_snapshot_target_id_ &&
      localized_lateral_profile_.target_id ==
          committed_pass_snapshot_target_id_ &&
      localized_lateral_profile_.pass_type ==
          committed_pass_snapshot_pass_type_;
  // commit済みPASSが現周期のwall/相手評価で一時不成立となった時も、同じ
  // target/sideへ張り付くATTACK_FOLLOWの候補評価用s(t)は維持する。PP proof
  // のN-2遅延を実行権限には使わず、固定snapshot、全fresh入力、正常MPCが
  // 揃う時だけ縦予測を作る。最終候補は下流のtrackability/SafetyEvaluator、
  // 実actuationはNode/Muxのexact/N-1 generation契約を引き続き必須とする。
  const bool committed_attack_follow_prediction_bootstrap_authorized =
      committed_spatial_prediction_bootstrap_authorized &&
      reentry_input.mpc_healthy && reentry_input.mpc_health_fresh &&
      !reentry_input.mpc_hard_failure;
  const bool pass_acceleration_inputs_complete =
      follow_gap_closing_inputs_complete ||
      initial_start_grid_pass_prediction_bootstrap ||
      (longitudinal_prediction_inputs_complete &&
       (committed_tracking_continuity_authorized ||
        pass_release_warmup_prediction_authorized)) ||
      committed_spatial_prediction_bootstrap_authorized;
  const bool attack_follow_acceleration_inputs_complete =
      follow_gap_closing_inputs_complete ||
      (longitudinal_prediction_inputs_complete &&
       (committed_tracking_continuity_authorized ||
        pass_release_warmup_prediction_authorized)) ||
      committed_attack_follow_prediction_bootstrap_authorized;
  const bool normal_front_prestart_hold_context =
      blocked.blocked && blocked.nearest_index >= 0 &&
      static_cast<std::size_t>(blocked.nearest_index) < opponents.size() &&
      std::isfinite(blocked.front_delta_s) &&
      blocked.front_delta_s >= config_.follow_gap_closing_engage_gap_m &&
      std::isfinite(blocked.front_rel_v) && blocked.front_rel_v <= 0.0 &&
      blocked.front_direction_known && blocked.front_same_direction &&
      !blocked.parallel_follow_candidate && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_side_by_side &&
      !blocked.future_yield_required && !blocked.parallel_yield_hold_lateral &&
      !blocked.stationary_front_obstacle &&
      !blocked.slow_obstacle_chain_active &&
      !blocked.slow_front_exception_active;
  const bool normal_front_gap_closing_context =
      normal_front_prestart_hold_context && !blocked.reentry_hold_active;
  blocked.follow_gap_closing_allowed = config_.follow_gap_closing_enabled &&
                                       follow_gap_closing_inputs_complete &&
                                       normal_front_gap_closing_context;
  blocked.attack_follow_acceleration_allowed =
      blocked.attack_follow_hold_pass_side &&
      !blocked.pass_reauthorization_lockout_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      blocked.maneuver_chain_tail_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_chain_tail_index) <
          opponents.size() &&
      std::isfinite(blocked.maneuver_chain_tail_relative_s_m) &&
      blocked.maneuver_chain_tail_relative_s_m > 0.0 &&
      (!blocked.braking_follow_active ||
       blocked.braking_follow_id == blocked.maneuver_chain_tail_id) &&
      attack_follow_acceleration_inputs_complete && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_side_by_side &&
      !blocked.future_corner_side_by_side && !blocked.future_outer_wall_risk &&
      !blocked.future_yield_required && !blocked.parallel_yield_hold_lateral &&
      !blocked.reentry_hold_active && !reentry_phase_active_ &&
      !reentry_lockout_active_;
  // PASSの速度capを上げる場合も、FOLLOWの加速と同じfreshness/MPC healthを
  // 必須にする。候補横移動のSafetyEvaluatorだけでなく、加速後のs(t)も同じ
  // horizonで評価するための明示的な入力gateである。
  const bool bounded_lateral_first_creep_acceleration_allowed =
      pass_acceleration_inputs_complete &&
      blocked.pass_lateral_first_speed_gate_active &&
      std::isfinite(blocked.pass_lateral_first_target_speed_mps) &&
      blocked.pass_lateral_first_target_speed_mps <=
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps) &&
      std::isfinite(blocked.pass_lateral_first_speed_cap_mps) &&
      blocked.pass_lateral_first_speed_cap_mps > 0.0 &&
      blocked.pass_lateral_first_speed_cap_mps <= 1.0 + 1.0e-9 &&
      std::isfinite(ego.v) &&
      ego.v + 1.0e-6 < blocked.pass_lateral_first_speed_cap_mps &&
      blocked.ego_wall_clearance_m >= 0.0 && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_corner_side_by_side &&
      !blocked.future_outer_wall_risk && !blocked.future_yield_required &&
      !blocked.parallel_yield_hold_lateral && !blocked.reentry_hold_active &&
      !reentry_phase_active_ && !reentry_lockout_active_;
  blocked.pass_proposal_acceleration_allowed =
      !blocked.pass_reauthorization_lockout_active &&
      (bounded_lateral_first_creep_acceleration_allowed ||
       (pass_acceleration_inputs_complete &&
        // race arm直後のstart-grid対象は、同一コリドーfront分類より先に
        // 検出される。ここで加速到達可能性を候補評価へ含めないと、速度0の
        // s(t)が全点0となり、PP必要空間を満たせずPASSを永続的に
        // untrackable扱いする。実際の加速許可はこの候補が回廊・全相手予測・
        // ControllerTracking契約を通った後だけであり、SafetyEvaluatorには
        // 3.0 m/s^2の最遠到達側を渡すので衝突判定を緩めない。
        (blocked.blocked || blocked.start_grid_target_active ||
         blocked.attack_follow_acceleration_allowed) &&
        !large_lateral_error && !blocked.side_by_side &&
        !blocked.corner_side_by_side && !blocked.future_side_by_side &&
        !blocked.future_yield_required &&
        !blocked.parallel_yield_hold_lateral && !blocked.reentry_hold_active));
  // 実行candidateは従来のgateだけを使う。start-grid等で追加した仮想加速は
  // V2 shadow
  // proposalのSafetyEvaluator入力に限定し、実行authorityへ漏らさない。
  blocked.pass_acceleration_allowed =
      follow_gap_closing_inputs_complete && blocked.blocked &&
      !large_lateral_error && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_side_by_side &&
      !blocked.future_yield_required && !blocked.parallel_yield_hold_lateral &&
      !blocked.stationary_front_obstacle &&
      !blocked.slow_obstacle_chain_active && !blocked.reentry_hold_active;
  // 通常ラインへの復帰は、d2を抜いた事実や単一front gapでは許可しない。
  // 実際にpublishするRECOVERY形状を全fresh相手車両へ評価し、未許可なら現在dを保持する。
  const ReentryMpcHealthState reentry_mpc_health =
      updateReentryMpcHealthState(reentry_input);
  const bool reentry_gate_was_permitted = reentry_gate_permitted_;
  const bool post_abort_curve_hold_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && !reentry_input.mpc_hard_failure &&
      (!config_.reentry_require_mpc_health ||
       (reentry_input.mpc_health_fresh &&
        reentry_mpc_health == ReentryMpcHealthState::HEALTHY));
  if (post_abort_curve_hold_active_ && !post_abort_curve_hold_inputs_complete) {
    // 専用holdはfreshなSafetyEvaluator入力に限る。鮮度/CBFの前提を失った
    // 周期はSPEED_GUARDのまま温存せず、reentry gate付きABORTへ戻す。
    post_abort_curve_hold_active_ = false;
    post_abort_curve_hold_reentry_required = true;
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = true;
    reentry_phase_active_ = true;
    reentry_gate_permitted_ = false;
    blocked.reentry_hold_active = true;
  }
  ReentryGateResult reentry_gate =
      evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                          reentry_input, reentry_mpc_health);
  if (post_abort_curve_hold_reentry_required) {
    // 設定上MPC healthを要求しない場合でも、post-ABORT hold中に検出した
    // stale/hard failureをPASSやFREE_RUNに戻さない。
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = true;
    reentry_phase_active_ = true;
    reentry_gate_permitted_ = false;
    reentry_gate.requested = true;
    reentry_gate.permitted = false;
    reentry_gate.clear_cycles = 0;
    if (reentry_gate.reason.empty()) {
      reentry_gate.reason = "post_abort_curve_hold_reentry_required";
    }
    blocked.reentry_hold_active = true;
  }
  // updateReentryPhaseは中心到達時にphaseを解除する。その前にこの周期の
  // SafetyEvaluator済みgateを確認しないと、ABORT->FREE_RUNの境界で
  // post-ABORT高速カーブholdを見落とす。
  const bool reentry_completed_this_cycle =
      !generic_recovery_phase_active_ && reentry_gate.requested &&
      reentry_gate.input_complete && reentry_gate.permitted &&
      std::isfinite(ego.frenet.d) &&
      std::abs(ego.frenet.d) <= reentryCompletionLateralErrorM(config_);
  updateReentryPhase(ego, mode_);
  if (reentry_gate.requested && !reentry_gate.permitted) {
    // generic中心補正は現d保持+SPEED_GUARDへ留める。PASS由来の実復帰だけを
    // ABORT_RECOVERY lockoutへ入れ、両者の失敗時挙動を混ぜない。
    if (!generic_recovery_phase_active_) {
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
    }
    blocked.reentry_hold_active = true;
    if (reentry_mpc_health == ReentryMpcHealthState::TRANSIENT_LATENCY) {
      blocked.reentry_hold_speed_cap_mps =
          config_.reentry_mpc_degraded_hold_v_max_mps;
    }
  }
  // generic中心復帰gate中は、まず加速なしのcurrent-d FOLLOWを独立評価する。
  // 実PASSのreentry/lockoutではこの経路を開かない。FOLLOWが同周期の全予測・
  // wall・CBF・controller契約を通れば後段でgeneric phaseだけを解除し、次周期
  // から改めて加速を含む攻めFOLLOWとPASS開始を評価する。
  if (blocked.reentry_hold_active) {
    blocked.follow_gap_closing_allowed = false;
  }
  const bool prestart_profile_target_matches_front =
      localized_lateral_profile_.active &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !blocked.start_grid_target_active &&
      isPassCandidate(localized_lateral_profile_.pass_type) &&
      !localized_lateral_profile_.target_id.empty() &&
      localized_lateral_profile_.target_id == blocked.nearest_id;
  const bool prestart_mode_allows_hold =
      mode_ == BehaviorMode::FREE_RUN ||
      mode_ == BehaviorMode::FOLLOW_BLOCKED ||
      mode_ == BehaviorMode::SPEED_GUARD || isPrepareOvertakeMode(mode_);
  const bool generic_current_d_handoff_context =
      generic_recovery_phase_active_ && !reentry_phase_active_ &&
      !reentry_lockout_active_;
  blocked.prestart_attack_follow_hold_lateral =
      config_.follow_gap_closing_enabled &&
      prestart_profile_target_matches_front && prestart_mode_allows_hold &&
      follow_gap_closing_inputs_complete &&
      normal_front_prestart_hold_context &&
      (!blocked.reentry_hold_active || generic_current_d_handoff_context) &&
      !blocked.pass_reauthorization_lockout_active &&
      !blocked.post_abort_curve_hold_active && !blocked.pass_decision_frozen &&
      !blocked.future_corner_side_by_side && !blocked.future_outer_wall_risk;
  // 処理ブロック: 状況に応じた候補集合を作る。
  // 設計意図:
  // PASS候補は内部評価へ残しつつ、FOLLOW/YIELD/RECOVERY/SAFE_STOPの代替候補も同時に安全評価する。
  // まず全状況でFASTEST候補を作り、閉塞時だけ追従/左右追い越し候補を増やす。
  // PASSの目標到達性は開始前だけを長い距離で確認する。PREPARE/OVERTAKE中に
  // 相手dが一時的に変わっても反対側へ切り替えず、同側の最新horizon安全評価で
  // 継続/譲りを決める。
  blocked.pass_target_corridor_preflight_required = !isAnyPassMode(mode_);
  std::vector<CandidateTrajectory> candidates;
  // 初回Gate 2で反対側を評価した場合、その候補を作ったlocalized profileも
  // candidateと同じ周期で保持する。candidateのscalar target_dだけを書き戻すと、
  // staged chain
  // waypointが元の側に残り、認可した軌道と次周期の軌道が不一致になる。
  std::optional<LocalizedLateralProfile> evaluated_alternate_left_profile;
  std::optional<LocalizedLateralProfile> evaluated_alternate_right_profile;
  candidates.push_back(
      makeCandidate(CandidateType::FASTEST, ego, blocked, opponents));
  if (freeze_overtake_decisions) {
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
    if (blocked.braking_follow_active ||
        (follow_fallback_context && hasForwardFollowFallbackTarget(blocked))) {
      const BlockedInfo follow_blocked = followFallbackInfo(blocked);
      candidates.push_back(
          makeCandidate(CandidateType::FOLLOW, ego, follow_blocked, opponents));
    }
  } else {
    if (blocked.side_by_side && !blocked.corner_side_by_side &&
        !blocked.future_yield_required) {
      candidates.push_back(makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego,
                                         blocked, opponents));
    }
    if (shouldYieldBehindSideBySide(ego, blocked)) {
      candidates.push_back(
          makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
    }
    if (hasFollowBlockedTarget(blocked)) {
      candidates.push_back(
          makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents));
    } else if ((follow_fallback_context || isAnyPassMode(mode_) ||
                parallel_side_current_d_follow_recheck) &&
               hasForwardFollowFallbackTarget(blocked)) {
      // PASS不能時だけ、現在dを保った攻め追従を同周期に評価する。
      // 相手がSafetyEvaluatorを通らなければ、従来のYIELD/RECOVERYへ残る。
      const BlockedInfo follow_blocked = followFallbackInfo(blocked);
      candidates.push_back(
          makeCandidate(CandidateType::FOLLOW, ego, follow_blocked, opponents));
    }
    const bool committed_pass_retry_context =
        blocked.maneuver_transaction_incomplete &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed;
    const bool pass_probe_context =
        !blocked.maneuver_unstarted_target_released &&
        (blocked.blocked || blocked.start_grid_target_active ||
         blocked.early_stationary_parallel_pass_target ||
         blocked.braking_follow_active ||
         blocked.early_low_speed_pass_target_active ||
         committed_pass_retry_context ||
         blocked.pass_start_target_continuity_active);
    if (pass_probe_context) {
      // 静的gapは診断値として残し、左右の物理的安全性は各候補を同じSafetyEvaluatorで判定する。
      // localized candidateが一度もGate 2認可されていない間だけ、同じtarget
      // IDで 反対側profileも評価する。認可後または実OVERTAKE中はsideを固定し、
      // 同側が不成立になっても反対へ横切らずFOLLOW/YIELD/RECOVERYへ戻す。
      const bool actual_overtake_active =
          mode_ == BehaviorMode::OVERTAKE_LEFT ||
          mode_ == BehaviorMode::OVERTAKE_RIGHT;
      const bool pass_side_reselection_allowed =
          localized_lateral_profile_.active &&
          !localized_lateral_profile_.pass_safety_approved_once &&
          !actual_overtake_active;
      const bool bounded_lateral_first_creep_proposal =
          blocked.start_grid_target_active &&
          blocked.pass_lateral_first_speed_gate_active &&
          blocked.pass_proposal_acceleration_allowed;
      // start-grid初回、release warm-up、commit済みspatial
      // snapshotのPASSだけは、
      // 実行authorityではなく加速s(t)をSafetyEvaluatorへ渡すproposalとして
      // 評価する。通常PASSは従来どおりEXECUTIONのgateを使う。
      const CandidatePurpose pass_probe_purpose =
          (initial_start_grid_pass_prediction_bootstrap ||
           pass_release_warmup_prediction_authorized ||
           committed_spatial_prediction_bootstrap_authorized ||
           bounded_lateral_first_creep_proposal)
              ? CandidatePurpose::PROPOSAL_SAFETY_EVALUATION
              : CandidatePurpose::EXECUTION;
      const auto make_pass_probe_candidate =
          [&](CandidateType pass_type) -> CandidateTrajectory {
        if (!pass_side_reselection_allowed ||
            pass_type == localized_lateral_profile_.pass_type ||
            blocked.maneuver_target_index < 0 ||
            static_cast<std::size_t>(blocked.maneuver_target_index) >=
                opponents.size()) {
          return CandidateBuilder(frame_, config_)
              .makeCandidate(pass_type, ego, blocked, opponents,
                             localized_lateral_profile_.active
                                 ? &localized_lateral_profile_
                                 : nullptr,
                             false, pass_probe_purpose);
        }
        // 反対側も、固定offsetの短い見かけ上の候補ではなく、同じtarget/chain
        // markerを持つlocalized profileとしてpreflight・SafetyEvaluatorへ通す。
        LocalizedLateralProfile alternate_profile = localized_lateral_profile_;
        alternate_profile.pass_type = pass_type;
        const auto &target =
            opponents[static_cast<std::size_t>(blocked.maneuver_target_index)];
        alternate_profile.target_d_m =
            targetOffsetForPass(pass_type, alternate_profile.start_d_m,
                                predicted_target_d_for_pass(pass_type, target));
        double staged_target_d_m = alternate_profile.target_d_m;
        for (auto &waypoint : alternate_profile.chain_waypoints) {
          const auto opponent_it =
              std::find_if(opponents.begin(), opponents.end(),
                           [&](const OpponentState &opponent) {
                             return opponent.id == waypoint.target_id;
                           });
          if (opponent_it == opponents.end()) {
            waypoint.target_d_m = std::numeric_limits<double>::quiet_NaN();
            continue;
          }
          const double waypoint_target_d_m = targetOffsetForPass(
              pass_type, alternate_profile.start_d_m,
              predicted_target_d_for_pass(pass_type, *opponent_it));
          staged_target_d_m =
              pass_type == CandidateType::PASS_LEFT
                  ? std::max(staged_target_d_m, waypoint_target_d_m)
                  : std::min(staged_target_d_m, waypoint_target_d_m);
          waypoint.target_d_m = staged_target_d_m;
        }
        const double configured_start_before_m =
            std::max(0.0, config_.localized_avoidance_start_before_target_m);
        const double configured_full_before_m = std::max(
            0.0, config_.localized_avoidance_full_offset_before_target_m);
        const double requested_transition_m =
            std::abs(configured_start_before_m - configured_full_before_m);
        double tracking_speed_cap_mps =
            std::max(0.0, std::max(ego.v, target.v));
        if (blocked.pass_lateral_first_speed_gate_active &&
            std::isfinite(blocked.pass_lateral_first_speed_cap_mps) &&
            blocked.pass_lateral_first_speed_cap_mps > 0.0) {
          tracking_speed_cap_mps = std::min(
              tracking_speed_cap_mps, blocked.pass_lateral_first_speed_cap_mps);
        }
        const double required_transition_m =
            CandidateBuilder(frame_, config_)
                .minimumTrackableLateralShiftDistance(
                    ego, alternate_profile.target_d_m, tracking_speed_cap_mps,
                    blocked, std::max(1.0, requested_transition_m));
        const double target_gap_m =
            alternate_profile.target_s_m - alternate_profile.anchor_s_m;
        const double maximum_full_before_m =
            target_gap_m - required_transition_m;
        alternate_profile.full_offset_before_target_m =
            std::isfinite(required_transition_m) && maximum_full_before_m >= 0.0
                ? std::clamp(configured_full_before_m, 0.0,
                             maximum_full_before_m)
                : 0.0;
        alternate_profile.avoid_start_before_target_m =
            std::isfinite(required_transition_m)
                ? std::max(
                      alternate_profile.full_offset_before_target_m,
                      std::min(
                          target_gap_m,
                          std::max(
                              configured_start_before_m,
                              alternate_profile.full_offset_before_target_m +
                                  required_transition_m)))
                : std::numeric_limits<double>::infinity();
        alternate_profile.full_offset_start_s_m =
            alternate_profile.target_s_m -
            alternate_profile.full_offset_before_target_m;
        alternate_profile.avoid_start_s_m =
            alternate_profile.target_s_m -
            alternate_profile.avoid_start_before_target_m;
        alternate_profile.pass_safety_approved_once = false;
        alternate_profile.pass_complete_confirmed = false;
        CandidateTrajectory candidate =
            CandidateBuilder(frame_, config_)
                .makeCandidate(pass_type, ego, blocked, opponents,
                               &alternate_profile, false, pass_probe_purpose);
        if (pass_type == CandidateType::PASS_LEFT) {
          evaluated_alternate_left_profile = alternate_profile;
        } else {
          evaluated_alternate_right_profile = alternate_profile;
        }
        return candidate;
      };
      const bool evaluate_left_pass =
          (pass_side_reselection_allowed || !isRightPassMode(mode_)) &&
          (pass_side_reselection_allowed ||
           !blocked.maneuver_transaction_incomplete ||
           blocked.maneuver_transaction_pass_type ==
               CandidateType::PASS_LEFT) &&
          (blocked.can_pass_left || config_.dynamic_pass_candidate_enabled);
      const bool evaluate_right_pass =
          (pass_side_reselection_allowed || !isLeftPassMode(mode_)) &&
          (pass_side_reselection_allowed ||
           !blocked.maneuver_transaction_incomplete ||
           blocked.maneuver_transaction_pass_type ==
               CandidateType::PASS_RIGHT) &&
          (blocked.can_pass_right || config_.dynamic_pass_candidate_enabled);
      if (evaluate_left_pass) {
        candidates.push_back(
            make_pass_probe_candidate(CandidateType::PASS_LEFT));
        blocked.pass_left_candidate_generated = true;
      }
      if (evaluate_right_pass) {
        candidates.push_back(
            make_pass_probe_candidate(CandidateType::PASS_RIGHT));
        blocked.pass_right_candidate_generated = true;
      }
      candidates.push_back(
          makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
      if (blocked.early_stationary_parallel_pass_target) {
        // PASSがGate
        // 2で失格ならFASTESTへ戻さず、同周期に復帰候補も安全評価する。
        candidates.push_back(
            makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
      }
    }
  }
  if (currentPassGapLost(mode_, blocked)) {
    candidates.push_back(
        makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
  }
  if (isPassMode(mode_) || mode_ == BehaviorMode::ABORT_RECOVERY ||
      blocked.authorized_pass_current_d_hold_active) {
    // 追い越し中や中止中は、中心線へ戻るRECOVERY候補も常に評価する。
    // 認可済み包絡内の未完了transactionではFOLLOWへ落ちた後も現在d保持を
    // 評価し、対象後端を抜く前の中心復帰を防ぐ。
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }
  if (isAnyPassMode(mode_) &&
      stationary_no_pass_safe_pass_constraint_latched_) {
    // 禁止区間の停止障害物例外を継続中は、再認証を失った同周期にも
    // FASTEST/PASSへ戻らずに減速できる候補を必ずSafetyEvaluatorへ載せる。
    // 採用は下の再認証結果で決め、ここではYIELDの物理安全性だけを先に確定する。
    candidates.push_back(
        makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents));
  }
  if (!freeze_overtake_decisions &&
      !blocked.maneuver_unstarted_target_released && isLeftPassMode(mode_) &&
      (!blocked.maneuver_transaction_incomplete ||
       blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT) &&
      !blocked.pass_left_candidate_generated) {
    // 前方閉塞が一時的に解けても、走行中PASSの継続可否は同じ時系列安全評価で確認する。
    candidates.push_back(
        makeCandidate(CandidateType::PASS_LEFT, ego, blocked, opponents));
    blocked.pass_left_candidate_generated = true;
  }
  if (!freeze_overtake_decisions &&
      !blocked.maneuver_unstarted_target_released && isRightPassMode(mode_) &&
      (!blocked.maneuver_transaction_incomplete ||
       blocked.maneuver_transaction_pass_type == CandidateType::PASS_RIGHT) &&
      !blocked.pass_right_candidate_generated) {
    candidates.push_back(
        makeCandidate(CandidateType::PASS_RIGHT, ego, blocked, opponents));
    blocked.pass_right_candidate_generated = true;
  }
  const bool needs_safe_stop_fallback_check =
      needsSafeStopFallbackCheck(config_, mode_, blocked);
  const bool has_recovery_candidate =
      std::any_of(candidates.begin(), candidates.end(),
                  [](const CandidateTrajectory &candidate) {
                    return candidate.type == CandidateType::RECOVERY;
                  });
  if ((needs_safe_stop_fallback_check || blocked.braking_follow_active) &&
      !has_recovery_candidate) {
    // SAFE_STOP判定前に、通常fallbackであるRECOVERYも必ず安全評価へ含める。
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }
  if (blocked.start_grid_lateral_release_pending &&
      !std::any_of(candidates.begin(), candidates.end(),
                   [](const CandidateTrajectory &candidate) {
                     return candidate.type == CandidateType::RECOVERY;
                   })) {
    // start-gridの横位置を使った後は、対象がFOLLOW範囲から消えた周期でも
    // FASTESTへ直接戻さない。RECOVERY自身をSafetyEvaluatorへ通し、後段の
    // long-horizon generic reentry gateで許可された時だけ中心へ収束させる。
    candidates.push_back(
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents));
  }

  const bool stopped_non_mpc_pp_bootstrap_authorized =
      reentry_input.pure_pursuit_release_ready &&
      reentry_input.verified_non_mpc_pure_pursuit &&
      !reentry_input.mpc_hard_failure;
  const auto holds_current_lateral =
      [&ego](const CandidateTrajectory &candidate) {
        return std::isfinite(ego.frenet.d) && !candidate.d.empty() &&
               std::all_of(candidate.d.begin(), candidate.d.end(),
                           [&ego](double candidate_d_m) {
                             return std::isfinite(candidate_d_m) &&
                                    std::abs(candidate_d_m - ego.frenet.d) <=
                                        1.0e-6;
                           });
      };
  const bool authorized_hold_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && predictions.size() == opponents.size() &&
      std::all_of(predictions.begin(), predictions.end(),
                  [this](const auto &prediction) {
                    return prediction.t.size() == config_.horizon_points &&
                           prediction.x.size() == config_.horizon_points &&
                           prediction.y.size() == config_.horizon_points &&
                           prediction.s.size() == config_.horizon_points &&
                           prediction.d.size() == config_.horizon_points;
                  });
  const auto apply_committed_pass_spatial_profile = [&](CandidateTrajectory
                                                            &candidate) {
    const bool pass_candidate = candidate.type == CandidateType::PASS_LEFT ||
                                candidate.type == CandidateType::PASS_RIGHT;
    const double max_snapshot_age_sec =
        std::max(0.25, 2.0 / std::max(1.0, config_.control_rate_hz));
    const bool frozen_geometry_candidate =
        pass_candidate && committed_pass_snapshot_valid_ &&
        committed_pass_spatial_profile_frozen_ &&
        !start_grid_tracking_release_pending_ &&
        localized_lateral_profile_.active &&
        localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed &&
        committed_pass_snapshot_target_id_ ==
            localized_lateral_profile_.target_id &&
        committed_pass_snapshot_pass_type_ ==
            localized_lateral_profile_.pass_type &&
        candidate.type == committed_pass_snapshot_pass_type_;
    if (!frozen_geometry_candidate) {
      return false;
    }
    const auto reject_geometry = [&](const std::string &reason) {
      candidate.feasible = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.desired_path_trackable = false;
      candidate.pure_pursuit_command_trackable = false;
      candidate.reject_reason = reason;
      return false;
    };
    const bool dynamic_geometry_identity_complete =
        !blocked.maneuver_target_id.empty() &&
        blocked.maneuver_target_id == committed_pass_snapshot_target_id_ &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed;
    if (!dynamic_geometry_identity_complete) {
      return reject_geometry("committed_pass_geometry_identity_incomplete");
    }
    const auto same_geometry_value = [](double lhs, double rhs) {
      return std::isfinite(lhs) && std::isfinite(rhs) &&
             static_cast<float>(lhs) == static_cast<float>(rhs);
    };
    const bool committed_profile_identity_complete =
        committed_pass_profile_snapshot_.active &&
        committed_pass_profile_snapshot_.pass_type ==
            committed_pass_snapshot_pass_type_ &&
        committed_pass_profile_snapshot_.target_id ==
            committed_pass_snapshot_target_id_ &&
        localized_lateral_profile_.chain_tail_id ==
            committed_pass_profile_snapshot_.chain_tail_id &&
        same_geometry_value(localized_lateral_profile_.start_d_m,
                            committed_pass_profile_snapshot_.start_d_m) &&
        same_geometry_value(localized_lateral_profile_.target_d_m,
                            committed_pass_profile_snapshot_.target_d_m) &&
        localized_lateral_profile_.chain_waypoints.size() ==
            committed_pass_profile_snapshot_.chain_waypoints.size() &&
        std::equal(
            localized_lateral_profile_.chain_waypoints.begin(),
            localized_lateral_profile_.chain_waypoints.end(),
            committed_pass_profile_snapshot_.chain_waypoints.begin(),
            [&same_geometry_value](const auto &current, const auto &committed) {
              return current.target_id == committed.target_id &&
                     same_geometry_value(current.target_d_m,
                                         committed.target_d_m);
            });
    if (!committed_profile_identity_complete) {
      return reject_geometry("committed_pass_geometry_wire_mismatch");
    }
    if (!std::isfinite(committed_pass_snapshot_sec_) ||
        !std::isfinite(now_sec) || now_sec < committed_pass_snapshot_sec_) {
      return reject_geometry("committed_pass_geometry_time_invalid");
    }
    const double source_age_sec = now_sec - committed_pass_snapshot_sec_;
    blocked.committed_pass_spatial_profile_source_age_sec = source_age_sec;
    const bool short_snapshot_bootstrap =
        source_age_sec <= max_snapshot_age_sec;
    // geometry候補の生成条件へ「前周期にこの候補を実行した証明」を入れると、
    // topic配送順で一周期statusが欠けた時にSTOP→final sourceがSTOP→次周期も
    // 証明不能という循環になる。ここはfresh入力、実測d接続、静的controller
    // trackability、現周期SafetyEvaluatorで候補を作り、実行可否は既存の
    // ControllerTrackingStatus/constraint authorityへ一方向に委ねる。
    if (!authorized_hold_inputs_complete) {
      return reject_geometry("committed_pass_geometry_inputs_incomplete");
    }
    const bool current_longitudinal_contract_usable =
        candidate.longitudinal_profile_valid &&
        candidate.pass_target_corridor_valid &&
        std::isfinite(candidate.required_controller_spatial_horizon_m) &&
        !candidate.longitudinal_offsets_m.empty() &&
        candidate.longitudinal_offsets_m.back() + 1.0e-6 >=
            candidate.required_controller_spatial_horizon_m;
    if (!current_longitudinal_contract_usable) {
      return reject_geometry("committed_pass_geometry_longitudinal_invalid");
    }
    const bool wire_shape_compatible =
        committed_pass_snapshot_.d.size() == candidate.d.size() &&
        !committed_pass_snapshot_.d.empty() &&
        std::isfinite(committed_pass_snapshot_.planned_target_d_m) &&
        std::isfinite(candidate.planned_target_d_m) &&
        static_cast<float>(committed_pass_snapshot_.planned_target_d_m) ==
            static_cast<float>(candidate.planned_target_d_m);
    if (!wire_shape_compatible) {
      return reject_geometry("committed_pass_geometry_wire_mismatch");
    }

    if (committed_pass_snapshot_.s.empty() ||
        committed_pass_snapshot_.longitudinal_offsets_m.size() !=
            committed_pass_snapshot_.d.size() ||
        !std::isfinite(committed_pass_snapshot_ego_unwrapped_s_m_) ||
        !std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) ||
        !std::isfinite(ego.frenet.s) || !std::isfinite(ego.frenet.d)) {
      return reject_geometry("committed_pass_geometry_state_invalid");
    }
    // 初回に固定したwireをunwrapped s上で前進させる。wrapped sの半周折返しや、
    // receding horizonごとのraw再アンカーで横移動開始点を前へ逃がさない。
    const double signed_progress_m =
        localized_lateral_profile_.ego_unwrapped_s_m -
        committed_pass_snapshot_ego_unwrapped_s_m_;
    const double backward_noise_tolerance_m = 0.05;
    const double snapshot_initial_speed_mps =
        committed_pass_snapshot_.longitudinal_initial_measured_speed_mps;
    const double bounded_speed_mps =
        std::max({0.0, std::abs(ego.v), std::abs(snapshot_initial_speed_mps)});
    const double localization_tolerance_m =
        std::max(0.10, config_.wall_localization_uncertainty_m);
    const double physically_reachable_progress_m =
        bounded_speed_mps * source_age_sec +
        0.5 * std::max(0.0, config_.pass_assumed_accel_mps2) * source_age_sec *
            source_age_sec +
        localization_tolerance_m;
    if (!std::isfinite(signed_progress_m) || !std::isfinite(source_age_sec) ||
        source_age_sec < 0.0 || !std::isfinite(ego.v) ||
        !std::isfinite(snapshot_initial_speed_mps) ||
        !std::isfinite(bounded_speed_mps) ||
        !std::isfinite(physically_reachable_progress_m) ||
        signed_progress_m < -backward_noise_tolerance_m ||
        signed_progress_m > physically_reachable_progress_m + 1.0e-6) {
      return reject_geometry("committed_pass_geometry_progress_invalid");
    }
    const double progress_m = std::max(0.0, signed_progress_m);
    const auto stored_current_d =
        sampleCommittedPassSpatialProfileD(progress_m);
    if (!stored_current_d.has_value() || !std::isfinite(*stored_current_d)) {
      return reject_geometry("committed_pass_geometry_sample_invalid");
    }
    const double tracking_error_m = ego.frenet.d - *stored_current_d;
    blocked.committed_pass_spatial_profile_tracking_error_m =
        std::abs(tracking_error_m);
    // staged chainではscalar planned_target_d_mは現在target（d2）の値で、
    // 後続d3/d4へ向けた正当な外側移動を「d2目標のovershoot」と誤認しては
    // ならない。初回に固定した全waypointの最外側dをtransaction包絡端にする。
    double directional_target_d_m = committed_pass_profile_snapshot_.target_d_m;
    for (const auto &waypoint :
         committed_pass_profile_snapshot_.chain_waypoints) {
      if (!std::isfinite(waypoint.target_d_m)) {
        return reject_geometry("committed_pass_geometry_tracking_error");
      }
      if (committed_pass_snapshot_pass_type_ == CandidateType::PASS_LEFT) {
        directional_target_d_m =
            std::max(directional_target_d_m, waypoint.target_d_m);
      } else {
        directional_target_d_m =
            std::min(directional_target_d_m, waypoint.target_d_m);
      }
    }
    const double pass_direction =
        committed_pass_snapshot_pass_type_ == CandidateType::PASS_LEFT ? 1.0
                                                                       : -1.0;
    const double directional_displacement_m =
        pass_direction *
        (directional_target_d_m - committed_pass_profile_snapshot_.start_d_m);
    if (!std::isfinite(tracking_error_m) ||
        !std::isfinite(directional_target_d_m) ||
        !std::isfinite(directional_displacement_m) ||
        directional_displacement_m <= 1.0e-6) {
      return reject_geometry("committed_pass_geometry_tracking_error");
    }
    const double directed_target_overshoot_m =
        pass_direction * (ego.frenet.d - directional_target_d_m);
    const double directional_tracking_advance_m =
        pass_direction * (ego.frenet.d - *stored_current_d);
    const double directional_tracking_lag_m =
        pass_direction * (*stored_current_d - ego.frenet.d);
    const bool excessive_target_overshoot =
        directed_target_overshoot_m >
        config_.safe_stop_lateral_error_threshold_m;
    const bool excessive_tracking_lag =
        directional_tracking_lag_m >
        config_.safe_stop_lateral_error_threshold_m;
    if (excessive_target_overshoot || excessive_tracking_lag) {
      return reject_geometry("committed_pass_geometry_tracking_error");
    }

    // 固定wireとの差だけではtracking failureとしない。候補先頭を実測dへ
    // 合わせるconnectorを含む、実際にpublishする最終形状のtrackabilityで
    // 可否を決める。さらに全点のcorridor/相手安全性を現周期SafetyEvaluatorへ
    // 通すため、固定形状との差を無条件に許可するものではない。

    // 横形状だけを固定transaction geometryから引き継ぐ。t/ds/v/predictedは
    // 現周期の実測速度と加減速契約から生成した値を保持し、古い縦予測や
    // SafetyEvaluator結果は一切再利用しない。先頭だけは実測ego.dへ厳密接続し、
    // その補正自体も下のtrackabilityとSafetyEvaluatorへ通す。
    CandidateTrajectory continuity = candidate;
    continuity.d.clear();
    continuity.d.reserve(candidate.longitudinal_offsets_m.size());
    const double connector_distance_m =
        std::max(1.0, config_.lateral_override_lookahead_min_distance_m);
    const auto smootherstep = [](double value) {
      const double z = std::clamp(value, 0.0, 1.0);
      return z * z * z * (z * (z * 6.0 - 15.0) + 10.0);
    };
    for (const double new_offset_m : candidate.longitudinal_offsets_m) {
      const auto sampled_d =
          sampleCommittedPassSpatialProfileD(progress_m + new_offset_m);
      if (!sampled_d.has_value() || !std::isfinite(*sampled_d)) {
        return reject_geometry("committed_pass_geometry_sample_invalid");
      }
      const double connector_ratio =
          smootherstep(new_offset_m / connector_distance_m);
      const double connected_d_m =
          *sampled_d + tracking_error_m * (1.0 - connector_ratio);
      // 未完了PASS中に実車が固定wireより外側へ進んだ周期は、点単位clampで
      // wireとの交点に折れを作らず、全horizonをcurrent-d保持する。固定wireが
      // 追いついた後にconnectorを再開し、どちらも下でtrackability、corridor、
      // 全相手SafetyEvaluatorへ改めて通す。
      continuity.d.push_back(directional_tracking_advance_m > 1.0e-6
                                 ? ego.frenet.d
                                 : connected_d_m);
    }
    continuity.safety_evaluated = false;
    continuity.feasible = false;
    continuity.reject_reason.clear();
    const bool rebased = rebaseCandidateToCurrentEgo(continuity, ego, frame_);
    const bool trackable =
        rebased && CandidateBuilder(frame_, config_)
                       .publishedLateralProfileTrackable(continuity, ego);
    continuity.desired_path_trackable = trackable;
    continuity.pure_pursuit_command_trackable = trackable;
    continuity.controller_tracking_profile_valid = trackable;
    if (!trackable) {
      // ACK完了直後など、固定snapshotからの実進捗がまだ無い周期は、実際に
      // PPへpublishして追従性を確認したfull wireを一周期handoffへ使える。
      // 進捗後の再anchorには使わず、現在egoへrebaseした上でtrackabilityと
      // 現周期の全相手SafetyEvaluatorを必ず再実行する。
      constexpr double kSnapshotWireHandoffProgressToleranceM = 0.05;
      CandidateTrajectory snapshot_handoff = committed_pass_snapshot_;
      const bool snapshot_handoff_connector_valid =
          snapshot_handoff.d.size() ==
              snapshot_handoff.longitudinal_offsets_m.size() &&
          !snapshot_handoff.d.empty() && std::isfinite(ego.frenet.d) &&
          std::isfinite(snapshot_handoff.d.front());
      if (snapshot_handoff_connector_valid) {
        const double snapshot_tracking_error_m =
            ego.frenet.d - snapshot_handoff.d.front();
        for (std::size_t index = 0U; index < snapshot_handoff.d.size();
             ++index) {
          const double connector_ratio =
              smootherstep(snapshot_handoff.longitudinal_offsets_m[index] /
                           connector_distance_m);
          snapshot_handoff.d[index] +=
              snapshot_tracking_error_m * (1.0 - connector_ratio);
        }
        // 丸め誤差を含めて実測poseへ厳密接続する。このconnectorを含む最終wireを
        // 下のtrackabilityとSafetyEvaluatorへ渡し、旧snapshot先頭からの評価で
        // 実走行区間を飛ばさない。
        snapshot_handoff.d.front() = ego.frenet.d;
      }
      const bool snapshot_handoff_rebased =
          snapshot_handoff_connector_valid &&
          progress_m <= kSnapshotWireHandoffProgressToleranceM &&
          candidateStartsFromCurrentEgoSpeed(snapshot_handoff, ego) &&
          rebaseCandidateToCurrentEgo(snapshot_handoff, ego, frame_);
      const bool snapshot_handoff_trackable =
          snapshot_handoff_rebased &&
          CandidateBuilder(frame_, config_)
              .publishedLateralProfileTrackable(snapshot_handoff, ego);
      snapshot_handoff.desired_path_trackable = snapshot_handoff_trackable;
      snapshot_handoff.pure_pursuit_command_trackable =
          snapshot_handoff_trackable;
      snapshot_handoff.controller_tracking_profile_valid =
          snapshot_handoff_trackable;
      if (snapshot_handoff_trackable) {
        snapshot_handoff.safety_evaluated = false;
        snapshot_handoff.feasible = false;
        snapshot_handoff.reject_reason.clear();
        evaluateCandidateAtOwnTimeAxis(snapshot_handoff, opponents, now_sec,
                                       predictions);
      }
      const bool snapshot_handoff_authorized =
          snapshot_handoff.safety_evaluated && snapshot_handoff.feasible &&
          snapshot_handoff.longitudinal_profile_valid &&
          snapshot_handoff.controller_tracking_profile_valid &&
          snapshot_handoff.pass_target_corridor_valid;
      if (snapshot_handoff_authorized) {
        candidate = std::move(snapshot_handoff);
        blocked.committed_pass_snapshot_continuity_used = true;
        blocked.committed_pass_spatial_profile_continuity_used = true;
        return true;
      }
      candidate = std::move(continuity);
      return reject_geometry("untrackable_lateral_profile");
    }
    evaluateCandidateAtOwnTimeAxis(continuity, opponents, now_sec, predictions);
    const bool freshly_authorized =
        continuity.safety_evaluated && continuity.feasible &&
        continuity.longitudinal_profile_valid &&
        continuity.controller_tracking_profile_valid &&
        continuity.pass_target_corridor_valid;
    if (!freshly_authorized) {
      candidate = std::move(continuity);
      return false;
    }
    candidate = std::move(continuity);
    blocked.committed_pass_snapshot_continuity_used = short_snapshot_bootstrap;
    blocked.committed_pass_spatial_profile_continuity_used = true;
    return true;
  };
  // 処理ブロック: 全候補を安全評価してscoreを付ける。
  // 設計意図:
  // 先にfeasibleを確定し、その後で追い越し意欲やfallback優先度を比較する。
  for (auto &candidate : candidates) {
    // 壁/他車との安全余裕を見てから、減速モデルが有効かも同時に確認する。
    evaluateCandidateAtOwnTimeAxis(candidate, opponents, now_sec, predictions);
    if (!candidate.pass_transition_deadline_reachable) {
      candidate.feasible = false;
      candidate.reject_reason = "pass_transition_deadline_unreachable";
    }
    apply_committed_pass_spatial_profile(candidate);
    const bool committed_attack_follow_identity_complete =
        candidate.type == CandidateType::FOLLOW &&
        blocked.attack_follow_hold_pass_side &&
        blocked.maneuver_transaction_incomplete &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        blocked.maneuver_transaction_pass_type ==
            localized_lateral_profile_.pass_type &&
        (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT ||
         blocked.maneuver_transaction_pass_type == CandidateType::PASS_RIGHT) &&
        !blocked.maneuver_target_id.empty() &&
        blocked.maneuver_target_id == localized_lateral_profile_.target_id &&
        localized_lateral_profile_.active &&
        localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed &&
        std::isfinite(candidate.committed_attack_follow_target_d_m) &&
        std::isfinite(localized_lateral_profile_.target_d_m) &&
        std::abs(candidate.committed_attack_follow_target_d_m -
                 localized_lateral_profile_.target_d_m) <= 1.0e-6;
    const bool current_d_hold_inputs_complete =
        authorized_hold_inputs_complete && reentry_input.mpc_healthy &&
        reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure &&
        !reentry_phase_active_ && !reentry_lockout_active_ &&
        !blocked.pass_reauthorization_lockout_active &&
        !start_grid_tracking_release_pending_ && !blocked.side_by_side &&
        !blocked.corner_side_by_side && !blocked.future_yield_required &&
        !blocked.future_corner_side_by_side && !blocked.future_outer_wall_risk;
    const bool committed_attack_follow_recoverable_reject =
        candidate.reject_reason == "opponent_collision" ||
        candidate.reject_reason == "wall_footprint_margin";
    if (committed_attack_follow_identity_complete &&
        current_d_hold_inputs_complete && candidate.safety_evaluated &&
        !candidate.feasible && committed_attack_follow_recoverable_reject) {
      // 失敗したtarget-d reconnectをfeasibleへ戻さない。同じcandidateの
      // t/s(t)/速度列をコピーした実測current-d別候補を生成し、壁・全相手・
      // controller追従性を同周期に最初から評価する。commit済みFOLLOWが
      // wall footprintで直接棄却された場合も同じ入口を使うが、後段の
      // current-d hold / inward connectorが全評価を通る周期だけ採用する。
      blocked.attack_follow_current_d_hold_variant_generated = true;
      blocked.attack_follow_current_d_hold_source_blocking_opponent_id =
          candidate.blocking_opponent_id;
      blocked.attack_follow_current_d_hold_source_blocking_time_sec =
          candidate.blocking_time_sec;
      const CandidateTrajectory rejected_attack_follow = candidate;
      CandidateTrajectory current_d_hold =
          CandidateBuilder(frame_, config_)
              .makeAttackFollowCurrentDHoldVariant(rejected_attack_follow, ego);
      const bool longitudinal_contract_unchanged =
          current_d_hold.t == rejected_attack_follow.t &&
          current_d_hold.longitudinal_offsets_m ==
              rejected_attack_follow.longitudinal_offsets_m &&
          current_d_hold.s == rejected_attack_follow.s &&
          current_d_hold.predicted_speed_mps ==
              rejected_attack_follow.predicted_speed_mps &&
          current_d_hold.v_ref == rejected_attack_follow.v_ref;
      if (!longitudinal_contract_unchanged) {
        current_d_hold.feasible = false;
        current_d_hold.controller_tracking_profile_valid = false;
        current_d_hold.reject_reason =
            "attack_follow_current_d_hold_longitudinal_mismatch";
      } else {
        evaluateCandidateAtOwnTimeAxis(current_d_hold, opponents, now_sec,
                                       predictions);
      }
      const bool current_d_hold_verified =
          longitudinal_contract_unchanged && current_d_hold.safety_evaluated &&
          current_d_hold.feasible &&
          current_d_hold.longitudinal_profile_valid &&
          current_d_hold.pass_target_corridor_valid &&
          current_d_hold.controller_tracking_profile_valid &&
          holds_current_lateral(current_d_hold);
      blocked.attack_follow_current_d_hold_variant_feasible =
          current_d_hold_verified;
      blocked.attack_follow_current_d_hold_variant_reject_reason =
          current_d_hold.reject_reason;
      const bool current_d_hold_wall_diagnostic_valid =
          current_d_hold.safety_evaluated && !current_d_hold.feasible &&
          current_d_hold.reject_reason == "wall_footprint_margin" &&
          current_d_hold.blocking_wall_footprint_valid;
      if (current_d_hold_wall_diagnostic_valid) {
        blocked.attack_follow_current_d_hold_blocking_wall_footprint_valid =
            true;
        blocked.attack_follow_current_d_hold_blocking_wall_segment_index =
            current_d_hold.blocking_wall_segment_index;
        blocked.attack_follow_current_d_hold_blocking_wall_segment_ratio =
            current_d_hold.blocking_wall_segment_ratio;
        blocked.attack_follow_current_d_hold_blocking_wall_corner_index =
            current_d_hold.blocking_wall_corner_index;
        blocked.attack_follow_current_d_hold_blocking_wall_time_sec =
            current_d_hold.blocking_wall_time_sec;
        blocked.attack_follow_current_d_hold_blocking_wall_candidate_x_m =
            current_d_hold.blocking_wall_candidate_x_m;
        blocked.attack_follow_current_d_hold_blocking_wall_candidate_y_m =
            current_d_hold.blocking_wall_candidate_y_m;
        blocked.attack_follow_current_d_hold_blocking_wall_candidate_yaw_rad =
            current_d_hold.blocking_wall_candidate_yaw_rad;
        blocked.attack_follow_current_d_hold_blocking_wall_candidate_s_m =
            current_d_hold.blocking_wall_candidate_s_m;
        blocked.attack_follow_current_d_hold_blocking_wall_candidate_d_m =
            current_d_hold.blocking_wall_candidate_d_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corner_x_m =
            current_d_hold.blocking_wall_corner_x_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corner_y_m =
            current_d_hold.blocking_wall_corner_y_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corner_s_m =
            current_d_hold.blocking_wall_corner_s_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corner_d_m =
            current_d_hold.blocking_wall_corner_d_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corridor_d_min_m =
            current_d_hold.blocking_wall_corridor_d_min_m;
        blocked.attack_follow_current_d_hold_blocking_wall_corridor_d_max_m =
            current_d_hold.blocking_wall_corridor_d_max_m;
        blocked
            .attack_follow_current_d_hold_blocking_wall_physical_clearance_m =
            current_d_hold.blocking_wall_physical_clearance_m;
        blocked
            .attack_follow_current_d_hold_blocking_wall_effective_clearance_m =
            current_d_hold.blocking_wall_effective_clearance_m;
        evaluateAttackFollowInnerBandDiagnostic(rejected_attack_follow,
                                                current_d_hold, ego, opponents,
                                                now_sec, predictions);

        // C-002AK: C-002AIの1 Hz shadow reportは決して認可根拠に使わない。
        // 現周期のwall rejectから境界方向を決め、小さい内側量から順に
        // connectorを新規生成する。各候補を全相手・壁・追従性へ通し、
        // 最初に成立した候補だけを、その周期の最小成立量として採用する。
        constexpr std::array<double, 4> kInwardConnectorShiftsM{0.15, 0.30,
                                                                0.45, 0.60};
        constexpr double kBoundaryAmbiguityM = 1.0e-6;
        constexpr double kCenterReserveM = 0.05;
        const bool connector_wall_signature_complete =
            std::isfinite(current_d_hold.blocking_wall_corner_d_m) &&
            std::isfinite(current_d_hold.blocking_wall_corridor_d_min_m) &&
            std::isfinite(current_d_hold.blocking_wall_corridor_d_max_m) &&
            std::isfinite(ego.frenet.d);
        if (connector_wall_signature_complete) {
          const double lower_distance_m =
              std::abs(current_d_hold.blocking_wall_corner_d_m -
                       current_d_hold.blocking_wall_corridor_d_min_m);
          const double upper_distance_m =
              std::abs(current_d_hold.blocking_wall_corridor_d_max_m -
                       current_d_hold.blocking_wall_corner_d_m);
          const bool direction_unambiguous =
              std::isfinite(lower_distance_m) &&
              std::isfinite(upper_distance_m) &&
              std::abs(lower_distance_m - upper_distance_m) >
                  kBoundaryAmbiguityM;
          if (direction_unambiguous) {
            const int inward_direction_sign =
                upper_distance_m < lower_distance_m ? -1 : 1;
            blocked.attack_follow_inward_connector_direction_sign =
                inward_direction_sign;
            for (const double inward_shift_m : kInwardConnectorShiftsM) {
              const double terminal_d_m =
                  ego.frenet.d +
                  static_cast<double>(inward_direction_sign) * inward_shift_m;
              const bool stays_outside_center_reserve =
                  inwardConnectorStaysOutsideCenterReserve(
                      ego.frenet.d, terminal_d_m, kCenterReserveM);
              if (!stays_outside_center_reserve ||
                  !std::isfinite(terminal_d_m) ||
                  std::abs(std::abs(terminal_d_m - ego.frenet.d) -
                           inward_shift_m) > 1.0e-12) {
                blocked.attack_follow_inward_connector_variant_reject_reason =
                    "attack_follow_inward_connector_center_crossing";
                break;
              }

              blocked.attack_follow_inward_connector_variant_generated = true;
              blocked.attack_follow_inward_connector_terminal_d_m =
                  terminal_d_m;
              CandidateTrajectory connector =
                  CandidateBuilder(frame_, config_)
                      .makeAttackFollowInwardConnectorVariant(
                          rejected_attack_follow, ego, terminal_d_m);
              const bool longitudinal_contract_unchanged =
                  connector.t == rejected_attack_follow.t &&
                  connector.longitudinal_offsets_m ==
                      rejected_attack_follow.longitudinal_offsets_m &&
                  connector.s == rejected_attack_follow.s &&
                  connector.predicted_speed_mps ==
                      rejected_attack_follow.predicted_speed_mps &&
                  connector.v_ref == rejected_attack_follow.v_ref &&
                  connector.committed_attack_follow_target_d_m ==
                      rejected_attack_follow.committed_attack_follow_target_d_m;
              const bool connector_shape_valid =
                  longitudinal_contract_unchanged && !connector.d.empty() &&
                  connector.d.size() == rejected_attack_follow.d.size() &&
                  std::abs(connector.d.front() - ego.frenet.d) <= 1.0e-9 &&
                  std::abs(connector.d.back() - terminal_d_m) <= 1.0e-9 &&
                  std::all_of(connector.d.cbegin(), connector.d.cend(),
                              [](double d_m) { return std::isfinite(d_m); }) &&
                  std::adjacent_find(
                      connector.d.cbegin(), connector.d.cend(),
                      [inward_direction_sign](double previous_d_m,
                                              double next_d_m) {
                        return static_cast<double>(inward_direction_sign) *
                                   (next_d_m - previous_d_m) <
                               -1.0e-12;
                      }) == connector.d.cend();
              if (!connector_shape_valid) {
                connector.feasible = false;
                connector.controller_tracking_profile_valid = false;
                connector.reject_reason =
                    "attack_follow_inward_connector_contract_mismatch";
              } else {
                evaluateCandidateAtOwnTimeAxis(connector, opponents, now_sec,
                                               predictions);
              }
              const bool connector_verified =
                  connector_shape_valid && connector.safety_evaluated &&
                  connector.feasible && connector.longitudinal_profile_valid &&
                  connector.pass_target_corridor_valid &&
                  connector.controller_tracking_profile_valid &&
                  connector.desired_path_trackable &&
                  connector.pure_pursuit_command_trackable &&
                  connector.attack_follow_safe_lateral_hold &&
                  !connector.attack_follow_opponent_collision_current_d_hold &&
                  std::isfinite(connector.planned_target_d_m) &&
                  std::abs(connector.planned_target_d_m - terminal_d_m) <=
                      1.0e-9;
              blocked.attack_follow_inward_connector_variant_feasible =
                  connector_verified;
              blocked.attack_follow_inward_connector_variant_reject_reason =
                  connector.reject_reason;
              if (connector_verified) {
                connector.attack_follow_opponent_collision_inward_connector =
                    true;
                candidate = std::move(connector);
                blocked.attack_follow_inward_connector_variant_used = true;
                break;
              }
            }

            // 固定幅connectorが全て不成立でも、同じ前方targetへの縦FOLLOWを
            // 捨てて通常RECOVERYへ落とさない。壁から遠ざかる方向が基準線側と
            // 一致する場合だけ、actual poseからd=0へ接続する独立FOLLOWを作る。
            // 元candidateのtime/s/v_refとtransaction target/sideは保持し、
            // 全相手・壁・corridor・PP追従性を同周期に再評価する。成立しなければ
            // 後段の未完了PASS current-d STOP契約へ閉じる。
            // Core結合fixtureでtransport authorityまで証明できるまでは、
            // Builderの純粋契約だけを保持してlive選択へ接続しない。
            constexpr bool kBaselineFollowRuntimeEnabled = false;
            const int baseline_direction_sign =
                ego.frenet.d > kCenterReserveM    ? -1
                : ego.frenet.d < -kCenterReserveM ? 1
                                                  : 0;
            const bool baseline_follow_direction_valid =
                kBaselineFollowRuntimeEnabled &&
                !blocked.attack_follow_inward_connector_variant_used &&
                baseline_direction_sign != 0 &&
                baseline_direction_sign == inward_direction_sign;
            if (baseline_follow_direction_valid) {
              constexpr double kBaselineTargetDM = 0.0;
              double baseline_evaluation_speed_mps =
                  std::max(1.0e-3, std::max(0.0, ego.v));
              for (const double speed_mps : rejected_attack_follow.v_ref) {
                if (std::isfinite(speed_mps) && speed_mps > 0.0) {
                  baseline_evaluation_speed_mps =
                      std::max(baseline_evaluation_speed_mps, speed_mps);
                }
              }
              for (const double speed_mps :
                   rejected_attack_follow.predicted_speed_mps) {
                if (std::isfinite(speed_mps) && speed_mps > 0.0) {
                  baseline_evaluation_speed_mps =
                      std::max(baseline_evaluation_speed_mps, speed_mps);
                }
              }
              const CandidateBuilder baseline_builder(frame_, config_);
              const double baseline_join_distance_m =
                  baseline_builder.minimumTrackableLateralShiftDistance(
                      ego, kBaselineTargetDM, baseline_evaluation_speed_mps,
                      blocked, 1.0);
              const bool baseline_join_available =
                  !rejected_attack_follow.longitudinal_offsets_m.empty() &&
                  std::isfinite(baseline_join_distance_m) &&
                  baseline_join_distance_m > 1.0e-6 &&
                  baseline_join_distance_m <=
                      rejected_attack_follow.longitudinal_offsets_m.back() +
                          1.0e-9;
              CandidateTrajectory baseline_follow =
                  baseline_join_available
                      ? baseline_builder.makeAttackFollowInwardConnectorVariant(
                            rejected_attack_follow, ego, kBaselineTargetDM,
                            baseline_join_distance_m)
                      : CandidateTrajectory{};
              if (!baseline_join_available) {
                baseline_follow.type = CandidateType::FOLLOW;
                baseline_follow.feasible = false;
                baseline_follow.pass_target_corridor_valid = false;
                baseline_follow.controller_tracking_profile_valid = false;
                baseline_follow.desired_path_trackable = false;
                baseline_follow.pure_pursuit_command_trackable = false;
                baseline_follow.reject_reason =
                    "attack_follow_baseline_spatial_horizon_unavailable";
              }
              const bool longitudinal_contract_unchanged =
                  baseline_follow.t == rejected_attack_follow.t &&
                  baseline_follow.longitudinal_offsets_m ==
                      rejected_attack_follow.longitudinal_offsets_m &&
                  baseline_follow.s == rejected_attack_follow.s &&
                  baseline_follow.predicted_speed_mps ==
                      rejected_attack_follow.predicted_speed_mps &&
                  baseline_follow.v_ref == rejected_attack_follow.v_ref &&
                  baseline_follow.committed_attack_follow_target_d_m ==
                      rejected_attack_follow.committed_attack_follow_target_d_m;
              const bool baseline_shape_valid =
                  longitudinal_contract_unchanged &&
                  !baseline_follow.d.empty() &&
                  baseline_follow.d.size() == rejected_attack_follow.d.size() &&
                  std::abs(baseline_follow.d.front() - ego.frenet.d) <=
                      1.0e-9 &&
                  std::abs(baseline_follow.d.back() - kBaselineTargetDM) <=
                      1.0e-9 &&
                  std::all_of(baseline_follow.d.cbegin(),
                              baseline_follow.d.cend(),
                              [](double d_m) { return std::isfinite(d_m); }) &&
                  std::adjacent_find(
                      baseline_follow.d.cbegin(), baseline_follow.d.cend(),
                      [baseline_direction_sign](double previous_d_m,
                                                double next_d_m) {
                        return static_cast<double>(baseline_direction_sign) *
                                   (next_d_m - previous_d_m) <
                               -1.0e-12;
                      }) == baseline_follow.d.cend();
              if (!baseline_shape_valid) {
                baseline_follow.feasible = false;
                baseline_follow.controller_tracking_profile_valid = false;
                baseline_follow.reject_reason =
                    "attack_follow_baseline_contract_mismatch";
              } else {
                evaluateCandidateAtOwnTimeAxis(baseline_follow, opponents,
                                               now_sec, predictions);
              }
              const bool baseline_follow_verified =
                  baseline_shape_valid && baseline_follow.safety_evaluated &&
                  baseline_follow.feasible &&
                  baseline_follow.longitudinal_profile_valid &&
                  baseline_follow.pass_target_corridor_valid &&
                  baseline_follow.controller_tracking_profile_valid &&
                  baseline_follow.desired_path_trackable &&
                  baseline_follow.pure_pursuit_command_trackable &&
                  baseline_follow.attack_follow_safe_lateral_hold &&
                  !baseline_follow
                       .attack_follow_opponent_collision_current_d_hold &&
                  std::isfinite(baseline_follow.planned_target_d_m) &&
                  std::abs(baseline_follow.planned_target_d_m -
                           kBaselineTargetDM) <= 1.0e-9;
              if (baseline_follow_verified) {
                // Transport上は同一target/sideを保持する内向きFOLLOWとして扱う。
                // 通常RECOVERYや新しいPASS generationへは変換しない。
                baseline_follow
                    .attack_follow_opponent_collision_inward_connector = true;
                candidate = std::move(baseline_follow);
                blocked.attack_follow_inward_connector_terminal_d_m =
                    kBaselineTargetDM;
                blocked.attack_follow_inward_connector_variant_feasible = true;
                blocked.attack_follow_inward_connector_variant_reject_reason
                    .clear();
                blocked.attack_follow_inward_connector_variant_used = true;
              }
            }
          } else {
            blocked.attack_follow_inward_connector_variant_reject_reason =
                "attack_follow_inward_connector_ambiguous_wall_boundary";
          }
        } else {
          blocked.attack_follow_inward_connector_variant_reject_reason =
              "attack_follow_inward_connector_invalid_wall_signature";
        }
      }
      if (current_d_hold_verified) {
        candidate = std::move(current_d_hold);
        blocked.attack_follow_current_d_hold_variant_used = true;
      }
    }
    const bool explicit_uncommitted_current_d_hold =
        candidate.type == CandidateType::FOLLOW ||
        candidate.type == CandidateType::RECOVERY ||
        candidate.type == CandidateType::YIELD_BEHIND ||
        candidate.type == CandidateType::SAFE_STOP;
    const bool uncommitted_current_d_mismatch =
        blocked.start_grid_uncommitted_hold_active &&
        candidate.type != CandidateType::PASS_LEFT &&
        candidate.type != CandidateType::PASS_RIGHT &&
        (!explicit_uncommitted_current_d_hold ||
         !holds_current_lateral(candidate));
    const bool uncommitted_current_d_corridor_invalid =
        blocked.start_grid_uncommitted_hold_active &&
        explicit_uncommitted_current_d_hold &&
        (!candidate.pass_target_corridor_valid ||
         !candidate.controller_tracking_profile_valid);
    if (uncommitted_current_d_mismatch ||
        uncommitted_current_d_corridor_invalid) {
      // 未commit start-grid例外中は、PASS以外に現在d一定の明示holdだけを
      // 許可する。FASTEST/SIDE_KEEPや将来corridor clampで横移動した候補を
      // selected_feasibleとしてFSMへ渡さず、横列なしの速度watchdogへ閉じる。
      candidate.feasible = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.reject_reason =
          uncommitted_current_d_mismatch
              ? "start_grid_uncommitted_hold_not_current_d"
              : "start_grid_uncommitted_hold_corridor_unreachable";
    }
    if (localizedLateralProfileEnabled() && isPassCandidate(candidate.type) &&
        (!localized_lateral_profile_.active ||
         localized_lateral_profile_.target_id.empty())) {
      // localized_latched運用では、PASSは必ずfreshな実車IDから作った局所profileと
      // 一体でGate 2へ渡す。分類だけがPASS文脈を作った周期に、固定offsetの
      // targetless PASSへ暗黙fallbackしてはならない。SafetyEvaluator自体は
      // 上で実行し、ここで契約欠損を追加のfail-closed条件として重ねる。
      candidate.feasible = false;
      candidate.reject_reason = "pass_target_profile_missing";
    }
    if (!candidate.longitudinal_profile_valid) {
      candidate.feasible = false;
      candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    if (blocked.start_grid_target_active &&
        blocked.start_grid_follow_hold_lateral &&
        (candidate.type == CandidateType::FOLLOW ||
         candidate.type == CandidateType::RECOVERY) &&
        (!candidate.pass_target_corridor_valid ||
         !candidate.controller_tracking_profile_valid)) {
      // anchorへ戻す横列は、評価horizon内だけ安全でも採用しない。補正完了まで
      // の回廊とactive controllerの操舵角/操舵速度を満たす時だけ実行する。
      candidate.feasible = false;
      const auto anchor_bounds =
          frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
      const double anchor_lower_d =
          anchor_bounds.d_min + config_.min_wall_margin_m;
      const double anchor_upper_d =
          anchor_bounds.d_max - config_.min_wall_margin_m;
      const bool anchor_valid =
          std::isfinite(blocked.start_grid_hold_target_d_m) &&
          std::isfinite(anchor_lower_d) && std::isfinite(anchor_upper_d) &&
          anchor_lower_d <= anchor_upper_d &&
          blocked.start_grid_hold_target_d_m >= anchor_lower_d &&
          blocked.start_grid_hold_target_d_m <= anchor_upper_d;
      candidate.reject_reason =
          uncommitted_current_d_mismatch
              ? "start_grid_uncommitted_hold_not_current_d"
          : uncommitted_current_d_corridor_invalid
              ? "start_grid_uncommitted_hold_corridor_unreachable"
          : !anchor_valid ? "start_grid_hold_anchor_invalid"
                          : (!candidate.pass_target_corridor_valid
                                 ? "start_grid_hold_corridor_unreachable"
                                 : "start_grid_hold_untrackable");
    }
    const bool prepared_attack_follow_transport_usable =
        blocked.maneuver_transaction_prepared &&
        !blocked.maneuver_transaction_incomplete &&
        reentry_input.pure_pursuit_attack_follow_transport_usable;
    if (blocked.start_grid_target_active &&
        (candidate.type == CandidateType::PASS_LEFT ||
         candidate.type == CandidateType::PASS_RIGHT) &&
        ((!reentry_input.mpc_healthy &&
          !committed_tracking_continuity_authorized &&
          !stopped_non_mpc_pp_bootstrap_authorized) ||
         (!reentry_input.pure_pursuit_primary_and_fresh &&
          // 既にGate 2を一度通ったprepared transactionだけは、同一targetの
          // ATTACK_FOLLOWについてmuxが検証した高々1世代のdelivery gapを
          // 次のPASS probe入力として扱う。実PASS release/reentry/V2は別経路の
          // exact current-generation proofを要求し続ける。
          !prepared_attack_follow_transport_usable &&
          !committed_tracking_continuity_authorized &&
          !initial_start_grid_pass_prediction_bootstrap &&
          !blocked.maneuver_transaction_tracking_release_pending &&
          !reentry_input.pure_pursuit_release_ready) ||
         (committed_retry_follow_blocked &&
          !blocked.maneuver_transaction_tracking_release_pending &&
          !reentry_input.pure_pursuit_release_ready &&
          !moving_attack_follow_retry_ready))) {
      // grid分類は制御契約が未準備でもFOLLOW対象を失わないため残すが、横PASSは
      // MPC healthに加え、final muxのPP command・generation一致まで確認できる
      // 周期だけ開始/継続させる。非稼働MPC運用でも、停止中のexact
      // plan/constraint/PP bundleとnon-MPC PP identityが揃えば、その周期は
      // 実走認可せず新PASS warm-up生成だけ許可する。hard failureは迂回しない。
      // 先行するwall/opponent/CBF等の棄却理由をtransport gapで上書きしない。
      // probe凍結は「物理評価は成立し、trackingだけ未到着」の候補に限る。
      if (candidate.feasible) {
        candidate.feasible = false;
        candidate.reject_reason = "start_grid_tracking_unhealthy";
      }
    }
    const bool start_grid_target_observed_moving =
        blocked.start_grid_target_active &&
        std::isfinite(blocked.start_grid_target_speed_mps) &&
        blocked.start_grid_target_speed_mps >
            std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
    const bool start_grid_target_not_continuously_stationary =
        blocked.start_grid_target_active &&
        !blocked.start_grid_target_confirmed_stationary;
    if (start_grid_target_not_continuously_stationary &&
        (candidate.type == CandidateType::PASS_LEFT ||
         candidate.type == CandidateType::PASS_RIGHT) &&
        std::isfinite(candidate.planned_target_d_m) &&
        std::isfinite(ego.frenet.d) &&
        std::abs(candidate.planned_target_d_m - ego.frenet.d) >
            config_.start_grid_moving_pass_max_lateral_displacement_m +
                1.0e-6) {
      // 20260719 dev3では、動き始めたd3が一周期だけ低速になった時に約3.9 m
      // 横断するPASSが成立し、次周期のtracking停止でD1が約15秒失った。
      // 同じIDの連続停止を時間/進行距離で確認するまでは大横断を開始せず、
      // 現dの攻めFOLLOWで追う。確認済み停止車も後段のSafetyEvaluator、
      // corridor、controller trackabilityを全て通過しなければ認可しない。
      candidate.feasible = false;
      candidate.reject_reason =
          start_grid_target_observed_moving
              ? "start_grid_moving_pass_lateral_displacement"
              : "start_grid_unconfirmed_pass_lateral_displacement";
    }
    if (candidate.type == CandidateType::FOLLOW &&
        blocked.prestart_attack_follow_hold_lateral &&
        !holds_current_lateral(candidate)) {
      // 初回PASS認可前の攻めFOLLOWは、同一targetを追いながら現在dを維持する
      // 縦方向だけの候補である。CandidateBuilderや将来のprofile変更で横移動が
      // 混入した場合は、診断だけを残してpublishへ進めずfail-closedにする。
      candidate.feasible = false;
      candidate.controller_tracking_profile_valid = false;
      candidate.reject_reason = "prestart_attack_follow_not_current_d";
    }
    // 同一車線FOLLOWの制動距離判定を、既にPASS側へ分離した横回避FOLLOWへ
    // そのまま適用すると、SafetyEvaluatorを通る軌道までchain tail直前で潰し、
    // 中心復帰ABORTを選んで別車へ入る。ラッチ済みPASS側・fresh chain tail・
    // controller追従可能性・同周期SafetyEvaluatorを全て満たすFOLLOWだけは、
    // 停止車向け速度capで減速しつつ横に通過する候補として扱う。
    const double required_handoff_lateral_clearance_m =
        std::isfinite(config_.safety_ellipse_b_m) &&
                config_.safety_ellipse_b_m > 0.0 &&
                std::isfinite(config_.min_ellipse_h)
            ? std::max(std::max(0.0, config_.same_corridor_width_m),
                       config_.safety_ellipse_b_m *
                           std::sqrt(1.0 +
                                     std::max(0.0, config_.min_ellipse_h) +
                                     0.10))
            : std::numeric_limits<double>::infinity();
    const bool verified_attack_follow_lateral_bypass =
        candidate.type == CandidateType::FOLLOW && candidate.safety_evaluated &&
        candidate.feasible && candidate.pass_target_corridor_valid &&
        candidate.controller_tracking_profile_valid &&
        blocked.attack_follow_hold_pass_side &&
        blocked.maneuver_transaction_incomplete &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        !blocked.braking_follow_id.empty() &&
        blocked.braking_follow_id == blocked.maneuver_chain_tail_id &&
        blocked.braking_follow_id == blocked.maneuver_target_id &&
        std::isfinite(blocked.maneuver_target_relative_d_m) &&
        std::isfinite(required_handoff_lateral_clearance_m) &&
        std::abs(blocked.maneuver_target_relative_d_m) >=
            required_handoff_lateral_clearance_m &&
        std::isfinite(candidate.planned_target_d_m);
    // start-gridの未認可targetを権威的な前方車へ引き継いだ後も、実車が既に
    // 十分横へ分離していれば、同一車線停止を仮定した制動距離だけでcurrent-d
    // FOLLOWを落とさない。これはPASS許可ではなく、freshな同一target、全相手
    // 予測、壁回廊、controller追従性、SafetyEvaluatorの予備marginを満たす
    // 「現在dを変えない追従」専用契約である。Gate 2認可/commit、別ID、stale、
    // 横並び・future risk、margin喪失のどれかで直ちに従来の制動判定へ戻る。
    const bool verified_uncommitted_current_d_follow =
        candidate.type == CandidateType::FOLLOW && candidate.safety_evaluated &&
        candidate.feasible && candidate.longitudinal_profile_valid &&
        candidate.controller_tracking_profile_valid &&
        candidate.pass_target_corridor_valid &&
        holds_current_lateral(candidate) && localized_lateral_profile_.active &&
        !localized_lateral_profile_.pass_safety_approved_once &&
        !localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed &&
        blocked.start_grid_target_reselection_suppressed &&
        !start_grid_handoff_target_id_.empty() &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        !blocked.maneuver_target_id.empty() && blocked.braking_follow_active &&
        blocked.braking_follow_hold_lateral &&
        localized_lateral_profile_.target_id == blocked.maneuver_target_id &&
        start_grid_handoff_target_id_ == blocked.maneuver_target_id &&
        blocked.start_grid_replacement_target_id ==
            blocked.maneuver_target_id &&
        blocked.braking_follow_id == blocked.maneuver_target_id &&
        blocked.braking_follow_id == blocked.maneuver_chain_tail_id &&
        std::isfinite(blocked.maneuver_target_relative_d_m) &&
        std::isfinite(required_handoff_lateral_clearance_m) &&
        std::abs(blocked.maneuver_target_relative_d_m) >=
            required_handoff_lateral_clearance_m &&
        authorized_hold_inputs_complete && reentry_input.mpc_healthy &&
        !reentry_input.mpc_hard_failure &&
        std::isfinite(candidate.min_safety_margin) &&
        candidate.min_safety_margin >= config_.min_ellipse_h + 0.10 &&
        !blocked.side_by_side && !blocked.corner_side_by_side &&
        !blocked.future_yield_required && !blocked.future_corner_side_by_side &&
        !blocked.future_outer_wall_risk;
    const bool verified_authorized_current_d_hold =
        candidate.type == CandidateType::RECOVERY &&
        blocked.authorized_pass_current_d_hold_active &&
        candidate.safety_evaluated && candidate.feasible &&
        candidate.longitudinal_profile_valid &&
        candidate.controller_tracking_profile_valid &&
        holds_current_lateral(candidate) && authorized_hold_inputs_complete;
    const bool stationary_braking_shortfall =
        (blocked.stationary_front_obstacle || blocked.braking_follow_active) &&
        (candidate.type == CandidateType::FOLLOW ||
         candidate.type == CandidateType::YIELD_BEHIND ||
         (blocked.braking_follow_active &&
          candidate.type == CandidateType::RECOVERY)) &&
        !verified_attack_follow_lateral_bypass &&
        !verified_uncommitted_current_d_follow &&
        !verified_authorized_current_d_hold &&
        candidate.required_brake_distance_m >
            candidate.available_brake_distance_m;
    if (stationary_braking_shortfall) {
      // 評価horizon内だけ安全に見えても、停止余裕が尽きる追従/譲りは通常候補に採用しない。
      candidate.feasible = false;
      candidate.reject_reason = "insufficient_braking_distance";
    }
    if (candidate.type == CandidateType::PASS_LEFT) {
      blocked.pass_left_candidate_feasible = candidate.feasible;
      blocked.pass_left_candidate_tracking_profile_valid =
          candidate.controller_tracking_profile_valid;
      blocked.pass_left_candidate_desired_path_trackable =
          candidate.desired_path_trackable;
      blocked.pass_left_candidate_pure_pursuit_command_trackable =
          candidate.pure_pursuit_command_trackable;
      blocked.pass_left_candidate_endpoint_arc_m =
          candidate.longitudinal_offsets_m.empty()
              ? std::numeric_limits<double>::quiet_NaN()
              : candidate.longitudinal_offsets_m.back();
      blocked.pass_left_candidate_required_arc_m =
          candidate.required_controller_spatial_horizon_m;
      blocked.pass_left_candidate_target_d_m = candidate.planned_target_d_m;
      blocked.pass_left_candidate_corridor_min_margin_m =
          candidate.corridor_min_margin_m;
      blocked.pass_left_candidate_transition_deadline =
          candidate.pass_transition_deadline;
      blocked.pass_left_candidate_reject_reason = candidate.reject_reason;
    } else if (candidate.type == CandidateType::PASS_RIGHT) {
      blocked.pass_right_candidate_feasible = candidate.feasible;
      blocked.pass_right_candidate_tracking_profile_valid =
          candidate.controller_tracking_profile_valid;
      blocked.pass_right_candidate_desired_path_trackable =
          candidate.desired_path_trackable;
      blocked.pass_right_candidate_pure_pursuit_command_trackable =
          candidate.pure_pursuit_command_trackable;
      blocked.pass_right_candidate_endpoint_arc_m =
          candidate.longitudinal_offsets_m.empty()
              ? std::numeric_limits<double>::quiet_NaN()
              : candidate.longitudinal_offsets_m.back();
      blocked.pass_right_candidate_required_arc_m =
          candidate.required_controller_spatial_horizon_m;
      blocked.pass_right_candidate_target_d_m = candidate.planned_target_d_m;
      blocked.pass_right_candidate_corridor_min_margin_m =
          candidate.corridor_min_margin_m;
      blocked.pass_right_candidate_transition_deadline =
          candidate.pass_transition_deadline;
      blocked.pass_right_candidate_reject_reason = candidate.reject_reason;
    } else if (candidate.type == CandidateType::FOLLOW) {
      if (!candidate.v_ref.empty()) {
        blocked.follow_candidate_speed_cap_mps = candidate.v_ref.front();
        blocked.follow_candidate_terminal_speed_mps = candidate.v_ref.back();
      }
      const bool prestart_attack_follow_candidate =
          blocked.prestart_attack_follow_hold_lateral &&
          !blocked.maneuver_transaction_incomplete;
      if ((blocked.attack_follow_hold_pass_side &&
           blocked.maneuver_transaction_incomplete) ||
          prestart_attack_follow_candidate) {
        blocked.attack_follow_candidate_generated = true;
        blocked.attack_follow_candidate_feasible = candidate.feasible;
        blocked.attack_follow_candidate_safe_lateral_hold =
            prestart_attack_follow_candidate
                ? holds_current_lateral(candidate)
                : candidate.attack_follow_safe_lateral_hold;
        blocked.attack_follow_candidate_opponent_collision_current_d_hold =
            candidate.attack_follow_opponent_collision_current_d_hold;
        blocked.attack_follow_candidate_opponent_collision_inward_connector =
            candidate.attack_follow_opponent_collision_inward_connector;
        blocked.attack_follow_candidate_tracking_profile_valid =
            candidate.controller_tracking_profile_valid;
        blocked.attack_follow_candidate_reject_reason = candidate.reject_reason;
        blocked.attack_follow_candidate_planned_target_d_m =
            candidate.planned_target_d_m;
        blocked.attack_follow_candidate_committed_target_d_m =
            candidate.committed_attack_follow_target_d_m;
        blocked.attack_follow_candidate_corridor_min_margin_m =
            candidate.corridor_min_margin_m;
        blocked.attack_follow_candidate_min_safety_margin =
            candidate.min_safety_margin;
        blocked.attack_follow_candidate_blocking_opponent_id =
            candidate.blocking_opponent_id;
        blocked.attack_follow_candidate_blocking_time_sec =
            candidate.blocking_time_sec;
        blocked.attack_follow_candidate_blocking_candidate_x_m =
            candidate.blocking_candidate_x_m;
        blocked.attack_follow_candidate_blocking_candidate_y_m =
            candidate.blocking_candidate_y_m;
        blocked.attack_follow_candidate_blocking_candidate_yaw_rad =
            candidate.blocking_candidate_yaw_rad;
        blocked.attack_follow_candidate_blocking_candidate_s_m =
            candidate.blocking_candidate_s_m;
        blocked.attack_follow_candidate_blocking_candidate_d_m =
            candidate.blocking_candidate_d_m;
        blocked.attack_follow_candidate_blocking_opponent_x_m =
            candidate.blocking_opponent_x_m;
        blocked.attack_follow_candidate_blocking_opponent_y_m =
            candidate.blocking_opponent_y_m;
        blocked.attack_follow_candidate_blocking_opponent_s_m =
            candidate.blocking_opponent_s_m;
        blocked.attack_follow_candidate_blocking_opponent_d_m =
            candidate.blocking_opponent_d_m;
      }
      if (blocked.parallel_follow_candidate && !blocked.blocked) {
        blocked.parallel_follow_feasible = candidate.feasible;
      }
      if (blocked.parallel_follow_recheck_attempted) {
        blocked.parallel_follow_recheck_reason =
            candidate.feasible                ? "safety_evaluated_feasible"
            : candidate.reject_reason.empty() ? "safety_evaluated_infeasible"
                                              : candidate.reject_reason;
      }
      if (blocked.braking_follow_active) {
        blocked.braking_follow_feasible = candidate.feasible;
        blocked.braking_follow_candidate_reject_reason =
            candidate.reject_reason;
        blocked.braking_follow_candidate_tracking_profile_valid =
            candidate.controller_tracking_profile_valid;
        blocked.braking_follow_candidate_min_safety_margin =
            candidate.min_safety_margin;
      }
      if (blocked.stationary_front_obstacle) {
        blocked.stationary_front_required_brake_distance_m =
            candidate.required_brake_distance_m;
        blocked.stationary_front_available_brake_distance_m =
            candidate.available_brake_distance_m;
        blocked.stationary_front_brake_feasible =
            candidate.longitudinal_profile_valid && candidate.feasible &&
            candidate.required_brake_distance_m <=
                candidate.available_brake_distance_m;
      }
    }
  }
  const auto fastest_wall_reject =
      std::find_if(candidates.cbegin(), candidates.cend(),
                   [](const CandidateTrajectory &candidate) {
                     return candidate.type == CandidateType::FASTEST &&
                            candidate.safety_evaluated && !candidate.feasible &&
                            candidate.reject_reason == "wall_footprint_margin";
                   });
  const bool recovery_already_evaluated =
      std::any_of(candidates.cbegin(), candidates.cend(),
                  [](const CandidateTrajectory &candidate) {
                    return candidate.type == CandidateType::RECOVERY &&
                           candidate.safety_evaluated;
                  });
  const double early_wall_recovery_clearance_m =
      std::hypot(std::max(std::max(0.0, config_.ego_front_extent_m),
                          std::max(0.0, config_.ego_rear_extent_m)),
                 std::max(0.0, config_.ego_half_width_m)) +
      std::max(0.0, config_.wall_localization_uncertainty_m) +
      std::max(0.0, config_.min_wall_margin_m) +
      std::max(0.0, config_.wall_footprint_max_sample_distance_m);
  // 20260727-151855 D1ではwall reject開始時に速度と曲率が同時に閾値未達
  // だった。単一の巨大ANDでは最初の不成立箇所をbagから判別できないため、
  // 同じ短絡順を固定tokenとして記録する。これはread-only診断であり、
  // candidate生成、score、mode、motion authorityには使わない。
  const auto early_wall_recovery_probe_first_false = [&]() -> const char * {
    if (!config_.high_speed_curve_lateral_hold_enabled) {
      return "disabled";
    }
    if (mode_ != BehaviorMode::FREE_RUN && mode_ != BehaviorMode::SPEED_GUARD) {
      return "mode_not_free_or_speed_guard";
    }
    if (!std::isfinite(ego.v)) {
      return "ego_speed_nonfinite";
    }
    if (ego.v <
        std::max(0.0, config_.high_speed_curve_lateral_hold_min_speed_mps)) {
      return "ego_speed_below_min";
    }
    if (!std::isfinite(blocked.corner_abs_curvature)) {
      return "corner_curvature_nonfinite";
    }
    if (blocked.corner_abs_curvature <
        std::max(0.0, config_.preemptive_wall_recovery_min_curvature_m_inv)) {
      return "corner_curvature_below_min";
    }
    if (blocked.blocked) {
      return "blocked_front";
    }
    if (blocked.side_by_side) {
      return "side_by_side";
    }
    if (blocked.corner_side_by_side) {
      return "corner_side_by_side";
    }
    if (blocked.parallel_side_candidate) {
      return "parallel_side";
    }
    if (blocked.future_side_by_side) {
      return "future_side_by_side";
    }
    if (blocked.future_corner_side_by_side) {
      return "future_corner_side_by_side";
    }
    if (blocked.future_yield_required) {
      return "future_yield";
    }
    if (blocked.maneuver_transaction_prepared) {
      return "transaction_prepared";
    }
    if (blocked.maneuver_transaction_incomplete) {
      return "transaction_incomplete";
    }
    if (!blocked.maneuver_target_id.empty()) {
      return "maneuver_target_present";
    }
    if (generic_recovery_phase_active_) {
      return "generic_recovery_active";
    }
    if (reentry_phase_active_) {
      return "reentry_phase_active";
    }
    if (reentry_lockout_active_) {
      return "reentry_lockout_active";
    }
    if (!std::isfinite(blocked.ego_wall_clearance_m)) {
      return "wall_clearance_nonfinite";
    }
    if (!std::isfinite(early_wall_recovery_clearance_m)) {
      return "clearance_threshold_nonfinite";
    }
    if (blocked.ego_wall_clearance_m < 0.0) {
      return "wall_clearance_negative";
    }
    if (blocked.ego_wall_clearance_m > early_wall_recovery_clearance_m) {
      return "wall_clearance_above_threshold";
    }
    if (!std::isfinite(ego.frenet.d)) {
      return "frenet_d_nonfinite";
    }
    if (std::abs(ego.frenet.d) <=
        std::max(0.0, config_.recovery_release_lateral_error_m)) {
      return "lateral_error_within_release_threshold";
    }
    return "none";
  }();
  const bool early_wall_recovery_probe_requested =
      std::string_view(early_wall_recovery_probe_first_false) == "none";
  blocked.early_wall_recovery_probe_requested =
      early_wall_recovery_probe_requested;
  blocked.early_wall_recovery_clearance_threshold_m =
      early_wall_recovery_clearance_m;
  blocked.early_wall_recovery_probe_first_false =
      early_wall_recovery_probe_first_false;
  if (((fastest_wall_reject != candidates.cend() &&
        !recovery_already_evaluated) ||
       early_wall_recovery_probe_requested) &&
      blocked.ego_wall_clearance_m >= 0.0) {
    // 20260727-135724 D3では、FASTESTのfootprintが壁marginを割った後も
    // 横overrideを失ったspeed-only fallbackを約0.25 s継続し、実車dが
    // 4.14 -> 4.39 mへ外側に進んでからlarge_lateral_errorへ入った。
    // 壁外へ出てからRECOVERYを短くするのではなく、まだ正のclearanceがある
    // 最初のwall reject周期では始端footprint自体が既に不成立だったため、
    // 車体中心から最遠cornerまでの半径 + localization uncertainty + hard
    // wall margin + 1 bounded footprint sampleの範囲へ入った周期から、
    // actual pose始端の既存RECOVERYを独立評価する。接線yawが変化するカーブでも
    // half-widthだけで発動が遅れない保守境界である。
    // 新state/latchは追加せず、全相手・壁・CBF・PP空間horizonが同周期に
    // 成立した候補だけが通常のscore仲裁へ進む。不成立なら従来どおり
    // speed-only fail-closedを維持する。
    BlockedInfo early_recovery_blocked = blocked;
    early_recovery_blocked.early_wall_recovery_probe_requested =
        early_wall_recovery_probe_requested;
    auto early_recovery_config = config_;
    constexpr double kD3BoundedMovingRecoveryHorizonSec = 8.0;
    const double stationary_opponent_speed_mps =
        std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
    const bool all_opponents_stationary = std::all_of(
        opponents.cbegin(), opponents.cend(),
        [stationary_opponent_speed_mps](const OpponentState &opponent) {
          return opponent.valid && std::isfinite(opponent.v) &&
                 std::isfinite(opponent.vx) && std::isfinite(opponent.vy) &&
                 std::max(std::abs(opponent.v),
                          std::hypot(opponent.vx, opponent.vy)) <=
                     stationary_opponent_speed_mps;
        });
    if (all_opponents_stationary) {
      // 8秒の空間horizonは、D3の単独wall recoveryのようにmoving相手の
      // 未校正な等速外挿を追加しない周期だけ許す。moving相手が1台でも
      // 存在すれば既存6秒上限を維持し、required arc不足はfail-closedにする。
      early_recovery_config.moving_lateral_override_max_evaluation_horizon_sec =
          std::max(early_recovery_config
                       .moving_lateral_override_max_evaluation_horizon_sec,
                   kD3BoundedMovingRecoveryHorizonSec);
    }
    CandidateTrajectory early_recovery =
        CandidateBuilder(frame_, early_recovery_config)
            .makeCandidate(CandidateType::RECOVERY, ego, early_recovery_blocked,
                           opponents);
    blocked.early_wall_recovery_probe_generated = true;
    evaluateCandidateAtOwnTimeAxis(early_recovery, opponents, now_sec,
                                   predictions);
    // SafetyEvaluatorのwall/opponent rejectを、後段のprofile proofで
    // 上書きしない。first-falseは実際の評価順の最初の権威的失敗に固定する。
    if (early_recovery.feasible && !early_recovery.longitudinal_profile_valid) {
      early_recovery.feasible = false;
      early_recovery.reject_reason = "invalid_longitudinal_brake_model";
    } else if (early_recovery.feasible &&
               !early_recovery.controller_spatial_horizon_proof_valid) {
      early_recovery.feasible = false;
      early_recovery.controller_tracking_profile_valid = false;
      early_recovery.reject_reason =
          "recovery_controller_spatial_horizon_unavailable";
    }
    blocked.early_wall_recovery_probe_feasible =
        early_recovery.safety_evaluated && early_recovery.feasible &&
        early_recovery.longitudinal_profile_valid &&
        early_recovery.controller_tracking_profile_valid &&
        early_recovery.controller_spatial_horizon_proof_valid;
    blocked.early_wall_recovery_reject_reason = early_recovery.reject_reason;
    candidates.push_back(std::move(early_recovery));
  }
  if (state_lattice_shadow_comparison.requested) {
    const auto find_pass = [&](CandidateType type) {
      return std::find_if(candidates.cbegin(), candidates.cend(),
                          [type](const CandidateTrajectory &candidate) {
                            return candidate.type == type &&
                                   candidate.safety_evaluated;
                          });
    };
    auto current_pass = candidates.cend();
    if (localized_lateral_profile_.active &&
        isPassCandidate(localized_lateral_profile_.pass_type)) {
      current_pass = find_pass(localized_lateral_profile_.pass_type);
    }
    if (current_pass == candidates.cend()) {
      current_pass = find_pass(CandidateType::PASS_LEFT);
    }
    if (current_pass == candidates.cend()) {
      current_pass = find_pass(CandidateType::PASS_RIGHT);
    }
    if (current_pass != candidates.cend()) {
      const bool snapshot_complete =
          reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
          reentry_input.all_observed_opponents_fresh &&
          reentry_input.all_observed_opponents_included &&
          reentry_input.reference_valid &&
          predictions.size() == opponents.size() &&
          std::all_of(predictions.cbegin(), predictions.cend(),
                      [this](const PredictedOpponent &prediction) {
                        return prediction.t.size() == config_.horizon_points &&
                               prediction.x.size() == config_.horizon_points &&
                               prediction.y.size() == config_.horizon_points &&
                               prediction.s.size() == config_.horizon_points &&
                               prediction.d.size() == config_.horizon_points;
                      });
      state_lattice_shadow_comparison = evaluateStateLatticeShadowComparison(
          *current_pass, ego, authoritativeTargetVehicleId(blocked), opponents,
          now_sec, predictions, snapshot_complete, pp_exact_snapshot);
    }
  }
  // PASS候補のhorizon先端だけを見ていると、停止したchain tailへ近づく最後の
  // 周期までPASSを選び、次周期にSafetyEvaluatorが不可避衝突を検出する。
  // 同方向・低速の前方車に対して現d保持FOLLOWが同じ周期に安全なら、marginの
  // 予備量を使って先に攻め追従へhandoffする。SafetyEvaluatorの判定自体を
  // 緩める経路ではなく、通過済みFOLLOWだけをPASSより優先する。
  if ((isAnyPassMode(mode_) || attack_follow_transaction_active) &&
      (blocked.front_vehicle_low_speed || blocked.slow_obstacle_chain_active ||
       blocked.braking_follow_active) &&
      (hasForwardFollowFallbackTarget(blocked) ||
       blocked.braking_follow_active)) {
    const auto follow_it = std::find_if(
        candidates.begin(), candidates.end(), [](const auto &candidate) {
          return candidate.type == CandidateType::FOLLOW &&
                 candidate.safety_evaluated && candidate.feasible;
        });
    if (follow_it != candidates.end()) {
      const double pass_margin_guard_h = config_.min_ellipse_h + 0.10;
      const bool passed_target_chain_handoff =
          blocked.slow_obstacle_chain_active &&
          blocked.maneuver_target_id != blocked.maneuver_chain_tail_id &&
          maneuverTargetPassed(config_, blocked);
      const bool braking_follow_handoff_required =
          blocked.maneuver_transaction_incomplete &&
          blocked.braking_follow_active && !blocked.braking_follow_id.empty() &&
          blocked.braking_follow_id != blocked.maneuver_target_id &&
          blocked.braking_follow_id == blocked.maneuver_chain_tail_id &&
          std::isfinite(blocked.braking_follow_delta_s) &&
          std::isfinite(blocked.braking_follow_required_distance_m) &&
          blocked.braking_follow_delta_s <=
              blocked.braking_follow_required_distance_m;
      for (auto &candidate : candidates) {
        if (!isPassCandidate(candidate.type) || !candidate.safety_evaluated ||
            !candidate.feasible) {
          continue;
        }
        // 制動距離は「同じdを走り続ける」FOLLOWの必要量であり、横へ抜ける
        // PASS候補を単独で不成立にする根拠にはしない。候補軌道自身の
        // SafetyEvaluator余裕が予備帯まで低下した時だけ、評価済みFOLLOWへ
        // handoffする。margin欠損は安全とは解釈せず従来どおりfail-closed。
        const bool pass_margin_guard_required =
            !std::isfinite(candidate.min_safety_margin) ||
            candidate.min_safety_margin < pass_margin_guard_h;
        const bool passed_target_margin_handoff_required =
            passed_target_chain_handoff && pass_margin_guard_required;
        const bool braking_margin_handoff_required =
            braking_follow_handoff_required && pass_margin_guard_required;
        if (!passed_target_margin_handoff_required &&
            !braking_margin_handoff_required && !pass_margin_guard_required) {
          continue;
        }
        candidate.feasible = false;
        candidate.reject_reason = braking_margin_handoff_required
                                      ? "pass_chain_braking_distance_follow"
                                  : passed_target_margin_handoff_required
                                      ? "pass_chain_tail_handoff_follow"
                                      : "pass_safety_margin_guard_follow";
        if (candidate.type == CandidateType::PASS_LEFT) {
          blocked.pass_left_candidate_feasible = false;
          blocked.pass_left_candidate_reject_reason = candidate.reject_reason;
        } else {
          blocked.pass_right_candidate_feasible = false;
          blocked.pass_right_candidate_reject_reason = candidate.reject_reason;
        }
      }
    }
  }
  if (blocked.pass_reauthorization_lockout_active) {
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::FASTEST ||
          candidate.type == CandidateType::PASS_LEFT ||
          candidate.type == CandidateType::PASS_RIGHT) {
        candidate.feasible = false;
        candidate.reject_reason = "pass_profile_recovery_latched";
      }
    }
    blocked.pass_left_candidate_feasible = false;
    blocked.pass_right_candidate_feasible = false;
    if (blocked.pass_left_candidate_generated) {
      blocked.pass_left_candidate_reject_reason =
          "pass_profile_recovery_latched";
    }
    if (blocked.pass_right_candidate_generated) {
      blocked.pass_right_candidate_reject_reason =
          "pass_profile_recovery_latched";
    }
  }
  const bool pass_start_observation_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid;
  const bool pass_start_prediction_complete =
      predictions.size() == opponents.size() &&
      std::all_of(predictions.begin(), predictions.end(),
                  [this](const auto &prediction) {
                    return prediction.t.size() == config_.horizon_points &&
                           prediction.x.size() == config_.horizon_points &&
                           prediction.y.size() == config_.horizon_points &&
                           prediction.s.size() == config_.horizon_points &&
                           prediction.d.size() == config_.horizon_points;
                  });
  const bool pass_start_live_tracking_usable =
      reentry_input.pure_pursuit_primary_and_fresh &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  const bool prepared_attack_follow_transport_probe_usable =
      blocked.start_grid_target_active &&
      blocked.maneuver_transaction_prepared &&
      !blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == blocked.start_grid_target_id &&
      reentry_input.pure_pursuit_attack_follow_transport_usable &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  const bool gate_two_authorization_inputs_ready =
      pass_start_observation_inputs_complete &&
      pass_start_prediction_complete &&
      (pass_start_live_tracking_usable ||
       prepared_attack_follow_transport_probe_usable ||
       initial_start_grid_pass_prediction_bootstrap);
  const bool localized_profile_target_identity_consistent =
      localized_lateral_profile_.active && blocked.maneuver_target_latched &&
      blocked.maneuver_target_id == localized_lateral_profile_.target_id &&
      blocked.maneuver_target_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_target_index) <
          opponents.size() &&
      opponents[static_cast<std::size_t>(blocked.maneuver_target_index)].id ==
          localized_lateral_profile_.target_id &&
      blocked.maneuver_chain_tail_observed &&
      blocked.maneuver_chain_tail_index >= 0 &&
      static_cast<std::size_t>(blocked.maneuver_chain_tail_index) <
          opponents.size() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      blocked.maneuver_chain_tail_id ==
          localized_lateral_profile_.chain_tail_id &&
      opponents[static_cast<std::size_t>(blocked.maneuver_chain_tail_index)]
              .id == blocked.maneuver_chain_tail_id;
  if (localized_profile_target_identity_consistent &&
      !localized_lateral_profile_.pass_safety_approved_once &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      mode_ != BehaviorMode::OVERTAKE_LEFT &&
      mode_ != BehaviorMode::OVERTAKE_RIGHT) {
    const auto gate_two_approved = [this, gate_two_authorization_inputs_ready](
                                       const CandidateTrajectory &candidate) {
      return gate_two_authorization_inputs_ready &&
             candidate.safety_evaluated && candidate.feasible &&
             std::isfinite(candidate.planned_target_d_m) &&
             !std::isnan(candidate.min_safety_margin) &&
             candidate.min_safety_margin >= config_.min_ellipse_h;
    };
    const auto current = std::find_if(
        candidates.begin(), candidates.end(), [this](const auto &candidate) {
          return candidate.type == localized_lateral_profile_.pass_type;
        });
    auto approved = candidates.end();
    if (current != candidates.end() && gate_two_approved(*current)) {
      approved = current;
    } else {
      approved = std::find_if(
          candidates.begin(), candidates.end(), [&](const auto &candidate) {
            return candidate.type != localized_lateral_profile_.pass_type &&
                   (candidate.type == CandidateType::PASS_LEFT ||
                    candidate.type == CandidateType::PASS_RIGHT) &&
                   gate_two_approved(candidate);
          });
    }
    if (approved != candidates.end()) {
      const CandidateType previous_pass_type =
          localized_lateral_profile_.pass_type;
      const std::optional<LocalizedLateralProfile> *approved_profile = nullptr;
      if (approved->type != previous_pass_type) {
        approved_profile = approved->type == CandidateType::PASS_LEFT
                               ? &evaluated_alternate_left_profile
                               : &evaluated_alternate_right_profile;
      }
      if (approved_profile != nullptr && !approved_profile->has_value()) {
        // SafetyEvaluatorを通したcandidateと同一のprofileを保持できない場合は
        // sideだけを書き換えない。これは内部契約破損なのでfail-closedにする。
        approved->feasible = false;
        approved->reject_reason = "gate2_alternate_profile_missing";
      } else if (approved_profile != nullptr) {
        localized_lateral_profile_ = approved_profile->value();
        attack_follow_inner_band_diagnostic_ =
            AttackFollowInnerBandDiagnostic{};
        last_attack_follow_inner_band_probe_sec_ =
            std::numeric_limits<double>::quiet_NaN();
      }
    }
    if (approved != candidates.end() && approved->feasible) {
      const CandidateType previous_pass_type =
          blocked.maneuver_transaction_pass_type;
      localized_lateral_profile_.pass_type = approved->type;
      localized_lateral_profile_.target_d_m = approved->planned_target_d_m;
      localized_lateral_profile_.pass_safety_approved_once = true;
      rememberAuthorizedPassEnvelope(*approved, localized_lateral_profile_);
      blocked.maneuver_transaction_pass_type = approved->type;
      if (blocked.attack_follow_hold_pass_side) {
        blocked.attack_follow_target_d_m = approved->planned_target_d_m;
      }
      if (approved->type != previous_pass_type) {
        // target IDとs markerは維持し、未実行のsideだけをGate 2で訂正する。
        // 次周期以降はapproved_onceにより反対候補を生成しない。
        blocked.maneuver_target_change_reason =
            "gate2_reselected_unapproved_pass_side";
      }
    }
  }
  if (localized_lateral_profile_.active && blocked.maneuver_target_latched &&
      blocked.maneuver_target_observed) {
    const auto pass_candidate = std::find_if(
        candidates.begin(), candidates.end(), [this](const auto &candidate) {
          return candidate.type == localized_lateral_profile_.pass_type;
        });
    if (pass_candidate != candidates.end()) {
      blocked.maneuver_target_safety_evaluated =
          pass_candidate->safety_evaluated;
      blocked.maneuver_target_pass_candidate_feasible =
          pass_candidate->feasible;
      blocked.maneuver_target_pass_min_safety_margin =
          pass_candidate->min_safety_margin;
      blocked.maneuver_target_pass_reject_reason =
          pass_candidate->reject_reason;
      const bool current_pass_candidate_safety_approved =
          localized_profile_target_identity_consistent &&
          pass_candidate->safety_evaluated && pass_candidate->feasible &&
          !std::isnan(pass_candidate->min_safety_margin) &&
          pass_candidate->min_safety_margin >= config_.min_ellipse_h;
      blocked.maneuver_target_pass_safety_approved =
          current_pass_candidate_safety_approved &&
          (localized_lateral_profile_.pass_safety_approved_once ||
           gate_two_authorization_inputs_ready);
      // Observation/prediction/tracking completeness is a start-authorization
      // contract.  Apply it only when the profile first becomes authorized;
      // reapplying it after preparation or execution would conflate a transient
      // controller-status gap with current geometric safety and can either
      // break the start-grid handshake or force a committed PASS to merge
      // before the latched-target completion contract is satisfied.
      if (!localized_lateral_profile_.pass_safety_approved_once &&
          blocked.maneuver_target_pass_safety_approved) {
        localized_lateral_profile_.pass_safety_approved_once = true;
      }
      if (current_pass_candidate_safety_approved &&
          localized_lateral_profile_.pass_safety_approved_once) {
        rememberAuthorizedPassEnvelope(*pass_candidate,
                                       localized_lateral_profile_);
      }
    }
    if (localized_lateral_profile_.pass_safety_approved_once &&
        !localized_lateral_profile_.pass_execution_committed) {
      // Gate 2でsideを確定した後、state machineへ両側のfeasible PASSを残すと、
      // score順で反対側を選び、最終payloadが固定profileと一致せずcommitできない。
      // 未実行なのでABORT transactionにはしないが、同じtargetのprepared
      // sideだけを 候補集合へ残し、実publishか安全な非PASS
      // fallbackのどちらかに限定する。
      for (auto &candidate : candidates) {
        const bool opposite_pass =
            (candidate.type == CandidateType::PASS_LEFT ||
             candidate.type == CandidateType::PASS_RIGHT) &&
            candidate.type != localized_lateral_profile_.pass_type;
        if (opposite_pass) {
          const bool was_feasible = candidate.feasible;
          candidate.feasible = false;
          if (was_feasible || candidate.reject_reason.empty()) {
            candidate.reject_reason = "gate2_prepared_side_locked";
          }
        }
      }
      if (localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT) {
        const bool opposite_was_feasible =
            blocked.pass_right_candidate_feasible;
        blocked.pass_right_candidate_feasible = false;
        if (blocked.pass_right_candidate_generated &&
            (opposite_was_feasible ||
             blocked.pass_right_candidate_reject_reason.empty())) {
          blocked.pass_right_candidate_reject_reason =
              "gate2_prepared_side_locked";
        }
      } else if (localized_lateral_profile_.pass_type ==
                 CandidateType::PASS_RIGHT) {
        const bool opposite_was_feasible = blocked.pass_left_candidate_feasible;
        blocked.pass_left_candidate_feasible = false;
        if (blocked.pass_left_candidate_generated &&
            (opposite_was_feasible ||
             blocked.pass_left_candidate_reject_reason.empty())) {
          blocked.pass_left_candidate_reject_reason =
              "gate2_prepared_side_locked";
        }
      }
    }
    blocked.maneuver_transaction_prepared =
        is_confirmed_start_grid_profile() &&
        localized_lateral_profile_.pass_safety_approved_once &&
        !localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed;
    // 初回Gate 2認可後はtarget ID/side/dを固定して走り切る契約なので、対象を
    // 幾何的に抜いた周期に別車両が現在のPASS候補をrejectしても、元対象の完了を
    // 取り消さない。一方、一度も認可されていない横すり抜けは完了扱いにしない。
    const bool maneuver_target_pass_complete_now =
        blocked.maneuver_target_pass_geometric_complete &&
        localized_lateral_profile_.pass_execution_committed;
    localized_lateral_profile_.pass_complete_confirmed =
        localized_lateral_profile_.pass_complete_confirmed ||
        maneuver_target_pass_complete_now;
    blocked.maneuver_target_pass_complete =
        localized_lateral_profile_.pass_complete_confirmed;
    if (maneuver_target_pass_complete_now) {
      const auto completed_waypoint = std::find_if(
          localized_lateral_profile_.chain_waypoints.begin(),
          localized_lateral_profile_.chain_waypoints.end(),
          [this](const LocalizedLateralWaypoint &waypoint) {
            return waypoint.target_id == localized_lateral_profile_.target_id;
          });
      const bool chain_successor_remains =
          completed_waypoint !=
              localized_lateral_profile_.chain_waypoints.end() &&
          std::next(completed_waypoint) !=
              localized_lateral_profile_.chain_waypoints.end();
      if (!chain_successor_remains) {
        // 最終対象の完了後だけ旧geometry authorityを即時破棄する。chain途中は
        // 次targetへ同じstaged wireを引き継ぎ、別profileの再生成を防ぐ。
        committed_pass_snapshot_valid_ = false;
        committed_pass_spatial_profile_frozen_ = false;
        committed_pass_snapshot_target_id_.clear();
        committed_pass_snapshot_pass_type_ = CandidateType::FASTEST;
        committed_pass_snapshot_sec_ = std::numeric_limits<double>::quiet_NaN();
        committed_pass_snapshot_ego_unwrapped_s_m_ =
            std::numeric_limits<double>::quiet_NaN();
        committed_pass_snapshot_ = CandidateTrajectory{};
        committed_pass_profile_snapshot_ = LocalizedLateralProfile{};
      }
    }
  }
  const bool committed_start_grid_pass =
      blocked.start_grid_target_active && !start_grid_target_id_.empty() &&
      blocked.start_grid_target_id == start_grid_target_id_ &&
      localized_lateral_profile_.active &&
      localized_lateral_profile_.target_id == start_grid_target_id_ &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  if (committed_start_grid_pass) {
    // 初期配置のdではなく、SafetyEvaluator/Gate 2を通過した横移動だけを
    // 安全復帰が必要なtransactionとして記録する。PASS完了までtarget
    // ID/side/dを保持し、対象消失時だけ長期reentry gateへ閉じる。
    start_grid_lateral_release_pending_ = true;
    blocked.start_grid_lateral_release_pending = true;
    blocked.start_grid_hold_target_d_m = start_grid_lateral_anchor_d_m_;
  }
  blocked.maneuver_transaction_incomplete =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  blocked.maneuver_transaction_prepared =
      is_confirmed_start_grid_profile() &&
      localized_lateral_profile_.pass_safety_approved_once &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed;
  blocked.maneuver_transaction_pass_type =
      localized_lateral_profile_.active ? localized_lateral_profile_.pass_type
                                        : CandidateType::FASTEST;
  // 同周期にGate 2が初回承認してPREPAREへ入った場合は、候補生成時だけ使った
  // 未commit current-d
  // hold診断を残さない。以降はACK/transaction契約が権威を持つ。
  update_start_grid_uncommitted_hold();
  // 停止/低速前走車のpermission例外は、PASS候補をSafetyEvaluatorで評価した
  // 後にだけ開始gateへ反映する。SafetyEvaluatorが見落とす入力欠損を「安全」と
  // 解釈しないため、Nodeが検証したfreshness/包含状態と予測列の完全性も要求する。
  const bool has_safe_pass = blocked.pass_left_candidate_feasible ||
                             blocked.pass_right_candidate_feasible;
  if (blocked.early_stationary_parallel_pass_target && !has_safe_pass) {
    // early probeは停止したparallel車のためにPASSを前倒し評価するだけであり、
    // Gate 2がPASSを拒否した時に通常のFASTESTへ戻して接近を続けない。
    // YIELD/RECOVERY/SAFE_STOPの既存fallbackだけを選択対象に残す。
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::FASTEST) {
        candidate.feasible = false;
        candidate.reject_reason = "early_stationary_parallel_pass_rejected";
      }
    }
  }
  if (blocked.braking_follow_active && !blocked.braking_follow_feasible) {
    // 必要制動距離を下回った停止車へ、FOLLOW不成立のままFASTESTへ戻すと
    // 加速継続になる。現d hold RECOVERY/SAFE_STOPが通らない時だけ既存の
    // speed-only fail-safeへ閉じるため、FASTEST候補を選択対象から外す。
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::FASTEST) {
        candidate.feasible = false;
        candidate.reject_reason = "braking_follow_rejected";
      }
    }
  }
  const bool exception_base_inputs_complete =
      pass_start_observation_inputs_complete;
  const bool slow_front_exception_inputs_complete =
      exception_base_inputs_complete && reentry_input.mpc_healthy;
  // single fresh slow-solveは速度capだけのsoft guardなので、ここで既に
  // SafetyEvaluatorを通った制限PASSまで止めない。stale/連続infeasibleは
  // hard failureとして拒否する。unit testの直接入力（sample sequence 0）は
  // 従来どおりmpc_healthyを明示要求する。
  const bool gentle_curve_mpc_ready =
      reentry_input.mpc_health_sample_sequence == 0U
          ? reentry_input.mpc_healthy
          : reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  const bool prediction_complete = pass_start_prediction_complete;
  blocked.gentle_curve_observation_inputs_complete =
      exception_base_inputs_complete;
  blocked.gentle_curve_prediction_complete = prediction_complete;
  blocked.gentle_curve_tracking_usable = pass_start_live_tracking_usable;
  blocked.gentle_curve_mpc_ready = gentle_curve_mpc_ready;
  const bool direct_slow_front = [&]() {
    if (blocked.slow_obstacle_chain_active || blocked.nearest_index < 0 ||
        static_cast<std::size_t>(blocked.nearest_index) >= opponents.size() ||
        !std::isfinite(blocked.front_delta_s) || blocked.front_delta_s <= 0.0 ||
        blocked.front_delta_s >
            std::max(0.0, config_.slow_front_exception_distance_m) ||
        !std::isfinite(blocked.front_vehicle_speed_mps) ||
        !std::isfinite(blocked.front_rel_v) || blocked.front_rel_v <= 0.0 ||
        !std::isfinite(blocked.front_s_dot_mps) ||
        blocked.front_s_dot_mps < 0.0) {
      return false;
    }
    const auto &front =
        opponents[static_cast<std::size_t>(blocked.nearest_index)];
    if (!front.valid ||
        !inputTimestampFresh(now_sec, front.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec)) {
      return false;
    }
    const double stationary_speed =
        std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
    if (blocked.front_vehicle_speed_mps <= stationary_speed) {
      return true;
    }
    return blocked.front_vehicle_speed_mps <=
               std::max(0.0, config_.slow_front_exception_speed_mps) &&
           blocked.front_direction_known && blocked.front_same_direction;
  }();
  // PREPAREへ入った停止parallel permission例外は、同一IDの確認済み対象が
  // 続く一周期だけ再評価する。別IDへのすり替わり、freshness欠損、通常PASSの
  // PREPAREはここを通れないため、禁止区間で例外を持ち越さない。
  const bool stationary_parallel_permission_prepare_continuation =
      isPrepareOvertakeMode(mode_) &&
      !stationary_parallel_permission_prepare_id_.empty() &&
      blocked.early_stationary_parallel_pass_target &&
      blocked.early_stationary_parallel_pass_id ==
          stationary_parallel_permission_prepare_id_;
  const bool stationary_parallel_permission_start_context =
      mode_ == BehaviorMode::FREE_RUN ||
      mode_ == BehaviorMode::FOLLOW_BLOCKED ||
      stationary_parallel_permission_prepare_continuation;
  // direct slow-front例外は既存仕様どおり、PREPARE中も同じSafetyEvaluator・
  // freshness条件で再承認する。禁止区間へPREPAREだけで侵入した通常PASSは、
  // 下の個別例外条件を満たせないため開始されない。
  const bool permission_exception_start_context =
      mode_ == BehaviorMode::FREE_RUN ||
      mode_ == BehaviorMode::FOLLOW_BLOCKED || isPrepareOvertakeMode(mode_);
  const bool no_pass_start_conflict =
      !blocked.side_by_side && !blocked.corner_side_by_side &&
      // 停止前走車へ近づく通常のPASS開始もfuture_side_by_sideになる。ここで
      // 禁じると停止障害物例外が実質発火しないため、実際に譲り/コーナーリスクへ
      // 昇格した状態だけを拒否する。PASS軌道そのものは直前にSafetyEvaluatorで
      // 全相手予測に対して評価済みである。
      !blocked.future_corner_side_by_side && !blocked.future_yield_required &&
      !blocked.parallel_yield_hold_lateral && !blocked.reentry_hold_active &&
      !reentry_phase_active_ && !reentry_lockout_active_ &&
      !generic_recovery_phase_active_ && permission_exception_start_context;
  const bool slow_front_safe_pass_permission_exception =
      config_.slow_front_permission_exception_enabled &&
      blocked.slow_front_exception_active && direct_slow_front &&
      slow_front_exception_inputs_complete && prediction_complete &&
      !blocked.overtake_permission_allowed &&
      !current_overtake_permission.allow_overtake && curvature_start_allowed &&
      has_safe_pass && no_pass_start_conflict;
  if (slow_front_safe_pass_permission_exception) {
    straight_overtake_start_allowed_ = true;
    blocked.straight_overtake_start_allowed = true;
    blocked.overtake_start_gate_reason = "slow_front_exception_permission";
    blocked.permission_start_exception_active = true;
  }
  // 停止parallel車の例外は通常のslow-front例外と混ぜない。CSV permissionは
  // falseのまま診断へ残し、同一ID確認済みearly probeがGate 2を通過した時だけ
  // state machineの開始gateを開く。lookahead先だけの禁止、曲率、future-yield、
  // reentry、壁/CBF拒否はいずれもこの条件を満たさず例外化しない。
  const bool stationary_parallel_safe_pass_permission_exception =
      config_.slow_front_permission_exception_enabled &&
      config_.early_stationary_parallel_pass_enabled &&
      config_.early_stationary_parallel_permission_exception_enabled &&
      blocked.early_stationary_parallel_pass_target &&
      (!blocked.start_grid_target_active ||
       blocked.start_grid_target_confirmed_stationary) &&
      blocked.early_stationary_parallel_pass_count >=
          std::max(1, config_.slow_front_exception_required_cycles) &&
      slow_front_exception_inputs_complete && prediction_complete &&
      !blocked.overtake_permission_allowed &&
      !current_overtake_permission.allow_overtake && curvature_start_allowed &&
      has_safe_pass && no_pass_start_conflict &&
      stationary_parallel_permission_start_context;
  blocked.confirmed_stationary_parallel_permission_exception =
      stationary_parallel_safe_pass_permission_exception;
  if (stationary_parallel_safe_pass_permission_exception) {
    straight_overtake_start_allowed_ = true;
    blocked.straight_overtake_start_allowed = true;
    blocked.permission_start_exception_active = true;
    blocked.overtake_start_gate_reason =
        "early_stationary_parallel_permission_exception";
    if (!isPrepareOvertakeMode(mode_)) {
      stationary_parallel_permission_prepare_id_ =
          blocked.early_stationary_parallel_pass_id;
    }
  }
  const double max_gentle_curve_cbf_slack =
      std::max(0.0, config_.gentle_curve_safe_pass_max_cbf_slack);
  const auto gentle_curve_candidate = std::find_if(
      candidates.begin(), candidates.end(), [this](const auto &candidate) {
        return localized_lateral_profile_.active &&
               candidate.type == localized_lateral_profile_.pass_type &&
               (candidate.type == CandidateType::PASS_LEFT ||
                candidate.type == CandidateType::PASS_RIGHT);
      });
  const bool has_safe_gentle_curve_pass =
      gentle_curve_candidate != candidates.end() &&
      localized_profile_target_identity_consistent &&
      gentle_curve_candidate->safety_evaluated &&
      gentle_curve_candidate->feasible &&
      std::isfinite(gentle_curve_candidate->cbf_slack) &&
      gentle_curve_candidate->cbf_slack <= max_gentle_curve_cbf_slack;
  blocked.gentle_curve_safe_pass_found = has_safe_gentle_curve_pass;
  blocked.gentle_curve_safe_pass_side = has_safe_gentle_curve_pass
                                            ? gentle_curve_candidate->type
                                            : CandidateType::FASTEST;
  blocked.gentle_curve_safe_pass_cbf_slack =
      gentle_curve_candidate != candidates.end()
          ? gentle_curve_candidate->cbf_slack
          : std::numeric_limits<double>::quiet_NaN();
  const bool gentle_curve_safe_pass_exception =
      blocked.gentle_curve_safe_pass_eligible &&
      exception_base_inputs_complete && gentle_curve_mpc_ready &&
      prediction_complete && pass_start_live_tracking_usable &&
      has_safe_gentle_curve_pass;
  blocked.gentle_curve_safe_pass_start_approved =
      gentle_curve_safe_pass_exception;
  blocked.gentle_curve_safe_pass_block_reason =
      gentle_curve_safe_pass_exception          ? "approved"
      : !config_.gentle_curve_safe_pass_enabled ? "disabled"
      : curvature_start_allowed ? "straight_curve_gate_already_open"
      : blocked.overtake_start_gate_reason != "curve" ? "not_curve_gate"
      : !blocked.overtake_permission_allowed          ? "permission_denied"
      : !(blocked.gentle_curve_direct_normal_target ||
          blocked.gentle_curve_direct_braking_target ||
          (blocked.gentle_curve_fresh_dynamic_gap_target &&
           blocked.gentle_curve_dynamic_target_speed_reachable) ||
          blocked.pass_start_target_continuity_active)
          ? "dynamic_target_context_unavailable"
      : !blocked.gentle_curve_context_clean ? "tactical_context_not_clean"
      : !blocked.gentle_curve_lateral_capacity_sufficient
          ? "lateral_capacity_insufficient"
      : !blocked.gentle_curve_dynamic_speed_cap_valid
          ? "dynamic_speed_cap_invalid"
      : !blocked.gentle_curve_curvature_within_limit ? "curvature_above_limit"
      : !exception_base_inputs_complete  ? "observation_inputs_incomplete"
      : !prediction_complete             ? "prediction_incomplete"
      : !pass_start_live_tracking_usable ? "tracking_unusable"
      : !gentle_curve_mpc_ready          ? "mpc_unready"
      : !has_safe_gentle_curve_pass
          ? (gentle_curve_candidate == candidates.end()
                 ? "authorized_side_candidate_missing"
             : !localized_profile_target_identity_consistent
                 ? "candidate_target_identity_mismatch"
             : !gentle_curve_candidate->safety_evaluated
                 ? "candidate_not_safety_evaluated"
             : !gentle_curve_candidate->feasible
                 ? (gentle_curve_candidate->reject_reason.empty()
                        ? "candidate_infeasible"
                        : gentle_curve_candidate->reject_reason)
             : !std::isfinite(gentle_curve_candidate->cbf_slack)
                 ? "candidate_cbf_nonfinite"
                 : "candidate_cbf_slack")
          : "not_approved";
  if (gentle_curve_safe_pass_exception) {
    // 曲率gateそのものを一般化して開かず、制限済みd/v列のPASSが同周期の
    // SafetyEvaluatorを通った時だけ状態機械へ許可を渡す。
    straight_overtake_start_allowed_ = true;
    blocked.straight_overtake_start_allowed = true;
    blocked.overtake_start_gate_reason = "gentle_curve_safe_pass";
  }
  const double max_stationary_no_pass_cbf_slack =
      std::max(0.0, config_.stationary_no_pass_safe_pass_max_cbf_slack);
  const bool has_safe_stationary_no_pass = std::any_of(
      candidates.begin(), candidates.end(),
      [max_stationary_no_pass_cbf_slack](const CandidateTrajectory &candidate) {
        return (candidate.type == CandidateType::PASS_LEFT ||
                candidate.type == CandidateType::PASS_RIGHT) &&
               candidate.feasible && std::isfinite(candidate.cbf_slack) &&
               candidate.cbf_slack <= max_stationary_no_pass_cbf_slack;
      });
  const bool stationary_no_pass_safe_pass_exception =
      blocked.stationary_no_pass_safe_pass_eligible &&
      slow_front_exception_inputs_complete && prediction_complete &&
      has_safe_stationary_no_pass;
  blocked.stationary_no_pass_safe_pass_start_approved =
      stationary_no_pass_safe_pass_exception;
  if (stationary_no_pass_safe_pass_exception) {
    // 停止障害物専用の制約PASSがGate 2を通った同周期だけ開始gateを開く。
    // 禁止区間ではCSV診断をfalseのまま残してpermissionだけを例外化し、
    // 許可区間では曲率gateだけを例外化する。通常の高曲率PASSへは波及させない。
    straight_overtake_start_allowed_ = true;
    blocked.straight_overtake_start_allowed = true;
    blocked.permission_start_exception_active =
        !blocked.overtake_permission_allowed;
    blocked.overtake_start_gate_reason =
        blocked.overtake_permission_allowed
            ? "stationary_high_curvature_safe_pass"
            : "stationary_no_pass_safe_pass";
  }
  const bool stationary_no_pass_safe_pass_reapproval_lost =
      isAnyPassMode(mode_) &&
      stationary_no_pass_safe_pass_constraint_latched_ &&
      !stationary_no_pass_safe_pass_exception;
  if (stationary_no_pass_safe_pass_reapproval_lost) {
    // 専用例外は開始時だけの許可ではない。stale/MPC不健全/対象ID変更/
    // Gate 2不成立を検出した周期はPASSとFASTESTを同時に除外し、再評価済みの
    // YIELDまたはRECOVERY、最終的には既存のfail-safeへ閉じる。
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::PASS_LEFT ||
          candidate.type == CandidateType::PASS_RIGHT ||
          candidate.type == CandidateType::FASTEST) {
        candidate.feasible = false;
        candidate.reject_reason =
            "stationary_no_pass_safe_pass_reapproval_lost";
      }
    }
  }

  const bool large_lateral_pass_start_inputs_complete =
      exception_base_inputs_complete && prediction_complete &&
      gentle_curve_mpc_ready;
  if (large_lateral_error && !isAnyPassMode(mode_) &&
      !large_lateral_pass_start_inputs_complete) {
    // 大きなdでも左右候補は必ずSafetyEvaluatorへ通す。ただし、その結果だけで
    // stale入力やMPC hard failure中の新規PASSを開始しない。物理候補の診断値は
    // 上で保持し、選択対象だけをfail-closedに落とす。
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::PASS_LEFT ||
          candidate.type == CandidateType::PASS_RIGHT) {
        candidate.feasible = false;
        candidate.reject_reason = "large_lateral_pass_start_inputs_incomplete";
      }
    }
  }

  const bool uncommitted_localized_pass_start =
      localized_lateral_profile_.active &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !blocked.start_grid_target_active &&
      (mode_ == BehaviorMode::FREE_RUN ||
       mode_ == BehaviorMode::FOLLOW_BLOCKED || isPrepareOvertakeMode(mode_));
  const bool dynamic_pass_start_tracking_usable =
      pass_start_live_tracking_usable;
  const bool dynamic_pass_start_inputs_complete =
      exception_base_inputs_complete && prediction_complete;
  if (uncommitted_localized_pass_start &&
      (!dynamic_pass_start_inputs_complete ||
       !dynamic_pass_start_tracking_usable)) {
    // 軌道形状が追従可能でも、実controller tracking/MPC health契約が
    // 揃わない、または観測対象を全て含むfreshな予測列が無い周期に
    // 新規dynamic PASSを開始・commitしない。候補診断は残し、FOLLOW/
    // YIELD/RECOVERY側へfail-closedに選択させる。
    for (auto &candidate : candidates) {
      if (candidate.type == CandidateType::PASS_LEFT ||
          candidate.type == CandidateType::PASS_RIGHT) {
        candidate.feasible = false;
        candidate.reject_reason = !dynamic_pass_start_inputs_complete
                                      ? "pass_start_inputs_incomplete"
                                      : "pass_start_tracking_unusable";
        if (candidate.type == CandidateType::PASS_LEFT) {
          blocked.pass_left_candidate_feasible = false;
          blocked.pass_left_candidate_reject_reason = candidate.reject_reason;
        } else {
          blocked.pass_right_candidate_feasible = false;
          blocked.pass_right_candidate_reject_reason = candidate.reject_reason;
        }
      }
    }
  }

  // PASS開始tracking不成立の原因を、候補生成後かつ選択前に一度だけ記録する。
  // このrecordは既存のfeasible/reject/score/state machineを書き換えず、Nodeの
  // controller status snapshotと候補形状を同一PlannerOutputで照合するだけ。
  const auto pass_start_diagnostic_candidate = [&]() {
    const CandidateType preferred_type =
        localized_lateral_profile_.active &&
                isPassCandidate(localized_lateral_profile_.pass_type)
            ? localized_lateral_profile_.pass_type
            : CandidateType::FASTEST;
    const auto preferred =
        std::find_if(candidates.cbegin(), candidates.cend(),
                     [preferred_type](const CandidateTrajectory &candidate) {
                       return candidate.type == preferred_type;
                     });
    if (preferred != candidates.cend()) {
      return preferred;
    }
    return std::find_if(candidates.cbegin(), candidates.cend(),
                        [](const CandidateTrajectory &candidate) {
                          return isPassCandidate(candidate.type);
                        });
  }();
  if (pass_start_diagnostic_candidate != candidates.cend()) {
    const auto &candidate = *pass_start_diagnostic_candidate;
    auto &diagnostic = blocked.pass_start_tracking_diagnostic;
    diagnostic.evaluated = true;
    diagnostic.candidate_present = true;
    diagnostic.candidate_type = candidate.type;
    diagnostic.endpoint_arc_m = candidate.longitudinal_offsets_m.empty()
                                    ? std::numeric_limits<double>::quiet_NaN()
                                    : candidate.longitudinal_offsets_m.back();
    diagnostic.required_arc_m = candidate.required_controller_spatial_horizon_m;
    diagnostic.transition_deadline = candidate.pass_transition_deadline;
    diagnostic.transition_deadline_evaluated =
        candidate.pass_transition_deadline.evaluated;
    diagnostic.transition_deadline_present =
        candidate.pass_transition_deadline.evaluated;
    diagnostic.controller_tracking_profile_valid =
        candidate.controller_tracking_profile_valid;
    diagnostic.desired_path_trackable = candidate.desired_path_trackable;
    diagnostic.pure_pursuit_command_trackable =
        candidate.pure_pursuit_command_trackable;
    diagnostic.actual_pose_start_evaluated =
        !candidate.x.empty() && candidate.x.size() == candidate.y.size() &&
        std::isfinite(ego.x) && std::isfinite(ego.y);
    diagnostic.actual_pose_start =
        diagnostic.actual_pose_start_evaluated &&
        std::hypot(candidate.x.front() - ego.x, candidate.y.front() - ego.y) <=
            1.0e-6;
    diagnostic.controller_status_received =
        reentry_input.controller_tracking_status_received;
    diagnostic.controller_plan_generation =
        reentry_input.controller_tracking_plan_generation;
    diagnostic.controller_expected_generation =
        reentry_input.controller_tracking_expected_generation;
    diagnostic.controller_status_reason =
        reentry_input.controller_tracking_status_reason;
    diagnostic.controller_mpc_horizon_usable =
        reentry_input.controller_tracking_mpc_horizon_usable;
    diagnostic.controller_continuity_usable =
        reentry_input.controller_tracking_continuity_usable;
    diagnostic.first_false =
        !diagnostic.actual_pose_start_evaluated
            ? "actual_pose_start_not_evaluated"
        : !diagnostic.actual_pose_start      ? "actual_pose_start_mismatch"
        : !diagnostic.desired_path_trackable ? "desired_path_untrackable"
        : !diagnostic.pure_pursuit_command_trackable
            ? "pure_pursuit_command_untrackable"
        : !diagnostic.controller_tracking_profile_valid
            ? "controller_tracking_profile_invalid"
        : !reentry_input.pure_pursuit_primary_and_fresh
            ? diagnostic.controller_status_reason.empty()
                  ? "controller_tracking_unusable"
                  : diagnostic.controller_status_reason
        : !diagnostic.controller_mpc_horizon_usable ? "mpc_horizon_unusable"
        : !diagnostic.controller_continuity_usable  ? "continuity_unusable"
                                                    : "none";
  }

  // raw PASS candidateがSafetyEvaluatorを通っていても、permission/transition
  // gateで最終不成立ならwide parallel FOLLOWを抑止してはいけない。ここでは
  // CandidateBuilderの集計済み最終PASS可否だけを仲裁入力にする。
  const bool feasible_pass_before_parallel_follow_recheck_selection =
      blocked.pass_left_candidate_feasible ||
      blocked.pass_right_candidate_feasible;
  for (auto &candidate : candidates) {
    candidate.score = candidateScore(candidate, blocked);
    if (early_wall_recovery_probe_requested &&
        candidate.type == CandidateType::RECOVERY &&
        candidate.safety_evaluated && candidate.feasible &&
        candidate.longitudinal_profile_valid &&
        candidate.controller_tracking_profile_valid &&
        candidate.controller_spatial_horizon_proof_valid) {
      // 新state/latchではなく同周期の候補優先度だけを変える。PASSの既存優先値
      // (-100)は上回らず、安全なPASSがあれば追越を妨げない。
      candidate.score = -95.0;
    }
    const bool verified_committed_attack_follow =
        candidate.type == CandidateType::FOLLOW && candidate.safety_evaluated &&
        candidate.feasible && candidate.longitudinal_profile_valid &&
        candidate.controller_tracking_profile_valid &&
        candidate.pass_target_corridor_valid &&
        blocked.maneuver_transaction_incomplete &&
        blocked.attack_follow_hold_pass_side &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        !blocked.maneuver_target_id.empty() &&
        !blocked.maneuver_chain_tail_id.empty() &&
        exception_base_inputs_complete && prediction_complete;
    if (verified_committed_attack_follow) {
      // 同じtarget/sideを保持したFOLLOW自身が全入力・操舵契約・SafetyEvaluatorを
      // 通った時だけ、PASS不成立周期のYIELD/FASTESTより優先する。成立中の
      // 同方向PASSは-100のため、FOLLOWが安全でも追越を早期解除しない。
      candidate.score = -90.0;
    }
    const bool verified_authorized_current_d_hold =
        candidate.type == CandidateType::RECOVERY &&
        blocked.authorized_pass_current_d_hold_active &&
        candidate.safety_evaluated && candidate.feasible &&
        candidate.longitudinal_profile_valid &&
        candidate.controller_tracking_profile_valid &&
        holds_current_lateral(candidate) && exception_base_inputs_complete &&
        prediction_complete;
    if (verified_authorized_current_d_hold) {
      // 同じ周期に成立するPASSは-100以下で優先する。PASSが不成立の時だけ、
      // SafetyEvaluator済み現在d保持をYIELD/SAFE_STOPより先に選ぶ。
      candidate.score = -90.0;
    }
    if (blocked.parallel_follow_recheck_attempted &&
        candidate.type == CandidateType::FOLLOW && candidate.feasible) {
      // PASSが成立した周期は、diagnostic目的で作ったparallel recheck FOLLOWを
      // 選ばない。PASS不成立時だけ通常のparallel FOLLOWと同じ優先度で
      // current-d FOLLOWへ遷移できる。
      candidate.score = feasible_pass_before_parallel_follow_recheck_selection
                            ? 1000.0
                            : -35.0;
    }
    if (freeze_overtake_decisions) {
      if (blocked.braking_follow_active &&
          candidate.type == CandidateType::FOLLOW) {
        candidate.score = -110.0;
      } else {
        candidate.score =
            candidate.type == CandidateType::RECOVERY ? -100.0 : 1000.0;
      }
    }
  }

  const bool no_feasible_pass =
      !hasFeasibleCandidate(candidates, isPassCandidate);
  const bool no_feasible_fallback =
      !hasFeasibleCandidate(candidates, isFallbackCandidate);
  const bool safe_stop_base_condition =
      needsSafeStopFallbackCheck(config_, mode_, blocked) && no_feasible_pass &&
      no_feasible_fallback;
  const bool start_grace_active =
      safe_stop_base_condition &&
      shouldSuppressSafeStopForStartGrace(now_sec, ego, blocked);
  const bool leader_priority_safe_stop_suppressed =
      safe_stop_base_condition && blocked.leader_priority_active;
  const bool effective_safe_stop_base_condition =
      safe_stop_base_condition && !start_grace_active &&
      !leader_priority_safe_stop_suppressed;

  // 処理ブロック: 回避不能状態が連続した時だけSAFE_STOP要求を作る。
  // 設計意図:
  // 一瞬のinfeasibleで停止に入ると走行が固まるため、trigger_cyclesで確定させる。
  if (effective_safe_stop_base_condition) {
    ++safe_stop_trigger_count_;
  } else {
    safe_stop_trigger_count_ = 0;
  }

  const int safe_stop_trigger_cycles_required =
      std::max(1, config_.safe_stop_trigger_cycles);
  SafeStopContext safe_stop_context;
  safe_stop_context.requested =
      effective_safe_stop_base_condition &&
      safe_stop_trigger_count_ >= safe_stop_trigger_cycles_required;
  safe_stop_context.trigger_count = safe_stop_trigger_count_;
  safe_stop_context.ego_speed_mps = ego.v;
  const auto ego_bounds =
      frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
  const double safe_stop_clamped_d =
      std::clamp(ego.frenet.d, ego_bounds.d_min + config_.min_wall_margin_m,
                 ego_bounds.d_max - config_.min_wall_margin_m);
  safe_stop_context.lateral_error_m = std::abs(safe_stop_clamped_d);

  bool safe_stop_candidate_infeasible = false;
  CandidateTrajectory safe_stop_candidate;
  if (safe_stop_context.requested || mode_ == BehaviorMode::SAFE_STOP) {
    // STOP要求時とSTOP保持中は、停止候補自体の安全性も毎周期確認する。
    safe_stop_candidate =
        makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(safe_stop_candidate, opponents, now_sec,
                                   predictions);
    if (!safe_stop_candidate.longitudinal_profile_valid) {
      safe_stop_candidate.feasible = false;
      safe_stop_candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    safe_stop_candidate.score = candidateScore(safe_stop_candidate, blocked);
    safe_stop_context.candidate_feasible = safe_stop_candidate.feasible;
    safe_stop_context.reason = safe_stop_candidate.feasible
                                   ? "no_pass_and_no_safe_fallback"
                                   : "safe_stop_infeasible";
    if (safe_stop_context.requested && safe_stop_candidate.feasible) {
      candidates.push_back(safe_stop_candidate);
    } else if (!safe_stop_candidate.feasible) {
      safe_stop_candidate_infeasible = true;
    }
  }
  if (safe_stop_candidate_infeasible && reentry_gate.requested &&
      !generic_recovery_phase_active_) {
    // 一度でも停止候補が不成立になった復帰文脈は、次周期に相手予測が欠けただけで
    // 通常ラインへ戻らないようlockoutする。解除は復帰ゲートの連続clear条件だけ。
    reentry_lockout_active_ = true;
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    if (reentry_gate.requested) {
      reentry_gate.permitted = false;
      reentry_gate.clear_cycles = 0;
      reentry_gate.reason = "safe_stop_infeasible";
      reentry_gate.blocking_vehicle_id =
          safe_stop_candidate.blocking_opponent_id;
      reentry_gate.min_safety_margin = safe_stop_candidate.min_safety_margin;
      reentry_gate.cbf_slack = safe_stop_candidate.cbf_slack;
      reentry_gate.blocking_time_sec = safe_stop_candidate.blocking_time_sec;
      blocked.reentry_hold_active = true;
    }
  }

  if (config_.supervisor_v2_shadow_enabled) {
    // V2は旧state machineの選択前に、SafetyEvaluator済み候補だけを見る。
    // 現行制御はこのdecisionを使わず、shadow topicへ記録するだけである。
    std::vector<CandidateTrajectory> supervisor_candidates = candidates;
    const CandidateBuilder v2_candidate_builder(frame_, config_);
    const V2LocalizedPassProfileBuilder v2_profile_builder(frame_, config_);

    // classifierがこの周期に示した対象と、PASS中にV2が固定している対象を
    // 分離する。PASS完了前はnearest/side分類の揺れでtargetを変えず、固定IDの
    // fresh観測を全opponentsから直接引き直す。
    std::string classifier_target_id = blocked.start_grid_target_id;
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.nearest_id;
    }
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.early_stationary_parallel_pass_id;
    }
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.stationary_front_id;
    }
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.early_low_speed_pass_target_id;
    }
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.parallel_follow_id;
    }
    if (classifier_target_id.empty()) {
      classifier_target_id = blocked.side_id;
    }
    const bool classifier_target_context =
        blocked.blocked || blocked.start_grid_target_active ||
        blocked.early_stationary_parallel_pass_target ||
        blocked.stationary_front_obstacle ||
        blocked.parallel_follow_candidate || blocked.braking_follow_active ||
        blocked.early_low_speed_pass_target_active || blocked.side_by_side;
    const auto fresh_target_by_id = [this, now_sec,
                                     &opponents](const std::string &id) {
      return std::find_if(
          opponents.begin(), opponents.end(),
          [this, now_sec, &id](const OpponentState &opponent) {
            return !id.empty() && opponent.id == id && opponent.valid &&
                   std::isfinite(opponent.frenet.s) &&
                   std::isfinite(opponent.frenet.d) &&
                   std::isfinite(opponent.v) &&
                   inputTimestampFresh(
                       now_sec, opponent.stamp_sec,
                       config_.opponent_stale_time_sec,
                       config_.input_future_stamp_tolerance_sec);
          });
    };
    const std::string &v2_latched_target_id = supervisor_v2_.targetVehicleId();
    bool v2_current_target_pass_complete = false;
    const bool v2_target_transaction_active =
        (supervisor_v2_.phase() == TacticalPhase::PASSING ||
         supervisor_v2_.phase() == TacticalPhase::ATTACK_FOLLOW) &&
        !v2_latched_target_id.empty();
    bool v2_committed_profile_progress_valid = true;
    bool v2_target_progress_observation_advanced = false;
    if (supervisor_v2_.phase() == TacticalPhase::PASSING) {
      const auto *committed_profile = supervisor_v2_.passProfile();
      if (committed_profile == nullptr || !committed_profile->active) {
        v2_committed_profile_progress_valid = false;
      } else {
        LocalizedLateralProfile advanced_profile = *committed_profile;
        double previous_target_stamp_sec =
            std::numeric_limits<double>::quiet_NaN();
        const auto previous_target_waypoint =
            std::find_if(advanced_profile.chain_waypoints.begin(),
                         advanced_profile.chain_waypoints.end(),
                         [&v2_latched_target_id](const auto &waypoint) {
                           return waypoint.target_id == v2_latched_target_id;
                         });
        if (previous_target_waypoint !=
            advanced_profile.chain_waypoints.end()) {
          previous_target_stamp_sec =
              previous_target_waypoint->last_observed_stamp_sec;
        }
        v2_committed_profile_progress_valid =
            v2_profile_builder.advanceLongitudinalProgress(
                now_sec, ego, opponents, advanced_profile) &&
            supervisor_v2_.applyPassProfileProgress(advanced_profile);
        if (v2_committed_profile_progress_valid) {
          const auto *updated_profile = supervisor_v2_.passProfile();
          const auto updated_target_waypoint =
              updated_profile == nullptr
                  ? std::vector<LocalizedLateralWaypoint>::const_iterator{}
                  : std::find_if(updated_profile->chain_waypoints.begin(),
                                 updated_profile->chain_waypoints.end(),
                                 [&v2_latched_target_id](const auto &waypoint) {
                                   return waypoint.target_id ==
                                          v2_latched_target_id;
                                 });
          v2_target_progress_observation_advanced =
              updated_profile != nullptr &&
              updated_target_waypoint !=
                  updated_profile->chain_waypoints.end() &&
              std::isfinite(previous_target_stamp_sec) &&
              updated_target_waypoint->last_observed_stamp_sec >
                  previous_target_stamp_sec + 1.0e-9;
        }
      }
    }
    bool v2_attack_follow_progress_valid = true;
    bool v2_attack_follow_target_observation_advanced = false;
    if (supervisor_v2_.phase() == TacticalPhase::ATTACK_FOLLOW &&
        !v2_latched_target_id.empty()) {
      const auto target_observation = fresh_target_by_id(v2_latched_target_id);
      if (!ego.valid || !std::isfinite(ego.stamp_sec) ||
          !std::isfinite(ego.frenet.s) ||
          target_observation == opponents.end()) {
        v2_attack_follow_progress_valid = false;
      } else if (supervisor_v2_progress_target_id_ != v2_latched_target_id ||
                 !std::isfinite(supervisor_v2_progress_ego_unwrapped_s_m_) ||
                 !std::isfinite(supervisor_v2_progress_target_unwrapped_s_m_)) {
        supervisor_v2_progress_target_id_ = v2_latched_target_id;
        supervisor_v2_progress_ego_unwrapped_s_m_ = ego.frenet.s;
        supervisor_v2_progress_last_ego_wrapped_s_m_ = ego.frenet.s;
        supervisor_v2_progress_last_ego_stamp_sec_ = ego.stamp_sec;
        supervisor_v2_progress_target_unwrapped_s_m_ =
            ego.frenet.s +
            frame_.deltaS(ego.frenet.s, target_observation->frenet.s);
        supervisor_v2_progress_last_target_wrapped_s_m_ =
            target_observation->frenet.s;
        supervisor_v2_progress_last_target_stamp_sec_ =
            target_observation->stamp_sec;
      } else {
        const auto plausible_progress = [&](double delta_s_m, double dt_sec,
                                            double speed_mps) {
          if (!std::isfinite(delta_s_m) || !std::isfinite(dt_sec) ||
              dt_sec < 0.0 || !std::isfinite(speed_mps) || speed_mps < 0.0 ||
              dt_sec > config_.opponent_stale_time_sec + 1.0e-9) {
            return false;
          }
          const double speed_bound_mps = std::max(
              {1.0, speed_mps, std::max(0.0, config_.v_passthrough_mps)});
          return std::abs(delta_s_m) <=
                 speed_bound_mps * dt_sec + 0.50 + 1.0e-9;
        };
        if (ego.stamp_sec + 1.0e-9 <
                supervisor_v2_progress_last_ego_stamp_sec_ ||
            target_observation->stamp_sec + 1.0e-9 <
                supervisor_v2_progress_last_target_stamp_sec_) {
          v2_attack_follow_progress_valid = false;
        } else {
          if (ego.stamp_sec >
              supervisor_v2_progress_last_ego_stamp_sec_ + 1.0e-9) {
            const double dt_sec =
                ego.stamp_sec - supervisor_v2_progress_last_ego_stamp_sec_;
            const double delta_s_m =
                signedDeltaS(supervisor_v2_progress_last_ego_wrapped_s_m_,
                             ego.frenet.s, frame_.length());
            if (!plausible_progress(delta_s_m, dt_sec, std::max(0.0, ego.v))) {
              v2_attack_follow_progress_valid = false;
            } else {
              supervisor_v2_progress_ego_unwrapped_s_m_ += delta_s_m;
              supervisor_v2_progress_last_ego_wrapped_s_m_ = ego.frenet.s;
              supervisor_v2_progress_last_ego_stamp_sec_ = ego.stamp_sec;
            }
          }
          if (v2_attack_follow_progress_valid &&
              target_observation->stamp_sec >
                  supervisor_v2_progress_last_target_stamp_sec_ + 1.0e-9) {
            const double dt_sec = target_observation->stamp_sec -
                                  supervisor_v2_progress_last_target_stamp_sec_;
            const double delta_s_m =
                signedDeltaS(supervisor_v2_progress_last_target_wrapped_s_m_,
                             target_observation->frenet.s, frame_.length());
            if (!plausible_progress(delta_s_m, dt_sec,
                                    std::max(0.0, target_observation->v)) ||
                delta_s_m < -0.05) {
              v2_attack_follow_progress_valid = false;
            } else {
              supervisor_v2_progress_target_unwrapped_s_m_ +=
                  std::max(0.0, delta_s_m);
              supervisor_v2_progress_last_target_wrapped_s_m_ =
                  target_observation->frenet.s;
              supervisor_v2_progress_last_target_stamp_sec_ =
                  target_observation->stamp_sec;
              v2_attack_follow_target_observation_advanced = true;
            }
          }
        }
      }
    } else if (!v2_target_transaction_active) {
      supervisor_v2_progress_target_id_.clear();
      supervisor_v2_progress_ego_unwrapped_s_m_ =
          std::numeric_limits<double>::quiet_NaN();
      supervisor_v2_progress_target_unwrapped_s_m_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    if (v2_target_transaction_active && ego.valid &&
        std::isfinite(ego.frenet.s) && std::isfinite(ego.v)) {
      // target IDの所有権と完了判定は同じ車両にそろえる。chain tailまで先頭IDを
      // 保持すると、途中車のために長くなったhorizonが一時的にrejectされた周期に
      // 「先頭は既に抜いた」という事実をhandoffへ使えずABORTへ落ちる。各targetを
      // 幾何学的に抜いた後、Supervisorが連続確認し、ATTACK_FOLLOWを1周期挟んで
      // 次targetを独立Gate
      // 2へ掛ける。chain全体のfreshness/SafetyEvaluatorは別途
      // 維持するため、未観測車を無視した完了にはならない。
      const auto v2_completion_obstacle =
          fresh_target_by_id(v2_latched_target_id);
      if (v2_completion_obstacle != opponents.end()) {
        double relative_s_m = signedDeltaS(
            ego.frenet.s, v2_completion_obstacle->frenet.s, frame_.length());
        bool completion_observation_valid = true;
        if (supervisor_v2_.phase() == TacticalPhase::PASSING) {
          const auto *committed_profile = supervisor_v2_.passProfile();
          completion_observation_valid =
              v2_committed_profile_progress_valid &&
              v2_target_progress_observation_advanced &&
              committed_profile != nullptr &&
              std::isfinite(committed_profile->target_s_m) &&
              std::isfinite(committed_profile->ego_unwrapped_s_m);
          if (completion_observation_valid) {
            relative_s_m = committed_profile->target_s_m -
                           committed_profile->ego_unwrapped_s_m;
          }
        } else if (supervisor_v2_.phase() == TacticalPhase::ATTACK_FOLLOW) {
          completion_observation_valid =
              v2_attack_follow_progress_valid &&
              v2_attack_follow_target_observation_advanced &&
              supervisor_v2_progress_target_id_ == v2_latched_target_id &&
              std::isfinite(supervisor_v2_progress_target_unwrapped_s_m_) &&
              std::isfinite(supervisor_v2_progress_ego_unwrapped_s_m_);
          if (completion_observation_valid) {
            relative_s_m = supervisor_v2_progress_target_unwrapped_s_m_ -
                           supervisor_v2_progress_ego_unwrapped_s_m_;
          }
        }
        const double relative_speed_mps = ego.v - v2_completion_obstacle->v;
        v2_current_target_pass_complete =
            completion_observation_valid &&
            relative_s_m <= -std::max(0.0, config_.merge_front_gap_m) &&
            relative_speed_mps >=
                -std::max(0.0, config_.dv_block_threshold_mps);
      }
    }

    std::string target_id = classifier_target_id;
    bool target_present = classifier_target_context && !target_id.empty() &&
                          fresh_target_by_id(target_id) != opponents.end();
    const bool v2_force_latched_target =
        v2_target_transaction_active && !supervisor_v2_.passCompletionPending();
    if (v2_force_latched_target) {
      target_id = v2_latched_target_id;
      target_present = fresh_target_by_id(target_id) != opponents.end();
    }
    const bool v2_use_committed_profile =
        supervisor_v2_.phase() == TacticalPhase::PASSING &&
        v2_committed_profile_progress_valid && !v2_latched_target_id.empty() &&
        target_id == v2_latched_target_id;
    bool v2_committed_chain_all_fresh = true;
    if (supervisor_v2_.phase() == TacticalPhase::PASSING) {
      const auto *committed_profile = supervisor_v2_.passProfile();
      if (committed_profile == nullptr || !committed_profile->active ||
          committed_profile->target_id != v2_latched_target_id ||
          committed_profile->chain_waypoints.empty()) {
        v2_committed_chain_all_fresh = false;
      } else {
        v2_committed_chain_all_fresh = std::all_of(
            committed_profile->chain_waypoints.begin(),
            committed_profile->chain_waypoints.end(),
            [&fresh_target_by_id,
             &opponents](const LocalizedLateralWaypoint &waypoint) {
              return !waypoint.target_id.empty() &&
                     fresh_target_by_id(waypoint.target_id) != opponents.end();
            });
      }
    }
    const bool v2_latched_target_temporarily_unavailable =
        v2_target_transaction_active &&
        !supervisor_v2_.passCompletionPending() &&
        (!target_present ||
         (supervisor_v2_.phase() == TacticalPhase::ATTACK_FOLLOW &&
          !v2_attack_follow_progress_valid) ||
         (supervisor_v2_.phase() == TacticalPhase::PASSING &&
          (!v2_committed_chain_all_fresh ||
           !v2_committed_profile_progress_valid)));

    std::optional<LocalizedLateralProfile> v2_left_profile;
    std::optional<LocalizedLateralProfile> v2_right_profile;
    if (target_present) {
      // 旧state/freezeが候補生成を抑えたことをV2へ持ち込まない。戦術選択は
      // 独立候補集合で行い、安全評価器・壁/他車閾値だけを共通化する。
      supervisor_candidates.erase(
          std::remove_if(supervisor_candidates.begin(),
                         supervisor_candidates.end(),
                         [](const CandidateTrajectory &candidate) {
                           return candidate.type == CandidateType::FOLLOW ||
                                  candidate.type == CandidateType::PASS_LEFT ||
                                  candidate.type == CandidateType::PASS_RIGHT;
                         }),
          supervisor_candidates.end());
      BlockedInfo v2_blocked = blocked;
      v2_blocked.pass_target_corridor_preflight_required =
          !v2_use_committed_profile;
      if (!v2_use_committed_profile && std::isfinite(ego.frenet.d)) {
        // PASS開始前のATTACK_FOLLOWは、候補PASSの横目標へ先回りして移動しない。
        // 現在dを維持したまま同一targetを追い、開始gateが開いた周期にだけ
        // SafetyEvaluator済みPASSへ遷移する。これによりgrid外側スタートで
        // 3 m超の横断をFOLLOWへ誤って要求し、trackability rejectで停止する
        // 経路を防ぐ。FOLLOW候補自体の壁・他車・制動・追従可能性評価は残す。
        v2_blocked.attack_follow_hold_pass_side = true;
        v2_blocked.attack_follow_target_d_m = ego.frenet.d;
      }
      const auto v2_target = fresh_target_by_id(target_id);
      if (v2_target != opponents.end()) {
        v2_left_profile =
            v2_profile_builder.build(now_sec, ego, *v2_target, opponents,
                                     CandidateType::PASS_LEFT, v2_blocked);
        v2_right_profile =
            v2_profile_builder.build(now_sec, ego, *v2_target, opponents,
                                     CandidateType::PASS_RIGHT, v2_blocked);
      }
      for (const auto type : {CandidateType::FOLLOW, CandidateType::PASS_LEFT,
                              CandidateType::PASS_RIGHT}) {
        // V2候補へ旧stateのlocalized profile実体を持ち込まない。開始前は
        // V2専用provisional profile、PASS中はSupervisorがGate 2でcommitした
        // exact
        // snapshotだけを使い、同じCandidateBuilder/SafetyEvaluatorで再評価する。
        const LocalizedLateralProfile *v2_pass_profile = nullptr;
        if (type == CandidateType::PASS_LEFT ||
            type == CandidateType::PASS_RIGHT) {
          const auto *committed_profile = supervisor_v2_.passProfile();
          if (supervisor_v2_.phase() == TacticalPhase::PASSING) {
            if (v2_use_committed_profile && committed_profile != nullptr &&
                committed_profile->active &&
                committed_profile->pass_type == type &&
                committed_profile->target_id == target_id) {
              v2_pass_profile = committed_profile;
            }
          } else if (type == CandidateType::PASS_LEFT &&
                     v2_left_profile.has_value()) {
            v2_pass_profile = &*v2_left_profile;
          } else if (type == CandidateType::PASS_RIGHT &&
                     v2_right_profile.has_value()) {
            v2_pass_profile = &*v2_right_profile;
          }
        }
        auto candidate = v2_candidate_builder.makeCandidate(
            type, ego, v2_blocked, opponents, v2_pass_profile,
            type == CandidateType::PASS_LEFT ||
                type == CandidateType::PASS_RIGHT,
            (type == CandidateType::PASS_LEFT ||
             type == CandidateType::PASS_RIGHT)
                ? CandidatePurpose::PROPOSAL_SAFETY_EVALUATION
                : CandidatePurpose::EXECUTION);
        evaluateCandidateAtOwnTimeAxis(candidate, opponents, now_sec,
                                       predictions);
        if ((type == CandidateType::PASS_LEFT ||
             type == CandidateType::PASS_RIGHT) &&
            v2_pass_profile == nullptr) {
          // V2 PASSは軌道とexact localized profileを1組でGate 2へ渡す。
          // legacy
          // smoothstepへ暗黙fallbackして、別形状のprofileをcommitしない。
          candidate.feasible = false;
          candidate.reject_reason = "v2_localized_profile_unavailable";
        }
        if (!candidate.longitudinal_profile_valid) {
          candidate.feasible = false;
          candidate.reject_reason = "invalid_longitudinal_brake_model";
        }
        const bool stationary_braking_shortfall =
            type == CandidateType::FOLLOW &&
            (blocked.stationary_front_obstacle ||
             blocked.braking_follow_active) &&
            candidate.required_brake_distance_m >
                candidate.available_brake_distance_m;
        if (stationary_braking_shortfall) {
          candidate.feasible = false;
          candidate.reject_reason = "insufficient_braking_distance";
        }
        candidate.score = candidateScore(candidate, v2_blocked);
        supervisor_candidates.push_back(std::move(candidate));
      }
    }
    BlockedInfo hold_blocked = blocked;
    hold_blocked.reentry_hold_active = true;
    CandidateTrajectory evaluated_hold = v2_candidate_builder.makeCandidate(
        CandidateType::RECOVERY, ego, hold_blocked, opponents, nullptr);
    evaluateCandidateAtOwnTimeAxis(evaluated_hold, opponents, now_sec,
                                   predictions);
    if (!evaluated_hold.longitudinal_profile_valid) {
      evaluated_hold.feasible = false;
      evaluated_hold.reject_reason = "invalid_longitudinal_brake_model";
    }
    const auto holds_current_lateral =
        [&ego](const CandidateTrajectory &candidate) {
          return std::isfinite(ego.frenet.d) && !candidate.d.empty() &&
                 std::all_of(candidate.d.begin(), candidate.d.end(),
                             [&ego](double candidate_d) {
                               return std::isfinite(candidate_d) &&
                                      std::abs(candidate_d - ego.frenet.d) <=
                                          1.0e-6;
                             });
        };
    if (!holds_current_lateral(evaluated_hold)) {
      evaluated_hold.feasible = false;
      evaluated_hold.reject_reason = "abort_hold_not_current_lateral";
    }
    evaluated_hold.score = candidateScore(evaluated_hold, hold_blocked);
    supervisor_candidates.push_back(evaluated_hold);

    BlockedInfo abort_stop_blocked = blocked;
    abort_stop_blocked.abort_safe_stop_hold_lateral = true;
    CandidateTrajectory evaluated_abort_stop =
        v2_candidate_builder.makeCandidate(CandidateType::SAFE_STOP, ego,
                                           abort_stop_blocked, opponents,
                                           nullptr);
    evaluateCandidateAtOwnTimeAxis(evaluated_abort_stop, opponents, now_sec,
                                   predictions);
    if (!evaluated_abort_stop.longitudinal_profile_valid) {
      evaluated_abort_stop.feasible = false;
      evaluated_abort_stop.reject_reason = "invalid_longitudinal_brake_model";
    }
    if (!holds_current_lateral(evaluated_abort_stop)) {
      evaluated_abort_stop.feasible = false;
      evaluated_abort_stop.reject_reason = "abort_stop_not_current_lateral";
    }
    evaluated_abort_stop.score =
        candidateScore(evaluated_abort_stop, abort_stop_blocked);

    const auto centering_assessment = assessSupervisorV2Centering(
        now_sec, ego, blocked, opponents, reentry_input);

    const bool safety_inputs_complete =
        exception_base_inputs_complete && prediction_complete &&
        !v2_latched_target_temporarily_unavailable;
    const bool tracking_usable = reentry_input.pure_pursuit_primary_and_fresh &&
                                 reentry_input.mpc_health_fresh &&
                                 !reentry_input.mpc_hard_failure;
    const bool pass_start_allowed =
        blocked.straight_overtake_start_allowed &&
        (blocked.overtake_permission_allowed ||
         blocked.permission_start_exception_active ||
         blocked.gentle_curve_safe_pass_start_approved ||
         blocked.stationary_no_pass_safe_pass_start_approved);
    const auto v2_pass_candidate_feasible = [&supervisor_candidates](
                                                CandidateType type) {
      return std::any_of(supervisor_candidates.begin(),
                         supervisor_candidates.end(),
                         [type](const CandidateTrajectory &candidate) {
                           return candidate.type == type && candidate.feasible;
                         });
    };
    const bool pass_left_start_allowed =
        pass_start_allowed &&
        (!blocked.gentle_curve_safe_pass_start_approved ||
         blocked.gentle_curve_safe_pass_side == CandidateType::PASS_LEFT) &&
        v2_left_profile.has_value() &&
        v2_pass_candidate_feasible(CandidateType::PASS_LEFT);
    const bool pass_right_start_allowed =
        pass_start_allowed &&
        (!blocked.gentle_curve_safe_pass_start_approved ||
         blocked.gentle_curve_safe_pass_side == CandidateType::PASS_RIGHT) &&
        v2_right_profile.has_value() &&
        v2_pass_candidate_feasible(CandidateType::PASS_RIGHT);
    const CandidateType abort_release_type =
        target_present ? CandidateType::FOLLOW : CandidateType::FASTEST;
    const bool abort_release_destination_feasible = std::any_of(
        supervisor_candidates.begin(), supervisor_candidates.end(),
        [abort_release_type](const CandidateTrajectory &candidate) {
          return candidate.type == abort_release_type && candidate.feasible;
        });
    const bool abort_release_allowed =
        centering_assessment.safe && safety_inputs_complete &&
        tracking_usable && std::isfinite(ego.frenet.d) &&
        std::isfinite(blocked.ego_wall_clearance_m) &&
        blocked.ego_wall_clearance_m >= 0.0 &&
        abort_release_destination_feasible;
    std::uint32_t authorization_failure_mask = SUPERVISOR_V2_AUTH_NONE;
    const auto add_authorization_failure =
        [&authorization_failure_mask](std::uint32_t failure) {
          authorization_failure_mask |= failure;
        };
    if (!reentry_input.ego_fresh) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_EGO_STALE);
    }
    if (!reentry_input.v2x_snapshot_fresh) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_V2X_STALE);
    }
    if (!reentry_input.all_observed_opponents_fresh) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_OPPONENT_STALE);
    }
    if (v2_latched_target_temporarily_unavailable) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_OPPONENT_STALE);
    }
    if (!v2_committed_profile_progress_valid ||
        !v2_attack_follow_progress_valid) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_PROFILE_PROGRESS_INVALID);
    }
    if (!reentry_input.all_observed_opponents_included) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_OPPONENT_EXCLUDED);
    }
    if (!reentry_input.reference_valid) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_REFERENCE_INVALID);
    }
    if (!prediction_complete) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_PREDICTION_INCOMPLETE);
    }
    if (!reentry_input.pure_pursuit_primary_and_fresh) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE);
    }
    if (!reentry_input.mpc_health_fresh) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_MPC_HEALTH_STALE);
    }
    if (reentry_input.mpc_hard_failure) {
      add_authorization_failure(SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE);
    }
    const auto candidate_for_type = [&supervisor_candidates](
                                        CandidateType type) {
      const auto candidate = std::find_if(
          supervisor_candidates.begin(), supervisor_candidates.end(),
          [type](const CandidateTrajectory &item) {
            return item.type == type;
          });
      return candidate == supervisor_candidates.end() ? nullptr : &*candidate;
    };
    const SupervisorV2PassProbe v2_left_probe{
        candidate_for_type(CandidateType::PASS_LEFT),
        v2_left_profile.has_value() ? &*v2_left_profile : nullptr, target_id};
    const SupervisorV2PassProbe v2_right_probe{
        candidate_for_type(CandidateType::PASS_RIGHT),
        v2_right_profile.has_value() ? &*v2_right_profile : nullptr, target_id};
    supervisor_v2_decision = supervisor_v2_.update(SupervisorV2Input{
        supervisor_v2_cycle_sequence_,
        target_present,
        target_id,
        safety_inputs_complete,
        tracking_usable,
        pass_left_start_allowed,
        pass_right_start_allowed,
        v2_current_target_pass_complete,
        v2_latched_target_temporarily_unavailable,
        centering_assessment.safe,
        std::isfinite(ego.frenet.d) &&
            std::abs(ego.frenet.d) <= reentryCompletionLateralErrorM(config_),
        abort_release_allowed,
        false,
        authorization_failure_mask,
        &supervisor_candidates,
        &evaluated_hold,
        &centering_assessment.candidate,
        &evaluated_abort_stop,
        v2_left_probe,
        v2_right_probe,
    });
    const auto candidate_diagnostic =
        [&supervisor_candidates](CandidateType type) {
          SupervisorV2CandidateDiagnostic diagnostic;
          const auto candidate = std::find_if(
              supervisor_candidates.begin(), supervisor_candidates.end(),
              [type](const CandidateTrajectory &item) {
                return item.type == type;
              });
          if (candidate == supervisor_candidates.end()) {
            return diagnostic;
          }
          diagnostic.generated = true;
          diagnostic.safety_evaluated = candidate->safety_evaluated;
          diagnostic.feasible = candidate->feasible;
          diagnostic.controller_tracking_profile_valid =
              candidate->controller_tracking_profile_valid;
          diagnostic.desired_path_trackable = candidate->desired_path_trackable;
          diagnostic.pure_pursuit_command_trackable =
              candidate->pure_pursuit_command_trackable;
          diagnostic.moving_target_relatively_reachable =
              candidate->moving_target_relatively_reachable;
          diagnostic.reject_reason = candidate->reject_reason;
          diagnostic.endpoint_arc_m =
              candidate->longitudinal_offsets_m.empty()
                  ? std::numeric_limits<double>::quiet_NaN()
                  : candidate->longitudinal_offsets_m.back();
          diagnostic.required_arc_m =
              candidate->required_controller_spatial_horizon_m;
          diagnostic.planned_target_d_m = candidate->planned_target_d_m;
          diagnostic.min_safety_margin = candidate->min_safety_margin;
          diagnostic.blocking_opponent_id = candidate->blocking_opponent_id;
          diagnostic.blocking_time_sec = candidate->blocking_time_sec;
          return diagnostic;
        };
    // 選択結果とは別に3候補を記録する。shadow観測専用であり、V2の
    // plan_generation・認可・候補選択には一切入力しない。
    supervisor_v2_decision.follow_candidate =
        candidate_diagnostic(CandidateType::FOLLOW);
    supervisor_v2_decision.pass_left_candidate =
        candidate_diagnostic(CandidateType::PASS_LEFT);
    supervisor_v2_decision.pass_right_candidate =
        candidate_diagnostic(CandidateType::PASS_RIGHT);
  }

  // 処理ブロック: 内部候補を選び、状態機械で運転modeを安定化する。
  // 設計意図:
  // score上の最良候補をそのままpublishせず、保持時間や連続安全回数を通してmodeを決める。
  CandidateTrajectory selected = selectCandidate(candidates);
  bool tracking_release_candidate_installed_this_cycle = false;
  std::string tracking_release_candidate_failure_reason{};
  const bool all_remaining_chain_waypoints_observed_fresh = [&]() {
    if (!localized_lateral_profile_.active ||
        localized_lateral_profile_.chain_waypoints.empty()) {
      return true;
    }
    const auto current_waypoint =
        std::find_if(localized_lateral_profile_.chain_waypoints.cbegin(),
                     localized_lateral_profile_.chain_waypoints.cend(),
                     [&blocked](const auto &waypoint) {
                       return waypoint.target_id == blocked.maneuver_target_id;
                     });
    if (current_waypoint == localized_lateral_profile_.chain_waypoints.cend()) {
      return false;
    }
    return std::all_of(
        current_waypoint, localized_lateral_profile_.chain_waypoints.cend(),
        [&opponents, now_sec, this](const auto &waypoint) {
          return std::any_of(
              opponents.cbegin(), opponents.cend(),
              [&waypoint, now_sec, this](const auto &opponent) {
                return opponent.valid && opponent.id == waypoint.target_id &&
                       inputTimestampFresh(
                           now_sec, opponent.stamp_sec,
                           config_.opponent_stale_time_sec,
                           config_.input_future_stamp_tolerance_sec);
              });
        });
  }();
  const auto is_transaction_pass = [&blocked](CandidateType type) {
    return type == blocked.maneuver_transaction_pass_type &&
           (type == CandidateType::PASS_LEFT ||
            type == CandidateType::PASS_RIGHT);
  };
  const auto recovery_nonmotion_probe_candidate = std::find_if(
      candidates.cbegin(), candidates.cend(),
      [&is_transaction_pass, this](const CandidateTrajectory &candidate) {
        return is_transaction_pass(candidate.type) &&
               candidate.safety_evaluated && candidate.feasible &&
               candidate.longitudinal_profile_valid &&
               candidate.controller_tracking_profile_valid &&
               candidate.desired_path_trackable &&
               candidate.pure_pursuit_command_trackable &&
               hasControllerSpatialHorizonProof(candidate);
      });
  const bool recovery_nonmotion_probe_target_matches =
      !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == blocked.nearest_id &&
      blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
      blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_chain_tail_id.empty();
  const bool recovery_nonmotion_probe_context =
      (mode_ == BehaviorMode::ABORT_RECOVERY ||
       mode_ == BehaviorMode::SPEED_GUARD) &&
      !blocked.maneuver_transaction_incomplete &&
      blocked.slow_front_exception_active &&
      recovery_nonmotion_probe_target_matches && reentry_gate.requested &&
      reentry_gate.input_complete && reentry_gate.permitted &&
      reentry_gate.reason == "reentry_clear" && !blocked.side_by_side &&
      !blocked.corner_side_by_side && !blocked.future_corner_side_by_side &&
      !blocked.future_yield_required && !blocked.parallel_yield_hold_lateral &&
      !blocked.pass_decision_frozen &&
      !blocked.pass_reauthorization_lockout_active &&
      !blocked.post_abort_curve_hold_active &&
      !blocked.straight_overtake_start_allowed &&
      blocked.overtake_start_gate_reason == "curve" &&
      blocked.overtake_permission_allowed && exception_base_inputs_complete &&
      prediction_complete && pass_start_live_tracking_usable &&
      gentle_curve_mpc_ready &&
      recovery_nonmotion_probe_candidate != candidates.cend();
  if (recovery_nonmotion_probe_context) {
    // 現周期の中心復帰自体が全入力・全相手評価を通過し、同じ対象へのPASSも
    // SafetyEvaluator/PP空間追従性を全て満たした場合だけ、RECOVERY stateとは
    // 独立したSTOP下warm-upへ候補を提出する。ここではmodeやreentry latchを
    // 解除せず、正速度・横motionは後続exact ACK/grantまで許可しない。
    selected = *recovery_nonmotion_probe_candidate;
  }
  const bool prepared_pass_probe_ready =
      blocked.start_grid_target_active &&
      blocked.start_grid_target_confirmed_stationary &&
      blocked.maneuver_transaction_prepared &&
      !blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == blocked.start_grid_target_id &&
      !blocked.maneuver_chain_tail_id.empty() &&
      blocked.maneuver_target_pass_safety_approved &&
      blocked.straight_overtake_start_allowed &&
      (blocked.overtake_permission_allowed ||
       blocked.permission_start_exception_active) &&
      exception_base_inputs_complete && prediction_complete &&
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      is_transaction_pass(selected.type);
  const bool committed_retry_pass_ready =
      (blocked.start_grid_target_active ||
       maneuver_execution_hold_identity_matches()) &&
      blocked.maneuver_transaction_retry_active &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      exception_base_inputs_complete && prediction_complete &&
      (reentry_input.pure_pursuit_release_ready ||
       moving_attack_follow_retry_ready) &&
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      is_transaction_pass(selected.type);
  const bool recovery_nonmotion_pass_probe_ready =
      recovery_nonmotion_probe_context && blocked.maneuver_target_latched &&
      all_remaining_chain_waypoints_observed_fresh &&
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      selected.desired_path_trackable &&
      selected.pure_pursuit_command_trackable &&
      hasControllerSpatialHorizonProof(selected) &&
      is_transaction_pass(selected.type);
  const bool start_grid_pass_probe_ready = prepared_pass_probe_ready ||
                                           committed_retry_pass_ready ||
                                           recovery_nonmotion_pass_probe_ready;
  bool start_grid_pass_ready_for_warmup = start_grid_pass_probe_ready;
  std::string tracking_probe_reset_reason{};
  blocked.maneuver_transaction_tracking_probe_required_cycles =
      std::max(2, config_.start_grid_tracking_probe_required_cycles);

  if (!start_grid_tracking_release_pending_) {
    const auto verified_follow = std::find_if(
        candidates.cbegin(), candidates.cend(), [](const auto &candidate) {
          return candidate.type == CandidateType::FOLLOW &&
                 candidate.safety_evaluated && candidate.feasible &&
                 candidate.longitudinal_profile_valid &&
                 candidate.controller_tracking_profile_valid;
        });
    const bool verified_follow_available = verified_follow != candidates.cend();
    const bool recognized_normal_transport_gap =
        reentry_input.start_grid_pass_probe_transport_evidence ==
        StartGridProbeTransportEvidence::NORMAL_DELIVERY_GAP;
    const double paused_probe_target_d_m =
        start_grid_tracking_probe_pass_type_ == CandidateType::PASS_LEFT
            ? blocked.pass_left_candidate_target_d_m
        : start_grid_tracking_probe_pass_type_ == CandidateType::PASS_RIGHT
            ? blocked.pass_right_candidate_target_d_m
            : std::numeric_limits<double>::quiet_NaN();
    const bool paused_probe_identity_matches =
        recognized_normal_transport_gap &&
        start_grid_tracking_probe_cycles_ > 0 &&
        start_grid_tracking_probe_candidate_valid_ &&
        !start_grid_tracking_probe_target_id_.empty() &&
        start_grid_tracking_probe_target_id_ == blocked.maneuver_target_id &&
        start_grid_tracking_probe_pass_type_ ==
            blocked.maneuver_transaction_pass_type &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        all_remaining_chain_waypoints_observed_fresh &&
        exception_base_inputs_complete && prediction_complete &&
        std::isfinite(start_grid_tracking_probe_target_d_m_) &&
        std::isfinite(paused_probe_target_d_m) &&
        static_cast<float>(start_grid_tracking_probe_target_d_m_) ==
            static_cast<float>(paused_probe_target_d_m);
    bool pause_existing_probe = false;
    if (paused_probe_identity_matches) {
      CandidateTrajectory previous_probe_candidate =
          start_grid_tracking_probe_candidate_;
      const bool rebased =
          rebaseCandidateToCurrentEgo(previous_probe_candidate, ego, frame_);
      const bool currently_trackable =
          rebased &&
          CandidateBuilder(frame_, config_)
              .publishedLateralProfileTrackable(previous_probe_candidate, ego);
      previous_probe_candidate.controller_tracking_profile_valid =
          currently_trackable;
      if (currently_trackable) {
        evaluateCandidateAtOwnTimeAxis(previous_probe_candidate, opponents,
                                       now_sec, predictions);
      }
      pause_existing_probe =
          currently_trackable && previous_probe_candidate.safety_evaluated &&
          previous_probe_candidate.feasible &&
          previous_probe_candidate.longitudinal_profile_valid &&
          previous_probe_candidate.controller_tracking_profile_valid &&
          is_transaction_pass(previous_probe_candidate.type);
    }
    constexpr double kFailedWarmupYawIdentityToleranceRad = 0.02;
    const bool failed_candidate_yaw_matches =
        std::isfinite(start_grid_tracking_failed_ego_yaw_rad_) &&
        std::isfinite(ego.yaw) &&
        std::abs(std::remainder(
            ego.yaw - start_grid_tracking_failed_ego_yaw_rad_,
            2.0 * std::acos(-1.0))) <= kFailedWarmupYawIdentityToleranceRad;
    const bool failed_candidate_identity_matches =
        start_grid_tracking_failed_candidate_valid_ &&
        start_grid_tracking_failed_target_id_ == blocked.maneuver_target_id &&
        start_grid_tracking_failed_pass_type_ == selected.type &&
        failed_candidate_yaw_matches;
    const bool matches_failed_warmup_wire =
        start_grid_pass_probe_ready && failed_candidate_identity_matches &&
        releaseCandidateWireSemanticallyEqual(
            start_grid_tracking_failed_candidate_, selected);
    if (start_grid_tracking_failed_candidate_valid_ &&
        !matches_failed_warmup_wire && start_grid_pass_probe_ready) {
      // target/sideまたは実wireが変化した時だけ、失敗snapshotの抑止を解除して
      // fresh観測probeから再開する。候補評価そのものは抑止中も毎周期実行済み。
      reset_start_grid_tracking_failed_candidate();
    }

    if (!start_grid_pass_probe_ready && pause_existing_probe) {
      // freshかつ高々1世代のplan/constraint配送順差では、既に全評価を通った
      // PASS probe snapshotを再評価してcountだけ凍結する。gap周期は未認可PASSを
      // 選ばず、warm-up/token/正速度も発行しない。exact current tupleが戻った
      // fresh target観測周期だけ、下の従来経路でprobeを再開する。
      start_grid_pass_ready_for_warmup = false;
      tracking_probe_reset_reason = "probe_paused_normal_transport_gap";
      if (verified_follow_available) {
        selected = *verified_follow;
      }
    } else if (!start_grid_pass_probe_ready) {
      reset_start_grid_tracking_probe();
      tracking_probe_reset_reason = "pass_not_ready";
    } else if (matches_failed_warmup_wire && verified_follow_available) {
      reset_start_grid_tracking_probe();
      selected = *verified_follow;
      start_grid_pass_ready_for_warmup = false;
      tracking_probe_reset_reason = "probe_matches_failed_warmup_wire";
    } else {
      // SafetyEvaluator・全相手包含・予測・区間permission・曲率gate・
      // PP空間trackabilityを通った時点で、走行権限を持たないtyped
      // PASS_WARMUP transactionへ直ちに進める。旧実装のtarget観測2周期は
      // transport/制御proofではないため廃止し、以後の2周期はMuxが返す
      // exact plan/constraint/command sampleだけを数える。
      reset_start_grid_tracking_probe();
      tracking_probe_reset_reason = "nonmotion_probe_transaction_ready";
    }
  }
  blocked.maneuver_transaction_tracking_probe_cycles =
      pass_probe_exact_ack_cycles_;
  blocked.maneuver_transaction_tracking_probe_reset_reason =
      tracking_probe_reset_reason;

  if (start_grid_pass_ready_for_warmup &&
      (!start_grid_tracking_release_pending_ ||
       start_grid_tracking_release_target_id_ != blocked.maneuver_target_id)) {
    // 初回Gate 2承認を、実行済みtransactionへ早まって昇格させない。同じ
    // target/side/profileのPASSを停止constraint下のPREPAREとして固定し、
    // final PPがこのgenerationを追従可能と返した後だけOVERTAKEへ進める。
    reset_start_grid_tracking_release();
    start_grid_tracking_release_pending_ = true;
    start_grid_tracking_release_cycles_ = 1;
    start_grid_tracking_release_target_id_ = blocked.maneuver_target_id;
    start_grid_tracking_release_candidate_ = selected;
    start_grid_tracking_release_candidate_valid_ = true;
    advance_start_grid_tracking_release_token();
    tracking_release_candidate_installed_this_cycle = true;
    blocked.maneuver_transaction_tracking_release_pending = true;
    blocked.maneuver_transaction_tracking_release_confirmed = false;
    blocked.maneuver_transaction_tracking_release_cycles = 1;
    blocked.maneuver_transaction_tracking_probe_cycles = 0;
    blocked.maneuver_transaction_tracking_probe_reset_reason = "warmup_started";
    reset_start_grid_tracking_probe();
    reset_start_grid_tracking_failed_candidate();
  }
  const bool tracking_release_inputs_complete =
      exception_base_inputs_complete && prediction_complete;
  const bool prepared_release_authority_still_valid =
      tracking_release_inputs_complete &&
      (!blocked.maneuver_transaction_prepared ||
       (blocked.start_grid_target_active &&
        blocked.start_grid_target_confirmed_stationary &&
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
        all_remaining_chain_waypoints_observed_fresh &&
        !blocked.maneuver_target_id.empty() &&
        blocked.maneuver_target_id == blocked.start_grid_target_id &&
        !blocked.maneuver_chain_tail_id.empty() &&
        blocked.straight_overtake_start_allowed &&
        (blocked.overtake_permission_allowed ||
         blocked.permission_start_exception_active)));
  if (blocked.maneuver_transaction_tracking_release_pending) {
    if (!blocked.maneuver_transaction_tracking_release_confirmed) {
      // pending中は候補の有無に依存せず縦STOPを既定にする。保存PASSが無効で
      // raw候補も不成立な周期に、この代入を候補分岐だけへ置くとSTOPが抜ける。
      blocked.maneuver_transaction_tracking_stop_active = true;
    }
    if (!tracking_release_inputs_complete) {
      // committed retryでも、全観測包含・freshness・予測完全性はSTOP下の
      // PASS操舵を認可するための毎周期契約である。欠損中はtarget/sideと保存
      // profileを保持する一方、proofの連続性を切り、縦STOPだけを残す。
      start_grid_tracking_release_confirmed_ = false;
      start_grid_tracking_release_cycles_ = 0;
      blocked.maneuver_transaction_tracking_release_confirmed = false;
      blocked.maneuver_transaction_tracking_release_cycles = 0;
      blocked.maneuver_transaction_tracking_stop_active = true;
      for (auto &candidate : candidates) {
        if (candidate.type == CandidateType::PASS_LEFT ||
            candidate.type == CandidateType::PASS_RIGHT) {
          candidate.feasible = false;
          candidate.reject_reason = "start_grid_release_inputs_incomplete";
        }
      }
      blocked.pass_left_candidate_feasible = false;
      blocked.pass_right_candidate_feasible = false;
      blocked.maneuver_target_pass_candidate_feasible = false;
      blocked.maneuver_target_pass_reject_reason =
          "start_grid_release_inputs_incomplete";
      if (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT) {
        blocked.pass_left_candidate_reject_reason =
            "start_grid_release_inputs_incomplete";
      } else if (blocked.maneuver_transaction_pass_type ==
                 CandidateType::PASS_RIGHT) {
        blocked.pass_right_candidate_reject_reason =
            "start_grid_release_inputs_incomplete";
      }
      selected = selectCandidate(candidates);
    } else if (blocked.maneuver_transaction_prepared &&
               !prepared_release_authority_still_valid) {
      // ACK待ちのPREPAREはまだ実行開始前である。区間permission・曲率gate・
      // 停止対象例外のいずれかが失効したら、以前のPASS payloadやACKを開始権限に
      // 使わず、同じtarget/sideを保持したSTOP/FOLLOWから再認可を待つ。
      start_grid_tracking_release_candidate_valid_ = false;
      start_grid_tracking_release_candidate_ = CandidateTrajectory{};
      start_grid_tracking_release_token_ = 0U;
      start_grid_tracking_release_confirmed_ = false;
      start_grid_tracking_release_cycles_ = 0;
      blocked.maneuver_transaction_tracking_release_confirmed = false;
      blocked.maneuver_transaction_tracking_release_cycles = 0;
      blocked.maneuver_transaction_tracking_stop_active = true;
      for (auto &candidate : candidates) {
        if (candidate.type == CandidateType::PASS_LEFT ||
            candidate.type == CandidateType::PASS_RIGHT) {
          candidate.feasible = false;
          candidate.reject_reason = "start_grid_release_authority_revoked";
        }
      }
      blocked.pass_left_candidate_feasible = false;
      blocked.pass_right_candidate_feasible = false;
      blocked.maneuver_target_pass_candidate_feasible = false;
      blocked.maneuver_target_pass_reject_reason =
          "start_grid_release_authority_revoked";
      if (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT) {
        blocked.pass_left_candidate_reject_reason =
            "start_grid_release_authority_revoked";
      } else if (blocked.maneuver_transaction_pass_type ==
                 CandidateType::PASS_RIGHT) {
        blocked.pass_right_candidate_reject_reason =
            "start_grid_release_authority_revoked";
      }
      selected = selectCandidate(candidates);
    } else if (start_grid_tracking_release_candidate_valid_ &&
               prepared_release_authority_still_valid &&
               !tracking_release_candidate_installed_this_cycle) {
      // relative d/ds payloadは固定する一方、絶対s/x/yはNodeのtyped
      // trajectoryと 同じ current ego.s + ds
      // へ張り直す。現在yaw/dから実PP追従性を再検査し、
      // 現在時刻の全相手予測へSafetyEvaluatorを通したsnapshotだけを継続する。
      CandidateTrajectory current_release_candidate =
          start_grid_tracking_release_candidate_;
      const bool rebased =
          rebaseCandidateToCurrentEgo(current_release_candidate, ego, frame_);
      const bool longitudinal_start_matches =
          candidateStartsFromCurrentEgoSpeed(current_release_candidate, ego);
      const bool currently_trackable =
          rebased && longitudinal_start_matches &&
          CandidateBuilder(frame_, config_)
              .publishedLateralProfileTrackable(current_release_candidate, ego);
      current_release_candidate.controller_tracking_profile_valid =
          currently_trackable;
      if (!currently_trackable) {
        current_release_candidate.pure_pursuit_command_trackable = false;
        current_release_candidate.feasible = false;
        current_release_candidate.reject_reason =
            !rebased ? "start_grid_current_ego_rebase_invalid"
            : !longitudinal_start_matches
                ? "start_grid_current_ego_speed_changed"
                : "start_grid_current_ego_untrackable";
      } else {
        evaluateCandidateAtOwnTimeAxis(current_release_candidate, opponents,
                                       now_sec, predictions);
      }
      if (current_release_candidate.safety_evaluated &&
          current_release_candidate.feasible &&
          current_release_candidate.longitudinal_profile_valid &&
          current_release_candidate.controller_tracking_profile_valid &&
          is_transaction_pass(current_release_candidate.type)) {
        // world座標側だけは現周期へ更新する。relative wireは変えず、同じfresh
        // ControllerTrackingStatusを待てるようtoken/generationを維持する。
        start_grid_tracking_release_candidate_ = current_release_candidate;
        selected = std::move(current_release_candidate);
      } else {
        tracking_release_candidate_failure_reason =
            current_release_candidate.reject_reason.empty()
                ? "current_candidate_invalid"
                : current_release_candidate.reject_reason;
        // 現在yawだけを理由にpublish済みwarm-up wireが追従不能になった場合、
        // yaw復旧後に同じrelative wireをfresh評価し直せるよう保存する。過去の
        // SafetyEvaluator結果は権限に使わず、再利用時にrebase・trackability・
        // 全車予測を必ず再計算する。
        start_grid_tracking_failed_candidate_valid_ = true;
        start_grid_tracking_failed_target_id_ = blocked.maneuver_target_id;
        start_grid_tracking_failed_pass_type_ =
            blocked.maneuver_transaction_pass_type;
        start_grid_tracking_failed_candidate_ =
            start_grid_tracking_release_candidate_;
        start_grid_tracking_failed_ego_yaw_rad_ = ego.yaw;
        // 固定snapshotが現在egoでは使えない時だけ同じtarget/side/profileを
        // 再生成する。再生成が成功した場合はwireが偶然同じでもtokenを進め、
        // 旧ACKを無効化して停止constraint下でもう一度warm-upする。
        const CandidatePurpose regenerated_purpose =
            (initial_start_grid_pass_prediction_bootstrap ||
             pass_release_warmup_prediction_authorized ||
             committed_spatial_prediction_bootstrap_authorized ||
             (blocked.start_grid_target_active &&
              blocked.pass_lateral_first_speed_gate_active &&
              blocked.pass_proposal_acceleration_allowed))
                ? CandidatePurpose::PROPOSAL_SAFETY_EVALUATION
                : CandidatePurpose::EXECUTION;
        CandidateTrajectory regenerated =
            CandidateBuilder(frame_, config_)
                .makeCandidate(blocked.maneuver_transaction_pass_type, ego,
                               blocked, opponents,
                               localized_lateral_profile_.active
                                   ? &localized_lateral_profile_
                                   : nullptr,
                               false, regenerated_purpose);
        const bool regenerated_rebased =
            rebaseCandidateToCurrentEgo(regenerated, ego, frame_);
        const bool regenerated_trackable =
            regenerated_rebased &&
            CandidateBuilder(frame_, config_)
                .publishedLateralProfileTrackable(regenerated, ego);
        regenerated.controller_tracking_profile_valid = regenerated_trackable;
        if (!regenerated_trackable) {
          regenerated.pure_pursuit_command_trackable = false;
          regenerated.feasible = false;
          regenerated.reject_reason =
              regenerated_rebased ? "start_grid_regenerated_untrackable"
                                  : "start_grid_regenerated_rebase_invalid";
        } else {
          evaluateCandidateAtOwnTimeAxis(regenerated, opponents, now_sec,
                                         predictions);
        }
        const bool regenerated_valid =
            regenerated.safety_evaluated && regenerated.feasible &&
            regenerated.longitudinal_profile_valid &&
            regenerated.controller_tracking_profile_valid &&
            is_transaction_pass(regenerated.type);
        if (regenerated_valid) {
          start_grid_tracking_release_candidate_ = regenerated;
          start_grid_tracking_release_candidate_valid_ = true;
          advance_start_grid_tracking_release_token();
          tracking_release_candidate_installed_this_cycle = true;
          start_grid_tracking_release_confirmed_ = false;
          start_grid_tracking_release_cycles_ = 1;
          blocked.maneuver_transaction_tracking_release_confirmed = false;
          blocked.maneuver_transaction_tracking_release_cycles = 1;
          selected = std::move(regenerated);
        } else {
          tracking_release_candidate_failure_reason =
              regenerated.reject_reason.empty()
                  ? "regenerated_candidate_invalid"
                  : regenerated.reject_reason;
          const CandidateTrajectory fallback = selectCandidate(candidates);
          const bool safe_prepared_follow =
              blocked.maneuver_transaction_prepared &&
              !blocked.maneuver_transaction_incomplete &&
              holds_current_lateral(fallback);
          const bool safe_transaction_follow =
              fallback.type == CandidateType::FOLLOW &&
              (blocked.attack_follow_hold_pass_side || safe_prepared_follow) &&
              fallback.safety_evaluated && fallback.feasible &&
              fallback.longitudinal_profile_valid &&
              fallback.controller_tracking_profile_valid;
          if (safe_transaction_follow) {
            // PASSが現在poseでは追従不能でも、安全評価済みFOLLOWまで60周期の
            // STOP timeoutへ巻き込まない。PASS
            // transactionのtarget/side/profileは
            // 保持し、warm-upだけ破棄して新しいFOLLOW
            // generationを下流で照合する。
            if (start_grid_tracking_release_candidate_valid_) {
              start_grid_tracking_failed_candidate_valid_ = true;
              start_grid_tracking_failed_target_id_ =
                  blocked.maneuver_target_id;
              start_grid_tracking_failed_pass_type_ =
                  blocked.maneuver_transaction_pass_type;
              start_grid_tracking_failed_candidate_ =
                  start_grid_tracking_release_candidate_;
              start_grid_tracking_failed_ego_yaw_rad_ = ego.yaw;
            }
            reset_start_grid_tracking_release();
            reset_start_grid_tracking_probe();
            blocked.maneuver_transaction_tracking_release_pending = false;
            blocked.maneuver_transaction_tracking_release_confirmed = false;
            blocked.maneuver_transaction_tracking_release_cycles = 0;
            blocked.maneuver_transaction_tracking_stop_active = false;
            selected = fallback;
          } else {
            // FOLLOWも安全評価/追従性を満たさない時だけ、target/side
            // transactionを 保持したcurrent-d
            // STOPへ閉じる。ABORTや別IDへは切り替えない。
            start_grid_tracking_release_candidate_valid_ = false;
            start_grid_tracking_release_candidate_ = CandidateTrajectory{};
            start_grid_tracking_release_token_ = 0U;
            start_grid_tracking_release_confirmed_ = false;
            blocked.maneuver_transaction_tracking_release_confirmed = false;
            blocked.maneuver_transaction_tracking_stop_active = true;
            for (auto &candidate : candidates) {
              if (candidate.type == CandidateType::PASS_LEFT ||
                  candidate.type == CandidateType::PASS_RIGHT) {
                candidate.feasible = false;
                candidate.reject_reason = "start_grid_tracking_unhealthy";
              }
            }
            blocked.pass_left_candidate_feasible = false;
            blocked.pass_right_candidate_feasible = false;
            if (blocked.maneuver_transaction_pass_type ==
                CandidateType::PASS_LEFT) {
              blocked.pass_left_candidate_reject_reason =
                  "start_grid_tracking_unhealthy";
            } else if (blocked.maneuver_transaction_pass_type ==
                       CandidateType::PASS_RIGHT) {
              blocked.pass_right_candidate_reject_reason =
                  "start_grid_tracking_unhealthy";
            }
            blocked.maneuver_target_pass_candidate_feasible = false;
            blocked.maneuver_target_pass_reject_reason =
                "start_grid_tracking_unhealthy";
            selected = fallback;
          }
        }
      }
    } else if (prepared_release_authority_still_valid &&
               !start_grid_tracking_release_candidate_valid_) {
      // execution hold中は未認可PASSをselectCandidateが選ばないため、selected
      // だけを見るとcurrent-d SAFE_STOPのまま再開不能になる。候補集合から
      // 同じtarget/sideのfresh SafetyEvaluator済みPASSだけを取り出し、新しい
      // tokenのwarm-upとしてinstallする。旧wire/旧ACKは再利用しない。
      std::optional<CandidateTrajectory> regenerated;
      constexpr double kFailedWarmupYawRetryToleranceRad = 0.02;
      const bool failed_candidate_yaw_changed =
          std::isfinite(start_grid_tracking_failed_ego_yaw_rad_) &&
          std::isfinite(ego.yaw) &&
          std::abs(std::remainder(
              ego.yaw - start_grid_tracking_failed_ego_yaw_rad_,
              2.0 * std::acos(-1.0))) > kFailedWarmupYawRetryToleranceRad;
      if (start_grid_tracking_failed_candidate_valid_ &&
          start_grid_tracking_failed_target_id_ == blocked.maneuver_target_id &&
          start_grid_tracking_failed_pass_type_ ==
              blocked.maneuver_transaction_pass_type &&
          failed_candidate_yaw_changed) {
        CandidateTrajectory retried = start_grid_tracking_failed_candidate_;
        const bool retried_rebased =
            rebaseCandidateToCurrentEgo(retried, ego, frame_);
        const bool retried_trackable =
            retried_rebased &&
            candidateStartsFromCurrentEgoSpeed(retried, ego) &&
            CandidateBuilder(frame_, config_)
                .publishedLateralProfileTrackable(retried, ego);
        retried.controller_tracking_profile_valid = retried_trackable;
        if (retried_trackable) {
          evaluateCandidateAtOwnTimeAxis(retried, opponents, now_sec,
                                         predictions);
        }
        if (retried.safety_evaluated && retried.feasible &&
            retried.longitudinal_profile_valid &&
            retried.controller_tracking_profile_valid &&
            is_transaction_pass(retried.type)) {
          regenerated = std::move(retried);
        }
      }
      if (!regenerated.has_value()) {
        const auto fresh = std::find_if(
            candidates.cbegin(), candidates.cend(),
            [&blocked, &is_transaction_pass](const auto &candidate) {
              return is_transaction_pass(candidate.type) &&
                     candidate.type == blocked.maneuver_transaction_pass_type &&
                     candidate.safety_evaluated && candidate.feasible &&
                     candidate.longitudinal_profile_valid &&
                     candidate.controller_tracking_profile_valid;
            });
        if (fresh != candidates.cend()) {
          regenerated = *fresh;
        }
      }
      if (regenerated.has_value()) {
        start_grid_tracking_release_candidate_ = *regenerated;
        start_grid_tracking_release_candidate_valid_ = true;
        advance_start_grid_tracking_release_token();
        tracking_release_candidate_installed_this_cycle = true;
        start_grid_tracking_release_confirmed_ = false;
        start_grid_tracking_release_cycles_ = 1;
        blocked.maneuver_transaction_tracking_release_confirmed = false;
        blocked.maneuver_transaction_tracking_release_cycles = 1;
        selected = *regenerated;
        tracking_probe_reset_reason = "release_candidate_regenerated";
        reset_start_grid_tracking_failed_candidate();
      } else {
        tracking_probe_reset_reason = "release_candidate_unavailable";
      }
    } else if (prepared_release_authority_still_valid &&
               selected.safety_evaluated && selected.feasible &&
               selected.longitudinal_profile_valid &&
               selected.controller_tracking_profile_valid &&
               is_transaction_pass(selected.type)) {
      start_grid_tracking_release_candidate_ = selected;
      start_grid_tracking_release_candidate_valid_ = true;
      advance_start_grid_tracking_release_token();
      tracking_release_candidate_installed_this_cycle = true;
      start_grid_tracking_release_confirmed_ = false;
      blocked.maneuver_transaction_tracking_release_confirmed = false;
    }
  }
  const bool committed_pass_rejected_for_start_grid_tracking =
      blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT
          ? blocked.pass_left_candidate_reject_reason ==
                "start_grid_tracking_unhealthy"
      : blocked.maneuver_transaction_pass_type == CandidateType::PASS_RIGHT
          ? blocked.pass_right_candidate_reject_reason ==
                "start_grid_tracking_unhealthy"
          : false;
  const bool tracking_release_has_no_usable_pass =
      blocked.maneuver_transaction_tracking_release_pending &&
      (!start_grid_tracking_release_candidate_valid_ ||
       selected.type != blocked.maneuver_transaction_pass_type ||
       !selected.safety_evaluated || !selected.feasible ||
       !selected.longitudinal_profile_valid ||
       !selected.controller_tracking_profile_valid);
  const bool selected_is_start_grid_tracking_release_pass =
      blocked.maneuver_transaction_tracking_release_pending &&
      (!blocked.maneuver_transaction_prepared ||
       prepared_release_authority_still_valid) &&
      (blocked.maneuver_transaction_prepared ||
       (blocked.maneuver_transaction_retry_active &&
        blocked.maneuver_transaction_incomplete)) &&
      selected.type == blocked.maneuver_transaction_pass_type &&
      (selected.type == CandidateType::PASS_LEFT ||
       selected.type == CandidateType::PASS_RIGHT) &&
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
      blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      exception_base_inputs_complete && prediction_complete;
  const bool verified_prepared_tracking_follow =
      selected.type == CandidateType::FOLLOW &&
      blocked.maneuver_transaction_prepared &&
      !blocked.maneuver_transaction_incomplete &&
      holds_current_lateral(selected);
  const bool verified_committed_attack_follow =
      selected.type == CandidateType::FOLLOW &&
      blocked.attack_follow_hold_pass_side &&
      blocked.attack_follow_candidate_feasible;
  const bool verified_transaction_follow_fallback =
      (verified_prepared_tracking_follow || verified_committed_attack_follow) &&
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
      blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      exception_base_inputs_complete && prediction_complete;
  const bool verified_authorized_current_d_hold =
      selected.type == CandidateType::RECOVERY &&
      blocked.authorized_pass_current_d_hold_active &&
      holds_current_lateral(selected) && selected.safety_evaluated &&
      selected.feasible && selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      exception_base_inputs_complete && prediction_complete;
  const bool selected_transaction_pass_usable =
      blocked.maneuver_transaction_incomplete &&
      is_transaction_pass(selected.type) && selected.safety_evaluated &&
      selected.feasible && selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
      blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      exception_base_inputs_complete && prediction_complete &&
      (!maneuver_execution_hold_identity_matches() ||
       (blocked.maneuver_transaction_tracking_release_pending &&
        blocked.maneuver_transaction_tracking_release_confirmed));
  if (selected_is_start_grid_tracking_release_pass &&
      !blocked.maneuver_transaction_tracking_release_confirmed) {
    // 初回PREPAREまたは再開warm-upでは、PASS候補を現在予測へ再評価した上で
    // trajectoryだけ先にpublishする。縦stopは同generation proofまで保持し、
    // target/side/dは変更しない。
    blocked.maneuver_transaction_tracking_stop_active = true;
  }
  const bool needs_prepared_tracking_safe_stop_hold =
      blocked.start_grid_target_active &&
      blocked.maneuver_transaction_prepared &&
      (isAnyPassMode(mode_) || mode_ == BehaviorMode::FOLLOW_BLOCKED) &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      (committed_pass_rejected_for_start_grid_tracking ||
       tracking_release_has_no_usable_pass) &&
      !verified_transaction_follow_fallback && exception_base_inputs_complete &&
      prediction_complete;
  const bool incomplete_pass_identity_latched =
      blocked.maneuver_transaction_incomplete &&
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      blocked.maneuver_target_latched && !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_target_id == localized_lateral_profile_.target_id &&
      blocked.maneuver_transaction_pass_type ==
          localized_lateral_profile_.pass_type &&
      (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT ||
       blocked.maneuver_transaction_pass_type == CandidateType::PASS_RIGHT);
  const bool explicit_transaction_centering_authorized =
      blocked.pass_reauthorization_lockout_active ||
      blocked.reentry_centering_authorized;
  if (explicit_transaction_centering_authorized &&
      maneuver_execution_hold_active_) {
    maneuver_execution_hold_active_ = false;
    maneuver_execution_hold_target_id_.clear();
    maneuver_execution_hold_pass_type_ = CandidateType::FASTEST;
    reset_start_grid_tracking_release();
    reset_start_grid_tracking_probe();
    reset_start_grid_tracking_failed_candidate();
  }
  const bool needs_incomplete_pass_execution_hold =
      incomplete_pass_identity_latched &&
      !explicit_transaction_centering_authorized &&
      !selected_transaction_pass_usable &&
      !selected_is_start_grid_tracking_release_pass &&
      !verified_transaction_follow_fallback &&
      !verified_authorized_current_d_hold;
  const bool needs_maneuver_safe_stop_hold =
      needs_prepared_tracking_safe_stop_hold ||
      needs_incomplete_pass_execution_hold;
  bool maneuver_execution_speed_only_stop_required = false;
  if (needs_maneuver_safe_stop_hold) {
    // 未完了PASSでは、通常のYIELD/中心向きRECOVERYへ横軌道ownerを渡さない。
    // 同じtarget/sideのPASSまたは認可済みcurrent-d
    // FOLLOW/HOLDが成立しない周期は、 target ID・PASS side・staged
    // waypointを保持したまま現在d SAFE_STOPへ閉じる。 current-d
    // STOPもfreshな全相手・残りwaypoint・trackabilityを満たさない時は、
    // 横列をpublishせずmuxのspeed-only最大制動へ委譲する。
    BlockedInfo stop_blocked = blocked;
    stop_blocked.abort_safe_stop_hold_lateral = true;
    CandidateTrajectory exact_current_d_stop =
        makeCandidate(CandidateType::SAFE_STOP, ego, stop_blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(exact_current_d_stop, opponents, now_sec,
                                   predictions);
    if (!exact_current_d_stop.longitudinal_profile_valid) {
      exact_current_d_stop.feasible = false;
      exact_current_d_stop.reject_reason = "invalid_longitudinal_brake_model";
    }
    const bool hold_inputs_complete =
        blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
        blocked.maneuver_chain_tail_observed &&
        all_remaining_chain_waypoints_observed_fresh &&
        exception_base_inputs_complete && prediction_complete;
    const bool exact_current_d_stop_valid =
        hold_inputs_complete && exact_current_d_stop.safety_evaluated &&
        exact_current_d_stop.feasible &&
        exact_current_d_stop.longitudinal_profile_valid &&
        exact_current_d_stop.controller_tracking_profile_valid &&
        holds_current_lateral(exact_current_d_stop);
    if (needs_incomplete_pass_execution_hold &&
        !maneuver_execution_hold_identity_matches()) {
      // STOPへ入った事実をtransaction identityへ束縛する。次周期に同じ
      // PASSが再び成立しても、このラッチが残る間は新generation/tokenの
      // warm-upとexact ControllerTrackingStatus ACKなしに再開させない。
      maneuver_execution_hold_active_ = true;
      maneuver_execution_hold_target_id_ = blocked.maneuver_target_id;
      maneuver_execution_hold_pass_type_ =
          blocked.maneuver_transaction_pass_type;
      const bool preserve_failed_warmup_for_same_pending =
          blocked.maneuver_transaction_tracking_release_pending &&
          start_grid_tracking_release_target_id_ == blocked.maneuver_target_id;
      if (preserve_failed_warmup_for_same_pending) {
        // 既にwarm-up中だった同じtarget/sideだけは再開identityを保持する。
        // 実wire・token・confirmedは必ず破棄し、次周期のfresh候補をtrackability
        // と全車SafetyEvaluatorへ通してから新tokenのexact ACKを要求する。
        start_grid_tracking_release_pending_ = true;
        start_grid_tracking_release_confirmed_ = false;
        start_grid_tracking_release_cycles_ = 0;
        start_grid_tracking_release_candidate_valid_ = false;
        start_grid_tracking_release_candidate_ = CandidateTrajectory{};
        start_grid_tracking_release_token_ = 0U;
      } else {
        // 通常PASS中の入力欠損・大横誤差などはfresh形状への再anchor権限に
        // 変換しない。既存どおりreleaseを破棄して固定geometry側でfail-closedに
        // 判定する。
        reset_start_grid_tracking_release();
      }
      reset_start_grid_tracking_probe();
      if (!preserve_failed_warmup_for_same_pending) {
        reset_start_grid_tracking_failed_candidate();
      }
    }
    blocked.maneuver_transaction_tracking_stop_active = true;
    if (exact_current_d_stop_valid) {
      blocked.abort_safe_stop_hold_lateral = true;
    } else {
      exact_current_d_stop.feasible = false;
      if (!hold_inputs_complete) {
        exact_current_d_stop.reject_reason =
            "incomplete_pass_hold_inputs_incomplete";
      } else if (exact_current_d_stop.reject_reason.empty()) {
        exact_current_d_stop.reject_reason =
            "incomplete_pass_current_d_stop_infeasible";
      }
      maneuver_execution_speed_only_stop_required = true;
    }
    selected = std::move(exact_current_d_stop);
  }
  const bool selected_holds_current_lateral = holds_current_lateral(selected);
  const bool verified_transaction_hold_common =
      selected.safety_evaluated && selected.feasible &&
      selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      selected_holds_current_lateral &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      exception_base_inputs_complete && prediction_complete;
  const bool verified_yield_hold =
      selected.type == CandidateType::YIELD_BEHIND &&
      blocked.parallel_yield_hold_lateral &&
      (!blocked.braking_follow_active ||
       blocked.braking_follow_id == blocked.maneuver_chain_tail_id);
  const bool verified_recovery_hold = selected.type == CandidateType::RECOVERY;
  const bool verified_safe_stop_hold =
      selected.type == CandidateType::SAFE_STOP;
  // PASSが同周期のSafetyEvaluatorで不成立でも、別途評価した現在d保持が成立する
  // 場合だけ同じtransactionを保持する。tracking失効時は横移動を伴わない
  // SAFE_STOPであり、PASS reject軌道をpublishしたりtarget/side/dを
  // 張り替えたりはしない。
  blocked.maneuver_transaction_safe_lateral_hold_active =
      verified_transaction_hold_common &&
      (verified_yield_hold || verified_recovery_hold ||
       verified_safe_stop_hold);
  const bool generic_centering_recovery =
      config_.reentry_gate_enabled && !reentry_phase_active_ &&
      !reentry_lockout_active_ && !isPassMode(mode_) &&
      needsGenericRecoveryGate(config_, mode_, blocked) &&
      selected.type == CandidateType::RECOVERY && selected.feasible &&
      candidateMovesTowardCenter(ego, selected);
  if (!generic_recovery_phase_active_ && generic_centering_recovery) {
    // 状態名ではなく、実際にpublish候補が中心側へ動く時だけ長期gateを開始する。
    // これによりwide parallelや壁寄りでのgeneric RECOVERYも、4秒先の全車両
    // 予測を通るまで通常ラインへ横断しない。
    generic_recovery_phase_active_ = true;
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    reentry_gate =
        evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                            reentry_input, reentry_mpc_health);
    if (!reentry_gate.permitted) {
      blocked.reentry_hold_active = true;
      if (reentry_mpc_health == ReentryMpcHealthState::TRANSIENT_LATENCY) {
        blocked.reentry_hold_speed_cap_mps =
            config_.reentry_mpc_degraded_hold_v_max_mps;
      }
    }
  }
  const bool release_front_gap_ready =
      blocked.nearest_index < 0 ||
      blocked.front_delta_s >= config_.safe_stop_release_front_gap_m;
  safe_stop_context.release_ready =
      ego.valid &&
      blocked.ego_wall_clearance_m >=
          config_.safe_stop_release_wall_clearance_m &&
      !blocked.side_by_side && !blocked.future_yield_required &&
      release_front_gap_ready &&
      hasFeasibleReleaseCandidate(candidates, blocked) &&
      ego.v <= config_.safe_stop_release_speed_mps &&
      safe_stop_context.lateral_error_m <=
          config_.safe_stop_lateral_error_threshold_m;

  // 候補選択だけで急にモードを切り替えず、状態機械で保持時間や継続条件をかける。
  const BehaviorMode mode_before_transition = mode_;
  mode_ = state_machine_.update(now_sec, mode_, selected.type, blocked,
                                selected.feasible, safe_stop_context);
  const BehaviorMode state_machine_mode = mode_;
  const bool verified_early_wall_recovery =
      early_wall_recovery_probe_requested &&
      selected.type == CandidateType::RECOVERY && selected.safety_evaluated &&
      selected.feasible && selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      selected.controller_spatial_horizon_proof_valid;
  if (verified_early_wall_recovery && mode_ == BehaviorMode::FREE_RUN) {
    // StateMachineは通常RECOVERYをFREE_RUNのmode holdで上書きするため、
    // 同周期に完全評価済みの早期壁RECOVERYだけSPEED_GUARDへ束縛する。
    // 不成立候補、PASS、FOLLOW、古いRECOVERY latchには適用しない。
    mode_ = BehaviorMode::SPEED_GUARD;
  }
  if (blocked.maneuver_transaction_prepared &&
      prepared_release_authority_still_valid &&
      blocked.maneuver_transaction_tracking_release_pending &&
      !blocked.maneuver_transaction_tracking_release_confirmed &&
      blocked.maneuver_transaction_tracking_stop_active &&
      selected.type == blocked.maneuver_transaction_pass_type &&
      (selected.type == CandidateType::PASS_LEFT ||
       selected.type == CandidateType::PASS_RIGHT)) {
    // StateMachineの連続安全回数が満たされても、初期グリッドPASSは同じ
    // generationのPP proofが届くまで実行modeへ進めない。安全評価済みPASS
    // payloadはPREPAREのまま停止constraint下でpublishし、proof周期だけ
    // OVERTAKEへ昇格させる。
    mode_ = selected.type == CandidateType::PASS_LEFT
                ? BehaviorMode::PREPARE_OVERTAKE_LEFT
                : BehaviorMode::PREPARE_OVERTAKE_RIGHT;
  }
  blocked.pass_left_safe_cycles = state_machine_.passLeftSafeCycles();
  blocked.pass_right_safe_cycles = state_machine_.passRightSafeCycles();
  blocked.pass_safe_cycle_reset_reason =
      state_machine_.passSafeCycleResetReason();
  const bool aging_gate2_approved_profile_without_direct_classifier =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_safety_approved_once &&
      !localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !has_direct_same_target_pass_probe_context();
  if (aging_gate2_approved_profile_without_direct_classifier) {
    // tracking/stale/corridor/yieldでその周期のcontinuityが不成立でも、
    // 分類外の過去Gate 2 profileの寿命は消費する。bad cycleでageを
    // 0へ戻すとsafe/bad交互入力で永久に復活できるため、safe-cycleの
    // resetとprofile lifetimeを分離する。
    localized_lateral_profile_.pass_start_target_continuity_cycles = std::min(
        pass_start_target_continuity_budget_cycles,
        localized_lateral_profile_.pass_start_target_continuity_cycles + 1);
  } else if (localized_lateral_profile_.active) {
    localized_lateral_profile_.pass_start_target_continuity_cycles = 0;
  }
  blocked.pass_start_target_continuity_cycles =
      localized_lateral_profile_.active
          ? localized_lateral_profile_.pass_start_target_continuity_cycles
          : 0;
  if (!isPrepareOvertakeMode(mode_)) {
    stationary_parallel_permission_prepare_id_.clear();
  }
  if (isAnyPassMode(mode_) &&
      blocked.gentle_curve_safe_pass_constraint_active) {
    // PREPAREからOVERTAKEへ進んだ次周期も、通常PASSのd/vへ戻さない。
    gentle_curve_safe_pass_constraint_latched_ = true;
    if (!std::isfinite(gentle_curve_safe_pass_anchor_d_m_)) {
      gentle_curve_safe_pass_anchor_d_m_ = ego.frenet.d;
    }
    if (std::isfinite(blocked.gentle_curve_safe_pass_speed_cap_mps) &&
        blocked.gentle_curve_safe_pass_speed_cap_mps > 0.0) {
      gentle_curve_safe_pass_speed_cap_mps_ =
          std::isfinite(gentle_curve_safe_pass_speed_cap_mps_)
              ? std::min(gentle_curve_safe_pass_speed_cap_mps_,
                         blocked.gentle_curve_safe_pass_speed_cap_mps)
              : blocked.gentle_curve_safe_pass_speed_cap_mps;
    }
  } else if (!isAnyPassMode(mode_)) {
    gentle_curve_safe_pass_constraint_latched_ = false;
    gentle_curve_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    gentle_curve_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
  }
  if (isAnyPassMode(mode_) &&
      blocked.stationary_no_pass_safe_pass_constraint_active) {
    stationary_no_pass_safe_pass_constraint_latched_ = true;
    if (stationary_no_pass_safe_pass_target_id_.empty() &&
        !stationary_no_pass_authoritative_target_id.empty()) {
      stationary_no_pass_safe_pass_target_id_ =
          stationary_no_pass_authoritative_target_id;
    }
    if (!std::isfinite(stationary_no_pass_safe_pass_anchor_d_m_)) {
      stationary_no_pass_safe_pass_anchor_d_m_ = ego.frenet.d;
    }
    if (std::isfinite(blocked.stationary_no_pass_safe_pass_speed_cap_mps) &&
        blocked.stationary_no_pass_safe_pass_speed_cap_mps > 0.0) {
      stationary_no_pass_safe_pass_speed_cap_mps_ =
          std::isfinite(stationary_no_pass_safe_pass_speed_cap_mps_)
              ? std::min(stationary_no_pass_safe_pass_speed_cap_mps_,
                         blocked.stationary_no_pass_safe_pass_speed_cap_mps)
              : blocked.stationary_no_pass_safe_pass_speed_cap_mps;
    }
  } else if (!isAnyPassMode(mode_)) {
    stationary_no_pass_safe_pass_constraint_latched_ = false;
    stationary_no_pass_safe_pass_target_id_.clear();
    stationary_no_pass_safe_pass_anchor_d_m_ =
        std::numeric_limits<double>::quiet_NaN();
    stationary_no_pass_safe_pass_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
  }
  // PASS実行中は回避開始を復帰と誤認しない。一方この周期にMERGE/YIELD/RECOVERYへ
  // 遷移した場合は、中心方向の候補をpublishする前に同じgateを必ず通す。
  const bool reentry_phase_before_update = reentry_phase_active_;
  const bool reentry_lockout_before_update = reentry_lockout_active_;
  const bool generic_recovery_before_update = generic_recovery_phase_active_;
  updateReentryPhase(ego, mode_before_transition);
  const bool reentry_phase_after_update = reentry_phase_active_;
  const bool reentry_lockout_after_update = reentry_lockout_active_;
  const bool generic_recovery_after_update = generic_recovery_phase_active_;
  if (!reentryRequested(ego) &&
      (reentry_gate_was_permitted || reentry_completed_this_cycle)) {
    // 既に前周期から許可済みのgateで物理的中心到達を確認した後は、古い
    // 許可結果を持ち越さない。この周期で初めてclearになった診断は残す。
    // post-ABORT曲線holdは次の分岐で専用診断に置き換える。
    reentry_gate = ReentryGateResult{};
  }
  if (!reentry_gate.requested && reentryRequested(ego)) {
    reentry_gate =
        evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                            reentry_input, reentry_mpc_health);
    if (reentry_gate.requested && !reentry_gate.permitted) {
      if (!generic_recovery_phase_active_) {
        reentry_lockout_active_ = true;
        reentry_phase_active_ = true;
      }
      blocked.reentry_hold_active = true;
      if (reentry_mpc_health == ReentryMpcHealthState::TRANSIENT_LATENCY) {
        blocked.reentry_hold_speed_cap_mps =
            config_.reentry_mpc_degraded_hold_v_max_mps;
      }
    }
  }
  const bool permitted_reentry_convergence =
      !generic_recovery_phase_active_ && reentry_gate.requested &&
      reentry_gate.input_complete && reentry_gate.permitted &&
      std::isfinite(ego.frenet.d) &&
      std::abs(ego.frenet.d) > reentryCompletionLateralErrorM(config_);
  const bool parallel_follow_preempts_reentry =
      blocked.parallel_follow_candidate && blocked.parallel_follow_feasible &&
      mode_ == BehaviorMode::FOLLOW_BLOCKED && reentry_gate.requested &&
      !reentry_gate.permitted;
  const bool start_grid_follow_preempts_reentry =
      blocked.start_grid_target_active &&
      mode_ == BehaviorMode::FOLLOW_BLOCKED && reentry_gate.requested &&
      !reentry_gate.permitted && selected.feasible &&
      selected.type == CandidateType::FOLLOW;
  const bool braking_follow_preempts_reentry =
      blocked.braking_follow_active && mode_ == BehaviorMode::FOLLOW_BLOCKED &&
      reentry_gate.requested && !reentry_gate.permitted && selected.feasible &&
      (selected.type == CandidateType::FOLLOW ||
       (selected.type == CandidateType::RECOVERY &&
        blocked.braking_follow_hold_lateral));
  const auto prestart_attack_follow_candidate = std::find_if(
      candidates.begin(), candidates.end(),
      [&holds_current_lateral](const CandidateTrajectory &candidate) {
        return candidate.type == CandidateType::FOLLOW &&
               candidate.safety_evaluated && candidate.feasible &&
               candidate.longitudinal_profile_valid &&
               candidate.controller_tracking_profile_valid &&
               candidate.pass_target_corridor_valid &&
               holds_current_lateral(candidate);
      });
  const bool prestart_attack_follow_preempts_reentry =
      blocked.prestart_attack_follow_hold_lateral &&
      !blocked.maneuver_transaction_incomplete &&
      mode_ == BehaviorMode::FOLLOW_BLOCKED && generic_recovery_phase_active_ &&
      !reentry_phase_active_ && !reentry_lockout_active_ &&
      reentry_gate.requested &&
      prestart_attack_follow_candidate != candidates.end() &&
      exception_base_inputs_complete && prediction_complete;
  const bool committed_transaction_hold_preempts_reentry =
      mode_ == BehaviorMode::FOLLOW_BLOCKED &&
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty() && selected.safety_evaluated &&
      selected.feasible && selected.longitudinal_profile_valid &&
      selected.controller_tracking_profile_valid &&
      exception_base_inputs_complete && prediction_complete &&
      ((selected.type == CandidateType::FOLLOW &&
        !candidateMovesTowardCenter(ego, selected)) ||
       blocked.maneuver_transaction_safe_lateral_hold_active);
  const bool tracking_stop_preempts_reentry =
      (mode_ == BehaviorMode::FOLLOW_BLOCKED || isPrepareOvertakeMode(mode_)) &&
      (blocked.maneuver_transaction_prepared ||
       blocked.maneuver_transaction_incomplete) &&
      blocked.maneuver_transaction_tracking_stop_active &&
      blocked.maneuver_target_latched && !blocked.maneuver_target_id.empty();
  if (committed_transaction_hold_preempts_reentry ||
      tracking_stop_preempts_reentry) {
    // PASS不成立後に同一target/sideを保持したFOLLOW、現d hold、または
    // tracking待ちSAFE_STOPが独立にSafetyEvaluatorを通った場合、以前の
    // 中心復帰phaseでその候補を後からSPEED_GUARDへ上書きしない。ここで
    // 解除するのは復帰要求だけで、transactionとPASS profileは保持する。
    reentry_phase_active_ = false;
    reentry_lockout_active_ = false;
    reentry_gate_permitted_ = false;
    reentry_clear_cycles_ = 0;
    blocked.reentry_hold_active = false;
    blocked.reentry_centering_authorized = false;
    reentry_gate = ReentryGateResult{};
  } else if (start_grid_follow_preempts_reentry ||
             parallel_follow_preempts_reentry ||
             braking_follow_preempts_reentry ||
             prestart_attack_follow_preempts_reentry) {
    // parallel前方車に対する現d保持FOLLOWがSafetyEvaluatorを通る場合は、
    // 中心線へ戻るreentryが未許可でもABORT_RECOVERYへ強制しない。
    // reentry gateの診断は残し、FOLLOW候補が消えた周期は既存fail-safeへ戻る。
    if (generic_recovery_phase_active_) {
      generic_recovery_phase_active_ = false;
      generic_recovery_hold_offsets_.clear();
      generic_recovery_hold_speed_cap_mps_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    blocked.reentry_hold_active = false;
  } else if (generic_recovery_phase_active_ && reentry_gate.requested) {
    // generic補正はgate拒否時だけ現dをholdする。許可済みでも中心到達までは
    // SPEED_GUARDを維持し、phaseだけ残ったFREE_RUNを作らない。
    if (!reentry_gate.permitted) {
      blocked.reentry_hold_active = true;
    }
    mode_ = BehaviorMode::SPEED_GUARD;
  } else if (reentry_gate.requested &&
             (!reentry_gate.permitted || !std::isfinite(ego.frenet.d))) {
    // 状態機械がblocked/frontだけを見てFREE_RUNへ戻る経路を、復帰ゲートで閉じる。
    // gate拒否または無効な横位置では従来どおりABORT holdへ即座に戻す。
    mode_ = BehaviorMode::ABORT_RECOVERY;
  } else if (permitted_reentry_convergence) {
    // SafetyEvaluator済みの中心復帰はABORT holdから分離する。ただし通常候補へは
    // 戻さず、下の専用分岐でRECOVERYだけを再生成・再評価してpublishする。
    blocked.reentry_hold_active = false;
    blocked.reentry_centering_authorized = true;
    mode_ = BehaviorMode::SPEED_GUARD;
  }
  const BehaviorMode post_reentry_arbitration_mode = mode_;
  if (mode_ == BehaviorMode::ABORT_RECOVERY ||
      mode_ == BehaviorMode::SAFE_STOP) {
    // gate拒否・SafetyEvaluator拒否では、完了後holdを持ち越さない。
    post_abort_curve_hold_active_ = false;
    blocked.post_abort_curve_hold_active = false;
  }
  if (reentry_completed_this_cycle &&
      shouldHoldPostAbortCurve(config_, ego, blocked) &&
      mode_ != BehaviorMode::ABORT_RECOVERY) {
    // reentry gateと実際の中心収束を通過した後だけ、ABORTではなく専用の
    // SPEED_GUARDへ移す。PASSはこの周期にpublishせず、現d RECOVERYを
    // 改めてSafetyEvaluatorへ通す。
    post_abort_curve_hold_active_ = true;
    blocked.post_abort_curve_hold_active = true;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason = "post_abort_curve_hold";
    // phaseは安全に完了済みなので、下流診断へ「gate待ち」を残さない。
    // この後の現d RECOVERYは専用holdとして改めてSafetyEvaluatorを通す。
    reentry_gate.requested = false;
    reentry_gate.permitted = false;
    reentry_gate.clear_cycles = 0;
    reentry_gate.reason = "reentry_complete_post_abort_curve_hold";
    mode_ = BehaviorMode::SPEED_GUARD;
  }
  const auto make_generic_recovery_hold_candidate = [&]() {
    // stale時とpublish再評価失敗時に使うholdも、通常のRECOVERYと同じ予測・
    // longitudinal modelで評価する。d列はCandidateBuilderが安全コリドー内の
    // 現在横位置へ固定し、速度はreentry hold cap以下になる。
    BlockedInfo hold_blocked = blocked;
    hold_blocked.reentry_hold_active = true;
    CandidateTrajectory hold_candidate =
        makeCandidate(CandidateType::RECOVERY, ego, hold_blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(hold_candidate, opponents, now_sec,
                                   predictions);
    if (!hold_candidate.longitudinal_profile_valid) {
      hold_candidate.feasible = false;
      hold_candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    return hold_candidate;
  };
  const auto make_generic_safe_stop_candidate = [&]() {
    CandidateTrajectory stop_candidate =
        makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(stop_candidate, opponents, now_sec,
                                   predictions);
    if (!stop_candidate.longitudinal_profile_valid) {
      stop_candidate.feasible = false;
      stop_candidate.reject_reason = "invalid_longitudinal_brake_model";
    }
    return stop_candidate;
  };
  CandidateTrajectory generic_safe_stop_candidate;
  bool generic_safe_stop_available = false;
  bool generic_no_safe_hold = false;
  if (generic_recovery_phase_active_ && reentry_gate.requested &&
      reentry_gate.input_complete) {
    // gateが許可済みでも、次周期にegoがstaleになれば中心側軌道を継続できない。
    // そのため完全入力で安全評価した現d holdを毎周期更新しておく。
    const CandidateTrajectory generic_hold =
        make_generic_recovery_hold_candidate();
    if (generic_hold.feasible) {
      rememberGenericRecoveryHold(generic_hold, reentry_gate);
    } else {
      // current-d holdが不成立なら古いholdを続けない。gate拒否中は同じ予測で
      // SAFE_STOPを再評価し、成立しなければ横列を出さずwatchdogへ委譲する。
      // gate許可済みの中心復帰自体はSafetyEvaluatorを通っているため、この周期は
      // その候補を維持する。ただしstale用holdは保存せず、次周期に入力が欠けたら
      // 横列なしのspeed-only watchdogへ閉じる。
      generic_recovery_hold_offsets_.clear();
      generic_recovery_hold_speed_cap_mps_ =
          std::numeric_limits<double>::quiet_NaN();
      if (!reentry_gate.permitted) {
        blocked.reentry_hold_active = true;
        reentry_clear_cycles_ = 0;
        reentry_gate_permitted_ = false;
        reentry_gate.permitted = false;
        reentry_gate.clear_cycles = 0;
        reentry_gate.reason = "generic_recovery_hold_infeasible";
        generic_safe_stop_candidate = make_generic_safe_stop_candidate();
        generic_safe_stop_available = generic_safe_stop_candidate.feasible;
        if (generic_safe_stop_available) {
          // stale時に再利用可能なのは、holdではなく評価済みSAFE_STOP列だけ。
          rememberGenericRecoveryHold(generic_safe_stop_candidate,
                                      reentry_gate);
          mode_ = BehaviorMode::SAFE_STOP;
        } else {
          generic_no_safe_hold = true;
          mode_ = BehaviorMode::SPEED_GUARD;
        }
      }
    }
  }
  const bool yield_lateral_hold =
      config_.yield_release_lateral_error_m >= 0.0 &&
      std::abs(ego.frenet.d) > config_.yield_release_lateral_error_m;
  if (mode_ == BehaviorMode::YIELD_BEHIND &&
      state_machine_.futureYieldHoldActive()) {
    const bool corner_still_relevant =
        config_.corner_side_yield_curvature_m_inv > 0.0 &&
        blocked.corner_abs_curvature >=
            config_.corner_side_yield_curvature_m_inv;
    blocked.future_yield_required = true;
    blocked.future_corner_side_by_side = blocked.future_corner_side_by_side ||
                                         blocked.corner_side_by_side ||
                                         corner_still_relevant;
    if (blocked.yield_reason.empty()) {
      blocked.yield_reason = "future_yield_hold";
    }
  } else if (mode_ == BehaviorMode::YIELD_BEHIND && yield_lateral_hold &&
             blocked.yield_reason.empty()) {
    blocked.yield_reason = "yield_lateral_error_hold";
  }
  if (blocked.pass_reauthorization_lockout_active) {
    // state machineのmode holdでFREE/FOLLOWが一周期残っても、lockout中に
    // FASTEST/PASS publishへ戻さない。STOP中はCandidateBuilderがcurrent-d、
    // 解除後は保存済み包絡のnearest boundを同じRECOVERY型で生成する。
    mode_ = BehaviorMode::ABORT_RECOVERY;
  }
  // 処理ブロック: modeに合わせてpublish用候補を再生成する。
  // 設計意図: 内部PASS評価は状態遷移へ使いつつ、FOLLOW中やPREPARE中のMPC
  // horizonは設定に応じて安全側へ差し替える。
  CandidateTrajectory output_selected = selected;
  if (generic_safe_stop_available) {
    output_selected = generic_safe_stop_candidate;
  } else if (mode_ == BehaviorMode::SAFE_STOP) {
    output_selected =
        makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (recovery_nonmotion_probe_context &&
             blocked.maneuver_transaction_tracking_release_pending) {
    // RECOVERY/SPEED_GUARDの状態名は維持したまま、今周期に全評価を通した
    // PASSだけをtyped STOP下のwarm-up payloadとして出す。下の通常RECOVERY
    // 再生成で上書きするとattempt/tokenを作れず、exact ACKへ到達しない。
    // 縦motionはSafetyConstraint STOPとMux grantで引き続き閉じる。
    output_selected = selected;
  } else if (mode_ == BehaviorMode::SPEED_GUARD &&
             generic_recovery_phase_active_) {
    // long-horizon gateが拒否なら現d保持、許可済みなら同じRECOVERYを継続する。
    // 候補選択がFASTESTへ戻ってもgeneric phase中の横断を途中で失わないよう、
    // publish対象はここで明示的にRECOVERYへ揃える。
    const BlockedInfo recovery_blocked =
        reentry_gate.permitted ? centeringEvaluationBlockedInfo(blocked)
                               : blocked;
    output_selected = CandidateBuilder(frame_, config_)
                          .makeCandidate(CandidateType::RECOVERY, ego,
                                         recovery_blocked, opponents, nullptr);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::SPEED_GUARD &&
             permitted_reentry_convergence) {
    // gate許可後も中心到達まではFASTEST/FOLLOW/PASSへ戻さない。現在周期の
    // 全相手予測を使ってRECOVERYを再評価し、許可を失えば次周期ABORTへ戻す。
    const BlockedInfo centering_blocked =
        centeringEvaluationBlockedInfo(blocked);
    output_selected = CandidateBuilder(frame_, config_)
                          .makeCandidate(CandidateType::RECOVERY, ego,
                                         centering_blocked, opponents, nullptr);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::SPEED_GUARD &&
             post_abort_curve_hold_active_) {
    // hold列は以前のPASS/RECOVERYを再利用せず、現在dを終端まで固定した
    // RECOVERYを毎周期SafetyEvaluatorで再評価する。
    BlockedInfo hold_blocked = blocked;
    hold_blocked.reentry_hold_active = true;
    hold_blocked.reentry_hold_speed_cap_mps =
        postAbortCurveHoldSpeedCap(config_);
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, hold_blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::ABORT_RECOVERY) {
    // 中止時は必ず中心線へ戻す候補を再生成し、最新予測で安全評価する。
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::MERGE_BACK) {
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP) {
    output_selected = makeCandidate(CandidateType::SIDE_BY_SIDE_KEEP, ego,
                                    blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  } else if (mode_ == BehaviorMode::YIELD_BEHIND) {
    if (blocked.early_stationary_parallel_pass_hold_lateral) {
      // 停止カートへ早期PASSを試みた結果、左右PASSが不成立なら、YIELDの
      // 中心寄せで直ちに元レーンを横断しない。現在dを保持するRECOVERYを
      // 最新予測で再評価し、成立する間だけ低速で機会待ちへ落とす。
      // 現d保持自体が不安全なら従来のYIELD候補を改めて評価するため、
      // SafetyEvaluatorのfail-closed経路は迂回しない。
      CandidateTrajectory early_stationary_hold =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      evaluateCandidateAtOwnTimeAxis(early_stationary_hold, opponents, now_sec,
                                     predictions);
      if (early_stationary_hold.feasible) {
        output_selected = std::move(early_stationary_hold);
      } else {
        output_selected =
            makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents);
        evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                       predictions);
      }
    } else {
      output_selected =
          makeCandidate(CandidateType::YIELD_BEHIND, ego, blocked, opponents);
      evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                     predictions);
    }
  } else if (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
             output_selected.type != CandidateType::FOLLOW &&
             !blocked.maneuver_transaction_safe_lateral_hold_active &&
             !blocked.maneuver_transaction_tracking_stop_active &&
             !blocked.maneuver_transaction_tracking_release_pending) {
    // 追従モードでは速度上限だけを落とすFOLLOW候補を優先する。
    output_selected =
        makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
    if (!output_selected.feasible) {
      output_selected =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                     predictions);
    }
  } else if (isPrepareOvertakeMode(mode_) &&
             !publishPassHorizonInPrepare(config_) &&
             !blocked.maneuver_transaction_tracking_release_pending) {
    // PASS候補は内部状態遷移に使うが、PREPARE中はMPCへ追い越しhorizonを
    // 出さず、OVERTAKEに入った周期から横オフセットをpublishする。
    output_selected =
        makeCandidate(CandidateType::FOLLOW, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
    if (!output_selected.feasible) {
      output_selected =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                     predictions);
    }
  } else if (mode_ == BehaviorMode::FREE_RUN) {
    output_selected =
        makeCandidate(CandidateType::FASTEST, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  }

  if (mode_ == BehaviorMode::SPEED_GUARD && post_abort_curve_hold_active_ &&
      !output_selected.feasible) {
    // 現d保持そのものが安全でなくなったら、SPEED_GUARDの名前だけ残して
    // PASS/通常走行へ戻さない。reentry gate付きABORTの既存fail-closed経路へ
    // 即時に戻し、同じ周期のRECOVERYも改めて評価する。
    post_abort_curve_hold_active_ = false;
    blocked.post_abort_curve_hold_active = false;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason = "post_abort_curve_hold_safety_reject";
    blocked.reentry_hold_active = true;
    reentry_clear_cycles_ = 0;
    reentry_lockout_active_ = true;
    reentry_phase_active_ = true;
    reentry_gate_permitted_ = false;
    mode_ = BehaviorMode::ABORT_RECOVERY;
    reentry_gate =
        evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                            reentry_input, reentry_mpc_health);
    reentry_gate.requested = true;
    reentry_gate.permitted = false;
    reentry_gate.clear_cycles = 0;
    reentry_gate.reason = "post_abort_curve_hold_safety_reject";
    output_selected =
        makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(output_selected, opponents, now_sec,
                                   predictions);
  }

  // ROSノードがMPC overrideとdebug
  // JSONを作れるよう、選択結果を平坦な出力に詰める。
  // 処理ブロック: 出力形式へ変換し、rate
  // limitと高速カーブholdを最後に適用する。 設計意図:
  // 候補生成後の急な横参照変化をpublish直前で抑え、MPCへの入力を安定させる。
  auto output_built =
      PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
          mode_, ego, output_selected, blocked, safe_stop_context,
          safe_stop_candidate, safe_stop_candidate_infeasible,
          safe_stop_trigger_count_, state_machine_.safeStopHoldCount(),
          state_machine_.safeStopReleaseCount(), wall_soft_margin,
          active_section, mpc_health,
          reentry_input.pure_pursuit_primary_and_fresh,
          reentry_input.verified_non_mpc_pure_pursuit});
  output_built.start_grace_active = start_grace_active;
  output_built.reentry_gate = reentry_gate;
  output_built.lateral_stop_inputs_complete =
      exception_base_inputs_complete && prediction_complete;
  if (maneuver_execution_speed_only_stop_required) {
    // current-d軌道そのものをSafetyEvaluatorで認可できない時は、YIELDや中心向き
    // RECOVERYの横列を代用しない。transaction identityを残したまま縦STOPだけを
    // authorityへ渡し、次周期にfreshな同側PASS/current-d holdを再評価する。
    const double stop_cap_mps = std::max(1.0e-3, config_.safe_stop_v_mps);
    mode_ = BehaviorMode::FOLLOW_BLOCKED;
    output_built.mode = BehaviorMode::FOLLOW_BLOCKED;
    output_built.selected = CandidateType::SAFE_STOP;
    output_built.blocked_info = blocked;
    output_built.blocked_info.abort_safe_stop_hold_lateral = false;
    output_built.blocked_info.maneuver_transaction_safe_lateral_hold_active =
        false;
    output_built.blocked_info.maneuver_transaction_tracking_stop_active = true;
    output_built.active_override = false;
    output_built.selected = CandidateType::SAFE_STOP;
    output_built.selected_lateral_profile_safety_verified = false;
    output_built.lateral_tracking_authorized_during_stop = false;
    output_built.solver_horizon_intent =
        PlannerOutput::SolverHorizonIntent::NONE;
    output_built.lateral_offsets.clear();
    output_built.longitudinal_offsets_m.clear();
    output_built.speed_caps.assign(
        std::max<std::size_t>(1U, config_.horizon_points), stop_cap_mps);
    output_built.longitudinal_speed_cap_active = true;
    output_built.applied_speed_cap_mps = stop_cap_mps;
    output_built.speed_only_fallback_active = true;
    output_built.safe_stop_triggered = true;
    output_built.safe_stop_reason = "incomplete_pass_current_d_stop_infeasible";
    output_built.safe_stop_reject_reason = selected.reject_reason;
    output_built.speed_cap_reason = "incomplete_pass_execution_speed_only_stop";
    output_built.reason = "incomplete_pass_execution_speed_only_stop";
    output_built.target_lateral_offset_m = ego.frenet.d;
  }
  const bool verified_maneuver_tracking_stop_trajectory =
      (blocked.maneuver_transaction_tracking_stop_active ||
       blocked.maneuver_transaction_tracking_release_pending) &&
      output_built.lateral_stop_inputs_complete &&
      (!blocked.maneuver_transaction_prepared ||
       prepared_release_authority_still_valid) &&
      (blocked.maneuver_transaction_prepared ||
       (blocked.maneuver_transaction_retry_active &&
        blocked.maneuver_transaction_incomplete)) &&
      blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
      blocked.maneuver_target_fresh && blocked.maneuver_chain_tail_observed &&
      all_remaining_chain_waypoints_observed_fresh &&
      !blocked.maneuver_target_id.empty() &&
      !blocked.maneuver_chain_tail_id.empty() &&
      output_selected.safety_evaluated && output_selected.feasible &&
      output_selected.longitudinal_profile_valid &&
      output_selected.controller_tracking_profile_valid &&
      output_built.active_override &&
      (output_selected.type == CandidateType::SAFE_STOP ||
       output_selected.type == blocked.maneuver_transaction_pass_type);
  if (verified_maneuver_tracking_stop_trajectory) {
    // 停止constraint下でもPPが同じtrajectory generationを生成・検証できる
    // ようにする。認可対象はこの周期の全車両SafetyEvaluatorを通った現d stop
    // または同じtarget/sideのPASSだけで、縦出力はmuxがstopへ合成する。
    output_built.solver_horizon_intent =
        PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
    output_built.lateral_tracking_authorized_during_stop = true;
  }
  if (mode_ == BehaviorMode::SPEED_GUARD && post_abort_curve_hold_active_) {
    output_built.blocked_info.post_abort_curve_hold_active = true;
    output_built.speed_cap_reason = "post_abort_curve_hold";
    output_built.applied_speed_cap_mps = postAbortCurveHoldSpeedCap(config_);
    output_built.reason = "post_abort_curve_hold";
  }
  output_built.lateral_profile_mode = config_.overtake_lateral_profile_mode;
  output_built.maneuver_latch_active = localized_lateral_profile_.active;
  if (localized_lateral_profile_.active) {
    output_built.maneuver_latch_target_id =
        localized_lateral_profile_.target_id;
    output_built.maneuver_latch_target_s_m =
        localized_lateral_profile_.target_s_m;
    output_built.maneuver_latch_avoid_start_s_m =
        localized_lateral_profile_.avoid_start_s_m;
    output_built.maneuver_latch_full_offset_start_s_m =
        localized_lateral_profile_.full_offset_start_s_m;
    output_built.maneuver_latch_full_offset_end_s_m =
        localized_lateral_profile_.full_offset_end_s_m;
    output_built.maneuver_latch_merge_end_s_m =
        localized_lateral_profile_.merge_end_s_m;
    output_built.maneuver_latch_waypoint_count =
        static_cast<int>(localized_lateral_profile_.chain_waypoints.size());
    if (!localized_lateral_profile_.chain_waypoints.empty()) {
      const auto &last_waypoint =
          localized_lateral_profile_.chain_waypoints.back();
      output_built.maneuver_latch_last_waypoint_id = last_waypoint.target_id;
      output_built.maneuver_latch_last_waypoint_s_m = last_waypoint.target_s_m;
      output_built.maneuver_latch_last_waypoint_d_m = last_waypoint.target_d_m;
    }
  }
  if (start_grace_active) {
    output_built.reason = "start_grace_safe_stop_suppressed";
  } else if (leader_priority_safe_stop_suppressed &&
             output_built.reason.empty()) {
    output_built.reason = "leader_priority_safe_stop_suppressed";
  }
  if (generic_no_safe_hold) {
    const double fallback_cap_mps = std::min(
        std::max(1.0e-3, config_.reentry_hold_v_max_mps),
        std::min(
            std::max(1.0e-3, config_.safe_stop_v_mps),
            std::min(std::max(1.0e-3, config_.speed_only_fallback_v_max_mps),
                     std::max(1.0e-3,
                              config_.opponent_collision_fallback_v_max_mps))));
    output_built.mode = BehaviorMode::SPEED_GUARD;
    output_built.selected = CandidateType::RECOVERY;
    output_built.blocked_info.reentry_hold_active = true;
    output_built.active_override = false;
    output_built.longitudinal_speed_cap_active = true;
    output_built.lateral_offsets.clear();
    output_built.longitudinal_offsets_m.clear();
    output_built.speed_caps.assign(config_.horizon_points, fallback_cap_mps);
    output_built.applied_speed_cap_mps = fallback_cap_mps;
    output_built.speed_cap_reason =
        "generic_recovery_no_safe_hold_speed_only_watchdog";
    output_built.reason = "generic_recovery_no_safe_hold_watchdog";
  }
  if (generic_recovery_phase_active_ && output_built.reentry_gate.requested &&
      !output_built.reentry_gate.input_complete) {
    // stale入力では再評価済みholdだけを許可し、無ければwatchdog委譲へ閉じる。
    applyGenericRecoveryStaleHold(output_built);
  }
  applyLateralTargetRateLimit(now_sec, output_built);
  applyHighSpeedCurveLateralHold(now_sec, ego, output_built);
  if (!revalidatePublishedLateral(output_built, output_selected, ego, opponents,
                                  now_sec, predictions)) {
    if (generic_recovery_phase_active_) {
      // generic復帰はPASS中止ではない。publish整形後のd列が安全でなくなっても
      // ABORTへ昇格/phase破棄せず、現d holdを再評価する。holdも不成立なら
      // SAFE_STOP、さらに不成立なら横列なしのwatchdog委譲へ閉じる。
      high_speed_curve_lateral_hold_active_ = false;
      high_speed_curve_lateral_hold_offsets_.clear();
      high_speed_curve_lateral_hold_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      blocked.reentry_hold_active = true;
      reentry_clear_cycles_ = 0;
      reentry_gate_permitted_ = false;
      reentry_gate =
          evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                              reentry_input, reentry_mpc_health);
      reentry_gate.requested = true;
      reentry_gate.permitted = false;
      reentry_gate.clear_cycles = 0;
      reentry_gate.reason = "generic_published_lateral_safety_reject";
      reentry_gate_permitted_ = false;

      const CandidateTrajectory generic_hold =
          make_generic_recovery_hold_candidate();
      if (generic_hold.feasible) {
        mode_ = BehaviorMode::SPEED_GUARD;
        output_built =
            PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
                mode_, ego, generic_hold, blocked, safe_stop_context,
                safe_stop_candidate, safe_stop_candidate_infeasible,
                safe_stop_trigger_count_, state_machine_.safeStopHoldCount(),
                state_machine_.safeStopReleaseCount(), wall_soft_margin,
                active_section, mpc_health,
                reentry_input.pure_pursuit_primary_and_fresh,
                reentry_input.verified_non_mpc_pure_pursuit});
        const double configured_hold_cap_mps =
            std::max(1.0e-3, config_.reentry_hold_v_max_mps);
        double hold_cap_mps = configured_hold_cap_mps;
        for (const double speed_cap_mps : generic_hold.v_ref) {
          if (std::isfinite(speed_cap_mps) && speed_cap_mps > 0.0) {
            hold_cap_mps = std::min(hold_cap_mps, speed_cap_mps);
          }
        }
        output_built.active_override = true;
        output_built.lateral_offsets = generic_hold.d;
        output_built.target_lateral_offset_m = generic_hold.d.back();
        output_built.mode = BehaviorMode::SPEED_GUARD;
        output_built.selected = CandidateType::RECOVERY;
        output_built.blocked_info.reentry_hold_active = true;
        output_built.speed_caps.assign(output_built.lateral_offsets.size(),
                                       hold_cap_mps);
        output_built.applied_speed_cap_mps = hold_cap_mps;
        output_built.speed_cap_reason =
            "generic_published_lateral_safety_reject_hold";
        output_built.reason = "generic_published_lateral_safety_reject_hold";
      } else {
        generic_recovery_hold_offsets_.clear();
        generic_recovery_hold_speed_cap_mps_ =
            std::numeric_limits<double>::quiet_NaN();
        const CandidateTrajectory generic_stop =
            make_generic_safe_stop_candidate();
        if (generic_stop.feasible) {
          // RECOVERY
          // holdを安全に出せないため、同周期に評価済みSAFE_STOPへ切替える。
          rememberGenericRecoveryHold(generic_stop, reentry_gate);
          mode_ = BehaviorMode::SAFE_STOP;
          output_built =
              PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
                  mode_, ego, generic_stop, blocked, safe_stop_context,
                  safe_stop_candidate, safe_stop_candidate_infeasible,
                  safe_stop_trigger_count_, state_machine_.safeStopHoldCount(),
                  state_machine_.safeStopReleaseCount(), wall_soft_margin,
                  active_section, mpc_health,
                  reentry_input.pure_pursuit_primary_and_fresh,
                  reentry_input.verified_non_mpc_pure_pursuit});
          output_built.mode = BehaviorMode::SAFE_STOP;
          output_built.selected = CandidateType::SAFE_STOP;
          output_built.reason =
              "generic_published_lateral_safety_reject_safe_stop";
        } else {
          // STOPも不成立なら横列を捏造しない。既存watchdogへ委譲しつつ、
          // speed-only fallbackと同等以下の低速capだけを診断へ残す。
          const double fallback_cap_mps = std::min(
              std::max(1.0e-3, config_.reentry_hold_v_max_mps),
              std::min(
                  std::max(1.0e-3, config_.safe_stop_v_mps),
                  std::min(
                      std::max(1.0e-3, config_.speed_only_fallback_v_max_mps),
                      std::max(
                          1.0e-3,
                          config_.opponent_collision_fallback_v_max_mps))));
          mode_ = BehaviorMode::SPEED_GUARD;
          output_built.mode = BehaviorMode::SPEED_GUARD;
          output_built.selected = CandidateType::RECOVERY;
          output_built.blocked_info.reentry_hold_active = true;
          output_built.active_override = false;
          output_built.longitudinal_speed_cap_active = true;
          output_built.lateral_offsets.clear();
          output_built.longitudinal_offsets_m.clear();
          output_built.speed_caps.assign(config_.horizon_points,
                                         fallback_cap_mps);
          output_built.applied_speed_cap_mps = fallback_cap_mps;
          output_built.speed_cap_reason =
              "generic_published_lateral_safety_reject_no_safe_hold_"
              "speed_only_watchdog";
          output_built.reason =
              "generic_published_lateral_safety_reject_no_safe_hold_"
              "watchdog";
        }
      }
      output_built.published_lateral_safety_rejected = true;
      output_built.reentry_gate = reentry_gate;
    } else {
      // publish直前のrate
      // limit/holdで形が変わったPASS軌道を、そのままMPCへ渡さない。
      // overrideを消して通常走行へ戻すと、停止障害物の前で再加速し得るため、
      // 同じ周期にRECOVERYを安全評価してpublishする。RECOVERYも不可なら
      // PlannerOutputBuilderの既存speed-only fallbackへ必ず渡す。
      mode_ = BehaviorMode::ABORT_RECOVERY;
      post_abort_curve_hold_active_ = false;
      high_speed_curve_lateral_hold_active_ = false;
      high_speed_curve_lateral_hold_offsets_.clear();
      high_speed_curve_lateral_hold_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      // publish後に形が変わって安全でなくなった場合も、reentry gateが有効なら
      // 同じ周期に通常ラインまでの候補を再評価する。未許可なら現在dを保持して
      // 二次衝突を防ぐ。gate無効時は既存どおりRECOVERYへ委譲する。
      blocked.reentry_hold_active = false;
      reentry_clear_cycles_ = 0;
      reentry_lockout_active_ = true;
      reentry_phase_active_ = true;
      reentry_gate_permitted_ = false;
      generic_recovery_phase_active_ = false;
      generic_recovery_hold_offsets_.clear();
      generic_recovery_hold_speed_cap_mps_ =
          std::numeric_limits<double>::quiet_NaN();
      if (config_.reentry_gate_enabled) {
        reentry_gate =
            evaluateReentryGate(now_sec, ego, blocked, opponents, mpc_health,
                                reentry_input, reentry_mpc_health);
        blocked.reentry_hold_active = !reentry_gate.permitted;
      }
      if (reentry_gate.requested) {
        reentry_gate.permitted = false;
        reentry_gate.clear_cycles = 0;
        reentry_gate.reason = "published_reentry_safety_reject";
        reentry_gate_permitted_ = false;
        reentry_lockout_active_ = true;
      }
      CandidateTrajectory recovery =
          makeCandidate(CandidateType::RECOVERY, ego, blocked, opponents);
      evaluateCandidateAtOwnTimeAxis(recovery, opponents, now_sec, predictions);
      if (!recovery.longitudinal_profile_valid) {
        recovery.feasible = false;
        recovery.reject_reason = "invalid_longitudinal_brake_model";
      }
      output_built =
          PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
              mode_, ego, recovery, blocked, safe_stop_context,
              safe_stop_candidate, safe_stop_candidate_infeasible,
              safe_stop_trigger_count_, state_machine_.safeStopHoldCount(),
              state_machine_.safeStopReleaseCount(), wall_soft_margin,
              active_section, mpc_health,
              reentry_input.pure_pursuit_primary_and_fresh,
              reentry_input.verified_non_mpc_pure_pursuit});
      output_built.reason = recovery.feasible
                                ? "published_lateral_safety_reject_recovery"
                                : "published_lateral_safety_reject_speed_only";
      output_built.published_lateral_safety_rejected = true;
      output_built.reentry_gate = reentry_gate;
    }
  }
  if (isMandatoryAbortLateralAvoidance(output_built)) {
    output_built.solver_horizon_intent =
        PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE;
    // SafetyConstraintReleaseGateが直前のstop要求を連続安全確認まで保持して
    // いる周期でも、最新の全車両予測でSafetyEvaluatorを通った「現在dの
    // 定位置hold」だけは横追従を継続する。ここをsafe_stop_triggeredの同一周期
    // に限定すると、縦停止中にsteer=0へ落ちて車体が壁側へ流れ、その流出dを
    // 次周期のhold目標にするratchetになる。中心復帰や過去dへの再操舵は許可せず、
    // E-stop/watchdog/control faultはmux側の独立stop経路を維持する。
    output_built.lateral_tracking_authorized_during_stop =
        detail::isSafetyEvaluatedCurrentLateralHoldDuringStop(output_built);
  }
  if (detail::isSafetyEvaluatedCurrentTransactionHoldDuringStop(output_built)) {
    // PASS不成立後のFOLLOW/RECOVERYへ縦stopが一周期遅れて残っても、同じ
    // target/sideを保持したSafetyEvaluator済みcurrent-d列まで捨てない。
    // これは新規横移動ではないためsolver horizon intentはNONEのままにし、
    // Nodeのauthoritative planとmuxの同generation PP proofだけで横追従する。
    output_built.lateral_tracking_authorized_during_stop = true;
  }
  const bool pass_profile_recovery_requires_lateral_tracking =
      output_built.blocked_info.pass_reauthorization_lockout_active &&
      output_built.selected == CandidateType::RECOVERY &&
      output_built.active_override &&
      output_built.selected_lateral_profile_safety_verified &&
      output_built.lateral_offsets.size() > 1U &&
      std::any_of(output_built.lateral_offsets.begin(),
                  output_built.lateral_offsets.end(), [&ego](double d_m) {
                    return std::isfinite(d_m) && std::isfinite(ego.frenet.d) &&
                           std::abs(d_m - ego.frenet.d) > 1.0e-6;
                  });
  if (pass_profile_recovery_requires_lateral_tracking) {
    // STOP解除後は保存済みGate 2包絡へ戻るRECOVERYを実際にPPへ追従させる。
    // current-d holdは上の専用stop契約だけで扱い、横移動を含む時に限って
    // OvertakePlan.lateral_maneuver_requiredへ到達する型付きintentを付ける。
    // Node側のconstraint/generation/stamp/trajectory完全性検証は従来どおり
    // 必須であり、stop中の新規横移動を認可するものではない。
    output_built.solver_horizon_intent =
        PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  }
  const bool current_safe_stop_lateral_authority_requested =
      output_built.mode == BehaviorMode::SAFE_STOP &&
      output_built.selected == CandidateType::SAFE_STOP &&
      output_built.safe_stop_triggered && output_built.active_override &&
      output_built.selected_lateral_profile_safety_verified &&
      output_built.lateral_stop_inputs_complete &&
      !output_built.published_lateral_safety_rejected &&
      isConstantLateralHold(output_built.lateral_offsets) &&
      std::isfinite(output_built.blocked_info.ego_lateral_offset_m) &&
      std::abs(output_built.lateral_offsets.front() -
               output_built.blocked_info.ego_lateral_offset_m) <= 1.0e-4;
  if (current_safe_stop_lateral_authority_requested) {
    // 通常SAFE_STOPは候補仲裁を変えない。横authorityへ昇格するこの最終地点で
    // だけ、同じcurrent-d/縦profileを必要arcまで延長して全相手へ再評価する。
    BlockedInfo stop_authority_blocked = blocked;
    stop_authority_blocked.abort_safe_stop_hold_lateral = true;
    CandidateTrajectory stop_authority_candidate = makeCandidate(
        CandidateType::SAFE_STOP, ego, stop_authority_blocked, opponents);
    evaluateCandidateAtOwnTimeAxis(stop_authority_candidate, opponents, now_sec,
                                   predictions);
    if (stop_authority_candidate.longitudinal_profile_valid &&
        stop_authority_candidate.controller_spatial_horizon_proof_valid) {
      output_selected = stop_authority_candidate;
      output_built.blocked_info = stop_authority_blocked;
      output_built.lateral_offsets = stop_authority_candidate.d;
      output_built.speed_caps = stop_authority_candidate.v_ref;
      output_built.longitudinal_offsets_m =
          stop_authority_candidate.longitudinal_offsets_m;
      output_built.target_lateral_offset_m = ego.frenet.d;
      output_built.min_cbf_h = stop_authority_candidate.min_safety_margin;
      output_built.cbf_slack = stop_authority_candidate.cbf_slack;
      output_built.active_cbf_constraint_count =
          stop_authority_candidate.active_safety_constraint_count;
      output_built.required_controller_spatial_horizon_m =
          stop_authority_candidate.required_controller_spatial_horizon_m;
      output_built.controller_spatial_horizon_proof_valid = true;
    } else {
      // proofを作れないSAFE_STOPは選択/transaction identityを変えず、
      // speed-onlyへ明示的にfail-closeする。未評価の短い横列を残さない。
      const double stop_cap_mps = std::max(1.0e-3, config_.safe_stop_v_mps);
      output_built.active_override = false;
      output_built.selected_lateral_profile_safety_verified = false;
      output_built.controller_spatial_horizon_proof_valid = false;
      output_built.required_controller_spatial_horizon_m =
          stop_authority_candidate.required_controller_spatial_horizon_m;
      output_built.lateral_tracking_authorized_during_stop = false;
      output_built.solver_horizon_intent =
          PlannerOutput::SolverHorizonIntent::NONE;
      output_built.lateral_offsets.clear();
      output_built.longitudinal_offsets_m.clear();
      output_built.speed_caps.assign(
          std::max<std::size_t>(1U, config_.horizon_points), stop_cap_mps);
      output_built.longitudinal_speed_cap_active = true;
      output_built.applied_speed_cap_mps = stop_cap_mps;
      output_built.speed_only_fallback_active = true;
      output_built.safe_stop_triggered = true;
      output_built.speed_cap_reason =
          "controller_spatial_horizon_unproven_speed_only_stop";
      output_built.reason =
          "controller_spatial_horizon_unproven_speed_only_stop";
    }
  }
  if (detail::isSafetyEvaluatedCurrentSafeStopLateralHold(output_built)) {
    // no-pass/no-fallbackで縦停止へ入っても、同周期の全入力と
    // SafetyEvaluatorで検証した「現在d一定」の横列まで消すと、高速車両が
    // stop完了前に接線方向へ直進して壁外へ出る。新規横移動や中心復帰は認可せず、
    // muxが同generationのPP proofを得た時だけ縦stopと合成できる型にする。
    output_built.solver_horizon_intent =
        PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE;
    output_built.lateral_tracking_authorized_during_stop = true;
  }
  if (!output_built.lateral_stop_inputs_complete) {
    // どの横HOLD/PASS経路も、全相手を含むfresh予測がないSTOP周期には
    // trajectory authorizationへ昇格させない。transaction identityは保持する。
    output_built.lateral_tracking_authorized_during_stop = false;
  }
  capNoPassNormalRecoverySpeed(config_, output_built);
  const bool final_tracking_release_pass =
      start_grid_tracking_release_candidate_valid_ &&
      start_grid_tracking_release_token_ != 0U &&
      blocked.maneuver_transaction_tracking_release_pending &&
      output_built.active_override &&
      output_built.selected == blocked.maneuver_transaction_pass_type &&
      (output_built.selected == CandidateType::PASS_LEFT ||
       output_built.selected == CandidateType::PASS_RIGHT) &&
      output_built.selected_lateral_profile_safety_verified &&
      !output_built.published_lateral_safety_rejected &&
      output_built.lateral_stop_inputs_complete &&
      output_built.lateral_offsets.size() == output_built.speed_caps.size() &&
      output_built.lateral_offsets.size() ==
          output_built.longitudinal_offsets_m.size() &&
      output_built.lateral_offsets.size() == output_selected.d.size();
  if (final_tracking_release_pass) {
    // rate limit/hold後に実publishするrelative
    // payloadを保存snapshotへ反映する。
    // 次周期のSafetyEvaluatorとControllerTrackingStatusが、raw候補ではなく
    // 実際にPPへ渡した同じd/v/dsを参照するための最終同期点。
    CandidateTrajectory published_release_candidate = output_selected;
    published_release_candidate.d = output_built.lateral_offsets;
    published_release_candidate.v_ref = output_built.speed_caps;
    published_release_candidate.longitudinal_offsets_m =
        output_built.longitudinal_offsets_m;
    // この周期にcurrent
    // egoから生成・installした候補は既に同じ基準へ揃っている。
    // 二重rebaseはせず、下のtrackabilityと全相手SafetyEvaluatorは省略しない。
    const bool published_rebased =
        tracking_release_candidate_installed_this_cycle ||
        rebaseCandidateToCurrentEgo(published_release_candidate, ego, frame_);
    const bool published_trackable =
        published_rebased &&
        CandidateBuilder(frame_, config_)
            .publishedLateralProfileTrackable(published_release_candidate, ego);
    published_release_candidate.controller_tracking_profile_valid =
        published_trackable;
    if (published_trackable) {
      evaluateCandidateAtOwnTimeAxis(published_release_candidate, opponents,
                                     now_sec, predictions);
    }
    const bool published_release_valid =
        published_release_candidate.safety_evaluated &&
        published_release_candidate.feasible &&
        published_release_candidate.longitudinal_profile_valid &&
        published_release_candidate.controller_tracking_profile_valid &&
        hasControllerSpatialHorizonProof(published_release_candidate) &&
        published_release_candidate.type ==
            blocked.maneuver_transaction_pass_type;
    if (published_release_valid) {
      const bool published_wire_changed =
          !releaseCandidateWireSemanticallyEqual(
              start_grid_tracking_release_candidate_,
              published_release_candidate);
      if (published_wire_changed &&
          !tracking_release_candidate_installed_this_cycle) {
        // 既に外へ出したwarm-up payloadを後段整形で変更した場合は新tokenにし、
        // 旧generation ACKで解除せず新payloadを停止下で再確認する。
        advance_start_grid_tracking_release_token();
        start_grid_tracking_release_confirmed_ = false;
        start_grid_tracking_release_cycles_ = 1;
        blocked.maneuver_transaction_tracking_release_confirmed = false;
        blocked.maneuver_transaction_tracking_release_cycles = 1;
        blocked.maneuver_transaction_tracking_stop_active = true;
        output_built.blocked_info
            .maneuver_transaction_tracking_release_confirmed = false;
        output_built.blocked_info.maneuver_transaction_tracking_release_cycles =
            1;
        output_built.blocked_info.maneuver_transaction_tracking_stop_active =
            true;
        if (output_built.reason.empty()) {
          output_built.reason = "tracking_release_wire_changed_" +
                                releaseCandidateWireChangeReason(
                                    start_grid_tracking_release_candidate_,
                                    published_release_candidate);
        }
      }
      // outputと評価済みcandidateを同じd/v/ds/required proofへ同期する。
      // post-mutationでこの束縛が切れた場合は、return直前のuniversal gateが
      // proofを流用せずspeed-only STOPへ閉じる。
      output_selected = published_release_candidate;
      output_built.required_controller_spatial_horizon_m =
          published_release_candidate.required_controller_spatial_horizon_m;
      output_built.controller_spatial_horizon_proof_valid =
          published_release_candidate.controller_spatial_horizon_proof_valid;
      start_grid_tracking_release_candidate_ =
          std::move(published_release_candidate);
      output_built.tracking_release_pass_warmup = true;
      output_built.tracking_release_token = start_grid_tracking_release_token_;
    } else {
      const bool attack_follow_transaction_identity_valid =
          localized_lateral_profile_.active &&
          localized_lateral_profile_.target_id == blocked.maneuver_target_id &&
          localized_lateral_profile_.pass_type ==
              blocked.maneuver_transaction_pass_type &&
          (blocked.maneuver_transaction_pass_type == CandidateType::PASS_LEFT ||
           blocked.maneuver_transaction_pass_type ==
               CandidateType::PASS_RIGHT) &&
          blocked.maneuver_transaction_retry_active &&
          blocked.maneuver_transaction_incomplete &&
          blocked.maneuver_transaction_tracking_continuity_armed &&
          blocked.attack_follow_hold_pass_side &&
          blocked.attack_follow_candidate_generated &&
          blocked.attack_follow_candidate_feasible &&
          blocked.attack_follow_candidate_tracking_profile_valid &&
          blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
          blocked.maneuver_target_fresh &&
          blocked.maneuver_chain_tail_observed &&
          exception_base_inputs_complete && prediction_complete;
      const auto verified_attack_follow = std::find_if(
          candidates.cbegin(), candidates.cend(),
          [&attack_follow_transaction_identity_valid,
           this](const CandidateTrajectory &candidate) {
            return attack_follow_transaction_identity_valid &&
                   candidate.type == CandidateType::FOLLOW &&
                   candidate.safety_evaluated && candidate.feasible &&
                   candidate.longitudinal_profile_valid &&
                   candidate.controller_tracking_profile_valid &&
                   candidate.pass_target_corridor_valid &&
                   (!candidate.attack_follow_safe_lateral_hold ||
                    (candidate
                         .attack_follow_opponent_collision_current_d_hold !=
                     candidate
                         .attack_follow_opponent_collision_inward_connector)) &&
                   std::isfinite(
                       candidate.committed_attack_follow_target_d_m) &&
                   std::isfinite(localized_lateral_profile_.target_d_m) &&
                   std::abs(candidate.committed_attack_follow_target_d_m -
                            localized_lateral_profile_.target_d_m) <= 1.0e-6;
          });
      bool attack_follow_publish_valid = false;
      PlannerOutput attack_follow_output;
      CandidateTrajectory attack_follow_candidate;
      if (verified_attack_follow != candidates.cend()) {
        attack_follow_candidate = *verified_attack_follow;
        BlockedInfo attack_follow_blocked = blocked;
        attack_follow_blocked.maneuver_transaction_tracking_release_pending =
            false;
        attack_follow_blocked.maneuver_transaction_tracking_release_confirmed =
            false;
        attack_follow_blocked.maneuver_transaction_tracking_release_cycles = 0;
        attack_follow_blocked.maneuver_transaction_tracking_stop_active = false;
        attack_follow_output =
            PlannerOutputBuilder(config_).build(PlannerOutputBuildInput{
                BehaviorMode::FOLLOW_BLOCKED, ego, attack_follow_candidate,
                attack_follow_blocked, safe_stop_context, safe_stop_candidate,
                safe_stop_candidate_infeasible, safe_stop_trigger_count_,
                state_machine_.safeStopHoldCount(),
                state_machine_.safeStopReleaseCount(), wall_soft_margin,
                active_section, mpc_health,
                reentry_input.pure_pursuit_primary_and_fresh,
                reentry_input.verified_non_mpc_pure_pursuit});
        attack_follow_output.start_grace_active =
            output_built.start_grace_active;
        attack_follow_output.reentry_gate = output_built.reentry_gate;
        attack_follow_output.lateral_stop_inputs_complete =
            output_built.lateral_stop_inputs_complete;
        attack_follow_output.lateral_profile_mode =
            output_built.lateral_profile_mode;
        attack_follow_output.maneuver_latch_active =
            output_built.maneuver_latch_active;
        attack_follow_output.maneuver_latch_target_id =
            output_built.maneuver_latch_target_id;
        attack_follow_output.maneuver_latch_target_s_m =
            output_built.maneuver_latch_target_s_m;
        attack_follow_output.maneuver_latch_avoid_start_s_m =
            output_built.maneuver_latch_avoid_start_s_m;
        attack_follow_output.maneuver_latch_full_offset_start_s_m =
            output_built.maneuver_latch_full_offset_start_s_m;
        attack_follow_output.maneuver_latch_full_offset_end_s_m =
            output_built.maneuver_latch_full_offset_end_s_m;
        attack_follow_output.maneuver_latch_merge_end_s_m =
            output_built.maneuver_latch_merge_end_s_m;
        attack_follow_output.maneuver_latch_waypoint_count =
            output_built.maneuver_latch_waypoint_count;
        attack_follow_output.maneuver_latch_last_waypoint_id =
            output_built.maneuver_latch_last_waypoint_id;
        attack_follow_output.maneuver_latch_last_waypoint_s_m =
            output_built.maneuver_latch_last_waypoint_s_m;
        attack_follow_output.maneuver_latch_last_waypoint_d_m =
            output_built.maneuver_latch_last_waypoint_d_m;
        capNoPassNormalRecoverySpeed(config_, attack_follow_output);
        applyLateralTargetRateLimit(now_sec, attack_follow_output);
        applyHighSpeedCurveLateralHold(now_sec, ego, attack_follow_output);
        attack_follow_publish_valid = revalidatePublishedLateral(
            attack_follow_output, attack_follow_candidate, ego, opponents,
            now_sec, predictions);
      }
      if (attack_follow_publish_valid) {
        // 不成立PASSの過去のSafetyEvaluator結果は権限に使わず、yawなどが
        // 回復した後に同じtarget/sideでrebase・追従性・全相手予測を
        // 再評価するための形状snapshotだけを保存する。
        start_grid_tracking_failed_candidate_valid_ = true;
        start_grid_tracking_failed_target_id_ = blocked.maneuver_target_id;
        start_grid_tracking_failed_pass_type_ =
            blocked.maneuver_transaction_pass_type;
        start_grid_tracking_failed_candidate_ = published_release_candidate;
        start_grid_tracking_failed_ego_yaw_rad_ = ego.yaw;
        reset_start_grid_tracking_release();
        reset_start_grid_tracking_probe();
        blocked.maneuver_transaction_tracking_release_pending = false;
        blocked.maneuver_transaction_tracking_release_confirmed = false;
        blocked.maneuver_transaction_tracking_release_cycles = 0;
        blocked.maneuver_transaction_tracking_stop_active = false;
        blocked.abort_safe_stop_hold_lateral = false;
        mode_ = BehaviorMode::FOLLOW_BLOCKED;
        selected = attack_follow_candidate;
        output_selected = attack_follow_candidate;
        attack_follow_output.mode = BehaviorMode::FOLLOW_BLOCKED;
        attack_follow_output.selected = CandidateType::FOLLOW;
        attack_follow_output.blocked_info = blocked;
        attack_follow_output.reason =
            "tracking_release_published_pass_rejected_attack_follow";
        attack_follow_output.published_lateral_safety_rejected = false;
        attack_follow_output.tracking_release_pass_warmup = false;
        attack_follow_output.tracking_release_token = 0U;
        output_built = std::move(attack_follow_output);
      } else {
        // revalidatePublishedLateralとの不整合は安全側へ閉じる。同じ周期に
        // 検証済みATTACK_FOLLOWもpublishできない時だけ、reject済みPASS列を
        // 残さず、現在d SAFE_STOPを別評価して置換する。
        start_grid_tracking_release_candidate_valid_ = false;
        start_grid_tracking_release_candidate_ = CandidateTrajectory{};
        start_grid_tracking_release_token_ = 0U;
        start_grid_tracking_release_confirmed_ = false;
        blocked.maneuver_transaction_tracking_release_confirmed = false;
        blocked.maneuver_transaction_tracking_stop_active = true;
        output_built.blocked_info
            .maneuver_transaction_tracking_release_confirmed = false;
        output_built.blocked_info.maneuver_transaction_tracking_stop_active =
            true;
        blocked.abort_safe_stop_hold_lateral = true;
        CandidateTrajectory exact_current_d_stop =
            makeCandidate(CandidateType::SAFE_STOP, ego, blocked, opponents);
        evaluateCandidateAtOwnTimeAxis(exact_current_d_stop, opponents, now_sec,
                                       predictions);
        if (!exact_current_d_stop.longitudinal_profile_valid) {
          exact_current_d_stop.feasible = false;
          exact_current_d_stop.reject_reason =
              "invalid_longitudinal_brake_model";
        }
        const bool exact_current_d_stop_valid =
            exact_current_d_stop.safety_evaluated &&
            exact_current_d_stop.feasible &&
            exact_current_d_stop.longitudinal_profile_valid &&
            exact_current_d_stop.controller_tracking_profile_valid &&
            holds_current_lateral(exact_current_d_stop);
        mode_ = BehaviorMode::FOLLOW_BLOCKED;
        output_built.mode = BehaviorMode::FOLLOW_BLOCKED;
        output_built.selected = CandidateType::SAFE_STOP;
        output_built.safe_stop_triggered = true;
        output_built.safe_stop_reason =
            "tracking_release_published_pass_rejected";
        const std::string published_release_reject_reason =
            !published_rebased     ? "rebase_invalid"
            : !published_trackable ? "untrackable"
            : published_release_candidate.reject_reason.empty()
                ? "safety_invalid"
                : published_release_candidate.reject_reason;
        output_built.reason =
            (exact_current_d_stop_valid
                 ? "tracking_release_published_pass_rejected_current_d_stop_"
                 : "tracking_release_published_pass_rejected_speed_only_"
                   "stop_") +
            published_release_reject_reason;
        output_built.published_lateral_safety_rejected = true;
        output_built.blocked_info = blocked;
        output_built.blocked_info.abort_safe_stop_hold_lateral =
            exact_current_d_stop_valid;
        output_built.blocked_info
            .maneuver_transaction_safe_lateral_hold_active =
            exact_current_d_stop_valid;
        output_built.active_override = exact_current_d_stop_valid;
        output_built.selected_lateral_profile_safety_verified =
            exact_current_d_stop_valid;
        output_built.lateral_tracking_authorized_during_stop =
            exact_current_d_stop_valid &&
            output_built.lateral_stop_inputs_complete;
        output_built.solver_horizon_intent =
            output_built.lateral_tracking_authorized_during_stop
                ? PlannerOutput::SolverHorizonIntent::MANDATORY_AVOIDANCE
                : PlannerOutput::SolverHorizonIntent::NONE;
        if (exact_current_d_stop_valid) {
          output_selected = exact_current_d_stop;
          output_built.lateral_offsets = exact_current_d_stop.d;
          output_built.speed_caps = exact_current_d_stop.v_ref;
          output_built.longitudinal_offsets_m =
              exact_current_d_stop.longitudinal_offsets_m;
          output_built.target_lateral_offset_m = ego.frenet.d;
          output_built.min_cbf_h = exact_current_d_stop.min_safety_margin;
          output_built.cbf_slack = exact_current_d_stop.cbf_slack;
          output_built.active_cbf_constraint_count =
              exact_current_d_stop.active_safety_constraint_count;
        } else {
          // current-d STOP自体が不成立なら横列を捏造しない。Muxへ縦停止を
          // 要求するspeed-only契約だけを残し、同じtarget/side
          // transactionを次周期へ保持する。
          output_built.lateral_offsets.clear();
          output_built.longitudinal_offsets_m.clear();
          output_built.speed_caps.assign(
              std::max<std::size_t>(1U, config_.horizon_points),
              std::max(1.0e-3, config_.safe_stop_v_mps));
          output_built.target_lateral_offset_m = 0.0;
        }
        output_built.longitudinal_speed_cap_active = true;
        output_built.applied_speed_cap_mps =
            std::max(1.0e-3, config_.safe_stop_v_mps);
        output_built.speed_cap_reason =
            "tracking_release_published_pass_rejected_stop";
        output_built.tracking_release_pass_warmup = false;
        output_built.tracking_release_token = 0U;
      }
    }
  } else {
    output_built.tracking_release_pass_warmup = false;
    output_built.tracking_release_token = 0U;
    if (blocked.maneuver_transaction_tracking_release_pending &&
        output_built.reason.empty()) {
      const std::string not_publishable_reason =
          !start_grid_tracking_release_candidate_valid_ ? "candidate_invalid"
          : start_grid_tracking_release_token_ == 0U    ? "token_missing"
          : !output_built.active_override               ? "override_inactive"
          : output_built.selected != blocked.maneuver_transaction_pass_type
              ? "selected_type_mismatch"
          : !output_built.selected_lateral_profile_safety_verified
              ? "lateral_safety_unverified"
          : output_built.published_lateral_safety_rejected
              ? "published_lateral_rejected"
          : !output_built.lateral_stop_inputs_complete ? "inputs_incomplete"
          : output_built.lateral_offsets.size() !=
                  output_built.speed_caps.size()
              ? "lateral_speed_shape_mismatch"
          : output_built.lateral_offsets.size() !=
                  output_built.longitudinal_offsets_m.size()
              ? "lateral_longitudinal_shape_mismatch"
          : output_built.lateral_offsets.size() != output_selected.d.size()
              ? "published_candidate_shape_mismatch"
              : "unknown";
      output_built.reason =
          "tracking_release_not_publishable_" + not_publishable_reason;
      if (!tracking_release_candidate_failure_reason.empty()) {
        output_built.reason += "_" + tracking_release_candidate_failure_reason;
      }
    }
  }
  output_built.transition_previous_mode = mode_before_transition;
  output_built.state_machine_mode = state_machine_mode;
  output_built.post_reentry_arbitration_mode = post_reentry_arbitration_mode;
  output_built.raw_selected = selected.type;
  output_built.raw_selected_feasible = selected.feasible;
  output_built.raw_selected_reject_reason = selected.reject_reason;
  output_built.reentry_phase_before_update = reentry_phase_before_update;
  output_built.reentry_lockout_before_update = reentry_lockout_before_update;
  output_built.generic_recovery_before_update = generic_recovery_before_update;
  output_built.reentry_phase_after_update = reentry_phase_after_update;
  output_built.reentry_lockout_after_update = reentry_lockout_after_update;
  output_built.generic_recovery_after_update = generic_recovery_after_update;
  output_built.supervisor_v2 = supervisor_v2_decision;
  if (localized_lateral_profile_.active) {
    const bool prepared_start_grid_commit_authorized =
        !blocked.maneuver_transaction_prepared ||
        (blocked.start_grid_target_active &&
         blocked.maneuver_transaction_tracking_release_pending &&
         blocked.maneuver_transaction_tracking_release_confirmed &&
         !blocked.maneuver_transaction_tracking_stop_active);
    const bool direct_first_pass_commit_authority_ready =
        localized_lateral_profile_.pass_safety_approved_once &&
        blocked.maneuver_target_pass_safety_approved &&
        blocked.maneuver_chain_tail_observed &&
        localized_profile_target_identity_consistent &&
        pass_start_observation_inputs_complete &&
        pass_start_prediction_complete && pass_start_live_tracking_usable;
    const bool prepared_start_grid_tracking_authority_ready =
        localized_lateral_profile_.pass_safety_approved_once &&
        blocked.start_grid_target_active &&
        blocked.maneuver_transaction_prepared &&
        blocked.maneuver_transaction_tracking_release_pending &&
        blocked.maneuver_transaction_tracking_release_confirmed &&
        !blocked.maneuver_transaction_tracking_stop_active &&
        blocked.maneuver_chain_tail_observed &&
        localized_profile_target_identity_consistent &&
        pass_start_observation_inputs_complete &&
        pass_start_prediction_complete && reentry_input.mpc_health_fresh &&
        !reentry_input.mpc_hard_failure;
    const bool pass_commit_authority_ready =
        localized_lateral_profile_.pass_execution_committed ||
        direct_first_pass_commit_authority_ready ||
        prepared_start_grid_tracking_authority_ready;
    const bool published_same_pass_transaction =
        blocked.maneuver_target_latched && blocked.maneuver_target_observed &&
        blocked.maneuver_target_fresh && output_built.maneuver_latch_active &&
        output_built.maneuver_latch_target_id ==
            localized_lateral_profile_.target_id &&
        output_built.selected == localized_lateral_profile_.pass_type &&
        output_built.active_override &&
        output_built.selected_lateral_profile_safety_verified &&
        output_built.solver_horizon_intent !=
            PlannerOutput::SolverHorizonIntent::NONE &&
        prepared_start_grid_commit_authorized && pass_commit_authority_ready;
    if (published_same_pass_transaction) {
      // raw候補がGate 2を通っただけではcommitしない。Nodeがauthoritative
      // reference_overrideへ運べる同じtarget/sideの安全評価済みPASS payloadが
      // 最終出力に残った周期だけ、以後のID/side保持とABORT回復契約を開始する。
      localized_lateral_profile_.pass_execution_committed = true;
    }
    const bool transaction_incomplete =
        localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed;
    output_built.blocked_info.maneuver_transaction_incomplete =
        transaction_incomplete;
    output_built.blocked_info.maneuver_transaction_prepared =
        is_confirmed_start_grid_profile() &&
        localized_lateral_profile_.pass_safety_approved_once &&
        !localized_lateral_profile_.pass_execution_committed &&
        !localized_lateral_profile_.pass_complete_confirmed;
    output_built.blocked_info.maneuver_transaction_pass_type =
        localized_lateral_profile_.pass_type;
    if (published_same_pass_transaction && blocked.start_grid_target_active &&
        !start_grid_target_id_.empty() &&
        localized_lateral_profile_.target_id == start_grid_target_id_) {
      start_grid_lateral_release_pending_ = true;
      output_built.blocked_info.start_grid_lateral_release_pending = true;
      output_built.blocked_info.start_grid_hold_target_d_m =
          start_grid_lateral_anchor_d_m_;
    }
    localized_lateral_profile_.pass_tracking_proof_published_last_cycle =
        transaction_incomplete && output_built.maneuver_latch_active &&
        output_built.maneuver_latch_target_id ==
            localized_lateral_profile_.target_id &&
        output_built.selected == localized_lateral_profile_.pass_type &&
        output_built.active_override &&
        output_built.solver_horizon_intent !=
            PlannerOutput::SolverHorizonIntent::NONE;
  }
  const bool tracking_release_snapshot_replacement =
      start_grid_tracking_release_pending_ &&
      output_built.tracking_release_pass_warmup;
  const bool published_committed_pass_snapshot =
      (!committed_pass_spatial_profile_frozen_ ||
       tracking_release_snapshot_replacement) &&
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      output_built.active_override &&
      output_built.selected == localized_lateral_profile_.pass_type &&
      (output_built.selected == CandidateType::PASS_LEFT ||
       output_built.selected == CandidateType::PASS_RIGHT) &&
      output_built.maneuver_latch_active &&
      output_built.maneuver_latch_target_id ==
          localized_lateral_profile_.target_id &&
      output_built.selected_lateral_profile_safety_verified &&
      !output_built.published_lateral_safety_rejected &&
      output_built.lateral_stop_inputs_complete &&
      output_built.lateral_offsets.size() == output_built.speed_caps.size() &&
      output_built.lateral_offsets.size() ==
          output_built.longitudinal_offsets_m.size() &&
      output_built.lateral_offsets.size() == output_selected.d.size() &&
      output_selected.type == localized_lateral_profile_.pass_type &&
      !output_built.blocked_info.committed_pass_spatial_profile_continuity_used;
  if (published_committed_pass_snapshot) {
    // raw候補ではなく、rate limit/hold後にNodeが実publishするd/v/dsを保存する。
    // 次周期に使う際もfresh縦profileへdだけを移し、追従性と全相手予測を
    // 再評価するため、ここで得た過去のSafetyEvaluator結果は継続権限にしない。
    CandidateTrajectory published_snapshot = output_selected;
    published_snapshot.d = output_built.lateral_offsets;
    published_snapshot.v_ref = output_built.speed_caps;
    published_snapshot.longitudinal_offsets_m =
        output_built.longitudinal_offsets_m;
    const bool snapshot_rebased =
        rebaseCandidateToCurrentEgo(published_snapshot, ego, frame_);
    const bool snapshot_trackable =
        snapshot_rebased &&
        CandidateBuilder(frame_, config_)
            .publishedLateralProfileTrackable(published_snapshot, ego);
    published_snapshot.controller_tracking_profile_valid = snapshot_trackable;
    if (snapshot_trackable) {
      evaluateCandidateAtOwnTimeAxis(published_snapshot, opponents, now_sec,
                                     predictions);
    }
    const bool snapshot_freshly_authorized =
        published_snapshot.safety_evaluated && published_snapshot.feasible &&
        published_snapshot.longitudinal_profile_valid &&
        published_snapshot.controller_tracking_profile_valid &&
        published_snapshot.pass_target_corridor_valid;
    constexpr double kPublishedWireBoundaryToleranceM = 0.02;
    const bool snapshot_extension_state_complete =
        !published_snapshot.longitudinal_offsets_m.empty() &&
        !published_snapshot.d.empty() &&
        published_snapshot.longitudinal_offsets_m.size() ==
            published_snapshot.d.size() &&
        std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) &&
        std::isfinite(published_snapshot.longitudinal_offsets_m.back()) &&
        std::isfinite(published_snapshot.d.back());
    const double transaction_extension_boundary_d_m =
        snapshot_extension_state_complete
            ? CandidateBuilder(frame_, config_)
                  .nominalLocalizedProfileDAtUnwrappedS(
                      localized_lateral_profile_,
                      localized_lateral_profile_.ego_unwrapped_s_m +
                          published_snapshot.longitudinal_offsets_m.back())
            : std::numeric_limits<double>::quiet_NaN();
    const bool snapshot_extension_boundary_continuous =
        snapshot_extension_state_complete &&
        std::isfinite(transaction_extension_boundary_d_m) &&
        std::abs(transaction_extension_boundary_d_m -
                 published_snapshot.d.back()) <=
            kPublishedWireBoundaryToleranceM;
    if (snapshot_freshly_authorized && snapshot_extension_boundary_continuous) {
      // 最初にPPへpublishした有限wireを即固定する。target dへ届くまでrolling
      // current-d候補を上書きし続けると、残距離が短くなるほど横移動が圧縮され、
      // 後から固定した瞬間に実車との誤差が跳ねる。tracking STOP後だけは新token
      // warm-upの実publish wireへ置換する。どちらも有限prefixと同一transaction
      // profileの境界連続性を確認し、fresh安全評価は毎周期やり直す。
      committed_pass_snapshot_valid_ = true;
      committed_pass_spatial_profile_frozen_ = true;
      committed_pass_snapshot_target_id_ = localized_lateral_profile_.target_id;
      committed_pass_snapshot_pass_type_ = localized_lateral_profile_.pass_type;
      committed_pass_snapshot_sec_ = now_sec;
      committed_pass_snapshot_ego_unwrapped_s_m_ =
          localized_lateral_profile_.ego_unwrapped_s_m;
      committed_pass_snapshot_ = std::move(published_snapshot);
      committed_pass_profile_snapshot_ = localized_lateral_profile_;
    }
  }
  output_built.attack_follow_inner_band_diagnostic =
      attack_follow_inner_band_diagnostic_;
  output_built.state_lattice_shadow_comparison =
      std::move(state_lattice_shadow_comparison);
  // STOP/HOLDの横authorityは、CandidateBuilderが同一d/v/dsを全相手で評価し、
  // PP必要arcまで証明したpayloadだけに限定する。rate limit、recovery置換、
  // tracking-release fallbackなどの後段mutationで束縛が切れた場合も、return
  // 直前にspeed-only STOPへ閉じる。
  const bool release_pending_pass_payload_requires_proof =
      output_built.blocked_info.maneuver_transaction_tracking_release_pending &&
      output_built.active_override &&
      (output_built.selected == CandidateType::PASS_LEFT ||
       output_built.selected == CandidateType::PASS_RIGHT);
  enforceStopLateralSpatialHorizonContract(
      output_built, output_selected,
      output_built.lateral_tracking_authorized_during_stop ||
          release_pending_pass_payload_requires_proof,
      config_.safe_stop_v_mps, config_.horizon_points);
  rememberGenericRecoveryHold(output_built);
  rememberPublishedLateralTarget(now_sec, output_built);
  return output_built;
}

// 入力: 相手車一覧と現在時刻。
// 出力: 各相手車の等速予測軌道。
// 処理概要: V2X速度推定を使い、planner
// horizon上の相手位置をFrenet/Cartesianで並べる。
std::vector<PredictedOpponent> OvertakePlannerCore::predictOpponents(
    const std::vector<OpponentState> &opponents, double now_sec,
    const std::vector<double> *time_points) const {
  // V2X位置から推定した速度を使い、短いhorizonでは等速直線運動として予測する。
  std::vector<PredictedOpponent> out;
  for (const auto &opp : opponents) {
    if (!opp.valid ||
        !inputTimestampFresh(now_sec, opp.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec)) {
      continue;
    }
    PredictedOpponent pred;
    pred.id = opp.id;
    const std::size_t count =
        time_points == nullptr ? config_.horizon_points : time_points->size();
    for (std::size_t i = 0; i < count; ++i) {
      const double t = time_points == nullptr
                           ? static_cast<double>(i) * config_.horizon_dt_sec
                           : (*time_points)[i];
      if (!std::isfinite(t) || t < 0.0) {
        continue;
      }
      const double x = opp.x + opp.vx * t;
      const double y = opp.y + opp.vy * t;
      const auto fr = frame_.cartesianToFrenet(x, y, 0.0);
      pred.t.push_back(t);
      pred.x.push_back(x);
      pred.y.push_back(y);
      pred.s.push_back(fr.s);
      pred.d.push_back(fr.d);
    }
    out.push_back(std::move(pred));
  }
  return out;
}

// 入力: 候補、fresh相手一覧、現在時刻、通常planner時刻列で生成済みの相手予測。
// 出力: 候補固有の時刻列と一致する相手予測でのSafetyEvaluator結果。
// 処理概要: 通常1.225秒horizonは既存予測を再利用する。低速ATTACK_FOLLOW等で
// candidate.tが延長された場合だけ、その同じ時刻列から全相手を再予測する。
// candidateだけを長くして相手予測を旧indexのまま使うfail-open/常時rejectを防ぐ。
bool OvertakePlannerCore::evaluateCandidateAtOwnTimeAxis(
    CandidateTrajectory &candidate, const std::vector<OpponentState> &opponents,
    double now_sec,
    const std::vector<PredictedOpponent> &default_predictions) const {
  const bool uses_default_time_axis =
      candidate.t.size() == config_.horizon_points &&
      std::all_of(candidate.t.begin(), candidate.t.end(),
                  [this, index = std::size_t{0U}](double value) mutable {
                    const double expected =
                        static_cast<double>(index++) * config_.horizon_dt_sec;
                    return std::isfinite(value) &&
                           std::abs(value - expected) <= 1.0e-9;
                  });
  const bool safety_evaluated =
      uses_default_time_axis
          ? safety_.evaluate(candidate, default_predictions)
          : safety_.evaluate(
                candidate, predictOpponents(opponents, now_sec, &candidate.t));
  candidate.controller_spatial_horizon_proof_valid =
      safety_evaluated && hasControllerSpatialHorizonProof(candidate);
  return safety_evaluated;
}

bool OvertakePlannerCore::evaluateShadowCandidate(
    CandidateTrajectory &candidate, const std::vector<OpponentState> &opponents,
    double now_sec) const {
  if (!std::isfinite(now_sec) || candidate.t.size() < 2U ||
      candidate.t.size() != candidate.d.size() ||
      candidate.t.size() != candidate.longitudinal_offsets_m.size() ||
      candidate.t.size() != candidate.predicted_speed_mps.size() ||
      candidate.t.size() != candidate.v_ref.size()) {
    candidate.feasible = false;
    candidate.safety_evaluated = false;
    candidate.reject_reason = "shadow_candidate_time_axis_invalid";
    return false;
  }
  return evaluateShadowCandidate(
      candidate, opponents, now_sec,
      predictOpponents(opponents, now_sec, &candidate.t));
}

bool OvertakePlannerCore::evaluateShadowCandidate(
    CandidateTrajectory &candidate, const std::vector<OpponentState> &opponents,
    double now_sec, const std::vector<PredictedOpponent> &predictions) const {
  // A lattice adapter has already supplied the current candidate's exact
  // longitudinal time/speed profile.  Empty/nonfinite inputs are not allowed
  // to fall through into a shortened prediction horizon.
  if (!std::isfinite(now_sec) || candidate.t.size() < 2U ||
      candidate.t.size() != candidate.d.size() ||
      candidate.t.size() != candidate.longitudinal_offsets_m.size() ||
      candidate.t.size() != candidate.predicted_speed_mps.size() ||
      candidate.t.size() != candidate.v_ref.size() ||
      predictions.size() != opponents.size()) {
    candidate.feasible = false;
    candidate.safety_evaluated = false;
    candidate.reject_reason = "shadow_candidate_time_axis_invalid";
    return false;
  }
  const bool safety_evaluated = safety_.evaluate(candidate, predictions);
  candidate.controller_spatial_horizon_proof_valid =
      safety_evaluated && hasControllerSpatialHorizonProof(candidate);
  return safety_evaluated;
}

StateLatticeShadowComparison
OvertakePlannerCore::evaluateStateLatticeShadowComparison(
    const CandidateTrajectory &current_candidate, const EgoState &ego,
    const std::string &target_id, const std::vector<OpponentState> &opponents,
    double now_sec, const std::vector<PredictedOpponent> &predictions,
    bool snapshot_complete,
    const PurePursuitExactSnapshot *pp_exact_snapshot) const {
  StateLatticeShadowComparison comparison;
  comparison.requested = true;
  comparison.snapshot_complete = snapshot_complete;
  comparison.status_reason = "not_evaluated";
  comparison.target_id = target_id;
  comparison.pass_type = current_candidate.type;
  comparison.pass_side = current_candidate.type == CandidateType::PASS_LEFT ? 1
                         : current_candidate.type == CandidateType::PASS_RIGHT
                             ? -1
                             : 0;
  comparison.current_pp_exact_snapshot_reason = "missing";
  if (pp_exact_snapshot != nullptr && pp_exact_snapshot->valid) {
    if (!std::isfinite(now_sec) ||
        !std::isfinite(pp_exact_snapshot->command_stamp_sec) ||
        !std::isfinite(pp_exact_snapshot->valid_until_sec) ||
        !std::isfinite(config_.input_future_stamp_tolerance_sec) ||
        config_.input_future_stamp_tolerance_sec < 0.0 ||
        pp_exact_snapshot->valid_until_sec <=
            pp_exact_snapshot->command_stamp_sec ||
        now_sec < pp_exact_snapshot->command_stamp_sec -
                      config_.input_future_stamp_tolerance_sec) {
      comparison.current_pp_exact_snapshot_reason =
          "command_stamp_future_or_invalid";
    } else if (now_sec > pp_exact_snapshot->valid_until_sec) {
      comparison.current_pp_exact_snapshot_reason = "lease_expired";
    } else if (pp_exact_snapshot->binding.target_vehicle_id != target_id ||
               pp_exact_snapshot->binding.pass_direction !=
                   (current_candidate.type == CandidateType::PASS_LEFT ? 1
                    : current_candidate.type == CandidateType::PASS_RIGHT
                        ? -1
                        : 0)) {
      comparison.current_pp_exact_snapshot_reason =
          "current_candidate_binding_mismatch";
    } else {
      comparison.current_pp_exact_snapshot_valid = true;
      comparison.current_pp_exact_snapshot_reason = "complete";
      comparison.current_pp_command_sequence =
          pp_exact_snapshot->command_sequence;
      comparison.current_pp_valid_until_sec =
          pp_exact_snapshot->valid_until_sec;
      comparison.current_pp_resolved_lookahead_m =
          pp_exact_snapshot->evaluator_input.active_lookahead_distance_m;
      comparison.current_pp_requested_steering_rad =
          pp_exact_snapshot->requested_output_steering_tire_angle_rad;
      comparison.current_pp_bounded_steering_rad =
          pp_exact_snapshot->bounded_steering_tire_angle_rad;
    }
  }
  comparison.snapshot_cycle = supervisor_v2_cycle_sequence_;
  comparison.snapshot_stamp_sec = now_sec;
  comparison.ego_stamp_sec = ego.stamp_sec;
  comparison.opponent_count = opponents.size();
  comparison.snapshot_hash = stateLatticeSnapshotHash(
      comparison.snapshot_cycle, now_sec, ego, opponents, predictions);
  comparison.current = stateLatticeShadowMetrics(current_candidate);

  const std::size_t point_count = current_candidate.t.size();
  const bool current_geometry_complete =
      comparison.pass_side != 0 && !target_id.empty() && point_count >= 3U &&
      point_count <= 256U && current_candidate.x.size() == point_count &&
      current_candidate.y.size() == point_count &&
      current_candidate.yaw.size() == point_count &&
      current_candidate.s.size() == point_count &&
      std::isfinite(current_candidate.planned_target_d_m);
  if (!current_geometry_complete) {
    comparison.status_reason = "current_pass_geometry_incomplete";
    comparison.lattice.first_reject_reason = comparison.status_reason;
    return comparison;
  }

  state_lattice_overtake_planner::ShadowGeometryRequest request;
  request.start = {current_candidate.x.front(), current_candidate.y.front(),
                   current_candidate.yaw.front()};
  request.goal = {current_candidate.x.back(), current_candidate.y.back(),
                  current_candidate.yaw.back()};
  request.start_curvature =
      frame_.interpolate(current_candidate.s.front()).kappa;
  request.goal_curvature = frame_.interpolate(current_candidate.s.back()).kappa;
  request.tangent_scale = 1.0;
  request.target_id = target_id;
  request.pass_side = comparison.pass_side;
  request.target_d_m = current_candidate.planned_target_d_m;
  request.sample_count = point_count;
  const auto geometry =
      state_lattice_overtake_planner::ShadowGeometryGenerator{}.generate(
          request);
  comparison.geometry_generated = geometry.valid;
  if (!geometry.valid) {
    comparison.status_reason = geometry.reason;
    comparison.lattice.first_reject_reason = geometry.reason;
    return comparison;
  }

  const auto adapted = StateLatticeShadowAdapter(frame_).adapt(
      geometry, current_candidate, current_candidate.type, target_id,
      comparison.pass_side, current_candidate.planned_target_d_m);
  comparison.adapter_valid = adapted.valid;
  if (!adapted.valid) {
    comparison.status_reason = adapted.reason;
    comparison.lattice.first_reject_reason = adapted.reason;
    return comparison;
  }

  CandidateTrajectory lattice_candidate = adapted.candidate;
  lattice_candidate.longitudinal_initial_measured_speed_mps =
      current_candidate.longitudinal_initial_measured_speed_mps;
  lattice_candidate.pass_target_corridor_valid =
      current_candidate.pass_target_corridor_valid;
  lattice_candidate.pass_transition_deadline_reachable = false;
  lattice_candidate.required_pass_transition_m =
      std::numeric_limits<double>::infinity();
  lattice_candidate.available_pass_transition_deadline_m =
      current_candidate.available_pass_transition_deadline_m;
  lattice_candidate.pass_transition_deadline =
      current_candidate.pass_transition_deadline;
  lattice_candidate.moving_target_relatively_reachable =
      current_candidate.moving_target_relatively_reachable;
  lattice_candidate.longitudinal_profile_valid =
      current_candidate.longitudinal_profile_valid;
  lattice_candidate.assumed_brake_decel_mps2 =
      current_candidate.assumed_brake_decel_mps2;
  lattice_candidate.response_delay_sec = current_candidate.response_delay_sec;
  lattice_candidate.required_brake_distance_m =
      current_candidate.required_brake_distance_m;
  lattice_candidate.available_brake_distance_m =
      current_candidate.available_brake_distance_m;
  lattice_candidate.required_controller_spatial_horizon_m =
      current_candidate.required_controller_spatial_horizon_m;

  CartesianTrackabilityConfig trackability_config;
  trackability_config.wheelbase_m = config_.attack_follow_tracking_wheelbase_m;
  trackability_config.steering_gain =
      config_.attack_follow_steering_tire_angle_gain;
  trackability_config.max_steering_angle_rad =
      config_.attack_follow_max_steering_angle_rad;
  trackability_config.max_steering_rate_radps =
      config_.attack_follow_max_steering_rate_radps *
      config_.attack_follow_steering_rate_reserve_ratio;
  trackability_config.pure_pursuit_required_arc_m =
      current_candidate.required_controller_spatial_horizon_m;
  trackability_config.target_d_m = current_candidate.planned_target_d_m;
  // deadline未設定(infinity)を無制限として通さない。shadow比較が実際に持つ
  // current candidateの検証済み空間終端を、明示かつ保守的な上限にする。
  trackability_config.target_d_deadline_arc_m =
      std::isfinite(current_candidate.available_pass_transition_deadline_m)
          ? current_candidate.available_pass_transition_deadline_m
          : (!current_candidate.longitudinal_offsets_m.empty() &&
                     std::isfinite(
                         current_candidate.longitudinal_offsets_m.back())
                 ? current_candidate.longitudinal_offsets_m.back()
                 : std::numeric_limits<double>::quiet_NaN());
  CartesianTrackabilityInput trackability_input;
  trackability_input.ego = ego;
  // Coreにはactive PPがadaptive/smoothing後に使ったlookahead snapshotも、
  // angle/rate boundのreferenceにしたfresh steering snapshotも無い。
  // gain/minや0 radを推測してexact command proofへ昇格させない。
  const CartesianTrackabilityResult trackability =
      CartesianTrackabilityEvaluator(frame_).evaluate(
          lattice_candidate, trackability_input, trackability_config);
  comparison.cartesian_trackability_valid = trackability.valid;
  comparison.cartesian_trackable = trackability.trackable;
  lattice_candidate.controller_tracking_profile_valid =
      trackability.valid && trackability.desired_path_trackable;
  lattice_candidate.desired_path_trackable =
      trackability.valid && trackability.desired_path_trackable;
  lattice_candidate.pure_pursuit_command_trackable =
      trackability.valid && trackability.pure_pursuit_command_trackable;
  lattice_candidate.pass_transition_deadline_reachable =
      trackability.valid && std::isfinite(trackability.target_d_reach_arc_m) &&
      trackability.target_d_reach_arc_m <=
          trackability_config.target_d_deadline_arc_m;
  lattice_candidate.required_pass_transition_m =
      trackability.target_d_reach_arc_m;
  comparison.lattice.first_reject_reason =
      trackability.trackable ? std::string{} : trackability.reason;

  const auto exact_predictions =
      predictOpponents(opponents, now_sec, &lattice_candidate.t);
  const auto finite_vector = [](const std::vector<double> &values) {
    return std::all_of(values.cbegin(), values.cend(),
                       [](double value) { return std::isfinite(value); });
  };
  const bool opponent_ids_unique = [&opponents]() {
    for (std::size_t first = 0U; first < opponents.size(); ++first) {
      for (std::size_t second = first + 1U; second < opponents.size();
           ++second) {
        if (opponents[first].id == opponents[second].id) {
          return false;
        }
      }
    }
    return true;
  }();
  const bool observed_snapshot_complete =
      ego.valid && std::isfinite(now_sec) && std::isfinite(ego.stamp_sec) &&
      std::isfinite(ego.x) && std::isfinite(ego.y) && std::isfinite(ego.yaw) &&
      std::isfinite(ego.v) && std::isfinite(ego.frenet.s) &&
      std::isfinite(ego.frenet.d) &&
      std::all_of(opponents.cbegin(), opponents.cend(),
                  [this, now_sec](const OpponentState &opponent) {
                    return opponent.valid && !opponent.id.empty() &&
                           inputTimestampFresh(
                               now_sec, opponent.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec) &&
                           std::isfinite(opponent.x) &&
                           std::isfinite(opponent.y) &&
                           std::isfinite(opponent.vx) &&
                           std::isfinite(opponent.vy) &&
                           std::isfinite(opponent.v) &&
                           std::isfinite(opponent.frenet.s) &&
                           std::isfinite(opponent.frenet.d);
                  }) &&
      opponent_ids_unique;
  const bool exact_prediction_snapshot_complete =
      observed_snapshot_complete &&
      exact_predictions.size() == opponents.size() &&
      std::equal(
          exact_predictions.cbegin(), exact_predictions.cend(),
          opponents.cbegin(),
          [&](const PredictedOpponent &prediction,
              const OpponentState &opponent) {
            return prediction.id == opponent.id &&
                   prediction.t.size() == point_count &&
                   prediction.x.size() == point_count &&
                   prediction.y.size() == point_count &&
                   prediction.s.size() == point_count &&
                   prediction.d.size() == point_count &&
                   finite_vector(prediction.t) && finite_vector(prediction.x) &&
                   finite_vector(prediction.y) && finite_vector(prediction.s) &&
                   finite_vector(prediction.d) &&
                   std::equal(prediction.t.cbegin(), prediction.t.cend(),
                              lattice_candidate.t.cbegin(),
                              [](double prediction_t, double candidate_t) {
                                return std::abs(prediction_t - candidate_t) <=
                                       1.0e-9;
                              });
          });
  comparison.snapshot_complete =
      comparison.snapshot_complete && exact_prediction_snapshot_complete;
  comparison.snapshot_hash = stateLatticeSnapshotHash(
      comparison.snapshot_cycle, now_sec, ego, opponents, exact_predictions);
  if (!comparison.snapshot_complete) {
    comparison.status_reason = "snapshot_incomplete";
    const std::string first_reject_reason =
        comparison.lattice.first_reject_reason;
    comparison.lattice = stateLatticeShadowMetrics(lattice_candidate);
    comparison.lattice.pure_pursuit_available_arc_m = trackability.total_arc_m;
    comparison.lattice.deadline_required_arc_m =
        trackability.target_d_reach_arc_m;
    comparison.lattice.deadline_available_arc_m =
        trackability_config.target_d_deadline_arc_m;
    comparison.lattice.deadline_slack_m =
        comparison.lattice.deadline_available_arc_m -
        comparison.lattice.deadline_required_arc_m;
    comparison.lattice.first_reject_reason = first_reject_reason.empty()
                                                 ? "snapshot_incomplete"
                                                 : first_reject_reason;
    return comparison;
  }

  evaluateShadowCandidate(lattice_candidate, opponents, now_sec,
                          exact_predictions);
  comparison.evaluated = lattice_candidate.safety_evaluated;
  comparison.status_reason = comparison.evaluated
                                 ? "comparison_recorded"
                                 : "shadow_safety_evaluation_failed";
  const std::string first_reject_reason =
      comparison.lattice.first_reject_reason;
  comparison.lattice = stateLatticeShadowMetrics(lattice_candidate);
  comparison.lattice.pure_pursuit_available_arc_m = trackability.total_arc_m;
  comparison.lattice.deadline_required_arc_m =
      trackability.target_d_reach_arc_m;
  comparison.lattice.deadline_available_arc_m =
      trackability_config.target_d_deadline_arc_m;
  comparison.lattice.deadline_slack_m =
      comparison.lattice.deadline_available_arc_m -
      comparison.lattice.deadline_required_arc_m;
  if (!first_reject_reason.empty()) {
    comparison.lattice.first_reject_reason = first_reject_reason;
  }
  return comparison;
}

void OvertakePlannerCore::evaluateAttackFollowInnerBandDiagnostic(
    const CandidateTrajectory &source,
    const CandidateTrajectory &current_d_hold, const EgoState &ego,
    const std::vector<OpponentState> &opponents, double now_sec,
    const std::vector<PredictedOpponent> &default_predictions) {
  constexpr double kProbeIntervalSec = 1.0;
  constexpr std::array<double, AttackFollowInnerBandDiagnostic::kMaxProbeCount>
      kInwardProbeDeltasM{0.15, 0.30, 0.45, 0.60};
  constexpr double kBoundaryAmbiguityM = 1.0e-6;
  constexpr double kCenterReserveM = 0.05;

  const bool same_identity = attack_follow_inner_band_diagnostic_.target_id ==
                                 localized_lateral_profile_.target_id &&
                             attack_follow_inner_band_diagnostic_.pass_type ==
                                 localized_lateral_profile_.pass_type;
  if (same_identity &&
      std::isfinite(last_attack_follow_inner_band_probe_sec_) &&
      std::isfinite(now_sec) &&
      now_sec - last_attack_follow_inner_band_probe_sec_ < kProbeIntervalSec) {
    attack_follow_inner_band_diagnostic_.rate_limited = true;
    return;
  }

  AttackFollowInnerBandDiagnostic diagnostic;
  diagnostic.evaluated = true;
  diagnostic.source_stamp_sec = now_sec;
  diagnostic.target_id = localized_lateral_profile_.target_id;
  diagnostic.pass_type = localized_lateral_profile_.pass_type;
  diagnostic.source_current_d_m = ego.frenet.d;
  diagnostic.committed_target_d_m = source.committed_attack_follow_target_d_m;
  last_attack_follow_inner_band_probe_sec_ = now_sec;

  const bool wall_signature_valid =
      current_d_hold.safety_evaluated && !current_d_hold.feasible &&
      current_d_hold.reject_reason == "wall_footprint_margin" &&
      current_d_hold.blocking_wall_footprint_valid &&
      std::isfinite(current_d_hold.blocking_wall_corner_d_m) &&
      std::isfinite(current_d_hold.blocking_wall_corridor_d_min_m) &&
      std::isfinite(current_d_hold.blocking_wall_corridor_d_max_m) &&
      std::isfinite(ego.frenet.d);
  if (!wall_signature_valid) {
    diagnostic.status_reason = "invalid_wall_reject_signature";
    diagnostic.other_reject_count = 1;
    attack_follow_inner_band_diagnostic_ = std::move(diagnostic);
    return;
  }

  const double lower_distance_m =
      std::abs(current_d_hold.blocking_wall_corner_d_m -
               current_d_hold.blocking_wall_corridor_d_min_m);
  const double upper_distance_m =
      std::abs(current_d_hold.blocking_wall_corridor_d_max_m -
               current_d_hold.blocking_wall_corner_d_m);
  if (!std::isfinite(lower_distance_m) || !std::isfinite(upper_distance_m) ||
      std::abs(lower_distance_m - upper_distance_m) <= kBoundaryAmbiguityM) {
    diagnostic.status_reason = "ambiguous_wall_boundary";
    diagnostic.other_reject_count = 1;
    attack_follow_inner_band_diagnostic_ = std::move(diagnostic);
    return;
  }
  diagnostic.inward_direction_sign =
      upper_distance_m < lower_distance_m ? -1 : 1;

  bool feasible_sample_seen = false;
  for (const double inward_delta_m : kInwardProbeDeltasM) {
    double terminal_d_m =
        ego.frenet.d +
        static_cast<double>(diagnostic.inward_direction_sign) * inward_delta_m;
    if (ego.frenet.d > kCenterReserveM && terminal_d_m < kCenterReserveM) {
      terminal_d_m = kCenterReserveM;
    } else if (ego.frenet.d < -kCenterReserveM &&
               terminal_d_m > -kCenterReserveM) {
      terminal_d_m = -kCenterReserveM;
    }
    if (diagnostic.evaluated_probe_count > 0 &&
        std::abs(terminal_d_m - diagnostic
                                    .probes[static_cast<std::size_t>(
                                        diagnostic.evaluated_probe_count - 1)]
                                    .terminal_d_m) <= 1.0e-9) {
      continue;
    }

    auto &probe_report =
        diagnostic
            .probes[static_cast<std::size_t>(diagnostic.evaluated_probe_count)];
    probe_report.inward_delta_m = std::abs(terminal_d_m - ego.frenet.d);
    probe_report.terminal_d_m = terminal_d_m;
    CandidateTrajectory probe = CandidateBuilder(frame_, config_)
                                    .makeAttackFollowInnerBandDiagnosticVariant(
                                        source, ego, terminal_d_m);
    probe_report.longitudinal_contract_unchanged =
        probe.t == source.t &&
        probe.longitudinal_offsets_m == source.longitudinal_offsets_m &&
        probe.s == source.s &&
        probe.predicted_speed_mps == source.predicted_speed_mps &&
        probe.v_ref == source.v_ref &&
        probe.committed_attack_follow_target_d_m ==
            source.committed_attack_follow_target_d_m &&
        probe.planned_target_d_m == source.planned_target_d_m;
    if (!probe_report.longitudinal_contract_unchanged) {
      probe.feasible = false;
      probe.controller_tracking_profile_valid = false;
      probe.reject_reason = "diagnostic_longitudinal_contract_mismatch";
    } else if (probe.pass_target_corridor_valid &&
               probe.controller_tracking_profile_valid) {
      evaluateCandidateAtOwnTimeAxis(probe, opponents, now_sec,
                                     default_predictions);
    }

    probe_report.safety_evaluated = probe.safety_evaluated;
    probe_report.corridor_valid = probe.pass_target_corridor_valid;
    probe_report.controller_tracking_profile_valid =
        probe.controller_tracking_profile_valid;
    probe_report.desired_path_trackable = probe.desired_path_trackable;
    probe_report.pure_pursuit_command_trackable =
        probe.pure_pursuit_command_trackable;
    probe_report.min_safety_margin = probe.min_safety_margin;
    probe_report.corridor_min_margin_m = probe.corridor_min_margin_m;
    probe_report.all_checks_pass =
        probe_report.longitudinal_contract_unchanged &&
        probe.safety_evaluated && probe.feasible &&
        probe.longitudinal_profile_valid && probe.pass_target_corridor_valid &&
        probe.controller_tracking_profile_valid &&
        probe.desired_path_trackable && probe.pure_pursuit_command_trackable;
    if (probe_report.all_checks_pass) {
      probe_report.reject_reason.clear();
      ++diagnostic.feasible_probe_count;
      if (!feasible_sample_seen) {
        diagnostic.outermost_feasible_d_m = terminal_d_m;
        feasible_sample_seen = true;
      }
      diagnostic.innermost_feasible_d_m = terminal_d_m;
    } else {
      if (!probe_report.longitudinal_contract_unchanged) {
        probe_report.reject_reason =
            "diagnostic_longitudinal_contract_mismatch";
      } else if (!probe.pass_target_corridor_valid) {
        probe_report.reject_reason = "pass_target_unreachable";
      } else if (!probe.controller_tracking_profile_valid) {
        probe_report.reject_reason = "untrackable_lateral_profile";
      } else if (!probe.longitudinal_profile_valid) {
        probe_report.reject_reason = "invalid_longitudinal_brake_model";
      } else if (!probe.safety_evaluated) {
        probe_report.reject_reason = "diagnostic_safety_not_evaluated";
      } else {
        probe_report.reject_reason = probe.reject_reason;
      }
      if (probe_report.reject_reason == "wall_footprint_margin") {
        ++diagnostic.wall_reject_count;
      } else if (probe_report.reject_reason == "opponent_collision") {
        ++diagnostic.opponent_reject_count;
      } else {
        ++diagnostic.other_reject_count;
      }
    }
    ++diagnostic.evaluated_probe_count;
  }
  diagnostic.complete = diagnostic.evaluated_probe_count > 0;
  diagnostic.status_reason = diagnostic.complete ? "complete" : "no_probe";
  attack_follow_inner_band_diagnostic_ = std::move(diagnostic);
}

// 入力: 自車、現在のBlockedInfo、相手車一覧。
// 出力: publish用horizonより長い、通常ラインまで戻るRECOVERY評価軌道。
// 処理概要: 短いMPC
// horizonだけが安全でも、merge終端で他車へ入る復帰を許可しない。
CandidateTrajectory OvertakePlannerCore::makeReentryEvaluationCandidate(
    const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  PlannerConfig evaluation_config = config_;
  // 通常publish horizonの25 ms刻みを24秒まで単純延長すると、停止近傍の
  // reentry判定だけで約1000点/車両を毎周期評価する。SafetyEvaluatorは点間の
  // 相対運動も補間検査するため、長期gateだけ50 ms以上へ間引いて終端まで見る。
  const double dt_sec =
      std::max(0.05, std::max(1.0e-3, config_.horizon_dt_sec));
  evaluation_config.horizon_dt_sec = dt_sec;

  const auto ego_bounds =
      frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
  const double lower_d = ego_bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = ego_bounds.d_max - config_.min_wall_margin_m;
  const bool outside_safe_corridor =
      ego.frenet.d < lower_d || ego.frenet.d > upper_d;
  const double base_shift_distance_m =
      outside_safe_corridor
          ? std::max(1.0,
                     0.5 * std::min(std::max(1.0, config_.merge_distance_m),
                                    std::max(1.0, config_.prepare_distance_m)))
          : std::max(1.0, config_.merge_distance_m);
  double recovery_evaluation_speed_mps =
      std::max(1.0e-3, config_.recovery_v_max_mps);
  if (outside_safe_corridor) {
    recovery_evaluation_speed_mps =
        std::min(recovery_evaluation_speed_mps,
                 std::max(1.0e-3, config_.wall_margin_recovery_v_max_mps));
  }
  if (config_.large_lateral_error_threshold_m >= 0.0 &&
      std::isfinite(ego.frenet.d) &&
      std::abs(ego.frenet.d) > config_.large_lateral_error_threshold_m &&
      config_.large_lateral_error_v_max_mps > 0.0) {
    recovery_evaluation_speed_mps = std::min(
        recovery_evaluation_speed_mps, config_.large_lateral_error_v_max_mps);
  }
  // 実候補が許し得る最大速度で全復帰距離を評価する。現在がそれより速い
  // 場合も初期速度を落とさず、制動profileを同じCandidateBuilderに作らせる。
  EgoState evaluation_ego = ego;
  evaluation_ego.v =
      std::max(std::max(0.0, ego.v), recovery_evaluation_speed_mps);
  const double full_reentry_sec =
      base_shift_distance_m / recovery_evaluation_speed_mps + dt_sec;
  const double horizon_sec =
      std::max({static_cast<double>(config_.horizon_points) *
                    std::max(1.0e-3, config_.horizon_dt_sec),
                std::max(0.0, config_.reentry_evaluation_horizon_sec),
                full_reentry_sec});
  evaluation_config.horizon_points = std::max<std::size_t>(
      config_.horizon_points,
      static_cast<std::size_t>(std::ceil(horizon_sec / dt_sec)) + 1U);
  // 実行時のRECOVERYが横誤差のrelease閾値で一時的に保持されていても、
  // ゲート評価は「通常ラインへ実際に戻る」最悪ケースを必ず検査する。
  evaluation_config.recovery_release_lateral_error_m = 0.0;
  // publish用のcurrent-d/anchor HOLDは、中心復帰が未認可の間に実車を
  // 横断させないための契約である。これをlong-horizon reentry評価へ
  // 引き継ぐと、中心復帰ではなくstart-grid anchorやPASS側dの保持列を評価し、
  // controller horizon不足をreentryのuntrackable判定として誤記録する。
  // 評価用コピーだけ保持文脈を解除し、実際にd=0へ収束するRECOVERY全体を
  // SafetyEvaluatorへ通す。publish側のHOLDフラグは変更しない。
  const BlockedInfo evaluation_blocked =
      centeringEvaluationBlockedInfo(blocked_info);
  return CandidateBuilder(frame_, evaluation_config)
      .makeCandidate(CandidateType::RECOVERY, evaluation_ego,
                     evaluation_blocked, opponents, nullptr);
}

// 入力: V2 shadowの現在状態、全相手、入力鮮度、MPC health。
// 出力: 長期horizonでSafetyEvaluator済みの中心復帰候補と、今周期の安全可否。
// 処理概要: legacy reentryのcounter/lockoutへ一切書き込まず、V2専用gateへ渡す
// pure assessmentだけを行う。複数のhold
// flagも明示解除してd=0への実軌道を評価する。
OvertakePlannerCore::SupervisorV2CenteringAssessment
OvertakePlannerCore::assessSupervisorV2Centering(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const ReentryInputStatus &reentry_input) const {
  SupervisorV2CenteringAssessment assessment;
  // legacy/V2で同じsanitize済みの中心復帰候補を使う。ここで別々にhold flagを
  // 管理すると片側だけがanchor保持を中心復帰と誤認するため、共通helperへ集約する。
  const BlockedInfo centering_blocked =
      centeringEvaluationBlockedInfo(blocked_info);
  assessment.candidate =
      makeReentryEvaluationCandidate(ego, centering_blocked, opponents);

  const auto predictions =
      predictOpponents(opponents, now_sec, &assessment.candidate.t);
  const std::size_t point_count = assessment.candidate.t.size();
  const bool candidate_shape_complete =
      point_count > 0U && assessment.candidate.x.size() == point_count &&
      assessment.candidate.y.size() == point_count &&
      assessment.candidate.yaw.size() == point_count &&
      assessment.candidate.d.size() == point_count &&
      assessment.candidate.s.size() == point_count &&
      assessment.candidate.v_ref.size() == point_count &&
      assessment.candidate.longitudinal_offsets_m.size() == point_count;
  const bool prediction_complete = predictions.size() == opponents.size();
  if (!candidate_shape_complete || !prediction_complete) {
    assessment.candidate.feasible = false;
    assessment.candidate.reject_reason =
        "incomplete_supervisor_v2_centering_prediction";
    return assessment;
  }

  safety_.evaluate(assessment.candidate, predictions);
  if (!assessment.candidate.longitudinal_profile_valid) {
    assessment.candidate.feasible = false;
    assessment.candidate.reject_reason = "invalid_longitudinal_brake_model";
  }

  const bool inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid &&
      reentry_input.pure_pursuit_primary_and_fresh &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  const bool ego_centered =
      std::isfinite(ego.frenet.d) &&
      std::abs(ego.frenet.d) <= reentryCompletionLateralErrorM(config_);
  const bool centerward_geometry =
      ego_centered || candidateMovesTowardCenter(ego, assessment.candidate);
  const bool trackable_centering_profile =
      CandidateBuilder(frame_, config_)
          .centeringProfileMatchesNominal(ego, centering_blocked,
                                          assessment.candidate);
  bool monotonic_centering = std::isfinite(ego.frenet.d);
  double previous_abs_d = std::abs(ego.frenet.d);
  for (const double candidate_d : assessment.candidate.d) {
    const bool crosses_center = ego.frenet.d > 0.0   ? candidate_d < -1.0e-6
                                : ego.frenet.d < 0.0 ? candidate_d > 1.0e-6
                                                     : false;
    if (!std::isfinite(candidate_d) || crosses_center ||
        std::abs(candidate_d) > previous_abs_d + 1.0e-6) {
      monotonic_centering = false;
      break;
    }
    previous_abs_d = std::abs(candidate_d);
  }
  const bool margin_safe =
      !std::isnan(assessment.candidate.min_safety_margin) &&
      assessment.candidate.min_safety_margin >=
          config_.reentry_min_safety_margin_h;
  const bool cbf_safe = std::isfinite(assessment.candidate.cbf_slack) &&
                        assessment.candidate.cbf_slack <= 0.0;
  const bool wall_state_safe =
      std::isfinite(blocked_info.ego_wall_clearance_m) &&
      blocked_info.ego_wall_clearance_m >= 0.0;
  const bool full_centering_evaluated =
      !assessment.candidate.d.empty() &&
      std::isfinite(assessment.candidate.d.back()) &&
      std::abs(assessment.candidate.d.back()) <=
          reentryCompletionLateralErrorM(config_);
  assessment.safe = inputs_complete && assessment.candidate.feasible &&
                    centerward_geometry && trackable_centering_profile &&
                    monotonic_centering && full_centering_evaluated &&
                    margin_safe && cbf_safe && wall_state_safe;
  if (!assessment.safe && assessment.candidate.reject_reason.empty()) {
    assessment.candidate.reject_reason =
        !inputs_complete       ? "incomplete_supervisor_v2_inputs"
        : !centerward_geometry ? "not_centerward_recovery"
        : !trackable_centering_profile
            ? "untrackable_supervisor_v2_centering_profile"
        : !monotonic_centering      ? "non_monotonic_centering"
        : !full_centering_evaluated ? "incomplete_reentry_centering_horizon"
        : !margin_safe              ? "reentry_margin_below_threshold"
        : !cbf_safe                 ? "reentry_cbf_slack"
        : !wall_state_safe          ? "unsafe_current_wall_clearance"
                                    : "supervisor_v2_centering_rejected";
  }
  return assessment;
}

// 入力: 自車状態。
// 出力: 現在のmodeで通常ラインへの横移動を始める/継続する必要があるならtrue。
// 処理概要:
// 通常走行や完全に中心へ戻った状態までV2X欠損で固定しないよう、復帰文脈だけを選ぶ。
bool OvertakePlannerCore::reentryRequested(const EgoState &ego) const {
  (void)ego;
  // PASSの横移動開始を中心線復帰と誤認しない。開始済みphaseは、物理的に
  // 中心へ戻った周期でもclear回数を満たすまで同じgateで評価する。
  return reentry_lockout_active_ || reentry_phase_active_ ||
         generic_recovery_phase_active_;
}

// 入力: 自車状態と安全評価済み候補。
// 出力: 候補横列が現在dより中心側へ実際に動くならtrue。
// 処理概要: 状態名だけではなくpublish予定のd列を直接確認し、中心への横断を
// 行わないSPEED_GUARD/FOLLOWには長期reentry gateを誤適用しない。
bool OvertakePlannerCore::candidateMovesTowardCenter(
    const EgoState &ego, const CandidateTrajectory &candidate) const {
  if (!candidate.feasible || !std::isfinite(ego.frenet.d) ||
      candidate.d.empty()) {
    return false;
  }
  const double current_abs_d = std::abs(ego.frenet.d);
  constexpr double kCenterwardMotionEpsilonM = 1.0e-3;
  if (current_abs_d <= kCenterwardMotionEpsilonM) {
    return false;
  }
  return std::any_of(candidate.d.begin(), candidate.d.end(),
                     [current_abs_d](double candidate_d) {
                       return std::isfinite(candidate_d) &&
                              std::abs(candidate_d) +
                                      kCenterwardMotionEpsilonM <
                                  current_abs_d;
                     });
}

// 入力: publish予定のPlannerOutput。
// 出力: なし。検証済みgeneric hold列があれば出力へ上書きする。
// 処理概要: V2X/MPC入力が欠けた周期でも、現在の候補を再計算して通常ラインへ
// 寄せず、最後に完全入力でpublish検証を通った横列と低速capを継続する。
void OvertakePlannerCore::applyGenericRecoveryStaleHold(
    PlannerOutput &output) const {
  const double no_safe_hold_cap_mps = std::min(
      std::max(1.0e-3, config_.reentry_hold_v_max_mps),
      std::min(
          std::max(1.0e-3, config_.safe_stop_v_mps),
          std::min(std::max(1.0e-3, config_.speed_only_fallback_v_max_mps),
                   std::max(1.0e-3,
                            config_.opponent_collision_fallback_v_max_mps))));
  if (generic_recovery_hold_offsets_.empty()) {
    // freshで安全なholdを作れなかった場合、stale入力だけで横列を推測しない。
    // Node/controller stale
    // watchdogがbaseline安全経路を選べるようoverrideを明示的に
    // 無効化し、診断上の速度capだけを低速側へ残す。
    output.mode = BehaviorMode::SPEED_GUARD;
    output.selected = CandidateType::RECOVERY;
    output.blocked_info.reentry_hold_active = true;
    output.active_override = false;
    output.longitudinal_speed_cap_active = true;
    output.lateral_offsets.clear();
    output.longitudinal_offsets_m.clear();
    output.speed_caps.assign(config_.horizon_points, no_safe_hold_cap_mps);
    output.applied_speed_cap_mps = no_safe_hold_cap_mps;
    output.speed_cap_reason =
        "generic_recovery_stale_input_no_safe_hold_watchdog";
    output.reason = "generic_recovery_stale_input_no_safe_hold_watchdog";
    return;
  }
  // stale V2X/MPCでは、保存済みholdが3 m/sであっても再利用しない。入力が
  // freshだった時の評価結果だけで継続高速化せず、hard fail-safe capへ閉じる。
  const double hold_cap_mps =
      std::min(std::isfinite(generic_recovery_hold_speed_cap_mps_) &&
                       generic_recovery_hold_speed_cap_mps_ > 0.0
                   ? generic_recovery_hold_speed_cap_mps_
                   : std::max(1.0e-3, config_.reentry_hold_v_max_mps),
               std::max(1.0e-3, config_.reentry_hold_v_max_mps));
  output.mode = BehaviorMode::SPEED_GUARD;
  output.selected = CandidateType::RECOVERY;
  output.blocked_info.reentry_hold_active = true;
  output.active_override = true;
  output.lateral_offsets = generic_recovery_hold_offsets_;
  // 保存時と異なる候補由来の距離軸をpublishしない。stale holdはv3へ落とす。
  output.longitudinal_offsets_m.clear();
  output.speed_caps.assign(output.lateral_offsets.size(), hold_cap_mps);
  output.target_lateral_offset_m = output.lateral_offsets.back();
  output.applied_speed_cap_mps = hold_cap_mps;
  output.speed_cap_reason = "generic_recovery_stale_input_hold";
  output.reason = "generic_recovery_stale_input_hold";
}

// 入力: fully freshな周期でSafetyEvaluatorを通したgeneric現d
// hold候補とgate結果。 出力:
// なし。次周期のstale時に使う横列と速度capを記憶する。 処理概要:
// gate許可時の中心復帰列とは分け、中心移動前に入力が欠けても必ず 現d
// holdへ落とせるよう、別途評価済みのRECOVERY holdだけを保存する。
void OvertakePlannerCore::rememberGenericRecoveryHold(
    const CandidateTrajectory &candidate, const ReentryGateResult &gate) {
  if (!generic_recovery_phase_active_ || !gate.input_complete ||
      !candidate.feasible || candidate.d.empty()) {
    return;
  }
  if (!isConstantLateralHold(candidate.d)) {
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }
  generic_recovery_hold_offsets_ = candidate.d;
  double min_speed_cap_mps = std::numeric_limits<double>::infinity();
  for (const double speed_cap_mps : candidate.v_ref) {
    if (std::isfinite(speed_cap_mps) && speed_cap_mps > 0.0) {
      min_speed_cap_mps = std::min(min_speed_cap_mps, speed_cap_mps);
    }
  }
  generic_recovery_hold_speed_cap_mps_ =
      std::isfinite(min_speed_cap_mps)
          ? min_speed_cap_mps
          : std::max(1.0e-3, config_.reentry_hold_v_max_mps);
}

// 入力: publish直前のPlannerOutput。
// 出力: なし。generic gate拒否時に再利用する検証済みhold列と速度capを更新する。
// 処理概要: ego/V2X/MPCのfreshnessが失われても、通常ラインへfall throughせず、
// 直前にSafetyEvaluatorとpublish再評価を通った現d保持列だけを出し続ける。
void OvertakePlannerCore::rememberGenericRecoveryHold(
    const PlannerOutput &output) {
  if (!generic_recovery_phase_active_ ||
      !output.blocked_info.reentry_hold_active || !output.active_override ||
      output.lateral_offsets.empty() || !output.reentry_gate.input_complete ||
      output.published_lateral_safety_rejected) {
    return;
  }
  if (!isConstantLateralHold(output.lateral_offsets)) {
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }
  generic_recovery_hold_offsets_ = output.lateral_offsets;
  double min_speed_cap_mps = std::numeric_limits<double>::infinity();
  for (double speed_cap_mps : output.speed_caps) {
    if (std::isfinite(speed_cap_mps) && speed_cap_mps > 0.0) {
      min_speed_cap_mps = std::min(min_speed_cap_mps, speed_cap_mps);
    }
  }
  generic_recovery_hold_speed_cap_mps_ =
      std::isfinite(min_speed_cap_mps)
          ? min_speed_cap_mps
          : std::max(1.0e-3, config_.reentry_hold_v_max_mps);
}

// 入力: 現在の自車横位置と直前/遷移後のbehavior mode。
// 出力: なし。通常ライン復帰を開始した文脈をラッチする。
// 処理概要: PASSの外向き横移動とFREE_RUN/FOLLOWのgeneric横ずれは除外する。
// 実PASS後のMERGE/YIELD/SAFE_STOPまたはpublish再評価失敗だけをphaseとして残し、
// permit済みかつ中心到達時にのみ解除する。
void OvertakePlannerCore::updateReentryPhase(const EgoState &ego,
                                             BehaviorMode previous_mode) {
  if (!config_.reentry_gate_enabled) {
    reentry_phase_active_ = false;
    reentry_lockout_active_ = false;
    reentry_gate_permitted_ = false;
    reentry_clear_cycles_ = 0;
    generic_recovery_phase_active_ = false;
    generic_recovery_hold_offsets_.clear();
    generic_recovery_hold_speed_cap_mps_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }
  const bool lateral_position_valid = std::isfinite(ego.frenet.d);
  const double abs_lateral_error =
      lateral_position_valid ? std::abs(ego.frenet.d)
                             : std::numeric_limits<double>::infinity();
  const bool incomplete_attack_follow_transaction =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !pass_reauthorization_lockout_active_;
  if (generic_recovery_phase_active_) {
    if (reentry_gate_permitted_ && lateral_position_valid &&
        abs_lateral_error <= reentryCompletionLateralErrorM(config_)) {
      generic_recovery_phase_active_ = false;
      reentry_gate_permitted_ = false;
      reentry_clear_cycles_ = 0;
      generic_recovery_hold_offsets_.clear();
      generic_recovery_hold_speed_cap_mps_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    // generic phase中はFREE_RUN/FOLLOWのmode変化で実復帰phaseへ昇格させない。
    return;
  }

  if (reentry_phase_active_ && reentry_gate_permitted_ &&
      lateral_position_valid &&
      abs_lateral_error <= reentryCompletionLateralErrorM(config_)) {
    // Gateの連続安全確認を通り、実際にも通常ラインへ収束したため、次の
    // FREE_RUN/FOLLOWを古い復帰phaseで拘束しない。
    reentry_phase_active_ = false;
    reentry_lockout_active_ = false;
    reentry_gate_permitted_ = false;
    reentry_clear_cycles_ = 0;
    return;
  }

  if (reentry_lockout_active_) {
    reentry_phase_active_ = true;
    return;
  }

  // PASS準備・実行中は回避側へ出る途中なので、ここで通常ライン復帰を開始しない。
  // permit済みで完了しきい値まで到達してphaseを解除した同一周期も、直前ABORTを
  // 根拠に再armしない。開始側も完了しきい値を共有して一回の解除を確定させる。
  if (!lateral_position_valid || incomplete_attack_follow_transaction ||
      abs_lateral_error <= reentryCompletionRearmLateralErrorM(config_) ||
      isPassMode(mode_)) {
    return;
  }

  const bool actual_reentry_context =
      isReentrySourceMode(mode_) || isReentrySourceMode(previous_mode);
  if (!reentry_phase_active_ && actual_reentry_context) {
    reentry_phase_active_ = true;
    reentry_gate_permitted_ = false;
    reentry_clear_cycles_ = 0;
  }
}

// 入力: Node側で分類したMPC healthの新規sample情報。
// 出力: 復帰に使うMPC health状態。
// 処理概要: CBFと無関係なsolve遅延だけを新規debug
// sample単位でhysteresis化する。 stale/infeasible/不明値はtimer周期を待たずhard
// fail-safeへ閉じる。
ReentryMpcHealthState OvertakePlannerCore::updateReentryMpcHealthState(
    const ReentryInputStatus &reentry_input) {
  if (!config_.reentry_require_mpc_health) {
    return ReentryMpcHealthState::HEALTHY;
  }

  // 既存core unit testの直接入力はsample sequenceを持たない。そこでは従来どおり
  // mpc_healthyだけを契約として使い、実ROS経路だけsample単位hysteresisを有効にする。
  if (reentry_input.mpc_health_sample_sequence == 0U) {
    return reentry_input.mpc_healthy ? ReentryMpcHealthState::HEALTHY
                                     : ReentryMpcHealthState::UNHEALTHY;
  }
  if (!reentry_input.mpc_health_fresh) {
    reentry_mpc_bad_sample_count_ = 0;
    reentry_mpc_good_sample_count_ = 0;
    reentry_mpc_health_state_ = ReentryMpcHealthState::STALE;
    return reentry_mpc_health_state_;
  }
  if (reentry_input.mpc_hard_failure ||
      (!reentry_input.mpc_healthy && !reentry_input.mpc_latency_warning)) {
    reentry_mpc_bad_sample_count_ = 0;
    reentry_mpc_good_sample_count_ = 0;
    reentry_mpc_health_state_ = ReentryMpcHealthState::UNHEALTHY;
    return reentry_mpc_health_state_;
  }

  const bool is_new_sample = reentry_input.mpc_health_sample_sequence !=
                             last_reentry_mpc_health_sample_sequence_;
  if (!is_new_sample) {
    return reentry_mpc_health_state_;
  }
  last_reentry_mpc_health_sample_sequence_ =
      reentry_input.mpc_health_sample_sequence;

  if (reentry_input.mpc_latency_warning) {
    reentry_mpc_good_sample_count_ = 0;
    ++reentry_mpc_bad_sample_count_;
    const int hard_fail_samples =
        std::max(1, config_.reentry_mpc_unhealthy_enter_samples);
    const int degraded_hold_samples =
        std::clamp(config_.reentry_mpc_latency_degraded_enter_samples, 1,
                   hard_fail_samples);
    if (reentry_mpc_bad_sample_count_ >= hard_fail_samples) {
      reentry_mpc_health_state_ = ReentryMpcHealthState::UNHEALTHY;
    } else if (reentry_mpc_bad_sample_count_ >= degraded_hold_samples ||
               reentry_mpc_health_state_ ==
                   ReentryMpcHealthState::TRANSIENT_LATENCY) {
      // 連続latencyは同じSafetyEvaluatorを通した現d holdだけに留める。
      reentry_mpc_health_state_ = ReentryMpcHealthState::TRANSIENT_LATENCY;
    } else {
      // freshな単発latencyはBuilderの速度capだけを適用し、新規PASS/復帰を
      // 停止させない。timer周期ではなく新規MPC sampleだけで数える。
      reentry_mpc_health_state_ = ReentryMpcHealthState::HEALTHY;
    }
    return reentry_mpc_health_state_;
  }

  reentry_mpc_bad_sample_count_ = 0;
  if (reentry_mpc_health_state_ == ReentryMpcHealthState::HEALTHY) {
    return reentry_mpc_health_state_;
  }
  ++reentry_mpc_good_sample_count_;
  const int release_samples =
      std::max(1, config_.reentry_mpc_healthy_release_samples);
  reentry_mpc_health_state_ = reentry_mpc_good_sample_count_ >= release_samples
                                  ? ReentryMpcHealthState::HEALTHY
                                  : ReentryMpcHealthState::TRANSIENT_LATENCY;
  return reentry_mpc_health_state_;
}

// 入力: 現在時刻、自車、周辺車、MPC health、Node側の入力鮮度状態。
// 出力: 通常ライン復帰を許可できるかと、拒否理由・ブロッカー。
// 処理概要:
// 全fresh相手の予測と復帰終端までの候補を同じSafetyEvaluatorで照合し、
// 連続安全周期を満たすまでfail-closedにする。
ReentryGateResult OvertakePlannerCore::evaluateReentryGate(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const MpcHealthStatus &mpc_health, const ReentryInputStatus &reentry_input,
    ReentryMpcHealthState reentry_mpc_health) {
  (void)mpc_health;
  ReentryGateResult gate;
  gate.requested = config_.reentry_gate_enabled && reentryRequested(ego);
  if (!gate.requested) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    return gate;
  }

  const bool mpc_input_ready =
      !config_.reentry_require_mpc_health ||
      reentry_mpc_health == ReentryMpcHealthState::HEALTHY ||
      reentry_mpc_health == ReentryMpcHealthState::TRANSIENT_LATENCY;
  gate.input_complete = reentry_input.ego_fresh &&
                        reentry_input.v2x_snapshot_fresh &&
                        reentry_input.all_observed_opponents_fresh &&
                        reentry_input.all_observed_opponents_included &&
                        reentry_input.reference_valid && mpc_input_ready;
  if (!gate.input_complete) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = !reentry_input.ego_fresh            ? "stale_ego"
                  : !reentry_input.v2x_snapshot_fresh ? "stale_v2x_snapshot"
                  : !reentry_input.all_observed_opponents_fresh
                      ? "stale_reentry_opponent"
                  : !reentry_input.all_observed_opponents_included
                      ? "untracked_reentry_opponent"
                  : !reentry_input.reference_valid ? "invalid_reference"
                  : reentry_mpc_health == ReentryMpcHealthState::STALE
                      ? "stale_mpc_health"
                      : "unhealthy_mpc";
    gate.clear_cycles = reentry_clear_cycles_;
    return gate;
  }

  const auto reentry_candidate =
      makeReentryEvaluationCandidate(ego, blocked_info, opponents);
  const auto predictions =
      predictOpponents(opponents, now_sec, &reentry_candidate.t);
  gate.evaluated_opponent_count = static_cast<int>(predictions.size());
  if (predictions.size() != opponents.size() || reentry_candidate.t.empty() ||
      reentry_candidate.x.size() != reentry_candidate.t.size() ||
      reentry_candidate.y.size() != reentry_candidate.t.size() ||
      reentry_candidate.yaw.size() != reentry_candidate.t.size()) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = "incomplete_reentry_prediction";
    gate.clear_cycles = reentry_clear_cycles_;
    return gate;
  }

  CandidateTrajectory evaluated = reentry_candidate;
  safety_.evaluate(evaluated, predictions);
  gate.min_safety_margin = evaluated.min_safety_margin;
  gate.cbf_slack = evaluated.cbf_slack;
  gate.blocking_vehicle_id = evaluated.blocking_opponent_id;
  gate.blocking_time_sec = evaluated.blocking_time_sec;
  const bool full_centering_evaluated =
      !evaluated.d.empty() && std::isfinite(evaluated.d.back()) &&
      std::abs(evaluated.d.back()) <= reentryCompletionLateralErrorM(config_);
  if (!evaluated.longitudinal_profile_valid) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = "invalid_longitudinal_brake_model";
  } else if (!evaluated.feasible) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = evaluated.reject_reason;
  } else if (!full_centering_evaluated) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = "incomplete_reentry_centering_horizon";
  } else if (std::isnan(evaluated.min_safety_margin) ||
             evaluated.min_safety_margin <
                 config_.reentry_min_safety_margin_h) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = "reentry_margin_below_threshold";
  } else if (evaluated.cbf_slack > 0.0) {
    reentry_clear_cycles_ = 0;
    reentry_gate_permitted_ = false;
    gate.reason = "reentry_cbf_slack";
  } else if (reentry_mpc_health == ReentryMpcHealthState::TRANSIENT_LATENCY) {
    // solve timeの一過性悪化では、同じ安全評価を通した現d holdだけを出し、
    // 中心復帰を禁じる。CBF clear回数は捨てず、healthy sampleのhysteresis後に
    // 既存のreentry_safe_cycles判定へ戻す。
    reentry_gate_permitted_ = false;
    gate.clear_cycles = reentry_clear_cycles_;
    gate.reason = "mpc_latency_degraded";
    return gate;
  } else {
    ++reentry_clear_cycles_;
    gate.clear_cycles = reentry_clear_cycles_;
    const int required_cycles = std::max(1, config_.reentry_safe_cycles);
    gate.permitted = reentry_clear_cycles_ >= required_cycles;
    gate.reason = gate.permitted ? "reentry_clear" : "reentry_clear_pending";
    if (gate.permitted) {
      reentry_lockout_active_ = false;
      reentry_gate_permitted_ = true;
    } else {
      reentry_gate_permitted_ = false;
    }
    return gate;
  }

  gate.clear_cycles = reentry_clear_cycles_;
  if (reentry_lockout_active_ && gate.reason.empty()) {
    gate.reason = "reentry_lockout";
  }
  return gate;
}

// 入力: 候補種別、自車状態、BlockedInfo、相手車一覧。
// 出力: CandidateBuilderで生成した候補軌道。
// 処理概要:
// core側で保持している局所横プロファイルを必要に応じて候補生成へ渡す。
CandidateTrajectory OvertakePlannerCore::makeCandidate(
    CandidateType type, const EgoState &ego, const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents) const {
  return CandidateBuilder(frame_, config_)
      .makeCandidate(type, ego, blocked_info, opponents,
                     localized_lateral_profile_.active
                         ? &localized_lateral_profile_
                         : nullptr);
}

// 入力: 安全評価済み候補とBlockedInfo。
// 出力: 小さいほど優先されるscore。
// 処理概要:
// 安全性を最優先にしつつ、状況に応じてPASS/FOLLOW/YIELD/RECOVERYの優先度を調整する。
double
OvertakePlannerCore::candidateScore(const CandidateTrajectory &candidate,
                                    const BlockedInfo &blocked_info) const {
  // 候補の優先順位を単純なコストへ落とし、まず安全性、その後に追い越し意欲を見る。
  if (!candidate.feasible) {
    if (candidate.reject_reason == "wall_margin" && blocked_info.side_by_side &&
        candidate.type == CandidateType::YIELD_BEHIND) {
      return blocked_info.corner_side_by_side ? -90.0 : -40.0;
    }
    if (candidate.reject_reason == "opponent_collision" &&
        blocked_info.side_by_side &&
        candidate.type == CandidateType::SIDE_BY_SIDE_KEEP &&
        !blocked_info.corner_side_by_side) {
      return -30.0;
    }
    if (candidate.reject_reason == "opponent_collision" &&
        candidate.type == CandidateType::YIELD_BEHIND) {
      return blocked_info.corner_side_by_side ? -70.0 : -50.0;
    }
    return 1.0e9;
  }
  const bool verified_attack_follow_transaction =
      blocked_info.maneuver_transaction_incomplete &&
      blocked_info.attack_follow_hold_pass_side &&
      blocked_info.parallel_yield_hold_lateral &&
      !blocked_info.future_yield_required &&
      !blocked_info.corner_side_by_side &&
      !blocked_info.future_corner_side_by_side &&
      !blocked_info.future_outer_wall_risk &&
      blocked_info.maneuver_target_latched &&
      blocked_info.maneuver_target_observed &&
      blocked_info.maneuver_target_fresh &&
      blocked_info.braking_follow_active &&
      blocked_info.braking_follow_feasible &&
      !blocked_info.braking_follow_id.empty() &&
      blocked_info.braking_follow_id != blocked_info.maneuver_target_id &&
      blocked_info.braking_follow_id == blocked_info.maneuver_chain_tail_id &&
      blocked_info.maneuver_chain_tail_observed;
  double score = 0.0;
  // 処理ブロック: feasible候補の基本優先度を種別ごとに決める。
  // 設計意図:
  // 通常はPASSを取りに行くが、横並びや未来譲りではYIELD/KEEPを優先できるようにする。
  switch (candidate.type) {
  case CandidateType::FASTEST:
    score = (blocked_info.blocked || blocked_info.start_grid_target_active ||
             blocked_info.start_grid_lateral_release_pending ||
             blocked_info.early_stationary_parallel_pass_target ||
             blocked_info.braking_follow_active)
                ? 50.0
                : 0.0;
    break;
  case CandidateType::FOLLOW:
    score = verified_attack_follow_transaction        ? -90.0
            : currentPassGapLost(mode_, blocked_info) ? -60.0
            : blocked_info.start_grid_target_active   ? -45.0
            : blocked_info.braking_follow_active      ? -40.0
            : blocked_info.parallel_follow_candidate && !blocked_info.blocked
                ? -35.0
                : 15.0;
    // 未完了PASSの同側FOLLOWは、ラッチ対象とchain tailがfreshで、かつ
    // FOLLOW候補自身がSafetyEvaluatorを通った場合だけstrict parallel YIELD
    // より優先する。YIELD候補は評価集合へ残すため、FOLLOWが不成立なら既存の
    // YIELD/RECOVERYへ必ず閉じる。
    break;
  case CandidateType::PASS_LEFT:
  case CandidateType::PASS_RIGHT: {
    score = blocked_info.side_by_side ? 200.0 : -10.0;
    const bool prepare_mode = mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                              mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT;
    const bool matching_prepared_pass =
        prepare_mode &&
        (blocked_info.maneuver_transaction_incomplete
             ? blocked_info.maneuver_transaction_pass_type == candidate.type
             : ((mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT &&
                 candidate.type == CandidateType::PASS_LEFT) ||
                (mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT &&
                 candidate.type == CandidateType::PASS_RIGHT)));
    const bool matching_committed_pass_retry =
        mode_ == BehaviorMode::FOLLOW_BLOCKED &&
        blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_target_latched &&
        blocked_info.maneuver_target_observed &&
        blocked_info.maneuver_target_fresh &&
        blocked_info.straight_overtake_start_allowed &&
        blocked_info.maneuver_transaction_pass_type == candidate.type;
    const bool matching_active_pass =
        (mode_ == BehaviorMode::OVERTAKE_LEFT &&
         candidate.type == CandidateType::PASS_LEFT) ||
        (mode_ == BehaviorMode::OVERTAKE_RIGHT &&
         candidate.type == CandidateType::PASS_RIGHT) ||
        matching_prepared_pass || matching_committed_pass_retry;
    if (matching_active_pass && !blocked_info.side_by_side &&
        !blocked_info.future_yield_required &&
        !currentPassGapLost(mode_, blocked_info)) {
      // PREPARE/OVERTAKE中は、同じ方向でこの周期にもSafetyEvaluatorを
      // 通過したPASSをFOLLOWのstart-grid優先scoreより上に保つ。対象IDと
      // 横位置のトランザクションを維持しつつ、gap喪失/YIELDは従来どおり
      // この優先を無効にして安全側候補へ譲る。
      score = -100.0;
    } else if ((mode_ == BehaviorMode::FREE_RUN ||
                mode_ == BehaviorMode::FOLLOW_BLOCKED) &&
               blocked_info.straight_overtake_start_allowed &&
               (blocked_info.blocked || blocked_info.start_grid_target_active ||
                blocked_info.braking_follow_active) &&
               (!blocked_info.side_by_side ||
                (blocked_info.start_grid_target_active &&
                 !blocked_info.corner_side_by_side)) &&
               !blocked_info.future_yield_required) {
      // PASS候補はこの周期にSafetyEvaluatorを通過済みである。静的gapは
      // 監視/hysteresis値として残すが、固定lookahead外の制動距離/TTC対象も含め、
      // 実マップ回廊と候補軌道を通ったPASSをFOLLOWの優先度で負けさせない。
      score = -100.0;
    }
    if ((mode_ == BehaviorMode::FREE_RUN ||
         mode_ == BehaviorMode::FOLLOW_BLOCKED) &&
        blocked_info.maneuver_transaction_incomplete &&
        blocked_info.maneuver_transaction_pass_type == candidate.type) {
      // 同周期に左右ともGate 2を通った時も、認可してtransactionへ反映した側を
      // vector順で上書きしない。target ID/sideの固定契約を選択scoreまで揃える。
      score = std::min(score, -110.0);
    }
    break;
  }
  case CandidateType::RECOVERY:
    score = blocked_info.start_grid_uncommitted_hold_active ? -40.0
            : blocked_info.braking_follow_hold_lateral      ? -30.0
            : blocked_info.start_grid_lateral_release_pending &&
                    !blocked_info.start_grid_target_active
                ? -20.0
                : 40.0;
    break;
  case CandidateType::SIDE_BY_SIDE_KEEP:
    score = blocked_info.side_by_side && !blocked_info.corner_side_by_side &&
                    !blocked_info.future_yield_required
                ? -30.0
                : 80.0;
    break;
  case CandidateType::YIELD_BEHIND:
    if (blocked_info.future_yield_required) {
      score = -80.0;
    } else if (blocked_info.parallel_yield_hold_lateral) {
      // strict parallel接近はPASS/FASTESTより先に減速して後方へ譲る。
      score = -65.0;
    } else if (blocked_info.corner_side_by_side) {
      score = -70.0;
    } else if (blocked_info.side_by_side &&
               blocked_info.side_delta_s > config_.side_yield_s_m) {
      score = -60.0;
    } else if (currentPassGapLost(mode_, blocked_info)) {
      score = -50.0;
    } else {
      score = (!blocked_info.can_pass_left && !blocked_info.can_pass_right)
                  ? 5.0
                  : 70.0;
    }
    break;
  case CandidateType::SAFE_STOP:
    score = -120.0;
    break;
  }
  if ((mode_ == BehaviorMode::OVERTAKE_LEFT &&
       candidate.type == CandidateType::PASS_LEFT) ||
      (mode_ == BehaviorMode::OVERTAKE_RIGHT &&
       candidate.type == CandidateType::PASS_RIGHT) ||
      (mode_ == BehaviorMode::FOLLOW_BLOCKED &&
       candidate.type == CandidateType::FOLLOW) ||
      (mode_ == BehaviorMode::SIDE_BY_SIDE_KEEP &&
       candidate.type == CandidateType::SIDE_BY_SIDE_KEEP) ||
      (mode_ == BehaviorMode::YIELD_BEHIND &&
       candidate.type == CandidateType::YIELD_BEHIND) ||
      (mode_ == BehaviorMode::SAFE_STOP &&
       candidate.type == CandidateType::SAFE_STOP)) {
    // いまのモードに沿う候補を少し優遇し、左右や追従/追い越しが細かく揺れないようにする。
    score -= config_.keep_mode_bonus;
  }
  return score;
}

// 入力: 開始sとlookahead距離[m]。
// 出力: 区間内の最大絶対曲率[1/m]。
// 処理概要:
// 追い越し開始gateや横並びコーナー判定のため、参照線曲率を粗くサンプリングする。
double OvertakePlannerCore::maxAbsCurvatureAhead(double s,
                                                 double lookahead_m) const {
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

// 入力: 現在s。
// 出力: 現在有効なsection safety設定。
// 処理概要:
// YAMLから読み込んだ区間ルールを探し、壁マージン/速度/外側譲りのスケールを返す。
ActiveSectionSafety OvertakePlannerCore::activeSectionSafety(double s) const {
  ActiveSectionSafety active;
  if (!config_.section_safety_profile_enabled) {
    return active;
  }
  for (const auto &rule : config_.section_safety_rules) {
    if (!sectionContainsS(rule, s)) {
      continue;
    }
    active.active = true;
    active.name = rule.name;
    active.profile = rule.profile.empty() ? "default" : rule.profile;
    active.role_policy =
        rule.role_policy.empty() ? "default" : rule.role_policy;
    if (active.profile == "wall_risk_moderate") {
      active.wall_margin_scale = 1.15;
      active.speed_cap_scale = 0.85;
    } else if (active.profile == "side_by_side_corner_strict") {
      active.wall_margin_scale = 1.35;
      active.speed_cap_scale = 0.65;
      active.force_outer_yield = true;
    }
    if (active.role_policy == "outer_yields") {
      active.force_outer_yield = true;
    }
    return active;
  }
  return active;
}

// 入力: 現在s。
// 出力: lookaheadも考慮した追い越し許可状態。
// 処理概要:
// 現在地点と前方区間を見て、近い将来の禁止区間へ入る前に追い越し開始を止める。
ActiveOvertakePermission
OvertakePlannerCore::activeOvertakePermission(double s) const {
  ActiveOvertakePermission current = overtakePermissionAtS(s);
  if (!config_.overtake_permission_profile_enabled ||
      config_.overtake_permission_lookahead_m <= 0.0) {
    return current;
  }

  const int sample_count = 8;
  const double ds = config_.overtake_permission_lookahead_m /
                    static_cast<double>(sample_count);
  for (int i = 1; i <= sample_count; ++i) {
    const auto ahead = overtakePermissionAtS(s + ds * static_cast<double>(i));
    if (!ahead.allow_overtake) {
      return ahead;
    }
  }
  return current;
}

// 入力: 評価対象s。
// 出力: そのs地点単体の追い越し許可状態。
// 処理概要: 許可CSVから該当区間を探し、見つからなければdefault設定を返す。
ActiveOvertakePermission
OvertakePlannerCore::overtakePermissionAtS(double s) const {
  ActiveOvertakePermission active;
  active.allow_overtake = config_.default_overtake_allowed;
  if (!config_.overtake_permission_profile_enabled) {
    active.allow_overtake = true;
    return active;
  }
  for (const auto &rule : config_.overtake_permission_rules) {
    if (!permissionRuleContainsS(rule, s)) {
      continue;
    }
    active.active = true;
    active.name = rule.name;
    active.allow_overtake = rule.allow_overtake;
    return active;
  }
  return active;
}

// 入力: section safety ruleと評価対象s。
// 出力: sがrule区間内ならtrue。
// 処理概要: 周回境界をまたぐ区間にも対応して、wrap済みsで包含判定する。
bool OvertakePlannerCore::sectionContainsS(const SectionSafetyRule &rule,
                                           double s) const {
  if (frame_.empty()) {
    return false;
  }
  const double start = frame_.wrapS(rule.s_start_m);
  const double end = frame_.wrapS(rule.s_end_m);
  const double wrapped_s = frame_.wrapS(s);
  if (start <= end) {
    return wrapped_s >= start && wrapped_s <= end;
  }
  return wrapped_s >= start || wrapped_s <= end;
}

// 入力: overtake permission ruleと評価対象s。
// 出力: sがrule区間内ならtrue。
// 処理概要: sectionContainsSと同じ閉ループ区間判定を追い越し許可CSVにも使う。
bool OvertakePlannerCore::permissionRuleContainsS(
    const OvertakePermissionRule &rule, double s) const {
  if (frame_.empty()) {
    return false;
  }
  const double start = frame_.wrapS(rule.s_start_m);
  const double end = frame_.wrapS(rule.s_end_m);
  const double wrapped_s = frame_.wrapS(s);
  if (start <= end) {
    return wrapped_s >= start && wrapped_s <= end;
  }
  return wrapped_s >= start || wrapped_s <= end;
}

// 入力: 現在有効なsection safety設定。
// 出力: section scaleを反映したsoft wall margin[m]。
// 処理概要: コーナー厳格区間などで壁に寄る判断を早めに危険扱いする。
double OvertakePlannerCore::effectiveWallSoftMargin(
    const ActiveSectionSafety &section) const {
  const double base = std::max(0.0, config_.wall_soft_margin_m);
  return base * std::max(1.0, section.wall_margin_scale);
}

// 入力: detectBlocked直後のBlockedInfoと相手車一覧。
// 出力: parallel-sideの低速/停止車を前方閉塞へ昇格したならtrue。
// 処理概要: 停止車列を1台ずつ抜く場面で、2台目以降がfrontではなく
// parallel-sideに見えても、既存PASS候補生成と状態機械に載せる。
bool OvertakePlannerCore::promoteSlowObstacleChain(
    BlockedInfo &blocked, const std::vector<OpponentState> &opponents) const {
  if (!config_.slow_obstacle_chain_enabled ||
      !config_.slow_front_exception_enabled || blocked.nearest_index >= 0 ||
      !blocked.parallel_side_candidate || blocked.parallel_side_index < 0 ||
      static_cast<std::size_t>(blocked.parallel_side_index) >=
          opponents.size()) {
    return false;
  }
  if (!std::isfinite(blocked.parallel_side_delta_s) ||
      blocked.parallel_side_delta_s <= 0.0 ||
      blocked.parallel_side_delta_s >
          std::max(0.0, config_.slow_obstacle_chain_distance_m)) {
    return false;
  }
  // 通常時は同じ走行コリドー内だけ昇格する。一方で既に追い越し側へ出ている時は、
  // 2台目以降が横に離れて見えるため、parallel検出幅まで停止車列として扱う。
  const bool pass_chain_context = isAnyPassMode(mode_) ||
                                  localized_lateral_profile_.active ||
                                  reentry_lockout_active_;
  const double chain_lateral_limit = pass_chain_context
                                         ? config_.parallel_side_margin_m
                                         : config_.same_corridor_width_m;
  if (std::abs(blocked.parallel_side_delta_d) >
          std::max(0.0, chain_lateral_limit) ||
      !std::isfinite(blocked.parallel_side_rel_v) ||
      blocked.parallel_side_rel_v <= 0.0) {
    return false;
  }

  const auto &target =
      opponents[static_cast<std::size_t>(blocked.parallel_side_index)];
  if (!std::isfinite(target.v) ||
      target.v > std::max(0.0, config_.slow_front_exception_speed_mps)) {
    return false;
  }

  blocked.slow_obstacle_chain_active = true;
  blocked.slow_obstacle_chain_id = target.id;
  blocked.slow_obstacle_chain_delta_s = blocked.parallel_side_delta_s;
  blocked.slow_obstacle_chain_delta_d = blocked.parallel_side_delta_d;
  blocked.slow_obstacle_chain_speed_mps = target.v;

  blocked.blocked = true;
  blocked.nearest_index = blocked.parallel_side_index;
  blocked.nearest_id = blocked.parallel_side_id;
  blocked.front_delta_s = blocked.parallel_side_delta_s;
  blocked.front_delta_d = blocked.parallel_side_delta_d;
  blocked.front_rel_v = blocked.parallel_side_rel_v;
  blocked.front_vehicle_speed_mps = target.v;
  blocked.front_s_dot_mps = blocked.parallel_side_s_dot_mps;
  blocked.front_direction_known = blocked.parallel_side_direction_known;
  blocked.front_same_direction = blocked.parallel_side_same_direction;
  return true;
}

// 入力: 現在時刻、自車、parallel観測を含むBlockedInfo、相手車一覧。
// 出力: race
// arm直後のgrid対象、または停止した前方parallel車をPASS評価対象にする。
// 処理概要: START_GRID_TARGETは前方と小さな後方側の隣接車を扱うが、分類だけで
// PASSを許可しない。左右候補は後段のSafetyEvaluator/Gate 2へ必ず通す。
void OvertakePlannerCore::classifyEarlyStationaryParallelPassTarget(
    double now_sec, const EgoState &ego,
    const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) {
  blocked.early_stationary_parallel_pass_target = false;
  blocked.early_stationary_parallel_pass_id.clear();
  blocked.early_stationary_parallel_pass_count = 0;
  blocked.early_stationary_parallel_pass_hold_lateral = false;
  blocked.start_grid_target_active = false;
  blocked.start_grid_target_confirmation_pending = false;
  blocked.start_grid_target_confirmed_stationary = false;
  blocked.start_grid_follow_hold_lateral = false;
  blocked.start_grid_target_superseded_by_blocked_front = false;
  blocked.start_grid_target_reselection_suppressed =
      start_grid_target_reselection_suppressed_;
  blocked.start_grid_superseded_target_id.clear();
  blocked.start_grid_replacement_target_id = start_grid_handoff_target_id_;
  const bool committed_start_grid_pass_incomplete =
      localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      !start_grid_target_id_.empty() &&
      localized_lateral_profile_.target_id == start_grid_target_id_;
  if (start_grid_lateral_release_pending_ &&
      !committed_start_grid_pass_incomplete && std::isfinite(ego.frenet.d) &&
      std::abs(ego.frenet.d) <= reentryCompletionLateralErrorM(config_)) {
    // Gate 2認可後に使った横位置だけを、長期reentry gateで実際に
    // 中心へ収束した後に解放する。PASS未完了中は一時的に中心付近でも
    // transactionを消さず、target ID/side/dを保持する。
    start_grid_lateral_release_pending_ = false;
    start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
    start_grid_target_id_.clear();
    start_grid_target_first_seen_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_first_seen_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
  }
  blocked.start_grid_lateral_release_pending =
      start_grid_lateral_release_pending_;
  blocked.start_grid_hold_target_d_m = start_grid_lateral_anchor_d_m_;
  blocked.start_grid_target_index = -1;
  blocked.start_grid_target_id.clear();
  blocked.start_grid_target_delta_s = std::numeric_limits<double>::infinity();
  blocked.start_grid_target_delta_d = 0.0;
  blocked.start_grid_target_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  blocked.start_grid_target_age_sec = 0.0;
  blocked.start_grid_ego_progress_m = 0.0;

  const auto reset_confirmation = [this]() {
    early_stationary_parallel_pass_id_.clear();
    early_stationary_parallel_pass_count_ = 0;
    // 欠測を跨いだ停止時間を足さない。ID/横anchorを安全復帰用に保持する
    // 場合でも、停止確認だけはfreshな連続観測から取り直す。
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    // 外側横位置を使用済みなら、入力欠損だけでtarget IDとanchorを捨てて
    // 未評価の中心復帰へ進まない。freshness回復までは後段reentry gateが閉じる。
    if (!start_grid_lateral_release_pending_) {
      start_grid_target_id_.clear();
      start_grid_target_first_seen_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      start_grid_target_first_seen_s_m_ =
          std::numeric_limits<double>::quiet_NaN();
      start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
    }
  };
  const bool base_safety_inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid;
  if (!base_safety_inputs_complete || !ego.valid) {
    reset_confirmation();
    return;
  }

  const double arm_elapsed_sec =
      std::isfinite(first_valid_update_sec_)
          ? std::max(0.0, now_sec - first_valid_update_sec_)
          : std::numeric_limits<double>::infinity();
  const double arm_progress_m =
      std::isfinite(first_valid_update_s_m_)
          ? std::max(0.0, signedDeltaS(first_valid_update_s_m_, ego.frenet.s,
                                       frame_.length()))
          : std::numeric_limits<double>::infinity();
  const bool start_grid_window_active =
      config_.start_grid_target_enabled &&
      !start_grid_target_reselection_suppressed_ &&
      std::isfinite(config_.start_grid_target_window_sec) &&
      config_.start_grid_target_window_sec > 0.0 &&
      arm_elapsed_sec <= config_.start_grid_target_window_sec &&
      std::isfinite(config_.start_grid_target_window_distance_m) &&
      config_.start_grid_target_window_distance_m > 0.0 &&
      arm_progress_m <= config_.start_grid_target_window_distance_m &&
      std::isfinite(ego.v) &&
      ego.v <= std::max(0.0, config_.start_grid_target_max_ego_speed_mps);

  const bool uncommitted_start_grid_target =
      !start_grid_target_id_.empty() && !start_grid_lateral_release_pending_ &&
      (!localized_lateral_profile_.active ||
       (!localized_lateral_profile_.pass_safety_approved_once &&
        !localized_lateral_profile_.pass_execution_committed));
  const bool different_authoritative_blocked_front =
      blocked.blocked && blocked.nearest_index >= 0 &&
      static_cast<std::size_t>(blocked.nearest_index) < opponents.size() &&
      !blocked.nearest_id.empty() &&
      blocked.nearest_id != start_grid_target_id_ &&
      std::isfinite(blocked.front_delta_s) && blocked.front_delta_s > 0.0 &&
      (!blocked.front_direction_known || blocked.front_same_direction) &&
      (blocked.slow_obstacle_chain_active || blocked.braking_follow_active ||
       (std::isfinite(blocked.front_rel_v) &&
        blocked.front_rel_v > config_.dv_block_threshold_mps));
  const bool start_grid_target_superseded =
      uncommitted_start_grid_target && different_authoritative_blocked_front;
  if (start_grid_target_superseded) {
    // 20260719 dev3では未認可のd3を保持したまま、実際の最接近低速車がd2へ
    // 変わってもFOLLOW対象が更新されず、d2の安全楕内で全候補が不成立に
    // なった。Gate 2認可前のprofileはID固定契約ではないため、権威的な
    // blocked frontに譲る。Gate 2認可済みのACK待ちprofileは、まだ実行commit前
    // でも旧target/side/generationを保持する契約なので、別IDへ引き継がない。
    blocked.start_grid_target_superseded_by_blocked_front = true;
    start_grid_target_reselection_suppressed_ = true;
    start_grid_handoff_target_id_ = blocked.nearest_id;
    blocked.start_grid_target_reselection_suppressed = true;
    blocked.start_grid_superseded_target_id = start_grid_target_id_;
    blocked.start_grid_replacement_target_id = start_grid_handoff_target_id_;
    start_grid_target_id_.clear();
    start_grid_target_first_seen_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_first_seen_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
  }

  const auto activate_start_grid_target = [this, now_sec, &ego, &blocked](
                                              const OpponentState &target,
                                              int target_index, double delta_s,
                                              double delta_d) {
    if (start_grid_target_id_.empty()) {
      start_grid_target_id_ = target.id;
      start_grid_target_first_seen_sec_ = now_sec;
      start_grid_target_first_seen_s_m_ = ego.frenet.s;
      start_grid_target_stationary_since_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      start_grid_target_stationary_since_s_m_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    if (!std::isfinite(start_grid_lateral_anchor_d_m_)) {
      start_grid_lateral_anchor_d_m_ = ego.frenet.d;
      // 初期グリッドの法的な横位置は「追越後の残留d」ではない。
      // ここではFOLLOW用anchorだけを保持し、reentry transactionは
      // SafetyEvaluator/Gate 2がPASSを認可した後にだけ開始する。
    }
    const double target_age_sec =
        std::isfinite(start_grid_target_first_seen_sec_)
            ? std::max(0.0, now_sec - start_grid_target_first_seen_sec_)
            : 0.0;
    const double target_progress_m =
        std::isfinite(start_grid_target_first_seen_s_m_)
            ? std::max(0.0, signedDeltaS(start_grid_target_first_seen_s_m_,
                                         ego.frenet.s, frame_.length()))
            : 0.0;
    const bool target_is_stationary =
        target.v <=
        std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
    if (target_is_stationary) {
      if (!std::isfinite(start_grid_target_stationary_since_sec_) ||
          !std::isfinite(start_grid_target_stationary_since_s_m_)) {
        start_grid_target_stationary_since_sec_ = now_sec;
        start_grid_target_stationary_since_s_m_ = ego.frenet.s;
      }
    } else {
      // 同じIDでも一度動けば停止確認を最初から取り直す。これにより、
      // moving->一周期だけ低速となった車へ大横断PASSをコミットしない。
      start_grid_target_stationary_since_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      start_grid_target_stationary_since_s_m_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    const double stationary_elapsed_sec =
        target_is_stationary &&
                std::isfinite(start_grid_target_stationary_since_sec_)
            ? std::max(0.0, now_sec - start_grid_target_stationary_since_sec_)
            : 0.0;
    const double stationary_progress_m =
        target_is_stationary &&
                std::isfinite(start_grid_target_stationary_since_s_m_)
            ? std::max(0.0,
                       signedDeltaS(start_grid_target_stationary_since_s_m_,
                                    ego.frenet.s, frame_.length()))
            : 0.0;
    const bool confirmed_by_time =
        target_is_stationary &&
        config_.start_grid_stationary_confirmation_sec > 0.0 &&
        stationary_elapsed_sec >=
            config_.start_grid_stationary_confirmation_sec;
    const bool confirmed_by_progress =
        target_is_stationary &&
        config_.start_grid_stationary_confirmation_distance_m > 0.0 &&
        stationary_progress_m >=
            config_.start_grid_stationary_confirmation_distance_m;

    if (target.id == early_stationary_parallel_pass_id_) {
      ++early_stationary_parallel_pass_count_;
    } else {
      early_stationary_parallel_pass_id_ = target.id;
      early_stationary_parallel_pass_count_ = 1;
    }
    blocked.start_grid_target_active = true;
    blocked.start_grid_target_confirmation_pending =
        !confirmed_by_time && !confirmed_by_progress;
    blocked.start_grid_target_confirmed_stationary =
        confirmed_by_time || confirmed_by_progress;
    blocked.start_grid_follow_hold_lateral = true;
    blocked.start_grid_lateral_release_pending =
        start_grid_lateral_release_pending_;
    blocked.start_grid_hold_target_d_m = start_grid_lateral_anchor_d_m_;
    blocked.start_grid_target_index = target_index;
    blocked.start_grid_target_id = target.id;
    blocked.start_grid_target_delta_s = delta_s;
    blocked.start_grid_target_delta_d = delta_d;
    blocked.start_grid_target_speed_mps = target.v;
    blocked.start_grid_target_age_sec = target_age_sec;
    blocked.start_grid_ego_progress_m = target_progress_m;
    blocked.early_stationary_parallel_pass_target = true;
    blocked.early_stationary_parallel_pass_id = target.id;
    blocked.early_stationary_parallel_pass_count =
        early_stationary_parallel_pass_count_;
    blocked.early_stationary_parallel_pass_hold_lateral = true;
  };

  if (start_grid_window_active && !start_grid_target_superseded) {
    const double min_delta_s = config_.start_grid_target_min_delta_s_m;
    const double max_delta_s =
        std::max(min_delta_s, config_.start_grid_target_max_delta_s_m);
    const double max_delta_d =
        std::max(0.0, config_.start_grid_target_lateral_width_m);
    int best_index = -1;
    double best_delta_s = std::numeric_limits<double>::infinity();
    double best_delta_d = 0.0;
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < opponents.size(); ++i) {
      const auto &candidate = opponents[i];
      // Gate 2認可後は、より近い別車を見つけてもtarget IDを張り替えない。
      // 同じIDがfreshでなくなった周期はreentry gateへ閉じる。
      if (start_grid_lateral_release_pending_ &&
          !start_grid_target_id_.empty() &&
          candidate.id != start_grid_target_id_) {
        continue;
      }
      const double start_grid_target_speed_limit_mps = std::max(
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps),
          std::max(0.0, config_.start_grid_attack_follow_v_max_mps));
      if (!candidate.valid ||
          !inputTimestampFresh(now_sec, candidate.stamp_sec,
                               config_.opponent_stale_time_sec,
                               config_.input_future_stamp_tolerance_sec) ||
          !std::isfinite(candidate.v) ||
          candidate.v > start_grid_target_speed_limit_mps) {
        continue;
      }
      const double delta_s =
          signedDeltaS(ego.frenet.s, candidate.frenet.s, frame_.length());
      const double delta_d = candidate.frenet.d - ego.frenet.d;
      if (!std::isfinite(delta_s) || !std::isfinite(delta_d) ||
          delta_s < min_delta_s || delta_s > max_delta_s ||
          std::abs(delta_d) > max_delta_d ||
          // 自車が明確に先行している時は、後方車を追越対象へしない。
          delta_s <
              -std::max(0.0, config_.side_by_side_leader_priority_enter_s_m)) {
        continue;
      }
      const double score = delta_s >= 0.0 ? delta_s : 100.0 + std::abs(delta_s);
      if (score < best_score) {
        best_score = score;
        best_index = static_cast<int>(i);
        best_delta_s = delta_s;
        best_delta_d = delta_d;
      }
    }

    if (best_index >= 0) {
      const auto &target = opponents[static_cast<std::size_t>(best_index)];
      if (target.id != start_grid_target_id_ &&
          !start_grid_lateral_release_pending_) {
        start_grid_target_id_ = target.id;
        start_grid_target_first_seen_sec_ = now_sec;
        start_grid_target_first_seen_s_m_ = ego.frenet.s;
        start_grid_target_stationary_since_sec_ =
            std::numeric_limits<double>::quiet_NaN();
        start_grid_target_stationary_since_s_m_ =
            std::numeric_limits<double>::quiet_NaN();
      }
      activate_start_grid_target(target, best_index, best_delta_s,
                                 best_delta_d);
      return;
    }
  }

  if (!start_grid_target_superseded && !start_grid_target_id_.empty()) {
    const auto retained = std::find_if(
        opponents.begin(), opponents.end(), [this](const auto &opponent) {
          return opponent.id == start_grid_target_id_;
        });
    if (retained != opponents.end() && retained->valid &&
        inputTimestampFresh(now_sec, retained->stamp_sec,
                            config_.opponent_stale_time_sec,
                            config_.input_future_stamp_tolerance_sec) &&
        std::isfinite(retained->v)) {
      const double delta_s =
          signedDeltaS(ego.frenet.s, retained->frenet.s, frame_.length());
      const double delta_d = retained->frenet.d - ego.frenet.d;
      const double retention_s_m =
          std::max({std::max(0.0, config_.start_grid_target_max_delta_s_m),
                    std::max(0.0, config_.follow_trigger_s_m),
                    std::max(0.0, config_.parallel_side_s_m)});
      const double retention_d_m =
          std::max({std::max(0.0, config_.start_grid_target_lateral_width_m),
                    std::max(0.0, config_.parallel_follow_lateral_width_m),
                    std::max(0.0, config_.safety_ellipse_b_m) *
                        std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h))});
      if (std::isfinite(delta_s) && std::isfinite(delta_d) && delta_s > 0.0 &&
          delta_s <= retention_s_m && std::abs(delta_d) <= retention_d_m) {
        // 初期window終了はtarget解放条件ではない。同じfresh IDが前方の
        // 攻めFOLLOW範囲にいる間はanchorを維持し、PASSを毎周期再評価する。
        activate_start_grid_target(
            *retained,
            static_cast<int>(std::distance(opponents.begin(), retained)),
            delta_s, delta_d);
        return;
      }
    }

    // Gate 2認可後の対象が前方FOLLOW範囲を外れた場合は、未評価の
    // FASTESTへ戻さず、generic reentryが中心到達するまでID/anchorを保持する。
    start_grid_target_stationary_since_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_stationary_since_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    if (start_grid_lateral_release_pending_) {
      blocked.start_grid_lateral_release_pending = true;
      blocked.start_grid_hold_target_d_m = start_grid_lateral_anchor_d_m_;
      return;
    }

    // 未認可の準備profileは横移動transactionではない。freshな同一IDを
    // 追跡できなくなったら初期anchorを解放し、ABORT/reentryを開始しない。
    start_grid_target_id_.clear();
    start_grid_target_first_seen_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_target_first_seen_s_m_ =
        std::numeric_limits<double>::quiet_NaN();
    start_grid_lateral_anchor_d_m_ = std::numeric_limits<double>::quiet_NaN();
  }

  start_grid_target_id_.clear();
  start_grid_target_first_seen_sec_ = std::numeric_limits<double>::quiet_NaN();
  start_grid_target_first_seen_s_m_ = std::numeric_limits<double>::quiet_NaN();
  start_grid_target_stationary_since_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  start_grid_target_stationary_since_s_m_ =
      std::numeric_limits<double>::quiet_NaN();

  const bool safety_inputs_complete =
      base_safety_inputs_complete && reentry_input.mpc_healthy;
  if (!config_.early_stationary_parallel_pass_enabled ||
      !safety_inputs_complete) {
    reset_confirmation();
    return;
  }

  const double max_distance_m =
      std::max(0.0, config_.early_stationary_parallel_pass_distance_m);
  const double same_corridor_width_m =
      std::max(0.0, config_.same_corridor_width_m);
  const double max_lateral_width_m =
      std::min(std::max(same_corridor_width_m,
                        config_.early_stationary_parallel_pass_lateral_width_m),
               std::max(0.0, config_.parallel_side_margin_m));
  if (max_distance_m <= 0.0 || max_lateral_width_m <= same_corridor_width_m) {
    reset_confirmation();
    return;
  }

  // parallel観測から通常frontへ昇格しても、同一IDの停止対象ならPASS確認を
  // 途切れさせない。別IDや入力欠損で継続することは絶対にしない。
  const bool latched_same_front =
      !early_stationary_parallel_pass_id_.empty() &&
      blocked.nearest_index >= 0 &&
      static_cast<std::size_t>(blocked.nearest_index) < opponents.size() &&
      (blocked.nearest_id == early_stationary_parallel_pass_id_ ||
       opponents[static_cast<std::size_t>(blocked.nearest_index)].id ==
           early_stationary_parallel_pass_id_);
  const bool parallel_probe =
      blocked.nearest_index < 0 && blocked.parallel_side_candidate &&
      blocked.parallel_side_index >= 0 &&
      static_cast<std::size_t>(blocked.parallel_side_index) < opponents.size();
  if (!latched_same_front && !parallel_probe) {
    reset_confirmation();
    return;
  }

  const int target_index =
      latched_same_front ? blocked.nearest_index : blocked.parallel_side_index;
  const auto &target = opponents[static_cast<std::size_t>(target_index)];
  const double delta_s = latched_same_front ? blocked.front_delta_s
                                            : blocked.parallel_side_delta_s;
  const double delta_d = latched_same_front ? blocked.front_delta_d
                                            : blocked.parallel_side_delta_d;
  const double rel_v =
      latched_same_front ? blocked.front_rel_v : blocked.parallel_side_rel_v;
  const bool direction_known = latched_same_front
                                   ? blocked.front_direction_known
                                   : blocked.parallel_side_direction_known;
  const bool same_direction = latched_same_front
                                  ? blocked.front_same_direction
                                  : blocked.parallel_side_same_direction;
  if (!std::isfinite(delta_s) || !std::isfinite(delta_d) ||
      !std::isfinite(rel_v) || delta_s <= 0.0 || delta_s > max_distance_m ||
      rel_v <= config_.dv_block_threshold_mps ||
      (direction_known && !same_direction)) {
    reset_confirmation();
    return;
  }
  if (parallel_probe) {
    const double abs_delta_d = std::abs(delta_d);
    if (abs_delta_d <= same_corridor_width_m ||
        abs_delta_d > max_lateral_width_m) {
      reset_confirmation();
      return;
    }
  }
  if (!target.valid ||
      !inputTimestampFresh(now_sec, target.stamp_sec,
                           config_.opponent_stale_time_sec,
                           config_.input_future_stamp_tolerance_sec) ||
      !std::isfinite(target.v) ||
      target.v >
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps)) {
    reset_confirmation();
    return;
  }
  const double target_s_dot_mps = blocked_risk_.opponentSDot(target);
  if (!std::isfinite(target_s_dot_mps) || target_s_dot_mps < 0.0) {
    reset_confirmation();
    return;
  }

  if (target.id == early_stationary_parallel_pass_id_) {
    ++early_stationary_parallel_pass_count_;
  } else {
    early_stationary_parallel_pass_id_ = target.id;
    early_stationary_parallel_pass_count_ = 1;
  }
  blocked.early_stationary_parallel_pass_target = true;
  blocked.early_stationary_parallel_pass_id = target.id;
  blocked.early_stationary_parallel_pass_count =
      early_stationary_parallel_pass_count_;
  blocked.early_stationary_parallel_pass_hold_lateral = true;
}

// 入力: 現在時刻、自車、前方/並走判定、相手車一覧。
// 出力: なし。same
// corridor外の前方parallel車をFOLLOW専用対象としてBlockedInfoへ記録する。
// 処理概要:
// 既存blocked/PASS判定は変えず、設定で明示ONの時だけ現在d保持FOLLOWの対象を追加する。
void OvertakePlannerCore::classifyParallelFollowCandidate(
    double now_sec, const EgoState &ego, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.parallel_follow_candidate = false;
  blocked.parallel_follow_feasible = false;
  blocked.parallel_follow_hold_lateral = false;
  blocked.parallel_follow_index = -1;
  blocked.parallel_follow_id.clear();
  blocked.parallel_follow_delta_s = std::numeric_limits<double>::infinity();
  blocked.parallel_follow_delta_d = 0.0;
  blocked.parallel_follow_rel_v = 0.0;
  blocked.parallel_follow_s_dot_mps = 0.0;
  blocked.parallel_follow_direction_known = false;
  blocked.parallel_follow_same_direction = true;

  if (!config_.parallel_follow_enabled || blocked.nearest_index >= 0 ||
      !ego.valid) {
    return;
  }
  const double max_s = std::isfinite(config_.parallel_follow_s_m) &&
                               config_.parallel_follow_s_m > 0.0
                           ? config_.parallel_follow_s_m
                           : config_.follow_trigger_s_m;
  const double same_corridor_width =
      std::max(0.0, config_.same_corridor_width_m);
  const double max_lateral_width =
      std::max(same_corridor_width, config_.parallel_follow_lateral_width_m);
  if (!std::isfinite(max_s) || max_s <= 0.0 ||
      !std::isfinite(max_lateral_width) ||
      max_lateral_width <= same_corridor_width) {
    return;
  }

  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto &opp = opponents[i];
    if (!opp.valid ||
        !inputTimestampFresh(now_sec, opp.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec)) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, opp.frenet.s);
    if (!std::isfinite(delta_s) || delta_s <= 0.0 || delta_s > max_s) {
      continue;
    }
    const double delta_d = opp.frenet.d - ego.frenet.d;
    const double abs_delta_d = std::abs(delta_d);
    if (!std::isfinite(abs_delta_d) || abs_delta_d <= same_corridor_width ||
        abs_delta_d > max_lateral_width) {
      continue;
    }
    const double s_dot = blocked_risk_.opponentSDot(opp);
    const bool direction_known = opp.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known || s_dot >= config_.same_direction_min_s_dot_mps;
    if (config_.same_direction_filter_enabled && direction_known &&
        !same_direction) {
      continue;
    }
    if (delta_s >= blocked.parallel_follow_delta_s) {
      continue;
    }

    blocked.parallel_follow_candidate = true;
    blocked.parallel_follow_hold_lateral = true;
    blocked.parallel_follow_index = static_cast<int>(i);
    blocked.parallel_follow_id = opp.id;
    blocked.parallel_follow_delta_s = delta_s;
    blocked.parallel_follow_delta_d = delta_d;
    blocked.parallel_follow_rel_v = ego.v - opp.v;
    blocked.parallel_follow_s_dot_mps = s_dot;
    blocked.parallel_follow_direction_known = direction_known;
    blocked.parallel_follow_same_direction = same_direction;
  }
}

// 入力: 現在時刻、自車、前方判定、相手車一覧。
// 出力: なし。停止/ほぼ停止した前方車だけをBlockedInfoへ明示する。
// 処理概要:
// parallel観測と分離し、fresh・同方向・前方・同一コリドー・有限TTCを満たす場合だけ分類する。
void OvertakePlannerCore::classifyStationaryFrontObstacle(
    double now_sec, const EgoState &ego, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.stationary_front_obstacle = false;
  blocked.stationary_front_id.clear();
  blocked.stationary_front_ttc_sec = std::numeric_limits<double>::infinity();
  if (blocked.nearest_index < 0 ||
      static_cast<std::size_t>(blocked.nearest_index) >= opponents.size() ||
      !std::isfinite(blocked.front_delta_s) || blocked.front_delta_s <= 0.0 ||
      !std::isfinite(blocked.front_delta_d) ||
      std::abs(blocked.front_delta_d) >
          std::max(0.0, config_.same_corridor_width_m) ||
      !std::isfinite(blocked.front_rel_v) || blocked.front_rel_v <= 0.0 ||
      (blocked.front_direction_known && !blocked.front_same_direction)) {
    return;
  }
  const auto &target =
      opponents[static_cast<std::size_t>(blocked.nearest_index)];
  if (!target.valid ||
      !inputTimestampFresh(now_sec, target.stamp_sec,
                           config_.opponent_stale_time_sec,
                           config_.input_future_stamp_tolerance_sec) ||
      !std::isfinite(target.v) ||
      target.v >
          std::max(0.0, config_.stationary_obstacle_speed_threshold_mps)) {
    return;
  }
  const double ttc_sec = blocked.front_delta_s / blocked.front_rel_v;
  if (!std::isfinite(ttc_sec) || ttc_sec <= 0.0) {
    return;
  }
  // egoを参照していることを明示し、NaN/負速度が分類を通らないようにする。
  if (!std::isfinite(ego.v) || ego.v < 0.0) {
    return;
  }
  blocked.stationary_front_obstacle = true;
  blocked.stationary_front_id = target.id;
  blocked.stationary_front_ttc_sec = ttc_sec;
}

// 入力: 現在時刻、自車、前方/並走判定、相手車一覧。
// 出力: なし。停止・低速前走車へ制動余裕を失う前からFOLLOW対象を記録する。
// 処理概要: 固定follow距離を広げるのではなく、CandidateBuilderと同じ遅れ・
// 最大制動モデルで必要距離を算出する。現d保持FOLLOW/RECOVERYも後段で
// SafetyEvaluatorを通るため、この分類だけで横軌道を許可することはない。
void OvertakePlannerCore::classifyBrakingFollowTarget(
    double now_sec, const EgoState &ego,
    const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.braking_follow_active = false;
  blocked.braking_follow_hold_lateral = false;
  blocked.braking_follow_feasible = false;
  blocked.braking_follow_candidate_reject_reason.clear();
  blocked.braking_follow_candidate_tracking_profile_valid = false;
  blocked.braking_follow_candidate_min_safety_margin =
      std::numeric_limits<double>::infinity();
  blocked.braking_follow_index = -1;
  blocked.braking_follow_id.clear();
  blocked.braking_follow_delta_s = std::numeric_limits<double>::infinity();
  blocked.braking_follow_required_distance_m =
      std::numeric_limits<double>::infinity();
  blocked.braking_follow_trigger_distance_m =
      std::numeric_limits<double>::infinity();
  blocked.braking_follow_available_distance_m =
      std::numeric_limits<double>::infinity();
  blocked.braking_follow_relative_speed_mps = 0.0;
  blocked.braking_follow_target_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  blocked.braking_follow_ttc_sec = std::numeric_limits<double>::infinity();
  blocked.braking_follow_speed_cap_mps =
      std::numeric_limits<double>::quiet_NaN();

  if (!config_.braking_follow_enabled || !ego.valid || !std::isfinite(ego.v) ||
      ego.v < 0.0 || !std::isfinite(config_.braking_follow_max_distance_m) ||
      config_.braking_follow_max_distance_m <= 0.0 ||
      !std::isfinite(config_.braking_follow_max_target_speed_mps) ||
      config_.braking_follow_max_target_speed_mps < 0.0 ||
      !std::isfinite(config_.braking_follow_trigger_margin_m) ||
      config_.braking_follow_trigger_margin_m < 0.0 ||
      !std::isfinite(config_.braking_follow_ttc_threshold_sec) ||
      !std::isfinite(config_.max_brake_decel_mps2) ||
      config_.max_brake_decel_mps2 <= 0.0 ||
      config_.max_brake_decel_mps2 > 1.5 ||
      !std::isfinite(config_.longitudinal_response_delay_sec) ||
      config_.longitudinal_response_delay_sec < 0.0) {
    return;
  }

  // 現dを横overrideとして出すため、低速車だけを見て入力欠損時に保持を
  // 継続してはいけない。Gate 2と同じく、全観測車・参照・MPCが健全な周期だけ
  // 横保持FOLLOW候補を作る。満たせない場合は後段のspeed-only fail-safeへ渡す。
  const bool lateral_hold_input_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && reentry_input.mpc_healthy;
  if (!lateral_hold_input_complete) {
    return;
  }

  const double same_corridor_width_m =
      std::max(0.0, config_.same_corridor_width_m);
  const double max_distance_m = config_.braking_follow_max_distance_m;
  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto &target = opponents[i];
    if (!target.valid ||
        !inputTimestampFresh(now_sec, target.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec) ||
        !std::isfinite(target.v) || target.v < 0.0) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, target.frenet.s);
    const double delta_d = target.frenet.d - ego.frenet.d;
    const bool active_start_grid_handoff_target =
        blocked.start_grid_target_superseded_by_blocked_front &&
        !blocked.start_grid_replacement_target_id.empty() &&
        target.id == blocked.start_grid_replacement_target_id;
    const bool active_pass_chain_target =
        localized_lateral_profile_.active &&
        !localized_lateral_profile_.pass_complete_confirmed &&
        (((isAnyPassMode(mode_) || mode_ == BehaviorMode::FOLLOW_BLOCKED) &&
          !localized_lateral_profile_.chain_tail_id.empty() &&
          target.id == localized_lateral_profile_.chain_tail_id) ||
         active_start_grid_handoff_target) &&
        std::abs(delta_d) <=
            std::max(same_corridor_width_m,
                     std::max(0.0, config_.parallel_side_margin_m));
    if (!std::isfinite(delta_s) || !std::isfinite(delta_d) || delta_s <= 0.0 ||
        delta_s > max_distance_m ||
        (std::abs(delta_d) > same_corridor_width_m &&
         !active_pass_chain_target)) {
      continue;
    }
    const double target_s_dot_mps = blocked_risk_.opponentSDot(target);
    const double reverse_tolerance_mps =
        std::max(0.0, config_.same_direction_min_s_dot_mps);
    if (!std::isfinite(target_s_dot_mps) ||
        target_s_dot_mps < -reverse_tolerance_mps ||
        target.v > config_.braking_follow_max_target_speed_mps) {
      continue;
    }
    const bool direction_known =
        target.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known ||
        target_s_dot_mps >= config_.same_direction_min_s_dot_mps;
    if (config_.same_direction_filter_enabled && direction_known &&
        !same_direction) {
      continue;
    }
    const double target_speed_along_track_mps = std::max(0.0, target_s_dot_mps);
    if (target_speed_along_track_mps >
        config_.braking_follow_max_target_speed_mps) {
      continue;
    }
    const double relative_speed_mps = ego.v - target_speed_along_track_mps;
    if (!std::isfinite(relative_speed_mps) ||
        relative_speed_mps <= std::max(0.0, config_.dv_block_threshold_mps)) {
      continue;
    }
    const double target_speed_mps = std::max(
        std::max(1.0e-3, config_.safe_stop_v_mps),
        target_speed_along_track_mps - config_.follow_speed_margin_mps);
    const double clamped_target_speed_mps =
        std::clamp(target_speed_mps, 0.0, ego.v);
    const double braking_distance_m =
        ego.v * config_.longitudinal_response_delay_sec +
        (ego.v * ego.v - clamped_target_speed_mps * clamped_target_speed_mps) /
            (2.0 * config_.max_brake_decel_mps2);
    const double required_distance_m =
        braking_distance_m + std::max(0.0, config_.safety_ellipse_a_m) +
        config_.braking_follow_trigger_margin_m;
    const double ttc_sec = delta_s / relative_speed_mps;
    const double ttc_trigger_distance_m =
        config_.braking_follow_ttc_threshold_sec > 0.0
            ? relative_speed_mps * config_.braking_follow_ttc_threshold_sec +
                  std::max(0.0, config_.safety_ellipse_a_m) +
                  config_.braking_follow_trigger_margin_m
            : 0.0;
    const double trigger_distance_m =
        std::max(required_distance_m, ttc_trigger_distance_m);
    if (!std::isfinite(required_distance_m) || !std::isfinite(ttc_sec) ||
        ttc_sec <= 0.0 || !std::isfinite(trigger_distance_m) ||
        delta_s > trigger_distance_m ||
        delta_s >= blocked.braking_follow_delta_s) {
      continue;
    }
    blocked.braking_follow_active = true;
    blocked.braking_follow_hold_lateral = true;
    blocked.braking_follow_index = static_cast<int>(i);
    blocked.braking_follow_id = target.id;
    blocked.braking_follow_delta_s = delta_s;
    blocked.braking_follow_required_distance_m = required_distance_m;
    blocked.braking_follow_trigger_distance_m = trigger_distance_m;
    blocked.braking_follow_available_distance_m =
        std::max(0.0, delta_s - std::max(0.0, config_.safety_ellipse_a_m));
    blocked.braking_follow_relative_speed_mps = relative_speed_mps;
    blocked.braking_follow_target_speed_mps = target_speed_along_track_mps;
    blocked.braking_follow_ttc_sec = ttc_sec;
    blocked.braking_follow_speed_cap_mps = clamped_target_speed_mps;
  }
}

// 入力: 現在時刻、自車、freshness契約、相手一覧。
// 出力: なし。FOLLOW制動を発動するより手前で、停止・低速対象をPASS候補生成へ
// 渡す一時分類だけをBlockedInfoへ記録する。
// 処理概要: 実測始端からの必要横遷移、PP required arc、safe-cycle/応答遅れ分を
// 同じ距離契約へ重ねる。braking_follow_activeや速度capは変更せず、後段の
// SafetyEvaluator・safe-cycle・exact ACK・motion grantをそのまま要求する。
void OvertakePlannerCore::classifyEarlyLowSpeedPassTarget(
    double now_sec, const EgoState &ego,
    const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.early_low_speed_pass_target_active = false;
  blocked.early_low_speed_pass_target_index = -1;
  blocked.early_low_speed_pass_target_id.clear();
  blocked.early_low_speed_pass_target_delta_s_m =
      std::numeric_limits<double>::infinity();
  blocked.early_low_speed_pass_target_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  blocked.early_low_speed_pass_required_distance_m =
      std::numeric_limits<double>::infinity();
  blocked.early_low_speed_pass_required_transition_m =
      std::numeric_limits<double>::infinity();
  blocked.early_low_speed_pass_required_controller_arc_m =
      std::numeric_limits<double>::infinity();

  const bool inputs_complete =
      reentry_input.ego_fresh && reentry_input.v2x_snapshot_fresh &&
      reentry_input.all_observed_opponents_fresh &&
      reentry_input.all_observed_opponents_included &&
      reentry_input.reference_valid && reentry_input.mpc_healthy &&
      reentry_input.mpc_health_fresh && !reentry_input.mpc_hard_failure;
  if (!config_.braking_follow_enabled || !inputs_complete || !ego.valid ||
      !std::isfinite(ego.v) || ego.v < 0.0 || !std::isfinite(ego.frenet.s) ||
      !std::isfinite(ego.frenet.d) ||
      !std::isfinite(config_.braking_follow_max_distance_m) ||
      config_.braking_follow_max_distance_m <= 0.0) {
    return;
  }

  const double low_speed_threshold_mps = std::min(
      std::max(0.0, config_.braking_follow_max_target_speed_mps),
      std::max(std::max(0.0, config_.stationary_obstacle_speed_threshold_mps),
               std::max(0.0, config_.slow_front_exception_speed_mps)));
  const double same_corridor_width_m =
      std::max(0.0, config_.same_corridor_width_m);
  const CandidateBuilder candidate_builder(frame_, config_);
  const double requested_transition_m = std::max(
      1.0, std::abs(config_.localized_avoidance_start_before_target_m -
                    config_.localized_avoidance_full_offset_before_target_m));
  const double controller_speed_mps =
      std::max(ego.v, std::max(0.0, config_.pass_speed_cap_mps));
  const double required_controller_arc_m =
      candidate_builder.requiredControllerSpatialHorizon(ego.v,
                                                         controller_speed_mps);
  const double longitudinal_clearance_m =
      std::max(0.0, config_.safety_ellipse_a_m) *
      std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h));
  const double safe_cycle_time_sec =
      std::ceil(std::max(1.0, config_.pass_safe_required_cycles)) /
      std::max(1.0, config_.control_rate_hz);
  const double delivery_reserve_m =
      ego.v * (std::max(0.0, config_.longitudinal_response_delay_sec) +
               safe_cycle_time_sec);

  int nearest_index = -1;
  double nearest_delta_s_m = std::numeric_limits<double>::infinity();
  double nearest_target_speed_mps = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto &target = opponents[i];
    if (!target.valid ||
        !inputTimestampFresh(now_sec, target.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec) ||
        !std::isfinite(target.v) || target.v < 0.0 ||
        target.v > low_speed_threshold_mps || !std::isfinite(target.frenet.s) ||
        !std::isfinite(target.frenet.d)) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, target.frenet.s);
    const double delta_d = target.frenet.d - ego.frenet.d;
    if (!std::isfinite(delta_s) || !std::isfinite(delta_d) || delta_s <= 0.0 ||
        delta_s > config_.braking_follow_max_distance_m ||
        std::abs(delta_d) > same_corridor_width_m) {
      continue;
    }
    const double target_s_dot_mps = blocked_risk_.opponentSDot(target);
    const double reverse_tolerance_mps =
        std::max(0.0, config_.same_direction_min_s_dot_mps);
    if (!std::isfinite(target_s_dot_mps) ||
        target_s_dot_mps < -reverse_tolerance_mps) {
      continue;
    }
    const bool direction_known =
        target.v >= config_.same_direction_min_speed_mps;
    if (config_.same_direction_filter_enabled && direction_known &&
        target_s_dot_mps < config_.same_direction_min_s_dot_mps) {
      continue;
    }
    const double target_speed_along_track_mps = std::max(0.0, target_s_dot_mps);
    const double relative_speed_mps = ego.v - target_speed_along_track_mps;
    if (!std::isfinite(relative_speed_mps) ||
        relative_speed_mps <= std::max(0.0, config_.dv_block_threshold_mps)) {
      continue;
    }
    if (delta_s < nearest_delta_s_m) {
      nearest_index = static_cast<int>(i);
      nearest_delta_s_m = delta_s;
      nearest_target_speed_mps = target_speed_along_track_mps;
    }
  }
  if (nearest_index < 0 ||
      static_cast<std::size_t>(nearest_index) >= opponents.size()) {
    return;
  }

  const auto &target = opponents[static_cast<std::size_t>(nearest_index)];
  const double left_target_d = targetOffsetForPass(
      CandidateType::PASS_LEFT, ego.frenet.d, target.frenet.d);
  const double right_target_d = targetOffsetForPass(
      CandidateType::PASS_RIGHT, ego.frenet.d, target.frenet.d);
  const double left_transition_m =
      candidate_builder.minimumTrackableLateralShiftDistance(
          ego, left_target_d, controller_speed_mps, blocked,
          requested_transition_m);
  const double right_transition_m =
      candidate_builder.minimumTrackableLateralShiftDistance(
          ego, right_target_d, controller_speed_mps, blocked,
          requested_transition_m);
  const double required_transition_m =
      std::min(left_transition_m, right_transition_m);
  const double required_distance_m =
      longitudinal_clearance_m +
      std::max(required_transition_m, required_controller_arc_m) +
      delivery_reserve_m;
  if (!std::isfinite(required_transition_m) ||
      !std::isfinite(required_controller_arc_m) ||
      !std::isfinite(required_distance_m) ||
      nearest_delta_s_m > required_distance_m) {
    return;
  }
  blocked.early_low_speed_pass_target_active = true;
  blocked.early_low_speed_pass_target_index = nearest_index;
  blocked.early_low_speed_pass_target_id = target.id;
  blocked.early_low_speed_pass_target_delta_s_m = nearest_delta_s_m;
  blocked.early_low_speed_pass_target_speed_mps = nearest_target_speed_mps;
  blocked.early_low_speed_pass_required_distance_m = required_distance_m;
  blocked.early_low_speed_pass_required_transition_m = required_transition_m;
  blocked.early_low_speed_pass_required_controller_arc_m =
      required_controller_arc_m;
}

// 入力: publish直前のPlannerOutput、基準候補、相手予測。
// 出力: 実際にpublishする横offset列が安全ならtrue。
// 処理概要: rate limit/hold後のd列をCartesianへ再構成し、候補評価時と同じ
// 楕円・壁制約に加え、PASSは最終d列のPP操舵契約も再評価する。
bool OvertakePlannerCore::revalidatePublishedLateral(
    const PlannerOutput &output, const CandidateTrajectory &base_candidate,
    const EgoState &ego, const std::vector<OpponentState> &opponents,
    double now_sec,
    const std::vector<PredictedOpponent> &default_predictions) const {
  // PASSだけでなく、通常ラインへ戻るRECOVERYと停止車early PASS不成立時の
  // 現d保持RECOVERYも、rate limit/hold後の形を再評価する。
  // SAFE_STOP・速度のみfallbackはそれぞれ専用の安全/下流fallback経路を維持する。
  const bool reentry_recovery =
      (output.reentry_gate.requested ||
       output.blocked_info.post_abort_curve_hold_active) &&
      base_candidate.type == CandidateType::RECOVERY;
  const bool pass_reauthorization_recovery =
      output.blocked_info.pass_reauthorization_lockout_active &&
      base_candidate.type == CandidateType::RECOVERY;
  const bool early_stationary_hold_recovery =
      output.blocked_info.early_stationary_parallel_pass_hold_lateral &&
      base_candidate.type == CandidateType::RECOVERY;
  const bool braking_follow_hold =
      output.blocked_info.braking_follow_hold_lateral &&
      (base_candidate.type == CandidateType::FOLLOW ||
       base_candidate.type == CandidateType::RECOVERY);
  const bool attack_follow_hold =
      output.blocked_info.attack_follow_hold_pass_side &&
      base_candidate.type == CandidateType::FOLLOW;
  const bool prestart_attack_follow_hold =
      output.blocked_info.prestart_attack_follow_hold_lateral &&
      base_candidate.type == CandidateType::FOLLOW;
  const bool transaction_safe_lateral_hold =
      output.blocked_info.maneuver_transaction_safe_lateral_hold_active &&
      (base_candidate.type == CandidateType::YIELD_BEHIND ||
       base_candidate.type == CandidateType::RECOVERY ||
       base_candidate.type == CandidateType::SAFE_STOP);
  if (!output.active_override ||
      (!isPassCandidate(base_candidate.type) && !reentry_recovery &&
       !pass_reauthorization_recovery && !early_stationary_hold_recovery &&
       !braking_follow_hold && !attack_follow_hold &&
       !prestart_attack_follow_hold && !transaction_safe_lateral_hold)) {
    return true;
  }
  if (!base_candidate.longitudinal_profile_valid) {
    return false;
  }
  if (output.lateral_offsets.empty() ||
      output.lateral_offsets.size() != base_candidate.s.size()) {
    return false;
  }
  const bool published_profile_changed =
      base_candidate.d.size() != output.lateral_offsets.size() ||
      !std::equal(base_candidate.d.begin(), base_candidate.d.end(),
                  output.lateral_offsets.begin(),
                  [](double candidate_d_m, double published_d_m) {
                    return std::isfinite(candidate_d_m) &&
                           std::isfinite(published_d_m) &&
                           std::abs(candidate_d_m - published_d_m) <= 1.0e-9;
                  });
  CandidateTrajectory published = base_candidate;
  published.d = output.lateral_offsets;
  published.x.clear();
  published.y.clear();
  published.yaw.clear();
  published.x.reserve(published.s.size());
  published.y.reserve(published.s.size());
  published.yaw.reserve(published.s.size());
  for (std::size_t i = 0; i < published.s.size(); ++i) {
    const auto point = frame_.frenetToCartesian(published.s[i], published.d[i]);
    published.x.push_back(point.x);
    published.y.push_back(point.y);
    published.yaw.push_back(point.yaw);
  }
  if (!evaluateCandidateAtOwnTimeAxis(published, opponents, now_sec,
                                      default_predictions)) {
    return false;
  }
  return !published_profile_changed ||
         CandidateBuilder(frame_, config_)
             .publishedLateralProfileTrackable(published, ego);
}

// 入力: 現在のBlockedInfo。
// 出力: 低速前走車例外が有効になったならtrue。
// 処理概要:
// 追い越し禁止区間でも、前走車が低速で一定周期続いた時だけ開始許可へ戻す。
bool OvertakePlannerCore::updateSlowFrontException(const BlockedInfo &blocked) {
  if (!config_.slow_front_exception_enabled ||
      !blocked.front_vehicle_low_speed || blocked.nearest_id.empty()) {
    slow_front_exception_count_ = 0;
    slow_front_exception_id_.clear();
    return false;
  }
  if (blocked.nearest_id == slow_front_exception_id_) {
    ++slow_front_exception_count_;
  } else {
    slow_front_exception_id_ = blocked.nearest_id;
    slow_front_exception_count_ = 1;
  }
  const int required_cycles =
      std::max(1, config_.slow_front_exception_required_cycles);
  return slow_front_exception_count_ >= required_cycles;
}

// 入力: 現在時刻、自車状態、BlockedInfo。
// 出力: スタート直後のsafe stopを抑制するならtrue。
// 処理概要:
// 低速スタート直後の横並び/未来譲りで、すぐ停止に落ちるのを短時間だけ避ける。
bool OvertakePlannerCore::shouldSuppressSafeStopForStartGrace(
    double now_sec, const EgoState &ego, const BlockedInfo &blocked) const {
  if (!config_.start_grace_safe_stop_enabled ||
      config_.start_grace_duration_sec <= 0.0 ||
      !std::isfinite(first_valid_update_sec_) || !std::isfinite(now_sec) ||
      now_sec < first_valid_update_sec_) {
    return false;
  }
  const double start_grace_reference_sec =
      std::isfinite(first_motion_update_sec_) ? first_motion_update_sec_
                                              : now_sec;
  if (now_sec < start_grace_reference_sec ||
      now_sec - start_grace_reference_sec > config_.start_grace_duration_sec) {
    return false;
  }
  if (config_.start_grace_max_speed_mps >= 0.0 &&
      ego.v > config_.start_grace_max_speed_mps) {
    return false;
  }
  if (blocked.blocked) {
    return false;
  }
  return blocked.side_by_side || blocked.parallel_side_candidate ||
         blocked.future_side_by_side || blocked.future_yield_required;
}

// 入力: BlockedInfoと、先行扱いに必要な相手後方距離margin。
// 出力: 自車が横並び/並走相手より明確に前ならtrue。
// 処理概要:
// 別の前方車に塞がれていない場面で、対象相手が後ろにいることをs差で判定する。
bool OvertakePlannerCore::leaderPriorityCandidate(const BlockedInfo &blocked,
                                                  double margin_m,
                                                  std::string &target_id,
                                                  double &target_delta_s,
                                                  std::string &reason) const {
  if (!config_.side_by_side_leader_priority_enabled) {
    return false;
  }
  const double required_margin = std::max(0.0, margin_m);
  const auto is_leading = [required_margin](double delta_s) {
    return std::isfinite(delta_s) && delta_s <= -required_margin;
  };
  const auto blocked_by_other_front = [&blocked](const std::string &id) {
    return blocked.blocked && !blocked.nearest_id.empty() &&
           blocked.nearest_id != id;
  };
  if (blocked.side_by_side && blocked.side_same_direction &&
      !blocked.side_id.empty() && !blocked_by_other_front(blocked.side_id) &&
      is_leading(blocked.side_delta_s)) {
    target_id = blocked.side_id;
    target_delta_s = blocked.side_delta_s;
    reason = "side_by_side_leader";
    return true;
  }
  if (blocked.parallel_side_candidate && blocked.parallel_side_same_direction &&
      !blocked.parallel_side_id.empty() &&
      !blocked_by_other_front(blocked.parallel_side_id) &&
      is_leading(blocked.parallel_side_delta_s)) {
    target_id = blocked.parallel_side_id;
    target_delta_s = blocked.parallel_side_delta_s;
    reason = "parallel_side_leader";
    return true;
  }
  return false;
}

// 入力: 現在時刻とBlockedInfo。
// 出力: なし。BlockedInfoへleader priority情報を付与する。
// 処理概要:
// 先行/後続の優先権が毎周期入れ替わらないよう、解除側に小さなヒステリシスと保持時間を持たせる。
void OvertakePlannerCore::updateLeaderPriority(double now_sec,
                                               BlockedInfo &blocked) {
  blocked.leader_priority_active = false;
  blocked.leader_priority_latched = false;
  blocked.leader_priority_id.clear();
  blocked.leader_priority_delta_s = std::numeric_limits<double>::infinity();
  blocked.leader_priority_reason.clear();

  if (!config_.side_by_side_leader_priority_enabled) {
    leader_priority_hold_active_ = false;
    leader_priority_hold_id_.clear();
    leader_priority_hold_until_sec_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  std::string target_id;
  double target_delta_s = std::numeric_limits<double>::infinity();
  std::string reason;
  bool active = leaderPriorityCandidate(
      blocked, config_.side_by_side_leader_priority_enter_s_m, target_id,
      target_delta_s, reason);
  bool latched = false;

  const bool hold_window_active =
      leader_priority_hold_active_ && std::isfinite(now_sec) &&
      std::isfinite(leader_priority_hold_until_sec_) &&
      now_sec <= leader_priority_hold_until_sec_;
  if (active && hold_window_active && target_id == leader_priority_hold_id_) {
    latched = true;
    reason = "leader_priority_hold";
  }

  if (!active && hold_window_active) {
    const double release_margin = std::min(
        std::max(0.0, config_.side_by_side_leader_priority_enter_s_m),
        std::max(0.0, config_.side_by_side_leader_priority_release_s_m));
    std::string held_id;
    double held_delta_s = std::numeric_limits<double>::infinity();
    std::string held_reason;
    if (leaderPriorityCandidate(blocked, release_margin, held_id, held_delta_s,
                                held_reason) &&
        held_id == leader_priority_hold_id_) {
      active = true;
      latched = true;
      target_id = held_id;
      target_delta_s = held_delta_s;
      reason = "leader_priority_hold";
    }
  }

  if (!active) {
    if (!std::isfinite(now_sec) ||
        !std::isfinite(leader_priority_hold_until_sec_) ||
        now_sec > leader_priority_hold_until_sec_) {
      leader_priority_hold_active_ = false;
      leader_priority_hold_id_.clear();
      leader_priority_hold_until_sec_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    return;
  }

  blocked.leader_priority_active = true;
  blocked.leader_priority_latched = latched;
  blocked.leader_priority_id = target_id;
  blocked.leader_priority_delta_s = target_delta_s;
  blocked.leader_priority_reason = reason;
  leader_priority_hold_active_ = true;
  leader_priority_hold_id_ = target_id;
  leader_priority_hold_until_sec_ =
      now_sec + std::max(0.0, config_.side_by_side_leader_priority_hold_sec);
}

// 入力: なし。
// 出力: 局所横プロファイルモードが有効ならtrue。
// 処理概要:
// legacyとlocalized_latchedを切り替え、実験機能をパラメータで隔離する。
bool OvertakePlannerCore::localizedLateralProfileEnabled() const {
  return config_.overtake_lateral_profile_mode == "localized_latched";
}

// 入力: 現在のBlockedInfo。
// 出力: 優先するPASS候補種別。使えない場合はFASTEST。
// 処理概要: 既に追い越し中の方向を優先し、未開始なら通れる側を選ぶ。
CandidateType
OvertakePlannerCore::preferredPassType(const BlockedInfo &blocked) const {
  if (mode_ == BehaviorMode::OVERTAKE_LEFT) {
    return CandidateType::PASS_LEFT;
  }
  if (mode_ == BehaviorMode::OVERTAKE_RIGHT) {
    return CandidateType::PASS_RIGHT;
  }
  if (blocked.start_grid_target_active &&
      std::isfinite(blocked.start_grid_target_delta_d) &&
      std::abs(blocked.start_grid_target_delta_d) >
          std::max(1.0e-3, config_.same_corridor_width_m)) {
    // grid隣接車が自車と別コリドーにいる時、壁gapの大きい反対側を選ぶと
    // 停止車を横切る長いprofileをラッチしてしまう。相手が右(delta_d<0)
    // なら現在の左側、相手が左なら現在の右側を候補化する。race arm前に
    // early-stationaryとして作られた同一対象ラッチより、この明確な配置を優先する。
    return blocked.start_grid_target_delta_d < 0.0 ? CandidateType::PASS_LEFT
                                                   : CandidateType::PASS_RIGHT;
  }
  if (mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT) {
    return CandidateType::PASS_LEFT;
  }
  if (mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT) {
    return CandidateType::PASS_RIGHT;
  }
  if (blocked.early_stationary_parallel_pass_target &&
      localized_lateral_profile_.active &&
      blocked.early_stationary_parallel_pass_id ==
          localized_lateral_profile_.target_id &&
      (localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT ||
       localized_lateral_profile_.pass_type == CandidateType::PASS_RIGHT)) {
    // 同一停止対象がparallelからfrontへ分類変更されても、静的gapの揺れで
    // 反対側PASSへ切り替えず、ラッチ済みの同側profileを再評価し続ける。
    return localized_lateral_profile_.pass_type;
  }
  if (blocked.can_pass_left) {
    return CandidateType::PASS_LEFT;
  }
  if (blocked.can_pass_right) {
    return CandidateType::PASS_RIGHT;
  }
  const bool stationary_curve_preflight_profile =
      config_.stationary_no_pass_safe_pass_enabled &&
      (blocked.braking_follow_active ||
       blocked.early_low_speed_pass_target_active) &&
      (!blocked.braking_follow_id.empty() ||
       !blocked.early_low_speed_pass_target_id.empty()) &&
      blocked.overtake_start_gate_reason == "curve" &&
      std::isfinite(blocked.overtake_start_abs_curvature) &&
      blocked.overtake_start_abs_curvature > 0.0 &&
      blocked.overtake_start_abs_curvature <=
          config_.stationary_no_pass_safe_pass_max_curvature_m_inv &&
      std::isfinite(blocked.ego_speed_mps) &&
      blocked.ego_speed_mps <=
          std::min(
              config_.stationary_no_pass_safe_pass_v_max_mps,
              std::sqrt(
                  config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 /
                  blocked.overtake_start_abs_curvature)) +
              1.0e-6;
  if (stationary_curve_preflight_profile) {
    // curve gateを開く前に、同じbraking-follow targetへactual-pose始端の
    // localized profileを用意する。これは候補生成の準備だけで、後段の
    // freshness、全相手、wall、CBF、PP、safe-cycleを通るまでauthorityはない。
    return blocked.left_pass_gap_m >= blocked.right_pass_gap_m
               ? CandidateType::PASS_LEFT
               : CandidateType::PASS_RIGHT;
  }
  if ((blocked.blocked || blocked.early_stationary_parallel_pass_target ||
       blocked.braking_follow_active ||
       blocked.early_low_speed_pass_target_active) &&
      blocked.straight_overtake_start_allowed &&
      blocked.overtake_permission_allowed) {
    // 静的gapが両側とも狭くても、候補安全評価用の局所プロファイルは広い診断側へ固定する。
    return blocked.left_pass_gap_m >= blocked.right_pass_gap_m
               ? CandidateType::PASS_LEFT
               : CandidateType::PASS_RIGHT;
  }
  return CandidateType::FASTEST;
}

// 入力: 現在のBlockedInfo。
// 出力: 局所回避プロファイルの対象相手index。無ければ-1。
// 処理概要: 前方閉塞車両、横並び車両、並走候補の順で対象を選ぶ。
int OvertakePlannerCore::localizedProfileTargetIndex(
    const BlockedInfo &blocked) const {
  if (blocked.start_grid_target_active &&
      blocked.start_grid_target_index >= 0) {
    return blocked.start_grid_target_index;
  }
  if (blocked.nearest_index >= 0) {
    return blocked.nearest_index;
  }
  if (blocked.braking_follow_active && blocked.braking_follow_index >= 0) {
    // 通常lookahead外でも、制動距離/TTCから先に検出した低速・停止車は
    // PASS候補のauthoritative targetである。ここへindexを渡さないと、
    // pass_probe_contextだけがtrueになり、target IDを持たない固定offset
    // PASSを生成できてしまう。停止車を無視せず、同じfresh IDへ局所profileと
    // SafetyEvaluatorを必ず結び付ける。
    return blocked.braking_follow_index;
  }
  if (blocked.early_low_speed_pass_target_active &&
      blocked.early_low_speed_pass_target_index >= 0) {
    return blocked.early_low_speed_pass_target_index;
  }
  if (blocked.side_index >= 0) {
    return blocked.side_index;
  }
  return blocked.parallel_side_index;
}

// 入力: 現在時刻、自車状態、BlockedInfo、相手車一覧。
// 出力: なし。localized_lateral_profile_を更新またはクリアする。
// 処理概要:
// PASS文脈で対象ID・side・目標dをラッチし、freshな同一IDのsだけ前進追従させる。
void OvertakePlannerCore::updateLocalizedLateralProfile(
    double now_sec, const EgoState &ego, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) {
  blocked.maneuver_target_latched = false;
  blocked.maneuver_target_id.clear();
  blocked.maneuver_target_index = -1;
  blocked.maneuver_target_observed = false;
  blocked.maneuver_target_fresh = false;
  blocked.maneuver_target_age_sec = std::numeric_limits<double>::infinity();
  blocked.maneuver_target_relative_s_m =
      std::numeric_limits<double>::infinity();
  blocked.maneuver_target_relative_d_m =
      std::numeric_limits<double>::quiet_NaN();
  blocked.maneuver_target_relative_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  blocked.maneuver_target_pass_geometric_complete = false;
  blocked.maneuver_target_pass_safety_approved = false;
  blocked.maneuver_target_pass_complete = false;
  blocked.maneuver_target_safety_evaluated = false;
  blocked.maneuver_target_pass_candidate_feasible = false;
  blocked.maneuver_target_pass_min_safety_margin =
      std::numeric_limits<double>::quiet_NaN();
  blocked.maneuver_target_pass_reject_reason.clear();
  blocked.maneuver_target_previous_id.clear();
  blocked.maneuver_target_new_id.clear();
  blocked.maneuver_target_change_reason.clear();
  blocked.maneuver_unstarted_target_released = false;
  blocked.maneuver_unstarted_target_reacquire_suppressed = false;
  blocked.maneuver_unstarted_target_pulling_away_cycles = 0;
  blocked.maneuver_pass_lateral_progress_m =
      std::numeric_limits<double>::quiet_NaN();
  blocked.maneuver_chain_tail_id.clear();
  blocked.maneuver_chain_tail_index = -1;
  blocked.maneuver_chain_target_count = 0;
  blocked.maneuver_chain_tail_observed = false;
  blocked.maneuver_chain_tail_relative_s_m =
      std::numeric_limits<double>::infinity();
  blocked.maneuver_chain_tail_relative_speed_mps =
      std::numeric_limits<double>::quiet_NaN();

  if (!localizedLateralProfileEnabled()) {
    clearLocalizedLateralProfile();
    return;
  }
  // modeが一時的にABORT/YIELDへ落ちても、認可済みtarget ID/side/profileを
  // 別車へ張り替えない。transactionの寿命はFSM名ではなく、固定targetを
  // 抜き切ったか、入力契約を失ったかで決める。
  const bool active_pass_profile = localized_lateral_profile_.active;
  // Gate 2が一度も認可していないprofileは候補生成の準備データであり、
  // target ID/sideを固定するPASS取引ではない。Gate 2認可だけでもsideの再選択は
  // 止めるが、未完了transactionは同じPASS payloadを実publishした後に開始する。
  const bool committed_pass_transaction =
      active_pass_profile &&
      localized_lateral_profile_.pass_safety_approved_once;
  const bool incomplete_attack_follow_transaction =
      committed_pass_transaction &&
      !localized_lateral_profile_.pass_complete_confirmed;
  if (!blocked.blocked && !blocked.early_stationary_parallel_pass_target &&
      !blocked.braking_follow_active &&
      !blocked.early_low_speed_pass_target_active && !blocked.side_by_side &&
      !isPassMode(mode_) && !incomplete_attack_follow_transaction) {
    // 広いparallel観測だけではPASS文脈を作らず、局所回避プロファイルも
    // ラッチしない。一方、制動距離/TTCで先に拾った低速・停止車は通常の
    // blocked lookahead外でもauthoritativeなPASS対象として保持する。
    clearLocalizedLateralProfile();
    return;
  }

  const bool start_grid_side_is_explicit =
      blocked.start_grid_target_active &&
      std::isfinite(blocked.start_grid_target_delta_d) &&
      std::abs(blocked.start_grid_target_delta_d) >
          std::max(1.0e-3, config_.same_corridor_width_m);
  const CandidateType start_grid_pass_type =
      blocked.start_grid_target_delta_d < 0.0 ? CandidateType::PASS_LEFT
                                              : CandidateType::PASS_RIGHT;
  const bool replace_prepared_start_grid_profile =
      active_pass_profile &&
      !localized_lateral_profile_.pass_safety_approved_once &&
      start_grid_side_is_explicit &&
      (mode_ == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
       mode_ == BehaviorMode::PREPARE_OVERTAKE_RIGHT) &&
      localized_lateral_profile_.target_id == blocked.start_grid_target_id &&
      localized_lateral_profile_.pass_type != start_grid_pass_type;
  bool completed_chain_handoff_expected = false;
  int completed_chain_target_index = -1;
  if (active_pass_profile &&
      localized_lateral_profile_.pass_complete_confirmed) {
    const auto current_waypoint = std::find_if(
        localized_lateral_profile_.chain_waypoints.begin(),
        localized_lateral_profile_.chain_waypoints.end(),
        [this](const LocalizedLateralWaypoint &waypoint) {
          return waypoint.target_id == localized_lateral_profile_.target_id;
        });
    if (current_waypoint != localized_lateral_profile_.chain_waypoints.end() &&
        std::next(current_waypoint) !=
            localized_lateral_profile_.chain_waypoints.end()) {
      completed_chain_handoff_expected = true;
      const std::string &next_target_id =
          std::next(current_waypoint)->target_id;
      const auto next_target = std::find_if(
          opponents.begin(), opponents.end(),
          [this, now_sec, &next_target_id](const OpponentState &opponent) {
            return opponent.id == next_target_id && opponent.valid &&
                   inputTimestampFresh(
                       now_sec, opponent.stamp_sec,
                       config_.opponent_stale_time_sec,
                       config_.input_future_stamp_tolerance_sec);
          });
      if (next_target != opponents.end()) {
        completed_chain_target_index =
            static_cast<int>(std::distance(opponents.begin(), next_target));
      }
    }
  }
  const CandidateType pass_type = replace_prepared_start_grid_profile
                                      ? start_grid_pass_type
                                  : (incomplete_attack_follow_transaction ||
                                     completed_chain_target_index >= 0)
                                      ? localized_lateral_profile_.pass_type
                                      : preferredPassType(blocked);
  const int classified_target_index = localizedProfileTargetIndex(blocked);
  int target_index = completed_chain_target_index >= 0
                         ? completed_chain_target_index
                         : classified_target_index;
  if (incomplete_attack_follow_transaction &&
      !replace_prepared_start_grid_profile) {
    // front/side/parallelの分類が変わっても、freshな全相手一覧からラッチIDを
    // 引き直す。別のnearestへ暗黙にすり替えない。
    const auto latched_it = std::find_if(
        opponents.begin(), opponents.end(), [this, now_sec](const auto &opp) {
          return opp.id == localized_lateral_profile_.target_id && opp.valid &&
                 inputTimestampFresh(now_sec, opp.stamp_sec,
                                     config_.opponent_stale_time_sec,
                                     config_.input_future_stamp_tolerance_sec);
        });
    if (latched_it == opponents.end()) {
      blocked.maneuver_target_latched = true;
      blocked.maneuver_target_id = localized_lateral_profile_.target_id;
      blocked.maneuver_target_previous_id =
          localized_lateral_profile_.target_id;
      blocked.maneuver_target_change_reason = "latched_target_missing_or_stale";
      // 未観測対象を除外したPASSを作らない。profile自体はABORT診断用に保持し、
      // 状態機械を明示的なtarget stale経路へ閉じる。
      return;
    }
    target_index =
        static_cast<int>(std::distance(opponents.begin(), latched_it));
  } else if (active_pass_profile &&
             localized_lateral_profile_.pass_complete_confirmed &&
             completed_chain_handoff_expected &&
             completed_chain_target_index < 0) {
    // 列内の次targetを観測できない周期に、さらに先のclassified targetへ
    // 飛ばしたり中心へmergeしたりしない。現profileを保持して次周期を待つ。
    blocked.maneuver_target_latched = true;
    blocked.maneuver_target_id = localized_lateral_profile_.target_id;
    blocked.maneuver_target_previous_id = localized_lateral_profile_.target_id;
    blocked.maneuver_target_change_reason =
        "next_chain_target_missing_or_stale";
    return;
  } else if (active_pass_profile &&
             localized_lateral_profile_.pass_complete_confirmed &&
             target_index >= 0 &&
             static_cast<std::size_t>(target_index) < opponents.size() &&
             opponents[static_cast<std::size_t>(target_index)].id !=
                 localized_lateral_profile_.target_id) {
    // 前対象を抜いた後も、同じslow chainの次対象なら初回から認可・publish
    // 済みのstaged profileを作り直さない。ここで現在egoから新規profileへ
    // 張り直すと、merge_front_gapを確保した時点で次車まで約1--2 mしかなく、
    // 本来すでに進んでいた外側dを失ってuntrackableとなる。target IDだけを
    // 次waypointへ進め、side/全waypoint d/初期transitionを保持する。
    const std::string previous_target_id = localized_lateral_profile_.target_id;
    const std::string &next_target_id =
        opponents[static_cast<std::size_t>(target_index)].id;
    const auto next_waypoint = std::find_if(
        localized_lateral_profile_.chain_waypoints.begin(),
        localized_lateral_profile_.chain_waypoints.end(),
        [&next_target_id](const LocalizedLateralWaypoint &waypoint) {
          return waypoint.target_id == next_target_id;
        });
    if (next_waypoint == localized_lateral_profile_.chain_waypoints.end() ||
        !std::isfinite(next_waypoint->target_s_m) ||
        !std::isfinite(next_waypoint->target_d_m)) {
      clearLocalizedLateralProfile();
      return;
    }
    localized_lateral_profile_.target_id = next_waypoint->target_id;
    localized_lateral_profile_.target_s_m = next_waypoint->target_s_m;
    localized_lateral_profile_.target_d_m = next_waypoint->target_d_m;
    localized_lateral_profile_.pass_complete_confirmed = false;
    localized_lateral_profile_.pass_start_target_continuity_cycles = 0;
    localized_lateral_profile_.unstarted_target_pulling_away_cycles = 0;
    localized_lateral_profile_.pass_tracking_continuity_armed = false;
    localized_lateral_profile_.pass_tracking_proof_published_last_cycle = false;
    // staged profileの横形状は継続しても、target IDはtyped plan契約の一部。
    // 新しいIDを旧generationのcontroller proofで直接実行せず、同じPASS sideを
    // 保ったcurrent-d STOPから新tokenのwarm-upを行う。
    maneuver_execution_hold_active_ = true;
    maneuver_execution_hold_target_id_ = next_waypoint->target_id;
    maneuver_execution_hold_pass_type_ = localized_lateral_profile_.pass_type;
    blocked.maneuver_target_previous_id = previous_target_id;
    blocked.maneuver_target_new_id = next_waypoint->target_id;
    blocked.maneuver_target_change_reason = "previous_chain_target_passed";
    if (committed_pass_snapshot_valid_ &&
        committed_pass_spatial_profile_frozen_ &&
        committed_pass_snapshot_target_id_ == previous_target_id &&
        committed_pass_snapshot_pass_type_ ==
            localized_lateral_profile_.pass_type) {
      // fixed prefixとchain continuationは同じpublish済みtransactionなので、
      // target identityだけを次waypointへ進める。次周期候補はfreshな全相手
      // SafetyEvaluatorとcontroller追従性を通るまで実行されない。
      committed_pass_snapshot_target_id_ = next_waypoint->target_id;
      committed_pass_snapshot_.planned_target_d_m = next_waypoint->target_d_m;
      committed_pass_profile_snapshot_.target_id = next_waypoint->target_id;
      committed_pass_profile_snapshot_.target_s_m = next_waypoint->target_s_m;
      committed_pass_profile_snapshot_.target_d_m = next_waypoint->target_d_m;
      committed_pass_profile_snapshot_.pass_complete_confirmed = false;
    }
  }
  const bool target_valid =
      target_index >= 0 &&
      static_cast<std::size_t>(target_index) < opponents.size() &&
      opponents[static_cast<std::size_t>(target_index)].valid &&
      inputTimestampFresh(
          now_sec, opponents[static_cast<std::size_t>(target_index)].stamp_sec,
          config_.opponent_stale_time_sec,
          config_.input_future_stamp_tolerance_sec);
  const bool pass_context = pass_type == CandidateType::PASS_LEFT ||
                            pass_type == CandidateType::PASS_RIGHT ||
                            isPassMode(mode_) ||
                            incomplete_attack_follow_transaction;
  const bool profile_min_hold_active =
      localized_lateral_profile_.active &&
      std::isfinite(localized_lateral_profile_.created_time_sec) &&
      now_sec - localized_lateral_profile_.created_time_sec <
          std::max(0.0, config_.maneuver_latch_min_hold_sec);

  if (!target_valid || (!pass_context && !profile_min_hold_active)) {
    clearLocalizedLateralProfile();
    return;
  }

  const auto &target = opponents[static_cast<std::size_t>(target_index)];
  const double released_target_interaction_gap_m = std::max(
      config_.unstarted_pass_target_release_min_gap_m,
      std::max(0.0, config_.localized_avoidance_start_before_target_m) +
          std::max(0.0, config_.safety_ellipse_a_m) *
              std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h)));
  const double released_target_relative_s_m =
      signedDeltaS(ego.frenet.s, target.frenet.s, frame_.length());
  const double released_target_relative_speed_mps = ego.v - target.v;
  const bool released_target_still_pulling_away =
      !unstarted_pass_released_target_id_.empty() &&
      target.id == unstarted_pass_released_target_id_ &&
      std::isfinite(released_target_relative_s_m) &&
      released_target_relative_s_m >= released_target_interaction_gap_m &&
      std::isfinite(released_target_relative_speed_mps) &&
      released_target_relative_speed_mps <=
          -config_.unstarted_pass_target_release_min_opening_speed_mps;
  if (released_target_still_pulling_away) {
    // 前周期に物理離脱を確認した同じ対象を再ラッチすると、profile生成と解放を
    // 周期ごとに繰り返してmode/generationが揺れる。対象が離れ続ける間だけ
    // PASS probeを閉じ、通常のFREE/FOLLOW評価へ戻す。別IDやcatch可能な相対速度
    // になれば下で抑止を解除し、改めてGate 2から評価する。
    blocked.maneuver_unstarted_target_released = true;
    blocked.maneuver_unstarted_target_reacquire_suppressed = true;
    blocked.maneuver_target_previous_id = unstarted_pass_released_target_id_;
    blocked.maneuver_target_change_reason =
        "unstarted_pass_released_target_reacquire_suppressed";
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason =
        "unstarted_pass_released_target_reacquire_suppressed";
    blocked.pass_gap_reason =
        "unstarted_pass_released_target_reacquire_suppressed";
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
    return;
  }
  if (!unstarted_pass_released_target_id_.empty()) {
    // 新しい対象へhandoffした、または同じ対象を再び追いつける物理状態に
    // なったため、永久ID blacklistにせず新規Gate 2評価を許可する。
    unstarted_pass_released_target_id_.clear();
  }
  if (localized_lateral_profile_.active) {
    const bool same_target = target.id == localized_lateral_profile_.target_id;
    const bool same_direction =
        pass_type == localized_lateral_profile_.pass_type ||
        pass_type == CandidateType::FASTEST;
    if (replace_prepared_start_grid_profile && same_target) {
      // PREPAREは横軌道の実行前なので、race arm前に壁gapだけで作られた側を
      // 同じstart-grid対象の実配置で訂正できる。最低hold時間はチャタリング用
      // であり、この一度限りの契約訂正には適用しない。新profileは後段で通常の
      // corridor/tracking/SafetyEvaluatorを通過しなければ採用されない。
      clearLocalizedLateralProfile();
    } else if (!same_target &&
               !localized_lateral_profile_.pass_safety_approved_once) {
      // 未認可profileは候補準備データであってtarget保持契約ではない。分類対象が
      // 変わった周期に最低holdを優先すると、旧IDのprofileへ新IDの観測値を
      // 詰めた混在候補をGate 2が認可し得るため、ID変更だけは即時作り直す。
      // 認可後は上のlatched target解決分岐を通り、この経路へは入らない。
      blocked.maneuver_target_previous_id =
          localized_lateral_profile_.target_id;
      clearLocalizedLateralProfile();
    } else if ((!same_target || !same_direction) && !profile_min_hold_active) {
      clearLocalizedLateralProfile();
    }
  }

  if (!localized_lateral_profile_.active) {
    // 処理ブロック: 新規ラッチを作る。
    // 設計意図:
    // 開始時の自車dと対象sを基準にし、以後の候補生成で同じ回避区間を使う。
    if (pass_type != CandidateType::PASS_LEFT &&
        pass_type != CandidateType::PASS_RIGHT) {
      return;
    }
    const std::string previous_target_id = blocked.maneuver_target_previous_id;
    localized_lateral_profile_.active = true;
    localized_lateral_profile_.pass_type = pass_type;
    localized_lateral_profile_.target_id = target.id;
    localized_lateral_profile_.created_time_sec = now_sec;
    localized_lateral_profile_.anchor_s_m = ego.frenet.s;
    localized_lateral_profile_.ego_unwrapped_s_m = ego.frenet.s;
    localized_lateral_profile_.last_ego_wrapped_s_m = ego.frenet.s;
    localized_lateral_profile_.last_ego_stamp_sec = ego.stamp_sec;
    localized_lateral_profile_.start_d_m = ego.frenet.d;
    localized_lateral_profile_.target_d_m =
        targetOffsetForPass(pass_type, ego.frenet.d, target.frenet.d);
    if (blocked.gentle_curve_safe_pass_constraint_active &&
        config_.gentle_curve_safe_pass_enabled &&
        config_.gentle_curve_safe_pass_max_lateral_displacement_m > 0.0) {
      const double max_displacement =
          config_.gentle_curve_safe_pass_max_lateral_displacement_m;
      const auto bounds =
          frame_.corridorBounds(ego.frenet.s, config_.d_min_m, config_.d_max_m);
      const double lower_d = bounds.d_min + config_.min_wall_margin_m;
      const double upper_d = bounds.d_max - config_.min_wall_margin_m;
      localized_lateral_profile_.target_d_m = std::clamp(
          localized_lateral_profile_.target_d_m,
          ego.frenet.d - max_displacement, ego.frenet.d + max_displacement);
      localized_lateral_profile_.target_d_m =
          std::clamp(localized_lateral_profile_.target_d_m, lower_d, upper_d);
    }
    const double target_s_m =
        localized_lateral_profile_.anchor_s_m +
        frame_.deltaS(localized_lateral_profile_.anchor_s_m, target.frenet.s);
    const double target_gap_m =
        target_s_m - localized_lateral_profile_.anchor_s_m;
    const bool early_low_speed_profile =
        blocked.early_low_speed_pass_target_active &&
        blocked.early_low_speed_pass_target_id == target.id;
    const double longitudinal_clearance_m =
        std::max(0.0, config_.safety_ellipse_a_m) *
        std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h));
    const double configured_start_before_m =
        early_low_speed_profile
            ? target_gap_m
            : std::max(0.0, config_.localized_avoidance_start_before_target_m);
    const double configured_full_before_m =
        early_low_speed_profile
            ? std::max(
                  std::max(
                      0.0,
                      config_.localized_avoidance_full_offset_before_target_m),
                  longitudinal_clearance_m)
            : std::max(0.0,
                       config_.localized_avoidance_full_offset_before_target_m);
    const double requested_transition_m =
        std::abs(configured_start_before_m - configured_full_before_m);
    double tracking_speed_cap_mps = std::max(0.0, std::max(ego.v, target.v));
    if (blocked.pass_lateral_first_speed_gate_active &&
        std::isfinite(blocked.pass_lateral_first_speed_cap_mps) &&
        blocked.pass_lateral_first_speed_cap_mps > 0.0) {
      tracking_speed_cap_mps = std::min(
          tracking_speed_cap_mps, blocked.pass_lateral_first_speed_cap_mps);
    }
    double required_transition_m =
        CandidateBuilder(frame_, config_)
            .minimumTrackableLateralShiftDistance(
                ego, localized_lateral_profile_.target_d_m,
                tracking_speed_cap_mps, blocked,
                std::max(1.0, requested_transition_m));
    if (early_low_speed_profile &&
        std::isfinite(blocked.early_low_speed_pass_required_transition_m)) {
      required_transition_m =
          std::max(required_transition_m,
                   blocked.early_low_speed_pass_required_transition_m);
    }
    // profile markerへ十分条件を重ねず、途中横分離を含む全候補点の楕円判定は
    // SafetyEvaluatorへ一本化する。ここはactive controllerが追従できる
    // transition距離を確保する役割だけを持つ。
    constexpr double required_full_before_m = 0.0;
    const double maximum_full_before_m = target_gap_m - required_transition_m;
    localized_lateral_profile_.full_offset_before_target_m =
        std::isfinite(required_transition_m) &&
                maximum_full_before_m >= required_full_before_m
            ? std::max(required_full_before_m,
                       std::min(std::max(configured_full_before_m,
                                         required_full_before_m),
                                maximum_full_before_m))
            : required_full_before_m;
    localized_lateral_profile_.avoid_start_before_target_m =
        std::isfinite(required_transition_m)
            ? std::max(localized_lateral_profile_.full_offset_before_target_m,
                       std::min(target_gap_m,
                                std::max(configured_start_before_m,
                                         localized_lateral_profile_
                                                 .full_offset_before_target_m +
                                             required_transition_m)))
            : std::numeric_limits<double>::infinity();
    localized_lateral_profile_.chain_tail_id = target.id;
    localized_lateral_profile_.chain_tail_s_m = target_s_m;
    localized_lateral_profile_.chain_tail_speed_mps = target.v;
    localized_lateral_profile_.chain_target_count = 1;
    localized_lateral_profile_.chain_waypoints = {LocalizedLateralWaypoint{
        target.id, target_s_m, localized_lateral_profile_.target_d_m,
        target_s_m, target.frenet.s, target.stamp_sec}};
    setLocalizedProfileMarkers(target_s_m);
    blocked.maneuver_target_previous_id = previous_target_id;
    blocked.maneuver_target_new_id = target.id;
    const bool start_grid_blocked_front_handoff =
        blocked.start_grid_target_superseded_by_blocked_front &&
        previous_target_id == blocked.start_grid_superseded_target_id &&
        target.id == blocked.start_grid_replacement_target_id;
    blocked.maneuver_target_change_reason =
        start_grid_blocked_front_handoff
            ? "unapproved_start_grid_target_superseded_by_blocked_front"
        : previous_target_id.empty() ? "pass_attempt_started"
                                     : "previous_target_passed";
  }

  // target ID・PASS side・認可済みtarget dはtransaction完了まで固定する一方、
  // 移動中の相手に対して縦markerまで初回位置へ固定すると、実相手より後方へ
  // PASS profileが取り残される。freshな同一ID観測だけを使い、s waypointを
  // 前進方向へ単調更新する。後退ノイズやout-of-order観測でmarkerは戻さない。
  const double alpha =
      std::clamp(config_.maneuver_latch_target_update_alpha, 0.0, 1.0);
  const auto advanceLongitudinalMarker = [alpha](double current_s_m,
                                                 double measured_s_m) {
    if (alpha <= 0.0 || !std::isfinite(current_s_m) ||
        !std::isfinite(measured_s_m)) {
      return current_s_m;
    }
    const double filtered_s_m =
        current_s_m * (1.0 - alpha) + measured_s_m * alpha;
    return std::max(current_s_m, filtered_s_m);
  };
  if (std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) &&
      std::isfinite(localized_lateral_profile_.last_ego_wrapped_s_m) &&
      std::isfinite(ego.frenet.s)) {
    // anchorからの差分は半周を越えると逆向きへ折り返す。制御周期間の差分なら
    // 半周を越えないため、周回を跨いでも連続したPASS距離契約を維持できる。
    localized_lateral_profile_.ego_unwrapped_s_m +=
        signedDeltaS(localized_lateral_profile_.last_ego_wrapped_s_m,
                     ego.frenet.s, frame_.length());
    localized_lateral_profile_.last_ego_wrapped_s_m = ego.frenet.s;
  }
  if (alpha > 0.0 && target.id == localized_lateral_profile_.target_id) {
    double updated_target_s_m = localized_lateral_profile_.target_s_m;
    for (auto &waypoint : localized_lateral_profile_.chain_waypoints) {
      const auto observed = std::find_if(
          opponents.begin(), opponents.end(),
          [this, now_sec, &waypoint](const OpponentState &opponent) {
            return opponent.id == waypoint.target_id && opponent.valid &&
                   inputTimestampFresh(
                       now_sec, opponent.stamp_sec,
                       config_.opponent_stale_time_sec,
                       config_.input_future_stamp_tolerance_sec);
          });
      if (observed == opponents.end()) {
        continue;
      }
      if (!std::isfinite(waypoint.observed_unwrapped_s_m) ||
          !std::isfinite(waypoint.last_observed_wrapped_s_m)) {
        // 旧fixture/旧profileとの互換。現在markerを連続座標の初期値にし、
        // 以後は相手自身の周期間進捗だけを積算する。
        waypoint.observed_unwrapped_s_m = waypoint.target_s_m;
        waypoint.last_observed_wrapped_s_m = observed->frenet.s;
      } else {
        waypoint.observed_unwrapped_s_m +=
            signedDeltaS(waypoint.last_observed_wrapped_s_m, observed->frenet.s,
                         frame_.length());
        waypoint.last_observed_wrapped_s_m = observed->frenet.s;
      }
      waypoint.target_s_m = advanceLongitudinalMarker(
          waypoint.target_s_m, waypoint.observed_unwrapped_s_m);
      if (waypoint.target_id == localized_lateral_profile_.target_id) {
        // scalar
        // markerと先頭waypointは同じ対象を表すため、相手自身の周回積算値を
        // そのまま採用する。egoから半周以上離れてもmarkerを凍結しない。
        updated_target_s_m = waypoint.target_s_m;
      }
      if (waypoint.target_id == localized_lateral_profile_.chain_tail_id) {
        localized_lateral_profile_.chain_tail_s_m = waypoint.target_s_m;
        localized_lateral_profile_.chain_tail_speed_mps = observed->v;
      }
    }
    const bool handed_off_inside_staged_chain =
        !localized_lateral_profile_.chain_waypoints.empty() &&
        localized_lateral_profile_.target_id !=
            localized_lateral_profile_.chain_waypoints.front().target_id;
    if (handed_off_inside_staged_chain) {
      // 初回avoid/full-offset markerはchain全体の開始形状である。d3/d4への
      // sequential handoffでこれを現在位置の前へ動かすと、既に達成した横移動を
      // start_dからやり直すため、waypoint列だけを前進更新する。
      localized_lateral_profile_.target_s_m = updated_target_s_m;
    } else {
      setLocalizedProfileMarkers(updated_target_s_m);
    }
    if (localized_lateral_profile_.chain_tail_id == target.id) {
      localized_lateral_profile_.chain_tail_s_m = updated_target_s_m;
      localized_lateral_profile_.chain_tail_speed_mps = target.v;
    }
  }

  // Gate 2認可だけを「横へコミット済み」とは扱わない。20260719 dev3では、
  // PASS_LEFTの目標d=約3.72 mに対して実車dが2.59->1.50 mと反対側へ戻り、
  // freshなd3が20 m以上先へ離れた後も旧transactionが近いd2の再評価を
  // 永久に塞いだ。対象/sideを通常は通過完了まで保持しつつ、この例外は
  // (1) freshな同一ID、(2)前方gap、(3)開いている相対速度、(4)選択側への
  // 実横進捗が閾値以下、を連続周期で確認した時だけ成立させる。
  const double pass_lateral_delta_m = localized_lateral_profile_.target_d_m -
                                      localized_lateral_profile_.start_d_m;
  const double pass_direction = pass_lateral_delta_m > 0.0 ? 1.0 : -1.0;
  const double pass_lateral_progress_m =
      pass_direction * (ego.frenet.d - localized_lateral_profile_.start_d_m);
  const double target_relative_s_m =
      localized_lateral_profile_.target_s_m -
      localized_lateral_profile_.ego_unwrapped_s_m;
  const double target_relative_speed_mps = ego.v - target.v;
  blocked.maneuver_pass_lateral_progress_m = pass_lateral_progress_m;
  const double unstarted_release_interaction_gap_m = std::max(
      config_.unstarted_pass_target_release_min_gap_m,
      std::max(0.0, config_.localized_avoidance_start_before_target_m) +
          std::max(0.0, config_.safety_ellipse_a_m) *
              std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h)));
  const bool unstarted_target_pulling_away =
      config_.unstarted_pass_target_release_enabled &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      std::isfinite(pass_lateral_delta_m) &&
      std::abs(pass_lateral_delta_m) >
          std::max(
              0.20,
              config_.unstarted_pass_target_release_max_lateral_progress_m) &&
      std::isfinite(pass_lateral_progress_m) &&
      pass_lateral_progress_m <=
          config_.unstarted_pass_target_release_max_lateral_progress_m &&
      std::isfinite(target_relative_s_m) &&
      target_relative_s_m >= unstarted_release_interaction_gap_m &&
      std::isfinite(target_relative_speed_mps) &&
      target_relative_speed_mps <=
          -config_.unstarted_pass_target_release_min_opening_speed_mps;
  if (unstarted_target_pulling_away) {
    localized_lateral_profile_.unstarted_target_pulling_away_cycles = std::min(
        config_.unstarted_pass_target_release_required_cycles,
        localized_lateral_profile_.unstarted_target_pulling_away_cycles + 1);
  } else {
    localized_lateral_profile_.unstarted_target_pulling_away_cycles = 0;
  }
  blocked.maneuver_unstarted_target_pulling_away_cycles =
      localized_lateral_profile_.unstarted_target_pulling_away_cycles;
  if (localized_lateral_profile_.unstarted_target_pulling_away_cycles >=
      config_.unstarted_pass_target_release_required_cycles) {
    const std::string released_target_id = localized_lateral_profile_.target_id;
    const int release_cycles =
        localized_lateral_profile_.unstarted_target_pulling_away_cycles;
    unstarted_pass_released_target_id_ = released_target_id;
    clearLocalizedLateralProfile();
    // 同周期にprofileなしのPASSを再認可しない。現在のBlockedInfoから作る
    // FOLLOW/RECOVERYだけをSafetyEvaluatorへ通し、次周期に近い対象を新しい
    // transactionとして独立評価する。中心復帰や速度制限の安全gateは維持する。
    blocked.can_pass_left = false;
    blocked.can_pass_right = false;
    blocked.pass_decision_frozen = true;
    blocked.pass_decision_freeze_reason =
        "unstarted_pass_target_pulled_away_handoff";
    blocked.pass_gap_reason = "unstarted_pass_target_pulled_away_handoff";
    blocked.maneuver_target_latched = false;
    blocked.maneuver_target_id.clear();
    blocked.maneuver_target_index = -1;
    blocked.maneuver_target_observed = false;
    blocked.maneuver_target_fresh = false;
    blocked.maneuver_target_previous_id = released_target_id;
    blocked.maneuver_target_new_id.clear();
    blocked.maneuver_target_change_reason =
        "unstarted_pass_target_pulled_away_handoff";
    blocked.maneuver_unstarted_target_released = true;
    blocked.maneuver_unstarted_target_pulling_away_cycles = release_cycles;
    blocked.maneuver_target_relative_s_m = target_relative_s_m;
    blocked.maneuver_target_relative_d_m = target.frenet.d - ego.frenet.d;
    blocked.maneuver_target_relative_speed_mps = target_relative_speed_mps;
    return;
  }

  extendLocalizedProfileForSlowObstacleChain(now_sec, ego, target, opponents);

  // target markerの再設定で既存chainの保持区間を縮めない。chain拡張条件が
  // 一時的に不成立でも、保持済みtailを抜く前に中心へmergeさせない。
  if (std::isfinite(localized_lateral_profile_.chain_tail_s_m)) {
    const double hold_after_m =
        std::max(0.0, config_.localized_avoidance_hold_after_target_m);
    const double merge_distance_m =
        std::max(1.0, config_.localized_avoidance_merge_distance_m);
    localized_lateral_profile_.full_offset_end_s_m =
        std::max(localized_lateral_profile_.full_offset_end_s_m,
                 localized_lateral_profile_.chain_tail_s_m + hold_after_m);
    localized_lateral_profile_.merge_end_s_m =
        localized_lateral_profile_.full_offset_end_s_m + merge_distance_m;
  }

  blocked.maneuver_target_latched = true;
  blocked.maneuver_target_id = localized_lateral_profile_.target_id;
  blocked.maneuver_target_index = target_index;
  blocked.maneuver_target_observed = true;
  blocked.maneuver_target_fresh = true;
  blocked.maneuver_target_age_sec = now_sec - target.stamp_sec;
  // 完了判定にも同じ連続座標を使う。wrapped観測値を直接引くと周回境界で
  // 前方8 mの対象を大きな負値（通過済み）と誤認し、認可済みPASSを途中終了する。
  blocked.maneuver_target_relative_s_m =
      localized_lateral_profile_.target_s_m -
      localized_lateral_profile_.ego_unwrapped_s_m;
  blocked.maneuver_target_relative_d_m = target.frenet.d - ego.frenet.d;
  blocked.maneuver_target_relative_speed_mps = ego.v - target.v;
  blocked.maneuver_chain_tail_id = localized_lateral_profile_.chain_tail_id;
  blocked.maneuver_chain_target_count =
      localized_lateral_profile_.chain_target_count;
  const auto chain_tail_it = std::find_if(
      opponents.begin(), opponents.end(), [this, now_sec](const auto &opp) {
        return opp.id == localized_lateral_profile_.chain_tail_id &&
               opp.valid &&
               inputTimestampFresh(now_sec, opp.stamp_sec,
                                   config_.opponent_stale_time_sec,
                                   config_.input_future_stamp_tolerance_sec);
      });
  blocked.maneuver_chain_tail_observed = chain_tail_it != opponents.end();
  if (chain_tail_it != opponents.end()) {
    blocked.maneuver_chain_tail_index =
        static_cast<int>(std::distance(opponents.begin(), chain_tail_it));
  }
  const double ego_unwrapped_s_m = localized_lateral_profile_.ego_unwrapped_s_m;
  // 周期間の符号付き進捗を積算するため、停止中の微小後退を1周進捗と
  // 誤認せず、かつ長時間のPASSが半周を越えてもanchor側へ折り返さない。
  if (std::isfinite(localized_lateral_profile_.chain_tail_s_m) &&
      std::isfinite(ego_unwrapped_s_m)) {
    blocked.maneuver_chain_tail_relative_s_m =
        localized_lateral_profile_.chain_tail_s_m - ego_unwrapped_s_m;
  }
  if (chain_tail_it != opponents.end() && std::isfinite(chain_tail_it->v)) {
    blocked.maneuver_chain_tail_relative_speed_mps = ego.v - chain_tail_it->v;
  }
  // transaction完了は列末尾ではなく、固定したtarget IDそのものを抜いた事実で
  // 判定する。後続車のために横profileを先へ延長しても、d2を抜いた後にd3へ
  // targetを引き継げる。sideは同じstaged profile上で保つため中心へmergeしない。
  blocked.maneuver_target_pass_geometric_complete =
      blocked.maneuver_target_observed &&
      blocked.maneuver_target_relative_s_m <=
          -std::max(0.0, config_.merge_front_gap_m) &&
      blocked.maneuver_target_relative_speed_mps >=
          -std::max(0.0, config_.dv_block_threshold_mps);
  if (blocked.maneuver_target_change_reason.empty()) {
    blocked.maneuver_target_previous_id = localized_lateral_profile_.target_id;
    blocked.maneuver_target_new_id = localized_lateral_profile_.target_id;
    blocked.maneuver_target_change_reason =
        classified_target_index >= 0 && classified_target_index != target_index
            ? "held_across_classifier_change"
            : "held_latched_target";
  }
}

// 入力: snapshot時のego位置を0 mとした、publish済みPASS wire上の前方距離。
// 出力: 同じ空間位置の横位置d。不正形状はnullopt。
// 処理概要:
// 最初のpublish済み有限horizonは線形補間し、その先を同じtarget/side/chain
// identityの局所profileで補完する。PASS継続候補と速度gateが同じ固定wireを
// 参照し、rolling current-d profileとの判定ずれを作らない。
std::optional<double>
OvertakePlannerCore::sampleCommittedPassSpatialProfileD(double offset_m) const {
  const auto &offsets_m = committed_pass_snapshot_.longitudinal_offsets_m;
  const auto &lateral_d_m = committed_pass_snapshot_.d;
  if (!std::isfinite(offset_m) || offsets_m.empty() ||
      offsets_m.size() != lateral_d_m.size() ||
      !std::isfinite(offsets_m.front()) || !std::isfinite(offsets_m.back()) ||
      !std::isfinite(lateral_d_m.front()) ||
      !std::isfinite(lateral_d_m.back())) {
    return std::nullopt;
  }
  if (offset_m <= offsets_m.front()) {
    return lateral_d_m.front();
  }
  if (offset_m > offsets_m.back()) {
    if (!localized_lateral_profile_.active ||
        !std::isfinite(committed_pass_snapshot_ego_unwrapped_s_m_)) {
      return std::nullopt;
    }
    const double extended_d_m =
        CandidateBuilder(frame_, config_)
            .nominalLocalizedProfileDAtUnwrappedS(
                localized_lateral_profile_,
                committed_pass_snapshot_ego_unwrapped_s_m_ + offset_m);
    if (!std::isfinite(extended_d_m)) {
      return std::nullopt;
    }
    return extended_d_m;
  }

  const auto upper =
      std::lower_bound(offsets_m.begin(), offsets_m.end(), offset_m);
  if (upper == offsets_m.end()) {
    return lateral_d_m.back();
  }
  const std::size_t upper_index =
      static_cast<std::size_t>(std::distance(offsets_m.begin(), upper));
  if (upper_index == 0U) {
    return lateral_d_m.front();
  }
  const std::size_t lower_index = upper_index - 1U;
  const double lower_offset_m = offsets_m[lower_index];
  const double upper_offset_m = offsets_m[upper_index];
  const double lower_d_m = lateral_d_m[lower_index];
  const double upper_d_m = lateral_d_m[upper_index];
  const double span_m = upper_offset_m - lower_offset_m;
  if (!std::isfinite(lower_offset_m) || !std::isfinite(upper_offset_m) ||
      !std::isfinite(lower_d_m) || !std::isfinite(upper_d_m) ||
      span_m <= 1.0e-9) {
    return std::nullopt;
  }
  const double ratio =
      std::clamp((offset_m - lower_offset_m) / span_m, 0.0, 1.0);
  return lower_d_m + ratio * (upper_d_m - lower_d_m);
}

// 入力: 現時刻、自車、ラッチ済みPASS情報、fresh相手一覧。
// 出力: BlockedInfoのlateral-first速度gate診断とPASS速度上限。
// 処理概要:
// 前方の固定対象（通過後はchain tail、制動対象がさらに近ければその車）に対し、
// 実横分離が安全楕円+PASS余裕へ届くまでは相手速度を超えない。停止/極低速車だけは
// 横profileを進めるための1 m/s以下creepを許すが、候補のs(t)とSafetyEvaluatorは
// 同じ上限を使う。対象より前へ出た後は縦加速が分離を増やすためgateを解除する。
void OvertakePlannerCore::updatePassLateralFirstSpeedGate(
    double now_sec, const EgoState &ego, BlockedInfo &blocked,
    const std::vector<OpponentState> &opponents) const {
  blocked.pass_lateral_first_speed_gate_active = false;
  blocked.pass_lateral_clearance_ready = false;
  blocked.pass_lateral_first_target_id.clear();
  blocked.pass_lateral_first_target_relative_s_m =
      std::numeric_limits<double>::infinity();
  blocked.pass_lateral_separation_actual_m =
      std::numeric_limits<double>::quiet_NaN();
  blocked.pass_lateral_separation_required_m =
      std::numeric_limits<double>::quiet_NaN();
  blocked.pass_lateral_first_target_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  blocked.pass_lateral_first_speed_cap_mps =
      std::numeric_limits<double>::quiet_NaN();

  if (!localized_lateral_profile_.active ||
      localized_lateral_profile_.pass_complete_confirmed || !ego.valid ||
      !std::isfinite(ego.frenet.d)) {
    return;
  }

  const OpponentState *speed_gate_target = nullptr;
  double speed_gate_relative_s_m = std::numeric_limits<double>::infinity();
  const auto consider_target = [&](int index, double relative_s_m) {
    if (index < 0 || static_cast<std::size_t>(index) >= opponents.size() ||
        !std::isfinite(relative_s_m) || relative_s_m <= 0.0 ||
        relative_s_m >= speed_gate_relative_s_m) {
      return;
    }
    const auto &target = opponents[static_cast<std::size_t>(index)];
    if (!target.valid || target.id.empty() || !std::isfinite(target.frenet.d) ||
        !inputTimestampFresh(now_sec, target.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec)) {
      return;
    }
    speed_gate_target = &target;
    speed_gate_relative_s_m = relative_s_m;
  };

  if (blocked.maneuver_target_observed && blocked.maneuver_target_fresh) {
    consider_target(blocked.maneuver_target_index,
                    blocked.maneuver_target_relative_s_m);
  }
  if (blocked.maneuver_chain_tail_observed) {
    consider_target(blocked.maneuver_chain_tail_index,
                    blocked.maneuver_chain_tail_relative_s_m);
  }
  if (blocked.braking_follow_active) {
    consider_target(blocked.braking_follow_index,
                    blocked.braking_follow_delta_s);
  }
  // staged chainの末尾だけを見ると、d2通過後もd4を対象にして中間d3の実横分離を
  // 見落とす。authoritative maneuver targetは固定したまま、速度gateだけは
  // chainに登録済みのfreshな各IDから最も近い前方車へ結び直す。
  for (const auto &waypoint : localized_lateral_profile_.chain_waypoints) {
    if (waypoint.target_id.empty()) {
      continue;
    }
    const auto waypoint_target =
        std::find_if(opponents.begin(), opponents.end(),
                     [&waypoint](const OpponentState &opponent) {
                       return opponent.id == waypoint.target_id;
                     });
    if (waypoint_target == opponents.end()) {
      continue;
    }
    const int waypoint_index =
        static_cast<int>(std::distance(opponents.begin(), waypoint_target));
    const double waypoint_relative_s_m =
        signedDeltaS(ego.frenet.s, waypoint_target->frenet.s, frame_.length());
    consider_target(waypoint_index, waypoint_relative_s_m);
  }
  if (speed_gate_target == nullptr) {
    return;
  }

  const double required_separation_m =
      std::max(0.0, config_.safety_ellipse_b_m) *
          std::sqrt(1.0 + std::max(0.0, config_.min_ellipse_h)) +
      std::max(0.0, config_.pass_target_lateral_margin_m);
  const double actual_separation_m =
      std::abs(speed_gate_target->frenet.d - ego.frenet.d);
  const double target_s_dot_mps =
      blocked_risk_.opponentSDot(*speed_gate_target);
  // opposite/不明方向を正速度として使わない。候補自体は既存の方向filterと
  // SafetyEvaluatorが拒否し、ここでは停止対象相当のbounded capへ閉じる。
  const double target_speed_mps =
      std::isfinite(target_s_dot_mps) && target_s_dot_mps > 0.0
          ? target_s_dot_mps
          : 0.0;
  const double stationary_creep_mps = std::clamp(
      config_.pass_lateral_first_stationary_creep_v_max_mps, 1.0e-3, 1.0);
  const double low_speed_threshold_mps =
      std::max(0.0, config_.stationary_obstacle_speed_threshold_mps);
  double lateral_first_speed_cap_mps =
      std::min(std::max(1.0e-3, config_.pass_speed_cap_mps),
               target_speed_mps <= low_speed_threshold_mps
                   ? std::max(target_speed_mps, stationary_creep_mps)
                   : target_speed_mps);
  const bool stationary_curve_current_speed_candidate =
      config_.stationary_no_pass_safe_pass_enabled &&
      blocked.braking_follow_active &&
      blocked.braking_follow_id == speed_gate_target->id &&
      target_speed_mps <= low_speed_threshold_mps &&
      blocked.overtake_start_gate_reason == "curve" &&
      std::isfinite(blocked.overtake_start_abs_curvature) &&
      blocked.overtake_start_abs_curvature > 0.0 &&
      blocked.overtake_start_abs_curvature <=
          config_.stationary_no_pass_safe_pass_max_curvature_m_inv &&
      std::isfinite(ego.v) && ego.v >= 0.0 &&
      ego.v <=
          std::min(
              config_.stationary_no_pass_safe_pass_v_max_mps,
              std::sqrt(
                  config_.stationary_no_pass_safe_pass_max_lateral_accel_mps2 /
                  blocked.overtake_start_abs_curvature)) +
              1.0e-6;
  if (stationary_curve_current_speed_candidate) {
    // 十分手前でfreshな停止targetを束縛でき、現在速度が設定済みの動力学上限内
    // なら、candidate全時刻列を現在速度で評価する。ここではauthorityを開かず、
    // 後段のwall/CBF/PP/deadlineが1つでも落ちればFOLLOW/STOPへ戻す。
    lateral_first_speed_cap_mps =
        std::min(std::max(1.0e-3, config_.pass_speed_cap_mps), ego.v);
  }

  // commit済み空間profileより実車がPASS方向へ遅れている時に速度を維持すると、
  // connectorが次周期ほど短くなり、最終的にtrackabilityまたは相手楕円を失う。
  // 停止・低速対象かつ実横分離が未完了の間だけ、固定profileの現在dと実測dを
  // 比較してbounded creepへ落とす。target/side/profileや安全閾値は変えず、
  // moving targetと通常走行には適用しない。
  if (localized_lateral_profile_.active &&
      localized_lateral_profile_.pass_execution_committed &&
      !localized_lateral_profile_.pass_complete_confirmed &&
      committed_pass_snapshot_valid_ &&
      committed_pass_spatial_profile_frozen_ &&
      committed_pass_snapshot_target_id_ ==
          localized_lateral_profile_.target_id &&
      committed_pass_snapshot_pass_type_ ==
          localized_lateral_profile_.pass_type &&
      target_speed_mps <= low_speed_threshold_mps &&
      actual_separation_m + 1.0e-6 < required_separation_m &&
      std::isfinite(committed_pass_snapshot_ego_unwrapped_s_m_) &&
      std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) &&
      std::isfinite(ego.frenet.d)) {
    const double signed_progress_m =
        localized_lateral_profile_.ego_unwrapped_s_m -
        committed_pass_snapshot_ego_unwrapped_s_m_;
    const auto committed_d_m = signed_progress_m >= -0.05
                                   ? sampleCommittedPassSpatialProfileD(
                                         std::max(0.0, signed_progress_m))
                                   : std::nullopt;
    const double pass_direction =
        localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT ? 1.0
        : localized_lateral_profile_.pass_type == CandidateType::PASS_RIGHT
            ? -1.0
            : 0.0;
    const double directional_tracking_lag_m =
        committed_d_m.has_value()
            ? pass_direction * (*committed_d_m - ego.frenet.d)
            : std::numeric_limits<double>::quiet_NaN();
    if (pass_direction != 0.0 && committed_d_m.has_value() &&
        std::isfinite(*committed_d_m) &&
        std::isfinite(directional_tracking_lag_m)) {
      blocked.committed_pass_spatial_profile_tracking_error_m =
          std::abs(*committed_d_m - ego.frenet.d);
      if (directional_tracking_lag_m >
          config_.pass_lateral_tracking_lag_threshold_m) {
        lateral_first_speed_cap_mps = std::min(
            lateral_first_speed_cap_mps,
            std::max(1.0e-3, config_.pass_lateral_tracking_lag_speed_cap_mps));
      }
    }
  }

  blocked.pass_lateral_first_target_id = speed_gate_target->id;
  blocked.pass_lateral_first_target_relative_s_m = speed_gate_relative_s_m;
  blocked.pass_lateral_separation_actual_m = actual_separation_m;
  blocked.pass_lateral_separation_required_m = required_separation_m;
  blocked.pass_lateral_first_target_speed_mps = target_speed_mps;
  blocked.pass_lateral_first_speed_cap_mps = lateral_first_speed_cap_mps;
  blocked.pass_lateral_clearance_ready =
      std::isfinite(required_separation_m) && required_separation_m > 0.0 &&
      actual_separation_m + 1.0e-6 >= required_separation_m;
  blocked.pass_lateral_first_speed_gate_active =
      !blocked.pass_lateral_clearance_ready &&
      std::isfinite(lateral_first_speed_cap_mps) &&
      lateral_first_speed_cap_mps > 0.0;
}

// 入力: なし。
// 出力: なし。局所横プロファイルを無効状態へ戻す。
// 処理概要: PASS文脈が切れた時に、古い対象車両の回避区間を次回へ持ち越さない。
void OvertakePlannerCore::clearLocalizedLateralProfile() {
  localized_lateral_profile_ = LocalizedLateralProfile{};
  attack_follow_inner_band_diagnostic_ = AttackFollowInnerBandDiagnostic{};
  last_attack_follow_inner_band_probe_sec_ =
      std::numeric_limits<double>::quiet_NaN();
  maneuver_execution_hold_active_ = false;
  maneuver_execution_hold_target_id_.clear();
  maneuver_execution_hold_pass_type_ = CandidateType::FASTEST;
  committed_pass_snapshot_valid_ = false;
  committed_pass_spatial_profile_frozen_ = false;
  committed_pass_snapshot_target_id_.clear();
  committed_pass_snapshot_pass_type_ = CandidateType::FASTEST;
  committed_pass_snapshot_sec_ = std::numeric_limits<double>::quiet_NaN();
  committed_pass_snapshot_ego_unwrapped_s_m_ =
      std::numeric_limits<double>::quiet_NaN();
  committed_pass_snapshot_ = CandidateTrajectory{};
  committed_pass_profile_snapshot_ = LocalizedLateralProfile{};
}

// 入力: ラッチ対象のunwrapped target_s。
// 出力: なし。局所プロファイルの開始/保持/マージmarkerを更新する。
// 処理概要:
// 対象車両の前から避け始め、通過後に一定距離保持してから中心へ戻す区間を作る。
void OvertakePlannerCore::setLocalizedProfileMarkers(double target_s_m) {
  localized_lateral_profile_.target_s_m = target_s_m;
  const double configured_start_before =
      std::max(0.0, config_.localized_avoidance_start_before_target_m);
  const double configured_full_before =
      std::max(0.0, config_.localized_avoidance_full_offset_before_target_m);
  const double nominal_avoid_before =
      std::max(configured_start_before, configured_full_before);
  const double nominal_full_before =
      std::min(configured_start_before, configured_full_before);
  const double configured_transition_distance =
      nominal_avoid_before - nominal_full_before;
  // markerは横形状だけを定義し、途中横分離を含む衝突可否は生成後の
  // SafetyEvaluatorへ一本化する。初回にcontroller契約から固定した値がある
  // 場合は、target更新後もその形を維持する。
  constexpr double required_full_before = 0.0;
  const double full_before =
      std::isfinite(localized_lateral_profile_.full_offset_before_target_m)
          ? std::max(required_full_before,
                     localized_lateral_profile_.full_offset_before_target_m)
          : std::max(nominal_full_before, required_full_before);
  const double start_before =
      std::isfinite(localized_lateral_profile_.avoid_start_before_target_m)
          ? std::max(full_before,
                     localized_lateral_profile_.avoid_start_before_target_m)
          : std::max(nominal_avoid_before,
                     full_before + configured_transition_distance);
  const double hold_after =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  localized_lateral_profile_.avoid_start_s_m = target_s_m - start_before;
  localized_lateral_profile_.full_offset_start_s_m = target_s_m - full_before;
  localized_lateral_profile_.full_offset_end_s_m = target_s_m + hold_after;
  localized_lateral_profile_.merge_end_s_m =
      localized_lateral_profile_.full_offset_end_s_m + merge_distance;
}

// 入力: 現在時刻、ラッチ済み先頭対象、全相手車両。
// 出力: なし。localized profileの横保持末尾と完了対象だけを必要時に延長する。
// 処理概要: fresh・同方向・低速・同一コリドーで、設定距離以内に連続する車両を
// s順に辿る。target IDとPASS方向は変えず、列末尾を抜く前のmergeだけを防ぐ。
void OvertakePlannerCore::extendLocalizedProfileForSlowObstacleChain(
    double now_sec, const EgoState &ego, const OpponentState &target,
    const std::vector<OpponentState> &opponents) {
  if (!localized_lateral_profile_.active ||
      !config_.slow_obstacle_chain_enabled ||
      !config_.slow_front_exception_enabled || !std::isfinite(target.v) ||
      target.v > std::max(0.0, config_.slow_front_exception_speed_mps) ||
      !std::isfinite(localized_lateral_profile_.anchor_s_m) ||
      !std::isfinite(localized_lateral_profile_.ego_unwrapped_s_m) ||
      !std::isfinite(ego.frenet.s) ||
      !std::isfinite(localized_lateral_profile_.target_s_m)) {
    return;
  }

  struct ChainCandidate {
    std::string id;
    double unwrapped_s_m{0.0};
    double wrapped_s_m{0.0};
    double lateral_d_m{0.0};
    double speed_mps{0.0};
  };
  std::vector<ChainCandidate> candidates;
  candidates.reserve(opponents.size());
  const double lateral_limit_m = std::max(0.0, config_.same_corridor_width_m);
  const double cumulative_lateral_limit_m =
      std::max(lateral_limit_m, std::max(0.0, config_.parallel_side_margin_m));
  for (const auto &opponent : opponents) {
    if (opponent.id == target.id || !opponent.valid ||
        !inputTimestampFresh(now_sec, opponent.stamp_sec,
                             config_.opponent_stale_time_sec,
                             config_.input_future_stamp_tolerance_sec) ||
        !std::isfinite(opponent.v) || opponent.v < 0.0 ||
        opponent.v > std::max(0.0, config_.slow_front_exception_speed_mps) ||
        !std::isfinite(opponent.frenet.d) ||
        std::abs(opponent.frenet.d - target.frenet.d) >
            cumulative_lateral_limit_m) {
      continue;
    }
    const double s_dot_mps = blocked_risk_.opponentSDot(opponent);
    const bool direction_known =
        opponent.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known || (std::isfinite(s_dot_mps) &&
                             s_dot_mps >= config_.same_direction_min_s_dot_mps);
    if (config_.same_direction_filter_enabled && !same_direction) {
      continue;
    }
    const double unwrapped_s_m =
        localized_lateral_profile_.ego_unwrapped_s_m +
        signedDeltaS(ego.frenet.s, opponent.frenet.s, frame_.length());
    if (!std::isfinite(unwrapped_s_m) ||
        unwrapped_s_m <= localized_lateral_profile_.target_s_m + 1.0e-6) {
      continue;
    }
    candidates.push_back(ChainCandidate{opponent.id, unwrapped_s_m,
                                        opponent.frenet.s, opponent.frenet.d,
                                        opponent.v});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const ChainCandidate &lhs, const ChainCandidate &rhs) {
              return lhs.unwrapped_s_m < rhs.unwrapped_s_m;
            });

  const double max_link_distance_m =
      std::max(0.0, config_.slow_obstacle_chain_distance_m);
  double reachable_tail_s_m = localized_lateral_profile_.target_s_m;
  std::string reachable_tail_id = target.id;
  double reachable_tail_speed_mps = target.v;
  double reachable_tail_d_m = target.frenet.d;
  double staged_target_d_m = localized_lateral_profile_.target_d_m;
  int reachable_count = 1;
  for (const auto &candidate : candidates) {
    if (candidate.unwrapped_s_m - reachable_tail_s_m >
        max_link_distance_m + 1.0e-6) {
      break;
    }
    // 列全体を先頭targetのdだけで判定すると、実ゲート2のように各車が
    // 0.9 m以内で少しずつ横へずれた列の3台目を除外し、直前でmergeしてしまう。
    // s順に到達済みtailとの隣接差を検査し、1linkの同一コリドー幅は広げない。
    // 初期targetからの累積ずれもparallel検出幅で先に上限化している。
    if (std::abs(candidate.lateral_d_m - reachable_tail_d_m) >
        lateral_limit_m + 1.0e-6) {
      continue;
    }
    reachable_tail_s_m = candidate.unwrapped_s_m;
    reachable_tail_id = candidate.id;
    reachable_tail_speed_mps = candidate.speed_mps;
    reachable_tail_d_m = candidate.lateral_d_m;
    const double candidate_target_d_m = targetOffsetForPass(
        localized_lateral_profile_.pass_type,
        localized_lateral_profile_.start_d_m, candidate.lateral_d_m);
    if (localized_lateral_profile_.pass_type == CandidateType::PASS_LEFT) {
      staged_target_d_m = std::max(staged_target_d_m, candidate_target_d_m);
    } else if (localized_lateral_profile_.pass_type ==
               CandidateType::PASS_RIGHT) {
      staged_target_d_m = std::min(staged_target_d_m, candidate_target_d_m);
    }
    const auto existing_waypoint =
        std::find_if(localized_lateral_profile_.chain_waypoints.begin(),
                     localized_lateral_profile_.chain_waypoints.end(),
                     [&](const LocalizedLateralWaypoint &waypoint) {
                       return waypoint.target_id == candidate.id;
                     });
    if (existing_waypoint == localized_lateral_profile_.chain_waypoints.end()) {
      // 初回観測時のs/dを固定する。認可後の相手揺れで既存waypointを
      // 内外へ動かさず、後からfreshな列車が現れた時だけ末尾へ追加する。
      localized_lateral_profile_.chain_waypoints.push_back(
          LocalizedLateralWaypoint{candidate.id, candidate.unwrapped_s_m,
                                   staged_target_d_m, candidate.unwrapped_s_m,
                                   candidate.wrapped_s_m});
    } else {
      staged_target_d_m = existing_waypoint->target_d_m;
    }
    ++reachable_count;
  }

  std::sort(localized_lateral_profile_.chain_waypoints.begin(),
            localized_lateral_profile_.chain_waypoints.end(),
            [](const LocalizedLateralWaypoint &lhs,
               const LocalizedLateralWaypoint &rhs) {
              return lhs.target_s_m < rhs.target_s_m;
            });

  // 一時的な分類欠落でmerge位置を手前へ戻さない。新しいfreshな列末尾だけを
  // 前方へ延長し、SafetyEvaluatorは延長後のprofile全体を毎周期再評価する。
  if (!std::isfinite(localized_lateral_profile_.chain_tail_s_m) ||
      reachable_tail_s_m > localized_lateral_profile_.chain_tail_s_m + 1.0e-6) {
    localized_lateral_profile_.chain_tail_s_m = reachable_tail_s_m;
    localized_lateral_profile_.chain_tail_id = reachable_tail_id;
    localized_lateral_profile_.chain_tail_speed_mps = reachable_tail_speed_mps;
  }
  localized_lateral_profile_.chain_target_count =
      std::max(localized_lateral_profile_.chain_target_count, reachable_count);
  // 各車の外側dはchain_waypointsで段階化する。最終dを先頭車の手前から
  // 一括適用すると操舵不能になるため、scalar target_d_mは固定target車用の
  // 値として保持し、CandidateBuilderが車間ごとに次のdへ接続する。
  const double hold_after_m =
      std::max(0.0, config_.localized_avoidance_hold_after_target_m);
  const double merge_distance_m =
      std::max(1.0, config_.localized_avoidance_merge_distance_m);
  localized_lateral_profile_.full_offset_end_s_m =
      std::max(localized_lateral_profile_.full_offset_end_s_m,
               localized_lateral_profile_.chain_tail_s_m + hold_after_m);
  localized_lateral_profile_.merge_end_s_m =
      localized_lateral_profile_.full_offset_end_s_m + merge_distance_m;
}

// 入力: PASS_LEFT/PASS_RIGHT。
// 出力: そのPASS方向の目標横オフセットd。
// 処理概要: CandidateBuilder以外でもPASS方向から目標dを参照できるようにする。
double OvertakePlannerCore::targetOffsetForPass(CandidateType pass_type,
                                                double ego_d_m,
                                                double opponent_d_m) const {
  const double required_gap =
      config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h) +
      std::max(0.0, config_.pass_target_lateral_margin_m);
  return passTargetOffset(config_, pass_type, ego_d_m, opponent_d_m,
                          required_gap);
}

// 入力: 自車状態とBlockedInfo。
// 出力: 横並び時に後方へ譲るべきならtrue。
// 処理概要:
// 未来譲り、相手が前寄り、コーナー壁余裕不足をまとめてYIELD_BEHINDへ誘導する。
bool OvertakePlannerCore::shouldYieldBehindSideBySide(
    const EgoState &ego, const BlockedInfo &blocked_info) const {
  if (blocked_info.future_yield_required) {
    return true;
  }
  if (blocked_info.parallel_yield_hold_lateral) {
    return true;
  }
  if (!blocked_info.side_by_side) {
    return false;
  }
  if (blocked_info.side_delta_s > config_.side_yield_s_m) {
    return true;
  }
  if (!blocked_info.corner_side_by_side) {
    return false;
  }
  const bool opponent_not_clearly_behind =
      blocked_info.side_delta_s > -config_.side_yield_s_m;
  const bool close_to_wall =
      blocked_risk_.wallClearance(ego.frenet.s, ego.frenet.d) <=
      config_.corner_side_yield_wall_clearance_m;
  return opponent_not_clearly_behind || close_to_wall;
}

// 入力: 現在時刻、自車状態、最終PlannerOutput。
// 出力: なし。必要ならoutput.lateral_offsetsを保持済み値へ置き換える。
// 処理概要:
// 高速カーブ中の復帰/譲り/速度guardで横参照が毎周期揺れないようにholdする。
void OvertakePlannerCore::applyHighSpeedCurveLateralHold(
    double now_sec, const EgoState &ego, PlannerOutput &output) {
  output.lateral_target_hold_active = false;
  output.lateral_target_hold_reason.clear();

  if (output.blocked_info.pass_reauthorization_lockout_active) {
    // lockoutのcurrent-d hold / envelope recoveryを、以前のカーブhold列で
    // 上書きしない。どちらもこの周期の空間軸で生成・安全評価済みである。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    output.lateral_target_hold_active = true;
    output.lateral_target_hold_reason = "pass_profile_recovery_latched";
    return;
  }

  if (output.blocked_info.maneuver_transaction_tracking_stop_active ||
      output.blocked_info.maneuver_transaction_safe_lateral_hold_active) {
    // 未完了PASSのcurrent-d STOP/HOLDへ、前周期に保存したPASS/curve列を
    // 上書きしない。現在周期の全相手で評価した同じd列をそのまま維持する。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    output.lateral_target_hold_active = true;
    output.lateral_target_hold_reason = "maneuver_transaction_hold";
    return;
  }

  if (output.blocked_info.post_abort_curve_hold_active) {
    // ABORT完了後の専用holdは、直前の中心復帰列を再利用しない。現在dから
    // 新規生成・SafetyEvaluator通過済みの列を、そのままpublish再検証へ渡す。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    output.lateral_target_hold_active = true;
    output.lateral_target_hold_reason = "post_abort_curve_hold";
    return;
  }

  if (output.reentry_gate.requested && !output.reentry_gate.permitted) {
    // 以前のカーブhold列が中心方向を向いていても、復帰ゲート閉鎖中の
    // 現在d保持候補を上書きしてはならない。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }

  if (output.blocked_info.early_stationary_parallel_pass_hold_lateral) {
    // 停止車early PASSのGate 2不成立中は、現在dでSafetyEvaluatorを通した
    // RECOVERYを前周期のカーブhold列で上書きしない。
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    output.lateral_target_hold_active = true;
    output.lateral_target_hold_reason = "early_stationary_parallel_pass_hold";
    return;
  }

  if (!config_.high_speed_curve_lateral_hold_enabled) {
    high_speed_curve_lateral_hold_active_ = false;
    high_speed_curve_lateral_hold_offsets_.clear();
    high_speed_curve_lateral_hold_sec_ =
        std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const bool usable_output =
      output.active_override && !output.lateral_offsets.empty();
  const bool stabilized_mode = isLateralStabilizationMode(output.mode);
  const double curvature = std::max(output.blocked_info.corner_abs_curvature,
                                    output.blocked_info.future_abs_curvature);
  const double enter_curvature = highSpeedCurveHoldEnterCurvature(config_);
  const double release_curvature = highSpeedCurveHoldReleaseCurvature(config_);
  const double enter_speed =
      std::max(0.0, config_.high_speed_curve_lateral_hold_min_speed_mps);
  const double release_speed =
      config_.high_speed_curve_lateral_hold_release_speed_mps >= 0.0
          ? config_.high_speed_curve_lateral_hold_release_speed_mps
          : enter_speed;

  if (high_speed_curve_lateral_hold_active_) {
    // 処理ブロック: hold中は解除条件を満たすまで前回offset列を再利用する。
    // 設計意図: 横参照の再計算でMPC入力が急に変わることを避ける。
    const bool release = !usable_output || !stabilized_mode ||
                         ego.v <= release_speed ||
                         curvature <= release_curvature;
    if (release) {
      high_speed_curve_lateral_hold_active_ = false;
      high_speed_curve_lateral_hold_offsets_.clear();
      high_speed_curve_lateral_hold_sec_ =
          std::numeric_limits<double>::quiet_NaN();
    } else if (!high_speed_curve_lateral_hold_offsets_.empty()) {
      for (std::size_t i = 0; i < output.lateral_offsets.size(); ++i) {
        output.lateral_offsets[i] =
            heldOffsetAt(high_speed_curve_lateral_hold_offsets_, i);
      }
      output.target_lateral_offset_m = output.lateral_offsets.back();
      output.lateral_target_hold_active = true;
      output.lateral_target_hold_reason = "high_speed_curve_hold";
      return;
    }
  }

  const bool should_enter =
      usable_output && stabilized_mode && hasLateralStabilizationRisk(output) &&
      ego.v >= enter_speed && curvature >= enter_curvature;
  if (!should_enter) {
    return;
  }

  // 処理ブロック: holdへ入る瞬間のoffset列を保存する。
  // 設計意図:
  // 以後の周期ではこの列を基準にし、高速カーブが落ち着くまで横目標を固定する。
  high_speed_curve_lateral_hold_active_ = true;
  high_speed_curve_lateral_hold_offsets_ = output.lateral_offsets;
  high_speed_curve_lateral_hold_sec_ = now_sec;
  output.lateral_target_hold_active = true;
  output.lateral_target_hold_reason = "high_speed_curve_hold";
}

// 入力: 現在時刻とPlannerOutput。
// 出力: なし。横オフセット列を前回publish値からの最大変化量以内に制限する。
// 処理概要: publish周期ごとのtarget d急変を抑え、MPCへ滑らかな参照を渡す。
void OvertakePlannerCore::applyLateralTargetRateLimit(double now_sec,
                                                      PlannerOutput &output) {
  if (output.blocked_info.maneuver_transaction_tracking_stop_active ||
      output.blocked_info.maneuver_transaction_safe_lateral_hold_active) {
    // current-d STOP/HOLDはCandidateBuilderとSafetyEvaluatorが同じ空間列を
    // 検証済み。前周期のPASS列とhorizon index単位で補間すると未評価の横移動へ
    // 変形するため、transaction所有中は周期間rate limitを適用しない。
    return;
  }
  if (output.blocked_info.pass_reauthorization_lockout_active) {
    // lockoutのcurrent-d holdと包絡内RECOVERYはCandidateBuilderが空間軸で
    // 生成し、その同じ列をSafetyEvaluator/trackabilityへ通している。
    // 直前PASSの別sに対応するhorizon indexを混ぜると、STOP holdを未評価の
    // 横移動へ変形し得るため、汎用の周期間rate limitを二重適用しない。
    return;
  }
  const bool evaluated_localized_pass =
      localized_lateral_profile_.active && isAnyPassMode(output.mode) &&
      output.selected == localized_lateral_profile_.pass_type;
  if (evaluated_localized_pass) {
    // localized_latchedの横列は、ラッチ済みtarget sに対する空間profileとして
    // CandidateBuilderが生成し、その同じ列をSafetyEvaluatorへ通している。
    // 前周期と同じhorizon indexを混ぜる旧rate limitは、車両が前進すると異なる
    // s同士を補間してしまい、評価済みPASSを未評価の衝突軌道へ変形させる。
    // 始点は毎周期ego
    // dへ連続接続されるため、この文脈だけは評価済み空間profileを
    // そのままpublishする。legacy PASSやtarget未ラッチ時の急変保護は維持する。
    return;
  }
  const bool evaluated_attack_follow =
      output.selected == CandidateType::FOLLOW &&
      (output.blocked_info.attack_follow_hold_pass_side ||
       output.blocked_info.prestart_attack_follow_hold_lateral);
  if (evaluated_attack_follow) {
    // PASS側dまたは現在dを使って生成・評価済みのFOLLOW空間profileへ、前周期の
    // 別s indexを混ぜない。publish直前にはrevalidatePublishedLateralで同じ
    // 安全制約を再確認する。
    return;
  }
  if (output.blocked_info.post_abort_curve_hold_active ||
      (output.reentry_gate.requested && !output.reentry_gate.permitted) ||
      output.blocked_info.parallel_follow_hold_lateral ||
      output.blocked_info.early_stationary_parallel_pass_hold_lateral ||
      output.blocked_info.braking_follow_hold_lateral) {
    // 前周期の通常ライン向けoffsetを混ぜると、hold中でも横方向に進む。
    // ゲート拒否時のRECOVERYとparallel FOLLOWは、CandidateBuilderが生成した
    // 現在d保持列をそのまま使う。
    return;
  }
  if (!output.active_override || output.lateral_offsets.empty() ||
      config_.lateral_target_max_step_m <= 0.0 ||
      last_published_lateral_offsets_.empty() ||
      !std::isfinite(last_published_lateral_target_sec_) ||
      now_sec - last_published_lateral_target_sec_ >
          kPublishedLateralTargetMemorySec) {
    return;
  }

  const double max_step = config_.lateral_target_max_step_m;
  for (std::size_t i = 0; i < output.lateral_offsets.size(); ++i) {
    const std::size_t prev_i =
        std::min(i, last_published_lateral_offsets_.size() - 1);
    const double prev = last_published_lateral_offsets_[prev_i];
    double &current = output.lateral_offsets[i];
    if (!std::isfinite(prev) || !std::isfinite(current)) {
      continue;
    }
    current = prev + std::clamp(current - prev, -max_step, max_step);
  }
  output.target_lateral_offset_m = output.lateral_offsets.back();
}

// 入力: 現在時刻とPlannerOutput。
// 出力: なし。次周期のrate limit/hold用に最後の横オフセット列を記憶する。
// 処理概要:
// overrideが途切れて一定時間経ったら古い記憶を破棄し、無関係な制限を残さない。
void OvertakePlannerCore::rememberPublishedLateralTarget(
    double now_sec, const PlannerOutput &output) {
  if (output.active_override && !output.lateral_offsets.empty()) {
    last_published_lateral_offsets_ = output.lateral_offsets;
    last_published_lateral_target_sec_ = now_sec;
    return;
  }
  if (std::isfinite(last_published_lateral_target_sec_) &&
      now_sec - last_published_lateral_target_sec_ >
          kPublishedLateralTargetMemorySec) {
    last_published_lateral_offsets_.clear();
    last_published_lateral_target_sec_ =
        std::numeric_limits<double>::quiet_NaN();
  }
}

// 入力: scoreとfeasibleが設定済みの候補集合。
// 出力: publish/状態機械へ渡す暫定選択候補。
// 処理概要:
// feasible候補を最優先し、全候補unsafeの場合だけ最小scoreのunsafe候補を診断用に返す。
CandidateTrajectory OvertakePlannerCore::selectCandidate(
    std::vector<CandidateTrajectory> &candidates) const {
  // feasible候補があるなら必ずそれを優先する。unsafe候補は全候補がunsafeの時だけ診断用に返す。
  const auto by_score = [](const CandidateTrajectory &a,
                           const CandidateTrajectory &b) {
    return a.score < b.score;
  };
  auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [&by_score](const CandidateTrajectory &a, const CandidateTrajectory &b) {
        if (a.feasible != b.feasible) {
          return a.feasible;
        }
        return by_score(a, b);
      });
  if (best == candidates.end()) {
    return {};
  }
  if (best->feasible) {
    return *best;
  }

  best = std::min_element(candidates.begin(), candidates.end(), by_score);
  if (best == candidates.end()) {
    return {};
  }
  return *best;
}

} // namespace overtake_planner
