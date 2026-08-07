#ifndef OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_
#define OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_

#include "overtake_planner/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace overtake_planner {

enum class ReferenceOverrideWireKind {
  INACTIVE,
  LATERAL_AND_SPEED_V1,
  SPEED_ONLY_V2,
  LATERAL_AND_SPEED_V3,
  SPATIAL_LATERAL_AND_SPEED_V4,
};

struct ReferenceOverrideWirePayload {
  ReferenceOverrideWireKind kind{ReferenceOverrideWireKind::INACTIVE};
  int mode_id{0};
  double speed_cap_mps{0.0};
  std::vector<float> data;
};

enum class OverrideTrackingIdentityKind {
  NONE,
  COMMITTED_PASS,
  ATTACK_FOLLOW,
};

struct OverrideTrackingIdentity {
  std::uint32_t generation{0U};
  std::uint64_t attempt_id{0U};
  OverrideTrackingIdentityKind kind{OverrideTrackingIdentityKind::NONE};
  std::string target_id{};
  CandidateType pass_type{CandidateType::FASTEST};
};

// 入力: current/previousの型付きpublish履歴、ControllerTrackingStatus世代、
// 現在のPlanner世代とattempt。
// 出力: exact currentまたは高々N-1で、同じattemptに属する履歴だけを返す。
// 処理概要: target/sideは呼び出し側のCoreが現transactionと再照合する。ここでは
// generation
// wrapを含む配送差だけを扱い、N-2、完了attempt、identityなしを拒否する。
inline const OverrideTrackingIdentity *overrideTrackingIdentityForProof(
    const OverrideTrackingIdentity &current,
    const OverrideTrackingIdentity &previous, std::uint32_t status_generation,
    std::uint32_t current_generation, std::uint64_t current_attempt_id,
    const std::string &current_target_id, CandidateType current_pass_type) {
  if (status_generation == 0U || current_generation == 0U ||
      current_attempt_id == 0U || current_target_id.empty() ||
      (current_pass_type != CandidateType::PASS_LEFT &&
       current_pass_type != CandidateType::PASS_RIGHT)) {
    return nullptr;
  }
  const std::uint32_t previous_generation =
      current_generation <= 1U ? 16777215U : current_generation - 1U;
  if (status_generation != current_generation &&
      status_generation != previous_generation) {
    return nullptr;
  }
  const auto identity_matches = [status_generation, current_attempt_id,
                                 &current_target_id,
                                 current_pass_type](const auto &identity) {
    return identity.kind != OverrideTrackingIdentityKind::NONE &&
           identity.generation == status_generation &&
           identity.attempt_id == current_attempt_id &&
           identity.target_id == current_target_id &&
           identity.pass_type == current_pass_type;
  };
  if (identity_matches(current)) {
    return &current;
  }
  return identity_matches(previous) ? &previous : nullptr;
}

// 入力: 通常payload/identity差分と、Coreが発行したPASS warm-up token。
// 出力: reference override generationを進める必要がある時だけtrue。
// 処理概要: 再生成後のFloat32列が偶然同じでもtoken変更を世代変更として扱い、
// 古いControllerTrackingStatusを別の安全評価snapshotへ流用しない。
inline bool referenceOverrideGenerationMustAdvance(
    bool payload_or_identity_changed, bool tracking_release_pass_warmup,
    std::uint64_t tracking_release_token, bool has_last_tracking_release_token,
    std::uint64_t last_tracking_release_token) {
  const bool tracking_release_token_changed =
      tracking_release_pass_warmup && tracking_release_token != 0U &&
      (!has_last_tracking_release_token ||
       tracking_release_token != last_tracking_release_token);
  return payload_or_identity_changed || tracking_release_token_changed;
}

// 入力: current generationのcontroller proofと、PASS warm-up期待identity。
// 出力: generic trajectory readyまたはexact PASS warm-up ACKだけtrue。
// 処理概要: current-d STOP bootstrapはACK期待前だけgeneric readyとして使う。
// warm-up開始後はtokenが紐付くgeneration/target/sideをすべて一致させ、入力欠損
// 周期のSTOP ACKや再生成前の旧ACKをPASS解除へ流用しない。
inline bool trackingReleaseProofReady(
    bool current_generation_ready, bool tracking_release_ack_expected,
    std::uint64_t expected_token, std::uint32_t expected_generation,
    std::uint32_t current_generation, bool last_latch_active,
    const std::string &last_target_id, const std::string &expected_target_id,
    CandidateType last_pass_type, CandidateType expected_pass_type) {
  if (!current_generation_ready) {
    return false;
  }
  if (!tracking_release_ack_expected) {
    return true;
  }
  return expected_token != 0U && expected_generation == current_generation &&
         last_latch_active && !expected_target_id.empty() &&
         last_target_id == expected_target_id &&
         last_pass_type == expected_pass_type &&
         (expected_pass_type == CandidateType::PASS_LEFT ||
          expected_pass_type == CandidateType::PASS_RIGHT);
}

// 入力: Muxが型付きで返したSTOP transport証明と、PlannerのPASS ACK状態。
// 出力: ACK未期待のATTACK_FOLLOW STOPで、証明が正確にN-1の時だけtrue。
// 処理概要: diagnostic reason文字列は認可契約へ使わない。current generationの
// exact proofは通常経路が所有し、PASS warm-up中の旧FOLLOW proofは拒否する。
inline bool attackFollowStopTransportReleaseProofReady(
    bool tracking_release_ack_expected, bool status_fresh,
    bool pp_command_fresh, bool safety_constraint_release_ready,
    bool attack_follow_stop_transport_release_ready,
    std::uint32_t status_generation, std::uint32_t current_generation) {
  if (tracking_release_ack_expected || !status_fresh || !pp_command_fresh ||
      !safety_constraint_release_ready ||
      !attack_follow_stop_transport_release_ready || status_generation == 0U ||
      current_generation == 0U) {
    return false;
  }
  const std::uint32_t previous_generation =
      current_generation <= 1U ? 16777215U : current_generation - 1U;
  return status_generation == previous_generation;
}

// 入力: Plannerが最終的にpublishする出力。
// 出力: tracking STOP transaction中のSAFE_STOP/FOLLOW構造ならtrue。
// 処理概要: 横認可やmodeが不整合でもgeneric lateral wireへ流さないため、
// authority判定より前にraw/selected/transactionだけを識別する。
inline bool isTrackingStopStructuralIntent(const PlannerOutput &output) {
  return output.raw_selected == CandidateType::SAFE_STOP &&
         (output.selected == CandidateType::SAFE_STOP ||
          output.selected == CandidateType::FOLLOW) &&
         output.blocked_info.maneuver_transaction_tracking_stop_active;
}

// 入力: Plannerが最終的にpublishする出力。
// 出力: start-grid transaction内の安全評価済みcurrent-d停止候補ならtrue。
// 処理概要: SAFE_STOP/FOLLOWの表示揺れに依存せず、同じtargetを保持した停止
// bootstrapの構造だけを検証する。ego/constraintとの一致は呼び出し側で重ねる。
inline bool isTrackingStopBootstrapOutput(const PlannerOutput &output) {
  const auto &blocked = output.blocked_info;
  const bool transaction_context =
      blocked.start_grid_target_active &&
      (blocked.maneuver_transaction_prepared ||
       (blocked.maneuver_transaction_retry_active &&
        blocked.maneuver_transaction_incomplete));
  const bool exact_safe_stop =
      output.selected == CandidateType::SAFE_STOP &&
      output.raw_selected == CandidateType::SAFE_STOP &&
      output.lateral_tracking_authorized_during_stop;
  const bool stopped_attack_follow =
      output.mode == BehaviorMode::FOLLOW_BLOCKED &&
      output.selected == CandidateType::FOLLOW &&
      output.raw_selected == CandidateType::SAFE_STOP &&
      !output.safe_stop_triggered &&
      !output.lateral_tracking_authorized_during_stop;
  if (!transaction_context ||
      !blocked.maneuver_transaction_tracking_stop_active ||
      !blocked.maneuver_target_latched || blocked.maneuver_target_id.empty() ||
      (!exact_safe_stop && !stopped_attack_follow) ||
      output.tracking_release_pass_warmup ||
      output.tracking_release_token != 0U || !output.active_override ||
      !output.selected_lateral_profile_safety_verified ||
      !output.lateral_stop_inputs_complete ||
      output.lateral_offsets.size() < 2U ||
      output.lateral_offsets.size() != output.speed_caps.size() ||
      output.lateral_offsets.size() != output.longitudinal_offsets_m.size() ||
      !std::isfinite(output.safe_stop_v_mps) || output.safe_stop_v_mps <= 0.0) {
    return false;
  }
  const double current_d_m = output.lateral_offsets.front();
  const bool constant_d_profile =
      std::isfinite(current_d_m) &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [current_d_m](double d_m) {
                    return std::isfinite(d_m) &&
                           std::abs(d_m - current_d_m) <= 1.0e-4;
                  });
  const bool finite_speed_profile = std::all_of(
      output.speed_caps.begin(), output.speed_caps.end(), [](double speed_mps) {
        return std::isfinite(speed_mps) && speed_mps >= 0.0;
      });
  const bool finite_monotonic_axis =
      std::isfinite(output.longitudinal_offsets_m.front()) &&
      std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-6 &&
      std::all_of(
          output.longitudinal_offsets_m.begin(),
          output.longitudinal_offsets_m.end(),
          [](double ds_m) { return std::isfinite(ds_m) && ds_m >= 0.0; }) &&
      std::adjacent_find(output.longitudinal_offsets_m.begin(),
                         output.longitudinal_offsets_m.end(),
                         [](double previous, double next) {
                           return next + 1.0e-9 < previous;
                         }) == output.longitudinal_offsets_m.end();
  const bool lateral_stop_profile_executable =
      !output.lateral_tracking_authorized_during_stop ||
      (output.longitudinal_offsets_m.back() > 1.0e-6 &&
       std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                   [](double speed_mps) { return speed_mps > 0.0; }));
  return constant_d_profile && finite_speed_profile && finite_monotonic_axis &&
         lateral_stop_profile_executable;
}

// 入力: 上で検証済みのtracking-stop出力。
// 出力: PP transport bootstrapとSafetyConstraintで共有する正の速度上限[m/s]。
// 処理概要: 軌道先頭のv_refは停止自車速度に接続されるため、全体上限へ使うと
// ほぼ0 m/sで自己デッドロックする。明示safe-stop capを基準にし、別途設定済みの
// global capだけをより厳しい上限として反映する。
inline double trackingStopBootstrapSpeedCapMps(const PlannerOutput &output) {
  double cap_mps = output.safe_stop_v_mps;
  if (std::isfinite(output.applied_speed_cap_mps) &&
      output.applied_speed_cap_mps > 0.0) {
    cap_mps = std::min(cap_mps, output.applied_speed_cap_mps);
  }
  return cap_mps;
}

// 入力: Plannerが最終的にpublishする出力、自車状態、同周期の停止constraint。
// 出力: 旧PASSを捨て、現在dの安全評価済みSTOPを一段目proofへ使える時だけtrue。
// 処理概要: 空間長0のSAFE_STOPまたはcurrent-d ATTACK_FOLLOWを架空のPASSへ
// 変換せず、target/sideを保持したtransactionの停止契約だけを厳密に識別する。
// これはPASS運動認可ではなく、次の新token PASS warm-upを発行するための
// bootstrap契約である。
inline bool isSafetyEvaluatedCurrentDTrackingStop(
    const PlannerOutput &output, const EgoState &ego, bool constraint_valid,
    bool constraint_stop_requested, bool constraint_release_authorized,
    double constraint_speed_limit_mps, const std::string &constraint_reason) {
  const bool accepted_constraint_reason =
      constraint_reason == "maneuver_transaction_tracking_stop" ||
      constraint_reason == "release_pending_safe_cycles";
  if (!isTrackingStopBootstrapOutput(output) || !ego.valid ||
      !std::isfinite(ego.frenet.d) || !constraint_valid ||
      !constraint_stop_requested || constraint_release_authorized ||
      !std::isfinite(constraint_speed_limit_mps) ||
      constraint_speed_limit_mps <= 0.0 ||
      constraint_speed_limit_mps > output.safe_stop_v_mps + 1.0e-6 ||
      !accepted_constraint_reason) {
    return false;
  }
  return std::abs(output.lateral_offsets.front() - ego.frenet.d) <= 1.0e-4;
}

// 入力: 最終PASS出力と、SafetyConstraintReleaseGateでSTOP保持された同周期の
// constraint。
// 出力: 縦方向は停止したまま、安全評価済みPASS trajectoryの横追従だけを
// typed planへ認可してよい時だけtrue。
// 処理概要: release debounce中にPASSING+lateral requiredをunauthorizedで送ると
// Muxでは不正planとなり、通常のSTOP確認がauthority faultへ自己増幅する。
// 初回候補、未評価profile、入力欠損、別target/side、外部STOP理由には広げず、
// commit済み同一transactionのrelease_pending_safe_cyclesだけを識別する。
inline bool isReleasePendingPassLateralTrackingAuthorized(
    const PlannerOutput &output, bool constraint_valid,
    bool constraint_stop_requested, bool constraint_release_authorized,
    const std::string &constraint_reason) {
  const auto &blocked = output.blocked_info;
  const bool selected_pass = output.selected == CandidateType::PASS_LEFT ||
                             output.selected == CandidateType::PASS_RIGHT;
  const bool mode_matches_selected =
      (output.selected == CandidateType::PASS_LEFT &&
       (output.mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
        output.mode == BehaviorMode::OVERTAKE_LEFT)) ||
      (output.selected == CandidateType::PASS_RIGHT &&
       (output.mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
        output.mode == BehaviorMode::OVERTAKE_RIGHT));
  const bool identity_matches =
      blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_target_latched && !blocked.maneuver_target_id.empty() &&
      blocked.maneuver_transaction_pass_type == output.selected;
  const bool profile_shape_valid =
      output.lateral_offsets.size() > 1U &&
      output.lateral_offsets.size() == output.speed_caps.size() &&
      output.lateral_offsets.size() == output.longitudinal_offsets_m.size() &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [](double d_m) { return std::isfinite(d_m); }) &&
      std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                  [](double speed_mps) {
                    return std::isfinite(speed_mps) && speed_mps >= 0.0;
                  }) &&
      std::all_of(output.longitudinal_offsets_m.begin(),
                  output.longitudinal_offsets_m.end(), [](double ds_m) {
                    return std::isfinite(ds_m) && ds_m >= 0.0;
                  });
  const bool spatial_horizon_proven =
      output.controller_spatial_horizon_proof_valid &&
      std::isfinite(output.required_controller_spatial_horizon_m) &&
      output.required_controller_spatial_horizon_m > 0.0 &&
      profile_shape_valid &&
      std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-5 &&
      std::is_sorted(output.longitudinal_offsets_m.begin(),
                     output.longitudinal_offsets_m.end()) &&
      output.longitudinal_offsets_m.back() >=
          output.required_controller_spatial_horizon_m;
  return constraint_valid && constraint_stop_requested &&
         !constraint_release_authorized &&
         constraint_reason == "release_pending_safe_cycles" && selected_pass &&
         mode_matches_selected && identity_matches && output.active_override &&
         output.solver_horizon_intent !=
             PlannerOutput::SolverHorizonIntent::NONE &&
         output.selected_lateral_profile_safety_verified &&
         !output.published_lateral_safety_rejected &&
         output.lateral_stop_inputs_complete && spatial_horizon_proven;
}

// 入力: 初回PASSをprepareした同周期の最終出力と、release debounceで
// STOP保持されたconstraint。
// 出力: 未commitの初回PASS payloadを、縦STOPのままPPへwarm-upしてよい時だけ
// true。
// 処理概要: commit後のincomplete transactionとは契約を分離する。Coreが明示した
// prepared/pending/warm-up/token、同一target/side、安全評価、完全入力、
// controller空間horizonをすべて要求し、一つでも欠ければfail-closedにする。
// 実際にpublishするv4 wireとのexact一致はNodeのauthorization boundaryで重ねる。
inline bool isInitialPreparedPassWarmupLateralTrackingAuthorized(
    const PlannerOutput &output, bool constraint_valid,
    bool constraint_stop_requested, bool constraint_release_authorized,
    const std::string &constraint_reason) {
  const auto &blocked = output.blocked_info;
  const bool selected_pass = output.selected == CandidateType::PASS_LEFT ||
                             output.selected == CandidateType::PASS_RIGHT;
  const bool mode_matches_selected =
      (output.selected == CandidateType::PASS_LEFT &&
       output.mode == BehaviorMode::PREPARE_OVERTAKE_LEFT) ||
      (output.selected == CandidateType::PASS_RIGHT &&
       output.mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT);
  const bool initial_transaction_identity_matches =
      blocked.maneuver_transaction_prepared &&
      !blocked.maneuver_transaction_incomplete &&
      blocked.maneuver_transaction_tracking_stop_active &&
      blocked.maneuver_transaction_tracking_release_pending &&
      output.tracking_release_pass_warmup &&
      output.tracking_release_token != 0U && blocked.maneuver_target_latched &&
      !blocked.maneuver_target_id.empty() && output.maneuver_latch_active &&
      output.maneuver_latch_target_id == blocked.maneuver_target_id &&
      blocked.maneuver_transaction_pass_type == output.selected;
  const bool profile_shape_valid =
      output.lateral_offsets.size() > 1U &&
      output.lateral_offsets.size() == output.speed_caps.size() &&
      output.lateral_offsets.size() == output.longitudinal_offsets_m.size() &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [](double d_m) { return std::isfinite(d_m); }) &&
      std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                  [](double speed_mps) {
                    return std::isfinite(speed_mps) && speed_mps >= 0.0;
                  }) &&
      std::all_of(output.longitudinal_offsets_m.begin(),
                  output.longitudinal_offsets_m.end(), [](double ds_m) {
                    return std::isfinite(ds_m) && ds_m >= 0.0;
                  });
  const bool spatial_horizon_proven =
      output.controller_spatial_horizon_proof_valid &&
      std::isfinite(output.required_controller_spatial_horizon_m) &&
      output.required_controller_spatial_horizon_m > 0.0 &&
      profile_shape_valid &&
      std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-5 &&
      std::is_sorted(output.longitudinal_offsets_m.begin(),
                     output.longitudinal_offsets_m.end()) &&
      output.longitudinal_offsets_m.back() >=
          output.required_controller_spatial_horizon_m;
  return constraint_valid && constraint_stop_requested &&
         !constraint_release_authorized &&
         constraint_reason == "release_pending_safe_cycles" && selected_pass &&
         mode_matches_selected && initial_transaction_identity_matches &&
         output.active_override &&
         output.solver_horizon_intent !=
             PlannerOutput::SolverHorizonIntent::NONE &&
         output.selected_lateral_profile_safety_verified &&
         !output.published_lateral_safety_rejected &&
         output.lateral_stop_inputs_complete && spatial_horizon_proven;
}

// 入力: 最終出力と、上のcurrent-d STOP判定。
// 出力: 型付きOvertakePlanに載せる方向（左=1、右=-1、非PASS=0）。
// 処理概要: transaction内部のsideは保持するが、current-d STOPを未認可PASSINGに
// 見せない。新tokenのPASSが再生成された世代ではselected/modeから方向を復元する。
inline std::int8_t
authoritativePlanPassDirection(const PlannerOutput &output,
                               bool current_d_tracking_stop) {
  if (current_d_tracking_stop) {
    return 0;
  }
  const bool committed_transaction =
      output.blocked_info.maneuver_transaction_incomplete;
  const bool left = output.mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
                    output.mode == BehaviorMode::OVERTAKE_LEFT ||
                    output.selected == CandidateType::PASS_LEFT ||
                    (committed_transaction &&
                     output.blocked_info.maneuver_transaction_pass_type ==
                         CandidateType::PASS_LEFT);
  const bool right = output.mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
                     output.mode == BehaviorMode::OVERTAKE_RIGHT ||
                     output.selected == CandidateType::PASS_RIGHT ||
                     (committed_transaction &&
                      output.blocked_info.maneuver_transaction_pass_type ==
                          CandidateType::PASS_RIGHT);
  return static_cast<std::int8_t>(left ? 1 : (right ? -1 : 0));
}

// 入力: Planner内部modeと最終publish候補。
// 出力: PP/MPCのwire契約で使うmode ID。
// 処理概要: 停止constraint下で同じPASS trajectoryをwarm-upするPREPAREと、
// proof確認後のOVERTAKEでpayload generationを変えない。両者は下流から見れば
// 同じ方向・同じ軌道のPASSINGであり、実行可否はSafetyConstraintが所有する。
inline int referenceOverrideWireModeId(const PlannerOutput &output) {
  if (output.active_override && output.selected == CandidateType::PASS_LEFT &&
      output.mode == BehaviorMode::PREPARE_OVERTAKE_LEFT) {
    return static_cast<int>(BehaviorMode::OVERTAKE_LEFT);
  }
  if (output.active_override && output.selected == CandidateType::PASS_RIGHT &&
      output.mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT) {
    return static_cast<int>(BehaviorMode::OVERTAKE_RIGHT);
  }
  return static_cast<int>(output.mode);
}

// 入力: 同じ契約で生成した2つのwire payload。
// 出力: generation以外の意味的な内容が一致する時だけtrue。
// 処理概要: generationは各変更を識別するための末尾フィールドであり、同じ
// overrideを再publishするかの判定には含めない。生成関数が作る全契約形式で
// generationは末尾にあるため、先頭から末尾直前までを比較する。
inline bool referenceOverrideWirePayloadSemanticallyEqual(
    const ReferenceOverrideWirePayload &lhs,
    const ReferenceOverrideWirePayload &rhs) {
  if (lhs.kind != rhs.kind || lhs.mode_id != rhs.mode_id || lhs.data.empty() ||
      lhs.data.size() != rhs.data.size()) {
    return false;
  }
  // v1 / explicit inactive append generation at the end. v2 is intentionally
  // [valid, mode, 0, version, generation, cap], so its generation is index 4.
  // v3 appends [version, generation, solver_horizon_intent].
  const std::size_t generation_index =
      lhs.kind == ReferenceOverrideWireKind::SPEED_ONLY_V2 ? 4U
      : (lhs.kind == ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3 ||
         lhs.kind == ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4)
          ? lhs.data.size() - 2U
          : lhs.data.size() - 1U;
  if (generation_index >= lhs.data.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.data.size(); ++i) {
    if (i != generation_index && lhs.data[i] != rhs.data[i]) {
      return false;
    }
  }
  return true;
}

// 入力: PlannerOutputとFloat32で正確に表せるgeneration。
// 出力: /overtake/reference_overrideへ出すv4/v3またはv2 payload。
// 処理概要: 安全評価済み横列と距離軸がある時はv4、横列だけならv3を使い、
// 横列を安全に作れない時はspeed-only v2へ落とす。どれも成立しない場合だけ
// 明示inactiveを出す。
inline ReferenceOverrideWirePayload
makeReferenceOverrideWirePayload(const PlannerOutput &output,
                                 std::uint32_t generation) {
  constexpr std::uint32_t kMaxGeneration = 16777215U;
  const auto valid_generation =
      std::clamp(generation, std::uint32_t{1}, kMaxGeneration);
  const int mode_id = referenceOverrideWireModeId(output);
  const auto inactive_payload = [valid_generation]() {
    ReferenceOverrideWirePayload payload;
    payload.data = {1.0F, 0.0F, 0.0F, 1.0F,
                    static_cast<float>(valid_generation)};
    return payload;
  };

  const bool tracking_stop_bootstrap = isTrackingStopBootstrapOutput(output);
  const bool tracking_stop_intent = isTrackingStopStructuralIntent(output);
  const auto tracking_stop_speed_only_payload = [&output, valid_generation,
                                                 &inactive_payload]() {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::SPEED_ONLY_V2;
    payload.mode_id = static_cast<int>(BehaviorMode::FOLLOW_BLOCKED);
    payload.speed_cap_mps = trackingStopBootstrapSpeedCapMps(output);
    if (!std::isfinite(payload.speed_cap_mps) || payload.speed_cap_mps <= 0.0) {
      return inactive_payload();
    }
    payload.data = {
        1.0F,
        static_cast<float>(payload.mode_id),
        0.0F,
        2.0F,
        static_cast<float>(valid_generation),
        static_cast<float>(payload.speed_cap_mps),
    };
    return payload;
  };
  if (tracking_stop_intent &&
      (!tracking_stop_bootstrap ||
       !output.lateral_tracking_authorized_during_stop)) {
    // 横認可のない停止bootstrapは従来どおり縦capだけを渡す。baseline操舵を
    // current-d追従proofへ格上げしない。構造上SAFE_STOPのまま横認可が失効
    // した時も、安全証跡、入力完全性、constant-d、実行horizonのどれかが
    // 欠けた時もgeneric v4へ流さない。
    return tracking_stop_speed_only_payload();
  }

  const bool lateral_payload_valid =
      output.active_override && mode_id > 0 &&
      output.lateral_offsets.size() == output.speed_caps.size() &&
      !output.lateral_offsets.empty() &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [](double value) { return std::isfinite(value); }) &&
      std::all_of(
          output.speed_caps.begin(), output.speed_caps.end(),
          [](double value) { return std::isfinite(value) && value > 0.0; });
  const bool spatial_axis_valid =
      lateral_payload_valid &&
      output.longitudinal_offsets_m.size() == output.lateral_offsets.size() &&
      !output.longitudinal_offsets_m.empty() &&
      std::isfinite(output.longitudinal_offsets_m.front()) &&
      std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-6 &&
      std::isfinite(output.longitudinal_offsets_m.back()) &&
      output.longitudinal_offsets_m.back() > 1.0e-6 &&
      std::adjacent_find(output.longitudinal_offsets_m.begin(),
                         output.longitudinal_offsets_m.end(),
                         [](double previous, double next) {
                           return !std::isfinite(previous) ||
                                  !std::isfinite(next) ||
                                  next + 1.0e-9 < previous;
                         }) == output.longitudinal_offsets_m.end();
  const bool spatial_axis_supplied = !output.longitudinal_offsets_m.empty();
  if (spatial_axis_valid) {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4;
    payload.mode_id = mode_id;
    payload.data.reserve(3 + output.lateral_offsets.size() * 3 + 3);
    payload.data = {1.0F, static_cast<float>(mode_id),
                    static_cast<float>(output.lateral_offsets.size())};
    for (const double offset_m : output.lateral_offsets) {
      payload.data.push_back(static_cast<float>(offset_m));
    }
    for (const double speed_cap_mps : output.speed_caps) {
      payload.data.push_back(static_cast<float>(speed_cap_mps));
    }
    for (const double distance_m : output.longitudinal_offsets_m) {
      payload.data.push_back(static_cast<float>(distance_m));
    }
    // v4: [valid, mode, n, d[n], v[n], ds[n], version=4, generation, intent].
    // dsは候補時間horizonの各点が先頭から何m先かを表す。MPCは従来どおり
    // 時間indexを使い、Pure Pursuitはdsで参照軌道へ再サンプルする。
    payload.data.push_back(4.0F);
    payload.data.push_back(static_cast<float>(valid_generation));
    payload.data.push_back(
        static_cast<float>(static_cast<int>(output.solver_horizon_intent)));
    return payload;
  }
  if (tracking_stop_bootstrap) {
    // current-d横追従を認可したSTOPは、PPが同じ空間profileを受理できる時だけ
    // 上のv4へ到達する。不完全な距離軸や非正速度をv3へ縮退させず、縦STOP
    // だけへ閉じる。
    return tracking_stop_speed_only_payload();
  }
  if (lateral_payload_valid && !spatial_axis_supplied) {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3;
    payload.mode_id = mode_id;
    payload.data.reserve(3 + output.lateral_offsets.size() * 2 + 3);
    payload.data = {1.0F, static_cast<float>(mode_id),
                    static_cast<float>(output.lateral_offsets.size())};
    for (const double offset_m : output.lateral_offsets) {
      payload.data.push_back(static_cast<float>(offset_m));
    }
    for (const double speed_cap_mps : output.speed_caps) {
      payload.data.push_back(static_cast<float>(speed_cap_mps));
    }
    // v3: [valid, mode, n, d[n], v[n], version=3, generation, intent].
    // intent=NONEの横列は通常trajectory用であり、solver horizonとしては
    // publish/採用してはならない。旧receiverはv3を厳密にrejectするため、
    // 横軌道を誤って継続するより安全側へ倒れる。
    payload.data.push_back(3.0F);
    payload.data.push_back(static_cast<float>(valid_generation));
    payload.data.push_back(
        static_cast<float>(static_cast<int>(output.solver_horizon_intent)));
    return payload;
  }

  double speed_cap_mps = output.applied_speed_cap_mps;
  if (!std::isfinite(speed_cap_mps) || speed_cap_mps <= 0.0) {
    speed_cap_mps = std::numeric_limits<double>::infinity();
    for (const double candidate_cap_mps : output.speed_caps) {
      if (std::isfinite(candidate_cap_mps) && candidate_cap_mps > 0.0) {
        speed_cap_mps = std::min(speed_cap_mps, candidate_cap_mps);
      }
    }
  }
  if (output.longitudinal_speed_cap_active && mode_id > 0 &&
      std::isfinite(speed_cap_mps) && speed_cap_mps > 0.0) {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::SPEED_ONLY_V2;
    payload.mode_id = mode_id;
    payload.speed_cap_mps = speed_cap_mps;
    // v2: [valid, mode_id, n=0, contract_version=2, generation, speed_cap].
    payload.data = {1.0F,
                    static_cast<float>(mode_id),
                    0.0F,
                    2.0F,
                    static_cast<float>(valid_generation),
                    static_cast<float>(speed_cap_mps)};
    return payload;
  }
  return inactive_payload();
}

// 入力: Planner出力、同じ周期にPPへpublishするwireとgeneration。
// 出力: d/v/ds、mode、intentを含む完全なv4 payloadが一致する時だけtrue。
// 処理概要: typed planの横列をgenerationだけで照合せず、PPが実際に受信する
// 全v4フィールドまで一致させる。speed-only、v3、不完全profileは拒否する。
inline bool
spatialLateralWireMatchesOutput(const PlannerOutput &output,
                                const ReferenceOverrideWirePayload &wire,
                                std::uint32_t expected_generation) {
  if (expected_generation == 0U || expected_generation > 16777215U ||
      wire.kind != ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4 ||
      wire.mode_id != referenceOverrideWireModeId(output) ||
      output.lateral_offsets.size() < 2U ||
      output.lateral_offsets.size() != output.speed_caps.size() ||
      output.lateral_offsets.size() != output.longitudinal_offsets_m.size()) {
    return false;
  }
  const std::size_t count = output.lateral_offsets.size();
  const std::size_t expected_size = 3U + 3U * count + 3U;
  if (wire.data.size() != expected_size || wire.data[0] != 1.0F ||
      wire.data[1] != static_cast<float>(wire.mode_id) ||
      wire.data[2] != static_cast<float>(count) ||
      wire.data[3U + 3U * count] != 4.0F ||
      wire.data[3U + 3U * count + 1U] !=
          static_cast<float>(expected_generation) ||
      wire.data[3U + 3U * count + 2U] !=
          static_cast<float>(static_cast<int>(output.solver_horizon_intent))) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (wire.data[3U + i] != static_cast<float>(output.lateral_offsets[i]) ||
        wire.data[3U + count + i] != static_cast<float>(output.speed_caps[i]) ||
        wire.data[3U + 2U * count + i] !=
            static_cast<float>(output.longitudinal_offsets_m[i])) {
      return false;
    }
  }
  return true;
}

// 入力: 初回prepared PASSの最終出力、constraint、同じ周期にPPへpublishする
// override wireとgeneration。
// 出力: 初回warm-up条件と完全なv4 payloadが同時に一致する時だけtrue。
// 処理概要: planだけをPASS_WARMUPへ昇格せず、PPが受け取るd/v/ds/mode/intentと
// generationをNodeのtyped authority境界でexactに束縛する。
inline bool isInitialPreparedPassWarmupWireAuthorized(
    const PlannerOutput &output, bool constraint_valid,
    bool constraint_stop_requested, bool constraint_release_authorized,
    const std::string &constraint_reason,
    const ReferenceOverrideWirePayload &wire, std::uint32_t expected_generation,
    std::uint64_t attempt_id) {
  return attempt_id != 0U &&
         isInitialPreparedPassWarmupLateralTrackingAuthorized(
             output, constraint_valid, constraint_stop_requested,
             constraint_release_authorized, constraint_reason) &&
         spatialLateralWireMatchesOutput(output, wire, expected_generation);
}

// 入力: STOP/HOLDのPlanner出力、同じ周期にPPへpublishするwireとgeneration。
// 出力: SafetyEvaluator済みcurrent-d横列が同じv4
// payloadへ完全に載る時だけtrue。 処理概要:
// STOP固有の安全条件は維持したまま、共有するv4完全照合を使う。
inline bool trackingStopLateralWireMatchesAuthorization(
    const PlannerOutput &output, const ReferenceOverrideWirePayload &wire,
    std::uint32_t expected_generation) {
  return isTrackingStopBootstrapOutput(output) &&
         output.lateral_tracking_authorized_during_stop &&
         spatialLateralWireMatchesOutput(output, wire, expected_generation);
}

// 入力: Core最終出力、実際にpublishするoverride wire、generation、attempt。
// 出力: ControllerTrackingStatusを次周期へ運べる型付きtrajectory identity。
// 処理概要: PASSとATTACK_FOLLOWを別kindで保存する。ATTACK_FOLLOWは同じ
// committed transactionのSafetyEvaluator済み・追従可能・非STOPの横wireだけを
// 対象にし、generic FOLLOW/current-d hold/SAFE_STOPを追越証明へ格上げしない。
inline OverrideTrackingIdentity makeOverrideTrackingIdentity(
    const PlannerOutput &output, const ReferenceOverrideWirePayload &wire,
    std::uint32_t generation, std::uint64_t attempt_id) {
  OverrideTrackingIdentity identity;
  identity.generation = generation;
  if (generation == 0U || attempt_id == 0U) {
    return identity;
  }

  const auto pass_type = output.blocked_info.maneuver_transaction_pass_type;
  const bool pass_side_valid = pass_type == CandidateType::PASS_LEFT ||
                               pass_type == CandidateType::PASS_RIGHT;
  const bool lateral_wire_published =
      wire.kind == ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3 ||
      wire.kind == ReferenceOverrideWireKind::SPATIAL_LATERAL_AND_SPEED_V4;
  const bool committed_pass =
      output.blocked_info.maneuver_transaction_incomplete &&
      output.maneuver_latch_active &&
      !output.maneuver_latch_target_id.empty() && pass_side_valid &&
      output.selected == pass_type && output.active_override &&
      output.solver_horizon_intent != PlannerOutput::SolverHorizonIntent::NONE;
  if (committed_pass) {
    identity.attempt_id = attempt_id;
    identity.kind = OverrideTrackingIdentityKind::COMMITTED_PASS;
    identity.target_id = output.maneuver_latch_target_id;
    identity.pass_type = pass_type;
    return identity;
  }

  const bool attack_follow_transaction_identity_valid =
      output.blocked_info.maneuver_transaction_retry_active &&
      output.blocked_info.maneuver_transaction_incomplete &&
      output.blocked_info.maneuver_transaction_tracking_continuity_armed &&
      output.blocked_info.maneuver_target_latched &&
      output.maneuver_latch_active &&
      !output.blocked_info.maneuver_target_id.empty() &&
      output.maneuver_latch_target_id ==
          output.blocked_info.maneuver_target_id &&
      pass_side_valid;
  if (!attack_follow_transaction_identity_valid || !lateral_wire_published ||
      !output.active_override ||
      !output.selected_lateral_profile_safety_verified ||
      output.published_lateral_safety_rejected ||
      output.blocked_info.maneuver_transaction_tracking_stop_active) {
    return identity;
  }

  const bool attack_follow =
      output.mode == BehaviorMode::FOLLOW_BLOCKED &&
      output.selected == CandidateType::FOLLOW &&
      output.raw_selected == CandidateType::FOLLOW &&
      output.blocked_info.attack_follow_hold_pass_side &&
      output.blocked_info.attack_follow_candidate_generated &&
      output.blocked_info.attack_follow_candidate_feasible &&
      output.blocked_info.attack_follow_candidate_tracking_profile_valid &&
      (!output.blocked_info.attack_follow_candidate_safe_lateral_hold ||
       (output.blocked_info
            .attack_follow_candidate_opponent_collision_current_d_hold !=
        output.blocked_info
            .attack_follow_candidate_opponent_collision_inward_connector)) &&
      !output.tracking_release_pass_warmup &&
      output.tracking_release_token == 0U;
  if (!attack_follow) {
    return identity;
  }

  identity.attempt_id = attempt_id;
  identity.kind = OverrideTrackingIdentityKind::ATTACK_FOLLOW;
  identity.target_id = output.maneuver_latch_target_id;
  identity.pass_type = pass_type;
  return identity;
}

// 入力: ATTACK_FOLLOWのCore出力。
// 出力: opponent-collision inward
// connectorとして扱う厳密な出力カテゴリだけtrue。 処理概要:
// identity/wire/trajectory authorityとは独立に、current-d HOLDや generic
// FOLLOWをconnectorカテゴリへ混ぜない。
inline bool isAttackFollowInwardConnectorOutput(const PlannerOutput &output) {
  const auto &blocked = output.blocked_info;
  return output.mode == BehaviorMode::FOLLOW_BLOCKED &&
         output.selected == CandidateType::FOLLOW &&
         output.raw_selected == CandidateType::FOLLOW &&
         output.active_override &&
         output.selected_lateral_profile_safety_verified &&
         !output.published_lateral_safety_rejected &&
         !output.tracking_release_pass_warmup &&
         output.tracking_release_token == 0U &&
         blocked.maneuver_transaction_retry_active &&
         blocked.maneuver_transaction_incomplete &&
         blocked.maneuver_transaction_tracking_continuity_armed &&
         !blocked.maneuver_transaction_tracking_stop_active &&
         blocked.maneuver_target_latched && output.maneuver_latch_active &&
         blocked.maneuver_target_observed && blocked.maneuver_target_fresh &&
         blocked.opponent_prediction_inputs_complete &&
         blocked.attack_follow_hold_pass_side &&
         blocked.attack_follow_candidate_generated &&
         blocked.attack_follow_candidate_feasible &&
         blocked.attack_follow_candidate_safe_lateral_hold &&
         blocked.attack_follow_candidate_opponent_collision_inward_connector &&
         !blocked.attack_follow_candidate_opponent_collision_current_d_hold &&
         !blocked.attack_follow_current_d_hold_variant_used &&
         blocked.attack_follow_inward_connector_variant_generated &&
         blocked.attack_follow_inward_connector_variant_feasible &&
         blocked.attack_follow_inward_connector_variant_used &&
         blocked.attack_follow_candidate_tracking_profile_valid;
}

// 入力: legacy横認可とinward connectorカテゴリ・完全authority照合結果。
// 出力: connectorカテゴリだけは完全照合結果、それ以外は従来の横認可。
// 処理概要: 将来のsolver intentがconnectorのidentity/wire照合を迂回しないよう、
// connectorだけをfail-closedで置換する。
inline bool
composeLateralManeuverRequired(bool legacy_lateral_maneuver_required,
                               bool inward_connector_category,
                               bool inward_connector_lateral_authorized) {
  return inward_connector_category ? inward_connector_lateral_authorized
                                   : legacy_lateral_maneuver_required;
}

// 入力: この周期にpublishしたATTACK_FOLLOW identity/wireと、最終typed plan
// trajectoryの認可結果。
// 出力: opponent-collision inward connectorだけが、exact current generationの
// typed lateral maneuverとしてMux continuityへ提示できる時だけtrue。
// 処理概要: previous identityや過去wireを参照せず、同一target/side・fresh入力・
// SafetyEvaluator済みconnector・完全一致v4 wire・非constant-d profileをすべて
// 確認する。どれか一つでも欠ければ通常STOP側へ閉じる。
inline bool isPublishedAttackFollowInwardConnectorLateralRequired(
    const PlannerOutput &output, const EgoState &ego,
    const OverrideTrackingIdentity &current_identity,
    const ReferenceOverrideWirePayload &current_wire,
    std::uint32_t current_generation, std::uint64_t current_attempt_id,
    const std::string &authoritative_target_id, std::int8_t pass_direction,
    bool final_trajectory_authorized) {
  constexpr std::uint32_t kMaxGeneration = 16777215U;
  constexpr double kLateralDifferenceToleranceM = 1.0e-6;
  const auto &blocked = output.blocked_info;
  const auto pass_type = blocked.maneuver_transaction_pass_type;
  const bool pass_type_valid = pass_type == CandidateType::PASS_LEFT ||
                               pass_type == CandidateType::PASS_RIGHT;
  const std::int8_t expected_pass_direction =
      pass_type == CandidateType::PASS_LEFT ? 1 : -1;
  const bool exact_current_identity =
      current_identity.kind == OverrideTrackingIdentityKind::ATTACK_FOLLOW &&
      current_generation != 0U && current_generation <= kMaxGeneration &&
      current_attempt_id != 0U &&
      current_identity.generation == current_generation &&
      current_identity.attempt_id == current_attempt_id;
  const bool target_and_side_match =
      pass_type_valid && !authoritative_target_id.empty() &&
      current_identity.target_id == authoritative_target_id &&
      current_identity.target_id == output.maneuver_latch_target_id &&
      current_identity.target_id == blocked.maneuver_target_id &&
      current_identity.pass_type == pass_type &&
      pass_direction == expected_pass_direction;
  const std::size_t point_count = output.lateral_offsets.size();
  const bool nonconstant_finite_lateral_profile =
      ego.valid && std::isfinite(ego.frenet.d) && point_count > 1U &&
      output.speed_caps.size() == point_count &&
      output.longitudinal_offsets_m.size() == point_count &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [](double d_m) { return std::isfinite(d_m); }) &&
      std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                  [](double speed_mps) {
                    return std::isfinite(speed_mps) && speed_mps > 0.0;
                  }) &&
      std::all_of(
          output.longitudinal_offsets_m.begin(),
          output.longitudinal_offsets_m.end(),
          [](double distance_m) { return std::isfinite(distance_m); }) &&
      std::any_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [&ego](double d_m) {
                    return std::abs(d_m - ego.frenet.d) >
                           kLateralDifferenceToleranceM;
                  });
  const bool exact_inward_connector =
      isAttackFollowInwardConnectorOutput(output);
  return final_trajectory_authorized && exact_current_identity &&
         target_and_side_match &&
         spatialLateralWireMatchesOutput(output, current_wire,
                                         current_generation) &&
         nonconstant_finite_lateral_profile && exact_inward_connector;
}

} // namespace overtake_planner

#endif // OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_
