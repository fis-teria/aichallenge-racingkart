#include "overtake_planner/planner_output_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace overtake_planner {

namespace {

// 入力: 候補値valueと、使えない場合のfallback。
// 出力: valueが有限かつ正ならvalue、それ以外はfallback。
// 処理概要: 速度上限の計算でNaN/0を伝播させないための小さなガード。
double finitePositiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

// 入力: 候補軌道とfallback速度[m/s]。
// 出力: 候補内で最も低い有効速度上限。無ければfallback。
// 処理概要: 不安全候補から速度だけfallbackする時に、元候補の速度意図をできるだけ残す。
double candidateSpeedCapOr(const CandidateTrajectory &candidate,
                           double fallback) {
  double out = fallback;
  for (double cap : candidate.v_ref) {
    if (std::isfinite(cap) && cap > 0.0) {
      out = std::min(out, cap);
    }
  }
  return out;
}

// 入力: 既存の速度列と適用したい一様速度上限。
// 出力: なし。speed_capsを上限以下へ直接更新する。
// 処理概要: 既に横overrideがある候補に、速度ガードだけを重ねる。
void applyUniformSpeedCap(std::vector<double> &speed_caps, double cap) {
  if (!std::isfinite(cap) || cap <= 0.0) {
    return;
  }
  if (speed_caps.empty()) {
    speed_caps.push_back(cap);
    return;
  }
  for (double &existing : speed_caps) {
    existing = std::isfinite(existing) && existing > 0.0
                   ? std::min(existing, cap)
                   : cap;
  }
}

// 入力: 要素数countと値value。
// 出力: 少なくとも1要素を持つ一様配列。
// 処理概要: 速度だけguardする時でもMPC overrideプロトコルに必要な配列長を確保する。
std::vector<double> uniformVector(std::size_t count, double value) {
  return std::vector<double>(std::max<std::size_t>(1, count), value);
}

// 入力: 現在modeとpublish対象候補。
// 出力: 復帰系速度ガードを適用する文脈ならtrue。
// 処理概要: ABORT_RECOVERYまたはRECOVERY候補では、壁/health/lateral errorに応じて速度を落とす。
bool isRecoverySpeedGuardMode(BehaviorMode mode, CandidateType selected) {
  return mode == BehaviorMode::ABORT_RECOVERY ||
         selected == CandidateType::RECOVERY;
}

// 入力: PlannerConfigと自車状態。
// 出力: 安全コリドー内にクランプした現在横位置d。
// 処理概要: 速度だけのguardで横参照を新規生成する時、急に中心線へ飛ばさず現在位置を保持する。
double clampedCurrentLateralOffset(const PlannerConfig &config,
                                   const EgoState &ego) {
  const double lower_d = config.d_min_m + config.min_wall_margin_m;
  const double upper_d = config.d_max_m - config.min_wall_margin_m;
  if (!std::isfinite(ego.frenet.d)) {
    return 0.0;
  }
  return std::clamp(ego.frenet.d, lower_d, upper_d);
}

} // namespace

// 入力: planner設定。
// 出力: PlannerOutput生成器のインスタンス。
// 処理概要: 候補選択結果に速度ガードやsafe stop情報を重ねるための設定を保持する。
PlannerOutputBuilder::PlannerOutputBuilder(const PlannerConfig &config)
    : config_(config) {}

// 入力: mode、選択候補、安全停止文脈、section profile、MPC healthなどの集約入力。
// 出力: ROS nodeがpublishしやすい平坦なPlannerOutput。
// 処理概要: 候補評価結果にsafe stop理由と速度ガードを重ね、最終override配列とdebug指標を作る。
PlannerOutput
PlannerOutputBuilder::build(const PlannerOutputBuildInput &input) const {
  const auto &selected = input.selected;
  const auto &blocked = input.blocked_info;
  const auto &safe_stop_context = input.safe_stop_context;
  const auto &safe_stop_candidate = input.safe_stop_candidate;
  const auto &active_section = input.active_section;
  const auto &mpc_health = input.mpc_health;

  PlannerOutput output;
  output.mode = input.mode;
  output.selected = selected.type;
  output.blocked_info = blocked;
  output.reason = selected.reject_reason;
  output.safe_stop_triggered =
      safe_stop_context.requested || input.mode == BehaviorMode::SAFE_STOP;
  output.safe_stop_release_ready = safe_stop_context.release_ready;
  output.safe_stop_v_mps = std::max(1.0e-3, config_.safe_stop_v_mps);
  output.safe_stop_trigger_count = input.safe_stop_trigger_count;
  output.safe_stop_hold_count = input.safe_stop_hold_count;
  output.safe_stop_release_count = input.safe_stop_release_count;
  // 処理ブロック: safe stopの状態理由を出力へ転記する。
  // 設計意図: plannerが止まった理由をdebug JSONとログで即座に追えるようにする。
  if (input.mode == BehaviorMode::SAFE_STOP) {
    if (!selected.feasible) {
      output.safe_stop_reason = "safe_stop_infeasible";
      output.safe_stop_reject_reason = selected.reject_reason;
    } else if (safe_stop_context.requested &&
               output.safe_stop_hold_count <= 1) {
      output.safe_stop_reason = "no_pass_and_no_safe_fallback";
    } else if (safe_stop_context.release_ready) {
      output.safe_stop_reason = "safe_stop_release_pending";
    } else {
      output.safe_stop_reason = "safe_stop_holding";
    }
  } else if (input.safe_stop_candidate_infeasible) {
    output.safe_stop_triggered = true;
    output.safe_stop_reason = "safe_stop_infeasible";
    output.safe_stop_reject_reason = safe_stop_candidate.reject_reason;
  }
  if (!output.safe_stop_reason.empty()) {
    output.reason = output.safe_stop_reason == "no_pass_and_no_safe_fallback"
                        ? "no_safe_avoidance"
                        : output.safe_stop_reason;
  }

  const double selected_target_d =
      selected.d.empty() ? input.ego.frenet.d : selected.d.back();
  bool selected_override_active = false;
  // 処理ブロック: 選択候補自体が横/速度overrideを持つかを決める。
  // 設計意図: FASTESTは通常走行なのでoverrideしないが、SAFE_STOPだけは停止候補として明示的に出す。
  if (selected.type == CandidateType::SAFE_STOP) {
    selected_override_active = selected.feasible;
  } else {
    selected_override_active = selected.type != CandidateType::FASTEST &&
                               !input.safe_stop_candidate_infeasible &&
                               selected.feasible;
  }

  double requested_speed_cap = std::numeric_limits<double>::infinity();
  std::string speed_cap_reason;
  bool speed_only_fallback = false;
  bool wall_risk_guard = false;
  bool mpc_health_guard = false;
  bool recovery_speed_guard = false;
  const bool allow_speed_guard = true;
  // 処理ブロック: 複数の速度制限要求から最も低い上限だけを採用する。
  // 設計意図: 壁リスク、MPC health、fallbackが同時に出ても、より安全側の速度を1つに集約する。
  const auto requestSpeedCap = [&](double cap_mps, const std::string &reason) {
    if (!std::isfinite(cap_mps) || cap_mps <= 0.0) {
      return;
    }
    if (cap_mps < requested_speed_cap) {
      requested_speed_cap = cap_mps;
      speed_cap_reason = reason;
    }
  };
  const auto speedOnlyFallbackCapForReject =
      [&](const std::string &reject_reason, bool &leader_priority_relaxed) {
        double cap = config_.speed_only_fallback_v_max_mps;
        if (reject_reason == "opponent_collision") {
          if (config_.side_by_side_leader_priority_enabled &&
              blocked.leader_priority_active) {
            leader_priority_relaxed = true;
            cap = finitePositiveOr(config_.side_by_side_leader_priority_v_max_mps,
                                   cap);
          } else {
            cap = std::min(
                cap, finitePositiveOr(
                         config_.opponent_collision_fallback_v_max_mps, cap));
          }
        }
        return cap;
      };

  if (allow_speed_guard && config_.speed_only_fallback_enabled &&
      input.safe_stop_candidate_infeasible) {
    bool leader_priority_relaxed = false;
    requestSpeedCap(
        speedOnlyFallbackCapForReject(safe_stop_candidate.reject_reason,
                                      leader_priority_relaxed),
        leader_priority_relaxed
            ? "speed_only_fallback_leader_priority_safe_stop_infeasible"
            : "speed_only_fallback_safe_stop_infeasible");
    speed_only_fallback = true;
  }

  if (allow_speed_guard && config_.speed_only_fallback_enabled &&
      !input.safe_stop_candidate_infeasible && !selected.feasible &&
      selected.type != CandidateType::FASTEST &&
      selected.type != CandidateType::SAFE_STOP) {
    bool leader_priority_relaxed = false;
    const double fallback_cap = speedOnlyFallbackCapForReject(
        selected.reject_reason, leader_priority_relaxed);
    const double selected_cap = candidateSpeedCapOr(selected, fallback_cap);
    const double requested_fallback_cap =
        leader_priority_relaxed ? fallback_cap
                                : std::min(selected_cap, fallback_cap);
    requestSpeedCap(
        requested_fallback_cap,
        leader_priority_relaxed
            ? "speed_only_fallback_leader_priority"
        : selected.reject_reason.empty()
            ? "speed_only_fallback"
            : "speed_only_fallback_" + selected.reject_reason);
    speed_only_fallback = true;
  }

  const bool wall_risk_speed_guard_condition =
      input.wall_soft_margin_m > 0.0 &&
      blocked.ego_wall_clearance_m < input.wall_soft_margin_m;
  const bool wall_risk_speed_guard_applies =
      config_.wall_risk_speed_guard_enabled && wall_risk_speed_guard_condition;
  if (allow_speed_guard && wall_risk_speed_guard_applies) {
    requestSpeedCap(config_.wall_risk_v_max_mps, "wall_risk_speed_guard");
    wall_risk_guard = true;
  }

  if (allow_speed_guard && active_section.active &&
      active_section.profile == "side_by_side_corner_strict" &&
      (blocked.side_by_side || blocked.parallel_side_candidate ||
       blocked.future_yield_required || blocked.corner_side_by_side ||
       blocked.future_corner_side_by_side)) {
    requestSpeedCap(config_.corner_yield_v_max_mps,
                    "section_profile_speed_guard");
  }

  const bool mpc_health_infeasible_guard =
      config_.mpc_health_infeasible_count_threshold > 0 &&
      mpc_health.infeasible_count >=
          config_.mpc_health_infeasible_count_threshold;
  const bool mpc_health_solve_time_guard =
      config_.mpc_health_solve_time_warn_ms > 0.0 &&
      std::isfinite(mpc_health.solve_time_ms) &&
      mpc_health.solve_time_ms >= config_.mpc_health_solve_time_warn_ms;
  const bool mpc_health_stale_guard =
      config_.mpc_health_stale_time_sec > 0.0 &&
      std::isfinite(mpc_health.age_sec) &&
      mpc_health.age_sec > config_.mpc_health_stale_time_sec;
  const bool mpc_health_speed_guard_condition =
      mpc_health.valid &&
      (mpc_health_infeasible_guard || mpc_health_solve_time_guard ||
       mpc_health_stale_guard);
  const bool mpc_health_speed_guard_applies =
      config_.mpc_health_speed_guard_enabled && mpc_health_speed_guard_condition;
  if (allow_speed_guard && mpc_health_speed_guard_applies) {
    const std::string reason =
        mpc_health_infeasible_guard   ? "mpc_health_infeasible_guard"
        : mpc_health_solve_time_guard ? "mpc_health_solve_time_guard"
                                      : "mpc_health_stale_guard";
    requestSpeedCap(config_.mpc_health_v_max_mps, reason);
    mpc_health_guard = true;
  }

  const bool large_lateral_error =
      config_.large_lateral_error_threshold_m >= 0.0 &&
      std::isfinite(input.ego.frenet.d) &&
      std::abs(input.ego.frenet.d) > config_.large_lateral_error_threshold_m;
  if (allow_speed_guard && config_.recovery_speed_guard_enabled &&
      isRecoverySpeedGuardMode(input.mode, selected.type) &&
      (wall_risk_speed_guard_applies || mpc_health_speed_guard_applies ||
       large_lateral_error)) {
    const std::string reason =
        mpc_health_speed_guard_applies
            ? "recovery_mpc_health_speed_guard"
            : wall_risk_speed_guard_applies
                  ? "recovery_wall_risk_speed_guard"
                  : "recovery_lateral_error_speed_guard";
    requestSpeedCap(config_.recovery_speed_guard_v_max_mps, reason);
    recovery_speed_guard = true;
  }

  if (std::isfinite(requested_speed_cap)) {
    requested_speed_cap = scaledSpeedCap(requested_speed_cap, active_section);
  }

  // 処理ブロック: 選択候補をPlannerOutputへコピーし、必要なら速度ガードを重ねる。
  // 設計意図: 横overrideがある場合は横列を維持し、速度だけのguardでは現在横位置保持の一様列を作る。
  output.active_override = selected_override_active;
  output.target_lateral_offset_m = selected.d.empty() ? 0.0 : selected_target_d;
  output.min_cbf_h = selected.min_safety_margin;
  output.cbf_slack = selected.cbf_slack;
  output.active_cbf_constraint_count = selected.active_safety_constraint_count;
  output.lateral_offsets = selected.d;
  output.speed_caps = selected.v_ref;
  if (std::isfinite(requested_speed_cap)) {
    if (selected_override_active && selected.feasible) {
      applyUniformSpeedCap(output.speed_caps, requested_speed_cap);
    } else {
      output.active_override = true;
      const double hold_d = clampedCurrentLateralOffset(config_, input.ego);
      output.lateral_offsets = uniformVector(config_.horizon_points, hold_d);
      output.speed_caps =
          uniformVector(config_.horizon_points, requested_speed_cap);
      output.target_lateral_offset_m = hold_d;
      if (output.mode == BehaviorMode::FREE_RUN) {
        output.mode = BehaviorMode::SPEED_GUARD;
      }
    }
    output.applied_speed_cap_mps = requested_speed_cap;
    output.speed_cap_reason = speed_cap_reason;
    if (output.reason.empty() && !selected_override_active) {
      output.reason = speed_cap_reason;
    }
  }
  output.speed_only_fallback_active = speed_only_fallback;
  output.wall_risk_speed_guard_active = wall_risk_guard;
  output.mpc_health_speed_guard_active = mpc_health_guard;
  output.recovery_speed_guard_active = recovery_speed_guard;
  output.wall_soft_margin_m = input.wall_soft_margin_m;
  output.active_section = active_section;
  output.mpc_health = mpc_health;
  return output;
}

// 入力: 速度上限[m/s]と有効section safety profile。
// 出力: sectionのscaleを掛けた速度上限[m/s]。
// 処理概要: コース区間ごとの安全プロファイルを、最終速度guardにも反映する。
double
PlannerOutputBuilder::scaledSpeedCap(double speed_cap_mps,
                                     const ActiveSectionSafety &section) const {
  const double scale =
      section.active ? std::clamp(section.speed_cap_scale, 0.05, 1.0) : 1.0;
  return finitePositiveOr(speed_cap_mps * scale, speed_cap_mps);
}

} // namespace overtake_planner
