#include "overtake_planner/overtake_supervisor_v2.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

bool sameScalar(double lhs, double rhs) {
  return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
}

bool sameVector(const std::vector<double> &lhs,
                const std::vector<double> &rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin(), sameScalar);
}

bool sameTrajectory(const CandidateTrajectory &lhs,
                    const CandidateTrajectory &rhs) {
  return lhs.type == rhs.type && sameVector(lhs.t, rhs.t) &&
         sameVector(lhs.longitudinal_offsets_m, rhs.longitudinal_offsets_m) &&
         sameVector(lhs.s, rhs.s) && sameVector(lhs.d, rhs.d) &&
         sameVector(lhs.x, rhs.x) && sameVector(lhs.y, rhs.y) &&
         sameVector(lhs.yaw, rhs.yaw) &&
         sameScalar(lhs.longitudinal_initial_measured_speed_mps,
                    rhs.longitudinal_initial_measured_speed_mps) &&
         sameVector(lhs.predicted_speed_mps, rhs.predicted_speed_mps) &&
         sameVector(lhs.v_ref, rhs.v_ref) &&
         lhs.safety_evaluated == rhs.safety_evaluated &&
         lhs.feasible == rhs.feasible &&
         lhs.pass_target_corridor_valid == rhs.pass_target_corridor_valid &&
         lhs.controller_tracking_profile_valid ==
             rhs.controller_tracking_profile_valid &&
         lhs.desired_path_trackable == rhs.desired_path_trackable &&
         lhs.pure_pursuit_command_trackable ==
             rhs.pure_pursuit_command_trackable &&
         lhs.moving_target_relatively_reachable ==
             rhs.moving_target_relatively_reachable &&
         sameScalar(lhs.planned_target_d_m, rhs.planned_target_d_m) &&
         sameScalar(lhs.committed_attack_follow_target_d_m,
                    rhs.committed_attack_follow_target_d_m) &&
         lhs.attack_follow_safe_lateral_hold ==
             rhs.attack_follow_safe_lateral_hold &&
         lhs.attack_follow_opponent_collision_current_d_hold ==
             rhs.attack_follow_opponent_collision_current_d_hold &&
         lhs.attack_follow_opponent_collision_inward_connector ==
             rhs.attack_follow_opponent_collision_inward_connector &&
         sameScalar(lhs.required_controller_spatial_horizon_m,
                    rhs.required_controller_spatial_horizon_m) &&
         lhs.controller_spatial_horizon_proof_valid ==
             rhs.controller_spatial_horizon_proof_valid &&
         sameScalar(lhs.score, rhs.score) &&
         sameScalar(lhs.min_safety_margin, rhs.min_safety_margin) &&
         sameScalar(lhs.cbf_slack, rhs.cbf_slack) &&
         lhs.active_safety_constraint_count ==
             rhs.active_safety_constraint_count &&
         lhs.longitudinal_profile_valid == rhs.longitudinal_profile_valid &&
         sameScalar(lhs.assumed_brake_decel_mps2,
                    rhs.assumed_brake_decel_mps2) &&
         sameScalar(lhs.response_delay_sec, rhs.response_delay_sec) &&
         sameScalar(lhs.required_brake_distance_m,
                    rhs.required_brake_distance_m) &&
         sameScalar(lhs.available_brake_distance_m,
                    rhs.available_brake_distance_m) &&
         lhs.reject_reason == rhs.reject_reason;
}

bool sameDecision(const SupervisorV2Decision &lhs,
                  const SupervisorV2Decision &rhs) {
  return lhs.phase == rhs.phase && lhs.attempt_id == rhs.attempt_id &&
         lhs.target_vehicle_id == rhs.target_vehicle_id &&
         lhs.pass_direction == rhs.pass_direction &&
         lhs.trajectory_authorized == rhs.trajectory_authorized &&
         lhs.safety_inputs_complete == rhs.safety_inputs_complete &&
         lhs.tracking_usable == rhs.tracking_usable &&
         lhs.lateral_maneuver_required == rhs.lateral_maneuver_required &&
         lhs.authorization_failure_mask == rhs.authorization_failure_mask &&
         lhs.candidate_reject_reason == rhs.candidate_reject_reason &&
         lhs.candidate_set_limited_by_legacy ==
             rhs.candidate_set_limited_by_legacy &&
         lhs.selected == rhs.selected && lhs.reason == rhs.reason &&
         sameTrajectory(lhs.trajectory, rhs.trajectory);
}

bool candidateSafetyApproved(const CandidateTrajectory *candidate) {
  return candidate != nullptr && candidate->safety_evaluated &&
         candidate->feasible;
}

bool localizedPassProfileValid(const LocalizedLateralProfile *profile,
                               CandidateType expected_type,
                               const std::string &expected_target_id) {
  if (profile == nullptr || !profile->active ||
      profile->pass_type != expected_type ||
      profile->target_id != expected_target_id || profile->target_id.empty() ||
      profile->pass_complete_confirmed ||
      !std::isfinite(profile->created_time_sec) ||
      !std::isfinite(profile->anchor_s_m) ||
      !std::isfinite(profile->target_s_m) ||
      !std::isfinite(profile->avoid_start_s_m) ||
      !std::isfinite(profile->full_offset_start_s_m) ||
      !std::isfinite(profile->full_offset_end_s_m) ||
      !std::isfinite(profile->merge_end_s_m) ||
      !std::isfinite(profile->start_d_m) ||
      !std::isfinite(profile->target_d_m) ||
      profile->avoid_start_s_m > profile->full_offset_start_s_m ||
      profile->full_offset_start_s_m > profile->target_s_m ||
      profile->target_s_m > profile->full_offset_end_s_m ||
      profile->full_offset_end_s_m > profile->merge_end_s_m ||
      (expected_type == CandidateType::PASS_LEFT &&
       profile->target_d_m + 1.0e-6 < profile->start_d_m) ||
      (expected_type == CandidateType::PASS_RIGHT &&
       profile->target_d_m - 1.0e-6 > profile->start_d_m) ||
      profile->chain_waypoints.empty()) {
    return false;
  }
  const auto &first = profile->chain_waypoints.front();
  if (first.target_id != expected_target_id ||
      !std::isfinite(first.target_s_m) || !std::isfinite(first.target_d_m)) {
    return false;
  }
  double previous_s_m = profile->target_s_m;
  double previous_d_m = first.target_d_m;
  for (const auto &waypoint : profile->chain_waypoints) {
    if (waypoint.target_id.empty() || !std::isfinite(waypoint.target_s_m) ||
        !std::isfinite(waypoint.target_d_m) ||
        waypoint.target_s_m + 1.0e-6 < previous_s_m ||
        (expected_type == CandidateType::PASS_LEFT &&
         waypoint.target_d_m + 1.0e-6 < previous_d_m) ||
        (expected_type == CandidateType::PASS_RIGHT &&
         waypoint.target_d_m - 1.0e-6 > previous_d_m)) {
      return false;
    }
    previous_s_m = waypoint.target_s_m;
    previous_d_m = waypoint.target_d_m;
  }
  return profile->chain_target_count ==
             static_cast<int>(profile->chain_waypoints.size()) &&
         profile->chain_tail_id == profile->chain_waypoints.back().target_id &&
         std::abs(profile->chain_tail_s_m -
                  profile->chain_waypoints.back().target_s_m) <= 1.0e-6;
}

bool passProbeValid(const SupervisorV2PassProbe &probe,
                    CandidateType expected_type,
                    const std::string &expected_target_id) {
  return probe.target_vehicle_id == expected_target_id &&
         candidateSafetyApproved(probe.trajectory) &&
         probe.trajectory->type == expected_type &&
         std::isfinite(probe.trajectory->planned_target_d_m) &&
         localizedPassProfileValid(probe.profile, expected_type,
                                   expected_target_id) &&
         std::abs(probe.trajectory->planned_target_d_m -
                  probe.profile->target_d_m) <= 1.0e-6;
}

int passDirection(CandidateType type) {
  if (type == CandidateType::PASS_LEFT) {
    return 1;
  }
  if (type == CandidateType::PASS_RIGHT) {
    return -1;
  }
  return 0;
}

} // namespace

OvertakeSupervisorV2::OvertakeSupervisorV2(int abort_release_cycles,
                                           int abort_centering_safe_cycles,
                                           int pass_completion_cycles,
                                           int target_missing_hold_cycles,
                                           int tracking_unusable_hold_cycles)
    : abort_release_cycles_(std::max(1, abort_release_cycles)),
      abort_centering_safe_cycles_(std::max(1, abort_centering_safe_cycles)),
      pass_completion_cycles_(std::max(1, pass_completion_cycles)),
      target_missing_hold_cycles_(
          std::clamp(target_missing_hold_cycles, 0, 1000)),
      tracking_unusable_hold_cycles_(
          std::clamp(tracking_unusable_hold_cycles, 0, 1000)) {}

bool OvertakeSupervisorV2::applyPassProfileProgress(
    const LocalizedLateralProfile &updated_profile) {
  if (phase_ != TacticalPhase::PASSING || !pass_profile_.has_value() ||
      !localizedPassProfileValid(&updated_profile, pass_type_,
                                 target_vehicle_id_)) {
    return false;
  }
  const auto &current = *pass_profile_;
  if (updated_profile.created_time_sec != current.created_time_sec ||
      updated_profile.anchor_s_m != current.anchor_s_m ||
      updated_profile.start_d_m != current.start_d_m ||
      updated_profile.target_d_m != current.target_d_m ||
      updated_profile.avoid_start_before_target_m !=
          current.avoid_start_before_target_m ||
      updated_profile.full_offset_before_target_m !=
          current.full_offset_before_target_m ||
      updated_profile.pass_safety_approved_once !=
          current.pass_safety_approved_once ||
      updated_profile.pass_execution_committed !=
          current.pass_execution_committed ||
      updated_profile.pass_tracking_continuity_armed !=
          current.pass_tracking_continuity_armed ||
      updated_profile.pass_tracking_proof_published_last_cycle !=
          current.pass_tracking_proof_published_last_cycle ||
      updated_profile.pass_complete_confirmed !=
          current.pass_complete_confirmed ||
      updated_profile.chain_waypoints.size() !=
          current.chain_waypoints.size() ||
      updated_profile.chain_tail_id != current.chain_tail_id ||
      updated_profile.chain_target_count != current.chain_target_count ||
      !std::isfinite(updated_profile.ego_unwrapped_s_m) ||
      !std::isfinite(updated_profile.last_ego_wrapped_s_m) ||
      !std::isfinite(updated_profile.last_ego_stamp_sec) ||
      !std::isfinite(updated_profile.target_s_m) ||
      updated_profile.target_s_m + 1.0e-9 < current.target_s_m ||
      updated_profile.chain_tail_s_m + 1.0e-9 < current.chain_tail_s_m ||
      updated_profile.full_offset_end_s_m + 1.0e-9 <
          current.full_offset_end_s_m ||
      updated_profile.merge_end_s_m + 1.0e-9 < current.merge_end_s_m) {
    return false;
  }
  for (std::size_t i = 0U; i < current.chain_waypoints.size(); ++i) {
    const auto &before = current.chain_waypoints[i];
    const auto &after = updated_profile.chain_waypoints[i];
    if (after.target_id != before.target_id ||
        after.target_d_m != before.target_d_m ||
        !std::isfinite(after.target_s_m) ||
        !std::isfinite(after.observed_unwrapped_s_m) ||
        !std::isfinite(after.last_observed_wrapped_s_m) ||
        !std::isfinite(after.last_observed_stamp_sec) ||
        after.target_s_m + 1.0e-9 < before.target_s_m ||
        after.last_observed_stamp_sec + 1.0e-9 <
            before.last_observed_stamp_sec) {
      return false;
    }
  }
  pass_profile_ = updated_profile;
  return true;
}

SupervisorV2Decision
OvertakeSupervisorV2::update(const SupervisorV2Input &input) {
  // Coreがearly returnした周期はV2評価が呼ばれない。連番が飛んだら、
  // それ以前のclear回数を「連続」として扱わずfail-closedに戻す。
  bool skipped_cycle = false;
  if (input.cycle_sequence != 0U) {
    if (has_last_cycle_sequence_ &&
        input.cycle_sequence != last_cycle_sequence_ + 1U) {
      skipped_cycle = true;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
    }
    last_cycle_sequence_ = input.cycle_sequence;
    has_last_cycle_sequence_ = true;
  }
  const bool complete_and_trackable =
      input.safety_inputs_complete && input.tracking_usable;

  // PASS中にCoreがearly returnした周期は、入力stale中の同側候補を評価できて
  // いない。復帰後にPASSを直再開せず、必ず今周期のHOLD/SAFE_STOPへ閉じる。
  if (skipped_cycle && phase_ == TacticalPhase::PASSING) {
    phase_ = TacticalPhase::ABORT_HOLD;
    pass_complete_pending_ = false;
    pass_completion_clear_cycles_ = 0;
    temporary_hold_cycles_ = 0;
    abort_centering_clear_cycles_ = 0;
    abort_clear_cycles_ = 0;
    abort_centering_active_ = false;
    return makeDecision(input, phase_, bestAbortFallback(input),
                        "passing_input_cycle_skipped");
  }

  if (phase_ == TacticalPhase::PASSING) {
    const CandidateTrajectory *same_side = findCandidate(input, pass_type_);
    const bool target_matches =
        input.target_present && input.target_vehicle_id == target_vehicle_id_;
    const bool completion_confirmation_cycle =
        input.current_target_pass_complete && input.safety_inputs_complete &&
        target_matches;
    pass_completion_clear_cycles_ =
        completion_confirmation_cycle
            ? std::min(pass_completion_cycles_,
                       pass_completion_clear_cycles_ + 1)
            : 0;
    if (pass_completion_clear_cycles_ >= pass_completion_cycles_) {
      // 完了観測と次target分類は同一周期とは限らない。完了を一周期だけで
      // 消さず、次のfresh targetまたは対象なしを確認するまで保持する。
      pass_complete_pending_ = true;
    }
    if (pass_complete_pending_ && input.target_present && !target_matches) {
      // 前targetを抜き切った後も、次targetは同周期にPASSへ暗黙handoffしない。
      // 一度ATTACK_FOLLOWへ戻し、次周期の独立Gate 2で新profileを選び直す。
      phase_ = TacticalPhase::ATTACK_FOLLOW;
      target_vehicle_id_ = input.target_vehicle_id;
      pass_type_ = CandidateType::FASTEST;
      pass_profile_.reset();
      pass_complete_pending_ = false;
      pass_completion_clear_cycles_ = 0;
      temporary_hold_cycles_ = 0;
      return makeDecision(input, phase_,
                          findCandidate(input, CandidateType::FOLLOW),
                          "completed_target_recheck_attack_follow");
    }
    if (pass_complete_pending_ && !input.target_present) {
      // 最終target通過後は即座に基準線へ横切らない。評価済み現在d holdへ
      // 閉じ、ABORT_HOLDのcentering gateから安全に復帰する。
      phase_ = TacticalPhase::ABORT_HOLD;
      target_vehicle_id_.clear();
      pass_type_ = CandidateType::FASTEST;
      pass_profile_.reset();
      pass_complete_pending_ = false;
      pass_completion_clear_cycles_ = 0;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "passing_complete_hold_for_centering");
    }
    if (pass_complete_pending_ && target_matches) {
      // 完了確認の最終周期に、chain先の別車が同側PASSをrejectしても、既に
      // 抜き切ったtargetの完了事実は取り消さない。危険なPASS軌道はpublishせず、
      // SafetyEvaluator済みcurrent-d
      // holdへ閉じたままprofile/side/targetを保持する。
      // 次周期はCoreがclassifierを再び有効化し、別targetなら上のATTACK_FOLLOW
      // handoffを必ず通してから新しいGate 2を評価する。
      const CandidateTrajectory *completion_hold =
          candidateSafetyApproved(same_side) ? same_side
                                             : bestAbortFallback(input);
      return makeDecision(input, TacticalPhase::PASSING, completion_hold,
                          "passing_complete_pending_target_recheck");
    }
    if (!pass_complete_pending_ &&
        input.latched_target_temporarily_unavailable) {
      // stale targetを無視してPASSを続けるのは禁止する。一方で短い欠測だけで
      // transactionを破棄すると、復帰周期に別target/sideへ飛ぶため、boundedな
      // 未認可current-d holdとして同じtarget/profileを保持する。
      temporary_hold_cycles_ =
          std::min(target_missing_hold_cycles_ + 1, temporary_hold_cycles_ + 1);
      pass_completion_clear_cycles_ = 0;
      if (temporary_hold_cycles_ <= target_missing_hold_cycles_) {
        return makeDecision(input, TacticalPhase::PASSING,
                            bestAbortFallback(input),
                            "passing_target_temporarily_unavailable_hold");
      }
      phase_ = TacticalPhase::ABORT_HOLD;
      pass_complete_pending_ = false;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "passing_target_unavailable_timeout");
    }
    // permission/curve start gate is latched at PASS entry. Continuation uses
    // the same-side trajectory's current SafetyEvaluator result, freshness and
    // tracking health; a start-only gate closing must not cause an immediate
    // merge-back.
    if (complete_and_trackable && target_matches &&
        candidateSafetyApproved(same_side)) {
      temporary_hold_cycles_ = 0;
      return makeDecision(input, TacticalPhase::PASSING, same_side,
                          "passing_same_generation_side");
    }
    const bool transient_tracking_unavailable =
        input.safety_inputs_complete && !input.tracking_usable &&
        target_matches && candidateSafetyApproved(same_side) &&
        (input.authorization_failure_mask &
         (SUPERVISOR_V2_AUTH_MPC_HEALTH_STALE |
          SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE)) == 0U;
    if (transient_tracking_unavailable) {
      // controller statusはplan/commandの非同期境界で一周期だけmismatchに
      // なり得る。横移動は未認可current-d holdへ閉じる一方、短い不成立だけで
      // target/side/profile
      // transactionを破棄しない。回復後は同じPASSを再評価する。
      temporary_hold_cycles_ = std::min(tracking_unusable_hold_cycles_ + 1,
                                        temporary_hold_cycles_ + 1);
      // targetの幾何学的完了はfreshなego/V2X/予測で確認済みであり、
      // plan/command境界のtracking mismatchとは独立である。横軌道は未認可
      // current-d holdへ閉じるが、完了確認回数まで消すと次周期にlegacy側の
      // chain handoffと競合して、既に抜いたtargetをABORT扱いしてしまう。
      if (temporary_hold_cycles_ <= tracking_unusable_hold_cycles_) {
        return makeDecision(input, TacticalPhase::PASSING,
                            bestAbortFallback(input),
                            "passing_tracking_temporarily_unavailable_hold");
      }
      phase_ = TacticalPhase::ABORT_HOLD;
      pass_complete_pending_ = false;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "passing_tracking_unavailable_timeout");
    }
    phase_ = TacticalPhase::ABORT_HOLD;
    pass_complete_pending_ = false;
    pass_completion_clear_cycles_ = 0;
    temporary_hold_cycles_ = 0;
    abort_centering_clear_cycles_ = 0;
    abort_clear_cycles_ = 0;
    abort_centering_active_ = false;
    return makeDecision(input, phase_, bestAbortFallback(input),
                        "passing_safety_or_freshness_lost");
  }

  if (phase_ == TacticalPhase::ATTACK_FOLLOW && !target_vehicle_id_.empty()) {
    const CandidateTrajectory *follow =
        findCandidate(input, CandidateType::FOLLOW);
    const bool target_matches =
        input.target_present && input.target_vehicle_id == target_vehicle_id_;
    const bool completion_confirmation_cycle =
        input.current_target_pass_complete && input.safety_inputs_complete &&
        target_matches;
    pass_completion_clear_cycles_ =
        completion_confirmation_cycle
            ? std::min(pass_completion_cycles_,
                       pass_completion_clear_cycles_ + 1)
            : 0;
    if (pass_completion_clear_cycles_ >= pass_completion_cycles_) {
      pass_complete_pending_ = true;
    }
    if (pass_complete_pending_ && input.target_present && !target_matches) {
      // PASSへ入れないまま同じ横位置で対象を抜いた場合も、classifier dropoutで
      // FREE_RUNへ落とさない。前targetの完了を連続確認した後だけ、次targetを
      // 同じATTACK_FOLLOWへ明示handoffし、その後の独立Gate 2へ進める。
      target_vehicle_id_ = input.target_vehicle_id;
      pass_complete_pending_ = false;
      pass_completion_clear_cycles_ = 0;
      temporary_hold_cycles_ = 0;
      return makeDecision(input, phase_, follow,
                          "completed_attack_follow_target_handoff");
    }
    if (pass_complete_pending_ && !input.target_present) {
      // 最終targetを横位置保持のまま抜いた後は、未評価のFREE_RUNへ直結せず、
      // PASS完了時と同じcentering gateへ閉じる。
      phase_ = TacticalPhase::ABORT_HOLD;
      target_vehicle_id_.clear();
      pass_complete_pending_ = false;
      pass_completion_clear_cycles_ = 0;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "attack_follow_complete_hold_for_centering");
    }
    if (pass_complete_pending_ && target_matches) {
      const CandidateTrajectory *completion_hold =
          candidateSafetyApproved(follow) ? follow : bestAbortFallback(input);
      return makeDecision(input, phase_, completion_hold,
                          "attack_follow_complete_pending_target_recheck");
    }
    if (!pass_complete_pending_ &&
        input.latched_target_temporarily_unavailable) {
      temporary_hold_cycles_ =
          std::min(target_missing_hold_cycles_ + 1, temporary_hold_cycles_ + 1);
      pass_completion_clear_cycles_ = 0;
      if (temporary_hold_cycles_ <= target_missing_hold_cycles_) {
        return makeDecision(
            input, phase_, bestAbortFallback(input),
            "attack_follow_target_temporarily_unavailable_hold");
      }
      phase_ = TacticalPhase::ABORT_HOLD;
      pass_complete_pending_ = false;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "attack_follow_target_unavailable_timeout");
    }
    if (!target_matches) {
      // 完了前の別IDへのすり替えは、PASS中と同じくfail-closedにする。
      phase_ = TacticalPhase::ABORT_HOLD;
      pass_complete_pending_ = false;
      pass_completion_clear_cycles_ = 0;
      temporary_hold_cycles_ = 0;
      abort_centering_clear_cycles_ = 0;
      abort_clear_cycles_ = 0;
      abort_centering_active_ = false;
      return makeDecision(input, phase_, bestAbortFallback(input),
                          "attack_follow_target_changed_before_completion");
    }
    temporary_hold_cycles_ = 0;
  }

  if (phase_ == TacticalPhase::ABORT_HOLD) {
    const bool centering_safe_cycle =
        input.abort_centering_safe && complete_and_trackable &&
        candidateSafetyApproved(input.abort_centering_candidate);
    abort_centering_clear_cycles_ =
        centering_safe_cycle ? abort_centering_clear_cycles_ + 1 : 0;
    if (!centering_safe_cycle) {
      abort_centering_active_ = false;
    } else if (abort_centering_clear_cycles_ >= abort_centering_safe_cycles_) {
      abort_centering_active_ = true;
    }

    const CandidateTrajectory *trajectory = nullptr;
    std::string hold_reason = "abort_hold_current_lateral";
    if (abort_centering_active_ && centering_safe_cycle) {
      trajectory = input.abort_centering_candidate;
      hold_reason = input.abort_centered ? "abort_center_release_confirm"
                                         : "abort_hold_centering_recovery";
    } else {
      trajectory = bestAbortFallback(input);
    }

    // CENTERINGの許可と、物理中心到達後の解除確認は別のhysteresisとする。
    // 中心外・stale・候補rejectを一周期でも挟んだら解除回数も破棄する。
    const bool release_cycle = input.abort_release_allowed &&
                               input.abort_centered &&
                               abort_centering_active_ && centering_safe_cycle;
    abort_clear_cycles_ = release_cycle ? abort_clear_cycles_ + 1 : 0;
    if (abort_clear_cycles_ < abort_release_cycles_) {
      return makeDecision(input, phase_, trajectory, hold_reason);
    }
    abort_centering_clear_cycles_ = 0;
    abort_clear_cycles_ = 0;
    abort_centering_active_ = false;
    target_vehicle_id_.clear();
    pass_type_ = CandidateType::FASTEST;
    pass_profile_.reset();
    pass_complete_pending_ = false;
    pass_completion_clear_cycles_ = 0;
    temporary_hold_cycles_ = 0;
    phase_ = input.target_present ? TacticalPhase::ATTACK_FOLLOW
                                  : TacticalPhase::FREE_RUN;
    const auto *released = input.target_present
                               ? findCandidate(input, CandidateType::FOLLOW)
                               : findCandidate(input, CandidateType::FASTEST);
    return makeDecision(input, phase_, released, "abort_hold_released");
  }

  const SupervisorV2PassProbe *pass_probe = bestPassProbe(input);
  const CandidateTrajectory *pass =
      pass_probe == nullptr ? nullptr : pass_probe->trajectory;
  if (input.target_present && complete_and_trackable && pass_probe != nullptr &&
      candidateSafetyApproved(pass)) {
    phase_ = TacticalPhase::PASSING;
    target_vehicle_id_ = input.target_vehicle_id;
    pass_type_ = pass->type;
    pass_profile_ = *pass_probe->profile;
    pass_profile_->pass_safety_approved_once = true;
    pass_profile_->pass_complete_confirmed = false;
    pass_complete_pending_ = false;
    pass_completion_clear_cycles_ = 0;
    temporary_hold_cycles_ = 0;
    ++attempt_id_;
    return makeDecision(input, phase_, pass, "gate2_pass_immediate");
  }

  if (input.target_present) {
    phase_ = TacticalPhase::ATTACK_FOLLOW;
    if (target_vehicle_id_.empty()) {
      target_vehicle_id_ = input.target_vehicle_id;
    }
    return makeDecision(input, phase_,
                        findCandidate(input, CandidateType::FOLLOW),
                        "pass_unavailable_attack_follow");
  }

  phase_ = TacticalPhase::FREE_RUN;
  target_vehicle_id_.clear();
  pass_type_ = CandidateType::FASTEST;
  pass_profile_.reset();
  pass_complete_pending_ = false;
  pass_completion_clear_cycles_ = 0;
  temporary_hold_cycles_ = 0;
  return makeDecision(input, phase_,
                      findCandidate(input, CandidateType::FASTEST),
                      "no_target_free_run");
}

const CandidateTrajectory *
OvertakeSupervisorV2::findCandidate(const SupervisorV2Input &input,
                                    CandidateType type) const {
  if (input.candidates == nullptr) {
    return nullptr;
  }
  const auto it =
      std::find_if(input.candidates->begin(), input.candidates->end(),
                   [type](const CandidateTrajectory &candidate) {
                     return candidate.type == type;
                   });
  return it == input.candidates->end() ? nullptr : &*it;
}

const SupervisorV2PassProbe *
OvertakeSupervisorV2::bestPassProbe(const SupervisorV2Input &input) const {
  const SupervisorV2PassProbe *left = &input.pass_left_probe;
  const SupervisorV2PassProbe *right = &input.pass_right_probe;
  if (!input.pass_left_start_allowed ||
      !passProbeValid(*left, CandidateType::PASS_LEFT,
                      input.target_vehicle_id)) {
    left = nullptr;
  }
  if (!input.pass_right_start_allowed ||
      !passProbeValid(*right, CandidateType::PASS_RIGHT,
                      input.target_vehicle_id)) {
    right = nullptr;
  }
  if (left == nullptr) {
    return right;
  }
  if (right == nullptr) {
    return left;
  }
  return left->trajectory->score <= right->trajectory->score ? left : right;
}

const CandidateTrajectory *
OvertakeSupervisorV2::bestAbortFallback(const SupervisorV2Input &input) const {
  if (candidateSafetyApproved(input.abort_hold_candidate)) {
    return input.abort_hold_candidate;
  }
  // ABORT中に未評価のFOLLOW/YIELDや別RECOVERYへfall throughしない。
  // 現在d保持が不成立なら、明示的に評価済みの停止候補だけを許す。
  if (candidateSafetyApproved(input.abort_stop_candidate)) {
    return input.abort_stop_candidate;
  }
  return nullptr;
}

SupervisorV2Decision OvertakeSupervisorV2::makeDecision(
    const SupervisorV2Input &input, TacticalPhase phase,
    const CandidateTrajectory *candidate, const std::string &reason) {
  SupervisorV2Decision decision;
  decision.phase = phase;
  decision.attempt_id =
      phase == TacticalPhase::PASSING || phase == TacticalPhase::ABORT_HOLD
          ? attempt_id_
          : 0U;
  decision.target_vehicle_id =
      target_vehicle_id_.empty() ? input.target_vehicle_id : target_vehicle_id_;
  decision.pass_direction =
      phase == TacticalPhase::PASSING || phase == TacticalPhase::ABORT_HOLD
          ? passDirection(pass_type_)
          : 0;
  decision.safety_inputs_complete = input.safety_inputs_complete;
  decision.tracking_usable = input.tracking_usable;
  decision.authorization_failure_mask = input.authorization_failure_mask;
  if (candidate == nullptr) {
    decision.authorization_failure_mask |= SUPERVISOR_V2_AUTH_CANDIDATE_MISSING;
    if (input.candidates != nullptr &&
        reason == "pass_unavailable_attack_follow") {
      for (const auto &evaluated_candidate : *input.candidates) {
        if (evaluated_candidate.type != CandidateType::PASS_LEFT &&
            evaluated_candidate.type != CandidateType::PASS_RIGHT) {
          continue;
        }
        if (!evaluated_candidate.safety_evaluated) {
          decision.authorization_failure_mask |=
              SUPERVISOR_V2_AUTH_CANDIDATE_NOT_SAFETY_EVALUATED;
        } else if (!evaluated_candidate.feasible) {
          decision.authorization_failure_mask |=
              SUPERVISOR_V2_AUTH_CANDIDATE_REJECTED;
          if (decision.candidate_reject_reason.empty()) {
            decision.candidate_reject_reason =
                evaluated_candidate.reject_reason;
          }
        }
      }
    }
  } else if (!candidate->safety_evaluated) {
    decision.authorization_failure_mask |=
        SUPERVISOR_V2_AUTH_CANDIDATE_NOT_SAFETY_EVALUATED;
  } else if (!candidate->feasible) {
    decision.authorization_failure_mask |=
        SUPERVISOR_V2_AUTH_CANDIDATE_REJECTED;
  }
  if (!input.safety_inputs_complete) {
    decision.authorization_failure_mask |=
        SUPERVISOR_V2_AUTH_SAFETY_INPUTS_INCOMPLETE;
  }
  if (!input.tracking_usable) {
    const std::uint32_t tracking_or_health_failure =
        SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE |
        SUPERVISOR_V2_AUTH_MPC_HEALTH_STALE |
        SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE;
    if ((decision.authorization_failure_mask & tracking_or_health_failure) ==
        0U) {
      decision.authorization_failure_mask |=
          SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE;
    }
  }
  decision.trajectory_authorized = candidateSafetyApproved(candidate) &&
                                   input.safety_inputs_complete &&
                                   input.tracking_usable;
  decision.lateral_maneuver_required =
      phase == TacticalPhase::PASSING || phase == TacticalPhase::ABORT_HOLD;
  decision.candidate_set_limited_by_legacy =
      input.candidate_set_limited_by_legacy;
  decision.selected =
      candidate == nullptr ? CandidateType::FASTEST : candidate->type;
  if (candidate != nullptr) {
    decision.trajectory = *candidate;
    decision.candidate_reject_reason = candidate->reject_reason;
  }
  decision.reason = reason;
  updateGeneration(decision);
  return decision;
}

void OvertakeSupervisorV2::updateGeneration(SupervisorV2Decision &decision) {
  if (!has_last_decision_ || !sameDecision(decision, last_decision_)) {
    if (generation_exhausted_ ||
        plan_generation_ == std::numeric_limits<std::uint32_t>::max()) {
      generation_exhausted_ = true;
      plan_generation_ = 0U;
    } else {
      ++plan_generation_;
    }
  }
  decision.plan_generation = plan_generation_;
  last_decision_ = decision;
  has_last_decision_ = true;
}

bool supervisorV2TrajectoryPublishable(const SupervisorV2Decision &decision) {
  const auto &trajectory = decision.trajectory;
  const std::size_t point_count = trajectory.x.size();
  if (point_count == 0U || trajectory.y.size() != point_count ||
      trajectory.yaw.size() != point_count ||
      trajectory.v_ref.size() != point_count) {
    return false;
  }
  const auto all_finite = [](const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
  };
  return all_finite(trajectory.x) && all_finite(trajectory.y) &&
         all_finite(trajectory.yaw) &&
         std::all_of(
             trajectory.v_ref.begin(), trajectory.v_ref.end(),
             [](double value) { return std::isfinite(value) && value >= 0.0; });
}

bool supervisorV2EffectivelyAuthorized(const SupervisorV2Decision &decision) {
  return decision.trajectory_authorized &&
         candidateSafetyApproved(&decision.trajectory) &&
         decision.authorization_failure_mask == SUPERVISOR_V2_AUTH_NONE &&
         supervisorV2TrajectoryPublishable(decision);
}

bool supervisorV2RequiresStop(const SupervisorV2Decision &decision) {
  return !supervisorV2EffectivelyAuthorized(decision) ||
         decision.selected == CandidateType::SAFE_STOP;
}

} // namespace overtake_planner
