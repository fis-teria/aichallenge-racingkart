#include "overtake_planner/overtake_supervisor_v2.hpp"
#include "overtake_planner/reference_override_contract.hpp"
#include "overtake_planner/safety_constraint_authority.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

overtake_planner::ReentryInputStatus completeInputs() {
  overtake_planner::ReentryInputStatus inputs;
  inputs.ego_fresh = true;
  inputs.v2x_snapshot_fresh = true;
  inputs.all_observed_opponents_fresh = true;
  inputs.all_observed_opponents_included = true;
  inputs.reference_valid = true;
  inputs.mpc_health_fresh = true;
  inputs.mpc_healthy = true;
  return inputs;
}

overtake_planner::EgoState validEgo(double speed_mps) {
  overtake_planner::EgoState ego;
  ego.valid = true;
  ego.v = speed_mps;
  return ego;
}

TEST(SafetyConstraintAuthority, UsesEffectiveLowCapAndRequestsBraking) {
  overtake_planner::PlannerOutput output;
  output.longitudinal_speed_cap_active = true;
  output.speed_caps = {0.5, 10.0};
  output.applied_speed_cap_mps = 0.5;
  output.speed_only_fallback_active = true;

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(4.0), completeInputs(), 10.0, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_FALSE(command.stop_requested);
  EXPECT_FALSE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 0.5);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 1.0);
}

TEST(SafetyConstraintAuthority,
     TrackingStopUsesExplicitBootstrapCapInsteadOfZeroInitialProfileSpeed) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  output.selected = overtake_planner::CandidateType::FOLLOW;
  output.raw_selected = overtake_planner::CandidateType::SAFE_STOP;
  output.safe_stop_triggered = false;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.lateral_stop_inputs_complete = true;
  output.lateral_tracking_authorized_during_stop = false;
  output.safe_stop_v_mps = 0.20;
  output.lateral_offsets = {3.35, 3.35, 3.35};
  // v_ref[0]は現在の停止速度への連続接続であり、global speed limitではない。
  output.speed_caps = {9.0e-11, 0.20, 0.40};
  output.longitudinal_offsets_m = {0.0, 0.2, 0.6};
  output.blocked_info.start_grid_target_active = true;
  output.blocked_info.maneuver_transaction_retry_active = true;
  output.blocked_info.maneuver_transaction_incomplete = true;
  output.blocked_info.maneuver_transaction_tracking_stop_active = true;
  output.blocked_info.maneuver_target_latched = true;
  output.blocked_info.maneuver_target_id = "d2";

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(0.0), completeInputs(), 10.0, 1.0);

  ASSERT_TRUE(overtake_planner::isTrackingStopBootstrapOutput(output));
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.stop_requested);
  EXPECT_FALSE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 0.20);
  EXPECT_EQ(command.reason, "maneuver_transaction_tracking_stop");

  output.applied_speed_cap_mps = 0.10;
  const auto tighter = overtake_planner::makeSafetyConstraint(
      output, validEgo(0.0), completeInputs(), 10.0, 1.0);
  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(output, 7U);
  EXPECT_DOUBLE_EQ(tighter.speed_limit_mps, 0.10);
  EXPECT_FLOAT_EQ(wire.data.back(), 0.10F);
}

TEST(ReferenceOverrideContract,
     ReleasePendingCommittedPassKeepsOnlyLateralTrackingAuthorized) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::OVERTAKE_RIGHT;
  output.selected = overtake_planner::CandidateType::PASS_RIGHT;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.published_lateral_safety_rejected = false;
  output.lateral_stop_inputs_complete = true;
  output.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  output.lateral_offsets = {3.20, 3.10, 3.00};
  output.speed_caps = {0.40, 0.50, 0.60};
  output.longitudinal_offsets_m = {0.0, 0.3, 0.7};
  output.required_controller_spatial_horizon_m = 0.7;
  output.controller_spatial_horizon_proof_valid = true;
  output.blocked_info.maneuver_transaction_incomplete = true;
  output.blocked_info.maneuver_target_latched = true;
  output.blocked_info.maneuver_target_id = "d2";
  output.blocked_info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  output.maneuver_latch_active = true;
  output.maneuver_latch_target_id = "d2";

  EXPECT_TRUE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      output, true, true, false, "release_pending_safe_cycles"));

  auto unsafe = output;
  unsafe.controller_spatial_horizon_proof_valid = false;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.longitudinal_offsets_m.back() =
      std::nextafter(output.required_controller_spatial_horizon_m, 0.0);
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.longitudinal_offsets_m[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.speed_caps[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.longitudinal_offsets_m.pop_back();
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.longitudinal_offsets_m[0] = 1.1e-5;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.longitudinal_offsets_m = {0.0, 0.7, 0.3};
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.selected_lateral_profile_safety_verified = false;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.lateral_stop_inputs_complete = false;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.published_lateral_safety_rejected = true;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  unsafe = output;
  unsafe.blocked_info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      unsafe, true, true, false, "release_pending_safe_cycles"));
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      output, true, true, false, "external_safety_stop"));
  EXPECT_FALSE(overtake_planner::isReleasePendingPassLateralTrackingAuthorized(
      output, true, false, true, "normal_limit"));
}

TEST(ReferenceOverrideContract,
     InitialPreparedPassWarmupKeepsOnlyLateralTrackingAuthorized) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::PREPARE_OVERTAKE_RIGHT;
  output.selected = overtake_planner::CandidateType::PASS_RIGHT;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.published_lateral_safety_rejected = false;
  output.lateral_stop_inputs_complete = true;
  output.solver_horizon_intent =
      overtake_planner::PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  output.lateral_offsets = {3.20, 3.10, 3.00};
  // v_ref[0]は停止へ連続接続する正の微小値。v4 wireは0以下を拒否する。
  output.speed_caps = {9.0e-11, 0.20, 0.40};
  output.longitudinal_offsets_m = {0.0, 0.3, 0.7};
  output.required_controller_spatial_horizon_m = 0.7;
  output.controller_spatial_horizon_proof_valid = true;
  output.tracking_release_pass_warmup = true;
  output.tracking_release_token = 2U;
  output.blocked_info.maneuver_transaction_prepared = true;
  output.blocked_info.maneuver_transaction_incomplete = false;
  output.blocked_info.maneuver_transaction_tracking_stop_active = true;
  output.blocked_info.maneuver_transaction_tracking_release_pending = true;
  output.blocked_info.maneuver_target_latched = true;
  output.blocked_info.maneuver_target_id = "d2";
  output.blocked_info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_RIGHT;
  output.maneuver_latch_active = true;
  output.maneuver_latch_target_id = "d2";

  EXPECT_TRUE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          output, true, true, false, "release_pending_safe_cycles"));
  constexpr std::uint32_t kGeneration = 23U;
  const auto wire =
      overtake_planner::makeReferenceOverrideWirePayload(output, kGeneration);
  EXPECT_TRUE(overtake_planner::isInitialPreparedPassWarmupWireAuthorized(
      output, true, true, false, "release_pending_safe_cycles", wire,
      kGeneration, 1U));
  EXPECT_FALSE(overtake_planner::isInitialPreparedPassWarmupWireAuthorized(
      output, true, true, false, "release_pending_safe_cycles", wire,
      kGeneration + 1U, 1U));
  EXPECT_FALSE(overtake_planner::isInitialPreparedPassWarmupWireAuthorized(
      output, true, true, false, "release_pending_safe_cycles", wire,
      kGeneration, 0U));
  auto mismatched_wire = wire;
  ASSERT_GT(mismatched_wire.data.size(), 3U);
  mismatched_wire.data[3] = std::nextafter(
      mismatched_wire.data[3], std::numeric_limits<float>::infinity());
  EXPECT_FALSE(overtake_planner::isInitialPreparedPassWarmupWireAuthorized(
      output, true, true, false, "release_pending_safe_cycles", mismatched_wire,
      kGeneration, 1U));

  auto invalid = output;
  invalid.blocked_info.maneuver_transaction_prepared = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.blocked_info.maneuver_transaction_incomplete = true;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.blocked_info.maneuver_transaction_tracking_stop_active = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.blocked_info.maneuver_transaction_tracking_release_pending = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.tracking_release_pass_warmup = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.tracking_release_token = 0U;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.blocked_info.maneuver_target_id.clear();
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.maneuver_latch_active = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.maneuver_latch_target_id = "d3";
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.blocked_info.maneuver_transaction_pass_type =
      overtake_planner::CandidateType::PASS_LEFT;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.mode = overtake_planner::BehaviorMode::OVERTAKE_RIGHT;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.selected_lateral_profile_safety_verified = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.published_lateral_safety_rejected = true;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.lateral_stop_inputs_complete = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.controller_spatial_horizon_proof_valid = false;
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  invalid = output;
  invalid.longitudinal_offsets_m.back() =
      std::nextafter(output.required_controller_spatial_horizon_m, 0.0);
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          invalid, true, true, false, "release_pending_safe_cycles"));
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          output, true, true, true, "release_pending_safe_cycles"));
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          output, true, false, false, "release_pending_safe_cycles"));
  EXPECT_FALSE(
      overtake_planner::isInitialPreparedPassWarmupLateralTrackingAuthorized(
          output, true, true, false, "external_safety_stop"));
}

TEST(PlannerOutput,
     StopLateralSpatialHorizonContractFailsClosedOnBindingMismatch) {
  overtake_planner::CandidateTrajectory candidate;
  candidate.safety_evaluated = candidate.feasible = true;
  candidate.longitudinal_profile_valid =
      candidate.controller_tracking_profile_valid = true;
  candidate.desired_path_trackable = candidate.pure_pursuit_command_trackable =
      true;
  candidate.controller_spatial_horizon_proof_valid = true;
  candidate.required_controller_spatial_horizon_m = 0.7;
  candidate.d = {0.0, 0.1, 0.2};
  candidate.v_ref = {0.2, 0.2, 0.2};
  candidate.longitudinal_offsets_m = {0.0, 0.3, 0.7};
  overtake_planner::PlannerOutput valid;
  valid.active_override = true;
  valid.controller_spatial_horizon_proof_valid = true;
  valid.lateral_tracking_authorized_during_stop = true;
  valid.lateral_offsets = candidate.d;
  valid.speed_caps = candidate.v_ref;
  valid.longitudinal_offsets_m = candidate.longitudinal_offsets_m;
  valid.required_controller_spatial_horizon_m = 0.7;
  valid.maneuver_latch_target_id = "d2";
  const auto valid_before = valid;
  EXPECT_TRUE(overtake_planner::enforceStopLateralSpatialHorizonContract(
      valid, candidate, true, 0.2, 3U));
  EXPECT_EQ(valid.active_override, valid_before.active_override);
  EXPECT_EQ(valid.lateral_tracking_authorized_during_stop,
            valid_before.lateral_tracking_authorized_during_stop);
  EXPECT_EQ(valid.lateral_offsets, valid_before.lateral_offsets);
  EXPECT_EQ(valid.speed_caps, valid_before.speed_caps);
  EXPECT_EQ(valid.longitudinal_offsets_m, valid_before.longitudinal_offsets_m);
  EXPECT_EQ(valid.required_controller_spatial_horizon_m,
            valid_before.required_controller_spatial_horizon_m);
  EXPECT_EQ(valid.reason, valid_before.reason);
  EXPECT_EQ(valid.maneuver_latch_target_id,
            valid_before.maneuver_latch_target_id);
  for (int kind = 0; kind < 4; ++kind) {
    overtake_planner::PlannerOutput output;
    output.active_override = true;
    output.controller_spatial_horizon_proof_valid = true;
    output.lateral_tracking_authorized_during_stop = true;
    output.lateral_offsets = candidate.d;
    output.speed_caps = candidate.v_ref;
    output.longitudinal_offsets_m = candidate.longitudinal_offsets_m;
    output.required_controller_spatial_horizon_m =
        candidate.required_controller_spatial_horizon_m;
    output.maneuver_latch_target_id = "d2";
    if (kind == 0)
      output.lateral_offsets.back() += 0.01;
    if (kind == 1)
      output.speed_caps.back() += 0.01;
    if (kind == 2)
      output.longitudinal_offsets_m.back() += 0.01;
    if (kind == 3)
      output.required_controller_spatial_horizon_m += 0.01;
    EXPECT_FALSE(overtake_planner::enforceStopLateralSpatialHorizonContract(
        output, candidate, true, 0.2, 3U));
    EXPECT_EQ(output.reason,
              "controller_spatial_horizon_unproven_speed_only_stop");
    EXPECT_FALSE(output.active_override);
    EXPECT_EQ(output.selected, overtake_planner::CandidateType::SAFE_STOP);
    EXPECT_TRUE(output.lateral_offsets.empty());
    EXPECT_TRUE(output.longitudinal_offsets_m.empty());
    EXPECT_FALSE(output.lateral_tracking_authorized_during_stop);
    EXPECT_FALSE(output.controller_spatial_horizon_proof_valid);
    EXPECT_EQ(output.solver_horizon_intent,
              overtake_planner::PlannerOutput::SolverHorizonIntent::NONE);
    EXPECT_TRUE(output.speed_only_fallback_active);
    EXPECT_TRUE(output.longitudinal_speed_cap_active);
    EXPECT_TRUE(output.safe_stop_triggered);
    ASSERT_EQ(output.speed_caps.size(), 3U);
    EXPECT_TRUE(std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                            [](double speed_mps) { return speed_mps <= 0.2; }));
    EXPECT_EQ(output.maneuver_latch_target_id, "d2");
  }
}

TEST(SafetyConstraintAuthority,
     SpatialFollowProfileDoesNotTurnInitialVelocityIntoGlobalSpeedLimit) {
  overtake_planner::PlannerOutput output;
  output.mode = overtake_planner::BehaviorMode::FOLLOW_BLOCKED;
  output.selected = overtake_planner::CandidateType::FOLLOW;
  output.raw_selected = overtake_planner::CandidateType::FOLLOW;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.lateral_offsets = {3.35, 3.35, 3.35};
  output.speed_caps = {9.0e-11, 0.45, 0.91};
  output.longitudinal_offsets_m = {0.0, 0.2, 0.6};

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(0.0), completeInputs(), 10.0, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_FALSE(command.stop_requested);
  EXPECT_TRUE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 10.0);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 0.0);
}

TEST(SafetyConstraintAuthority, IncompleteInputsFailClosed) {
  overtake_planner::PlannerOutput output;
  auto inputs = completeInputs();
  inputs.v2x_snapshot_fresh = false;

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(4.0), inputs, 10.0, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.stop_requested);
  EXPECT_FALSE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 1.0);
  EXPECT_EQ(command.reason, "safety_input_incomplete");
}

TEST(SafetyConstraintAuthority, SafeNormalCycleCanExplicitlyRelease) {
  overtake_planner::PlannerOutput output;

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(4.0), completeInputs(), 10.0, 1.0);

  EXPECT_TRUE(command.valid);
  EXPECT_FALSE(command.stop_requested);
  EXPECT_TRUE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 10.0);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 0.0);
}

TEST(SafetyConstraintAuthority, MpcHealthDoesNotDisableAvailableMuxFallback) {
  overtake_planner::PlannerOutput output;
  output.mpc_health_speed_guard_active = true;
  output.longitudinal_speed_cap_active = true;
  output.speed_caps = {0.5};
  output.applied_speed_cap_mps = 0.5;
  auto inputs = completeInputs();
  inputs.mpc_health_fresh = false;
  inputs.mpc_healthy = false;

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(4.0), inputs, 10.0, 1.0);

  EXPECT_FALSE(command.stop_requested);
  EXPECT_FALSE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 1.0);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 0.5);
}

TEST(SafetyConstraintAuthority, LowCapCanReleaseBrakeWithoutRaisingCap) {
  overtake_planner::PlannerOutput output;
  output.wall_risk_speed_guard_active = true;
  output.longitudinal_speed_cap_active = true;
  output.speed_caps = {0.5};
  output.applied_speed_cap_mps = 0.5;

  const auto command = overtake_planner::makeSafetyConstraint(
      output, validEgo(0.4), completeInputs(), 10.0, 1.0);

  EXPECT_FALSE(command.stop_requested);
  EXPECT_TRUE(command.release_authorized);
  EXPECT_DOUBLE_EQ(command.speed_limit_mps, 0.5);
  EXPECT_DOUBLE_EQ(command.required_brake_decel_mps2, 0.0);
}

TEST(SafetyConstraintAuthority,
     LowCapBrakeReleaseDoesNotPrearmHigherSpeedRelease) {
  overtake_planner::SafetyConstraintReleaseGate gate(3);
  overtake_planner::SafetyConstraintCommand low_braking;
  low_braking.valid = true;
  low_braking.stop_requested = false;
  low_braking.release_authorized = false;
  low_braking.speed_limit_mps = 0.5;
  low_braking.required_brake_decel_mps2 = 1.0;
  low_braking.reason = "wall_risk";
  const auto low_initial = gate.filter(low_braking);
  EXPECT_DOUBLE_EQ(low_initial.speed_limit_mps, 0.5);
  EXPECT_DOUBLE_EQ(low_initial.required_brake_decel_mps2, 1.0);

  auto low_released = low_braking;
  low_released.release_authorized = true;
  low_released.required_brake_decel_mps2 = 0.0;
  EXPECT_FALSE(gate.filter(low_released).release_authorized);
  EXPECT_FALSE(gate.filter(low_released).release_authorized);
  const auto low_release = gate.filter(low_released);
  EXPECT_TRUE(low_release.release_authorized);
  EXPECT_DOUBLE_EQ(low_release.speed_limit_mps, 0.5);
  EXPECT_DOUBLE_EQ(low_release.required_brake_decel_mps2, 0.0);

  auto normal = low_released;
  normal.speed_limit_mps = 10.0;
  normal.reason = "normal_limit";
  const auto first_normal = gate.filter(normal);
  const auto second_normal = gate.filter(normal);
  const auto third_normal = gate.filter(normal);
  EXPECT_FALSE(first_normal.release_authorized);
  EXPECT_DOUBLE_EQ(first_normal.speed_limit_mps, 0.5);
  EXPECT_FALSE(second_normal.release_authorized);
  EXPECT_DOUBLE_EQ(second_normal.speed_limit_mps, 0.5);
  EXPECT_TRUE(third_normal.release_authorized);
  EXPECT_DOUBLE_EQ(third_normal.speed_limit_mps, 10.0);
}

TEST(SafetyConstraintAuthority, ReleaseRequiresConsecutiveSafeCycles) {
  overtake_planner::SafetyConstraintReleaseGate gate(3);
  overtake_planner::SafetyConstraintCommand safe;
  safe.valid = true;
  safe.stop_requested = false;
  safe.release_authorized = true;
  safe.speed_limit_mps = 10.0;
  safe.reason = "normal_limit";

  const auto first = gate.filter(safe);
  const auto second = gate.filter(safe);
  const auto third = gate.filter(safe);

  EXPECT_FALSE(first.release_authorized);
  EXPECT_EQ(first.reason, "release_pending_safe_cycles");
  EXPECT_FALSE(second.release_authorized);
  EXPECT_TRUE(third.release_authorized);
  EXPECT_EQ(third.reason, "normal_limit");
}

TEST(SafetyConstraintAuthority, UnsafeCycleResetsReleaseConfirmation) {
  overtake_planner::SafetyConstraintReleaseGate gate(2);
  overtake_planner::SafetyConstraintCommand safe;
  safe.valid = true;
  safe.stop_requested = false;
  safe.release_authorized = true;
  safe.speed_limit_mps = 10.0;
  safe.reason = "normal_limit";
  auto unsafe = safe;
  unsafe.stop_requested = true;
  unsafe.release_authorized = false;

  EXPECT_FALSE(gate.filter(safe).release_authorized);
  EXPECT_FALSE(gate.filter(unsafe).release_authorized);
  EXPECT_FALSE(gate.filter(safe).release_authorized);
}

TEST(SafetyConstraintAuthority,
     ConservativeReleaseProgressesAcrossDynamicSafeCaps) {
  overtake_planner::SafetyConstraintReleaseGate gate(3, true);
  overtake_planner::SafetyConstraintCommand stop;
  stop.valid = true;
  stop.stop_requested = true;
  stop.release_authorized = false;
  stop.speed_limit_mps = 0.1;
  stop.required_brake_decel_mps2 = 1.0;
  stop.reason = "shadow_stop";
  EXPECT_TRUE(gate.filter(stop).stop_requested);

  auto safe = stop;
  safe.stop_requested = false;
  safe.release_authorized = true;
  safe.required_brake_decel_mps2 = 0.0;
  safe.reason = "shadow_authorized";
  safe.speed_limit_mps = 3.0;
  const auto first = gate.filter(safe);
  safe.speed_limit_mps = 2.8;
  const auto second = gate.filter(safe);
  safe.speed_limit_mps = 3.2;
  const auto third = gate.filter(safe);

  EXPECT_TRUE(first.stop_requested);
  EXPECT_FALSE(first.release_authorized);
  EXPECT_TRUE(second.stop_requested);
  EXPECT_FALSE(second.release_authorized);
  EXPECT_FALSE(third.stop_requested);
  EXPECT_TRUE(third.release_authorized);
  EXPECT_DOUBLE_EQ(third.speed_limit_mps, 2.8);
}

TEST(SafetyConstraintAuthority, ConservativeReleaseResetsAfterUnsafeCycle) {
  overtake_planner::SafetyConstraintReleaseGate gate(2, true);
  overtake_planner::SafetyConstraintCommand stop;
  stop.valid = true;
  stop.stop_requested = true;
  stop.release_authorized = false;
  stop.speed_limit_mps = 0.1;
  stop.required_brake_decel_mps2 = 1.0;
  gate.filter(stop);

  auto safe = stop;
  safe.stop_requested = false;
  safe.release_authorized = true;
  safe.speed_limit_mps = 3.0;
  safe.required_brake_decel_mps2 = 0.0;
  EXPECT_FALSE(gate.filter(safe).release_authorized);
  EXPECT_FALSE(gate.filter(stop).release_authorized);
  EXPECT_FALSE(gate.filter(safe).release_authorized);
}

TEST(SafetyConstraintAuthority,
     ReleasedConservativeConstraintKeepsMotionWhileLooserCapIsConfirmed) {
  overtake_planner::SafetyConstraintReleaseGate gate(3, true);
  overtake_planner::SafetyConstraintCommand stop;
  stop.valid = true;
  stop.stop_requested = true;
  stop.release_authorized = false;
  stop.speed_limit_mps = 0.20;
  stop.required_brake_decel_mps2 = 1.0;
  stop.reason = "maneuver_transaction_tracking_stop";
  EXPECT_TRUE(gate.filter(stop).stop_requested);

  auto follow = stop;
  follow.stop_requested = false;
  follow.release_authorized = true;
  follow.required_brake_decel_mps2 = 0.0;
  follow.reason = "attack_follow";
  follow.speed_limit_mps = 0.87;
  EXPECT_TRUE(gate.filter(follow).stop_requested);
  follow.speed_limit_mps = 0.86;
  EXPECT_TRUE(gate.filter(follow).stop_requested);
  follow.speed_limit_mps = 0.88;
  const auto released = gate.filter(follow);
  ASSERT_TRUE(released.release_authorized);
  ASSERT_FALSE(released.stop_requested);
  ASSERT_DOUBLE_EQ(released.speed_limit_mps, 0.86);

  // A higher safe FOLLOW cap is a relaxation, so keep the already-authorized
  // lower cap while the new target is reconfirmed. Motion must not fall back
  // to STOP merely because the observed target speed changed slightly.
  follow.speed_limit_mps = 0.89;
  const auto reconfirming = gate.filter(follow);
  EXPECT_FALSE(reconfirming.release_authorized);
  EXPECT_FALSE(reconfirming.stop_requested);
  EXPECT_DOUBLE_EQ(reconfirming.speed_limit_mps, 0.86);
  EXPECT_EQ(reconfirming.reason, "release_pending_safe_cycles");
}

TEST(SafetyConstraintAuthority,
     ShadowAuthorizedTrajectoryPairsWithNonStopConstraintAfterConfirmation) {
  overtake_planner::SupervisorV2Decision decision;
  decision.trajectory_authorized = true;
  decision.selected = overtake_planner::CandidateType::FOLLOW;
  decision.trajectory.safety_evaluated = true;
  decision.trajectory.feasible = true;
  decision.trajectory.x = {0.0, 1.0};
  decision.trajectory.y = {0.0, 0.0};
  decision.trajectory.yaw = {0.0, 0.0};
  decision.trajectory.v_ref = {3.0, 3.0};
  ASSERT_TRUE(overtake_planner::supervisorV2EffectivelyAuthorized(decision));

  overtake_planner::PlannerOutput shadow_output;
  shadow_output.active_override = true;
  shadow_output.longitudinal_speed_cap_active = true;
  shadow_output.speed_caps = decision.trajectory.v_ref;
  shadow_output.applied_speed_cap_mps = 3.0;
  shadow_output.reason = "shadow_authorized";
  auto candidate_constraint = overtake_planner::makeSafetyConstraint(
      shadow_output, validEgo(2.0), completeInputs(), 10.0, 1.0);
  ASSERT_FALSE(candidate_constraint.stop_requested);
  ASSERT_TRUE(candidate_constraint.release_authorized);

  overtake_planner::SafetyConstraintReleaseGate gate(3, true);
  const auto first = gate.filter(candidate_constraint);
  const auto second = gate.filter(candidate_constraint);
  const auto third = gate.filter(candidate_constraint);
  EXPECT_FALSE(first.release_authorized);
  EXPECT_TRUE(first.stop_requested);
  EXPECT_FALSE(second.release_authorized);
  EXPECT_TRUE(second.stop_requested);
  EXPECT_TRUE(third.release_authorized);
  EXPECT_FALSE(third.stop_requested);
  EXPECT_TRUE(decision.trajectory_authorized && third.valid &&
              !third.stop_requested);
}

} // namespace
