#include "overtake_planner/safety_constraint_authority.hpp"

#include <gtest/gtest.h>

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

} // namespace
