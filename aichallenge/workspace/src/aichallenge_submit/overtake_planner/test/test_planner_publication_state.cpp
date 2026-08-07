#include "overtake_planner/planner_publication_state.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace {

using overtake_planner::BehaviorMode;
using overtake_planner::CandidateType;
using overtake_planner::PlannerOutput;
using overtake_planner::PlannerPublicationState;
using overtake_planner::ReferenceOverrideWirePayload;
using overtake_planner::SafetyConstraintCommand;
using SafetyConstraint = multi_purpose_mpc_ros_msgs::msg::SafetyConstraint;

struct PublicationSnapshot {
  ReferenceOverrideWirePayload override_payload{};
  SafetyConstraint constraint{};
  PlannerPublicationState state{};
};

PlannerOutput passOutput() {
  PlannerOutput output;
  output.mode = BehaviorMode::OVERTAKE_RIGHT;
  output.selected = CandidateType::PASS_RIGHT;
  output.raw_selected = CandidateType::PASS_RIGHT;
  output.active_override = true;
  output.selected_lateral_profile_safety_verified = true;
  output.lateral_stop_inputs_complete = true;
  output.solver_horizon_intent =
      PlannerOutput::SolverHorizonIntent::MANEUVER_AUTHORIZED;
  output.lateral_offsets = {3.2, 3.1, 3.0};
  output.speed_caps = {0.4, 0.5, 0.6};
  output.longitudinal_offsets_m = {0.0, 0.3, 0.7};
  output.blocked_info.maneuver_transaction_incomplete = true;
  output.blocked_info.maneuver_target_latched = true;
  output.blocked_info.maneuver_target_id = "d2";
  output.blocked_info.maneuver_transaction_pass_type =
      CandidateType::PASS_RIGHT;
  output.blocked_info.maneuver_transaction_tracking_release_pending = true;
  output.maneuver_latch_active = true;
  output.maneuver_latch_target_id = "d2";
  output.tracking_release_pass_warmup = true;
  output.tracking_release_token = 41U;
  return output;
}

PlannerOutput stopOutput() {
  auto output = passOutput();
  output.mode = BehaviorMode::FOLLOW_BLOCKED;
  output.selected = CandidateType::SAFE_STOP;
  output.raw_selected = CandidateType::SAFE_STOP;
  output.active_override = false;
  output.selected_lateral_profile_safety_verified = false;
  output.solver_horizon_intent = PlannerOutput::SolverHorizonIntent::NONE;
  output.lateral_offsets.clear();
  output.speed_caps.clear();
  output.longitudinal_offsets_m.clear();
  output.tracking_release_pass_warmup = false;
  output.tracking_release_token = 0U;
  return output;
}

PlannerOutput releaseOutput() {
  PlannerOutput output;
  output.mode = BehaviorMode::FREE_RUN;
  output.selected = CandidateType::FASTEST;
  output.raw_selected = CandidateType::FASTEST;
  return output;
}

SafetyConstraintCommand passConstraint() {
  SafetyConstraintCommand command;
  command.valid = true;
  command.stop_requested = false;
  command.release_authorized = false;
  command.speed_limit_mps = 3.0;
  command.required_brake_decel_mps2 = 0.0;
  command.reason = "pass_warmup";
  return command;
}

SafetyConstraintCommand stopConstraint() {
  SafetyConstraintCommand command;
  command.valid = true;
  command.stop_requested = true;
  command.release_authorized = false;
  command.speed_limit_mps = 0.2;
  command.required_brake_decel_mps2 = 1.5;
  command.reason = "tracking_stop";
  return command;
}

SafetyConstraintCommand releaseConstraint() {
  SafetyConstraintCommand command;
  command.valid = true;
  command.stop_requested = false;
  command.release_authorized = true;
  command.speed_limit_mps = 12.0;
  command.required_brake_decel_mps2 = 0.0;
  command.reason = "normal_limit";
  return command;
}

std::vector<PublicationSnapshot> runSequence(bool shadow_requested) {
  std::array<PlannerOutput, 4> outputs{passOutput(), stopOutput(), stopOutput(),
                                       releaseOutput()};
  for (auto &output : outputs) {
    output.state_lattice_shadow_comparison.requested = shadow_requested;
    output.state_lattice_shadow_comparison.status_reason =
        shadow_requested ? "evaluated" : "disabled";
  }
  const std::array<SafetyConstraintCommand, 4> commands{
      passConstraint(), stopConstraint(), stopConstraint(),
      releaseConstraint()};

  PlannerPublicationState state;
  std::vector<PublicationSnapshot> snapshots;
  for (std::size_t i = 0; i < outputs.size(); ++i) {
    const auto override_publication =
        overtake_planner::makeReferenceOverridePublication(state, outputs[i],
                                                           7U);
    state = override_publication.next_state;

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 100 + static_cast<std::int32_t>(i);
    stamp.nanosec = 123456789U + static_cast<std::uint32_t>(i);
    const auto constraint_publication =
        overtake_planner::makeSafetyConstraintPublication(state, commands[i],
                                                          stamp);
    state = constraint_publication.next_state;
    snapshots.push_back({override_publication.wire_payload,
                         constraint_publication.message, state});
  }
  return snapshots;
}

void expectOverrideEqual(const ReferenceOverrideWirePayload &lhs,
                         const ReferenceOverrideWirePayload &rhs) {
  EXPECT_EQ(lhs.kind, rhs.kind);
  EXPECT_EQ(lhs.mode_id, rhs.mode_id);
  EXPECT_DOUBLE_EQ(lhs.speed_cap_mps, rhs.speed_cap_mps);
  EXPECT_EQ(lhs.data, rhs.data);
}

void expectConstraintEqual(const SafetyConstraint &lhs,
                           const SafetyConstraint &rhs) {
  EXPECT_EQ(lhs.header.stamp.sec, rhs.header.stamp.sec);
  EXPECT_EQ(lhs.header.stamp.nanosec, rhs.header.stamp.nanosec);
  EXPECT_EQ(lhs.header.frame_id, rhs.header.frame_id);
  EXPECT_EQ(lhs.constraint_generation, rhs.constraint_generation);
  EXPECT_EQ(lhs.plan_generation, rhs.plan_generation);
  EXPECT_EQ(lhs.valid, rhs.valid);
  EXPECT_EQ(lhs.stop_requested, rhs.stop_requested);
  EXPECT_EQ(lhs.release_authorized, rhs.release_authorized);
  EXPECT_FLOAT_EQ(lhs.speed_limit_mps, rhs.speed_limit_mps);
  EXPECT_FLOAT_EQ(lhs.required_brake_decel_mps2, rhs.required_brake_decel_mps2);
  EXPECT_EQ(lhs.reason, rhs.reason);
}

TEST(PlannerPublicationState,
     StateLatticeShadowPreservesPassStopRepeatStopReleasePublication) {
  const auto current = runSequence(false);
  const auto shadow = runSequence(true);
  ASSERT_EQ(current.size(), 4U);
  ASSERT_EQ(current.size(), shadow.size());

  for (std::size_t i = 0; i < current.size(); ++i) {
    expectOverrideEqual(current[i].override_payload,
                        shadow[i].override_payload);
    expectConstraintEqual(current[i].constraint, shadow[i].constraint);
  }

  EXPECT_EQ(current[0].constraint.plan_generation, 1U);
  EXPECT_EQ(current[1].constraint.plan_generation, 2U);
  EXPECT_EQ(current[2].constraint.plan_generation, 2U);
  EXPECT_EQ(current[3].constraint.plan_generation, 3U);
  EXPECT_EQ(current[0].constraint.constraint_generation, 1U);
  EXPECT_EQ(current[1].constraint.constraint_generation, 2U);
  EXPECT_EQ(current[2].constraint.constraint_generation, 2U);
  EXPECT_EQ(current[3].constraint.constraint_generation, 3U);

  EXPECT_TRUE(current[0].state.tracking_release_ack_expected);
  EXPECT_EQ(current[0].state.tracking_release_expected_token, 41U);
  EXPECT_EQ(current[0].state.tracking_release_expected_generation, 1U);
  EXPECT_TRUE(current[1].state.tracking_release_ack_expected);
  EXPECT_TRUE(current[2].state.tracking_release_ack_expected);
  EXPECT_FALSE(current[3].state.tracking_release_ack_expected);
  EXPECT_EQ(current[3].state.tracking_release_expected_token, 0U);
  EXPECT_EQ(current[3].state.tracking_release_expected_generation, 0U);
}

TEST(PlannerPublicationState,
     ChangedWarmupTokenAdvancesOnceAndKeepsExactTrackingIdentity) {
  PlannerPublicationState state;
  auto output = passOutput();
  const auto first =
      overtake_planner::makeReferenceOverridePublication(state, output, 7U);
  ASSERT_EQ(first.next_state.override_generation, 1U);
  ASSERT_EQ(first.next_state.current_override_tracking_identity.generation, 1U);

  output.tracking_release_token = 42U;
  const auto changed = overtake_planner::makeReferenceOverridePublication(
      first.next_state, output, 7U);
  EXPECT_EQ(changed.next_state.override_generation, 2U);
  EXPECT_EQ(changed.next_state.tracking_release_expected_token, 42U);
  EXPECT_EQ(changed.next_state.tracking_release_expected_generation, 2U);
  EXPECT_EQ(changed.next_state.previous_override_tracking_identity.generation,
            1U);
  EXPECT_EQ(changed.next_state.current_override_tracking_identity.generation,
            2U);
  EXPECT_EQ(changed.next_state.current_override_tracking_identity.target_id,
            "d2");
  EXPECT_EQ(changed.next_state.current_override_tracking_identity.pass_type,
            CandidateType::PASS_RIGHT);

  const auto repeated = overtake_planner::makeReferenceOverridePublication(
      changed.next_state, output, 7U);
  EXPECT_EQ(repeated.next_state.override_generation, 2U);
  EXPECT_EQ(repeated.next_state.previous_override_tracking_identity.generation,
            1U);
}

TEST(PlannerPublicationState, GenerationWrapRulesRemainFailClosed) {
  PlannerPublicationState state;
  state.override_generation = 16777215U;
  state.has_last_override_identity = true;
  state.last_override_wire_payload =
      overtake_planner::makeReferenceOverrideWirePayload(releaseOutput(),
                                                         16777215U);

  auto changed = passOutput();
  const auto override_publication =
      overtake_planner::makeReferenceOverridePublication(state, changed, 7U);
  EXPECT_EQ(override_publication.next_state.override_generation, 1U);

  state = override_publication.next_state;
  state.safety_constraint_generation =
      std::numeric_limits<std::uint32_t>::max();
  state.last_safety_constraint_plan_generation = 0U;
  builtin_interfaces::msg::Time stamp;
  const auto constraint_publication =
      overtake_planner::makeSafetyConstraintPublication(state, stopConstraint(),
                                                        stamp);
  EXPECT_EQ(constraint_publication.message.constraint_generation, 1U);
}

} // namespace
