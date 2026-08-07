#include "overtake_planner/planner_publication_state.hpp"

#include "overtake_planner/planner_output_builder.hpp"

#include <limits>
#include <utility>

namespace overtake_planner {

ReferenceOverridePublication
makeReferenceOverridePublication(const PlannerPublicationState &state,
                                 const PlannerOutput &output,
                                 std::uint64_t attempt_id) {
  ReferenceOverridePublication publication;
  publication.next_state = state;

  auto wire_payload =
      makeReferenceOverrideWirePayload(output, state.override_generation);
  const CandidateType pass_type =
      output.maneuver_latch_active
          ? output.blocked_info.maneuver_transaction_pass_type
          : CandidateType::FASTEST;
  const std::string authoritative_target_id =
      authoritativeTargetVehicleId(output.blocked_info);
  const bool identity_changed =
      !state.has_last_override_identity ||
      output.maneuver_latch_active != state.last_override_latch_active ||
      output.maneuver_latch_target_id != state.last_override_target_id ||
      authoritative_target_id != state.last_override_authoritative_target_id ||
      pass_type != state.last_override_pass_type;
  const bool wire_or_identity_changed =
      identity_changed || !referenceOverrideWirePayloadSemanticallyEqual(
                              wire_payload, state.last_override_wire_payload);
  const bool tracking_release_warmup_valid =
      output.tracking_release_pass_warmup &&
      output.tracking_release_token != 0U && output.active_override &&
      output.selected == pass_type && output.maneuver_latch_active &&
      !output.maneuver_latch_target_id.empty() &&
      (pass_type == CandidateType::PASS_LEFT ||
       pass_type == CandidateType::PASS_RIGHT);
  const bool payload_changed = referenceOverrideGenerationMustAdvance(
      wire_or_identity_changed, tracking_release_warmup_valid,
      output.tracking_release_token, state.has_last_tracking_release_token,
      state.last_tracking_release_token);
  if (payload_changed) {
    publication.next_state.previous_override_tracking_identity =
        state.current_override_tracking_identity;
    publication.next_state.override_generation =
        state.override_generation >= 16777215U ? 1U
                                               : state.override_generation + 1U;
    wire_payload = makeReferenceOverrideWirePayload(
        output, publication.next_state.override_generation);
  }

  publication.next_state.current_override_tracking_identity =
      makeOverrideTrackingIdentity(output, wire_payload,
                                   publication.next_state.override_generation,
                                   attempt_id);
  publication.next_state.last_override_wire_payload = wire_payload;
  publication.next_state.has_last_override_identity = true;
  publication.next_state.last_override_latch_active =
      output.maneuver_latch_active;
  publication.next_state.last_override_target_id =
      output.maneuver_latch_target_id;
  publication.next_state.last_override_authoritative_target_id =
      authoritative_target_id;
  publication.next_state.last_override_pass_type = pass_type;
  if (tracking_release_warmup_valid) {
    publication.next_state.has_last_tracking_release_token = true;
    publication.next_state.last_tracking_release_token =
        output.tracking_release_token;
    publication.next_state.tracking_release_ack_expected = true;
    publication.next_state.tracking_release_expected_token =
        output.tracking_release_token;
    publication.next_state.tracking_release_expected_generation =
        publication.next_state.override_generation;
    publication.next_state.tracking_release_expected_target_id =
        output.maneuver_latch_target_id;
    publication.next_state.tracking_release_expected_pass_type = pass_type;
  } else if (!output.blocked_info
                  .maneuver_transaction_tracking_release_pending) {
    publication.next_state.tracking_release_ack_expected = false;
    publication.next_state.tracking_release_expected_token = 0U;
    publication.next_state.tracking_release_expected_generation = 0U;
    publication.next_state.tracking_release_expected_target_id.clear();
    publication.next_state.tracking_release_expected_pass_type =
        CandidateType::FASTEST;
  }
  publication.wire_payload = std::move(wire_payload);
  return publication;
}

SafetyConstraintPublication makeSafetyConstraintPublication(
    const PlannerPublicationState &state,
    const SafetyConstraintCommand &command,
    const builtin_interfaces::msg::Time &contract_stamp) {
  SafetyConstraintPublication publication;
  publication.next_state = state;
  if (!safetyConstraintSemanticallyEqual(
          command, state.last_safety_constraint_command) ||
      state.last_safety_constraint_plan_generation !=
          state.override_generation) {
    publication.next_state.safety_constraint_generation =
        state.safety_constraint_generation ==
                std::numeric_limits<std::uint32_t>::max()
            ? 1U
            : state.safety_constraint_generation + 1U;
  }
  publication.next_state.last_safety_constraint_command = command;
  publication.next_state.last_safety_constraint_plan_generation =
      state.override_generation;

  auto &msg = publication.message;
  msg.header.stamp = contract_stamp;
  msg.header.frame_id = "map";
  msg.constraint_generation =
      publication.next_state.safety_constraint_generation;
  msg.plan_generation = state.override_generation;
  msg.valid = command.valid;
  msg.stop_requested = command.stop_requested;
  msg.release_authorized = command.release_authorized;
  msg.speed_limit_mps = static_cast<float>(command.speed_limit_mps);
  msg.required_brake_decel_mps2 =
      static_cast<float>(command.required_brake_decel_mps2);
  msg.reason = command.reason;
  return publication;
}

} // namespace overtake_planner
