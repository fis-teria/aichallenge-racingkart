#pragma once

#include "overtake_planner/reference_override_contract.hpp"
#include "overtake_planner/safety_constraint_authority.hpp"

#include <multi_purpose_mpc_ros_msgs/msg/safety_constraint.hpp>

#include <cstdint>
#include <string>

namespace overtake_planner {

// Nodeのpublish世代とtracking identity/token履歴を、副作用なしで遷移させる
// ための値オブジェクト。subscriber側のproof判定は従来どおりNodeが所有する。
struct PlannerPublicationState {
  ReferenceOverrideWirePayload last_override_wire_payload{};
  std::uint32_t override_generation{0U};
  bool has_last_override_identity{false};
  bool last_override_latch_active{false};
  std::string last_override_target_id{};
  std::string last_override_authoritative_target_id{};
  CandidateType last_override_pass_type{CandidateType::FASTEST};
  OverrideTrackingIdentity current_override_tracking_identity{};
  OverrideTrackingIdentity previous_override_tracking_identity{};
  bool has_last_tracking_release_token{false};
  std::uint64_t last_tracking_release_token{0U};
  bool tracking_release_ack_expected{false};
  std::uint64_t tracking_release_expected_token{0U};
  std::uint32_t tracking_release_expected_generation{0U};
  std::string tracking_release_expected_target_id{};
  CandidateType tracking_release_expected_pass_type{CandidateType::FASTEST};
  SafetyConstraintCommand last_safety_constraint_command{};
  std::uint32_t safety_constraint_generation{0U};
  std::uint32_t last_safety_constraint_plan_generation{0U};
};

struct ReferenceOverridePublication {
  PlannerPublicationState next_state{};
  ReferenceOverrideWirePayload wire_payload{};
};

struct SafetyConstraintPublication {
  PlannerPublicationState next_state{};
  multi_purpose_mpc_ros_msgs::msg::SafetyConstraint message{};
};

ReferenceOverridePublication
makeReferenceOverridePublication(const PlannerPublicationState &state,
                                 const PlannerOutput &output,
                                 std::uint64_t attempt_id);

SafetyConstraintPublication makeSafetyConstraintPublication(
    const PlannerPublicationState &state,
    const SafetyConstraintCommand &command,
    const builtin_interfaces::msg::Time &contract_stamp);

} // namespace overtake_planner
