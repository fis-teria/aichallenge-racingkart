#include "overtake_transport_contract/c002ay0_shadow_binding.hpp"

#include <cstdint>

namespace overtake_transport_contract::c002ay0 {
namespace {

using Authorized =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using Base =
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;

std::int64_t timeNs(const builtin_interfaces::msg::Time &value) {
  return static_cast<std::int64_t>(value.sec) * 1000000000LL +
         static_cast<std::int64_t>(value.nanosec);
}

bool validTime(const builtin_interfaces::msg::Time &value) {
  return value.sec >= 0 && value.nanosec < 1000000000U;
}

bool sameTime(const builtin_interfaces::msg::Time &lhs,
              const builtin_interfaces::msg::Time &rhs) {
  return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

bool sourceIdentityMatches(const Base &base, const Authorized &proposal) {
  return base.frame_id == proposal.frame_id &&
         base.race_arm_epoch == proposal.plan_sample_key.race_arm_epoch &&
         base.controller_instance_id == proposal.source_controller_instance_id &&
         base.base_source_kind == proposal.base_source_kind &&
         sameTime(base.base_source_stamp, proposal.base_source_stamp) &&
         base.base_source_generation == proposal.base_source_generation &&
         base.base_original_point_count ==
             proposal.base_original_point_count &&
         base.base_source_digest_state == proposal.base_source_digest_state &&
         base.canonical_algorithm_version ==
             proposal.canonical_algorithm_version &&
         base.base_source_sha256 == proposal.base_source_sha256 &&
         base.controller_implementation_sha256 ==
             proposal.controller_implementation_sha256 &&
         base.controller_config_sha256 == proposal.controller_config_sha256;
}

bool spatialTupleMatches(const Base &base, const Authorized &proposal) {
  return sourceIdentityMatches(base, proposal) &&
         base.first_source_index == proposal.base_first_source_index &&
         base.last_source_index == proposal.base_last_source_index &&
         base.nearest_source_index == proposal.base_nearest_source_index &&
         base.base_geometry_sha256 == proposal.base_geometry_sha256;
}

bool sourceStampAndGenerationMatch(const Base &base,
                                   const Authorized &proposal) {
  return base.frame_id == proposal.frame_id &&
         base.race_arm_epoch == proposal.plan_sample_key.race_arm_epoch &&
         base.controller_instance_id == proposal.source_controller_instance_id &&
         base.base_source_kind == proposal.base_source_kind &&
         sameTime(base.base_source_stamp, proposal.base_source_stamp) &&
         base.base_source_generation == proposal.base_source_generation;
}

ShadowBindingResult result(
    ShadowBindingDisposition disposition,
    ValidationError validation_error = ValidationError::NONE) {
  ShadowBindingResult value;
  value.disposition = disposition;
  value.validation_error = validation_error;
  value.lateral_authority_eligible = false;
  return value;
}

} // namespace

ShadowBindingResult evaluateShadowBinding(
    const Base &current, const std::optional<Base> &previous,
    const Authorized &proposal, const builtin_interfaces::msg::Time &now) {
  const auto current_error = validateBaseSnapshotV1(current);
  if (current_error != ValidationError::NONE) {
    return result(ShadowBindingDisposition::kRejectedInvalidCurrent,
                  current_error);
  }
  const auto proposal_error = validateAuthorizedTrajectoryV1(proposal);
  if (proposal_error != ValidationError::NONE) {
    return result(ShadowBindingDisposition::kRejectedInvalidProposal,
                  proposal_error);
  }
  if (!validTime(now) || timeNs(now) >= timeNs(current.lease_valid_until) ||
      timeNs(now) >= timeNs(proposal.base_lease_valid_until) ||
      timeNs(now) >= timeNs(proposal.safety_valid_until) ||
      timeNs(proposal.plan_stamp) > timeNs(now)) {
    return result(ShadowBindingDisposition::kRejectedStale);
  }
  if (spatialTupleMatches(current, proposal)) {
    return result(ShadowBindingDisposition::kExactCurrent);
  }
  if (previous.has_value() &&
      validateBaseSnapshotV1(previous.value()) == ValidationError::NONE &&
      spatialTupleMatches(previous.value(), proposal)) {
    if (timeNs(now) >= timeNs(previous->lease_valid_until)) {
      return result(ShadowBindingDisposition::kRejectedStale);
    }
    return result(ShadowBindingDisposition::kDeferredPredecessor);
  }
  if (sourceStampAndGenerationMatch(current, proposal)) {
    if (current.base_source_sha256 != proposal.base_source_sha256 ||
        current.controller_implementation_sha256 !=
            proposal.controller_implementation_sha256 ||
        current.controller_config_sha256 !=
            proposal.controller_config_sha256) {
      return result(ShadowBindingDisposition::kRejectedSourceMutation);
    }
    if (current.nearest_source_index > proposal.base_nearest_source_index) {
      return result(
          ShadowBindingDisposition::kReplanRequiredNearestAdvanced);
    }
    if (current.nearest_source_index < proposal.base_nearest_source_index) {
      return result(
          ShadowBindingDisposition::kReplanRequiredNearestRegressed);
    }
    return result(ShadowBindingDisposition::kRejectedSourceMutation);
  }
  if (current.frame_id == proposal.frame_id &&
      current.race_arm_epoch == proposal.plan_sample_key.race_arm_epoch &&
      current.controller_instance_id ==
          proposal.source_controller_instance_id) {
    return result(ShadowBindingDisposition::kDeferredUnpairedDelivery);
  }
  return result(ShadowBindingDisposition::kRejectedIdentity);
}

const char *toString(ShadowBindingDisposition disposition) {
  switch (disposition) {
  case ShadowBindingDisposition::kExactCurrent:
    return "exact_current";
  case ShadowBindingDisposition::kDeferredPredecessor:
    return "deferred_predecessor";
  case ShadowBindingDisposition::kDeferredUnpairedDelivery:
    return "deferred_unpaired_delivery";
  case ShadowBindingDisposition::kReplanRequiredNearestAdvanced:
    return "replan_required_nearest_advanced";
  case ShadowBindingDisposition::kReplanRequiredNearestRegressed:
    return "replan_required_nearest_regressed";
  case ShadowBindingDisposition::kRejectedSourceMutation:
    return "rejected_source_mutation";
  case ShadowBindingDisposition::kRejectedIdentity:
    return "rejected_identity";
  case ShadowBindingDisposition::kRejectedStale:
    return "rejected_stale";
  case ShadowBindingDisposition::kRejectedInvalidCurrent:
    return "rejected_invalid_current";
  case ShadowBindingDisposition::kRejectedInvalidProposal:
    return "rejected_invalid_proposal";
  }
  return "rejected_unknown";
}

} // namespace overtake_transport_contract::c002ay0
