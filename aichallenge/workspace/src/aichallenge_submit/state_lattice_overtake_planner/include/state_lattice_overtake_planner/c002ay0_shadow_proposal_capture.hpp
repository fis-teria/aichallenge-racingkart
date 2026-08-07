#pragma once

#include "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp"

#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner::c002ay0_shadow {

enum class FixedProposalCaptureResult : std::uint8_t {
  kBuilt,
  kBaseInvalid,
  kCandidateInvalid,
  kOpponentLimit,
  kIdentityInvalid,
};

bool copyFixedBaseRecord(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &base,
    std::uint64_t session_generation, std::uint64_t session_nonce,
    FixedBaseRecord *record, FixedIncomingBaseProvenance *provenance) noexcept;

FixedProposalCaptureResult buildFixedProposalRecord(
    const FixedBaseRecord &base, const FixedIncomingBaseProvenance &provenance,
    const CandidateTrajectory &candidate, const EgoState &ego,
    const std::vector<OpponentState> &opponents,
    const Ay0ShadowSafetyEvidence &evidence,
    const builtin_interfaces::msg::Time &plan_stamp,
    const Ay0ShadowProposalIdentity &identity, std::uint64_t session_generation,
    std::uint64_t session_nonce, FixedProposalRecord *record) noexcept;

} // namespace state_lattice_overtake_planner::c002ay0_shadow
