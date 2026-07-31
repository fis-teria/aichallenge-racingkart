#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"
#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <cstdint>
#include <optional>

namespace simple_pure_pursuit {

// This is deliberately a shadow-only classifier.  It never grants lateral
// authority; a later live contract must use a separately reviewed application
// status/ACK path.
enum class StateLatticeShadowBindingDisposition : std::uint8_t {
  kExactCurrent = 0U,
  kDeferredPredecessor,
  kDeferredUnpairedDelivery,
  kReplanRequiredNearestAdvanced,
  kReplanRequiredNearestRegressed,
  kRejectedSourceMutation,
  kRejectedIdentity,
  kRejectedStale,
  kRejectedInvalidCurrent,
  kRejectedInvalidProposal,
};

struct StateLatticeShadowBindingResult {
  StateLatticeShadowBindingDisposition disposition{
      StateLatticeShadowBindingDisposition::kRejectedInvalidCurrent};
  overtake_transport_contract::c002ay0::ValidationError validation_error{
      overtake_transport_contract::c002ay0::ValidationError::NONE};

  // Kept explicit to prevent this diagnostic result being accidentally wired
  // into a motion-authority path.
  bool lateral_authority_eligible{false};
};

// Compare the source tuple that actually reached PP with a State Lattice
// shadow proposal.  `previous` represents only the immediately preceding
// capture; it is allowed to classify a normal N-1 delivery gap, never to
// apply lateral motion or release STOP.
StateLatticeShadowBindingResult evaluateStateLatticeShadowBinding(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &current,
    const std::optional<
        multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot>
        &previous,
    const multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory
        &proposal,
    const builtin_interfaces::msg::Time &now);

const char *toString(StateLatticeShadowBindingDisposition disposition);

}  // namespace simple_pure_pursuit
