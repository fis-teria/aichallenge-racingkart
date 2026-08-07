#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"
#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <cstdint>
#include <optional>

namespace overtake_transport_contract::c002ay0 {

enum class ShadowBindingDisposition : std::uint8_t {
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

struct ShadowBindingResult {
  ShadowBindingDisposition disposition{
      ShadowBindingDisposition::kRejectedInvalidCurrent};
  ValidationError validation_error{ValidationError::NONE};
  bool lateral_authority_eligible{false};
};

ShadowBindingResult evaluateShadowBinding(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &current,
    const std::optional<
        multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot>
        &previous,
    const multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory
        &proposal,
    const builtin_interfaces::msg::Time &now);

const char *toString(ShadowBindingDisposition disposition);

} // namespace overtake_transport_contract::c002ay0
