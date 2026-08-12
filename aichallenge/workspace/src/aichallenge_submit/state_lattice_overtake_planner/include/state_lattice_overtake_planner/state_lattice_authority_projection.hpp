#pragma once

#include <string>
#include <optional>

#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/controller_tracking_status.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/overtake_plan.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/safety_constraint.hpp"

namespace state_lattice_overtake_planner {

struct StateLatticeAuthorityProjection {
  bool valid{false};
  bool warmup{true};
  std::string reason{"invalid"};
  multi_purpose_mpc_ros_msgs::msg::OvertakePlan plan;
  multi_purpose_mpc_ros_msgs::msg::SafetyConstraint constraint;
};

struct StateLatticeAuthorityTransaction {
  std::uint32_t attempt_id{0U};
  std::uint64_t connector_transaction_id{0U};
  std::uint64_t authority_token{0U};
  std::string target_id;
  std::int8_t pass_direction{0};
};

class StateLatticeAuthorityTransactionState {
public:
  const StateLatticeAuthorityTransaction *select(const std::string &target_id,
                                                  std::int8_t pass_direction);
  void reset() noexcept { active_.reset(); }

private:
  std::optional<StateLatticeAuthorityTransaction> active_;
  std::uint32_t last_attempt_id_{0U};
};

StateLatticeAuthorityProjection projectStateLatticeAuthority(
    const multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory
        &proposal,
    const multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus
        *tracking_status,
    const multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory
        *predecessor_proposal,
    bool predecessor_fresh, bool race_armed, std::uint64_t race_arm_epoch);

} // namespace state_lattice_overtake_planner
