#pragma once

#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/grid_map.hpp"
#include "state_lattice_overtake_planner/types.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"
#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner {

enum class Ay0ShadowProposalFailure : std::uint8_t {
  NONE = 0U,
  INVALID_INPUT,
  BASE_INVALID,
  BASE_NOT_CURRENT,
  CANDIDATE_NOT_FEASIBLE,
  CANDIDATE_START_MISMATCH,
  GEOMETRY_INVALID,
  PROVENANCE_INVALID,
};

struct Ay0ShadowProposalIdentity {
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  std::uint64_t connector_transaction_id{0U};
  std::uint64_t authority_token{0U};
  std::uint64_t safety_snapshot_id{0U};
  std::uint32_t plan_generation{0U};
  std::uint32_t candidate_revision{0U};
  std::string target_id;
};

struct Ay0ShadowSafetyEvidence {
  overtake_transport_contract::c002ay0::Digest
      evaluator_implementation_sha256{};
  overtake_transport_contract::c002ay0::Digest evaluator_config_sha256{};
};

struct Ay0ShadowProposalResult {
  Ay0ShadowProposalFailure failure{Ay0ShadowProposalFailure::INVALID_INPUT};
  std::string reason{"invalid_input"};
  multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory trajectory;

  bool valid() const noexcept {
    return failure == Ay0ShadowProposalFailure::NONE;
  }
};

overtake_transport_contract::c002ay0::Digest
stateLatticeSafetyEvaluatorImplementationDigest();

overtake_transport_contract::c002ay0::Digest
stateLatticeSafetyEvaluatorConfigDigest(const PlannerConfig &config,
                                        const GridMap &map,
                                        const FrenetFrame &frame);

Ay0ShadowProposalResult buildAy0ShadowProposal(
    const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &base,
    const CandidateTrajectory &candidate, const EgoState &ego,
    const std::vector<OpponentState> &opponents, const PlannerConfig &config,
    const Ay0ShadowSafetyEvidence &safety_evidence,
    const builtin_interfaces::msg::Time &plan_stamp,
    const Ay0ShadowProposalIdentity &identity);

const char *toString(Ay0ShadowProposalFailure failure);

} // namespace state_lattice_overtake_planner
