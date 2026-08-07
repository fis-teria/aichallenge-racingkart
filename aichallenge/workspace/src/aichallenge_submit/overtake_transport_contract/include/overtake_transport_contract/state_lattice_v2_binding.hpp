#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory_v2.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/state_lattice_v2_identity.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/state_lattice_v2_base_attestation.hpp"

namespace overtake_transport_contract::state_lattice_v2 {

using Proposal =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectoryV2;
using BaseAttestation =
    multi_purpose_mpc_ros_msgs::msg::StateLatticeV2BaseAttestation;

enum class RejectReason : std::uint8_t {
  kNone = 0U,
  kMalformed,
  kDuplicateOrReplay,
  kOutOfOrder,
  kIdentityMismatch,
  kDeadlineMiss,
  kStale,
  kFrameMismatch,
  kOverflow,
  kClockFault,
  kBaseAttestationMissing,
  kBaseAttestationMismatch,
  kBaseAttestationExpired,
};

enum class AvailabilityTransition : std::uint8_t {
  kNone = 0U,
  kFirst,
  kHeld,
  kReplaced,
};

struct CycleResult {
  struct ObservationEvent {
    std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity>
        identity;
    bool first_uptake{false};
    std::uint32_t hold_cycle_index{0U};
    RejectReason reject_reason{RejectReason::kNone};
    std::uint64_t receive_monotonic_ns{0U};
    std::uint64_t accepted_monotonic_ns{0U};
  };

  static constexpr std::size_t kMaxEvents = 11U;
  std::optional<Proposal> accepted;
  std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity>
      observed_identity;
  bool first_uptake{false};
  std::uint32_t hold_cycle_index{0U};
  RejectReason reject_reason{RejectReason::kNone};
  std::uint64_t receive_monotonic_ns{0U};
  std::uint64_t accepted_monotonic_ns{0U};
  bool run_invalid{false};
  std::uint32_t overflow_count{0U};
  std::uint64_t overflow_first_sequence{0U};
  std::uint64_t overflow_last_sequence{0U};
  std::uint64_t pp_cycle_sequence{0U};
  bool availability_present{false};
  std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity>
      availability_identity;
  builtin_interfaces::msg::Time availability_safety_valid_until{};
  AvailabilityTransition availability_transition{AvailabilityTransition::kNone};
  std::uint64_t timer_entry_monotonic_ns{0U};
  std::array<ObservationEvent, kMaxEvents> events{};
  std::size_t event_count{0U};
};

// One PP cycle can publish ten concurrent hold observations plus one terminal
// candidate/reject event and its mandatory availability summary. Keep the
// status DDS history large enough for one complete cycle burst.
constexpr std::size_t kStatusQosDepth = 17U;
static_assert(kStatusQosDepth >= CycleResult::kMaxEvents + 1U);

// Non-authoritative fixed-capacity handoff. It has no command or Mux output.
class BindingStore {
public:
  static constexpr std::size_t kCapacity = 8U;
  // Retained as the acceptance evidence target. Availability itself is driven
  // exclusively by safety_valid_until, not by this count.
  static constexpr std::uint32_t kHoldCycles = 10U;

  explicit BindingStore(std::string expected_producer_instance_id,
                        std::string expected_base_producer_instance_id = {},
                        std::string expected_base_session_id = {});

  bool enqueue(const Proposal &proposal, std::uint64_t receive_monotonic_ns,
               RejectReason *reason);
  CycleResult beginCycle(const builtin_interfaces::msg::Time &now_ros,
                         std::uint64_t now_monotonic_ns);
  bool recordBaseAttestation(const BaseAttestation &attestation,
                             const builtin_interfaces::msg::Time &now_ros,
                             RejectReason *reason);
  std::size_t pendingSize() const;

private:
  friend struct BindingStoreTestPeer;
  struct PendingProposal {
    Proposal proposal;
    std::uint64_t receive_monotonic_ns{0U};
  };

  struct BaseKey {
    std::string producer_instance_id;
    std::string session_id;
    std::uint64_t attestation_sequence{0U};
    std::uint64_t controller_instance_id{0U};
    std::uint64_t controller_sequence{0U};
    std::uint64_t base_lease_id{0U};
    builtin_interfaces::msg::Time lease_valid_until{};
    std::uint8_t base_source_kind{0U};
    builtin_interfaces::msg::Time base_source_stamp{};
    std::uint32_t base_source_generation{0U};
    std::uint32_t base_original_point_count{0U};
    std::uint32_t first_source_index{0U};
    std::uint32_t last_source_index{0U};
    std::uint32_t nearest_source_index{0U};
    std::uint8_t base_source_digest_state{0U};
    std::uint8_t canonical_algorithm_version{0U};
    std::array<std::uint8_t, 32U> base_geometry_sha256{};
    std::array<std::uint8_t, 32U> base_source_sha256{};
    std::array<std::uint8_t, 32U> controller_implementation_sha256{};
    std::array<std::uint8_t, 32U> controller_config_sha256{};
    std::array<std::uint8_t, 32U> snapshot_sha256{};
  };

  std::array<std::optional<PendingProposal>, kCapacity> pending_{};
  std::optional<PendingProposal> pending_enqueue_reject_;
  mutable std::mutex mutex_;
  std::size_t head_{0U};
  std::size_t size_{0U};
  struct ObservationHold {
    multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity identity;
    builtin_interfaces::msg::Time safety_valid_until{};
    std::uint32_t hold_cycle_index{0U};
    std::uint64_t receive_monotonic_ns{0U};
    std::uint64_t accepted_monotonic_ns{0U};
  };
  std::optional<ObservationHold> active_availability_hold_;
  const std::string expected_producer_instance_id_;
  const std::string expected_base_producer_instance_id_;
  const std::string expected_base_session_id_;
  std::string active_producer_instance_id_;
  std::string last_session_id_;
  std::uint64_t last_sequence_{0U};
  std::uint64_t last_plan_generation_{0U};
  std::uint64_t pp_cycle_sequence_{0U};
  std::int64_t last_now_ros_ns_{-1};
  bool clock_recovery_pending_{false};
  bool run_invalid_{false};
  std::uint32_t overflow_count_{0U};
  std::uint64_t overflow_first_sequence_{0U};
  std::uint64_t overflow_last_sequence_{0U};
  std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity>
      run_invalid_identity_;
  std::uint64_t run_invalid_receive_monotonic_ns_{0U};
  std::array<std::optional<BaseKey>, kCapacity> base_history_{};
  std::size_t base_history_head_{0U};
  std::size_t base_history_size_{0U};

  void clearCohort(bool preserve_run_invalid = false) noexcept;
  RejectReason matchBaseAttestation(const Proposal &proposal,
                                    const builtin_interfaces::msg::Time &now_ros) const;
};

RejectReason validate(const Proposal &proposal,
                      const builtin_interfaces::msg::Time &now_ros);
RejectReason validateBaseAttestation(
    const BaseAttestation &attestation,
    const std::string &expected_producer_instance_id,
    const std::string &expected_session_id,
    const builtin_interfaces::msg::Time &now_ros);

} // namespace overtake_transport_contract::state_lattice_v2
