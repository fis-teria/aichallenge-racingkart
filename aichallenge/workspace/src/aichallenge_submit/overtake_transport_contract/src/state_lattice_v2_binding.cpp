#include "overtake_transport_contract/state_lattice_v2_binding.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "overtake_transport_contract/c002ay0_canonical.hpp"

namespace overtake_transport_contract::state_lattice_v2 {
namespace {

std::int64_t timeNs(const builtin_interfaces::msg::Time &value) {
  if (value.sec < 0 || value.nanosec >= 1000000000U) {
    return -1;
  }
  return static_cast<std::int64_t>(value.sec) * 1000000000LL +
         static_cast<std::int64_t>(value.nanosec);
}

bool digestEqual(const std::array<std::uint8_t, 32U> &left,
                 const std::array<std::uint8_t, 32U> &right) {
  return std::equal(left.begin(), left.end(), right.begin());
}

} // namespace

RejectReason validateBaseAttestation(
    const BaseAttestation &attestation,
    const std::string &expected_producer_instance_id,
    const std::string &expected_session_id,
    const builtin_interfaces::msg::Time &now_ros) {
  const auto now_ns = timeNs(now_ros);
  const auto &base = attestation.base;
  if (now_ns <= 0) {
    return RejectReason::kClockFault;
  }
  if (attestation.schema_version !=
          BaseAttestation::SCHEMA_V1_NON_AUTHORITATIVE ||
      attestation.producer_instance_id.empty() || attestation.session_id.empty() ||
      attestation.attestation_sequence == 0U ||
      attestation.header.frame_id != "map" || base.frame_id != "map" ||
      base.authority_eligible ||
      c002ay0::validateBaseSnapshotV1(base) !=
          c002ay0::ValidationError::NONE) {
    return RejectReason::kMalformed;
  }
  if (attestation.producer_instance_id != expected_producer_instance_id ||
      attestation.session_id != expected_session_id ||
      attestation.session_id != std::to_string(base.race_arm_epoch) ||
      attestation.attestation_sequence != base.controller_sequence ||
      timeNs(attestation.header.stamp) != timeNs(base.record_stamp)) {
    return RejectReason::kIdentityMismatch;
  }
  if (timeNs(attestation.header.stamp) > now_ns) {
    return RejectReason::kClockFault;
  }
  if (now_ns >= timeNs(base.lease_valid_until)) {
    return RejectReason::kBaseAttestationExpired;
  }
  return RejectReason::kNone;
}

RejectReason validate(const Proposal &proposal,
                      const builtin_interfaces::msg::Time &now_ros) {
  const auto now_ns = timeNs(now_ros);
  if (now_ns <= 0) {
    return RejectReason::kClockFault;
  }
  if (proposal.schema_version != Proposal::SCHEMA_V2_NON_AUTHORITATIVE) {
    return RejectReason::kMalformed;
  }
  if (proposal.header.frame_id != "map" ||
      proposal.identity.frame_id != "map" ||
      proposal.proposal.frame_id != "map") {
    return RejectReason::kFrameMismatch;
  }
  const auto &identity = proposal.identity;
  if (identity.producer_instance_id.empty() || identity.session_id.empty() ||
      identity.proposal_sequence == 0U || identity.plan_generation == 0U ||
      identity.source_generation == 0U || identity.source_stamp.sec < 0) {
    return RejectReason::kMalformed;
  }
  if (c002ay0::validateAuthorizedTrajectoryV1(proposal.proposal) !=
      c002ay0::ValidationError::NONE) {
    return RejectReason::kMalformed;
  }
  const auto &payload = proposal.proposal;
  if (identity.plan_generation != payload.plan_sample_key.plan_generation ||
      identity.producer_instance_id !=
          std::to_string(payload.plan_sample_key.planner_instance_id) ||
      identity.session_id !=
          std::to_string(payload.plan_sample_key.race_arm_epoch) ||
      identity.source_generation != payload.base_source_generation ||
      timeNs(identity.source_stamp) != timeNs(payload.base_source_stamp) ||
      timeNs(proposal.header.stamp) != timeNs(payload.plan_stamp) ||
      !digestEqual(identity.canonical_sha256, payload.payload_sha256)) {
    return RejectReason::kIdentityMismatch;
  }
  if (timeNs(proposal.header.stamp) > now_ns) {
    return RejectReason::kClockFault;
  }
  if (now_ns >= timeNs(payload.safety_valid_until)) {
    return RejectReason::kDeadlineMiss;
  }
  return RejectReason::kNone;
}

BindingStore::BindingStore(std::string expected_producer_instance_id,
                           std::string expected_base_producer_instance_id,
                           std::string expected_base_session_id)
    : expected_producer_instance_id_(std::move(expected_producer_instance_id)),
      expected_base_producer_instance_id_(
          std::move(expected_base_producer_instance_id)),
      expected_base_session_id_(std::move(expected_base_session_id)) {
}

bool BindingStore::enqueue(const Proposal &proposal,
                           std::uint64_t receive_monotonic_ns,
                           RejectReason *reason) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (run_invalid_ || size_ == kCapacity) {
    // Preserve only the first observable overflow without mutating the bounded
    // proposal queue.  It is terminalized at the next control-cycle entry,
    // never in the subscription callback. Later rejected callbacks are still
    // fail-closed but cannot overwrite that terminal evidence.
    if (!run_invalid_identity_.has_value()) {
      pending_enqueue_reject_ = PendingProposal{proposal, receive_monotonic_ns};
      run_invalid_identity_ = proposal.identity;
      run_invalid_receive_monotonic_ns_ = receive_monotonic_ns;
    }
    run_invalid_ = true;
    if (overflow_count_ != std::numeric_limits<std::uint32_t>::max()) {
      ++overflow_count_;
    }
    const auto sequence = proposal.identity.proposal_sequence;
    if (overflow_first_sequence_ == 0U) {
      overflow_first_sequence_ = sequence;
    }
    overflow_last_sequence_ = sequence;
    if (reason != nullptr) {
      *reason = RejectReason::kOverflow;
    }
    return false;
  }
  pending_[(head_ + size_) % kCapacity] =
      PendingProposal{proposal, receive_monotonic_ns};
  ++size_;
  if (reason != nullptr) {
    *reason = RejectReason::kNone;
  }
  return true;
}

void BindingStore::clearCohort(bool preserve_run_invalid) noexcept {
  if (!preserve_run_invalid) {
    for (auto &entry : pending_) {
      entry.reset();
    }
    pending_enqueue_reject_.reset();
    head_ = 0U;
    size_ = 0U;
  }
  active_availability_hold_.reset();
  active_producer_instance_id_.clear();
  last_session_id_.clear();
  last_sequence_ = 0U;
  for (auto &entry : base_history_) {
    entry.reset();
  }
  base_history_head_ = 0U;
  base_history_size_ = 0U;
  if (!preserve_run_invalid) {
    run_invalid_ = false;
    overflow_count_ = 0U;
    overflow_first_sequence_ = 0U;
    overflow_last_sequence_ = 0U;
    run_invalid_identity_.reset();
    run_invalid_receive_monotonic_ns_ = 0U;
  }
}

bool BindingStore::recordBaseAttestation(
    const BaseAttestation &attestation,
    const builtin_interfaces::msg::Time &now_ros, RejectReason *reason) {
  // This is called only after PP command/status publication.  Keep the lock
  // bounded to a compact key copy; the 256-point ROS message is never retained.
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto validation = validateBaseAttestation(
      attestation, expected_base_producer_instance_id_,
      expected_base_session_id_, now_ros);
  if (validation != RejectReason::kNone || run_invalid_ ||
      clock_recovery_pending_) {
    if (reason != nullptr) {
      *reason = validation != RejectReason::kNone ? validation
                                                  : RejectReason::kClockFault;
    }
    return false;
  }
  const auto &base = attestation.base;
  BaseKey key;
  key.producer_instance_id = attestation.producer_instance_id;
  key.session_id = attestation.session_id;
  key.attestation_sequence = attestation.attestation_sequence;
  key.controller_instance_id = base.controller_instance_id;
  key.controller_sequence = base.controller_sequence;
  key.base_lease_id = base.base_lease_id;
  key.lease_valid_until = base.lease_valid_until;
  key.base_source_kind = base.base_source_kind;
  key.base_source_stamp = base.base_source_stamp;
  key.base_source_generation = base.base_source_generation;
  key.base_original_point_count = base.base_original_point_count;
  key.first_source_index = base.first_source_index;
  key.last_source_index = base.last_source_index;
  key.nearest_source_index = base.nearest_source_index;
  key.base_source_digest_state = base.base_source_digest_state;
  key.canonical_algorithm_version = base.canonical_algorithm_version;
  key.base_geometry_sha256 = base.base_geometry_sha256;
  key.base_source_sha256 = base.base_source_sha256;
  key.controller_implementation_sha256 = base.controller_implementation_sha256;
  key.controller_config_sha256 = base.controller_config_sha256;
  key.snapshot_sha256 = base.snapshot_sha256;
  const auto index = (base_history_head_ + base_history_size_) % kCapacity;
  if (base_history_size_ == kCapacity) {
    base_history_[base_history_head_] = std::move(key);
    base_history_head_ = (base_history_head_ + 1U) % kCapacity;
  } else {
    base_history_[index] = std::move(key);
    ++base_history_size_;
  }
  if (reason != nullptr) {
    *reason = RejectReason::kNone;
  }
  return true;
}

RejectReason BindingStore::matchBaseAttestation(
    const Proposal &proposal,
    const builtin_interfaces::msg::Time &now_ros) const {
  // Phase 1 unit users remain proposal-only.  The live PP constructs this
  // store with both explicit IDs, which makes direct attestation mandatory.
  if (expected_base_producer_instance_id_.empty() &&
      expected_base_session_id_.empty()) {
    return RejectReason::kNone;
  }
  if (base_history_size_ == 0U) {
    return RejectReason::kBaseAttestationMissing;
  }
  const auto &payload = proposal.proposal;
  bool matching_expired = false;
  for (std::size_t offset = 0U; offset < base_history_size_; ++offset) {
    const auto &entry = base_history_[(base_history_head_ + offset) % kCapacity];
    if (!entry.has_value()) {
      continue;
    }
    const auto &base = entry.value();
    const bool exact =
        base.producer_instance_id == expected_base_producer_instance_id_ &&
        base.session_id == expected_base_session_id_ &&
        base.controller_instance_id == payload.source_controller_instance_id &&
        base.controller_sequence == payload.source_controller_sequence &&
        base.attestation_sequence == payload.source_controller_sequence &&
        base.base_lease_id == payload.base_lease_id &&
        timeNs(base.lease_valid_until) == timeNs(payload.base_lease_valid_until) &&
        base.base_source_kind == payload.base_source_kind &&
        timeNs(base.base_source_stamp) == timeNs(payload.base_source_stamp) &&
        base.base_source_generation == payload.base_source_generation &&
        base.base_original_point_count == payload.base_original_point_count &&
        base.first_source_index == payload.base_first_source_index &&
        base.last_source_index == payload.base_last_source_index &&
        base.nearest_source_index == payload.base_nearest_source_index &&
        base.base_source_digest_state == payload.base_source_digest_state &&
        base.canonical_algorithm_version == payload.canonical_algorithm_version &&
        digestEqual(base.base_geometry_sha256, payload.base_geometry_sha256) &&
        digestEqual(base.base_source_sha256, payload.base_source_sha256) &&
        digestEqual(base.controller_implementation_sha256,
                    payload.controller_implementation_sha256) &&
        digestEqual(base.controller_config_sha256,
                    payload.controller_config_sha256) &&
        digestEqual(base.snapshot_sha256, payload.base_snapshot_sha256);
    if (!exact) {
      continue;
    }
    if (timeNs(now_ros) >= timeNs(base.lease_valid_until)) {
      matching_expired = true;
      continue;
    }
    return RejectReason::kNone;
  }
  return matching_expired ? RejectReason::kBaseAttestationExpired
                          : RejectReason::kBaseAttestationMismatch;
}

CycleResult
BindingStore::beginCycle(const builtin_interfaces::msg::Time &now_ros,
                         std::uint64_t now_monotonic_ns) {
  const std::lock_guard<std::mutex> lock(mutex_);
  CycleResult result;
  ++pp_cycle_sequence_;
  result.pp_cycle_sequence = pp_cycle_sequence_;
  result.timer_entry_monotonic_ns = now_monotonic_ns;
  result.run_invalid = run_invalid_;
  result.overflow_count = overflow_count_;
  result.overflow_first_sequence = overflow_first_sequence_;
  result.overflow_last_sequence = overflow_last_sequence_;
  auto append_event = [&result](
                          const std::optional<
                              multi_purpose_mpc_ros_msgs::msg::
                                  StateLatticeV2Identity> &identity,
                          bool first_uptake, std::uint32_t hold_cycle_index,
                          RejectReason reject_reason,
                          std::uint64_t receive_monotonic_ns,
                          std::uint64_t accepted_monotonic_ns,
                          bool primary) {
    if (result.event_count >= result.events.size()) {
      return false;
    }
    auto &event = result.events[result.event_count++];
    event.identity = identity;
    event.first_uptake = first_uptake;
    event.hold_cycle_index = hold_cycle_index;
    event.reject_reason = reject_reason;
    event.receive_monotonic_ns = receive_monotonic_ns;
    event.accepted_monotonic_ns = accepted_monotonic_ns;
    if (primary || !result.observed_identity.has_value()) {
      result.observed_identity = identity;
      result.first_uptake = first_uptake;
      result.hold_cycle_index = hold_cycle_index;
      result.reject_reason = reject_reason;
      result.receive_monotonic_ns = receive_monotonic_ns;
      result.accepted_monotonic_ns = accepted_monotonic_ns;
    }
    return true;
  };
  const auto now_ros_ns = timeNs(now_ros);
  if (now_ros_ns <= 0 ||
      (last_now_ros_ns_ >= 0 && now_ros_ns < last_now_ros_ns_)) {
    std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2Identity>
        identity;
    std::uint64_t receive_ns = 0U;
    std::uint64_t accepted_ns = 0U;
    if (active_availability_hold_.has_value()) {
      identity = active_availability_hold_->identity;
      receive_ns = active_availability_hold_->receive_monotonic_ns;
      accepted_ns = active_availability_hold_->accepted_monotonic_ns;
    }
    if (!identity.has_value() && size_ != 0U && pending_[head_].has_value()) {
      identity = pending_[head_]->proposal.identity;
      receive_ns = pending_[head_]->receive_monotonic_ns;
    }
    (void)append_event(identity, false, 0U, RejectReason::kClockFault,
                       receive_ns, accepted_ns, true);
    clearCohort(run_invalid_);
    last_now_ros_ns_ = -1;
    clock_recovery_pending_ = true;
    result.run_invalid = true;
    return result;
  }
  if (clock_recovery_pending_) {
    if (last_now_ros_ns_ < 0) {
      last_now_ros_ns_ = now_ros_ns;
      (void)append_event(std::nullopt, false, 0U, RejectReason::kClockFault,
                         0U, 0U, true);
      result.run_invalid = true;
      return result;
    }
    if (now_ros_ns <= last_now_ros_ns_) {
      (void)append_event(std::nullopt, false, 0U, RejectReason::kClockFault,
                         0U, 0U, true);
      result.run_invalid = true;
      return result;
    }
    clearCohort(run_invalid_);
    clock_recovery_pending_ = false;
  }
  last_now_ros_ns_ = now_ros_ns;

  // Preserve whether this PP cycle entered with an active cohort even when
  // that cohort expires at the equality boundary below.  A newer proposal
  // exact-uptaken later in this same cycle is REPLACED (continuous cycle
  // ownership), while the expired cohort itself receives no availability
  // credit.
  const bool had_availability_at_entry =
      active_availability_hold_.has_value();
  if (active_availability_hold_.has_value() &&
      now_ros_ns >= timeNs(active_availability_hold_->safety_valid_until)) {
    (void)append_event(active_availability_hold_->identity, false,
                       active_availability_hold_->hold_cycle_index,
                       RejectReason::kStale,
                       active_availability_hold_->receive_monotonic_ns,
                       active_availability_hold_->accepted_monotonic_ns, false);
    active_availability_hold_.reset();
  }
  if (active_availability_hold_.has_value()) {
    result.availability_present = true;
    result.availability_identity = active_availability_hold_->identity;
    result.availability_safety_valid_until =
        active_availability_hold_->safety_valid_until;
    result.availability_transition = AvailabilityTransition::kHeld;
  }

  if (run_invalid_) {
    // Overflow invalidates the run permanently, but terminalize the first
    // dropped proposal and every proposal already admitted to the bounded
    // queue exactly once. This preserves one-candidate-per-cycle evidence
    // without repeating an identity after the queue is drained.
    if (pending_enqueue_reject_.has_value()) {
      (void)append_event(pending_enqueue_reject_->proposal.identity, false, 0U,
                         RejectReason::kOverflow,
                         pending_enqueue_reject_->receive_monotonic_ns, 0U,
                         true);
      pending_enqueue_reject_.reset();
    } else if (size_ != 0U) {
      auto candidate = std::move(pending_[head_]);
      pending_[head_].reset();
      head_ = (head_ + 1U) % kCapacity;
      --size_;
      if (candidate.has_value()) {
        (void)append_event(candidate->proposal.identity, false, 0U,
                           RejectReason::kOverflow,
                           candidate->receive_monotonic_ns, 0U, true);
      } else {
        (void)append_event(std::nullopt, false, 0U, RejectReason::kOverflow,
                           0U, 0U, true);
      }
    }
    result.run_invalid = true;
    result.overflow_count = overflow_count_;
    result.overflow_first_sequence = overflow_first_sequence_;
    result.overflow_last_sequence = overflow_last_sequence_;
    active_availability_hold_.reset();
    result.availability_present = false;
    result.availability_identity.reset();
    result.availability_safety_valid_until =
        builtin_interfaces::msg::Time{};
    result.availability_transition = AvailabilityTransition::kNone;
    return result;
  }

  // Keep exactly one active availability cohort until its safety deadline.
  // A counter wrap would falsify identity-bound evidence, so fail closed.
  if (active_availability_hold_.has_value()) {
    auto &hold = active_availability_hold_.value();
    if (hold.hold_cycle_index == std::numeric_limits<std::uint32_t>::max()) {
      run_invalid_ = true;
      run_invalid_identity_ = hold.identity;
      run_invalid_receive_monotonic_ns_ = hold.receive_monotonic_ns;
      if (overflow_count_ != std::numeric_limits<std::uint32_t>::max()) {
        ++overflow_count_;
      }
      if (overflow_first_sequence_ == 0U) {
        overflow_first_sequence_ = hold.identity.proposal_sequence;
      }
      overflow_last_sequence_ = hold.identity.proposal_sequence;
      (void)append_event(hold.identity, false, hold.hold_cycle_index,
                         RejectReason::kOverflow, hold.receive_monotonic_ns,
                         hold.accepted_monotonic_ns, true);
      active_availability_hold_.reset();
      result.run_invalid = true;
      result.overflow_count = overflow_count_;
      result.overflow_first_sequence = overflow_first_sequence_;
      result.overflow_last_sequence = overflow_last_sequence_;
      result.availability_present = false;
      result.availability_identity.reset();
      result.availability_safety_valid_until =
          builtin_interfaces::msg::Time{};
      result.availability_transition = AvailabilityTransition::kNone;
      return result;
    }
    ++hold.hold_cycle_index;
    (void)append_event(hold.identity, false, hold.hold_cycle_index,
                       RejectReason::kNone, hold.receive_monotonic_ns,
                       hold.accepted_monotonic_ns, false);
  }

  // Exactly one candidate (including a callback-time overflow) is
  // terminalized per control cycle.  Additional queue entries wait for later
  // PP periods and therefore cannot overwrite evidence from this period.
  if (pending_enqueue_reject_.has_value()) {
    (void)append_event(pending_enqueue_reject_->proposal.identity, false, 0U,
                       RejectReason::kOverflow,
                       pending_enqueue_reject_->receive_monotonic_ns, 0U,
                       true);
    pending_enqueue_reject_.reset();
  } else if (size_ != 0U) {
    auto candidate = std::move(pending_[head_]);
    pending_[head_].reset();
    head_ = (head_ + 1U) % kCapacity;
    --size_;
    if (!candidate.has_value()) {
      (void)append_event(std::nullopt, false, 0U, RejectReason::kMalformed,
                         0U, 0U, true);
    } else {
      const auto validation = validate(candidate->proposal, now_ros);
      if (validation != RejectReason::kNone) {
        (void)append_event(candidate->proposal.identity, false, 0U, validation,
                           candidate->receive_monotonic_ns, 0U, true);
      } else {
        const auto &identity = candidate->proposal.identity;
        if (expected_producer_instance_id_.empty() ||
            identity.producer_instance_id != expected_producer_instance_id_) {
          (void)append_event(identity, false, 0U,
                             RejectReason::kIdentityMismatch,
                             candidate->receive_monotonic_ns, 0U, true);
        } else if (!active_producer_instance_id_.empty() &&
                   (identity.producer_instance_id !=
                        active_producer_instance_id_ ||
                    identity.session_id != last_session_id_)) {
          // A BindingStore instance is an explicit single producer/session
          // cohort.  A new planner process or race session requires a fresh PP
          // BindingStore, preventing session switching from bypassing replay
          // ordering.
          (void)append_event(identity, false, 0U,
                             RejectReason::kIdentityMismatch,
                             candidate->receive_monotonic_ns, 0U, true);
        } else if (identity.proposal_sequence == last_sequence_) {
          (void)append_event(identity, false, 0U,
                             RejectReason::kDuplicateOrReplay,
                             candidate->receive_monotonic_ns, 0U, true);
        } else if (identity.proposal_sequence < last_sequence_ ||
                   identity.plan_generation <= last_plan_generation_) {
          (void)append_event(identity, false, 0U, RejectReason::kOutOfOrder,
                             candidate->receive_monotonic_ns, 0U, true);
        } else {
          const auto base_match = matchBaseAttestation(candidate->proposal, now_ros);
          if (base_match != RejectReason::kNone) {
            (void)append_event(identity, false, 0U, base_match,
                               candidate->receive_monotonic_ns, 0U, true);
          } else {
            active_producer_instance_id_ = identity.producer_instance_id;
            last_session_id_ = identity.session_id;
            last_sequence_ = identity.proposal_sequence;
            last_plan_generation_ = identity.plan_generation;
            ObservationHold hold;
            hold.identity = identity;
            hold.safety_valid_until =
                candidate->proposal.proposal.safety_valid_until;
            hold.hold_cycle_index = 1U;
            hold.receive_monotonic_ns = candidate->receive_monotonic_ns;
            hold.accepted_monotonic_ns = now_monotonic_ns;
            active_availability_hold_ = std::move(hold);
            result.availability_present = true;
            result.availability_identity = identity;
            result.availability_safety_valid_until =
                candidate->proposal.proposal.safety_valid_until;
            result.availability_transition = had_availability_at_entry
                                                ? AvailabilityTransition::kReplaced
                                                : AvailabilityTransition::kFirst;
            (void)append_event(identity, true, 1U, RejectReason::kNone,
                               candidate->receive_monotonic_ns,
                               now_monotonic_ns, true);
            result.accepted = std::move(candidate->proposal);
          }
        }
      }
    }
  }
  return result;
}

std::size_t BindingStore::pendingSize() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return size_;
}

} // namespace overtake_transport_contract::state_lattice_v2
