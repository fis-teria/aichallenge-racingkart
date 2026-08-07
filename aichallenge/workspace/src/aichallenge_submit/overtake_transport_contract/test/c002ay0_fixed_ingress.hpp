#ifndef OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY0_FIXED_INGRESS_HPP_
#define OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY0_FIXED_INGRESS_HPP_

#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace overtake_transport_contract::c002ay0::test_fixture {

enum class IngressPushResult : std::uint8_t {
  INSERTED,
  BUSY,
  SHUTDOWN,
  STRUCTURAL_LIMIT,
  COPY_FAILED,
};

enum class IngressSlotState : std::uint8_t {
  FREE,
  INGRESS_WRITING,
  READY_FOR_WORKER,
  WORKER_OWNED,
};

enum class MutationLedgerResult : std::uint8_t {
  INSERTED,
  CONSISTENT_DUPLICATE,
  SAME_GENERATION_MUTATION,
  EPOCH_REGRESSION,
  RESOURCE_LIMIT,
};

inline std::uint64_t
absoluteTimeNs(const builtin_interfaces::msg::Time &stamp) noexcept {
  if (stamp.sec < 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

inline bool
workerInputExpired(const AuthorizedCartesianTrajectory &trajectory,
                   const std::uint64_t current_ros_time_ns) noexcept {
  return current_ros_time_ns >=
             absoluteTimeNs(trajectory.base_lease_valid_until) ||
         current_ros_time_ns >= absoluteTimeNs(trajectory.safety_valid_until);
}

template <std::size_t Capacity> class BoundedMutationLedger {
  static_assert(Capacity > 0U);

public:
  MutationLedgerResult
  observe(const AuthorizedCartesianTrajectory &trajectory) noexcept {
    const auto epoch = trajectory.plan_sample_key.race_arm_epoch;
    if (!have_epoch_) {
      current_epoch_ = epoch;
      have_epoch_ = true;
    } else if (epoch < current_epoch_) {
      return MutationLedgerResult::EPOCH_REGRESSION;
    } else if (epoch > current_epoch_) {
      clear();
      current_epoch_ = epoch;
      have_epoch_ = true;
    }

    for (auto &entry : entries_) {
      if (!entry.occupied || !sameKey(entry, trajectory)) {
        continue;
      }
      if (entry.mutation_latched ||
          entry.payload_sha256 != trajectory.payload_sha256) {
        entry.mutation_latched = true;
        return MutationLedgerResult::SAME_GENERATION_MUTATION;
      }
      return MutationLedgerResult::CONSISTENT_DUPLICATE;
    }
    for (auto &entry : entries_) {
      if (entry.occupied) {
        continue;
      }
      entry.occupied = true;
      entry.planner_instance_id =
          trajectory.plan_sample_key.planner_instance_id;
      entry.plan_generation = trajectory.plan_sample_key.plan_generation;
      entry.candidate_revision = trajectory.candidate_revision;
      entry.payload_sha256 = trajectory.payload_sha256;
      return MutationLedgerResult::INSERTED;
    }
    return MutationLedgerResult::RESOURCE_LIMIT;
  }

private:
  struct Entry {
    bool occupied{false};
    bool mutation_latched{false};
    std::uint64_t planner_instance_id{0U};
    std::uint32_t plan_generation{0U};
    std::uint32_t candidate_revision{0U};
    Digest payload_sha256{};
  };

  static bool
  sameKey(const Entry &entry,
          const AuthorizedCartesianTrajectory &trajectory) noexcept {
    return entry.planner_instance_id ==
               trajectory.plan_sample_key.planner_instance_id &&
           entry.plan_generation ==
               trajectory.plan_sample_key.plan_generation &&
           entry.candidate_revision == trajectory.candidate_revision;
  }

  void clear() noexcept {
    for (auto &entry : entries_) {
      entry = Entry{};
    }
  }

  std::array<Entry, Capacity> entries_{};
  std::uint64_t current_epoch_{0U};
  bool have_epoch_{false};
};

template <std::size_t SlotCount> class AuthorizedFixedIngress {
  static_assert(SlotCount > 0U);

public:
  using Trajectory = AuthorizedCartesianTrajectory;

  struct WorkerView {
    std::size_t slot_index{SlotCount};
    std::uint64_t enqueue_sequence{0U};
    std::uint64_t receipt_steady_ns{0U};
    const Trajectory *trajectory{nullptr};
  };

  AuthorizedFixedIngress() {
    for (auto &slot : slots_) {
      slot.payload.points.reserve(kMaxCartesianPoints);
      slot.payload.frame_id.reserve(128U);
      slot.payload.plan_sample_key.target_vehicle_id.reserve(64U);
    }
  }

  AuthorizedFixedIngress(const AuthorizedFixedIngress &) = delete;
  AuthorizedFixedIngress &operator=(const AuthorizedFixedIngress &) = delete;

  static constexpr std::size_t slotCount() noexcept { return SlotCount; }

  void setPrePublishBarrierForTest(std::atomic<bool> *entered,
                                   std::atomic<bool> *release) noexcept {
    pre_publish_entered_ = entered;
    pre_publish_release_ = release;
  }

  bool atomicsLockFree() const noexcept {
    return accepting_.is_lock_free() && outstanding_.is_lock_free() &&
           ready_slot_.is_lock_free() && accepted_count_.is_lock_free() &&
           busy_count_.is_lock_free() &&
           structural_limit_count_.is_lock_free() &&
           copy_failure_count_.is_lock_free() &&
           shutdown_discard_count_.is_lock_free() &&
           slots_.front().state.is_lock_free();
  }

  IngressPushResult tryPush(const Trajectory &trajectory,
                            const std::uint64_t enqueue_sequence,
                            const std::uint64_t receipt_steady_ns) noexcept {
    if (!accepting_.load(std::memory_order_acquire)) {
      return IngressPushResult::SHUTDOWN;
    }
    if (!structurallyBounded(trajectory)) {
      structural_limit_count_.fetch_add(1U, std::memory_order_relaxed);
      return IngressPushResult::STRUCTURAL_LIMIT;
    }

    bool expected = false;
    if (!outstanding_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
      busy_count_.fetch_add(1U, std::memory_order_relaxed);
      return IngressPushResult::BUSY;
    }

    const auto slot_index = producer_cursor_;
    auto &slot = slots_[slot_index];
    if (slot.state.load(std::memory_order_acquire) != IngressSlotState::FREE) {
      outstanding_.store(false, std::memory_order_release);
      copy_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      return IngressPushResult::COPY_FAILED;
    }

    slot.state.store(IngressSlotState::INGRESS_WRITING,
                     std::memory_order_relaxed);
    try {
      slot.payload = trajectory;
    } catch (...) {
      slot.state.store(IngressSlotState::FREE, std::memory_order_release);
      outstanding_.store(false, std::memory_order_release);
      copy_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      return IngressPushResult::COPY_FAILED;
    }
    slot.enqueue_sequence = enqueue_sequence;
    slot.receipt_steady_ns = receipt_steady_ns;
    if (pre_publish_entered_ != nullptr && pre_publish_release_ != nullptr) {
      pre_publish_entered_->store(true, std::memory_order_release);
      while (!pre_publish_release_->load(std::memory_order_acquire)) {
      }
    }
    slot.state.store(IngressSlotState::READY_FOR_WORKER,
                     std::memory_order_release);
    accepted_count_.fetch_add(1U, std::memory_order_relaxed);
    ready_slot_.store(static_cast<std::uint8_t>(slot_index),
                      std::memory_order_release);
    if (!accepting_.load(std::memory_order_acquire)) {
      const auto published_slot =
          ready_slot_.exchange(kNoSlot, std::memory_order_acq_rel);
      if (published_slot == slot_index &&
          slot.state.load(std::memory_order_acquire) ==
              IngressSlotState::READY_FOR_WORKER) {
        slot.state.store(IngressSlotState::FREE, std::memory_order_release);
        outstanding_.store(false, std::memory_order_release);
        shutdown_discard_count_.fetch_add(1U, std::memory_order_relaxed);
      }
      return IngressPushResult::SHUTDOWN;
    }
    producer_cursor_ = (producer_cursor_ + 1U) % SlotCount;
    return IngressPushResult::INSERTED;
  }

  bool tryAcquire(WorkerView &view) noexcept {
    const auto slot_value =
        ready_slot_.exchange(kNoSlot, std::memory_order_acq_rel);
    if (slot_value == kNoSlot || slot_value >= SlotCount) {
      return false;
    }
    auto &slot = slots_[slot_value];
    if (slot.state.load(std::memory_order_acquire) !=
        IngressSlotState::READY_FOR_WORKER) {
      copy_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    if (!accepting_.load(std::memory_order_acquire)) {
      slot.state.store(IngressSlotState::FREE, std::memory_order_release);
      outstanding_.store(false, std::memory_order_release);
      shutdown_discard_count_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    slot.state.store(IngressSlotState::WORKER_OWNED, std::memory_order_release);
    if (!accepting_.load(std::memory_order_acquire)) {
      slot.state.store(IngressSlotState::FREE, std::memory_order_release);
      outstanding_.store(false, std::memory_order_release);
      shutdown_discard_count_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    view.slot_index = slot_value;
    view.enqueue_sequence = slot.enqueue_sequence;
    view.receipt_steady_ns = slot.receipt_steady_ns;
    view.trajectory = &slot.payload;
    return true;
  }

  bool release(const WorkerView &view) noexcept {
    if (view.slot_index >= SlotCount) {
      return false;
    }
    auto &slot = slots_[view.slot_index];
    if (view.trajectory != &slot.payload ||
        slot.state.load(std::memory_order_acquire) !=
            IngressSlotState::WORKER_OWNED) {
      return false;
    }
    slot.state.store(IngressSlotState::FREE, std::memory_order_release);
    outstanding_.store(false, std::memory_order_release);
    return true;
  }

  void shutdown() noexcept {
    accepting_.store(false, std::memory_order_release);
    const auto slot_value =
        ready_slot_.exchange(kNoSlot, std::memory_order_acq_rel);
    if (slot_value < SlotCount) {
      auto &slot = slots_[slot_value];
      if (slot.state.load(std::memory_order_acquire) ==
          IngressSlotState::READY_FOR_WORKER) {
        slot.state.store(IngressSlotState::FREE, std::memory_order_release);
        outstanding_.store(false, std::memory_order_release);
        shutdown_discard_count_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
  }

  bool accepting() const noexcept {
    return accepting_.load(std::memory_order_acquire);
  }

  bool outstanding() const noexcept {
    return outstanding_.load(std::memory_order_acquire);
  }

  std::uint64_t acceptedCount() const noexcept {
    return accepted_count_.load(std::memory_order_relaxed);
  }

  std::uint64_t busyCount() const noexcept {
    return busy_count_.load(std::memory_order_relaxed);
  }

  std::uint64_t structuralLimitCount() const noexcept {
    return structural_limit_count_.load(std::memory_order_relaxed);
  }

  std::uint64_t copyFailureCount() const noexcept {
    return copy_failure_count_.load(std::memory_order_relaxed);
  }

  std::uint64_t shutdownDiscardCount() const noexcept {
    return shutdown_discard_count_.load(std::memory_order_relaxed);
  }

  std::size_t estimatedResidentBytes() const noexcept {
    std::size_t bytes = sizeof(*this);
    for (const auto &slot : slots_) {
      bytes += slot.payload.points.capacity() *
               sizeof(typename decltype(slot.payload.points)::value_type);
      bytes += slot.payload.frame_id.capacity();
      bytes += slot.payload.plan_sample_key.target_vehicle_id.capacity();
    }
    return bytes;
  }

private:
  struct Slot {
    std::atomic<IngressSlotState> state{IngressSlotState::FREE};
    Trajectory payload{};
    std::uint64_t enqueue_sequence{0U};
    std::uint64_t receipt_steady_ns{0U};
  };

  static bool structurallyBounded(const Trajectory &trajectory) noexcept {
    return trajectory.points.size() >= 2U &&
           trajectory.points.size() <= kMaxCartesianPoints &&
           trajectory.frame_id.size() <= 128U &&
           trajectory.plan_sample_key.target_vehicle_id.size() <= 64U &&
           !trajectory.authority_eligible;
  }

  static constexpr std::uint8_t kNoSlot = static_cast<std::uint8_t>(SlotCount);

  std::array<Slot, SlotCount> slots_{};
  std::atomic<bool> accepting_{true};
  std::atomic<bool> outstanding_{false};
  std::atomic<std::uint8_t> ready_slot_{kNoSlot};
  std::atomic<std::uint64_t> accepted_count_{0U};
  std::atomic<std::uint64_t> busy_count_{0U};
  std::atomic<std::uint64_t> structural_limit_count_{0U};
  std::atomic<std::uint64_t> copy_failure_count_{0U};
  std::atomic<std::uint64_t> shutdown_discard_count_{0U};
  std::atomic<bool> *pre_publish_entered_{nullptr};
  std::atomic<bool> *pre_publish_release_{nullptr};
  std::size_t producer_cursor_{0U};
};

} // namespace overtake_transport_contract::c002ay0::test_fixture

#endif // OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY0_FIXED_INGRESS_HPP_
