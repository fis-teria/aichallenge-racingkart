#include "simple_pure_pursuit/aw2_shadow_transport.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace simple_pure_pursuit::aw2_shadow {
namespace {

constexpr std::size_t kLedgerReadAttempts = 8U;

void beginLedgerWrite(SharedLossLedger &ledger) noexcept {
  const auto version = atomicLoadAcquire(ledger.seqlock);
  atomicStoreRelease(ledger.seqlock, version + 1U);
}

void endLedgerWrite(SharedLossLedger &ledger) noexcept {
  const auto version = atomicLoadAcquire(ledger.seqlock);
  atomicStoreRelease(ledger.seqlock, version + 1U);
}

void recordDrop(SharedLossLedger &shared,
                const FixedControllerSampleKey &key) noexcept {
  beginLedgerWrite(shared);
  auto &ledger = shared.value;
  ++ledger.loss_generation;
  ++ledger.cumulative_drop_count;
  const std::uint64_t sequence = key.controller_sequence;
  if (ledger.global_membership_unknown) {
    endLedgerWrite(shared);
    return;
  }

  if (ledger.range_count > 0U) {
    auto &last = ledger.ranges[ledger.range_count - 1U];
    if (last.last_sequence != std::numeric_limits<std::uint64_t>::max() &&
        last.last_sequence + 1U == sequence) {
      last.last_sequence = sequence;
      last.last_key = key;
      endLedgerWrite(shared);
      return;
    }
  }

  if (ledger.range_count >= kLossRangeCapacity) {
    ledger.global_membership_unknown = true;
    ledger.first_uncertain_sequence = sequence;
    endLedgerWrite(shared);
    return;
  }

  auto &range = ledger.ranges[ledger.range_count++];
  range.first_sequence = sequence;
  range.last_sequence = sequence;
  range.first_key = key;
  range.last_key = key;
  endLedgerWrite(shared);
}

} // namespace

bool hostIsLittleEndian() noexcept {
  const std::uint32_t value = 1U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 1U;
}

bool sharedAtomicsAreLockFree() noexcept {
  alignas(8) std::uint64_t value64 = 0U;
  alignas(4) std::uint32_t value32 = 0U;
  return __atomic_always_lock_free(sizeof(value64), nullptr) &&
         __atomic_always_lock_free(sizeof(value32), nullptr) &&
         __atomic_is_lock_free(sizeof(value64), &value64) &&
         __atomic_is_lock_free(sizeof(value32), &value32);
}

std::uint64_t atomicLoadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void atomicStoreRelease(std::uint64_t &destination,
                        std::uint64_t value) noexcept {
  __atomic_store_n(&destination, value, __ATOMIC_RELEASE);
}

bool initializeSharedRegions(SharedDataRegion &data, SharedAckRegion &ack,
                             std::size_t capacity,
                             std::uint64_t ring_generation,
                             std::uint64_t controller_instance_id) noexcept {
  data = SharedDataRegion{};
  ack = SharedAckRegion{};
  if (!hostIsLittleEndian() || !sharedAtomicsAreLockFree() || capacity == 0U ||
      capacity > kQueueCapacity || ring_generation == 0U ||
      controller_instance_id == 0U) {
    return false;
  }
  data.magic = kSharedMagic;
  data.abi_version = kAbiVersion;
  data.layout_version = kLayoutVersion;
  data.endianness = kLittleEndianMarker;
  data.slot_size = sizeof(SharedSlot);
  data.capacity = static_cast<std::uint32_t>(capacity);
  data.ring_generation = ring_generation;
  data.controller_instance_id = controller_instance_id;
  atomicStoreRelease(data.accepting, 1U);

  ack.magic = kSharedMagic;
  ack.abi_version = kAbiVersion;
  ack.layout_version = kLayoutVersion;
  ack.ring_generation = ring_generation;
  ack.controller_instance_id = controller_instance_id;
  return true;
}

bool validateSharedAbi(const SharedDataRegion &data,
                       const SharedAckRegion &ack) noexcept {
  return hostIsLittleEndian() && sharedAtomicsAreLockFree() &&
         data.magic == kSharedMagic && ack.magic == kSharedMagic &&
         data.abi_version == kAbiVersion && ack.abi_version == kAbiVersion &&
         data.layout_version == kLayoutVersion &&
         ack.layout_version == kLayoutVersion &&
         data.endianness == kLittleEndianMarker &&
         data.slot_size == sizeof(SharedSlot) && data.capacity > 0U &&
         data.capacity <= kQueueCapacity && data.ring_generation != 0U &&
         data.ring_generation == ack.ring_generation &&
         data.controller_instance_id != 0U &&
         data.controller_instance_id == ack.controller_instance_id;
}

bool tryPush(SharedDataRegion &data, const SharedAckRegion &ack,
             const FixedSnapshot &snapshot) noexcept {
  if (!validateSharedAbi(data, ack) ||
      atomicLoadAcquire(data.accepting) == 0U) {
    return false;
  }
  const std::uint64_t write_index = atomicLoadAcquire(data.write_index);
  const std::uint64_t read_index = atomicLoadAcquire(ack.read_index);
  if (write_index < read_index || write_index - read_index >= data.capacity) {
    recordDrop(data.loss_ledger, snapshot.controller_sample_key);
    return false;
  }

  auto &slot = data.slots[write_index % data.capacity];
  slot.payload = snapshot;
  atomicStoreRelease(slot.commit_index, write_index + 1U);
  atomicStoreRelease(data.write_index, write_index + 1U);
  const std::uint64_t depth = write_index + 1U - read_index;
  if (depth > atomicLoadAcquire(data.queue_high_water)) {
    atomicStoreRelease(data.queue_high_water, depth);
  }
  recordAcceptedSequence(data.loss_ledger,
                         snapshot.controller_sample_key.controller_sequence);
  return true;
}

PopResult tryPop(const SharedDataRegion &data, SharedAckRegion &ack,
                 FixedSnapshot &snapshot) noexcept {
  if (!validateSharedAbi(data, ack)) {
    return PopResult::kAbiMismatch;
  }
  const std::uint64_t read_index = atomicLoadAcquire(ack.read_index);
  const std::uint64_t write_index = atomicLoadAcquire(data.write_index);
  if (read_index >= write_index) {
    return PopResult::kEmpty;
  }

  const auto &slot = data.slots[read_index % data.capacity];
  const std::uint64_t expected_commit = read_index + 1U;
  if (atomicLoadAcquire(slot.commit_index) != expected_commit) {
    atomicStoreRelease(ack.torn_slot_count,
                       atomicLoadAcquire(ack.torn_slot_count) + 1U);
    atomicStoreRelease(ack.read_index, expected_commit);
    return PopResult::kTorn;
  }
  snapshot = slot.payload;
  if (atomicLoadAcquire(slot.commit_index) != expected_commit) {
    atomicStoreRelease(ack.torn_slot_count,
                       atomicLoadAcquire(ack.torn_slot_count) + 1U);
    atomicStoreRelease(ack.read_index, expected_commit);
    return PopResult::kTorn;
  }
  atomicStoreRelease(ack.read_index, expected_commit);
  return PopResult::kPopped;
}

void recordAcceptedSequence(SharedLossLedger &shared,
                            std::uint64_t sequence) noexcept {
  beginLedgerWrite(shared);
  shared.value.last_accepted_sequence =
      std::max(shared.value.last_accepted_sequence, sequence);
  endLedgerWrite(shared);
}

bool readLossLedger(const SharedLossLedger &shared,
                    FixedLossLedger &ledger) noexcept {
  for (std::size_t attempt = 0U; attempt < kLedgerReadAttempts; ++attempt) {
    const auto before = atomicLoadAcquire(shared.seqlock);
    if ((before & 1U) != 0U) {
      continue;
    }
    ledger = shared.value;
    const auto after = atomicLoadAcquire(shared.seqlock);
    if (before == after && (after & 1U) == 0U) {
      return true;
    }
  }
  return false;
}

bool sequenceIsDefinitelyLost(const FixedLossLedger &ledger,
                              std::uint64_t sequence) noexcept {
  for (std::size_t index = 0U;
       index < ledger.range_count && index < kLossRangeCapacity; ++index) {
    if (sequence >= ledger.ranges[index].first_sequence &&
        sequence <= ledger.ranges[index].last_sequence) {
      return true;
    }
  }
  return false;
}

FixedSnapshot makeResourceLimitSnapshot(const FixedControllerSampleKey &key,
                                        ResourceLimitKind kind) noexcept {
  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kResourceLimit;
  snapshot.resource_limit_kind = kind;
  snapshot.controller_sample_key = key;
  return snapshot;
}

} // namespace simple_pure_pursuit::aw2_shadow
