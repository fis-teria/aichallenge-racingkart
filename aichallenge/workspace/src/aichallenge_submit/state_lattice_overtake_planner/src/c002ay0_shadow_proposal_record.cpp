#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp"

#include <type_traits>

namespace state_lattice_overtake_planner::c002ay0_shadow {
namespace {

std::uint64_t loadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t next) noexcept {
  __atomic_store_n(&value, next, __ATOMIC_RELEASE);
}

void incrementRelaxed(std::uint64_t &value) noexcept {
  (void)__atomic_fetch_add(&value, 1U, __ATOMIC_RELAXED);
}

bool validStaticConfig(const FixedProposalStaticConfig &config) noexcept {
  return config.safety_evaluation_enabled == 1U && config.frame_size == 3U &&
         config.frame[0] == 'm' && config.frame[1] == 'a' &&
         config.frame[2] == 'p' && config.wheel_base_m > 0.0 &&
         config.ego_stale_sec > 0.0;
}

bool validQueue(const FixedProposalQueue &queue) noexcept {
  return queue.magic == kFixedProposalQueueMagic &&
         queue.abi_version == kFixedProposalQueueAbiVersion &&
         queue.capacity > 0U && queue.capacity <= kFixedProposalQueueCapacity &&
         queue.session_generation != 0U && queue.session_nonce != 0U &&
         validStaticConfig(queue.static_config) &&
         fixedProposalAtomicsAreLockFree();
}

} // namespace

static_assert(std::is_trivially_copyable_v<FixedProposalRecord>);
static_assert(std::is_standard_layout_v<FixedProposalRecord>);
static_assert(std::is_trivially_copyable_v<FixedProposalQueue>);
static_assert(std::is_standard_layout_v<FixedProposalQueue>);

bool fixedProposalAtomicsAreLockFree() noexcept {
  alignas(8) std::uint64_t value = 0U;
  return __atomic_always_lock_free(sizeof(value), nullptr) &&
         __atomic_is_lock_free(sizeof(value), &value);
}

bool initializeFixedProposalQueue(
    FixedProposalQueue &queue, std::size_t capacity,
    std::uint64_t session_generation, std::uint64_t session_nonce,
    const FixedProposalStaticConfig &config) noexcept {
  queue = FixedProposalQueue{};
  if (!fixedProposalAtomicsAreLockFree() || capacity == 0U ||
      capacity > kFixedProposalQueueCapacity || session_generation == 0U ||
      session_nonce == 0U || !validStaticConfig(config)) {
    return false;
  }
  queue.magic = kFixedProposalQueueMagic;
  queue.abi_version = kFixedProposalQueueAbiVersion;
  queue.capacity = static_cast<std::uint32_t>(capacity);
  queue.session_generation = session_generation;
  queue.session_nonce = session_nonce;
  queue.static_config = config;
  storeRelease(queue.accepting, 1U);
  return true;
}

void disableFixedProposalQueue(FixedProposalQueue &queue) noexcept {
  storeRelease(queue.accepting, 0U);
}

FixedProposalQueueResult
tryPushFixedProposal(FixedProposalQueue &queue,
                     const FixedProposalRecord &record) noexcept {
  if (!validQueue(queue)) {
    return FixedProposalQueueResult::kInvalid;
  }
  if (record.schema_version != 1U ||
      record.capture_monotonic_ns == 0U ||
      record.session_generation != queue.session_generation ||
      record.session_nonce != queue.session_nonce) {
    incrementRelaxed(queue.invalid_record_count);
    return FixedProposalQueueResult::kInvalid;
  }
  if (loadAcquire(queue.accepting) == 0U ||
      loadAcquire(queue.worker_ready) == 0U) {
    return FixedProposalQueueResult::kDisabled;
  }
  const std::uint64_t write_index = loadAcquire(queue.write_index);
  const std::uint64_t read_index = loadAcquire(queue.read_index);
  if (write_index < read_index) {
    incrementRelaxed(queue.invalid_record_count);
    return FixedProposalQueueResult::kInvalid;
  }
  if (write_index - read_index >= queue.capacity) {
    incrementRelaxed(queue.dropped_full_count);
    return FixedProposalQueueResult::kFull;
  }
  auto &slot = queue.slots[write_index % queue.capacity];
  slot.payload = record;
  if (loadAcquire(queue.accepting) == 0U) {
    return FixedProposalQueueResult::kDisabled;
  }
  storeRelease(slot.commit_index, write_index + 1U);
  storeRelease(queue.write_index, write_index + 1U);
  return loadAcquire(queue.accepting) == 0U
             ? FixedProposalQueueResult::kDisabled
             : FixedProposalQueueResult::kAccepted;
}

FixedProposalQueueResult
tryPopFixedProposal(FixedProposalQueue &queue,
                    FixedProposalRecord &record) noexcept {
  if (!validQueue(queue)) {
    return FixedProposalQueueResult::kInvalid;
  }
  const std::uint64_t read_index = loadAcquire(queue.read_index);
  const std::uint64_t write_index = loadAcquire(queue.write_index);
  if (read_index > write_index) {
    return FixedProposalQueueResult::kInvalid;
  }
  if (read_index == write_index) {
    return FixedProposalQueueResult::kEmpty;
  }
  const auto &slot = queue.slots[read_index % queue.capacity];
  const std::uint64_t expected_commit = read_index + 1U;
  if (loadAcquire(slot.commit_index) != expected_commit) {
    storeRelease(queue.read_index, expected_commit);
    return FixedProposalQueueResult::kTorn;
  }
  record = slot.payload;
  if (loadAcquire(slot.commit_index) != expected_commit) {
    return FixedProposalQueueResult::kTorn;
  }
  storeRelease(queue.read_index, expected_commit);
  if (record.schema_version != 1U ||
      record.capture_monotonic_ns == 0U ||
      record.session_generation != queue.session_generation ||
      record.session_nonce != queue.session_nonce) {
    incrementRelaxed(queue.invalid_record_count);
    return FixedProposalQueueResult::kInvalid;
  }
  return FixedProposalQueueResult::kAccepted;
}

} // namespace state_lattice_overtake_planner::c002ay0_shadow
