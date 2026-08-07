#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>

namespace overtake_transport_contract::c002ay0 {
namespace {

constexpr std::uint64_t kFixedBaseQueueMagic = 0x433030324159304cULL;
constexpr std::uint32_t kFixedBaseQueueAbiVersion = 1U;

std::uint64_t loadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t next) noexcept {
  __atomic_store_n(&value, next, __ATOMIC_RELEASE);
}

builtin_interfaces::msg::Time toRosTime(const FixedTime &value) {
  builtin_interfaces::msg::Time result;
  result.sec = value.sec;
  result.nanosec = value.nanosec;
  return result;
}

CandidateExecutionPoint toRosPoint(const FixedPoint &value) {
  CandidateExecutionPoint result;
  result.time_from_start.sec = value.time_from_start.sec;
  result.time_from_start.nanosec = value.time_from_start.nanosec;
  result.position_x_m = value.position_x_m;
  result.position_y_m = value.position_y_m;
  result.position_z_m = value.position_z_m;
  result.orientation_x = value.orientation_x;
  result.orientation_y = value.orientation_y;
  result.orientation_z = value.orientation_z;
  result.orientation_w = value.orientation_w;
  result.longitudinal_velocity_mps = value.longitudinal_velocity_mps;
  result.lateral_velocity_mps = value.lateral_velocity_mps;
  result.acceleration_mps2 = value.acceleration_mps2;
  result.heading_rate_rps = value.heading_rate_rps;
  result.front_wheel_angle_rad = value.front_wheel_angle_rad;
  result.rear_wheel_angle_rad = value.rear_wheel_angle_rad;
  return result;
}

bool validQueue(const FixedBaseQueue &queue) noexcept {
  return queue.magic == kFixedBaseQueueMagic &&
         queue.abi_version == kFixedBaseQueueAbiVersion &&
         queue.capacity > 0U && queue.capacity <= kFixedBaseQueueCapacity &&
         queue.session_generation != 0U && queue.session_nonce != 0U &&
         fixedBaseAtomicsAreLockFree();
}

} // namespace

static_assert(std::is_trivially_copyable_v<FixedBaseRecord>);
static_assert(std::is_standard_layout_v<FixedBaseRecord>);

bool fixedBaseAtomicsAreLockFree() noexcept {
  alignas(8) std::uint64_t value = 0U;
  return __atomic_always_lock_free(sizeof(value), nullptr) &&
         __atomic_is_lock_free(sizeof(value), &value);
}

bool initializeFixedBaseQueue(FixedBaseQueue &queue, std::size_t capacity,
                              std::uint64_t session_generation,
                              std::uint64_t session_nonce) noexcept {
  queue = FixedBaseQueue{};
  if (!fixedBaseAtomicsAreLockFree() || capacity == 0U ||
      capacity > kFixedBaseQueueCapacity || session_generation == 0U ||
      session_nonce == 0U) {
    return false;
  }
  queue.magic = kFixedBaseQueueMagic;
  queue.abi_version = kFixedBaseQueueAbiVersion;
  queue.capacity = static_cast<std::uint32_t>(capacity);
  queue.session_generation = session_generation;
  queue.session_nonce = session_nonce;
  storeRelease(queue.accepting, 1U);
  return true;
}

void disableFixedBaseQueue(FixedBaseQueue &queue) noexcept {
  storeRelease(queue.accepting, 0U);
}

FixedQueueResult tryPushFixedBase(FixedBaseQueue &queue,
                                  const FixedBaseRecord &record) noexcept {
  if (!validQueue(queue)) {
    return FixedQueueResult::kInvalid;
  }
  if (record.session_generation != queue.session_generation ||
      record.session_nonce != queue.session_nonce) {
    return FixedQueueResult::kInvalid;
  }
  if (loadAcquire(queue.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  const std::uint64_t write_index = loadAcquire(queue.write_index);
  const std::uint64_t read_index = loadAcquire(queue.read_index);
  if (write_index < read_index) {
    return FixedQueueResult::kInvalid;
  }
  if (write_index - read_index >= queue.capacity) {
    storeRelease(queue.dropped_count, loadAcquire(queue.dropped_count) + 1U);
    return FixedQueueResult::kFull;
  }
  auto &slot = queue.slots[write_index % queue.capacity];
  slot.payload = record;
  if (loadAcquire(queue.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  storeRelease(slot.commit_index, write_index + 1U);
  storeRelease(queue.write_index, write_index + 1U);
  if (loadAcquire(queue.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  return FixedQueueResult::kAccepted;
}

FixedQueueResult tryPopFixedBase(FixedBaseQueue &queue,
                                 FixedBaseRecord &record) noexcept {
  if (!validQueue(queue)) {
    return FixedQueueResult::kInvalid;
  }
  if (loadAcquire(queue.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  const std::uint64_t read_index = loadAcquire(queue.read_index);
  const std::uint64_t write_index = loadAcquire(queue.write_index);
  if (read_index > write_index) {
    return FixedQueueResult::kInvalid;
  }
  if (read_index == write_index) {
    return FixedQueueResult::kEmpty;
  }
  const auto &slot = queue.slots[read_index % queue.capacity];
  const std::uint64_t expected_commit = read_index + 1U;
  if (loadAcquire(slot.commit_index) != expected_commit) {
    storeRelease(queue.read_index, expected_commit);
    return FixedQueueResult::kTorn;
  }
  record = slot.payload;
  if (loadAcquire(slot.commit_index) != expected_commit ||
      loadAcquire(queue.accepting) == 0U) {
    return loadAcquire(queue.accepting) == 0U ? FixedQueueResult::kDisabled
                                              : FixedQueueResult::kTorn;
  }
  if (record.session_generation != queue.session_generation ||
      record.session_nonce != queue.session_nonce) {
    storeRelease(queue.read_index, expected_commit);
    return FixedQueueResult::kInvalid;
  }
  storeRelease(queue.read_index, expected_commit);
  return FixedQueueResult::kAccepted;
}

ValidationError
buildBaseSnapshotFromFixedRecord(const FixedBaseRecord &record,
                                 ControllerBaseTrajectorySnapshot &snapshot) {
  snapshot = ControllerBaseTrajectorySnapshot{};
  if (record.schema_version != 1U) {
    return ValidationError::SCHEMA_UNSUPPORTED;
  }
  if (record.frame_size == 0U || record.frame_size > record.frame.size()) {
    return ValidationError::FRAME_INVALID;
  }
  if (record.point_count < 2U || record.point_count > record.points.size() ||
      record.base_original_point_count != record.point_count ||
      record.nearest_source_index >= record.point_count) {
    return ValidationError::SOURCE_WINDOW_INVALID;
  }

  ControllerBaseTrajectorySnapshot candidate;
  candidate.schema_version = ControllerBaseTrajectorySnapshot::SCHEMA_V1_SHADOW;
  candidate.authority_eligible = false;
  candidate.record_stamp = toRosTime(record.record_stamp);
  candidate.frame_id = std::string(record.frame.data(), record.frame_size);
  candidate.race_arm_epoch = record.race_arm_epoch;
  candidate.controller_instance_id = record.controller_instance_id;
  candidate.controller_sequence = record.controller_sequence;
  candidate.base_lease_id = record.base_lease_id;
  candidate.lease_valid_until = toRosTime(record.lease_valid_until);
  candidate.base_source_kind = record.base_source_kind;
  candidate.base_source_stamp = toRosTime(record.base_source_stamp);
  candidate.base_source_generation = record.base_source_generation;
  candidate.base_original_point_count = record.base_original_point_count;
  candidate.first_source_index = 0U;
  candidate.last_source_index = record.point_count - 1U;
  candidate.nearest_source_index = record.nearest_source_index;
  candidate.base_points.reserve(record.point_count);
  for (std::size_t index = 0U; index < record.point_count; ++index) {
    candidate.base_points.push_back(toRosPoint(record.points[index]));
  }
  candidate.base_source_digest_state =
      ControllerBaseTrajectorySnapshot::BASE_SOURCE_DIGEST_COMPLETE;
  candidate.canonical_algorithm_version = 1U;
  candidate.controller_implementation_sha256 =
      record.controller_implementation_sha256;
  candidate.controller_config_sha256 = record.controller_config_sha256;

  const auto source = canonicalizeBaseSourceV1(
      candidate.base_source_kind, candidate.frame_id,
      candidate.base_source_stamp, candidate.base_source_generation,
      candidate.base_original_point_count, candidate.base_points);
  if (!source.valid()) {
    return source.error;
  }
  candidate.base_source_sha256 = source.sha256;

  const auto canonical = canonicalizeBaseSnapshotV1(candidate);
  if (!canonical.valid()) {
    return canonical.error;
  }
  candidate.base_geometry_sha256 = canonical.geometry_sha256;
  candidate.snapshot_sha256 = canonical.sha256;
  const auto validation = validateBaseSnapshotV1(candidate);
  if (validation != ValidationError::NONE) {
    return validation;
  }
  snapshot = std::move(candidate);
  return ValidationError::NONE;
}

} // namespace overtake_transport_contract::c002ay0
