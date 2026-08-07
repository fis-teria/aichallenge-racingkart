#pragma once

#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace overtake_transport_contract::c002ay0 {

constexpr std::size_t kFixedBaseQueueCapacity = 4U;
constexpr std::size_t kFixedFrameCapacity = 128U;
constexpr std::uint8_t kFixedBaseSourceMpcHorizon = 1U;
constexpr std::uint8_t kFixedBaseSourceReferenceTrajectory = 2U;

struct FixedTime {
  std::int32_t sec{0};
  std::uint32_t nanosec{0U};
};

struct FixedPoint {
  FixedTime time_from_start{};
  double position_x_m{0.0};
  double position_y_m{0.0};
  double position_z_m{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  float longitudinal_velocity_mps{0.0F};
  float lateral_velocity_mps{0.0F};
  float acceleration_mps2{0.0F};
  float heading_rate_rps{0.0F};
  float front_wheel_angle_rad{0.0F};
  float rear_wheel_angle_rad{0.0F};
};

struct FixedBaseRecord {
  std::uint32_t schema_version{1U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  FixedTime record_stamp{};
  std::uint16_t frame_size{0U};
  std::array<char, kFixedFrameCapacity> frame{};
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t controller_instance_id{0U};
  std::uint64_t controller_sequence{0U};
  std::uint64_t base_lease_id{0U};
  FixedTime lease_valid_until{};
  std::uint8_t base_source_kind{0U};
  FixedTime base_source_stamp{};
  std::uint32_t base_source_generation{0U};
  std::uint32_t base_original_point_count{0U};
  std::uint32_t nearest_source_index{0U};
  std::uint32_t point_count{0U};
  std::array<FixedPoint, kMaxCartesianPoints> points{};
  Digest controller_implementation_sha256{};
  Digest controller_config_sha256{};
};

struct alignas(64) FixedBaseSlot {
  alignas(8) std::uint64_t commit_index{0U};
  FixedBaseRecord payload{};
};

struct alignas(64) FixedBaseQueue {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t capacity{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  alignas(8) std::uint64_t accepting{0U};
  alignas(8) std::uint64_t write_index{0U};
  alignas(8) std::uint64_t read_index{0U};
  alignas(8) std::uint64_t dropped_count{0U};
  std::array<FixedBaseSlot, kFixedBaseQueueCapacity> slots{};
};

enum class FixedQueueResult : std::uint8_t {
  kAccepted,
  kEmpty,
  kFull,
  kDisabled,
  kInvalid,
  kTorn,
};

bool fixedBaseAtomicsAreLockFree() noexcept;
bool initializeFixedBaseQueue(FixedBaseQueue &queue, std::size_t capacity,
                              std::uint64_t session_generation,
                              std::uint64_t session_nonce) noexcept;
void disableFixedBaseQueue(FixedBaseQueue &queue) noexcept;
FixedQueueResult tryPushFixedBase(FixedBaseQueue &queue,
                                  const FixedBaseRecord &record) noexcept;
FixedQueueResult tryPopFixedBase(FixedBaseQueue &queue,
                                 FixedBaseRecord &record) noexcept;

ValidationError
buildBaseSnapshotFromFixedRecord(const FixedBaseRecord &record,
                                 ControllerBaseTrajectorySnapshot &snapshot);

} // namespace overtake_transport_contract::c002ay0
