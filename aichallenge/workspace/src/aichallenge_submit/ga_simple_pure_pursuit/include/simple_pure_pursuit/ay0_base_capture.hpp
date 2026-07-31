#pragma once

#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace simple_pure_pursuit {

struct Ay0BaseCaptureInput {
  const autoware_auto_planning_msgs::msg::Trajectory *trajectory{nullptr};
  std::size_t nearest_source_index{0U};
  std::uint8_t source_kind{0U};
  std::uint32_t source_generation{0U};
  overtake_transport_contract::c002ay0::FixedTime record_stamp{};
  overtake_transport_contract::c002ay0::FixedTime lease_valid_until{};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t controller_instance_id{0U};
  std::uint64_t controller_sequence{0U};
  std::uint64_t base_lease_id{0U};
  overtake_transport_contract::c002ay0::Digest
      controller_implementation_sha256{};
  overtake_transport_contract::c002ay0::Digest controller_config_sha256{};
};

enum class Ay0BaseCaptureResult : std::uint8_t {
  kBuilt,
  kMissingTrajectory,
  kFrameInvalid,
  kPointLimit,
  kNearestInvalid,
  kIdentityInvalid,
};

Ay0BaseCaptureResult buildAy0FixedBaseRecord(
    const Ay0BaseCaptureInput &input,
    overtake_transport_contract::c002ay0::FixedBaseRecord &record) noexcept;

} // namespace simple_pure_pursuit
