#pragma once

#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace state_lattice_overtake_planner::c002ay0_shadow {

constexpr std::size_t kFixedProposalQueueCapacity = 2U;
constexpr std::size_t kFixedProposalOpponentCapacity = 8U;
constexpr std::size_t kFixedProposalIdCapacity = 64U;
constexpr std::uint64_t kFixedProposalQueueMagic = 0x534c415930505251ULL;
constexpr std::uint32_t kFixedProposalQueueAbiVersion = 3U;

using FixedTime = overtake_transport_contract::c002ay0::FixedTime;
using Digest = overtake_transport_contract::c002ay0::Digest;
using FixedBaseRecord = overtake_transport_contract::c002ay0::FixedBaseRecord;

struct FixedIncomingBaseProvenance {
  Digest base_source_sha256{};
  Digest base_geometry_sha256{};
  Digest base_snapshot_sha256{};
};

struct FixedProposalPoint {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double s_m{0.0};
  double d_m{0.0};
  double kappa_radpm{0.0};
  double speed_mps{0.0};
  double time_sec{0.0};
};

struct FixedProposalEgo {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double stamp_sec{0.0};
  double speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double curvature_radpm{0.0};
  double frenet_s_m{0.0};
  double frenet_d_m{0.0};
  double frenet_yaw_error_rad{0.0};
  std::uint32_t frenet_segment_index{0U};
  std::uint8_t valid{0U};
  std::uint8_t frenet_valid{0U};
};

struct FixedProposalOpponent {
  std::uint8_t id_size{0U};
  std::array<char, kFixedProposalIdCapacity> id{};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double stamp_sec{0.0};
  double speed_mps{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double sigma_x_m{0.0};
  double sigma_y_m{0.0};
  double uncertainty_x_m{0.0};
  double uncertainty_y_m{0.0};
  double frenet_s_m{0.0};
  double frenet_d_m{0.0};
  double frenet_yaw_error_rad{0.0};
  std::uint32_t frenet_segment_index{0U};
  std::uint8_t valid{0U};
  std::uint8_t frenet_valid{0U};
};

struct FixedProposalStaticConfig {
  std::uint8_t safety_evaluation_enabled{0U};
  std::uint16_t frame_size{0U};
  std::array<char, overtake_transport_contract::c002ay0::kFixedFrameCapacity>
      frame{};
  double wheel_base_m{0.0};
  double ego_stale_sec{0.0};
  Digest evaluator_implementation_sha256{};
  Digest evaluator_config_sha256{};
};

struct FixedProposalRecord {
  std::uint32_t schema_version{1U};
  // CLOCK_MONOTONIC receipt time set at capture.  It is diagnostic-only and
  // never enters the shadow trajectory's ROS-time/provenance contract.
  std::uint64_t capture_monotonic_ns{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  std::uint64_t connector_transaction_id{0U};
  std::uint64_t authority_token{0U};
  std::uint64_t safety_snapshot_id{0U};
  std::uint32_t plan_generation{0U};
  std::uint32_t candidate_revision{0U};
  FixedTime plan_stamp{};
  Digest evaluator_implementation_sha256{};
  Digest evaluator_config_sha256{};
  std::uint8_t target_id_size{0U};
  std::array<char, kFixedProposalIdCapacity> target_id{};
  std::uint16_t candidate_point_count{0U};
  std::uint8_t candidate_feasible{0U};
  double candidate_goal_d_m{0.0};
  std::array<FixedProposalPoint,
             overtake_transport_contract::c002ay0::kMaxCartesianPoints>
      candidate_points{};
  FixedProposalEgo ego{};
  std::uint8_t opponent_count{0U};
  std::array<FixedProposalOpponent, kFixedProposalOpponentCapacity> opponents{};
  FixedBaseRecord base{};
  FixedIncomingBaseProvenance incoming_base{};
};

struct alignas(64) FixedProposalSlot {
  alignas(8) std::uint64_t commit_index{0U};
  FixedProposalRecord payload{};
};

struct alignas(64) FixedProposalQueue {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t capacity{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  FixedProposalStaticConfig static_config{};
  alignas(8) std::uint64_t accepting{0U};
  alignas(8) std::uint64_t worker_ready{0U};
  alignas(8) std::uint64_t write_index{0U};
  alignas(8) std::uint64_t read_index{0U};
  alignas(8) std::uint64_t dropped_full_count{0U};
  alignas(8) std::uint64_t invalid_record_count{0U};
  alignas(8) std::uint64_t validation_reject_count{0U};
  alignas(8) std::uint64_t serialization_reject_count{0U};
  alignas(8) std::uint64_t coalesced_count{0U};
  alignas(8) std::uint64_t published_count{0U};
  alignas(8) std::uint64_t worker_started_monotonic_ns{0U};
  alignas(8) std::uint64_t last_publish_queue_age_ns{0U};
  std::array<FixedProposalSlot, kFixedProposalQueueCapacity> slots{};
};

enum class FixedProposalQueueResult : std::uint8_t {
  kAccepted,
  kEmpty,
  kFull,
  kDisabled,
  kInvalid,
  kTorn,
};

bool fixedProposalAtomicsAreLockFree() noexcept;
bool initializeFixedProposalQueue(
    FixedProposalQueue &queue, std::size_t capacity,
    std::uint64_t session_generation, std::uint64_t session_nonce,
    const FixedProposalStaticConfig &config) noexcept;
void disableFixedProposalQueue(FixedProposalQueue &queue) noexcept;
FixedProposalQueueResult
tryPushFixedProposal(FixedProposalQueue &queue,
                     const FixedProposalRecord &record) noexcept;
FixedProposalQueueResult
tryPopFixedProposal(FixedProposalQueue &queue,
                    FixedProposalRecord &record) noexcept;

} // namespace state_lattice_overtake_planner::c002ay0_shadow
