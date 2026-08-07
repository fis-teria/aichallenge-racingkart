#include "c002ay0_phase1_fixture.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <new>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace {
std::atomic<bool> g_track_allocations{false};
std::atomic<std::size_t> g_allocation_count{0U};
} // namespace

void *operator new(const std::size_t size) {
  if (void *memory = std::malloc(size)) {
    if (g_track_allocations.load(std::memory_order_relaxed)) {
      g_allocation_count.fetch_add(1U, std::memory_order_relaxed);
    }
    return memory;
  }
  throw std::bad_alloc();
}

void *operator new[](const std::size_t size) { return ::operator new(size); }

void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete[](void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t) noexcept {
  std::free(memory);
}

namespace {

namespace contract = overtake_transport_contract::c002ay0;
namespace fixture = contract::test_fixture;

constexpr std::size_t kMaxCdrBytes = 24U * 1024U;
constexpr std::size_t kBaseSlotCount = 4U;
constexpr std::size_t kAuthorizedSlotCount = 3U;

std::uint64_t timeNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

template <typename Message> std::size_t serializedSize(const Message &message) {
  rclcpp::Serialization<Message> serializer;
  rclcpp::SerializedMessage serialized(kMaxCdrBytes);
  serializer.serialize_message(&message, &serialized);
  return serialized.size();
}

template <typename Duration>
std::uint64_t percentileNs(std::vector<Duration> samples,
                           const double percentile) {
  std::sort(samples.begin(), samples.end());
  const auto rank = static_cast<std::size_t>(
      std::ceil(percentile * static_cast<double>(samples.size())));
  const auto index =
      std::min(samples.size() - 1U, std::max<std::size_t>(1U, rank) - 1U);
  return static_cast<std::uint64_t>(samples[index].count());
}

enum class InsertResult {
  INSERTED,
  DUPLICATE,
  EXPIRED,
  GENERATION_REGRESSION,
  SAME_GENERATION_MUTATION,
  RESOURCE_LIMIT,
  BASE_UNKNOWN,
  INFLIGHT_LIMIT,
};

struct BaseSlot {
  bool occupied{false};
  std::uint64_t controller_instance_id{0U};
  std::uint8_t source_kind{0U};
  std::uint32_t generation{0U};
  std::uint64_t lease_id{0U};
  std::uint64_t valid_until_ns{0U};
  contract::Digest snapshot_sha256{};
};

struct AuthorizedSlot {
  bool occupied{false};
  std::uint64_t base_lease_id{0U};
  std::uint32_t plan_generation{0U};
  contract::Digest payload_sha256{};
};

class DeterministicShadowSlots {
public:
  InsertResult
  insertBase(const contract::ControllerBaseTrajectorySnapshot &snapshot,
             const std::uint64_t now_ns) {
    expire(now_ns);
    if (now_ns >= timeNs(snapshot.lease_valid_until)) {
      return InsertResult::EXPIRED;
    }
    for (const auto &slot : base_slots_) {
      if (!slot.occupied ||
          slot.controller_instance_id != snapshot.controller_instance_id ||
          slot.source_kind != snapshot.base_source_kind) {
        continue;
      }
      if (slot.generation == snapshot.base_source_generation) {
        return slot.snapshot_sha256 == snapshot.snapshot_sha256
                   ? InsertResult::DUPLICATE
                   : InsertResult::SAME_GENERATION_MUTATION;
      }
      if (snapshot.base_source_generation < slot.generation) {
        return InsertResult::GENERATION_REGRESSION;
      }
    }
    for (auto &slot : base_slots_) {
      if (!slot.occupied) {
        slot = BaseSlot{true,
                        snapshot.controller_instance_id,
                        snapshot.base_source_kind,
                        snapshot.base_source_generation,
                        snapshot.base_lease_id,
                        timeNs(snapshot.lease_valid_until),
                        snapshot.snapshot_sha256};
        return InsertResult::INSERTED;
      }
    }
    return InsertResult::RESOURCE_LIMIT;
  }

  InsertResult
  beginAuthorized(const contract::AuthorizedCartesianTrajectory &trajectory,
                  const std::uint64_t now_ns) {
    expire(now_ns);
    if (inflight_) {
      return InsertResult::INFLIGHT_LIMIT;
    }
    const auto base = std::find_if(
        base_slots_.begin(), base_slots_.end(),
        [&trajectory](const auto &slot) {
          return slot.occupied && slot.lease_id == trajectory.base_lease_id &&
                 slot.generation == trajectory.base_source_generation &&
                 slot.snapshot_sha256 == trajectory.base_snapshot_sha256;
        });
    if (base == base_slots_.end()) {
      return InsertResult::BASE_UNKNOWN;
    }
    for (const auto &slot : authorized_slots_) {
      if (!slot.occupied ||
          slot.plan_generation != trajectory.plan_sample_key.plan_generation) {
        continue;
      }
      return slot.payload_sha256 == trajectory.payload_sha256
                 ? InsertResult::DUPLICATE
                 : InsertResult::SAME_GENERATION_MUTATION;
    }
    const auto empty =
        std::find_if(authorized_slots_.begin(), authorized_slots_.end(),
                     [](const auto &slot) { return !slot.occupied; });
    if (empty == authorized_slots_.end()) {
      return InsertResult::RESOURCE_LIMIT;
    }
    *empty = AuthorizedSlot{true, trajectory.base_lease_id,
                            trajectory.plan_sample_key.plan_generation,
                            trajectory.payload_sha256};
    inflight_ = true;
    return InsertResult::INSERTED;
  }

  void completeAuthorized() { inflight_ = false; }

  std::size_t baseCount() const {
    return static_cast<std::size_t>(
        std::count_if(base_slots_.begin(), base_slots_.end(),
                      [](const auto &slot) { return slot.occupied; }));
  }

  std::size_t authorizedCount() const {
    return static_cast<std::size_t>(
        std::count_if(authorized_slots_.begin(), authorized_slots_.end(),
                      [](const auto &slot) { return slot.occupied; }));
  }

private:
  void expire(const std::uint64_t now_ns) {
    for (auto &base : base_slots_) {
      if (!base.occupied || now_ns < base.valid_until_ns) {
        continue;
      }
      for (auto &authorized : authorized_slots_) {
        if (authorized.occupied && authorized.base_lease_id == base.lease_id) {
          authorized = AuthorizedSlot{};
        }
      }
      base = BaseSlot{};
    }
    if (authorizedCount() == 0U) {
      inflight_ = false;
    }
  }

  std::array<BaseSlot, kBaseSlotCount> base_slots_{};
  std::array<AuthorizedSlot, kAuthorizedSlotCount> authorized_slots_{};
  bool inflight_{false};
};

void recanonicalize(contract::ControllerBaseTrajectorySnapshot &snapshot) {
  const auto canonical = contract::canonicalizeBaseSnapshotV1(snapshot);
  ASSERT_TRUE(canonical.valid());
  snapshot.base_geometry_sha256 = canonical.geometry_sha256;
  snapshot.snapshot_sha256 = canonical.sha256;
}

void recanonicalize(contract::AuthorizedCartesianTrajectory &trajectory) {
  const auto canonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory.geometry_sha256 = canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  const auto with_embedded =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(with_embedded.valid());
  trajectory.payload_sha256 = with_embedded.sha256;
}

TEST(C002Ay0Phase1Measurement, MaximumLegalCdrFits24KiBWireLimit) {
  const auto snapshot = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(snapshot);
  const auto status = fixture::applicationStatus(trajectory);
  ASSERT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::NONE);
  ASSERT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::NONE);
  ASSERT_EQ(contract::validateApplicationStatusAgainstAuthorizedTrajectoryV1(
                trajectory, status),
            contract::ValidationError::NONE);

  const auto snapshot_bytes = serializedSize(snapshot);
  const auto trajectory_bytes = serializedSize(trajectory);
  const auto status_bytes = serializedSize(status);
  EXPECT_LE(snapshot_bytes, kMaxCdrBytes);
  EXPECT_LE(trajectory_bytes, kMaxCdrBytes);
  EXPECT_LE(status_bytes, kMaxCdrBytes);
  std::cout << "C002AY0_PHASE1_CDR"
            << " base_bytes=" << snapshot_bytes
            << " authorized_bytes=" << trajectory_bytes
            << " status_bytes=" << status_bytes << '\n';
}

TEST(C002Ay0Phase1Measurement,
     PreallocatedFourBaseAndThreeAuthorizedPayloadSlotsCopyWithoutAllocation) {
  const auto snapshot = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(snapshot);
  std::array<contract::ControllerBaseTrajectorySnapshot, kBaseSlotCount>
      base_slots;
  std::array<contract::AuthorizedCartesianTrajectory, kAuthorizedSlotCount>
      authorized_slots;
  for (auto &slot : base_slots) {
    slot.base_points.reserve(contract::kMaxCartesianPoints);
    slot.frame_id.reserve(128U);
  }
  for (auto &slot : authorized_slots) {
    slot.points.reserve(contract::kMaxCartesianPoints);
    slot.frame_id.reserve(128U);
    slot.plan_sample_key.target_vehicle_id.reserve(64U);
  }

  const auto allocations_before =
      g_allocation_count.load(std::memory_order_relaxed);
  g_track_allocations.store(true, std::memory_order_relaxed);
  for (auto &slot : base_slots) {
    slot = snapshot;
  }
  for (auto &slot : authorized_slots) {
    slot = trajectory;
  }
  g_track_allocations.store(false, std::memory_order_relaxed);
  const auto allocation_delta =
      g_allocation_count.load(std::memory_order_relaxed) - allocations_before;

  EXPECT_EQ(allocation_delta, 0U);
  for (const auto &slot : base_slots) {
    EXPECT_EQ(slot.snapshot_sha256, snapshot.snapshot_sha256);
    EXPECT_EQ(slot.base_points.size(), contract::kMaxCartesianPoints);
  }
  for (const auto &slot : authorized_slots) {
    EXPECT_EQ(slot.payload_sha256, trajectory.payload_sha256);
    EXPECT_EQ(slot.points.size(), contract::kMaxCartesianPoints);
  }
}

TEST(C002Ay0Phase1Measurement, ComponentPercentilesAreReportedNotCiGated) {
  const auto snapshot = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(snapshot);
  const auto status = fixture::applicationStatus(trajectory);
  constexpr std::size_t kWarmup = 128U;
  constexpr std::size_t kSamples = 2048U;
  using Duration = std::chrono::nanoseconds;
  std::vector<Duration> planner_samples;
  std::vector<Duration> pp_samples;
  planner_samples.reserve(kSamples);
  pp_samples.reserve(kSamples);

  for (std::size_t index = 0U; index < kWarmup + kSamples; ++index) {
    const auto planner_start = std::chrono::steady_clock::now();
    const auto planner_canonical =
        contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
    const auto planner_end = std::chrono::steady_clock::now();
    ASSERT_TRUE(planner_canonical.valid());

    const auto pp_start = std::chrono::steady_clock::now();
    const auto trajectory_error =
        contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot, trajectory);
    const auto status_error =
        contract::validateApplicationStatusAgainstAuthorizedTrajectoryV1(
            trajectory, status);
    const auto pp_end = std::chrono::steady_clock::now();
    ASSERT_EQ(trajectory_error, contract::ValidationError::NONE);
    ASSERT_EQ(status_error, contract::ValidationError::NONE);
    if (index >= kWarmup) {
      planner_samples.push_back(
          std::chrono::duration_cast<Duration>(planner_end - planner_start));
      pp_samples.push_back(
          std::chrono::duration_cast<Duration>(pp_end - pp_start));
    }
  }

  const auto planner_p99_ns = percentileNs(planner_samples, 0.99);
  const auto pp_p999_ns = percentileNs(pp_samples, 0.999);
  EXPECT_GT(planner_p99_ns, 0U);
  EXPECT_GT(pp_p999_ns, 0U);
  std::cout << "C002AY0_PHASE1_COMPONENT"
            << " samples=" << kSamples
            << " planner_prepare_p99_ns=" << planner_p99_ns
            << " pp_contract_p999_ns=" << pp_p999_ns << " ci_time_gate=false\n";
}

TEST(C002Ay0Phase1Measurement,
     InjectedClockEnforcesFourThreeOneSlotsAndLeaseBoundary) {
  DeterministicShadowSlots slots;
  auto snapshot = fixture::baseSnapshot();
  const std::uint64_t before_expiry = timeNs(snapshot.lease_valid_until) - 1U;
  ASSERT_EQ(slots.insertBase(snapshot, before_expiry), InsertResult::INSERTED);
  EXPECT_EQ(slots.insertBase(snapshot, before_expiry), InsertResult::DUPLICATE);

  auto mutated = snapshot;
  mutated.base_source_sha256[0] ^= 0x01U;
  recanonicalize(mutated);
  EXPECT_EQ(slots.insertBase(mutated, before_expiry),
            InsertResult::SAME_GENERATION_MUTATION);

  auto regressed = snapshot;
  --regressed.base_source_generation;
  ++regressed.base_lease_id;
  recanonicalize(regressed);
  EXPECT_EQ(slots.insertBase(regressed, before_expiry),
            InsertResult::GENERATION_REGRESSION);

  for (std::uint32_t offset = 1U; offset < kBaseSlotCount; ++offset) {
    auto next = snapshot;
    next.base_lease_id += offset;
    next.base_source_generation += offset;
    recanonicalize(next);
    EXPECT_EQ(slots.insertBase(next, before_expiry), InsertResult::INSERTED);
  }
  EXPECT_EQ(slots.baseCount(), kBaseSlotCount);
  auto overflow = snapshot;
  overflow.base_lease_id += 10U;
  overflow.base_source_generation += 10U;
  recanonicalize(overflow);
  EXPECT_EQ(slots.insertBase(overflow, before_expiry),
            InsertResult::RESOURCE_LIMIT);

  auto trajectory = fixture::authorizedTrajectory(snapshot);
  ASSERT_EQ(slots.beginAuthorized(trajectory, before_expiry),
            InsertResult::INSERTED);
  EXPECT_EQ(slots.beginAuthorized(trajectory, before_expiry),
            InsertResult::INFLIGHT_LIMIT);
  slots.completeAuthorized();
  EXPECT_EQ(slots.beginAuthorized(trajectory, before_expiry),
            InsertResult::DUPLICATE);
  auto auth_mutated = trajectory;
  auth_mutated.payload_sha256[0] ^= 0x01U;
  EXPECT_EQ(slots.beginAuthorized(auth_mutated, before_expiry),
            InsertResult::SAME_GENERATION_MUTATION);

  for (std::uint32_t offset = 1U; offset < kAuthorizedSlotCount; ++offset) {
    auto next = trajectory;
    next.plan_sample_key.plan_generation += offset;
    next.candidate_revision += offset;
    next.authority_token += offset;
    recanonicalize(next);
    EXPECT_EQ(slots.beginAuthorized(next, before_expiry),
              InsertResult::INSERTED);
    slots.completeAuthorized();
  }
  EXPECT_EQ(slots.authorizedCount(), kAuthorizedSlotCount);
  auto auth_overflow = trajectory;
  auth_overflow.plan_sample_key.plan_generation += 10U;
  auth_overflow.candidate_revision += 10U;
  auth_overflow.authority_token += 10U;
  recanonicalize(auth_overflow);
  EXPECT_EQ(slots.beginAuthorized(auth_overflow, before_expiry),
            InsertResult::RESOURCE_LIMIT);

  EXPECT_EQ(slots.insertBase(overflow, timeNs(snapshot.lease_valid_until)),
            InsertResult::EXPIRED);
  EXPECT_EQ(slots.baseCount(), 0U);
  EXPECT_EQ(slots.authorizedCount(), 0U);
  EXPECT_EQ(
      slots.beginAuthorized(trajectory, timeNs(snapshot.lease_valid_until)),
      InsertResult::BASE_UNKNOWN);
}

} // namespace
