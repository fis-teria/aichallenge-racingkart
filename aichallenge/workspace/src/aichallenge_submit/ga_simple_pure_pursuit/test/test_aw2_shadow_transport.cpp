#include "simple_pure_pursuit/aw2_shadow_transport.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <string>
#include <type_traits>

namespace {
std::atomic<std::uint64_t> allocation_count{0U};
}

void *operator new(std::size_t size) {
  allocation_count.fetch_add(1U, std::memory_order_relaxed);
  if (void *memory = std::malloc(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void *operator new[](std::size_t size) { return ::operator new(size); }

void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete[](void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t) noexcept {
  std::free(memory);
}

namespace simple_pure_pursuit::aw2_shadow {
namespace {

FixedControllerSampleKey makeKey(std::uint64_t sequence) {
  FixedControllerSampleKey key{};
  key.race_arm_epoch = 4U;
  key.planner_instance_id = 5U;
  key.attempt_id = 6U;
  key.pass_direction = -1;
  key.connector_transaction_id = 7U;
  key.plan_stamp = FixedTime{8, 9U};
  key.plan_generation = 10U;
  key.controller_role = kControllerRolePrimary;
  key.controller_instance_id = 11U;
  key.controller_sequence = sequence;
  key.controller_command_stamp = FixedTime{12, 13U};
  EXPECT_TRUE(copyFixedString("D2", key.target_vehicle_id));
  return key;
}

FixedSnapshot makeSnapshot(std::uint64_t sequence) {
  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kSample;
  snapshot.controller_sample_key = makeKey(sequence);
  snapshot.output_command.longitudinal_speed_mps = static_cast<float>(sequence);
  return snapshot;
}

TEST(Aw2SharedAbi, IsFixedTrivialAlignedAndLockFree) {
  static_assert(std::is_trivially_copyable_v<FixedSnapshot>);
  static_assert(std::is_standard_layout_v<FixedSnapshot>);
  static_assert(std::is_trivially_copyable_v<SharedDataRegion>);
  static_assert(std::is_standard_layout_v<SharedDataRegion>);
  static_assert(std::is_trivially_copyable_v<SharedAckRegion>);
  static_assert(std::is_standard_layout_v<SharedAckRegion>);
  static_assert(
      offsetof(SharedDataRegion, write_index) % alignof(std::uint64_t) == 0U);
  static_assert(
      offsetof(SharedAckRegion, read_index) % alignof(std::uint64_t) == 0U);
  EXPECT_TRUE(sharedAtomicsAreLockFree());
  EXPECT_TRUE(hostIsLittleEndian());
}

TEST(Aw2SharedQueue, RejectsZeroAndSupportsOneAndEight) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  EXPECT_FALSE(initializeSharedRegions(data, ack, 0U, 1U, 2U));

  ASSERT_TRUE(initializeSharedRegions(data, ack, 1U, 1U, 2U));
  EXPECT_EQ(data.capacity, 1U);
  EXPECT_TRUE(tryPush(data, ack, makeSnapshot(1U)));
  EXPECT_FALSE(tryPush(data, ack, makeSnapshot(2U)));

  FixedSnapshot output{};
  EXPECT_EQ(tryPop(data, ack, output), PopResult::kPopped);
  EXPECT_EQ(output.controller_sample_key.controller_sequence, 1U);

  ASSERT_TRUE(initializeSharedRegions(data, ack, 8U, 3U, 4U));
  EXPECT_EQ(data.capacity, 8U);
}

TEST(Aw2SharedQueue, PreservesFifoWrapAndDropsNewWithoutOverwrite) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  ASSERT_TRUE(initializeSharedRegions(data, ack, 8U, 9U, 10U));

  for (std::uint64_t sequence = 1U; sequence <= 8U; ++sequence) {
    ASSERT_TRUE(tryPush(data, ack, makeSnapshot(sequence)));
  }
  EXPECT_FALSE(tryPush(data, ack, makeSnapshot(9U)));

  FixedSnapshot output{};
  for (std::uint64_t sequence = 1U; sequence <= 4U; ++sequence) {
    ASSERT_EQ(tryPop(data, ack, output), PopResult::kPopped);
    EXPECT_EQ(output.controller_sample_key.controller_sequence, sequence);
  }
  for (std::uint64_t sequence = 10U; sequence <= 13U; ++sequence) {
    ASSERT_TRUE(tryPush(data, ack, makeSnapshot(sequence)));
  }
  for (const std::uint64_t sequence :
       std::array<std::uint64_t, 8U>{5U, 6U, 7U, 8U, 10U, 11U, 12U, 13U}) {
    ASSERT_EQ(tryPop(data, ack, output), PopResult::kPopped);
    EXPECT_EQ(output.controller_sample_key.controller_sequence, sequence);
  }
  EXPECT_EQ(tryPop(data, ack, output), PopResult::kEmpty);
}

TEST(Aw2SharedQueue, RejectsPartialAndTornSlots) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  ASSERT_TRUE(initializeSharedRegions(data, ack, 1U, 11U, 12U));

  data.slots[0].payload = makeSnapshot(21U);
  atomicStoreRelease(data.write_index, 1U);
  FixedSnapshot output{};
  EXPECT_EQ(tryPop(data, ack, output), PopResult::kTorn);
  EXPECT_EQ(atomicLoadAcquire(ack.read_index), 1U);

  ASSERT_TRUE(initializeSharedRegions(data, ack, 1U, 13U, 14U));
  ASSERT_TRUE(tryPush(data, ack, makeSnapshot(22U)));
  atomicStoreRelease(data.slots[0].commit_index, 2U);
  EXPECT_EQ(tryPop(data, ack, output), PopResult::kTorn);
}

TEST(Aw2LossLedger, KeepsNoncontinuousRangesAndNeverLosesAcceptedSequence) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  ASSERT_TRUE(initializeSharedRegions(data, ack, 1U, 15U, 16U));
  ASSERT_TRUE(tryPush(data, ack, makeSnapshot(1U)));

  EXPECT_FALSE(tryPush(data, ack, makeSnapshot(10U)));
  recordAcceptedSequence(data.loss_ledger, 11U);
  EXPECT_FALSE(tryPush(data, ack, makeSnapshot(12U)));
  EXPECT_FALSE(tryPush(data, ack, makeSnapshot(13U)));

  FixedLossLedger ledger{};
  ASSERT_TRUE(readLossLedger(data.loss_ledger, ledger));
  ASSERT_EQ(ledger.range_count, 2U);
  EXPECT_EQ(ledger.ranges[0].first_sequence, 10U);
  EXPECT_EQ(ledger.ranges[0].last_sequence, 10U);
  EXPECT_EQ(ledger.ranges[1].first_sequence, 12U);
  EXPECT_EQ(ledger.ranges[1].last_sequence, 13U);
  EXPECT_FALSE(sequenceIsDefinitelyLost(ledger, 11U));
  EXPECT_TRUE(sequenceIsDefinitelyLost(ledger, 12U));
}

TEST(Aw2LossLedger, OverflowBecomesGlobalMembershipUnknown) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  ASSERT_TRUE(initializeSharedRegions(data, ack, 1U, 17U, 18U));
  ASSERT_TRUE(tryPush(data, ack, makeSnapshot(1U)));

  std::uint64_t sequence = 100U;
  for (std::size_t index = 0U; index < kLossRangeCapacity + 1U; ++index) {
    EXPECT_FALSE(tryPush(data, ack, makeSnapshot(sequence)));
    recordAcceptedSequence(data.loss_ledger, sequence + 1U);
    sequence += 2U;
  }

  FixedLossLedger ledger{};
  ASSERT_TRUE(readLossLedger(data.loss_ledger, ledger));
  EXPECT_TRUE(ledger.global_membership_unknown);
  EXPECT_EQ(ledger.first_uncertain_sequence, 100U + 2U * kLossRangeCapacity);
  EXPECT_FALSE(sequenceIsDefinitelyLost(ledger, 101U));
}

TEST(Aw2SharedQueue, ResourceLossRetainsCompleteBoundedSampleKey) {
  const auto resource =
      makeResourceLimitSnapshot(makeKey(30U), ResourceLimitKind::kTarget);
  EXPECT_EQ(resource.kind, SnapshotKind::kResourceLimit);
  EXPECT_EQ(resource.resource_limit_kind, ResourceLimitKind::kTarget);
  EXPECT_EQ(resource.controller_sample_key.target_vehicle_id.size, 2U);
  EXPECT_EQ(
      std::string(resource.controller_sample_key.target_vehicle_id.bytes.data(),
                  resource.controller_sample_key.target_vehicle_id.size),
      "D2");
  EXPECT_EQ(resource.record_frame_id.size, 0U);
  EXPECT_EQ(resource.source_wire_size, 0U);
  EXPECT_EQ(resource.base_geometry.point_count, 0U);
  EXPECT_EQ(resource.applied_geometry.point_count, 0U);
  EXPECT_EQ(resource.rollout_count, 0U);
}

TEST(Aw2SharedQueue, ProducerHotPathAllocatesNothing) {
  SharedDataRegion data{};
  SharedAckRegion ack{};
  ASSERT_TRUE(initializeSharedRegions(data, ack, 8U, 31U, 32U));
  const auto snapshot = makeSnapshot(33U);
  allocation_count.store(0U, std::memory_order_relaxed);
  const bool pushed = tryPush(data, ack, snapshot);
  const auto allocations = allocation_count.load(std::memory_order_relaxed);
  EXPECT_TRUE(pushed);
  EXPECT_EQ(allocations, 0U);
}

} // namespace
} // namespace simple_pure_pursuit::aw2_shadow
