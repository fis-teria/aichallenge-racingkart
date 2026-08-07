#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>
#include <sched.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <type_traits>
#include <unistd.h>

namespace {

std::atomic<bool> count_allocations{false};
std::atomic<std::uint64_t> allocation_count{0U};

} // namespace

void *operator new(std::size_t size) {
  if (count_allocations.load(std::memory_order_relaxed)) {
    allocation_count.fetch_add(1U, std::memory_order_relaxed);
  }
  if (void *memory = std::malloc(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void *memory) noexcept { std::free(memory); }

void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }

namespace overtake_transport_contract::c002ay0 {
namespace {

constexpr std::uint64_t kSessionGeneration = 41U;
constexpr std::uint64_t kSessionNonce = 43U;

Digest filledDigest(std::uint8_t seed) {
  Digest digest{};
  for (std::size_t index = 0U; index < digest.size(); ++index) {
    digest[index] = static_cast<std::uint8_t>(seed + index);
  }
  return digest;
}

FixedBaseRecord makeRecord(std::uint64_t sequence,
                           std::size_t point_count = 4U) {
  FixedBaseRecord record;
  record.session_generation = kSessionGeneration;
  record.session_nonce = kSessionNonce;
  record.record_stamp = {100, 20000000U};
  record.frame_size = 3U;
  record.frame[0] = 'm';
  record.frame[1] = 'a';
  record.frame[2] = 'p';
  record.race_arm_epoch = 7U;
  record.controller_instance_id = 11U;
  record.controller_sequence = sequence;
  record.base_lease_id = 17U + sequence;
  record.lease_valid_until = {101, 0U};
  record.base_source_kind =
      ControllerBaseTrajectorySnapshot::SOURCE_REFERENCE_TRAJECTORY;
  record.base_source_stamp = {100, 10000000U};
  record.base_source_generation = 19U;
  record.base_original_point_count = static_cast<std::uint32_t>(point_count);
  record.nearest_source_index = 1U;
  record.point_count = static_cast<std::uint32_t>(point_count);
  for (std::size_t index = 0U; index < point_count; ++index) {
    auto &point = record.points[index];
    const std::uint64_t elapsed_ns =
        static_cast<std::uint64_t>(index) * 25000000ULL;
    point.time_from_start = {
        static_cast<std::int32_t>(elapsed_ns / 1000000000ULL),
        static_cast<std::uint32_t>(elapsed_ns % 1000000000ULL)};
    point.position_x_m = 0.25 * static_cast<double>(index);
    point.orientation_w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
  }
  record.controller_implementation_sha256 = filledDigest(0x11U);
  record.controller_config_sha256 = filledDigest(0x31U);
  return record;
}

TEST(C002Ay0FixedRecord, AbiIsFixedLockFreeAndBounded) {
  static_assert(std::is_trivially_copyable_v<FixedBaseRecord>);
  static_assert(std::is_standard_layout_v<FixedBaseRecord>);
  EXPECT_TRUE(fixedBaseAtomicsAreLockFree());
  EXPECT_EQ(FixedBaseRecord{}.points.size(), kMaxCartesianPoints);
  EXPECT_EQ(FixedBaseQueue{}.slots.size(), kFixedBaseQueueCapacity);
}

TEST(C002Ay0FixedRecord, QueuePreservesFifoAndDropsNew) {
  FixedBaseQueue queue;
  ASSERT_TRUE(
      initializeFixedBaseQueue(queue, 2U, kSessionGeneration, kSessionNonce));
  const auto first = makeRecord(1U);
  const auto second = makeRecord(2U);
  const auto dropped = makeRecord(3U);

  EXPECT_EQ(tryPushFixedBase(queue, first), FixedQueueResult::kAccepted);
  EXPECT_EQ(tryPushFixedBase(queue, second), FixedQueueResult::kAccepted);
  EXPECT_EQ(tryPushFixedBase(queue, dropped), FixedQueueResult::kFull);
  EXPECT_EQ(queue.dropped_count, 1U);

  FixedBaseRecord output;
  ASSERT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 1U);
  ASSERT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 2U);
  EXPECT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kEmpty);

  disableFixedBaseQueue(queue);
  EXPECT_EQ(tryPushFixedBase(queue, dropped), FixedQueueResult::kDisabled);

  ASSERT_TRUE(
      initializeFixedBaseQueue(queue, 2U, kSessionGeneration, kSessionNonce));
  queue.read_index = 1U;
  EXPECT_EQ(tryPushFixedBase(queue, first), FixedQueueResult::kInvalid);
  EXPECT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kInvalid);
}

TEST(C002Ay0FixedRecord, DisableRejectsConcurrentAndOldSessionRecords) {
  for (std::size_t iteration = 0U; iteration < 100U; ++iteration) {
    FixedBaseQueue queue;
    ASSERT_TRUE(
        initializeFixedBaseQueue(queue, 1U, kSessionGeneration, kSessionNonce));
    const auto input = makeRecord(iteration + 1U, kMaxCartesianPoints);
    std::atomic<bool> start{false};
    FixedQueueResult producer_result = FixedQueueResult::kInvalid;
    std::thread producer([&]() {
      while (!start.load(std::memory_order_acquire)) {
      }
      producer_result = tryPushFixedBase(queue, input);
    });
    start.store(true, std::memory_order_release);
    disableFixedBaseQueue(queue);
    producer.join();
    EXPECT_TRUE(producer_result == FixedQueueResult::kAccepted ||
                producer_result == FixedQueueResult::kDisabled);
    FixedBaseRecord output;
    EXPECT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kDisabled);
  }

  FixedBaseQueue queue;
  ASSERT_TRUE(initializeFixedBaseQueue(queue, 1U, kSessionGeneration + 1U,
                                       kSessionNonce + 1U));
  EXPECT_EQ(tryPushFixedBase(queue, makeRecord(101U)),
            FixedQueueResult::kInvalid);
}

TEST(C002Ay0FixedRecord, ProducerEnqueueAllocatesNothing) {
  FixedBaseQueue queue;
  ASSERT_TRUE(
      initializeFixedBaseQueue(queue, 1U, kSessionGeneration, kSessionNonce));
  const auto input = makeRecord(1U, kMaxCartesianPoints);
  FixedBaseRecord output;

  ASSERT_EQ(tryPushFixedBase(queue, input), FixedQueueResult::kAccepted);
  ASSERT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kAccepted);

  allocation_count.store(0U, std::memory_order_relaxed);
  count_allocations.store(true, std::memory_order_release);
  const auto push = tryPushFixedBase(queue, input);
  const auto pop = tryPopFixedBase(queue, output);
  count_allocations.store(false, std::memory_order_release);

  EXPECT_EQ(push, FixedQueueResult::kAccepted);
  EXPECT_EQ(pop, FixedQueueResult::kAccepted);
  EXPECT_EQ(allocation_count.load(std::memory_order_relaxed), 0U);
}

TEST(C002Ay0FixedRecord, ProducerMaxRecordP999StaysWithinOneMillisecond) {
  constexpr std::size_t kSamples = 4096U;
  FixedBaseQueue queue;
  ASSERT_TRUE(
      initializeFixedBaseQueue(queue, 1U, kSessionGeneration, kSessionNonce));
  const auto input = makeRecord(1U, kMaxCartesianPoints);
  FixedBaseRecord output;
  std::array<std::uint64_t, kSamples> elapsed_ns{};

  for (std::size_t index = 0U; index < kSamples; ++index) {
    const auto start = std::chrono::steady_clock::now();
    ASSERT_EQ(tryPushFixedBase(queue, input), FixedQueueResult::kAccepted);
    const auto end = std::chrono::steady_clock::now();
    ASSERT_EQ(tryPopFixedBase(queue, output), FixedQueueResult::kAccepted);
    elapsed_ns[index] = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
            .count());
  }
  std::sort(elapsed_ns.begin(), elapsed_ns.end());
  const auto p999_ns = elapsed_ns[(kSamples * 999U) / 1000U];
  std::cout << "C002AY0_FIXED_RECORD size_bytes=" << sizeof(FixedBaseRecord)
            << " queue_bytes=" << sizeof(FixedBaseQueue)
            << " producer_p999_ns=" << p999_ns << '\n';
  EXPECT_LT(p999_ns, 1000000U);
}

TEST(C002Ay0FixedRecord, WorkerBuildsCanonicalShadowSnapshot) {
  const auto record = makeRecord(23U);
  ControllerBaseTrajectorySnapshot snapshot;
  ASSERT_EQ(buildBaseSnapshotFromFixedRecord(record, snapshot),
            ValidationError::NONE);
  EXPECT_FALSE(snapshot.authority_eligible);
  EXPECT_EQ(snapshot.schema_version,
            ControllerBaseTrajectorySnapshot::SCHEMA_V1_SHADOW);
  EXPECT_EQ(snapshot.controller_sequence, record.controller_sequence);
  EXPECT_EQ(snapshot.base_points.size(), record.point_count);
  EXPECT_EQ(snapshot.first_source_index, 0U);
  EXPECT_EQ(snapshot.last_source_index, record.point_count - 1U);
  EXPECT_NE(snapshot.base_source_sha256, Digest{});
  EXPECT_NE(snapshot.base_geometry_sha256, Digest{});
  EXPECT_NE(snapshot.snapshot_sha256, Digest{});
  EXPECT_EQ(validateBaseSnapshotV1(snapshot), ValidationError::NONE);
}

TEST(C002Ay0FixedRecord, SeparateLowPriorityProcessOwnsHeavyWork) {
  void *mapping = mmap(nullptr, sizeof(FixedBaseQueue), PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  ASSERT_NE(mapping, MAP_FAILED);
  auto *queue = new (mapping) FixedBaseQueue;
  ASSERT_TRUE(
      initializeFixedBaseQueue(*queue, 1U, kSessionGeneration, kSessionNonce));

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    struct sched_param parameters {};
    if (sched_setscheduler(0, SCHED_IDLE, &parameters) != 0) {
      _exit(10);
    }
    FixedBaseRecord record;
    for (std::size_t attempt = 0U; attempt < 10000U; ++attempt) {
      const auto result = tryPopFixedBase(*queue, record);
      if (result == FixedQueueResult::kAccepted) {
        ControllerBaseTrajectorySnapshot snapshot;
        if (buildBaseSnapshotFromFixedRecord(record, snapshot) !=
            ValidationError::NONE) {
          _exit(11);
        }
        try {
          rclcpp::Serialization<ControllerBaseTrajectorySnapshot> serializer;
          rclcpp::SerializedMessage serialized(32U * 1024U);
          serializer.serialize_message(&snapshot, &serialized);
          _exit(serialized.size() > 0U ? 0 : 12);
        } catch (...) {
          _exit(13);
        }
      }
      if (result != FixedQueueResult::kEmpty) {
        _exit(14);
      }
      usleep(100U);
    }
    _exit(15);
  }

  EXPECT_EQ(tryPushFixedBase(*queue, makeRecord(29U, kMaxCartesianPoints)),
            FixedQueueResult::kAccepted);
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  queue->~FixedBaseQueue();
  EXPECT_EQ(munmap(mapping, sizeof(FixedBaseQueue)), 0);
}

TEST(C002Ay0FixedRecord, OversizeOrIncompleteSourceFailsClosed) {
  auto record = makeRecord(31U);
  record.base_original_point_count += 1U;
  ControllerBaseTrajectorySnapshot snapshot;
  EXPECT_EQ(buildBaseSnapshotFromFixedRecord(record, snapshot),
            ValidationError::SOURCE_WINDOW_INVALID);

  record = makeRecord(32U);
  record.frame_size = 0U;
  EXPECT_EQ(buildBaseSnapshotFromFixedRecord(record, snapshot),
            ValidationError::FRAME_INVALID);

  record = makeRecord(33U);
  ++record.schema_version;
  EXPECT_EQ(buildBaseSnapshotFromFixedRecord(record, snapshot),
            ValidationError::SCHEMA_UNSUPPORTED);

  record = makeRecord(34U);
  record.points[2].orientation_w = 0.0;
  snapshot.frame_id = "must_be_cleared";
  EXPECT_EQ(buildBaseSnapshotFromFixedRecord(record, snapshot),
            ValidationError::GEOMETRY_POINT_INVALID);
  EXPECT_TRUE(snapshot.frame_id.empty());
  EXPECT_TRUE(snapshot.base_points.empty());
}

} // namespace
} // namespace overtake_transport_contract::c002ay0
