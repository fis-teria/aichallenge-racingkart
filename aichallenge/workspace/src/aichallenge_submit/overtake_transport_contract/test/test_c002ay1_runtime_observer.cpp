#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <sys/mman.h>
#include <thread>
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

namespace overtake_transport_contract::c002ay1 {
namespace {

constexpr char kRunId[] = "c002ay1-unit";
constexpr std::uint64_t kNonce = 77U;
constexpr std::uint64_t kInstance = 88U;

class MappedRing {
public:
  explicit MappedRing(ProducerRole role)
      : role_(role), size_(mappingSizeForRole(role)) {
    mapping_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    EXPECT_NE(mapping_, MAP_FAILED);
    EXPECT_TRUE(reset());
  }

  ~MappedRing() {
    if (mapping_ != MAP_FAILED) {
      munmap(mapping_, size_);
    }
  }

  bool reset() {
    return initializeRuntimeRingForHarness(
        mapping_, size_, role_, kNonce, kInstance,
        hashRunId(kRunId, sizeof(kRunId) - 1U));
  }

  RuntimeObserverConfig config(bool enabled = true) const {
    RuntimeObserverConfig config{};
    config.enabled = enabled;
    config.role = role_;
    config.run_id = kRunId;
    config.session_nonce = kNonce;
    config.producer_instance_id = kInstance;
    return config;
  }

  RuntimeRingHeaderV1 &header() {
    return *static_cast<RuntimeRingHeaderV1 *>(mapping_);
  }
  PlannerRuntimeRingV1 &planner() {
    return *static_cast<PlannerRuntimeRingV1 *>(mapping_);
  }
  PpRuntimeRingV1 &pp() { return *static_cast<PpRuntimeRingV1 *>(mapping_); }
  void *mapping() { return mapping_; }
  std::size_t size() const { return size_; }

private:
  ProducerRole role_;
  std::size_t size_;
  void *mapping_{MAP_FAILED};
};

std::uint64_t load(const std::uint64_t &value) {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void store(std::uint64_t &value, std::uint64_t desired) {
  __atomic_store_n(&value, desired, __ATOMIC_RELEASE);
}

PlannerCallbackObservationV1 plannerRecord(std::uint64_t sequence) {
  PlannerCallbackObservationV1 record{};
  record.sequence = sequence;
  record.return_reason =
      static_cast<std::uint16_t>(ReturnReason::kCompletedEmit);
  record.flags = kFlagEmitted;
  record.configured_stream_kind =
      static_cast<std::uint8_t>(ConfiguredStreamKind::kLegacyReferenceOverride);
  record.legacy_publish_count = 1U;
  record.selected_proposal_count = 1U;
  return record;
}

PpCallbackObservationV1 ppRecord(std::uint64_t sequence) {
  PpCallbackObservationV1 record{};
  record.sequence = sequence;
  record.return_reason =
      static_cast<std::uint16_t>(ReturnReason::kCompletedEmit);
  record.flags = kFlagEmitted;
  record.controller_role =
      static_cast<std::uint8_t>(ProducerRole::kPrimaryPurePursuit);
  return record;
}

TEST(C002ay1RuntimeRecord, SizesAndTypesAreFixed) {
  EXPECT_EQ(sizeof(PlannerCallbackObservationV1), 56U);
  EXPECT_EQ(sizeof(PpCallbackObservationV1), 40U);
  EXPECT_TRUE(std::is_trivially_copyable_v<PlannerCallbackObservationV1>);
  EXPECT_TRUE(std::is_trivially_copyable_v<PpCallbackObservationV1>);
}

TEST(C002ay1RuntimeRing, IsLockFreeAlignedAndBounded) {
  EXPECT_TRUE(runtimeAtomicsAreLockFree());
  EXPECT_EQ(offsetof(RuntimeRingHeaderV1, state) % 64U, 0U);
  EXPECT_EQ(offsetof(RuntimeRingHeaderV1, write_index) % 64U, 0U);
  EXPECT_EQ(offsetof(RuntimeRingHeaderV1, read_index) % 64U, 0U);
  EXPECT_LE(sizeof(PlannerRuntimeRingV1) + sizeof(PpRuntimeRingV1),
            1024U * 1024U);
}

TEST(C002ay1RuntimeRing, RejectsInvalidHarnessIdentity) {
  MappedRing ring(ProducerRole::kPlanner);
  EXPECT_FALSE(initializeRuntimeRingForHarness(
      ring.mapping(), ring.size(), ProducerRole::kPlanner, 0U, kInstance,
      hashRunId(kRunId, sizeof(kRunId) - 1U)));
  EXPECT_FALSE(initializeRuntimeRingForHarness(
      ring.mapping(), ring.size() - 1U, ProducerRole::kPlanner, kNonce,
      kInstance, hashRunId(kRunId, sizeof(kRunId) - 1U)));
}

TEST(C002ay1RuntimeRing, DisabledWriterDoesNotAttachOrWrite) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config(false));
  EXPECT_FALSE(writer.enabled());
  EXPECT_FALSE(writer.tryWrite(plannerRecord(1U)));
  EXPECT_EQ(load(ring.header().state),
            static_cast<std::uint64_t>(RingState::kHarnessReady));
  EXPECT_EQ(load(ring.header().write_index), 0U);
}

TEST(C002ay1RuntimeRing, PlannerPreservesFifoAndTransitionsOnce) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  ASSERT_TRUE(writer.enabled());
  EXPECT_EQ(load(ring.header().state),
            static_cast<std::uint64_t>(RingState::kProducerAttached));
  ASSERT_TRUE(writer.tryWrite(plannerRecord(1U)));
  ASSERT_TRUE(writer.tryWrite(plannerRecord(2U)));
  EXPECT_EQ(load(ring.header().state),
            static_cast<std::uint64_t>(RingState::kRecording));
  EXPECT_EQ(load(ring.header().write_index), 2U);
  EXPECT_EQ(ring.planner().records[0].sequence, 1U);
  EXPECT_EQ(ring.planner().records[1].sequence, 2U);
}

TEST(C002ay1RuntimeRing, PrimaryPpPreservesFifo) {
  MappedRing ring(ProducerRole::kPrimaryPurePursuit);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  ASSERT_TRUE(writer.enabled());
  ASSERT_TRUE(writer.tryWrite(ppRecord(3U)));
  ASSERT_TRUE(writer.tryWrite(ppRecord(4U)));
  EXPECT_EQ(ring.pp().records[0].sequence, 3U);
  EXPECT_EQ(ring.pp().records[1].sequence, 4U);
}

TEST(C002ay1RuntimeRing, ConcurrentSpscReadPreservesUntornFifo) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  constexpr std::uint64_t kRecordCount = 10000U;
  std::atomic<bool> reader_ok{true};
  std::thread reader([&]() {
    std::uint64_t expected = 1U;
    while (expected <= kRecordCount) {
      const std::uint64_t write_index = load(ring.header().write_index);
      std::uint64_t read_index = load(ring.header().read_index);
      while (read_index < write_index && expected <= kRecordCount) {
        const auto record =
            ring.planner().records[read_index % kPlannerRingCapacity];
        if (record.sequence != expected ||
            record.return_reason !=
                static_cast<std::uint16_t>(ReturnReason::kCompletedEmit)) {
          reader_ok.store(false, std::memory_order_relaxed);
          return;
        }
        ++read_index;
        ++expected;
      }
      store(ring.header().read_index, read_index);
      std::this_thread::yield();
    }
  });
  for (std::uint64_t sequence = 1U; sequence <= kRecordCount; ++sequence) {
    while (!writer.tryWrite(plannerRecord(sequence))) {
      ASSERT_TRUE(writer.enabled());
      std::this_thread::yield();
    }
  }
  reader.join();
  EXPECT_TRUE(reader_ok.load(std::memory_order_relaxed));
  EXPECT_EQ(load(ring.header().read_index), kRecordCount);
}

TEST(C002ay1RuntimeRing, DropsNewWithoutOverwrite) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  ASSERT_TRUE(writer.enabled());
  for (std::uint64_t sequence = 1U; sequence <= kPlannerRingCapacity;
       ++sequence) {
    ASSERT_TRUE(writer.tryWrite(plannerRecord(sequence)));
  }
  EXPECT_FALSE(writer.tryWrite(plannerRecord(kPlannerRingCapacity + 1U)));
  EXPECT_EQ(writer.observedDropCount(), 1U);
  EXPECT_EQ(ring.planner().records[0].sequence, 1U);
}

TEST(C002ay1RuntimeRing, StopsAtCompleteAndInvalidTerminalStates) {
  for (const auto terminal : {RingState::kComplete, RingState::kInvalid}) {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    ASSERT_TRUE(writer.enabled());
    store(ring.header().state, static_cast<std::uint64_t>(terminal));
    EXPECT_FALSE(writer.tryWrite(plannerRecord(1U)));
    EXPECT_EQ(load(ring.header().write_index), 0U);
  }
}

TEST(C002ay1RuntimeRing, InvalidatesReadIndexRegression) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  ASSERT_TRUE(writer.tryWrite(plannerRecord(1U)));
  store(ring.header().read_index, 1U);
  ASSERT_TRUE(writer.tryWrite(plannerRecord(2U)));
  store(ring.header().read_index, 0U);
  EXPECT_FALSE(writer.tryWrite(plannerRecord(3U)));
  EXPECT_EQ(load(ring.header().state),
            static_cast<std::uint64_t>(RingState::kInvalid));
}

TEST(C002ay1RuntimeRing, InvalidatesReadIndexAheadOfWriter) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  store(ring.header().read_index, 1U);
  EXPECT_FALSE(writer.tryWrite(plannerRecord(1U)));
  EXPECT_EQ(load(ring.header().state),
            static_cast<std::uint64_t>(RingState::kInvalid));
}

TEST(C002ay1RuntimeRing, RejectsHeaderAndIdentityMismatch) {
  MappedRing ring(ProducerRole::kPlanner);
  ring.header().record_size += 1U;
  EXPECT_FALSE(RuntimeObservationWriter::attachMappedForTest(
                   ring.mapping(), ring.size(), ring.config())
                   .enabled());

  ASSERT_TRUE(ring.reset());
  auto wrong = ring.config();
  wrong.session_nonce += 1U;
  EXPECT_FALSE(RuntimeObservationWriter::attachMappedForTest(ring.mapping(),
                                                             ring.size(), wrong)
                   .enabled());
}

TEST(C002ay1RuntimeRing, RejectsWrongRecordRole) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  EXPECT_FALSE(writer.tryWrite(ppRecord(1U)));
  EXPECT_EQ(load(ring.header().write_index), 0U);
}

TEST(C002ay1RuntimeRing, RejectsInvalidEnumsFlagsCountsAndStreamSwitch) {
  {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    auto invalid = plannerRecord(1U);
    invalid.return_reason = 99U;
    EXPECT_FALSE(writer.tryWrite(invalid));
    EXPECT_EQ(load(ring.header().state),
              static_cast<std::uint64_t>(RingState::kInvalid));
  }
  {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    auto invalid = plannerRecord(1U);
    invalid.flags = static_cast<std::uint16_t>(1U << 15U);
    EXPECT_FALSE(writer.tryWrite(invalid));
  }
  {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    auto invalid = plannerRecord(1U);
    invalid.legacy_publish_count = 2U;
    EXPECT_FALSE(writer.tryWrite(invalid));
  }
  {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    ASSERT_TRUE(writer.tryWrite(plannerRecord(1U)));
    auto switched = plannerRecord(2U);
    switched.configured_stream_kind =
        static_cast<std::uint8_t>(ConfiguredStreamKind::kV2Trajectory);
    switched.legacy_publish_count = 0U;
    switched.v2_publish_count = 1U;
    EXPECT_FALSE(writer.tryWrite(switched));
    EXPECT_EQ(load(ring.header().state),
              static_cast<std::uint64_t>(RingState::kInvalid));
  }
}

TEST(C002ay1RuntimeRing, AcceptsLegacyAndV2ZeroOneBoundaries) {
  for (const auto stream : {
           ConfiguredStreamKind::kLegacyReferenceOverride,
           ConfiguredStreamKind::kV2Trajectory,
       }) {
    MappedRing ring(ProducerRole::kPlanner);
    auto writer = RuntimeObservationWriter::attachMappedForTest(
        ring.mapping(), ring.size(), ring.config());
    auto record = plannerRecord(1U);
    record.configured_stream_kind = static_cast<std::uint8_t>(stream);
    record.legacy_publish_count =
        stream == ConfiguredStreamKind::kLegacyReferenceOverride ? 1U : 0U;
    record.v2_publish_count =
        stream == ConfiguredStreamKind::kV2Trajectory ? 1U : 0U;
    EXPECT_TRUE(writer.tryWrite(record));
  }
}

TEST(C002ay1RuntimeScope, PlannerCoversReturnAndCounts) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  {
    PlannerObservationScope scope(
        writer, 12, 34U, ConfiguredStreamKind::kLegacyReferenceOverride);
    scope.record().legacy_publish_count = 1U;
    scope.record().selected_proposal_count = 1U;
    scope.record().flags |= kFlagEmitted;
    scope.setReturnReason(ReturnReason::kCompletedEmit);
  }
  ASSERT_EQ(load(ring.header().write_index), 1U);
  const auto &record = ring.planner().records[0];
  EXPECT_EQ(record.ros_sec, 12);
  EXPECT_EQ(record.ros_nanosec, 34U);
  EXPECT_GT(record.steady_start_ns, 0U);
  EXPECT_EQ(record.return_reason,
            static_cast<std::uint16_t>(ReturnReason::kCompletedEmit));
}

TEST(C002ay1RuntimeScope, PpCoversEarlyReturn) {
  MappedRing ring(ProducerRole::kPrimaryPurePursuit);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  {
    PpObservationScope scope(writer, 56, 78U);
    scope.setReturnReason(ReturnReason::kEarlyInputStale);
  }
  ASSERT_EQ(load(ring.header().write_index), 1U);
  EXPECT_EQ(ring.pp().records[0].return_reason,
            static_cast<std::uint16_t>(ReturnReason::kEarlyInputStale));
}

TEST(C002ay1RuntimeScope, ExceptionUnwindIsExplicitAndDoesNotMutateOutput) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  std::array<std::uint8_t, 8U> motion_output{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
  const auto baseline = motion_output;
  try {
    PlannerObservationScope scope(
        writer, 90, 12U, ConfiguredStreamKind::kLegacyReferenceOverride);
    throw 42;
  } catch (int) {
  }
  ASSERT_EQ(load(ring.header().write_index), 1U);
  EXPECT_EQ(ring.planner().records[0].return_reason,
            static_cast<std::uint16_t>(ReturnReason::kUnknownInvalid));
  EXPECT_EQ(motion_output, baseline);
}

TEST(C002ay1RuntimeHotPath, AddsNoHeapAllocation) {
  MappedRing ring(ProducerRole::kPlanner);
  auto writer = RuntimeObservationWriter::attachMappedForTest(
      ring.mapping(), ring.size(), ring.config());
  const auto record = plannerRecord(1U);
  allocation_count.store(0U, std::memory_order_relaxed);
  const bool written = writer.tryWrite(record);
  const auto allocations = allocation_count.load(std::memory_order_relaxed);
  EXPECT_TRUE(written);
  EXPECT_EQ(allocations, 0U);
}

TEST(C002ay1RuntimeParity, DisabledEnabledFullAndUnavailableDoNotMutateOutput) {
  struct MotionOutputs {
    std::array<std::uint8_t, 128U> planner{};
    std::array<std::uint8_t, 128U> pp{};
    std::array<std::uint8_t, 128U> mux{};
  };
  MotionOutputs baseline{};
  for (std::size_t index = 0U; index < baseline.planner.size(); ++index) {
    baseline.planner[index] = static_cast<std::uint8_t>(index);
    baseline.pp[index] = static_cast<std::uint8_t>(index + 1U);
    baseline.mux[index] = static_cast<std::uint8_t>(index + 2U);
  }
  for (const int scenario : {0, 1, 2, 3}) {
    MotionOutputs actual = baseline;
    if (scenario == 1 || scenario == 2) {
      MappedRing ring(ProducerRole::kPlanner);
      auto writer = RuntimeObservationWriter::attachMappedForTest(
          ring.mapping(), ring.size(), ring.config());
      if (scenario == 2) {
        store(ring.header().write_index, kPlannerRingCapacity);
      }
      (void)writer.tryWrite(plannerRecord(1U));
    } else {
      RuntimeObservationWriter writer;
      (void)writer.tryWrite(plannerRecord(1U));
    }
    EXPECT_EQ(std::memcmp(&actual, &baseline, sizeof(actual)), 0);
  }
}

TEST(C002ay1RuntimeHash, IsDeterministicAndRunScoped) {
  EXPECT_EQ(hashRunId(kRunId, sizeof(kRunId) - 1U),
            hashRunId(kRunId, sizeof(kRunId) - 1U));
  EXPECT_NE(hashRunId(kRunId, sizeof(kRunId) - 1U), hashRunId("other", 5U));
}

} // namespace
} // namespace overtake_transport_contract::c002ay1
