#include "c002ay0_fixed_ingress.hpp"
#include "c002ay0_phase1_fixture.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <new>
#include <thread>

#include <gtest/gtest.h>

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
using Ingress = fixture::AuthorizedFixedIngress<3U>;
using PushResult = fixture::IngressPushResult;
using Trajectory = contract::AuthorizedCartesianTrajectory;

constexpr std::size_t kResidentBudgetBytes = 192U * 1024U;

std::uint64_t timeNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

void recanonicalize(Trajectory &trajectory) {
  auto canonical = contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory.geometry_sha256 = canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  canonical = contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory.payload_sha256 = canonical.sha256;
}

enum class WorkerResult {
  VALIDATED_SHADOW,
  EXPIRED_BEFORE_WORKER,
  SAME_GENERATION_MUTATION,
  VALIDATION_FAILED,
};

class DeterministicWorker {
public:
  WorkerResult process(const contract::ControllerBaseTrajectorySnapshot &base,
                       const Trajectory &trajectory,
                       const std::uint64_t now_ns) {
    if (fixture::workerInputExpired(trajectory, now_ns)) {
      return WorkerResult::EXPIRED_BEFORE_WORKER;
    }
    if (contract::validateTrajectoryAgainstBaseSnapshotV1(base, trajectory) !=
        contract::ValidationError::NONE) {
      return WorkerResult::VALIDATION_FAILED;
    }
    const auto ledger_result = ledger_.observe(trajectory);
    if (ledger_result ==
            fixture::MutationLedgerResult::SAME_GENERATION_MUTATION ||
        ledger_result == fixture::MutationLedgerResult::EPOCH_REGRESSION ||
        ledger_result == fixture::MutationLedgerResult::RESOURCE_LIMIT) {
      return WorkerResult::SAME_GENERATION_MUTATION;
    }
    return WorkerResult::VALIDATED_SHADOW;
  }

private:
  fixture::BoundedMutationLedger<3U> ledger_;
};

TEST(C002Ay0FixedIngress, OwnsOneInflightProposalAndRotatesThreeSlots) {
  const auto base = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(base);
  Ingress ingress;
  ASSERT_TRUE(ingress.atomicsLockFree());

  ASSERT_EQ(ingress.tryPush(trajectory, 1U, 100U), PushResult::INSERTED);
  EXPECT_EQ(ingress.tryPush(trajectory, 2U, 101U), PushResult::BUSY);
  Ingress::WorkerView first;
  ASSERT_TRUE(ingress.tryAcquire(first));
  ASSERT_NE(first.trajectory, nullptr);
  EXPECT_EQ(first.enqueue_sequence, 1U);
  EXPECT_EQ(first.trajectory->payload_sha256, trajectory.payload_sha256);
  EXPECT_EQ(ingress.tryPush(trajectory, 3U, 102U), PushResult::BUSY);
  ASSERT_TRUE(ingress.release(first));

  ASSERT_EQ(ingress.tryPush(trajectory, 4U, 103U), PushResult::INSERTED);
  Ingress::WorkerView second;
  ASSERT_TRUE(ingress.tryAcquire(second));
  EXPECT_NE(second.slot_index, first.slot_index);
  ASSERT_TRUE(ingress.release(second));
  EXPECT_EQ(ingress.acceptedCount(), 2U);
  EXPECT_EQ(ingress.busyCount(), 2U);
  EXPECT_FALSE(ingress.outstanding());
}

TEST(C002Ay0FixedIngress, MaximumPayloadCopyAllocatesNothingAfterReserve) {
  const auto base = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(base);
  Ingress ingress;
  const auto allocations_before =
      g_allocation_count.load(std::memory_order_relaxed);
  g_track_allocations.store(true, std::memory_order_relaxed);
  const auto result = ingress.tryPush(trajectory, 1U, 100U);
  g_track_allocations.store(false, std::memory_order_relaxed);
  const auto allocation_delta =
      g_allocation_count.load(std::memory_order_relaxed) - allocations_before;

  EXPECT_EQ(result, PushResult::INSERTED);
  EXPECT_EQ(allocation_delta, 0U);
  Ingress::WorkerView view;
  ASSERT_TRUE(ingress.tryAcquire(view));
  EXPECT_EQ(view.trajectory->points.size(), contract::kMaxCartesianPoints);
  EXPECT_TRUE(ingress.release(view));
}

TEST(C002Ay0FixedIngress, ExistingFourPlusThreeSlotsFitResidentBudget) {
  std::array<contract::ControllerBaseTrajectorySnapshot, 4U> base_slots;
  const auto base = fixture::baseSnapshot();
  for (auto &slot : base_slots) {
    slot.base_points.reserve(contract::kMaxCartesianPoints);
    slot.frame_id.reserve(128U);
    slot = base;
  }
  Ingress ingress;
  std::size_t resident_bytes = sizeof(base_slots);
  for (const auto &slot : base_slots) {
    resident_bytes += slot.base_points.capacity() *
                      sizeof(typename decltype(slot.base_points)::value_type);
    resident_bytes += slot.frame_id.capacity();
  }
  resident_bytes += ingress.estimatedResidentBytes();
  std::cout << "C002AY0_FIXED_INGRESS resident_bytes=" << resident_bytes
            << " budget_bytes=" << kResidentBudgetBytes << '\n';
  EXPECT_LE(resident_bytes, kResidentBudgetBytes);
}

TEST(C002Ay0FixedIngress, WorkerRejectsExpiryAndLatchesMutation) {
  const auto base = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(base);
  DeterministicWorker worker;
  const auto before_expiry = timeNs(trajectory.safety_valid_until) - 1U;
  EXPECT_EQ(worker.process(base, trajectory, before_expiry),
            WorkerResult::VALIDATED_SHADOW);

  auto intervening = trajectory;
  ++intervening.plan_sample_key.plan_generation;
  recanonicalize(intervening);
  EXPECT_EQ(worker.process(base, intervening, before_expiry),
            WorkerResult::VALIDATED_SHADOW);

  auto mutation = trajectory;
  mutation.points[1].longitudinal_velocity_mps = 9.0F;
  recanonicalize(mutation);
  EXPECT_EQ(worker.process(base, mutation, before_expiry),
            WorkerResult::SAME_GENERATION_MUTATION);
  EXPECT_EQ(worker.process(base, trajectory, before_expiry),
            WorkerResult::SAME_GENERATION_MUTATION);

  DeterministicWorker expiry_worker;
  EXPECT_EQ(expiry_worker.process(base, trajectory,
                                  timeNs(trajectory.safety_valid_until)),
            WorkerResult::EXPIRED_BEFORE_WORKER);
}

TEST(C002Ay0FixedIngress, WorkerExpiryAndLedgerBoundariesFailClosed) {
  const auto base = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(base);

  auto base_lease_first = trajectory;
  base_lease_first.base_lease_valid_until.sec = 250;
  base_lease_first.safety_valid_until.sec = 300;
  EXPECT_TRUE(fixture::workerInputExpired(base_lease_first, 250000000000ULL));

  auto safety_first = trajectory;
  safety_first.base_lease_valid_until.sec = 300;
  safety_first.safety_valid_until.sec = 250;
  EXPECT_TRUE(fixture::workerInputExpired(safety_first, 250000000000ULL));

  Ingress ingress;
  ASSERT_EQ(ingress.tryPush(safety_first, 1U, 100U), PushResult::INSERTED);
  Ingress::WorkerView delayed;
  ASSERT_TRUE(ingress.tryAcquire(delayed));
  ASSERT_NE(delayed.trajectory, nullptr);
  EXPECT_TRUE(
      fixture::workerInputExpired(*delayed.trajectory, 250000000000ULL));
  EXPECT_TRUE(ingress.release(delayed));

  fixture::BoundedMutationLedger<1U> ledger;
  EXPECT_EQ(ledger.observe(trajectory),
            fixture::MutationLedgerResult::INSERTED);
  auto second_key = trajectory;
  ++second_key.plan_sample_key.plan_generation;
  EXPECT_EQ(ledger.observe(second_key),
            fixture::MutationLedgerResult::RESOURCE_LIMIT);
  auto prior_epoch = trajectory;
  prior_epoch.plan_sample_key.race_arm_epoch = 0U;
  EXPECT_EQ(ledger.observe(prior_epoch),
            fixture::MutationLedgerResult::EPOCH_REGRESSION);
}

TEST(C002Ay0FixedIngress, StructuralLimitAndShutdownFailClosed) {
  const auto base = fixture::baseSnapshot();
  auto trajectory = fixture::authorizedTrajectory(base);
  Ingress ingress;
  trajectory.authority_eligible = true;
  EXPECT_EQ(ingress.tryPush(trajectory, 1U, 100U),
            PushResult::STRUCTURAL_LIMIT);
  EXPECT_EQ(ingress.structuralLimitCount(), 1U);
  EXPECT_FALSE(ingress.outstanding());

  trajectory.authority_eligible = false;
  ASSERT_EQ(ingress.tryPush(trajectory, 2U, 101U), PushResult::INSERTED);
  ingress.shutdown();
  Ingress::WorkerView discarded;
  EXPECT_FALSE(ingress.tryAcquire(discarded));
  EXPECT_EQ(ingress.shutdownDiscardCount(), 1U);
  EXPECT_FALSE(ingress.outstanding());
  EXPECT_EQ(ingress.tryPush(trajectory, 2U, 101U), PushResult::SHUTDOWN);
  EXPECT_FALSE(ingress.outstanding());

  Ingress worker_owned_ingress;
  ASSERT_EQ(worker_owned_ingress.tryPush(trajectory, 3U, 102U),
            PushResult::INSERTED);
  Ingress::WorkerView worker_owned;
  ASSERT_TRUE(worker_owned_ingress.tryAcquire(worker_owned));
  worker_owned_ingress.shutdown();
  EXPECT_FALSE(worker_owned_ingress.accepting());
  EXPECT_TRUE(worker_owned_ingress.outstanding());
  EXPECT_TRUE(worker_owned_ingress.release(worker_owned));
  EXPECT_FALSE(worker_owned_ingress.outstanding());
  EXPECT_EQ(worker_owned_ingress.tryPush(trajectory, 4U, 103U),
            PushResult::SHUTDOWN);
}

TEST(C002Ay0FixedIngress, ShutdownLinearizesBeforeConcurrentReadyPublish) {
  const auto base = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(base);
  Ingress ingress;
  std::atomic<bool> copy_completed{false};
  std::atomic<bool> allow_ready_publish{false};
  ingress.setPrePublishBarrierForTest(&copy_completed, &allow_ready_publish);

  PushResult producer_result = PushResult::COPY_FAILED;
  std::thread producer(
      [&]() { producer_result = ingress.tryPush(trajectory, 1U, 100U); });
  while (!copy_completed.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  ingress.shutdown();
  allow_ready_publish.store(true, std::memory_order_release);
  producer.join();

  EXPECT_EQ(producer_result, PushResult::SHUTDOWN);
  EXPECT_EQ(ingress.acceptedCount(), 1U);
  EXPECT_EQ(ingress.shutdownDiscardCount(), 1U);
  EXPECT_EQ(ingress.acceptedCount() - ingress.shutdownDiscardCount(), 0U);
  EXPECT_FALSE(ingress.outstanding());
  Ingress::WorkerView view;
  EXPECT_FALSE(ingress.tryAcquire(view));
}

} // namespace
