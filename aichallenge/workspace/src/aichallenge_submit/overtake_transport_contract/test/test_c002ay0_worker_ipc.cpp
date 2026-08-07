#include "overtake_transport_contract/c002ay0_worker_ipc.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace overtake_transport_contract::c002ay0 {
namespace {

std::string siblingWorkerPath() {
  std::array<char, 4096U> path{};
  const ssize_t length =
      readlink("/proc/self/exe", path.data(), path.size() - 1U);
  if (length <= 0 || static_cast<std::size_t>(length) >= path.size()) {
    return {};
  }
  path[static_cast<std::size_t>(length)] = '\0';
  std::string result(path.data());
  const auto separator = result.find_last_of('/');
  if (separator == std::string::npos) {
    return {};
  }
  result.resize(separator + 1U);
  result += "c002ay0_shadow_worker";
  return result;
}

FixedBaseRecord makeRecord(std::uint64_t sequence,
                           std::uint64_t generation = 101U,
                           std::uint64_t nonce = 102U,
                           std::uint64_t instance = 103U) {
  FixedBaseRecord record{};
  record.session_generation = generation;
  record.session_nonce = nonce;
  record.record_stamp = FixedTime{10, 20U};
  constexpr char kFrame[] = "map";
  record.frame_size = sizeof(kFrame) - 1U;
  std::memcpy(record.frame.data(), kFrame, record.frame_size);
  record.race_arm_epoch = 11U;
  record.controller_instance_id = instance;
  record.controller_sequence = sequence;
  record.base_lease_id = 1000U + sequence;
  record.lease_valid_until = FixedTime{11, 0U};
  record.base_source_kind =
      ControllerBaseTrajectorySnapshot::SOURCE_REFERENCE_TRAJECTORY;
  record.base_source_stamp = FixedTime{10, 0U};
  record.base_source_generation = 7U;
  record.base_original_point_count = 2U;
  record.nearest_source_index = 0U;
  record.point_count = 2U;
  for (std::size_t index = 0U; index < record.point_count; ++index) {
    auto &point = record.points[index];
    point.position_x_m = 0.25 * static_cast<double>(index);
    point.orientation_w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
    point.time_from_start.nanosec =
        static_cast<std::uint32_t>(index * 100000000U);
  }
  record.controller_implementation_sha256[0] = 1U;
  record.controller_config_sha256[0] = 2U;
  return record;
}

FixedBaseWorkerConfig makeConfig() {
  FixedBaseWorkerConfig config;
  config.enabled = true;
  config.executable_path = siblingWorkerPath();
  config.session_generation = 101U;
  config.session_nonce = 102U;
  config.controller_instance_id = 103U;
  config.handshake_timeout = std::chrono::milliseconds(1000);
  config.shutdown_budget.drain = std::chrono::milliseconds(250);
  config.shutdown_budget.interrupt = std::chrono::milliseconds(20);
  config.shutdown_budget.terminate = std::chrono::milliseconds(20);
  config.shutdown_budget.kill = std::chrono::milliseconds(100);
  return config;
}

TEST(C002ay0FixedBaseIpc, DirectionalQueuePreservesFifoAndDropsNew) {
  FixedBaseDataRegion data{};
  FixedBaseAckRegion ack{};
  ASSERT_TRUE(initializeFixedBaseIpc(data, ack, 2U, 101U, 102U, 103U));
  ASSERT_TRUE(validateFixedBaseIpc(data, ack));
  data.accepting = 1U;

  EXPECT_EQ(tryPushFixedBaseIpc(data, ack, makeRecord(1U)),
            FixedQueueResult::kAccepted);
  EXPECT_EQ(tryPushFixedBaseIpc(data, ack, makeRecord(2U)),
            FixedQueueResult::kAccepted);
  EXPECT_EQ(tryPushFixedBaseIpc(data, ack, makeRecord(3U)),
            FixedQueueResult::kFull);
  EXPECT_EQ(data.dropped_count, 1U);

  FixedBaseRecord output{};
  EXPECT_EQ(tryPopFixedBaseIpc(data, ack, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 1U);
  EXPECT_EQ(tryPushFixedBaseIpc(data, ack, makeRecord(3U)),
            FixedQueueResult::kAccepted);
  EXPECT_EQ(tryPopFixedBaseIpc(data, ack, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 2U);
  EXPECT_EQ(tryPopFixedBaseIpc(data, ack, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 3U);
}

TEST(C002ay0FixedBaseIpc,
     ClosingProducerAdmissionStillAllowsCommittedRecordsToDrain) {
  FixedBaseDataRegion data{};
  FixedBaseAckRegion ack{};
  ASSERT_TRUE(initializeFixedBaseIpc(data, ack, 2U, 101U, 102U, 103U));
  data.accepting = 1U;
  ASSERT_EQ(tryPushFixedBaseIpc(data, ack, makeRecord(1U)),
            FixedQueueResult::kAccepted);

  disableFixedBaseIpc(data);

  FixedBaseRecord output{};
  EXPECT_EQ(tryPopFixedBaseIpc(data, ack, output), FixedQueueResult::kAccepted);
  EXPECT_EQ(output.controller_sequence, 1U);
  EXPECT_EQ(tryPopFixedBaseIpc(data, ack, output), FixedQueueResult::kEmpty);
}

TEST(C002ay0WorkerLifecycle, DisabledOrUnavailableDoesNotCreateAWorker) {
  FixedBaseWorkerConfig disabled;
  EXPECT_EQ(FixedBaseWorkerSession::start(disabled), nullptr);

  auto unavailable = makeConfig();
  unavailable.executable_path = "/definitely/not/a/worker";
  EXPECT_EQ(FixedBaseWorkerSession::start(unavailable), nullptr);
}

TEST(C002ay0WorkerLifecycle,
     LowPriorityWorkerValidatesAndSerializesOutsideProducer) {
  const auto config = makeConfig();
  ASSERT_FALSE(config.executable_path.empty());
  ASSERT_EQ(access(config.executable_path.c_str(), X_OK), 0);
  auto session = FixedBaseWorkerSession::start(config);
  ASSERT_NE(session, nullptr);
  ASSERT_TRUE(session->ready());
  const auto record = makeRecord(1U);
  ControllerBaseTrajectorySnapshot direct_snapshot;
  ASSERT_EQ(buildBaseSnapshotFromFixedRecord(record, direct_snapshot),
            ValidationError::NONE);
  ASSERT_TRUE(session->tryCapture(record));

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (session->processedCount() < 1U &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(session->processedCount(), 1U);
  EXPECT_EQ(session->droppedCount(), 0U);
  EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kDrained);
}

TEST(C002ay0WorkerLifecycle, GracefulShutdownDrainsCommittedRecordBeforeReap) {
  auto config = makeConfig();
  config.shutdown_budget.drain = std::chrono::milliseconds(500);
  auto session = FixedBaseWorkerSession::start(config);
  ASSERT_NE(session, nullptr);
  ASSERT_TRUE(session->tryCapture(makeRecord(1U)));

  EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kDrained);
  EXPECT_EQ(session->processedCount(), 1U);
  EXPECT_EQ(session->workerPid(), -1);
}

TEST(C002ay0WorkerLifecycle, SigintAndSigtermStopWorkerAndParentReapsIt) {
  for (const int signal_number : {SIGINT, SIGTERM}) {
    auto session = FixedBaseWorkerSession::start(makeConfig());
    ASSERT_NE(session, nullptr);
    const pid_t worker = session->workerPid();
    ASSERT_GT(worker, 1);
    ASSERT_EQ(kill(worker, signal_number), 0);

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (session->ready() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_FALSE(session->ready());
    const auto outcome = session->shutdown();
    EXPECT_TRUE(outcome == WorkerShutdownOutcome::kDrained ||
                outcome == WorkerShutdownOutcome::kAlreadyExited);
    EXPECT_EQ(session->workerPid(), -1);
  }
}

TEST(C002ay0WorkerLifecycle,
     StoppedWorkerIsKilledAndReapedThroughPidfdWithoutRespawn) {
  auto config = makeConfig();
  config.shutdown_budget.drain = std::chrono::milliseconds(5);
  config.shutdown_budget.interrupt = std::chrono::milliseconds(5);
  config.shutdown_budget.terminate = std::chrono::milliseconds(5);
  auto session = FixedBaseWorkerSession::start(config);
  ASSERT_NE(session, nullptr);
  const pid_t worker = session->workerPid();
  ASSERT_GT(worker, 1);
  ASSERT_EQ(kill(worker, SIGSTOP), 0);

  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kKilled);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));
  EXPECT_EQ(session->workerPid(), -1);
  errno = 0;
  EXPECT_EQ(kill(worker, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(C002ay0WorkerLifecycle,
     PidfdUnavailableStartupReapsChildWithoutLeavingFallbackWorker) {
  auto config = makeConfig();
  config.test_force_pidfd_unavailable = true;
  EXPECT_EQ(FixedBaseWorkerSession::start(config), nullptr);

  int status = 0;
  errno = 0;
  EXPECT_EQ(waitpid(-1, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(C002ay0WorkerLifecycle,
     UnconfirmedStartupCleanupReturnsNonReadyOwnerAndRetryReaps) {
  auto config = makeConfig();
  config.test_force_pidfd_unavailable = true;
  config.test_force_initial_cleanup_unconfirmed = true;
  auto session = FixedBaseWorkerSession::start(config);
  ASSERT_NE(session, nullptr);
  EXPECT_FALSE(session->ready());
  EXPECT_EQ(session->shutdownOutcome(),
            WorkerShutdownOutcome::kReapUnconfirmed);
  EXPECT_FALSE(session->tryCapture(makeRecord(1U)));
  const pid_t worker = session->workerPid();
  ASSERT_GT(worker, 1);

  bool exited = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!exited && std::chrono::steady_clock::now() < deadline) {
    siginfo_t info{};
    const int result = waitid(P_PID, static_cast<id_t>(worker), &info,
                              WEXITED | WNOHANG | WNOWAIT);
    ASSERT_EQ(result, 0);
    exited = info.si_pid == worker;
    if (!exited) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_TRUE(exited);

  EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kAlreadyExited);
  EXPECT_EQ(session->workerPid(), -1);
  errno = 0;
  EXPECT_EQ(kill(worker, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(C002ay0WorkerLifecycle,
     UnconfirmedStartupCleanupIsRetriedBySessionDestructor) {
  auto config = makeConfig();
  config.test_force_pidfd_unavailable = true;
  config.test_force_initial_cleanup_unconfirmed = true;
  pid_t worker = -1;
  {
    auto session = FixedBaseWorkerSession::start(config);
    ASSERT_NE(session, nullptr);
    ASSERT_FALSE(session->ready());
    worker = session->workerPid();
    ASSERT_GT(worker, 1);

    bool exited = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!exited && std::chrono::steady_clock::now() < deadline) {
      siginfo_t info{};
      const int result = waitid(P_PID, static_cast<id_t>(worker), &info,
                                WEXITED | WNOHANG | WNOWAIT);
      ASSERT_EQ(result, 0);
      exited = info.si_pid == worker;
      if (!exited) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    ASSERT_TRUE(exited);
  }

  errno = 0;
  EXPECT_EQ(kill(worker, 0), -1);
  EXPECT_EQ(errno, ESRCH);
  int status = 0;
  errno = 0;
  EXPECT_EQ(waitpid(worker, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(C002ay0WorkerLifecycle, RejectsOldSessionRecordWithoutRestart) {
  auto session = FixedBaseWorkerSession::start(makeConfig());
  ASSERT_NE(session, nullptr);
  EXPECT_FALSE(session->tryCapture(makeRecord(1U, 999U, 102U, 103U)));
  EXPECT_EQ(session->processedCount(), 0U);
  EXPECT_TRUE(session->ready());
}

TEST(C002ay0WorkerLifecycle, ShutdownBudgetIsBoundedAndValidated) {
  WorkerShutdownBudget valid;
  EXPECT_TRUE(valid.valid());
  valid.kill = std::chrono::milliseconds(1001);
  EXPECT_FALSE(valid.valid());
  valid.kill = std::chrono::milliseconds(100);
  valid.drain = std::chrono::milliseconds(1000);
  valid.interrupt = std::chrono::milliseconds(1000);
  EXPECT_FALSE(valid.valid());
}

} // namespace
} // namespace overtake_transport_contract::c002ay0
