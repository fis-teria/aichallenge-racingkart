#include "simple_pure_pursuit/aw2_shadow_ipc.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <future>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

namespace simple_pure_pursuit::aw2_shadow {
namespace {

std::string siblingSupervisorPath() {
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
  result += "test_aw2_shadow_exec_consumer";
  return result;
}

std::string siblingExecutablePath(const char *name) {
  auto result = siblingSupervisorPath();
  const auto separator = result.find_last_of('/');
  if (separator == std::string::npos) {
    return {};
  }
  result.resize(separator + 1U);
  result += name;
  return result;
}

bool processExitedOrDisappeared(pid_t pid, std::chrono::milliseconds timeout) {
  const auto exited_or_disappeared = [pid]() {
    errno = 0;
    if (kill(pid, 0) != 0) {
      return errno == ESRCH;
    }
    std::ifstream stat("/proc/" + std::to_string(pid) + "/stat");
    std::string line;
    if (!std::getline(stat, line)) {
      return false;
    }
    const auto command_end = line.rfind(')');
    return command_end != std::string::npos && command_end + 2U < line.size() &&
           line[command_end + 2U] == 'Z';
  };
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (exited_or_disappeared()) {
      return true;
    }
    usleep(1000U);
  } while (std::chrono::steady_clock::now() < deadline);
  return exited_or_disappeared();
}

pid_t waitForChildOf(pid_t parent, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const std::string path = "/proc/" + std::to_string(parent) + "/task/" +
                           std::to_string(parent) + "/children";
  do {
    std::ifstream children(path);
    pid_t child = -1;
    if (children >> child) {
      return child;
    }
    usleep(1000U);
  } while (std::chrono::steady_clock::now() < deadline);
  return -1;
}

TEST(Aw2DirectionalMemfd, EnforcesDataReadOnlyAndAckProducerReadOnly) {
  auto channels = DirectionalMemfdChannels::create(21U, 22U);
  ASSERT_TRUE(channels.has_value());
  ASSERT_NE(channels->data(), nullptr);
  ASSERT_NE(channels->ack(), nullptr);

  EXPECT_EQ(fcntl(channels->shadowDataFd(), F_GETFL) & O_ACCMODE, O_RDONLY);
  EXPECT_EQ(fcntl(channels->shadowAckFd(), F_GETFL) & O_ACCMODE, O_RDWR);
  EXPECT_EQ(fcntl(channels->producerAckFd(), F_GETFL) & O_ACCMODE, O_RDONLY);
  EXPECT_TRUE(channels->validate());

  errno = 0;
  std::uint8_t byte = 1U;
  EXPECT_EQ(pwrite(channels->shadowDataFd(), &byte, 1U, 0), -1);
  EXPECT_EQ(errno, EBADF);
  errno = 0;
  EXPECT_EQ(pwrite(channels->producerAckFd(), &byte, 1U, 0), -1);
  EXPECT_EQ(errno, EBADF);

  void *forbidden =
      mmap(nullptr, sizeof(SharedDataRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, channels->shadowDataFd(), 0);
  EXPECT_EQ(forbidden, MAP_FAILED);
  EXPECT_EQ(errno, EACCES);

  const int seals = fcntl(channels->producerDataFd(), F_GET_SEALS);
  EXPECT_NE(seals & F_SEAL_GROW, 0);
  EXPECT_NE(seals & F_SEAL_SHRINK, 0);
  EXPECT_NE(seals & F_SEAL_SEAL, 0);
}

TEST(Aw2SharedRingProcess, TransfersAcrossForkWithoutTornRead) {
  auto channels = DirectionalMemfdChannels::create(23U, 24U);
  ASSERT_TRUE(channels.has_value());

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    const auto result = consumeOneInChild(channels->shadowDataFd(),
                                          channels->shadowAckFd(), 41U);
    _exit(result ? 0 : 1);
  }

  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kSample;
  snapshot.controller_sample_key.controller_instance_id = 24U;
  snapshot.controller_sample_key.controller_sequence = 41U;
  ASSERT_TRUE(tryPush(*channels->data(), *channels->ack(), snapshot));

  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(Aw2SeqpacketCredentials, RejectsUnexpectedPidBeforeFdTransfer) {
  int sockets[2]{-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets), 0);
  const auto credentials = getPeerCredentials(sockets[0]);
  ASSERT_TRUE(credentials.has_value());
  EXPECT_EQ(credentials->uid, getuid());
  EXPECT_FALSE(
      peerMatchesExpected(*credentials, getpid() + 1, getuid(), getgid()));
  EXPECT_TRUE(peerMatchesExpected(*credentials, getpid(), getuid(), getgid()));
  close(sockets[0]);
  close(sockets[1]);
}

TEST(Aw2PublisherIsolation, AppliesIdleSchedulingAndFixedLimits) {
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    if (!applyPublisherResourceIsolation()) {
      _exit(1);
    }
    struct rlimit address_space {};
    struct rlimit nofile {};
    if (sched_getscheduler(0) != SCHED_IDLE ||
        getrlimit(RLIMIT_AS, &address_space) != 0 ||
        getrlimit(RLIMIT_NOFILE, &nofile) != 0 ||
        address_space.rlim_cur != kPublisherAddressSpaceLimitBytes ||
        nofile.rlim_cur != kPublisherOpenFileLimit) {
      _exit(2);
    }
    _exit(0);
  }
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(Aw2PublisherIsolation,
     ParentDeathBeforePdeathsigSetupFailsClosedWithoutOrphan) {
  int ready_pipe[2]{-1, -1};
  int release_pipe[2]{-1, -1};
  int result_pipe[2]{-1, -1};
  ASSERT_EQ(pipe2(ready_pipe, O_CLOEXEC), 0);
  ASSERT_EQ(pipe2(release_pipe, O_CLOEXEC), 0);
  ASSERT_EQ(pipe2(result_pipe, O_CLOEXEC), 0);
  const pid_t supervisor = fork();
  ASSERT_GE(supervisor, 0);
  if (supervisor == 0) {
    close(ready_pipe[0]);
    close(release_pipe[1]);
    close(result_pipe[0]);
    const pid_t expected_supervisor_pid = getpid();
    const pid_t publisher = fork();
    if (publisher < 0) {
      _exit(2);
    }
    if (publisher == 0) {
      const pid_t publisher_pid = getpid();
      if (write(ready_pipe[1], &publisher_pid, sizeof(publisher_pid)) !=
          static_cast<ssize_t>(sizeof(publisher_pid))) {
        _exit(3);
      }
      std::uint8_t release = 0U;
      if (read(release_pipe[0], &release, sizeof(release)) !=
          static_cast<ssize_t>(sizeof(release))) {
        _exit(4);
      }
      const bool configured =
          applyPublisherParentDeathSignal(expected_supervisor_pid);
      const std::uint8_t result = configured ? 1U : 0U;
      if (write(result_pipe[1], &result, sizeof(result)) !=
          static_cast<ssize_t>(sizeof(result))) {
        _exit(5);
      }
      _exit(configured ? 6 : 0);
    }
    for (;;) {
      pause();
    }
  }

  close(ready_pipe[1]);
  close(release_pipe[0]);
  close(result_pipe[1]);
  pid_t publisher = -1;
  ASSERT_EQ(read(ready_pipe[0], &publisher, sizeof(publisher)),
            static_cast<ssize_t>(sizeof(publisher)));
  close(ready_pipe[0]);
  ASSERT_GT(publisher, 0);
  ASSERT_EQ(kill(supervisor, SIGKILL), 0);
  int supervisor_status = 0;
  ASSERT_EQ(waitpid(supervisor, &supervisor_status, 0), supervisor);
  ASSERT_TRUE(WIFSIGNALED(supervisor_status));
  ASSERT_EQ(WTERMSIG(supervisor_status), SIGKILL);
  const std::uint8_t release = 1U;
  ASSERT_EQ(write(release_pipe[1], &release, sizeof(release)),
            static_cast<ssize_t>(sizeof(release)));
  close(release_pipe[1]);
  std::uint8_t configured = 1U;
  ASSERT_EQ(read(result_pipe[0], &configured, sizeof(configured)),
            static_cast<ssize_t>(sizeof(configured)));
  close(result_pipe[0]);
  EXPECT_EQ(configured, 0U);
  EXPECT_TRUE(processExitedOrDisappeared(publisher, std::chrono::seconds(3)));
}

TEST(Aw2SupervisorShutdown, EscalatesStoppedChildWithoutRespawn) {
  int ready_pipe[2]{-1, -1};
  ASSERT_EQ(pipe2(ready_pipe, O_CLOEXEC), 0);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    close(ready_pipe[0]);
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);
    const std::uint8_t ready = 1U;
    if (write(ready_pipe[1], &ready, sizeof(ready)) !=
        static_cast<ssize_t>(sizeof(ready))) {
      _exit(2);
    }
    close(ready_pipe[1]);
    for (;;) {
      pause();
    }
  }
  close(ready_pipe[1]);
  std::uint8_t ready = 0U;
  ASSERT_EQ(read(ready_pipe[0], &ready, sizeof(ready)),
            static_cast<ssize_t>(sizeof(ready)));
  close(ready_pipe[0]);
  ASSERT_EQ(kill(child, SIGSTOP), 0);
  const auto outcome = terminateOneShotChild(
      child, std::chrono::milliseconds(5), std::chrono::milliseconds(5));
  EXPECT_EQ(outcome, ChildTerminationOutcome::kKilled);
  int status = 0;
  EXPECT_EQ(waitpid(child, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(Aw2SupervisorShutdown, ReapsSignalResponsiveChildGracefully) {
  int ready_pipe[2]{-1, -1};
  ASSERT_EQ(pipe2(ready_pipe, O_CLOEXEC), 0);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    close(ready_pipe[0]);
    signal(SIGINT, SIG_DFL);
    const std::uint8_t ready = 1U;
    if (write(ready_pipe[1], &ready, sizeof(ready)) !=
        static_cast<ssize_t>(sizeof(ready))) {
      _exit(2);
    }
    close(ready_pipe[1]);
    for (;;) {
      pause();
    }
  }
  close(ready_pipe[1]);
  std::uint8_t ready = 0U;
  ASSERT_EQ(read(ready_pipe[0], &ready, sizeof(ready)),
            static_cast<ssize_t>(sizeof(ready)));
  close(ready_pipe[0]);
  const auto outcome = terminateOneShotChild(
      child, std::chrono::milliseconds(50), std::chrono::milliseconds(5));
  EXPECT_EQ(outcome, ChildTerminationOutcome::kGraceful);
  int status = 0;
  EXPECT_EQ(waitpid(child, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(Aw2SupervisorShutdown, DistinguishesCleanForcedAndUnconfirmedCleanup) {
  EXPECT_EQ(supervisorExitCodeForChildTermination(
                ChildTerminationOutcome::kAlreadyExited),
            0);
  EXPECT_EQ(
      supervisorExitCodeForChildTermination(ChildTerminationOutcome::kGraceful),
      0);
  EXPECT_EQ(supervisorExitCodeForChildTermination(
                ChildTerminationOutcome::kTerminated),
            kSupervisorForcedCleanupExitCode);
  EXPECT_EQ(
      supervisorExitCodeForChildTermination(ChildTerminationOutcome::kKilled),
      kSupervisorForcedCleanupExitCode);
  EXPECT_EQ(supervisorExitCodeForChildTermination(
                ChildTerminationOutcome::kKillUnconfirmed),
            kSupervisorUnconfirmedCleanupExitCode);
  EXPECT_EQ(
      supervisorExitCodeForChildTermination(ChildTerminationOutcome::kError),
      kSupervisorUnconfirmedCleanupExitCode);
}

TEST(Aw2SupervisorBootstrap, AuthenticatesExecPeerAndTransfersExactlyTwoFds) {
  const auto executable = siblingSupervisorPath();
  ASSERT_FALSE(executable.empty());
  ASSERT_EQ(access(executable.c_str(), X_OK), 0);
  auto session = ProducerSession::start(executable, 25U, 26U,
                                        std::chrono::milliseconds(1000));
  ASSERT_TRUE(session.has_value());
  std::array<std::uint8_t, 32U> build_digest{};
  std::array<std::uint8_t, 32U> config_digest{};
  build_digest[0] = 1U;
  config_digest[0] = 2U;
  session->setProducerDigests(build_digest, config_digest);
  EXPECT_TRUE(session->valid());
  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kSample;
  snapshot.controller_sample_key.controller_role = kControllerRolePrimary;
  snapshot.controller_sample_key.controller_instance_id = 26U;
  snapshot.controller_sample_key.controller_sequence = 1U;
  snapshot.controller_sample_key.plan_generation = 1U;
  snapshot.record_stamp = FixedTime{1, 2U};
  snapshot.output_command.command_stamp = snapshot.record_stamp;
  snapshot.output_command.lateral_stamp = snapshot.record_stamp;
  snapshot.output_command.longitudinal_stamp = snapshot.record_stamp;
  snapshot.raw_command = snapshot.output_command;
  ASSERT_TRUE(copyFixedString("base_link", snapshot.record_frame_id));
  ASSERT_TRUE(session->tryCapture(snapshot));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (session->consumedCount() == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    usleep(1000U);
  }
  EXPECT_EQ(session->consumedCount(), 1U);
}

TEST(Aw2SupervisorLifetime, BootstrapThreadExitDoesNotKillSupervisorPublisher) {
  const auto executable =
      siblingExecutablePath("controller_applied_shadow_supervisor");
  ASSERT_FALSE(executable.empty());
  ASSERT_EQ(access(executable.c_str(), X_OK), 0);

  std::promise<std::optional<ProducerSession>> promise;
  auto future = promise.get_future();
  std::thread bootstrap_thread([&promise, &executable]() {
    promise.set_value(ProducerSession::start(executable, 201U, 202U,
                                             std::chrono::milliseconds(1000)));
  });
  auto session = future.get();
  bootstrap_thread.join();
  ASSERT_TRUE(session.has_value());

  std::array<std::uint8_t, 32U> build_digest{};
  std::array<std::uint8_t, 32U> config_digest{};
  session->setProducerDigests(build_digest, config_digest);
  FixedSnapshot snapshot{};
  snapshot.kind = SnapshotKind::kSample;
  snapshot.controller_sample_key.controller_role = kControllerRolePrimary;
  snapshot.controller_sample_key.controller_instance_id = 202U;
  snapshot.controller_sample_key.controller_sequence = 1U;
  snapshot.controller_sample_key.plan_generation = 1U;
  snapshot.record_stamp = FixedTime{1, 2U};
  snapshot.output_command.command_stamp = snapshot.record_stamp;
  snapshot.output_command.lateral_stamp = snapshot.record_stamp;
  snapshot.output_command.longitudinal_stamp = snapshot.record_stamp;
  snapshot.raw_command = snapshot.output_command;
  ASSERT_TRUE(copyFixedString("base_link", snapshot.record_frame_id));
  ASSERT_TRUE(session->tryCapture(snapshot));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (session->consumedCount() == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    usleep(1000U);
  }
  EXPECT_EQ(session->consumedCount(), 1U);
}

TEST(Aw2SupervisorLifetime, ProducerExitRemovesSupervisorAndPublisher) {
  const auto executable =
      siblingExecutablePath("controller_applied_shadow_supervisor");
  ASSERT_EQ(access(executable.c_str(), X_OK), 0);
  int pid_pipe[2]{-1, -1};
  ASSERT_EQ(pipe2(pid_pipe, O_CLOEXEC), 0);
  const pid_t producer = fork();
  ASSERT_GE(producer, 0);
  if (producer == 0) {
    close(pid_pipe[0]);
    auto session = ProducerSession::start(executable, 203U, 204U,
                                          std::chrono::milliseconds(1000));
    const pid_t pids[2]{session.has_value() ? session->supervisorPid() : -1,
                        session.has_value()
                            ? waitForChildOf(session->supervisorPid(),
                                             std::chrono::milliseconds(1000))
                            : -1};
    const ssize_t written = write(pid_pipe[1], pids, sizeof(pids));
    close(pid_pipe[1]);
    _exit(session.has_value() && pids[1] > 0 &&
                  written == static_cast<ssize_t>(sizeof(pids))
              ? 0
              : 2);
  }
  close(pid_pipe[1]);
  pid_t pids[2]{-1, -1};
  ASSERT_EQ(read(pid_pipe[0], pids, sizeof(pids)),
            static_cast<ssize_t>(sizeof(pids)));
  close(pid_pipe[0]);
  ASSERT_GT(pids[0], 0);
  ASSERT_GT(pids[1], 0);
  int status = 0;
  ASSERT_EQ(waitpid(producer, &status, 0), producer);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  EXPECT_TRUE(processExitedOrDisappeared(pids[0], std::chrono::seconds(3)));
  EXPECT_TRUE(processExitedOrDisappeared(pids[1], std::chrono::seconds(3)));
}

TEST(Aw2SupervisorBootstrap, TimeoutKillsAndReapsSpawnedSupervisor) {
  const auto executable =
      siblingExecutablePath("test_aw2_shadow_hanging_supervisor");
  ASSERT_EQ(access(executable.c_str(), X_OK), 0);
  BootstrapFailureDetails failure;
  auto session = ProducerSession::start(
      executable, 205U, 206U, std::chrono::milliseconds(10), &failure);
  EXPECT_FALSE(session.has_value());
  EXPECT_EQ(failure.stage, BootstrapFailureStage::kHandshakeTimeout);
  ASSERT_GT(failure.spawned_supervisor_pid, 0);
  EXPECT_EQ(failure.termination_outcome, ChildTerminationOutcome::kKilled);
  int status = 0;
  EXPECT_EQ(waitpid(failure.spawned_supervisor_pid, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(Aw2AsyncBootstrap, ImmediateOwnerDestructionIsLifetimeSafeAndBounded) {
  std::array<std::uint8_t, 32U> build_digest{};
  std::array<std::uint8_t, 32U> config_digest{};
  build_digest[0] = 1U;
  config_digest[0] = 2U;
  for (std::uint64_t instance = 100U; instance < 164U; ++instance) {
    auto session = AsyncProducerSession::start(
        "/definitely/not/an/aw2/supervisor", instance, instance, build_digest,
        config_digest, std::chrono::milliseconds(1));
    ASSERT_NE(session, nullptr);
    session.reset();
  }
  auto late_success = AsyncProducerSession::start(
      siblingSupervisorPath(), 200U, 200U, build_digest, config_digest,
      std::chrono::milliseconds(1000));
  ASSERT_NE(late_success, nullptr);
  late_success.reset();

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (activeAsyncBootstrapCount() != 0U &&
         std::chrono::steady_clock::now() < deadline) {
    usleep(1000U);
  }
  EXPECT_EQ(activeAsyncBootstrapCount(), 0U);
}

} // namespace
} // namespace simple_pure_pursuit::aw2_shadow
