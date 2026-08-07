#include "overtake_transport_contract/c002ay0_worker_ipc.hpp"
#include "overtake_transport_contract/c002ay0_shadow_binding.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <linux/memfd.h>
#include <optional>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sched.h>
#include <spawn.h>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/random.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>

extern char **environ;

namespace overtake_transport_contract::c002ay0 {
namespace {

constexpr std::uint64_t kWorkerHandshakeMagic = 0x4330303241593057ULL;
constexpr std::uint64_t kWorkerAddressSpaceLimitBytes =
    1024ULL * 1024ULL * 1024ULL;
constexpr rlim_t kWorkerOpenFileLimit = 128U;
constexpr char kTestReliableBindingAuditEnvironment[] =
    "C002AY0_TEST_RELIABLE_BINDING_AUDIT";
constexpr char kTestReliableBindingAuditTopic[] =
    "/test/c002ay0/state_lattice/binding_callback_terminal";

struct WorkerHandshake {
  std::uint64_t magic{0U};
  std::uint64_t nonce_high{0U};
  std::uint64_t nonce_low{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t controller_instance_id{0U};
};

std::uint64_t loadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t next) noexcept {
  __atomic_store_n(&value, next, __ATOMIC_RELEASE);
}

void incrementRelaxed(std::uint64_t &value) noexcept {
  (void)__atomic_fetch_add(&value, 1U, __ATOMIC_RELAXED);
}

bool validRegionHeader(std::uint64_t magic, std::uint32_t abi_version,
                       std::uint32_t capacity, std::uint64_t session_generation,
                       std::uint64_t session_nonce,
                       std::uint64_t controller_instance_id) noexcept {
  return magic == kFixedBaseIpcMagic &&
         abi_version == kFixedBaseIpcAbiVersion && capacity > 0U &&
         capacity <= kFixedBaseQueueCapacity && session_generation != 0U &&
         session_nonce != 0U && controller_instance_id != 0U &&
         fixedBaseAtomicsAreLockFree();
}

void closeFd(int &fd) noexcept {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

template <typename Region> void unmapRegion(Region *&region) noexcept {
  if (region != nullptr && region != MAP_FAILED) {
    munmap(region, sizeof(Region));
  }
  region = nullptr;
}

int createMemfd(const char *name, std::size_t size) noexcept {
  const int fd = static_cast<int>(
      syscall(SYS_memfd_create, name, MFD_CLOEXEC | MFD_ALLOW_SEALING));
  if (fd < 0 || ftruncate(fd, static_cast<off_t>(size)) != 0) {
    if (fd >= 0) {
      ::close(fd);
    }
    return -1;
  }
  return fd;
}

int reopenReadOnly(int fd) noexcept {
  std::array<char, 64U> path{};
  const int length =
      std::snprintf(path.data(), path.size(), "/proc/self/fd/%d", fd);
  if (length <= 0 || static_cast<std::size_t>(length) >= path.size()) {
    return -1;
  }
  return open(path.data(), O_RDONLY | O_CLOEXEC);
}

bool fdModeIs(int fd, int expected_mode) noexcept {
  const int flags = fcntl(fd, F_GETFL);
  return flags >= 0 && (flags & O_ACCMODE) == expected_mode &&
         (fcntl(fd, F_GETFD) & FD_CLOEXEC) != 0;
}

bool fdSizeIs(int fd, std::size_t size) noexcept {
  struct stat status {};
  return fstat(fd, &status) == 0 && status.st_size == static_cast<off_t>(size);
}

bool makeSocketAddress(const std::string &path, sockaddr_un &address) noexcept {
  if (path.empty() || path.size() >= sizeof(address.sun_path)) {
    return false;
  }
  address = {};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);
  return true;
}

bool waitForFd(int fd, short events,
               std::chrono::steady_clock::time_point deadline) noexcept {
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
    pollfd descriptor{fd, events, 0};
    const int result =
        poll(&descriptor, 1, std::max(1, static_cast<int>(remaining.count())));
    if (result > 0) {
      return (descriptor.revents & events) != 0 &&
             (descriptor.revents & (POLLERR | POLLNVAL)) == 0;
    }
    if (result < 0 && errno != EINTR) {
      return false;
    }
  }
  return false;
}

bool sendTwoFds(int socket_fd, int data_fd, int ack_fd) noexcept {
  std::uint8_t payload = 1U;
  iovec io{&payload, sizeof(payload)};
  alignas(cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int) * 2U)>
      control{};
  msghdr message{};
  message.msg_iov = &io;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  cmsghdr *header = CMSG_FIRSTHDR(&message);
  if (header == nullptr) {
    return false;
  }
  header->cmsg_level = SOL_SOCKET;
  header->cmsg_type = SCM_RIGHTS;
  header->cmsg_len = CMSG_LEN(sizeof(int) * 2U);
  const int descriptors[2]{data_fd, ack_fd};
  std::memcpy(CMSG_DATA(header), descriptors, sizeof(descriptors));
  return sendmsg(socket_fd, &message, MSG_NOSIGNAL) ==
         static_cast<ssize_t>(sizeof(payload));
}

void closeReceivedFds(msghdr &message) noexcept {
  for (cmsghdr *header = CMSG_FIRSTHDR(&message); header != nullptr;
       header = CMSG_NXTHDR(&message, header)) {
    if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
        header->cmsg_len < CMSG_LEN(sizeof(int))) {
      continue;
    }
    const std::size_t count = (header->cmsg_len - CMSG_LEN(0U)) / sizeof(int);
    for (std::size_t index = 0U; index < count; ++index) {
      int fd = -1;
      std::memcpy(&fd,
                  reinterpret_cast<const std::byte *>(CMSG_DATA(header)) +
                      index * sizeof(int),
                  sizeof(int));
      if (fd >= 0) {
        ::close(fd);
      }
    }
  }
}

bool receiveTwoFds(int socket_fd, int &data_fd, int &ack_fd) noexcept {
  std::uint8_t payload = 0U;
  iovec io{&payload, sizeof(payload)};
  alignas(cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int) * 2U)>
      control{};
  msghdr message{};
  message.msg_iov = &io;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  const ssize_t received =
      recvmsg(socket_fd, &message, MSG_CMSG_CLOEXEC | MSG_WAITALL);
  cmsghdr *header = CMSG_FIRSTHDR(&message);
  if (received != static_cast<ssize_t>(sizeof(payload)) || payload != 1U ||
      (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
      header == nullptr || CMSG_NXTHDR(&message, header) != nullptr ||
      header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
      header->cmsg_len != CMSG_LEN(sizeof(int) * 2U)) {
    closeReceivedFds(message);
    return false;
  }
  int descriptors[2]{-1, -1};
  std::memcpy(descriptors, CMSG_DATA(header), sizeof(descriptors));
  data_fd = descriptors[0];
  ack_fd = descriptors[1];
  return true;
}

bool peerMatches(int socket_fd, pid_t expected_pid) noexcept {
  ucred credentials{};
  socklen_t size = sizeof(credentials);
  return expected_pid > 0 &&
         getsockopt(socket_fd, SOL_SOCKET, SO_PEERCRED, &credentials, &size) ==
             0 &&
         size == sizeof(credentials) && credentials.pid == expected_pid &&
         credentials.uid == geteuid() && credentials.gid == getegid();
}

int openPidfd(pid_t pid) noexcept {
#ifdef SYS_pidfd_open
  return static_cast<int>(syscall(SYS_pidfd_open, pid, 0U));
#else
  (void)pid;
  errno = ENOSYS;
  return -1;
#endif
}

bool pidfdExited(int pidfd) noexcept {
  pollfd descriptor{pidfd, POLLIN, 0};
  const int result = poll(&descriptor, 1, 0);
  return result == 1 && (descriptor.revents & POLLIN) != 0;
}

bool waitPidfd(int pidfd,
               std::chrono::steady_clock::time_point deadline) noexcept {
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{pidfd, POLLIN, 0};
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
    const int timeout = std::max(
        1, static_cast<int>(std::min<std::int64_t>(1, remaining.count())));
    const int result = poll(&descriptor, 1, timeout);
    if (result == 1 && (descriptor.revents & POLLIN) != 0) {
      return true;
    }
    if (result < 0 && errno != EINTR) {
      return false;
    }
  }
  return pidfdExited(pidfd);
}

bool signalPidfd(int pidfd, int signal_number) noexcept {
#ifdef SYS_pidfd_send_signal
  return syscall(SYS_pidfd_send_signal, pidfd, signal_number, nullptr, 0U) == 0;
#else
  (void)pidfd;
  (void)signal_number;
  errno = ENOSYS;
  return false;
#endif
}

bool reapPidfd(int pidfd, pid_t pid) noexcept {
#ifndef P_PIDFD
  constexpr idtype_t kPidfdIdType = static_cast<idtype_t>(3);
#else
  constexpr idtype_t kPidfdIdType = P_PIDFD;
#endif
  for (;;) {
    siginfo_t info{};
    if (waitid(kPidfdIdType, static_cast<id_t>(pidfd), &info, WEXITED) == 0) {
      return info.si_pid == pid;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == ECHILD) {
      return true;
    }
    break;
  }

  int status = 0;
  for (;;) {
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) {
      return true;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
}

bool waitAndReapChild(pid_t pid,
                      std::chrono::steady_clock::time_point deadline) noexcept {
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) {
      return true;
    }
    if (result < 0 && errno != EINTR) {
      return false;
    }
    (void)poll(nullptr, 0, 1);
  }
  return false;
}

enum class NonBlockingReapResult : std::uint8_t {
  kReaped,
  kRunning,
  kError,
};

NonBlockingReapResult tryReapChild(pid_t pid) noexcept {
  for (;;) {
    int status = 0;
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) {
      return NonBlockingReapResult::kReaped;
    }
    if (result == 0) {
      return NonBlockingReapResult::kRunning;
    }
    if (errno != EINTR) {
      return NonBlockingReapResult::kError;
    }
  }
}

bool validPositiveDecimal(const char *text, std::uint64_t &value) noexcept {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed == 0U) {
    return false;
  }
  value = static_cast<std::uint64_t>(parsed);
  return true;
}

int workerFailure(int code) noexcept { return code; }

} // namespace

static_assert(std::is_trivially_copyable_v<FixedBaseDataRegion>);
static_assert(std::is_trivially_copyable_v<FixedBaseAckRegion>);
static_assert(std::is_standard_layout_v<FixedBaseDataRegion>);
static_assert(std::is_standard_layout_v<FixedBaseAckRegion>);

bool initializeFixedBaseIpc(FixedBaseDataRegion &data, FixedBaseAckRegion &ack,
                            std::size_t capacity,
                            std::uint64_t session_generation,
                            std::uint64_t session_nonce,
                            std::uint64_t controller_instance_id) noexcept {
  data = FixedBaseDataRegion{};
  ack = FixedBaseAckRegion{};
  if (capacity == 0U || capacity > kFixedBaseQueueCapacity ||
      session_generation == 0U || session_nonce == 0U ||
      controller_instance_id == 0U || !fixedBaseAtomicsAreLockFree()) {
    return false;
  }
  data.magic = kFixedBaseIpcMagic;
  data.abi_version = kFixedBaseIpcAbiVersion;
  data.capacity = static_cast<std::uint32_t>(capacity);
  data.session_generation = session_generation;
  data.session_nonce = session_nonce;
  data.controller_instance_id = controller_instance_id;
  ack.magic = kFixedBaseIpcMagic;
  ack.abi_version = kFixedBaseIpcAbiVersion;
  ack.capacity = static_cast<std::uint32_t>(capacity);
  ack.session_generation = session_generation;
  ack.session_nonce = session_nonce;
  ack.controller_instance_id = controller_instance_id;
  return true;
}

bool validateFixedBaseIpc(const FixedBaseDataRegion &data,
                          const FixedBaseAckRegion &ack) noexcept {
  return validRegionHeader(data.magic, data.abi_version, data.capacity,
                           data.session_generation, data.session_nonce,
                           data.controller_instance_id) &&
         validRegionHeader(ack.magic, ack.abi_version, ack.capacity,
                           ack.session_generation, ack.session_nonce,
                           ack.controller_instance_id) &&
         data.capacity == ack.capacity &&
         data.session_generation == ack.session_generation &&
         data.session_nonce == ack.session_nonce &&
         data.controller_instance_id == ack.controller_instance_id;
}

void disableFixedBaseIpc(FixedBaseDataRegion &data) noexcept {
  storeRelease(data.accepting, 0U);
}

FixedQueueResult tryPushFixedBaseIpc(FixedBaseDataRegion &data,
                                     const FixedBaseAckRegion &ack,
                                     const FixedBaseRecord &record) noexcept {
  if (!validateFixedBaseIpc(data, ack) ||
      record.session_generation != data.session_generation ||
      record.session_nonce != data.session_nonce ||
      record.controller_instance_id != data.controller_instance_id) {
    return FixedQueueResult::kInvalid;
  }
  if (loadAcquire(data.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  const std::uint64_t write_index = loadAcquire(data.write_index);
  const std::uint64_t read_index = loadAcquire(ack.read_index);
  if (write_index < read_index) {
    return FixedQueueResult::kInvalid;
  }
  if (write_index - read_index >= data.capacity) {
    incrementRelaxed(data.dropped_count);
    return FixedQueueResult::kFull;
  }
  auto &slot = data.slots[write_index % data.capacity];
  slot.payload = record;
  if (loadAcquire(data.accepting) == 0U) {
    return FixedQueueResult::kDisabled;
  }
  storeRelease(slot.commit_index, write_index + 1U);
  storeRelease(data.write_index, write_index + 1U);
  return loadAcquire(data.accepting) == 0U ? FixedQueueResult::kDisabled
                                           : FixedQueueResult::kAccepted;
}

FixedQueueResult tryPopFixedBaseIpc(const FixedBaseDataRegion &data,
                                    FixedBaseAckRegion &ack,
                                    FixedBaseRecord &record) noexcept {
  if (!validateFixedBaseIpc(data, ack)) {
    return FixedQueueResult::kInvalid;
  }
  const std::uint64_t read_index = loadAcquire(ack.read_index);
  const std::uint64_t write_index = loadAcquire(data.write_index);
  if (read_index > write_index) {
    return FixedQueueResult::kInvalid;
  }
  if (read_index == write_index) {
    return FixedQueueResult::kEmpty;
  }
  const auto &slot = data.slots[read_index % data.capacity];
  const std::uint64_t expected_commit = read_index + 1U;
  if (loadAcquire(slot.commit_index) != expected_commit) {
    incrementRelaxed(ack.torn_count);
    storeRelease(ack.read_index, expected_commit);
    return FixedQueueResult::kTorn;
  }
  record = slot.payload;
  if (loadAcquire(slot.commit_index) != expected_commit) {
    return FixedQueueResult::kTorn;
  }
  if (record.session_generation != data.session_generation ||
      record.session_nonce != data.session_nonce ||
      record.controller_instance_id != data.controller_instance_id) {
    storeRelease(ack.read_index, expected_commit);
    return FixedQueueResult::kInvalid;
  }
  storeRelease(ack.read_index, expected_commit);
  return FixedQueueResult::kAccepted;
}

bool WorkerShutdownBudget::valid() const noexcept {
  const auto each_valid = [](std::chrono::milliseconds value) {
    return value.count() > 0 && value.count() <= 1000;
  };
  return each_valid(drain) && each_valid(interrupt) && each_valid(terminate) &&
         each_valid(kill) &&
         drain + interrupt + terminate + kill <=
             std::chrono::milliseconds(2000);
}

struct FixedBaseWorkerSession::Impl {
  int producer_data_fd{-1};
  int producer_ack_fd{-1};
  int child_data_fd{-1};
  int child_ack_fd{-1};
  int control_socket_fd{-1};
  FixedBaseDataRegion *data{nullptr};
  FixedBaseAckRegion *ack{nullptr};
  std::string socket_directory;
  std::string socket_path;

  ~Impl() {
    unmapRegion(data);
    unmapRegion(ack);
    closeFd(producer_data_fd);
    closeFd(producer_ack_fd);
    closeFd(child_data_fd);
    closeFd(child_ack_fd);
    closeFd(control_socket_fd);
    if (!socket_path.empty()) {
      unlink(socket_path.c_str());
    }
    if (!socket_directory.empty()) {
      rmdir(socket_directory.c_str());
    }
  }
};

std::unique_ptr<FixedBaseWorkerSession>
FixedBaseWorkerSession::start(const FixedBaseWorkerConfig &config) noexcept {
  if (!config.enabled || config.executable_path.empty() ||
      config.session_generation == 0U || config.session_nonce == 0U ||
      config.controller_instance_id == 0U ||
      config.handshake_timeout.count() <= 0 ||
      config.handshake_timeout.count() > 5000 ||
      !config.shutdown_budget.valid()) {
    return nullptr;
  }

  auto session =
      std::unique_ptr<FixedBaseWorkerSession>(new FixedBaseWorkerSession);
  session->impl_ = std::make_unique<Impl>();
  session->shutdown_budget_ = config.shutdown_budget;
  auto &impl = *session->impl_;

  impl.producer_data_fd =
      createMemfd("c002ay0-base-data", sizeof(FixedBaseDataRegion));
  const int ack_write_fd =
      createMemfd("c002ay0-base-ack", sizeof(FixedBaseAckRegion));
  if (impl.producer_data_fd < 0 || ack_write_fd < 0) {
    if (ack_write_fd >= 0) {
      ::close(ack_write_fd);
    }
    return nullptr;
  }
  impl.data = static_cast<FixedBaseDataRegion *>(
      mmap(nullptr, sizeof(FixedBaseDataRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, impl.producer_data_fd, 0));
  auto *ack_write_mapping = static_cast<FixedBaseAckRegion *>(
      mmap(nullptr, sizeof(FixedBaseAckRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, ack_write_fd, 0));
  if (impl.data == MAP_FAILED || ack_write_mapping == MAP_FAILED) {
    impl.data = nullptr;
    if (ack_write_mapping != MAP_FAILED) {
      munmap(ack_write_mapping, sizeof(FixedBaseAckRegion));
    }
    ::close(ack_write_fd);
    return nullptr;
  }
  if (!initializeFixedBaseIpc(*impl.data, *ack_write_mapping,
                              kFixedBaseQueueCapacity,
                              config.session_generation, config.session_nonce,
                              config.controller_instance_id)) {
    munmap(ack_write_mapping, sizeof(FixedBaseAckRegion));
    ::close(ack_write_fd);
    return nullptr;
  }

  impl.child_data_fd = reopenReadOnly(impl.producer_data_fd);
  impl.producer_ack_fd = reopenReadOnly(ack_write_fd);
  impl.child_ack_fd = fcntl(ack_write_fd, F_DUPFD_CLOEXEC, 3);
  if (impl.child_data_fd < 0 || impl.producer_ack_fd < 0 ||
      impl.child_ack_fd < 0) {
    munmap(ack_write_mapping, sizeof(FixedBaseAckRegion));
    ::close(ack_write_fd);
    return nullptr;
  }
  impl.ack = static_cast<FixedBaseAckRegion *>(
      mmap(nullptr, sizeof(FixedBaseAckRegion), PROT_READ, MAP_SHARED,
           impl.producer_ack_fd, 0));
  munmap(ack_write_mapping, sizeof(FixedBaseAckRegion));
  ::close(ack_write_fd);
  if (impl.ack == MAP_FAILED) {
    impl.ack = nullptr;
    return nullptr;
  }

  int data_seals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
#ifdef F_SEAL_FUTURE_WRITE
  data_seals |= F_SEAL_FUTURE_WRITE;
#endif
  const int ack_seals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (fcntl(impl.producer_data_fd, F_ADD_SEALS, data_seals) != 0 ||
      fcntl(impl.child_ack_fd, F_ADD_SEALS, ack_seals) != 0 ||
      !fdModeIs(impl.producer_data_fd, O_RDWR) ||
      !fdModeIs(impl.child_data_fd, O_RDONLY) ||
      !fdModeIs(impl.producer_ack_fd, O_RDONLY) ||
      !fdModeIs(impl.child_ack_fd, O_RDWR) ||
      !fdSizeIs(impl.producer_data_fd, sizeof(FixedBaseDataRegion)) ||
      !fdSizeIs(impl.producer_ack_fd, sizeof(FixedBaseAckRegion))) {
    return nullptr;
  }

  std::array<std::uint64_t, 2U> nonce{};
  if (getrandom(nonce.data(), sizeof(nonce), 0) !=
      static_cast<ssize_t>(sizeof(nonce))) {
    return nullptr;
  }
  std::array<char, 112U> directory{};
  const int directory_length = std::snprintf(
      directory.data(), directory.size(), "/tmp/c002ay0-worker-%u-%d-%016lx",
      static_cast<unsigned>(geteuid()), static_cast<int>(getpid()),
      static_cast<unsigned long>(nonce[0]));
  if (directory_length <= 0 ||
      static_cast<std::size_t>(directory_length) >= directory.size() ||
      mkdir(directory.data(), 0700) != 0) {
    return nullptr;
  }
  impl.socket_directory = directory.data();
  impl.socket_path = impl.socket_directory + "/bootstrap.sock";
  sockaddr_un address{};
  if (!makeSocketAddress(impl.socket_path, address)) {
    return nullptr;
  }
  const int listener =
      socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (listener < 0 ||
      bind(listener, reinterpret_cast<const sockaddr *>(&address),
           sizeof(address)) != 0 ||
      chmod(impl.socket_path.c_str(), 0600) != 0 || listen(listener, 1) != 0) {
    if (listener >= 0) {
      ::close(listener);
    }
    return nullptr;
  }

  const std::string parent_pid = std::to_string(getpid());
  const std::string nonce_high = std::to_string(nonce[0]);
  const std::string nonce_low = std::to_string(nonce[1]);
  const std::string session_generation =
      std::to_string(config.session_generation);
  const std::string session_nonce = std::to_string(config.session_nonce);
  const std::string controller_instance =
      std::to_string(config.controller_instance_id);
  const std::string source_binding_enabled =
      config.source_binding_enabled ? "1" : "0";
  const std::string use_sim_time = config.use_sim_time ? "1" : "0";
  char *const arguments[]{const_cast<char *>(config.executable_path.c_str()),
                          const_cast<char *>(impl.socket_path.c_str()),
                          const_cast<char *>(parent_pid.c_str()),
                          const_cast<char *>(nonce_high.c_str()),
                          const_cast<char *>(nonce_low.c_str()),
                          const_cast<char *>(session_generation.c_str()),
                          const_cast<char *>(session_nonce.c_str()),
                          const_cast<char *>(controller_instance.c_str()),
                          const_cast<char *>(source_binding_enabled.c_str()),
                          const_cast<char *>(use_sim_time.c_str()),
                          nullptr};
  if (posix_spawn(&session->worker_pid_, config.executable_path.c_str(),
                  nullptr, nullptr, arguments, environ) != 0) {
    ::close(listener);
    session->worker_pid_ = -1;
    return nullptr;
  }
  session->pidfd_ = config.test_force_pidfd_unavailable
                        ? -1
                        : openPidfd(session->worker_pid_);
  if (session->pidfd_ < 0) {
    ::close(listener);
    unlink(impl.socket_path.c_str());
    bool reaped = false;
    if (!config.test_force_initial_cleanup_unconfirmed) {
      const auto natural_deadline =
          std::chrono::steady_clock::now() + config.handshake_timeout;
      reaped = waitAndReapChild(session->worker_pid_, natural_deadline);
      if (!reaped) {
        // The PID still names our unreaped direct child, so it cannot have been
        // recycled. This fallback is used only when the kernel cannot provide
        // a pidfd; no production worker is allowed to remain after failed
        // startup.
        (void)kill(session->worker_pid_, SIGKILL);
        reaped = waitAndReapChild(session->worker_pid_,
                                  std::chrono::steady_clock::now() +
                                      config.shutdown_budget.kill);
      }
    }
    if (!reaped) {
      disableFixedBaseIpc(*impl.data);
      closeFd(impl.child_data_fd);
      closeFd(impl.child_ack_fd);
      session->shutdown_outcome_ = WorkerShutdownOutcome::kReapUnconfirmed;
      return session;
    }
    session->worker_pid_ = -1;
    session->shutdown_outcome_ = WorkerShutdownOutcome::kPidfdUnavailable;
    return nullptr;
  }

  const auto handshake_deadline =
      std::chrono::steady_clock::now() + config.handshake_timeout;
  if (!waitForFd(listener, POLLIN, handshake_deadline)) {
    ::close(listener);
    (void)session->shutdown();
    return nullptr;
  }
  const int connection = accept4(listener, nullptr, nullptr, SOCK_CLOEXEC);
  ::close(listener);
  WorkerHandshake handshake{};
  const bool authenticated =
      connection >= 0 && peerMatches(connection, session->worker_pid_) &&
      recv(connection, &handshake, sizeof(handshake), MSG_WAITALL) ==
          static_cast<ssize_t>(sizeof(handshake)) &&
      handshake.magic == kWorkerHandshakeMagic &&
      handshake.nonce_high == nonce[0] && handshake.nonce_low == nonce[1] &&
      handshake.session_generation == config.session_generation &&
      handshake.session_nonce == config.session_nonce &&
      handshake.controller_instance_id == config.controller_instance_id;
  if (!authenticated ||
      !sendTwoFds(connection, impl.child_data_fd, impl.child_ack_fd)) {
    if (connection >= 0) {
      ::close(connection);
    }
    (void)session->shutdown();
    return nullptr;
  }
  impl.control_socket_fd = connection;
  closeFd(impl.child_data_fd);
  closeFd(impl.child_ack_fd);
  storeRelease(impl.data->accepting, 1U);
  const auto ready_deadline =
      std::chrono::steady_clock::now() + config.handshake_timeout;
  while (loadAcquire(impl.ack->worker_ready) == 0U &&
         std::chrono::steady_clock::now() < ready_deadline &&
         !pidfdExited(session->pidfd_)) {
    pollfd descriptor{session->pidfd_, POLLIN, 0};
    (void)poll(&descriptor, 1, 1);
  }
  if (loadAcquire(impl.ack->worker_ready) == 0U ||
      pidfdExited(session->pidfd_)) {
    (void)session->shutdown();
    return nullptr;
  }
  return session;
}

FixedBaseWorkerSession::~FixedBaseWorkerSession() {
  (void)shutdown();
  closeFd(pidfd_);
}

bool FixedBaseWorkerSession::tryCapture(
    const FixedBaseRecord &record) noexcept {
  return impl_ != nullptr && loadAcquire(impl_->data->accepting) != 0U &&
         loadAcquire(impl_->ack->worker_ready) != 0U &&
         tryPushFixedBaseIpc(*impl_->data, *impl_->ack, record) ==
             FixedQueueResult::kAccepted;
}

bool FixedBaseWorkerSession::ready() const noexcept {
  return impl_ != nullptr && pidfd_ >= 0 && !pidfdExited(pidfd_) &&
         validateFixedBaseIpc(*impl_->data, *impl_->ack) &&
         loadAcquire(impl_->data->accepting) != 0U &&
         loadAcquire(impl_->ack->worker_ready) != 0U;
}

std::uint64_t FixedBaseWorkerSession::processedCount() const noexcept {
  return impl_ == nullptr ? 0U : loadAcquire(impl_->ack->processed_count);
}

std::uint64_t FixedBaseWorkerSession::droppedCount() const noexcept {
  return impl_ == nullptr ? 0U : loadAcquire(impl_->data->dropped_count);
}

WorkerShutdownOutcome FixedBaseWorkerSession::shutdown() noexcept {
  if (shutdown_outcome_ == WorkerShutdownOutcome::kReapUnconfirmed) {
    if (worker_pid_ <= 1) {
      return shutdown_outcome_;
    }
    if (pidfd_ >= 0) {
      if (pidfdExited(pidfd_) && reapPidfd(pidfd_, worker_pid_)) {
        worker_pid_ = -1;
        shutdown_outcome_ = WorkerShutdownOutcome::kAlreadyExited;
      }
      return shutdown_outcome_;
    }
    const auto reap_result = tryReapChild(worker_pid_);
    if (reap_result == NonBlockingReapResult::kReaped) {
      worker_pid_ = -1;
      shutdown_outcome_ = WorkerShutdownOutcome::kAlreadyExited;
      return shutdown_outcome_;
    }
    if (reap_result == NonBlockingReapResult::kError) {
      return shutdown_outcome_;
    }
    errno = 0;
    const bool kill_sent = kill(worker_pid_, SIGKILL) == 0;
    if (!kill_sent && errno != ESRCH) {
      return shutdown_outcome_;
    }
    if (waitAndReapChild(worker_pid_, std::chrono::steady_clock::now() +
                                          shutdown_budget_.kill)) {
      worker_pid_ = -1;
      shutdown_outcome_ = kill_sent ? WorkerShutdownOutcome::kKilled
                                    : WorkerShutdownOutcome::kAlreadyExited;
    }
    return shutdown_outcome_;
  }
  if (shutdown_outcome_ != WorkerShutdownOutcome::kNotStarted) {
    return shutdown_outcome_;
  }
  if (impl_ != nullptr && impl_->data != nullptr) {
    disableFixedBaseIpc(*impl_->data);
  }
  if (impl_ != nullptr) {
    closeFd(impl_->control_socket_fd);
  }
  if (worker_pid_ <= 1) {
    shutdown_outcome_ = WorkerShutdownOutcome::kNotStarted;
    return shutdown_outcome_;
  }
  if (pidfd_ < 0) {
    shutdown_outcome_ = WorkerShutdownOutcome::kPidfdUnavailable;
    return shutdown_outcome_;
  }

  const auto wait_and_reap = [this](std::chrono::milliseconds duration,
                                    WorkerShutdownOutcome outcome) -> bool {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    if (!waitPidfd(pidfd_, deadline)) {
      return false;
    }
    if (reapPidfd(pidfd_, worker_pid_)) {
      shutdown_outcome_ = outcome;
      worker_pid_ = -1;
    } else {
      shutdown_outcome_ = WorkerShutdownOutcome::kReapUnconfirmed;
    }
    return true;
  };
  if (pidfdExited(pidfd_)) {
    if (!reapPidfd(pidfd_, worker_pid_)) {
      shutdown_outcome_ = WorkerShutdownOutcome::kReapUnconfirmed;
    } else {
      shutdown_outcome_ = WorkerShutdownOutcome::kAlreadyExited;
      worker_pid_ = -1;
    }
    return shutdown_outcome_;
  }
  if (wait_and_reap(shutdown_budget_.drain, WorkerShutdownOutcome::kDrained)) {
    return shutdown_outcome_;
  }
  if (!signalPidfd(pidfd_, SIGINT)) {
    shutdown_outcome_ = WorkerShutdownOutcome::kError;
    return shutdown_outcome_;
  }
  if (wait_and_reap(shutdown_budget_.interrupt,
                    WorkerShutdownOutcome::kInterrupted)) {
    return shutdown_outcome_;
  }
  if (!signalPidfd(pidfd_, SIGTERM)) {
    shutdown_outcome_ = WorkerShutdownOutcome::kError;
    return shutdown_outcome_;
  }
  if (wait_and_reap(shutdown_budget_.terminate,
                    WorkerShutdownOutcome::kTerminated)) {
    return shutdown_outcome_;
  }
  if (!signalPidfd(pidfd_, SIGKILL)) {
    shutdown_outcome_ = WorkerShutdownOutcome::kError;
    return shutdown_outcome_;
  }
  if (wait_and_reap(shutdown_budget_.kill, WorkerShutdownOutcome::kKilled)) {
    return shutdown_outcome_;
  }
  shutdown_outcome_ = WorkerShutdownOutcome::kReapUnconfirmed;
  return shutdown_outcome_;
}

bool applyFixedBaseWorkerIsolation() noexcept {
  sched_param idle{};
  if (sched_setscheduler(0, SCHED_IDLE, &idle) != 0 ||
      sched_getscheduler(0) != SCHED_IDLE) {
    return false;
  }
  const rlimit address_space{kWorkerAddressSpaceLimitBytes,
                             kWorkerAddressSpaceLimitBytes};
  const rlimit open_files{kWorkerOpenFileLimit, kWorkerOpenFileLimit};
  return setrlimit(RLIMIT_AS, &address_space) == 0 &&
         setrlimit(RLIMIT_NOFILE, &open_files) == 0;
}

int runFixedBaseShadowWorker(int argc, char **argv) {
  if (argc != 10) {
    return workerFailure(2);
  }
  std::uint64_t parent_pid_u64 = 0U;
  std::uint64_t nonce_high = 0U;
  std::uint64_t nonce_low = 0U;
  std::uint64_t session_generation = 0U;
  std::uint64_t session_nonce = 0U;
  std::uint64_t controller_instance_id = 0U;
  if (!validPositiveDecimal(argv[2], parent_pid_u64) ||
      !validPositiveDecimal(argv[3], nonce_high) ||
      !validPositiveDecimal(argv[4], nonce_low) ||
      !validPositiveDecimal(argv[5], session_generation) ||
      !validPositiveDecimal(argv[6], session_nonce) ||
      !validPositiveDecimal(argv[7], controller_instance_id) ||
      (std::strcmp(argv[8], "0") != 0 && std::strcmp(argv[8], "1") != 0) ||
      (std::strcmp(argv[9], "0") != 0 && std::strcmp(argv[9], "1") != 0) ||
      parent_pid_u64 >
          static_cast<std::uint64_t>(std::numeric_limits<pid_t>::max())) {
    return workerFailure(3);
  }
  const pid_t parent_pid = static_cast<pid_t>(parent_pid_u64);
  const bool source_binding_enabled = std::strcmp(argv[8], "1") == 0;
  const bool use_sim_time = std::strcmp(argv[9], "1") == 0;
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || getppid() != parent_pid ||
      !applyFixedBaseWorkerIsolation()) {
    return workerFailure(4);
  }
  sockaddr_un address{};
  if (!makeSocketAddress(argv[1], address)) {
    return workerFailure(5);
  }
  const int connection = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (connection < 0 ||
      connect(connection, reinterpret_cast<const sockaddr *>(&address),
              sizeof(address)) != 0 ||
      !peerMatches(connection, parent_pid)) {
    if (connection >= 0) {
      ::close(connection);
    }
    return workerFailure(6);
  }
  const WorkerHandshake handshake{
      kWorkerHandshakeMagic, nonce_high,    nonce_low,
      session_generation,    session_nonce, controller_instance_id};
  if (send(connection, &handshake, sizeof(handshake), MSG_NOSIGNAL) !=
      static_cast<ssize_t>(sizeof(handshake))) {
    ::close(connection);
    return workerFailure(7);
  }
  int data_fd = -1;
  int ack_fd = -1;
  if (!receiveTwoFds(connection, data_fd, ack_fd) ||
      !fdModeIs(data_fd, O_RDONLY) || !fdModeIs(ack_fd, O_RDWR) ||
      !fdSizeIs(data_fd, sizeof(FixedBaseDataRegion)) ||
      !fdSizeIs(ack_fd, sizeof(FixedBaseAckRegion))) {
    closeFd(data_fd);
    closeFd(ack_fd);
    ::close(connection);
    return workerFailure(8);
  }
  auto *data = static_cast<const FixedBaseDataRegion *>(mmap(
      nullptr, sizeof(FixedBaseDataRegion), PROT_READ, MAP_SHARED, data_fd, 0));
  auto *ack = static_cast<FixedBaseAckRegion *>(
      mmap(nullptr, sizeof(FixedBaseAckRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, ack_fd, 0));
  closeFd(data_fd);
  closeFd(ack_fd);
  if (data == MAP_FAILED || ack == MAP_FAILED ||
      !validateFixedBaseIpc(*data, *ack) ||
      data->session_generation != session_generation ||
      data->session_nonce != session_nonce ||
      data->controller_instance_id != controller_instance_id) {
    if (data != MAP_FAILED) {
      munmap(const_cast<FixedBaseDataRegion *>(data),
             sizeof(FixedBaseDataRegion));
    }
    if (ack != MAP_FAILED) {
      munmap(ack, sizeof(FixedBaseAckRegion));
    }
    ::close(connection);
    return workerFailure(9);
  }

  int ros_argc = 1;
  char *ros_argv[]{argv[0], nullptr};
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<ControllerBaseTrajectorySnapshot>::SharedPtr publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr binding_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      test_reliable_binding_audit_publisher;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>::SharedPtr
      proposal_subscription;
  std::optional<ControllerBaseTrajectorySnapshot> current_snapshot;
  std::optional<ControllerBaseTrajectorySnapshot> previous_snapshot;
  try {
    rclcpp::init(ros_argc, ros_argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", use_sim_time)});
    node = std::make_shared<rclcpp::Node>("c002ay0_shadow_worker", options);
    publisher = node->create_publisher<ControllerBaseTrajectorySnapshot>(
        "/control/overtake/base_trajectory_snapshot",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    if (source_binding_enabled) {
      const auto shadow_qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      binding_publisher = node->create_publisher<std_msgs::msg::String>(
          "/debug/overtake/state_lattice/source_binding", shadow_qos);
      const char *test_reliable_binding_audit =
          std::getenv(kTestReliableBindingAuditEnvironment);
      if (test_reliable_binding_audit != nullptr &&
          std::strcmp(test_reliable_binding_audit, "1") == 0) {
        test_reliable_binding_audit_publisher =
            node->create_publisher<std_msgs::msg::String>(
                kTestReliableBindingAuditTopic,
                rclcpp::QoS(rclcpp::KeepLast(64))
                    .reliable()
                    .durability_volatile());
      }
      proposal_subscription = node->create_subscription<
          multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>(
          "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
          shadow_qos,
          [&, session_generation, session_nonce, controller_instance_id](
              const multi_purpose_mpc_ros_msgs::msg::
                  AuthorizedCartesianTrajectory::SharedPtr proposal) {
            if (!current_snapshot.has_value() || proposal == nullptr ||
                !binding_publisher) {
              return;
            }
            const auto binding_callback_monotonic_ns =
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
            const auto binding_now = node->get_clock()->now();
            const auto result = evaluateShadowBinding(current_snapshot.value(),
                                                      previous_snapshot,
                                                      *proposal, binding_now);
            std::ostringstream json;
            json << "{\"schema_version\":1,\"shadow_only\":true,"
                 << "\"lateral_authority_eligible\":false,\"session_"
                    "generation\":"
                 << session_generation << ",\"session_nonce\":" << session_nonce
                 << ",\"controller_instance_id\":" << controller_instance_id
                 << ",\"disposition\":\"" << toString(result.disposition)
                 << "\",\"validation_error\":"
                 << static_cast<unsigned int>(result.validation_error)
                 << ",\"current_race_arm_epoch\":"
                 << current_snapshot->race_arm_epoch
                 << ",\"current_controller_sequence\":"
                 << current_snapshot->controller_sequence
                 << ",\"proposal_race_arm_epoch\":"
                 << proposal->plan_sample_key.race_arm_epoch
                 << ",\"proposal_planner_instance_id\":"
                 << proposal->plan_sample_key.planner_instance_id
                 << ",\"proposal_attempt_id\":"
                 << proposal->plan_sample_key.attempt_id
                 << ",\"proposal_connector_transaction_id\":"
                 << proposal->plan_sample_key.connector_transaction_id
                 << ",\"proposal_plan_generation\":"
                 << proposal->plan_sample_key.plan_generation
                 << ",\"proposal_candidate_revision\":"
                 << proposal->candidate_revision
                 << ",\"proposal_authority_token\":"
                 << proposal->authority_token
                 << ",\"proposal_safety_snapshot_id\":"
                 << proposal->safety_snapshot_id
                 << ",\"proposal_controller_instance_id\":"
                 << proposal->source_controller_instance_id
                 << ",\"proposal_controller_sequence\":"
                 << proposal->source_controller_sequence
                 << ",\"proposal_base_lease_id\":" << proposal->base_lease_id
                 << ",\"current_source_stamp_sec\":"
                 << current_snapshot->base_source_stamp.sec
                 << ",\"current_source_stamp_nanosec\":"
                 << current_snapshot->base_source_stamp.nanosec
                 << ",\"current_source_generation\":"
                 << current_snapshot->base_source_generation
                 << ",\"current_nearest_source_index\":"
                 << current_snapshot->nearest_source_index
                 << ",\"proposal_source_stamp_sec\":"
                 << proposal->base_source_stamp.sec
                 << ",\"proposal_source_stamp_nanosec\":"
                 << proposal->base_source_stamp.nanosec
                 << ",\"proposal_source_generation\":"
                 << proposal->base_source_generation
                 << ",\"proposal_nearest_source_index\":"
                 << proposal->base_nearest_source_index
                 << ",\"proposal_safety_valid_until_sec\":"
                 << proposal->safety_valid_until.sec
                 << ",\"proposal_safety_valid_until_nanosec\":"
                 << proposal->safety_valid_until.nanosec
                 << ",\"binding_ros_now_sec\":"
                 << binding_now.nanoseconds() / 1000000000LL
                 << ",\"binding_ros_now_nanosec\":"
                 << static_cast<std::uint32_t>(binding_now.nanoseconds() %
                                               1000000000LL)
                 << ",\"binding_callback_monotonic_ns\":"
                 << binding_callback_monotonic_ns << "}";
            std_msgs::msg::String message;
            message.data = json.str();
            binding_publisher->publish(message);
            if (test_reliable_binding_audit_publisher) {
              std::ostringstream audit_json;
              const auto &current = current_snapshot.value();
              audit_json
                  << message.data.substr(0U, message.data.size() - 1U)
                  << ",\"test_audit_schema_version\":1"
                  << ",\"current_controller_instance_id\":"
                  << current.controller_instance_id
                  << ",\"current_base_lease_id\":" << current.base_lease_id
                  << ",\"current_first_source_index\":"
                  << current.first_source_index
                  << ",\"current_last_source_index\":"
                  << current.last_source_index
                  << ",\"previous_present\":"
                  << (previous_snapshot.has_value() ? "true" : "false");
              if (previous_snapshot.has_value()) {
                const auto &previous = previous_snapshot.value();
                audit_json
                    << ",\"previous_race_arm_epoch\":"
                    << previous.race_arm_epoch
                    << ",\"previous_controller_instance_id\":"
                    << previous.controller_instance_id
                    << ",\"previous_controller_sequence\":"
                    << previous.controller_sequence
                    << ",\"previous_base_lease_id\":"
                    << previous.base_lease_id
                    << ",\"previous_source_stamp_sec\":"
                    << previous.base_source_stamp.sec
                    << ",\"previous_source_stamp_nanosec\":"
                    << previous.base_source_stamp.nanosec
                    << ",\"previous_source_generation\":"
                    << previous.base_source_generation
                    << ",\"previous_first_source_index\":"
                    << previous.first_source_index
                    << ",\"previous_last_source_index\":"
                    << previous.last_source_index
                    << ",\"previous_nearest_source_index\":"
                    << previous.nearest_source_index;
              }
              audit_json << "}";
              std_msgs::msg::String audit_message;
              audit_message.data = audit_json.str();
              test_reliable_binding_audit_publisher->publish(audit_message);
            }
          });
    }
  } catch (...) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    munmap(const_cast<FixedBaseDataRegion *>(data),
           sizeof(FixedBaseDataRegion));
    munmap(ack, sizeof(FixedBaseAckRegion));
    ::close(connection);
    return workerFailure(10);
  }
  storeRelease(ack->worker_ready, 1U);

  rclcpp::Serialization<ControllerBaseTrajectorySnapshot> serializer;
  constexpr auto kMinimumPublishInterval = std::chrono::milliseconds(50);
  auto last_publish = std::chrono::steady_clock::time_point::min();
  std::optional<FixedBaseRecord> pending_record;
  bool producer_shutdown_requested = false;
  for (;;) {
    if (source_binding_enabled) {
      rclcpp::spin_some(node);
    }
    pollfd control{connection, POLLIN | POLLHUP, 0};
    const int poll_result = poll(&control, 1, 1);
    if (poll_result > 0 &&
        (control.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      producer_shutdown_requested = true;
    }
    if (!rclcpp::ok()) {
      break;
    }
    producer_shutdown_requested =
        producer_shutdown_requested || loadAcquire(data->accepting) == 0U;
    FixedBaseRecord record{};
    const auto result = tryPopFixedBaseIpc(*data, *ack, record);
    if (result != FixedQueueResult::kAccepted &&
        result != FixedQueueResult::kEmpty) {
      if (result == FixedQueueResult::kTorn) {
        continue;
      }
      incrementRelaxed(ack->validation_failure_count);
      continue;
    }
    if (result == FixedQueueResult::kAccepted) {
      pending_record = std::move(record);
    }

    const auto now = std::chrono::steady_clock::now();
    if (!pending_record.has_value()) {
      if (producer_shutdown_requested && result == FixedQueueResult::kEmpty) {
        break;
      }
      continue;
    }
    if (last_publish != std::chrono::steady_clock::time_point::min() &&
        now - last_publish < kMinimumPublishInterval) {
      continue;
    }
    ControllerBaseTrajectorySnapshot snapshot;
    if (buildBaseSnapshotFromFixedRecord(*pending_record, snapshot) !=
        ValidationError::NONE) {
      pending_record.reset();
      incrementRelaxed(ack->validation_failure_count);
      continue;
    }
    if (source_binding_enabled) {
      if (current_snapshot.has_value() &&
          (current_snapshot->race_arm_epoch != snapshot.race_arm_epoch ||
           current_snapshot->controller_instance_id !=
               snapshot.controller_instance_id)) {
        current_snapshot.reset();
        previous_snapshot.reset();
      }
      previous_snapshot = std::move(current_snapshot);
      current_snapshot = snapshot;
    }
    try {
      rclcpp::SerializedMessage serialized;
      serializer.serialize_message(&snapshot, &serialized);
      if (serialized.size() == 0U) {
        pending_record.reset();
        incrementRelaxed(ack->serialization_failure_count);
        continue;
      }
    } catch (...) {
      pending_record.reset();
      incrementRelaxed(ack->serialization_failure_count);
      continue;
    }
    try {
      publisher->publish(snapshot);
      pending_record.reset();
      last_publish = now;
    } catch (...) {
      pending_record.reset();
      incrementRelaxed(ack->serialization_failure_count);
      continue;
    }
    incrementRelaxed(ack->processed_count);
  }
  storeRelease(ack->worker_ready, 0U);
  proposal_subscription.reset();
  binding_publisher.reset();
  publisher.reset();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  munmap(const_cast<FixedBaseDataRegion *>(data), sizeof(FixedBaseDataRegion));
  munmap(ack, sizeof(FixedBaseAckRegion));
  ::close(connection);
  return 0;
}

} // namespace overtake_transport_contract::c002ay0
