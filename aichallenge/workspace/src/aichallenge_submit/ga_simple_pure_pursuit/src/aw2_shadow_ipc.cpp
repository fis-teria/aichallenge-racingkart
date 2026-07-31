#include "simple_pure_pursuit/aw2_shadow_ipc.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/memfd.h>
#include <poll.h>
#include <sched.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/random.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>

extern char **environ;

namespace simple_pure_pursuit::aw2_shadow {
namespace {

constexpr std::uint64_t kHandshakeMagic = 0x41573248414e4431ULL;
std::atomic<std::uint32_t> active_async_bootstraps{0U};

struct HandshakePacket {
  std::uint64_t magic;
  std::uint64_t nonce_high;
  std::uint64_t nonce_low;
};

int createMemfd(const char *name, std::size_t size) noexcept {
  const int fd = static_cast<int>(
      syscall(SYS_memfd_create, name, MFD_CLOEXEC | MFD_ALLOW_SEALING));
  if (fd < 0 || ftruncate(fd, static_cast<off_t>(size)) != 0) {
    if (fd >= 0) {
      close(fd);
    }
    return -1;
  }
  return fd;
}

int reopenReadOnly(int fd) noexcept {
  char path[64]{};
  const int length = std::snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
  if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(path)) {
    return -1;
  }
  return open(path, O_RDONLY | O_CLOEXEC);
}

void closeFd(int &fd) noexcept {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

bool fdModeIs(int fd, int expected_mode) noexcept {
  const int flags = fcntl(fd, F_GETFL);
  return flags >= 0 && (flags & O_ACCMODE) == expected_mode &&
         (fcntl(fd, F_GETFD) & FD_CLOEXEC) != 0;
}

bool fdSizeIs(int fd, std::size_t expected_size) noexcept {
  struct stat status {};
  return fstat(fd, &status) == 0 &&
         status.st_size == static_cast<off_t>(expected_size);
}

bool fdsAreDistinct(int left, int right) noexcept {
  struct stat left_status {};
  struct stat right_status {};
  return fstat(left, &left_status) == 0 && fstat(right, &right_status) == 0 &&
         (left_status.st_dev != right_status.st_dev ||
          left_status.st_ino != right_status.st_ino);
}

bool childExited(pid_t child, int *status) noexcept {
  const pid_t result = waitpid(child, status, WNOHANG);
  return result == child || (result < 0 && errno == ECHILD);
}

bool waitBounded(pid_t child, std::chrono::milliseconds duration,
                 int *status) noexcept {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  do {
    if (childExited(child, status)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < deadline);
  return childExited(child, status);
}

ChildTerminationOutcome killAndReapSpawnedSupervisor(pid_t child) noexcept {
  if (child <= 0) {
    return ChildTerminationOutcome::kError;
  }
  int status = 0;
  if (childExited(child, &status)) {
    return ChildTerminationOutcome::kAlreadyExited;
  }
  if (kill(child, SIGKILL) != 0 && errno != ESRCH) {
    return ChildTerminationOutcome::kError;
  }
  return waitBounded(child, std::chrono::milliseconds(250), &status)
             ? ChildTerminationOutcome::kKilled
             : ChildTerminationOutcome::kKillUnconfirmed;
}

void recordBootstrapFailure(BootstrapFailureDetails *details,
                            BootstrapFailureStage stage, pid_t child,
                            ChildTerminationOutcome outcome) noexcept {
  if (details == nullptr) {
    return;
  }
  details->stage = stage;
  details->spawned_supervisor_pid = child;
  details->termination_outcome = outcome;
}

void closeMapping(SharedDataRegion *&mapping) noexcept {
  if (mapping != nullptr) {
    munmap(mapping, sizeof(SharedDataRegion));
    mapping = nullptr;
  }
}

void closeMapping(SharedAckRegion *&mapping) noexcept {
  if (mapping != nullptr) {
    munmap(mapping, sizeof(SharedAckRegion));
    mapping = nullptr;
  }
}

bool makeSocketAddress(const std::string &path,
                       struct sockaddr_un &address) noexcept {
  if (path.empty() || path.size() >= sizeof(address.sun_path)) {
    return false;
  }
  address = {};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);
  return true;
}

bool sendBootstrapFds(int socket_fd, int data_fd, int ack_fd) noexcept {
  std::uint8_t payload = 1U;
  struct iovec iov {
    &payload, sizeof(payload)
  };
  alignas(struct cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int) * 2U)>
      control{};
  struct msghdr message {};
  message.msg_iov = &iov;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  auto *header = CMSG_FIRSTHDR(&message);
  header->cmsg_level = SOL_SOCKET;
  header->cmsg_type = SCM_RIGHTS;
  header->cmsg_len = CMSG_LEN(sizeof(int) * 2U);
  const int fds[2]{data_fd, ack_fd};
  std::memcpy(CMSG_DATA(header), fds, sizeof(fds));
  return sendmsg(socket_fd, &message, MSG_NOSIGNAL) ==
         static_cast<ssize_t>(sizeof(payload));
}

bool receiveBootstrapFds(int socket_fd, int &data_fd, int &ack_fd) noexcept {
  std::uint8_t payload = 0U;
  struct iovec iov {
    &payload, sizeof(payload)
  };
  alignas(struct cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int) * 2U)>
      control{};
  struct msghdr message {};
  message.msg_iov = &iov;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  const ssize_t received = recvmsg(socket_fd, &message, MSG_CMSG_CLOEXEC);
  if (received != static_cast<ssize_t>(sizeof(payload)) ||
      (message.msg_flags & (MSG_CTRUNC | MSG_TRUNC)) != 0) {
    return false;
  }
  auto *header = CMSG_FIRSTHDR(&message);
  if (header == nullptr || CMSG_NXTHDR(&message, header) != nullptr ||
      header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
      header->cmsg_len != CMSG_LEN(sizeof(int) * 2U)) {
    return false;
  }
  int fds[2]{-1, -1};
  std::memcpy(fds, CMSG_DATA(header), sizeof(fds));
  data_fd = fds[0];
  ack_fd = fds[1];
  return true;
}

} // namespace

std::optional<DirectionalMemfdChannels> DirectionalMemfdChannels::create(
    std::uint64_t ring_generation,
    std::uint64_t controller_instance_id) noexcept {
  DirectionalMemfdChannels channels;
  channels.producer_data_fd_ =
      createMemfd("aw2-data", sizeof(SharedDataRegion));
  int ack_write_fd = createMemfd("aw2-ack", sizeof(SharedAckRegion));
  if (channels.producer_data_fd_ < 0 || ack_write_fd < 0) {
    closeFd(channels.producer_data_fd_);
    closeFd(ack_write_fd);
    return std::nullopt;
  }

  channels.data_mapping_ = static_cast<SharedDataRegion *>(
      mmap(nullptr, sizeof(SharedDataRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, channels.producer_data_fd_, 0));
  auto *ack_write_mapping = static_cast<SharedAckRegion *>(
      mmap(nullptr, sizeof(SharedAckRegion), PROT_READ | PROT_WRITE, MAP_SHARED,
           ack_write_fd, 0));
  if (channels.data_mapping_ == MAP_FAILED || ack_write_mapping == MAP_FAILED) {
    if (channels.data_mapping_ != MAP_FAILED) {
      munmap(channels.data_mapping_, sizeof(SharedDataRegion));
    }
    if (ack_write_mapping != MAP_FAILED) {
      munmap(ack_write_mapping, sizeof(SharedAckRegion));
    }
    channels.data_mapping_ = nullptr;
    closeFd(channels.producer_data_fd_);
    closeFd(ack_write_fd);
    return std::nullopt;
  }

  if (!initializeSharedRegions(*channels.data_mapping_, *ack_write_mapping,
                               kQueueCapacity, ring_generation,
                               controller_instance_id)) {
    munmap(channels.data_mapping_, sizeof(SharedDataRegion));
    munmap(ack_write_mapping, sizeof(SharedAckRegion));
    channels.data_mapping_ = nullptr;
    closeFd(channels.producer_data_fd_);
    closeFd(ack_write_fd);
    return std::nullopt;
  }

  channels.shadow_data_fd_ = reopenReadOnly(channels.producer_data_fd_);
  channels.producer_ack_fd_ = reopenReadOnly(ack_write_fd);
  channels.shadow_ack_fd_ = fcntl(ack_write_fd, F_DUPFD_CLOEXEC, 3);
  if (channels.shadow_data_fd_ < 0 || channels.producer_ack_fd_ < 0 ||
      channels.shadow_ack_fd_ < 0) {
    munmap(channels.data_mapping_, sizeof(SharedDataRegion));
    munmap(ack_write_mapping, sizeof(SharedAckRegion));
    channels.data_mapping_ = nullptr;
    closeFd(channels.producer_data_fd_);
    closeFd(channels.producer_ack_fd_);
    closeFd(channels.shadow_data_fd_);
    closeFd(channels.shadow_ack_fd_);
    closeFd(ack_write_fd);
    return std::nullopt;
  }

  channels.ack_mapping_ = static_cast<SharedAckRegion *>(
      mmap(nullptr, sizeof(SharedAckRegion), PROT_READ, MAP_SHARED,
           channels.producer_ack_fd_, 0));
  munmap(ack_write_mapping, sizeof(SharedAckRegion));
  closeFd(ack_write_fd);
  if (channels.ack_mapping_ == MAP_FAILED) {
    channels.ack_mapping_ = nullptr;
    return std::nullopt;
  }

  int data_seals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
#ifdef F_SEAL_FUTURE_WRITE
  data_seals |= F_SEAL_FUTURE_WRITE;
#endif
  const int ack_seals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (fcntl(channels.producer_data_fd_, F_ADD_SEALS, data_seals) != 0 ||
      fcntl(channels.shadow_ack_fd_, F_ADD_SEALS, ack_seals) != 0 ||
      !channels.validate()) {
    return std::nullopt;
  }
  return channels;
}

DirectionalMemfdChannels::~DirectionalMemfdChannels() {
  closeMapping(data_mapping_);
  closeMapping(ack_mapping_);
  closeFd(producer_data_fd_);
  closeFd(producer_ack_fd_);
  closeFd(shadow_data_fd_);
  closeFd(shadow_ack_fd_);
}

DirectionalMemfdChannels::DirectionalMemfdChannels(
    DirectionalMemfdChannels &&other) noexcept {
  *this = std::move(other);
}

DirectionalMemfdChannels &
DirectionalMemfdChannels::operator=(DirectionalMemfdChannels &&other) noexcept {
  if (this == &other) {
    return *this;
  }
  closeMapping(data_mapping_);
  closeMapping(ack_mapping_);
  closeFd(producer_data_fd_);
  closeFd(producer_ack_fd_);
  closeFd(shadow_data_fd_);
  closeFd(shadow_ack_fd_);
  producer_data_fd_ = other.producer_data_fd_;
  producer_ack_fd_ = other.producer_ack_fd_;
  shadow_data_fd_ = other.shadow_data_fd_;
  shadow_ack_fd_ = other.shadow_ack_fd_;
  data_mapping_ = other.data_mapping_;
  ack_mapping_ = other.ack_mapping_;
  other.producer_data_fd_ = -1;
  other.producer_ack_fd_ = -1;
  other.shadow_data_fd_ = -1;
  other.shadow_ack_fd_ = -1;
  other.data_mapping_ = nullptr;
  other.ack_mapping_ = nullptr;
  return *this;
}

void DirectionalMemfdChannels::closeShadowAliases() noexcept {
  closeFd(shadow_data_fd_);
  closeFd(shadow_ack_fd_);
}

bool DirectionalMemfdChannels::validate() const noexcept {
  if (data_mapping_ == nullptr || ack_mapping_ == nullptr ||
      !validateSharedAbi(*data_mapping_, *ack_mapping_) ||
      !fdModeIs(producer_data_fd_, O_RDWR) ||
      !fdModeIs(shadow_data_fd_, O_RDONLY) ||
      !fdModeIs(producer_ack_fd_, O_RDONLY) ||
      !fdModeIs(shadow_ack_fd_, O_RDWR) ||
      !fdSizeIs(producer_data_fd_, sizeof(SharedDataRegion)) ||
      !fdSizeIs(shadow_data_fd_, sizeof(SharedDataRegion)) ||
      !fdSizeIs(producer_ack_fd_, sizeof(SharedAckRegion)) ||
      !fdSizeIs(shadow_ack_fd_, sizeof(SharedAckRegion)) ||
      !fdsAreDistinct(producer_data_fd_, producer_ack_fd_)) {
    return false;
  }
  const int data_seals = fcntl(producer_data_fd_, F_GET_SEALS);
  const int ack_seals = fcntl(shadow_ack_fd_, F_GET_SEALS);
  const int required = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  return data_seals >= 0 && ack_seals >= 0 &&
         (data_seals & required) == required &&
         (ack_seals & required) == required;
}

std::optional<ProducerSession>
ProducerSession::start(const std::string &supervisor_executable,
                       std::uint64_t ring_generation,
                       std::uint64_t controller_instance_id,
                       std::chrono::milliseconds handshake_timeout,
                       BootstrapFailureDetails *failure_details) noexcept {
  if (failure_details != nullptr) {
    *failure_details = BootstrapFailureDetails{};
  }
  auto channels =
      DirectionalMemfdChannels::create(ring_generation, controller_instance_id);
  if (!channels.has_value() || supervisor_executable.empty()) {
    return std::nullopt;
  }
  atomicStoreRelease(channels->data()->accepting, 0U);

  std::array<std::uint64_t, 2U> nonce{};
  if (getrandom(nonce.data(), sizeof(nonce), 0) !=
      static_cast<ssize_t>(sizeof(nonce))) {
    return std::nullopt;
  }
  char directory[96]{};
  const int directory_length = std::snprintf(
      directory, sizeof(directory), "/tmp/aw2-shadow-%u-%d-%016lx",
      static_cast<unsigned>(getuid()), static_cast<int>(getpid()),
      static_cast<unsigned long>(nonce[0]));
  if (directory_length <= 0 ||
      static_cast<std::size_t>(directory_length) >= sizeof(directory) ||
      mkdir(directory, 0700) != 0) {
    return std::nullopt;
  }
  const std::string socket_directory(directory);
  const std::string socket_path = socket_directory + "/bootstrap.sock";
  struct sockaddr_un address {};
  if (!makeSocketAddress(socket_path, address)) {
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }

  const int listener =
      socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (listener < 0 ||
      bind(listener, reinterpret_cast<const struct sockaddr *>(&address),
           sizeof(address)) != 0 ||
      chmod(socket_path.c_str(), 0600) != 0 || listen(listener, 1) != 0) {
    if (listener >= 0) {
      close(listener);
    }
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }

  const std::string producer_pid = std::to_string(getpid());
  const std::string nonce_high = std::to_string(nonce[0]);
  const std::string nonce_low = std::to_string(nonce[1]);
  char *const arguments[]{const_cast<char *>(supervisor_executable.c_str()),
                          const_cast<char *>(socket_path.c_str()),
                          const_cast<char *>(producer_pid.c_str()),
                          const_cast<char *>(nonce_high.c_str()),
                          const_cast<char *>(nonce_low.c_str()),
                          nullptr};
  pid_t supervisor_pid = -1;
  const int spawn_result =
      posix_spawn(&supervisor_pid, supervisor_executable.c_str(), nullptr,
                  nullptr, arguments, environ);
  if (spawn_result != 0) {
    recordBootstrapFailure(failure_details, BootstrapFailureStage::kSpawn, -1,
                           ChildTerminationOutcome::kError);
    close(listener);
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }

  struct pollfd poll_descriptor {
    listener, POLLIN, 0
  };
  const auto bounded_timeout =
      std::clamp<std::int64_t>(handshake_timeout.count(), 0, 5000);
  if (poll(&poll_descriptor, 1, static_cast<int>(bounded_timeout)) != 1) {
    close(listener);
    const auto outcome = killAndReapSpawnedSupervisor(supervisor_pid);
    recordBootstrapFailure(failure_details,
                           BootstrapFailureStage::kHandshakeTimeout,
                           supervisor_pid, outcome);
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }
  const int connection = accept4(listener, nullptr, nullptr, SOCK_CLOEXEC);
  close(listener);
  if (connection < 0) {
    const auto outcome = killAndReapSpawnedSupervisor(supervisor_pid);
    recordBootstrapFailure(failure_details, BootstrapFailureStage::kAccept,
                           supervisor_pid, outcome);
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }
  const auto credentials = getPeerCredentials(connection);
  HandshakePacket packet{};
  const bool authenticated =
      credentials.has_value() &&
      peerMatchesExpected(*credentials, supervisor_pid, getuid(), getgid()) &&
      recv(connection, &packet, sizeof(packet), MSG_WAITALL) ==
          static_cast<ssize_t>(sizeof(packet)) &&
      packet.magic == kHandshakeMagic && packet.nonce_high == nonce[0] &&
      packet.nonce_low == nonce[1];
  if (!authenticated || !sendBootstrapFds(connection, channels->shadowDataFd(),
                                          channels->shadowAckFd())) {
    close(connection);
    const auto outcome = killAndReapSpawnedSupervisor(supervisor_pid);
    recordBootstrapFailure(failure_details,
                           BootstrapFailureStage::kAuthentication,
                           supervisor_pid, outcome);
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }
  channels->closeShadowAliases();
  try {
    std::thread([supervisor_pid]() {
      int status = 0;
      while (waitpid(supervisor_pid, &status, 0) < 0 && errno == EINTR) {
      }
    }).detach();
  } catch (...) {
    close(connection);
    const auto outcome = killAndReapSpawnedSupervisor(supervisor_pid);
    recordBootstrapFailure(failure_details,
                           BootstrapFailureStage::kBootstrapThread,
                           supervisor_pid, outcome);
    unlink(socket_path.c_str());
    rmdir(socket_directory.c_str());
    return std::nullopt;
  }

  ProducerSession session;
  session.channels_ = std::move(channels.value());
  session.control_socket_fd_ = connection;
  session.supervisor_pid_ = supervisor_pid;
  session.socket_directory_ = socket_directory;
  session.socket_path_ = socket_path;
  return session;
}

ProducerSession::~ProducerSession() {
  if (channels_.data() != nullptr) {
    atomicStoreRelease(channels_.data()->accepting, 0U);
    atomicStoreRelease(channels_.data()->closing, 1U);
  }
  closeFd(control_socket_fd_);
  if (!socket_path_.empty()) {
    unlink(socket_path_.c_str());
  }
  if (!socket_directory_.empty()) {
    rmdir(socket_directory_.c_str());
  }
}

ProducerSession::ProducerSession(ProducerSession &&other) noexcept {
  *this = std::move(other);
}

ProducerSession &ProducerSession::operator=(ProducerSession &&other) noexcept {
  if (this == &other) {
    return *this;
  }
  if (channels_.data() != nullptr) {
    atomicStoreRelease(channels_.data()->accepting, 0U);
    atomicStoreRelease(channels_.data()->closing, 1U);
  }
  closeFd(control_socket_fd_);
  channels_ = std::move(other.channels_);
  control_socket_fd_ = other.control_socket_fd_;
  supervisor_pid_ = other.supervisor_pid_;
  socket_directory_ = std::move(other.socket_directory_);
  socket_path_ = std::move(other.socket_path_);
  other.control_socket_fd_ = -1;
  other.supervisor_pid_ = -1;
  other.socket_directory_.clear();
  other.socket_path_.clear();
  return *this;
}

bool ProducerSession::tryCapture(const FixedSnapshot &snapshot) noexcept {
  if (!valid()) {
    return false;
  }
  atomicStoreRelease(channels_.data()->race_arm_epoch,
                     snapshot.controller_sample_key.race_arm_epoch);
  return tryPush(*channels_.data(), *channels_.ack(), snapshot);
}

void ProducerSession::setProducerDigests(
    const std::array<std::uint8_t, 32U> &build_digest,
    const std::array<std::uint8_t, 32U> &config_digest) noexcept {
  if (channels_.data() == nullptr) {
    return;
  }
  channels_.data()->producer_build_digest = build_digest;
  channels_.data()->producer_config_digest = config_digest;
  atomicStoreRelease(channels_.data()->accepting, 1U);
}

bool ProducerSession::valid() const noexcept {
  return channels_.data() != nullptr && channels_.ack() != nullptr &&
         control_socket_fd_ >= 0 &&
         atomicLoadAcquire(channels_.data()->accepting) != 0U &&
         atomicLoadAcquire(channels_.data()->closing) == 0U;
}

std::uint64_t ProducerSession::consumedCount() const noexcept {
  return channels_.ack() == nullptr
             ? 0U
             : atomicLoadAcquire(channels_.ack()->read_index);
}

struct AsyncProducerSession::State {
  std::atomic<std::uint32_t> references{2U};
  std::atomic<bool> owner_alive{true};
  std::atomic<ProducerSession *> ready_session{nullptr};

  ~State() { delete ready_session.load(std::memory_order_relaxed); }
};

std::unique_ptr<AsyncProducerSession> AsyncProducerSession::start(
    const std::string &supervisor_executable, std::uint64_t ring_generation,
    std::uint64_t controller_instance_id,
    const std::array<std::uint8_t, 32U> &build_digest,
    const std::array<std::uint8_t, 32U> &config_digest,
    std::chrono::milliseconds handshake_timeout) noexcept {
  State *state = nullptr;
  try {
    state = new State;
    auto owner =
        std::unique_ptr<AsyncProducerSession>(new AsyncProducerSession(state));
    active_async_bootstraps.fetch_add(1U, std::memory_order_relaxed);
    std::thread([state, supervisor_executable, ring_generation,
                 controller_instance_id, build_digest, config_digest,
                 handshake_timeout]() {
      auto session =
          ProducerSession::start(supervisor_executable, ring_generation,
                                 controller_instance_id, handshake_timeout);
      if (session.has_value()) {
        session->setProducerDigests(build_digest, config_digest);
        auto *candidate =
            new (std::nothrow) ProducerSession(std::move(session.value()));
        if (candidate != nullptr) {
          if (state->owner_alive.load(std::memory_order_acquire)) {
            state->ready_session.store(candidate, std::memory_order_release);
          } else {
            delete candidate;
          }
        }
      }
      if (state->references.fetch_sub(1U, std::memory_order_acq_rel) == 1U) {
        delete state;
      }
      active_async_bootstraps.fetch_sub(1U, std::memory_order_relaxed);
    }).detach();
    return owner;
  } catch (...) {
    if (active_async_bootstraps.load(std::memory_order_relaxed) != 0U) {
      active_async_bootstraps.fetch_sub(1U, std::memory_order_relaxed);
    }
    if (state != nullptr) {
      state->references.store(1U, std::memory_order_relaxed);
      delete state;
    }
    return nullptr;
  }
}

std::uint32_t activeAsyncBootstrapCount() noexcept {
  return active_async_bootstraps.load(std::memory_order_acquire);
}

AsyncProducerSession::~AsyncProducerSession() {
  if (state_ == nullptr) {
    return;
  }
  state_->owner_alive.store(false, std::memory_order_release);
  delete state_->ready_session.exchange(nullptr, std::memory_order_acq_rel);
  if (state_->references.fetch_sub(1U, std::memory_order_acq_rel) == 1U) {
    delete state_;
  }
  state_ = nullptr;
}

bool AsyncProducerSession::tryCapture(const FixedSnapshot &snapshot) noexcept {
  if (state_ == nullptr) {
    return false;
  }
  auto *session = state_->ready_session.load(std::memory_order_acquire);
  return session != nullptr && session->tryCapture(snapshot);
}

bool AsyncProducerSession::ready() const noexcept {
  return state_ != nullptr &&
         state_->ready_session.load(std::memory_order_acquire) != nullptr;
}

std::optional<ReceivedBootstrap>
receiveBootstrap(const std::string &socket_path, pid_t expected_producer_pid,
                 std::uint64_t nonce_high, std::uint64_t nonce_low) noexcept {
  struct sockaddr_un address {};
  if (!makeSocketAddress(socket_path, address)) {
    return std::nullopt;
  }
  const int connection = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (connection < 0 ||
      connect(connection, reinterpret_cast<const struct sockaddr *>(&address),
              sizeof(address)) != 0) {
    if (connection >= 0) {
      close(connection);
    }
    return std::nullopt;
  }
  const auto credentials = getPeerCredentials(connection);
  if (!credentials.has_value() ||
      !peerMatchesExpected(*credentials, expected_producer_pid, getuid(),
                           getgid())) {
    close(connection);
    return std::nullopt;
  }
  const HandshakePacket packet{kHandshakeMagic, nonce_high, nonce_low};
  if (send(connection, &packet, sizeof(packet), MSG_NOSIGNAL) !=
      static_cast<ssize_t>(sizeof(packet))) {
    close(connection);
    return std::nullopt;
  }
  ReceivedBootstrap bootstrap;
  bootstrap.control_socket_fd = connection;
  bootstrap.producer_pid = expected_producer_pid;
  bootstrap.nonce_high = nonce_high;
  bootstrap.nonce_low = nonce_low;
  if (!receiveBootstrapFds(connection, bootstrap.data_fd, bootstrap.ack_fd) ||
      !fdModeIs(bootstrap.data_fd, O_RDONLY) ||
      !fdModeIs(bootstrap.ack_fd, O_RDWR) ||
      !fdSizeIs(bootstrap.data_fd, sizeof(SharedDataRegion)) ||
      !fdSizeIs(bootstrap.ack_fd, sizeof(SharedAckRegion)) ||
      !fdsAreDistinct(bootstrap.data_fd, bootstrap.ack_fd)) {
    closeBootstrap(bootstrap);
    return std::nullopt;
  }
  const int required = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  const int data_seals = fcntl(bootstrap.data_fd, F_GET_SEALS);
  const int ack_seals = fcntl(bootstrap.ack_fd, F_GET_SEALS);
  if (data_seals < 0 || ack_seals < 0 || (data_seals & required) != required ||
      (ack_seals & required) != required) {
    closeBootstrap(bootstrap);
    return std::nullopt;
  }
  return bootstrap;
}

void closeBootstrap(ReceivedBootstrap &bootstrap) noexcept {
  closeFd(bootstrap.control_socket_fd);
  closeFd(bootstrap.data_fd);
  closeFd(bootstrap.ack_fd);
}

std::optional<PeerCredentials> getPeerCredentials(int socket_fd) noexcept {
  struct ucred credentials {};
  socklen_t size = sizeof(credentials);
  if (getsockopt(socket_fd, SOL_SOCKET, SO_PEERCRED, &credentials, &size) !=
          0 ||
      size != sizeof(credentials)) {
    return std::nullopt;
  }
  return PeerCredentials{credentials.pid, credentials.uid, credentials.gid};
}

bool peerMatchesExpected(const PeerCredentials &credentials, pid_t expected_pid,
                         uid_t expected_uid, gid_t expected_gid) noexcept {
  return credentials.pid == expected_pid && credentials.uid == expected_uid &&
         credentials.gid == expected_gid;
}

bool consumeOneInChild(int data_fd, int ack_fd,
                       std::uint64_t expected_sequence) noexcept {
  const auto *data = static_cast<const SharedDataRegion *>(mmap(
      nullptr, sizeof(SharedDataRegion), PROT_READ, MAP_SHARED, data_fd, 0));
  auto *ack = static_cast<SharedAckRegion *>(
      mmap(nullptr, sizeof(SharedAckRegion), PROT_READ | PROT_WRITE, MAP_SHARED,
           ack_fd, 0));
  if (data == MAP_FAILED || ack == MAP_FAILED ||
      !validateSharedAbi(*data, *ack)) {
    if (data != MAP_FAILED) {
      munmap(const_cast<SharedDataRegion *>(data), sizeof(SharedDataRegion));
    }
    if (ack != MAP_FAILED) {
      munmap(ack, sizeof(SharedAckRegion));
    }
    return false;
  }
  FixedSnapshot snapshot{};
  bool matched = false;
  for (std::size_t attempt = 0U; attempt < 100000U; ++attempt) {
    if (tryPop(*data, *ack, snapshot) == PopResult::kPopped) {
      matched = snapshot.controller_sample_key.controller_sequence ==
                expected_sequence;
      break;
    }
    sched_yield();
  }
  munmap(const_cast<SharedDataRegion *>(data), sizeof(SharedDataRegion));
  munmap(ack, sizeof(SharedAckRegion));
  return matched;
}

bool applyPublisherParentDeathSignal(pid_t expected_supervisor_pid) noexcept {
  return expected_supervisor_pid > 1 && prctl(PR_SET_PDEATHSIG, SIGKILL) == 0 &&
         getppid() == expected_supervisor_pid;
}

bool applyPublisherResourceIsolation() noexcept {
  struct sched_param idle {};
  if (sched_setscheduler(0, SCHED_IDLE, &idle) != 0 ||
      sched_getscheduler(0) != SCHED_IDLE) {
    return false;
  }
  const struct rlimit address_space {
    kPublisherAddressSpaceLimitBytes, kPublisherAddressSpaceLimitBytes
  };
  const struct rlimit nofile {
    kPublisherOpenFileLimit, kPublisherOpenFileLimit
  };
  if (setrlimit(RLIMIT_AS, &address_space) != 0 ||
      setrlimit(RLIMIT_NOFILE, &nofile) != 0) {
    return false;
  }
  struct rlimit verified_address_space {};
  struct rlimit verified_nofile {};
  return getrlimit(RLIMIT_AS, &verified_address_space) == 0 &&
         getrlimit(RLIMIT_NOFILE, &verified_nofile) == 0 &&
         verified_address_space.rlim_cur == kPublisherAddressSpaceLimitBytes &&
         verified_nofile.rlim_cur == kPublisherOpenFileLimit;
}

ChildTerminationOutcome
terminateOneShotChild(pid_t child, std::chrono::milliseconds shutdown_deadline,
                      std::chrono::milliseconds terminate_grace) noexcept {
  if (child <= 0) {
    return ChildTerminationOutcome::kError;
  }
  int status = 0;
  if (childExited(child, &status)) {
    return ChildTerminationOutcome::kAlreadyExited;
  }
  if (kill(child, SIGINT) != 0 && errno != ESRCH) {
    return ChildTerminationOutcome::kError;
  }
  if (waitBounded(child, shutdown_deadline, &status)) {
    return ChildTerminationOutcome::kGraceful;
  }
  if (kill(child, SIGTERM) != 0 && errno != ESRCH) {
    return ChildTerminationOutcome::kError;
  }
  if (waitBounded(child, terminate_grace, &status)) {
    return ChildTerminationOutcome::kTerminated;
  }
  if (kill(child, SIGKILL) != 0 && errno != ESRCH) {
    return ChildTerminationOutcome::kError;
  }
  if (waitBounded(child, terminate_grace, &status)) {
    return ChildTerminationOutcome::kKilled;
  }
  return ChildTerminationOutcome::kKillUnconfirmed;
}

int supervisorExitCodeForChildTermination(
    ChildTerminationOutcome outcome) noexcept {
  switch (outcome) {
  case ChildTerminationOutcome::kAlreadyExited:
  case ChildTerminationOutcome::kGraceful:
    return 0;
  case ChildTerminationOutcome::kTerminated:
  case ChildTerminationOutcome::kKilled:
    return kSupervisorForcedCleanupExitCode;
  case ChildTerminationOutcome::kKillUnconfirmed:
  case ChildTerminationOutcome::kError:
    return kSupervisorUnconfirmedCleanupExitCode;
  }
  return kSupervisorUnconfirmedCleanupExitCode;
}

} // namespace simple_pure_pursuit::aw2_shadow
