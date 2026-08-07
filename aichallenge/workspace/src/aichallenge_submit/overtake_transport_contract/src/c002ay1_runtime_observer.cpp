#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <utility>

namespace overtake_transport_contract::c002ay1 {
namespace {

std::uint64_t loadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t desired) noexcept {
  __atomic_store_n(&value, desired, __ATOMIC_RELEASE);
}

bool compareExchange(std::uint64_t &value, std::uint64_t expected,
                     std::uint64_t desired) noexcept {
  return __atomic_compare_exchange_n(&value, &expected, desired, false,
                                     __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

void incrementRelaxed(std::uint64_t &value) noexcept {
  (void)__atomic_fetch_add(&value, 1U, __ATOMIC_RELAXED);
}

RuntimeRingHeaderV1 *headerFrom(void *mapping) noexcept {
  return static_cast<RuntimeRingHeaderV1 *>(mapping);
}

std::size_t recordSizeForRole(ProducerRole role) noexcept {
  switch (role) {
  case ProducerRole::kPlanner:
    return sizeof(PlannerCallbackObservationV1);
  case ProducerRole::kPrimaryPurePursuit:
    return sizeof(PpCallbackObservationV1);
  default:
    return 0U;
  }
}

std::size_t capacityForRole(ProducerRole role) noexcept {
  switch (role) {
  case ProducerRole::kPlanner:
    return kPlannerRingCapacity;
  case ProducerRole::kPrimaryPurePursuit:
    return kPpRingCapacity;
  default:
    return 0U;
  }
}

std::byte *recordBytes(void *mapping, ProducerRole role,
                       std::size_t index) noexcept {
  if (role == ProducerRole::kPlanner) {
    auto *ring = static_cast<PlannerRuntimeRingV1 *>(mapping);
    return reinterpret_cast<std::byte *>(&ring->records[index]);
  }
  auto *ring = static_cast<PpRuntimeRingV1 *>(mapping);
  return reinterpret_cast<std::byte *>(&ring->records[index]);
}

bool isTerminal(std::uint64_t state) noexcept {
  return state == static_cast<std::uint64_t>(RingState::kComplete) ||
         state == static_cast<std::uint64_t>(RingState::kInvalid);
}

bool isKnownState(std::uint64_t state) noexcept {
  return state <= static_cast<std::uint64_t>(RingState::kInvalid);
}

bool isKnownReturnReason(std::uint16_t reason) noexcept {
  switch (static_cast<ReturnReason>(reason)) {
  case ReturnReason::kCompletedNoEmit:
  case ReturnReason::kCompletedEmit:
  case ReturnReason::kEarlyInputUnavailable:
  case ReturnReason::kEarlyInputStale:
  case ReturnReason::kEarlyInputInvalid:
  case ReturnReason::kExternalAuthorityStop:
  case ReturnReason::kExistingExceptionGuard:
  case ReturnReason::kUnknownInvalid:
    return true;
  default:
    return false;
  }
}

bool flagsAreKnown(std::uint16_t flags) noexcept {
  constexpr std::uint16_t kKnownFlags =
      kFlagEmitted | kFlagIdentityCaptured | kFlagDropSeen;
  return (flags & static_cast<std::uint16_t>(~kKnownFlags)) == 0U;
}

bool plannerRecordIsValid(const PlannerCallbackObservationV1 &record) noexcept {
  const auto stream =
      static_cast<ConfiguredStreamKind>(record.configured_stream_kind);
  return (stream == ConfiguredStreamKind::kLegacyReferenceOverride ||
          stream == ConfiguredStreamKind::kV2Trajectory) &&
         isKnownReturnReason(record.return_reason) &&
         flagsAreKnown(record.flags) && record.legacy_publish_count <= 1U &&
         record.v2_publish_count <= 1U && record.selected_proposal_count <= 1U;
}

bool ppRecordIsValid(const PpCallbackObservationV1 &record) noexcept {
  return record.controller_role ==
             static_cast<std::uint8_t>(ProducerRole::kPrimaryPurePursuit) &&
         isKnownReturnReason(record.return_reason) &&
         flagsAreKnown(record.flags);
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

bool waitForEvent(int fd, short events,
                  std::chrono::steady_clock::time_point deadline) noexcept {
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int timeout_ms = std::max(1, static_cast<int>(remaining.count()));
    pollfd descriptor{fd, events, 0};
    const int result = poll(&descriptor, 1U, timeout_ms);
    if (result > 0) {
      if ((descriptor.revents & (POLLERR | POLLNVAL)) != 0) {
        return false;
      }
      // A one-shot collector closes its peer immediately after sending the
      // sealed memfd.  Linux may therefore report POLLIN and POLLHUP
      // together even though the complete SOCK_SEQPACKET payload is ready.
      // recvmsg() still validates the exact envelope and descriptor below.
      return (descriptor.revents & events) != 0;
    }
    if (result == 0) {
      return false;
    }
    if (errno != EINTR) {
      return false;
    }
  }
}

bool socketPeerIsSameUid(int fd) noexcept {
  ucred credentials{};
  socklen_t length = sizeof(credentials);
  return getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &credentials, &length) == 0 &&
         length == sizeof(credentials) && credentials.uid == geteuid();
}

bool fdIsSealedMemfd(int fd, std::size_t expected_size) noexcept {
  struct stat status {};
  if (fstat(fd, &status) != 0 || !S_ISREG(status.st_mode) ||
      status.st_size != static_cast<off_t>(expected_size)) {
    return false;
  }
  const int seals = fcntl(fd, F_GET_SEALS);
  const int required = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (seals < 0 || (seals & required) != required) {
    return false;
  }
  std::array<char, 64U> fd_path{};
  std::array<char, 256U> target{};
  const int written =
      std::snprintf(fd_path.data(), fd_path.size(), "/proc/self/fd/%d", fd);
  if (written <= 0 || static_cast<std::size_t>(written) >= fd_path.size()) {
    return false;
  }
  const ssize_t length =
      readlink(fd_path.data(), target.data(), target.size() - 1U);
  if (length <= 0) {
    return false;
  }
  target[static_cast<std::size_t>(length)] = '\0';
  return std::strstr(target.data(), "memfd:") != nullptr;
}

void closeReceivedRights(msghdr &message) noexcept {
  for (cmsghdr *header = CMSG_FIRSTHDR(&message); header != nullptr;
       header = CMSG_NXTHDR(&message, header)) {
    if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
        header->cmsg_len < CMSG_LEN(sizeof(int))) {
      continue;
    }
    const std::size_t descriptor_count =
        (header->cmsg_len - CMSG_LEN(0U)) / sizeof(int);
    const auto *data = reinterpret_cast<const std::byte *>(CMSG_DATA(header));
    for (std::size_t index = 0U; index < descriptor_count; ++index) {
      int received_fd = -1;
      std::memcpy(&received_fd, data + index * sizeof(int), sizeof(int));
      if (received_fd >= 0) {
        ::close(received_fd);
      }
    }
  }
}

int receiveSingleFd(int socket_fd,
                    std::chrono::steady_clock::time_point deadline) noexcept {
  if (!waitForEvent(socket_fd, POLLIN, deadline)) {
    return -1;
  }
  std::uint8_t payload = 0U;
  iovec io{&payload, sizeof(payload)};
  alignas(cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int))> control{};
  msghdr message{};
  message.msg_iov = &io;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  const ssize_t received =
      recvmsg(socket_fd, &message, MSG_DONTWAIT | MSG_CMSG_CLOEXEC);
  if (received != static_cast<ssize_t>(sizeof(payload)) || payload != 1U ||
      (message.msg_flags & (MSG_CTRUNC | MSG_TRUNC)) != 0) {
    closeReceivedRights(message);
    return -1;
  }
  cmsghdr *header = CMSG_FIRSTHDR(&message);
  if (header == nullptr || CMSG_NXTHDR(&message, header) != nullptr ||
      header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS ||
      header->cmsg_len != CMSG_LEN(sizeof(int))) {
    closeReceivedRights(message);
    return -1;
  }
  int fd = -1;
  std::memcpy(&fd, CMSG_DATA(header), sizeof(fd));
  return fd;
}

bool sendHandshake(int socket_fd, const RuntimeObserverConfig &config,
                   std::chrono::steady_clock::time_point deadline) noexcept {
  if (config.run_id.empty() || config.run_id.size() > kMaxRunIdBytes) {
    return false;
  }
  RuntimeHandshakeV1 handshake{};
  handshake.producer_role = static_cast<std::uint8_t>(config.role);
  handshake.producer_pid = static_cast<std::int32_t>(getpid());
  handshake.run_id_size = static_cast<std::uint32_t>(config.run_id.size());
  handshake.session_nonce = config.session_nonce;
  handshake.producer_instance_id = config.producer_instance_id;
  std::memcpy(handshake.run_id.data(), config.run_id.data(),
              config.run_id.size());
  if (!waitForEvent(socket_fd, POLLOUT, deadline)) {
    return false;
  }
  return send(socket_fd, &handshake, sizeof(handshake),
              MSG_DONTWAIT | MSG_NOSIGNAL) ==
         static_cast<ssize_t>(sizeof(handshake));
}

bool headerMatches(const RuntimeRingHeaderV1 &header, std::size_t mapping_size,
                   const RuntimeObserverConfig &config) noexcept {
  return header.magic == kRuntimeRingMagic &&
         header.abi_version == kRuntimeAbiVersion &&
         header.layout_version == kRuntimeLayoutVersion &&
         header.header_size == sizeof(RuntimeRingHeaderV1) &&
         header.record_size == recordSizeForRole(config.role) &&
         header.capacity == capacityForRole(config.role) &&
         header.producer_role == static_cast<std::uint8_t>(config.role) &&
         header.mapping_size == mapping_size &&
         header.session_nonce == config.session_nonce &&
         header.producer_instance_id == config.producer_instance_id &&
         header.run_id_hash ==
             hashRunId(config.run_id.data(), config.run_id.size());
}

} // namespace

std::uint64_t steadyNowNs() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::uint64_t hashRunId(const char *data, std::size_t size) noexcept {
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= static_cast<std::uint8_t>(data[index]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::size_t mappingSizeForRole(ProducerRole role) noexcept {
  switch (role) {
  case ProducerRole::kPlanner:
    return sizeof(PlannerRuntimeRingV1);
  case ProducerRole::kPrimaryPurePursuit:
    return sizeof(PpRuntimeRingV1);
  default:
    return 0U;
  }
}

bool runtimeAtomicsAreLockFree() noexcept {
  std::uint64_t value = 0U;
  return __atomic_is_lock_free(sizeof(value), &value);
}

bool initializeRuntimeRingForHarness(void *mapping, std::size_t mapping_size,
                                     ProducerRole role,
                                     std::uint64_t session_nonce,
                                     std::uint64_t producer_instance_id,
                                     std::uint64_t run_id_hash) noexcept {
  const std::size_t expected_size = mappingSizeForRole(role);
  if (mapping == nullptr || expected_size == 0U ||
      mapping_size != expected_size || session_nonce == 0U ||
      producer_instance_id == 0U || run_id_hash == 0U ||
      !runtimeAtomicsAreLockFree()) {
    return false;
  }
  std::memset(mapping, 0, mapping_size);
  auto &header = *headerFrom(mapping);
  header.magic = kRuntimeRingMagic;
  header.abi_version = kRuntimeAbiVersion;
  header.layout_version = kRuntimeLayoutVersion;
  header.header_size = sizeof(RuntimeRingHeaderV1);
  header.record_size = recordSizeForRole(role);
  header.capacity = capacityForRole(role);
  header.producer_role = static_cast<std::uint8_t>(role);
  header.mapping_size = mapping_size;
  header.session_nonce = session_nonce;
  header.producer_instance_id = producer_instance_id;
  header.run_id_hash = run_id_hash;
  storeRelease(header.state,
               static_cast<std::uint64_t>(RingState::kHarnessReady));
  return true;
}

RuntimeObservationWriter
RuntimeObservationWriter::attach(const RuntimeObserverConfig &config) noexcept {
  RuntimeObservationWriter disabled;
  if (!config.enabled || config.role == ProducerRole::kInvalid ||
      config.session_nonce == 0U || config.producer_instance_id == 0U ||
      mappingSizeForRole(config.role) == 0U || config.socket_path.empty() ||
      config.run_id.empty() || config.run_id.size() > kMaxRunIdBytes ||
      !runtimeAtomicsAreLockFree()) {
    return disabled;
  }

  sockaddr_un address{};
  if (!makeSocketAddress(config.socket_path, address)) {
    return disabled;
  }
  const auto deadline = std::chrono::steady_clock::now() + kAttachDeadline;
  int socket_fd =
      socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_fd < 0) {
    return disabled;
  }
  const int connect_result =
      connect(socket_fd, reinterpret_cast<sockaddr *>(&address),
              sizeof(sa_family_t) + config.socket_path.size() + 1U);
  if (connect_result != 0) {
    if (errno != EINPROGRESS || !waitForEvent(socket_fd, POLLOUT, deadline)) {
      ::close(socket_fd);
      return disabled;
    }
    int socket_error = 0;
    socklen_t error_size = sizeof(socket_error);
    if (getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &socket_error,
                   &error_size) != 0 ||
        socket_error != 0) {
      ::close(socket_fd);
      return disabled;
    }
  }
  if (!socketPeerIsSameUid(socket_fd) ||
      !sendHandshake(socket_fd, config, deadline)) {
    ::close(socket_fd);
    return disabled;
  }
  int ring_fd = receiveSingleFd(socket_fd, deadline);
  ::close(socket_fd);
  const std::size_t mapping_size = mappingSizeForRole(config.role);
  if (ring_fd < 0 || !fdIsSealedMemfd(ring_fd, mapping_size)) {
    if (ring_fd >= 0) {
      ::close(ring_fd);
    }
    return disabled;
  }
  void *mapping = mmap(nullptr, mapping_size, PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_POPULATE, ring_fd, 0);
  if (mapping == MAP_FAILED) {
    ::close(ring_fd);
    return disabled;
  }
  return validateMapped(mapping, mapping_size, ring_fd, true, config);
}

RuntimeObservationWriter RuntimeObservationWriter::attachMappedForTest(
    void *mapping, std::size_t mapping_size,
    const RuntimeObserverConfig &config) noexcept {
  if (!config.enabled) {
    return {};
  }
  return validateMapped(mapping, mapping_size, -1, false, config);
}

RuntimeObservationWriter RuntimeObservationWriter::validateMapped(
    void *mapping, std::size_t mapping_size, int owned_fd, bool owns_mapping,
    const RuntimeObserverConfig &config) noexcept {
  RuntimeObservationWriter writer;
  if (mapping == nullptr || mapping == MAP_FAILED ||
      !headerMatches(*headerFrom(mapping), mapping_size, config)) {
    if (owns_mapping && mapping != nullptr && mapping != MAP_FAILED) {
      munmap(mapping, mapping_size);
    }
    if (owned_fd >= 0) {
      ::close(owned_fd);
    }
    return writer;
  }
  auto &header = *headerFrom(mapping);
  if (!compareExchange(
          header.state, static_cast<std::uint64_t>(RingState::kHarnessReady),
          static_cast<std::uint64_t>(RingState::kProducerAttached))) {
    if (owns_mapping) {
      munmap(mapping, mapping_size);
    }
    if (owned_fd >= 0) {
      ::close(owned_fd);
    }
    return writer;
  }
  writer.mapping_ = mapping;
  writer.mapping_size_ = mapping_size;
  writer.owned_fd_ = owned_fd;
  writer.owns_mapping_ = owns_mapping;
  writer.role_ = config.role;
  writer.last_read_index_ = loadAcquire(header.read_index);
  return writer;
}

RuntimeObservationWriter::~RuntimeObservationWriter() { close(); }

RuntimeObservationWriter::RuntimeObservationWriter(
    RuntimeObservationWriter &&other) noexcept {
  *this = std::move(other);
}

RuntimeObservationWriter &
RuntimeObservationWriter::operator=(RuntimeObservationWriter &&other) noexcept {
  if (this == &other) {
    return *this;
  }
  close();
  mapping_ = other.mapping_;
  mapping_size_ = other.mapping_size_;
  owned_fd_ = other.owned_fd_;
  owns_mapping_ = other.owns_mapping_;
  role_ = other.role_;
  configured_stream_kind_ = other.configured_stream_kind_;
  last_read_index_ = other.last_read_index_;
  sequence_ = other.sequence_;
  other.mapping_ = nullptr;
  other.mapping_size_ = 0U;
  other.owned_fd_ = -1;
  other.owns_mapping_ = false;
  other.role_ = ProducerRole::kInvalid;
  other.configured_stream_kind_ = ConfiguredStreamKind::kNone;
  return *this;
}

bool RuntimeObservationWriter::tryWrite(
    const PlannerCallbackObservationV1 &record) noexcept {
  if (role_ != ProducerRole::kPlanner || !plannerRecordIsValid(record)) {
    invalidate();
    return false;
  }
  const auto stream =
      static_cast<ConfiguredStreamKind>(record.configured_stream_kind);
  if (configured_stream_kind_ == ConfiguredStreamKind::kNone) {
    configured_stream_kind_ = stream;
  } else if (configured_stream_kind_ != stream) {
    invalidate();
    return false;
  }
  return tryWriteRaw(&record, sizeof(record));
}

bool RuntimeObservationWriter::tryWrite(
    const PpCallbackObservationV1 &record) noexcept {
  if (role_ != ProducerRole::kPrimaryPurePursuit || !ppRecordIsValid(record)) {
    invalidate();
    return false;
  }
  return tryWriteRaw(&record, sizeof(record));
}

bool RuntimeObservationWriter::tryWriteRaw(const void *record,
                                           std::size_t record_size) noexcept {
  if (mapping_ == nullptr || record == nullptr ||
      record_size != recordSizeForRole(role_)) {
    return false;
  }
  auto &header = *headerFrom(mapping_);
  std::uint64_t state = loadAcquire(header.state);
  if (!isKnownState(state)) {
    invalidate();
    return false;
  }
  if (isTerminal(state)) {
    return false;
  }
  if (state == static_cast<std::uint64_t>(RingState::kProducerAttached)) {
    if (!compareExchange(
            header.state,
            static_cast<std::uint64_t>(RingState::kProducerAttached),
            static_cast<std::uint64_t>(RingState::kRecording))) {
      state = loadAcquire(header.state);
      if (state != static_cast<std::uint64_t>(RingState::kRecording)) {
        if (!isTerminal(state)) {
          invalidate();
        }
        return false;
      }
    }
  } else if (state != static_cast<std::uint64_t>(RingState::kRecording)) {
    invalidate();
    return false;
  }

  const std::uint64_t write_index = loadAcquire(header.write_index);
  const std::uint64_t read_index = loadAcquire(header.read_index);
  const std::size_t capacity = capacityForRole(role_);
  if (read_index < last_read_index_ || read_index > write_index ||
      write_index - read_index > capacity ||
      write_index == std::numeric_limits<std::uint64_t>::max()) {
    invalidate();
    return false;
  }
  last_read_index_ = read_index;
  if (write_index - read_index == capacity) {
    incrementRelaxed(header.drop_count);
    return false;
  }
  const std::size_t slot = static_cast<std::size_t>(write_index % capacity);
  std::memcpy(recordBytes(mapping_, role_, slot), record, record_size);
  storeRelease(header.write_index, write_index + 1U);
  return true;
}

std::uint64_t RuntimeObservationWriter::observedDropCount() const noexcept {
  return mapping_ == nullptr ? 0U
                             : loadAcquire(headerFrom(mapping_)->drop_count);
}

void RuntimeObservationWriter::invalidate() noexcept {
  if (mapping_ == nullptr) {
    return;
  }
  auto &state = headerFrom(mapping_)->state;
  for (;;) {
    const std::uint64_t current = loadAcquire(state);
    if (isTerminal(current)) {
      return;
    }
    if (compareExchange(state, current,
                        static_cast<std::uint64_t>(RingState::kInvalid))) {
      return;
    }
  }
}

void RuntimeObservationWriter::close() noexcept {
  if (owns_mapping_ && mapping_ != nullptr) {
    munmap(mapping_, mapping_size_);
  }
  if (owned_fd_ >= 0) {
    ::close(owned_fd_);
  }
  mapping_ = nullptr;
  mapping_size_ = 0U;
  owned_fd_ = -1;
  owns_mapping_ = false;
  role_ = ProducerRole::kInvalid;
  configured_stream_kind_ = ConfiguredStreamKind::kNone;
}

PlannerObservationScope::PlannerObservationScope(
    RuntimeObservationWriter &writer, std::int32_t ros_sec,
    std::uint32_t ros_nanosec, ConfiguredStreamKind stream) noexcept {
  if (!writer.enabled() || writer.role() != ProducerRole::kPlanner) {
    return;
  }
  writer_ = &writer;
  record_.sequence = writer.nextSequence();
  record_.steady_start_ns = steadyNowNs();
  record_.ros_sec = ros_sec;
  record_.ros_nanosec = ros_nanosec;
  record_.configured_stream_kind = static_cast<std::uint8_t>(stream);
  record_.return_reason =
      static_cast<std::uint16_t>(ReturnReason::kUnknownInvalid);
}

PlannerObservationScope::~PlannerObservationScope() {
  if (writer_ == nullptr) {
    return;
  }
  const std::uint64_t end_ns = steadyNowNs();
  record_.duration_ns =
      end_ns >= record_.steady_start_ns ? end_ns - record_.steady_start_ns : 0U;
  if (writer_->observedDropCount() != 0U) {
    record_.flags |= kFlagDropSeen;
  }
  (void)writer_->tryWrite(record_);
}

void PlannerObservationScope::setReturnReason(ReturnReason reason) noexcept {
  record_.return_reason = static_cast<std::uint16_t>(reason);
}

PpObservationScope::PpObservationScope(RuntimeObservationWriter &writer,
                                       std::int32_t ros_sec,
                                       std::uint32_t ros_nanosec) noexcept {
  if (!writer.enabled() || writer.role() != ProducerRole::kPrimaryPurePursuit) {
    return;
  }
  writer_ = &writer;
  record_.sequence = writer.nextSequence();
  record_.steady_start_ns = steadyNowNs();
  record_.ros_sec = ros_sec;
  record_.ros_nanosec = ros_nanosec;
  record_.controller_role =
      static_cast<std::uint8_t>(ProducerRole::kPrimaryPurePursuit);
  record_.return_reason =
      static_cast<std::uint16_t>(ReturnReason::kUnknownInvalid);
}

PpObservationScope::~PpObservationScope() {
  if (writer_ == nullptr) {
    return;
  }
  const std::uint64_t end_ns = steadyNowNs();
  record_.duration_ns =
      end_ns >= record_.steady_start_ns ? end_ns - record_.steady_start_ns : 0U;
  if (writer_->observedDropCount() != 0U) {
    record_.flags |= kFlagDropSeen;
  }
  (void)writer_->tryWrite(record_);
}

void PpObservationScope::setReturnReason(ReturnReason reason) noexcept {
  record_.return_reason = static_cast<std::uint16_t>(reason);
}

} // namespace overtake_transport_contract::c002ay1
