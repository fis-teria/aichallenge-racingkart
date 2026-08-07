#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <linux/memfd.h>
#include <optional>
#include <poll.h>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace c002ay1 = overtake_transport_contract::c002ay1;

namespace {

struct Options {
  std::string socket_path;
  std::string ready_file;
  std::string artifact_path;
  std::string run_id;
  std::uint64_t planner_nonce{0U};
  std::uint64_t planner_instance{0U};
  std::uint64_t pp_nonce{0U};
  std::uint64_t pp_instance{0U};
  std::size_t planner_quota{2000U};
  std::size_t pp_quota{10000U};
  std::uint64_t timeout_sec{180U};
  std::uint64_t drain_period_ms{5U};
  std::size_t planner_max_batch{4U};
  std::size_t pp_max_batch{16U};
  int nice_value{10};
};

struct CollectorDiagnostics {
  pid_t pid{0};
  pid_t tid{0};
  int scheduler_policy{-1};
  int nice_value{0};
  std::vector<int> affinity_cpus;
  int initial_cpu{-1};
  int final_cpu{-1};
  std::uint64_t iterations{0U};
  std::uint64_t planner_max_batch_observed{0U};
  std::uint64_t pp_max_batch_observed{0U};
  std::uint64_t timer_expiration_count{0U};
  std::uint64_t missed_deadline_count{0U};
  std::uint64_t catch_up_count{0U};
  std::uint64_t thread_cpu_ns{0U};
  std::uint64_t wall_ns{0U};
  pid_t planner_peer_pid{0};
  pid_t pp_peer_pid{0};
};

std::optional<std::uint64_t> parseUnsigned(const char *text) {
  if (text == nullptr || *text == '\0') {
    return std::nullopt;
  }
  char *end = nullptr;
  errno = 0;
  const auto value = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(value);
}

std::optional<Options> parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index + 1 < argc; index += 2) {
    const std::string key = argv[index];
    const char *value = argv[index + 1];
    if (key == "--socket") {
      options.socket_path = value;
    } else if (key == "--ready-file") {
      options.ready_file = value;
    } else if (key == "--artifact") {
      options.artifact_path = value;
    } else if (key == "--run-id") {
      options.run_id = value;
    } else {
      const auto parsed = parseUnsigned(value);
      if (!parsed.has_value()) {
        return std::nullopt;
      }
      if (key == "--planner-nonce") {
        options.planner_nonce = parsed.value();
      } else if (key == "--planner-instance") {
        options.planner_instance = parsed.value();
      } else if (key == "--pp-nonce") {
        options.pp_nonce = parsed.value();
      } else if (key == "--pp-instance") {
        options.pp_instance = parsed.value();
      } else if (key == "--planner-quota") {
        options.planner_quota = static_cast<std::size_t>(parsed.value());
      } else if (key == "--pp-quota") {
        options.pp_quota = static_cast<std::size_t>(parsed.value());
      } else if (key == "--timeout-sec") {
        options.timeout_sec = parsed.value();
      } else if (key == "--drain-period-ms") {
        options.drain_period_ms = parsed.value();
      } else if (key == "--planner-max-batch") {
        options.planner_max_batch = static_cast<std::size_t>(parsed.value());
      } else if (key == "--pp-max-batch") {
        options.pp_max_batch = static_cast<std::size_t>(parsed.value());
      } else if (key == "--nice") {
        if (parsed.value() > 19U) {
          return std::nullopt;
        }
        options.nice_value = static_cast<int>(parsed.value());
      } else {
        return std::nullopt;
      }
    }
  }
  if ((argc - 1) % 2 != 0 || options.socket_path.empty() ||
      options.ready_file.empty() || options.artifact_path.empty() ||
      options.run_id.empty() ||
      options.run_id.size() > c002ay1::kMaxRunIdBytes ||
      options.planner_nonce == 0U || options.planner_instance == 0U ||
      options.pp_nonce == 0U || options.pp_instance == 0U ||
      options.planner_quota == 0U || options.pp_quota == 0U ||
      options.timeout_sec == 0U || options.drain_period_ms == 0U ||
      options.drain_period_ms > 1000U || options.planner_max_batch == 0U ||
      options.planner_max_batch > c002ay1::kPlannerRingCapacity ||
      options.pp_max_batch == 0U ||
      options.pp_max_batch > c002ay1::kPpRingCapacity) {
    return std::nullopt;
  }
  return options;
}

std::uint64_t timespecToNs(const timespec &value) {
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

timespec addNs(timespec value, std::uint64_t nanoseconds) {
  const std::uint64_t total =
      static_cast<std::uint64_t>(value.tv_nsec) + nanoseconds;
  value.tv_sec += static_cast<time_t>(total / 1000000000ULL);
  value.tv_nsec = static_cast<long>(total % 1000000000ULL);
  return value;
}

bool atOrAfter(const timespec &lhs, const timespec &rhs) {
  return lhs.tv_sec > rhs.tv_sec ||
         (lhs.tv_sec == rhs.tv_sec && lhs.tv_nsec >= rhs.tv_nsec);
}

std::optional<CollectorDiagnostics>
configureCollectorScheduler(const Options &options) {
  if (sched_getscheduler(0) != SCHED_OTHER ||
      setpriority(PRIO_PROCESS, 0, options.nice_value) != 0) {
    return std::nullopt;
  }
  errno = 0;
  const int effective_nice = getpriority(PRIO_PROCESS, 0);
  if (errno != 0 || effective_nice < options.nice_value) {
    return std::nullopt;
  }
  cpu_set_t affinity;
  CPU_ZERO(&affinity);
  if (sched_getaffinity(0, sizeof(affinity), &affinity) != 0) {
    return std::nullopt;
  }
  CollectorDiagnostics diagnostics;
  diagnostics.pid = getpid();
  diagnostics.tid = static_cast<pid_t>(syscall(SYS_gettid));
  diagnostics.scheduler_policy = sched_getscheduler(0);
  diagnostics.nice_value = effective_nice;
  diagnostics.initial_cpu = sched_getcpu();
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &affinity)) {
      diagnostics.affinity_cpus.push_back(cpu);
    }
  }
  if (diagnostics.affinity_cpus.empty() || diagnostics.initial_cpu < 0) {
    return std::nullopt;
  }
  return diagnostics;
}

std::uint64_t loadAcquire(const std::uint64_t &value) {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t desired) {
  __atomic_store_n(&value, desired, __ATOMIC_RELEASE);
}

bool compareExchange(std::uint64_t &value, std::uint64_t expected,
                     std::uint64_t desired) {
  return __atomic_compare_exchange_n(&value, &expected, desired, false,
                                     __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

class HarnessRing {
public:
  static std::optional<HarnessRing> create(c002ay1::ProducerRole role,
                                           std::uint64_t nonce,
                                           std::uint64_t instance,
                                           std::uint64_t run_id_hash) {
    HarnessRing ring;
    ring.role_ = role;
    ring.size_ = c002ay1::mappingSizeForRole(role);
    ring.fd_ = static_cast<int>(syscall(SYS_memfd_create,
                                        role == c002ay1::ProducerRole::kPlanner
                                            ? "c002ay1-planner"
                                            : "c002ay1-primary-pp",
                                        MFD_CLOEXEC | MFD_ALLOW_SEALING));
    if (ring.fd_ < 0 ||
        ftruncate(ring.fd_, static_cast<off_t>(ring.size_)) != 0) {
      return std::nullopt;
    }
    ring.mapping_ = mmap(nullptr, ring.size_, PROT_READ | PROT_WRITE,
                         MAP_SHARED | MAP_POPULATE, ring.fd_, 0);
    if (ring.mapping_ == MAP_FAILED ||
        !c002ay1::initializeRuntimeRingForHarness(
            ring.mapping_, ring.size_, role, nonce, instance, run_id_hash)) {
      return std::nullopt;
    }
    volatile std::byte *bytes =
        static_cast<volatile std::byte *>(ring.mapping_);
    for (std::size_t offset = 0U; offset < ring.size_;
         offset += static_cast<std::size_t>(sysconf(_SC_PAGESIZE))) {
      (void)bytes[offset];
    }
    const int required = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
    if (fcntl(ring.fd_, F_ADD_SEALS, required) != 0) {
      return std::nullopt;
    }
    return ring;
  }

  HarnessRing() = default;
  ~HarnessRing() {
    if (mapping_ != MAP_FAILED) {
      munmap(mapping_, size_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }
  HarnessRing(const HarnessRing &) = delete;
  HarnessRing &operator=(const HarnessRing &) = delete;
  HarnessRing(HarnessRing &&other) noexcept { *this = std::move(other); }
  HarnessRing &operator=(HarnessRing &&other) noexcept {
    if (this == &other) {
      return *this;
    }
    role_ = other.role_;
    size_ = other.size_;
    fd_ = other.fd_;
    mapping_ = other.mapping_;
    other.fd_ = -1;
    other.mapping_ = MAP_FAILED;
    other.size_ = 0U;
    return *this;
  }

  int fd() const { return fd_; }
  c002ay1::RuntimeRingHeaderV1 &header() {
    return *static_cast<c002ay1::RuntimeRingHeaderV1 *>(mapping_);
  }
  const c002ay1::RuntimeRingHeaderV1 &header() const {
    return *static_cast<const c002ay1::RuntimeRingHeaderV1 *>(mapping_);
  }
  c002ay1::PlannerRuntimeRingV1 &planner() {
    return *static_cast<c002ay1::PlannerRuntimeRingV1 *>(mapping_);
  }
  c002ay1::PpRuntimeRingV1 &pp() {
    return *static_cast<c002ay1::PpRuntimeRingV1 *>(mapping_);
  }

private:
  c002ay1::ProducerRole role_{c002ay1::ProducerRole::kInvalid};
  std::size_t size_{0U};
  int fd_{-1};
  void *mapping_{MAP_FAILED};
};

std::optional<ucred> peerCredentials(int fd) {
  ucred credentials{};
  socklen_t length = sizeof(credentials);
  if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &credentials, &length) != 0 ||
      length != sizeof(credentials) || credentials.uid != geteuid()) {
    return std::nullopt;
  }
  return credentials;
}

bool sendSingleFd(int socket_fd, int ring_fd) {
  std::uint8_t payload = 1U;
  iovec io{&payload, sizeof(payload)};
  alignas(cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(int))> control{};
  msghdr message{};
  message.msg_iov = &io;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  cmsghdr *header = CMSG_FIRSTHDR(&message);
  header->cmsg_level = SOL_SOCKET;
  header->cmsg_type = SCM_RIGHTS;
  header->cmsg_len = CMSG_LEN(sizeof(int));
  std::memcpy(CMSG_DATA(header), &ring_fd, sizeof(ring_fd));
  return sendmsg(socket_fd, &message, MSG_NOSIGNAL) ==
         static_cast<ssize_t>(sizeof(payload));
}

bool handshakeMatches(const c002ay1::RuntimeHandshakeV1 &handshake,
                      const Options &options, c002ay1::ProducerRole role,
                      pid_t peer_pid) {
  const bool planner = role == c002ay1::ProducerRole::kPlanner;
  return handshake.magic == c002ay1::kRuntimeHandshakeMagic &&
         handshake.abi_version == c002ay1::kRuntimeAbiVersion &&
         handshake.producer_role == static_cast<std::uint8_t>(role) &&
         handshake.producer_pid > 0 && handshake.producer_pid == peer_pid &&
         handshake.run_id_size == options.run_id.size() &&
         handshake.run_id_size <= c002ay1::kMaxRunIdBytes &&
         std::memcmp(handshake.run_id.data(), options.run_id.data(),
                     options.run_id.size()) == 0 &&
         handshake.session_nonce ==
             (planner ? options.planner_nonce : options.pp_nonce) &&
         handshake.producer_instance_id ==
             (planner ? options.planner_instance : options.pp_instance);
}

int createServer(const std::string &path) {
  if (path.size() >= sizeof(sockaddr_un::sun_path)) {
    return -1;
  }
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path());
  chmod(std::filesystem::path(path).parent_path().c_str(), 0700);
  unlink(path.c_str());
  const int fd =
      socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);
  if (bind(fd, reinterpret_cast<sockaddr *>(&address),
           sizeof(sa_family_t) + path.size() + 1U) != 0 ||
      chmod(path.c_str(), 0600) != 0 || listen(fd, 2) != 0) {
    close(fd);
    unlink(path.c_str());
    return -1;
  }
  return fd;
}

bool acceptProducer(int server_fd, const Options &options,
                    HarnessRing &planner_ring, HarnessRing &pp_ring,
                    bool &planner_attached, bool &pp_attached,
                    pid_t &planner_peer_pid, pid_t &pp_peer_pid,
                    std::chrono::steady_clock::time_point deadline) {
  pollfd descriptor{server_fd, POLLIN, 0};
  const int ready = poll(&descriptor, 1U, 20);
  if (ready <= 0) {
    return std::chrono::steady_clock::now() < deadline;
  }
  const int peer =
      accept4(server_fd, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
  if (peer < 0) {
    return errno == EAGAIN || errno == EINTR;
  }
  pollfd peer_descriptor{peer, POLLIN, 0};
  const int peer_ready = poll(&peer_descriptor, 1U, 100);
  c002ay1::RuntimeHandshakeV1 handshake{};
  iovec handshake_io{&handshake, sizeof(handshake)};
  msghdr handshake_message{};
  handshake_message.msg_iov = &handshake_io;
  handshake_message.msg_iovlen = 1U;
  const ssize_t received =
      peer_ready > 0 ? recvmsg(peer, &handshake_message, MSG_DONTWAIT) : -1;
  const auto role = static_cast<c002ay1::ProducerRole>(handshake.producer_role);
  const auto peer_credentials = peerCredentials(peer);
  bool accepted = peer_credentials.has_value() &&
                  received == static_cast<ssize_t>(sizeof(handshake)) &&
                  (handshake_message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) == 0;
  if (accepted && role == c002ay1::ProducerRole::kPlanner &&
      !planner_attached &&
      handshakeMatches(handshake, options, role, peer_credentials->pid)) {
    accepted = sendSingleFd(peer, planner_ring.fd());
    planner_attached = accepted;
    if (accepted) {
      planner_peer_pid = peer_credentials->pid;
    }
  } else if (accepted && role == c002ay1::ProducerRole::kPrimaryPurePursuit &&
             !pp_attached &&
             handshakeMatches(handshake, options, role,
                              peer_credentials->pid)) {
    accepted = sendSingleFd(peer, pp_ring.fd());
    pp_attached = accepted;
    if (accepted) {
      pp_peer_pid = peer_credentials->pid;
    }
  } else {
    accepted = false;
  }
  close(peer);
  return accepted || std::chrono::steady_clock::now() < deadline;
}

template <typename Ring, typename Record>
std::size_t drainRing(Ring &ring, std::vector<Record> &records,
                      std::size_t quota, std::size_t max_batch) {
  auto &header = ring.header;
  if (loadAcquire(header.state) !=
      static_cast<std::uint64_t>(c002ay1::RingState::kRecording)) {
    return 0U;
  }
  const std::size_t initial_size = records.size();
  std::uint64_t read_index = loadAcquire(header.read_index);
  const std::uint64_t write_index = loadAcquire(header.write_index);
  while (read_index < write_index && records.size() < quota &&
         records.size() - initial_size < max_batch) {
    records.push_back(ring.records[read_index % header.capacity]);
    ++read_index;
  }
  storeRelease(header.read_index, read_index);
  return records.size() - initial_size;
}

void completeIfRecording(c002ay1::RuntimeRingHeaderV1 &header) {
  (void)compareExchange(
      header.state, static_cast<std::uint64_t>(c002ay1::RingState::kRecording),
      static_cast<std::uint64_t>(c002ay1::RingState::kComplete));
}

bool writeArtifact(
    const Options &options, const HarnessRing &planner_ring,
    const HarnessRing &pp_ring,
    const std::vector<c002ay1::PlannerCallbackObservationV1> &planner,
    const std::vector<c002ay1::PpCallbackObservationV1> &pp,
    const CollectorDiagnostics &diagnostics, bool complete) {
  std::filesystem::create_directories(
      std::filesystem::path(options.artifact_path).parent_path());
  std::ofstream output(options.artifact_path, std::ios::trunc);
  if (!output) {
    return false;
  }
  const auto &planner_header = planner_ring.header();
  const auto &pp_header = pp_ring.header();
  output << "{\n"
         << "  \"schema\":\"C-002AY1-PROD-MEASURE-001A\",\n"
         << "  \"run_id\":\"" << options.run_id << "\",\n"
         << "  \"complete\":" << (complete ? "true" : "false") << ",\n"
         << "  \"authority_eligible\":false,\n"
         << "  \"collector_scheduler\":{\"pid\":" << diagnostics.pid
         << ",\"tid\":" << diagnostics.tid
         << ",\"policy\":\"SCHED_OTHER\",\"nice\":" << diagnostics.nice_value
         << ",\"affinity_cpus\":[";
  for (std::size_t index = 0U; index < diagnostics.affinity_cpus.size();
       ++index) {
    output << (index == 0U ? "" : ",") << diagnostics.affinity_cpus[index];
  }
  output << "],\"initial_cpu\":" << diagnostics.initial_cpu
         << ",\"final_cpu\":" << diagnostics.final_cpu << "},\n"
         << "  \"collector_drain\":{\"period_ms\":" << options.drain_period_ms
         << ",\"planner_max_batch_limit\":" << options.planner_max_batch
         << ",\"pp_max_batch_limit\":" << options.pp_max_batch
         << ",\"planner_max_batch_observed\":"
         << diagnostics.planner_max_batch_observed
         << ",\"pp_max_batch_observed\":" << diagnostics.pp_max_batch_observed
         << ",\"iterations\":" << diagnostics.iterations
         << ",\"thread_cpu_ns\":" << diagnostics.thread_cpu_ns
         << ",\"wall_ns\":" << diagnostics.wall_ns
         << ",\"timer_expiration_count\":" << diagnostics.timer_expiration_count
         << ",\"missed_deadline_count\":" << diagnostics.missed_deadline_count
         << ",\"catch_up_count\":" << diagnostics.catch_up_count << "},\n"
         << "  \"producer_peers\":{\"planner_pid\":"
         << diagnostics.planner_peer_pid
         << ",\"pp_pid\":" << diagnostics.pp_peer_pid << "},\n"
         << "  \"planner_count\":" << planner.size() << ",\n"
         << "  \"pp_count\":" << pp.size() << ",\n"
         << "  \"planner_drop_count\":"
         << loadAcquire(planner_header.drop_count) << ",\n"
         << "  \"pp_drop_count\":" << loadAcquire(pp_header.drop_count) << ",\n"
         << "  \"planner_state\":" << loadAcquire(planner_header.state) << ",\n"
         << "  \"pp_state\":" << loadAcquire(pp_header.state) << ",\n"
         << "  \"planner_records\":[";
  for (std::size_t index = 0U; index < planner.size(); ++index) {
    const auto &record = planner[index];
    output << (index == 0U ? "\n" : ",\n")
           << "    {\"sequence\":" << record.sequence
           << ",\"steady_start_ns\":" << record.steady_start_ns
           << ",\"duration_ns\":" << record.duration_ns
           << ",\"ros_sec\":" << record.ros_sec
           << ",\"ros_nanosec\":" << record.ros_nanosec
           << ",\"selected_proposal_sequence\":"
           << record.selected_proposal_sequence
           << ",\"cadence_identity_hint_token\":"
           << record.cadence_identity_hint_token
           << ",\"return_reason\":" << record.return_reason
           << ",\"flags\":" << record.flags << ",\"configured_stream_kind\":"
           << static_cast<unsigned>(record.configured_stream_kind)
           << ",\"legacy_publish_count\":"
           << static_cast<unsigned>(record.legacy_publish_count)
           << ",\"v2_publish_count\":"
           << static_cast<unsigned>(record.v2_publish_count)
           << ",\"selected_proposal_count\":"
           << static_cast<unsigned>(record.selected_proposal_count) << "}";
  }
  output << (planner.empty() ? "" : "\n") << "  ],\n"
         << "  \"pp_records\":[";
  for (std::size_t index = 0U; index < pp.size(); ++index) {
    const auto &record = pp[index];
    output << (index == 0U ? "\n" : ",\n")
           << "    {\"sequence\":" << record.sequence
           << ",\"steady_start_ns\":" << record.steady_start_ns
           << ",\"duration_ns\":" << record.duration_ns
           << ",\"ros_sec\":" << record.ros_sec
           << ",\"ros_nanosec\":" << record.ros_nanosec
           << ",\"return_reason\":" << record.return_reason
           << ",\"flags\":" << record.flags << ",\"controller_role\":"
           << static_cast<unsigned>(record.controller_role) << "}";
  }
  output << (pp.empty() ? "" : "\n") << "  ]\n"
         << "}\n";
  return static_cast<bool>(output);
}

} // namespace

int main(int argc, char **argv) {
  const auto options = parseOptions(argc, argv);
  if (!options.has_value()) {
    std::cerr << "invalid collector arguments\n";
    return 2;
  }
  auto diagnostics = configureCollectorScheduler(options.value());
  if (!diagnostics.has_value()) {
    std::cerr << "collector scheduler configuration unavailable\n";
    return 6;
  }
  const auto run_hash =
      c002ay1::hashRunId(options->run_id.data(), options->run_id.size());
  auto planner_ring = HarnessRing::create(c002ay1::ProducerRole::kPlanner,
                                          options->planner_nonce,
                                          options->planner_instance, run_hash);
  auto pp_ring =
      HarnessRing::create(c002ay1::ProducerRole::kPrimaryPurePursuit,
                          options->pp_nonce, options->pp_instance, run_hash);
  if (!planner_ring.has_value() || !pp_ring.has_value()) {
    return 3;
  }
  const int server_fd = createServer(options->socket_path);
  if (server_fd < 0) {
    return 4;
  }
  {
    std::ofstream ready(options->ready_file, std::ios::trunc);
    ready << "ready\n";
  }

  std::vector<c002ay1::PlannerCallbackObservationV1> planner_records;
  std::vector<c002ay1::PpCallbackObservationV1> pp_records;
  planner_records.reserve(options->planner_quota);
  pp_records.reserve(options->pp_quota);
  bool planner_attached = false;
  bool pp_attached = false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(options->timeout_sec);

  while (std::chrono::steady_clock::now() < deadline &&
         (!planner_attached || !pp_attached)) {
    if (!acceptProducer(server_fd, options.value(), planner_ring.value(),
                        pp_ring.value(), planner_attached, pp_attached,
                        diagnostics->planner_peer_pid, diagnostics->pp_peer_pid,
                        deadline)) {
      break;
    }
  }
  timespec drain_wall_start{};
  timespec drain_cpu_start{};
  timespec next_wake{};
  clock_gettime(CLOCK_MONOTONIC, &drain_wall_start);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &drain_cpu_start);
  next_wake = addNs(drain_wall_start, options->drain_period_ms * 1000000ULL);
  while (std::chrono::steady_clock::now() < deadline && planner_attached &&
         pp_attached &&
         (planner_records.size() < options->planner_quota ||
          pp_records.size() < options->pp_quota)) {
    const std::size_t planner_batch =
        drainRing(planner_ring->planner(), planner_records,
                  options->planner_quota, options->planner_max_batch);
    const std::size_t pp_batch = drainRing(
        pp_ring->pp(), pp_records, options->pp_quota, options->pp_max_batch);
    diagnostics->planner_max_batch_observed =
        std::max(diagnostics->planner_max_batch_observed,
                 static_cast<std::uint64_t>(planner_batch));
    diagnostics->pp_max_batch_observed =
        std::max(diagnostics->pp_max_batch_observed,
                 static_cast<std::uint64_t>(pp_batch));
    ++diagnostics->iterations;
    int sleep_result = 0;
    do {
      sleep_result =
          clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);
    } while (sleep_result == EINTR);
    if (sleep_result != 0) {
      break;
    }
    ++diagnostics->timer_expiration_count;
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    next_wake = addNs(next_wake, options->drain_period_ms * 1000000ULL);
    if (atOrAfter(now, next_wake)) {
      ++diagnostics->missed_deadline_count;
      next_wake = addNs(now, options->drain_period_ms * 1000000ULL);
    }
  }
  const std::size_t final_planner_batch =
      drainRing(planner_ring->planner(), planner_records,
                options->planner_quota, options->planner_max_batch);
  const std::size_t final_pp_batch = drainRing(
      pp_ring->pp(), pp_records, options->pp_quota, options->pp_max_batch);
  diagnostics->planner_max_batch_observed =
      std::max(diagnostics->planner_max_batch_observed,
               static_cast<std::uint64_t>(final_planner_batch));
  diagnostics->pp_max_batch_observed =
      std::max(diagnostics->pp_max_batch_observed,
               static_cast<std::uint64_t>(final_pp_batch));
  timespec drain_wall_end{};
  timespec drain_cpu_end{};
  clock_gettime(CLOCK_MONOTONIC, &drain_wall_end);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &drain_cpu_end);
  diagnostics->wall_ns =
      timespecToNs(drain_wall_end) - timespecToNs(drain_wall_start);
  diagnostics->thread_cpu_ns =
      timespecToNs(drain_cpu_end) - timespecToNs(drain_cpu_start);
  diagnostics->final_cpu = sched_getcpu();
  const bool complete = planner_records.size() >= options->planner_quota &&
                        pp_records.size() >= options->pp_quota &&
                        loadAcquire(planner_ring->header().drop_count) == 0U &&
                        loadAcquire(pp_ring->header().drop_count) == 0U;
  if (complete) {
    completeIfRecording(planner_ring->header());
    completeIfRecording(pp_ring->header());
  }
  const bool artifact_written =
      writeArtifact(options.value(), planner_ring.value(), pp_ring.value(),
                    planner_records, pp_records, diagnostics.value(), complete);
  close(server_fd);
  unlink(options->socket_path.c_str());
  return complete && artifact_written ? 0 : 5;
}
