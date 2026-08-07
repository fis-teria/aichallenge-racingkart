#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp"

#include "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <linux/memfd.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <sched.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <thread>
#include <time.h>
#include <tuple>
#include <unistd.h>

extern char **environ;

namespace state_lattice_overtake_planner::c002ay0_shadow {
namespace {

using ShutdownOutcome =
    overtake_transport_contract::c002ay0::WorkerShutdownOutcome;
using ShutdownBudget =
    overtake_transport_contract::c002ay0::WorkerShutdownBudget;

constexpr int kWorkerDataFd = 3;
constexpr std::uint64_t kWorkerAddressSpaceLimitBytes =
    1024ULL * 1024ULL * 1024ULL;
constexpr rlim_t kWorkerOpenFileLimit = 128U;
// The shadow proposal carries a 50 ms safety lifetime.  Keep queue residence
// well below that lifetime so an otherwise current proposal cannot expire
// solely while waiting for this isolated publisher.
constexpr auto kPublishInterval = std::chrono::milliseconds(10);
constexpr std::uint64_t kMaximumPublishQueueAgeNs = 20000000ULL;

std::uint64_t loadAcquire(const std::uint64_t &value) noexcept {
  return __atomic_load_n(&value, __ATOMIC_ACQUIRE);
}

void storeRelease(std::uint64_t &value, std::uint64_t next) noexcept {
  __atomic_store_n(&value, next, __ATOMIC_RELEASE);
}

void incrementRelaxed(std::uint64_t &value) noexcept {
  (void)__atomic_fetch_add(&value, 1U, __ATOMIC_RELAXED);
}

std::uint64_t monotonicNanoseconds() noexcept {
  timespec stamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0 || stamp.tv_sec < 0 ||
      stamp.tv_nsec < 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.tv_nsec);
}

bool parseU64(const char *text, std::uint64_t *value) noexcept {
  if (text == nullptr || value == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const auto parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

bool reliableProposalAuditEnabled() noexcept {
  const char *value = std::getenv("C002AY0_TEST_RELIABLE_PROPOSAL_AUDIT");
  return value != nullptr && std::strcmp(value, "1") == 0;
}

std::string proposalAuditJson(const FixedProposalRecord &record,
                              std::uint64_t dequeue_monotonic_ns,
                              std::uint64_t build_begin_monotonic_ns,
                              std::uint64_t publish_call_entry_monotonic_ns,
                              std::uint64_t publish_call_return_monotonic_ns) {
  // This fixture-only payload deliberately carries only fixed-record identity
  // and CLOCK_MONOTONIC boundaries; it cannot influence planner authority.
  return "{\"schema_version\":1,\"proposal_race_arm_epoch\":" +
         std::to_string(record.base.race_arm_epoch) +
         ",\"proposal_planner_instance_id\":" +
         std::to_string(record.planner_instance_id) +
         ",\"proposal_attempt_id\":" + std::to_string(record.attempt_id) +
         ",\"proposal_connector_transaction_id\":" +
         std::to_string(record.connector_transaction_id) +
         ",\"proposal_plan_generation\":" +
         std::to_string(record.plan_generation) +
         ",\"proposal_candidate_revision\":" +
         std::to_string(record.candidate_revision) +
         ",\"proposal_authority_token\":" +
         std::to_string(record.authority_token) +
         ",\"proposal_safety_snapshot_id\":" +
         std::to_string(record.safety_snapshot_id) +
         ",\"proposal_controller_instance_id\":" +
         std::to_string(record.base.controller_instance_id) +
         ",\"proposal_controller_sequence\":" +
         std::to_string(record.base.controller_sequence) +
         ",\"proposal_base_lease_id\":" +
         std::to_string(record.base.base_lease_id) +
         ",\"capture_monotonic_ns\":" +
         std::to_string(record.capture_monotonic_ns) +
         ",\"worker_dequeue_monotonic_ns\":" +
         std::to_string(dequeue_monotonic_ns) +
         ",\"worker_build_begin_monotonic_ns\":" +
         std::to_string(build_begin_monotonic_ns) +
         ",\"publish_call_entry_monotonic_ns\":" +
         std::to_string(publish_call_entry_monotonic_ns) +
         ",\"publish_call_return_monotonic_ns\":" +
         std::to_string(publish_call_return_monotonic_ns) + "}";
}

int createMemfd(const char *name, std::size_t size) noexcept {
  const int fd = static_cast<int>(
      syscall(SYS_memfd_create, name, MFD_CLOEXEC | MFD_ALLOW_SEALING));
  if (fd < 0 || ftruncate(fd, static_cast<off_t>(size)) != 0) {
    if (fd >= 0) {
      (void)::close(fd);
    }
    return -1;
  }
  return fd;
}

bool workerIsolation() noexcept {
  sched_param idle{};
  if (sched_setscheduler(0, SCHED_IDLE, &idle) != 0 ||
      sched_getscheduler(0) != SCHED_IDLE ||
      prctl(PR_SET_PDEATHSIG, SIGKILL) != 0) {
    return false;
  }
  const rlimit address_space{kWorkerAddressSpaceLimitBytes,
                             kWorkerAddressSpaceLimitBytes};
  const rlimit open_files{kWorkerOpenFileLimit, kWorkerOpenFileLimit};
  return setrlimit(RLIMIT_AS, &address_space) == 0 &&
         setrlimit(RLIMIT_NOFILE, &open_files) == 0;
}

bool finite(double value) noexcept { return std::isfinite(value); }

bool copyFixedString(const std::array<char, kFixedProposalIdCapacity> &input,
                     std::uint8_t size, std::string *output) {
  if (output == nullptr || size == 0U || size > input.size()) {
    return false;
  }
  *output = std::string(input.data(), size);
  return true;
}

builtin_interfaces::msg::Time toRosTime(const FixedTime &value) {
  builtin_interfaces::msg::Time result;
  result.sec = value.sec;
  result.nanosec = value.nanosec;
  return result;
}

bool restoreRecord(
    const FixedProposalRecord &record,
    const FixedProposalStaticConfig &static_config,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot *base,
    CandidateTrajectory *candidate, EgoState *ego,
    std::vector<OpponentState> *opponents, PlannerConfig *config,
    Ay0ShadowSafetyEvidence *evidence, Ay0ShadowProposalIdentity *identity) {
  if (base == nullptr || candidate == nullptr || ego == nullptr ||
      opponents == nullptr || config == nullptr || evidence == nullptr ||
      identity == nullptr || record.candidate_feasible != 1U ||
      record.candidate_point_count < 2U ||
      record.candidate_point_count > record.candidate_points.size() ||
      record.opponent_count > record.opponents.size() ||
      !copyFixedString(record.target_id, record.target_id_size,
                       &identity->target_id) ||
      overtake_transport_contract::c002ay0::buildBaseSnapshotFromFixedRecord(
          record.base, *base) !=
          overtake_transport_contract::c002ay0::ValidationError::NONE ||
      base->base_source_sha256 != record.incoming_base.base_source_sha256 ||
      base->base_geometry_sha256 != record.incoming_base.base_geometry_sha256 ||
      base->snapshot_sha256 != record.incoming_base.base_snapshot_sha256) {
    return false;
  }
  for (double value :
       {record.ego.x_m, record.ego.y_m, record.ego.yaw_rad,
        record.ego.stamp_sec, record.ego.speed_mps, record.ego.yaw_rate_radps,
        record.ego.curvature_radpm, record.ego.frenet_s_m,
        record.ego.frenet_d_m, record.ego.frenet_yaw_error_rad,
        record.candidate_goal_d_m}) {
    if (!finite(value)) {
      return false;
    }
  }
  config->safety_evaluation_enabled =
      static_config.safety_evaluation_enabled == 1U;
  config->map_frame =
      std::string(static_config.frame.data(), static_config.frame_size);
  config->wheel_base_m = static_config.wheel_base_m;
  config->ego_stale_sec = static_config.ego_stale_sec;
  ego->x = record.ego.x_m;
  ego->y = record.ego.y_m;
  ego->yaw = record.ego.yaw_rad;
  ego->stamp_sec = record.ego.stamp_sec;
  ego->speed_mps = record.ego.speed_mps;
  ego->yaw_rate_radps = record.ego.yaw_rate_radps;
  ego->curvature = record.ego.curvature_radpm;
  ego->frenet.s = record.ego.frenet_s_m;
  ego->frenet.d = record.ego.frenet_d_m;
  ego->frenet.yaw_error = record.ego.frenet_yaw_error_rad;
  ego->frenet.segment_index = record.ego.frenet_segment_index;
  ego->frenet.valid = record.ego.frenet_valid == 1U;
  ego->valid = record.ego.valid == 1U;
  if (!ego->valid || !ego->frenet.valid) {
    return false;
  }
  candidate->goal_d_m = record.candidate_goal_d_m;
  candidate->feasible = true;
  candidate->dense.reserve(record.candidate_point_count);
  double previous_time = -1.0;
  for (std::size_t index = 0U; index < record.candidate_point_count; ++index) {
    const auto &source = record.candidate_points[index];
    if (!finite(source.x_m) || !finite(source.y_m) || !finite(source.yaw_rad) ||
        !finite(source.s_m) || !finite(source.d_m) ||
        !finite(source.kappa_radpm) || !finite(source.speed_mps) ||
        !finite(source.time_sec) || source.speed_mps < 0.0 ||
        source.time_sec < 0.0 || source.time_sec <= previous_time) {
      return false;
    }
    candidate->dense.push_back({source.x_m, source.y_m, source.yaw_rad,
                                source.time_sec, 0.0, source.s_m, source.d_m,
                                source.kappa_radpm, source.speed_mps});
    previous_time = source.time_sec;
  }
  opponents->reserve(record.opponent_count);
  for (std::size_t index = 0U; index < record.opponent_count; ++index) {
    const auto &source = record.opponents[index];
    OpponentState opponent;
    if (!copyFixedString(source.id, source.id_size, &opponent.id) ||
        source.valid != 1U || source.frenet_valid != 1U) {
      return false;
    }
    for (double value :
         {source.x_m, source.y_m, source.yaw_rad, source.stamp_sec,
          source.speed_mps, source.vx_mps, source.vy_mps, source.sigma_x_m,
          source.sigma_y_m, source.uncertainty_x_m, source.uncertainty_y_m,
          source.frenet_s_m, source.frenet_d_m, source.frenet_yaw_error_rad}) {
      if (!finite(value)) {
        return false;
      }
    }
    opponent.x = source.x_m;
    opponent.y = source.y_m;
    opponent.yaw = source.yaw_rad;
    opponent.stamp_sec = source.stamp_sec;
    opponent.speed_mps = source.speed_mps;
    opponent.vx_mps = source.vx_mps;
    opponent.vy_mps = source.vy_mps;
    opponent.sigma_x_m = source.sigma_x_m;
    opponent.sigma_y_m = source.sigma_y_m;
    opponent.uncertainty_x_m = source.uncertainty_x_m;
    opponent.uncertainty_y_m = source.uncertainty_y_m;
    opponent.frenet.s = source.frenet_s_m;
    opponent.frenet.d = source.frenet_d_m;
    opponent.frenet.yaw_error = source.frenet_yaw_error_rad;
    opponent.frenet.segment_index = source.frenet_segment_index;
    opponent.frenet.valid = true;
    opponent.valid = true;
    opponents->push_back(std::move(opponent));
  }
  evidence->evaluator_implementation_sha256 =
      record.evaluator_implementation_sha256;
  evidence->evaluator_config_sha256 = record.evaluator_config_sha256;
  if (evidence->evaluator_implementation_sha256 !=
          static_config.evaluator_implementation_sha256 ||
      evidence->evaluator_config_sha256 !=
          static_config.evaluator_config_sha256) {
    return false;
  }
  identity->planner_instance_id = record.planner_instance_id;
  identity->attempt_id = record.attempt_id;
  identity->connector_transaction_id = record.connector_transaction_id;
  identity->authority_token = record.authority_token;
  identity->safety_snapshot_id = record.safety_snapshot_id;
  identity->plan_generation = record.plan_generation;
  identity->candidate_revision = record.candidate_revision;
  return true;
}

bool waitForChild(pid_t pid, std::chrono::milliseconds duration) noexcept {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  int status = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto result = waitpid(pid, &status, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) {
      return true;
    }
    if (result < 0 && errno != EINTR) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

} // namespace

struct FixedProposalWorkerSession::Impl {
  FixedProposalQueue *queue{nullptr};
  ~Impl() {
    if (queue != nullptr) {
      munmap(queue, sizeof(FixedProposalQueue));
    }
  }
};

std::unique_ptr<FixedProposalWorkerSession> FixedProposalWorkerSession::start(
    const FixedProposalWorkerConfig &config) noexcept {
  if (!config.enabled || config.executable_path.empty() ||
      config.session_generation == 0U || config.session_nonce == 0U ||
      config.handshake_timeout.count() <= 0 ||
      config.handshake_timeout.count() > 5000 ||
      !config.shutdown_budget.valid()) {
    return nullptr;
  }
  auto session = std::unique_ptr<FixedProposalWorkerSession>(
      new FixedProposalWorkerSession);
  session->shutdown_budget_ = config.shutdown_budget;
  session->impl_ = std::make_unique<Impl>();
  session->data_fd_ =
      createMemfd("c002sl-ay0-proposal", sizeof(FixedProposalQueue));
  if (session->data_fd_ < 0) {
    return nullptr;
  }
  session->impl_->queue = static_cast<FixedProposalQueue *>(
      mmap(nullptr, sizeof(FixedProposalQueue), PROT_READ | PROT_WRITE,
           MAP_SHARED, session->data_fd_, 0));
  if (session->impl_->queue == MAP_FAILED) {
    session->impl_->queue = nullptr;
    return nullptr;
  }
  if (!initializeFixedProposalQueue(
          *session->impl_->queue, kFixedProposalQueueCapacity,
          config.session_generation, config.session_nonce,
          config.static_config)) {
    return nullptr;
  }
  if (fcntl(session->data_fd_, F_ADD_SEALS,
            F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL) != 0) {
    return nullptr;
  }
  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0 ||
      posix_spawn_file_actions_adddup2(&actions, session->data_fd_,
                                       kWorkerDataFd) != 0) {
    (void)posix_spawn_file_actions_destroy(&actions);
    return nullptr;
  }
  const std::string parent_pid = std::to_string(getpid());
  const std::string generation = std::to_string(config.session_generation);
  const std::string nonce = std::to_string(config.session_nonce);
  const std::string worker_fd = std::to_string(kWorkerDataFd);
  char *const arguments[]{const_cast<char *>(config.executable_path.c_str()),
                          const_cast<char *>("--c002ay0-state-lattice-worker"),
                          const_cast<char *>(parent_pid.c_str()),
                          const_cast<char *>(worker_fd.c_str()),
                          const_cast<char *>(generation.c_str()),
                          const_cast<char *>(nonce.c_str()),
                          nullptr};
  const int spawn_result =
      posix_spawn(&session->worker_pid_, config.executable_path.c_str(),
                  &actions, nullptr, arguments, environ);
  (void)posix_spawn_file_actions_destroy(&actions);
  if (spawn_result != 0 || session->worker_pid_ <= 1) {
    session->worker_pid_ = -1;
    return nullptr;
  }
  const auto deadline =
      std::chrono::steady_clock::now() + config.handshake_timeout;
  while (loadAcquire(session->impl_->queue->worker_ready) == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    if (waitpid(session->worker_pid_, &status, WNOHANG) ==
        session->worker_pid_) {
      session->worker_pid_ = -1;
      return nullptr;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (loadAcquire(session->impl_->queue->worker_ready) == 0U) {
    (void)session->shutdown();
    return nullptr;
  }
  return session;
}

FixedProposalWorkerSession::~FixedProposalWorkerSession() {
  (void)shutdown();
  if (data_fd_ >= 0) {
    (void)::close(data_fd_);
  }
}

bool FixedProposalWorkerSession::tryCapture(
    const FixedProposalRecord &record) noexcept {
  return impl_ != nullptr && impl_->queue != nullptr && ready() &&
         tryPushFixedProposal(*impl_->queue, record) ==
             FixedProposalQueueResult::kAccepted;
}

bool FixedProposalWorkerSession::ready() const noexcept {
  if (impl_ == nullptr || impl_->queue == nullptr || worker_pid_ <= 1 ||
      loadAcquire(impl_->queue->accepting) == 0U ||
      loadAcquire(impl_->queue->worker_ready) == 0U) {
    return false;
  }
  int status = 0;
  return waitpid(worker_pid_, &status, WNOHANG) == 0;
}

std::uint64_t FixedProposalWorkerSession::droppedCount() const noexcept {
  return impl_ == nullptr || impl_->queue == nullptr
             ? 0U
             : loadAcquire(impl_->queue->dropped_full_count);
}

std::uint64_t FixedProposalWorkerSession::publishedCount() const noexcept {
  return impl_ == nullptr || impl_->queue == nullptr
             ? 0U
             : loadAcquire(impl_->queue->published_count);
}

std::uint64_t
FixedProposalWorkerSession::validationRejectCount() const noexcept {
  return impl_ == nullptr || impl_->queue == nullptr
             ? 0U
             : loadAcquire(impl_->queue->validation_reject_count);
}

FixedProposalWorkerDiagnostics
FixedProposalWorkerSession::diagnostics() const noexcept {
  FixedProposalWorkerDiagnostics result;
  if (impl_ == nullptr || impl_->queue == nullptr) {
    return result;
  }
  const auto &queue = *impl_->queue;
  result.accepting = loadAcquire(queue.accepting) != 0U;
  result.worker_ready = loadAcquire(queue.worker_ready) != 0U;
  result.dropped_full_count = loadAcquire(queue.dropped_full_count);
  result.invalid_record_count = loadAcquire(queue.invalid_record_count);
  result.validation_reject_count = loadAcquire(queue.validation_reject_count);
  result.serialization_reject_count =
      loadAcquire(queue.serialization_reject_count);
  result.coalesced_count = loadAcquire(queue.coalesced_count);
  result.published_count = loadAcquire(queue.published_count);
  const auto worker_started = loadAcquire(queue.worker_started_monotonic_ns);
  const auto monotonic_now = monotonicNanoseconds();
  if (worker_started != 0U && monotonic_now >= worker_started) {
    result.worker_process_age_ns = monotonic_now - worker_started;
  }
  result.last_publish_queue_age_ns =
      loadAcquire(queue.last_publish_queue_age_ns);
  return result;
}

ShutdownOutcome FixedProposalWorkerSession::shutdown() noexcept {
  if (shutdown_outcome_ != ShutdownOutcome::kNotStarted) {
    return shutdown_outcome_;
  }
  if (impl_ != nullptr && impl_->queue != nullptr) {
    disableFixedProposalQueue(*impl_->queue);
  }
  if (worker_pid_ <= 1) {
    return shutdown_outcome_;
  }
  if (waitForChild(worker_pid_, shutdown_budget_.drain)) {
    worker_pid_ = -1;
    shutdown_outcome_ = ShutdownOutcome::kDrained;
    return shutdown_outcome_;
  }
  for (const auto &[signal, duration, outcome] :
       std::array<std::tuple<int, std::chrono::milliseconds, ShutdownOutcome>,
                  3U>{std::make_tuple(SIGINT, shutdown_budget_.interrupt,
                                      ShutdownOutcome::kInterrupted),
                      std::make_tuple(SIGTERM, shutdown_budget_.terminate,
                                      ShutdownOutcome::kTerminated),
                      std::make_tuple(SIGKILL, shutdown_budget_.kill,
                                      ShutdownOutcome::kKilled)}) {
    if (kill(worker_pid_, signal) != 0 && errno != ESRCH) {
      shutdown_outcome_ = ShutdownOutcome::kError;
      return shutdown_outcome_;
    }
    if (waitForChild(worker_pid_, duration)) {
      worker_pid_ = -1;
      shutdown_outcome_ = outcome;
      return shutdown_outcome_;
    }
  }
  shutdown_outcome_ = ShutdownOutcome::kReapUnconfirmed;
  return shutdown_outcome_;
}

int runFixedProposalShadowWorker(int argc, char **argv) {
  if (argc != 6 ||
      std::strcmp(argv[1], "--c002ay0-state-lattice-worker") != 0) {
    return 2;
  }
  std::uint64_t parent_pid = 0U;
  std::uint64_t worker_fd = 0U;
  std::uint64_t session_generation = 0U;
  std::uint64_t session_nonce = 0U;
  if (!parseU64(argv[2], &parent_pid) || !parseU64(argv[3], &worker_fd) ||
      !parseU64(argv[4], &session_generation) ||
      !parseU64(argv[5], &session_nonce) ||
      parent_pid != static_cast<std::uint64_t>(getppid()) ||
      worker_fd != kWorkerDataFd || !workerIsolation()) {
    return 3;
  }
  auto *queue = static_cast<FixedProposalQueue *>(
      mmap(nullptr, sizeof(FixedProposalQueue), PROT_READ | PROT_WRITE,
           MAP_SHARED, kWorkerDataFd, 0));
  if (queue == MAP_FAILED || queue->magic != kFixedProposalQueueMagic ||
      queue->abi_version != kFixedProposalQueueAbiVersion ||
      queue->session_generation != session_generation ||
      queue->session_nonce != session_nonce || queue->capacity == 0U ||
      queue->capacity > kFixedProposalQueueCapacity ||
      !fixedProposalAtomicsAreLockFree()) {
    if (queue != MAP_FAILED) {
      (void)munmap(queue, sizeof(FixedProposalQueue));
    }
    return 4;
  }
  int ros_argc = 1;
  char *ros_argv[]{argv[0], nullptr};
  try {
    rclcpp::init(ros_argc, ros_argv);
    auto node =
        std::make_shared<rclcpp::Node>("c002ay0_state_lattice_shadow_worker");
    auto publisher = node->create_publisher<
        multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>(
        "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    const bool reliable_proposal_audit = reliableProposalAuditEnabled();
    auto proposal_audit_publisher = reliable_proposal_audit
        ? node->create_publisher<std_msgs::msg::String>(
              "/test/c002ay0/state_lattice/proposal_publish_audit",
              rclcpp::QoS(rclcpp::KeepLast(128)).reliable().durability_volatile())
        : rclcpp::Publisher<std_msgs::msg::String>::SharedPtr{};
    rclcpp::Serialization<
        multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory>
        serializer;
    const auto worker_started = monotonicNanoseconds();
    if (worker_started == 0U) {
      (void)munmap(queue, sizeof(FixedProposalQueue));
      return 5;
    }
    storeRelease(queue->worker_started_monotonic_ns, worker_started);
    storeRelease(queue->worker_ready, 1U);
    std::optional<std::pair<FixedProposalRecord, std::uint64_t>> pending;
    auto last_publish = std::chrono::steady_clock::time_point::min();
    while (rclcpp::ok()) {
      FixedProposalRecord record{};
      for (;;) {
        const auto result = tryPopFixedProposal(*queue, record);
        if (result == FixedProposalQueueResult::kAccepted) {
          if (pending.has_value()) {
            incrementRelaxed(queue->coalesced_count);
          }
          const auto dequeue_monotonic_ns = monotonicNanoseconds();
          if (dequeue_monotonic_ns == 0U) {
            incrementRelaxed(queue->validation_reject_count);
          } else {
            pending = std::make_pair(record, dequeue_monotonic_ns);
          }
          continue;
        }
        if (result == FixedProposalQueueResult::kInvalid ||
            result == FixedProposalQueueResult::kTorn) {
          incrementRelaxed(queue->validation_reject_count);
        }
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      if (pending.has_value()) {
        const auto monotonic_now_ns = monotonicNanoseconds();
        if (monotonic_now_ns == 0U ||
            monotonic_now_ns < pending->first.capture_monotonic_ns ||
            monotonic_now_ns - pending->first.capture_monotonic_ns >=
                kMaximumPublishQueueAgeNs) {
          pending.reset();
          incrementRelaxed(queue->validation_reject_count);
        }
      }
      if (pending.has_value() &&
          (last_publish == std::chrono::steady_clock::time_point::min() ||
           now - last_publish >= kPublishInterval)) {
        multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot base;
        CandidateTrajectory candidate;
        EgoState ego;
        std::vector<OpponentState> opponents;
        PlannerConfig config;
        Ay0ShadowSafetyEvidence evidence;
        Ay0ShadowProposalIdentity identity;
        const auto dequeue_monotonic_ns = pending->second;
        if (!restoreRecord(pending->first, queue->static_config, &base, &candidate,
                           &ego, &opponents, &config, &evidence, &identity)) {
          incrementRelaxed(queue->validation_reject_count);
        } else {
          const auto build_begin_monotonic_ns = monotonicNanoseconds();
          if (build_begin_monotonic_ns == 0U ||
              build_begin_monotonic_ns < dequeue_monotonic_ns) {
            incrementRelaxed(queue->validation_reject_count);
            pending.reset();
            continue;
          }
          const auto proposal = buildAy0ShadowProposal(
              base, candidate, ego, opponents, config, evidence,
              toRosTime(pending->first.plan_stamp), identity);
          if (!proposal.valid() || proposal.trajectory.authority_eligible) {
            incrementRelaxed(queue->validation_reject_count);
          } else {
            try {
              rclcpp::SerializedMessage serialized;
              serializer.serialize_message(&proposal.trajectory, &serialized);
              if (serialized.size() == 0U) {
                incrementRelaxed(queue->serialization_reject_count);
              } else {
                const auto publish_call_entry_monotonic_ns = monotonicNanoseconds();
                if (publish_call_entry_monotonic_ns == 0U ||
                    publish_call_entry_monotonic_ns < build_begin_monotonic_ns) {
                  incrementRelaxed(queue->validation_reject_count);
                  pending.reset();
                  continue;
                }
                publisher->publish(proposal.trajectory);
                const auto publish_call_return_monotonic_ns = monotonicNanoseconds();
                if (publish_call_return_monotonic_ns == 0U ||
                    publish_call_return_monotonic_ns < publish_call_entry_monotonic_ns) {
                  incrementRelaxed(queue->validation_reject_count);
                  pending.reset();
                  continue;
                }
                if (proposal_audit_publisher) {
                  std_msgs::msg::String audit;
                  audit.data = proposalAuditJson(
                      pending->first, dequeue_monotonic_ns, build_begin_monotonic_ns,
                      publish_call_entry_monotonic_ns,
                      publish_call_return_monotonic_ns);
                  proposal_audit_publisher->publish(audit);
                }
                incrementRelaxed(queue->published_count);
                const auto published_monotonic_ns = monotonicNanoseconds();
                if (published_monotonic_ns >= pending->first.capture_monotonic_ns) {
                  storeRelease(queue->last_publish_queue_age_ns,
                               published_monotonic_ns -
                                   pending->first.capture_monotonic_ns);
                }
                last_publish = now;
              }
            } catch (...) {
              incrementRelaxed(queue->serialization_reject_count);
            }
          }
        }
        pending.reset();
      }
      if (loadAcquire(queue->accepting) == 0U && !pending.has_value() &&
          loadAcquire(queue->read_index) == loadAcquire(queue->write_index)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    storeRelease(queue->worker_ready, 0U);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  } catch (...) {
    storeRelease(queue->worker_ready, 0U);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    (void)munmap(queue, sizeof(FixedProposalQueue));
    return 5;
  }
  (void)munmap(queue, sizeof(FixedProposalQueue));
  return 0;
}

} // namespace state_lattice_overtake_planner::c002ay0_shadow
