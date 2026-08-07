#include "c002ay0_fixed_ingress.hpp"
#include "c002ay0_phase1_fixture.hpp"
#include "c002ay1_cadence_gate.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <new>
#include <string>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>

namespace {
thread_local bool g_track_allocations = false;
thread_local std::size_t g_allocation_count = 0U;
} // namespace

void *operator new(const std::size_t size) {
  if (void *memory = std::malloc(size)) {
    if (g_track_allocations) {
      ++g_allocation_count;
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
namespace cadence = overtake_transport_contract::c002ay1::test_fixture;
using Trajectory = contract::AuthorizedCartesianTrajectory;
using Ingress = fixture::AuthorizedFixedIngress<3U>;
using PushResult = fixture::IngressPushResult;
using SteadyClock = std::chrono::steady_clock;

constexpr std::size_t kMaxCdrBytes = 24U * 1024U;
#if C002AY1_CADENCE_SEPARATED
constexpr bool kCadenceSeparated = true;
constexpr std::size_t kWarmupSamples = 200U;
constexpr std::size_t kMeasuredSamples = cadence::kProposalMeasuredSamples;
constexpr auto kPeriod = std::chrono::milliseconds(50);
constexpr char kFixtureName[] = "C002AY1_SEPARATED_CADENCE_RMW_V1";
constexpr char kTopic[] =
    "/c002ay0_measurement/cadence_authorized_cartesian_trajectory";
#else
constexpr bool kCadenceSeparated = false;
constexpr std::size_t kWarmupSamples = 1000U;
constexpr std::size_t kMeasuredSamples = 10000U;
constexpr auto kPeriod = std::chrono::milliseconds(10);
constexpr char kFixtureName[] = "C002AY0_PHASE1_WORKER_RMW_V1";
constexpr char kTopic[] =
    "/c002ay0_measurement/worker_authorized_cartesian_trajectory";
#endif
constexpr std::size_t kPpProbeWarmupSamples = 1000U;
constexpr auto kRunDeadline = std::chrono::seconds(180);
constexpr std::uint64_t kCallbackBudgetNs = 1000000ULL;
constexpr std::uint64_t kTimerGapBudgetNs = 12000000ULL;

std::uint64_t steadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch())
          .count());
}

std::string fingerprintFromEnvironment(const char *name) {
  const auto *value = std::getenv(name);
  if (value == nullptr) {
    return {};
  }
  const std::string fingerprint(value);
  const bool valid =
      fingerprint.size() == 64U &&
      std::all_of(fingerprint.begin(), fingerprint.end(), [](const char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
      });
  return valid ? fingerprint : std::string{};
}

template <std::size_t Size>
std::uint64_t percentile(const std::array<std::uint64_t, Size> &data,
                         const std::size_t count, const double fraction) {
  if (count == 0U) {
    return 0U;
  }
  std::array<std::uint64_t, Size> sorted{};
  std::copy_n(data.begin(), count, sorted.begin());
  std::sort(sorted.begin(), sorted.begin() + count);
  const auto rank = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(count)));
  return sorted[std::min(count - 1U, std::max<std::size_t>(1U, rank) - 1U)];
}

struct MeasurementState {
  std::array<std::uint64_t, kMeasuredSamples> end_to_end_ns{};
  std::array<std::uint64_t, kMeasuredSamples> callback_ns{};
  std::array<std::uint64_t, kMeasuredSamples> timer_gap_ns{};
  std::array<std::uint64_t, kMeasuredSamples> worker_ns{};
  std::array<std::uint64_t, kMeasuredSamples> worker_queue_age_ns{};
  std::array<std::uint64_t, cadence::kPpProbeMeasuredSamples> pp_probe_gap_ns{};
  std::array<std::uint64_t, cadence::kPpProbeMeasuredSamples>
      pp_probe_callback_ns{};
  SteadyClock::time_point publish_time{};
  SteadyClock::time_point last_timer_time{};
  std::size_t warmup_received{0U};
  std::size_t measured_received{0U};
  std::size_t measured_timer_gaps{0U};
  std::size_t sent{0U};
  std::size_t deadline_miss_inflight{0U};
  std::size_t unexpected_callbacks{0U};
  std::size_t ingress_busy{0U};
  std::size_t ingress_rejected{0U};
  std::size_t ingress_copy_allocations{0U};
  std::size_t pp_probe_warmup{0U};
  std::size_t pp_probe_measured{0U};
  std::atomic<std::size_t> worker_processed{0U};
  std::atomic<std::size_t> worker_measured_recorded{0U};
  std::atomic<std::size_t> worker_validation_failures{0U};
  std::atomic<std::size_t> worker_serialization_failures{0U};
  std::atomic<std::size_t> worker_expired{0U};
  std::atomic<std::size_t> worker_mutations{0U};
  std::atomic<std::size_t> worker_ledger_failures{0U};
  std::atomic<std::size_t> worker_exceptions{0U};
  std::atomic<bool> worker_low_priority_configured{false};
  bool inflight{false};
  bool have_last_timer{false};
  bool pp_probe_have_last_timer{false};
  SteadyClock::time_point pp_probe_last_timer_time{};
};

class MeasurementSubscriber : public rclcpp::Node {
public:
  MeasurementSubscriber(std::shared_ptr<MeasurementState> state,
                        std::shared_ptr<Ingress> ingress)
      : Node("c002ay0_worker_measurement_subscriber", "/c002ay0_measurement"),
        state_(std::move(state)), ingress_(std::move(ingress)) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    subscription_ = create_subscription<Trajectory>(
        kTopic, qos, [this](const Trajectory::ConstSharedPtr message) {
          const auto callback_start = SteadyClock::now();
          const auto allocations_before = g_allocation_count;
          g_track_allocations = true;
          const auto push_result =
              ingress_->tryPush(*message, ++enqueue_sequence_, steadyNowNs());
          g_track_allocations = false;
          state_->ingress_copy_allocations +=
              g_allocation_count - allocations_before;
          const auto callback_end = SteadyClock::now();
          if (push_result == PushResult::BUSY) {
            ++state_->ingress_busy;
          } else if (push_result != PushResult::INSERTED) {
            ++state_->ingress_rejected;
          }
          if (!state_->inflight) {
            ++state_->unexpected_callbacks;
            return;
          }
          if (state_->warmup_received < kWarmupSamples) {
            ++state_->warmup_received;
          } else if (state_->measured_received < kMeasuredSamples) {
            const auto index = state_->measured_received++;
            state_->end_to_end_ns[index] = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    callback_end - state_->publish_time)
                    .count());
            state_->callback_ns[index] = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    callback_end - callback_start)
                    .count());
          }
          state_->inflight = false;
        });
  }

private:
  std::shared_ptr<MeasurementState> state_;
  std::shared_ptr<Ingress> ingress_;
  std::uint64_t enqueue_sequence_{0U};
  rclcpp::Subscription<Trajectory>::SharedPtr subscription_;
};

class MeasurementPublisher : public rclcpp::Node {
public:
  MeasurementPublisher(std::shared_ptr<MeasurementState> state,
                       Trajectory trajectory)
      : Node("c002ay0_worker_measurement_publisher", "/c002ay0_measurement"),
        state_(std::move(state)), trajectory_(std::move(trajectory)) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    publisher_ = create_publisher<Trajectory>(kTopic, qos);
  }

  std::size_t subscriptionCount() const {
    return publisher_->get_subscription_count();
  }

  void start() {
    timer_ = create_wall_timer(kPeriod, [this]() {
      const auto now = SteadyClock::now();
      const bool measured = state_->warmup_received >= kWarmupSamples &&
                            state_->measured_received < kMeasuredSamples;
      if (measured && state_->have_last_timer &&
          state_->measured_timer_gaps < kMeasuredSamples) {
        state_->timer_gap_ns[state_->measured_timer_gaps++] =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now - state_->last_timer_time)
                    .count());
      }
      state_->last_timer_time = now;
      state_->have_last_timer = true;
      if (state_->inflight) {
        ++state_->deadline_miss_inflight;
        return;
      }
      state_->inflight = true;
      state_->publish_time = now;
      ++state_->sent;
      publisher_->publish(trajectory_);
      if (state_->measured_received >= kMeasuredSamples) {
        timer_->cancel();
      }
    });
  }

private:
  std::shared_ptr<MeasurementState> state_;
  Trajectory trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class PpCadenceProbe : public rclcpp::Node {
public:
  explicit PpCadenceProbe(std::shared_ptr<MeasurementState> state)
      : Node("c002ay1_pp_cadence_probe", "/c002ay0_measurement"),
        state_(std::move(state)) {}

  void start() {
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
      const auto callback_start = SteadyClock::now();
      if (state_->pp_probe_warmup < kPpProbeWarmupSamples) {
        ++state_->pp_probe_warmup;
      } else if (state_->pp_probe_measured < cadence::kPpProbeMeasuredSamples) {
        const auto index = state_->pp_probe_measured++;
        if (state_->pp_probe_have_last_timer) {
          state_->pp_probe_gap_ns[index] = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  callback_start - state_->pp_probe_last_timer_time)
                  .count());
        }
        state_->pp_probe_callback_ns[index] = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                SteadyClock::now() - callback_start)
                .count());
      }
      state_->pp_probe_last_timer_time = callback_start;
      state_->pp_probe_have_last_timer = true;
      if (state_->pp_probe_measured >= cadence::kPpProbeMeasuredSamples) {
        timer_->cancel();
      }
    });
  }

private:
  std::shared_ptr<MeasurementState> state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void workerLoop(const std::shared_ptr<Ingress> &ingress,
                const std::shared_ptr<MeasurementState> &state,
                const contract::ControllerBaseTrajectorySnapshot &snapshot,
                const std::uint64_t logical_ros_start_ns,
                const std::uint64_t steady_start_ns,
                const std::shared_ptr<std::atomic<bool>> &stop_requested) {
  const auto thread_id = static_cast<id_t>(::syscall(SYS_gettid));
  state->worker_low_priority_configured.store(
      ::setpriority(PRIO_PROCESS, thread_id, 19) == 0,
      std::memory_order_release);
  rclcpp::Serialization<Trajectory> serializer;
  fixture::BoundedMutationLedger<3U> mutation_ledger;
  while (!stop_requested->load(std::memory_order_acquire)) {
    Ingress::WorkerView view;
    if (!ingress->tryAcquire(view)) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    const auto worker_start = SteadyClock::now();
    try {
      const auto current_ros_time_ns =
          logical_ros_start_ns + (steadyNowNs() - steady_start_ns);
      if (view.trajectory == nullptr) {
        state->worker_validation_failures.fetch_add(1U,
                                                    std::memory_order_relaxed);
      } else if (fixture::workerInputExpired(*view.trajectory,
                                             current_ros_time_ns)) {
        state->worker_expired.fetch_add(1U, std::memory_order_relaxed);
      } else if (contract::validateTrajectoryAgainstBaseSnapshotV1(
                     snapshot, *view.trajectory) !=
                 contract::ValidationError::NONE) {
        state->worker_validation_failures.fetch_add(1U,
                                                    std::memory_order_relaxed);
      } else {
        const auto mutation_result = mutation_ledger.observe(*view.trajectory);
        if (mutation_result ==
            fixture::MutationLedgerResult::SAME_GENERATION_MUTATION) {
          state->worker_mutations.fetch_add(1U, std::memory_order_relaxed);
        } else if (mutation_result ==
                       fixture::MutationLedgerResult::EPOCH_REGRESSION ||
                   mutation_result ==
                       fixture::MutationLedgerResult::RESOURCE_LIMIT) {
          state->worker_ledger_failures.fetch_add(1U,
                                                  std::memory_order_relaxed);
        } else {
          rclcpp::SerializedMessage serialized(kMaxCdrBytes);
          serializer.serialize_message(view.trajectory, &serialized);
          if (serialized.size() > kMaxCdrBytes) {
            state->worker_serialization_failures.fetch_add(
                1U, std::memory_order_relaxed);
          }
        }
      }
    } catch (...) {
      state->worker_exceptions.fetch_add(1U, std::memory_order_relaxed);
    }
    const auto worker_end = SteadyClock::now();
    state->worker_processed.fetch_add(1U, std::memory_order_relaxed);
    if (view.enqueue_sequence > kWarmupSamples &&
        view.enqueue_sequence <= kWarmupSamples + kMeasuredSamples) {
      const auto index =
          static_cast<std::size_t>(view.enqueue_sequence - kWarmupSamples - 1U);
      state->worker_ns[index] = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(worker_end -
                                                               worker_start)
              .count());
      state->worker_queue_age_ns[index] =
          steadyNowNs() - view.receipt_steady_ns;
      state->worker_measured_recorded.fetch_add(1U, std::memory_order_relaxed);
    }
    if (!ingress->release(view)) {
      state->worker_exceptions.fetch_add(1U, std::memory_order_relaxed);
    }
  }
}

std::size_t cdrSize(const Trajectory &trajectory) {
  rclcpp::Serialization<Trajectory> serializer;
  rclcpp::SerializedMessage serialized(kMaxCdrBytes);
  serializer.serialize_message(&trajectory, &serialized);
  return serialized.size();
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto source_fingerprint =
      fingerprintFromEnvironment("C002AY_SOURCE_FINGERPRINT");
  const auto binary_fingerprint =
      fingerprintFromEnvironment("C002AY_BINARY_FINGERPRINT");
  const bool fingerprints_present =
      !source_fingerprint.empty() && !binary_fingerprint.empty();
  auto snapshot = fixture::baseSnapshot();
  snapshot.lease_valid_until.sec = 400;
  auto snapshot_canonical = contract::canonicalizeBaseSnapshotV1(snapshot);
  snapshot.base_geometry_sha256 = snapshot_canonical.geometry_sha256;
  snapshot.snapshot_sha256 = snapshot_canonical.sha256;
  auto trajectory = fixture::authorizedTrajectory(snapshot);
  trajectory.safety_valid_until.sec = 400;
  auto trajectory_canonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.geometry_sha256 = trajectory_canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = trajectory_canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      trajectory_canonical.control_pose_sha256;
  trajectory_canonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.payload_sha256 = trajectory_canonical.sha256;
  if (contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot, trajectory) !=
      contract::ValidationError::NONE) {
    std::cerr << "fixture_validation_failed\n";
    rclcpp::shutdown();
    return 2;
  }
  const auto cdr_bytes = cdrSize(trajectory);
  if (cdr_bytes > kMaxCdrBytes) {
    std::cerr << "cdr_limit_exceeded bytes=" << cdr_bytes << '\n';
    rclcpp::shutdown();
    return 2;
  }

  auto state = std::make_shared<MeasurementState>();
  auto ingress = std::make_shared<Ingress>();
  const bool ingress_atomics_lock_free = ingress->atomicsLockFree();
  auto stop_requested = std::make_shared<std::atomic<bool>>(false);
  const auto steady_start_ns = steadyNowNs();
  const auto logical_ros_start_ns =
      fixture::absoluteTimeNs(trajectory.plan_stamp);
  std::thread worker(workerLoop, ingress, state, std::cref(snapshot),
                     logical_ros_start_ns, steady_start_ns, stop_requested);
  auto publisher = std::make_shared<MeasurementPublisher>(state, trajectory);
  auto subscriber = std::make_shared<MeasurementSubscriber>(state, ingress);
  auto pp_probe = std::make_shared<PpCadenceProbe>(state);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);
  if (kCadenceSeparated) {
    executor.add_node(pp_probe);
  }

  const auto discovery_deadline = SteadyClock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && publisher->subscriptionCount() == 0U &&
         SteadyClock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (publisher->subscriptionCount() == 0U) {
    ingress->shutdown();
    stop_requested->store(true, std::memory_order_release);
    worker.join();
    rclcpp::shutdown();
    return 2;
  }

  publisher->start();
  if (kCadenceSeparated) {
    pp_probe->start();
  }
  const auto run_deadline = SteadyClock::now() + kRunDeadline;
  while (rclcpp::ok() &&
         (state->measured_received < kMeasuredSamples ||
          (kCadenceSeparated &&
           state->pp_probe_measured < cadence::kPpProbeMeasuredSamples)) &&
         SteadyClock::now() < run_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  const bool complete =
      state->measured_received == kMeasuredSamples &&
      (!kCadenceSeparated ||
       state->pp_probe_measured == cadence::kPpProbeMeasuredSamples);
  const auto drain_deadline = SteadyClock::now() + std::chrono::seconds(2);
  while (ingress->outstanding() && SteadyClock::now() < drain_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool drain_completed = !ingress->outstanding();
  ingress->shutdown();
  stop_requested->store(true, std::memory_order_release);
  worker.join();

  const auto worker_count =
      std::min(state->worker_measured_recorded.load(std::memory_order_relaxed),
               kMeasuredSamples);
  const auto worker_processed_total =
      state->worker_processed.load(std::memory_order_relaxed);
  const auto end_to_end_p999_ns =
      complete ? percentile(state->end_to_end_ns, kMeasuredSamples, 0.999) : 0U;
  const auto callback_p999_ns =
      complete ? percentile(state->callback_ns, kMeasuredSamples, 0.999) : 0U;
  const auto timer_gap_max_ns =
      complete && state->measured_timer_gaps > 0U
          ? *std::max_element(state->timer_gap_ns.begin(),
                              state->timer_gap_ns.begin() +
                                  state->measured_timer_gaps)
          : 0U;
  const auto timer_gap_min_ns =
      complete && state->measured_timer_gaps > 0U
          ? *std::min_element(state->timer_gap_ns.begin(),
                              state->timer_gap_ns.begin() +
                                  state->measured_timer_gaps)
          : 0U;
  const auto pp_probe_gap_max_ns =
      kCadenceSeparated && state->pp_probe_measured > 0U
          ? *std::max_element(state->pp_probe_gap_ns.begin(),
                              state->pp_probe_gap_ns.begin() +
                                  state->pp_probe_measured)
          : 0U;
  const auto pp_probe_callback_p999_ns =
      kCadenceSeparated ? percentile(state->pp_probe_callback_ns,
                                     state->pp_probe_measured, 0.999)
                        : 0U;
  const auto worker_p999_ns = percentile(state->worker_ns, worker_count, 0.999);
  const auto worker_queue_age_max_ns =
      worker_count > 0U
          ? *std::max_element(state->worker_queue_age_ns.begin(),
                              state->worker_queue_age_ns.begin() + worker_count)
          : 0U;
  const bool common_gate_pass =
      complete && state->deadline_miss_inflight == 0U &&
      state->unexpected_callbacks == 0U && state->ingress_busy == 0U &&
      state->ingress_rejected == 0U && state->ingress_copy_allocations == 0U &&
      ingress->copyFailureCount() == 0U &&
      state->worker_validation_failures.load(std::memory_order_relaxed) == 0U &&
      state->worker_serialization_failures.load(std::memory_order_relaxed) ==
          0U &&
      state->worker_expired.load(std::memory_order_relaxed) == 0U &&
      state->worker_mutations.load(std::memory_order_relaxed) == 0U &&
      state->worker_ledger_failures.load(std::memory_order_relaxed) == 0U &&
      state->worker_exceptions.load(std::memory_order_relaxed) == 0U &&
      ingress_atomics_lock_free && drain_completed && !ingress->outstanding() &&
      worker_processed_total == ingress->acceptedCount() &&
      worker_count == kMeasuredSamples &&
      state->worker_low_priority_configured.load(std::memory_order_acquire);
  std::uint32_t cadence_failures = cadence::CADENCE_OK;
  if (kCadenceSeparated) {
    cadence::CadenceGateInput cadence_input;
    cadence_input.proposal_samples = state->measured_received;
    cadence_input.pp_probe_samples = state->pp_probe_measured;
    cadence_input.proposal_gap_min_ns = timer_gap_min_ns;
    cadence_input.proposal_gap_max_ns = timer_gap_max_ns;
    cadence_input.proposal_callback_p999_ns = callback_p999_ns;
    cadence_input.pp_probe_gap_max_ns = pp_probe_gap_max_ns;
    cadence_input.pp_probe_callback_p999_ns = pp_probe_callback_p999_ns;
    cadence_failures = cadence::evaluateCadenceGate(cadence_input);
  }
  const bool ingress_sub_gate_pass =
      common_gate_pass &&
      (kCadenceSeparated ? cadence_failures == cadence::CADENCE_OK
                         : callback_p999_ns <= kCallbackBudgetNs &&
                               timer_gap_max_ns <= kTimerGapBudgetNs) &&
      (!kCadenceSeparated || fingerprints_present);

  std::cout
      << "{\"fixture\":\"" << kFixtureName << "\""
      << ",\"topic\":\"" << kTopic << "\""
      << ",\"rmw\":\"" << rmw_get_implementation_identifier() << "\""
      << ",\"source_fingerprint\":\"" << source_fingerprint << "\""
      << ",\"binary_fingerprint\":\"" << binary_fingerprint << "\""
      << ",\"authority_eligible\":false"
      << ",\"cdr_bytes\":" << cdr_bytes
      << ",\"proposal_warmup\":" << state->warmup_received
      << ",\"samples\":" << state->measured_received
      << ",\"sent\":" << state->sent
      << ",\"deadline_miss_inflight\":" << state->deadline_miss_inflight
      << ",\"unexpected_callbacks\":" << state->unexpected_callbacks
      << ",\"ingress_busy\":" << state->ingress_busy
      << ",\"ingress_rejected\":" << state->ingress_rejected
      << ",\"ingress_copy_allocations\":" << state->ingress_copy_allocations
      << ",\"worker_processed\":" << worker_processed_total
      << ",\"worker_samples_recorded\":" << worker_count
      << ",\"worker_validation_failures\":"
      << state->worker_validation_failures.load(std::memory_order_relaxed)
      << ",\"worker_serialization_failures\":"
      << state->worker_serialization_failures.load(std::memory_order_relaxed)
      << ",\"worker_expired\":"
      << state->worker_expired.load(std::memory_order_relaxed)
      << ",\"worker_mutations\":"
      << state->worker_mutations.load(std::memory_order_relaxed)
      << ",\"worker_ledger_failures\":"
      << state->worker_ledger_failures.load(std::memory_order_relaxed)
      << ",\"worker_exceptions\":"
      << state->worker_exceptions.load(std::memory_order_relaxed)
      << ",\"worker_low_priority_configured\":"
      << (state->worker_low_priority_configured.load(std::memory_order_acquire)
              ? "true"
              : "false")
      << ",\"end_to_end_p999_ns\":" << end_to_end_p999_ns
      << ",\"ingress_callback_p999_ns\":" << callback_p999_ns
      << ",\"proposal_timer_gap_min_ns\":" << timer_gap_min_ns
      << ",\"timer_gap_max_ns\":" << timer_gap_max_ns
      << ",\"pp_probe_samples\":" << state->pp_probe_measured
      << ",\"pp_probe_warmup\":" << state->pp_probe_warmup
      << ",\"pp_probe_gap_max_ns\":" << pp_probe_gap_max_ns
      << ",\"pp_probe_callback_p999_ns\":" << pp_probe_callback_p999_ns
      << ",\"cadence_failures\":" << cadence_failures
      << ",\"cadence_gate_evaluated\":"
      << (kCadenceSeparated ? "true" : "false")
      << ",\"worker_p999_ns\":" << worker_p999_ns
      << ",\"worker_queue_age_max_ns\":" << worker_queue_age_max_ns
      << ",\"drain_completed\":" << (drain_completed ? "true" : "false")
      << ",\"worker_ros_clock_injected\":true"
      << ",\"ingress_atomics_lock_free\":"
      << (ingress_atomics_lock_free ? "true" : "false")
      << ",\"ingress_sub_gate_pass\":"
      << (ingress_sub_gate_pass ? "true" : "false")
      << ",\"phase1_full_gate_evaluated\":false"
      << ",\"worker_age_gate_evaluated\":false"
      << ",\"transport_loss_measured\":false"
      << ",\"production_callback_measured\":false"
      << ",\"motion_authority_granted\":false}\n";
  rclcpp::shutdown();
  if (!complete) {
    return 2;
  }
  return ingress_sub_gate_pass ? 0 : 3;
}
