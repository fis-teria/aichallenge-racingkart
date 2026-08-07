#include "c002ay0_phase1_fixture.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>

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
using Trajectory = contract::AuthorizedCartesianTrajectory;
using SteadyClock = std::chrono::steady_clock;

constexpr std::size_t kMaxCdrBytes = 24U * 1024U;
constexpr std::size_t kWarmupSamples = 1000U;
constexpr std::size_t kMeasuredSamples = 10000U;
constexpr auto kPeriod = std::chrono::milliseconds(10);
constexpr auto kRunDeadline = std::chrono::seconds(180);
constexpr std::uint64_t kCallbackBudgetNs = 1000000ULL;
constexpr std::uint64_t kTimerGapBudgetNs = 12000000ULL;
constexpr char kTopic[] =
    "/c002ay0_measurement/authorized_cartesian_trajectory";

struct MeasurementState {
  std::array<std::uint64_t, kMeasuredSamples> end_to_end_ns{};
  std::array<std::uint64_t, kMeasuredSamples> callback_ns{};
  std::array<std::uint64_t, kMeasuredSamples> timer_gap_ns{};
  SteadyClock::time_point publish_time{};
  SteadyClock::time_point last_timer_time{};
  std::size_t warmup_received{0U};
  std::size_t measured_received{0U};
  std::size_t measured_timer_gaps{0U};
  std::size_t sent{0U};
  std::size_t deadline_miss_inflight{0U};
  std::size_t unexpected_callbacks{0U};
  std::size_t validation_failures{0U};
  std::size_t slot_copy_allocations{0U};
  bool inflight{false};
  bool have_last_timer{false};
};

std::uint64_t
percentile(const std::array<std::uint64_t, kMeasuredSamples> &data,
           const double fraction) {
  auto sorted = data;
  std::sort(sorted.begin(), sorted.end());
  const auto rank = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(sorted.size())));
  return sorted[std::min(sorted.size() - 1U,
                         std::max<std::size_t>(1U, rank) - 1U)];
}

class MeasurementSubscriber : public rclcpp::Node {
public:
  MeasurementSubscriber(std::shared_ptr<MeasurementState> state,
                        contract::ControllerBaseTrajectorySnapshot snapshot)
      : Node("c002ay0_phase1_measurement_subscriber", "/c002ay0_measurement"),
        state_(std::move(state)), snapshot_(std::move(snapshot)) {
    for (auto &slot : slots_) {
      slot.points.reserve(contract::kMaxCartesianPoints);
      slot.frame_id.reserve(128U);
      slot.plan_sample_key.target_vehicle_id.reserve(64U);
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();
    subscription_ = create_subscription<Trajectory>(
        kTopic, qos, [this](const Trajectory::ConstSharedPtr message) {
          const auto callback_start = SteadyClock::now();
          const auto validation =
              contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot_,
                                                                *message);
          const auto allocations_before =
              g_allocation_count.load(std::memory_order_relaxed);
          g_track_allocations.store(true, std::memory_order_relaxed);
          auto &slot = slots_[slot_index_];
          slot = *message;
          g_track_allocations.store(false, std::memory_order_relaxed);
          state_->slot_copy_allocations +=
              g_allocation_count.load(std::memory_order_relaxed) -
              allocations_before;
          slot_index_ = (slot_index_ + 1U) % slots_.size();
          const auto callback_end = SteadyClock::now();
          if (validation != contract::ValidationError::NONE ||
              slot.payload_sha256 != message->payload_sha256) {
            ++state_->validation_failures;
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
  contract::ControllerBaseTrajectorySnapshot snapshot_;
  std::array<Trajectory, 3U> slots_;
  std::size_t slot_index_{0U};
  rclcpp::Subscription<Trajectory>::SharedPtr subscription_;
};

class MeasurementPublisher : public rclcpp::Node {
public:
  MeasurementPublisher(std::shared_ptr<MeasurementState> state,
                       Trajectory trajectory)
      : Node("c002ay0_phase1_measurement_publisher", "/c002ay0_measurement"),
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

std::size_t cdrSize(const Trajectory &trajectory) {
  rclcpp::Serialization<Trajectory> serializer;
  rclcpp::SerializedMessage serialized(kMaxCdrBytes);
  serializer.serialize_message(&trajectory, &serialized);
  return serialized.size();
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto snapshot = fixture::baseSnapshot();
  const auto trajectory = fixture::authorizedTrajectory(snapshot);
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
  auto publisher = std::make_shared<MeasurementPublisher>(state, trajectory);
  auto subscriber = std::make_shared<MeasurementSubscriber>(state, snapshot);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);

  const auto discovery_deadline = SteadyClock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && publisher->subscriptionCount() == 0U &&
         SteadyClock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (publisher->subscriptionCount() == 0U) {
    std::cerr << "rmw_discovery_timeout\n";
    rclcpp::shutdown();
    return 2;
  }

  publisher->start();
  const auto run_deadline = SteadyClock::now() + kRunDeadline;
  while (rclcpp::ok() && state->measured_received < kMeasuredSamples &&
         SteadyClock::now() < run_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  const bool complete = state->measured_received == kMeasuredSamples;
  const auto end_to_end_p999_ns =
      complete ? percentile(state->end_to_end_ns, 0.999) : 0U;
  const auto callback_p999_ns =
      complete ? percentile(state->callback_ns, 0.999) : 0U;
  const auto timer_gap_max_ns =
      complete ? *std::max_element(state->timer_gap_ns.begin(),
                                   state->timer_gap_ns.begin() +
                                       state->measured_timer_gaps)
               : 0U;
  const bool gate_pass = complete && state->deadline_miss_inflight == 0U &&
                         state->unexpected_callbacks == 0U &&
                         state->validation_failures == 0U &&
                         state->slot_copy_allocations == 0U &&
                         end_to_end_p999_ns <= kCallbackBudgetNs &&
                         timer_gap_max_ns <= kTimerGapBudgetNs;
  std::cout << "{\"fixture\":\"C002AY0_PHASE1_RMW_V1\""
            << ",\"topic\":\"" << kTopic << "\""
            << ",\"rmw\":\"" << rmw_get_implementation_identifier() << "\""
            << ",\"authority_eligible\":false"
            << ",\"cdr_bytes\":" << cdr_bytes
            << ",\"warmup\":" << kWarmupSamples
            << ",\"samples\":" << state->measured_received
            << ",\"sent\":" << state->sent
            << ",\"deadline_miss_inflight\":" << state->deadline_miss_inflight
            << ",\"unexpected_callbacks\":" << state->unexpected_callbacks
            << ",\"transport_loss_measured\":false"
            << ",\"validation_failures\":" << state->validation_failures
            << ",\"slot_copy_allocations\":" << state->slot_copy_allocations
            << ",\"end_to_end_p999_ns\":" << end_to_end_p999_ns
            << ",\"callback_p999_ns\":" << callback_p999_ns
            << ",\"timer_gap_max_ns\":" << timer_gap_max_ns
            << ",\"gate_pass\":" << (gate_pass ? "true" : "false")
            << ",\"production_callback_measured\":false"
            << ",\"motion_authority_granted\":false}\n";
  rclcpp::shutdown();
  if (!complete) {
    return 2;
  }
  return gate_pass ? 0 : 3;
}
