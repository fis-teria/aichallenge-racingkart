#include "overtake_transport_contract/c002ay0_canonical.hpp"
#include "overtake_transport_contract/c002ay0_worker_ipc.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

namespace overtake_transport_contract::c002ay0 {
namespace {

using Authorized =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using Base = multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;

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

builtin_interfaces::msg::Time rosTime(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time value;
  value.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  value.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return value;
}

FixedTime fixedTime(std::int64_t nanoseconds) {
  return {static_cast<std::int32_t>(nanoseconds / 1000000000LL),
          static_cast<std::uint32_t>(nanoseconds % 1000000000LL)};
}

Digest digest(std::uint8_t value) {
  Digest result{};
  result.fill(value);
  return result;
}

FixedBaseRecord makeRecord(std::int64_t now_ns, std::uint64_t sequence) {
  FixedBaseRecord record{};
  record.session_generation = 501U;
  record.session_nonce = 502U;
  record.record_stamp = fixedTime(now_ns);
  constexpr char kFrame[] = "map";
  record.frame_size = sizeof(kFrame) - 1U;
  std::memcpy(record.frame.data(), kFrame, record.frame_size);
  record.race_arm_epoch = 3U;
  record.controller_instance_id = 503U;
  record.controller_sequence = sequence;
  record.base_lease_id = 600U + sequence;
  record.lease_valid_until = fixedTime(now_ns + 5000000000LL);
  record.base_source_kind = Base::SOURCE_REFERENCE_TRAJECTORY;
  record.base_source_stamp = fixedTime(now_ns);
  record.base_source_generation = 13U;
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
  record.controller_implementation_sha256 = digest(0x41U);
  record.controller_config_sha256 = digest(0x51U);
  return record;
}

void recanonicalize(Authorized &trajectory) {
  const auto geometry = canonicalizeGeometryV1(
      trajectory.points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  ASSERT_TRUE(geometry.valid());
  trajectory.geometry_sha256 = geometry.sha256;
  const auto canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  const auto complete = canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(complete.valid());
  trajectory.payload_sha256 = complete.sha256;
}

Authorized proposalFrom(const Base &snapshot, std::int64_t now_ns) {
  Authorized value;
  value.schema_version = Authorized::SCHEMA_V1_SHADOW;
  value.authority_eligible = false;
  value.plan_stamp = rosTime(now_ns);
  value.frame_id = snapshot.frame_id;
  value.plan_sample_key.race_arm_epoch = snapshot.race_arm_epoch;
  value.plan_sample_key.planner_instance_id = 17U;
  value.plan_sample_key.attempt_id = 19U;
  value.plan_sample_key.target_vehicle_id = "d2";
  value.plan_sample_key.pass_direction = 1;
  value.plan_sample_key.connector_transaction_id = 23U;
  value.plan_sample_key.plan_stamp = value.plan_stamp;
  value.plan_sample_key.plan_generation = 29U;
  value.candidate_revision = 31U;
  value.authority_token = 37U;
  value.candidate_type = Authorized::CANDIDATE_PASS_LEFT;
  value.phase = Authorized::PHASE_PASSING;
  value.authorization_state = Authorized::AUTHORIZATION_AUTHORIZED;
  value.source_controller_instance_id = snapshot.controller_instance_id;
  value.source_controller_sequence = snapshot.controller_sequence;
  value.base_lease_id = snapshot.base_lease_id;
  value.base_lease_valid_until = snapshot.lease_valid_until;
  value.base_source_kind = snapshot.base_source_kind;
  value.base_source_stamp = snapshot.base_source_stamp;
  value.base_source_generation = snapshot.base_source_generation;
  value.base_original_point_count = snapshot.base_original_point_count;
  value.base_first_source_index = snapshot.first_source_index;
  value.base_last_source_index = snapshot.last_source_index;
  value.base_nearest_source_index = snapshot.nearest_source_index;
  value.base_source_digest_state = snapshot.base_source_digest_state;
  value.canonical_algorithm_version = snapshot.canonical_algorithm_version;
  value.base_geometry_sha256 = snapshot.base_geometry_sha256;
  value.base_source_sha256 = snapshot.base_source_sha256;
  value.base_snapshot_sha256 = snapshot.snapshot_sha256;
  value.points = snapshot.base_points;
  value.original_candidate_point_count =
      static_cast<std::uint32_t>(snapshot.base_points.size());
  value.total_arc_length_m = 0.25;
  value.required_spatial_horizon_m = 0.25;
  value.join_end_arc_length_m = 0.25;
  value.post_join_arc_length_m = 0.0;
  value.safety_snapshot_id = 41U;
  value.safety_evaluation_result = Authorized::SAFETY_PASSED;
  value.safety_evaluation_stamp = rosTime(now_ns);
  value.safety_valid_until = rosTime(now_ns + 4000000000LL);
  value.world_safety_snapshot_sha256 = digest(0x61U);
  value.safety_evaluator_implementation_sha256 = digest(0x71U);
  value.safety_evaluator_config_sha256 = digest(0x81U);
  value.controller_implementation_sha256 =
      snapshot.controller_implementation_sha256;
  value.controller_config_sha256 = snapshot.controller_config_sha256;
  value.candidate_start_control_pose.orientation.w = 1.0;
  value.candidate_start_control_pose_stamp = value.plan_stamp;
  recanonicalize(value);
  return value;
}

struct RuntimeResult {
  bool snapshot_received{false};
  std::string binding;
  std::string binding_audit;
  bool binding_audit_publisher_seen{false};
};

enum class ProposalMutation {
  kNone,
  kPredecessor,
  kNearestRegressed,
  kStaleSafety,
};

RuntimeResult runRuntime(bool binding_enabled,
                         ProposalMutation mutation = ProposalMutation::kNone,
                         bool binding_audit_enabled = false) {
  static std::atomic<std::uint64_t> run_sequence{0U};
  const auto run_id =
      run_sequence.fetch_add(1U, std::memory_order_relaxed) + 1U;
  auto node = std::make_shared<rclcpp::Node>(
      std::string(binding_enabled ? "ay0_binding_on_test_"
                                  : "ay0_binding_off_test_") +
      std::to_string(run_id));
  const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  Base snapshot;
  RuntimeResult result;
  const auto snapshot_subscription = node->create_subscription<Base>(
      "/control/overtake/base_trajectory_snapshot", qos,
      [&](const Base::SharedPtr message) {
        snapshot = *message;
        result.snapshot_received = true;
      });
  const auto binding_subscription =
      node->create_subscription<std_msgs::msg::String>(
          "/debug/overtake/state_lattice/source_binding", qos,
          [&](const std_msgs::msg::String::SharedPtr message) {
            result.binding = message->data;
          });
  const auto audit_qos =
      rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();
  const auto binding_audit_subscription =
      node->create_subscription<std_msgs::msg::String>(
          "/test/c002ay0/state_lattice/binding_callback_terminal", audit_qos,
          [&](const std_msgs::msg::String::SharedPtr message) {
            result.binding_audit = message->data;
          });
  const auto proposal_publisher = node->create_publisher<Authorized>(
      "/debug/overtake/state_lattice/authorized_cartesian_trajectory", qos);

  FixedBaseWorkerConfig config;
  config.enabled = true;
  config.executable_path = siblingWorkerPath();
  config.session_generation = 501U;
  config.session_nonce = 502U;
  config.controller_instance_id = 503U;
  config.source_binding_enabled = binding_enabled;
  config.use_sim_time = false;
  config.handshake_timeout = std::chrono::milliseconds(1000);
  config.shutdown_budget.drain = std::chrono::milliseconds(1000);
  config.shutdown_budget.interrupt = std::chrono::milliseconds(500);
  config.shutdown_budget.terminate = std::chrono::milliseconds(250);
  config.shutdown_budget.kill = std::chrono::milliseconds(250);
  if (binding_audit_enabled) {
    EXPECT_EQ(setenv("C002AY0_TEST_RELIABLE_BINDING_AUDIT", "1", 1), 0);
  } else {
    EXPECT_EQ(unsetenv("C002AY0_TEST_RELIABLE_BINDING_AUDIT"), 0);
  }
  auto session = FixedBaseWorkerSession::start(config);
  EXPECT_EQ(unsetenv("C002AY0_TEST_RELIABLE_BINDING_AUDIT"), 0);
  EXPECT_NE(session, nullptr);
  if (session == nullptr) {
    return result;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (snapshot_subscription->get_publisher_count() == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_GT(snapshot_subscription->get_publisher_count(), 0U);
  if (snapshot_subscription->get_publisher_count() == 0U) {
    EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kDrained);
    executor.remove_node(node);
    return result;
  }
  auto now_ns = node->get_clock()->now().nanoseconds();
  auto next_capture = std::chrono::steady_clock::now();
  std::uint64_t capture_sequence = 1U;
  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!result.snapshot_received &&
         std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_capture) {
      now_ns = node->get_clock()->now().nanoseconds();
      EXPECT_TRUE(session->tryCapture(makeRecord(now_ns, capture_sequence++)));
      next_capture = now + std::chrono::milliseconds(100);
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(result.snapshot_received);
  EXPECT_GE(session->processedCount(), 1U);
  if (result.snapshot_received) {
    auto proposal = proposalFrom(snapshot, now_ns);
    if (mutation == ProposalMutation::kPredecessor) {
      const auto proposal_controller_sequence =
          proposal.source_controller_sequence;
      result.snapshot_received = false;
      EXPECT_TRUE(session->tryCapture(makeRecord(
          node->get_clock()->now().nanoseconds(), capture_sequence++)));
      deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while ((!result.snapshot_received ||
              snapshot.controller_sequence == proposal_controller_sequence) &&
             std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      EXPECT_TRUE(result.snapshot_received);
      EXPECT_NE(snapshot.controller_sequence, proposal_controller_sequence);
    } else if (mutation == ProposalMutation::kNearestRegressed) {
      proposal.base_nearest_source_index = snapshot.nearest_source_index + 1U;
      recanonicalize(proposal);
    } else if (mutation == ProposalMutation::kStaleSafety) {
      proposal.safety_evaluation_stamp = rosTime(now_ns - 2000000000LL);
      proposal.safety_valid_until = rosTime(now_ns - 1LL);
      recanonicalize(proposal);
    }
    if (binding_enabled) {
      deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while ((proposal_publisher->get_subscription_count() == 0U ||
              binding_subscription->get_publisher_count() == 0U ||
              (binding_audit_enabled &&
               binding_audit_subscription->get_publisher_count() == 0U)) &&
             std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      EXPECT_GT(proposal_publisher->get_subscription_count(), 0U);
      EXPECT_GT(binding_subscription->get_publisher_count(), 0U);
      if (binding_audit_enabled) {
        EXPECT_GT(binding_audit_subscription->get_publisher_count(), 0U);
      }
    }
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((result.binding.empty() ||
            (binding_audit_enabled && result.binding_audit.empty())) &&
           std::chrono::steady_clock::now() < deadline) {
      proposal_publisher->publish(proposal);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    result.binding_audit_publisher_seen =
        binding_audit_subscription->get_publisher_count() != 0U;
  }
  EXPECT_EQ(session->shutdown(), WorkerShutdownOutcome::kDrained);
  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while ((snapshot_subscription->get_publisher_count() != 0U ||
          (binding_enabled &&
           (proposal_publisher->get_subscription_count() != 0U ||
            binding_subscription->get_publisher_count() != 0U ||
            binding_audit_subscription->get_publisher_count() != 0U))) &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(snapshot_subscription->get_publisher_count(), 0U);
  if (binding_enabled) {
    EXPECT_EQ(proposal_publisher->get_subscription_count(), 0U);
    EXPECT_EQ(binding_subscription->get_publisher_count(), 0U);
    EXPECT_EQ(binding_audit_subscription->get_publisher_count(), 0U);
  }
  executor.remove_node(node);
  return result;
}

class C002ay0ShadowBindingRuntime : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 1;
    char name[] = "test_c002ay0_shadow_binding_runtime";
    char *argv[] = {name, nullptr};
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(C002ay0ShadowBindingRuntime,
       WorkerPublishesExactCurrentOnlyWhenBindingEnabled) {
  const auto disabled = runRuntime(false);
  EXPECT_TRUE(disabled.snapshot_received);
  EXPECT_TRUE(disabled.binding.empty());

  const auto enabled = runRuntime(true);
  EXPECT_TRUE(enabled.snapshot_received);
  EXPECT_TRUE(enabled.binding_audit.empty());
  EXPECT_FALSE(enabled.binding_audit_publisher_seen);
  EXPECT_NE(enabled.binding.find("\"disposition\":\"exact_current\""),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"shadow_only\":true"), std::string::npos);
  EXPECT_NE(enabled.binding.find("\"lateral_authority_eligible\":false"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"session_generation\":501"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"session_nonce\":502"), std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_race_arm_epoch\":3"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_planner_instance_id\":17"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_attempt_id\":19"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_connector_transaction_id\":23"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_plan_generation\":29"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_candidate_revision\":31"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_authority_token\":37"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_safety_snapshot_id\":41"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_controller_instance_id\":503"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_controller_sequence\":"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"proposal_base_lease_id\":"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"binding_ros_now_sec\":"),
            std::string::npos);
  EXPECT_NE(enabled.binding.find("\"binding_callback_monotonic_ns\":"),
            std::string::npos);
}

TEST_F(C002ay0ShadowBindingRuntime,
       WorkerRejectsNearestMismatchAndStaleProposal) {
  const auto nearest_regressed =
      runRuntime(true, ProposalMutation::kNearestRegressed);
  EXPECT_NE(nearest_regressed.binding.find(
                "\"disposition\":\"replan_required_nearest_regressed\""),
            std::string::npos);
  EXPECT_NE(
      nearest_regressed.binding.find("\"lateral_authority_eligible\":false"),
      std::string::npos);

  const auto stale = runRuntime(true, ProposalMutation::kStaleSafety);
  EXPECT_NE(stale.binding.find("\"disposition\":\"rejected_stale\""),
            std::string::npos);
  EXPECT_NE(stale.binding.find("\"lateral_authority_eligible\":false"),
            std::string::npos);
}

TEST_F(C002ay0ShadowBindingRuntime,
       TestOnlyReliableAuditMirrorsExactAndRejectedTerminals) {
  const auto exact = runRuntime(true, ProposalMutation::kNone, true);
  EXPECT_TRUE(exact.binding_audit_publisher_seen);
  EXPECT_NE(exact.binding_audit.find("\"disposition\":\"exact_current\""),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"lateral_authority_eligible\":false"),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"proposal_attempt_id\":19"),
            std::string::npos);
  EXPECT_EQ(exact.binding.find("\"test_audit_schema_version\":"),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"test_audit_schema_version\":1"),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"current_controller_instance_id\":503"),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"current_base_lease_id\":"),
            std::string::npos);
  EXPECT_NE(exact.binding_audit.find("\"previous_present\":"),
            std::string::npos);

  const auto nearest =
      runRuntime(true, ProposalMutation::kNearestRegressed, true);
  EXPECT_NE(nearest.binding_audit.find(
                "\"disposition\":\"replan_required_nearest_regressed\""),
            std::string::npos);
  EXPECT_NE(nearest.binding_audit.find("\"lateral_authority_eligible\":false"),
            std::string::npos);
  EXPECT_NE(nearest.binding_audit.find("\"proposal_attempt_id\":19"),
            std::string::npos);
  EXPECT_EQ(nearest.binding.find("\"test_audit_schema_version\":"),
            std::string::npos);

  const auto stale = runRuntime(true, ProposalMutation::kStaleSafety, true);
  EXPECT_NE(stale.binding_audit.find("\"disposition\":\"rejected_stale\""),
            std::string::npos);
  EXPECT_NE(stale.binding_audit.find("\"lateral_authority_eligible\":false"),
            std::string::npos);
  EXPECT_NE(stale.binding_audit.find("\"proposal_attempt_id\":19"),
            std::string::npos);
  EXPECT_EQ(stale.binding.find("\"test_audit_schema_version\":"),
            std::string::npos);

  const auto predecessor =
      runRuntime(true, ProposalMutation::kPredecessor, true);
  EXPECT_NE(predecessor.binding_audit.find(
                "\"disposition\":\"deferred_predecessor\""),
            std::string::npos);
  EXPECT_NE(predecessor.binding_audit.find("\"previous_present\":true"),
            std::string::npos);
  EXPECT_NE(predecessor.binding_audit.find(
                "\"previous_controller_sequence\":1"),
            std::string::npos);
  EXPECT_NE(predecessor.binding_audit.find(
                "\"current_controller_sequence\":2"),
            std::string::npos);
  EXPECT_EQ(predecessor.binding.find("\"test_audit_schema_version\":"),
            std::string::npos);
}

} // namespace
} // namespace overtake_transport_contract::c002ay0
