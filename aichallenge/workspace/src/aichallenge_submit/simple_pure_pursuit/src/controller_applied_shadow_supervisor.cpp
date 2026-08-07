#include "simple_pure_pursuit/aw2_shadow_ipc.hpp"
#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <multi_purpose_mpc_ros_msgs/msg/controller_applied_transport_status.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <limits>
#include <poll.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {

namespace shadow = simple_pure_pursuit::aw2_shadow;
using multi_purpose_mpc_ros_msgs::msg::ControllerAppliedEnvelope;
using multi_purpose_mpc_ros_msgs::msg::ControllerAppliedTransportStatus;
using multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope;
using multi_purpose_mpc_ros_msgs::msg::ControllerSampleKey;
using multi_purpose_mpc_ros_msgs::msg::OvertakePlan;
using simple_pure_pursuit::Aw2CanonicalSourceWire;
using simple_pure_pursuit::ControllerAppliedBindingTracker;
using simple_pure_pursuit::ControllerAppliedEnvelopeInput;

builtin_interfaces::msg::Time toRosTime(const shadow::FixedTime &source) {
  builtin_interfaces::msg::Time destination;
  destination.sec = source.sec;
  destination.nanosec = source.nanosec;
  return destination;
}

template <std::size_t Capacity>
std::string toString(const shadow::FixedString<Capacity> &source) {
  return std::string(source.bytes.data(), source.size);
}

autoware_auto_control_msgs::msg::AckermannControlCommand
toCommand(const shadow::FixedCommand &source) {
  autoware_auto_control_msgs::msg::AckermannControlCommand destination;
  destination.stamp = toRosTime(source.command_stamp);
  destination.lateral.stamp = toRosTime(source.lateral_stamp);
  destination.lateral.steering_tire_angle = source.steering_tire_angle_rad;
  destination.lateral.steering_tire_rotation_rate =
      source.steering_tire_rotation_rate_radps;
  destination.longitudinal.stamp = toRosTime(source.longitudinal_stamp);
  destination.longitudinal.speed = source.longitudinal_speed_mps;
  destination.longitudinal.acceleration = source.longitudinal_acceleration_mps2;
  destination.longitudinal.jerk = source.longitudinal_jerk_mps3;
  return destination;
}

autoware_auto_planning_msgs::msg::TrajectoryPoint
toPoint(const shadow::FixedTrajectoryPoint &source) {
  autoware_auto_planning_msgs::msg::TrajectoryPoint destination;
  destination.time_from_start.sec = source.time_from_start.sec;
  destination.time_from_start.nanosec = source.time_from_start.nanosec;
  destination.pose.position.x = source.position_x_m;
  destination.pose.position.y = source.position_y_m;
  destination.pose.position.z = source.position_z_m;
  destination.pose.orientation.x = source.orientation_x;
  destination.pose.orientation.y = source.orientation_y;
  destination.pose.orientation.z = source.orientation_z;
  destination.pose.orientation.w = source.orientation_w;
  destination.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
  destination.lateral_velocity_mps = source.lateral_velocity_mps;
  destination.acceleration_mps2 = source.acceleration_mps2;
  destination.heading_rate_rps = source.heading_rate_rps;
  destination.front_wheel_angle_rad = source.front_wheel_angle_rad;
  destination.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
  return destination;
}

autoware_auto_planning_msgs::msg::Trajectory
toTrajectory(const shadow::FixedGeometry &source) {
  autoware_auto_planning_msgs::msg::Trajectory destination;
  destination.header.frame_id = toString(source.frame_id);
  destination.header.stamp = toRosTime(source.source_stamp);
  destination.points.reserve(source.point_count);
  for (std::uint32_t index = 0U; index < source.point_count; ++index) {
    destination.points.push_back(toPoint(source.points[index]));
  }
  return destination;
}

geometry_msgs::msg::Pose toPose(const shadow::FixedPose &source) {
  geometry_msgs::msg::Pose destination;
  destination.position.x = source.position_x_m;
  destination.position.y = source.position_y_m;
  destination.position.z = source.position_z_m;
  destination.orientation.x = source.orientation_x;
  destination.orientation.y = source.orientation_y;
  destination.orientation.z = source.orientation_z;
  destination.orientation.w = source.orientation_w;
  return destination;
}

ControllerSampleKey toKey(const shadow::FixedControllerSampleKey &source) {
  ControllerSampleKey destination;
  destination.plan_sample_key.race_arm_epoch = source.race_arm_epoch;
  destination.plan_sample_key.planner_instance_id = source.planner_instance_id;
  destination.plan_sample_key.attempt_id = source.attempt_id;
  destination.plan_sample_key.target_vehicle_id =
      toString(source.target_vehicle_id);
  destination.plan_sample_key.pass_direction = source.pass_direction;
  destination.plan_sample_key.connector_transaction_id =
      source.connector_transaction_id;
  destination.plan_sample_key.plan_stamp = toRosTime(source.plan_stamp);
  destination.plan_sample_key.plan_generation = source.plan_generation;
  destination.controller_role = source.controller_role;
  destination.controller_instance_id = source.controller_instance_id;
  destination.controller_sequence = source.controller_sequence;
  destination.controller_command_stamp =
      toRosTime(source.controller_command_stamp);
  return destination;
}

const char *resourceReason(shadow::ResourceLimitKind kind) {
  switch (kind) {
  case shadow::ResourceLimitKind::kTarget:
    return "capture_target_limit_exceeded";
  case shadow::ResourceLimitKind::kFrame:
    return "capture_frame_limit_exceeded";
  case shadow::ResourceLimitKind::kSource:
    return "capture_source_limit_exceeded";
  case shadow::ResourceLimitKind::kGeometry:
    return "capture_geometry_limit_exceeded";
  case shadow::ResourceLimitKind::kGeometryInterval:
    return "geometry_interval_point_limit_exceeded";
  case shadow::ResourceLimitKind::kHorizonUnavailable:
    return "required_horizon_unavailable";
  case shadow::ResourceLimitKind::kRollout:
    return "capture_rollout_limit_exceeded";
  default:
    return "capture_resource_limit_exceeded";
  }
}

ControllerAppliedEnvelope
buildEnvelope(const shadow::FixedSnapshot &snapshot,
              const shadow::SharedDataRegion &data,
              ControllerAppliedBindingTracker &tracker) {
  ControllerCommandEnvelope command;
  command.header.stamp = toRosTime(snapshot.record_stamp);
  command.header.frame_id = toString(snapshot.record_frame_id);
  command.schema_version = 1U;
  command.producer_instance_id =
      snapshot.controller_sample_key.controller_instance_id;
  command.command_sequence = snapshot.controller_sample_key.controller_sequence;
  command.plan_generation = snapshot.controller_sample_key.plan_generation;
  command.command = toCommand(snapshot.output_command);

  ControllerAppliedEnvelopeInput input;
  input.header = command.header;
  if (snapshot.kind == shadow::SnapshotKind::kResourceLimit) {
    input.capture_resource_limit_exceeded = true;
    input.capture_resource_limit_reason =
        resourceReason(snapshot.resource_limit_kind);
    input.resource_sample_key_present = true;
    input.resource_sample_key = toKey(snapshot.controller_sample_key);
    input.resource_candidate_revision = snapshot.candidate_revision;
    input.resource_candidate_content_sha256 = snapshot.candidate_content_sha256;
    return simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
        command, input, nullptr, false, &tracker);
  }
  input.controller_role = snapshot.controller_sample_key.controller_role;
  input.geometry_relation = snapshot.geometry_relation;
  input.source_generation = snapshot.source_generation;
  input.source_wire.original_size_bytes = snapshot.source_original_size_bytes;
  input.source_wire.complete = snapshot.source_wire_complete;
  input.source_wire.bytes.assign(snapshot.source_wire.begin(),
                                 snapshot.source_wire.begin() +
                                     snapshot.source_wire_size);
  if (input.source_wire.complete) {
    input.source_wire.sha256 =
        simple_pure_pursuit::aw2Sha256(input.source_wire.bytes);
  }
  input.base_trajectory = toTrajectory(snapshot.base_geometry);
  input.applied_trajectory = toTrajectory(snapshot.applied_geometry);
  input.base_original_point_count = snapshot.base_geometry.original_point_count;
  input.applied_original_point_count =
      snapshot.applied_geometry.original_point_count;
  input.control_pose = toPose(snapshot.control_pose);
  input.control_pose_stamp = toRosTime(snapshot.control_pose_stamp);
  input.nearest_trajectory_index =
      snapshot.applied_geometry.nearest_source_index;
  input.speed_cap_trajectory_index =
      snapshot.applied_geometry.speed_cap_source_index;
  input.curvature_last_read_trajectory_index =
      snapshot.applied_geometry.curvature_last_read_source_index;
  input.lookahead_selected_trajectory_index =
      snapshot.applied_geometry.lookahead_selected_source_index;
  input.required_horizon_end_trajectory_index =
      snapshot.applied_geometry.required_horizon_end_source_index;
  input.lookahead_endpoint_fallback =
      snapshot.applied_geometry.lookahead_endpoint_fallback;
  input.trajectory_progress_m = snapshot.trajectory_progress_m;
  input.controller_adapter_implementation_sha256 = data.producer_build_digest;
  input.controller_adapter_config_sha256 = data.producer_config_digest;
  input.raw_controller_command = toCommand(snapshot.raw_command);
  input.raw_steering_tire_angle_rad = snapshot.raw_steering_tire_angle_rad;
  input.output_steering_tire_angle_rad =
      snapshot.output_steering_tire_angle_rad;
  input.raw_steering_tire_rotation_rate_radps =
      snapshot.raw_steering_tire_rotation_rate_radps;
  input.output_steering_tire_rotation_rate_radps =
      snapshot.output_steering_tire_rotation_rate_radps;
  input.available_spatial_horizon_m = snapshot.available_spatial_horizon_m;
  input.required_spatial_horizon_m = snapshot.required_spatial_horizon_m;
  input.rollout_state = snapshot.rollout_state;

  OvertakePlan plan;
  OvertakePlan *plan_pointer = nullptr;
  if (snapshot.typed_plan_present) {
    plan.header = command.header;
    plan.trajectory = input.applied_trajectory;
    plan.trajectory.header = plan.header;
    plan.aw2_identity_schema_version =
        snapshot.typed_plan_identity_schema_version;
    plan.race_arm_epoch = snapshot.controller_sample_key.race_arm_epoch;
    plan.planner_instance_id =
        snapshot.controller_sample_key.planner_instance_id;
    plan.attempt_id = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(snapshot.controller_sample_key.attempt_id,
                                std::numeric_limits<std::uint32_t>::max()));
    plan.target_vehicle_id =
        toString(snapshot.controller_sample_key.target_vehicle_id);
    plan.pass_direction = snapshot.controller_sample_key.pass_direction;
    plan.connector_transaction_id =
        snapshot.controller_sample_key.connector_transaction_id;
    plan.plan_generation = snapshot.controller_sample_key.plan_generation;
    plan.candidate_revision = snapshot.candidate_revision;
    plan.candidate_content_sha256 = snapshot.candidate_content_sha256;
    plan.trajectory_authorized = snapshot.typed_plan_trajectory_authorized;
    plan_pointer = &plan;
  }
  return simple_pure_pursuit::makeControllerAppliedEnvelopeV1(
      command, input, plan_pointer, snapshot.typed_plan_fresh, &tracker);
}

ControllerAppliedTransportStatus
buildStatus(const shadow::SharedDataRegion &data,
            const shadow::SharedAckRegion &ack,
            const shadow::FixedSnapshot &snapshot,
            const builtin_interfaces::msg::Time &stamp) {
  ControllerAppliedTransportStatus status;
  status.stamp = stamp;
  status.schema_version = ControllerAppliedTransportStatus::SCHEMA_V1;
  status.authority_eligible = false;
  status.ring_generation = data.ring_generation;
  status.controller_instance_id = data.controller_instance_id;
  status.consumer_instance_id =
      shadow::atomicLoadAcquire(ack.consumer_instance_id);
  status.race_arm_epoch = shadow::atomicLoadAcquire(data.race_arm_epoch);
  status.queue_capacity = data.capacity;
  const auto write = shadow::atomicLoadAcquire(data.write_index);
  const auto read = shadow::atomicLoadAcquire(ack.read_index);
  status.queue_depth = static_cast<std::uint32_t>(
      std::min<std::uint64_t>(write - read, data.capacity));
  status.queue_high_water = static_cast<std::uint32_t>(
      shadow::atomicLoadAcquire(data.queue_high_water));
  status.torn_slot_count = shadow::atomicLoadAcquire(ack.torn_slot_count);
  status.abi_error_count = shadow::atomicLoadAcquire(ack.abi_error_count);
  status.dds_error_count = shadow::atomicLoadAcquire(ack.dds_error_count);

  shadow::FixedLossLedger ledger{};
  if (shadow::readLossLedger(data.loss_ledger, ledger)) {
    status.loss_generation = ledger.loss_generation;
    status.cumulative_drop_count = ledger.cumulative_drop_count;
    status.global_loss_membership_unknown = ledger.global_membership_unknown;
    status.first_uncertain_sequence = ledger.first_uncertain_sequence;
    for (std::uint32_t index = 0U; index < ledger.range_count; ++index) {
      status.first_dropped_sequence.push_back(
          ledger.ranges[index].first_sequence);
      status.last_dropped_sequence.push_back(
          ledger.ranges[index].last_sequence);
      status.first_dropped_key.push_back(toKey(ledger.ranges[index].first_key));
      status.last_dropped_key.push_back(toKey(ledger.ranges[index].last_key));
    }
  }
  if (snapshot.kind == shadow::SnapshotKind::kResourceLimit) {
    status.reason =
        ControllerAppliedTransportStatus::REASON_RESOURCE_LIMIT_EXCEEDED;
    status.detail = resourceReason(snapshot.resource_limit_kind);
  } else if (status.dds_error_count != 0U) {
    status.reason = ControllerAppliedTransportStatus::REASON_DDS_ERROR;
    status.detail = "shadow_dds_publish_error";
  } else if (status.abi_error_count != 0U) {
    status.reason = ControllerAppliedTransportStatus::REASON_ABI_MISMATCH;
    status.detail = "shared_abi_mismatch";
  } else if (status.torn_slot_count != 0U) {
    status.reason = ControllerAppliedTransportStatus::REASON_TORN_SLOT;
    status.detail = "torn_slot_discarded";
  } else if (status.global_loss_membership_unknown) {
    status.reason =
        ControllerAppliedTransportStatus::REASON_GLOBAL_LOSS_MEMBERSHIP_UNKNOWN;
    status.detail = "loss_range_capacity_exceeded";
  } else if (status.cumulative_drop_count != 0U) {
    status.reason = ControllerAppliedTransportStatus::REASON_QUEUE_DROP;
    status.detail = "producer_drop_new";
  } else {
    status.reason = ControllerAppliedTransportStatus::REASON_NONE;
    status.detail = "ok";
  }
  return status;
}

int runPublisher(int argc, char **argv, const shadow::ReceivedBootstrap &ipc,
                 pid_t expected_supervisor_pid) {
  if (!shadow::applyPublisherParentDeathSignal(expected_supervisor_pid) ||
      !shadow::applyPublisherResourceIsolation()) {
    return 10;
  }
  const auto *data = static_cast<const shadow::SharedDataRegion *>(
      mmap(nullptr, sizeof(shadow::SharedDataRegion), PROT_READ, MAP_SHARED,
           ipc.data_fd, 0));
  auto *ack = static_cast<shadow::SharedAckRegion *>(
      mmap(nullptr, sizeof(shadow::SharedAckRegion), PROT_READ | PROT_WRITE,
           MAP_SHARED, ipc.ack_fd, 0));
  if (data == MAP_FAILED || ack == MAP_FAILED ||
      !shadow::validateSharedAbi(*data, *ack)) {
    return 11;
  }

  try {
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<rclcpp::Node>("controller_applied_shadow_publisher");
    const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(8)).best_effort().durability_volatile();
    auto envelope_publisher = node->create_publisher<ControllerAppliedEnvelope>(
        "/hybrid_control/pure_pursuit/controller_applied_envelope", qos);
    auto status_publisher =
        node->create_publisher<ControllerAppliedTransportStatus>(
            "/hybrid_control/pure_pursuit/controller_applied_transport_status",
            qos);
    shadow::atomicStoreRelease(
        ack->consumer_instance_id,
        static_cast<std::uint64_t>(std::max<pid_t>(1, getpid())));
    shadow::atomicStoreRelease(ack->consumer_ready, 1U);
    ControllerAppliedBindingTracker tracker;
    while (rclcpp::ok() && shadow::atomicLoadAcquire(data->closing) == 0U &&
           shadow::atomicLoadAcquire(ack->shadow_disabled) == 0U) {
      shadow::FixedSnapshot snapshot{};
      const auto result = shadow::tryPop(*data, *ack, snapshot);
      if (result == shadow::PopResult::kPopped) {
        const auto envelope = buildEnvelope(snapshot, *data, tracker);
        envelope_publisher->publish(envelope);
        status_publisher->publish(
            buildStatus(*data, *ack, snapshot, envelope.record_stamp));
      } else if (result == shadow::PopResult::kAbiMismatch) {
        shadow::atomicStoreRelease(
            ack->abi_error_count,
            shadow::atomicLoadAcquire(ack->abi_error_count) + 1U);
        shadow::atomicStoreRelease(ack->shadow_disabled, 1U);
      } else {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    rclcpp::shutdown();
  } catch (const std::exception &) {
    shadow::atomicStoreRelease(ack->dds_error_count,
                               shadow::atomicLoadAcquire(ack->dds_error_count) +
                                   1U);
    shadow::atomicStoreRelease(ack->shadow_disabled, 1U);
    return 12;
  } catch (...) {
    shadow::atomicStoreRelease(ack->dds_error_count,
                               shadow::atomicLoadAcquire(ack->dds_error_count) +
                                   1U);
    shadow::atomicStoreRelease(ack->shadow_disabled, 1U);
    return 13;
  }
  return 0;
}

bool parseUnsigned(const char *text, std::uint64_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const auto parsed = std::strtoull(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0') {
    return false;
  }
  value = parsed;
  return true;
}

int openProducerPidfd(pid_t producer_pid) noexcept {
#ifdef SYS_pidfd_open
  return static_cast<int>(syscall(SYS_pidfd_open, producer_pid, 0U));
#else
  (void)producer_pid;
  errno = ENOSYS;
  return -1;
#endif
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 5) {
    return 2;
  }
  std::uint64_t producer_pid_value = 0U;
  std::uint64_t nonce_high = 0U;
  std::uint64_t nonce_low = 0U;
  if (!parseUnsigned(argv[2], producer_pid_value) ||
      !parseUnsigned(argv[3], nonce_high) ||
      !parseUnsigned(argv[4], nonce_low) ||
      producer_pid_value >
          static_cast<std::uint64_t>(std::numeric_limits<pid_t>::max())) {
    return 3;
  }
  const auto producer_pid = static_cast<pid_t>(producer_pid_value);
  const int producer_pidfd = openProducerPidfd(producer_pid);
  if (producer_pidfd < 0) {
    return 6;
  }
  auto bootstrap =
      shadow::receiveBootstrap(argv[1], producer_pid, nonce_high, nonce_low);
  if (!bootstrap.has_value()) {
    close(producer_pidfd);
    return 4;
  }
  const pid_t expected_supervisor_pid = getpid();
  const pid_t publisher = fork();
  if (publisher < 0) {
    shadow::closeBootstrap(bootstrap.value());
    close(producer_pidfd);
    return 5;
  }
  if (publisher == 0) {
    close(producer_pidfd);
    close(bootstrap->control_socket_fd);
    bootstrap->control_socket_fd = -1;
    const int result =
        runPublisher(argc, argv, bootstrap.value(), expected_supervisor_pid);
    shadow::closeBootstrap(bootstrap.value());
    _exit(result);
  }

  close(bootstrap->data_fd);
  close(bootstrap->ack_fd);
  bootstrap->data_fd = -1;
  bootstrap->ack_fd = -1;
  struct pollfd producer_liveness[2]{
      {bootstrap->control_socket_fd, POLLHUP | POLLERR, 0},
      {producer_pidfd, POLLIN | POLLHUP | POLLERR, 0}};
  int publisher_cleanup_exit_code = 0;
  for (;;) {
    int status = 0;
    if (waitpid(publisher, &status, WNOHANG) == publisher) {
      break;
    }
    const int poll_result = poll(producer_liveness, 2, 100);
    if (poll_result > 0 &&
        (((producer_liveness[0].revents & (POLLHUP | POLLERR)) != 0) ||
         ((producer_liveness[1].revents & (POLLIN | POLLHUP | POLLERR)) !=
          0))) {
      publisher_cleanup_exit_code =
          shadow::supervisorExitCodeForChildTermination(
              shadow::terminateOneShotChild(publisher,
                                            std::chrono::milliseconds(250),
                                            std::chrono::milliseconds(100)));
      break;
    }
    if (poll_result < 0 && errno != EINTR) {
      publisher_cleanup_exit_code =
          shadow::supervisorExitCodeForChildTermination(
              shadow::terminateOneShotChild(publisher,
                                            std::chrono::milliseconds(250),
                                            std::chrono::milliseconds(100)));
      break;
    }
  }
  close(producer_pidfd);
  shadow::closeBootstrap(bootstrap.value());
  return publisher_cleanup_exit_code;
}
