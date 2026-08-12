#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace simple_pure_pursuit {
namespace {

using CandidateExecutionPoint =
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint;
using ControllerGeometry = multi_purpose_mpc_ros_msgs::msg::ControllerGeometry;
using ControllerSampleKey =
    multi_purpose_mpc_ros_msgs::msg::ControllerSampleKey;

constexpr std::size_t kAw2MaxGeometryPoints =
    aw2_shadow::kMaxGeometryPoints;
constexpr std::size_t kAw2MaxRolloutSamples = 100U;
constexpr std::size_t kAw2MaxSourceBytes = 4096U;
constexpr std::size_t kAw2MaxFrameBytes = 128U;
constexpr std::size_t kAw2MaxTargetBytes = 64U;
constexpr std::size_t kAw2MaxRecordBytes = 1024U * 1024U;
constexpr std::size_t kAw2TrackerEntries = 8U;

constexpr std::array<std::uint32_t, 64U> kSha256RoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

std::uint32_t rotateRight(std::uint32_t value, std::uint32_t bits) {
  return (value >> bits) | (value << (32U - bits));
}

void appendU8(std::vector<std::uint8_t> &bytes, std::uint8_t value) {
  bytes.push_back(value);
}

void appendU32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  for (std::uint32_t shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void appendU64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
  for (std::uint32_t shift = 0U; shift < 64U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void appendI32(std::vector<std::uint8_t> &bytes, std::int32_t value) {
  appendU32(bytes, static_cast<std::uint32_t>(value));
}

void appendFloat(std::vector<std::uint8_t> &bytes, float value) {
  std::uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32(bytes, bits);
}

void appendDouble(std::vector<std::uint8_t> &bytes, double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendU64(bytes, bits);
}

void appendString(std::vector<std::uint8_t> &bytes, const std::string &value) {
  appendU32(bytes, static_cast<std::uint32_t>(value.size()));
  bytes.insert(bytes.end(), value.begin(), value.end());
}

void appendDomain(std::vector<std::uint8_t> &bytes, const char *domain) {
  bytes.insert(bytes.end(), domain, domain + std::strlen(domain));
}

void appendTime(std::vector<std::uint8_t> &bytes,
                const builtin_interfaces::msg::Time &stamp) {
  appendI32(bytes, stamp.sec);
  appendU32(bytes, stamp.nanosec);
}

bool stampValid(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

bool digestPresent(const Aw2Sha256Digest &digest) {
  return std::any_of(digest.begin(), digest.end(),
                     [](std::uint8_t value) { return value != 0U; });
}

bool poseFiniteAndNormalized(const Pose &pose) {
  const auto &q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(q.x) &&
         std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         std::isfinite(norm) && std::abs(norm - 1.0) <= 1.0e-6;
}

bool pointValid(const TrajectoryPoint &point) {
  return point.time_from_start.sec >= 0 &&
         point.time_from_start.nanosec < 1000000000U &&
         poseFiniteAndNormalized(point.pose) &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         std::isfinite(point.lateral_velocity_mps) &&
         std::isfinite(point.acceleration_mps2) &&
         std::isfinite(point.heading_rate_rps) &&
         std::isfinite(point.front_wheel_angle_rad) &&
         std::isfinite(point.rear_wheel_angle_rad);
}

CandidateExecutionPoint copyPoint(const TrajectoryPoint &point) {
  CandidateExecutionPoint copy;
  copy.time_from_start = point.time_from_start;
  copy.position_x_m = point.pose.position.x;
  copy.position_y_m = point.pose.position.y;
  copy.position_z_m = point.pose.position.z;
  copy.orientation_x = point.pose.orientation.x;
  copy.orientation_y = point.pose.orientation.y;
  copy.orientation_z = point.pose.orientation.z;
  copy.orientation_w = point.pose.orientation.w;
  copy.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  copy.lateral_velocity_mps = point.lateral_velocity_mps;
  copy.acceleration_mps2 = point.acceleration_mps2;
  copy.heading_rate_rps = point.heading_rate_rps;
  copy.front_wheel_angle_rad = point.front_wheel_angle_rad;
  copy.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
  return copy;
}

void appendPoint(std::vector<std::uint8_t> &bytes,
                 const CandidateExecutionPoint &point) {
  appendI32(bytes, point.time_from_start.sec);
  appendU32(bytes, point.time_from_start.nanosec);
  appendDouble(bytes, point.position_x_m);
  appendDouble(bytes, point.position_y_m);
  appendDouble(bytes, point.position_z_m);
  appendDouble(bytes, point.orientation_x);
  appendDouble(bytes, point.orientation_y);
  appendDouble(bytes, point.orientation_z);
  appendDouble(bytes, point.orientation_w);
  appendFloat(bytes, point.longitudinal_velocity_mps);
  appendFloat(bytes, point.lateral_velocity_mps);
  appendFloat(bytes, point.acceleration_mps2);
  appendFloat(bytes, point.heading_rate_rps);
  appendFloat(bytes, point.front_wheel_angle_rad);
  appendFloat(bytes, point.rear_wheel_angle_rad);
}

void appendPose(std::vector<std::uint8_t> &bytes, const Pose &pose) {
  appendDouble(bytes, pose.position.x);
  appendDouble(bytes, pose.position.y);
  appendDouble(bytes, pose.position.z);
  appendDouble(bytes, pose.orientation.x);
  appendDouble(bytes, pose.orientation.y);
  appendDouble(bytes, pose.orientation.z);
  appendDouble(bytes, pose.orientation.w);
}

bool commandValid(const AckermannControlCommand &command) {
  return stampValid(command.stamp) && stampValid(command.lateral.stamp) &&
         stampValid(command.longitudinal.stamp) &&
         std::isfinite(command.lateral.steering_tire_angle) &&
         std::isfinite(command.lateral.steering_tire_rotation_rate) &&
         std::isfinite(command.longitudinal.speed) &&
         std::isfinite(command.longitudinal.acceleration) &&
         std::isfinite(command.longitudinal.jerk);
}

std::vector<std::uint8_t>
canonicalCommand(const AckermannControlCommand &command) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(128U);
  appendDomain(bytes, "AW2:CONTROLLER_COMMAND:v1");
  appendTime(bytes, command.stamp);
  appendTime(bytes, command.lateral.stamp);
  appendFloat(bytes, command.lateral.steering_tire_angle);
  appendFloat(bytes, command.lateral.steering_tire_rotation_rate);
  appendTime(bytes, command.longitudinal.stamp);
  appendFloat(bytes, command.longitudinal.speed);
  appendFloat(bytes, command.longitudinal.acceleration);
  appendFloat(bytes, command.longitudinal.jerk);
  return bytes;
}

void appendCommandFields(std::vector<std::uint8_t> &bytes,
                         const AckermannControlCommand &command) {
  appendTime(bytes, command.stamp);
  appendTime(bytes, command.lateral.stamp);
  appendFloat(bytes, command.lateral.steering_tire_angle);
  appendFloat(bytes, command.lateral.steering_tire_rotation_rate);
  appendTime(bytes, command.longitudinal.stamp);
  appendFloat(bytes, command.longitudinal.speed);
  appendFloat(bytes, command.longitudinal.acceleration);
  appendFloat(bytes, command.longitudinal.jerk);
}

ControllerGeometryBuildResult buildGeometryImpl(
    const Trajectory &trajectory, std::uint32_t original_point_count,
    std::size_t nearest_source_index, std::size_t speed_cap_source_index,
    std::size_t curvature_last_read_source_index,
    std::size_t lookahead_selected_source_index,
    std::size_t required_horizon_end_source_index,
    bool lookahead_endpoint_fallback, double required_spatial_horizon_m) {
  ControllerGeometryBuildResult result;
  result.geometry.frame_id = trajectory.header.frame_id;
  result.geometry.source_stamp = trajectory.header.stamp;
  const std::size_t source_original_point_count = original_point_count == 0U
                                                      ? trajectory.points.size()
                                                      : original_point_count;
  result.geometry.original_point_count = static_cast<std::uint32_t>(std::min(
      source_original_point_count,
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
  const auto last_source_index = std::max(
      {nearest_source_index, speed_cap_source_index,
       curvature_last_read_source_index, lookahead_selected_source_index,
       required_horizon_end_source_index});
  const bool source_is_full =
      source_original_point_count == trajectory.points.size();
  const std::size_t local_first_index =
      source_is_full ? nearest_source_index : 0U;
  const std::size_t local_last_index =
      source_is_full
          ? last_source_index
          : (trajectory.points.empty() ? 0U : trajectory.points.size() - 1U);
  result.bounded =
      !trajectory.points.empty() &&
      source_original_point_count >= trajectory.points.size() &&
      nearest_source_index < source_original_point_count &&
      speed_cap_source_index >= nearest_source_index &&
      curvature_last_read_source_index >= nearest_source_index &&
      lookahead_selected_source_index >= nearest_source_index &&
      required_horizon_end_source_index >= nearest_source_index &&
      last_source_index < source_original_point_count &&
      last_source_index >= nearest_source_index &&
      last_source_index - nearest_source_index + 1U <= kAw2MaxGeometryPoints &&
      local_first_index < trajectory.points.size() &&
      local_last_index < trajectory.points.size() &&
      local_last_index - local_first_index + 1U ==
          last_source_index - nearest_source_index + 1U;
  if (result.bounded) {
    result.geometry.first_source_index =
        static_cast<std::uint32_t>(nearest_source_index);
    result.geometry.last_source_index =
        static_cast<std::uint32_t>(last_source_index);
    result.geometry.nearest_source_index =
        static_cast<std::uint32_t>(nearest_source_index);
    result.geometry.speed_cap_source_index =
        static_cast<std::uint32_t>(speed_cap_source_index);
    result.geometry.curvature_last_read_source_index =
        static_cast<std::uint32_t>(curvature_last_read_source_index);
    result.geometry.lookahead_selected_source_index =
        static_cast<std::uint32_t>(lookahead_selected_source_index);
    result.geometry.required_horizon_end_source_index =
        static_cast<std::uint32_t>(required_horizon_end_source_index);
    result.geometry.lookahead_endpoint_fallback = lookahead_endpoint_fallback;
    result.geometry.points.reserve(local_last_index - local_first_index + 1U);
    for (std::size_t index = local_first_index; index <= local_last_index;
         ++index) {
      result.geometry.points.push_back(copyPoint(trajectory.points[index]));
    }
  }
  result.valid =
      result.bounded && !trajectory.header.frame_id.empty() &&
      trajectory.header.frame_id.size() <= kAw2MaxFrameBytes &&
      stampValid(trajectory.header.stamp) &&
      std::isfinite(required_spatial_horizon_m) &&
      required_spatial_horizon_m > 0.0 &&
      std::all_of(trajectory.points.begin() +
                      static_cast<std::ptrdiff_t>(local_first_index),
                  trajectory.points.begin() +
                      static_cast<std::ptrdiff_t>(local_last_index + 1U),
                  pointValid);
  if (!result.valid) {
    return result;
  }

  double total_arc_length_m = 0.0;
  for (std::size_t index = local_first_index + 1U; index <= local_last_index;
       ++index) {
    const auto &previous = trajectory.points[index - 1U].pose.position;
    const auto &current = trajectory.points[index].pose.position;
    total_arc_length_m +=
        std::hypot(current.x - previous.x, current.y - previous.y);
  }
  if (!std::isfinite(total_arc_length_m)) {
    result.valid = false;
    return result;
  }
  result.geometry.total_arc_length_m = total_arc_length_m;
  result.geometry.required_spatial_horizon_m = required_spatial_horizon_m;

  std::vector<std::uint8_t> geometry_bytes;
  geometry_bytes.reserve(
      256U + result.geometry.points.size() * 88U);
  appendDomain(geometry_bytes, "AW2:CONTROLLER_USED_INTERVAL:v1");
  appendString(geometry_bytes, trajectory.header.frame_id);
  appendTime(geometry_bytes, trajectory.header.stamp);
  appendU32(geometry_bytes, result.geometry.original_point_count);
  appendU32(geometry_bytes, result.geometry.first_source_index);
  appendU32(geometry_bytes, result.geometry.last_source_index);
  appendU32(geometry_bytes, result.geometry.nearest_source_index);
  appendU32(geometry_bytes, result.geometry.speed_cap_source_index);
  appendU32(geometry_bytes, result.geometry.curvature_last_read_source_index);
  appendU32(geometry_bytes, result.geometry.lookahead_selected_source_index);
  appendU32(geometry_bytes, result.geometry.required_horizon_end_source_index);
  appendU8(geometry_bytes,
           result.geometry.lookahead_endpoint_fallback ? 1U : 0U);
  for (const auto &point : result.geometry.points) {
    appendPoint(geometry_bytes, point);
  }
  appendDouble(geometry_bytes, total_arc_length_m);
  appendDouble(geometry_bytes, required_spatial_horizon_m);
  result.geometry.geometry_sha256 = aw2Sha256(std::move(geometry_bytes));

  std::vector<std::uint8_t> point_bytes;
  point_bytes.reserve(128U);
  appendDomain(point_bytes, "AW2:CONTROLLER_USED_INTERVAL:v1");
  appendPoint(point_bytes, result.geometry.points.front());
  result.geometry.start_point_sha256 = aw2Sha256(point_bytes);
  point_bytes.clear();
  appendDomain(point_bytes, "AW2:CONTROLLER_USED_INTERVAL:v1");
  appendPoint(point_bytes, result.geometry.points.back());
  result.geometry.end_point_sha256 = aw2Sha256(point_bytes);
  return result;
}

void appendDigest(std::vector<std::uint8_t> &bytes,
                  const Aw2Sha256Digest &digest) {
  bytes.insert(bytes.end(), digest.begin(), digest.end());
}

void appendGeometry(std::vector<std::uint8_t> &bytes,
                    const ControllerGeometry &geometry) {
  appendString(bytes, geometry.frame_id);
  appendTime(bytes, geometry.source_stamp);
  appendU32(bytes, geometry.original_point_count);
  appendU32(bytes, geometry.first_source_index);
  appendU32(bytes, geometry.last_source_index);
  appendU32(bytes, geometry.nearest_source_index);
  appendU32(bytes, geometry.speed_cap_source_index);
  appendU32(bytes, geometry.curvature_last_read_source_index);
  appendU32(bytes, geometry.lookahead_selected_source_index);
  appendU32(bytes, geometry.required_horizon_end_source_index);
  appendU8(bytes, geometry.lookahead_endpoint_fallback ? 1U : 0U);
  appendU32(bytes, static_cast<std::uint32_t>(geometry.points.size()));
  for (const auto &point : geometry.points) {
    appendPoint(bytes, point);
  }
  appendDigest(bytes, geometry.geometry_sha256);
  appendDouble(bytes, geometry.total_arc_length_m);
  appendDouble(bytes, geometry.required_spatial_horizon_m);
  appendDigest(bytes, geometry.start_point_sha256);
  appendDigest(bytes, geometry.end_point_sha256);
  appendU8(bytes, geometry.full_source_digest_state);
  appendDigest(bytes, geometry.full_source_sha256);
}

bool executionPointValid(const CandidateExecutionPoint &point) {
  const double quaternion_norm =
      std::sqrt(point.orientation_x * point.orientation_x +
                point.orientation_y * point.orientation_y +
                point.orientation_z * point.orientation_z +
                point.orientation_w * point.orientation_w);
  return point.time_from_start.sec >= 0 &&
         point.time_from_start.nanosec < 1000000000U &&
         std::isfinite(point.position_x_m) &&
         std::isfinite(point.position_y_m) &&
         std::isfinite(point.position_z_m) &&
         std::isfinite(point.orientation_x) &&
         std::isfinite(point.orientation_y) &&
         std::isfinite(point.orientation_z) &&
         std::isfinite(point.orientation_w) &&
         std::isfinite(quaternion_norm) &&
         std::abs(quaternion_norm - 1.0) <= 1.0e-6 &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         std::isfinite(point.lateral_velocity_mps) &&
         std::isfinite(point.acceleration_mps2) &&
         std::isfinite(point.heading_rate_rps) &&
         std::isfinite(point.front_wheel_angle_rad) &&
         std::isfinite(point.rear_wheel_angle_rad);
}

bool geometryValidForAckV2(const ControllerGeometry &geometry) {
  const auto point_count = geometry.points.size();
  return !geometry.frame_id.empty() &&
         geometry.frame_id.size() <= kAw2MaxFrameBytes &&
         stampValid(geometry.source_stamp) &&
         geometry.original_point_count >= 2U && point_count >= 2U &&
         point_count <= kAw2MaxGeometryPoints &&
         geometry.first_source_index <= geometry.nearest_source_index &&
         geometry.nearest_source_index <= geometry.last_source_index &&
         geometry.last_source_index < geometry.original_point_count &&
         geometry.speed_cap_source_index >= geometry.first_source_index &&
         geometry.speed_cap_source_index <= geometry.last_source_index &&
         geometry.curvature_last_read_source_index >=
             geometry.first_source_index &&
         geometry.curvature_last_read_source_index <=
             geometry.last_source_index &&
         geometry.lookahead_selected_source_index >=
             geometry.first_source_index &&
         geometry.lookahead_selected_source_index <=
             geometry.last_source_index &&
         geometry.required_horizon_end_source_index >=
             geometry.first_source_index &&
         geometry.required_horizon_end_source_index <=
             geometry.last_source_index &&
         point_count ==
             static_cast<std::size_t>(geometry.last_source_index -
                                      geometry.first_source_index + 1U) &&
         std::all_of(geometry.points.begin(), geometry.points.end(),
                     executionPointValid) &&
         digestPresent(geometry.geometry_sha256) &&
         std::isfinite(geometry.total_arc_length_m) &&
         geometry.total_arc_length_m >= 0.0 &&
         std::isfinite(geometry.required_spatial_horizon_m) &&
         geometry.required_spatial_horizon_m > 0.0 &&
         digestPresent(geometry.start_point_sha256) &&
         digestPresent(geometry.end_point_sha256) &&
         digestPresent(geometry.full_source_sha256);
}

bool freeRunExecutionAckValidV2(const FreeRunExecutionAck &ack) {
  const auto &plan = ack.plan_key;
  const auto &source = ack.source_key;
  return ack.schema_version == FreeRunExecutionAck::SCHEMA_V2 &&
         ack.ack_eligible &&
         ack.evidence_state == FreeRunExecutionAck::EVIDENCE_COMPLETE &&
         ack.evidence_reason.size() <= 256U &&
         !ack.header.frame_id.empty() && stampValid(ack.header.stamp) &&
         plan.canonical_algorithm_version != 0U &&
         plan.race_arm_epoch > 0U && plan.planner_instance_id > 0U &&
         plan.plan_generation > 0U && stampValid(plan.plan_stamp) &&
         digestPresent(plan.canonical_plan_payload_sha256) &&
         source.canonical_algorithm_version != 0U &&
         source.baseline_instance_id > 0U &&
         source.controller_instance_id > 0U &&
         source.source_generation > 0U && stampValid(source.source_stamp) &&
         source.original_point_count >= 2U &&
         digestPresent(source.baseline_reference_sha256) &&
         digestPresent(source.controller_implementation_sha256) &&
         digestPresent(source.controller_config_sha256) &&
         ack.controller_sequence > 0U &&
         stampValid(ack.controller_command_stamp) &&
         geometryValidForAckV2(ack.base_geometry) &&
         geometryValidForAckV2(ack.applied_geometry) &&
         poseFiniteAndNormalized(ack.control_pose) &&
         stampValid(ack.control_pose_stamp) &&
         commandValid(ack.raw_controller_command) &&
         commandValid(ack.output_controller_command) &&
         digestPresent(ack.raw_controller_command_sha256) &&
         digestPresent(ack.output_controller_command_sha256) &&
         std::isfinite(ack.trajectory_progress_m) &&
         std::isfinite(ack.raw_steering_tire_angle_rad) &&
         std::isfinite(ack.output_steering_tire_angle_rad) &&
         std::isfinite(ack.hard_steering_tire_angle_limit_rad) &&
         ack.hard_steering_tire_angle_limit_rad > 0.0 &&
         std::isfinite(ack.required_spatial_horizon_m) &&
         ack.required_spatial_horizon_m > 0.0 &&
         std::isfinite(ack.available_spatial_horizon_m) &&
         ack.available_spatial_horizon_m >=
             ack.required_spatial_horizon_m;
}

std::vector<std::uint8_t>
canonicalFreeRunExecutionAckWireV2(const FreeRunExecutionAck &ack) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(2048U + (ack.base_geometry.points.size() +
                         ack.applied_geometry.points.size()) *
                            128U);
  appendDomain(bytes, "FREE_RUN_EXECUTION_ACK_CANONICAL_V2");
  appendTime(bytes, ack.header.stamp);
  appendString(bytes, ack.header.frame_id);
  appendU8(bytes, ack.schema_version);
  appendU8(bytes, ack.ack_eligible ? 1U : 0U);
  appendU8(bytes, ack.evidence_state);
  appendString(bytes, ack.evidence_reason);

  appendU8(bytes, ack.plan_key.canonical_algorithm_version);
  appendU64(bytes, ack.plan_key.race_arm_epoch);
  appendU64(bytes, ack.plan_key.planner_instance_id);
  appendU32(bytes, ack.plan_key.plan_generation);
  appendTime(bytes, ack.plan_key.plan_stamp);
  appendDigest(bytes, ack.plan_key.canonical_plan_payload_sha256);

  appendU8(bytes, ack.source_key.canonical_algorithm_version);
  appendU64(bytes, ack.source_key.baseline_instance_id);
  appendU64(bytes, ack.source_key.controller_instance_id);
  appendU32(bytes, ack.source_key.source_generation);
  appendTime(bytes, ack.source_key.source_stamp);
  appendU32(bytes, ack.source_key.original_point_count);
  appendDigest(bytes, ack.source_key.baseline_reference_sha256);
  appendDigest(bytes, ack.source_key.controller_implementation_sha256);
  appendDigest(bytes, ack.source_key.controller_config_sha256);

  appendU64(bytes, ack.controller_sequence);
  appendTime(bytes, ack.controller_command_stamp);
  appendGeometry(bytes, ack.base_geometry);
  appendGeometry(bytes, ack.applied_geometry);
  appendPose(bytes, ack.control_pose);
  appendTime(bytes, ack.control_pose_stamp);
  appendU32(bytes, ack.nearest_trajectory_index);
  appendDouble(bytes, ack.trajectory_progress_m);
  appendCommandFields(bytes, ack.raw_controller_command);
  appendCommandFields(bytes, ack.output_controller_command);
  appendDigest(bytes, ack.raw_controller_command_sha256);
  appendDigest(bytes, ack.output_controller_command_sha256);
  appendDouble(bytes, ack.raw_steering_tire_angle_rad);
  appendDouble(bytes, ack.output_steering_tire_angle_rad);
  appendDouble(bytes, ack.hard_steering_tire_angle_limit_rad);
  appendDouble(bytes, ack.required_spatial_horizon_m);
  appendDouble(bytes, ack.available_spatial_horizon_m);
  appendDigest(bytes, Aw2Sha256Digest{});
  return bytes;
}

void appendSampleKey(std::vector<std::uint8_t> &bytes,
                     const ControllerSampleKey &key) {
  const auto &plan = key.plan_sample_key;
  appendU64(bytes, plan.race_arm_epoch);
  appendU64(bytes, plan.planner_instance_id);
  appendU64(bytes, plan.attempt_id);
  appendString(bytes, plan.target_vehicle_id);
  appendU8(bytes, static_cast<std::uint8_t>(plan.pass_direction));
  appendU64(bytes, plan.connector_transaction_id);
  appendTime(bytes, plan.plan_stamp);
  appendU32(bytes, plan.plan_generation);
  appendU8(bytes, key.controller_role);
  appendU64(bytes, key.controller_instance_id);
  appendU64(bytes, key.controller_sequence);
  appendTime(bytes, key.controller_command_stamp);
}

void appendRolloutSample(std::vector<std::uint8_t> &bytes,
                         const ExecutionSweepSample &sample) {
  appendFloat(bytes, sample.elapsed_time_sec);
  appendFloat(bytes, sample.progress_m);
  appendPose(bytes, sample.pose);
  appendFloat(bytes, sample.speed_mps);
  appendFloat(bytes, sample.raw_steering_tire_angle_rad);
  appendFloat(bytes, sample.bounded_steering_tire_angle_rad);
  appendFloat(bytes, sample.raw_steering_tire_rotation_rate_radps);
  appendFloat(bytes, sample.bounded_steering_tire_rotation_rate_radps);
}

std::vector<std::uint8_t>
canonicalEnvelope(const ControllerAppliedEnvelope &envelope,
                  bool include_evidence) {
  std::vector<std::uint8_t> bytes;
  appendDomain(bytes, "AW2:CONTROLLER_APPLIED_ENVELOPE:v1");
  appendTime(bytes, envelope.record_stamp);
  appendString(bytes, envelope.record_frame_id);
  appendU8(bytes, envelope.schema_version);
  appendU8(bytes, envelope.authority_eligible ? 1U : 0U);
  appendSampleKey(bytes, envelope.controller_sample_key);
  appendU32(bytes, envelope.candidate_revision);
  appendDigest(bytes, envelope.candidate_content_sha256);
  appendU8(bytes, envelope.geometry_relation);
  appendU32(bytes, envelope.source_generation);
  appendU32(bytes, envelope.source_original_size_bytes);
  appendU8(bytes, envelope.source_wire_complete ? 1U : 0U);
  appendU32(bytes,
            static_cast<std::uint32_t>(envelope.canonical_source_wire.size()));
  bytes.insert(bytes.end(), envelope.canonical_source_wire.begin(),
               envelope.canonical_source_wire.end());
  appendDigest(bytes, envelope.canonical_source_sha256);
  appendGeometry(bytes, envelope.base_geometry);
  appendGeometry(bytes, envelope.applied_geometry);
  appendPose(bytes, envelope.control_pose);
  appendTime(bytes, envelope.control_pose_stamp);
  appendU32(bytes, envelope.nearest_trajectory_index);
  appendDouble(bytes, envelope.trajectory_progress_m);
  appendDigest(bytes, envelope.controller_adapter_implementation_sha256);
  appendDigest(bytes, envelope.controller_adapter_config_sha256);
  const auto raw_command = canonicalCommand(envelope.raw_controller_command);
  bytes.insert(bytes.end(), raw_command.begin(), raw_command.end());
  const auto output_command =
      canonicalCommand(envelope.output_controller_command);
  bytes.insert(bytes.end(), output_command.begin(), output_command.end());
  appendDigest(bytes, envelope.raw_controller_command_sha256);
  appendDigest(bytes, envelope.output_controller_command_sha256);
  appendDouble(bytes, envelope.raw_steering_tire_angle_rad);
  appendDouble(bytes, envelope.output_steering_tire_angle_rad);
  appendDouble(bytes, envelope.raw_steering_tire_rotation_rate_radps);
  appendDouble(bytes, envelope.output_steering_tire_rotation_rate_radps);
  appendU8(bytes, envelope.hard_actuator_limits_present ? 1U : 0U);
  appendDouble(bytes, envelope.hard_steering_tire_angle_limit_rad);
  appendDouble(bytes, envelope.hard_steering_tire_rotation_rate_limit_radps);
  appendDouble(bytes, envelope.available_spatial_horizon_m);
  appendDouble(bytes, envelope.required_spatial_horizon_m);
  appendU8(bytes, envelope.rollout_state);
  appendU32(bytes, envelope.rollout_original_sample_count);
  appendU32(bytes, static_cast<std::uint32_t>(envelope.rollout_samples.size()));
  for (const auto &sample : envelope.rollout_samples) {
    appendRolloutSample(bytes, sample);
  }
  if (include_evidence) {
    appendU8(bytes, envelope.evidence_state);
    appendString(bytes, envelope.evidence_reason);
  }
  return bytes;
}

bool rolloutValid(const ControllerAppliedEnvelope &envelope) {
  if (envelope.rollout_state != ControllerAppliedEnvelope::ROLLOUT_COMPLETE ||
      envelope.rollout_samples.empty() ||
      envelope.rollout_original_sample_count !=
          envelope.rollout_samples.size()) {
    return false;
  }
  float previous_time = -1.0F;
  float previous_progress = static_cast<float>(envelope.trajectory_progress_m);
  for (std::size_t index = 0U; index < envelope.rollout_samples.size();
       ++index) {
    const auto &sample = envelope.rollout_samples[index];
    const bool first_matches =
        index != 0U ||
        (sample.elapsed_time_sec == 0.0F &&
         sample.progress_m ==
             static_cast<float>(envelope.trajectory_progress_m) &&
         sample.pose == envelope.control_pose);
    if (!first_matches || !poseFiniteAndNormalized(sample.pose) ||
        !std::isfinite(sample.elapsed_time_sec) ||
        !std::isfinite(sample.progress_m) || !std::isfinite(sample.speed_mps) ||
        !std::isfinite(sample.raw_steering_tire_angle_rad) ||
        !std::isfinite(sample.bounded_steering_tire_angle_rad) ||
        !std::isfinite(sample.raw_steering_tire_rotation_rate_radps) ||
        !std::isfinite(sample.bounded_steering_tire_rotation_rate_radps) ||
        sample.elapsed_time_sec < previous_time ||
        sample.progress_m < previous_progress) {
      return false;
    }
    previous_time = sample.elapsed_time_sec;
    previous_progress = sample.progress_m;
  }
  return static_cast<double>(envelope.rollout_samples.back().progress_m) -
             envelope.trajectory_progress_m >=
         envelope.required_spatial_horizon_m;
}

bool sampleKeyEqual(const ControllerSampleKey &left,
                    const ControllerSampleKey &right) {
  return left == right;
}

} // namespace

Aw2Sha256Digest aw2Sha256(const std::vector<std::uint8_t> &bytes) {
  return aw2Sha256(std::vector<std::uint8_t>(bytes));
}

Aw2Sha256Digest aw2Sha256(std::vector<std::uint8_t> &&bytes) {
  std::vector<std::uint8_t> padded = std::move(bytes);
  const std::uint64_t bit_length =
      static_cast<std::uint64_t>(padded.size()) * 8U;
  padded.push_back(0x80U);
  while (padded.size() % 64U != 56U) {
    padded.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    padded.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xffU));
  }

  std::array<std::uint32_t, 8U> state{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                      0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                      0x1f83d9abU, 0x5be0cd19U};
  for (std::size_t offset = 0U; offset < padded.size(); offset += 64U) {
    std::array<std::uint32_t, 64U> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t base = offset + index * 4U;
      words[index] = (static_cast<std::uint32_t>(padded[base]) << 24U) |
                     (static_cast<std::uint32_t>(padded[base + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(padded[base + 2U]) << 8U) |
                     static_cast<std::uint32_t>(padded[base + 3U]);
    }
    for (std::size_t index = 16U; index < 64U; ++index) {
      const std::uint32_t s0 = rotateRight(words[index - 15U], 7U) ^
                               rotateRight(words[index - 15U], 18U) ^
                               (words[index - 15U] >> 3U);
      const std::uint32_t s1 = rotateRight(words[index - 2U], 17U) ^
                               rotateRight(words[index - 2U], 19U) ^
                               (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];
    for (std::size_t index = 0U; index < 64U; ++index) {
      const std::uint32_t sum1 =
          rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 =
          h + sum1 + choose + kSha256RoundConstants[index] + words[index];
      const std::uint32_t sum0 =
          rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  Aw2Sha256Digest digest{};
  for (std::size_t index = 0U; index < state.size(); ++index) {
    digest[index * 4U] =
        static_cast<std::uint8_t>((state[index] >> 24U) & 0xffU);
    digest[index * 4U + 1U] =
        static_cast<std::uint8_t>((state[index] >> 16U) & 0xffU);
    digest[index * 4U + 2U] =
        static_cast<std::uint8_t>((state[index] >> 8U) & 0xffU);
    digest[index * 4U + 3U] = static_cast<std::uint8_t>(state[index] & 0xffU);
  }
  return digest;
}

Aw2Sha256Digest canonicalControllerCommandDigestV1(
    const AckermannControlCommand &command) {
  if (!commandValid(command)) {
    return {};
  }
  return aw2Sha256(canonicalCommand(command));
}

Aw2Sha256Digest canonicalFreeRunExecutionAckDigestV2(
    const FreeRunExecutionAck &ack) {
  if (!freeRunExecutionAckValidV2(ack)) {
    return {};
  }
  return aw2Sha256(canonicalFreeRunExecutionAckWireV2(ack));
}

ControllerGeometryBuildResult buildControllerGeometryV1(
    const Trajectory &trajectory, std::uint32_t original_point_count,
    std::size_t nearest_source_index, std::size_t speed_cap_source_index,
    std::size_t curvature_last_read_source_index,
    std::size_t lookahead_selected_source_index,
    std::size_t required_horizon_end_source_index,
    bool lookahead_endpoint_fallback, double required_spatial_horizon_m) {
  return buildGeometryImpl(
      trajectory, original_point_count, nearest_source_index,
      speed_cap_source_index, curvature_last_read_source_index,
      lookahead_selected_source_index, required_horizon_end_source_index,
      lookahead_endpoint_fallback, required_spatial_horizon_m);
}

Aw2CanonicalSourceWire serializeAw2SourceWire(const Float32MultiArray &source) {
  Aw2CanonicalSourceWire result;
  std::uint64_t original_size = std::strlen("AW2:SOURCE_WIRE:v1") + 4U + 4U;
  for (const auto &dimension : source.layout.dim) {
    original_size += 4U + dimension.label.size() + 4U + 4U;
  }
  original_size += 4U + source.data.size() * sizeof(float);
  result.original_size_bytes =
      static_cast<std::uint32_t>(std::min<std::uint64_t>(
          original_size, std::numeric_limits<std::uint32_t>::max()));
  if (original_size > kAw2MaxSourceBytes ||
      source.layout.dim.size() > std::numeric_limits<std::uint32_t>::max() ||
      source.data.size() > std::numeric_limits<std::uint32_t>::max() ||
      std::any_of(source.layout.dim.begin(), source.layout.dim.end(),
                  [](const auto &dimension) {
                    return dimension.label.size() > kAw2MaxFrameBytes;
                  })) {
    return result;
  }
  if (std::any_of(source.data.begin(), source.data.end(),
                  [](float value) { return !std::isfinite(value); })) {
    return result;
  }

  appendDomain(result.bytes, "AW2:SOURCE_WIRE:v1");
  appendU32(result.bytes, source.layout.data_offset);
  appendU32(result.bytes, static_cast<std::uint32_t>(source.layout.dim.size()));
  for (const auto &dimension : source.layout.dim) {
    appendString(result.bytes, dimension.label);
    appendU32(result.bytes, dimension.size);
    appendU32(result.bytes, dimension.stride);
  }
  appendU32(result.bytes, static_cast<std::uint32_t>(source.data.size()));
  for (const float value : source.data) {
    appendFloat(result.bytes, value);
  }
  result.complete = result.bytes.size() == result.original_size_bytes &&
                    result.bytes.size() <= kAw2MaxSourceBytes;
  return result;
}

Aw2CanonicalSourceWire
canonicalizeAw2SourceWire(const Float32MultiArray &source) {
  auto result = serializeAw2SourceWire(source);
  if (result.complete) {
    result.sha256 = aw2Sha256(result.bytes);
  }
  return result;
}

Aw2Sha256Digest aw2ControllerAdapterImplementationDigestV1() {
  std::vector<std::uint8_t> bytes;
  appendDomain(bytes, "AW2:CONTROLLER_ADAPTER_IMPLEMENTATION:v1");
  return aw2Sha256(bytes);
}

Aw2Sha256Digest
aw2ControllerAdapterConfigDigestV1(const std::vector<double> &scalar_values,
                                   const std::vector<bool> &boolean_values) {
  if (scalar_values.size() > 64U || boolean_values.size() > 64U ||
      std::any_of(scalar_values.begin(), scalar_values.end(),
                  [](double value) { return !std::isfinite(value); })) {
    return {};
  }
  std::vector<std::uint8_t> bytes;
  appendDomain(bytes, "AW2:CONTROLLER_ADAPTER_CONFIG:v1");
  appendU32(bytes, static_cast<std::uint32_t>(scalar_values.size()));
  for (const double value : scalar_values) {
    appendDouble(bytes, value);
  }
  appendU32(bytes, static_cast<std::uint32_t>(boolean_values.size()));
  for (const bool value : boolean_values) {
    appendU8(bytes, value ? 1U : 0U);
  }
  return aw2Sha256(bytes);
}

ControllerAppliedBindingTracker::Observation
ControllerAppliedBindingTracker::observe(
    const ControllerSampleKey &key, std::uint32_t candidate_revision,
    const Aw2Sha256Digest &candidate_content_sha256,
    const Aw2Sha256Digest &payload_sha256) {
  for (const auto &entry : entries_) {
    const auto &left = entry.key.plan_sample_key;
    const auto &right = key.plan_sample_key;
    if (left.race_arm_epoch == right.race_arm_epoch &&
        left.planner_instance_id == right.planner_instance_id &&
        left.plan_generation == right.plan_generation) {
      const bool transaction_mutated =
          left.attempt_id != right.attempt_id ||
          left.target_vehicle_id != right.target_vehicle_id ||
          left.pass_direction != right.pass_direction ||
          left.connector_transaction_id != right.connector_transaction_id;
      if (transaction_mutated ||
          entry.candidate_revision != candidate_revision ||
          entry.candidate_content_sha256 != candidate_content_sha256) {
        return Observation::SAME_GENERATION_PAYLOAD_MUTATION;
      }
    }
    if (entry.key.controller_instance_id == key.controller_instance_id) {
      if (key.controller_sequence < entry.key.controller_sequence) {
        return Observation::SEQUENCE_REGRESSION;
      }
      if (key.controller_sequence == entry.key.controller_sequence &&
          !sampleKeyEqual(entry.key, key)) {
        return Observation::DUPLICATE_CONFLICT;
      }
    }
    if (sampleKeyEqual(entry.key, key)) {
      return entry.payload_sha256 == payload_sha256
                 ? Observation::CONSISTENT_DUPLICATE
                 : Observation::DUPLICATE_CONFLICT;
    }
  }
  if (entries_.size() == kAw2TrackerEntries) {
    entries_.erase(entries_.begin());
  }
  entries_.push_back(
      Entry{key, candidate_revision, candidate_content_sha256, payload_sha256});
  return Observation::ACCEPTED;
}

ControllerAppliedEnvelope makeControllerAppliedEnvelopeV1(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerAppliedEnvelopeInput &input, const OvertakePlan *typed_plan,
    bool typed_plan_fresh, ControllerAppliedBindingTracker *binding_tracker) {
  ControllerAppliedEnvelope envelope;
  envelope.record_stamp = input.header.stamp;
  envelope.record_frame_id = input.header.frame_id;
  envelope.authority_eligible = false;
  envelope.controller_sample_key.controller_role = input.controller_role;
  envelope.controller_sample_key.controller_instance_id =
      command_envelope.producer_instance_id;
  envelope.controller_sample_key.controller_sequence =
      command_envelope.command_sequence;
  envelope.controller_sample_key.controller_command_stamp =
      command_envelope.command.stamp;
  if (input.capture_resource_limit_exceeded) {
    if (input.resource_sample_key_present) {
      envelope.controller_sample_key = input.resource_sample_key;
      envelope.candidate_revision = input.resource_candidate_revision;
      envelope.candidate_content_sha256 =
          input.resource_candidate_content_sha256;
    }
    envelope.schema_version = ControllerAppliedEnvelope::SCHEMA_INVALID;
    envelope.authority_eligible = false;
    envelope.evidence_state =
        ControllerAppliedEnvelope::EVIDENCE_RESOURCE_LIMIT_EXCEEDED;
    envelope.evidence_reason =
        input.capture_resource_limit_reason.empty()
            ? "capture_resource_limit_exceeded"
            : input.capture_resource_limit_reason.substr(0U, 256U);
    return envelope;
  }
  envelope.geometry_relation = input.geometry_relation;
  envelope.source_generation = input.source_generation;
  envelope.source_original_size_bytes = input.source_wire.original_size_bytes;
  envelope.source_wire_complete = input.source_wire.complete;
  envelope.canonical_source_wire.assign(input.source_wire.bytes.begin(),
                                        input.source_wire.bytes.end());
  envelope.canonical_source_sha256 = input.source_wire.sha256;
  envelope.control_pose = input.control_pose;
  envelope.control_pose_stamp = input.control_pose_stamp;
  envelope.nearest_trajectory_index = static_cast<std::uint32_t>(
      std::min<std::size_t>(input.nearest_trajectory_index,
                            std::numeric_limits<std::uint32_t>::max()));
  envelope.trajectory_progress_m = input.trajectory_progress_m;
  envelope.controller_adapter_implementation_sha256 =
      input.controller_adapter_implementation_sha256;
  envelope.controller_adapter_config_sha256 =
      input.controller_adapter_config_sha256;
  envelope.raw_controller_command = input.raw_controller_command;
  envelope.output_controller_command = command_envelope.command;
  if (commandValid(envelope.raw_controller_command)) {
    envelope.raw_controller_command_sha256 =
        aw2Sha256(canonicalCommand(envelope.raw_controller_command));
  }
  if (commandValid(envelope.output_controller_command)) {
    envelope.output_controller_command_sha256 =
        aw2Sha256(canonicalCommand(envelope.output_controller_command));
  }
  envelope.raw_steering_tire_angle_rad = input.raw_steering_tire_angle_rad;
  envelope.output_steering_tire_angle_rad =
      input.output_steering_tire_angle_rad;
  envelope.raw_steering_tire_rotation_rate_radps =
      input.raw_steering_tire_rotation_rate_radps;
  envelope.output_steering_tire_rotation_rate_radps =
      input.output_steering_tire_rotation_rate_radps;
  envelope.hard_actuator_limits_present = input.hard_actuator_limits_present;
  envelope.hard_steering_tire_angle_limit_rad =
      input.hard_steering_tire_angle_limit_rad;
  envelope.hard_steering_tire_rotation_rate_limit_radps =
      input.hard_steering_tire_rotation_rate_limit_radps;
  envelope.available_spatial_horizon_m = input.available_spatial_horizon_m;
  envelope.required_spatial_horizon_m = input.required_spatial_horizon_m;
  envelope.rollout_state = input.rollout_state;
  envelope.rollout_original_sample_count = static_cast<std::uint32_t>(
      std::min<std::size_t>(input.rollout_samples.size(),
                            std::numeric_limits<std::uint32_t>::max()));
  const auto copied_rollout_count =
      std::min(input.rollout_samples.size(), kAw2MaxRolloutSamples);
  envelope.rollout_samples.assign(
      input.rollout_samples.begin(),
      input.rollout_samples.begin() +
          static_cast<std::ptrdiff_t>(copied_rollout_count));

  const auto base = buildControllerGeometryV1(
      input.base_trajectory, input.base_original_point_count,
      input.nearest_trajectory_index, input.speed_cap_trajectory_index,
      input.curvature_last_read_trajectory_index,
      input.lookahead_selected_trajectory_index,
      input.required_horizon_end_trajectory_index,
      input.lookahead_endpoint_fallback, input.required_spatial_horizon_m);
  const auto applied = buildControllerGeometryV1(
      input.applied_trajectory, input.applied_original_point_count,
      input.nearest_trajectory_index, input.speed_cap_trajectory_index,
      input.curvature_last_read_trajectory_index,
      input.lookahead_selected_trajectory_index,
      input.required_horizon_end_trajectory_index,
      input.lookahead_endpoint_fallback, input.required_spatial_horizon_m);
  envelope.base_geometry = base.geometry;
  envelope.applied_geometry = applied.geometry;

  envelope.evidence_state = ControllerAppliedEnvelope::EVIDENCE_COMPLETE;
  envelope.evidence_reason = "complete";
  bool schema_valid = true;
  const auto fail = [&envelope](std::uint8_t state, const char *reason) {
    if (envelope.evidence_state ==
        ControllerAppliedEnvelope::EVIDENCE_COMPLETE) {
      envelope.evidence_state = state;
      envelope.evidence_reason = reason;
    }
  };

  if (typed_plan == nullptr) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_MISSING_INPUT,
         "typed_plan_missing");
  } else {
    auto &plan_key = envelope.controller_sample_key.plan_sample_key;
    plan_key.race_arm_epoch = typed_plan->race_arm_epoch;
    plan_key.planner_instance_id = typed_plan->planner_instance_id;
    plan_key.attempt_id = typed_plan->attempt_id;
    plan_key.target_vehicle_id = typed_plan->target_vehicle_id;
    plan_key.pass_direction = typed_plan->pass_direction;
    plan_key.connector_transaction_id = typed_plan->connector_transaction_id;
    plan_key.plan_stamp = typed_plan->header.stamp;
    plan_key.plan_generation = typed_plan->plan_generation;
    envelope.candidate_revision = typed_plan->candidate_revision;
    envelope.candidate_content_sha256 = typed_plan->candidate_content_sha256;

    if (!typed_plan_fresh) {
      fail(ControllerAppliedEnvelope::EVIDENCE_STALE_INPUT,
           "typed_plan_stale_or_out_of_order");
    } else if (typed_plan->aw2_identity_schema_version != 1U ||
               plan_key.race_arm_epoch == 0U ||
               plan_key.planner_instance_id == 0U ||
               plan_key.attempt_id == 0U ||
               plan_key.target_vehicle_id.empty() ||
               plan_key.target_vehicle_id.size() > kAw2MaxTargetBytes ||
               (plan_key.pass_direction != -1 &&
                plan_key.pass_direction != 1) ||
               plan_key.connector_transaction_id == 0U ||
               !stampValid(plan_key.plan_stamp) ||
               plan_key.plan_generation == 0U ||
               envelope.candidate_revision == 0U ||
               !digestPresent(envelope.candidate_content_sha256)) {
      schema_valid = false;
      fail(ControllerAppliedEnvelope::EVIDENCE_KEY_MISMATCH,
           "plan_sample_key_invalid");
    } else if (!typed_plan->trajectory_authorized) {
      fail(ControllerAppliedEnvelope::EVIDENCE_MISSING_INPUT,
           "typed_plan_not_authorized");
    }
  }

  if (input.controller_role != ControllerAppliedEnvelope::ROLE_PRIMARY) {
    fail(ControllerAppliedEnvelope::EVIDENCE_KEY_MISMATCH,
         input.controller_role == ControllerAppliedEnvelope::ROLE_RECOVERY
             ? "recovery_not_continuation_input"
             : "controller_role_unknown");
  } else if (input.geometry_relation <=
                 ControllerAppliedEnvelope::GEOMETRY_RELATION_UNKNOWN ||
             input.geometry_relation >
                 ControllerAppliedEnvelope::
                     GEOMETRY_RELATION_TRAJECTORY_BRIDGE_OUTPUT) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_RELATION_UNPROVEN,
         "geometry_relation_unknown");
  }

  if (input.header.frame_id.empty() ||
      input.header.frame_id.size() > kAw2MaxFrameBytes ||
      !stampValid(input.header.stamp) ||
      input.header != command_envelope.header ||
      command_envelope.schema_version != 1U ||
      command_envelope.producer_instance_id == 0U ||
      command_envelope.command_sequence == 0U ||
      command_envelope.plan_generation == 0U ||
      command_envelope.command.stamp != command_envelope.header.stamp ||
      !stampValid(input.control_pose_stamp) ||
      input.control_pose_stamp != command_envelope.header.stamp ||
      !poseFiniteAndNormalized(input.control_pose)) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_KEY_MISMATCH,
         "controller_sample_key_or_pose_invalid");
  }
  if (typed_plan != nullptr &&
      (typed_plan->plan_generation != command_envelope.plan_generation ||
       typed_plan->header.frame_id !=
           input.applied_trajectory.header.frame_id ||
       typed_plan->header != typed_plan->trajectory.header)) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_KEY_MISMATCH,
         "plan_controller_binding_mismatch");
  }

  const std::size_t applied_original_point_count =
      input.applied_original_point_count == 0U
          ? input.applied_trajectory.points.size()
          : input.applied_original_point_count;
  if (input.required_horizon_end_trajectory_index >=
      applied_original_point_count) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_HORIZON_INSUFFICIENT,
         "required_horizon_unavailable");
  } else if (!base.bounded || !applied.bounded) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_LIMIT_EXCEEDED,
         "geometry_point_limit_exceeded");
  } else if (!base.valid || !applied.valid ||
             input.nearest_trajectory_index >= applied_original_point_count ||
             !std::isfinite(input.trajectory_progress_m) ||
             input.trajectory_progress_m < 0.0) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_INVALID,
         "geometry_invalid");
  } else if (input.base_trajectory.header.frame_id !=
                 input.applied_trajectory.header.frame_id ||
             input.applied_trajectory.header.frame_id !=
                 input.header.frame_id) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_FRAME_OR_MANIFEST_MISMATCH,
         "geometry_frame_mismatch");
  } else if (!std::isfinite(input.available_spatial_horizon_m) ||
             !std::isfinite(input.required_spatial_horizon_m) ||
             input.available_spatial_horizon_m <
                 input.required_spatial_horizon_m) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_HORIZON_INSUFFICIENT,
         "spatial_horizon_insufficient");
  }

  if (!digestPresent(input.controller_adapter_implementation_sha256) ||
      !digestPresent(input.controller_adapter_config_sha256)) {
    fail(ControllerAppliedEnvelope::EVIDENCE_MISSING_INPUT,
         "controller_adapter_digest_missing");
  } else if (input.geometry_relation ==
             ControllerAppliedEnvelope::
                 GEOMETRY_RELATION_DERIVED_REFERENCE_OVERRIDE) {
    if (!input.source_wire.complete || input.source_wire.bytes.empty() ||
        input.source_wire.bytes.size() > kAw2MaxSourceBytes ||
        input.source_wire.original_size_bytes !=
            input.source_wire.bytes.size() ||
        !digestPresent(input.source_wire.sha256) ||
        input.source_wire.sha256 != aw2Sha256(input.source_wire.bytes) ||
        input.source_generation != command_envelope.plan_generation) {
      schema_valid = false;
      fail(ControllerAppliedEnvelope::EVIDENCE_PAYLOAD_DIGEST_MISMATCH,
           "derived_source_binding_invalid");
    }
  } else if (input.geometry_relation ==
             ControllerAppliedEnvelope::GEOMETRY_RELATION_DIRECT_APPLIED) {
    if (base.geometry.geometry_sha256 != applied.geometry.geometry_sha256) {
      fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_RELATION_UNPROVEN,
           "direct_geometry_digest_mismatch");
    }
  } else {
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_RELATION_UNPROVEN,
         "controller_cannot_prove_geometry_relation");
  }

  if (!commandValid(input.raw_controller_command) ||
      !commandValid(command_envelope.command) ||
      input.raw_controller_command.stamp != command_envelope.header.stamp ||
      input.raw_controller_command.lateral.stamp !=
          command_envelope.header.stamp ||
      input.raw_controller_command.longitudinal.stamp !=
          command_envelope.header.stamp ||
      command_envelope.command.lateral.stamp != command_envelope.header.stamp ||
      command_envelope.command.longitudinal.stamp !=
          command_envelope.header.stamp ||
      !std::isfinite(input.raw_steering_tire_angle_rad) ||
      !std::isfinite(input.output_steering_tire_angle_rad) ||
      !std::isfinite(input.raw_steering_tire_rotation_rate_radps) ||
      !std::isfinite(input.output_steering_tire_rotation_rate_radps) ||
      input.raw_controller_command.lateral.steering_tire_angle !=
          static_cast<float>(input.raw_steering_tire_angle_rad) ||
      command_envelope.command.lateral.steering_tire_angle !=
          static_cast<float>(input.output_steering_tire_angle_rad) ||
      input.raw_controller_command.lateral.steering_tire_rotation_rate !=
          static_cast<float>(input.raw_steering_tire_rotation_rate_radps) ||
      command_envelope.command.lateral.steering_tire_rotation_rate !=
          static_cast<float>(input.output_steering_tire_rotation_rate_radps)) {
    schema_valid = false;
    fail(ControllerAppliedEnvelope::EVIDENCE_PAYLOAD_DIGEST_MISMATCH,
         "controller_command_binding_invalid");
  }

  if (!input.hard_actuator_limits_present) {
    fail(ControllerAppliedEnvelope::EVIDENCE_ACTUATOR_LIMIT_UNKNOWN,
         "hard_actuator_limits_unavailable");
  } else if (!std::isfinite(input.hard_steering_tire_angle_limit_rad) ||
             !std::isfinite(
                 input.hard_steering_tire_rotation_rate_limit_radps) ||
             input.hard_steering_tire_angle_limit_rad <= 0.0 ||
             input.hard_steering_tire_rotation_rate_limit_radps <= 0.0 ||
             std::abs(input.raw_steering_tire_angle_rad) >
                 input.hard_steering_tire_angle_limit_rad ||
             std::abs(input.output_steering_tire_angle_rad) >
                 input.hard_steering_tire_angle_limit_rad ||
             std::abs(input.raw_steering_tire_rotation_rate_radps) >
                 input.hard_steering_tire_rotation_rate_limit_radps ||
             std::abs(input.output_steering_tire_rotation_rate_radps) >
                 input.hard_steering_tire_rotation_rate_limit_radps) {
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_INVALID,
         "controller_command_hard_limit_exceeded");
  }

  if (input.rollout_samples.size() > kAw2MaxRolloutSamples) {
    schema_valid = false;
    envelope.rollout_state = ControllerAppliedEnvelope::ROLLOUT_LIMIT_EXCEEDED;
    fail(ControllerAppliedEnvelope::EVIDENCE_RESOURCE_LIMIT_EXCEEDED,
         "rollout_sample_limit_exceeded");
  } else if (input.rollout_state ==
                 ControllerAppliedEnvelope::ROLLOUT_UNAVAILABLE ||
             input.rollout_samples.empty()) {
    fail(ControllerAppliedEnvelope::EVIDENCE_ROLLOUT_UNAVAILABLE,
         "reachable_rollout_unavailable");
  } else if (!rolloutValid(envelope)) {
    envelope.rollout_state = ControllerAppliedEnvelope::ROLLOUT_INVALID;
    fail(ControllerAppliedEnvelope::EVIDENCE_GEOMETRY_INVALID,
         "rollout_invalid_or_insufficient");
  }

  envelope.schema_version = schema_valid
                                ? ControllerAppliedEnvelope::SCHEMA_V1
                                : ControllerAppliedEnvelope::SCHEMA_INVALID;
  if (envelope.schema_version == ControllerAppliedEnvelope::SCHEMA_V1) {
    auto payload_bytes = canonicalEnvelope(envelope, false);
    if (payload_bytes.size() > kAw2MaxRecordBytes) {
      envelope.schema_version = ControllerAppliedEnvelope::SCHEMA_INVALID;
      fail(ControllerAppliedEnvelope::EVIDENCE_RESOURCE_LIMIT_EXCEEDED,
           "canonical_record_limit_exceeded");
    } else if (envelope.evidence_state ==
                   ControllerAppliedEnvelope::EVIDENCE_COMPLETE &&
               binding_tracker != nullptr) {
      const auto observation = binding_tracker->observe(
          envelope.controller_sample_key, envelope.candidate_revision,
          envelope.candidate_content_sha256, aw2Sha256(payload_bytes));
      if (observation == ControllerAppliedBindingTracker::Observation::
                             SAME_GENERATION_PAYLOAD_MUTATION) {
        fail(ControllerAppliedEnvelope::
                 EVIDENCE_SAME_GENERATION_PAYLOAD_MUTATION,
             "same_generation_payload_mutation");
      } else if (observation == ControllerAppliedBindingTracker::Observation::
                                    DUPLICATE_CONFLICT ||
                 observation == ControllerAppliedBindingTracker::Observation::
                                    SEQUENCE_REGRESSION) {
        fail(ControllerAppliedEnvelope::EVIDENCE_DUPLICATE_CONFLICT,
             observation == ControllerAppliedBindingTracker::Observation::
                                SEQUENCE_REGRESSION
                 ? "controller_sequence_regression"
                 : "controller_sample_duplicate_conflict");
      }
    }
    if (envelope.schema_version == ControllerAppliedEnvelope::SCHEMA_V1) {
      const auto record_bytes = canonicalEnvelope(envelope, true);
      if (record_bytes.size() <= kAw2MaxRecordBytes) {
        envelope.controller_applied_envelope_sha256 = aw2Sha256(record_bytes);
      } else {
        envelope.schema_version = ControllerAppliedEnvelope::SCHEMA_INVALID;
        fail(ControllerAppliedEnvelope::EVIDENCE_RESOURCE_LIMIT_EXCEEDED,
             "canonical_record_limit_exceeded");
      }
    }
  }
  // AW2-2 is shadow-only for every schema/evidence state.
  envelope.authority_eligible = false;
  return envelope;
}

} // namespace simple_pure_pursuit
