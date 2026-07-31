#include "simple_pure_pursuit/ay0_base_capture.hpp"

#include <algorithm>
#include <cstring>

namespace simple_pure_pursuit {
namespace {

overtake_transport_contract::c002ay0::FixedPoint toFixedPoint(
    const autoware_auto_planning_msgs::msg::TrajectoryPoint &source) noexcept {
  overtake_transport_contract::c002ay0::FixedPoint point{};
  point.time_from_start.sec = source.time_from_start.sec;
  point.time_from_start.nanosec = source.time_from_start.nanosec;
  point.position_x_m = source.pose.position.x;
  point.position_y_m = source.pose.position.y;
  point.position_z_m = source.pose.position.z;
  point.orientation_x = source.pose.orientation.x;
  point.orientation_y = source.pose.orientation.y;
  point.orientation_z = source.pose.orientation.z;
  point.orientation_w = source.pose.orientation.w;
  point.longitudinal_velocity_mps = source.longitudinal_velocity_mps;
  point.lateral_velocity_mps = source.lateral_velocity_mps;
  point.acceleration_mps2 = source.acceleration_mps2;
  point.heading_rate_rps = source.heading_rate_rps;
  point.front_wheel_angle_rad = source.front_wheel_angle_rad;
  point.rear_wheel_angle_rad = source.rear_wheel_angle_rad;
  return point;
}

} // namespace

Ay0BaseCaptureResult buildAy0FixedBaseRecord(
    const Ay0BaseCaptureInput &input,
    overtake_transport_contract::c002ay0::FixedBaseRecord &record) noexcept {
  using overtake_transport_contract::c002ay0::kFixedFrameCapacity;
  using overtake_transport_contract::c002ay0::kMaxCartesianPoints;

  record = {};
  if (input.trajectory == nullptr) {
    return Ay0BaseCaptureResult::kMissingTrajectory;
  }
  const auto &trajectory = *input.trajectory;
  if (trajectory.header.frame_id.empty() ||
      trajectory.header.frame_id.size() > kFixedFrameCapacity) {
    return Ay0BaseCaptureResult::kFrameInvalid;
  }
  if (trajectory.points.size() < 2U ||
      trajectory.points.size() > kMaxCartesianPoints) {
    return Ay0BaseCaptureResult::kPointLimit;
  }
  if (input.nearest_source_index >= trajectory.points.size()) {
    return Ay0BaseCaptureResult::kNearestInvalid;
  }
  if (input.session_generation == 0U || input.session_nonce == 0U ||
      input.controller_instance_id == 0U || input.controller_sequence == 0U ||
      input.base_lease_id == 0U || input.source_generation == 0U ||
      input.source_kind == 0U) {
    return Ay0BaseCaptureResult::kIdentityInvalid;
  }

  record.session_generation = input.session_generation;
  record.session_nonce = input.session_nonce;
  record.record_stamp = input.record_stamp;
  record.frame_size =
      static_cast<std::uint16_t>(trajectory.header.frame_id.size());
  std::memcpy(record.frame.data(), trajectory.header.frame_id.data(),
              trajectory.header.frame_id.size());
  record.race_arm_epoch = input.race_arm_epoch;
  record.controller_instance_id = input.controller_instance_id;
  record.controller_sequence = input.controller_sequence;
  record.base_lease_id = input.base_lease_id;
  record.lease_valid_until = input.lease_valid_until;
  record.base_source_kind = input.source_kind;
  record.base_source_stamp.sec = trajectory.header.stamp.sec;
  record.base_source_stamp.nanosec = trajectory.header.stamp.nanosec;
  record.base_source_generation = input.source_generation;
  record.base_original_point_count =
      static_cast<std::uint32_t>(trajectory.points.size());
  record.nearest_source_index =
      static_cast<std::uint32_t>(input.nearest_source_index);
  record.point_count = static_cast<std::uint32_t>(trajectory.points.size());
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    record.points[index] = toFixedPoint(trajectory.points[index]);
  }
  record.controller_implementation_sha256 =
      input.controller_implementation_sha256;
  record.controller_config_sha256 = input.controller_config_sha256;
  return Ay0BaseCaptureResult::kBuilt;
}

} // namespace simple_pure_pursuit
