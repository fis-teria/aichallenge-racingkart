#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

namespace overtake_transport_contract::c002ay0 {
namespace {

constexpr double kGeometryTolerance = 1.0e-12;
constexpr double kScalarTolerance = 1.0e-9;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kMaxCanonicalRecordBytes = 64U * 1024U;

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

void appendI8(std::vector<std::uint8_t> &bytes, std::int8_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value));
}

void appendBool(std::vector<std::uint8_t> &bytes, bool value) {
  appendU8(bytes, value ? 1U : 0U);
}

void appendU32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  for (std::uint32_t shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void appendI32(std::vector<std::uint8_t> &bytes, std::int32_t value) {
  appendU32(bytes, static_cast<std::uint32_t>(value));
}

void appendU64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
  for (std::uint32_t shift = 0U; shift < 64U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
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

void appendString(std::vector<std::uint8_t> &bytes, std::string_view value) {
  appendU32(bytes, static_cast<std::uint32_t>(value.size()));
  bytes.insert(bytes.end(), value.begin(), value.end());
}

template <typename TimeLike>
void appendTime(std::vector<std::uint8_t> &bytes, const TimeLike &time) {
  appendI32(bytes, time.sec);
  appendU32(bytes, time.nanosec);
}

void appendDigest(std::vector<std::uint8_t> &bytes, const Digest &digest) {
  bytes.insert(bytes.end(), digest.begin(), digest.end());
}

bool digestMissing(const Digest &digest) {
  return std::all_of(digest.begin(), digest.end(),
                     [](std::uint8_t value) { return value == 0U; });
}

bool validTime(std::int32_t sec, std::uint32_t nanosec) {
  return sec >= 0 && nanosec < 1000000000U;
}

template <typename TimeLike> bool validTime(const TimeLike &time) {
  return validTime(time.sec, time.nanosec);
}

template <typename LeftTime, typename RightTime>
bool timeLess(const LeftTime &left, const RightTime &right) {
  return left.sec < right.sec ||
         (left.sec == right.sec && left.nanosec < right.nanosec);
}

template <typename LeftTime, typename RightTime>
bool sameTime(const LeftTime &left, const RightTime &right) {
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

template <typename TimeLike>
std::uint64_t timeNanoseconds(const TimeLike &time) {
  return static_cast<std::uint64_t>(time.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(time.nanosec);
}

bool finitePose(const geometry_msgs::msg::Pose &pose) {
  const double norm = std::sqrt(pose.orientation.x * pose.orientation.x +
                                pose.orientation.y * pose.orientation.y +
                                pose.orientation.z * pose.orientation.z +
                                pose.orientation.w * pose.orientation.w);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && std::isfinite(norm) &&
         std::abs(norm - 1.0) <= 1.0e-6;
}

void appendPose(std::vector<std::uint8_t> &bytes,
                const geometry_msgs::msg::Pose &pose) {
  appendDouble(bytes, pose.position.x);
  appendDouble(bytes, pose.position.y);
  appendDouble(bytes, pose.position.z);
  appendDouble(bytes, pose.orientation.x);
  appendDouble(bytes, pose.orientation.y);
  appendDouble(bytes, pose.orientation.z);
  appendDouble(bytes, pose.orientation.w);
}

bool finitePoint(const CandidateExecutionPoint &point) {
  const double norm = std::sqrt(point.orientation_x * point.orientation_x +
                                point.orientation_y * point.orientation_y +
                                point.orientation_z * point.orientation_z +
                                point.orientation_w * point.orientation_w);
  return validTime(point.time_from_start) &&
         std::isfinite(point.position_x_m) &&
         std::isfinite(point.position_y_m) &&
         std::isfinite(point.position_z_m) &&
         std::isfinite(point.orientation_x) &&
         std::isfinite(point.orientation_y) &&
         std::isfinite(point.orientation_z) &&
         std::isfinite(point.orientation_w) && std::isfinite(norm) &&
         std::abs(norm - 1.0) <= 1.0e-6 &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         point.longitudinal_velocity_mps >= 0.0F &&
         point.longitudinal_velocity_mps <=
             static_cast<float>(kExecutionSpeedCeilingMps) &&
         std::isfinite(point.lateral_velocity_mps) &&
         std::isfinite(point.acceleration_mps2) &&
         std::isfinite(point.heading_rate_rps) &&
         std::isfinite(point.front_wheel_angle_rad) &&
         std::isfinite(point.rear_wheel_angle_rad);
}

double pointYaw(const CandidateExecutionPoint &point) {
  const double sin_yaw = 2.0 * (point.orientation_w * point.orientation_z +
                                point.orientation_x * point.orientation_y);
  const double cos_yaw =
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z);
  return std::atan2(sin_yaw, cos_yaw);
}

void appendPoint(std::vector<std::uint8_t> &bytes,
                 const CandidateExecutionPoint &point) {
  appendTime(bytes, point.time_from_start);
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

void appendPlanKey(std::vector<std::uint8_t> &bytes,
                   const multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &key) {
  appendU64(bytes, key.race_arm_epoch);
  appendU64(bytes, key.planner_instance_id);
  appendU64(bytes, key.attempt_id);
  appendString(bytes, key.target_vehicle_id);
  appendI8(bytes, key.pass_direction);
  appendU64(bytes, key.connector_transaction_id);
  appendTime(bytes, key.plan_stamp);
  appendU32(bytes, key.plan_generation);
}

bool samePlanKey(const multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &left,
                 const multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &right) {
  return left.race_arm_epoch == right.race_arm_epoch &&
         left.planner_instance_id == right.planner_instance_id &&
         left.attempt_id == right.attempt_id &&
         left.target_vehicle_id == right.target_vehicle_id &&
         left.pass_direction == right.pass_direction &&
         left.connector_transaction_id == right.connector_transaction_id &&
         sameTime(left.plan_stamp, right.plan_stamp) &&
         left.plan_generation == right.plan_generation;
}

bool positiveZero(double value) { return value == 0.0 && !std::signbit(value); }

bool zeroTime(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

bool absentPose(const geometry_msgs::msg::Pose &pose) {
  return positiveZero(pose.position.x) && positiveZero(pose.position.y) &&
         positiveZero(pose.position.z) && positiveZero(pose.orientation.x) &&
         positiveZero(pose.orientation.y) && positiveZero(pose.orientation.z) &&
         positiveZero(pose.orientation.w);
}

bool applicationEvidenceAbsent(
    const CartesianTrajectoryApplicationStatus &status) {
  return digestMissing(status.applied_geometry_sha256) &&
         status.nearest_index == 0U && status.lookahead_index == 0U &&
         !status.endpoint_fallback &&
         positiveZero(status.available_spatial_horizon_m) &&
         positiveZero(status.required_spatial_horizon_m) &&
         positiveZero(status.handoff_position_error_m) &&
         positiveZero(status.handoff_yaw_error_rad) &&
         absentPose(status.applied_control_pose) &&
         zeroTime(status.applied_control_pose_stamp) &&
         digestMissing(status.applied_control_pose_sha256) &&
         zeroTime(status.controller_command_stamp) &&
         status.command_sequence == 0U;
}

ValidationError
validatePlanKey(const multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &key) {
  if (key.race_arm_epoch == 0U || key.planner_instance_id == 0U ||
      key.attempt_id == 0U || key.connector_transaction_id == 0U ||
      key.plan_generation == 0U || key.target_vehicle_id.empty() ||
      key.target_vehicle_id.size() > 64U) {
    return ValidationError::IDENTITY_INVALID;
  }
  if (key.pass_direction != -1 && key.pass_direction != 1) {
    return ValidationError::ENUM_INVALID;
  }
  if (!validTime(key.plan_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  return ValidationError::NONE;
}

bool validSourceKind(std::uint8_t source_kind) {
  return source_kind == ControllerBaseTrajectorySnapshot::SOURCE_MPC_HORIZON ||
         source_kind ==
             ControllerBaseTrajectorySnapshot::SOURCE_REFERENCE_TRAJECTORY;
}

template <typename Points>
ValidationError
validateSourceWindow(std::uint32_t original_count, std::uint32_t first_index,
                     std::uint32_t last_index, std::uint32_t nearest_index,
                     const Points &points) {
  if (original_count < 2U || points.size() < 2U || last_index < first_index ||
      last_index >= original_count || nearest_index < first_index ||
      nearest_index > last_index ||
      static_cast<std::uint64_t>(last_index) -
              static_cast<std::uint64_t>(first_index) + 1ULL !=
          points.size()) {
    return ValidationError::SOURCE_WINDOW_INVALID;
  }
  return ValidationError::NONE;
}

ValidationError validateSourceWindowIdentity(std::uint32_t original_count,
                                             std::uint32_t first_index,
                                             std::uint32_t last_index,
                                             std::uint32_t nearest_index) {
  if (original_count < 2U || last_index < first_index ||
      last_index >= original_count || nearest_index < first_index ||
      nearest_index > last_index) {
    return ValidationError::SOURCE_WINDOW_INVALID;
  }
  return ValidationError::NONE;
}

template <typename Points>
CanonicalRecord canonicalizeGeometry(const Points &points,
                                     std::string_view domain) {
  CanonicalRecord result;
  if (domain.empty() || domain.size() > 128U) {
    result.error = ValidationError::IDENTITY_INVALID;
    return result;
  }
  if (points.size() < 2U || points.size() > kMaxCartesianPoints) {
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }

  double total_arc_length_m = 0.0;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (!finitePoint(points[index])) {
      result.error = ValidationError::GEOMETRY_POINT_INVALID;
      return result;
    }
    if (index == 0U) {
      continue;
    }
    if (timeNanoseconds(points[index].time_from_start) <=
        timeNanoseconds(points[index - 1U].time_from_start)) {
      result.error = ValidationError::GEOMETRY_TIME_NONMONOTONIC;
      return result;
    }
    const double dx =
        points[index].position_x_m - points[index - 1U].position_x_m;
    const double dy =
        points[index].position_y_m - points[index - 1U].position_y_m;
    const double dz =
        points[index].position_z_m - points[index - 1U].position_z_m;
    const double spacing_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(spacing_m) || spacing_m <= kGeometryTolerance) {
      result.error = ValidationError::GEOMETRY_ARC_NONMONOTONIC;
      return result;
    }
    if (spacing_m > kMaxTransportArcSpacingM) {
      result.error = ValidationError::GEOMETRY_ARC_SPACING_EXCEEDED;
      return result;
    }
    const double yaw_step_rad = std::abs(std::remainder(
        pointYaw(points[index]) - pointYaw(points[index - 1U]), kTwoPi));
    if (!std::isfinite(yaw_step_rad) ||
        yaw_step_rad > kMaxTransportYawStepRad) {
      result.error = ValidationError::GEOMETRY_YAW_STEP_EXCEEDED;
      return result;
    }
    total_arc_length_m += spacing_m;
  }
  if (total_arc_length_m > kMaxTransportArcLengthM) {
    result.error = ValidationError::GEOMETRY_ARC_LENGTH_MISMATCH;
    return result;
  }

  appendString(result.bytes, domain);
  appendU32(result.bytes, 1U);
  appendU32(result.bytes, static_cast<std::uint32_t>(points.size()));
  for (const auto &point : points) {
    appendPoint(result.bytes, point);
  }
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.geometry_sha256 = result.sha256;
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord
canonicalizeBaseGeometry(const ControllerBaseTrajectorySnapshot &snapshot) {
  auto result = canonicalizeGeometry(snapshot.base_points,
                                     "C002AY0_BASE_GEOMETRY_POINTS_V1");
  if (!result.valid()) {
    return result;
  }
  std::vector<std::uint8_t> bytes;
  appendString(bytes, "C002AY0_BASE_GEOMETRY_V1");
  appendU32(bytes, 1U);
  appendU32(bytes, snapshot.base_original_point_count);
  appendU32(bytes, snapshot.first_source_index);
  appendU32(bytes, snapshot.last_source_index);
  appendU32(bytes, snapshot.nearest_source_index);
  appendU32(bytes, static_cast<std::uint32_t>(snapshot.base_points.size()));
  for (const auto &point : snapshot.base_points) {
    appendPoint(bytes, point);
  }
  result.bytes = std::move(bytes);
  result.sha256 = sha256(result.bytes);
  result.geometry_sha256 = result.sha256;
  return result;
}

ValidationError
validateSnapshotContent(const ControllerBaseTrajectorySnapshot &snapshot) {
  if (snapshot.schema_version !=
      ControllerBaseTrajectorySnapshot::SCHEMA_V1_SHADOW) {
    return ValidationError::SCHEMA_UNSUPPORTED;
  }
  if (snapshot.authority_eligible) {
    return ValidationError::AUTHORITY_FLAG_INVALID;
  }
  if (snapshot.frame_id != "map") {
    return ValidationError::FRAME_INVALID;
  }
  if (snapshot.race_arm_epoch == 0U || snapshot.controller_instance_id == 0U ||
      snapshot.controller_sequence == 0U || snapshot.base_lease_id == 0U ||
      snapshot.base_source_generation == 0U) {
    return ValidationError::IDENTITY_INVALID;
  }
  if (!validTime(snapshot.record_stamp) ||
      !validTime(snapshot.lease_valid_until) ||
      !validTime(snapshot.base_source_stamp) ||
      timeLess(snapshot.record_stamp, snapshot.base_source_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  if (!timeLess(snapshot.record_stamp, snapshot.lease_valid_until)) {
    return ValidationError::LEASE_INVALID;
  }
  if (!validSourceKind(snapshot.base_source_kind)) {
    return ValidationError::ENUM_INVALID;
  }
  const auto source_window = validateSourceWindow(
      snapshot.base_original_point_count, snapshot.first_source_index,
      snapshot.last_source_index, snapshot.nearest_source_index,
      snapshot.base_points);
  if (source_window != ValidationError::NONE) {
    return source_window;
  }
  const auto captured_count =
      static_cast<std::uint32_t>(snapshot.base_points.size());
  if (snapshot.canonical_algorithm_version == 1U) {
    if (snapshot.base_original_point_count != captured_count ||
        snapshot.first_source_index != 0U ||
        snapshot.last_source_index != captured_count - 1U) {
      return ValidationError::SOURCE_WINDOW_INVALID;
    }
  } else if (snapshot.canonical_algorithm_version == 2U) {
    if (snapshot.base_original_point_count <= kMaxCartesianPoints ||
        captured_count != kMaxCartesianPoints) {
      return ValidationError::SOURCE_WINDOW_INVALID;
    }
    const auto expected_first =
        std::min(snapshot.nearest_source_index,
                 snapshot.base_original_point_count - captured_count);
    if (snapshot.first_source_index != expected_first) {
      return ValidationError::SOURCE_WINDOW_INVALID;
    }
  }
  if (snapshot.base_source_digest_state !=
          ControllerBaseTrajectorySnapshot::BASE_SOURCE_DIGEST_COMPLETE ||
      (snapshot.canonical_algorithm_version != 1U &&
       snapshot.canonical_algorithm_version != 2U) ||
      digestMissing(snapshot.base_source_sha256) ||
      digestMissing(snapshot.controller_implementation_sha256) ||
      digestMissing(snapshot.controller_config_sha256)) {
    return ValidationError::DIGEST_MISSING;
  }
  if (snapshot.canonical_algorithm_version == 2U) {
    const auto source = canonicalizeBaseSourceWindowV2(
        snapshot.base_source_kind, snapshot.frame_id,
        snapshot.base_source_stamp, snapshot.base_source_generation,
        snapshot.base_original_point_count, snapshot.first_source_index,
        snapshot.last_source_index, snapshot.nearest_source_index,
        snapshot.base_points);
    if (!source.valid()) {
      return source.error;
    }
    if (snapshot.base_source_sha256 != source.sha256) {
      return ValidationError::DIGEST_MISMATCH;
    }
  }
  return ValidationError::NONE;
}

ValidationError
validateTrajectoryContent(const AuthorizedCartesianTrajectory &trajectory) {
  if (trajectory.schema_version !=
      AuthorizedCartesianTrajectory::SCHEMA_V1_SHADOW) {
    return ValidationError::SCHEMA_UNSUPPORTED;
  }
  if (trajectory.authority_eligible) {
    return ValidationError::AUTHORITY_FLAG_INVALID;
  }
  if (trajectory.frame_id != "map") {
    return ValidationError::FRAME_INVALID;
  }
  const auto plan_key = validatePlanKey(trajectory.plan_sample_key);
  if (plan_key != ValidationError::NONE) {
    return plan_key;
  }
  if (!sameTime(trajectory.plan_stamp, trajectory.plan_sample_key.plan_stamp) ||
      !validTime(trajectory.base_lease_valid_until) ||
      !validTime(trajectory.base_source_stamp) ||
      !validTime(trajectory.safety_evaluation_stamp) ||
      !validTime(trajectory.safety_valid_until) ||
      !validTime(trajectory.candidate_start_control_pose_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  if (trajectory.candidate_revision == 0U || trajectory.authority_token == 0U ||
      trajectory.source_controller_instance_id == 0U ||
      trajectory.source_controller_sequence == 0U ||
      trajectory.base_lease_id == 0U ||
      trajectory.base_source_generation == 0U ||
      trajectory.safety_snapshot_id == 0U) {
    return ValidationError::IDENTITY_INVALID;
  }
  if (!validSourceKind(trajectory.base_source_kind) ||
      trajectory.candidate_type >
          AuthorizedCartesianTrajectory::CANDIDATE_SAFE_STOP ||
      trajectory.phase > AuthorizedCartesianTrajectory::PHASE_ABORT_HOLD ||
      trajectory.authorization_state !=
          AuthorizedCartesianTrajectory::AUTHORIZATION_AUTHORIZED ||
      trajectory.safety_evaluation_result !=
          AuthorizedCartesianTrajectory::SAFETY_PASSED) {
    return ValidationError::ENUM_INVALID;
  }
  if (!timeLess(trajectory.plan_stamp, trajectory.base_lease_valid_until) ||
      !timeLess(trajectory.safety_evaluation_stamp,
                trajectory.safety_valid_until) ||
      timeLess(trajectory.plan_stamp, trajectory.safety_evaluation_stamp)) {
    return ValidationError::LEASE_INVALID;
  }
  if (!sameTime(trajectory.candidate_start_control_pose_stamp,
                trajectory.plan_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  const auto source_window = validateSourceWindowIdentity(
      trajectory.base_original_point_count, trajectory.base_first_source_index,
      trajectory.base_last_source_index, trajectory.base_nearest_source_index);
  if (source_window != ValidationError::NONE) {
    return source_window;
  }
  if (trajectory.base_source_digest_state !=
          AuthorizedCartesianTrajectory::BASE_SOURCE_DIGEST_COMPLETE ||
      (trajectory.canonical_algorithm_version != 1U &&
       trajectory.canonical_algorithm_version != 2U) ||
      digestMissing(trajectory.base_geometry_sha256) ||
      digestMissing(trajectory.base_source_sha256) ||
      digestMissing(trajectory.base_snapshot_sha256) ||
      digestMissing(trajectory.world_safety_snapshot_sha256) ||
      digestMissing(trajectory.safety_evaluator_implementation_sha256) ||
      digestMissing(trajectory.safety_evaluator_config_sha256) ||
      digestMissing(trajectory.controller_implementation_sha256) ||
      digestMissing(trajectory.controller_config_sha256)) {
    return ValidationError::DIGEST_MISSING;
  }
  if (!finitePose(trajectory.candidate_start_control_pose)) {
    return ValidationError::GEOMETRY_POINT_INVALID;
  }
  if (!std::isfinite(trajectory.total_arc_length_m) ||
      !std::isfinite(trajectory.required_spatial_horizon_m) ||
      !std::isfinite(trajectory.join_end_arc_length_m) ||
      !std::isfinite(trajectory.post_join_arc_length_m) ||
      trajectory.total_arc_length_m <= 0.0 ||
      trajectory.total_arc_length_m > kMaxTransportArcLengthM ||
      trajectory.required_spatial_horizon_m <= 0.0 ||
      trajectory.required_spatial_horizon_m > trajectory.total_arc_length_m ||
      trajectory.join_end_arc_length_m < 0.0 ||
      trajectory.post_join_arc_length_m < 0.0 ||
      trajectory.join_end_arc_length_m + trajectory.post_join_arc_length_m >
          trajectory.total_arc_length_m ||
      trajectory.original_candidate_point_count != trajectory.points.size()) {
    return ValidationError::HORIZON_INVALID;
  }
  return ValidationError::NONE;
}

ValidationError
validateStatusContent(const CartesianTrajectoryApplicationStatus &status) {
  if (status.schema_version !=
      CartesianTrajectoryApplicationStatus::SCHEMA_V1_SHADOW) {
    return ValidationError::SCHEMA_UNSUPPORTED;
  }
  if (status.authority_eligible) {
    return ValidationError::AUTHORITY_FLAG_INVALID;
  }
  if (status.frame_id != "map") {
    return ValidationError::FRAME_INVALID;
  }
  if (!validTime(status.status_stamp) || !validTime(status.base_source_stamp) ||
      !validTime(status.applied_control_pose_stamp) ||
      !validTime(status.controller_command_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  const auto plan_key = validatePlanKey(status.plan_sample_key);
  if (plan_key != ValidationError::NONE) {
    return plan_key;
  }
  if (status.candidate_revision == 0U || status.authority_token == 0U ||
      status.controller_instance_id == 0U || status.controller_sequence == 0U ||
      status.base_lease_id == 0U || status.base_source_generation == 0U) {
    return ValidationError::IDENTITY_INVALID;
  }
  if (!validSourceKind(status.base_source_kind) ||
      status.application_state >
          CartesianTrajectoryApplicationStatus::APPLICATION_INVALIDATED ||
      status.reject_reason > CartesianTrajectoryApplicationStatus::
                                 REJECT_EXISTING_AUTHORITY_INCOMPLETE) {
    return ValidationError::ENUM_INVALID;
  }
  // Schema V1 is shadow-only. It can prove that an immutable slot was warmed,
  // but it cannot assert that geometry affected a controller command.
  if (status.application_state ==
      CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  if (!std::isfinite(status.available_spatial_horizon_m) ||
      !std::isfinite(status.required_spatial_horizon_m) ||
      !std::isfinite(status.handoff_position_error_m) ||
      !std::isfinite(status.handoff_yaw_error_rad)) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  if (digestMissing(status.expected_base_source_sha256) ||
      digestMissing(status.expected_base_geometry_sha256) ||
      digestMissing(status.expected_base_snapshot_sha256) ||
      digestMissing(status.expected_geometry_sha256) ||
      digestMissing(status.expected_payload_sha256) ||
      digestMissing(status.expected_controller_implementation_sha256) ||
      digestMissing(status.expected_controller_config_sha256)) {
    return ValidationError::DIGEST_MISSING;
  }
  const bool slot_ready =
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_WARMED ||
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  const bool applied =
      status.application_state ==
      CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  if (slot_ready &&
      (status.reject_reason !=
           CartesianTrajectoryApplicationStatus::REJECT_NONE ||
       digestMissing(status.observed_base_source_sha256) ||
       digestMissing(status.observed_base_geometry_sha256) ||
       digestMissing(status.observed_base_snapshot_sha256) ||
       digestMissing(status.observed_payload_sha256) ||
       digestMissing(status.observed_controller_implementation_sha256) ||
       digestMissing(status.observed_controller_config_sha256))) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  if (applied &&
      (status.endpoint_fallback || status.command_sequence == 0U ||
       !finitePose(status.applied_control_pose) ||
       digestMissing(status.applied_geometry_sha256) ||
       status.lookahead_index < status.nearest_index ||
       status.required_spatial_horizon_m <= 0.0 ||
       status.available_spatial_horizon_m < status.required_spatial_horizon_m ||
       timeLess(status.applied_control_pose_stamp,
                status.plan_sample_key.plan_stamp) ||
       timeLess(status.controller_command_stamp,
                status.applied_control_pose_stamp) ||
       timeLess(status.status_stamp, status.controller_command_stamp))) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  if (!applied && !applicationEvidenceAbsent(status)) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  const bool requires_reject_reason =
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_REJECTED ||
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_INVALIDATED;
  if (requires_reject_reason &&
      status.reject_reason ==
          CartesianTrajectoryApplicationStatus::REJECT_NONE) {
    return ValidationError::APPLICATION_STATUS_INVALID;
  }
  if (timeLess(status.status_stamp, status.plan_sample_key.plan_stamp) ||
      timeLess(status.status_stamp, status.base_source_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  return ValidationError::NONE;
}

double geometryArcLength(const BoundedCandidateExecutionPoints &points) {
  double arc_length_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double dx =
        points[index].position_x_m - points[index - 1U].position_x_m;
    const double dy =
        points[index].position_y_m - points[index - 1U].position_y_m;
    const double dz =
        points[index].position_z_m - points[index - 1U].position_z_m;
    arc_length_m += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return arc_length_m;
}

} // namespace

PointInvalidField
classifyPointInvalidFieldV1(const CandidateExecutionPoint &point) noexcept {
  if (!validTime(point.time_from_start))
    return PointInvalidField::TIME_FROM_START;
  if (!std::isfinite(point.position_x_m))
    return PointInvalidField::POSITION_X;
  if (!std::isfinite(point.position_y_m))
    return PointInvalidField::POSITION_Y;
  if (!std::isfinite(point.position_z_m))
    return PointInvalidField::POSITION_Z;
  if (!std::isfinite(point.orientation_x))
    return PointInvalidField::ORIENTATION_X;
  if (!std::isfinite(point.orientation_y))
    return PointInvalidField::ORIENTATION_Y;
  if (!std::isfinite(point.orientation_z))
    return PointInvalidField::ORIENTATION_Z;
  if (!std::isfinite(point.orientation_w))
    return PointInvalidField::ORIENTATION_W;
  const double norm = std::sqrt(point.orientation_x * point.orientation_x +
                                point.orientation_y * point.orientation_y +
                                point.orientation_z * point.orientation_z +
                                point.orientation_w * point.orientation_w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-6) {
    return PointInvalidField::ORIENTATION_NORM;
  }
  if (!std::isfinite(point.longitudinal_velocity_mps)) {
    return PointInvalidField::LONGITUDINAL_VELOCITY_NONFINITE;
  }
  if (point.longitudinal_velocity_mps < 0.0F ||
      point.longitudinal_velocity_mps >
          static_cast<float>(kExecutionSpeedCeilingMps)) {
    return PointInvalidField::LONGITUDINAL_VELOCITY_RANGE;
  }
  if (!std::isfinite(point.lateral_velocity_mps))
    return PointInvalidField::LATERAL_VELOCITY;
  if (!std::isfinite(point.acceleration_mps2))
    return PointInvalidField::ACCELERATION;
  if (!std::isfinite(point.heading_rate_rps))
    return PointInvalidField::HEADING_RATE;
  if (!std::isfinite(point.front_wheel_angle_rad))
    return PointInvalidField::FRONT_WHEEL_ANGLE;
  if (!std::isfinite(point.rear_wheel_angle_rad))
    return PointInvalidField::REAR_WHEEL_ANGLE;
  return PointInvalidField::NONE;
}

Digest sha256(const std::vector<std::uint8_t> &bytes) {
  std::vector<std::uint8_t> padded = bytes;
  const std::uint64_t bit_count =
      static_cast<std::uint64_t>(padded.size()) * 8U;
  padded.push_back(0x80U);
  while ((padded.size() % 64U) != 56U) {
    padded.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    padded.push_back(static_cast<std::uint8_t>((bit_count >> shift) & 0xffU));
  }

  std::array<std::uint32_t, 8U> state{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                      0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                      0x1f83d9abU, 0x5be0cd19U};
  for (std::size_t chunk = 0U; chunk < padded.size(); chunk += 64U) {
    std::array<std::uint32_t, 64U> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t offset = chunk + index * 4U;
      words[index] = (static_cast<std::uint32_t>(padded[offset]) << 24U) |
                     (static_cast<std::uint32_t>(padded[offset + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(padded[offset + 2U]) << 8U) |
                     static_cast<std::uint32_t>(padded[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
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
    for (std::size_t index = 0U; index < words.size(); ++index) {
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

  Digest digest{};
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

CanonicalRecord canonicalizeFreeRunPlanV1(const OvertakePlan &plan) {
  CanonicalRecord result;
  result.error = ValidationError::IDENTITY_INVALID;
  constexpr std::size_t kMaxReasonBytes = 256U;
  constexpr std::size_t kMaxReasonCount = 64U;
  const auto strings_bounded = [&]() {
    if (plan.header.frame_id.size() > 128U ||
        plan.decision_reason.size() > kMaxReasonBytes ||
        plan.candidate_reject_reason.size() > kMaxReasonBytes ||
        plan.constraint_reason.size() > kMaxReasonBytes ||
        plan.authorization_failure_reasons.size() > kMaxReasonCount) {
      return false;
    }
    return std::all_of(plan.authorization_failure_reasons.begin(),
                       plan.authorization_failure_reasons.end(),
                       [kMaxReasonBytes](const std::string &reason) {
                         return reason.size() <= kMaxReasonBytes;
                       });
  };
  if (plan.header.frame_id != "map" || !strings_bounded() ||
      plan.phase != OvertakePlan::FREE_RUN || plan.attempt_id != 0U ||
      !plan.target_vehicle_id.empty() || plan.pass_direction != 0 ||
      plan.trajectory_authorized || plan.lateral_maneuver_required ||
      plan.lateral_stop_authority_kind != OvertakePlan::LATERAL_STOP_NONE ||
      plan.lateral_stop_transaction_pass_direction != 0 ||
      plan.lateral_stop_authority_token != 0U ||
      !plan.trajectory.points.empty() ||
      (!plan.trajectory.header.frame_id.empty() &&
       plan.trajectory.header.frame_id != plan.header.frame_id)) {
    return result;
  }

  result.bytes.reserve(1024U);
  appendString(result.bytes, "FREE_RUN_PLAN_PAYLOAD_V1");
  appendString(result.bytes, plan.header.frame_id);
  appendU8(result.bytes, plan.phase);
  appendU32(result.bytes, plan.attempt_id);
  appendString(result.bytes, plan.target_vehicle_id);
  appendI8(result.bytes, plan.pass_direction);
  appendBool(result.bytes, plan.trajectory_authorized);
  appendBool(result.bytes, plan.lateral_maneuver_required);
  appendString(result.bytes, plan.decision_reason);
  appendU32(result.bytes, plan.authorization_failure_mask);
  appendU32(result.bytes, static_cast<std::uint32_t>(
                              plan.authorization_failure_reasons.size()));
  for (const auto &reason : plan.authorization_failure_reasons) {
    appendString(result.bytes, reason);
  }
  appendString(result.bytes, plan.candidate_reject_reason);
  appendBool(result.bytes, plan.safety_inputs_complete);
  appendBool(result.bytes, plan.tracking_usable);
  appendBool(result.bytes, plan.trajectory_publishable);
  appendString(result.bytes, plan.constraint_reason);
  appendU8(result.bytes, plan.lateral_stop_authority_kind);
  appendI8(result.bytes, plan.lateral_stop_transaction_pass_direction);
  appendU64(result.bytes, plan.lateral_stop_authority_token);
  appendString(result.bytes, plan.trajectory.header.frame_id);
  appendU32(result.bytes, 0U);
  appendU8(result.bytes, plan.aw2_identity_schema_version);
  appendU64(result.bytes, plan.planner_instance_id);
  appendU64(result.bytes, plan.race_arm_epoch);
  appendU64(result.bytes, plan.connector_transaction_id);
  appendU32(result.bytes, plan.candidate_revision);
  result.bytes.insert(result.bytes.end(), plan.candidate_content_sha256.begin(),
                      plan.candidate_content_sha256.end());
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::SOURCE_WINDOW_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.error = ValidationError::NONE;
  return result;
}

bool populateFreeRunPlanIdentityV1(OvertakePlan &plan) {
  plan.free_run_canonical_algorithm_version = 0U;
  plan.free_run_canonical_payload_sha256.fill(0U);
  const auto canonical = canonicalizeFreeRunPlanV1(plan);
  if (!canonical.valid()) {
    return false;
  }
  plan.free_run_canonical_algorithm_version =
      OvertakePlan::FREE_RUN_CANONICAL_ALGORITHM_V1;
  plan.free_run_canonical_payload_sha256 = canonical.sha256;
  return true;
}

bool validateFreeRunPlanIdentityV1(const OvertakePlan &plan) {
  if (plan.free_run_canonical_algorithm_version !=
      OvertakePlan::FREE_RUN_CANONICAL_ALGORITHM_V1) {
    return false;
  }
  const auto canonical = canonicalizeFreeRunPlanV1(plan);
  return canonical.valid() &&
         plan.free_run_canonical_payload_sha256 == canonical.sha256;
}

CanonicalRecord canonicalizeFreeRunReferenceV1(const Trajectory &trajectory) {
  CanonicalRecord result;
  result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
  if (trajectory.header.frame_id.empty() ||
      trajectory.header.frame_id.size() > 128U ||
      trajectory.points.size() < 2U ||
      trajectory.points.size() > kMaxFreeRunReferencePoints) {
    return result;
  }

  result.bytes.reserve(64U + trajectory.points.size() * 128U);
  appendString(result.bytes, "FREE_RUN_REFERENCE_V1");
  appendString(result.bytes, trajectory.header.frame_id);
  appendU32(result.bytes, static_cast<std::uint32_t>(trajectory.points.size()));
  builtin_interfaces::msg::Duration previous_time{};
  bool has_previous_time = false;
  for (const auto &point : trajectory.points) {
    const bool finite = validTime(point.time_from_start) &&
                        finitePose(point.pose) &&
                        std::isfinite(point.longitudinal_velocity_mps) &&
                        point.longitudinal_velocity_mps >= 0.0F &&
                        std::isfinite(point.lateral_velocity_mps) &&
                        std::isfinite(point.acceleration_mps2) &&
                        std::isfinite(point.heading_rate_rps) &&
                        std::isfinite(point.front_wheel_angle_rad) &&
                        std::isfinite(point.rear_wheel_angle_rad);
    if (!finite ||
        (has_previous_time && timeLess(point.time_from_start, previous_time))) {
      result.bytes.clear();
      result.error = ValidationError::GEOMETRY_POINT_INVALID;
      return result;
    }
    appendTime(result.bytes, point.time_from_start);
    appendPose(result.bytes, point.pose);
    appendFloat(result.bytes, point.longitudinal_velocity_mps);
    appendFloat(result.bytes, point.lateral_velocity_mps);
    appendFloat(result.bytes, point.acceleration_mps2);
    appendFloat(result.bytes, point.heading_rate_rps);
    appendFloat(result.bytes, point.front_wheel_angle_rad);
    appendFloat(result.bytes, point.rear_wheel_angle_rad);
    previous_time = point.time_from_start;
    has_previous_time = true;
  }
  result.sha256 = sha256(result.bytes);
  result.geometry_sha256 = result.sha256;
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord
canonicalizeGeometryV1(const std::vector<CandidateExecutionPoint> &points,
                       std::string_view domain) {
  return canonicalizeGeometry(points, domain);
}

CanonicalRecord
canonicalizeGeometryV1(const BoundedCandidateExecutionPoints &points,
                       std::string_view domain) {
  return canonicalizeGeometry(points, domain);
}

CanonicalRecord
canonicalizeBaseSourceV1(std::uint8_t source_kind, std::string_view frame_id,
                         const builtin_interfaces::msg::Time &source_stamp,
                         std::uint32_t source_generation,
                         std::uint32_t original_point_count,
                         const BoundedCandidateExecutionPoints &points) {
  CanonicalRecord result;
  if (!validSourceKind(source_kind)) {
    result.error = ValidationError::ENUM_INVALID;
    return result;
  }
  if (frame_id != "map") {
    result.error = ValidationError::FRAME_INVALID;
    return result;
  }
  if (!validTime(source_stamp)) {
    result.error = ValidationError::STAMP_INVALID;
    return result;
  }
  if (source_generation == 0U || original_point_count != points.size()) {
    result.error = ValidationError::IDENTITY_INVALID;
    return result;
  }
  const auto geometry =
      canonicalizeGeometry(points, "C002AY0_BASE_SOURCE_POINTS_V1");
  if (!geometry.valid()) {
    result.error = geometry.error;
    return result;
  }
  appendString(result.bytes, "C002AY0_BASE_SOURCE_V1");
  appendU32(result.bytes, 1U);
  appendU8(result.bytes, source_kind);
  appendString(result.bytes, frame_id);
  appendTime(result.bytes, source_stamp);
  appendU32(result.bytes, source_generation);
  appendU32(result.bytes, original_point_count);
  appendU32(result.bytes, static_cast<std::uint32_t>(points.size()));
  for (const auto &point : points) {
    appendPoint(result.bytes, point);
  }
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.geometry_sha256 = geometry.sha256;
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord canonicalizeBaseSourceWindowV2(
    std::uint8_t source_kind, std::string_view frame_id,
    const builtin_interfaces::msg::Time &source_stamp,
    std::uint32_t source_generation, std::uint32_t original_point_count,
    std::uint32_t first_source_index, std::uint32_t last_source_index,
    std::uint32_t nearest_source_index,
    const BoundedCandidateExecutionPoints &points) {
  CanonicalRecord result;
  if (!validSourceKind(source_kind)) {
    result.error = ValidationError::ENUM_INVALID;
    return result;
  }
  if (frame_id != "map") {
    result.error = ValidationError::FRAME_INVALID;
    return result;
  }
  if (!validTime(source_stamp)) {
    result.error = ValidationError::STAMP_INVALID;
    return result;
  }
  if (source_generation == 0U) {
    result.error = ValidationError::IDENTITY_INVALID;
    return result;
  }
  result.error =
      validateSourceWindow(original_point_count, first_source_index,
                           last_source_index, nearest_source_index, points);
  if (result.error != ValidationError::NONE) {
    return result;
  }
  const auto geometry =
      canonicalizeGeometry(points, "C002AY0_BASE_SOURCE_WINDOW_POINTS_V2");
  if (!geometry.valid()) {
    result.error = geometry.error;
    return result;
  }
  appendString(result.bytes, "C002AY0_BASE_SOURCE_WINDOW_V2");
  appendU32(result.bytes, 2U);
  appendU8(result.bytes, source_kind);
  appendString(result.bytes, frame_id);
  appendTime(result.bytes, source_stamp);
  appendU32(result.bytes, source_generation);
  appendU32(result.bytes, original_point_count);
  appendU32(result.bytes, first_source_index);
  appendU32(result.bytes, last_source_index);
  appendU32(result.bytes, nearest_source_index);
  appendU32(result.bytes, static_cast<std::uint32_t>(points.size()));
  for (const auto &point : points) {
    appendPoint(result.bytes, point);
  }
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.geometry_sha256 = geometry.sha256;
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord
canonicalizeControlPoseV1(const geometry_msgs::msg::Pose &pose,
                          const builtin_interfaces::msg::Time &stamp,
                          std::string_view frame_id, std::string_view domain) {
  CanonicalRecord result;
  if (frame_id != "map") {
    result.error = ValidationError::FRAME_INVALID;
    return result;
  }
  if (domain.empty() || domain.size() > 128U) {
    result.error = ValidationError::IDENTITY_INVALID;
    return result;
  }
  if (!validTime(stamp)) {
    result.error = ValidationError::STAMP_INVALID;
    return result;
  }
  if (!finitePose(pose)) {
    result.error = ValidationError::GEOMETRY_POINT_INVALID;
    return result;
  }
  appendString(result.bytes, domain);
  appendU32(result.bytes, 1U);
  appendString(result.bytes, frame_id);
  appendPose(result.bytes, pose);
  appendTime(result.bytes, stamp);
  result.sha256 = sha256(result.bytes);
  result.control_pose_sha256 = result.sha256;
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord
canonicalizeBaseSnapshotV1(const ControllerBaseTrajectorySnapshot &snapshot) {
  CanonicalRecord result;
  result.error = validateSnapshotContent(snapshot);
  if (!result.valid()) {
    return result;
  }
  const auto geometry = canonicalizeBaseGeometry(snapshot);
  if (!geometry.valid()) {
    result.error = geometry.error;
    return result;
  }
  result.geometry_sha256 = geometry.sha256;

  appendString(result.bytes, "C002AY0_BASE_SNAPSHOT_V1");
  appendU32(result.bytes, 1U);
  appendU8(result.bytes, snapshot.schema_version);
  appendBool(result.bytes, snapshot.authority_eligible);
  appendTime(result.bytes, snapshot.record_stamp);
  appendString(result.bytes, snapshot.frame_id);
  appendU64(result.bytes, snapshot.race_arm_epoch);
  appendU64(result.bytes, snapshot.controller_instance_id);
  appendU64(result.bytes, snapshot.controller_sequence);
  appendU64(result.bytes, snapshot.base_lease_id);
  appendTime(result.bytes, snapshot.lease_valid_until);
  appendU8(result.bytes, snapshot.base_source_kind);
  appendTime(result.bytes, snapshot.base_source_stamp);
  appendU32(result.bytes, snapshot.base_source_generation);
  appendU32(result.bytes, snapshot.base_original_point_count);
  appendU32(result.bytes, snapshot.first_source_index);
  appendU32(result.bytes, snapshot.last_source_index);
  appendU32(result.bytes, snapshot.nearest_source_index);
  appendU32(result.bytes,
            static_cast<std::uint32_t>(snapshot.base_points.size()));
  for (const auto &point : snapshot.base_points) {
    appendPoint(result.bytes, point);
  }
  appendU8(result.bytes, snapshot.base_source_digest_state);
  appendU8(result.bytes, snapshot.canonical_algorithm_version);
  appendDigest(result.bytes, result.geometry_sha256);
  appendDigest(result.bytes, snapshot.base_source_sha256);
  appendDigest(result.bytes, snapshot.controller_implementation_sha256);
  appendDigest(result.bytes, snapshot.controller_config_sha256);
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord canonicalizeAuthorizedTrajectoryV1(
    const AuthorizedCartesianTrajectory &trajectory) {
  CanonicalRecord result;
  result.error = validateTrajectoryContent(trajectory);
  if (!result.valid()) {
    return result;
  }
  const auto geometry = canonicalizeGeometryV1(
      trajectory.points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  if (!geometry.valid()) {
    result.error = geometry.error;
    return result;
  }
  const double arc_length_m = geometryArcLength(trajectory.points);
  if (std::abs(arc_length_m - trajectory.total_arc_length_m) >
      kScalarTolerance) {
    result.error = ValidationError::GEOMETRY_ARC_LENGTH_MISMATCH;
    return result;
  }
  result.geometry_sha256 = geometry.sha256;
  const auto start_pose = canonicalizeControlPoseV1(
      trajectory.candidate_start_control_pose,
      trajectory.candidate_start_control_pose_stamp, trajectory.frame_id,
      "C002AY0_CANDIDATE_START_CONTROL_POSE_V1");
  if (!start_pose.valid()) {
    result.error = start_pose.error;
    return result;
  }
  result.control_pose_sha256 = start_pose.sha256;

  std::vector<std::uint8_t> safety_bytes;
  appendString(safety_bytes, "C002AY0_SAFETY_PROOF_V1");
  appendU32(safety_bytes, 1U);
  appendDigest(safety_bytes, result.geometry_sha256);
  appendU64(safety_bytes, trajectory.safety_snapshot_id);
  appendU8(safety_bytes, trajectory.safety_evaluation_result);
  appendTime(safety_bytes, trajectory.safety_evaluation_stamp);
  appendTime(safety_bytes, trajectory.safety_valid_until);
  appendDigest(safety_bytes, trajectory.world_safety_snapshot_sha256);
  appendDigest(safety_bytes, trajectory.safety_evaluator_implementation_sha256);
  appendDigest(safety_bytes, trajectory.safety_evaluator_config_sha256);
  result.safety_proof_sha256 = sha256(safety_bytes);

  appendString(result.bytes, "C002AY0_AUTHORIZED_PAYLOAD_V1");
  appendU32(result.bytes, 1U);
  appendU8(result.bytes, trajectory.schema_version);
  appendBool(result.bytes, trajectory.authority_eligible);
  appendTime(result.bytes, trajectory.plan_stamp);
  appendString(result.bytes, trajectory.frame_id);
  appendPlanKey(result.bytes, trajectory.plan_sample_key);
  appendU32(result.bytes, trajectory.candidate_revision);
  appendU64(result.bytes, trajectory.authority_token);
  appendU8(result.bytes, trajectory.candidate_type);
  appendU8(result.bytes, trajectory.phase);
  appendU8(result.bytes, trajectory.authorization_state);
  appendU64(result.bytes, trajectory.source_controller_instance_id);
  appendU64(result.bytes, trajectory.source_controller_sequence);
  appendU64(result.bytes, trajectory.base_lease_id);
  appendTime(result.bytes, trajectory.base_lease_valid_until);
  appendU8(result.bytes, trajectory.base_source_kind);
  appendTime(result.bytes, trajectory.base_source_stamp);
  appendU32(result.bytes, trajectory.base_source_generation);
  appendU32(result.bytes, trajectory.base_original_point_count);
  appendU32(result.bytes, trajectory.base_first_source_index);
  appendU32(result.bytes, trajectory.base_last_source_index);
  appendU32(result.bytes, trajectory.base_nearest_source_index);
  appendU8(result.bytes, trajectory.base_source_digest_state);
  appendU8(result.bytes, trajectory.canonical_algorithm_version);
  appendDigest(result.bytes, trajectory.base_geometry_sha256);
  appendDigest(result.bytes, trajectory.base_source_sha256);
  appendDigest(result.bytes, trajectory.base_snapshot_sha256);
  appendU32(result.bytes, static_cast<std::uint32_t>(trajectory.points.size()));
  for (const auto &point : trajectory.points) {
    appendPoint(result.bytes, point);
  }
  appendU32(result.bytes, trajectory.original_candidate_point_count);
  appendDouble(result.bytes, trajectory.total_arc_length_m);
  appendDouble(result.bytes, trajectory.required_spatial_horizon_m);
  appendDouble(result.bytes, trajectory.join_end_arc_length_m);
  appendDouble(result.bytes, trajectory.post_join_arc_length_m);
  appendU64(result.bytes, trajectory.safety_snapshot_id);
  appendU8(result.bytes, trajectory.safety_evaluation_result);
  appendTime(result.bytes, trajectory.safety_evaluation_stamp);
  appendTime(result.bytes, trajectory.safety_valid_until);
  appendDigest(result.bytes, trajectory.world_safety_snapshot_sha256);
  appendDigest(result.bytes, trajectory.safety_evaluator_implementation_sha256);
  appendDigest(result.bytes, trajectory.safety_evaluator_config_sha256);
  appendDigest(result.bytes, result.safety_proof_sha256);
  appendDigest(result.bytes, trajectory.controller_implementation_sha256);
  appendDigest(result.bytes, trajectory.controller_config_sha256);
  appendPose(result.bytes, trajectory.candidate_start_control_pose);
  appendTime(result.bytes, trajectory.candidate_start_control_pose_stamp);
  appendDigest(result.bytes, result.control_pose_sha256);
  appendDigest(result.bytes, result.geometry_sha256);
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::GEOMETRY_POINT_COUNT_INVALID;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.error = ValidationError::NONE;
  return result;
}

CanonicalRecord canonicalizeApplicationStatusV1(
    const CartesianTrajectoryApplicationStatus &status) {
  CanonicalRecord result;
  result.error = validateStatusContent(status);
  if (!result.valid()) {
    return result;
  }
  const bool applied =
      status.application_state ==
      CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  if (applied) {
    const auto applied_pose = canonicalizeControlPoseV1(
        status.applied_control_pose, status.applied_control_pose_stamp,
        status.frame_id, "C002AY0_APPLIED_CONTROL_POSE_V1");
    if (!applied_pose.valid()) {
      result.error = applied_pose.error;
      return result;
    }
    result.control_pose_sha256 = applied_pose.sha256;
  }
  appendString(result.bytes, "C002AY0_APPLICATION_ACK_V1");
  appendU32(result.bytes, 1U);
  appendU8(result.bytes, status.schema_version);
  appendBool(result.bytes, status.authority_eligible);
  appendTime(result.bytes, status.status_stamp);
  appendString(result.bytes, status.frame_id);
  appendU8(result.bytes, status.application_state);
  appendU8(result.bytes, status.reject_reason);
  appendPlanKey(result.bytes, status.plan_sample_key);
  appendU32(result.bytes, status.candidate_revision);
  appendU64(result.bytes, status.authority_token);
  appendU64(result.bytes, status.controller_instance_id);
  appendU64(result.bytes, status.controller_sequence);
  appendU64(result.bytes, status.base_lease_id);
  appendU8(result.bytes, status.base_source_kind);
  appendTime(result.bytes, status.base_source_stamp);
  appendU32(result.bytes, status.base_source_generation);
  appendDigest(result.bytes, status.expected_base_source_sha256);
  appendDigest(result.bytes, status.observed_base_source_sha256);
  appendDigest(result.bytes, status.expected_base_geometry_sha256);
  appendDigest(result.bytes, status.observed_base_geometry_sha256);
  appendDigest(result.bytes, status.expected_base_snapshot_sha256);
  appendDigest(result.bytes, status.observed_base_snapshot_sha256);
  appendDigest(result.bytes, status.expected_geometry_sha256);
  appendDigest(result.bytes, status.applied_geometry_sha256);
  appendDigest(result.bytes, status.expected_payload_sha256);
  appendDigest(result.bytes, status.observed_payload_sha256);
  appendDigest(result.bytes, status.expected_controller_implementation_sha256);
  appendDigest(result.bytes, status.observed_controller_implementation_sha256);
  appendDigest(result.bytes, status.expected_controller_config_sha256);
  appendDigest(result.bytes, status.observed_controller_config_sha256);
  appendU32(result.bytes, status.nearest_index);
  appendU32(result.bytes, status.lookahead_index);
  appendBool(result.bytes, status.endpoint_fallback);
  appendDouble(result.bytes, status.available_spatial_horizon_m);
  appendDouble(result.bytes, status.required_spatial_horizon_m);
  appendDouble(result.bytes, status.handoff_position_error_m);
  appendDouble(result.bytes, status.handoff_yaw_error_rad);
  appendPose(result.bytes, status.applied_control_pose);
  appendTime(result.bytes, status.applied_control_pose_stamp);
  appendDigest(result.bytes, result.control_pose_sha256);
  appendTime(result.bytes, status.controller_command_stamp);
  appendU64(result.bytes, status.command_sequence);
  result.sha256 = sha256(result.bytes);
  result.error = ValidationError::NONE;
  return result;
}

ValidationError
validateBaseSnapshotV1(const ControllerBaseTrajectorySnapshot &snapshot) {
  const auto canonical = canonicalizeBaseSnapshotV1(snapshot);
  if (!canonical.valid()) {
    return canonical.error;
  }
  if (digestMissing(snapshot.base_geometry_sha256) ||
      digestMissing(snapshot.snapshot_sha256)) {
    return ValidationError::DIGEST_MISSING;
  }
  if (snapshot.base_geometry_sha256 != canonical.geometry_sha256 ||
      snapshot.snapshot_sha256 != canonical.sha256) {
    return ValidationError::DIGEST_MISMATCH;
  }
  return ValidationError::NONE;
}

ValidationError validateAuthorizedTrajectoryV1(
    const AuthorizedCartesianTrajectory &trajectory) {
  const auto canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  if (!canonical.valid()) {
    return canonical.error;
  }
  if (digestMissing(trajectory.geometry_sha256) ||
      digestMissing(trajectory.safety_proof_sha256) ||
      digestMissing(trajectory.candidate_start_control_pose_sha256) ||
      digestMissing(trajectory.payload_sha256)) {
    return ValidationError::DIGEST_MISSING;
  }
  if (trajectory.geometry_sha256 != canonical.geometry_sha256 ||
      trajectory.safety_proof_sha256 != canonical.safety_proof_sha256 ||
      trajectory.candidate_start_control_pose_sha256 !=
          canonical.control_pose_sha256 ||
      trajectory.payload_sha256 != canonical.sha256) {
    return ValidationError::DIGEST_MISMATCH;
  }
  return ValidationError::NONE;
}

ValidationError validateApplicationStatusV1(
    const CartesianTrajectoryApplicationStatus &status) {
  const auto canonical = canonicalizeApplicationStatusV1(status);
  if (!canonical.valid()) {
    return canonical.error;
  }
  const bool slot_ready =
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_WARMED ||
      status.application_state ==
          CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  if (slot_ready &&
      (status.expected_base_source_sha256 !=
           status.observed_base_source_sha256 ||
       status.expected_base_geometry_sha256 !=
           status.observed_base_geometry_sha256 ||
       status.expected_base_snapshot_sha256 !=
           status.observed_base_snapshot_sha256 ||
       status.expected_payload_sha256 != status.observed_payload_sha256 ||
       status.expected_controller_implementation_sha256 !=
           status.observed_controller_implementation_sha256 ||
       status.expected_controller_config_sha256 !=
           status.observed_controller_config_sha256)) {
    return ValidationError::DIGEST_MISMATCH;
  }
  const bool applied =
      status.application_state ==
      CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  if (applied &&
      (digestMissing(status.applied_control_pose_sha256) ||
       status.expected_geometry_sha256 != status.applied_geometry_sha256 ||
       status.applied_control_pose_sha256 != canonical.control_pose_sha256)) {
    return ValidationError::DIGEST_MISMATCH;
  }
  return ValidationError::NONE;
}

ValidationError validateTrajectoryAgainstBaseSnapshotV1(
    const ControllerBaseTrajectorySnapshot &snapshot,
    const AuthorizedCartesianTrajectory &trajectory) {
  const auto snapshot_error = validateBaseSnapshotV1(snapshot);
  if (snapshot_error != ValidationError::NONE) {
    return snapshot_error;
  }
  const auto trajectory_error = validateAuthorizedTrajectoryV1(trajectory);
  if (trajectory_error != ValidationError::NONE) {
    return trajectory_error;
  }
  if (trajectory.frame_id != snapshot.frame_id ||
      trajectory.plan_sample_key.race_arm_epoch != snapshot.race_arm_epoch ||
      trajectory.source_controller_instance_id !=
          snapshot.controller_instance_id ||
      trajectory.source_controller_sequence != snapshot.controller_sequence ||
      trajectory.base_lease_id != snapshot.base_lease_id ||
      !sameTime(trajectory.base_lease_valid_until,
                snapshot.lease_valid_until) ||
      trajectory.base_source_kind != snapshot.base_source_kind ||
      !sameTime(trajectory.base_source_stamp, snapshot.base_source_stamp) ||
      trajectory.base_source_generation != snapshot.base_source_generation ||
      trajectory.base_original_point_count !=
          snapshot.base_original_point_count ||
      trajectory.base_first_source_index != snapshot.first_source_index ||
      trajectory.base_last_source_index != snapshot.last_source_index ||
      trajectory.base_nearest_source_index != snapshot.nearest_source_index ||
      trajectory.base_source_digest_state !=
          snapshot.base_source_digest_state ||
      trajectory.canonical_algorithm_version !=
          snapshot.canonical_algorithm_version ||
      trajectory.base_geometry_sha256 != snapshot.base_geometry_sha256 ||
      trajectory.base_source_sha256 != snapshot.base_source_sha256 ||
      trajectory.base_snapshot_sha256 != snapshot.snapshot_sha256 ||
      trajectory.controller_implementation_sha256 !=
          snapshot.controller_implementation_sha256 ||
      trajectory.controller_config_sha256 !=
          snapshot.controller_config_sha256) {
    return ValidationError::DIGEST_MISMATCH;
  }
  if (timeLess(trajectory.plan_stamp, snapshot.record_stamp)) {
    return ValidationError::STAMP_INVALID;
  }
  return ValidationError::NONE;
}

ValidationError validateApplicationStatusAgainstAuthorizedTrajectoryV1(
    const AuthorizedCartesianTrajectory &trajectory,
    const CartesianTrajectoryApplicationStatus &status) {
  const auto trajectory_error = validateAuthorizedTrajectoryV1(trajectory);
  if (trajectory_error != ValidationError::NONE) {
    return trajectory_error;
  }
  const auto status_error = validateApplicationStatusV1(status);
  if (status_error != ValidationError::NONE) {
    return status_error;
  }
  if (status.frame_id != trajectory.frame_id ||
      !samePlanKey(status.plan_sample_key, trajectory.plan_sample_key) ||
      status.candidate_revision != trajectory.candidate_revision ||
      status.authority_token != trajectory.authority_token ||
      status.controller_instance_id !=
          trajectory.source_controller_instance_id ||
      status.controller_sequence != trajectory.source_controller_sequence ||
      status.base_lease_id != trajectory.base_lease_id ||
      status.base_source_kind != trajectory.base_source_kind ||
      !sameTime(status.base_source_stamp, trajectory.base_source_stamp) ||
      status.base_source_generation != trajectory.base_source_generation ||
      status.expected_base_source_sha256 != trajectory.base_source_sha256 ||
      status.expected_base_geometry_sha256 != trajectory.base_geometry_sha256 ||
      status.expected_base_snapshot_sha256 != trajectory.base_snapshot_sha256 ||
      status.expected_geometry_sha256 != trajectory.geometry_sha256 ||
      status.expected_payload_sha256 != trajectory.payload_sha256 ||
      status.expected_controller_implementation_sha256 !=
          trajectory.controller_implementation_sha256 ||
      status.expected_controller_config_sha256 !=
          trajectory.controller_config_sha256) {
    return ValidationError::DIGEST_MISMATCH;
  }
  return ValidationError::NONE;
}

} // namespace overtake_transport_contract::c002ay0
