#include "simple_pure_pursuit/ay0_base_capture.hpp"

#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace simple_pure_pursuit {
namespace {

Ay0BaseCaptureValidationReason toValidationReason(
    const overtake_transport_contract::c002ay0::ValidationError error) {
  using Error = overtake_transport_contract::c002ay0::ValidationError;
  using Reason = Ay0BaseCaptureValidationReason;
  switch (error) {
  case Error::NONE:
    return Reason::kNone;
  case Error::SCHEMA_UNSUPPORTED:
    return Reason::kSchemaUnsupported;
  case Error::AUTHORITY_FLAG_INVALID:
    return Reason::kAuthorityFlagInvalid;
  case Error::FRAME_INVALID:
    return Reason::kFrameInvalid;
  case Error::IDENTITY_INVALID:
    return Reason::kIdentityInvalid;
  case Error::STAMP_INVALID:
    return Reason::kStampInvalid;
  case Error::ENUM_INVALID:
    return Reason::kEnumInvalid;
  case Error::DIGEST_MISSING:
    return Reason::kDigestMissing;
  case Error::DIGEST_MISMATCH:
    return Reason::kDigestMismatch;
  case Error::GEOMETRY_POINT_COUNT_INVALID:
    return Reason::kGeometryPointCountInvalid;
  case Error::GEOMETRY_POINT_INVALID:
    return Reason::kGeometryPointInvalid;
  case Error::GEOMETRY_TIME_NONMONOTONIC:
    return Reason::kGeometryTimeNonmonotonic;
  case Error::GEOMETRY_ARC_NONMONOTONIC:
    return Reason::kGeometryArcNonmonotonic;
  case Error::GEOMETRY_ARC_SPACING_EXCEEDED:
    return Reason::kGeometryArcSpacingExceeded;
  case Error::GEOMETRY_YAW_STEP_EXCEEDED:
    return Reason::kGeometryYawStepExceeded;
  case Error::GEOMETRY_ARC_LENGTH_MISMATCH:
    return Reason::kGeometryArcLengthMismatch;
  case Error::SOURCE_WINDOW_INVALID:
    return Reason::kSourceWindowInvalid;
  case Error::LEASE_INVALID:
    return Reason::kLeaseInvalid;
  case Error::SAFETY_PROOF_INVALID:
    return Reason::kSafetyProofInvalid;
  case Error::HORIZON_INVALID:
    return Reason::kHorizonInvalid;
  case Error::APPLICATION_STATUS_INVALID:
    return Reason::kApplicationStatusInvalid;
  }
  return Reason::kUnknown;
}

Ay0BaseCapturePointInvalidField toPointInvalidField(
    const overtake_transport_contract::c002ay0::PointInvalidField field) {
  using Source = overtake_transport_contract::c002ay0::PointInvalidField;
  using Result = Ay0BaseCapturePointInvalidField;
  switch (field) {
  case Source::NONE:
    return Result::kNone;
  case Source::TIME_FROM_START:
    return Result::kTimeFromStart;
  case Source::POSITION_X:
    return Result::kPositionX;
  case Source::POSITION_Y:
    return Result::kPositionY;
  case Source::POSITION_Z:
    return Result::kPositionZ;
  case Source::ORIENTATION_X:
    return Result::kOrientationX;
  case Source::ORIENTATION_Y:
    return Result::kOrientationY;
  case Source::ORIENTATION_Z:
    return Result::kOrientationZ;
  case Source::ORIENTATION_W:
    return Result::kOrientationW;
  case Source::ORIENTATION_NORM:
    return Result::kOrientationNorm;
  case Source::LONGITUDINAL_VELOCITY_NONFINITE:
    return Result::kLongitudinalVelocityNonfinite;
  case Source::LONGITUDINAL_VELOCITY_RANGE:
    return Result::kLongitudinalVelocityRange;
  case Source::LATERAL_VELOCITY:
    return Result::kLateralVelocity;
  case Source::ACCELERATION:
    return Result::kAcceleration;
  case Source::HEADING_RATE:
    return Result::kHeadingRate;
  case Source::FRONT_WHEEL_ANGLE:
    return Result::kFrontWheelAngle;
  case Source::REAR_WHEEL_ANGLE:
    return Result::kRearWheelAngle;
  }
  return Result::kUnknown;
}

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

Ay0BaseCaptureResult buildAy0BaseSnapshotImpl(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage,
    Ay0BaseCaptureValidationReason &validation_reason,
    Ay0BaseCapturePointDiagnostic &point_diagnostic) {
  using Snapshot =
      multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;
  using namespace overtake_transport_contract::c002ay0;
  snapshot =
      multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot();
  if (input.trajectory == nullptr) {
    return Ay0BaseCaptureResult::kMissingTrajectory;
  }
  const auto &trajectory = *input.trajectory;
  if (trajectory.header.frame_id.empty() ||
      trajectory.header.frame_id != "map") {
    return Ay0BaseCaptureResult::kFrameInvalid;
  }
  if (trajectory.points.size() < 2U ||
      trajectory.points.size() > std::numeric_limits<std::uint32_t>::max()) {
    return Ay0BaseCaptureResult::kPointLimit;
  }
  if (input.nearest_source_index >= trajectory.points.size()) {
    return Ay0BaseCaptureResult::kNearestInvalid;
  }
  if (input.race_arm_epoch == 0U || input.controller_instance_id == 0U ||
      input.controller_sequence == 0U || input.base_lease_id == 0U ||
      input.source_generation == 0U || input.source_kind == 0U) {
    return Ay0BaseCaptureResult::kIdentityInvalid;
  }

  const std::size_t count =
      std::min(trajectory.points.size(), kMaxCartesianPoints);
  const std::size_t first =
      std::min(input.nearest_source_index, trajectory.points.size() - count);
  const std::size_t last = first + count - 1U;

  snapshot.schema_version = Snapshot::SCHEMA_V1_SHADOW;
  snapshot.authority_eligible = false;
  snapshot.record_stamp.sec = input.record_stamp.sec;
  snapshot.record_stamp.nanosec = input.record_stamp.nanosec;
  snapshot.frame_id = trajectory.header.frame_id;
  snapshot.race_arm_epoch = input.race_arm_epoch;
  snapshot.controller_instance_id = input.controller_instance_id;
  snapshot.controller_sequence = input.controller_sequence;
  snapshot.base_lease_id = input.base_lease_id;
  snapshot.lease_valid_until.sec = input.lease_valid_until.sec;
  snapshot.lease_valid_until.nanosec = input.lease_valid_until.nanosec;
  snapshot.base_source_kind = input.source_kind;
  snapshot.base_source_stamp = trajectory.header.stamp;
  snapshot.base_source_generation = input.source_generation;
  snapshot.base_original_point_count =
      static_cast<std::uint32_t>(trajectory.points.size());
  snapshot.first_source_index = static_cast<std::uint32_t>(first);
  snapshot.last_source_index = static_cast<std::uint32_t>(last);
  snapshot.nearest_source_index =
      static_cast<std::uint32_t>(input.nearest_source_index);
  snapshot.base_points.reserve(count);
  for (std::size_t index = first; index <= last; ++index) {
    const auto fixed = toFixedPoint(trajectory.points[index]);
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint point;
    point.time_from_start.sec = fixed.time_from_start.sec;
    point.time_from_start.nanosec = fixed.time_from_start.nanosec;
    point.position_x_m = fixed.position_x_m;
    point.position_y_m = fixed.position_y_m;
    point.position_z_m = fixed.position_z_m;
    point.orientation_x = fixed.orientation_x;
    point.orientation_y = fixed.orientation_y;
    point.orientation_z = fixed.orientation_z;
    point.orientation_w = fixed.orientation_w;
    point.longitudinal_velocity_mps = fixed.longitudinal_velocity_mps;
    point.lateral_velocity_mps = fixed.lateral_velocity_mps;
    point.acceleration_mps2 = fixed.acceleration_mps2;
    point.heading_rate_rps = fixed.heading_rate_rps;
    point.front_wheel_angle_rad = fixed.front_wheel_angle_rad;
    point.rear_wheel_angle_rad = fixed.rear_wheel_angle_rad;
    snapshot.base_points.push_back(point);
  }
  snapshot.base_source_digest_state = Snapshot::BASE_SOURCE_DIGEST_COMPLETE;
  snapshot.canonical_algorithm_version =
      trajectory.points.size() <= kMaxCartesianPoints ? 1U : 2U;
  const auto source =
      snapshot.canonical_algorithm_version == 1U
          ? canonicalizeBaseSourceV1(
                snapshot.base_source_kind, snapshot.frame_id,
                snapshot.base_source_stamp, snapshot.base_source_generation,
                snapshot.base_original_point_count, snapshot.base_points)
          : canonicalizeBaseSourceWindowV2(
                snapshot.base_source_kind, snapshot.frame_id,
                snapshot.base_source_stamp, snapshot.base_source_generation,
                snapshot.base_original_point_count, snapshot.first_source_index,
                snapshot.last_source_index, snapshot.nearest_source_index,
                snapshot.base_points);
  if (!source.valid()) {
    diagnostic_stage = Ay0BaseCaptureDiagnosticStage::kSourceCanonical;
    validation_reason = toValidationReason(source.error);
    point_diagnostic.field = Ay0BaseCapturePointInvalidField::kNotApplicable;
    if (source.error == ValidationError::GEOMETRY_POINT_INVALID) {
      for (std::size_t window_index = 0U;
           window_index < snapshot.base_points.size(); ++window_index) {
        const auto field =
            classifyPointInvalidFieldV1(snapshot.base_points[window_index]);
        if (field != PointInvalidField::NONE) {
          point_diagnostic.field = toPointInvalidField(field);
          point_diagnostic.window_index =
              static_cast<std::uint32_t>(window_index);
          point_diagnostic.source_index =
              snapshot.first_source_index + point_diagnostic.window_index;
          break;
        }
      }
    }
    return Ay0BaseCaptureResult::kPointLimit;
  }
  snapshot.base_source_sha256 = source.sha256;
  snapshot.controller_implementation_sha256 =
      input.controller_implementation_sha256;
  snapshot.controller_config_sha256 = input.controller_config_sha256;
  const auto canonical = canonicalizeBaseSnapshotV1(snapshot);
  if (!canonical.valid()) {
    diagnostic_stage = Ay0BaseCaptureDiagnosticStage::kSnapshotCanonical;
    validation_reason = toValidationReason(canonical.error);
    point_diagnostic.field = Ay0BaseCapturePointInvalidField::kNotApplicable;
    return Ay0BaseCaptureResult::kPointLimit;
  }
  snapshot.base_geometry_sha256 = canonical.geometry_sha256;
  snapshot.snapshot_sha256 = canonical.sha256;
  const auto final_validation = validateBaseSnapshotV1(snapshot);
  if (final_validation != ValidationError::NONE) {
    diagnostic_stage = Ay0BaseCaptureDiagnosticStage::kFinalValidation;
    validation_reason = toValidationReason(final_validation);
    point_diagnostic.field = Ay0BaseCapturePointInvalidField::kNotApplicable;
    return Ay0BaseCaptureResult::kPointLimit;
  }
  validation_reason = Ay0BaseCaptureValidationReason::kNone;
  point_diagnostic.field = Ay0BaseCapturePointInvalidField::kNone;
  return Ay0BaseCaptureResult::kBuilt;
}

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &snapshot) noexcept {
  Ay0BaseCaptureDiagnosticStage diagnostic_stage{};
  Ay0BaseCaptureValidationReason validation_reason{};
  Ay0BaseCapturePointDiagnostic point_diagnostic{};
  return buildAy0BaseSnapshot(input, snapshot, diagnostic_stage,
                              validation_reason, point_diagnostic);
}

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage) noexcept {
  Ay0BaseCaptureValidationReason validation_reason{};
  Ay0BaseCapturePointDiagnostic point_diagnostic{};
  return buildAy0BaseSnapshot(input, snapshot, diagnostic_stage,
                              validation_reason, point_diagnostic);
}

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage,
    Ay0BaseCaptureValidationReason &validation_reason) noexcept {
  Ay0BaseCapturePointDiagnostic point_diagnostic{};
  return buildAy0BaseSnapshot(input, snapshot, diagnostic_stage,
                              validation_reason, point_diagnostic);
}

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage,
    Ay0BaseCaptureValidationReason &validation_reason,
    Ay0BaseCapturePointDiagnostic &point_diagnostic) noexcept {
  diagnostic_stage = Ay0BaseCaptureDiagnosticStage::kNotRun;
  validation_reason = Ay0BaseCaptureValidationReason::kNotRun;
  point_diagnostic = Ay0BaseCapturePointDiagnostic{};
  try {
    return buildAy0BaseSnapshotImpl(input, snapshot, diagnostic_stage,
                                    validation_reason, point_diagnostic);
  } catch (...) {
    // The typed snapshot and canonical byte buffers allocate.  Allocation or
    // middleware message failures must suppress this non-authoritative
    // attestation, never terminate the controller process.
    snapshot =
        multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot();
    diagnostic_stage = Ay0BaseCaptureDiagnosticStage::kException;
    return Ay0BaseCaptureResult::kPointLimit;
  }
}

} // namespace simple_pure_pursuit
