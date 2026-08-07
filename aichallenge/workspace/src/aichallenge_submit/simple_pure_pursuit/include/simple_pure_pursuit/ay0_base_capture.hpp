#pragma once

#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"
#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace simple_pure_pursuit {

struct Ay0BaseCaptureInput {
  const autoware_auto_planning_msgs::msg::Trajectory *trajectory{nullptr};
  std::size_t nearest_source_index{0U};
  std::uint8_t source_kind{0U};
  std::uint32_t source_generation{0U};
  overtake_transport_contract::c002ay0::FixedTime record_stamp{};
  overtake_transport_contract::c002ay0::FixedTime lease_valid_until{};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t controller_instance_id{0U};
  std::uint64_t controller_sequence{0U};
  std::uint64_t base_lease_id{0U};
  overtake_transport_contract::c002ay0::Digest
      controller_implementation_sha256{};
  overtake_transport_contract::c002ay0::Digest controller_config_sha256{};
};

enum class Ay0BaseCaptureResult : std::uint8_t {
  kBuilt,
  kMissingTrajectory,
  kFrameInvalid,
  kPointLimit,
  kNearestInvalid,
  kIdentityInvalid,
};

// Observation-only provenance for failures that retain kPointLimit as the
// public, fail-closed build result.  These values never grant authority.
enum class Ay0BaseCaptureDiagnosticStage : std::uint8_t {
  kNotRun = 0U,
  kSourceCanonical = 1U,
  kSnapshotCanonical = 2U,
  kFinalValidation = 3U,
  kException = 4U,
};

// Stable observation-only mapping of the canonical validator result.  This is
// deliberately independent of the internal ValidationError numeric ABI.
enum class Ay0BaseCaptureValidationReason : std::uint8_t {
  kNotRun = 0U,
  kNone = 1U,
  kSchemaUnsupported = 2U,
  kAuthorityFlagInvalid = 3U,
  kFrameInvalid = 4U,
  kIdentityInvalid = 5U,
  kStampInvalid = 6U,
  kEnumInvalid = 7U,
  kDigestMissing = 8U,
  kDigestMismatch = 9U,
  kGeometryPointCountInvalid = 10U,
  kGeometryPointInvalid = 11U,
  kGeometryTimeNonmonotonic = 12U,
  kGeometryArcNonmonotonic = 13U,
  kGeometryArcSpacingExceeded = 14U,
  kGeometryYawStepExceeded = 15U,
  kGeometryArcLengthMismatch = 16U,
  kSourceWindowInvalid = 17U,
  kLeaseInvalid = 18U,
  kSafetyProofInvalid = 19U,
  kHorizonInvalid = 20U,
  kApplicationStatusInvalid = 21U,
  kUnknown = 255U,
};

enum class Ay0BaseCapturePointInvalidField : std::uint8_t {
  kNotRun = 0U,
  kNone = 1U,
  kNotApplicable = 2U,
  kTimeFromStart = 3U,
  kPositionX = 4U,
  kPositionY = 5U,
  kPositionZ = 6U,
  kOrientationX = 7U,
  kOrientationY = 8U,
  kOrientationZ = 9U,
  kOrientationW = 10U,
  kOrientationNorm = 11U,
  kLongitudinalVelocityNonfinite = 12U,
  kLongitudinalVelocityRange = 13U,
  kLateralVelocity = 14U,
  kAcceleration = 15U,
  kHeadingRate = 16U,
  kFrontWheelAngle = 17U,
  kRearWheelAngle = 18U,
  kUnknown = 255U,
};

struct Ay0BaseCapturePointDiagnostic {
  static constexpr std::uint32_t kNoIndex =
      std::numeric_limits<std::uint32_t>::max();
  Ay0BaseCapturePointInvalidField field{
      Ay0BaseCapturePointInvalidField::kNotRun};
  std::uint32_t window_index{kNoIndex};
  std::uint32_t source_index{kNoIndex};
};

Ay0BaseCaptureResult buildAy0FixedBaseRecord(
    const Ay0BaseCaptureInput &input,
    overtake_transport_contract::c002ay0::FixedBaseRecord &record) noexcept;

// Builds the live typed snapshot. Sources larger than the transport bound are
// represented by a contiguous, absolute-indexed window of at most 256 points.
Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
        &snapshot) noexcept;

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage) noexcept;

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage,
    Ay0BaseCaptureValidationReason &validation_reason) noexcept;

Ay0BaseCaptureResult buildAy0BaseSnapshot(
    const Ay0BaseCaptureInput &input,
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot &snapshot,
    Ay0BaseCaptureDiagnosticStage &diagnostic_stage,
    Ay0BaseCaptureValidationReason &validation_reason,
    Ay0BaseCapturePointDiagnostic &point_diagnostic) noexcept;

} // namespace simple_pure_pursuit
