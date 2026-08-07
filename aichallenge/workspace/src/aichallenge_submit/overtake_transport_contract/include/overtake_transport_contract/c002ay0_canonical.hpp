#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/candidate_execution_point.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/cartesian_trajectory_application_status.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp"
#include "multi_purpose_mpc_ros_msgs/msg/overtake_plan.hpp"

namespace overtake_transport_contract::c002ay0 {

constexpr std::size_t kSha256Size = 32U;
constexpr std::size_t kMaxCartesianPoints = 256U;
constexpr double kMaxTransportArcSpacingM = 0.25;
constexpr double kMaxTransportYawStepRad = 0.05;
constexpr double kMaxTransportArcLengthM = 63.75;
constexpr double kExecutionSpeedCeilingMps = 10.0;
constexpr std::size_t kMaxFreeRunReferencePoints = 4096U;

using Digest = std::array<std::uint8_t, kSha256Size>;
using CandidateExecutionPoint =
    multi_purpose_mpc_ros_msgs::msg::CandidateExecutionPoint;
using ControllerBaseTrajectorySnapshot =
    multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot;
using AuthorizedCartesianTrajectory =
    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory;
using CartesianTrajectoryApplicationStatus =
    multi_purpose_mpc_ros_msgs::msg::CartesianTrajectoryApplicationStatus;
using OvertakePlan = multi_purpose_mpc_ros_msgs::msg::OvertakePlan;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using BoundedCandidateExecutionPoints =
    ControllerBaseTrajectorySnapshot::_base_points_type;

enum class ValidationError : std::uint8_t {
  NONE = 0U,
  SCHEMA_UNSUPPORTED,
  AUTHORITY_FLAG_INVALID,
  FRAME_INVALID,
  IDENTITY_INVALID,
  STAMP_INVALID,
  ENUM_INVALID,
  DIGEST_MISSING,
  DIGEST_MISMATCH,
  GEOMETRY_POINT_COUNT_INVALID,
  GEOMETRY_POINT_INVALID,
  GEOMETRY_TIME_NONMONOTONIC,
  GEOMETRY_ARC_NONMONOTONIC,
  GEOMETRY_ARC_SPACING_EXCEEDED,
  GEOMETRY_YAW_STEP_EXCEEDED,
  GEOMETRY_ARC_LENGTH_MISMATCH,
  SOURCE_WINDOW_INVALID,
  LEASE_INVALID,
  SAFETY_PROOF_INVALID,
  HORIZON_INVALID,
  APPLICATION_STATUS_INVALID,
};

// Observation-only detail for GEOMETRY_POINT_INVALID.  This does not
// participate in canonical bytes, digests, validity, or authority.
enum class PointInvalidField : std::uint8_t {
  NONE = 0U,
  TIME_FROM_START = 1U,
  POSITION_X = 2U,
  POSITION_Y = 3U,
  POSITION_Z = 4U,
  ORIENTATION_X = 5U,
  ORIENTATION_Y = 6U,
  ORIENTATION_Z = 7U,
  ORIENTATION_W = 8U,
  ORIENTATION_NORM = 9U,
  LONGITUDINAL_VELOCITY_NONFINITE = 10U,
  LONGITUDINAL_VELOCITY_RANGE = 11U,
  LATERAL_VELOCITY = 12U,
  ACCELERATION = 13U,
  HEADING_RATE = 14U,
  FRONT_WHEEL_ANGLE = 15U,
  REAR_WHEEL_ANGLE = 16U,
};

struct CanonicalRecord {
  ValidationError error{ValidationError::SCHEMA_UNSUPPORTED};
  std::vector<std::uint8_t> bytes;
  Digest sha256{};
  Digest geometry_sha256{};
  Digest safety_proof_sha256{};
  Digest control_pose_sha256{};

  bool valid() const noexcept { return error == ValidationError::NONE; }
};

Digest sha256(const std::vector<std::uint8_t> &bytes);

PointInvalidField
classifyPointInvalidFieldV1(const CandidateExecutionPoint &point) noexcept;

// Stamp and delivery-generation independent semantic identity for a normal
// FREE_RUN plan. The result never grants motion authority.
CanonicalRecord canonicalizeFreeRunPlanV1(const OvertakePlan &plan);

// Populate or validate the live identity carried by OvertakePlan. The identity
// fields themselves are excluded from the canonical payload.
bool populateFreeRunPlanIdentityV1(OvertakePlan &plan);
bool validateFreeRunPlanIdentityV1(const OvertakePlan &plan);

// Full immutable reference identity computed outside the control timer.
// Source stamp/generation are carried separately in FreeRunSourceKey.
CanonicalRecord canonicalizeFreeRunReferenceV1(const Trajectory &trajectory);

CanonicalRecord
canonicalizeGeometryV1(const std::vector<CandidateExecutionPoint> &points,
                       std::string_view domain);

CanonicalRecord
canonicalizeGeometryV1(const BoundedCandidateExecutionPoints &points,
                       std::string_view domain);

CanonicalRecord
canonicalizeBaseSourceV1(std::uint8_t source_kind, std::string_view frame_id,
                         const builtin_interfaces::msg::Time &source_stamp,
                         std::uint32_t source_generation,
                         std::uint32_t original_point_count,
                         const BoundedCandidateExecutionPoints &points);

// Canonical identity for a bounded, contiguous window of a larger source.
// Absolute source indices and the original point count are part of the digest;
// this digest deliberately does not claim to cover points outside the window.
CanonicalRecord canonicalizeBaseSourceWindowV2(
    std::uint8_t source_kind, std::string_view frame_id,
    const builtin_interfaces::msg::Time &source_stamp,
    std::uint32_t source_generation, std::uint32_t original_point_count,
    std::uint32_t first_source_index, std::uint32_t last_source_index,
    std::uint32_t nearest_source_index,
    const BoundedCandidateExecutionPoints &points);

CanonicalRecord
canonicalizeControlPoseV1(const geometry_msgs::msg::Pose &pose,
                          const builtin_interfaces::msg::Time &stamp,
                          std::string_view frame_id, std::string_view domain);

CanonicalRecord
canonicalizeBaseSnapshotV1(const ControllerBaseTrajectorySnapshot &snapshot);

CanonicalRecord canonicalizeAuthorizedTrajectoryV1(
    const AuthorizedCartesianTrajectory &trajectory);

CanonicalRecord canonicalizeApplicationStatusV1(
    const CartesianTrajectoryApplicationStatus &status);

ValidationError
validateBaseSnapshotV1(const ControllerBaseTrajectorySnapshot &snapshot);

ValidationError
validateAuthorizedTrajectoryV1(const AuthorizedCartesianTrajectory &trajectory);

ValidationError
validateApplicationStatusV1(const CartesianTrajectoryApplicationStatus &status);

ValidationError validateTrajectoryAgainstBaseSnapshotV1(
    const ControllerBaseTrajectorySnapshot &snapshot,
    const AuthorizedCartesianTrajectory &trajectory);

ValidationError validateApplicationStatusAgainstAuthorizedTrajectoryV1(
    const AuthorizedCartesianTrajectory &trajectory,
    const CartesianTrajectoryApplicationStatus &status);

// These validators prove schema, canonical identity, and provenance only.
// ValidationError::NONE never grants motion authority and does not prove
// trackability, longitudinal reachability, collision safety, or freshness
// relative to the current ROS time.

} // namespace overtake_transport_contract::c002ay0
