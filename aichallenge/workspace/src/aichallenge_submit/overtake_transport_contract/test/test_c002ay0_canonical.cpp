#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace contract = overtake_transport_contract::c002ay0;

contract::Digest filledDigest(std::uint8_t value) {
  contract::Digest digest{};
  digest.fill(value);
  return digest;
}

std::string digestHex(const contract::Digest &digest) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : digest) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

contract::CandidateExecutionPoint point(std::int32_t index, double x_m,
                                        double yaw_rad = 0.0) {
  contract::CandidateExecutionPoint result;
  const std::uint64_t time_ns = static_cast<std::uint64_t>(index) * 25000000ULL;
  result.time_from_start.sec =
      static_cast<std::int32_t>(time_ns / 1000000000ULL);
  result.time_from_start.nanosec =
      static_cast<std::uint32_t>(time_ns % 1000000000ULL);
  result.position_x_m = x_m;
  result.orientation_z = std::sin(0.5 * yaw_rad);
  result.orientation_w = std::cos(0.5 * yaw_rad);
  result.longitudinal_velocity_mps = 1.0F;
  return result;
}

contract::OvertakePlan validFreeRunPlan() {
  contract::OvertakePlan plan;
  plan.header.frame_id = "map";
  plan.header.stamp.sec = 10;
  plan.phase = contract::OvertakePlan::FREE_RUN;
  plan.plan_generation = 7U;
  plan.decision_reason = "free_run";
  plan.safety_inputs_complete = true;
  plan.tracking_usable = true;
  plan.trajectory_publishable = true;
  plan.constraint_reason = "released";
  plan.trajectory.header.frame_id = "map";
  plan.trajectory.header.stamp = plan.header.stamp;
  plan.race_arm_epoch = 3U;
  plan.planner_instance_id = 5U;
  return plan;
}

contract::Trajectory validFreeRunReference(std::size_t point_count = 1000U) {
  contract::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 10;
  trajectory.points.reserve(point_count);
  for (std::size_t index = 0U; index < point_count; ++index) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
    const auto time_ns = static_cast<std::uint64_t>(index) * 10000000ULL;
    trajectory_point.time_from_start.sec =
        static_cast<std::int32_t>(time_ns / 1000000000ULL);
    trajectory_point.time_from_start.nanosec =
        static_cast<std::uint32_t>(time_ns % 1000000000ULL);
    trajectory_point.pose.position.x = static_cast<double>(index) * 0.05;
    trajectory_point.pose.orientation.w = 1.0;
    trajectory_point.longitudinal_velocity_mps = 3.0F;
    trajectory.points.push_back(trajectory_point);
  }
  return trajectory;
}

void setPlanKey(multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &key) {
  key.race_arm_epoch = 3U;
  key.planner_instance_id = 5U;
  key.attempt_id = 7U;
  key.target_vehicle_id = "D2";
  key.pass_direction = -1;
  key.connector_transaction_id = 11U;
  key.plan_stamp.sec = 23;
  key.plan_stamp.nanosec = 30000000U;
  key.plan_generation = 19U;
}

contract::ControllerBaseTrajectorySnapshot validSnapshot() {
  contract::ControllerBaseTrajectorySnapshot snapshot;
  snapshot.schema_version =
      contract::ControllerBaseTrajectorySnapshot::SCHEMA_V1_SHADOW;
  snapshot.authority_eligible = false;
  snapshot.record_stamp.sec = 23;
  snapshot.record_stamp.nanosec = 29000000U;
  snapshot.frame_id = "map";
  snapshot.race_arm_epoch = 3U;
  snapshot.controller_instance_id = 31U;
  snapshot.controller_sequence = 37U;
  snapshot.base_lease_id = 41U;
  snapshot.lease_valid_until.sec = 24;
  snapshot.base_source_kind =
      contract::ControllerBaseTrajectorySnapshot::SOURCE_MPC_HORIZON;
  snapshot.base_source_stamp.sec = 23;
  snapshot.base_source_stamp.nanosec = 25000000U;
  snapshot.base_source_generation = 43U;
  snapshot.base_original_point_count = 2U;
  snapshot.first_source_index = 0U;
  snapshot.last_source_index = 1U;
  snapshot.nearest_source_index = 0U;
  snapshot.base_points = {point(0, 0.0), point(1, 0.25)};
  snapshot.base_source_digest_state =
      contract::ControllerBaseTrajectorySnapshot::BASE_SOURCE_DIGEST_COMPLETE;
  snapshot.canonical_algorithm_version = 1U;
  snapshot.base_source_sha256 = filledDigest(0x21U);
  snapshot.controller_implementation_sha256 = filledDigest(0x31U);
  snapshot.controller_config_sha256 = filledDigest(0x41U);
  const auto canonical = contract::canonicalizeBaseSnapshotV1(snapshot);
  snapshot.base_geometry_sha256 = canonical.geometry_sha256;
  snapshot.snapshot_sha256 = canonical.sha256;
  return snapshot;
}

contract::AuthorizedCartesianTrajectory validTrajectory() {
  contract::AuthorizedCartesianTrajectory trajectory;
  trajectory.schema_version =
      contract::AuthorizedCartesianTrajectory::SCHEMA_V1_SHADOW;
  trajectory.authority_eligible = false;
  trajectory.plan_stamp.sec = 23;
  trajectory.plan_stamp.nanosec = 30000000U;
  trajectory.frame_id = "map";
  setPlanKey(trajectory.plan_sample_key);
  trajectory.candidate_revision = 23U;
  trajectory.authority_token = 29U;
  trajectory.candidate_type = 2U;
  trajectory.phase = 2U;
  trajectory.authorization_state =
      contract::AuthorizedCartesianTrajectory::AUTHORIZATION_AUTHORIZED;
  trajectory.source_controller_instance_id = 31U;
  trajectory.source_controller_sequence = 37U;
  trajectory.base_lease_id = 41U;
  trajectory.base_lease_valid_until.sec = 24;
  trajectory.base_source_kind =
      contract::AuthorizedCartesianTrajectory::SOURCE_MPC_HORIZON;
  trajectory.base_source_stamp.sec = 23;
  trajectory.base_source_stamp.nanosec = 25000000U;
  trajectory.base_source_generation = 43U;
  trajectory.base_original_point_count = 2U;
  trajectory.base_first_source_index = 0U;
  trajectory.base_last_source_index = 1U;
  trajectory.base_nearest_source_index = 0U;
  trajectory.base_source_digest_state =
      contract::AuthorizedCartesianTrajectory::BASE_SOURCE_DIGEST_COMPLETE;
  trajectory.canonical_algorithm_version = 1U;
  trajectory.base_geometry_sha256 = filledDigest(0x11U);
  trajectory.base_source_sha256 = filledDigest(0x21U);
  trajectory.base_snapshot_sha256 = filledDigest(0x31U);
  trajectory.points = {point(0, 0.0), point(1, 0.25)};
  trajectory.original_candidate_point_count = 2U;
  trajectory.total_arc_length_m = 0.25;
  trajectory.required_spatial_horizon_m = 0.20;
  trajectory.join_end_arc_length_m = 0.10;
  trajectory.post_join_arc_length_m = 0.15;
  trajectory.safety_snapshot_id = 47U;
  trajectory.safety_evaluation_result =
      contract::AuthorizedCartesianTrajectory::SAFETY_PASSED;
  trajectory.safety_evaluation_stamp.sec = 23;
  trajectory.safety_evaluation_stamp.nanosec = 29500000U;
  trajectory.safety_valid_until.sec = 24;
  trajectory.world_safety_snapshot_sha256 = filledDigest(0x51U);
  trajectory.safety_evaluator_implementation_sha256 = filledDigest(0x61U);
  trajectory.safety_evaluator_config_sha256 = filledDigest(0x71U);
  trajectory.controller_implementation_sha256 = filledDigest(0x31U);
  trajectory.controller_config_sha256 = filledDigest(0x41U);
  trajectory.candidate_start_control_pose.orientation.w = 1.0;
  trajectory.candidate_start_control_pose_stamp = trajectory.plan_stamp;
  const auto geometry = contract::canonicalizeGeometryV1(
      trajectory.points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  trajectory.geometry_sha256 = geometry.sha256;
  const auto canonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  const auto with_safety =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.payload_sha256 = with_safety.sha256;
  return trajectory;
}

contract::CartesianTrajectoryApplicationStatus validStatus() {
  contract::CartesianTrajectoryApplicationStatus status;
  status.schema_version =
      contract::CartesianTrajectoryApplicationStatus::SCHEMA_V1_SHADOW;
  status.authority_eligible = false;
  status.status_stamp.sec = 23;
  status.status_stamp.nanosec = 40000000U;
  status.frame_id = "map";
  status.application_state =
      contract::CartesianTrajectoryApplicationStatus::APPLICATION_APPLIED;
  status.reject_reason =
      contract::CartesianTrajectoryApplicationStatus::REJECT_NONE;
  setPlanKey(status.plan_sample_key);
  status.candidate_revision = 23U;
  status.authority_token = 29U;
  status.controller_instance_id = 31U;
  status.controller_sequence = 37U;
  status.base_lease_id = 41U;
  status.base_source_kind =
      contract::ControllerBaseTrajectorySnapshot::SOURCE_MPC_HORIZON;
  status.base_source_stamp.sec = 23;
  status.base_source_generation = 43U;
  status.expected_base_source_sha256 = filledDigest(0x11U);
  status.observed_base_source_sha256 = filledDigest(0x11U);
  status.expected_base_geometry_sha256 = filledDigest(0x21U);
  status.observed_base_geometry_sha256 = filledDigest(0x21U);
  status.expected_base_snapshot_sha256 = filledDigest(0x31U);
  status.observed_base_snapshot_sha256 = filledDigest(0x31U);
  status.expected_geometry_sha256 = filledDigest(0x41U);
  status.applied_geometry_sha256 = filledDigest(0x41U);
  status.expected_payload_sha256 = filledDigest(0x51U);
  status.observed_payload_sha256 = filledDigest(0x51U);
  status.expected_controller_implementation_sha256 = filledDigest(0x61U);
  status.observed_controller_implementation_sha256 = filledDigest(0x61U);
  status.expected_controller_config_sha256 = filledDigest(0x71U);
  status.observed_controller_config_sha256 = filledDigest(0x71U);
  status.nearest_index = 0U;
  status.lookahead_index = 1U;
  status.available_spatial_horizon_m = 0.25;
  status.required_spatial_horizon_m = 0.20;
  status.applied_control_pose.orientation.w = 1.0;
  status.applied_control_pose_stamp.sec = 23;
  status.applied_control_pose_stamp.nanosec = 35000000U;
  status.controller_command_stamp.sec = 23;
  status.controller_command_stamp.nanosec = 39000000U;
  status.command_sequence = 53U;
  const auto canonical = contract::canonicalizeApplicationStatusV1(status);
  status.applied_control_pose_sha256 = canonical.control_pose_sha256;
  return status;
}

void clearApplicationEvidence(
    contract::CartesianTrajectoryApplicationStatus &status) {
  status.applied_geometry_sha256.fill(0U);
  status.nearest_index = 0U;
  status.lookahead_index = 0U;
  status.endpoint_fallback = false;
  status.available_spatial_horizon_m = 0.0;
  status.required_spatial_horizon_m = 0.0;
  status.handoff_position_error_m = 0.0;
  status.handoff_yaw_error_rad = 0.0;
  status.applied_control_pose = geometry_msgs::msg::Pose{};
  status.applied_control_pose.orientation.w = 0.0;
  status.applied_control_pose_stamp = builtin_interfaces::msg::Time{};
  status.applied_control_pose_sha256.fill(0U);
  status.controller_command_stamp = builtin_interfaces::msg::Time{};
  status.command_sequence = 0U;
}

contract::CartesianTrajectoryApplicationStatus validWarmedStatus() {
  auto status = validStatus();
  status.application_state =
      contract::CartesianTrajectoryApplicationStatus::APPLICATION_WARMED;
  clearApplicationEvidence(status);
  return status;
}

void recanonicalizeTrajectory(
    contract::AuthorizedCartesianTrajectory &trajectory) {
  const auto canonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(canonical.valid());
  trajectory.geometry_sha256 = canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  trajectory.payload_sha256 = canonical.sha256;
}

contract::AuthorizedCartesianTrajectory
boundTrajectory(const contract::ControllerBaseTrajectorySnapshot &snapshot) {
  auto trajectory = validTrajectory();
  trajectory.source_controller_instance_id = snapshot.controller_instance_id;
  trajectory.source_controller_sequence = snapshot.controller_sequence;
  trajectory.base_lease_id = snapshot.base_lease_id;
  trajectory.base_lease_valid_until = snapshot.lease_valid_until;
  trajectory.base_source_kind = snapshot.base_source_kind;
  trajectory.base_source_stamp = snapshot.base_source_stamp;
  trajectory.base_source_generation = snapshot.base_source_generation;
  trajectory.base_original_point_count = snapshot.base_original_point_count;
  trajectory.base_first_source_index = snapshot.first_source_index;
  trajectory.base_last_source_index = snapshot.last_source_index;
  trajectory.base_nearest_source_index = snapshot.nearest_source_index;
  trajectory.base_source_digest_state = snapshot.base_source_digest_state;
  trajectory.canonical_algorithm_version = snapshot.canonical_algorithm_version;
  trajectory.base_geometry_sha256 = snapshot.base_geometry_sha256;
  trajectory.base_source_sha256 = snapshot.base_source_sha256;
  trajectory.base_snapshot_sha256 = snapshot.snapshot_sha256;
  trajectory.controller_implementation_sha256 =
      snapshot.controller_implementation_sha256;
  trajectory.controller_config_sha256 = snapshot.controller_config_sha256;
  recanonicalizeTrajectory(trajectory);
  return trajectory;
}

contract::CartesianTrajectoryApplicationStatus
boundStatus(const contract::AuthorizedCartesianTrajectory &trajectory,
            const bool applied) {
  auto status = validStatus();
  status.frame_id = trajectory.frame_id;
  status.plan_sample_key = trajectory.plan_sample_key;
  status.candidate_revision = trajectory.candidate_revision;
  status.authority_token = trajectory.authority_token;
  status.controller_instance_id = trajectory.source_controller_instance_id;
  status.controller_sequence = trajectory.source_controller_sequence;
  status.base_lease_id = trajectory.base_lease_id;
  status.base_source_kind = trajectory.base_source_kind;
  status.base_source_stamp = trajectory.base_source_stamp;
  status.base_source_generation = trajectory.base_source_generation;
  status.expected_base_source_sha256 = trajectory.base_source_sha256;
  status.observed_base_source_sha256 = trajectory.base_source_sha256;
  status.expected_base_geometry_sha256 = trajectory.base_geometry_sha256;
  status.observed_base_geometry_sha256 = trajectory.base_geometry_sha256;
  status.expected_base_snapshot_sha256 = trajectory.base_snapshot_sha256;
  status.observed_base_snapshot_sha256 = trajectory.base_snapshot_sha256;
  status.expected_geometry_sha256 = trajectory.geometry_sha256;
  status.applied_geometry_sha256 = trajectory.geometry_sha256;
  status.expected_payload_sha256 = trajectory.payload_sha256;
  status.observed_payload_sha256 = trajectory.payload_sha256;
  status.expected_controller_implementation_sha256 =
      trajectory.controller_implementation_sha256;
  status.observed_controller_implementation_sha256 =
      trajectory.controller_implementation_sha256;
  status.expected_controller_config_sha256 =
      trajectory.controller_config_sha256;
  status.observed_controller_config_sha256 =
      trajectory.controller_config_sha256;
  if (!applied) {
    status.application_state =
        contract::CartesianTrajectoryApplicationStatus::APPLICATION_WARMED;
    clearApplicationEvidence(status);
  }
  return status;
}

TEST(C002Ay0Canonical, PublishedSha256VectorMatches) {
  const std::vector<std::uint8_t> abc{'a', 'b', 'c'};
  const contract::Digest expected{
      0xbaU, 0x78U, 0x16U, 0xbfU, 0x8fU, 0x01U, 0xcfU, 0xeaU,
      0x41U, 0x41U, 0x40U, 0xdeU, 0x5dU, 0xaeU, 0x22U, 0x23U,
      0xb0U, 0x03U, 0x61U, 0xa3U, 0x96U, 0x17U, 0x7aU, 0x9cU,
      0xb4U, 0x10U, 0xffU, 0x61U, 0xf2U, 0x00U, 0x15U, 0xadU};
  EXPECT_EQ(contract::sha256(abc), expected);
}

TEST(C002Ay0Canonical, ValidShadowRecordsCanonicalizeAndValidate) {
  const auto snapshot = validSnapshot();
  const auto trajectory = validTrajectory();
  const auto warmed = validWarmedStatus();

  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::NONE);
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::NONE);
  EXPECT_EQ(contract::validateApplicationStatusV1(warmed),
            contract::ValidationError::NONE);
}

TEST(C002Ay0Canonical, ShadowSchemaRejectsAppliedState) {
  const auto applied = validStatus();

  EXPECT_EQ(contract::validateApplicationStatusV1(applied),
            contract::ValidationError::APPLICATION_STATUS_INVALID);
  EXPECT_EQ(contract::canonicalizeApplicationStatusV1(applied).error,
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  auto unsupported = applied;
  ++unsupported.schema_version;
  EXPECT_EQ(contract::validateApplicationStatusV1(unsupported),
            contract::ValidationError::SCHEMA_UNSUPPORTED);

  auto invalid_enum = applied;
  invalid_enum.reject_reason = std::numeric_limits<std::uint8_t>::max();
  EXPECT_EQ(contract::validateApplicationStatusV1(invalid_enum),
            contract::ValidationError::ENUM_INVALID);
}

TEST(C002Ay0Canonical, CanonicalGoldenRecordsRemainStable) {
  const auto snapshot = validSnapshot();
  const auto trajectory = validTrajectory();
  const auto status = validWarmedStatus();

  EXPECT_EQ(digestHex(snapshot.snapshot_sha256),
            "0194eeae6b20bd4ccf95d9d8daf82e99d21dd78e1222b7769b3b90a8394acd26");
  EXPECT_EQ(digestHex(trajectory.safety_proof_sha256),
            "82258f0da16bb90195721e9e35dcdd32ad2f76d619926f17d7ef04797bb123e9");
  EXPECT_EQ(digestHex(trajectory.candidate_start_control_pose_sha256),
            "5d398afcbaa438ec3d630c1e64f950318badd9a97c0cb41369c7281f52da4730");
  EXPECT_EQ(digestHex(trajectory.payload_sha256),
            "713c6cf27d4fb1f1d53957ed5a2668c0bf96d762842670a3bd97c4467d97b5e6");
  EXPECT_EQ(digestHex(status.applied_control_pose_sha256),
            "0000000000000000000000000000000000000000000000000000000000000000");
  EXPECT_EQ(digestHex(contract::canonicalizeApplicationStatusV1(status).sha256),
            "016713fc27d0173afba97030ad550b8dc02daeec820d40b18de1e38cbec8161c");
}

TEST(C002Ay0Canonical, PhaseZeroRejectsAuthorityEligible) {
  auto snapshot = validSnapshot();
  snapshot.authority_eligible = true;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::AUTHORITY_FLAG_INVALID);

  auto trajectory = validTrajectory();
  trajectory.authority_eligible = true;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::AUTHORITY_FLAG_INVALID);

  auto status = validStatus();
  status.authority_eligible = true;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::AUTHORITY_FLAG_INVALID);
}

TEST(C002Ay0Canonical, GeometryAccepts256AndRejects257Points) {
  std::vector<contract::CandidateExecutionPoint> points;
  points.reserve(257U);
  for (std::size_t i = 0U; i < 256U; ++i) {
    points.push_back(
        point(static_cast<std::int32_t>(i), 0.25 * static_cast<double>(i)));
  }
  EXPECT_TRUE(
      contract::canonicalizeGeometryV1(points, "C002AY0_AUTHORIZED_GEOMETRY_V1")
          .valid());

  points.push_back(point(256, 64.0));
  EXPECT_EQ(
      contract::canonicalizeGeometryV1(points, "C002AY0_AUTHORIZED_GEOMETRY_V1")
          .error,
      contract::ValidationError::GEOMETRY_POINT_COUNT_INVALID);

  contract::ControllerBaseTrajectorySnapshot bounded;
  for (std::size_t index = 0U; index < 256U; ++index) {
    bounded.base_points.push_back(point(static_cast<std::int32_t>(index),
                                        0.25 * static_cast<double>(index)));
  }
  EXPECT_EQ(bounded.base_points.size(), 256U);
  EXPECT_THROW(bounded.base_points.push_back(point(256, 64.0)),
               std::length_error);
}

TEST(C002Ay0Canonical, GeometryRejectsHardBoundNextValues) {
  auto points = std::vector<contract::CandidateExecutionPoint>{point(0, 0.0),
                                                               point(1, 0.25)};
  points[1].longitudinal_velocity_mps = 10.0F;
  EXPECT_TRUE(contract::canonicalizeGeometryV1(points, "speed-bound").valid());
  points[1].longitudinal_velocity_mps =
      std::nextafter(10.0F, std::numeric_limits<float>::infinity());
  EXPECT_EQ(contract::canonicalizeGeometryV1(points, "speed-bound").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);

  points = {point(0, 0.0),
            point(1, std::nextafter(contract::kMaxTransportArcSpacingM,
                                    std::numeric_limits<double>::infinity()))};
  EXPECT_EQ(contract::canonicalizeGeometryV1(points, "spacing-bound").error,
            contract::ValidationError::GEOMETRY_ARC_SPACING_EXCEEDED);

  points = {point(0, 0.0), point(1, 0.25, 0.05)};
  EXPECT_TRUE(contract::canonicalizeGeometryV1(points, "yaw-bound").valid());
  points[1] = point(
      1, 0.25, std::nextafter(0.05, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(contract::canonicalizeGeometryV1(points, "yaw-bound").error,
            contract::ValidationError::GEOMETRY_YAW_STEP_EXCEEDED);

  points.clear();
  const double length_m =
      std::nextafter(contract::kMaxTransportArcLengthM,
                     std::numeric_limits<double>::infinity());
  for (std::size_t index = 0U; index < 256U; ++index) {
    points.push_back(point(static_cast<std::int32_t>(index),
                           length_m * static_cast<double>(index) / 255.0));
  }
  EXPECT_FALSE(contract::canonicalizeGeometryV1(points, "arc-bound").valid());
}

TEST(C002Ay0Canonical, GeometryRejectsInvalidCrossFieldContracts) {
  auto points = std::vector<contract::CandidateExecutionPoint>{point(0, 0.0),
                                                               point(1, 0.25)};

  auto changed = points;
  changed[1].time_from_start.nanosec = 0U;
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_TIME_NONMONOTONIC);

  changed = points;
  changed[1].position_x_m = 0.250000001;
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_ARC_SPACING_EXCEEDED);

  changed = points;
  changed[1].orientation_z = std::sin(0.5 * 0.051);
  changed[1].orientation_w = std::cos(0.5 * 0.051);
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_YAW_STEP_EXCEEDED);

  changed = points;
  changed[1].longitudinal_velocity_mps = -0.01F;
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);

  changed = points;
  changed[1].orientation_w = 2.0;
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);

  changed = points;
  changed[1].position_x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);

  changed = points;
  changed[1].position_x_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);

  changed = points;
  changed[1].time_from_start.nanosec = 1000000000U;
  EXPECT_EQ(contract::canonicalizeGeometryV1(changed, "domain").error,
            contract::ValidationError::GEOMETRY_POINT_INVALID);
}

TEST(C002Ay0Canonical, ClassifiesPointInvalidFieldWithoutChangingValidation) {
  auto candidate = point(1, 0.25);
  EXPECT_EQ(contract::classifyPointInvalidFieldV1(candidate),
            contract::PointInvalidField::NONE);

  candidate.position_x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(contract::classifyPointInvalidFieldV1(candidate),
            contract::PointInvalidField::POSITION_X);
  const auto invalid_points =
      std::vector<contract::CandidateExecutionPoint>{point(0, 0.0), candidate};
  EXPECT_EQ(
      contract::canonicalizeGeometryV1(invalid_points, "point-field").error,
      contract::ValidationError::GEOMETRY_POINT_INVALID);

  candidate = point(1, 0.25);
  candidate.orientation_w = 2.0;
  EXPECT_EQ(contract::classifyPointInvalidFieldV1(candidate),
            contract::PointInvalidField::ORIENTATION_NORM);

  candidate = point(1, 0.25);
  candidate.longitudinal_velocity_mps = -0.01F;
  EXPECT_EQ(contract::classifyPointInvalidFieldV1(candidate),
            contract::PointInvalidField::LONGITUDINAL_VELOCITY_RANGE);

  candidate = point(1, 0.25);
  candidate.time_from_start.nanosec = 1000000000U;
  EXPECT_EQ(contract::classifyPointInvalidFieldV1(candidate),
            contract::PointInvalidField::TIME_FROM_START);
}

TEST(C002Ay0Canonical, FullSourceDigestAndEmbeddedDigestsAreMandatory) {
  auto snapshot = validSnapshot();
  snapshot.base_source_digest_state = contract::
      ControllerBaseTrajectorySnapshot::BASE_SOURCE_DIGEST_INDETERMINATE;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::DIGEST_MISSING);

  snapshot = validSnapshot();
  snapshot.base_source_sha256.fill(0U);
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::DIGEST_MISSING);

  snapshot = validSnapshot();
  snapshot.base_geometry_sha256[0] ^= 0x01U;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::DIGEST_MISMATCH);
}

TEST(C002Ay0Canonical, SourceWindowAndLeaseBoundariesFailClosed) {
  auto snapshot = validSnapshot();
  snapshot.last_source_index = 0U;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::SOURCE_WINDOW_INVALID);

  snapshot = validSnapshot();
  snapshot.lease_valid_until = snapshot.record_stamp;
  EXPECT_EQ(contract::validateBaseSnapshotV1(snapshot),
            contract::ValidationError::LEASE_INVALID);

  auto trajectory = validTrajectory();
  trajectory.base_original_point_count = 3U;
  trajectory.base_last_source_index = 2U;
  const auto recanonical =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.safety_proof_sha256 = recanonical.safety_proof_sha256;
  trajectory.payload_sha256 = recanonical.sha256;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::NONE);

  trajectory = validTrajectory();
  trajectory.safety_valid_until = trajectory.safety_evaluation_stamp;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::LEASE_INVALID);
}

TEST(C002Ay0Canonical, AuthorizedTrajectoryMustMatchExactBaseSnapshot) {
  const auto snapshot = validSnapshot();
  const auto original = boundTrajectory(snapshot);
  ASSERT_EQ(
      contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot, original),
      contract::ValidationError::NONE);

  const auto expect_rejected =
      [&snapshot, &original](
          const std::function<void(contract::AuthorizedCartesianTrajectory &)>
              &mutate) {
        auto changed = original;
        mutate(changed);
        recanonicalizeTrajectory(changed);
        EXPECT_NE(contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot,
                                                                    changed),
                  contract::ValidationError::NONE);
      };
  expect_rejected([](auto &value) { ++value.source_controller_instance_id; });
  expect_rejected([](auto &value) { ++value.source_controller_sequence; });
  expect_rejected([](auto &value) { ++value.base_lease_id; });
  expect_rejected([](auto &value) { ++value.base_lease_valid_until.nanosec; });
  expect_rejected([](auto &value) {
    value.base_source_kind =
        contract::AuthorizedCartesianTrajectory::SOURCE_REFERENCE_TRAJECTORY;
  });
  expect_rejected([](auto &value) { ++value.base_source_stamp.nanosec; });
  expect_rejected([](auto &value) { ++value.base_source_generation; });
  expect_rejected([](auto &value) {
    value.base_original_point_count = 3U;
    value.base_last_source_index = 2U;
  });
  expect_rejected([](auto &value) { ++value.base_nearest_source_index; });
  expect_rejected([](auto &value) { value.base_geometry_sha256[0] ^= 0x01U; });
  expect_rejected([](auto &value) { value.base_source_sha256[0] ^= 0x01U; });
  expect_rejected([](auto &value) { value.base_snapshot_sha256[0] ^= 0x01U; });
  expect_rejected(
      [](auto &value) { value.controller_implementation_sha256[0] ^= 0x01U; });
  expect_rejected(
      [](auto &value) { value.controller_config_sha256[0] ^= 0x01U; });
  expect_rejected([](auto &value) { ++value.plan_sample_key.race_arm_epoch; });

  auto before_snapshot = original;
  before_snapshot.plan_stamp.nanosec = 28000000U;
  before_snapshot.plan_sample_key.plan_stamp = before_snapshot.plan_stamp;
  before_snapshot.candidate_start_control_pose_stamp =
      before_snapshot.plan_stamp;
  before_snapshot.safety_evaluation_stamp.nanosec = 27000000U;
  recanonicalizeTrajectory(before_snapshot);
  EXPECT_EQ(contract::validateTrajectoryAgainstBaseSnapshotV1(snapshot,
                                                              before_snapshot),
            contract::ValidationError::STAMP_INVALID);
}

TEST(C002Ay0Canonical, ApplicationStatusMustMatchExactAuthorizedTrajectory) {
  const auto snapshot = validSnapshot();
  const auto trajectory = boundTrajectory(snapshot);
  const auto applied = boundStatus(trajectory, true);
  const auto warmed = boundStatus(trajectory, false);

  ASSERT_EQ(contract::validateApplicationStatusAgainstAuthorizedTrajectoryV1(
                trajectory, applied),
            contract::ValidationError::APPLICATION_STATUS_INVALID);
  ASSERT_EQ(contract::validateApplicationStatusAgainstAuthorizedTrajectoryV1(
                trajectory, warmed),
            contract::ValidationError::NONE);

  const auto expect_rejected =
      [&trajectory, &warmed](
          const std::function<void(
              contract::CartesianTrajectoryApplicationStatus &)> &mutate) {
        auto changed = warmed;
        mutate(changed);
        ASSERT_EQ(contract::validateApplicationStatusV1(changed),
                  contract::ValidationError::NONE);
        EXPECT_EQ(
            contract::validateApplicationStatusAgainstAuthorizedTrajectoryV1(
                trajectory, changed),
            contract::ValidationError::DIGEST_MISMATCH);
      };
  expect_rejected([](auto &value) { ++value.plan_sample_key.plan_generation; });
  expect_rejected([](auto &value) { ++value.candidate_revision; });
  expect_rejected([](auto &value) { ++value.authority_token; });
  expect_rejected([](auto &value) { ++value.controller_instance_id; });
  expect_rejected([](auto &value) { ++value.controller_sequence; });
  expect_rejected([](auto &value) { ++value.base_lease_id; });
  expect_rejected([](auto &value) {
    value.base_source_kind =
        contract::ControllerBaseTrajectorySnapshot::SOURCE_REFERENCE_TRAJECTORY;
  });
  expect_rejected([](auto &value) { ++value.base_source_stamp.nanosec; });
  expect_rejected([](auto &value) { ++value.base_source_generation; });
  expect_rejected([](auto &value) {
    value.expected_base_source_sha256[0] ^= 0x01U;
    value.observed_base_source_sha256 = value.expected_base_source_sha256;
  });
  expect_rejected([](auto &value) {
    value.expected_base_geometry_sha256[0] ^= 0x01U;
    value.observed_base_geometry_sha256 = value.expected_base_geometry_sha256;
  });
  expect_rejected([](auto &value) {
    value.expected_base_snapshot_sha256[0] ^= 0x01U;
    value.observed_base_snapshot_sha256 = value.expected_base_snapshot_sha256;
  });
  expect_rejected(
      [](auto &value) { value.expected_geometry_sha256[0] ^= 0x01U; });
  expect_rejected([](auto &value) {
    value.expected_payload_sha256[0] ^= 0x01U;
    value.observed_payload_sha256 = value.expected_payload_sha256;
  });
  expect_rejected([](auto &value) {
    value.expected_controller_implementation_sha256[0] ^= 0x01U;
    value.observed_controller_implementation_sha256 =
        value.expected_controller_implementation_sha256;
  });
  expect_rejected([](auto &value) {
    value.expected_controller_config_sha256[0] ^= 0x01U;
    value.observed_controller_config_sha256 =
        value.expected_controller_config_sha256;
  });
}

TEST(C002Ay0Canonical, SafetyProofAndPayloadBindAllActuationFields) {
  const auto original = validTrajectory();
  auto changed = original;
  changed.points[1].front_wheel_angle_rad = 0.01F;
  const auto changed_geometry = contract::canonicalizeGeometryV1(
      changed.points, "C002AY0_AUTHORIZED_GEOMETRY_V1");
  changed.geometry_sha256 = changed_geometry.sha256;
  const auto canonical = contract::canonicalizeAuthorizedTrajectoryV1(changed);
  changed.safety_proof_sha256 = canonical.safety_proof_sha256;
  const auto with_safety =
      contract::canonicalizeAuthorizedTrajectoryV1(changed);
  EXPECT_NE(with_safety.sha256, original.payload_sha256);

  changed = original;
  changed.safety_valid_until.nanosec = 1U;
  const auto changed_safety =
      contract::canonicalizeAuthorizedTrajectoryV1(changed);
  EXPECT_NE(changed_safety.safety_proof_sha256, original.safety_proof_sha256);
}

TEST(C002Ay0Canonical, ControlPoseDigestBindsPoseStampAndFrame) {
  auto trajectory = validTrajectory();
  const auto original_pose_digest =
      trajectory.candidate_start_control_pose_sha256;

  trajectory.candidate_start_control_pose.position.x += 0.001;
  const auto changed_position =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(changed_position.valid());
  trajectory.geometry_sha256 = changed_position.geometry_sha256;
  trajectory.safety_proof_sha256 = changed_position.safety_proof_sha256;
  trajectory.payload_sha256 = changed_position.sha256;
  trajectory.candidate_start_control_pose_sha256 = original_pose_digest;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::DIGEST_MISMATCH);

  trajectory = validTrajectory();
  ++trajectory.plan_stamp.nanosec;
  trajectory.plan_sample_key.plan_stamp = trajectory.plan_stamp;
  trajectory.candidate_start_control_pose_stamp = trajectory.plan_stamp;
  const auto changed_stamp =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(changed_stamp.valid());
  trajectory.geometry_sha256 = changed_stamp.geometry_sha256;
  trajectory.safety_proof_sha256 = changed_stamp.safety_proof_sha256;
  trajectory.payload_sha256 = changed_stamp.sha256;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::DIGEST_MISMATCH);

  trajectory = validTrajectory();
  trajectory.candidate_start_control_pose.orientation.z = std::sin(0.0005);
  trajectory.candidate_start_control_pose.orientation.w = std::cos(0.0005);
  const auto changed_yaw =
      contract::canonicalizeAuthorizedTrajectoryV1(trajectory);
  ASSERT_TRUE(changed_yaw.valid());
  trajectory.geometry_sha256 = changed_yaw.geometry_sha256;
  trajectory.safety_proof_sha256 = changed_yaw.safety_proof_sha256;
  trajectory.payload_sha256 = changed_yaw.sha256;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::DIGEST_MISMATCH);

  auto status = validStatus();
  status.frame_id = "odom";
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::FRAME_INVALID);
}

TEST(C002Ay0Canonical, GeometryDigestBindsEveryActuationFieldBitPattern) {
  const std::vector<contract::CandidateExecutionPoint> original{point(0, 0.0),
                                                                point(1, 0.25)};
  const auto original_digest =
      contract::canonicalizeGeometryV1(original, "field-binding").sha256;
  const auto expect_changed =
      [&original, &original_digest](
          const std::function<void(
              std::vector<contract::CandidateExecutionPoint> &)> &mutate) {
        auto changed = original;
        mutate(changed);
        const auto canonical =
            contract::canonicalizeGeometryV1(changed, "field-binding");
        ASSERT_TRUE(canonical.valid());
        EXPECT_NE(canonical.sha256, original_digest);
      };

  expect_changed([](auto &points) { points[1].time_from_start.nanosec += 1U; });
  expect_changed([](auto &points) { points[0].position_x_m += 0.001; });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.position_y_m += 0.001;
    }
  });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.position_z_m += 0.001;
    }
  });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.orientation_x = std::sin(0.0005);
      point.orientation_w = std::cos(0.0005);
    }
  });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.orientation_y = std::sin(0.0005);
      point.orientation_w = std::cos(0.0005);
    }
  });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.orientation_z = std::sin(0.0005);
      point.orientation_w = std::cos(0.0005);
    }
  });
  expect_changed([](auto &points) {
    for (auto &point : points) {
      point.orientation_w = -1.0;
    }
  });
  expect_changed(
      [](auto &points) { points[1].longitudinal_velocity_mps = 1.001F; });
  expect_changed([](auto &points) { points[1].lateral_velocity_mps = -0.0F; });
  expect_changed([](auto &points) {
    points[1].acceleration_mps2 = std::numeric_limits<float>::denorm_min();
  });
  expect_changed([](auto &points) { points[1].heading_rate_rps = 0.001F; });
  expect_changed(
      [](auto &points) { points[1].front_wheel_angle_rad = 0.001F; });
  expect_changed([](auto &points) { points[1].rear_wheel_angle_rad = 0.001F; });
}

TEST(C002Ay0Canonical, ApplicationStatusRequiresExactObservedDigests) {
  auto status = validWarmedStatus();
  status.observed_payload_sha256[0] ^= 0x01U;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::DIGEST_MISMATCH);

  status = validStatus();
  status.endpoint_fallback = true;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validWarmedStatus();
  status.command_sequence = 1U;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validWarmedStatus();
  status.applied_control_pose.position.x = 0.001;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validWarmedStatus();
  status.controller_command_stamp.sec = 1;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validWarmedStatus();
  status.required_spatial_horizon_m = 0.001;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validStatus();
  status.command_sequence = 0U;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);

  status = validStatus();
  status.application_state =
      contract::CartesianTrajectoryApplicationStatus::APPLICATION_INVALIDATED;
  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::APPLICATION_STATUS_INVALID);
}

TEST(C002Ay0Canonical, RejectedStatusPreservesFirstFalseEvidence) {
  auto status = validStatus();
  status.application_state =
      contract::CartesianTrajectoryApplicationStatus::APPLICATION_REJECTED;
  status.reject_reason = contract::CartesianTrajectoryApplicationStatus::
      REJECT_BASE_BINDING_MISMATCH;
  status.observed_base_source_sha256.fill(0U);
  status.observed_base_geometry_sha256.fill(0U);
  status.observed_base_snapshot_sha256.fill(0U);
  status.observed_payload_sha256.fill(0U);
  status.observed_controller_implementation_sha256.fill(0U);
  status.observed_controller_config_sha256.fill(0U);
  clearApplicationEvidence(status);

  EXPECT_EQ(contract::validateApplicationStatusV1(status),
            contract::ValidationError::NONE);
}

TEST(C002Ay0Canonical, AuthorizedTrajectoryRejectsUndefinedClosedEnums) {
  auto trajectory = validTrajectory();
  trajectory.candidate_type =
      contract::AuthorizedCartesianTrajectory::CANDIDATE_SAFE_STOP + 1U;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::ENUM_INVALID);

  trajectory = validTrajectory();
  trajectory.phase =
      contract::AuthorizedCartesianTrajectory::PHASE_ABORT_HOLD + 1U;
  EXPECT_EQ(contract::validateAuthorizedTrajectoryV1(trajectory),
            contract::ValidationError::ENUM_INVALID);
}

TEST(C002Ay0Canonical, FreeRunPlanDigestIgnoresOnlyDeliveryIdentity) {
  const auto baseline = validFreeRunPlan();
  const auto first = contract::canonicalizeFreeRunPlanV1(baseline);
  ASSERT_TRUE(first.valid());

  auto next_delivery = baseline;
  next_delivery.header.stamp.sec = 11;
  next_delivery.trajectory.header.stamp = next_delivery.header.stamp;
  next_delivery.plan_generation = 8U;
  const auto second = contract::canonicalizeFreeRunPlanV1(next_delivery);
  ASSERT_TRUE(second.valid());
  EXPECT_EQ(first.sha256, second.sha256);

  auto semantic_change = baseline;
  semantic_change.decision_reason = "changed";
  const auto changed = contract::canonicalizeFreeRunPlanV1(semantic_change);
  ASSERT_TRUE(changed.valid());
  EXPECT_NE(first.sha256, changed.sha256);

  auto lateral_mode = baseline;
  lateral_mode.phase = contract::OvertakePlan::PASSING;
  EXPECT_FALSE(contract::canonicalizeFreeRunPlanV1(lateral_mode).valid());
}

TEST(C002Ay0Canonical, FreeRunPlanLiveIdentityIsSelfVerifying) {
  auto plan = validFreeRunPlan();
  ASSERT_TRUE(contract::populateFreeRunPlanIdentityV1(plan));
  EXPECT_EQ(plan.free_run_canonical_algorithm_version,
            contract::OvertakePlan::FREE_RUN_CANONICAL_ALGORITHM_V1);
  EXPECT_TRUE(contract::validateFreeRunPlanIdentityV1(plan));

  auto next_delivery = plan;
  next_delivery.header.stamp.sec += 1;
  next_delivery.trajectory.header.stamp = next_delivery.header.stamp;
  next_delivery.plan_generation += 1U;
  EXPECT_TRUE(contract::validateFreeRunPlanIdentityV1(next_delivery));

  auto mutated = plan;
  mutated.decision_reason = "mutated";
  EXPECT_FALSE(contract::validateFreeRunPlanIdentityV1(mutated));

  auto non_free_run = plan;
  non_free_run.phase = contract::OvertakePlan::PASSING;
  EXPECT_FALSE(contract::populateFreeRunPlanIdentityV1(non_free_run));
  EXPECT_EQ(non_free_run.free_run_canonical_algorithm_version, 0U);
  EXPECT_TRUE(
      std::all_of(non_free_run.free_run_canonical_payload_sha256.begin(),
                  non_free_run.free_run_canonical_payload_sha256.end(),
                  [](std::uint8_t value) { return value == 0U; }));
}

TEST(C002Ay0Canonical, FreeRunReferenceDigestCoversFullSource) {
  const auto baseline = validFreeRunReference();
  const auto first = contract::canonicalizeFreeRunReferenceV1(baseline);
  ASSERT_TRUE(first.valid());

  auto next_delivery = baseline;
  next_delivery.header.stamp.sec = 11;
  const auto second = contract::canonicalizeFreeRunReferenceV1(next_delivery);
  ASSERT_TRUE(second.valid());
  EXPECT_EQ(first.sha256, second.sha256);

  auto changed = baseline;
  changed.points[999].pose.position.y = 0.001;
  const auto changed_result = contract::canonicalizeFreeRunReferenceV1(changed);
  ASSERT_TRUE(changed_result.valid());
  EXPECT_NE(first.sha256, changed_result.sha256);

  auto nonfinite = baseline;
  nonfinite.points[500].pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(contract::canonicalizeFreeRunReferenceV1(nonfinite).valid());

  EXPECT_FALSE(
      contract::canonicalizeFreeRunReferenceV1(
          validFreeRunReference(contract::kMaxFreeRunReferencePoints + 1U))
          .valid());
}

} // namespace
