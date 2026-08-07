#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "overtake_transport_contract/c002ay0_canonical.hpp"

namespace overtake_transport_contract::c002ay0::test_fixture {

inline Digest filledDigest(const std::uint8_t seed) {
  Digest digest{};
  for (std::size_t index = 0U; index < digest.size(); ++index) {
    digest[index] = static_cast<std::uint8_t>(seed + index);
  }
  return digest;
}

inline CandidateExecutionPoint point(const std::size_t index) {
  CandidateExecutionPoint value;
  const double arc_m = kMaxTransportArcSpacingM * static_cast<double>(index);
  const std::uint64_t time_ns = static_cast<std::uint64_t>(index) * 25000000ULL;
  value.time_from_start.sec =
      static_cast<std::int32_t>(time_ns / 1000000000ULL);
  value.time_from_start.nanosec =
      static_cast<std::uint32_t>(time_ns % 1000000000ULL);
  value.position_x_m = arc_m;
  value.orientation_w = 1.0;
  value.longitudinal_velocity_mps =
      static_cast<float>(kExecutionSpeedCeilingMps);
  return value;
}

inline void setPlanKey(multi_purpose_mpc_ros_msgs::msg::PlanSampleKey &key,
                       const builtin_interfaces::msg::Time &stamp) {
  key.race_arm_epoch = 7U;
  key.planner_instance_id = 11U;
  key.attempt_id = 13U;
  key.target_vehicle_id = std::string(64U, 'd');
  key.pass_direction = 1;
  key.connector_transaction_id = 17U;
  key.plan_stamp = stamp;
  key.plan_generation = 19U;
}

inline ControllerBaseTrajectorySnapshot
baseSnapshot(const std::size_t point_count = kMaxCartesianPoints) {
  ControllerBaseTrajectorySnapshot snapshot;
  snapshot.schema_version = ControllerBaseTrajectorySnapshot::SCHEMA_V1_SHADOW;
  snapshot.authority_eligible = false;
  snapshot.record_stamp.sec = 100;
  snapshot.record_stamp.nanosec = 20000000U;
  snapshot.frame_id = "map";
  snapshot.race_arm_epoch = 7U;
  snapshot.controller_instance_id = 23U;
  snapshot.controller_sequence = 29U;
  snapshot.base_lease_id = 31U;
  snapshot.lease_valid_until.sec = 101;
  snapshot.base_source_kind =
      ControllerBaseTrajectorySnapshot::SOURCE_MPC_HORIZON;
  snapshot.base_source_stamp.sec = 100;
  snapshot.base_source_generation = 37U;
  snapshot.base_original_point_count = static_cast<std::uint32_t>(point_count);
  snapshot.first_source_index = 0U;
  snapshot.last_source_index = static_cast<std::uint32_t>(point_count - 1U);
  snapshot.nearest_source_index = 0U;
  snapshot.base_points.reserve(kMaxCartesianPoints);
  for (std::size_t index = 0U; index < point_count; ++index) {
    snapshot.base_points.push_back(point(index));
  }
  snapshot.base_source_digest_state =
      ControllerBaseTrajectorySnapshot::BASE_SOURCE_DIGEST_COMPLETE;
  snapshot.canonical_algorithm_version = 1U;
  snapshot.base_source_sha256 = filledDigest(0x11U);
  snapshot.controller_implementation_sha256 = filledDigest(0x31U);
  snapshot.controller_config_sha256 = filledDigest(0x51U);
  const auto canonical = canonicalizeBaseSnapshotV1(snapshot);
  snapshot.base_geometry_sha256 = canonical.geometry_sha256;
  snapshot.snapshot_sha256 = canonical.sha256;
  return snapshot;
}

inline AuthorizedCartesianTrajectory
authorizedTrajectory(const ControllerBaseTrajectorySnapshot &snapshot) {
  AuthorizedCartesianTrajectory trajectory;
  trajectory.schema_version = AuthorizedCartesianTrajectory::SCHEMA_V1_SHADOW;
  trajectory.authority_eligible = false;
  trajectory.plan_stamp.sec = 100;
  trajectory.plan_stamp.nanosec = 30000000U;
  trajectory.frame_id = "map";
  setPlanKey(trajectory.plan_sample_key, trajectory.plan_stamp);
  trajectory.candidate_revision = 41U;
  trajectory.authority_token = 43U;
  trajectory.candidate_type =
      AuthorizedCartesianTrajectory::CANDIDATE_PASS_LEFT;
  trajectory.phase = AuthorizedCartesianTrajectory::PHASE_PASSING;
  trajectory.authorization_state =
      AuthorizedCartesianTrajectory::AUTHORIZATION_AUTHORIZED;
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
  trajectory.points.reserve(kMaxCartesianPoints);
  for (const auto &value : snapshot.base_points) {
    trajectory.points.push_back(value);
  }
  trajectory.original_candidate_point_count =
      static_cast<std::uint32_t>(trajectory.points.size());
  trajectory.total_arc_length_m =
      kMaxTransportArcSpacingM *
      static_cast<double>(trajectory.points.size() - 1U);
  trajectory.required_spatial_horizon_m =
      std::min(61.23, trajectory.total_arc_length_m);
  trajectory.join_end_arc_length_m =
      std::min(8.75, trajectory.total_arc_length_m);
  trajectory.post_join_arc_length_m =
      trajectory.total_arc_length_m - trajectory.join_end_arc_length_m;
  trajectory.safety_snapshot_id = 47U;
  trajectory.safety_evaluation_result =
      AuthorizedCartesianTrajectory::SAFETY_PASSED;
  trajectory.safety_evaluation_stamp.sec = 100;
  trajectory.safety_evaluation_stamp.nanosec = 25000000U;
  trajectory.safety_valid_until.sec = 101;
  trajectory.world_safety_snapshot_sha256 = filledDigest(0x61U);
  trajectory.safety_evaluator_implementation_sha256 = filledDigest(0x81U);
  trajectory.safety_evaluator_config_sha256 = filledDigest(0xa1U);
  trajectory.controller_implementation_sha256 =
      snapshot.controller_implementation_sha256;
  trajectory.controller_config_sha256 = snapshot.controller_config_sha256;
  trajectory.candidate_start_control_pose.orientation.w = 1.0;
  trajectory.candidate_start_control_pose_stamp = trajectory.plan_stamp;
  auto canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.geometry_sha256 = canonical.geometry_sha256;
  trajectory.safety_proof_sha256 = canonical.safety_proof_sha256;
  trajectory.candidate_start_control_pose_sha256 =
      canonical.control_pose_sha256;
  canonical = canonicalizeAuthorizedTrajectoryV1(trajectory);
  trajectory.payload_sha256 = canonical.sha256;
  return trajectory;
}

inline CartesianTrajectoryApplicationStatus
applicationStatus(const AuthorizedCartesianTrajectory &trajectory) {
  CartesianTrajectoryApplicationStatus status;
  status.schema_version =
      CartesianTrajectoryApplicationStatus::SCHEMA_V1_SHADOW;
  status.authority_eligible = false;
  status.status_stamp.sec = 100;
  status.status_stamp.nanosec = 50000000U;
  status.frame_id = trajectory.frame_id;
  status.application_state =
      CartesianTrajectoryApplicationStatus::APPLICATION_WARMED;
  status.reject_reason = CartesianTrajectoryApplicationStatus::REJECT_NONE;
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
  status.applied_control_pose.orientation.w = 0.0;
  return status;
}

} // namespace overtake_transport_contract::c002ay0::test_fixture
